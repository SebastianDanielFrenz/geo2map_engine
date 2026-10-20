// tools/xs_probe/main.cpp - g2m_xs_probe: read-only dump of the join-aware cross-section of OSM ways (vault G2M-025).
//
//   g2m_xs_probe STORE SCOPE TILE_KEY WAY_ID... [--census]
//   g2m_xs_probe STORE SCOPE TILE_KEY --merge-census E0 N0 E1 N1 [--merge-rules on|off]
//
// Derives the roads.graph of the level-2 tile and its eight neighbours from the immutable source store, merges them (world
// node degrees), and prints for every stretch of each named way: the bands (kind, direction, width, turn bits, start and
// end ramps), the anchor, the offset correction, both joins, and the degree of the end nodes. With --census instead of ways,
// every way of the centre tile is checked for lane-count changes at nodes: the node degree, the counted lanes of the ways
// meeting there and whether the cross-section resolver gave the end a join (and so a taper).
//
// --merge-census walks every level-2 tile of the UTM box (the tile key only names the zone and level; each node is counted in
// the tile that contains it) and counts the nodes of degree three where one one-way arm has the counted lanes of the other
// two together and one of those is a *_link (merge-lane / exit candidates, G2M-028), how many of them the cross-section
// resolver gave a landing, and the degree-two nodes whose ways differ in counted lanes, with and without a join (a taper).
// --merge-rules off resolves with merge_lane.enabled = false (the state before G2M-028).
//
// Not part of any product; used to inspect lane adds and lane drops by hand.
#include "g2m/core/tile_key.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_rules.h"
#include "g2m/server/tile_store.h"

#include <algorithm>
#include <array>
#include <memory>
#include <optional>
#include <cstdio>
#include <iostream>
#include <map>
#include <set>
#include <string_view>

using namespace g2m;

namespace {
const char* kind_name(LaneKind k) {
    static const char* n[] = {"driving", "turn", "bus", "cycle", "parking", "shoulder", "verge", "sidewalk"};
    return n[static_cast<int>(k)];
}
const char* dir_name(LaneDirection d) {
    static const char* n[] = {"fwd", "bwd", "both", "none"};
    return n[static_cast<int>(d)];
}
const char* join_name(JoinKind k) {
    static const char* n[] = {"none", "consistent", "lane_change", "inferred_alignment", "placement_conflict", "landing"};
    return n[static_cast<int>(k)];
}
Result<RoadGraph> derive_graph(TileStore& store, const char* scope, const TileKey& key) {
    RoadGraphDeriver deriver;
    std::vector<InputTile> inputs;
    for (const auto& ref : deriver.inputs_for(key)) {
        auto tile = store.get({scope, ref.layer, ref.key});
        if (!tile) return tile.error();
        InputTile input;
        input.ref = ref;
        if (*tile) { input.header = std::move((*tile)->header); input.body = std::move((*tile)->body); }
        inputs.push_back(std::move(input));
    }
    const auto derived = deriver.derive({key, 0, inputs});
    if (!derived) return derived.error();
    const auto body = decode_body(derived->body);
    if (!body) return body.error();
    auto tile = decode_road_graph(*body);
    if (!tile) return tile.error();
    return std::move(tile->graph);
}
} // namespace

namespace {
int merge_census(TileStore& store, const char* scope, const TileKey& zone_key, double e0, double n0, double e1, double n1, bool rules_on, bool list_missed) {
    RoadRules rules = builtin_road_rules_de();
    rules.merge_lane.enabled = rules_on && rules.merge_lane.enabled;
    const auto k0 = tile_key_at(zone_key.zone, 2, e0, n0), k1 = tile_key_at(zone_key.zone, 2, e1, n1);
    if (!k0 || !k1) { std::cerr << "box outside the zone\n"; return 1; }
    std::map<std::pair<std::int32_t, std::int32_t>, std::shared_ptr<RoadGraph>> cache;   // (y, x)
    const auto graph_at = [&](const TileKey& k) -> std::shared_ptr<RoadGraph> {
        const auto it = cache.find({k.y, k.x});
        if (it != cache.end()) return it->second;
        auto g = derive_graph(store, scope, k);
        auto ptr = g ? std::make_shared<RoadGraph>(std::move(*g)) : std::shared_ptr<RoadGraph>{};
        cache.emplace(std::make_pair(k.y, k.x), ptr);
        return ptr;
    };
    std::size_t tiles = 0, deg3 = 0, candidates = 0, merges = 0, diverges = 0, landed = 0, landed_merge = 0, landed_diverge = 0, through_tapers = 0;
    std::size_t d2_differ = 0, d2_taper = 0;
    std::map<std::string, std::size_t> missed;
    for (std::int32_t ty = k0->y; ty <= k1->y; ++ty) {
        for (auto it = cache.begin(); it != cache.end();) it = it->first.first < ty - 1 ? cache.erase(it) : std::next(it);   // keep three tile rows
        for (std::int32_t tx = k0->x; tx <= k1->x; ++tx) {
            TileKey key = *k0; key.x = tx; key.y = ty;
            std::vector<std::shared_ptr<RoadGraph>> keep;
            std::vector<const RoadGraph*> ptrs;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) {
                    TileKey k = key; k.x += dx; k.y += dy;
                    if (auto g = graph_at(k)) { keep.push_back(g); ptrs.push_back(g.get()); }
                }
            const auto merged = merge_road_graphs(ptrs);
            if (!merged) continue;
            const auto builder = RoadReferenceBuilder::make(*merged);
            if (!builder) continue;
            ++tiles;
            const double lo_e = static_cast<double>(key.min_easting()), lo_n = static_cast<double>(key.min_northing()), size = static_cast<double>(key.size_m());
            const auto info = [&](std::int64_t way_id, int& counted, bool& oneway_fwd, std::string& hw) {
                counted = 0; oneway_fwd = false;
                const auto* w = std::lower_bound(merged->ways.data(), merged->ways.data() + merged->ways.size(), way_id, [](const RoadGraphWay& a, std::int64_t v) { return a.osm_id < v; });
                if (w == merged->ways.data() + merged->ways.size() || w->osm_id != way_id) return;
                for (const auto& t : w->tags) if (t.first == "highway") hw = t.second;
                const auto b = resolve_cross_section_base(*w, rules);
                if (!b || !b->xs) return;
                oneway_fwd = (b->xs->flags & kXsOneway) && !(b->xs->flags & kXsReversedOneway);
                for (const auto& band : b->xs->bands) counted += band.counted;
            };
            const auto section_at = [&](std::int64_t way_id, std::uint32_t ref, bool at_end) -> std::optional<RoadCrossSection> {
                const auto fits = builder->fit_stretches(way_id);
                if (!fits) return {};
                for (const auto& fit : *fits) {
                    if ((at_end ? fit.stretch.end_ref : fit.stretch.start_ref) != ref || !fit.reference) continue;
                    const auto xs = resolve_cross_section(*builder, way_id, fit.stretch, fit.reference->length_m, rules);
                    if (xs && xs->xs) return *xs->xs;
                }
                return {};
            };
            for (const auto& node : merged->nodes) {
                const double e = static_cast<double>(node.position.x) / 1000.0, n = static_cast<double>(node.position.y) / 1000.0;
                if (e < lo_e || e >= lo_e + size || n < lo_n || n >= lo_n + size) continue;
                if (e < e0 || e > e1 || n < n0 || n > n1) continue;
                const auto deg = builder->degree(node.osm_id);
                const auto arms = builder->arms(node.osm_id);
                if (deg == 2 && arms.size() == 2 && arms[0].way_id != arms[1].way_id) {
                    int c0, c1; bool o0, o1; std::string h0, h1;
                    info(arms[0].way_id, c0, o0, h0); info(arms[1].way_id, c1, o1, h1);
                    if (c0 > 0 && c1 > 0 && c0 != c1) {
                        ++d2_differ;
                        const auto x = section_at(arms[0].way_id, arms[0].ref, !arms[0].leaves_node);
                        if (x && (arms[0].leaves_node ? x->start_join : x->end_join).kind == JoinKind::LaneChange) ++d2_taper;
                    }
                    continue;
                }
                if (deg != 3 || arms.size() != 3) continue;
                ++deg3;
                if (arms[0].way_id == arms[1].way_id || arms[1].way_id == arms[2].way_id || arms[0].way_id == arms[2].way_id) continue;
                std::array<int, 3> cnt{}; std::array<bool, 3> one{}; std::array<std::string, 3> hws;
                bool all = true;
                for (std::size_t i = 0; i < 3; ++i) { info(arms[i].way_id, cnt[i], one[i], hws[i]); all = all && one[i] && cnt[i] > 0; }
                if (!all) continue;
                const int leaving = int(arms[0].leaves_node) + int(arms[1].leaves_node) + int(arms[2].leaves_node);
                if (leaving != 1 && leaving != 2) continue;
                const bool merge = leaving == 1;
                std::size_t si = 0;
                for (std::size_t i = 0; i < 3; ++i) if (arms[i].leaves_node == merge) si = i;
                const std::size_t xi = (si + 1) % 3, yi = (si + 2) % 3;
                if (cnt[si] != cnt[xi] + cnt[yi]) continue;
                const auto is_link = [&](std::size_t i) { const auto& h = hws[i]; return h.size() > 5 && h.substr(h.size() - 5) == "_link"; };
                if (!is_link(xi) && !is_link(yi)) continue;
                ++candidates; (merge ? merges : diverges) += 1;
                bool got_landing = false, got_through = false;
                for (const auto& a : arms) {
                    const auto x = section_at(a.way_id, a.ref, !a.leaves_node);
                    if (!x) continue;
                    const auto& j = a.leaves_node ? x->start_join : x->end_join;
                    got_landing = got_landing || j.kind == JoinKind::Landing;
                    got_through = got_through || j.kind == JoinKind::LaneChange;
                }
                if (got_landing) { ++landed; (merge ? landed_merge : landed_diverge) += 1; }
                else {
                    missed[got_through ? "taper_without_landing" : "no_taper"] += 1;
                    if (list_missed) {
                        std::printf("miss %s node %lld E %.1f N %.1f arms %lld/%lld/%lld lanes %d/%d/%d single %zu merge %d\n", got_through ? "taper" : "none", static_cast<long long>(node.osm_id), e, n,
                            static_cast<long long>(arms[0].way_id), static_cast<long long>(arms[1].way_id), static_cast<long long>(arms[2].way_id), cnt[0], cnt[1], cnt[2], si, merge ? 1 : 0);
                    }
                }
                if (got_through) ++through_tapers;
            }
        }
    }
    std::cout << "merge_census rules=" << (rules.merge_lane.enabled ? "on" : "off") << " tiles=" << tiles << " degree3_nodes=" << deg3
              << " candidates=" << candidates << " (merge " << merges << ", diverge " << diverges << ") landed=" << landed << " (merge " << landed_merge << ", diverge " << landed_diverge
              << ") candidates_with_lane_taper=" << through_tapers << " deg2_lane_count_changes=" << d2_differ << " deg2_with_taper=" << d2_taper << '\n';
    for (const auto& [k, v] : missed) std::cout << "  candidate without landing: " << k << " " << v << '\n';
    return 0;
}
} // namespace

int main(int argc, char** argv) {
    if (argc < 5) { std::cerr << "usage: g2m_xs_probe STORE SCOPE TILE_KEY WAY_ID... [--census]\n"; return 1; }
    const auto key = parse_tile_path(argv[3]);
    if (!key || key->level != 2) { std::cerr << "level-2 tile key required\n"; return 1; }
    TileStoreConfig config; config.dir = argv[1]; config.read_only = true;
    auto store = TileStore::open(config);
    if (!store) { std::cerr << store.error().message << '\n'; return 1; }
    if (std::string_view(argv[4]) == "--merge-census") {
        if (argc < 9) { std::cerr << "usage: ... --merge-census E0 N0 E1 N1 [--merge-rules on|off]\n"; return 1; }
        const bool on = !(argc >= 11 && std::string_view(argv[10]) == "off");
        return merge_census(**store, argv[2], *key, std::atof(argv[5]), std::atof(argv[6]), std::atof(argv[7]), std::atof(argv[8]), on, argc >= 12 && std::string_view(argv[11]) == "list");
    }
    std::vector<std::int64_t> ways;
    bool census = false;
    for (int i = 4; i < argc; ++i) {
        if (std::string_view(argv[i]) == "--census") census = true;
        else ways.push_back(std::atoll(argv[i]));
    }
    std::vector<RoadGraph> graphs;
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
            TileKey k = *key; k.x += dx; k.y += dy;
            auto g = derive_graph(**store, argv[2], k);
            if (g) graphs.push_back(std::move(*g));
        }
    std::vector<const RoadGraph*> ptrs;
    for (const auto& g : graphs) ptrs.push_back(&g);
    const auto merged = merge_road_graphs(ptrs);
    if (!merged) { std::cerr << merged.error().message << '\n'; return 1; }
    const auto builder = RoadReferenceBuilder::make(*merged);
    if (!builder) { std::cerr << builder.error().message << '\n'; return 1; }
    const RoadRules& rules = builtin_road_rules_de();
    const auto lanes_of = [&](std::int64_t id) {
        for (const auto& w : merged->ways)
            if (w.osm_id == id) {
                const auto b = resolve_cross_section_base(w, rules);
                int n = 0;
                if (b && b->xs) for (const auto& band : b->xs->bands) n += band.counted;
                return n;
            }
        return -1;
    };
    if (census) {
        // every way's two ends: world degree, lane counts of the ways meeting there, whether the resolver gave a join
        std::size_t ends = 0, joins = 0, differing_deg2 = 0, differing_junction = 0;
        std::map<std::string, std::size_t> by_class;
        for (const auto& w : merged->ways) {
            const auto st = builder->stretches(w.osm_id);
            if (!st) continue;
            for (const auto& s : *st) {
                for (const bool end : {false, true}) {
                    ++ends;
                    const auto join = builder->join(w.osm_id, s, end);
                    if (join && *join) { ++joins; if (lanes_of(w.osm_id) != lanes_of((*join)->way_id)) ++differing_deg2; continue; }
                    // a junction: other ways at the node with a different counted-lane total
                    const auto node = w.node_ids[end ? s.end_ref : s.start_ref];
                    if (builder->degree(node) < 3) continue;
                    std::set<int> counts;
                    for (const auto& o : merged->ways)
                        if (std::find(o.node_ids.begin(), o.node_ids.end(), node) != o.node_ids.end()) counts.insert(lanes_of(o.osm_id));
                    if (counts.size() > 1) ++differing_junction;
                }
            }
        }
        std::cout << "stretch_ends=" << ends << " joins=" << joins << " joins_with_lane_change=" << differing_deg2
                  << " junction_ends_with_other_lane_counts=" << differing_junction << '\n';
        return 0;
    }
    for (const auto id : ways) {
        const auto* way = std::lower_bound(merged->ways.data(), merged->ways.data() + merged->ways.size(), id,
            [](const RoadGraphWay& w, std::int64_t v) { return w.osm_id < v; });
        if (way == merged->ways.data() + merged->ways.size() || way->osm_id != id) { std::cout << "way " << id << ": not in graph\n"; continue; }
        std::cout << "way " << id << " nodes " << way->node_ids.size() << " first " << way->node_ids.front() << " (deg " << builder->degree(way->node_ids.front())
                  << ") last " << way->node_ids.back() << " (deg " << builder->degree(way->node_ids.back()) << ")\n  tags:";
        for (const auto& t : way->tags) std::cout << ' ' << t.first << '=' << t.second;
        std::cout << '\n';
        for (std::size_t i = 0; i < way->node_ids.size(); ++i) {
            const auto it = std::lower_bound(merged->nodes.begin(), merged->nodes.end(), way->node_ids[i], [](const auto& n, auto v) { return n.osm_id < v; });
            if (it != merged->nodes.end() && it->osm_id == way->node_ids[i])
                std::printf("  node %zu id %lld E %.3f N %.3f\n", i, static_cast<long long>(it->osm_id), it->position.x / 1000.0, it->position.y / 1000.0);
        }
        const auto fits = builder->fit_stretches(id);
        if (!fits) { std::cout << "  fit: " << fits.error().message << '\n'; continue; }
        for (const auto& fit : *fits) {
            std::cout << "  stretch refs " << fit.stretch.start_ref << ".." << fit.stretch.end_ref;
            if (!fit.reference) { std::cout << " declined: " << fit.reference.error().message << '\n'; continue; }
            std::cout << " length " << fit.reference->length_m << " m\n";
            const auto xs = resolve_cross_section(*builder, id, fit.stretch, fit.reference->length_m, rules);
            if (!xs || !xs->xs) { std::cout << "    no section\n"; continue; }
            const auto& x = *xs->xs;
            std::cout << "    anchor band " << int(x.anchor.band) << " edge " << int(x.anchor.edge) << " source " << int(x.anchor.source)
                      << "  offset mode " << int(x.offset.mode) << " start " << x.offset.start_mm << "/" << x.offset.start_length_mm << " end " << x.offset.end_mm << "/" << x.offset.end_length_mm
                      << "  start_join " << join_name(x.start_join.kind) << " (" << x.start_join.neighbour_way_id << ") end_join " << join_name(x.end_join.kind) << " (" << x.end_join.neighbour_way_id << ")"
                      << " flags " << x.flags << '\n';
            for (std::size_t i = 0; i < x.bands.size(); ++i) {
                const auto& b = x.bands[i];
                std::cout << "    band " << i << ' ' << kind_name(b.kind) << ' ' << dir_name(b.direction) << (b.counted ? " counted" : "") << " w " << b.width_mm
                          << " turn 0x" << std::hex << b.turn << std::dec << " change " << int(b.change) << " start " << b.start.length_mm << "/" << b.start.from_width_mm << " end " << b.end.length_mm << "/" << b.end.from_width_mm << '\n';
            }
        }
    }
    return 0;
}
