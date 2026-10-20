// tools/xs_probe/main.cpp - g2m_xs_probe: read-only dump of the join-aware cross-section of OSM ways (vault G2M-025).
//
//   g2m_xs_probe STORE SCOPE TILE_KEY WAY_ID... [--census]
//
// Derives the roads.graph of the level-2 tile and its eight neighbours from the immutable source store, merges them (world
// node degrees), and prints for every stretch of each named way: the bands (kind, direction, width, turn bits, start and
// end ramps), the anchor, the offset correction, both joins, and the degree of the end nodes. With --census instead of ways,
// every way of the centre tile is checked for lane-count changes at nodes: the node degree, the counted lanes of the ways
// meeting there and whether the cross-section resolver gave the end a join (and so a taper).
//
// Not part of any product; used to inspect lane adds and lane drops by hand.
#include "g2m/core/tile_key.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_rules.h"
#include "g2m/server/tile_store.h"

#include <algorithm>
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
    static const char* n[] = {"none", "consistent", "lane_change", "inferred_alignment", "placement_conflict"};
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

int main(int argc, char** argv) {
    if (argc < 5) { std::cerr << "usage: g2m_xs_probe STORE SCOPE TILE_KEY WAY_ID... [--census]\n"; return 1; }
    const auto key = parse_tile_path(argv[3]);
    if (!key || key->level != 2) { std::cerr << "level-2 tile key required\n"; return 1; }
    TileStoreConfig config; config.dir = argv[1]; config.read_only = true;
    auto store = TileStore::open(config);
    if (!store) { std::cerr << store.error().message << '\n'; return 1; }
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
