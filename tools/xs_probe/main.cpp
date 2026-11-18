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
// The stretches are fitted with the served reference limits (curvature exceptions, chord bound from the rules), as the
// served roads.geom does. `strict` after `list` fits with ReferenceFitLimits{} instead; then a stretch the strict fit
// declines reports stretch_fit_declined although the served roads.geom draws it.
//   g2m_xs_probe STORE SCOPE TILE_KEY --diag-0b E0 N0 E1 N1   (stage 0b diagnostics, see diag_0b below)
// With `list` after it, every candidate without a landing or without a lane taper, every other degree-three node with a
// *_link arm, and every degree-two lane-count change without a lane-change join get a line
// `reason KIND DIR CODE node ...` (diagnostics for the ramp-lane work): KIND cand_landing / cand_taper / near / d2; CODE
// merge_<classification refusal>, join_<kind>_ramped|_unramped, or the resolver's end note (landing_offset_hold, ...).
//
// Not part of any product; used to inspect lane adds and lane drops by hand.
#include "g2m/core/tile_key.h"
#include "g2m/layer/road_attributes.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_rules.h"
#include "g2m/server/tile_store.h"

#include <algorithm>
#include <cmath>
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
ReferenceFitLimits served_fit_limits(const RoadRules& rules);
int merge_census(TileStore& store, const char* scope, const TileKey& zone_key, double e0, double n0, double e1, double n1, bool rules_on, bool list_missed, bool served_fits) {
    RoadRules rules = builtin_road_rules_de();
    rules.merge_lane.enabled = rules_on && rules.merge_lane.enabled;
    const ReferenceFitLimits fit_limits = served_fits ? served_fit_limits(rules) : ReferenceFitLimits{};
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
            struct EndSection { RoadCrossSection xs; CrossSectionEndTrace trace; };
            std::string section_miss;   // why the last section_at found no section
            const auto section_at = [&](std::int64_t way_id, std::uint32_t ref, bool at_end) -> std::optional<EndSection> {
                const auto fits = builder->fit_stretches(way_id, fit_limits);
                section_miss = "stretch_not_found";
                if (!fits) { section_miss = "stretch_fit_error"; return {}; }
                for (const auto& fit : *fits) {
                    if ((at_end ? fit.stretch.end_ref : fit.stretch.start_ref) != ref) continue;
                    if (!fit.reference) {
                        section_miss = "stretch_fit_declined:" + fit.reference.error().message;
                        for (auto& ch : section_miss) if (ch == ' ') ch = '_';
                        continue;
                    }
                    CrossSectionTrace trace;
                    const auto xs = resolve_cross_section(*builder, way_id, fit.stretch, fit.reference->length_m, rules, &trace);
                    if (xs && xs->xs) return EndSection{*xs->xs, trace.ends[at_end ? 1 : 0]};
                    section_miss = xs ? "section_declined" : "section_error";
                }
                return {};
            };
            const auto is_link_hw = [](const std::string& h) { return h.size() > 5 && h.substr(h.size() - 5) == "_link"; };
            // reason KIND DIR CODE node ID E e N n arms a/b/c lanes x/y/z
            const auto reason_line = [&](const char* kind, const char* dir, const std::string& reason, const RoadGraphNode& nd, double e, double n,
                                         const std::vector<RoadNodeArm>& arms, const int* cnt) {
                if (!list_missed) return;
                std::printf("reason %s %s %s node %lld E %.1f N %.1f arms", kind, dir, reason.c_str(), static_cast<long long>(nd.osm_id), e, n);
                for (std::size_t i = 0; i < arms.size(); ++i) std::printf("%s%lld", i ? "/" : " ", static_cast<long long>(arms[i].way_id));
                std::printf(" lanes");
                for (std::size_t i = 0; i < arms.size(); ++i) std::printf("%s%d", i ? "/" : " ", cnt[i]);
                std::printf("\n");
            };
            const auto end_reason = [&](const std::optional<EndSection>& x) -> std::string {
                if (!x) return "?";
                const auto& t = x->trace;
                if (t.note == CrossSectionEndNote::Joined) return std::string("join_") + join_name(t.kind) + (t.tapered ? "_ramped" : "_unramped");
                if (t.note == CrossSectionEndNote::NoJoin && t.merge_ran) return std::string("merge_") + merge_node_reason_name(t.merge_reason);
                return cross_section_end_note_name(t.note);
            };
            for (const auto& node : merged->nodes) {
                const double e = static_cast<double>(node.position.x) / 1000.0, n = static_cast<double>(node.position.y) / 1000.0;
                if (e < lo_e || e >= lo_e + size || n < lo_n || n >= lo_n + size) continue;
                if (e < e0 || e > e1 || n < n0 || n > n1) continue;
                const auto deg = builder->degree(node.osm_id);
                const auto arms = builder->arms(node.osm_id);
                if (deg == 2 && arms.size() == 2 && arms[0].way_id != arms[1].way_id) {
                    int c[2]; bool o0, o1; std::string h0, h1;
                    info(arms[0].way_id, c[0], o0, h0); info(arms[1].way_id, c[1], o1, h1);
                    if (c[0] > 0 && c[1] > 0 && c[0] != c[1]) {
                        ++d2_differ;
                        const auto x = section_at(arms[0].way_id, arms[0].ref, !arms[0].leaves_node);
                        if (x && (arms[0].leaves_node ? x->xs.start_join : x->xs.end_join).kind == JoinKind::LaneChange) ++d2_taper;
                        else reason_line("d2", "-", x ? end_reason(x) : section_miss, node, e, n, arms, c);
                    }
                    continue;
                }
                if (deg != 3 || arms.size() != 3) continue;
                ++deg3;
                if (arms[0].way_id == arms[1].way_id || arms[1].way_id == arms[2].way_id || arms[0].way_id == arms[2].way_id) continue;
                std::array<int, 3> cnt{}; std::array<bool, 3> one{}; std::array<std::string, 3> hws;
                bool all = true;
                for (std::size_t i = 0; i < 3; ++i) { info(arms[i].way_id, cnt[i], one[i], hws[i]); all = all && one[i] && cnt[i] > 0; }
                const int leaving = int(arms[0].leaves_node) + int(arms[1].leaves_node) + int(arms[2].leaves_node);
                const bool merge = leaving == 1;
                std::size_t si = 0;
                for (std::size_t i = 0; i < 3; ++i) if (arms[i].leaves_node == merge) si = i;
                const std::size_t xi = (si + 1) % 3, yi = (si + 2) % 3;
                const bool any_link = is_link_hw(hws[0]) || is_link_hw(hws[1]) || is_link_hw(hws[2]);
                const bool candidate = all && (leaving == 1 || leaving == 2) && cnt[si] == cnt[xi] + cnt[yi] && (is_link_hw(hws[xi]) || is_link_hw(hws[yi]));
                if (!candidate) {
                    // A degree-three node with a *_link arm that is no candidate: the classification's refusal.
                    if (any_link) {
                        const auto d = diagnose_merge_node(*builder, node.osm_id, rules);
                        std::string why = merge_node_reason_name(d.reason);
                        if (d.reason == MergeNodeReason::LaneSum)   // single = the arm alone in its direction
                            why += cnt[si] > cnt[xi] + cnt[yi] ? "_single_gt_sum" : cnt[si] == std::max(cnt[xi], cnt[yi]) ? "_single_eq_max" : "_single_lt_sum";
                        reason_line("near", leaving == 1 ? "merge" : leaving == 2 ? "diverge" : "-", why, node, e, n, arms, cnt.data());
                    }
                    continue;
                }
                ++candidates; (merge ? merges : diverges) += 1;
                bool got_landing = false, got_through = false;
                std::array<std::optional<EndSection>, 3> secs;
                std::array<std::string, 3> misses;
                for (std::size_t i = 0; i < 3; ++i) {
                    const auto& a = arms[i];
                    secs[i] = section_at(a.way_id, a.ref, !a.leaves_node);
                    misses[i] = section_miss;
                    if (!secs[i]) continue;
                    const auto& j = a.leaves_node ? secs[i]->xs.start_join : secs[i]->xs.end_join;
                    got_landing = got_landing || j.kind == JoinKind::Landing;
                    got_through = got_through || j.kind == JoinKind::LaneChange;
                }
                const char* dir = merge ? "merge" : "diverge";
                if (!got_landing || !got_through) {
                    const auto d = diagnose_merge_node(*builder, node.osm_id, rules);
                    const auto arm_of = [&](std::int64_t id) { for (std::size_t i = 0; i < 3; ++i) if (arms[i].way_id == id) return i; return std::size_t{3}; };
                    if (!got_landing) {
                        std::string why;
                        if (d.reason != MergeNodeReason::Ok) why = std::string("merge_") + merge_node_reason_name(d.reason);
                        else { const auto r = arm_of(d.ramp_id); why = r < 3 ? (secs[r] ? end_reason(secs[r]) : misses[r]) : "ramp_arm_missing"; }
                        reason_line("cand_landing", dir, why, node, e, n, arms, cnt.data());
                    }
                    if (!got_through) {
                        std::string why;
                        if (d.reason != MergeNodeReason::Ok) why = std::string("merge_") + merge_node_reason_name(d.reason);
                        else { const auto t = arm_of(d.through_id); why = t < 3 ? (secs[t] ? end_reason(secs[t]) : misses[t]) : "through_arm_missing"; }
                        reason_line("cand_taper", dir, why, node, e, n, arms, cnt.data());
                    }
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

// ---- --diag-0b (ramp-lane work, stage 0b): diagnostics only, never read by the product ----
// Prints, for the level-2 tiles of the box (each item counted in the tile that holds its node):
//   S  every way stretch (owned by the tile of its start node): class, polyline length, the reference fit with the probe's
//      strict limits (ReferenceFitLimits{}) and with the served limits (curvature exceptions on, chord bound from the rules,
//      as derive_road_geom / build_road_geom_tile), and for a strict decline the corner that fails the rounded fallback.
//   C  every merge-lane candidate (as --merge-census): role R / T / S of each arm with both fit outcomes.
//   D  every degree-two node whose ways differ in counted lanes: both ends' join kind and ramp, split, the lateral jump of
//      matched counted lane edges at the node (served limits), the added lanes' kinds.
//   N  every degree-three node refused with lane_sum where the lone arm has as many lanes as the larger other arm: lane
//      pattern, lane tags of the lone and the larger arm, and a short way with one more lane within two joins.
ReferenceFitLimits served_fit_limits(const RoadRules& rules) {
    ReferenceFitLimits l;
    l.allow_curvature_exceptions = true;
    l.chord_half_width_factor = static_cast<double>(rules.reference.chord_half_width_percent) / 100.0;
    l.chord_margin_m = static_cast<double>(rules.reference.chord_margin_mm) / 1000.0;
    return l;
}
std::string fit_code(const Result<RoadReferenceLine>& r) {
    if (r) return std::string(r->curvature_exception ? "ok_exception" : "ok") + (r->chord_bound_relaxed ? "_relaxed" : "");
    const auto& m = r.error().message;
    if (m == "road reference: rounded corner corridor") return "corridor";
    if (m == "road reference: overlapping rounded corners") return "overlap";
    std::string c = "other:" + m;
    for (auto& ch : c) if (ch == ' ') ch = '_';
    return c;
}
const std::string& way_tag(const RoadGraphWay& w, std::string_view key) {
    static const std::string empty;
    for (const auto& t : w.tags) if (t.first == key) return t.second;
    return empty;
}
// tag flags of a way: t turn:lanes*, d destination:lanes*, p placement*, m a merge_to_* or slight_* token in turn:lanes*
std::string lane_tag_flags(const RoadGraphWay& w) {
    bool t = false, d = false, p = false, m = false;
    for (const auto& [k, v] : w.tags) {
        if (k.rfind("turn:lanes", 0) == 0) { t = true; if (v.find("merge_to") != std::string::npos || v.find("slight_") != std::string::npos) m = true; }
        if (k.rfind("destination:lanes", 0) == 0) d = true;
        if (k.rfind("placement", 0) == 0) p = true;
    }
    std::string s;
    if (t) s += 't'; if (d) s += 'd'; if (p) s += 'p'; if (m) s += 'm';
    return s.empty() ? "-" : s;
}

int diag_0b(TileStore& store, const char* scope, const TileKey& zone_key, double e0, double n0, double e1, double n1) {
    const RoadRules& rules = builtin_road_rules_de();
    const ReferenceFitLimits strict{}, served = served_fit_limits(rules);
    const auto k0 = tile_key_at(zone_key.zone, 2, e0, n0), k1 = tile_key_at(zone_key.zone, 2, e1, n1);
    if (!k0 || !k1) { std::cerr << "box outside the zone\n"; return 1; }
    std::map<std::pair<std::int32_t, std::int32_t>, std::shared_ptr<RoadGraph>> cache;
    const auto graph_at = [&](const TileKey& k) -> std::shared_ptr<RoadGraph> {
        const auto it = cache.find({k.y, k.x});
        if (it != cache.end()) return it->second;
        auto g = derive_graph(store, scope, k);
        auto ptr = g ? std::make_shared<RoadGraph>(std::move(*g)) : std::shared_ptr<RoadGraph>{};
        cache.emplace(std::make_pair(k.y, k.x), ptr);
        return ptr;
    };
    for (std::int32_t ty = k0->y; ty <= k1->y; ++ty) {
        for (auto it = cache.begin(); it != cache.end();) it = it->first.first < ty - 1 ? cache.erase(it) : std::next(it);
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
            const RoadGraph& G = *merged;
            const double lo_e = static_cast<double>(key.min_easting()), lo_n = static_cast<double>(key.min_northing()), size = static_cast<double>(key.size_m());
            const auto node_of = [&](std::int64_t id) -> const RoadGraphNode* {
                const auto it = std::lower_bound(G.nodes.begin(), G.nodes.end(), id, [](const auto& n, auto v) { return n.osm_id < v; });
                return it != G.nodes.end() && it->osm_id == id ? &*it : nullptr;
            };
            const auto way_of = [&](std::int64_t id) -> const RoadGraphWay* {
                const auto it = std::lower_bound(G.ways.begin(), G.ways.end(), id, [](const auto& w, auto v) { return w.osm_id < v; });
                return it != G.ways.end() && it->osm_id == id ? &*it : nullptr;
            };
            const auto owned = [&](const RoadGraphNode* nd, double& e, double& n) {
                if (!nd) return false;
                e = nd->position.x / 1000.0; n = nd->position.y / 1000.0;
                return e >= lo_e && e < lo_e + size && n >= lo_n && n < lo_n + size && e >= e0 && e <= e1 && n >= n0 && n <= n1;
            };
            const auto counted_of = [&](const RoadGraphWay& w) {
                const auto b = resolve_cross_section_base(w, rules);
                int c = 0;
                if (b && b->xs) for (const auto& band : b->xs->bands) c += band.counted;
                return c;
            };
            // the polyline of a stretch, coincident points dropped
            const auto polyline = [&](const RoadGraphWay& w, RoadStretch st) {
                std::vector<std::pair<double, double>> p;
                for (std::uint32_t i = st.start_ref; i <= st.end_ref; ++i) {
                    const auto* nd = node_of(w.node_ids[i]); if (!nd) continue;
                    const std::pair<double, double> q{nd->position.x / 1000.0, nd->position.y / 1000.0};
                    if (p.empty() || p.back() != q) p.push_back(q);
                }
                return p;
            };
            const auto poly_len = [](const std::vector<std::pair<double, double>>& p) {
                double L = 0;
                for (std::size_t i = 1; i < p.size(); ++i) L += std::hypot(p[i].first - p[i - 1].first, p[i].second - p[i - 1].second);
                return L;
            };
            // ---- S: every stretch owned by this tile ----
            for (const auto& w : G.ways) {
                const auto sts = builder->stretches(w.osm_id); if (!sts) continue;
                const auto attrs = resolve_road_attributes(w);
                for (const auto st : *sts) {
                    double e, n;
                    if (!owned(node_of(w.node_ids[st.start_ref]), e, n)) continue;
                    const auto p = polyline(w, st);
                    const auto a = builder->fit_stretch(w.osm_id, st, strict);
                    const auto b = builder->fit_stretch(w.osm_id, st, served);
                    std::printf("S way %lld refs %u..%u E %.1f N %.1f hw %s rank %d len %.1f strict %s served %s",
                        static_cast<long long>(w.osm_id), st.start_ref, st.end_ref, e, n, way_tag(w, "highway").c_str(), w.rank, poly_len(p),
                        fit_code(a).c_str(), fit_code(b).c_str());
                    if (b) std::printf(" fitted_r %.1f", b->fitted_min_radius_m);
                    if (!a && attrs && p.size() >= 3) {
                        const double R = attrs->min_horizontal_radius_m, dev = w.rank >= 5 ? 0.5 : 3.0;
                        const std::size_t k = p.size();
                        std::vector<double> ang(k, 0.0), L(k - 1, 0.0);
                        for (std::size_t i = 0; i + 1 < k; ++i) L[i] = std::hypot(p[i + 1].first - p[i].first, p[i + 1].second - p[i].second);
                        const bool closed = p.front() == p.back();
                        for (std::size_t i = 1; i + 1 < k; ++i) {
                            const double ux = (p[i].first - p[i - 1].first) / L[i - 1], uy = (p[i].second - p[i - 1].second) / L[i - 1];
                            const double vx = (p[i + 1].first - p[i].first) / L[i], vy = (p[i + 1].second - p[i].second) / L[i];
                            ang[i] = std::fabs(std::atan2(ux * vy - uy * vx, ux * vx + uy * vy));
                        }
                        const auto req = [&](double x) { return x >= 3.14159265358979 - 1e-6 ? 1e9 : R * (1 / std::cos(x / 2) - 1); };
                        const auto rloc = [&](std::size_t i) { return ang[i] < 1e-8 ? 1e9 : 0.5 * (L[i - 1] + L[i]) / ang[i]; };
                        const auto rcirc = [&](std::size_t i) {
                            const double ax = p[i - 1].first, ay = p[i - 1].second, bx = p[i].first, by = p[i].second, cx = p[i + 1].first, cy = p[i + 1].second;
                            const double cr = std::fabs((bx - ax) * (cy - ay) - (by - ay) * (cx - ax));
                            return cr < 1e-9 ? 1e9 : std::hypot(bx - ax, by - ay) * std::hypot(cx - bx, cy - by) * std::hypot(cx - ax, cy - ay) / (2 * cr);
                        };
                        // the radius at which the rounded fallback would pass, and the tightest local radius of the polyline
                        double rpass = 1e9, rmin = 1e9;
                        for (std::size_t i = 1; i + 1 < k; ++i) {
                            if (ang[i] < 1e-8) continue;
                            rpass = std::min(rpass, ang[i] >= 3.14159265358979 - 1e-6 ? 0.0 : dev / (1 / std::cos(ang[i] / 2) - 1));
                            rmin = std::min(rmin, rloc(i));
                        }
                        for (std::size_t i = 0; i + 1 < k; ++i) {
                            const double t = (i ? std::tan(ang[i] / 2) : 0) + (i + 1 < k - 1 ? std::tan(ang[i + 1] / 2) : 0);
                            if (t > 1e-12) rpass = std::min(rpass, L[i] / t);
                        }
                        std::size_t worst = 0; double wreq = 0;
                        for (std::size_t i = 1; i + 1 < k; ++i) if (ang[i] >= 1e-8 && req(ang[i]) > dev + 1e-10 && req(ang[i]) > wreq) { wreq = req(ang[i]); worst = i; }
                        std::printf(" R %.0f dev %.1f knots %zu closed %d rpass %.1f rmin_local %.1f", R, dev, k, closed ? 1 : 0, rpass, rmin);
                        if (worst) {
                            std::printf(" fail corridor req %.2f ang %.1f lin %.1f lout %.1f rloc %.1f rcirc %.1f", wreq, ang[worst] * 57.29577951,
                                L[worst - 1], L[worst], rloc(worst), rcirc(worst));
                        } else {
                            for (std::size_t i = 0; i + 1 < k; ++i) {
                                const double ta = i ? R * std::tan(ang[i] / 2) : 0, tb = i + 1 < k - 1 ? R * std::tan(ang[i + 1] / 2) : 0;
                                if (ta + tb > L[i] + 1e-10) {
                                    const std::size_t c = (i && (i + 1 >= k - 1 || ang[i] >= ang[i + 1])) ? i : i + 1;
                                    std::printf(" fail overlap need %.2f seg %.1f ang %.1f lin %.1f lout %.1f rloc %.1f rcirc %.1f", ta + tb, L[i],
                                        ang[c] * 57.29577951, L[c - 1], L[c], rloc(c), rcirc(c));
                                    break;
                                }
                            }
                        }
                    }
                    std::printf("\n");
                }
            }
            // sections with the served limits
            struct EndSection { RoadCrossSection xs; CrossSectionEndTrace trace; double station = 0; bool ok = false; std::string fit_strict, fit_served; };
            const auto section_at = [&](std::int64_t way_id, std::uint32_t ref, bool at_end) {
                EndSection out; out.fit_strict = out.fit_served = "stretch_not_found";
                const auto sts = builder->stretches(way_id); if (!sts) return out;
                for (const auto st : *sts) {
                    if ((at_end ? st.end_ref : st.start_ref) != ref) continue;
                    const auto a = builder->fit_stretch(way_id, st, strict);
                    const auto b = builder->fit_stretch(way_id, st, served);
                    out.fit_strict = fit_code(a); out.fit_served = fit_code(b);
                    if (!b) return out;
                    CrossSectionTrace trace;
                    const auto xs = resolve_cross_section(*builder, way_id, st, b->length_m, rules, &trace);
                    if (!xs || !xs->xs) { out.fit_served += "+no_section"; return out; }
                    out.xs = *xs->xs; out.trace = trace.ends[at_end ? 1 : 0]; out.station = at_end ? xs->xs->length_m : 0.0; out.ok = true;
                    return out;
                }
                return out;
            };
            const auto is_link = [](const std::string& h) { return h.size() > 5 && h.substr(h.size() - 5) == "_link"; };
            // walk away from a node along an arm through degree-two joins: the ways after the 1st and 2nd join
            struct Hop { std::int64_t way = 0; int counted = 0; double len = 0, from_node = 0; };
            const auto walk = [&](const RoadNodeArm& arm, int hops) {
                std::vector<Hop> out;
                std::int64_t wid = arm.way_id; std::uint32_t ref = arm.ref; bool leaves = arm.leaves_node; double dist = 0;
                for (int h = 0; h <= hops; ++h) {
                    const auto* w = way_of(wid); if (!w) break;
                    const auto sts = builder->stretches(wid); if (!sts) break;
                    std::optional<RoadStretch> st;
                    for (const auto s : *sts) if ((leaves ? s.start_ref : s.end_ref) == ref) st = s;
                    if (!st) break;
                    const double len = poly_len(polyline(*w, *st));
                    if (h) out.push_back({wid, counted_of(*w), len, dist});
                    dist += len;
                    const auto j = builder->join(wid, *st, leaves); if (!j || !*j) break;
                    wid = (*j)->way_id; ref = (*j)->ref; leaves = (*j)->leaves_node;
                }
                return out;
            };
            for (const auto& node : G.nodes) {
                double e, n;
                if (!owned(&node, e, n)) continue;
                const auto deg = builder->degree(node.osm_id);
                const auto arms = builder->arms(node.osm_id);
                // ---- D ----
                if (deg == 2 && arms.size() == 2 && arms[0].way_id != arms[1].way_id) {
                    const auto* w0 = way_of(arms[0].way_id); const auto* w1 = way_of(arms[1].way_id);
                    if (!w0 || !w1) continue;
                    const int c0 = counted_of(*w0), c1 = counted_of(*w1);
                    if (c0 <= 0 || c1 <= 0 || c0 == c1) continue;
                    std::array<EndSection, 2> x{section_at(arms[0].way_id, arms[0].ref, !arms[0].leaves_node),
                                                section_at(arms[1].way_id, arms[1].ref, !arms[1].leaves_node)};
                    std::printf("D node %lld E %.1f N %.1f ways %lld/%lld hw %s/%s lanes %d/%d", static_cast<long long>(node.osm_id), e, n,
                        static_cast<long long>(arms[0].way_id), static_cast<long long>(arms[1].way_id), way_tag(*w0, "highway").c_str(), way_tag(*w1, "highway").c_str(), c0, c1);
                    if (!x[0].ok || !x[1].ok) {
                        std::printf(" nosection %s/%s %s/%s\n", x[0].fit_strict.c_str(), x[0].fit_served.c_str(), x[1].fit_strict.c_str(), x[1].fit_served.c_str());
                        continue;
                    }
                    std::printf(" kind %s/%s ramp %d/%d split %d/%d delta %d shortened %d/%d", join_name(x[0].trace.kind), join_name(x[1].trace.kind),
                        x[0].trace.tapered ? 1 : 0, x[1].trace.tapered ? 1 : 0, x[0].trace.split ? 1 : 0, x[1].trace.split ? 1 : 0, x[0].trace.delta_mm,
                        (x[0].xs.flags & (kXsTaperShortenedStart | kXsTaperShortenedEnd)) ? 1 : 0, (x[1].xs.flags & (kXsTaperShortenedStart | kXsTaperShortenedEnd)) ? 1 : 0);
                    std::printf(" offset %d/%d", int(x[0].xs.offset.mode), int(x[1].xs.offset.mode));
                    if (x[0].trace.note == CrossSectionEndNote::Joined && x[1].trace.note == CrossSectionEndNote::Joined) {
                        // lateral jump of matched counted lane edges at the node, in arm 0's frame
                        const auto& A = x[0].xs; const auto& B = x[1].xs;
                        std::vector<double> ba(A.bands.size() + 1), bb(B.bands.size() + 1);
                        A.boundaries_at(x[0].station, ba); B.boundaries_at(x[1].station, bb);
                        const bool same = x[0].trace.neighbour_same;
                        double jump = 0; int matched = 0;
                        std::string added;
                        for (std::size_t i = 0; i < A.bands.size() && i < 32; ++i) {
                            const int j = x[0].trace.partner[i];
                            if (!A.bands[i].counted) continue;
                            if (j < 0) { added += kind_name(A.bands[i].kind); added += ','; continue; }
                            const auto jj = static_cast<std::size_t>(j);
                            const double bl = same ? bb[jj] : -bb[jj + 1], br = same ? bb[jj + 1] : -bb[jj];
                            jump = std::max({jump, std::fabs(ba[i] - bl), std::fabs(ba[i + 1] - br)});
                            ++matched;
                        }
                        for (std::size_t j = 0; j < B.bands.size() && j < 32; ++j)
                            if (B.bands[j].counted && x[1].trace.partner[j] < 0) { added += kind_name(B.bands[j].kind); added += ','; }
                        std::printf(" matched %d jump %.3f added %s", matched, jump, added.empty() ? "-" : added.c_str());
                        // the carriageway's outer counted edges (meaningful also for a split join, where no counted lane matches)
                        const auto outer = [](const RoadCrossSection& s, const std::vector<double>& b, double& l, double& r, int& f, int& bw, int& both) {
                            int first = -1, last = -1; f = bw = both = 0;
                            for (std::size_t i = 0; i < s.bands.size(); ++i) if (s.bands[i].counted) {
                                if (first < 0) first = static_cast<int>(i);
                                last = static_cast<int>(i);
                                (s.bands[i].direction == LaneDirection::Forward ? f : s.bands[i].direction == LaneDirection::Backward ? bw : both) += 1;
                            }
                            l = first < 0 ? 0 : b[static_cast<std::size_t>(first)]; r = last < 0 ? 0 : b[static_cast<std::size_t>(last) + 1];
                        };
                        double al, ar, bl, br; int af, ab, ao, bf, bbw, bo;
                        outer(A, ba, al, ar, af, ab, ao); outer(B, bb, bl, br, bf, bbw, bo);
                        if (!same) { const double t = bl; bl = -br; br = -t; std::swap(bf, bbw); }
                        std::printf(" cw_jump %.3f %.3f dirs %d+%d+%d/%d+%d+%d", std::fabs(al - bl), std::fabs(ar - br), af, ab, ao, bf, bbw, bo);
                    }
                    std::printf("\n");
                    continue;
                }
                if (deg != 3 || arms.size() != 3) continue;
                if (arms[0].way_id == arms[1].way_id || arms[1].way_id == arms[2].way_id || arms[0].way_id == arms[2].way_id) continue;
                std::array<const RoadGraphWay*, 3> ws{way_of(arms[0].way_id), way_of(arms[1].way_id), way_of(arms[2].way_id)};
                if (!ws[0] || !ws[1] || !ws[2]) continue;
                std::array<int, 3> cnt{}; std::array<bool, 3> one{}; std::array<std::string, 3> hws;
                bool all = true;
                for (std::size_t i = 0; i < 3; ++i) {
                    hws[i] = way_tag(*ws[i], "highway");
                    const auto b = resolve_cross_section_base(*ws[i], rules);
                    cnt[i] = 0; one[i] = false;
                    if (b && b->xs) { one[i] = (b->xs->flags & kXsOneway) && !(b->xs->flags & kXsReversedOneway); for (const auto& band : b->xs->bands) cnt[i] += band.counted; }
                    all = all && one[i] && cnt[i] > 0;
                }
                const int leaving = int(arms[0].leaves_node) + int(arms[1].leaves_node) + int(arms[2].leaves_node);
                const bool merge = leaving == 1;
                std::size_t si = 0;
                for (std::size_t i = 0; i < 3; ++i) if (arms[i].leaves_node == merge) si = i;
                const std::size_t xi = (si + 1) % 3, yi = (si + 2) % 3;
                const bool any_link = is_link(hws[0]) || is_link(hws[1]) || is_link(hws[2]);
                const bool candidate = all && (leaving == 1 || leaving == 2) && cnt[si] == cnt[xi] + cnt[yi] && (is_link(hws[xi]) || is_link(hws[yi]));
                const char* dir = leaving == 1 ? "merge" : leaving == 2 ? "diverge" : "-";
                if (candidate) {
                    // ---- C ----
                    const auto d = diagnose_merge_node(*builder, node.osm_id, rules);
                    std::size_t ri, ti;
                    if (d.reason == MergeNodeReason::Ok) {
                        ri = ti = 3;
                        for (std::size_t i = 0; i < 3; ++i) { if (arms[i].way_id == d.ramp_id) ri = i; if (arms[i].way_id == d.through_id) ti = i; }
                    } else {
                        const bool lx = is_link(hws[xi]), ly = is_link(hws[yi]);
                        ri = lx && !ly ? xi : !lx && ly ? yi : (cnt[xi] <= cnt[yi] ? xi : yi);
                        ti = ri == xi ? yi : xi;
                    }
                    std::printf("C node %lld E %.1f N %.1f %s classify %s", static_cast<long long>(node.osm_id), e, n, dir, merge_node_reason_name(d.reason));
                    const std::array<std::pair<char, std::size_t>, 3> roles{{{'R', ri}, {'T', ti}, {'S', si}}};
                    for (const auto& [role, i] : roles) {
                        if (i >= 3) { std::printf(" %c=?", role); continue; }
                        const auto s = section_at(arms[i].way_id, arms[i].ref, !arms[i].leaves_node);
                        std::printf(" %c=%s:%d:%s/%s", role, hws[i].c_str(), cnt[i], s.fit_strict.c_str(), s.fit_served.c_str());
                    }
                    std::printf("\n");
                    continue;
                }
                // ---- N ----
                if (!any_link || !all || !(leaving == 1 || leaving == 2) || cnt[si] != std::max(cnt[xi], cnt[yi])) continue;
                const auto d = diagnose_merge_node(*builder, node.osm_id, rules);
                if (d.reason != MergeNodeReason::LaneSum) continue;
                const bool lx = is_link(hws[xi]), ly = is_link(hws[yi]);
                const std::size_t li = lx && !ly ? xi : !lx && ly ? yi : (cnt[xi] <= cnt[yi] ? xi : yi);   // the link (or the smaller)
                const std::size_t gi = li == xi ? yi : xi;                                                 // the larger / through
                std::printf("N node %lld E %.1f N %.1f %s pattern %d/%d/%d hw %s/%s/%s tags lone %s link %s larger %s", static_cast<long long>(node.osm_id), e, n, dir,
                    cnt[si], cnt[li], cnt[gi], hws[si].c_str(), hws[li].c_str(), hws[gi].c_str(), lane_tag_flags(*ws[si]).c_str(), lane_tag_flags(*ws[li]).c_str(), lane_tag_flags(*ws[gi]).c_str());
                for (const auto& [label, ai] : std::array<std::pair<const char*, std::size_t>, 2>{{{"lone", si}, {"larger", gi}}}) {
                    const auto hops = walk(arms[ai], 2);
                    const Hop* hit = nullptr;
                    for (const auto& hp : hops) if (hp.counted == cnt[ai] + 1 && hp.len < 150) { hit = &hp; break; }
                    std::printf(" %s_hops", label);
                    for (const auto& hp : hops) std::printf(" %d:%.0f@%.0f", hp.counted, hp.len, hp.from_node);
                    if (hops.empty()) std::printf(" -");
                    std::printf(" %s_short_plus1 %s", label, hit ? "yes" : "no");
                    if (hit) std::printf(" way %lld len %.0f at %.0f", static_cast<long long>(hit->way), hit->len, hit->from_node);
                }
                std::printf("\n");
            }
        }
    }
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
    if (std::string_view(argv[4]) == "--diag-0b") {
        if (argc < 9) { std::cerr << "usage: ... --diag-0b E0 N0 E1 N1\n"; return 1; }
        return diag_0b(**store, argv[2], *key, std::atof(argv[5]), std::atof(argv[6]), std::atof(argv[7]), std::atof(argv[8]));
    }
    if (std::string_view(argv[4]) == "--merge-census") {
        if (argc < 9) { std::cerr << "usage: ... --merge-census E0 N0 E1 N1 [--merge-rules on|off]\n"; return 1; }
        const bool on = !(argc >= 11 && std::string_view(argv[10]) == "off");
        return merge_census(**store, argv[2], *key, std::atof(argv[5]), std::atof(argv[6]), std::atof(argv[7]), std::atof(argv[8]), on, argc >= 12 && std::string_view(argv[11]) == "list",
            !(argc >= 13 && std::string_view(argv[12]) == "strict"));
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
