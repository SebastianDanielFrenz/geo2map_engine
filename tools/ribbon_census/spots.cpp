// tools/ribbon_census/spots.cpp - g2m_ribbon_census --spots E0 N0 E1 N1: the served road data at every stretch end in a box,
// tied to the ribbon the game draws there (diagnostics, stage 0c of the merge-lane work). Prints line records:
//
//   W   way hw lanes oneway n E N E N ...      the way's OSM polyline (every way with a stretch in the box), once
//   X   way start_ref end_ref len_m hw counted   one served entry (stretch) with a cross-section in the box
//   B   way start_ref i kind dir counted width_mm start_len start_from end_len end_from   its bands
//   O   way start_ref mode start_mm start_len end_mm end_len   its offset correction
//   G   way start_ref at_end node nose_mm reached min_gap_mm min_gap_at_mm shoulder_mm   the gore-shoulder rule ran at this end
//       (ROADS stage 1, resolve_cross_section's trace: the nose station from the node, whether the lane-edge gap reached both
//       facing shoulders within gore_max_m, the smallest gap before the nose and its station, this way's facing shoulder)
//       ... link_nose_mm ramp_mm overlap_mm overlap_at_mm capped stretch_bound other_shorter trimmed (ROADS stage 2b: the link's nose,
//       the shared ramp length L_g, the worst shoulder overlap with it and its station, no length met the condition within the
//       cap, a stretch set that cap, the other way's stretch allows less, the served section's far-end ramps cut L_g)
//   L   way start_ref at_end node base_mm clear_mm clear_reached final_mm min_gap_mm min_gap_at_mm   the clearance landing ran at
//       this ramp end (ROADS stage 2a: the slope-derived landing length, the first station where the unshifted ramp clears the
//       through lanes, whether it did within the cap, the served landing length, the worst lane-edge gap over it and its station)
//   END way start_ref at_end node deg E N kind neighbour raw_delta_mm      a stretch end in the box with its served join
//   R   way start_ref v ref_E ref_N lat_lo lat_hi l_E l_N r_E r_N w lanes flags   one ribbon row of the entry's road bed
//       (strip 1, from build_road_ribbons on a flat ground, PointSampled): lat_lo/lat_hi = lateral extent from the
//       reference line, l/r = the bed's left/right edge point, w/lanes/flags = the row's vertex attributes
//   NODE node E N deg codes                  a stretch end node; codes: PATCH (a served junction patch), JD:<decline>,
//       LAND (a ramp landed here), TAPER (a LaneChange join here), M:<merge reason> (degree-3 classification), J:<kind>
//   TJ  junctionNode way stretchLen trim landingLenMm estMm  (ROADS stage 3b; estMm: 3c-3's resolve-time trim estimate) a used approach of a built junction whose stretch lands at its other end
//   E   node deg S T R kindS kindT kindR lanesS lanesU dLaneL dLaneR dBedL dBedR intData intDrawn gapTR rside  (ROADS stage 3a) at a node
//       where a ramp landed: in the node's cross frame (S's reference pose at the node, t positive left of S's way direction),
//       S's outer counted-lane and road-bed edges minus those of the union of T and R, signed OUTWARD in mm (positive = S
//       reaches further out than T and R on that side); intData = the largest distance in mm from an interior lane boundary of
//       one side (S, or T and R incl. their facing edges) to the nearest boundary of the other, from the served bands;
//       intDrawn = the same for the boundaries the shader draws (equal division of the counted span, mark_width/lanes);
//       gapTR = the gap in mm between T's and R's facing lane edges at the node (negative = they overlap); rside = L or R, R's side of S;
//       then (ROADS stage 3b) phi = the angle between R's and S's cross lines at the node in millidegrees, resLane / resBed = the cos
//       residue in mm of R's ramp-side lane / bed edge (its distance from R's counted-lane centre times 1 - cos phi: the amount the
//       landing cannot close, since R's end cut is tilted), rRamp = the length in mm of R's counted-lane width ramp at the landed end
//       (0 = no landing width ramp there), skip = 1 when the 3b rule gave R no landing widths at this end (its far end is a junction -
//       degree > 2, no join - and the stretch minus the far-end clearance (ROADS 3c-3: the far trim estimate plus
//       merge_lane.landing_width_far_margin_m, at most landing_width_far_max_m; before it landing_width_far_clear_m) is shorter
//       than gore_step_m; the served resolve's trace); sRef / tRef / rRef
//       = the lateral distance in mm (node frame, signed towards rside) of S's / T's / R's reference point at the node from the node;
//       zoneL / zoneG = R's landing length and its gore shoulder ramp length (0 = none) at the landed end in mm: the merge-zone
//       markings (broken T edge, hidden R edge) run over zoneG, else zoneL (ROADS stage 4a; before it: zoneL, LaneChange only);
//       rCtr = R's counted-lane centre minus the centre of S's ramp-side lanes R lands on (as many as R has), node frame, signed
//       towards rside, mm (-999999 when S's lane boundaries are not contiguous)
//   OV  wayA srA wayB srB s0 s1 maxw meanw E N   two entries' road beds overlapping more than 0.5 m wide over more than
//       10 m (a raster of 0.25 m cells, 1 m station bins of A; s0/s1 stations of A)
//
// Rows are matched to entries by geometry: a vertex at station v belongs to the entry whose reference normal at v
// passes through it (tangential residual < 2 cm, lateral < 40 m). Rows of a tile are the ones the tile owns.
#include "g2m/core/tile_key.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_rules.h"
#include "g2m/world/road_ribbons.h"
#include "g2m/world/terrain_service.h"
#include "g2m/world/ticket.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <iostream>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <vector>

using namespace g2m;
using namespace g2m::world;

namespace {

const char* join_name(JoinKind k) {
    switch (k) {
    case JoinKind::None: return "none";
    case JoinKind::Consistent: return "consistent";
    case JoinKind::LaneChange: return "lane_change";
    case JoinKind::InferredAlignment: return "inferred_alignment";
    case JoinKind::PlacementConflict: return "placement_conflict";
    case JoinKind::Landing: return "landing";
    }
    return "?";
}
const char* decline_name(JunctionDecline d) {
    switch (d) {
    case JunctionDecline::None: return "none";
    case JunctionDecline::ClusterTooLarge: return "cluster_too_large";
    case JunctionDecline::ReachExceeded: return "reach_exceeded";
    case JunctionDecline::SingleApproach: return "single_approach";
    case JunctionDecline::Degenerate: return "degenerate";
    case JunctionDecline::Underground: return "underground";
    case JunctionDecline::MergeLane: return "merge_lane";
    }
    return "?";
}
const char* offset_name(OffsetMode m) {
    switch (m) {
    case OffsetMode::None: return "none";
    case OffsetMode::Ramps: return "ramps";
    case OffsetMode::Hold: return "hold";
    case OffsetMode::Transition: return "transition";
    }
    return "?";
}
std::string tag_of(const RoadGraphWay& w, const char* key) {
    for (const auto& [k, v] : w.tags) if (k == key) return v;
    return "-";
}
const RoadGraphWay* find_way(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.ways.begin(), g.ways.end(), id, [](const RoadGraphWay& w, std::int64_t x) { return w.osm_id < x; });
    return it != g.ways.end() && it->osm_id == id ? &*it : nullptr;
}
const RoadGraphNode* find_node(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.nodes.begin(), g.nodes.end(), id, [](const RoadGraphNode& n, std::int64_t x) { return n.osm_id < x; });
    return it != g.nodes.end() && it->osm_id == id ? &*it : nullptr;
}
using EntryKey = std::pair<std::int64_t, std::uint32_t>;

// One arm's lateral layout at the node, in the node frame (origin o, unit normal n pointing left of S).
struct ArmAtNode {
    bool ok = false;
    double lane_l = 0, lane_r = 0, bed_l = 0, bed_r = 0;   // counted-lane span and road bed (ribbon span_at), node frame
    std::vector<double> bounds;                            // every counted-lane boundary incl. the outer two, node frame
    std::vector<double> drawn;                             // the boundaries the shader draws: equal division of the span
    int lanes = 0;
    double nrx = 0, nry = 0;                               // the arm's own cross-line normal at the node
    double px = 0, py = 0;                                 // the arm's reference point at the node
    double half_l = 0, half_r = 0, bedd_l = 0, bedd_r = 0; // own frame: lane / bed edge distance from the counted-lane centre
    std::int32_t lane_ramp_mm = 0;                         // length of the first counted band's width ramp at this end
};
ArmAtNode arm_at_node(const RoadGeomEntry& en, bool at_end, double ox, double oy, double nx, double ny) {
    ArmAtNode a;
    if (!en.cross_section || !en.profile) return a;
    const auto& xs = *en.cross_section;
    const std::size_t nb = xs.bands.size();
    if (nb == 0 || nb > 32) return a;
    const double s = at_end ? en.profile->reference.length_m : 0.0;
    const auto p = en.profile->reference.at(s);
    if (!p) return a;
    std::array<double, 33> t{};
    if (!xs.boundaries_at(s, std::span<double>(t.data(), nb + 1))) return a;
    const double sh = std::sin(p->heading), ch = std::cos(p->heading);
    const auto proj = [&](double tt) { return (p->x - sh * tt - ox) * nx + (p->y + ch * tt - oy) * ny; };
    // counted lanes as the ribbon draws them (span_at): their extent, the bed around them
    double left = -1e9, right = 1e9;
    for (std::size_t i = 0; i < nb; ++i)
        if (xs.bands[i].counted) {
            left = std::max(left, t[i]); right = std::min(right, t[i + 1]);
            if (t[i] - t[i + 1] > 1e-6) { a.bounds.push_back(proj(t[i])); a.bounds.push_back(proj(t[i + 1])); ++a.lanes; }
        }
    const auto fp = carve_footprint(xs, s);
    if (left <= right) { left = fp.carriageway_left; right = fp.carriageway_right; }
    if (!(left > right)) return a;
    const double bl = std::max(fp.roadbed_left, left), br = std::min(fp.roadbed_right, right);
    const double pl = proj(left), pr = proj(right), qbl = proj(bl), qbr = proj(br);
    a.lane_l = std::max(pl, pr); a.lane_r = std::min(pl, pr);
    a.bed_l = std::max(qbl, qbr); a.bed_r = std::min(qbl, qbr);
    a.nrx = -sh; a.nry = ch; a.px = p->x; a.py = p->y;
    const double c = 0.5 * (left + right);
    a.half_l = left - c; a.half_r = c - right; a.bedd_l = bl - c; a.bedd_r = c - br;
    for (std::size_t i = 0; i < nb; ++i)
        if (xs.bands[i].counted) { a.lane_ramp_mm = (at_end ? xs.bands[i].end : xs.bands[i].start).length_mm; break; }
    std::sort(a.bounds.begin(), a.bounds.end());
    a.bounds.erase(std::unique(a.bounds.begin(), a.bounds.end(), [](double x, double y) { return std::abs(x - y) < 1e-6; }), a.bounds.end());
    if (a.lanes > 0)
        for (int k = 0; k <= a.lanes; ++k) a.drawn.push_back(a.lane_r + (a.lane_l - a.lane_r) * k / a.lanes);
    a.ok = true;
    return a;
}
// Largest distance from an interior value of `x` to the nearest value of `y`, and the other way round.
double interior_mismatch(const std::vector<double>& x, const std::vector<double>& y, double lo_x, double hi_x, double lo_y, double hi_y) {
    double worst = 0;
    const auto one = [&](const std::vector<double>& from, double lo, double hi, const std::vector<double>& to) {
        for (const double v : from) {
            if (v <= lo + 1e-6 || v >= hi - 1e-6) continue;   // outer edges are reported separately
            double best = 1e30;
            for (const double w : to) best = std::min(best, std::abs(v - w));
            worst = std::max(worst, best);
        }
    };
    one(x, lo_x, hi_x, y);
    one(y, lo_y, hi_y, x);
    return worst;
}

} // namespace

int run_spots(TerrainService& svc, const geo::UtmZone& zone, const double box[4]) {
    const RoadRules& rules = builtin_road_rules_de();
    const auto k0 = tile_key_at(zone, 2, box[0], box[1]), k1 = tile_key_at(zone, 2, box[2], box[3]);
    if (!k0 || !k1) return 1;
    const auto in_box = [&](double e, double n, double m = 0) { return e >= box[0] - m && e <= box[2] + m && n >= box[1] - m && n <= box[3] + m; };
    std::set<std::int64_t> ways_printed;
    std::map<std::int64_t, std::set<std::string>> node_codes;
    std::map<std::int64_t, std::tuple<double, double, std::uint32_t>> node_pos;
    std::set<EntryKey> entries_printed;
    struct Ov { std::map<int, int> cells; std::map<int, std::pair<double, double>> at; };
    std::map<std::tuple<std::int64_t, std::uint32_t, std::int64_t, std::uint32_t>, Ov> overlaps;
    const RoadRibbonGround flat = [](double, double) -> std::optional<double> { return 0.0; };
    RoadRibbonOptions point;
    point.drape = RibbonDrape::PointSampled;
    for (std::int32_t ty = k0->y; ty <= k1->y; ++ty)
        for (std::int32_t tx = k0->x; tx <= k1->x; ++tx) {
            TileKey key = *k0; key.x = tx; key.y = ty;
            auto ticket = svc.road_geom(key, Priority::Render);
            ticket.wait_for(std::chrono::minutes(30));
            const auto geometry = ticket.value();
            if (!geometry) {   // a failed tile must not vanish from the counts unnoticed
                if (ticket.state() == TileState::Failed) std::cerr << "tile " << tx << ' ' << ty << " failed: " << ticket.message() << '\n';
                continue;
            }
            const RoadGraph& G = geometry->source.graph;
            const auto builder = RoadReferenceBuilder::make(G);
            std::map<std::int64_t, int> degree;
            for (const auto& e : G.edges) { ++degree[e.from_node]; ++degree[e.to_node]; }
            const double ox = static_cast<double>(key.min_easting()), oy = static_cast<double>(key.min_northing());
            const auto owned = [&](double e, double n) { return std::floor(e / 1024) == key.x && std::floor(n / 1024) == key.y; };
            // entries in the box (any reference point within 60 m of it)
            std::vector<std::size_t> sel;
            for (std::size_t i = 0; i < geometry->entries.size(); ++i) {
                const auto& en = geometry->entries[i];
                if (!en.profile) continue;
                const auto& ref = en.profile->reference;
                bool hit = false;
                for (double s = 0; s <= ref.length_m + 1e-9 && !hit; s += 2.0) {
                    const auto p = ref.at(std::min(s, ref.length_m));
                    if (p && in_box(p->x, p->y, 60)) hit = true;
                }
                if (hit) sel.push_back(i);
            }
            // TJ records (ROADS stage 3b): every used approach of a built junction whose stretch has a Landing join at its
            // OTHER end: way, stretch length, the trim at this junction, the landing length at the other end (mm)
            for (const auto& j : geometry->junctions) {
                if (j.decline != JunctionDecline::None) continue;
                for (const auto& a : j.approaches) {
                    if (a.status != JunctionApproachStatus::Used || a.entry_index >= geometry->entries.size()) continue;
                    const auto& en = geometry->entries[a.entry_index];
                    if (!en.cross_section || !en.profile) continue;
                    const auto& xs = *en.cross_section;
                    const auto& other = a.at_end ? xs.start_join : xs.end_join;
                    if (other.kind != JoinKind::Landing) continue;
                    // ROADS 3c-3: the resolve-time estimate of this trim (the landed end's trace), 0 when none
                    std::int32_t est = 0;
                    CrossSectionTrace etr;
                    if (builder && resolve_cross_section(*builder, en.way_id, en.stretch, en.profile->reference.length_m, rules, &etr))
                        est = etr.ends[a.at_end ? 0 : 1].far_trim_estimate_mm;
                    std::printf("TJ %lld %lld %.3f %.3f %d %d\n", static_cast<long long>(j.nodes.front()), static_cast<long long>(en.way_id),
                                en.profile->reference.length_m, a.trim_m, a.at_end ? xs.offset.start_length_mm : xs.offset.end_length_mm, est);
                }
            }
            // junction codes (owner tile records)
            for (const auto& j : geometry->junctions)
                for (const auto n : j.nodes) {
                    if (j.decline == JunctionDecline::None) node_codes[n].insert("PATCH");
                    else node_codes[n].insert(std::string("JD:") + decline_name(j.decline));
                }
            for (const std::size_t i : sel) {
                const auto& en = geometry->entries[i];
                const auto* way = find_way(G, en.way_id);
                if (!way) continue;
                const EntryKey ek{en.way_id, en.stretch.start_ref};
                const bool first = entries_printed.insert(ek).second;
                if (ways_printed.insert(en.way_id).second) {
                    std::printf("W %lld %s %s %s %zu", static_cast<long long>(en.way_id), tag_of(*way, "highway").c_str(), tag_of(*way, "lanes").c_str(),
                                tag_of(*way, "oneway").c_str(), way->node_ids.size());
                    for (const auto id : way->node_ids) {
                        const auto* n = find_node(G, id);
                        if (n) std::printf(" %.2f %.2f", static_cast<double>(n->position.x) / 1000.0, static_cast<double>(n->position.y) / 1000.0);
                        else std::printf(" nan nan");
                    }
                    std::printf("\n");
                }
                if (!first) continue;
                int counted = 0;
                if (en.cross_section) for (const auto& b : en.cross_section->bands) counted += b.counted ? 1 : 0;
                std::printf("X %lld %u %u %.2f %s %d %s\n", static_cast<long long>(en.way_id), en.stretch.start_ref, en.stretch.end_ref, en.profile->reference.length_m,
                            tag_of(*way, "highway").c_str(), counted, en.cross_section ? "xs" : "noxs");
                if (!en.cross_section) continue;
                const auto& xs = *en.cross_section;
                for (std::size_t b = 0; b < xs.bands.size(); ++b) {
                    const auto& band = xs.bands[b];
                    std::printf("B %lld %u %zu %d %d %d %d %d %d %d %d\n", static_cast<long long>(en.way_id), en.stretch.start_ref, b, static_cast<int>(band.kind),
                                static_cast<int>(band.direction), band.counted ? 1 : 0, band.width_mm, band.start.length_mm, band.start.from_width_mm, band.end.length_mm,
                                band.end.from_width_mm);
                }
                std::printf("O %lld %u %s %d %d %d %d\n", static_cast<long long>(en.way_id), en.stretch.start_ref, offset_name(xs.offset.mode), xs.offset.start_mm,
                            xs.offset.start_length_mm, xs.offset.end_mm, xs.offset.end_length_mm);
                // gore shoulders (ROADS stage 1): the trace of the served resolve, per end where the rule ran
                CrossSectionTrace gtr;
                if (builder && resolve_cross_section(*builder, en.way_id, en.stretch, en.profile->reference.length_m, rules, &gtr))
                    for (int ge = 0; ge < 2; ++ge) {
                        const auto& t = gtr.ends[ge];
                        const std::uint32_t gr = ge ? en.stretch.end_ref : en.stretch.start_ref;
                        if (t.landing_clear)
                            std::printf("L %lld %u %d %lld %d %d %d %d %d %d\n", static_cast<long long>(en.way_id), en.stretch.start_ref, ge,
                                        static_cast<long long>(gr < way->node_ids.size() ? way->node_ids[gr] : -1), t.landing_base_mm, t.landing_clear_mm,
                                        t.landing_clear_reached ? 1 : 0, t.landing_final_mm, t.landing_min_gap_mm, t.landing_min_gap_at_mm);
                        if (!t.gore) continue;
                        std::printf("G %lld %u %d %lld %d %d %d %d %d %d %d %d %d %d %d %d %d\n", static_cast<long long>(en.way_id), en.stretch.start_ref, ge,
                                    static_cast<long long>(gr < way->node_ids.size() ? way->node_ids[gr] : -1), t.gore_nose_mm, t.gore_nose_reached ? 1 : 0,
                                    t.gore_min_gap_mm, t.gore_min_gap_at_mm, t.gore_shoulder_mm, t.gore_link_nose_mm, t.gore_ramp_mm,
                                    t.gore_overlap_mm, t.gore_overlap_at_mm, t.gore_capped ? 1 : 0, t.gore_stretch_bound ? 1 : 0, t.gore_other_shorter ? 1 : 0,
                                    t.gore_ramp_trimmed ? 1 : 0);
                    }
                for (const bool at_end : {false, true}) {
                    const std::uint32_t r = at_end ? en.stretch.end_ref : en.stretch.start_ref;
                    if (r >= way->node_ids.size()) continue;
                    const std::int64_t node = way->node_ids[r];
                    const auto* n = find_node(G, node);
                    if (!n) continue;
                    const double e = static_cast<double>(n->position.x) / 1000.0, nn = static_cast<double>(n->position.y) / 1000.0;
                    const auto& jr = at_end ? xs.end_join : xs.start_join;
                    const int deg = degree[node];
                    std::printf("END %lld %u %d %lld %d %.2f %.2f %s %lld %d\n", static_cast<long long>(en.way_id), en.stretch.start_ref, at_end ? 1 : 0,
                                static_cast<long long>(node), deg, e, nn, join_name(jr.kind), static_cast<long long>(jr.neighbour_way_id), jr.raw_delta_mm);
                    if (owned(e, nn)) node_pos[node] = {e, nn, static_cast<std::uint32_t>(deg)};
                    if (jr.kind == JoinKind::Landing) node_codes[node].insert("LAND");
                    else if (jr.kind == JoinKind::LaneChange) node_codes[node].insert("TAPER");
                    else if (jr.kind != JoinKind::None) node_codes[node].insert(std::string("J:") + join_name(jr.kind));
                    if (deg >= 3 && builder && owned(e, nn)) {
                        const auto d = diagnose_merge_node(*builder, node, rules);
                        node_codes[node].insert(std::string("M:") + merge_node_reason_name(d.reason));
                    }
                }
            }
            // E records (ROADS stage 3a): every node the tile owns where a ramp's end carries the Landing join
            if (builder) {
                std::map<std::pair<std::int64_t, std::int64_t>, std::pair<std::size_t, bool>> at_node;   // (way, node) -> entry, at_end
                for (std::size_t i = 0; i < geometry->entries.size(); ++i) {
                    const auto& en = geometry->entries[i];
                    if (!en.cross_section || !en.profile) continue;
                    const auto* way = find_way(G, en.way_id);
                    if (!way) continue;
                    for (const bool at_end : {false, true}) {
                        const std::uint32_t r = at_end ? en.stretch.end_ref : en.stretch.start_ref;
                        if (r < way->node_ids.size()) at_node[{en.way_id, way->node_ids[r]}] = {i, at_end};
                    }
                }
                for (const auto& [k, v] : at_node) {
                    const auto& rn = geometry->entries[v.first];
                    const auto& rj = v.second ? rn.cross_section->end_join : rn.cross_section->start_join;
                    if (rj.kind != JoinKind::Landing) continue;
                    const std::int64_t node = k.second;
                    const auto* n = find_node(G, node);
                    if (!n) continue;
                    const double e = static_cast<double>(n->position.x) / 1000.0, nn = static_cast<double>(n->position.y) / 1000.0;
                    if (!owned(e, nn) || !in_box(e, nn, 0)) continue;
                    const auto d = diagnose_merge_node(*builder, node, rules);
                    const std::int64_t S = rj.neighbour_way_id, T = d.through_id, R = rn.way_id;
                    const auto si = at_node.find({S, node}), ti = at_node.find({T, node});
                    if (si == at_node.end() || ti == at_node.end()) {
                        std::printf("E %lld %d %lld %lld %lld missing\n", static_cast<long long>(node), degree[node], static_cast<long long>(S),
                                    static_cast<long long>(T), static_cast<long long>(R));
                        continue;
                    }
                    const auto& sn = geometry->entries[si->second.first];
                    const auto& tn = geometry->entries[ti->second.first];
                    const bool s_end = si->second.second;
                    const auto sp = sn.profile->reference.at(s_end ? sn.profile->reference.length_m : 0.0);
                    if (!sp) continue;
                    const double nx = -std::sin(sp->heading), ny = std::cos(sp->heading);
                    const auto as = arm_at_node(sn, s_end, sp->x, sp->y, nx, ny), at = arm_at_node(tn, ti->second.second, sp->x, sp->y, nx, ny),
                               ar = arm_at_node(rn, v.second, sp->x, sp->y, nx, ny);
                    if (!as.ok || !at.ok || !ar.ok) continue;
                    const double ul = std::max(at.lane_l, ar.lane_l), ur = std::min(at.lane_r, ar.lane_r);
                    const double ubl = std::max(at.bed_l, ar.bed_l), ubr = std::min(at.bed_r, ar.bed_r);
                    std::vector<double> ub = at.bounds, ud = at.drawn;
                    ub.insert(ub.end(), ar.bounds.begin(), ar.bounds.end());
                    ud.insert(ud.end(), ar.drawn.begin(), ar.drawn.end());
                    const bool r_left = (ar.lane_l + ar.lane_r) > (at.lane_l + at.lane_r);
                    const double gap = r_left ? ar.lane_r - at.lane_l : at.lane_r - ar.lane_l;
                    const auto& sj = s_end ? sn.cross_section->end_join : sn.cross_section->start_join;
                    const auto& tj = ti->second.second ? tn.cross_section->end_join : tn.cross_section->start_join;
                    const auto mm = [](double x) { return static_cast<long long>(std::llround(x * 1000.0)); };
                    // cos residue of R's ramp-side edges: R's own side that projects outward (towards rside)
                    const double dotn = ar.nrx * nx + ar.nry * ny, cphi = std::min(1.0, std::abs(dotn));
                    const bool own_left_out = (dotn > 0) == r_left;
                    const double res_lane = (own_left_out ? ar.half_l : ar.half_r) * (1.0 - cphi);
                    const double res_bed = (own_left_out ? ar.bedd_l : ar.bedd_r) * (1.0 - cphi);
                    const long long phi_mdeg = std::llround(std::acos(cphi) * 180000.0 / 3.14159265358979323846);
                    const double side = r_left ? 1.0 : -1.0;
                    const auto ref_off = [&](const ArmAtNode& a) { return side * ((a.px - e) * nx + (a.py - nn) * ny); };
                    std::int32_t zone_l = v.second ? rn.cross_section->offset.end_length_mm : rn.cross_section->offset.start_length_mm, zone_g = 0;
                    {
                        const auto& bands = rn.cross_section->bands;
                        int first = -1, last = -1;
                        for (std::size_t i = 0; i < bands.size(); ++i)
                            if (bands[i].counted) { if (first < 0) first = static_cast<int>(i); last = static_cast<int>(i); }
                        const int f = rj.raw_delta_mm < 0 ? first - 1 : last + 1;
                        if (first >= 0 && f >= 0 && f < static_cast<int>(bands.size())) {
                            const auto& b = bands[static_cast<std::size_t>(f)];
                            const auto& r = v.second ? b.end : b.start;
                            if (b.kind == LaneKind::Shoulder && r.length_mm > 0 && r.from_width_mm == 0) zone_g = r.length_mm;
                        }
                    }
                    long long r_ctr = -999999;
                    if (as.bounds.size() == static_cast<std::size_t>(as.lanes) + 1 && ar.lanes <= as.lanes) {
                        const std::size_t n = as.bounds.size() - 1, k = static_cast<std::size_t>(ar.lanes);
                        const double sc = r_left ? 0.5 * (as.bounds[n] + as.bounds[n - k]) : 0.5 * (as.bounds[0] + as.bounds[k]);
                        r_ctr = mm(side * (0.5 * (ar.lane_l + ar.lane_r) - sc));
                    }
                    int skip = 0;
                    if (CrossSectionTrace str; builder && rn.profile &&
                        resolve_cross_section(*builder, rn.way_id, rn.stretch, rn.profile->reference.length_m, rules, &str))
                        skip = str.ends[v.second ? 1 : 0].landing_widths_skipped ? 1 : 0;
                    std::printf("E %lld %d %lld %lld %lld %s %s %s %d %d %lld %lld %lld %lld %lld %lld %lld %c %lld %lld %lld %d %d %lld %lld %lld %d %d %lld\n", static_cast<long long>(node), degree[node],
                                static_cast<long long>(S), static_cast<long long>(T), static_cast<long long>(R), join_name(sj.kind), join_name(tj.kind),
                                join_name(rj.kind), as.lanes, at.lanes + ar.lanes, mm(as.lane_l - ul), mm(ur - as.lane_r), mm(as.bed_l - ubl), mm(ubr - as.bed_r),
                                mm(interior_mismatch(as.bounds, ub, as.lane_r, as.lane_l, ur, ul)), mm(interior_mismatch(as.drawn, ud, as.lane_r, as.lane_l, ur, ul)),
                                mm(gap), r_left ? 'L' : 'R', phi_mdeg, mm(res_lane), mm(res_bed), static_cast<int>(ar.lane_ramp_mm), skip, mm(ref_off(as)), mm(ref_off(at)), mm(ref_off(ar)), static_cast<int>(zone_l), static_cast<int>(zone_g), r_ctr);
                }
            }
            // ribbon rows of the selected entries, and the bed overlap raster
            const RoadRibbonTile mesh = build_road_ribbons(*geometry, key, flat, 0.0, {}, point);
            struct Cand { std::size_t entry; double lo_e, lo_n, hi_e, hi_n; };
            std::vector<Cand> cands;
            for (const std::size_t i : sel) {
                const auto& ref = geometry->entries[i].profile->reference;
                Cand c{i, 1e30, 1e30, -1e30, -1e30};
                for (double s = 0; s <= ref.length_m + 1e-9; s += 2.0) {
                    const auto p = ref.at(std::min(s, ref.length_m));
                    if (!p) continue;
                    c.lo_e = std::min(c.lo_e, p->x); c.hi_e = std::max(c.hi_e, p->x); c.lo_n = std::min(c.lo_n, p->y); c.hi_n = std::max(c.hi_n, p->y);
                }
                if (const auto p = ref.at(ref.length_m)) { c.lo_e = std::min(c.lo_e, p->x); c.hi_e = std::max(c.hi_e, p->x); c.lo_n = std::min(c.lo_n, p->y); c.hi_n = std::max(c.hi_n, p->y); }
                cands.push_back(c);
            }
            // vertex -> entry (or npos)
            std::vector<std::size_t> owner(mesh.vertices.size(), static_cast<std::size_t>(-1));
            std::vector<double> lateral(mesh.vertices.size(), 0.0);
            for (std::size_t vi = 0; vi < mesh.vertices.size(); ++vi) {
                const auto& v = mesh.vertices[vi];
                if ((static_cast<int>(v.flags) & (16 | 32)) || v.verge > 0.5f || v.lanes == 0) continue;
                const double e = ox + v.x, n = oy + v.y;
                double best = 0.02;
                for (const auto& c : cands) {
                    if (e < c.lo_e - 40 || e > c.hi_e + 40 || n < c.lo_n - 40 || n > c.hi_n + 40) continue;
                    const auto& ref = geometry->entries[c.entry].profile->reference;
                    const double s = std::clamp(static_cast<double>(v.v), 0.0, ref.length_m);
                    const auto p = ref.at(s);
                    if (!p) continue;
                    const double dx = e - p->x, dy = n - p->y, ce = std::cos(p->heading), se = std::sin(p->heading);
                    const double tan_r = std::abs(dx * ce + dy * se), lat = -dx * se + dy * ce;
                    if (tan_r < best && std::abs(lat) < 40) { best = tan_r; owner[vi] = c.entry; lateral[vi] = lat; }
                }
            }
            // rows: per (entry, v) the lateral extent
            struct RowAcc { double lo = 1e30, hi = -1e30; float w = 0, ln = 0, fl = 0; };
            std::map<std::pair<std::size_t, long long>, RowAcc> rows;
            for (std::size_t vi = 0; vi < mesh.vertices.size(); ++vi) {
                if (owner[vi] == static_cast<std::size_t>(-1)) continue;
                const auto& v = mesh.vertices[vi];
                auto& r = rows[{owner[vi], std::llround(static_cast<double>(v.v) * 1000.0)}];
                r.lo = std::min(r.lo, lateral[vi]); r.hi = std::max(r.hi, lateral[vi]); r.w = v.width; r.ln = v.lanes; r.fl = std::max(r.fl, v.flags);
            }
            for (const auto& [k, r] : rows) {
                const auto& en = geometry->entries[k.first];
                const double s = static_cast<double>(k.second) / 1000.0;
                const auto p = en.profile->reference.at(std::clamp(s, 0.0, en.profile->reference.length_m));
                if (!p) continue;
                const double nx = -std::sin(p->heading), ny = std::cos(p->heading);
                std::printf("R %lld %u %.3f %.2f %.2f %.3f %.3f %.2f %.2f %.2f %.2f %.3f %g %g\n", static_cast<long long>(en.way_id), en.stretch.start_ref, s, p->x, p->y,
                            r.lo, r.hi, p->x + nx * r.hi, p->y + ny * r.hi, p->x + nx * r.lo, p->y + ny * r.lo, static_cast<double>(r.w), static_cast<double>(r.ln),
                            static_cast<double>(r.fl));
            }
            // raster of the bed triangles inside the box: first owner wins, a second entry records an overlap cell
            const double cell = 0.25;
            const double r_lo_e = std::max(ox, box[0]), r_lo_n = std::max(oy, box[1]), r_hi_e = std::min(ox + 1024, box[2]), r_hi_n = std::min(oy + 1024, box[3]);
            if (r_hi_e <= r_lo_e || r_hi_n <= r_lo_n) continue;
            const int nx_c = static_cast<int>(std::ceil((r_hi_e - r_lo_e) / cell)), ny_c = static_cast<int>(std::ceil((r_hi_n - r_lo_n) / cell));
            std::vector<std::int32_t> cell_owner(static_cast<std::size_t>(nx_c) * static_cast<std::size_t>(ny_c), -1);
            std::vector<float> cell_station(cell_owner.size(), 0.f);
            for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
                const std::size_t a = static_cast<std::size_t>(mesh.indices[t]), b = static_cast<std::size_t>(mesh.indices[t + 1]), c = static_cast<std::size_t>(mesh.indices[t + 2]);
                if (owner[a] == static_cast<std::size_t>(-1) || owner[a] != owner[b] || owner[a] != owner[c]) continue;
                const std::size_t ei = owner[a];
                const double xa = ox + mesh.vertices[a].x, ya = oy + mesh.vertices[a].y, xb = ox + mesh.vertices[b].x, yb = oy + mesh.vertices[b].y,
                             xc = ox + mesh.vertices[c].x, yc = oy + mesh.vertices[c].y;
                const double den = (yb - yc) * (xa - xc) + (xc - xb) * (ya - yc);
                if (std::abs(den) < 1e-12) continue;
                const int i0 = std::max(0, static_cast<int>(std::floor((std::min({xa, xb, xc}) - r_lo_e) / cell))),
                          i1 = std::min(nx_c - 1, static_cast<int>(std::floor((std::max({xa, xb, xc}) - r_lo_e) / cell)));
                const int j0 = std::max(0, static_cast<int>(std::floor((std::min({ya, yb, yc}) - r_lo_n) / cell))),
                          j1 = std::min(ny_c - 1, static_cast<int>(std::floor((std::max({ya, yb, yc}) - r_lo_n) / cell)));
                for (int j = j0; j <= j1; ++j)
                    for (int i = i0; i <= i1; ++i) {
                        const double px = r_lo_e + (i + 0.5) * cell, py = r_lo_n + (j + 0.5) * cell;
                        const double l0 = ((yb - yc) * (px - xc) + (xc - xb) * (py - yc)) / den, l1 = ((yc - ya) * (px - xc) + (xa - xc) * (py - yc)) / den, l2 = 1 - l0 - l1;
                        if (l0 < 0 || l1 < 0 || l2 < 0) continue;
                        const double st = l0 * mesh.vertices[a].v + l1 * mesh.vertices[b].v + l2 * mesh.vertices[c].v;
                        const std::size_t ci = static_cast<std::size_t>(j) * static_cast<std::size_t>(nx_c) + static_cast<std::size_t>(i);
                        if (cell_owner[ci] < 0) { cell_owner[ci] = static_cast<std::int32_t>(ei); cell_station[ci] = static_cast<float>(st); continue; }
                        const std::size_t ej = static_cast<std::size_t>(cell_owner[ci]);
                        if (ej == ei) continue;
                        const auto& A = geometry->entries[ej];
                        const auto& Bn = geometry->entries[ei];
                        auto& ov = overlaps[{A.way_id, A.stretch.start_ref, Bn.way_id, Bn.stretch.start_ref}];
                        const int bin = static_cast<int>(std::floor(cell_station[ci]));
                        ++ov.cells[bin];
                        ov.at[bin] = {px, py};
                    }
            }
        }
    for (const auto& [node, codes] : node_codes) {
        const auto it = node_pos.find(node);
        if (it == node_pos.end()) continue;
        const auto [e, n, deg] = it->second;
        if (!in_box(e, n, 0)) continue;
        std::printf("NODE %lld %.2f %.2f %u", static_cast<long long>(node), e, n, deg);
        for (const auto& c : codes) std::printf(" %s", c.c_str());
        std::printf("\n");
    }
    for (const auto& [node, pos] : node_pos)
        if (!node_codes.contains(node) && in_box(std::get<0>(pos), std::get<1>(pos), 0))
            std::printf("NODE %lld %.2f %.2f %u\n", static_cast<long long>(node), std::get<0>(pos), std::get<1>(pos), std::get<2>(pos));
    for (const auto& [k, ov] : overlaps) {
        // runs of consecutive 1 m bins of A wider than 0.5 m
        int run0 = 0, prev = 0, len = 0;
        double maxw = 0, sumw = 0;
        const auto flush = [&]() {
            if (len > 10) {
                const auto at = ov.at.at(run0 + len / 2);
                std::printf("OV %lld %u %lld %u %d %d %.2f %.2f %.1f %.1f\n", static_cast<long long>(std::get<0>(k)), std::get<1>(k), static_cast<long long>(std::get<2>(k)),
                            std::get<3>(k), run0, run0 + len, maxw, sumw / len, at.first, at.second);
            }
            len = 0; maxw = 0; sumw = 0;
        };
        for (const auto& [bin, cells] : ov.cells) {
            const double w = cells * 0.0625;
            if (w <= 0.5) { flush(); continue; }
            if (len > 0 && bin != prev + 1) flush();
            if (len == 0) run0 = bin;
            ++len; prev = bin; maxw = std::max(maxw, w); sumw += w;
        }
        flush();
    }
    return 0;
}
