// Merge lanes and exits at a node of degree three (vault G2M-028, owner site 4: B 8 west of Hoechst): a one-way link that meets two
// one-way carriageways, where the carriageway on the other side has exactly the link's lanes more than the through arm, is a
// lane-add (merge) / lane-drop (diverge) join of the two carriageways, and the link lands on the centre line of the added lane.
//
// Recorded sabotages (2026-10-20; each makes the named tests fail, then reverted):
//   S1 road_cross_section_join.cpp classify_merge_node: skip the landing (return the ramp's landing as 0)       -> M1, M3, M4 fail
//   S2 classify_merge_node: no cos(phi) division (landing = target)                                              -> M1 (1 mm tolerance) fails
//   S3 classify_merge_node: accept a ramp whose highway is not a *_link                                          -> M5 "not a link" fails
//   S4 classify_merge_node: pick the through arm by the SMALLER deflection                                       -> M1, M2, M3, M4 fail
//   S5 classify_merge_node: refuse when the lowest-id arm leaves the node (asymmetric in the arms)                  -> M3, M4 fail
//   S6 resolve_cross_section: give the through arms no partner (skip the synthesized join)                       -> M2, M3, M4 fail
//   S7 classify_merge_node: ignore max_ramp_angle_deg                                                            -> M5b fails
//   S8 classify_merge_node: the ramp's side read the wrong way round (ramp_right inverted)                        -> M1, M5b fail
#include "road_fixture.h"
#include "g2m/layer/road_cross_section.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <string>
using namespace g2m;
namespace {
constexpr std::int64_t m = 1000;
using Tags = std::vector<std::pair<std::string, std::string>>;

RoadGraphWay way(std::int64_t id, std::vector<std::int64_t> nodes, Tags tags) {
    std::sort(tags.begin(), tags.end());
    return RoadGraphWay{id, 3, std::move(nodes), std::move(tags)};
}
RoadGraph graph_of(std::vector<std::pair<std::int64_t, std::pair<int, int>>> pts, std::vector<RoadGraphWay> ways) {
    RoadGraph g;
    for (const auto& [id, p] : pts) g.nodes.push_back({id, {p.first * m, p.second * m}});
    std::sort(g.nodes.begin(), g.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    std::sort(ways.begin(), ways.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    g.ways = std::move(ways);
    for (const auto& w : g.ways) for (std::size_t i = 1; i < w.node_ids.size(); ++i)
        g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    return g;
}
struct Resolved { RoadStretch stretch; RoadCrossSection xs; double length = 0; };
Resolved resolve_in(const RoadReferenceBuilder& builder, std::int64_t way_id, const RoadRules& rules) {
    const auto stretches = builder.stretches(way_id); REQUIRE(stretches);
    REQUIRE(stretches->size() == 1);
    ReferenceFitLimits limits; limits.allow_curvature_exceptions = true;
    auto fit = builder.fit_stretch(way_id, (*stretches)[0], limits); REQUIRE(fit);
    const auto xs = resolve_cross_section(builder, way_id, (*stretches)[0], fit->length_m, rules);
    REQUIRE(xs); REQUIRE(xs->xs.has_value());
    return {(*stretches)[0], *xs->xs, fit->length_m};
}
std::vector<double> boundaries(const RoadCrossSection& xs, double s) {
    std::vector<double> b(xs.bands.size() + 1);
    REQUIRE(xs.boundaries_at(s, b));
    return b;
}
std::size_t last_counted(const RoadCrossSection& xs) {
    for (std::size_t i = xs.bands.size(); i-- > 0;) if (xs.bands[i].counted) return i;
    return 0;
}

// The site-4 shape on +x: the through carriageway A (2 lanes) meets the single carriageway B (3 lanes, the added lane on the right) at
// node 2; the link R arrives from the right (south) at a shallow angle. `ids` are the way ids of A, B, R.
struct Scene { RoadGraph graph; std::int64_t a, b, r; };
Scene merge_scene(std::array<std::int64_t, 3> ids = {10, 11, 12}, const std::string& ramp_highway = "trunk_link", int ramp_y = -12, int b_lanes = 3,
                  bool diverge = false) {
    Scene s{{}, ids[0], ids[1], ids[2]};
    Tags b_tags = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", std::to_string(b_lanes)}};
    if (b_lanes == 3) { b_tags.emplace_back("placement", "right_of:1"); b_tags.emplace_back("turn:lanes", "none|none|merge_to_left"); }
    if (!diverge) {
        s.graph = graph_of({{1, {-400, 0}}, {2, {0, 0}}, {3, {200, 0}}, {4, {-150, ramp_y}}},
            {way(ids[0], {1, 2}, {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}}), way(ids[1], {2, 3}, b_tags),
             way(ids[2], {4, 2}, {{"highway", ramp_highway}, {"oneway", "yes"}, {"lanes", "1"}})});
    } else {
        // Mirrored in time: B (3 lanes) arrives at node 2, A (2 lanes) and the link R leave.
        Tags d_tags = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", std::to_string(b_lanes)}};
        if (b_lanes == 3) { d_tags.emplace_back("placement", "right_of:1"); d_tags.emplace_back("turn:lanes", "none|none|slight_right"); }
        s.graph = graph_of({{1, {-400, 0}}, {2, {0, 0}}, {3, {200, 0}}, {4, {150, ramp_y}}},
            {way(ids[1], {1, 2}, d_tags), way(ids[0], {2, 3}, {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}}),
             way(ids[2], {2, 4}, {{"highway", ramp_highway}, {"oneway", "yes"}, {"lanes", "1"}})});
    }
    return s;
}
} // namespace

TEST_CASE("M1 a merge ramp lands on the centre line of the added lane, over the landing length", "[layer][road_merge_lane][roads]") {
    const auto s = merge_scene();
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto& rules = builtin_road_rules_de();
    const auto a = resolve_in(*builder, s.a, rules), b = resolve_in(*builder, s.b, rules), r = resolve_in(*builder, s.r, rules);
    // B's added lane is its last counted band; its centre AT THE NODE as B serves it there (its other lanes at their join widths,
    // the added lane full width; ROADS stage 3b), seen from B's way line.
    const auto l = last_counted(b.xs);
    const auto bd = boundaries(b.xs, 0.0);
    const double centre_mm = std::round((bd[l] + bd[l + 1]) / 2 * 1000.0);
    // The ramp's last 150 m run from (-150, -12) to (0, 0): direction cos = 150 / hypot(150, 12).
    const double ramp_cos = 150.0 / std::hypot(150.0, 12.0);
    REQUIRE(r.xs.offset.mode == OffsetMode::Ramps);
    CHECK(r.xs.offset.start_mm == 0); CHECK(r.xs.offset.start_length_mm == 0);
    CHECK(centre_mm < 0);   // to the right of the way line
    CHECK(std::abs(r.xs.offset.end_mm - std::round(centre_mm / ramp_cos)) <= 1);
    const double offset_m = std::abs(r.xs.offset.end_mm) / 1000.0;
    const auto& ml = rules.merge_lane;
    const std::int64_t slope_mm = std::clamp<std::int64_t>(static_cast<std::int64_t>(ml.landing_m_per_m * offset_m * 1000.0 + 0.5), ml.landing_min_m * 1000, ml.landing_max_m * 1000);
    // The slope-derived length; this 12 m link still runs inside A's lanes beyond it, so the clearance landing (ROADS stage 2a,
    // test_road_gore.cpp L1-L3) serves a longer one.
    auto no_clear = rules; no_clear.merge_lane.landing_clear_max_m = 0;
    CHECK(resolve_in(*builder, s.r, no_clear).xs.offset.end_length_mm == slope_mm);
    CHECK(r.xs.offset.end_length_mm > slope_mm); CHECK(r.xs.offset.end_length_mm <= ml.landing_clear_max_m * 1000);
    // The ramp's end carries the landing record: the single arm and the offset.
    CHECK(r.xs.end_join.kind == JoinKind::Landing); CHECK(r.xs.end_join.neighbour_way_id == s.b); CHECK(r.xs.end_join.raw_delta_mm == r.xs.offset.end_mm);
    CHECK(r.xs.start_join.kind == JoinKind::None);
    // ROADS stage 3b: R's landed lane starts at the added lane's width and its outer (right) shoulder at B's right shoulder width at
    // the node, both ramping to R's own widths over the landing length; at the node R's lane edges are the added lane's (R's frame,
    // up to the cos residue the frames differ by).
    const auto rl = last_counted(r.xs);
    REQUIRE(rl + 1 < r.xs.bands.size()); REQUIRE(r.xs.bands[rl + 1].kind == LaneKind::Shoulder);
    REQUIRE(l + 1 < b.xs.bands.size()); REQUIRE(b.xs.bands[l + 1].kind == LaneKind::Shoulder);
    const auto added_mm = b.xs.bands[l].width_mm;
    CHECK(r.xs.bands[rl].end == WidthRamp{r.xs.offset.end_length_mm, added_mm});
    const double b_shoulder_mm = std::round((bd[l + 1] - bd[l + 2]) * 1000.0);
    if (r.xs.bands[rl + 1].width_mm != b_shoulder_mm) CHECK(r.xs.bands[rl + 1].end == WidthRamp{r.xs.offset.end_length_mm, static_cast<std::int32_t>(b_shoulder_mm)});
    const auto rd = boundaries(r.xs, r.length);
    CHECK(std::abs(std::round((rd[rl] - rd[rl + 1]) * 1000.0) - added_mm) <= 1);
    CHECK(std::abs(std::round((rd[rl] + rd[rl + 1]) / 2 * 1000.0) - r.xs.offset.end_mm) <= 1);   // centred on the landing
    CHECK(std::abs(std::round((rd[rl + 1] - rd[rl + 2]) * 1000.0) - b_shoulder_mm) <= 1);
    // Far from the node R has its own widths again.
    const auto rf = boundaries(r.xs, r.length - r.xs.offset.end_length_mm / 1000.0 - 1.0);
    CHECK(std::abs(std::round((rf[rl] - rf[rl + 1]) * 1000.0) - r.xs.bands[rl].width_mm) <= 1);
    (void)a;
}

TEST_CASE("M2 the through pair of a merge joins like a degree-two node; the added lane is full width at the node", "[layer][road_merge_lane][roads]") {
    const auto s = merge_scene();
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto& rules = builtin_road_rules_de();
    const auto a = resolve_in(*builder, s.a, rules), b = resolve_in(*builder, s.b, rules);
    CHECK(a.xs.end_join.kind == JoinKind::LaneChange); CHECK(a.xs.end_join.neighbour_way_id == s.b);
    CHECK(b.xs.start_join.kind == JoinKind::LaneChange); CHECK(b.xs.start_join.neighbour_way_id == s.a);
    // G2M-032: the link lands on the added lane, so the lane is already whole at the node (no taper from zero, which left the
    // link's square end standing beside the carriageway).
    const auto l = last_counted(b.xs);
    CHECK(b.xs.bands[l].start.length_mm == 0);
    const auto bd = boundaries(b.xs, 0.0);
    CHECK(std::round((bd[l] - bd[l + 1]) * 1000.0) == b.xs.bands[l].width_mm);
    // B ends in a lane drop at its far node (degree 2, nothing joins it): no taper there, but the section is whole.
    CHECK(b.xs.end_join.kind == JoinKind::None);
}

TEST_CASE("M3 the verdict is the same whatever the way ids and the order of the arms", "[layer][road_merge_lane][roads]") {
    const auto& rules = builtin_road_rules_de();
    std::vector<std::array<std::int64_t, 3>> orders = {{10, 11, 12}, {12, 11, 10}, {11, 10, 12}, {11, 12, 10}, {12, 10, 11}, {10, 12, 11}};
    std::int32_t landing = 0, length = 0;
    for (const auto& ids : orders) {
        INFO("ids " << ids[0] << " " << ids[1] << " " << ids[2]);
        const auto s = merge_scene(ids);
        const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
        const auto r = resolve_in(*builder, s.r, rules);
        const auto a = resolve_in(*builder, s.a, rules);
        REQUIRE(r.xs.end_join.kind == JoinKind::Landing);
        CHECK(a.xs.end_join.kind == JoinKind::LaneChange);
        if (landing == 0) { landing = r.xs.offset.end_mm; length = r.xs.offset.end_length_mm; }
        CHECK(r.xs.offset.end_mm == landing); CHECK(r.xs.offset.end_length_mm == length);
    }
}

TEST_CASE("M4 an exit (diverge) is the same join read forwards: the ramp leaves from the centre line of the lane that ends", "[layer][road_merge_lane][roads]") {
    const auto s = merge_scene({10, 11, 12}, "trunk_link", -12, 3, true);
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto& rules = builtin_road_rules_de();
    const auto a = resolve_in(*builder, s.a, rules), b = resolve_in(*builder, s.b, rules), r = resolve_in(*builder, s.r, rules);
    CHECK(b.xs.end_join.kind == JoinKind::LaneChange); CHECK(a.xs.start_join.kind == JoinKind::LaneChange);
    REQUIRE(r.xs.start_join.kind == JoinKind::Landing);
    CHECK(r.xs.start_join.neighbour_way_id == s.b);
    CHECK(r.xs.offset.mode == OffsetMode::Ramps);
    CHECK(r.xs.offset.end_mm == 0);
    CHECK(r.xs.offset.start_mm < 0);   // the lane on the right: the ramp starts to the right of the way line and drifts out
    CHECK(r.xs.offset.start_length_mm > 0);
    // G2M-032: the lane that ends is whole up to the node, where the link takes it over (the far end of B keeps its own join).
    const auto l = last_counted(b.xs);
    CHECK(b.xs.bands[l].end.length_mm == 0);
    const auto bd = boundaries(b.xs, b.length);
    CHECK(std::round((bd[l] - bd[l + 1]) * 1000.0) == b.xs.bands[l].width_mm);
}

TEST_CASE("M5 what is no merge lane gets no landing and no join", "[layer][road_merge_lane][roads]") {
    const auto& rules = builtin_road_rules_de();
    const auto none = [&](const Scene& s, const char* why) {
        INFO(why);
        const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
        const auto r = resolve_in(*builder, s.r, rules), a = resolve_in(*builder, s.a, rules);
        CHECK(r.xs.end_join.kind == JoinKind::None);
        CHECK(r.xs.offset.mode == OffsetMode::None);
        CHECK(a.xs.end_join.kind == JoinKind::None);
    };
    none(merge_scene({10, 11, 12}, "tertiary"), "not a link");
    none(merge_scene({10, 11, 12}, "trunk_link", -12, 4), "the lanes do not add up (4 = 2 + 1 fails)");   // 2 = 2 is a taper (T1)
}

TEST_CASE("M5b a ramp steeper than max_ramp_angle_deg is a side road, not a merge lane", "[layer][road_merge_lane][roads]") {
    // A wide offset bound, so that the angle alone decides (at 63 degrees the landing offset would also exceed the 8 m bound).
    auto rules = builtin_road_rules_de();
    rules.merge_lane.max_landing_offset_mm = 20000;
    const auto landed = [&](int ramp_y) {
        const auto s = merge_scene({10, 11, 12}, "trunk_link", ramp_y);
        const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
        return resolve_in(*builder, s.r, rules).xs.end_join.kind == JoinKind::Landing;
    };
    CHECK(landed(-214));          // (-150, -214) -> (0, 0): 55 degrees
    CHECK_FALSE(landed(-300));    // 63 degrees
    CHECK_FALSE(landed(214));     // 55 degrees from the left: the added lane is on the right, the ramp must be on its side
}

TEST_CASE("M6 the rules switch: merge_lane.enabled false leaves every end as before", "[layer][road_merge_lane][roads]") {
    auto rules = builtin_road_rules_de();
    rules.merge_lane.enabled = false;
    const auto s = merge_scene();
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto r = resolve_in(*builder, s.r, rules), a = resolve_in(*builder, s.a, rules), b = resolve_in(*builder, s.b, rules);
    CHECK(r.xs.offset.mode == OffsetMode::None); CHECK(r.xs.end_join.kind == JoinKind::None);
    CHECK(a.xs.end_join.kind == JoinKind::None); CHECK(b.xs.start_join.kind == JoinKind::None);
}

// The owner's exit on the B 8 eastbound west of Hoechst (2026-11-02, UTM 32N 465849.8 / 5552329.5), the real ways: B 8 way 313879335 (3 lanes,
// the right one slight_right) arrives at node 48689210 (motorway_junction), way 173934133 (2 lanes) continues, the trunk_link 1455975794 leaves.
// Coordinates are the OSM nodes in UTM 32N, millimetres relative to E 465800 / N 5552300 (rounded to 1 mm).
TEST_CASE("M7 the B 8 exit at Hoechst: the exit lane is whole at the diverge node and the link leaves from its centre line", "[layer][road_merge_lane][roads]") {
    const std::vector<std::pair<std::int64_t, std::pair<std::int64_t, std::int64_t>>> pts = {
        {2098369182, {-12374, 53832}}, {48689210, {55276, 34029}}, {2556744247, {91847, 22520}}, {3263357421, {121140, 13182}},
        {13155763701, {68067, 25452}}, {13155708098, {71673, 23628}}, {13155708099, {75274, 22181}}, {13155708100, {85595, 18490}},
        {2556744245, {92506, 15967}}, {13155708097, {103121, 11462}}, {3263357419, {111575, 6460}}};
    RoadGraph g;
    for (const auto& [id, p] : pts) g.nodes.push_back({id, {p.first, p.second}});
    std::sort(g.nodes.begin(), g.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    g.ways = {way(173934133, {48689210, 2556744247, 3263357421}, {{"highway", "trunk"}, {"lanes", "2"}, {"oneway", "yes"}}),
              way(313879335, {2098369182, 48689210}, {{"highway", "trunk"}, {"lanes", "3"}, {"oneway", "yes"}, {"placement", "right_of:1"}, {"turn:lanes", "none|none|slight_right"}}),
              way(1455975794, {48689210, 13155763701, 13155708098, 13155708099, 13155708100, 2556744245, 13155708097, 3263357419},
                  {{"highway", "trunk_link"}, {"lanes", "1"}, {"oneway", "yes"}})};
    for (const auto& w : g.ways) for (std::size_t i = 1; i < w.node_ids.size(); ++i)
        g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto& rules = builtin_road_rules_de();
    const auto b8 = resolve_in(*builder, 313879335, rules), cont = resolve_in(*builder, 173934133, rules), link = resolve_in(*builder, 1455975794, rules);
    // The three arms are one diverge: the B 8 pair joins as a lane drop, the link takes the exit lane.
    CHECK(b8.xs.end_join.kind == JoinKind::LaneChange); CHECK(cont.xs.start_join.kind == JoinKind::LaneChange);
    REQUIRE(link.xs.start_join.kind == JoinKind::Landing);
    CHECK(link.xs.start_join.neighbour_way_id == 313879335);
    CHECK(link.xs.offset.start_mm < 0);   // to the right of its way line, on the exit lane
    // The exit lane (the third counted band) is full width right up to the node: no tent that ends in a point there.
    const auto l = last_counted(b8.xs);
    REQUIRE(l == 4);
    CHECK(b8.xs.bands[l].end.length_mm == 0);
    const auto bd = boundaries(b8.xs, b8.length);
    CHECK(std::round((bd[l] - bd[l + 1]) * 1000.0) == b8.xs.bands[l].width_mm);
    // The link starts on the exit lane's centre line (the real case measured 5637 mm right of its way line; the lane's own centre, the shoulder
    // and the join's width correction of B 8 decide it, M1/M4 pin the exact rule on the synthetic scenes).
    CHECK(link.xs.offset.start_mm <= -5300); CHECK(link.xs.offset.start_mm >= -6000);
}

// ROADS stage 6: taper nodes. The carriageway keeps its lanes across the node (S == T), so there is no added lane: a one-lane link
// lands with its counted width 0 on the through arm's outermost lane edge on the ramp's side and widens to its own over
// L_t = max(taper_length_m, the clearance-landing length), capped by landing_clear_max_m and the stretch less its far-end clearance.
//
// Recorded sabotages (2026-11-18; each makes the named tests fail, then reverted):
//   S9  classify_merge_node: a taper's target is the outer lane's centre, not its edge                    -> T1, T2, T3, T4, T6 fail
//   S10 resolve_impl (3b block): the taper lane starts at R's own width, not 0 (no width ramp)             -> T1, T2, T3, T4, T6 fail
//   S11 resolve_impl (3b block): a taper end without room for its width ramp keeps the centre on the edge (no half-width shift)
//                                                                                                          -> T7 fails
//   S12 resolve_impl (3b block): a taper end without room for its width ramp gets no same-width merge-zone marker      -> T7, MK7 fail
//   (the markings-zone sabotage is recorded in tests/world/test_merge_markings.cpp, MK5)
namespace {
// The taper scene: A and B both 2 lanes (S == T), or one 2-lane way 10 passing through node 2 (`passing`); the link R as in
// merge_scene with `ramp_lanes` lanes; `ramp_x` its distance along x from the node (merge: west of it; diverge: east).
Scene taper_scene(bool diverge = false, int ramp_y = -12, bool passing = false, int ramp_lanes = 1, int ramp_x = 150) {
    Scene s{{}, 10, passing ? 10 : 11, 12};
    const Tags t2 = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}};
    const Tags tr = {{"highway", "trunk_link"}, {"oneway", "yes"}, {"lanes", std::to_string(ramp_lanes)}};
    std::vector<RoadGraphWay> ways;
    if (passing) ways.push_back(way(10, {1, 2, 3}, t2));
    else { ways.push_back(way(10, {1, 2}, t2)); ways.push_back(way(11, {2, 3}, t2)); }
    if (diverge) { ways.push_back(way(12, {2, 4}, tr)); s.graph = graph_of({{1, {-400, 0}}, {2, {0, 0}}, {3, {200, 0}}, {4, {ramp_x, ramp_y}}}, ways); }
    else { ways.push_back(way(12, {4, 2}, tr)); s.graph = graph_of({{1, {-400, 0}}, {2, {0, 0}}, {3, {200, 0}}, {4, {-ramp_x, ramp_y}}}, ways); }
    return s;
}
// One stretch of a way (a way passing through node 2 has two).
Resolved resolve_nth(const RoadReferenceBuilder& builder, std::int64_t way_id, const RoadRules& rules, std::size_t n) {
    const auto stretches = builder.stretches(way_id); REQUIRE(stretches);
    REQUIRE(stretches->size() > n);
    ReferenceFitLimits limits; limits.allow_curvature_exceptions = true;
    auto fit = builder.fit_stretch(way_id, (*stretches)[n], limits); REQUIRE(fit);
    const auto xs = resolve_cross_section(builder, way_id, (*stretches)[n], fit->length_m, rules);
    REQUIRE(xs); REQUIRE(xs->xs.has_value());
    return {(*stretches)[n], *xs->xs, fit->length_m};
}
std::size_t first_counted(const RoadCrossSection& xs) {
    for (std::size_t i = 0; i < xs.bands.size(); ++i) if (xs.bands[i].counted) return i;
    return 0;
}
// The checks every taper shares. `single` is S's stretch with the node at `s_at_end`; the link's landed end is `r_at_end`.
void check_taper(const Resolved& r, bool r_at_end, const Resolved& single, bool s_at_end, double ramp_cos, bool ramp_right, const RoadRules& rules,
                 double cap_m = 1e9) {
    const auto& ml = rules.merge_lane;
    const auto& join = r_at_end ? r.xs.end_join : r.xs.start_join;
    REQUIRE(join.kind == JoinKind::Landing);
    const std::int32_t off = r_at_end ? r.xs.offset.end_mm : r.xs.offset.start_mm;
    const std::int32_t len = r_at_end ? r.xs.offset.end_length_mm : r.xs.offset.start_length_mm;
    // L_t: at least the taper length (unless capped), within landing_clear_max_m and the stretch.
    CHECK(len >= std::min<double>(ml.taper_length_m * 1000.0, cap_m * 1000.0) - 1);
    CHECK(len <= ml.landing_clear_max_m * 1000); CHECK(len <= r.length * 1000.0);
    // The link's one counted lane ramps from width 0 over L_t; the offset ramps over the same length.
    const auto rl = last_counted(r.xs);
    CHECK(first_counted(r.xs) == rl);
    CHECK((r_at_end ? r.xs.bands[rl].end : r.xs.bands[rl].start) == WidthRamp{len, 0});
    // The single arm keeps its lanes: no band of S ramps at the node.
    for (const auto& b : single.xs.bands) CHECK((s_at_end ? b.end : b.start).length_mm == 0);
    // Target: S's outermost counted lane edge on the ramp's side at the node, seen from S's way line; the link's (zero-width)
    // lane sits on it, in R's frame up to cos(phi).
    const auto sd = boundaries(single.xs, s_at_end ? single.length : 0.0);
    const double edge_mm = std::round((ramp_right ? sd[last_counted(single.xs) + 1] : sd[first_counted(single.xs)]) * 1000.0);
    CHECK(std::abs(off - std::round(edge_mm / ramp_cos)) <= 1);
    const auto rd = boundaries(r.xs, r_at_end ? r.length : 0.0);
    CHECK(std::abs(std::round(rd[rl] * 1000.0) - off) <= 1);
    CHECK(std::abs(std::round(rd[rl + 1] * 1000.0) - off) <= 1);   // width 0: both lane edges on the target
    // The outer shoulder starts at S's shoulder on the ramp's side (the bed edge continues S -> R).
    const auto sb = ramp_right ? last_counted(single.xs) + 1 : first_counted(single.xs) - 1;
    const auto rb = ramp_right ? rl + 1 : rl - 1;
    if (sb < single.xs.bands.size() && single.xs.bands[sb].kind == LaneKind::Shoulder && rb < r.xs.bands.size() && r.xs.bands[rb].kind == LaneKind::Shoulder) {
        const double s_sh = std::abs(sd[sb] - sd[sb + 1]) * 1000.0, r_sh = std::abs(rd[rb] - rd[rb + 1]) * 1000.0;
        CHECK(std::abs(r_sh - s_sh) <= 1);
    }
    // Beyond L_t the link has its own widths.
    const double far = r_at_end ? r.length - len / 1000.0 - 1.0 : len / 1000.0 + 1.0;
    if (far > 0 && far < r.length) {
        const auto rf = boundaries(r.xs, far);
        CHECK(std::abs(std::round((rf[rl] - rf[rl + 1]) * 1000.0) - r.xs.bands[rl].width_mm) <= 1);
    }
}
double cos_of(int dx, int dy) { return std::abs(dx) / std::hypot(static_cast<double>(dx), static_cast<double>(dy)); }
} // namespace

TEST_CASE("T1 a taper merge: S == T, the one-lane link lands with width 0 on the outer lane edge and widens over L_t", "[layer][road_merge_lane][roads]") {
    const auto s = taper_scene();
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto& rules = builtin_road_rules_de();
    const auto d = diagnose_merge_node(*builder, 2, rules);
    REQUIRE(d.reason == MergeNodeReason::Ok); CHECK(d.taper);
    CHECK(d.through_id == s.a); CHECK(d.single_id == s.b); CHECK(d.ramp_id == s.r);
    const auto a = resolve_in(*builder, s.a, rules), b = resolve_in(*builder, s.b, rules), r = resolve_in(*builder, s.r, rules);
    CHECK(a.xs.end_join.kind != JoinKind::None); CHECK(b.xs.start_join.kind != JoinKind::None);   // the through pair joins
    check_taper(r, true, b, false, cos_of(150, 12), true, rules);
}

TEST_CASE("T2 a taper diverge: the link leaves the outer lane edge with width 0", "[layer][road_merge_lane][roads]") {
    const auto s = taper_scene(true);
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto& rules = builtin_road_rules_de();
    REQUIRE(diagnose_merge_node(*builder, 2, rules).taper);
    // Diverge: S (10, 2 lanes) arrives, T (11) and R leave.
    const auto a = resolve_in(*builder, s.a, rules), r = resolve_in(*builder, s.r, rules);
    check_taper(r, false, a, true, cos_of(150, 12), true, rules);
}

TEST_CASE("T3 a taper on the left (mirrored side): the target is the leftmost lane edge", "[layer][road_merge_lane][roads]") {
    const auto s = taper_scene(false, 12);
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto& rules = builtin_road_rules_de();
    REQUIRE(diagnose_merge_node(*builder, 2, rules).taper);
    const auto b = resolve_in(*builder, s.b, rules), r = resolve_in(*builder, s.r, rules);
    check_taper(r, true, b, false, cos_of(150, 12), false, rules);
    CHECK(r.xs.offset.end_mm > 0);   // left of the way line
}

TEST_CASE("T4 a way passing through the node is the single and the through arm, chosen by direction", "[layer][road_merge_lane][roads]") {
    for (const bool diverge : {false, true}) {
        INFO("diverge " << diverge);
        const auto s = taper_scene(diverge, -12, true);
        const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
        const auto& rules = builtin_road_rules_de();
        const auto d = diagnose_merge_node(*builder, 2, rules);
        REQUIRE(d.reason == MergeNodeReason::Ok); CHECK(d.taper);
        CHECK(d.through_id == 10); CHECK(d.single_id == 10);
        // Stretch 0 runs 1 -> 2 (ends at the node), stretch 1 runs 2 -> 3 (starts there).
        const auto w0 = resolve_nth(*builder, 10, rules, 0), w1 = resolve_nth(*builder, 10, rules, 1);
        REQUIRE(w0.stretch.start_ref < w1.stretch.start_ref);
        CHECK(w0.xs.end_join.kind != JoinKind::None); CHECK(w1.xs.start_join.kind != JoinKind::None);
        CHECK(w0.xs.end_join.neighbour_way_id == 10); CHECK(w1.xs.start_join.neighbour_way_id == 10);
        const auto r = resolve_in(*builder, s.r, rules);
        // Merge: S leaves (stretch 1); diverge: S arrives (stretch 0).
        if (diverge) check_taper(r, false, w0, true, cos_of(150, 12), true, rules);
        else check_taper(r, true, w1, false, cos_of(150, 12), true, rules);
    }
}

TEST_CASE("T5 a link with two lanes at an S == T node is refused (taper_ramp_lanes)", "[layer][road_merge_lane][roads]") {
    const auto& rules = builtin_road_rules_de();
    for (const bool passing : {false, true}) {
        INFO("passing " << passing);
        const auto s = taper_scene(false, -12, passing, 2);
        const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
        CHECK(diagnose_merge_node(*builder, 2, rules).reason == MergeNodeReason::TaperRampLanes);
        const auto r = resolve_in(*builder, s.r, rules);
        CHECK(r.xs.end_join.kind == JoinKind::None);
        CHECK(r.xs.offset.mode == OffsetMode::None);
    }
    // The taper switch: taper_length_m 0 (rules built in code) keeps the old refusals.
    auto off = rules; off.merge_lane.taper_length_m = 0;
    const auto s = taper_scene();
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    CHECK(diagnose_merge_node(*builder, 2, off).reason == MergeNodeReason::LaneSum);
    const auto p = taper_scene(false, -12, true);
    const auto pb = RoadReferenceBuilder::make(p.graph); REQUIRE(pb);
    CHECK(diagnose_merge_node(*pb, 2, off).reason == MergeNodeReason::WayPassesThrough);
}

TEST_CASE("T6 a short link: L_t is capped by the stretch", "[layer][road_merge_lane][roads]") {
    // A 40 m link (a dead end at its far node: no far-end clearance) cannot hold a 60 m taper.
    const auto s = taper_scene(false, -3, false, 1, 40);
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto& rules = builtin_road_rules_de();
    REQUIRE(diagnose_merge_node(*builder, 2, rules).taper);
    const auto b = resolve_in(*builder, s.b, rules), r = resolve_in(*builder, s.r, rules);
    REQUIRE(r.length < rules.merge_lane.taper_length_m);
    check_taper(r, true, b, false, cos_of(40, 3), true, rules, r.length - 0.01);
    CHECK(r.xs.offset.end_length_mm <= static_cast<std::int32_t>(std::floor(r.length * 1000.0)));
}

TEST_CASE("T7 no room for the taper (far-end clearance): the link keeps its lane and its facing edge lands on the outer lane edge", "[layer][road_merge_lane][roads]") {
    // A 20 m link starting at a junction with a two-way street: the far-end clearance leaves less than gore_step_m for the width
    // ramp, so the lane cannot start at 0; it keeps its width and lands beside the edge instead of across it.
    auto s = taper_scene(false, -3, false, 1, 20);
    const Tags street = {{"highway", "residential"}};
    s.graph = [&] {
        std::vector<RoadGraphWay> ways = {way(10, {1, 2}, {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}}),
                                          way(11, {2, 3}, {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}}),
                                          way(12, {4, 2}, {{"highway", "trunk_link"}, {"oneway", "yes"}, {"lanes", "1"}}),
                                          way(13, {5, 4, 6}, street)};
        return graph_of({{1, {-400, 0}}, {2, {0, 0}}, {3, {200, 0}}, {4, {-20, -3}}, {5, {-20, -60}}, {6, {-20, 60}}}, ways);
    }();
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto& rules = builtin_road_rules_de();
    REQUIRE(diagnose_merge_node(*builder, 2, rules).taper);
    const auto b = resolve_in(*builder, s.b, rules);
    const auto stretches = builder->stretches(s.r); REQUIRE(stretches); REQUIRE(stretches->size() == 1);
    ReferenceFitLimits limits; limits.allow_curvature_exceptions = true;
    const auto fit = builder->fit_stretch(s.r, (*stretches)[0], limits); REQUIRE(fit);
    CrossSectionTrace tr;
    const auto res = resolve_cross_section(*builder, s.r, (*stretches)[0], fit->length_m, rules, &tr);
    REQUIRE(res); REQUIRE(res->xs.has_value());
    const auto& xs = *res->xs;
    REQUIRE(tr.ends[1].landing_widths_skipped);
    REQUIRE(xs.end_join.kind == JoinKind::Landing);
    const auto rl = last_counted(xs);
    // No width change: the lane keeps its own width; a ramp from that same width over the landing length marks the merge zone.
    CHECK(xs.bands[rl].end.from_width_mm == xs.bands[rl].width_mm);
    CHECK(xs.bands[rl].end.length_mm == xs.offset.end_length_mm);
    CHECK(xs.offset.end_length_mm > 0);
    // The link's left (facing) lane edge on S's rightmost lane edge at the node, in the link's frame up to cos(phi).
    const auto sd = boundaries(b.xs, 0.0), rd = boundaries(xs, fit->length_m);
    const double edge_mm = sd[last_counted(b.xs) + 1] * 1000.0;
    CHECK(std::abs(rd[rl] * 1000.0 - edge_mm / cos_of(20, 3)) <= 2);
    CHECK(std::abs((rd[rl] - rd[rl + 1]) * 1000.0 - xs.bands[rl].width_mm) <= 1);   // the lane is whole, right of the edge
}
