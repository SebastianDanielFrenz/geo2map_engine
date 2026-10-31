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
    // B's added lane is its last counted band; its centre at full width, seen from B's way line.
    const auto l = last_counted(b.xs);
    const auto bd = boundaries(b.xs, 100.0);
    const double centre_mm = std::round((bd[l] + bd[l + 1]) / 2 * 1000.0);
    // The ramp's last 150 m run from (-150, -12) to (0, 0): direction cos = 150 / hypot(150, 12).
    const double ramp_cos = 150.0 / std::hypot(150.0, 12.0);
    REQUIRE(r.xs.offset.mode == OffsetMode::Ramps);
    CHECK(r.xs.offset.start_mm == 0); CHECK(r.xs.offset.start_length_mm == 0);
    CHECK(centre_mm < 0);   // to the right of the way line
    CHECK(std::abs(r.xs.offset.end_mm - std::round(centre_mm / ramp_cos)) <= 1);
    const double offset_m = std::abs(r.xs.offset.end_mm) / 1000.0;
    const auto& ml = rules.merge_lane;
    CHECK(r.xs.offset.end_length_mm == std::clamp<std::int64_t>(static_cast<std::int64_t>(ml.landing_m_per_m * offset_m * 1000.0 + 0.5), ml.landing_min_m * 1000, ml.landing_max_m * 1000));
    // The ramp's end carries the landing record: the single arm and the offset.
    CHECK(r.xs.end_join.kind == JoinKind::Landing); CHECK(r.xs.end_join.neighbour_way_id == s.b); CHECK(r.xs.end_join.raw_delta_mm == r.xs.offset.end_mm);
    CHECK(r.xs.start_join.kind == JoinKind::None);
    (void)a;
}

TEST_CASE("M2 the through pair of a merge joins like a degree-two node: the added lane ramps in from zero", "[layer][road_merge_lane][roads]") {
    const auto s = merge_scene();
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto& rules = builtin_road_rules_de();
    const auto a = resolve_in(*builder, s.a, rules), b = resolve_in(*builder, s.b, rules);
    CHECK(a.xs.end_join.kind == JoinKind::LaneChange); CHECK(a.xs.end_join.neighbour_way_id == s.b);
    CHECK(b.xs.start_join.kind == JoinKind::LaneChange); CHECK(b.xs.start_join.neighbour_way_id == s.a);
    const auto l = last_counted(b.xs);
    CHECK(b.xs.bands[l].start.from_width_mm == 0);
    CHECK(b.xs.bands[l].start.length_mm == rules.find_class("trunk")->taper.lane_add_m * 1000);
    // The shared lanes meet exactly at the node.
    const auto bd = boundaries(b.xs, 0.0);
    CHECK(bd[l] == bd[l + 1]);
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
    none(merge_scene({10, 11, 12}, "trunk_link", -12, 2), "the lanes do not add up (2 = 2 + 1 fails)");
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
