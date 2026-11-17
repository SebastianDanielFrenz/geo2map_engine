// Gore shoulders (ROADS stage 1): where a link lands beside a through carriageway (merge and diverge alike), the two are one pavement
// from the node to the gore nose. The facing shoulders (the link's on the through side, the through way's on the link's side) start at
// width 0 at the node and reach full width at the nose, the first station where the lane-edge gap holds both full shoulders
// (merge_lane.gore_max_m / gore_step_m). One smoothstep ramp per band end (the band format), so between node and nose the shoulders
// follow the gap only approximately; the ribbon test (tests/world/test_gore_ribbons.cpp) bounds the remaining overlap.
//
// Recorded sabotages (2026-11-17; each makes the named tests fail, then reverted):
//   S1 road_cross_section_join.cpp apply_gore_shoulders: ramp the shoulder on the side AWAY from the other way      -> G1, G2, G3 fail
//   S2 apply_gore_shoulders: nose where the gap reaches the LARGER shoulder only (need = max, not sum)               -> G5 fails (the nose
//      moves before the crossing; G1-G3 only check the ramp against the nose - the ribbon test GR1/GR2 catches the short nose)
//   S3 resolve_cross_section: gore disabled (resolve_impl(..., false) on the served path)                           -> G1-G5 fail
//   S4 apply_gore_shoulders: cap the gore ramp by its own band's far ramp only (the first version)                   -> G6 fails
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
struct Resolved { RoadCrossSection xs; CrossSectionTrace trace; double length = 0; };
Resolved resolve_in(const RoadReferenceBuilder& builder, std::int64_t way_id, const RoadRules& rules) {
    const auto stretches = builder.stretches(way_id); REQUIRE(stretches);
    REQUIRE(stretches->size() == 1);
    ReferenceFitLimits limits; limits.allow_curvature_exceptions = true;
    auto fit = builder.fit_stretch(way_id, (*stretches)[0], limits); REQUIRE(fit);
    Resolved out;
    const auto xs = resolve_cross_section(builder, way_id, (*stretches)[0], fit->length_m, rules, &out.trace);
    REQUIRE(xs); REQUIRE(xs->xs.has_value());
    out.xs = *xs->xs; out.length = fit->length_m;
    return out;
}
double band_width(const RoadCrossSection& xs, std::size_t i, double s) {
    std::vector<double> b(xs.bands.size() + 1);
    REQUIRE(xs.boundaries_at(s, b));
    return b[i] - b[i + 1];
}
std::size_t first_counted(const RoadCrossSection& xs) { for (std::size_t i = 0; i < xs.bands.size(); ++i) if (xs.bands[i].counted) return i; return 0; }
std::size_t last_counted(const RoadCrossSection& xs) { for (std::size_t i = xs.bands.size(); i-- > 0;) if (xs.bands[i].counted) return i; return 0; }

// The merge-lane scene of test_road_merge_lane.cpp: A (2 lanes) and B (3 lanes) on +x, the link R landing on B's added lane at node 2.
// `left` mirrors the side: the added lane is B's leftmost and R arrives from the north. R's far node lies `ramp_dy` m off the axis,
// 150 m back: at 40 m (15 degrees) the lanes stay apart; at the merge-lane tests' 12 m the landing S-ramp pulls R's lane over A's
// (the way line converges more slowly than the landing offset decays), a negative gap the rule reports and leaves alone.
struct Scene { RoadGraph graph; std::int64_t a = 10, b = 11, r = 12; };
Scene scene(bool diverge, bool left, int ramp_dy = 40) {
    Scene s;
    const int ramp_y = left ? ramp_dy : -ramp_dy;
    const std::string placement = left ? "right_of:2" : "right_of:1";
    const std::string turns = diverge ? (left ? "slight_left|none|none" : "none|none|slight_right")
                                      : (left ? "merge_to_right|none|none" : "none|none|merge_to_left");
    Tags b_tags = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "3"}, {"placement", placement}, {"turn:lanes", turns}};
    Tags a_tags = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}};
    Tags r_tags = {{"highway", "trunk_link"}, {"oneway", "yes"}, {"lanes", "1"}};
    if (!diverge)
        s.graph = graph_of({{1, {-400, 0}}, {2, {0, 0}}, {3, {200, 0}}, {4, {-150, ramp_y}}},
            {way(s.a, {1, 2}, a_tags), way(s.b, {2, 3}, b_tags), way(s.r, {4, 2}, r_tags)});
    else
        s.graph = graph_of({{1, {-400, 0}}, {2, {0, 0}}, {3, {200, 0}}, {4, {150, ramp_y}}},
            {way(s.b, {1, 2}, b_tags), way(s.a, {2, 3}, a_tags), way(s.r, {2, 4}, r_tags)});
    return s;
}

// The facing shoulder of one way at the node: width 0 at the node, about half at half the nose, full from the nose on.
void check_arm(const Resolved& w, bool at_end, bool facing_left, const char* what) {
    INFO(what);
    const auto& t = w.trace.ends[at_end ? 1 : 0];
    REQUIRE(t.gore);
    CHECK(t.gore_nose_reached);
    REQUIRE(t.gore_nose_mm > 0);
    CHECK(t.gore_shoulder_mm > 0);
    CHECK(t.gore_min_gap_mm > -50);   // the fixtures' lanes never cross
    const std::size_t i = facing_left ? first_counted(w.xs) - 1 : last_counted(w.xs) + 1;
    REQUIRE(i < w.xs.bands.size());
    REQUIRE(w.xs.bands[i].kind == LaneKind::Shoulder);
    const auto& ramp = at_end ? w.xs.bands[i].end : w.xs.bands[i].start;
    CHECK(ramp.from_width_mm == 0); CHECK(ramp.length_mm == t.gore_nose_mm);
    const double full = w.xs.bands[i].width_mm / 1000.0, nose = t.gore_nose_mm / 1000.0;
    const auto st = [&](double d) { return at_end ? w.length - d : d; };
    CHECK(std::abs(band_width(w.xs, i, st(0.0))) < 1e-6);
    CHECK(std::abs(band_width(w.xs, i, st(nose / 2)) - full / 2) < 0.01);   // the smoothstep's midpoint
    CHECK(std::abs(band_width(w.xs, i, st(nose)) - full) < 1e-6);
    CHECK(std::abs(band_width(w.xs, i, st(nose + 5.0)) - full) < 1e-6);
    // The outer shoulder is untouched.
    const std::size_t o = facing_left ? last_counted(w.xs) + 1 : first_counted(w.xs) - 1;
    if (o < w.xs.bands.size() && w.xs.bands[o].kind == LaneKind::Shoulder) {
        const auto& outer = at_end ? w.xs.bands[o].end : w.xs.bands[o].start;
        CHECK(outer.length_mm == 0);
    }
}
void check_scene(bool diverge, bool left) {
    const auto s = scene(diverge, left);
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto& rules = builtin_road_rules_de();
    REQUIRE(rules.merge_lane.gore_max_m == 150); REQUIRE(rules.merge_lane.gore_step_m == 5);
    const auto a = resolve_in(*builder, s.a, rules), b = resolve_in(*builder, s.b, rules), r = resolve_in(*builder, s.r, rules);
    const bool at_end = !diverge;   // a merge ends at node 2 for A and R; an exit starts there
    REQUIRE((at_end ? r.xs.end_join : r.xs.start_join).kind == JoinKind::Landing);
    // R lies right of A for a right-hand ramp: A's right shoulder and R's left shoulder face each other.
    check_arm(a, at_end, left, "the through way");
    check_arm(r, at_end, !left, "the link");
    // The single arm keeps its shoulders.
    const auto& tb = b.trace.ends[at_end ? 0 : 1];
    CHECK_FALSE(tb.gore);
    for (const auto& band : b.xs.bands) if (band.kind == LaneKind::Shoulder) {
        CHECK((at_end ? band.start : band.end).from_width_mm == 0);   // no new ramp at the node ...
        CHECK((at_end ? band.start : band.end).length_mm == 0);       // ... at all
    }
}
} // namespace

TEST_CASE("G1 a merge: both facing shoulders grow from 0 at the node to full width at the gore nose", "[layer][road_gore][roads]") {
    check_scene(false, false);
}
TEST_CASE("G2 a diverge: the same rule read forwards", "[layer][road_gore][roads]") {
    check_scene(true, false);
}
TEST_CASE("G3 the mirrored side: a link on the left of the carriageway, merge and diverge", "[layer][road_gore][roads]") {
    SECTION("merge") { check_scene(false, true); }
    SECTION("diverge") { check_scene(true, true); }
}
TEST_CASE("G4 no landing, no gore: merge_lane disabled leaves the shoulders as they were", "[layer][road_gore][roads]") {
    auto rules = builtin_road_rules_de();
    rules.merge_lane.enabled = false;
    const auto s = scene(false, false);
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    for (const auto id : {s.a, s.b, s.r}) {
        const auto w = resolve_in(*builder, id, rules);
        CHECK_FALSE(w.trace.ends[0].gore); CHECK_FALSE(w.trace.ends[1].gore);
        for (const auto& band : w.xs.bands) if (band.kind == LaneKind::Shoulder) { CHECK(band.start.length_mm == 0); CHECK(band.end.length_mm == 0); }
    }
    // The served rules switch it on.
    const auto r = resolve_in(*builder, s.r, builtin_road_rules_de());
    CHECK(r.trace.ends[1].gore);
}
TEST_CASE("G5 lanes that cross before the nose are reported, not fixed", "[layer][road_gore][roads]") {
    const auto s = scene(false, false, 12);
    const auto builder = RoadReferenceBuilder::make(s.graph); REQUIRE(builder);
    const auto r = resolve_in(*builder, s.r, builtin_road_rules_de());
    const auto& t = r.trace.ends[1];
    REQUIRE(t.gore);
    CHECK(t.gore_min_gap_mm < -500);                                         // measured -952 mm (2026-11-17)
    CHECK(t.gore_min_gap_at_mm > 0); CHECK(t.gore_min_gap_at_mm < t.gore_nose_mm);
}
TEST_CASE("G6 a short through stretch already full of tapers: the gore ramp yields, the section stays within the stretch", "[layer][road_gore][roads]") {
    // A short way A between a lane-count change and the landed node: its tapers already span the whole stretch. The section format
    // (validate_cross_section, road_geom_tile.cpp) bounds the longest start ramp plus the longest end ramp, over every band and the
    // offset, by the stretch length; a gore ramp that only looked at its own band's far end broke whole tiles.
    for (int L : {20, 40}) for (int lanes_p : {1, 3}) {   // a 16 m offset link: the nose (12-40 m) outruns the tapers' end share
        INFO("A " << L << " m, preceded by " << lanes_p << " lanes");
        Tags a_tags = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}};
        Tags p_tags = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", std::to_string(lanes_p)}};
        Tags b_tags = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "3"}, {"placement", "right_of:1"}, {"turn:lanes", "none|none|merge_to_left"}};
        Tags r_tags = {{"highway", "trunk_link"}, {"oneway", "yes"}, {"lanes", "1"}};
        const auto g = graph_of({{1, {-400, 0}}, {5, {-L, 0}}, {2, {0, 0}}, {3, {200, 0}}, {4, {-150, -16}}},
            {way(9, {1, 5}, p_tags), way(10, {5, 2}, a_tags), way(11, {2, 3}, b_tags), way(12, {4, 2}, r_tags)});
        const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
        const auto a = resolve_in(*builder, 10, builtin_road_rules_de());
        CHECK(a.trace.ends[1].gore); CHECK(a.trace.ends[1].gore_nose_mm > 10000);
        std::int64_t ms = a.xs.offset.start_length_mm, me = a.xs.offset.end_length_mm;
        for (const auto& b : a.xs.bands) { ms = std::max<std::int64_t>(ms, b.start.length_mm); me = std::max<std::int64_t>(me, b.end.length_mm); }
        CHECK(ms + me <= static_cast<std::int64_t>(std::ceil(a.length * 1000.0)) + 1);
    }
}
