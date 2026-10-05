// Street fidelity P1 slice 4: join resolution (design section 4).
#include "road_fixture.h"
#include "g2m/layer/road_cross_section.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <optional>
#include <string>
#include <tuple>
using namespace g2m;
namespace {
const RoadRules& rules() { return builtin_road_rules_de(); }
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
struct Resolved { RoadStretch stretch; RoadCrossSection xs; RoadReferenceLine ref; };
Resolved resolve_at(const RoadReferenceBuilder& builder, std::int64_t way_id, std::size_t stretch_index) {
    const auto stretches = builder.stretches(way_id); REQUIRE(stretches);
    REQUIRE(stretch_index < stretches->size());
    ReferenceFitLimits limits; limits.allow_curvature_exceptions = true;
    auto fit = builder.fit_stretch(way_id, (*stretches)[stretch_index], limits); REQUIRE(fit);
    const auto xs = resolve_cross_section(builder, way_id, (*stretches)[stretch_index], fit->length_m, rules());
    REQUIRE(xs); REQUIRE(xs->xs.has_value());
    return {(*stretches)[stretch_index], *xs->xs, std::move(*fit)};
}
std::size_t first_counted(const RoadCrossSection& xs) {
    for (std::size_t i = 0; i < xs.bands.size(); ++i) if (xs.bands[i].counted) return i;
    return xs.bands.size();
}
std::size_t last_counted(const RoadCrossSection& xs) {
    for (std::size_t i = xs.bands.size(); i-- > 0;) if (xs.bands[i].counted) return i;
    return 0;
}
// The lateral position (m) of the anchor point at station s.
[[maybe_unused]] double anchor_t(const RoadCrossSection& xs, double s) {
    std::vector<double> b(xs.bands.size() + 1);
    REQUIRE(xs.boundaries_at(s, b));
    const auto a = xs.anchor.band;
    return xs.anchor.edge == AnchorEdge::Left ? b[a] : xs.anchor.edge == AnchorEdge::Right ? b[a + 1] : (b[a] + b[a + 1]) / 2;
}
std::vector<double> boundaries(const RoadCrossSection& xs, double s) {
    std::vector<double> b(xs.bands.size() + 1);
    REQUIRE(xs.boundaries_at(s, b));
    return b;
}
// The stretch of `way_id` that has `node` at one of its ends.
struct AtNode { Resolved r; bool at_end = false; };
AtNode stretch_at_node(const RoadGraph& graph, const RoadReferenceBuilder& builder, std::int64_t way_id, std::int64_t node) {
    const RoadGraphWay* way = nullptr;
    for (const auto& x : graph.ways) if (x.osm_id == way_id) way = &x;
    REQUIRE(way);
    const auto stretches = builder.stretches(way_id); REQUIRE(stretches);
    for (std::size_t i = 0; i < stretches->size(); ++i) {
        const auto st = (*stretches)[i];
        if (way->node_ids[st.start_ref] == node) return {resolve_at(builder, way_id, i), false};
        if (way->node_ids[st.end_ref] == node) return {resolve_at(builder, way_id, i), true};
    }
    FAIL("no stretch of way " << way_id << " ends at node " << node);
    return {};
}
} // namespace

TEST_CASE("A consistent join has no ramps and no offset, from either side", "[layer][road_joins][roads]") {
    // 1 -> 2 -> 3 straight; two residential ways joined at node 2 (degree 2), both untagged 1+1.
    const auto g = graph_of({{1, {0, 0}}, {2, {200, 0}}, {3, {400, 0}}},
        {way(10, {1, 2}, {{"highway", "residential"}}), way(11, {2, 3}, {{"highway", "residential"}})});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto a = resolve_at(*builder, 10, 0), b = resolve_at(*builder, 11, 0);
    CHECK(a.xs.end_join.kind == JoinKind::Consistent); CHECK(a.xs.end_join.neighbour_way_id == 11); CHECK(a.xs.end_join.raw_delta_mm == 0);
    CHECK(b.xs.start_join.kind == JoinKind::Consistent); CHECK(b.xs.start_join.neighbour_way_id == 10);
    CHECK(a.xs.start_join.kind == JoinKind::None); CHECK(b.xs.end_join.kind == JoinKind::None);
    CHECK(a.xs.offset.mode == OffsetMode::None); CHECK(b.xs.offset.mode == OffsetMode::None);
    for (const auto& band : a.xs.bands) { CHECK(band.start.length_mm == 0); CHECK(band.end.length_mm == 0); }
    CHECK(a.xs.length_m == a.ref.length_m);
    CHECK(boundaries(a.xs, a.xs.length_m) == boundaries(b.xs, 0.0));
}

TEST_CASE("An added right lane ramps in from zero inside the wider way", "[layer][road_joins][roads]") {
    // F1 shape: U is an untagged 2-lane oneway, D has 3 lanes with right_of:1 and a merge on the third.
    const auto g = graph_of({{1, {0, 0}}, {2, {300, 0}}, {3, {600, 0}}},
        {way(10, {1, 2}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}}),
         way(11, {2, 3}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"placement", "right_of:1"},
                          {"turn:lanes", "none|none|merge_to_left"}})});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto u = resolve_at(*builder, 10, 0), d = resolve_at(*builder, 11, 0);
    CHECK(u.xs.end_join.kind == JoinKind::LaneChange); CHECK(u.xs.end_join.raw_delta_mm == 0);
    CHECK(d.xs.start_join.kind == JoinKind::LaneChange);
    CHECK(u.xs.offset.mode == OffsetMode::None); CHECK(d.xs.offset.mode == OffsetMode::None);
    const auto f = first_counted(d.xs), l = last_counted(d.xs);
    CHECK(d.xs.bands[l].start.length_mm == rules().find_class("primary")->taper.lane_add_m * 1000);
    CHECK(d.xs.bands[l].start.from_width_mm == 0);
    CHECK(d.xs.bands[f].start.length_mm == 0); CHECK(d.xs.bands[f + 1].start.length_mm == 0);
    // The shared lanes and the lane line meet exactly; the added lane has zero width at the join.
    const auto bu = boundaries(u.xs, u.xs.length_m), bd = boundaries(d.xs, 0.0);
    const auto fu = first_counted(u.xs);
    CHECK(bu[fu] == bd[f]); CHECK(bu[fu + 1] == bd[f + 1]); CHECK(bu[fu + 2] == bd[f + 2]);
    CHECK(bd[l] == bd[l + 1]);
    CHECK(d.xs.band_width_m(l, d.xs.length_m / 2) == d.xs.bands[l].width_mm / 1000.0);
}

TEST_CASE("The side with more lanes absorbs an untagged residual and holds it over a junction end", "[layer][road_joins][roads]") {
    // U: 2 lanes (anchor right of lane 1), D: 3 lanes (anchor centre of lane 2): the lanes differ by half a lane.
    const auto g = graph_of({{1, {0, 0}}, {2, {300, 0}}, {3, {600, 0}}, {4, {600, 100}}, {5, {600, -100}}},
        {way(10, {1, 2}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}}),
         way(11, {2, 3}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}}),
         way(12, {3, 4}, {{"highway", "residential"}}), way(13, {3, 5}, {{"highway", "residential"}})});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto u = resolve_at(*builder, 10, 0), d = resolve_at(*builder, 11, 0);
    // Delta = t_D - t_U at the left edge of the first lane: 5250 - 3500 = 1750 mm; D has more lanes and absorbs.
    CHECK(u.xs.end_join.kind == JoinKind::InferredAlignment); CHECK(u.xs.end_join.raw_delta_mm == 1750);
    CHECK(d.xs.start_join.kind == JoinKind::InferredAlignment); CHECK(d.xs.start_join.raw_delta_mm == 1750);
    CHECK(u.xs.offset.mode == OffsetMode::None);
    // D's other end (node 3) is a junction: the whole stretch shifts.
    CHECK(d.xs.offset.mode == OffsetMode::Hold); CHECK(d.xs.offset.start_mm == -1750);
    const auto bu = boundaries(u.xs, u.xs.length_m), bd = boundaries(d.xs, 0.0);
    const auto fu = first_counted(u.xs), fd = first_counted(d.xs);
    CHECK(std::fabs(bu[fu] - bd[fd]) < 1e-9); CHECK(std::fabs(bu[fu + 1] - bd[fd + 1]) < 1e-9);
}

TEST_CASE("Two explicit placements split a residual between both stretches", "[layer][road_joins][roads]") {
    // Both far ends are dead ends, so each side holds its share over the whole stretch.
    const auto g = graph_of({{1, {0, 0}}, {2, {300, 0}}, {3, {600, 0}}},
        {way(10, {1, 2}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"placement", "right_of:1"}}),
         way(11, {2, 3}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"placement", "right_of:2"}})});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto u = resolve_at(*builder, 10, 0), d = resolve_at(*builder, 11, 0);
    CHECK(u.xs.end_join.kind == JoinKind::PlacementConflict); CHECK(d.xs.start_join.kind == JoinKind::PlacementConflict);
    CHECK(u.xs.end_join.raw_delta_mm == 3500); CHECK(d.xs.start_join.raw_delta_mm == 3500);
    // D takes floor(3500/2) = 1750 towards U, U takes the remaining 1750.
    CHECK(d.xs.offset.mode == OffsetMode::Hold); CHECK(d.xs.offset.start_mm == -1750);
    CHECK(u.xs.offset.mode == OffsetMode::Hold); CHECK(u.xs.offset.start_mm == 1750);
    const auto bu = boundaries(u.xs, u.xs.length_m), bd = boundaries(d.xs, 0.0);
    const auto fu = first_counted(u.xs), fd = first_counted(d.xs);
    CHECK(std::fabs(bu[fu] - bd[fd]) < 1e-9);
    // An odd residual: the downstream side takes floor(delta / 2). Lane 2 of D is 3.002 m against 3.0 m, equal
    // provenance, so both ramp to 3001 and the left edge of lane 1 of D sits at 3000 + 3001 = 6001 against 3000.
    const auto g2 = graph_of({{1, {0, 0}}, {2, {300, 0}}, {3, {600, 0}}},
        {way(10, {1, 2}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"placement", "right_of:1"}, {"width:lanes", "3.0|3.0"}}),
         way(11, {2, 3}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"placement", "right_of:2"}, {"width:lanes", "3.0|3.002"}})});
    const auto b2 = RoadReferenceBuilder::make(g2); REQUIRE(b2);
    const auto u2 = resolve_at(*b2, 10, 0), d2 = resolve_at(*b2, 11, 0);
    CHECK(u2.xs.end_join.raw_delta_mm == 3001); CHECK(d2.xs.start_join.raw_delta_mm == 3001);
    CHECK(d2.xs.offset.start_mm == -1500); CHECK(u2.xs.offset.start_mm == 1501);
    CHECK(u2.xs.offset.start_mm - d2.xs.offset.start_mm == u2.xs.end_join.raw_delta_mm);
    const auto bu2 = boundaries(u2.xs, u2.xs.length_m), bd2 = boundaries(d2.xs, 0.0);
    CHECK(std::fabs(bu2[first_counted(u2.xs)] - bd2[first_counted(d2.xs)]) < 1e-9);
    // The width ramp: lane 2 of D starts at the mean of both sides and returns to its own 3.002 m.
    CHECK(d2.xs.bands[first_counted(d2.xs) + 1].start.from_width_mm == 3001);
    CHECK(d2.xs.bands[first_counted(d2.xs) + 1].start.length_mm == rules().find_class("primary")->taper.width_change_m * 1000);
}

TEST_CASE("Both stretches of a join, in any orientation, derive one outcome", "[layer][road_joins][roads]") {
    // A is a two-way 1+1 road; B is two-way with 1 backward and 2 forward lanes, the second forward lane added on the
    // right (slight_right). Four layouts: A's end meets B's start, A's start meets B's end (same direction) and both
    // opposite orientations; the lower way id switches between A and B. B without placement: its default anchor is the
    // middle of its three lanes, half a lane away from A's centre line, so B (more lanes) absorbs +- 1750 mm.
    for (const bool a_lower : {true, false}) for (const int layout : {0, 1, 2, 3}) {
        const std::int64_t ia = a_lower ? 10 : 11, ib = a_lower ? 11 : 10;
        // 0: A 1->2, B 2->3.  1: A 2->3, B 1->2.  2: A 1->2, B 3->2 (head to head).  3: A 2->1, B 2->3 (tail to tail).
        const std::vector<std::int64_t> na = layout == 1 ? std::vector<std::int64_t>{2, 3} : layout == 3 ? std::vector<std::int64_t>{2, 1} : std::vector<std::int64_t>{1, 2};
        const std::vector<std::int64_t> nb = layout == 1 ? std::vector<std::int64_t>{1, 2} : layout == 2 ? std::vector<std::int64_t>{3, 2} : std::vector<std::int64_t>{2, 3};
        const auto g = graph_of({{1, {0, 0}}, {2, {300, 0}}, {3, {600, 0}}},
            {way(ia, na, {{"highway", "tertiary"}}),
             way(ib, nb, {{"highway", "tertiary"}, {"lanes", "3"}, {"lanes:forward", "2"}, {"lanes:backward", "1"}, {"turn:lanes:forward", "none|slight_right"}})});
        const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
        const auto a = stretch_at_node(g, *builder, ia, 2), b = stretch_at_node(g, *builder, ib, 2);
        INFO("a_lower=" << a_lower << " layout=" << layout);
        const auto& ja = a.at_end ? a.r.xs.end_join : a.r.xs.start_join;
        const auto& jb = b.at_end ? b.r.xs.end_join : b.r.xs.start_join;
        CHECK(ja.kind == JoinKind::InferredAlignment); CHECK(jb.kind == JoinKind::InferredAlignment);
        CHECK(ja.neighbour_way_id == ib); CHECK(jb.neighbour_way_id == ia);
        CHECK(ja.raw_delta_mm == jb.raw_delta_mm);
        // The three-lane way absorbs; A keeps its section.
        CHECK(a.r.xs.offset.mode == OffsetMode::None); CHECK(b.r.xs.offset.mode == OffsetMode::Hold);
        // The centre line (the boundary between the backward and the forward group) meets in the plane.
        const auto join_point = [&](const AtNode& x, std::size_t boundary_index) {
            const double s = x.at_end ? x.r.xs.length_m : 0.0;
            const auto t = boundaries(x.r.xs, s)[boundary_index];
            const auto p = lateral_point(x.r.ref, s, t); REQUIRE(p);
            return *p;
        };
        // A: [verge, bwd, fwd, verge]; B: [verge, bwd, fwd, fwd, verge]. The centre line is boundary index 2 of both.
        const auto pa = join_point(a, 2), pb = join_point(b, 2);
        CHECK(std::hypot(pa.x - pb.x, pa.y - pb.y) < 1e-6);
    }
}

TEST_CASE("Explicit placements decide an ambiguous added side by the residual", "[layer][road_joins][roads]") {
    // U oneway 2 lanes anchor right_of:1; D 3 lanes right_of:1 without turn tokens: only the right addition has delta 0.
    const auto g = graph_of({{1, {0, 0}}, {2, {300, 0}}, {3, {600, 0}}},
        {way(10, {1, 2}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}}),
         way(11, {2, 3}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"placement", "right_of:1"}})});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto u = resolve_at(*builder, 10, 0), d = resolve_at(*builder, 11, 0);
    CHECK(u.xs.end_join.kind == JoinKind::LaneChange); CHECK(u.xs.end_join.raw_delta_mm == 0);
    const auto l = last_counted(d.xs);
    CHECK(d.xs.bands[l].start.from_width_mm == 0); CHECK(d.xs.bands[l].start.length_mm > 0);
    CHECK(d.xs.bands[first_counted(d.xs)].start.length_mm == 0);
}

TEST_CASE("A transition way absorbs at its own ends and varies smoothly", "[layer][road_joins][roads]") {
    // 10 (2 lanes) -> 11 (transition, 2 lanes) -> 12 (3 lanes, default centre): the delta at the second join is absorbed by 11.
    const auto g = graph_of({{1, {0, 0}}, {2, {300, 0}}, {3, {900, 0}}, {4, {1200, 0}}},
        {way(10, {1, 2}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}}),
         way(11, {2, 3}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}, {"placement", "transition"}}),
         way(12, {3, 4}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}})});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto a = resolve_at(*builder, 10, 0), t = resolve_at(*builder, 11, 0), c = resolve_at(*builder, 12, 0);
    CHECK(t.xs.anchor.source == PlacementSource::Transition);
    CHECK(a.xs.end_join.raw_delta_mm == 0); // 2 lanes default anchor == the transition's default anchor
    CHECK(t.xs.end_join.raw_delta_mm == c.xs.start_join.raw_delta_mm);
    CHECK(t.xs.end_join.raw_delta_mm != 0);
    CHECK(t.xs.offset.mode == OffsetMode::Transition);
    CHECK(t.xs.offset.start_mm == 0); CHECK(t.xs.offset.end_mm == t.xs.end_join.raw_delta_mm); // U absorbs +delta
    CHECK(c.xs.offset.mode == OffsetMode::None);
    const auto bt = boundaries(t.xs, t.xs.length_m), bc = boundaries(c.xs, 0.0);
    const auto ft = first_counted(t.xs), fc = first_counted(c.xs);
    CHECK(std::fabs(bt[ft] - bc[fc]) < 1e-9);
    const auto ba = boundaries(a.xs, a.xs.length_m), b0 = boundaries(t.xs, 0.0);
    CHECK(std::fabs(ba[first_counted(a.xs)] - b0[ft]) < 1e-9);
}

TEST_CASE("Shifts and ramps are clipped to short stretches without breaking continuity", "[layer][road_joins][roads]") {
    // A 20 m stretch between two joins that both need a lane change and a shift.
    const auto g = graph_of({{1, {0, 0}}, {2, {300, 0}}, {3, {320, 0}}, {4, {600, 0}}},
        {way(10, {1, 2}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}}),
         way(11, {2, 3}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "3"}, {"turn:lanes", "none|none|right"}}),
         way(12, {3, 4}, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", "2"}})});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto b = resolve_at(*builder, 11, 0);
    const auto l = last_counted(b.xs);
    CHECK((b.xs.flags & (kXsTaperShortenedStart | kXsTaperShortenedEnd)) == (kXsTaperShortenedStart | kXsTaperShortenedEnd));
    const std::int64_t sum = b.xs.bands[l].start.length_mm + b.xs.bands[l].end.length_mm;
    CHECK(sum <= static_cast<std::int64_t>(std::floor(b.xs.length_m * 1000.0)));
    CHECK(b.xs.band_width_m(l, 0.0) == 0.0); CHECK(b.xs.band_width_m(l, b.xs.length_m) == 0.0);
}

namespace {
struct World {
    RoadGraph graph;
    std::optional<RoadReferenceBuilder> builder;
};
World load_world(const char* file) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + file);
    auto graph = build_road_graph(data, g2m_test::kZone32N); REQUIRE(graph);
    World w; w.graph = std::move(*graph);
    auto b = RoadReferenceBuilder::make(w.graph); REQUIRE(b);
    w.builder = std::move(*b);
    return w;
}
AtNode stretch_at(const World& w, std::int64_t way_id, std::int64_t node) {
    return stretch_at_node(w.graph, *w.builder, way_id, node);
}
void dump(const char* label, const AtNode& a) {
    const auto& xs = a.r.xs;
    const auto& j = a.at_end ? xs.end_join : xs.start_join;
    std::printf("  %s way-end=%s kind=%d nb=%lld delta=%d offset=(mode %d, %d/%d, %d/%d) bands:", label, a.at_end ? "end" : "start",
        static_cast<int>(j.kind), static_cast<long long>(j.neighbour_way_id), j.raw_delta_mm, static_cast<int>(xs.offset.mode),
        xs.offset.start_mm, xs.offset.start_length_mm, xs.offset.end_mm, xs.offset.end_length_mm);
    for (const auto& b : xs.bands) {
        const auto& r = a.at_end ? b.end : b.start;
        std::printf(" [k%d w%d%s ramp %d<-%d]", static_cast<int>(b.kind), b.width_mm, b.counted ? "c" : "", r.length_mm, r.from_width_mm);
    }
    std::printf("\n");
}
struct Case { const char* id; std::int64_t a, b, node; };
void dump_cases(const World& w, std::initializer_list<Case> cases) {
    for (const auto& c : cases) {
        std::printf("%s: deg(node %lld) = %u\n", c.id, static_cast<long long>(c.node), w.builder->degree(c.node));
        dump("A", stretch_at(w, c.a, c.node)); dump("B", stretch_at(w, c.b, c.node));
    }
}
}

TEST_CASE("diag: print the design's fixture cases", "[.][diag][layer][road_joins]") {
    const auto w = load_world("frankfurt_hoechst.osm.pbf");
    dump_cases(w, {{"F1", 264519302, 264519307, 504378}, {"F2", 264519313, 264519317, 2146570192}, {"F3", 23596416, 264519314, 2701860424},
        {"F4a", 177564867, 336837972, 504296}, {"F4b", 336837972, 318862793, 3439247819},
        {"F6", 256917288, 4537572, 1596079422}, {"F7", 4846955, 259399524, 2648065658}, {"F8", 37804346, 259399543, 2648065583},
        {"F9", 137332121, 588017497, 31919397}, {"F10", 175431016, 843264244, 32145074}});
}

namespace {
// Nodes that are an endpoint (first or last node) of both ways.
std::vector<std::int64_t> common_ends(const World& w, std::int64_t a, std::int64_t b) {
    const auto find = [&](std::int64_t id) -> const RoadGraphWay& {
        const auto it = std::find_if(w.graph.ways.begin(), w.graph.ways.end(), [&](const auto& x) { return x.osm_id == id; });
        REQUIRE(it != w.graph.ways.end());
        return *it;
    };
    const auto& wa = find(a); const auto& wb = find(b);
    std::vector<std::int64_t> r;
    for (const auto na : {wa.node_ids.front(), wa.node_ids.back()})
        for (const auto nb : {wb.node_ids.front(), wb.node_ids.back()}) if (na == nb) r.push_back(na);
    return r;
}
} // namespace

namespace {
// The counted-lane index (0-based among counted bands) whose ramp at the join end starts from zero width, or -1.
int added_lane(const AtNode& a) {
    int k = 0, found = -1;
    for (const auto& b : a.r.xs.bands) {
        if (!b.counted) continue;
        const auto& r = a.at_end ? b.end : b.start;
        if (r.length_mm > 0 && r.from_width_mm == 0) { REQUIRE(found == -1); found = k; }
        ++k;
    }
    return found;
}
int counted_lanes(const AtNode& a) {
    int k = 0;
    for (const auto& b : a.r.xs.bands) k += b.counted ? 1 : 0;
    return k;
}
bool any_ramp_at_join(const AtNode& a) {
    for (const auto& b : a.r.xs.bands) if ((a.at_end ? b.end : b.start).length_mm > 0) return true;
    return false;
}
struct Pair { AtNode a, b; };
Pair join_at(const World& w, std::int64_t wa, std::int64_t wb, std::int64_t node, JoinKind kind, int abs_delta_mm) {
    REQUIRE(w.builder->degree(node) == 2);
    Pair p{stretch_at(w, wa, node), stretch_at(w, wb, node)};
    const auto& ja = p.a.at_end ? p.a.r.xs.end_join : p.a.r.xs.start_join;
    const auto& jb = p.b.at_end ? p.b.r.xs.end_join : p.b.r.xs.start_join;
    CHECK(ja.kind == kind); CHECK(jb.kind == kind);
    CHECK(ja.neighbour_way_id == wb); CHECK(jb.neighbour_way_id == wa);
    CHECK(std::abs(ja.raw_delta_mm) == abs_delta_mm); CHECK(std::abs(jb.raw_delta_mm) == abs_delta_mm);
    return p;
}
// The way with more counted lanes owns the lane change.
const AtNode& wider(const Pair& p) { return counted_lanes(p.a) >= counted_lanes(p.b) ? p.a : p.b; }
const AtNode& narrower(const Pair& p) { return counted_lanes(p.a) >= counted_lanes(p.b) ? p.b : p.a; }
} // namespace

TEST_CASE("Kalbach fixture: lane adds, drops, pockets and transitions (design F1-F10)", "[layer][road_joins][roads]") {
    const auto w = load_world("frankfurt_hoechst.osm.pbf");
    SECTION("F1 acceleration lane: added right, delta 0") {
        const auto p = join_at(w, 264519302, 264519307, 504378, JoinKind::LaneChange, 0);
        CHECK(counted_lanes(narrower(p)) == 2); CHECK(counted_lanes(wider(p)) == 3);
        CHECK(added_lane(wider(p)) == 2); CHECK(added_lane(narrower(p)) == -1);
        CHECK(p.a.r.xs.offset.mode == OffsetMode::None); CHECK(p.b.r.xs.offset.mode == OffsetMode::None);
    }
    SECTION("F2 lane end: dropped right, delta 0, the dropped lane ramps to zero inside the wider way") {
        const auto p = join_at(w, 264519313, 264519317, 2146570192, JoinKind::LaneChange, 0);
        CHECK(counted_lanes(wider(p)) == 3); CHECK(added_lane(wider(p)) == 2); CHECK(added_lane(narrower(p)) == -1);
        CHECK(wider(p).r.ref.way_id == 264519313);
        CHECK(p.a.r.xs.offset.mode == OffsetMode::None); CHECK(p.b.r.xs.offset.mode == OffsetMode::None);
    }
    SECTION("F3 deceleration lane: added right, delta 0") {
        const auto p = join_at(w, 23596416, 264519314, 2701860424, JoinKind::LaneChange, 0);
        CHECK(counted_lanes(wider(p)) == 3); CHECK(added_lane(wider(p)) == 2); CHECK(added_lane(narrower(p)) == -1);
        CHECK(wider(p).r.ref.way_id == 264519314);
    }
    SECTION("F4 transition way absorbs half a lane at the first join, nothing at the second") {
        const auto p = join_at(w, 177564867, 336837972, 504296, JoinKind::InferredAlignment, 1812);
        const auto& t = p.a.r.ref.way_id == 336837972 ? p.a : p.b;
        const auto& o = p.a.r.ref.way_id == 336837972 ? p.b : p.a;
        CHECK(t.r.xs.anchor.source == PlacementSource::Transition);
        CHECK(t.r.xs.offset.mode == OffsetMode::Transition); CHECK(o.r.xs.offset.mode == OffsetMode::None);
        CHECK(counted_lanes(o) == 3); CHECK(counted_lanes(t) == 2);
        const auto q = join_at(w, 336837972, 318862793, 3439247819, JoinKind::Consistent, 0);
        const auto& t2 = q.a.r.ref.way_id == 336837972 ? q.a : q.b;
        CHECK(t2.r.xs.offset.mode == OffsetMode::Transition);
        // The transition way shifts by 0 at one end and by half a lane at the other.
        const auto& off = t.r.xs.offset;
        CHECK(((off.start_mm == 0 && std::abs(off.end_mm) == 1812) || (off.end_mm == 0 && std::abs(off.start_mm) == 1812)));
    }
    SECTION("F5 link with left_of:1 pins the left edge of the lane to the reference line") {
        const auto r = resolve_at(*w.builder, 50876963, 0);
        CHECK(r.xs.anchor.edge == AnchorEdge::Left); CHECK(r.xs.anchor.source == PlacementSource::Explicit);
        const auto b = boundaries(r.xs, r.xs.length_m / 2);
        CHECK(std::fabs(b[r.xs.anchor.band]) < 1e-3);
    }
    SECTION("F6 urban pocket on the right: the wider way absorbs and holds") {
        const auto p = join_at(w, 256917288, 4537572, 1596079422, JoinKind::InferredAlignment, 1625);
        CHECK(counted_lanes(wider(p)) == 3); CHECK(added_lane(wider(p)) == 2);
        CHECK(wider(p).r.xs.offset.mode == OffsetMode::Hold); CHECK(std::abs(wider(p).r.xs.offset.start_mm) == 1625);
        CHECK(narrower(p).r.xs.offset.mode == OffsetMode::None);
    }
    SECTION("F7 ambiguous left+right tokens: no explicit placement, the lanes are added on the right") {
        const auto p = join_at(w, 4846955, 259399524, 2648065658, JoinKind::InferredAlignment, 1625);
        const auto& x = wider(p);
        CHECK(counted_lanes(x) == 3);
        CHECK(x.r.xs.offset.mode == OffsetMode::Hold);
        int k = 0;
        for (const auto& band : x.r.xs.bands) {
            if (!band.counted) continue;
            const auto& r = x.at_end ? band.end : band.start;
            CHECK((k == 0) == (r.length_mm == 0));
            ++k;
        }
    }
    SECTION("F8 asymmetric two-way: forward group grows on the right, the backward lane is shared") {
        const auto p = join_at(w, 37804346, 259399543, 2648065583, JoinKind::InferredAlignment, 1500);
        CHECK(counted_lanes(narrower(p)) == 3); CHECK(counted_lanes(wider(p)) == 4);
        CHECK(added_lane(wider(p)) == 3); CHECK(wider(p).r.xs.offset.mode == OffsetMode::Hold);
    }
    SECTION("F9 untagged against tagged: both resolve to 1+1, consistent") {
        join_at(w, 137332121, 588017497, 31919397, JoinKind::Consistent, 0);
    }
    SECTION("F10 class change: one Both lane against 1+1, the narrower side ramps its widths") {
        const auto p = join_at(w, 175431016, 843264244, 32145074, JoinKind::Consistent, 0);
        CHECK(any_ramp_at_join(p.a)); CHECK(any_ramp_at_join(p.b));
        CHECK(std::min(counted_lanes(p.a), counted_lanes(p.b)) == 1); CHECK(std::max(counted_lanes(p.a), counted_lanes(p.b)) == 2);
    }
}

TEST_CASE("Home tile fixture: A 66 and B 8 joins (design H1-H7)", "[layer][road_joins][roads]") {
    const auto w = load_world("hoechst_a66.osm.pbf");
    SECTION("H1 the A 66 widening meets at a junction node (deviation from the design's scan: degree 3, no join)") {
        const auto n = common_ends(w, 23658591, 24504854);
        REQUIRE(n.size() == 1);
        CHECK(w.builder->degree(n[0]) == 3);
        const auto a = stretch_at(w, 23658591, n[0]), b = stretch_at(w, 24504854, n[0]);
        CHECK((a.at_end ? a.r.xs.end_join : a.r.xs.start_join).kind == JoinKind::None);
        CHECK((b.at_end ? b.r.xs.end_join : b.r.xs.start_join).kind == JoinKind::None);
        CHECK(counted_lanes(a) == 4); CHECK(counted_lanes(b) == 5);
        CHECK(!any_ramp_at_join(a)); CHECK(!any_ramp_at_join(b));
    }
    SECTION("H2 bridge joins are consistent without ramps") {
        const std::pair<std::int64_t, std::int64_t> pairs[] = {{1045544333, 1045544334}, {1045544334, 148711394}};
        for (const auto& [x, y] : pairs) {
            const auto n = common_ends(w, x, y); REQUIRE(n.size() == 1);
            const auto p = join_at(w, x, y, n[0], JoinKind::Consistent, 0);
            CHECK(!any_ramp_at_join(p.a)); CHECK(!any_ramp_at_join(p.b));
            CHECK(p.a.r.xs.offset.mode == OffsetMode::None); CHECK(p.b.r.xs.offset.mode == OffsetMode::None);
        }
    }
    SECTION("H3 drop right, then add right, both with delta 0") {
        const auto n1 = common_ends(w, 155798152, 155798154), n2 = common_ends(w, 155798154, 313879336);
        REQUIRE(n1.size() == 1); REQUIRE(n2.size() == 1);
        const auto p = join_at(w, 155798152, 155798154, n1[0], JoinKind::LaneChange, 0);
        CHECK(counted_lanes(wider(p)) == 3); CHECK(added_lane(wider(p)) == 2); CHECK(added_lane(narrower(p)) == -1);
        const auto q = join_at(w, 155798154, 313879336, n2[0], JoinKind::LaneChange, 0);
        CHECK(counted_lanes(wider(q)) == 3); CHECK(added_lane(wider(q)) == 2); CHECK(added_lane(narrower(q)) == -1);
    }
    SECTION("H4 middle_of:2 four-lane drop equals the three-lane default centre") {
        const auto n = common_ends(w, 45674354, 313879334); REQUIRE(n.size() == 1);
        const auto p = join_at(w, 45674354, 313879334, n[0], JoinKind::LaneChange, 0);
        CHECK(counted_lanes(wider(p)) == 4); CHECK(added_lane(wider(p)) == 3);
    }
    SECTION("H5 left pocket on a one-way road: added left, delta 0") {
        const auto n = common_ends(w, 104183076, 324588712); REQUIRE(n.size() == 1);
        const auto p = join_at(w, 104183076, 324588712, n[0], JoinKind::LaneChange, 0);
        CHECK(counted_lanes(wider(p)) == 2); CHECK(added_lane(wider(p)) == 0);
    }
    SECTION("H6 left pocket with right_of: added left, delta 0") {
        const auto n = common_ends(w, 333909891, 480160099); REQUIRE(n.size() == 1);
        const auto p = join_at(w, 333909891, 480160099, n[0], JoinKind::LaneChange, 0);
        CHECK(counted_lanes(wider(p)) == 3); CHECK(added_lane(wider(p)) == 0);
    }
    SECTION("H7 link with ambiguous tokens: default right, inferred alignment") {
        const auto n = common_ends(w, 8602212, 155798159); REQUIRE(n.size() == 1);
        const auto p = join_at(w, 8602212, 155798159, n[0], JoinKind::InferredAlignment, 1750);
        CHECK(counted_lanes(wider(p)) == 3); CHECK(added_lane(wider(p)) == 2);
    }
}

namespace {
struct JoinEnd { std::int64_t way = 0; std::size_t stretch = 0; bool at_end = false; };
// Boundary points (plane) of a resolved stretch end, left to right.
std::vector<LateralPoint> end_points(const Resolved& r, bool at_end) {
    const double s = at_end ? r.xs.length_m : 0.0;
    const auto b = boundaries(r.xs, s);
    std::vector<LateralPoint> out;
    for (const double t : b) { const auto p = lateral_point(r.ref, s, t); REQUIRE(p); out.push_back(*p); }
    return out;
}
std::vector<LateralPoint> distinct(const std::vector<LateralPoint>& in) {
    std::vector<LateralPoint> out;
    for (const auto& p : in) {
        bool dup = false;
        for (const auto& q : out) if (std::hypot(p.x - q.x, p.y - q.y) < 1e-3) dup = true;
        if (!dup) out.push_back(p);
    }
    return out;
}
// The direction (radians) along the way pointing from the join into the way's own body.
double body_heading(const Resolved& r, bool at_end) {
    const auto pose = r.ref.at(at_end ? r.ref.length_m : 0.0); REQUIRE(pose);
    return pose->heading + (at_end ? 3.14159265358979323846 : 0.0);
}
// The rounded-corner fallback of the reference fit (a clothoid beyond the class's minimum radius) ends along the chord and
// does not take the shared-bisector heading; at such a join the two references meet at a point but not tangentially, so
// lateral points differ by t * delta-heading by construction (a slice-1 property, not a cross-section one). Real
// corners (up to 60 degrees) at a way join are the usual cause.
bool tangent_break(const Resolved& a, bool a_end, const Resolved& b, bool b_end) {
    double d = body_heading(a, a_end) - body_heading(b, b_end) - 3.14159265358979323846;
    d = std::remainder(d, 2 * 3.14159265358979323846);
    return std::fabs(d) > 5e-4; // at most ~9 m of lateral extent: 4.5 mm
}
// Checks every degree-2 join of a fixture; returns {joins checked, joins skipped (rounded fallback end)}.
std::pair<int, int> check_edge_continuity(const World& w, double& worst_m) {
    int checked = 0, skipped = 0;
    for (const auto& node : w.graph.nodes) {
        if (w.builder->degree(node.osm_id) != 2) continue;
        std::vector<JoinEnd> ends;
        for (const auto& way : w.graph.ways) {
            const auto stretches = w.builder->stretches(way.osm_id);
            if (!stretches) continue;
            for (std::size_t i = 0; i < stretches->size(); ++i) {
                const auto& st = (*stretches)[i];
                if (way.node_ids[st.start_ref] == node.osm_id) ends.push_back({way.osm_id, i, false});
                if (way.node_ids[st.end_ref] == node.osm_id) ends.push_back({way.osm_id, i, true});
            }
        }
        if (ends.size() != 2) continue;
        const auto ra = resolve_at(*w.builder, ends[0].way, ends[0].stretch);
        const auto rb = resolve_at(*w.builder, ends[1].way, ends[1].stretch);
        if (tangent_break(ra, ends[0].at_end, rb, ends[1].at_end)) { ++skipped; continue; }
        const auto pa = distinct(end_points(ra, ends[0].at_end)), pb = distinct(end_points(rb, ends[1].at_end));
        const auto& small = pa.size() <= pb.size() ? pa : pb;
        const auto& large = pa.size() <= pb.size() ? pb : pa;
        INFO("node " << node.osm_id << " ways " << ends[0].way << " / " << ends[1].way);
        for (const auto& p : small) {
            double best = 1e9;
            for (const auto& q : large) best = std::min(best, std::hypot(p.x - q.x, p.y - q.y));
            worst_m = std::max(worst_m, best);
            CHECK(best < 0.01);
        }
        ++checked;
    }
    return {checked, skipped};
}
} // namespace

TEST_CASE("Edge continuity: every degree-2 join of both fixtures meets within 1 cm", "[layer][road_joins][roads]") {
    for (const char* file : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto w = load_world(file);
        double worst = 0;
        const auto [checked, skipped] = check_edge_continuity(w, worst);
        std::printf("edge continuity %s: %d joins checked, %d skipped (rounded fallback, no shared tangent), worst %.6f m\n", file, checked, skipped, worst);
        CHECK(checked > 20);
    }
}

namespace {
// The graph that holds only the given ways (their nodes and edges): what a consumer sees when the rest of the world is
// not loaded. Far ends turn into dead ends, the join node keeps degree two.
RoadGraph subgraph_of(const RoadGraph& g, const std::vector<std::int64_t>& way_ids) {
    RoadGraph out;
    std::vector<std::int64_t> node_ids;
    for (const auto& way : g.ways) {
        if (std::find(way_ids.begin(), way_ids.end(), way.osm_id) == way_ids.end()) continue;
        out.ways.push_back(way);
        for (const auto n : way.node_ids) node_ids.push_back(n);
    }
    std::sort(node_ids.begin(), node_ids.end());
    node_ids.erase(std::unique(node_ids.begin(), node_ids.end()), node_ids.end());
    for (const auto& node : g.nodes) if (std::binary_search(node_ids.begin(), node_ids.end(), node.osm_id)) out.nodes.push_back(node);
    for (const auto& edge : g.edges) if (std::find(way_ids.begin(), way_ids.end(), edge.way_id) != way_ids.end()) out.edges.push_back(edge);
    return out;
}
bool same_at_join(const AtNode& full, const AtNode& part) {
    const auto& jf = full.at_end ? full.r.xs.end_join : full.r.xs.start_join;
    const auto& jp = part.at_end ? part.r.xs.end_join : part.r.xs.start_join;
    if (!(jf == jp) || full.at_end != part.at_end || full.r.xs.bands.size() != part.r.xs.bands.size()) return false;
    for (std::size_t i = 0; i < full.r.xs.bands.size(); ++i) {
        const auto& bf = full.r.xs.bands[i]; const auto& bp = part.r.xs.bands[i];
        if (!((full.at_end ? bf.end : bf.start) == (part.at_end ? bp.end : bp.start))) return false;
    }
    const double s = full.at_end ? full.r.xs.length_m : 0.0;
    return boundaries(full.r.xs, s) == boundaries(part.r.xs, part.at_end ? part.r.xs.length_m : 0.0);
}
} // namespace

TEST_CASE("A join resolves identically whether or not the rest of the world is loaded", "[layer][road_joins][roads]") {
    for (const char* file : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto w = load_world(file);
        int compared = 0, clipped = 0;
        for (const auto& node : w.graph.nodes) {
            if (w.builder->degree(node.osm_id) != 2) continue;
            std::vector<std::int64_t> ids;
            for (const auto& way : w.graph.ways)
                if (way.node_ids.front() == node.osm_id || way.node_ids.back() == node.osm_id) ids.push_back(way.osm_id);
            if (ids.size() != 2) continue;
            const auto sub = subgraph_of(w.graph, ids);
            auto b = RoadReferenceBuilder::make(sub); REQUIRE(b);
            INFO(file << " node " << node.osm_id << " ways " << ids[0] << " / " << ids[1]);
            for (const auto id : ids) {
                const auto full = stretch_at(w, id, node.osm_id);
                const auto part = stretch_at_node(sub, *b, id, node.osm_id);
                // A stretch whose ramps were clipped to its length shares that length with its other end's join,
                // which the subgraph does not hold; only unclipped stretches are context-free.
                if ((full.r.xs.flags & (kXsTaperShortenedStart | kXsTaperShortenedEnd)) != 0) { ++clipped; continue; }
                const bool same = same_at_join(full, part);
                CHECK(same);
                if (!same) { dump("full", full); dump("part", part); std::printf("  lengths %.3f %.3f\n", full.r.xs.length_m, part.r.xs.length_m); }
            }
            if (++compared >= 150) break;
        }
        std::printf("context invariance %s: %d joins, %d stretch ends skipped (clipped ramps)\n", file, compared, clipped);
        CHECK(compared > 20);
    }
}

TEST_CASE("diag: print the home-tile fixture cases", "[.][diag][layer][road_joins]") {
    const auto w = load_world("hoechst_a66.osm.pbf");
    const std::pair<const char*, std::pair<std::int64_t, std::int64_t>> pairs[] = {
        {"H1", {23658591, 24504854}}, {"H2a", {1045544333, 1045544334}}, {"H2b", {1045544334, 148711394}},
        {"H3a", {155798152, 155798154}}, {"H3b", {155798154, 313879336}}, {"H4", {45674354, 313879334}},
        {"H5", {104183076, 324588712}}, {"H6", {333909891, 480160099}}, {"H7", {8602212, 155798159}}};
    for (const auto& [id, p] : pairs) {
        const auto nodes = common_ends(w, p.first, p.second);
        std::printf("%s: %zu common end nodes\n", id, nodes.size());
        for (const auto n : nodes) dump_cases(w, {{id, p.first, p.second, n}});
    }
}

TEST_CASE("diag: one join in detail", "[.][diag][layer][road_joins]") {
    const auto w = load_world("frankfurt_hoechst.osm.pbf");
    for (const auto& [wa, wb, node] : {std::tuple<std::int64_t, std::int64_t, std::int64_t>{23596398, 23596399, 504286}, {23740206, 23740207, 504287}}) {
        const auto a = stretch_at(w, wa, node), b = stretch_at(w, wb, node);
        dump("A", a); dump("B", b);
        for (const auto* x : {&a, &b}) {
            const double s = x->at_end ? x->r.xs.length_m : 0.0;
            const auto pose = x->r.ref.at(s); REQUIRE(pose);
            std::printf("  heading=%.6f way %lld at_end=%d len=%.2f pose=(%.3f, %.3f) piece=%u nseg=%zu flags=%u\n", pose->heading, static_cast<long long>(x->r.ref.way_id),
                x->at_end ? 1 : 0, x->r.xs.length_m, pose->x, pose->y, (x->at_end ? x->r.ref.segments.back() : x->r.ref.segments.front()).piece_index,
                x->r.ref.segments.size(), static_cast<unsigned>(x->r.xs.flags));
            const auto pts = end_points(x->r, x->at_end);
            for (const auto& p : pts) std::printf("    (%.3f, %.3f)\n", p.x, p.y);
        }
    }
}
