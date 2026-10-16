// G4 slice 4: the guidance manoeuvre list (route_guidance.h). Synthetic
// scenes pin each manoeuvre kind; a scripted route on the hoechst_a66
// fixture checks the invariants and pins the kind sequence.
#include "route_fixture_io.h"
#include "route_reference.h"
#include "route_scenes.h"
#include "splitmix64.h"
#include "synthetic.h"

#include "g2m/route/route_guidance.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <string>
#include <vector>

using namespace g2m;
using namespace g2m::route;
using g2m_test::GraphBuilder;
using Tags = std::vector<std::pair<std::string, std::string>>;

namespace {

struct Guided {
    Route route;
    std::vector<Maneuver> steps;
};

void check_invariants(const Route& route, const std::vector<Maneuver>& steps) {
    REQUIRE(steps.size() >= 2);
    CHECK(steps.front().kind == ManeuverKind::Depart);
    CHECK(steps.front().distance_from_start_mm == 0);
    CHECK(steps.back().kind == ManeuverKind::Arrive);
    CHECK(steps.back().distance_from_start_mm == route.length_mm);
    CHECK(steps.back().distance_to_next_mm == 0);
    std::uint64_t sum = 0;
    for (std::size_t k = 0; k + 1 < steps.size(); ++k) {
        CHECK(steps[k].distance_from_start_mm <= steps[k + 1].distance_from_start_mm);
        CHECK(steps[k].step <= steps[k + 1].step);
        sum += steps[k].distance_to_next_mm;
    }
    CHECK(sum == route.length_mm);
}

Guided guide_nodes(const RoadGraph& roads, std::int64_t from, std::int64_t to) {
    auto g = build_routing_graph(roads, {});
    REQUIRE(g);
    RouteSearcher searcher(*g);
    auto r = searcher.find(RouteQuery{from, to});
    REQUIRE(r);
    REQUIRE(r->reachable);
    auto steps = build_guidance(*g, roads, *r);
    REQUIRE(steps);
    check_invariants(*r, *steps);
    return {*r, *steps};
}

std::vector<ManeuverKind> kinds(const std::vector<Maneuver>& steps) {
    std::vector<ManeuverKind> out;
    for (const auto& m : steps) out.push_back(m.kind);
    return out;
}

using K = ManeuverKind;

} // namespace

TEST_CASE("Guidance: depart, turn, arrive with distances and names", "[route][guidance]") {
    GraphBuilder b;
    b.node(1, 0, 0); b.node(2, 0, -100); b.node(3, 0, 100); b.node(4, -100, 0); b.node(5, 100, 0);
    b.way(10, {2, 1}, Tags{{"name", "Hauptstrasse"}});
    b.way(30, {1, 3}, Tags{{"name", "Hauptstrasse"}});
    b.way(20, {1, 4}, Tags{{"name", "Westweg"}});
    b.way(40, {1, 5}, Tags{{"name", "Ostweg"}, {"ref", "K 1"}});
    const auto roads = b.build();

    const auto right = guide_nodes(roads, 2, 5);
    REQUIRE(kinds(right.steps) == std::vector<K>{K::Depart, K::Right, K::Arrive});
    CHECK(right.steps[0].name == "Hauptstrasse");
    CHECK(right.steps[1].distance_from_start_mm == 100000);
    CHECK(right.steps[1].distance_to_next_mm == 100000);
    CHECK(right.steps[1].angle_deg == -90);
    CHECK(right.steps[1].name == "Ostweg");
    CHECK(right.steps[1].ref == "K 1");
    CHECK(right.steps[1].position == geom::PointMm{0, 0});
    CHECK(right.steps[2].distance_from_start_mm == 200000);
    CHECK(right.steps[2].position == geom::PointMm{100000, 0});

    CHECK(kinds(guide_nodes(roads, 2, 4).steps) == std::vector<K>{K::Depart, K::Left, K::Arrive});
    // Straight on along the same name: nothing to say at the crossing.
    CHECK(kinds(guide_nodes(roads, 2, 3).steps) == std::vector<K>{K::Depart, K::Arrive});

    // Straight on onto a new name: Continue.
    GraphBuilder c = b;
    c.graph.ways[1].tags = {{"highway", "residential"}, {"name", "Bahnhofstrasse"}};
    const auto renamed = guide_nodes(c.build(), 2, 3);
    REQUIRE(kinds(renamed.steps) == std::vector<K>{K::Depart, K::Continue, K::Arrive});
    CHECK(renamed.steps[1].name == "Bahnhofstrasse");

    // Errors.
    auto g = build_routing_graph(roads, {});
    REQUIRE(g);
    CHECK_FALSE(build_guidance(*g, roads, Route{}));
    RouteSearcher searcher(*g);
    auto r = searcher.find(RouteQuery{2, 5});
    REQUIRE(r);
    CHECK_FALSE(build_guidance(*g, RoadGraph{}, *r));
}

TEST_CASE("Guidance: a fork gives keep left / keep right, sharp and slight turns", "[route][guidance]") {
    GraphBuilder b; // from the south, two branches 16.7 deg either side of north
    b.node(1, 0, 0); b.node(2, 0, -100); b.node(3, -30, 100); b.node(4, 30, 100);
    b.way(10, {2, 1}); b.way(20, {1, 3}); b.way(30, {1, 4});
    const auto fork = b.build();
    CHECK(kinds(guide_nodes(fork, 2, 3).steps) == std::vector<K>{K::Depart, K::KeepLeft, K::Arrive});
    CHECK(kinds(guide_nodes(fork, 2, 4).steps) == std::vector<K>{K::Depart, K::KeepRight, K::Arrive});

    GraphBuilder s; // a slight right next to a sharp left: no fork, a slight turn
    s.node(1, 0, 0); s.node(2, 0, -100); s.node(3, 60, 100); s.node(4, -100, -60);
    s.way(10, {2, 1}); s.way(20, {1, 3}); s.way(30, {1, 4});
    const auto split = s.build();
    const auto slight = guide_nodes(split, 2, 3);
    REQUIRE(kinds(slight.steps) == std::vector<K>{K::Depart, K::SlightRight, K::Arrive});
    CHECK(slight.steps[1].angle_deg == -31);
    const auto sharp = guide_nodes(split, 2, 4);
    REQUIRE(kinds(sharp.steps) == std::vector<K>{K::Depart, K::SharpLeft, K::Arrive});
    CHECK(sharp.steps[1].angle_deg == 121);
}

TEST_CASE("Guidance: roundabout exit numbers", "[route][guidance][roundabout]") {
    // Ring counter-clockwise (right-hand traffic) S -> E -> N -> W -> S, one arm per ring node.
    GraphBuilder b;
    b.node(1, 0, -20); b.node(2, 20, 0); b.node(3, 0, 20); b.node(4, -20, 0);
    b.node(11, 0, -100); b.node(12, 100, 0); b.node(13, 0, 100); b.node(14, -100, 0);
    const Tags ring{{"junction", "roundabout"}, {"name", "Kreisel"}};
    b.way(1, {1, 2}, ring); b.way(2, {2, 3}, ring); b.way(3, {3, 4}, ring); b.way(4, {4, 1}, ring);
    b.way(11, {11, 1}, Tags{{"name", "Sued"}}); b.way(12, {2, 12}, Tags{{"name", "Ost"}});
    b.way(13, {3, 13}, Tags{{"name", "Nord"}}); b.way(14, {4, 14}, Tags{{"name", "West"}});
    const auto roads = b.build();
    for (const auto& [to, exit] : std::vector<std::pair<std::int64_t, int>>{{12, 1}, {13, 2}, {14, 3}}) {
        const auto r = guide_nodes(roads, 11, to);
        REQUIRE(kinds(r.steps) == std::vector<K>{K::Depart, K::RoundaboutEnter, K::Arrive});
        CHECK(r.steps[1].exit_number == exit);
        CHECK(r.steps[1].distance_from_start_mm == 80000);
        CHECK(r.steps[1].name == "Kreisel");
    }
    // Starting on the ring: the exit is announced as RoundaboutExit.
    const auto on_ring = guide_nodes(roads, 2, 14);
    REQUIRE(kinds(on_ring.steps) == std::vector<K>{K::Depart, K::RoundaboutExit, K::Arrive});
    CHECK(on_ring.steps[1].exit_number == 2);
    CHECK(on_ring.steps[1].name == "West");
}

TEST_CASE("Guidance: ramps and merges", "[route][guidance]") {
    GraphBuilder b; // motorway west -> east, an exit ramp off it and an entry ramp onto it
    b.node(1, 0, 0); b.node(2, 1000, 0); b.node(3, 2000, 0); b.node(4, 3000, 0);
    b.node(5, 1200, -60); b.node(6, 1300, -200); b.node(7, 1300, -400); b.node(8, 1300, 400);
    b.node(9, 1700, -60);
    const Tags mw{{"highway", "motorway"}, {"ref", "A 66"}};
    b.way(1, {1, 2}, mw); b.way(2, {2, 3}, mw); b.way(3, {3, 4}, mw);
    b.way(10, {2, 5, 6}, Tags{{"highway", "motorway_link"}, {"oneway", "yes"}, {"destination", "Hoechst"}, {"destination:ref", "B 40"}});
    b.way(11, {6, 9, 3}, Tags{{"highway", "motorway_link"}, {"oneway", "yes"}});
    b.way(20, {7, 6, 8}, Tags{{"highway", "primary"}, {"name", "Kasinostrasse"}});
    const auto roads = b.build();
    const auto off = guide_nodes(roads, 1, 7);
    REQUIRE(kinds(off.steps) == std::vector<K>{K::Depart, K::RampOff, K::SlightRight, K::Arrive});
    CHECK(off.steps[1].distance_from_start_mm == 1000000);
    CHECK(off.steps[1].destination == "Hoechst");
    CHECK(off.steps[1].destination_ref == "B 40");
    const auto on = guide_nodes(roads, 7, 4);
    REQUIRE(kinds(on.steps) == std::vector<K>{K::Depart, K::RampOn, K::Merge, K::Arrive});
    CHECK(on.steps[2].ref == "A 66");
    // Staying on the motorway past the exit ramp: nothing to announce.
    CHECK(kinds(guide_nodes(roads, 1, 4).steps) == std::vector<K>{K::Depart, K::Arrive});
}

TEST_CASE("Guidance: U-turn at a dead end on a point route", "[route][guidance][snap]") {
    GraphBuilder b;
    b.node(1, 0, 0); b.node(2, 100, 0); b.node(3, 200, 0);
    b.way(10, {1, 2, 3}, Tags{{"name", "Sackgasse"}});
    const auto roads = b.build();
    auto g = build_routing_graph(roads, {});
    REQUIRE(g);
    SnapOptions east;
    east.heading_dx = 1.0;
    auto from = g->snap({150000, 5000}, east);
    SnapOptions west;
    west.heading_dx = -1.0;
    auto to = g->snap({120000, -5000}, west);
    REQUIRE(from);
    REQUIRE(to);
    RouteSearcher searcher(*g);
    auto r = searcher.find(*from, *to);
    REQUIRE(r);
    REQUIRE(r->reachable);
    CHECK(r->length_mm == 50000 + 80000);
    auto steps = build_guidance(*g, roads, *r);
    REQUIRE(steps);
    check_invariants(*r, *steps);
    REQUIRE(kinds(*steps) == std::vector<K>{K::Depart, K::UTurn, K::Arrive});
    CHECK((*steps)[1].distance_from_start_mm == 50000);
    CHECK((*steps)[1].position == geom::PointMm{200000, 0});
    CHECK((*steps)[0].position == geom::PointMm{150000, 0});
    CHECK((*steps)[2].position == geom::PointMm{120000, 0});
}

namespace {
// 4-way crossing at node 1 (west 4, east 5, north 3, south 2). `approach` tags the way from the south.
RoadGraph lane_scene(const Tags& approach, bool reversed_way = false) {
    GraphBuilder b;
    b.node(1, 0, 0); b.node(2, 0, -100); b.node(3, 0, 100); b.node(4, -100, 0); b.node(5, 100, 0);
    b.way(10, reversed_way ? std::vector<std::int64_t>{1, 2} : std::vector<std::int64_t>{2, 1}, approach);
    b.way(30, {1, 3}, Tags{{"name", "Nordweg"}});
    b.way(20, {1, 4}, Tags{{"name", "Westweg"}});
    b.way(40, {1, 5}, Tags{{"name", "Ostweg"}});
    return b.build();
}
std::vector<LaneHint> lanes_of(const RoadGraph& roads, std::int64_t to, K expect) {
    const auto r = guide_nodes(roads, 2, to);
    REQUIRE(r.steps.size() == 3);
    CHECK(r.steps[1].kind == expect);
    return r.steps[1].lanes;
}
std::vector<bool> valid_of(const std::vector<LaneHint>& l) {
    std::vector<bool> v;
    for (const auto& h : l) v.push_back(h.valid);
    return v;
}
} // namespace

TEST_CASE("Lane hints: parse and print turn:lanes", "[route][guidance][lanes]") {
    std::vector<std::uint16_t> lanes;
    REQUIRE(parse_turn_lanes("left|through|through;right", lanes));
    REQUIRE(lanes.size() == 3);
    CHECK(lanes[0] == lane_dir::kLeft);
    CHECK(lanes[2] == (lane_dir::kThrough | lane_dir::kRight));
    CHECK(lane_directions_to_string(lanes[2]) == "through;right");
    REQUIRE(parse_turn_lanes("none||slight_left;merge_to_right", lanes));
    CHECK(lanes.size() == 3);
    CHECK(lanes[0] == 0);
    CHECK(lanes[1] == 0);
    CHECK(lane_directions_to_string(0) == "none");
    CHECK_FALSE(parse_turn_lanes("left|banana", lanes));
    CHECK(lanes.empty());
    CHECK_FALSE(parse_turn_lanes("", lanes));
}

TEST_CASE("Lane hints: 3-lane approach left|through|through;right", "[route][guidance][lanes]") {
    const auto roads = lane_scene({{"oneway", "yes"}, {"lanes", "3"}, {"turn:lanes", "left|through|through;right"}});
    const auto right = lanes_of(roads, 5, K::Right);
    REQUIRE(right.size() == 3);
    CHECK(valid_of(right) == std::vector<bool>{false, false, true});
    CHECK(right[2].directions == (lane_dir::kThrough | lane_dir::kRight));
    CHECK(valid_of(lanes_of(roads, 4, K::Left)) == std::vector<bool>{true, false, false});
    // Straight on onto another name: both through lanes.
    CHECK(valid_of(lanes_of(roads, 3, K::Continue)) == std::vector<bool>{false, true, true});
    // Depart / Arrive carry nothing.
    const auto r = guide_nodes(roads, 2, 5);
    CHECK(r.steps.front().lanes.empty());
    CHECK(r.steps.back().lanes.empty());
}

TEST_CASE("Lane hints: fallbacks and untagged lanes", "[route][guidance][lanes]") {
    // No lane says right: a right turn falls back to nothing valid -> no hint, never a guess.
    CHECK(lanes_of(lane_scene({{"oneway", "yes"}, {"turn:lanes", "left|through|through"}}), 5, K::Right).empty());
    // Untagged (none / empty) lanes are through on the outer lanes only.
    const auto roads = lane_scene({{"oneway", "yes"}, {"turn:lanes", "left|none|"}});
    CHECK(valid_of(lanes_of(roads, 3, K::Continue)) == std::vector<bool>{false, false, true});
    const auto mid = lane_scene({{"oneway", "yes"}, {"turn:lanes", "left|none|none|right"}});
    CHECK(lanes_of(mid, 3, K::Continue).empty()); // inner untagged lanes are not through: nothing can take it
    // A slight right without a slight_right lane: the right lane takes it, through only on the right edge.
    GraphBuilder s;
    s.node(1, 0, 0); s.node(2, 0, -100); s.node(3, 60, 100);
    s.way(10, {2, 1}, Tags{{"oneway", "yes"}, {"turn:lanes", "left|through|right"}});
    s.way(20, {1, 3}, Tags{{"name", "Rueckweg"}});
    s.node(4, -100, 0); s.way(30, {1, 4});
    const auto sharp = guide_nodes(s.build(), 2, 3);
    REQUIRE(sharp.steps.size() == 3);
    CHECK(sharp.steps[1].kind == K::SlightRight);
    // slight right, no slight_right lane: through falls back on the right edge lane only.
    CHECK(valid_of(sharp.steps[1].lanes) == std::vector<bool>{false, false, true});
}

TEST_CASE("Lane hints: forward/backward split and direction rules", "[route][guidance][lanes]") {
    const Tags split{{"lanes", "5"}, {"lanes:forward", "2"}, {"lanes:backward", "3"}, {"turn:lanes:forward", "left|through"},
                     {"turn:lanes:backward", "left|through|right"}};
    // Way {2,1}: travelling 2 -> 1 is forward.
    const auto fwd = lane_scene(split);
    CHECK(valid_of(lanes_of(fwd, 4, K::Left)) == std::vector<bool>{true, false});
    CHECK(lanes_of(fwd, 5, K::Right).empty()); // forward has no right lane
    // Way {1,2}: travelling 2 -> 1 is backward.
    const auto bwd = lane_scene(split, true);
    const auto br = lanes_of(bwd, 5, K::Right);
    REQUIRE(br.size() == 3);
    CHECK(valid_of(br) == std::vector<bool>{false, false, true});
    // Plain turn:lanes on a two-way road is ambiguous: no hint. With oneway=-1 it applies against the ref order.
    CHECK(lanes_of(lane_scene({{"turn:lanes", "left|through|right"}}), 5, K::Right).empty());
    CHECK(valid_of(lanes_of(lane_scene({{"oneway", "-1"}, {"turn:lanes", "left|through|right"}}, true), 5, K::Right)) ==
          std::vector<bool>{false, false, true});
    // lanes disagreeing with turn:lanes: no hint.
    CHECK(lanes_of(lane_scene({{"oneway", "yes"}, {"lanes", "4"}, {"turn:lanes", "left|through|right"}}), 5, K::Right).empty());
}

TEST_CASE("Lane hints: no tag, no hint", "[route][guidance][lanes]") {
    const auto roads = lane_scene({{"oneway", "yes"}, {"lanes", "3"}});
    CHECK(lanes_of(roads, 5, K::Right).empty());
    CHECK(lanes_of(roads, 4, K::Left).empty());
}

TEST_CASE("Guidance on the hoechst_a66 fixture: invariants and a pinned route", "[route][guidance][real-data]") {
    const auto data = g2m_test::read_route_fixture("hoechst_a66");
    const auto roads = build_road_graph(data, g2m_test::kZone32N);
    REQUIRE(roads);
    const auto restrictions = extract_turn_restrictions(data);
    REQUIRE(restrictions);
    const auto g = build_routing_graph(*roads, restrictions->restrictions);
    REQUIRE(g);
    RouteSearcher searcher(*g);
    g2m_test::SplitMix64 rng(0x6d1);
    std::size_t routes = 0, total_steps = 0, hinted = 0, unhinted_junction = 0;
    std::vector<Maneuver> longest;
    Route longest_route;
    for (int q = 0; q < 200; ++q) {
        const auto& a = g->nodes()[rng.next() % g->nodes().size()];
        const auto& z = g->nodes()[rng.next() % g->nodes().size()];
        auto r = searcher.find(RouteQuery{a.osm_id, z.osm_id});
        REQUIRE(r);
        if (!r->reachable || r->arcs.empty()) continue;
        auto steps = build_guidance(*g, *roads, *r);
        REQUIRE(steps);
        check_invariants(*r, *steps);
        ++routes;
        total_steps += steps->size();
        for (const auto& m : *steps) {
            if (m.lanes.empty()) { ++unhinted_junction; continue; }
            ++hinted;
            bool any_valid = false;
            for (const auto& l : m.lanes) any_valid = any_valid || l.valid;
            CHECK(any_valid);
            CHECK(m.kind != ManeuverKind::Depart);
            CHECK(m.kind != ManeuverKind::Arrive);
        }
        if (steps->size() > longest.size()) {
            longest = *steps;
            longest_route = *r;
        }
    }
    std::printf("guidance on hoechst_a66: %zu routes, %zu manoeuvres; longest %zu manoeuvres over %.0f m:\n", routes,
                total_steps, longest.size(), static_cast<double>(longest_route.length_mm) / 1000.0);
    std::printf("lane hints: %zu manoeuvres with a hint, %zu without\n", hinted, unhinted_junction);
    std::string seq;
    for (const auto& m : longest) {
        std::printf("  %8.0f m  %-16s %4d deg exit %u  %s %s\n", static_cast<double>(m.distance_from_start_mm) / 1000.0,
                    to_string(m.kind), m.angle_deg, m.exit_number, m.name.c_str(), m.ref.c_str());
        for (const auto& l : m.lanes)
            std::printf("             lane %-24s %s\n", lane_directions_to_string(l.directions).c_str(), l.valid ? "valid" : "-");
        seq += std::string(to_string(m.kind)) + " ";
    }
    CHECK(routes > 100);
    CHECK(longest.size() >= 10);
    CHECK(seq == "depart right slight_right slight_right left continue left continue slight_left right left continue keep_left "
                 "ramp_off keep_left continue slight_right left keep_right left arrive ");
}
