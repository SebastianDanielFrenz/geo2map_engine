// Routing graph + A* on synthetic graphs (G4 slice 1): costs, oneway,
// restrictions, U-turns, unreachable targets, ties, and A* == Dijkstra on
// random graphs. Coordinates in metres (GraphBuilder scales to mm).
#include "route_reference.h"
#include "route_scenes.h"
#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdio>

using namespace g2m;
using namespace g2m::route;
using g2m_test::GraphBuilder;
using g2m_test::via_node_rule;
using g2m_test::cross;
using g2m_test::random_scene;

namespace {

std::vector<std::int64_t> ways_of(const RoutingGraph& g, const Route& r) {
    std::vector<std::int64_t> out;
    for (const auto a : r.arcs) out.push_back(g.arcs()[a].way_id);
    return out;
}

Route solve(const RoutingGraph& g, std::int64_t from, std::int64_t to) {
    RouteSearcher searcher(g);
    auto r = searcher.find({from, to});
    REQUIRE(r);
    CHECK(g2m_test::validate_route(g, *r, from, to).empty());
    const Route ref = g2m_test::dijkstra_reference(g, from, to);
    CHECK(ref.reachable == r->reachable);
    CHECK(ref.cost_ms == r->cost_ms);
    return *r;
}

} // namespace

TEST_CASE("Routing graph collapses shape nodes and costs arcs in integer ms", "[route]") {
    const auto g = build_routing_graph(cross(), {});
    REQUIRE(g);
    CHECK(g->nodes().size() == 5); // node 6 is a shape node of way 50
    CHECK(g->arcs().size() == 10);
    const auto r = solve(*g, 2, 4);
    REQUIRE(r.reachable);
    // 2 x 100 m at 30 km/h (12000 ms each) + a left turn (4000 ms) at the 4-way node.
    CHECK(r.cost_ms == 28000);
    CHECK(r.length_mm == 200000);
    CHECK(ways_of(*g, r) == std::vector<std::int64_t>{10, 20});
    for (const auto& a : g->arcs())
        if (a.way_id == 50) { CHECK(a.length_mm == 200000); CHECK(a.seg_first == 0); CHECK(a.seg_last == 1); }
}

TEST_CASE("Speeds: maxspeed, direction tags, zones, class defaults and the cap", "[route]") {
    GraphBuilder b;
    for (int i = 0; i < 12; ++i) b.node(i + 1, 1000 * (i % 2), 100 * i);
    b.way(1, {1, 2}, {{"maxspeed", "50"}});
    b.way(2, {3, 4}, {{"maxspeed:forward", "100"}, {"maxspeed:backward", "30"}});
    b.way(3, {5, 6}, {{"maxspeed", "DE:urban"}});
    b.way(4, {7, 8}, {{"highway", "motorway"}, {"maxspeed", "none"}});
    b.way(5, {9, 10}, {{"highway", "primary"}, {"maxspeed", "200"}});
    b.way(6, {11, 12}, {{"highway", "primary"}, {"maxspeed", "30 mph"}});
    const auto g = build_routing_graph(b.build(), {});
    REQUIRE(g);
    std::vector<std::pair<std::int64_t, std::uint16_t>> speeds;
    for (const auto& a : g->arcs()) speeds.emplace_back(a.way_id * (a.forward ? 1 : -1), a.speed_kmh);
    // Typed helper: pair's converting constructor from int literals trips MSVC C4244 (warnings are errors).
    const auto sp = [](std::int64_t way, std::uint16_t kmh) { return std::pair<std::int64_t, std::uint16_t>{way, kmh}; };
    CHECK(speeds == std::vector<std::pair<std::int64_t, std::uint16_t>>{
        sp(1, 50), sp(-1, 50), sp(2, 100), sp(-2, 30), sp(3, 50), sp(-3, 50), sp(4, 120), sp(5, 130), sp(-5, 130),
        sp(6, 48), sp(-6, 48)});
    // 1000 m (the arc rises 100 m too: sqrt(1000^2 + 100^2) = 1004.988 m, ceil to mm) at 50 km/h.
    CHECK(g->arcs()[0].length_mm == 1004988);
    CHECK(g->arcs()[0].cost_ms == (1004988ull * 36 + 499) / 500);
}

TEST_CASE("Access tags close ways; the most specific key decides", "[route]") {
    GraphBuilder b;
    for (int i = 0; i < 8; ++i) b.node(i + 1, 100 * i, 0);
    b.way(1, {1, 2}, {{"access", "private"}});
    b.way(2, {3, 4}, {{"access", "no"}, {"motorcar", "yes"}});
    b.way(3, {5, 6}, {{"motor_vehicle", "agricultural"}});
    b.way(4, {7, 8}, {{"highway", "path"}});
    const auto g = build_routing_graph(b.build(), {});
    REQUIRE(g);
    CHECK(g->stats().ways_used == 1);
    CHECK(g->stats().ways_access_blocked == 2);
    CHECK(g->stats().ways_not_routable == 1);
    REQUIRE(g->arcs().size() == 2);
    CHECK(g->arcs()[0].way_id == 2);
}

TEST_CASE("Oneway: yes, -1, implied (motorway, roundabout), reversible", "[route][oneway]") {
    {
        const auto g = build_routing_graph(cross(false, 100, {{"oneway", "yes"}}), {});
        REQUIRE(g);
        const auto r = solve(*g, 3, 2); // N -> S may not use C <- N
        REQUIRE(r.reachable);
        CHECK(ways_of(*g, r) == std::vector<std::int64_t>{50, 20, 10});
        CHECK(r.cost_ms == 24000 + 12000 + 2000 + 12000); // right turn at C
        const auto back = solve(*g, 2, 3);
        CHECK(ways_of(*g, back) == std::vector<std::int64_t>{10, 30});
    }
    {
        const auto g = build_routing_graph(cross(false, 100, {{"oneway", "-1"}}), {});
        REQUIRE(g);
        CHECK(ways_of(*g, solve(*g, 3, 2)) == std::vector<std::int64_t>{30, 10});
        CHECK(ways_of(*g, solve(*g, 2, 3)) == std::vector<std::int64_t>{10, 20, 50});
    }
    {
        // A one-way into a dead end: nothing leaves N, so N -> S is unreachable.
        const auto g = build_routing_graph(cross(true, 100, {{"oneway", "yes"}}), {});
        REQUIRE(g);
        CHECK_FALSE(solve(*g, 3, 2).reachable);
        CHECK(solve(*g, 2, 3).reachable);
    }
    for (const auto& tags : std::vector<std::vector<std::pair<std::string, std::string>>>{
             {{"highway", "motorway"}}, {{"junction", "roundabout"}}, {{"junction", "roundabout"}, {"oneway", "yes"}}}) {
        const auto g = build_routing_graph(cross(false, 100, tags), {});
        REQUIRE(g);
        CHECK(ways_of(*g, solve(*g, 3, 2)) == std::vector<std::int64_t>{50, 20, 10});
    }
    {
        const auto g = build_routing_graph(cross(false, 100, {{"highway", "motorway"}, {"oneway", "no"}}), {});
        REQUIRE(g);
        CHECK(ways_of(*g, solve(*g, 3, 2)) == std::vector<std::int64_t>{30, 10});
    }
    {
        const auto g = build_routing_graph(cross(false, 100, {{"oneway", "reversible"}}), {});
        REQUIRE(g);
        CHECK(g->stats().ways_no_direction == 1);
        CHECK(ways_of(*g, solve(*g, 2, 3)) == std::vector<std::int64_t>{10, 20, 50});
    }
}

TEST_CASE("no_left_turn forces the detour; the forbidden transition never appears", "[route][restriction]") {
    const std::vector<TurnRestriction> rules{via_node_rule(10, 1, 20, "no_left_turn")};
    const auto g = build_routing_graph(cross(), rules);
    REQUIRE(g);
    CHECK(g->stats().restrictions_applied == 1);
    const auto r = solve(*g, 2, 4);
    REQUIRE(r.reachable);
    CHECK(ways_of(*g, r) == std::vector<std::int64_t>{10, 30, 50});
    CHECK(r.cost_ms == 48000);
    // The rule binds only arrivals on way 10: W -> S (a right turn from 20 onto 10) stays direct.
    const auto back = solve(*g, 4, 2);
    CHECK(ways_of(*g, back) == std::vector<std::int64_t>{20, 10});
}

TEST_CASE("only_straight_on allows nothing but the named way", "[route][restriction]") {
    const std::vector<TurnRestriction> rules{via_node_rule(10, 1, 30, "only_straight_on")};
    const auto g = build_routing_graph(cross(), rules);
    REQUIRE(g);
    const auto r = solve(*g, 2, 5); // S -> E: the right turn at C is not allowed
    REQUIRE(r.reachable);
    CHECK(ways_of(*g, r) == std::vector<std::int64_t>{10, 30, 50, 20, 40});
    CHECK(r.cost_ms == 72000);
    // Arriving from the west, the rule does not apply.
    CHECK(ways_of(*g, solve(*g, 4, 5)) == std::vector<std::int64_t>{20, 40});
}

TEST_CASE("U-turns only at a dead end, and a same-way no_u_turn keeps straight on", "[route][restriction]") {
    {
        // No link way; N is 150 m out so the detour over the dead-end E arm is the cheapest.
        const std::vector<TurnRestriction> rules{via_node_rule(10, 1, 20, "no_left_turn")};
        const auto g = build_routing_graph(cross(true, 150), rules);
        REQUIRE(g);
        const auto r = solve(*g, 2, 4);
        REQUIRE(r.reachable);
        CHECK(ways_of(*g, r) == std::vector<std::int64_t>{10, 40, 40, 20});
        CHECK(r.cost_ms == 12000 + 2000 + 12000 + 20000 + 12000 + 12000);
        // Two dead ends at equal cost (N at 100 m): a tie, resolved identically every time.
        const auto tied = build_routing_graph(cross(true, 100), rules);
        REQUIRE(tied);
        const auto first = solve(*tied, 2, 4);
        CHECK(first.cost_ms == 70000);
        for (int i = 0; i < 3; ++i) {
            const auto again = build_routing_graph(cross(true, 100), rules);
            REQUIRE(again);
            RouteSearcher searcher(*again);
            const auto r2 = searcher.find({2, 4});
            REQUIRE(r2);
            CHECK(r2->arcs == first.arcs);
        }
    }
    {
        // A U-turn is not offered where another exit exists.
        const auto g = build_routing_graph(cross(), {});
        REQUIRE(g);
        std::vector<Transition> succ;
        for (std::uint32_t a = 0; a < g->arcs().size(); ++a) {
            g->successors(a, succ);
            for (const auto& t : succ)
                if (t.arc == g->arcs()[a].reverse) CHECK(g->nodes()[g->arcs()[a].to].degree == 1);
        }
    }
    {
        GraphBuilder b;
        b.node(1, 0, 0); b.node(2, 0, -100); b.node(3, 0, 100); b.node(4, -100, 0);
        b.way(10, {2, 1, 3});
        b.way(20, {1, 4});
        const std::vector<TurnRestriction> rules{via_node_rule(10, 1, 10, "no_u_turn")};
        const auto g = build_routing_graph(b.build(), rules);
        REQUIRE(g);
        CHECK(g->stats().restrictions_applied == 1);
        CHECK(ways_of(*g, solve(*g, 2, 3)) == std::vector<std::int64_t>{10, 10});
    }
}

TEST_CASE("Unknown nodes, unreachable components, unmatched and via-way restrictions", "[route][restriction]") {
    RoadGraph graph = cross();
    graph.nodes.push_back({100, {5000000, 0}, {}});
    graph.nodes.push_back({101, {5100000, 0}, {}});
    graph.ways.push_back({99, 3, {100, 101}, {{"highway", "residential"}}});
    TurnRestriction via_way = via_node_rule(10, 0, 20, "no_u_turn");
    via_way.via_ways = {40};
    const std::vector<TurnRestriction> rules{via_node_rule(10, 1, 77, "no_left_turn"), via_node_rule(10, 4, 20, "no_left_turn"), via_way};
    const auto g = build_routing_graph(graph, rules);
    REQUIRE(g);
    CHECK(g->stats().restrictions_unmatched == 2); // way 77 is absent; way 10 never reaches node 4
    CHECK(g->stats().restrictions_via_way == 1);
    CHECK(g->stats().restrictions_applied == 0);
    CHECK_FALSE(solve(*g, 2, 100).reachable);
    CHECK(solve(*g, 2, 2).reachable);
    RouteSearcher searcher(*g);
    const auto bad = searcher.find({2, 6}); // a collapsed shape node is not a graph node
    REQUIRE_FALSE(bad);
    CHECK(bad.error().code == ErrorCode::InvalidArgument);
    RouteConfig tight = default_route_config();
    tight.max_settled = 1;
    const auto small = build_routing_graph(graph, {}, tight);
    REQUIRE(small);
    RouteSearcher limited(*small);
    const auto capped = limited.find({2, 100}); // exhausting the component needs more than one settle
    REQUIRE_FALSE(capped);
    CHECK(capped.error().code == ErrorCode::LimitExceeded);
}


TEST_CASE("A* equals Dijkstra on random restricted grids", "[route][restriction][oneway]") {
    std::size_t reachable = 0, total = 0, with_turns = 0;
    for (std::uint64_t seed = 1; seed <= 6; ++seed) {
        const auto scene = random_scene(seed, 14, 0.3);
        const auto g = build_routing_graph(scene.graph, scene.rules);
        REQUIRE(g);
        CHECK(g->stats().restrictions_applied > 0);
        RouteSearcher searcher(*g);
        g2m_test::SplitMix64 rng(seed * 977);
        std::vector<std::int64_t> graph_nodes;
        for (const auto& n : g->nodes()) graph_nodes.push_back(n.osm_id);
        for (int q = 0; q < 120; ++q) {
            const auto from = graph_nodes[rng.next() % graph_nodes.size()];
            const auto to = graph_nodes[rng.next() % graph_nodes.size()];
            const auto r = searcher.find({from, to});
            REQUIRE(r);
            const auto ref = g2m_test::dijkstra_reference(*g, from, to);
            ++total;
            REQUIRE(r->reachable == ref.reachable);
            REQUIRE(r->cost_ms == ref.cost_ms);
            REQUIRE(g2m_test::validate_route(*g, *r, from, to).empty());
            if (r->reachable) ++reachable;
            if (r->arcs.size() > 3) ++with_turns;
        }
    }
    std::printf("route random grids: %zu queries, %zu reachable, %zu with > 3 arcs\n", total, reachable, with_turns);
    CHECK(reachable > total / 2);
}

// Region-scale timing (not asserted): a 301 x 301 street grid, 200 m blocks
// (60 km square, 90k nodes) with arterials every 10th line; plain A* and A*
// with 16 ALT landmarks. Run by hand:
//   g2m_route_tests "[bench]" -s
TEST_CASE("Route timing on a region-sized grid", "[.][route][bench]") {
    const int n = 301; // 300 blocks per side, divisible by the 5-block way length
    GraphBuilder b;
    const auto id = [&](int i, int j) { return std::int64_t{1} + i * n + j; };
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) b.node(id(i, j), 200 * j, 200 * i);
    std::int64_t way_id = 1;
    for (int dir = 0; dir < 2; ++dir)
        for (int line = 0; line < n; ++line)
            for (int k = 0; k + 5 <= n - 1; k += 5) {
                std::vector<std::int64_t> refs;
                for (int m = k; m <= k + 5; ++m) refs.push_back(dir == 0 ? id(line, m) : id(m, line));
                const char* cls = line % 30 == 0 ? "primary" : line % 10 == 0 ? "secondary" : "residential";
                b.way(way_id++, refs, {{"highway", cls}});
            }
    const auto t0 = std::chrono::steady_clock::now();
    const auto g = build_routing_graph(b.build(), {});
    REQUIRE(g);
    const auto t1 = std::chrono::steady_clock::now();
    std::printf("grid: nodes=%zu arcs=%zu build_ms=%.1f\n", g->nodes().size(), g->arcs().size(),
                std::chrono::duration<double, std::milli>(t1 - t0).count());
    const auto a0 = std::chrono::steady_clock::now();
    const auto alt = RouteLandmarks::build(*g, 16);
    REQUIRE(alt);
    std::printf("grid: ALT 16 landmarks preprocessing %.0f ms\n",
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a0).count());
    RouteSearcher searcher(*g);
    RouteSearcher alt_searcher(*g, &*alt);
    g2m_test::SplitMix64 rng(42);
    for (const double km : {5.0, 15.0, 30.0}) {
        double sum_ms = 0, max_ms = 0, sum_dij = 0, sum_alt = 0, max_alt = 0;
        std::size_t settled_alt = 0;
        std::size_t settled = 0;
        const int queries = 40;
        for (int q = 0; q < queries; ++q) {
            const int si = static_cast<int>(rng.next() % n), sj = static_cast<int>(rng.next() % n);
            const int step = static_cast<int>(km * 1000 / 200 / 1.4142);
            const int ti = std::clamp(si + (rng.next() % 2 ? step : -step), 0, n - 1);
            const int tj = std::clamp(sj + (rng.next() % 2 ? step : -step), 0, n - 1);
            const auto a = std::chrono::steady_clock::now();
            const auto r = searcher.find({id(si, sj), id(ti, tj)});
            const auto z = std::chrono::steady_clock::now();
            REQUIRE(r);
            const double ms = std::chrono::duration<double, std::milli>(z - a).count();
            sum_ms += ms; max_ms = std::max(max_ms, ms); settled += searcher.stats().settled;
            const auto b0 = std::chrono::steady_clock::now();
            const auto ra = alt_searcher.find({id(si, sj), id(ti, tj)});
            const double alt_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - b0).count();
            REQUIRE(ra);
            sum_alt += alt_ms; max_alt = std::max(max_alt, alt_ms); settled_alt += alt_searcher.stats().settled;
            CHECK(ra->cost_ms == r->cost_ms);
            if (q < 3) {
                const auto d0 = std::chrono::steady_clock::now();
                const auto ref = g2m_test::dijkstra_reference(*g, id(si, sj), id(ti, tj));
                sum_dij += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - d0).count();
                CHECK(ref.cost_ms == r->cost_ms);
            }
        }
        // Snapping cost: two snaps per request (start with a heading, destination without).
        const auto s0 = std::chrono::steady_clock::now();
        for (int q = 0; q < 100; ++q) {
            SnapOptions o;
            o.heading_dx = 1.0;
            o.max_distance_mm = 150000; // blocks are 200 m
            const auto p = g->snap({static_cast<std::int64_t>(rng.next() % 60000000), static_cast<std::int64_t>(rng.next() % 60000000)}, o);
            REQUIRE(p);
        }
        const double snap_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - s0).count() / 100;
        std::printf("grid snap: %.3f ms per snap\n", snap_ms);
        std::printf("grid %.0f km (straight line): A* mean %.2f ms, max %.2f ms, settled mean %zu; ALT mean %.2f ms, max %.2f ms, "
                    "settled mean %zu; reference Dijkstra mean %.1f ms\n",
                    km, sum_ms / queries, max_ms, settled / queries, sum_alt / queries, max_alt, settled_alt / queries, sum_dij / 3);
    }
}
