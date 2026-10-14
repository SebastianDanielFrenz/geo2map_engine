// Routing graph + A* on synthetic graphs (G4 slice 1): costs, oneway,
// restrictions, U-turns, unreachable targets, ties, and A* == Dijkstra on
// random graphs. Coordinates in metres (GraphBuilder scales to mm).
#include "route_reference.h"
#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdio>
#include <functional>
#include <queue>
#include <set>
#include <tuple>

using namespace g2m;
using namespace g2m::route;
using g2m_test::GraphBuilder;
using g2m_test::via_node_rule;

namespace {

// A cross at C(0,0): S south, N north (y = north_y), W west, E east, 100 m arms.
// Way 50 links W to N through (-100, north_y) unless `no_link`.
RoadGraph cross(bool no_link = false, std::int64_t north_y = 100, std::vector<std::pair<std::string, std::string>> north_tags = {}) {
    GraphBuilder b;
    b.node(1, 0, 0); b.node(2, 0, -100); b.node(3, 0, north_y); b.node(4, -100, 0); b.node(5, 100, 0); b.node(6, -100, north_y);
    b.way(10, {2, 1});
    b.way(20, {1, 4});
    b.way(30, {1, 3}, std::move(north_tags));
    b.way(40, {1, 5});
    if (!no_link) b.way(50, {4, 6, 3});
    return b.build();
}

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
    CHECK(speeds == std::vector<std::pair<std::int64_t, std::uint16_t>>{
        {1, 50}, {-1, 50}, {2, 100}, {-2, 30}, {3, 50}, {-3, 50}, {4, 120}, {5, 130}, {-5, 130}, {6, 48}, {-6, 48}});
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
    b.way(4, {7, 8}, {{"highway", "track"}});
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

namespace {

// A jittered grid of streets with random oneways, speeds, gaps and restrictions.
struct RandomScene {
    RoadGraph graph;
    std::vector<TurnRestriction> rules;
    std::vector<std::int64_t> node_ids;
};
RandomScene random_scene(std::uint64_t seed, int n, double restriction_share) {
    g2m_test::SplitMix64 rng(seed);
    GraphBuilder b;
    RandomScene scene;
    const auto id = [&](int i, int j) { return std::int64_t{1000} + i * n + j; };
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const auto jx = static_cast<std::int64_t>(rng.next() % 41);
            const auto jy = static_cast<std::int64_t>(rng.next() % 41);
            b.node(id(i, j), 200 * j + jx - 20, 200 * i + jy - 20);
        }
    std::int64_t way_id = 1;
    std::vector<std::vector<std::pair<std::int64_t, std::int64_t>>> incident(static_cast<std::size_t>(n * n));
    const char* classes[] = {"residential", "tertiary", "secondary", "primary", "service"};
    for (int dir = 0; dir < 2; ++dir) {
        for (int line = 0; line < n; ++line) {
            int k = 0;
            while (k < n - 1) {
                const int run = 1 + static_cast<int>(rng.next() % 4);
                const int end = std::min(n - 1, k + run);
                const bool gap = rng.next() % 9 == 0;
                if (!gap) {
                    std::vector<std::int64_t> refs;
                    for (int m = k; m <= end; ++m) refs.push_back(dir == 0 ? id(line, m) : id(m, line));
                    std::vector<std::pair<std::string, std::string>> tags{{"highway", classes[rng.next() % 5]}};
                    const auto r = rng.next() % 10;
                    if (r == 0) tags.emplace_back("oneway", "yes");
                    if (r == 1) tags.emplace_back("oneway", "-1");
                    const auto s = rng.next() % 6;
                    if (s == 0) tags.emplace_back("maxspeed", "30");
                    if (s == 1) tags.emplace_back("maxspeed", "80");
                    for (const auto ref : refs) incident[static_cast<std::size_t>(ref - 1000)].push_back({way_id, ref});
                    b.way(way_id, refs, tags);
                }
                ++way_id;
                k = end;
            }
        }
    }
    scene.graph = b.build();
    const char* values[] = {"no_left_turn", "no_right_turn", "no_straight_on", "no_u_turn", "only_straight_on", "only_left_turn"};
    for (std::size_t v = 0; v < incident.size(); ++v) {
        if (incident[v].size() < 2 || rng.unit() >= restriction_share) continue;
        const auto& from = incident[v][rng.next() % incident[v].size()];
        const auto& to = incident[v][rng.next() % incident[v].size()];
        TurnRestriction r = via_node_rule(from.first, from.second, to.first, values[rng.next() % 6]);
        r.relation_id = static_cast<std::int64_t>(v);
        scene.rules.push_back(r);
    }
    for (const auto& node : scene.graph.nodes) scene.node_ids.push_back(node.osm_id);
    return scene;
}

} // namespace

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

namespace {

// ALT experiment for the design decision (docs/routing_design.md, slice 2):
// landmark lower bounds on the node graph (travel cost only; turn costs are
// >= 0 and restrictions only remove options, so the bound stays admissible
// for the edge-based search). Test-only until slice 2 moves it into g2m_route.
struct AltTable {
    std::vector<std::vector<std::uint64_t>> from, to; // [landmark][node]
};
std::vector<std::uint64_t> node_dijkstra(const RoutingGraph& g, std::uint32_t source, bool reverse) {
    const auto n = g.nodes().size();
    std::vector<std::vector<std::pair<std::uint32_t, std::uint32_t>>> adj(n);
    for (const auto& a : g.arcs()) (reverse ? adj[a.to] : adj[a.from]).push_back({reverse ? a.from : a.to, a.cost_ms});
    std::vector<std::uint64_t> d(n, UINT64_MAX);
    std::set<std::pair<std::uint64_t, std::uint32_t>> open{{0, source}};
    d[source] = 0;
    while (!open.empty()) {
        const auto [du, u] = *open.begin();
        open.erase(open.begin());
        for (const auto& [v, c] : adj[u])
            if (du + c < d[v]) { open.erase({d[v], v}); d[v] = du + c; open.insert({d[v], v}); }
    }
    return d;
}
AltTable build_alt(const RoutingGraph& g, int count) {
    AltTable t;
    std::vector<std::uint64_t> nearest(g.nodes().size(), UINT64_MAX);
    std::uint32_t next = 0;
    for (int k = 0; k < count; ++k) {
        t.from.push_back(node_dijkstra(g, next, false));
        t.to.push_back(node_dijkstra(g, next, true));
        std::uint64_t best = 0;
        for (std::uint32_t v = 0; v < nearest.size(); ++v) {
            const auto dv = std::min(t.from.back()[v], t.to.back()[v]);
            nearest[v] = std::min(nearest[v], dv);
            if (nearest[v] != UINT64_MAX && nearest[v] > best) { best = nearest[v]; next = v; }
        }
    }
    return t;
}
std::uint64_t alt_bound(const AltTable& t, std::uint32_t v, std::uint32_t target) {
    std::uint64_t h = 0;
    for (std::size_t k = 0; k < t.from.size(); ++k) {
        const auto lv = t.from[k][v], lt = t.from[k][target], vl = t.to[k][v], tl = t.to[k][target];
        if (lt != UINT64_MAX && lv != UINT64_MAX && lt > lv) h = std::max(h, lt - lv);
        if (vl != UINT64_MAX && tl != UINT64_MAX && vl > tl) h = std::max(h, vl - tl);
    }
    return h;
}
// Edge-based A* with the ALT bound; returns (cost, settled).
std::pair<std::uint64_t, std::size_t> alt_search(const RoutingGraph& g, const AltTable& t, std::uint32_t s, std::uint32_t target) {
    const auto arcs = g.arcs();
    std::vector<std::uint64_t> best(arcs.size(), UINT64_MAX);
    using E = std::tuple<std::uint64_t, std::uint64_t, std::uint32_t>;
    std::priority_queue<E, std::vector<E>, std::greater<E>> open;
    for (const auto& a : g.out_arcs(s)) {
        const auto i = static_cast<std::uint32_t>(&a - arcs.data());
        best[i] = a.cost_ms;
        open.push({a.cost_ms + alt_bound(t, a.to, target), a.cost_ms, i});
    }
    std::vector<Transition> succ;
    std::size_t settled = 0;
    while (!open.empty()) {
        const auto [f, gc, a] = open.top();
        open.pop();
        if (gc != best[a]) continue;
        if (arcs[a].to == target) return {gc, settled};
        ++settled;
        g.successors(a, succ);
        for (const auto& tr : succ) {
            const auto ng = gc + tr.turn_ms + arcs[tr.arc].cost_ms;
            if (ng < best[tr.arc]) { best[tr.arc] = ng; open.push({ng + alt_bound(t, arcs[tr.arc].to, target), ng, tr.arc}); }
        }
    }
    return {UINT64_MAX, settled};
}

} // namespace

// Region-scale timing (not asserted): a 301 x 301 street grid, 200 m blocks
// (60 km square, 90k nodes) with arterials every 10th line. Run by hand:
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
    const AltTable alt = build_alt(*g, 16);
    std::printf("grid: ALT 16 landmarks preprocessing %.0f ms (test code, std::set Dijkstra)\n",
                std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a0).count());
    RouteSearcher searcher(*g);
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
            const auto [alt_cost, alt_settled] = alt_search(*g, alt, g->find_node(id(si, sj)), g->find_node(id(ti, tj)));
            const double alt_ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - b0).count();
            sum_alt += alt_ms; max_alt = std::max(max_alt, alt_ms); settled_alt += alt_settled;
            CHECK(alt_cost == r->cost_ms);
            if (q < 3) {
                const auto d0 = std::chrono::steady_clock::now();
                const auto ref = g2m_test::dijkstra_reference(*g, id(si, sj), id(ti, tj));
                sum_dij += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - d0).count();
                CHECK(ref.cost_ms == r->cost_ms);
            }
        }
        std::printf("grid %.0f km (straight line): A* mean %.2f ms, max %.2f ms, settled mean %zu; ALT mean %.2f ms, max %.2f ms, "
                    "settled mean %zu; reference Dijkstra mean %.1f ms\n",
                    km, sum_ms / queries, max_ms, settled / queries, sum_alt / queries, max_alt, settled_alt / queries, sum_dij / 3);
    }
}
