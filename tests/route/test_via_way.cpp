// Via-way turn restrictions in the edge-based search (G4 slice 3): synthetic scenes for No and
// Only rules with one and two via ways, point routes, and A* (plain and with ALT) == a reference
// Dijkstra on random restricted grids. The reference does not use the library's rule states: its
// state is (arc, the last runs of one-way-after-another), and a rule is checked on that history
// by plain way-id comparison (route_via_reference below).
#include "route_reference.h"
#include "route_scenes.h"
#include "splitmix64.h"
#include "via_reference.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <map>
#include <queue>
#include <set>

using namespace g2m;
using namespace g2m::route;
using g2m_test::GraphBuilder;
using g2m_test::via_node_rule;
using g2m_test::via_way_rule;
using g2m_test::ViaReference;
using g2m_test::route_via_reference;
using g2m_test::route_violates;

namespace {

Result<RoutingGraph> build_with(const RoadGraph& graph, std::vector<TurnRestriction> rules) { return build_routing_graph(graph, rules); }

std::vector<std::int64_t> ways_of(const RoutingGraph& g, const Route& r) {
    std::vector<std::int64_t> out;
    for (const auto a : r.arcs) out.push_back(g.arcs()[a].way_id);
    return out;
}

// ---- the reference ---------------------------------------------------------------------------

// ---- scenes ----------------------------------------------------------------------------------

// F = 10 (1 -> 2), V = 20 (2 -> 3), T = 30 (3 -> 4), and a longer bypass 2 -> 5 -> 4 (ways 40, 50).
//
//        3 ---T--- 4
//        |          \
//        V           50
//        |            \
//  1 -F- 2 ---40---    5   (5 is far south-east so the bypass is longer)
RoadGraph via_scene(bool two_via = false) {
    GraphBuilder b;
    b.node(1, -100, 0); b.node(2, 0, 0); b.node(3, 0, 100); b.node(4, 100, 100); b.node(5, 100, -400);
    b.way(10, {1, 2});
    b.way(40, {2, 5});
    b.way(50, {5, 4});
    if (!two_via) {
        b.way(20, {2, 3});
        b.way(30, {3, 4});
    } else {
        b.node(7, 50, 100);                   // V1 = 21 (2 -> 3), V2 = 22 (3 -> 7), T = 30 (7 -> 4)
        b.way(21, {2, 3});
        b.way(22, {3, 7});
        b.way(30, {7, 4});
    }
    return b.build();
}

} // namespace

TEST_CASE("A No rule over a via way forces the bypass, only after the whole sequence", "[route][via_way]") {
    const auto graph = via_scene();
    const auto plain = build_routing_graph(graph, {});
    REQUIRE(plain);
    RouteSearcher plain_search(*plain);
    const auto direct = plain_search.find({1, 4});
    REQUIRE(direct);
    CHECK(ways_of(*plain, *direct) == std::vector<std::int64_t>{10, 20, 30});

    const std::vector<TurnRestriction> rules{via_way_rule(1, "no_straight_on", 10, {20}, 30)};
    const auto g = build_routing_graph(graph, rules);
    REQUIRE(g);
    CHECK(g->stats().restrictions_via_way == 1);
    CHECK(g->stats().restrictions_via_way_applied == 1);
    CHECK(g->stats().via_way_instances == 1);
    CHECK(g->stats().restrictions_unmatched == 0);
    CHECK(g->stats().via_way_states > 0);
    CHECK(g->state_count() == g->arcs().size() + g->stats().via_way_states);
    RouteSearcher search(*g);
    const auto r = search.find({1, 4});
    REQUIRE(r);
    CHECK(ways_of(*g, *r) == std::vector<std::int64_t>{10, 40, 50});
    CHECK(r->cost_ms > direct->cost_ms);
    for (const auto a : r->arcs) CHECK(a < g->arcs().size()); // routes hold real arcs only
    // Parts of the sequence are fine: ending on V, starting on V, and the opposite direction.
    CHECK(ways_of(*g, *search.find({1, 3})) == std::vector<std::int64_t>{10, 20});
    CHECK(ways_of(*g, *search.find({2, 4})) == std::vector<std::int64_t>{20, 30});
    CHECK(ways_of(*g, *search.find({4, 1})) == std::vector<std::int64_t>{30, 20, 10});
    // Without the F arrival there is no history: from the bypass onto V and T is allowed.
    CHECK(ways_of(*g, *search.find({5, 4})) == std::vector<std::int64_t>{50});
    // The plain graph and the graph with the rules agree on every query the rule does not touch.
    for (const std::int64_t from : {2, 3, 4, 5})
        for (const std::int64_t to : {1, 2, 3, 4, 5}) {
            RouteSearcher ps(*plain);
            CHECK(search.find({from, to})->cost_ms == ps.find({from, to})->cost_ms);
        }
    // Without ALT and with: same.
    const auto landmarks = RouteLandmarks::build(*g, 3);
    REQUIRE(landmarks);
    RouteSearcher with_alt(*g, &*landmarks);
    CHECK(with_alt.find({1, 4})->cost_ms == r->cost_ms);
}

TEST_CASE("An Only rule over a via way forces the sequence from the first arrival", "[route][via_way]") {
    const auto graph = via_scene();
    const std::vector<TurnRestriction> rules{via_way_rule(1, "only_straight_on", 10, {20}, 30)};
    const auto g = build_routing_graph(graph, rules);
    REQUIRE(g);
    CHECK(g->stats().restrictions_via_way_applied == 1);
    RouteSearcher search(*g);
    // 1 -> 5 would take way 40 at node 2; with the rule the car must go V, T, then back along 50.
    const auto r = search.find({1, 5});
    REQUIRE(r);
    CHECK(ways_of(*g, *r) == std::vector<std::int64_t>{10, 20, 30, 50});
    // Not arriving on F: free choice at node 2 (from 5 back to 2 and on to 3 or 1).
    CHECK(ways_of(*g, *search.find({5, 3})) == std::vector<std::int64_t>{40, 20});
    // The rule says nothing about 1 -> 4 (it takes the sequence anyway) or about ending at node 2.
    CHECK(ways_of(*g, *search.find({1, 4})) == std::vector<std::int64_t>{10, 20, 30});
    CHECK(ways_of(*g, *search.find({1, 2})) == std::vector<std::int64_t>{10});
    // The same query without the rule takes the bypass.
    const auto relaxed = build_routing_graph(graph, {});
    REQUIRE(relaxed);
    RouteSearcher free_search(*relaxed);
    CHECK(ways_of(*relaxed, *free_search.find({1, 5})) == std::vector<std::int64_t>{10, 40});
}

TEST_CASE("Rules with two via ways, and rules that cannot be matched", "[route][via_way]") {
    const auto graph = via_scene(true);
    const auto g = build_with(graph, {via_way_rule(1, "no_straight_on", 10, {21, 22}, 30)});
    REQUIRE(g);
    CHECK(g->stats().restrictions_via_way_applied == 1);
    RouteSearcher search(*g);
    CHECK(ways_of(*g, *search.find({1, 4})) == std::vector<std::int64_t>{10, 40, 50});
    CHECK(ways_of(*g, *search.find({1, 7})) == std::vector<std::int64_t>{10, 21, 22}); // the sequence stops before T
    CHECK(ways_of(*g, *search.find({2, 4})) == std::vector<std::int64_t>{21, 22, 30});
    // Unmatched: a via way that is not in the graph, one that does not connect, a missing from way, too many via ways.
    const auto bad = build_with(graph, {via_way_rule(2, "no_straight_on", 10, {99}, 30), via_way_rule(3, "no_straight_on", 10, {50}, 30),
                                                 via_way_rule(4, "no_straight_on", 77, {21}, 30),
                                                 via_way_rule(5, "no_straight_on", 10, std::vector<std::int64_t>(17, 21), 30)});
    REQUIRE(bad);
    CHECK(bad->stats().restrictions_via_way == 4);
    CHECK(bad->stats().restrictions_via_way_applied == 0);
    CHECK(bad->stats().restrictions_unmatched == 4);
    CHECK(bad->state_count() == bad->arcs().size());
    // A No rule and an Only rule on the same sequence, and the same rule twice, build.
    const auto both = build_with(graph, {via_way_rule(1, "no_straight_on", 10, {21, 22}, 30), via_way_rule(2, "only_straight_on", 10, {21, 22}, 30),
                                                  via_way_rule(3, "no_straight_on", 10, {21, 22}, 30)});
    REQUIRE(both);
    CHECK(both->stats().via_way_instances == 3);
    RouteSearcher both_search(*both);
    // Only forces the sequence from F, No forbids its last step: node 7 is reachable from 1, node 4 is not.
    CHECK(both_search.find({1, 7})->reachable);
    CHECK_FALSE(both_search.find({1, 4})->reachable);
    CHECK(both_search.find({5, 4})->reachable); // not arriving on F
}

TEST_CASE("Via-way rules apply to point routes", "[route][via_way]") {
    const auto graph = via_scene();
    const auto g = build_with(graph, {via_way_rule(1, "no_straight_on", 10, {20}, 30)});
    REQUIRE(g);
    // A point on F, west of node 2, and a point on T, east of node 3.
    const auto from = g->snap({-50000, 0}, {50000, 1.0, 0.0, 60.0});
    const auto to = g->snap({50000, 100000}); // both directions: it has to be entered from the east
    REQUIRE(from); REQUIRE(to);
    RouteSearcher search(*g);
    const auto r = search.find(*from, *to);
    REQUIRE(r);
    REQUIRE(r->reachable);
    CHECK(ways_of(*g, *r) == std::vector<std::int64_t>{10, 40, 50, 30}); // around the block, entering T from the east
    CHECK(g2m_test::validate_point_route(*g, *r, *from, *to).empty());
}

// ---- random grids ----------------------------------------------------------------------------

TEST_CASE("A* and A* + ALT equal the history reference on random grids with via-way rules", "[route][via_way][restriction]") {
    std::size_t total = 0, reachable = 0, differ_from_plain = 0, violated_without = 0, rules_total = 0, instances_total = 0;
    for (std::uint64_t seed = 1; seed <= 8; ++seed) {
        auto scene = g2m_test::random_scene(seed, 10, 0.2);
        g2m_test::add_via_way_rules(scene.graph, scene.rules, seed, 0.35);
        const auto g = build_routing_graph(scene.graph, scene.rules);
        REQUIRE(g);
        const ViaReference ref(scene.graph, scene.rules);
        rules_total += ref.rules.size();
        instances_total += g->stats().via_way_instances;
        CHECK(g->stats().restrictions_via_way == ref.rules.size());
        CHECK(g->stats().via_way_states > 0);
        // The same scene without the via-way rules: the negative control.
        std::vector<TurnRestriction> node_rules;
        for (const auto& r : scene.rules) if (r.via_node != 0) node_rules.push_back(r);
        const auto plain = build_routing_graph(scene.graph, node_rules);
        REQUIRE(plain);
        const auto landmarks = RouteLandmarks::build(*g, 8);
        REQUIRE(landmarks);
        RouteSearcher searcher(*g), alt(*g, &*landmarks), plain_search(*plain);
        g2m_test::SplitMix64 rng(seed * 7919);
        std::vector<std::int64_t> ids;
        for (const auto& n : g->nodes()) ids.push_back(n.osm_id);
        for (int q = 0; q < 70; ++q) {
            const auto from = ids[rng.next() % ids.size()], to = ids[rng.next() % ids.size()];
            const auto r = searcher.find({from, to});
            const auto a = alt.find({from, to});
            REQUIRE(r); REQUIRE(a);
            const Route expect = route_via_reference(*g, ref, from, to);
            ++total;
            if (r->reachable != expect.reachable) {
                std::printf("MISMATCH seed %llu %lld -> %lld impl %d ref %d\n", (unsigned long long)seed, (long long)from, (long long)to, r->reachable, expect.reachable);
                for (const auto& rule : ref.rules) {
                    std::printf("  rule %s F%lld V", std::string(rule.value).c_str(), (long long)rule.from_way);
                    for (auto v : rule.via_ways) std::printf(" %lld", (long long)v);
                    std::printf(" T%lld\n", (long long)rule.to_way);
                }
                for (auto arc_id : r->arcs) std::printf("  arc way %lld %lld->%lld\n", (long long)g->arcs()[arc_id].way_id, (long long)g->nodes()[g->arcs()[arc_id].from].osm_id, (long long)g->nodes()[g->arcs()[arc_id].to].osm_id);
            }
            REQUIRE(r->reachable == expect.reachable);
            REQUIRE(r->cost_ms == expect.cost_ms);
            REQUIRE(a->reachable == expect.reachable);
            REQUIRE(a->cost_ms == expect.cost_ms);
            if (!r->reachable) continue;
            ++reachable;
            CHECK(g2m_test::validate_route(*g, *r, from, to).empty());
            CHECK_FALSE(route_violates(*g, ref, *r));
            const auto p = plain_search.find({from, to});
            REQUIRE(p);
            if (p->cost_ms != r->cost_ms || p->arcs != r->arcs) ++differ_from_plain;
            if (route_violates(*g, ref, *p)) ++violated_without;
        }
    }
    std::printf("via-way grids: %zu rules (%zu node chains), %zu queries, %zu reachable, %zu routes differ from the graph without the via-way rules, "
                "%zu of those plain routes break a rule\n", rules_total, instances_total, total, reachable, differ_from_plain, violated_without);
    CHECK(rules_total > 10);
    CHECK(differ_from_plain > 0);       // negative control: dropping the rules changes routes
    CHECK(violated_without > 0);        // ... and the dropped-rule routes really do break them
}
