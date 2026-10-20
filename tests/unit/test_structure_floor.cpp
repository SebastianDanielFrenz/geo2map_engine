// tests/unit/test_structure_floor.cpp - the design floor of a tunnel NETWORK (terrain migration phase 7a-2,
// include/g2m/layer/structure_floor.h, docs/structure_classification.md 3.2).
//
// Recorded sabotages (each made the named cases fail, then was reverted):
//   S5 stretches solved on their own (each stretch a one-stretch network, the junction an interior end): "a junction of
//      three stretches has one floor value" fails (the junction takes the ground-needed value 106.3 m, not the
//      99.11 m the portal chords give: the chord check of that case).
//   S6 the chord over every portal pair instead of the adjacent pairs with the station on a shortest route:
//      "a through portal splits the chord" fails (the A-C chord, 100.0 m, lowers the middle of A-B from 100.5 m).
//   S7 the reach handler skipped: "anchors the floor cannot climb between are moved toward each other" fails (the
//      anchors stay 10 m apart over a 100 m stretch: both pair counters stay 1 and the cones disagree by 5 m).
#include "g2m/layer/structure_classify.h"
#include "g2m/layer/structure_floor.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

using namespace g2m::structure;

namespace {
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

FloorStretch stretch(int from, int to, double length, double ground) {
    FloorStretch s; s.from = from; s.to = to; s.length_m = length; s.ground.assign(station_count(length), ground); return s;
}
FloorStretch stretch_profile(int from, int to, double length, const std::vector<double>& ground) {
    FloorStretch s; s.from = from; s.to = to; s.length_m = length; s.ground = ground; REQUIRE(ground.size() == station_count(length)); return s;
}
double lipschitz_excess(const NetworkFloor& f, const FloorNetwork& net, double grade) {
    double worst = 0;
    for (std::size_t e = 0; e < net.stretches.size(); ++e)
        for (std::size_t k = 1; k < f.floor[e].size(); ++k)
            worst = std::max(worst, std::fabs(f.floor[e][k] - f.floor[e][k - 1]) -
                grade * (station_m(net.stretches[e].length_m, k) - station_m(net.stretches[e].length_m, k - 1)));
    return worst;
}
} // namespace

TEST_CASE("a chain of two portals equals the one-chain design floor bit for bit", "[structure][floor]") {
    struct Case { double length; double z0, z1; std::vector<double> ground; };
    std::vector<Case> cases;
    {   // flat city ground: the floor drops from each portal and follows the ground at the needed depth
        Case c{400.0, 100.0, 100.0, std::vector<double>(401, 101.0)}; cases.push_back(c);
        // a hill: the chord
        Case h{250.5, 100.0, 103.0, {}}; h.ground.resize(station_count(h.length));
        for (std::size_t i = 0; i < h.ground.size(); ++i) h.ground[i] = 100.0 + 30.0 * std::sin(0.0125 * static_cast<double>(i));
        cases.push_back(h);
        // a valley just past the reach: the portal cones matter
        Case v{300.0, 100.0, 100.0, std::vector<double>(301, 100.0)};
        for (std::size_t i = 130; i < 170; ++i) v.ground[i] = 80.0;
        cases.push_back(v);
        // a ground with a hole in the DEM
        Case n{150.0, 50.0, 52.0, std::vector<double>(151, 60.0)};
        for (std::size_t i = 40; i < 60; ++i) n.ground[i] = kNaN;
        cases.push_back(n);
    }
    for (const auto& c : cases) {
        const double grade = 0.05, needed = 5.7;
        FloorNetwork net; net.node_count = 2;
        net.stretches.push_back(stretch_profile(0, 1, c.length, c.ground));
        net.portals = {{0, c.z0, false}, {1, c.z1, false}};
        NetworkFloorParams p; p.max_grade = grade; p.needed = needed; p.reach_factor = 10.0;   // no reach shift: compare the floors
        const auto f = network_design_floor(net, p);
        REQUIRE(f.ok); REQUIRE(f.converged);
        const auto reference = design_floor(c.ground, c.length, c.z0, c.z1, true, true, grade, needed);
        REQUIRE(f.floor[0].size() == reference.size());
        for (std::size_t i = 0; i < reference.size(); ++i) CHECK(f.floor[0][i] == reference[i]);
        CHECK(f.anchor_misses == 0);
    }
}

TEST_CASE("a junction of three stretches has one floor value", "[structure][floor]") {
    // Portals A (node 0, 100 m), B (node 1, 104 m) and C (node 2, 98 m) around the junction J (node 3), 200 m, 150 m and
    // 250 m from it; the ground is flat 6 m above the portals, so the floor must drop to the needed depth in the middle.
    FloorNetwork net; net.node_count = 4;
    net.stretches = {stretch(0, 3, 200, 112.0), stretch(1, 3, 150, 112.0), stretch(2, 3, 250, 112.0)};
    net.portals = {{0, 100.0, false}, {1, 104.0, false}, {2, 98.0, false}};
    NetworkFloorParams p; p.max_grade = 0.05; p.needed = 5.7;
    const auto f = network_design_floor(net, p);
    REQUIRE(f.ok); REQUIRE(f.converged);
    const double at_a = f.floor[0].back(), at_b = f.floor[1].back(), at_c = f.floor[2].back();
    CHECK(at_a == at_b); CHECK(at_b == at_c); CHECK(f.node_floor[3] == at_a);
    CHECK(f.floor[0].front() == 100.0); CHECK(f.floor[1].front() == 104.0); CHECK(f.floor[2].front() == 98.0);
    CHECK(f.anchor_misses == 0);
    CHECK(lipschitz_excess(f, net, 0.05) <= 1e-12);
    // The junction is bounded by the lowest portal chord through it (A-C: 100 + (98 - 100) * 200 / 450), not by the ground.
    CHECK(at_a == Catch::Approx(100.0 - 2.0 * 200.0 / 450.0).margin(1e-9));
    // Control (sabotage S5): the same three stretches solved on their own, the junction an interior end, disagree.
    std::vector<double> alone;
    for (std::size_t e = 0; e < 3; ++e) {
        FloorNetwork one; one.node_count = 2;
        FloorStretch s = net.stretches[e]; s.from = 0; s.to = 1;
        one.stretches = {s};
        one.portals = {{0, net.portals[e].z_m, false}};
        alone.push_back(network_design_floor(one, p).floor[0].back());
    }
    for (const double v : alone) CHECK(v != at_a);   // each lone stretch ends at the ground-needed value, 7 m above the junction floor
}

TEST_CASE("a through portal splits the chord", "[structure][floor]") {
    // A - B - C in a line, B a portal (a ground road joins) 5 m above the straight A-C line: a hill tunnel A-B and a
    // second one B-C. The A-B stretch is bounded by the A-B chord only; the A-C pair passes B and is no chord.
    FloorNetwork net; net.node_count = 3;
    net.stretches = {stretch(0, 1, 100, 200.0), stretch(1, 2, 100, 200.0)};
    net.portals = {{0, 100.0, false}, {1, 101.0, false}, {2, 100.0, false}};
    NetworkFloorParams p; p.max_grade = 0.05; p.needed = 5.7;
    const auto f = network_design_floor(net, p);
    REQUIRE(f.ok);
    CHECK(f.floor[0][50] == Catch::Approx(100.5).margin(1e-12));
    CHECK(f.floor[1][50] == Catch::Approx(100.5).margin(1e-12));
    CHECK(f.floor[0].back() == 101.0); CHECK(f.floor[1].front() == 101.0);
    CHECK(f.pairs == 2);
}

TEST_CASE("anchors the floor cannot climb between are moved toward each other", "[structure][floor]") {
    // Two portals 100 m apart whose heights differ by 10 m: a 5 % floor climbs 5 m. The reach rule (0.9 of the grade)
    // leaves 4.5 m: 5.5 m excess, half each way.
    const auto build = [](bool free_a) {
        FloorNetwork net; net.node_count = 2;
        net.stretches = {stretch(0, 1, 100, 300.0)};
        net.portals = {{0, 100.0, free_a}, {1, 110.0, false}};
        return net;
    };
    NetworkFloorParams p; p.max_grade = 0.05; p.needed = 5.7; p.quantum_m = 1.0 / 256.0;
    {
        const auto net = build(false);
        const auto f = network_design_floor(net, p);
        REQUIRE(f.ok);
        CHECK(f.pairs == 1); CHECK(f.pairs_shifted == 1); CHECK(f.pairs_beyond_grade == 1); CHECK(f.pairs_still_beyond_grade == 0);
        CHECK(f.portal_shift_m[0] == Catch::Approx(2.75).margin(1.0 / 256.0));
        CHECK(f.portal_shift_m[1] == Catch::Approx(-2.75).margin(1.0 / 256.0));
        CHECK(f.portal_z_m[1] - f.portal_z_m[0] <= 0.9 * 0.05 * 100 + 1.0 / 128.0);
        CHECK(f.floor[0].front() == f.portal_z_m[0]); CHECK(f.floor[0].back() == f.portal_z_m[1]);
        CHECK(f.anchor_misses == 0);
        CHECK(lipschitz_excess(f, net, 0.05) <= 1e-12);
        // The moves are multiples of the quantum.
        CHECK(std::fabs(f.portal_shift_m[0] * 256.0 - std::round(f.portal_shift_m[0] * 256.0)) < 1e-9);
    }
    {   // a free end gives way alone
        const auto f = network_design_floor(build(true), p);
        CHECK(f.portal_shift_m[0] == Catch::Approx(5.5).margin(1.0 / 256.0)); CHECK(f.portal_shift_m[1] == 0.0);
    }
    {   // the same anchors with room to climb need no move
        FloorNetwork net = build(false); net.stretches = {stretch(0, 1, 250, 300.0)};
        const auto f = network_design_floor(net, p);
        CHECK(f.pairs_shifted == 0); CHECK(f.pairs_beyond_grade == 0);
    }
}

TEST_CASE("a dead end or an interior end constrains nothing, a lone ground profile alone cannot fail", "[structure][floor]") {
    // One portal only: no chord, the floor is the needed depth under the ground and the cone of the portal.
    FloorNetwork net; net.node_count = 2;
    net.stretches = {stretch(0, 1, 300, 50.0)};
    net.portals = {{0, 45.0, false}};
    NetworkFloorParams p; p.max_grade = 0.05; p.needed = 5.7;
    const auto f = network_design_floor(net, p);
    REQUIRE(f.ok);
    CHECK(f.floor[0].front() == 45.0);
    CHECK(f.floor[0].back() == Catch::Approx(50.0 - 5.7).margin(1e-12));   // 114 m from the portal: past the depth reach
    CHECK(lipschitz_excess(f, net, 0.05) <= 1e-12);
    // No portal and no ground: nothing to build a floor from.
    FloorNetwork none; none.node_count = 2;
    FloorStretch s = stretch(0, 1, 50, 0.0); s.ground.assign(s.ground.size(), kNaN);
    none.stretches = {s};
    CHECK_FALSE(network_design_floor(none, p).ok);
}

TEST_CASE("the floor does not depend on the order the stretches are given in", "[structure][floor]") {
    FloorNetwork net; net.node_count = 5;
    std::vector<double> hill(251); for (std::size_t i = 0; i < hill.size(); ++i) hill[i] = 108.0 + 0.02 * static_cast<double>(i);
    net.stretches = {stretch(0, 4, 180, 109.0), stretch_profile(1, 4, 250, hill), stretch(2, 3, 90, 106.0), stretch(3, 4, 120, 110.0)};
    net.portals = {{0, 100.0, false}, {1, 103.0, false}, {2, 99.0, true}};
    NetworkFloorParams p; p.max_grade = 0.05; p.needed = 5.7;
    const auto a = network_design_floor(net, p);
    FloorNetwork rev = net; std::reverse(rev.stretches.begin(), rev.stretches.end());
    const auto b = network_design_floor(rev, p);
    REQUIRE(a.ok); REQUIRE(b.ok); REQUIRE(a.converged); REQUIRE(b.converged);
    for (std::size_t e = 0; e < net.stretches.size(); ++e) {
        const auto& fa = a.floor[e]; const auto& fb = b.floor[net.stretches.size() - 1 - e];
        REQUIRE(fa.size() == fb.size());
        for (std::size_t k = 0; k < fa.size(); ++k) CHECK(fa[k] == fb[k]);
    }
    for (std::size_t n = 0; n < 5; ++n) CHECK(a.node_floor[n] == b.node_floor[n]);
    CHECK(a.anchor_misses == 0);
    CHECK(lipschitz_excess(a, net, 0.05) <= 1e-12);
}
