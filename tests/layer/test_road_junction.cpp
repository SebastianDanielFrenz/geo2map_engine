// Street fidelity P2 slice 3: single-node junction polygons (design sections 2 and 3, tests T2 and T3): approaches,
// edge polylines, corners, trims, rings, simplicity, bow-tie repair, hull fallback and declines, on synthetic X, T and
// Y junctions with hand-computed offset intersections and on the two committed OSM fixtures.
#include "road_fixture.h"
#include "g2m/core/geom/ring_set.h"
#include "g2m/layer/road_junction.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <map>
#include <memory>
#include <set>
#include <tuple>
#include <string>
using namespace g2m;
using Catch::Approx;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};
const TileKey kTile{zone, 2, 455, 5421};
constexpr double kOriginE = 455.0 * 1024.0 + 512.0, kOriginN = 5421.0 * 1024.0 + 512.0;   // tile centre, metres
constexpr double kPi = 3.14159265358979323846;
using Tags = std::vector<std::pair<std::string, std::string>>;

struct N { std::int64_t id; double x, y; };
struct W { std::int64_t id; std::vector<std::int64_t> nodes; Tags tags = {{"highway", "residential"}}; };

struct Scene {
    RoadGraph graph;
    RoadGeomTile tile;
    std::optional<RoadReferenceBuilder> builder;
};
// Metres relative to the tile centre -> a roads.geom tile built by the real deriver with its context.
std::unique_ptr<Scene> make_scene(const std::vector<N>& nodes, std::vector<W> ways) {
    auto scene = std::make_unique<Scene>();
    for (const auto& n : nodes) scene->graph.nodes.push_back({n.id, {std::llround((kOriginE + n.x) * 1000.0), std::llround((kOriginN + n.y) * 1000.0)}, {}});
    std::sort(scene->graph.nodes.begin(), scene->graph.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    std::sort(ways.begin(), ways.end(), [](const auto& a, const auto& b) { return a.id < b.id; });
    for (auto& w : ways) {
        Tags tags = w.tags; std::sort(tags.begin(), tags.end());
        scene->graph.ways.push_back({w.id, 3, w.nodes, tags});
        for (std::size_t i = 1; i < w.nodes.size(); ++i) scene->graph.edges.push_back({w.id, static_cast<std::uint32_t>(i - 1), w.nodes[i - 1], w.nodes[i]});
    }
    auto tile = g2m_test::geometry_in_tile(scene->graph, kTile);
    REQUIRE(tile);
    scene->tile = std::move(*tile);
    auto builder = RoadReferenceBuilder::make(scene->graph);
    REQUIRE(builder);
    scene->builder = std::move(*builder);
    return scene;
}
std::unique_ptr<Scene> x_scene(double arm = 300.0) {
    return make_scene({{1, 0, 0}, {2, -arm, 0}, {3, arm, 0}, {4, 0, -arm}, {5, 0, arm}}, {{10, {2, 1, 3}}, {11, {4, 1, 5}}});
}
std::unique_ptr<Scene> t_scene() {
    return make_scene({{1, 0, 0}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}}, {{10, {2, 1, 3}}, {11, {1, 4}}});
}
std::unique_ptr<Scene> y_scene(double degrees) {
    const double a = degrees * kPi / 180.0;
    return make_scene({{1, 0, 0}, {2, -300, 0}, {3, 300, 0}, {4, 300 * std::cos(a), 300 * std::sin(a)}}, {{10, {2, 1, 3}}, {11, {1, 4}}});
}
RoadJunction junction_of(const Scene& scene, std::int64_t node, const RoadRules& rules = builtin_road_rules_de(), JunctionBuildStats* stats = nullptr) {
    auto j = build_node_junction(scene.tile, *scene.builder, node, rules, stats);
    REQUIRE(j);
    return std::move(*j);
}

struct Arm { double ux = 0, uy = 0, ell = 0, rho = 0; };   // outward unit vector, outer left / right offsets (positive distances)
// The straight-arm oracle: direction from the far node, offsets from the entry's own footprint 10 m from the node.
Arm arm_of(const Scene& scene, const JunctionApproach& a, std::int64_t node_id) {
    const auto& e = scene.tile.entries[a.entry_index];
    const auto& way = *std::find_if(scene.graph.ways.begin(), scene.graph.ways.end(), [&](const auto& w) { return w.osm_id == e.way_id; });
    const std::int64_t far = way.node_ids[a.at_end ? e.stretch.start_ref : e.stretch.end_ref];
    const auto pos = [&](std::int64_t id) { return std::find_if(scene.graph.nodes.begin(), scene.graph.nodes.end(), [&](const auto& n) { return n.osm_id == id; })->position; };
    const double dx = static_cast<double>(pos(far).x - pos(node_id).x), dy = static_cast<double>(pos(far).y - pos(node_id).y);
    const double len = std::sqrt(dx * dx + dy * dy);
    const double L = e.profile->reference.length_m;
    const auto f = carve_footprint(*e.cross_section, a.at_end ? L - 10.0 : 10.0);
    Arm arm; arm.ux = dx / len; arm.uy = dy / len;
    arm.ell = a.at_end ? -f.outer_right : f.outer_left;
    arm.rho = a.at_end ? f.outer_left : -f.outer_right;
    return arm;
}
// Corner of arm i's left edge with arm j's right edge; empty when parallel or when it lies behind the node.
struct OracleCorner { bool found = false; double x = 0, y = 0, si = 0, sj = 0; };
OracleCorner oracle_corner(const Arm& i, const Arm& j) {
    // cross(u_i, p) = ell_i and cross(u_j, p) = -rho_j, with cross(u, p) = ux*py - uy*px.
    const double det = (-i.uy) * j.ux - i.ux * (-j.uy);   // | -uy_i  ux_i ; -uy_j ux_j |
    OracleCorner c;
    if (std::abs(det) < 1e-9) return c;
    const double b1 = i.ell, b2 = -j.rho;
    c.x = (b1 * j.ux - i.ux * b2) / det;
    c.y = ((-i.uy) * b2 - (-j.uy) * b1) / det;
    c.si = c.x * i.ux + c.y * i.uy; c.sj = c.x * j.ux + c.y * j.uy;
    c.found = c.si >= 0 && c.sj >= 0;
    return c;
}
double angle_of(const Arm& a) { double t = std::atan2(a.uy, a.ux); return t < 0 ? t + 2 * kPi : t; }

// Distance (mm, double, test only) from p to the segment a-b.
double distance_to_segment(geom::PointMm p, geom::PointMm a, geom::PointMm b) {
    const double px = static_cast<double>(p.x - a.x), py = static_cast<double>(p.y - a.y), dx = static_cast<double>(b.x - a.x), dy = static_cast<double>(b.y - a.y);
    const double len2 = dx * dx + dy * dy;
    const double t = len2 > 0 ? std::clamp((px * dx + py * dy) / len2, 0.0, 1.0) : 0.0;
    return std::hypot(px - t * dx, py - t * dy);
}
// Road bed inside paved inside outer. Vertices are rounded to mm independently per layer, so a vertex on an edge the
// layers share may lie up to ~1.5 mm outside the next ring: that counts as inside.
bool inside_or_near(const geom::RingSet& ring, const std::vector<geom::PointMm>& v, geom::PointMm p, double tolerance_mm = 2.5) {
    if (geom::contains(ring, p)) return true;
    for (std::size_t i = 0; i < v.size(); ++i) if (distance_to_segment(p, v[i], v[(i + 1) % v.size()]) <= tolerance_mm) return true;
    return false;
}
// Number of inner-ring vertices outside the next ring beyond the rounding tolerance; `worst_mm` receives the largest
// distance outside (0 when nested).
std::size_t nested_violations(const RoadJunction& j, double* worst_mm = nullptr) {
    std::size_t bad = 0;
    if (worst_mm) *worst_mm = 0;
    for (int inner = 0; inner < 2; ++inner) {
        geom::RingSet outer; outer.outers.push_back(j.rings[inner + 1].vertices); outer.bbox = geom::bounding_box(outer);
        for (const auto& p : j.rings[inner].vertices) {
            if (inside_or_near(outer, j.rings[inner + 1].vertices, p)) continue;
            ++bad;
            if (worst_mm) {
                double d = 1e18; const auto& v = j.rings[inner + 1].vertices;
                for (std::size_t i = 0; i < v.size(); ++i) d = std::min(d, distance_to_segment(p, v[i], v[(i + 1) % v.size()]));
                *worst_mm = std::max(*worst_mm, d);
            }
        }
    }
    return bad;
}
void check_nested(const RoadJunction& j) { CHECK(nested_violations(j) == 0); }
// The full oracle check of a single-node junction of straight arms: order, trims within 1.5 mm, corners are outer ring
// vertices, the rings valid and nested.
void check_straight_junction(const Scene& scene, const RoadJunction& j, std::int64_t node_id, double trim_min = 1.0, double margin = 0.5) {
    REQUIRE(j.decline == JunctionDecline::None);
    const std::size_t k = j.approaches.size();
    std::vector<Arm> arms;
    for (const auto& a : j.approaches) { REQUIRE(a.status == JunctionApproachStatus::Used); arms.push_back(arm_of(scene, a, node_id)); }
    // Counter-clockwise, starting at the smallest key: the angles rise by less than a full turn in total.
    double turned = 0;
    for (std::size_t i = 0; i < k; ++i) {
        double d = angle_of(arms[(i + 1) % k]) - angle_of(arms[i]); if (d <= 0) d += 2 * kPi;
        turned += d;
    }
    CHECK(turned == Approx(2 * kPi).margin(1e-9));
    const auto pos = std::find_if(scene.graph.nodes.begin(), scene.graph.nodes.end(), [&](const auto& n) { return n.osm_id == node_id; })->position;
    for (std::size_t i = 0; i < k; ++i) {
        const auto next = oracle_corner(arms[i], arms[(i + 1) % k]), prev = oracle_corner(arms[(i + k - 1) % k], arms[i]);
        double station = trim_min;
        if (next.found) station = std::max(station, next.si);
        if (prev.found) station = std::max(station, prev.sj);
        INFO("approach " << i << " way " << scene.tile.entries[j.approaches[i].entry_index].way_id);
        CHECK(j.approaches[i].trim_m == Approx(station + margin).margin(0.0015));
        if (next.found) {
            const geom::PointMm expect{pos.x + std::llround(next.x * 1000.0), pos.y + std::llround(next.y * 1000.0)};
            const bool present = std::any_of(j.rings[2].vertices.begin(), j.rings[2].vertices.end(),
                [&](geom::PointMm v) { return std::abs(v.x - expect.x) <= 2 && std::abs(v.y - expect.y) <= 2; });
            CHECK(present);
        }
    }
    for (const auto& ring : j.rings) CHECK(junction_ring_valid(ring));
    check_nested(j);
}
}

TEST_CASE("T2: X junction trims equal the hand-computed offset intersections", "[layer][junction]") {
    const auto scene = x_scene();
    JunctionBuildStats stats;
    const auto j = junction_of(*scene, 1, builtin_road_rules_de(), &stats);
    REQUIRE(j.approaches.size() == 4);
    CHECK(j.nodes == std::vector<std::int64_t>{1});
    CHECK(j.flags == 0);
    check_straight_junction(*scene, j, 1);
    CHECK(stats.declines[0] == 1); CHECK(stats.hull_fallbacks == 0); CHECK(stats.bowtie_rounds == 0);
    // The record starts at the smallest (way id, start_ref, at_end): way 10's first stretch end.
    const auto key = [&](const JunctionApproach& a) { const auto& e = scene->tile.entries[a.entry_index]; return std::make_tuple(e.way_id, e.stretch.start_ref, a.at_end); };
    for (const auto& a : j.approaches) CHECK(key(j.approaches[0]) <= key(a));
    // Perpendicular arms: the corner of (i, i+1) sits at (rho_{i+1}, ell_i) in the frame of arm i, so the outer ring is
    // the cross-shaped polygon with 4 corner vertices and 8 trim vertices (collinear points removed).
    CHECK(j.rings[2].vertices.size() == 12);
}

TEST_CASE("T2: T junction, the straight pair is open and carries no bevel", "[layer][junction]") {
    const auto scene = t_scene();
    const auto j = junction_of(*scene, 1);
    REQUIRE(j.approaches.size() == 3);
    check_straight_junction(*scene, j, 1);
    // The main road runs west-east; its north edge (the open pair) is a straight line, so no outer vertex lies
    // strictly north of the node unless it is on one of the two arms' own north edges (equal offsets there).
    const auto node = std::find_if(scene->graph.nodes.begin(), scene->graph.nodes.end(), [](const auto& n) { return n.osm_id == 1; })->position;
    std::set<std::int64_t> north;
    for (const auto& v : j.rings[2].vertices) if (v.y > node.y + 2) north.insert(v.y - node.y);
    // Two trim points (one per main-road arm), both on the north edge, with the same offset to the mm.
    REQUIRE(north.size() >= 1);
    const auto lo = *north.begin(), hi = *north.rbegin();
    CHECK(hi - lo <= 2);
    // 3 trim pairs and 2 corners; the open pair adds no vertex.
    CHECK(j.rings[2].vertices.size() == 8);
}

TEST_CASE("T2: Y junctions at 30 and 10 degrees", "[layer][junction]") {
    for (const double degrees : {30.0, 10.0}) {
        INFO("angle " << degrees);
        const auto scene = y_scene(degrees);
        JunctionBuildStats stats;
        const auto j = junction_of(*scene, 1, builtin_road_rules_de(), &stats);
        REQUIRE(j.approaches.size() == 3);
        check_straight_junction(*scene, j, 1);
        // A shallow merge trims far (w / sin(theta)): the long thin ring stays legal under the 128 m reach.
        double longest = 0; for (const auto& a : j.approaches) longest = std::max(longest, a.trim_m);
        CHECK(longest > (degrees == 10.0 ? 30.0 : 10.0));
        CHECK(longest < 128.0);
        std::printf("Y %.0f deg: trims %.3f %.3f %.3f, outer ring %zu vertices, repairs %zu, hull %zu\n", degrees, j.approaches[0].trim_m,
            j.approaches[1].trim_m, j.approaches[2].trim_m, j.rings[2].vertices.size(), stats.bowtie_rounds, stats.hull_fallbacks);
    }
}

TEST_CASE("T2: a curved approach agrees with a 5 times finer edge sampling", "[layer][junction]") {
    // Way 11 leaves the node northwards and curves east on a 100 m radius arc; way 10 crosses west-east.
    std::vector<N> nodes = {{1, 0, 0}, {2, -300, 0}, {3, 300, 0}};
    std::vector<std::int64_t> arc = {1};
    for (int i = 1; i <= 9; ++i) {
        const double phi = i * 10.0 * kPi / 180.0;
        nodes.push_back({100 + i, 100.0 - 100.0 * std::cos(phi), 100.0 * std::sin(phi)});
        arc.push_back(100 + i);
    }
    const auto scene = make_scene(nodes, {{10, {2, 1, 3}}, {11, arc}});
    const auto coarse = junction_of(*scene, 1);
    auto fine_rules = builtin_road_rules_de(); fine_rules.junction.edge_station_spacing_mm = 100;
    const auto fine = junction_of(*scene, 1, fine_rules);
    REQUIRE(coarse.decline == JunctionDecline::None); REQUIRE(fine.decline == JunctionDecline::None);
    REQUIRE(coarse.approaches.size() == 3); REQUIRE(fine.approaches.size() == 3);
    for (std::size_t i = 0; i < 3; ++i) {
        INFO("approach " << i);
        CHECK(coarse.approaches[i].entry_index == fine.approaches[i].entry_index);
        CHECK(coarse.approaches[i].trim_m == Approx(fine.approaches[i].trim_m).margin(0.006));   // chord sagitta 3.1 mm at 10 m
    }
    for (const auto& ring : coarse.rings) CHECK(junction_ring_valid(ring));
    for (const auto& ring : fine.rings) CHECK(junction_ring_valid(ring));
}

TEST_CASE("T3: rings are counter-clockwise, simple and nested; the validity predicate catches broken rings", "[layer][junction]") {
    const auto scene = x_scene();
    const auto j = junction_of(*scene, 1);
    for (const auto& ring : j.rings) CHECK(junction_ring_valid(ring));
    // Clockwise: rejected.
    auto cw = j.rings[2]; std::reverse(cw.vertices.begin(), cw.vertices.end());
    CHECK_FALSE(junction_ring_valid(cw));
    // A bow-tie (swap two vertices): rejected. A repeated vertex: rejected. Too short: rejected.
    JunctionRing bow; bow.vertices = {{0, 0}, {10000, 10000}, {10000, 0}, {0, 10000}};
    CHECK_FALSE(junction_ring_valid(bow));
    auto repeated = j.rings[2]; repeated.vertices.push_back(repeated.vertices[3]);
    CHECK_FALSE(junction_ring_valid(repeated));
    JunctionRing two; two.vertices = {j.rings[2].vertices[0], j.rings[2].vertices[1]};
    CHECK_FALSE(junction_ring_valid(two));
    // A spike (a vertex folding back along the previous edge): rejected.
    JunctionRing spike; spike.vertices = {{0, 0}, {10000, 0}, {5000, 0}, {5000, 8000}};
    CHECK_FALSE(junction_ring_valid(spike));
    JunctionRing square; square.vertices = {{0, 0}, {10000, 0}, {10000, 10000}, {0, 10000}};
    CHECK(junction_ring_valid(square));
}

namespace {
// A junction whose offset ring is not simple: the diag scan below finds one; this fixture is a tight fork with a
// strongly curving branch.
std::unique_ptr<Scene> fork_scene(double degrees, double radius, double turn = 1.0) {
    const double a = degrees * kPi / 180.0;
    std::vector<N> nodes = {{1, 0, 0}, {2, -300, 0}, {3, 300, 0}};
    std::vector<std::int64_t> branch = {1};
    // The branch leaves at `degrees` and curves back towards the main road (a loop-like turn of radius `radius`).
    for (int i = 1; i <= 12; ++i) {
        const double phi = i * 12.0 * kPi / 180.0;
        const double cx = -turn * std::sin(a) * radius, cy = turn * std::cos(a) * radius;   // centre of curvature (left of the branch for turn = 1)
        nodes.push_back({100 + i, cx + turn * radius * std::sin(a + turn * phi), cy - turn * radius * std::cos(a + turn * phi)});
        branch.push_back(100 + i);
    }
    return make_scene(nodes, {{10, {2, 1, 3}}, {11, branch}});
}
}

namespace {
// Straight arms of 300 m from the node at the given angles (degrees), each its own way ending at the node.
std::unique_ptr<Scene> star_scene(const std::vector<double>& degrees, const Tags& tags = {{"highway", "residential"}}) {
    std::vector<N> nodes = {{1, 0, 0}};
    std::vector<W> ways;
    for (std::size_t i = 0; i < degrees.size(); ++i) {
        const double a = degrees[i] * kPi / 180.0;
        nodes.push_back({static_cast<std::int64_t>(10 + i), 300 * std::cos(a), 300 * std::sin(a)});
        ways.push_back({static_cast<std::int64_t>(100 + i), {1, static_cast<std::int64_t>(10 + i)}, tags});
    }
    return make_scene(nodes, ways);
}
bool has(const RoadJunction& j, JunctionFlags f) { return (j.flags & static_cast<std::uint16_t>(f)) != 0; }
}

TEST_CASE("T3: a repairable bow-tie is repaired by trim extension; without the repair it falls back to the hull", "[layer][junction][roads]") {
    // Hoechst A 66 node 527505 (three arms at -17, -55 and 125 degrees): the first ring is not simple, one extension
    // round of 1 m on the involved approaches makes every layer simple (measured in the census).
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/hoechst_a66.osm.pbf");
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const auto builder = RoadReferenceBuilder::make(*world); REQUIRE(builder);
    auto g = g2m_test::geometry_in_tile(*world, TileKey{g2m_test::kZone32N, 2, 455, 5421}); REQUIRE(g);
    JunctionBuildStats stats;
    const auto repaired = build_node_junction(*g, *builder, 527505, builtin_road_rules_de(), &stats); REQUIRE(repaired);
    CHECK(repaired->decline == JunctionDecline::None);
    CHECK_FALSE(has(*repaired, JunctionFlags::HullFallback));
    CHECK(stats.bowtie_rounds == 1); CHECK(stats.bowtie_repaired == 1);
    for (const auto& ring : repaired->rings) CHECK(junction_ring_valid(ring));
    // Sabotage in the test: with the repair disabled (0 rounds) the same junction takes the hull for its non-simple layer.
    auto no_repair = builtin_road_rules_de(); no_repair.junction.bowtie_rounds = 0;
    JunctionBuildStats stats0;
    const auto hull = build_node_junction(*g, *builder, 527505, no_repair, &stats0); REQUIRE(hull);
    CHECK(has(*hull, JunctionFlags::HullFallback));
    CHECK(stats0.bowtie_rounds == 0); CHECK(stats0.hull_fallbacks == 1);
    for (const auto& ring : hull->rings) CHECK(junction_ring_valid(ring));
    CHECK_FALSE(*hull == *repaired);
    // The extension is exactly what changed: the repaired trims are the unrepaired ones plus whole metres.
    for (std::size_t i = 0; i < repaired->approaches.size(); ++i) {
        const double extra = repaired->approaches[i].trim_m - hull->approaches[i].trim_m;
        CHECK((std::abs(extra) < 1e-9 || std::abs(extra - 1.0) < 1e-9));
    }
}

TEST_CASE("T3: constructed self-crossing junctions fall back to the convex hull with the flag", "[layer][junction]") {
    // Two motorway carriageways leaving 3 degrees apart: their facing edges have not met within the 128 m reach, the
    // open-pair ring crosses itself at the node and no extension of 3 m helps.
    const auto scene = star_scene({0.0, 3.0, 180.0}, {{"highway", "motorway"}, {"oneway", "yes"}});
    JunctionBuildStats stats;
    const auto j = junction_of(*scene, 1, builtin_road_rules_de(), &stats);
    CHECK(j.decline == JunctionDecline::None);
    CHECK(has(j, JunctionFlags::HullFallback));
    CHECK(stats.hull_fallbacks == 1); CHECK(stats.bowtie_rounds == static_cast<std::size_t>(builtin_road_rules_de().junction.bowtie_rounds));
    for (const auto& ring : j.rings) { CHECK(junction_ring_valid(ring)); CHECK(ring.vertices.size() >= 3); }
    // A branch that curves back across the main road, the same: hull, flag, valid rings.
    const auto fork = fork_scene(10.0, 40.0, -1.0);
    const auto jf = junction_of(*fork, 1);
    CHECK(has(jf, JunctionFlags::HullFallback));
    for (const auto& ring : jf.rings) CHECK(junction_ring_valid(ring));
    // A convex hull is convex: every turn is a left turn.
    const auto& hull = j.rings[2].vertices;
    for (std::size_t i = 0; i < hull.size(); ++i)
        CHECK(geom::orient(hull[i], hull[(i + 1) % hull.size()], hull[(i + 2) % hull.size()]) > 0);
}

TEST_CASE("Junction declines, excluded approaches and flags", "[layer][junction]") {
    const auto base = x_scene();
    // One approach without a profile: three used approaches remain, the junction stays (Partial).
    {
        auto scene = x_scene();
        auto j0 = junction_of(*scene, 1);
        scene->tile.entries[j0.approaches[0].entry_index].profile.reset();
        const auto j = junction_of(*scene, 1);
        CHECK(j.decline == JunctionDecline::None);
        CHECK(has(j, JunctionFlags::Partial));
        REQUIRE(j.approaches.size() == 4);
        CHECK(j.approaches.back().status == JunctionApproachStatus::NoProfile);
        CHECK(j.approaches.back().trim_m == 0.0);
        for (std::size_t i = 0; i < 3; ++i) CHECK(j.approaches[i].status == JunctionApproachStatus::Used);
        for (const auto& ring : j.rings) CHECK(junction_ring_valid(ring));
    }
    // A used approach whose cross-section declined: NoCrossSection, excluded.
    {
        auto scene = x_scene();
        auto j0 = junction_of(*scene, 1);
        const auto victim = j0.approaches[1].entry_index;
        scene->tile.entries[victim].cross_section.reset();
        scene->tile.entries[victim].cross_section_decline = CrossSectionDecline{};
        const auto j = junction_of(*scene, 1);
        REQUIRE(j.approaches.size() == 4);
        CHECK(j.approaches.back().status == JunctionApproachStatus::NoCrossSection);
        CHECK(has(j, JunctionFlags::Partial));
    }
    // One used approach: SingleApproach, record kept with its approaches, no rings, no trims.
    {
        auto scene = t_scene();
        auto j0 = junction_of(*scene, 1);
        scene->tile.entries[j0.approaches[0].entry_index].profile.reset();
        scene->tile.entries[j0.approaches[1].entry_index].cross_section.reset();
        JunctionBuildStats stats;
        const auto j = junction_of(*scene, 1, builtin_road_rules_de(), &stats);
        CHECK(j.decline == JunctionDecline::SingleApproach);
        REQUIRE(j.approaches.size() == 3);
        for (const auto& ring : j.rings) CHECK(ring.vertices.empty());
        for (const auto& a : j.approaches) CHECK(a.trim_m == 0.0);
        CHECK(stats.declines[static_cast<std::size_t>(JunctionDecline::SingleApproach)] == 1);
    }
    // Every approach a tunnel: Underground, each approach with the Tunnel status.
    {
        const auto scene = star_scene({0.0, 120.0, 240.0}, {{"highway", "residential"}, {"tunnel", "yes"}, {"layer", "-1"}});
        const auto j = junction_of(*scene, 1);
        CHECK(j.decline == JunctionDecline::Underground);
        REQUIRE(j.approaches.size() == 3);
        for (const auto& a : j.approaches) CHECK(a.status == JunctionApproachStatus::Tunnel);
    }
    // Reach: a ring vertex beyond reach_max (white-box: the rule lowered to 3 m).
    {
        auto rules = builtin_road_rules_de(); rules.junction.reach_max_m = 3;
        const auto j = junction_of(*base, 1, rules);
        CHECK(j.decline == JunctionDecline::ReachExceeded);
        for (const auto& ring : j.rings) CHECK(ring.vertices.empty());
        for (const auto& a : j.approaches) CHECK(a.trim_m == 0.0);
    }
    // Degenerate: an approach shorter than the polyline margin cannot be sampled (white-box: a reference shortened to 0.3 m).
    {
        auto scene = x_scene();
        auto j0 = junction_of(*scene, 1);
        scene->tile.entries[j0.approaches[0].entry_index].profile->reference.length_m = 0.3;
        const auto j = junction_of(*scene, 1);
        CHECK(j.decline == JunctionDecline::Degenerate);
    }
    // Bridges: P1 declines two or more bridges meeting at a node, so a bridge star has no used approach (NoProfile) and
    // is SingleApproach; the OnStructure flag itself is exercised white-box by marking every profile as a bridge.
    {
        const auto scene = star_scene({0.0, 120.0, 240.0}, {{"highway", "residential"}, {"bridge", "yes"}, {"layer", "1"}});
        const auto j = junction_of(*scene, 1);
        CHECK(j.decline == JunctionDecline::SingleApproach);
        for (const auto& a : j.approaches) CHECK(a.status == JunctionApproachStatus::NoProfile);
    }
    {
        auto scene = x_scene();
        for (auto& e : scene->tile.entries) e.profile->attributes.bridge = true;
        const auto j = junction_of(*scene, 1);
        CHECK(j.decline == JunctionDecline::None);
        CHECK(has(j, JunctionFlags::OnStructure));
        CHECK_FALSE(has(junction_of(*x_scene(), 1), JunctionFlags::OnStructure));
    }
}

TEST_CASE("build_node_junctions: degrees from the context, incomplete nodes skipped, deterministic", "[layer][junction]") {
    const auto scene = x_scene();
    JunctionBuildStats stats;
    const auto all = build_node_junctions(scene->tile, *scene->builder, builtin_road_rules_de(), &stats);
    REQUIRE(all);
    REQUIRE(all->size() == 1);
    CHECK(all->front().nodes == std::vector<std::int64_t>{1});
    CHECK(stats.junction_nodes == 1); CHECK(stats.skipped_incomplete == 0); CHECK(stats.built == 1);
    CHECK(stats.approaches_used == 4); CHECK(stats.declines[0] == 1);
    // Bit-identical on a second build and equal to the single-node call.
    const auto again = build_node_junctions(scene->tile, *scene->builder); REQUIRE(again);
    CHECK(*all == *again);
    CHECK(all->front() == junction_of(*scene, 1));
    // A node of degree 2 or 1 is not a junction.
    CHECK_FALSE(build_node_junction(scene->tile, *scene->builder, 2));
    CHECK_FALSE(build_node_junction(scene->tile, *scene->builder, 999));
    // A tile that lacks one approach of the node (another tile owns it) is skipped, not built partially.
    auto partial = x_scene();
    partial->tile.entries.pop_back();
    JunctionBuildStats stats2;
    const auto none = build_node_junctions(partial->tile, *partial->builder, builtin_road_rules_de(), &stats2);
    REQUIRE(none);
    CHECK(none->empty()); CHECK(stats2.skipped_incomplete == 1); CHECK(stats2.junction_nodes == 1);
    CHECK_FALSE(build_node_junction(partial->tile, *partial->builder, 1));
    // A node whose degree in the context is below 3 is no junction.
    const auto small = make_scene({{1, 0, 0}, {2, -300, 0}, {3, 300, 0}}, {{10, {2, 1, 3}}});
    CHECK(build_node_junctions(small->tile, *small->builder)->empty());
}


namespace {
struct Census {
    JunctionBuildStats stats;
    double nested_worst_mm = 0; std::size_t nested_junctions = 0;
    std::size_t seen_nodes = 0, skipped = 0;
    std::size_t tiles = 0, owned_nodes = 0, nested_bad = 0, invalid_rings = 0, multi_tile_nodes = 0, multi_tile_mismatch = 0;
    std::map<int, std::size_t> by_degree;
    std::size_t max_outer = 0;
    double max_trim = 0;
};
// Every L2 tile of the fixture that owns edges, its junctions built with the world graph's degrees; a junction is
// COUNTED in the tile that contains its node, and every build of the same node from another tile must be identical.
Census census_of(const std::string& pbf, const std::vector<TileKey>& only = {}) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const auto builder = RoadReferenceBuilder::make(*world); REQUIRE(builder);
    std::set<TileKey> keys;
    for (const auto& n : world->nodes) {
        const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0);
        if (k) keys.insert(*k);
    }
    Census c;
    struct Rec { RoadJunction j; const RoadGeomTile* tile = nullptr; };
    std::map<std::int64_t, std::vector<RoadJunction>> by_node;
    std::vector<std::unique_ptr<RoadGeomTile>> tiles;
    for (const auto& key : keys) {
        if (!only.empty() && std::find(only.begin(), only.end(), key) == only.end()) continue;
        const auto probe = select_road_graph_tile(*world, key);
        if (!probe || probe->owned_edges.empty()) continue;
        auto g = g2m_test::geometry_in_tile(*world, key); REQUIRE(g);
        ++c.tiles;
        JunctionBuildStats stats;
        auto js = build_node_junctions(*g, *builder, builtin_road_rules_de(), &stats); REQUIRE(js);
        c.seen_nodes += stats.junction_nodes; c.skipped += stats.skipped_incomplete;
        for (auto& j : *js) {
            const auto node = std::find_if(world->nodes.begin(), world->nodes.end(), [&](const auto& n) { return n.osm_id == j.nodes.front(); });
            const auto home = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(node->position.x) / 1000.0, static_cast<double>(node->position.y) / 1000.0);
            // Normalise the entry indices (they differ per tile) to the approaches' (way, start_ref) identity for the comparison.
            for (auto& a : j.approaches) { const auto& e = g->entries[a.entry_index]; a.entry_index = static_cast<std::size_t>(e.way_id * 1000 + e.stretch.start_ref); }
            by_node[j.nodes.front()].push_back(j);
            if (*home != key) continue;
            ++c.owned_nodes;
            // The census counters come from the owner tile's own build of the node (one count per node).
            { const auto again = build_node_junction(*g, *builder, j.nodes.front(), builtin_road_rules_de(), &c.stats); REQUIRE(again); }
            ++c.by_degree[static_cast<int>(j.approaches.size())];
            if (j.decline == JunctionDecline::None) {
                c.max_outer = std::max(c.max_outer, j.rings[2].vertices.size());
                for (const auto& ring : j.rings) if (!junction_ring_valid(ring)) ++c.invalid_rings;
                double worst = 0; const auto bad = nested_violations(j, &worst);
                c.nested_bad += bad; if (bad) ++c.nested_junctions; c.nested_worst_mm = std::max(c.nested_worst_mm, worst);
                for (const auto& a : j.approaches) c.max_trim = std::max(c.max_trim, a.trim_m);
            }
        }
        tiles.push_back(std::make_unique<RoadGeomTile>(std::move(*g)));
    }
    for (const auto& [node, list] : by_node) {
        if (list.size() < 2) continue;
        ++c.multi_tile_nodes;
        for (std::size_t i = 1; i < list.size(); ++i) if (!(list[i] == list[0])) { ++c.multi_tile_mismatch; break; }
    }
    return c;
}
void print_census(const char* name, const Census& c) {
    std::printf("[junction census] %s: tiles %zu, junctions (degree >= 3, counted once in the tile of the node) %zu; node builds over all tiles %zu, skipped as incomplete %zu\n", name, c.tiles,
        c.owned_nodes, c.seen_nodes, c.skipped);
    std::printf("  declines: none %zu, cluster %zu, reach %zu, single approach %zu, degenerate %zu, underground %zu\n", c.stats.declines[0], c.stats.declines[1],
        c.stats.declines[2], c.stats.declines[3], c.stats.declines[4], c.stats.declines[5]);
    std::printf("  partial %zu, on structure %zu, hull fallbacks %zu, bow-tie repaired %zu (extension rounds spent incl. failed ones %zu), excluded approaches %zu\n", c.stats.partial, c.stats.on_structure,
        c.stats.hull_fallbacks, c.stats.bowtie_repaired, c.stats.bowtie_rounds, c.stats.excluded_approaches);
    std::printf("  approaches per junction:"); for (const auto& [d, n] : c.by_degree) std::printf(" %d:%zu", d, n);
    std::printf("\n  largest outer ring %zu vertices, largest trim %.2f m, invalid rings %zu, nesting violations %zu in %zu junctions (worst %.0f mm), nodes built in >= 2 tiles %zu (mismatching %zu)\n",
        c.max_outer, c.max_trim, c.invalid_rings, c.nested_bad, c.nested_junctions, c.nested_worst_mm, c.multi_tile_nodes, c.multi_tile_mismatch);
}
}

TEST_CASE("Fixtures: every junction ring is valid, the same node builds identically in every tile (nesting is measured, not asserted)", "[layer][junction][roads]") {
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        INFO(pbf);
        const auto c = census_of(pbf);
        print_census(pbf, c);
        CHECK(c.owned_nodes > 50);
        CHECK(c.invalid_rings == 0);
        CHECK(c.multi_tile_mismatch == 0);
    }
}
