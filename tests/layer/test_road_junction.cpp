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
// The offset-intersect-trim tests (T2, the bow-tie repair) check the SHARP corners of design 3.3-3.5; the corner
// fillets (kerb returns) on top of them have their own tests in test_junction_fillets.cpp.
JunctionBuildOptions sharp_corners() { JunctionBuildOptions o; o.corner_fillets = false; return o; }
RoadJunction junction_of(const Scene& scene, std::int64_t node, const RoadRules& rules = builtin_road_rules_de(), JunctionBuildStats* stats = nullptr,
    const JunctionBuildOptions& options = sharp_corners()) {
    auto j = build_node_junction(scene.tile, *scene.builder, node, rules, stats, options);
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
    // Hoechst A 66 node 2152308547: the first ring is not simple, one extension round of 1 m on the involved
    // approaches makes every layer simple (measured in the census). Slice 3 used node 527505 (arms at -17, -55 and
    // 125 degrees); its bow-tie was the open-pair chord of finding (a), which the corner caps (D17) now close, so it
    // builds without any extension.
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/hoechst_a66.osm.pbf");
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const auto builder = RoadReferenceBuilder::make(*world); REQUIRE(builder);
    {
        auto g0 = g2m_test::geometry_in_tile(*world, TileKey{g2m_test::kZone32N, 2, 455, 5421}); REQUIRE(g0);
        JunctionBuildStats s0;
        const auto capped = build_node_junction(*g0, *builder, 527505, builtin_road_rules_de(), &s0); REQUIRE(capped);
        CHECK(capped->decline == JunctionDecline::None);
        CHECK(s0.bowtie_rounds == 0); CHECK(s0.hull_fallbacks == 0);
        JunctionBuildOptions no_caps; no_caps.corner_caps = false;
        JunctionBuildStats s1;
        REQUIRE(build_node_junction(*g0, *builder, 527505, builtin_road_rules_de(), &s1, no_caps));
        CHECK(s1.bowtie_rounds == 1);   // the slice-3 behaviour
    }
    constexpr std::int64_t kNode = 2152308547;
    const auto node = std::find_if(world->nodes.begin(), world->nodes.end(), [](const auto& n) { return n.osm_id == kNode; });
    REQUIRE(node != world->nodes.end());
    const auto home = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(node->position.x) / 1000.0, static_cast<double>(node->position.y) / 1000.0);
    REQUIRE(home);
    auto g = g2m_test::geometry_in_tile(*world, *home); REQUIRE(g);
    JunctionBuildStats stats;
    const auto repaired = build_node_junction(*g, *builder, kNode, builtin_road_rules_de(), &stats, sharp_corners()); REQUIRE(repaired);
    CHECK(repaired->decline == JunctionDecline::None);
    CHECK_FALSE(has(*repaired, JunctionFlags::HullFallback));
    CHECK(stats.bowtie_rounds == 1); CHECK(stats.bowtie_repaired == 1);
    for (const auto& ring : repaired->rings) CHECK(junction_ring_valid(ring));
    // Sabotage in the test: with the repair disabled (0 rounds) the same junction takes the hull for its non-simple layer.
    auto no_repair = builtin_road_rules_de(); no_repair.junction.bowtie_rounds = 0;
    JunctionBuildStats stats0;
    const auto hull = build_node_junction(*g, *builder, kNode, no_repair, &stats0, sharp_corners()); REQUIRE(hull);
    CHECK(has(*hull, JunctionFlags::HullFallback));
    CHECK(stats0.bowtie_rounds == 0); CHECK(stats0.hull_fallbacks == 1);
    for (const auto& ring : hull->rings) CHECK(junction_ring_valid(ring));
    CHECK_FALSE(*hull == *repaired);
    // With the corner fillets the kerb returns push these trims out far enough that the first ring is already simple.
    JunctionBuildStats filleted;
    const auto rounded = build_node_junction(*g, *builder, kNode, builtin_road_rules_de(), &filleted); REQUIRE(rounded);
    CHECK(rounded->decline == JunctionDecline::None); CHECK_FALSE(has(*rounded, JunctionFlags::HullFallback));
    CHECK(filleted.fillets > 0); CHECK(filleted.bowtie_rounds == 0);
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
    CHECK(all->front() == junction_of(*scene, 1, builtin_road_rules_de(), nullptr, JunctionBuildOptions{}));
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

TEST_CASE("Fixtures: every junction ring is valid and nested, the same node builds identically in every tile", "[layer][junction][roads]") {
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        INFO(pbf);
        const auto c = census_of(pbf);
        print_census(pbf, c);
        CHECK(c.owned_nodes > 50);
        CHECK(c.invalid_rings == 0);
        CHECK(c.nested_bad == 0);
        CHECK(c.multi_tile_mismatch == 0);
    }
}

// ---- slice 4: corner caps (D17), clusters (3.2, D16-D20) ----

namespace {
geom::PointMm at(double x, double y) { return {std::llround((kOriginE + x) * 1000.0), std::llround((kOriginN + y) * 1000.0)}; }
bool inside(const JunctionRing& ring, geom::PointMm p) {
    geom::RingSet set; set.outers.push_back(ring.vertices); set.bbox = geom::bounding_box(set);
    return geom::contains(set, p);
}
std::vector<RoadJunction> road_junctions(const Scene& scene, const RoadRules& rules = builtin_road_rules_de(), JunctionBuildStats* stats = nullptr,
    const JunctionBuildOptions& options = {}) {
    auto js = build_road_junctions(scene.tile, *scene.builder, rules, stats, options);
    REQUIRE(js);
    return std::move(*js);
}
const RoadJunction* record_of(const std::vector<RoadJunction>& js, std::int64_t owner, bool cluster_too_large = false) {
    for (const auto& j : js) if (j.nodes.front() == owner && (j.decline == JunctionDecline::ClusterTooLarge) == cluster_too_large) return &j;
    return nullptr;
}
void check_cluster_rings(const RoadJunction& j) {
    for (const auto& ring : j.rings) CHECK(junction_ring_valid(ring));
    check_nested(j);
}
// Two one-way carriageways per road, crossing: nodes 1 (-d,-d), 2 (d,-d), 3 (d,d), 4 (-d,d).
std::unique_ptr<Scene> dual_crossing_scene(double d) {
    const Tags t = {{"highway", "primary"}, {"oneway", "yes"}};
    return make_scene({{1, -d, -d}, {2, d, -d}, {3, d, d}, {4, -d, d}, {101, -300, -d}, {102, 300, -d}, {103, 300, d}, {104, -300, d},
                          {105, d, -300}, {106, d, 300}, {107, -d, 300}, {108, -d, -300}},
        {{20, {101, 1, 2, 102}, t}, {21, {103, 3, 4, 104}, t}, {22, {105, 2, 3, 106}, t}, {23, {107, 4, 1, 108}, t}});
}
// A main road west-east with a side road south at node 1 (0,0) and north at node 2 (s,0). The middle stretch
// optionally its own way with extra tags.
std::unique_ptr<Scene> staggered_scene(double s, const Tags& middle = {}) {
    std::vector<N> nodes = {{1, 0, 0}, {2, s, 0}, {3, -300, 0}, {4, 300, 0}, {5, 0, -300}, {6, s, 300}};
    if (middle.empty()) return make_scene(nodes, {{10, {3, 1, 2, 4}}, {11, {1, 5}}, {12, {2, 6}}});
    return make_scene(nodes, {{10, {3, 1}}, {11, {1, 5}}, {12, {2, 6}}, {13, {1, 2}, middle}, {14, {2, 4}}});
}
// A main road west-east with nodes 1..n every `spacing` metres from x = 0, each crossed by its own diagonal road at
// 30 degrees: both main-road arms of every node pair acutely with a diagonal arm and trim far.
std::unique_ptr<Scene> chain_scene(int n, double spacing) {
    std::vector<N> nodes = {{900, -300, 0}, {901, 300 + spacing * n, 0}};
    std::vector<W> ways;
    std::vector<std::int64_t> main = {900};
    for (int i = 1; i <= n; ++i) {
        const double x = spacing * (i - 1);
        nodes.push_back({i, x, 0});
        const double c = 200.0 * std::cos(kPi / 6.0), s = 200.0 * std::sin(kPi / 6.0);
        nodes.push_back({100 + i, x - c, -s}); nodes.push_back({200 + i, x + c, s});
        ways.push_back({100 + i, {100 + i, i, 200 + i}});
        main.push_back(i);
    }
    main.push_back(901);
    ways.push_back({10, main});
    return make_scene(nodes, ways);
}
}

TEST_CASE("D17: corner caps close an obtuse pair whose edges meet behind the node; the rings nest", "[layer][junction][roads]") {
    // Hoechst A 66 node 507567: an obtuse pair (~137 degrees) with unequal outer offsets. Without the caps the pair
    // is open, its chord runs on the wrong side of the node and the inner rings stick out of the outer one (slice-3
    // finding (a), up to 876 mm); with the caps the pair closes on the cap of the narrower arm.
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/hoechst_a66.osm.pbf");
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const auto builder = RoadReferenceBuilder::make(*world); REQUIRE(builder);
    const auto home_of = [&](std::int64_t id) {
        const auto n = std::find_if(world->nodes.begin(), world->nodes.end(), [&](const auto& x) { return x.osm_id == id; });
        REQUIRE(n != world->nodes.end());
        const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n->position.x) / 1000.0, static_cast<double>(n->position.y) / 1000.0);
        REQUIRE(k);
        return *k;
    };
    {
        auto g = g2m_test::geometry_in_tile(*world, home_of(507567)); REQUIRE(g);
        const auto capped = build_node_junction(*g, *builder, 507567); REQUIRE(capped);
        REQUIRE(capped->decline == JunctionDecline::None);
        CHECK_FALSE(has(*capped, JunctionFlags::HullFallback));
        for (const auto& ring : capped->rings) CHECK(junction_ring_valid(ring));
        CHECK(nested_violations(*capped) == 0);
        // Sabotage in the test: without the caps the same junction does not nest (D34's hull rescue off, it would
        // otherwise send the junction to the hull, which nests).
        JunctionBuildOptions no_caps; no_caps.corner_caps = false; no_caps.hull_when_not_nested = false;
        const auto open = build_node_junction(*g, *builder, 507567, builtin_road_rules_de(), nullptr, no_caps); REQUIRE(open);
        double worst = 0;
        CHECK(nested_violations(*open, &worst) > 0);
        CHECK(worst > 100.0);
        JunctionBuildOptions rescued_options; rescued_options.corner_caps = false;
        const auto rescued = build_node_junction(*g, *builder, 507567, builtin_road_rules_de(), nullptr, rescued_options); REQUIRE(rescued);
        CHECK(has(*rescued, JunctionFlags::HullFallback));
        CHECK(nested_violations(*rescued) == 0);
        CHECK(junction_rings_nested(*rescued));
        CHECK_FALSE(junction_rings_nested(*open));
    }
    // One non-simple layer sends all three to the hull (D17): node 562041 falls back on one layer; the hulls of the
    // three layers' trim points nest, an offset ring next to a hull does not (the sabotage).
    {
        auto g = g2m_test::geometry_in_tile(*world, home_of(562041)); REQUIRE(g);
        const auto all = build_node_junction(*g, *builder, 562041); REQUIRE(all);
        REQUIRE(has(*all, JunctionFlags::HullFallback));
        CHECK(nested_violations(*all) == 0);
        JunctionBuildOptions one; one.hull_all_layers = false;
        const auto mixed = build_node_junction(*g, *builder, 562041, builtin_road_rules_de(), nullptr, one); REQUIRE(mixed);
        CHECK(nested_violations(*mixed) > 0);
    }
}

TEST_CASE("T4: a dual-carriageway crossing is one cluster of four nodes around the median", "[layer][junction]") {
    const auto scene = dual_crossing_scene(5.25);
    JunctionBuildStats stats;
    const auto js = road_junctions(*scene, builtin_road_rules_de(), &stats);
    REQUIRE(js.size() == 1);
    const RoadJunction& j = js.front();
    CHECK(j.nodes == std::vector<std::int64_t>{1, 2, 3, 4});
    CHECK(j.decline == JunctionDecline::None);
    CHECK(has(j, JunctionFlags::Cluster));
    CHECK_FALSE(has(j, JunctionFlags::HullFallback));
    CHECK(j.absorbed.size() == 4);
    CHECK(std::is_sorted(j.absorbed.begin(), j.absorbed.end()));
    REQUIRE(j.approaches.size() == 8);
    for (const auto& a : j.approaches) { CHECK(a.status == JunctionApproachStatus::Used); CHECK(a.trim_m > 0.0); }
    check_cluster_rings(j);
    // The median and every node lie inside the road bed.
    CHECK(inside(j.rings[0], at(0, 0)));
    for (const auto& [x, y] : std::vector<std::pair<double, double>>{{-5.25, -5.25}, {5.25, -5.25}, {5.25, 5.25}, {-5.25, 5.25}}) CHECK(inside(j.rings[0], at(x, y)));
    // Stubs counter-clockwise around the cluster: the outward directions turn by exactly one full turn.
    const auto pos = [&](std::int64_t id) { return std::find_if(scene->graph.nodes.begin(), scene->graph.nodes.end(), [&](const auto& n) { return n.osm_id == id; })->position; };
    double turned = 0, prev = 0;
    for (std::size_t i = 0; i <= j.approaches.size(); ++i) {
        const auto& a = j.approaches[i % j.approaches.size()];
        const auto& e = scene->tile.entries[a.entry_index];
        const auto& way = *std::find_if(scene->graph.ways.begin(), scene->graph.ways.end(), [&](const auto& w) { return w.osm_id == e.way_id; });
        const auto near = pos(way.node_ids[a.at_end ? e.stretch.end_ref : e.stretch.start_ref]);
        const auto far = pos(way.node_ids[a.at_end ? e.stretch.start_ref : e.stretch.end_ref]);
        const double ang = std::atan2(static_cast<double>(far.y - near.y), static_cast<double>(far.x - near.x));
        if (i > 0) { double d = ang - prev; while (d < -1e-12) d += 2 * kPi; turned += d; }   // parallel stubs of two nodes turn by 0
        prev = ang;
    }
    CHECK(turned == Approx(2 * kPi).margin(1e-9));
    CHECK(stats.clusters == 1); CHECK(stats.cluster_nodes == 4); CHECK(stats.absorbed == 4); CHECK(stats.max_cluster_nodes == 4);
    CHECK(stats.junction_nodes == 4); CHECK(stats.skipped_unresolved == 0); CHECK(stats.face_fallbacks == 0);
    // Bit-identical on a second build. Far apart, the four nodes stay single junctions.
    CHECK(road_junctions(*scene) == js);
    const auto apart = road_junctions(*dual_crossing_scene(40.0));
    REQUIRE(apart.size() == 4);
    for (const auto& s : apart) { CHECK(s.nodes.size() == 1); CHECK(s.absorbed.empty()); CHECK(s.decline == JunctionDecline::None); }
}

TEST_CASE("T4: a staggered T pair is one cluster; the main road between the nodes stays covered", "[layer][junction]") {
    const auto scene = staggered_scene(5.0);
    const auto js = road_junctions(*scene);
    REQUIRE(js.size() == 1);
    const RoadJunction& j = js.front();
    CHECK(j.nodes == std::vector<std::int64_t>{1, 2});
    REQUIRE(j.decline == JunctionDecline::None);
    CHECK_FALSE(has(j, JunctionFlags::HullFallback));
    CHECK(j.absorbed.size() == 1);
    CHECK(j.approaches.size() == 4);
    check_cluster_rings(j);
    // Both edges of the main road between the two nodes, just inside the road bed.
    const auto& e = scene->tile.entries[j.absorbed.front()];
    const auto f = carve_footprint(*e.cross_section, e.profile->reference.length_m / 2);
    for (const double x : {0.5, 2.5, 4.5}) {
        CHECK(inside(j.rings[0], at(x, f.roadbed_left - 0.05)));
        CHECK(inside(j.rings[0], at(x, f.roadbed_right + 0.05)));
    }
    // The single-node builds do not link at 40 m: two records.
    CHECK(road_junctions(*staggered_scene(40.0)).size() == 2);
}

TEST_CASE("T4: a roundabout stretch never links; without the rule it does", "[layer][junction]") {
    const auto plain = road_junctions(*staggered_scene(5.0, {{"highway", "residential"}}));
    REQUIRE(plain.size() == 1);
    CHECK(plain.front().nodes.size() == 2);
    const Tags ring = {{"highway", "residential"}, {"junction", "roundabout"}};
    const auto scene = staggered_scene(5.0, ring);
    const auto js = road_junctions(*scene);
    REQUIRE(js.size() == 2);
    for (const auto& j : js) { CHECK(j.nodes.size() == 1); CHECK(j.absorbed.empty()); }
    // Negative control: the rule switched off links the two nodes again.
    JunctionBuildOptions sabotage; sabotage.roundabouts_never_link = false;
    const auto linked = road_junctions(*scene, builtin_road_rules_de(), nullptr, sabotage);
    REQUIRE(linked.size() == 1);
    CHECK(linked.front().nodes.size() == 2);
}

TEST_CASE("T4: cluster caps - a chain within 48 m is one cluster, a longer one keeps its singles plus a ClusterTooLarge record", "[layer][junction]") {
    constexpr double kSpacing = 18.0;
    // Three nodes over 36 m: one cluster.
    {
        const auto scene = chain_scene(3, kSpacing);
        JunctionBuildStats stats;
        const auto js = road_junctions(*scene, builtin_road_rules_de(), &stats);
        REQUIRE(js.size() == 1);
        CHECK(js.front().nodes == std::vector<std::int64_t>{1, 2, 3});
        CHECK(js.front().absorbed.size() == 2);
        CHECK(js.front().decline == JunctionDecline::None);
        CHECK(js.front().approaches.size() == 8);   // 12 stretch ends, 4 of them on the two absorbed stretches
        check_cluster_rings(js.front());
        // White-box: a node cap of 2 makes the same chain too large.
        auto rules = builtin_road_rules_de(); rules.junction.cluster_max_nodes = 2;
        const auto capped = road_junctions(*scene, rules);
        CHECK(capped.size() == 4);   // three singles plus one ClusterTooLarge record from node 1
        REQUIRE(record_of(capped, 1, true));
    }
    // Five nodes over 72 m exceed the 48 m extent.
    const auto scene = chain_scene(5, kSpacing);
    JunctionBuildStats stats;
    const auto js = road_junctions(*scene, builtin_road_rules_de(), &stats);
    std::size_t singles = 0, too_large = 0;
    for (const auto& j : js) {
        if (j.decline == JunctionDecline::ClusterTooLarge) {
            ++too_large;
            CHECK(has(j, JunctionFlags::Cluster));
            CHECK(j.nodes.size() >= 2);
            CHECK(j.nodes.front() == 1);
            for (const auto& ring : j.rings) CHECK(ring.vertices.empty());
            for (const auto& a : j.approaches) CHECK(a.trim_m == 0.0);
            CHECK_FALSE(j.absorbed.empty());
        } else {
            ++singles;
            CHECK(j.nodes.size() == 1);
            CHECK(j.decline == JunctionDecline::None);
        }
    }
    CHECK(singles == 5);
    CHECK(too_large == 1);
    CHECK(stats.declines[static_cast<std::size_t>(JunctionDecline::ClusterTooLarge)] == 1);
    CHECK(stats.clusters == 0);
    // Ascending by owner; for one owner the single record first.
    for (std::size_t i = 1; i < js.size(); ++i) CHECK(js[i - 1].nodes.front() <= js[i].nodes.front());
    CHECK(js[0].nodes.size() == 1); CHECK(js[1].decline == JunctionDecline::ClusterTooLarge);
    // Negative control: without the caps the five nodes form one cluster.
    JunctionBuildOptions no_caps; no_caps.cluster_caps = false;
    const auto all = road_junctions(*scene, builtin_road_rules_de(), nullptr, no_caps);
    REQUIRE(all.size() == 1);
    CHECK(all.front().nodes.size() == 5);
}

namespace {
struct ClusterCensus {
    std::size_t tiles = 0, owned_records = 0, uncovered = 0, multi_tile_records = 0, multi_tile_mismatch = 0;
    std::size_t skipped_unresolved = 0, invalid_rings = 0, nested_bad = 0, nested_junctions = 0, face_fallbacks = 0;
    double nested_worst_mm = 0;
    std::map<std::size_t, std::size_t> sizes;                          // nodes per built record (ClusterTooLarge excluded)
    std::array<std::size_t, kJunctionDeclineCount> declines{};
    std::size_t hulls = 0, hull_shallow = 0, hull_short = 0, hull_other = 0, ring_records = 0, absorbed = 0;
    std::size_t nesting_hulls = 0, nesting_declined = 0;              // D34, summed over every tile's build
    std::size_t max_outer = 0;
    double max_extent_m = 0;                                          // largest bounding-box side of a cluster's nodes
};
// Every L2 tile of the fixture, its junctions built WITH clustering. A record is counted in the tile of its owner node.
// Every build of the same record (owner, decline kind) in another tile must be identical after normalising entry
// indices; every junction node of a built tile must be covered by a record of its owner's tile.
ClusterCensus cluster_census_of(const std::string& pbf, const JunctionBuildOptions& options = {}) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const auto builder = RoadReferenceBuilder::make(*world); REQUIRE(builder);
    const auto home_of = [&](std::int64_t id) {
        const auto n = std::lower_bound(world->nodes.begin(), world->nodes.end(), id, [](const auto& x, std::int64_t v) { return x.osm_id < v; });
        return *tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n->position.x) / 1000.0, static_cast<double>(n->position.y) / 1000.0);
    };
    std::set<TileKey> keys;
    for (const auto& n : world->nodes) {
        const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0);
        if (k) keys.insert(*k);
    }
    ClusterCensus c;
    std::map<std::pair<std::int64_t, bool>, std::vector<RoadJunction>> by_record;
    std::set<std::int64_t> covered, built_junction_nodes;
    for (const auto& key : keys) {
        const auto probe = select_road_graph_tile(*world, key);
        if (!probe || probe->owned_edges.empty()) continue;
        auto g = g2m_test::geometry_in_tile(*world, key); REQUIRE(g);
        ++c.tiles;
        JunctionBuildStats stats;
        auto js = build_road_junctions(*g, *builder, builtin_road_rules_de(), &stats, options); REQUIRE(js);
        c.skipped_unresolved += stats.skipped_unresolved;
        for (const auto& n : g->source.graph.nodes) if (builder->degree(n.osm_id) >= 3 && home_of(n.osm_id) == key) built_junction_nodes.insert(n.osm_id);
        for (auto& j : *js) {
            const bool too_large = j.decline == JunctionDecline::ClusterTooLarge;
            // Normalise per-tile entry indices to (way id, start_ref) identities.
            double min_gap = 360, min_len = 1e9;
            std::vector<double> angs;
            for (auto& a : j.approaches) {
                const auto& e = g->entries[a.entry_index];
                if (a.status == JunctionApproachStatus::Used && e.profile) {
                    const double L = e.profile->reference.length_m; min_len = std::min(min_len, L);
                    const auto p0 = lateral_point(e.profile->reference, a.at_end ? L : 0.0, 0.0);
                    const auto p1 = lateral_point(e.profile->reference, a.at_end ? std::max(L - 5.0, 0.0) : std::min(5.0, L), 0.0);
                    if (p0 && p1) angs.push_back(std::atan2(p1->y - p0->y, p1->x - p0->x) * 180.0 / kPi);
                }
                a.entry_index = static_cast<std::size_t>(e.way_id * 1000 + e.stretch.start_ref);
            }
            for (auto& x : j.absorbed) { const auto& e = g->entries[x]; x = static_cast<std::size_t>(e.way_id * 1000 + e.stretch.start_ref); }
            std::sort(j.absorbed.begin(), j.absorbed.end());
            by_record[{j.nodes.front(), too_large}].push_back(j);
            if (home_of(j.nodes.front()) != key) continue;
            ++c.owned_records;
            ++c.declines[static_cast<std::size_t>(j.decline)];
            if (!too_large) {
                ++c.sizes[j.nodes.size()]; for (const auto n : j.nodes) covered.insert(n); c.absorbed += j.absorbed.size();
                std::int64_t x0 = INT64_MAX, x1 = INT64_MIN, y0 = INT64_MAX, y1 = INT64_MIN;
                for (const auto n : j.nodes) {
                    const auto p = std::lower_bound(world->nodes.begin(), world->nodes.end(), n, [](const auto& x, std::int64_t v) { return x.osm_id < v; })->position;
                    x0 = std::min(x0, p.x); x1 = std::max(x1, p.x); y0 = std::min(y0, p.y); y1 = std::max(y1, p.y);
                }
                c.max_extent_m = std::max(c.max_extent_m, static_cast<double>(std::max(x1 - x0, y1 - y0)) / 1000.0);
            }
            if (j.decline != JunctionDecline::None) continue;
            ++c.ring_records;
            c.max_outer = std::max(c.max_outer, j.rings[2].vertices.size());
            for (const auto& ring : j.rings) if (!junction_ring_valid(ring)) ++c.invalid_rings;
            double worst = 0; const auto bad = nested_violations(j, &worst);
            c.nested_bad += bad; if (bad) ++c.nested_junctions; c.nested_worst_mm = std::max(c.nested_worst_mm, worst);
            if (has(j, JunctionFlags::HullFallback)) {
                ++c.hulls;
                std::sort(angs.begin(), angs.end());
                for (std::size_t i = 0; i < angs.size(); ++i) min_gap = std::min(min_gap, (i + 1 < angs.size() ? angs[i + 1] : angs[0] + 360.0) - angs[i]);
                if (min_gap < 25.0) ++c.hull_shallow; else if (min_len < 12.0) ++c.hull_short; else ++c.hull_other;
            }
        }
        c.face_fallbacks += stats.face_fallbacks;
        c.nesting_hulls += stats.nesting_hulls; c.nesting_declined += stats.nesting_declined;
    }
    for (const auto n : built_junction_nodes) if (!covered.count(n)) ++c.uncovered;
    for (const auto& [k, list] : by_record) {
        if (list.size() < 2) continue;
        ++c.multi_tile_records;
        for (std::size_t i = 1; i < list.size(); ++i) if (!(list[i] == list[0])) { ++c.multi_tile_mismatch; break; }
    }
    return c;
}
void print_cluster_census(const char* name, const ClusterCensus& c) {
    std::printf("[cluster census] %s: tiles %zu, records counted in the owner's tile %zu (ring records %zu), records by node count:", name, c.tiles, c.owned_records, c.ring_records);
    for (const auto& [n, k] : c.sizes) std::printf(" %zu:%zu", n, k);
    std::printf(", largest cluster extent %.1f m, absorbed stretches %zu\n  declines: none %zu, cluster too large %zu, reach %zu, single approach %zu, degenerate %zu, underground %zu\n", c.max_extent_m, c.absorbed,
        c.declines[0], c.declines[1], c.declines[2], c.declines[3], c.declines[4], c.declines[5]);
    std::printf("  hull fallbacks %zu of %zu (shallow pair < 25 deg %zu, an approach shorter than 12 m %zu, other %zu), face-walk fallbacks over all tiles %zu\n", c.hulls,
        c.ring_records, c.hull_shallow, c.hull_short, c.hull_other, c.face_fallbacks);
    std::printf("  invalid rings %zu, nesting violations %zu in %zu records (worst %.0f mm), unresolved skips over all tiles %zu, junction nodes not covered by their tile %zu, records built in >= 2 tiles %zu (mismatching %zu)\n",
        c.invalid_rings, c.nested_bad, c.nested_junctions, c.nested_worst_mm, c.skipped_unresolved, c.uncovered, c.multi_tile_records, c.multi_tile_mismatch);
    std::printf("  largest outer ring %zu vertices; D34 nesting hulls over all tiles' builds %zu, nesting declines %zu\n", c.max_outer, c.nesting_hulls, c.nesting_declined);
}
}

TEST_CASE("Fixtures with clustering: valid rings, every junction node covered in its own tile, every record identical in every tile that builds it", "[layer][junction][roads]") {
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        INFO(pbf);
        const auto c = cluster_census_of(pbf);
        print_cluster_census(pbf, c);
        CHECK(c.owned_records > 50);
        CHECK(c.sizes.size() > 1);            // some clusters exist
        CHECK(c.invalid_rings == 0);
        CHECK(c.nested_bad == 0);             // D34 (slice 8): asserted, encoding 7 rejects a record that does not nest
        CHECK(c.nesting_declined == 0);
        CHECK(c.face_fallbacks == 0);
        CHECK(c.uncovered == 0);
        CHECK(c.multi_tile_records > 50);
        CHECK(c.multi_tile_mismatch == 0);
    }
}

TEST_CASE("D34: simple offset rings that do not nest take the all-layer hull, which nests", "[layer][junction][roads]") {
    // D21 handed slice 8 two junctions of frankfurt_hoechst (5 vertices, worst 352 mm outside) whose offset rings are
    // simple but not nested. The builder now sends them to the all-layer hull; with the rule off (the sabotage) the
    // census finds the violations again.
    const auto with = cluster_census_of("frankfurt_hoechst.osm.pbf");
    JunctionBuildOptions off; off.hull_when_not_nested = false;
    const auto without = cluster_census_of("frankfurt_hoechst.osm.pbf", off);
    std::printf("[D34] frankfurt_hoechst: nesting hulls %zu (all tiles' builds), hulls among owned records %zu (rule off: %zu), nesting violations %zu in %zu records (rule off: %zu in %zu, worst %.0f mm)\n",
        with.nesting_hulls, with.hulls, without.hulls, with.nested_bad, with.nested_junctions, without.nested_bad, without.nested_junctions, without.nested_worst_mm);
    CHECK(with.nesting_hulls > 0);
    CHECK(with.nested_bad == 0);
    CHECK(with.hulls == without.hulls + without.nested_junctions);
    CHECK(without.nested_bad > 0);
}

TEST_CASE("Negative control: a possible link to an incomplete node treated as absent breaks tile invariance", "[layer][junction][roads]") {
    // The same census with the sabotage: a tile that sees a cluster member only partially builds a different record
    // than the tile that sees the whole cluster. This shows the identity check can fail.
    JunctionBuildOptions sabotage; sabotage.unknown_links_are_absent = true;
    std::size_t mismatches = 0;
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) mismatches += cluster_census_of(pbf, sabotage).multi_tile_mismatch;
    std::printf("[cluster census, sabotage unknown links absent] records mismatching between tiles: %zu\n", mismatches);
    CHECK(mismatches > 0);
}

TEST_CASE("Diag: junction pairs joined over two stretches (a degree-2 way join) that the single-entry link rule cannot cluster", "[.][diag][junction]") {
    // D20: a link needs ONE stretch between the two junction nodes. Two ways joined end to end at a degree-2 node
    // make the connection two stretches; this counts the pairs whose capped single-node trims would cover both.
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
        const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
        const auto builder = RoadReferenceBuilder::make(*world); REQUIRE(builder);
        std::set<TileKey> keys;
        for (const auto& n : world->nodes) { const auto k = tile_key_at(g2m_test::kZone32N, 2, n.position.x / 1000.0, n.position.y / 1000.0); if (k) keys.insert(*k); }
        std::set<std::pair<std::int64_t, std::int64_t>> chained, direct;
        for (const auto& key : keys) {
            const auto probe = select_road_graph_tile(*world, key);
            if (!probe || probe->owned_edges.empty()) continue;
            auto g = g2m_test::geometry_in_tile(*world, key); REQUIRE(g);
            auto js = build_node_junctions(*g, *builder); REQUIRE(js);
            std::map<std::int64_t, const RoadJunction*> by_node;
            for (const auto& j : *js) by_node[j.nodes.front()] = &j;
            const auto& ways = g->source.graph.ways;
            const auto way_of = [&](std::int64_t id) { return &*std::lower_bound(ways.begin(), ways.end(), id, [](const auto& w, std::int64_t v) { return w.osm_id < v; }); };
            const auto ends = [&](std::size_t entry) { const auto& e = g->entries[entry]; const auto* w = way_of(e.way_id); return std::make_pair(w->node_ids[e.stretch.start_ref], w->node_ids[e.stretch.end_ref]); };
            const auto trim_at = [&](const RoadJunction& j, std::size_t entry, bool at_end) { for (const auto& a : j.approaches) if (a.entry_index == entry && a.at_end == at_end) return std::min(a.trim_m, 30.0); return 0.0; };
            for (const auto& [x, jx] : by_node) {
                for (const auto& a : jx->approaches) {
                    if (a.status != JunctionApproachStatus::Used) continue;
                    const auto [s0, s1] = ends(a.entry_index);
                    const std::int64_t y = a.at_end ? s0 : s1;
                    const double l1 = g->entries[a.entry_index].profile->reference.length_m;
                    if (builder->degree(y) >= 3) { if (by_node.count(y) && trim_at(*jx, a.entry_index, a.at_end) + trim_at(*by_node[y], a.entry_index, !a.at_end) >= l1) direct.insert({std::min(x, y), std::max(x, y)}); continue; }
                    if (builder->degree(y) != 2) continue;
                    for (std::size_t k = 0; k < g->entries.size(); ++k) {
                        if (k == a.entry_index || !g->entries[k].profile) continue;
                        const auto [t0, t1] = ends(k);
                        if (t0 != y && t1 != y) continue;
                        const std::int64_t z = t0 == y ? t1 : t0;
                        if (z == x || builder->degree(z) < 3 || !by_node.count(z)) continue;
                        const double l2 = g->entries[k].profile->reference.length_m;
                        if (trim_at(*jx, a.entry_index, a.at_end) + trim_at(*by_node[z], k, t1 == z) >= l1 + l2) chained.insert({std::min(x, z), std::max(x, z)});
                    }
                }
            }
        }
        std::printf("[chained links] %s: junction pairs linked over one stretch %zu, pairs whose trims cover a two-stretch connection %zu\n", pbf, direct.size(), chained.size());
    }
}
