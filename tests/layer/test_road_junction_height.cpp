// Street fidelity P2 slice 5: junction heights and raised junction tables (design sections 4.2-4.4, tests T5 and T6):
// the blended approach surfaces z_J, their properties on the trim lines, and the table / hump ramps and the feature
// suppression, on synthetic junctions over a tilted, bumpy DEM and on the two committed OSM fixtures.
#include "alloc_counter.h"
#include "road_fixture.h"
#include "g2m/core/geom/ring_set.h"
#include "g2m/layer/road_junction.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
using namespace g2m;
using Catch::Approx;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};
const TileKey kTile{zone, 2, 455, 5421};
constexpr double kOriginE = 455.0 * 1024.0 + 512.0, kOriginN = 5421.0 * 1024.0 + 512.0;   // tile centre, metres
constexpr double kPi = 3.14159265358979323846;
using Tags = std::vector<std::pair<std::string, std::string>>;

struct N { std::int64_t id; double x, y; Tags tags = {}; };
struct W { std::int64_t id; std::vector<std::int64_t> nodes; Tags tags = {{"highway", "residential"}}; };

// A tilted, bumpy terrain: every approach of a junction gets its own grade, so the blend has something to do.
double tilted(double e, double n) {
    const double x = e - kOriginE, y = n - kOriginN;
    return 100.0 + 0.03 * x + 0.02 * y + 1.5 * std::sin(x / 23.0) * std::cos(y / 31.0);
}

struct Scene {
    RoadGraph graph;
    RoadGeomTile tile;
    std::optional<RoadReferenceBuilder> builder;
};
std::unique_ptr<Scene> make_scene(const std::vector<N>& nodes, std::vector<W> ways, bool tilt = true) {
    auto scene = std::make_unique<Scene>();
    for (const auto& n : nodes) {
        Tags tags = n.tags; std::sort(tags.begin(), tags.end());
        scene->graph.nodes.push_back({n.id, {std::llround((kOriginE + n.x) * 1000.0), std::llround((kOriginN + n.y) * 1000.0)}, tags});
    }
    std::sort(scene->graph.nodes.begin(), scene->graph.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    std::sort(ways.begin(), ways.end(), [](const auto& a, const auto& b) { return a.id < b.id; });
    for (auto& w : ways) {
        Tags tags = w.tags; std::sort(tags.begin(), tags.end());
        scene->graph.ways.push_back({w.id, 3, w.nodes, tags});
        for (std::size_t i = 1; i < w.nodes.size(); ++i) scene->graph.edges.push_back({w.id, static_cast<std::uint32_t>(i - 1), w.nodes[i - 1], w.nodes[i]});
    }
    auto tile = g2m_test::geometry_in_tile(scene->graph, kTile, true, tilt ? std::function<double(double, double)>(tilted) : std::function<double(double, double)>());
    REQUIRE(tile);
    scene->tile = std::move(*tile);
    auto builder = RoadReferenceBuilder::make(scene->graph);
    REQUIRE(builder);
    scene->builder = std::move(*builder);
    return scene;
}
std::unique_ptr<Scene> x_scene(const Tags& node_tags = {}, bool tilt = true) {
    return make_scene({{1, 0, 0, node_tags}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}, {5, 0, 300}}, {{10, {2, 1, 3}}, {11, {4, 1, 5}}}, tilt);
}
std::unique_ptr<Scene> t_scene() {
    return make_scene({{1, 0, 0}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}}, {{10, {2, 1, 3}}, {11, {1, 4}}});
}
std::unique_ptr<Scene> y_scene(double degrees) {
    const double a = degrees * kPi / 180.0;
    return make_scene({{1, 0, 0}, {2, -300, 0}, {3, 300, 0}, {4, 300 * std::cos(a), 300 * std::sin(a)}}, {{10, {2, 1, 3}}, {11, {1, 4}}});
}
// A T whose side arm leaves southwards and bends east on a 100 m radius arc.
std::unique_ptr<Scene> curved_scene() {
    std::vector<N> nodes = {{1, 0, 0}, {2, -300, 0}, {3, 300, 0}};
    std::vector<std::int64_t> arc = {1};
    for (int i = 1; i <= 9; ++i) {
        const double phi = i * 10.0 * kPi / 180.0;
        nodes.push_back({100 + i, 100.0 - 100.0 * std::cos(phi), -100.0 * std::sin(phi)});
        arc.push_back(100 + i);
    }
    return make_scene(nodes, {{10, {2, 1, 3}}, {11, arc}});
}
RoadJunction junction_of(const Scene& scene, std::int64_t node, JunctionBuildStats* stats = nullptr, const JunctionBuildOptions& options = {}) {
    auto j = build_node_junction(scene.tile, *scene.builder, node, builtin_road_rules_de(), stats, options);
    REQUIRE(j);
    return std::move(*j);
}

// ---- the probes of T5, all parameterised by the weight function so the sabotages can swap it ----

using WeightFn = std::function<void(const JunctionSurface&, double, double, std::span<double>)>;
void real_weights(const JunctionSurface& s, double x, double y, std::span<double> w) { junction_weights(s, x, y, w); }
double seg_dist(double px, double py, double ax, double ay, double bx, double by) {
    const double dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
    const double u = len2 > 0 ? std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0.0, 1.0) : 0.0;
    return std::hypot(ax + u * dx - px, ay + u * dy - py);
}
// SABOTAGE: the product form with plain distances instead of squared distances (the weights then change at O(d)
// across a trim line, not O(d^2): the surface has a kink there).
void plain_distance_weights(const JunctionSurface& s, double x, double y, std::span<double> w) {
    const std::size_t n = s.approaches.size();
    double sum = 0;
    for (std::size_t m = 0; m < n; ++m) {
        double p = 1;
        for (std::size_t j = 0; j < n; ++j) if (j != m) { const auto& a = s.approaches[j]; p *= seg_dist(x, y, a.trim_ax, a.trim_ay, a.trim_bx, a.trim_by); }
        w[m] = p; sum += p;
    }
    for (std::size_t m = 0; m < n; ++m) w[m] /= sum;
}
// SABOTAGE: a plain average of the approach surfaces.
void average_weights(const JunctionSurface& s, double, double, std::span<double> w) {
    for (std::size_t m = 0; m < s.approaches.size(); ++m) w[m] = 1.0 / static_cast<double>(s.approaches.size());
}
double z_blend(const JunctionSurface& s, const RoadGeomTile& tile, double x, double y, const WeightFn& weights) {
    std::vector<double> w(s.approaches.size());
    weights(s, x, y, w);
    double z = 0;
    for (std::size_t i = 0; i < w.size(); ++i) z += w[i] * junction_approach_height_m(s, tile, i, x, y);
    return z;
}

// Largest |z_J - z_i| over `points` points of every trim segment i.
double worst_match(const JunctionSurface& s, const RoadGeomTile& tile, const WeightFn& weights, int points) {
    double worst = 0;
    for (std::size_t i = 0; i < s.approaches.size(); ++i) {
        const auto& a = s.approaches[i];
        for (int k = 0; k < points; ++k) {
            const double u = points > 1 ? static_cast<double>(k) / static_cast<double>(points - 1) : 0.5;
            const double x = a.trim_ax + u * (a.trim_bx - a.trim_ax), y = a.trim_ay + u * (a.trim_by - a.trim_ay);
            worst = std::max(worst, std::abs(z_blend(s, tile, x, y, weights) - junction_approach_height_m(s, tile, i, x, y)));
        }
    }
    return worst;
}
// Largest difference between the slope of z_J and of z_i along the inward normal of trim segment i: the ONE-SIDED
// difference into the junction over h = 1 um. (A central difference cancels a symmetric kink |d| and would pass the
// plain-distance sabotage; the true weights deviate by O(d^2), so the one-sided difference is O(h).)
double worst_slope_mismatch(const JunctionSurface& s, const RoadGeomTile& tile, const WeightFn& weights, int points, double h = 1e-6) {
    double worst = 0;
    for (std::size_t i = 0; i < s.approaches.size(); ++i) {
        const auto& a = s.approaches[i];
        double nx = -(a.trim_by - a.trim_ay), ny = a.trim_bx - a.trim_ax;
        const double len = std::hypot(nx, ny); nx /= len; ny /= len;
        // Inward = against the direction in which the reference leaves the node.
        const double rx = a.ref_x[1] - a.ref_x[0], ry = a.ref_y[1] - a.ref_y[0];
        if (nx * rx + ny * ry > 0) { nx = -nx; ny = -ny; }
        for (int k = 1; k < points; ++k) {   // interior points: the ends are the ring's trim corners, where two segments meet
            const double u = static_cast<double>(k) / static_cast<double>(points);
            const double x = a.trim_ax + u * (a.trim_bx - a.trim_ax), y = a.trim_ay + u * (a.trim_by - a.trim_ay);
            const double sj = (z_blend(s, tile, x + h * nx, y + h * ny, weights) - z_blend(s, tile, x, y, weights)) / h;
            const double si = (junction_approach_height_m(s, tile, i, x + h * nx, y + h * ny) - junction_approach_height_m(s, tile, i, x, y)) / h;
            worst = std::max(worst, std::abs(sj - si));
        }
    }
    return worst;
}
// Points of a `step` m lattice inside the outer ring where z_J lies outside [min z_i, max z_i] (+ 1e-9).
std::size_t bound_violations(const JunctionSurface& s, const RoadGeomTile& tile, const RoadJunction& j, double step, std::size_t* tested = nullptr) {
    geom::RingSet ring; ring.outers.push_back(j.rings[2].vertices); ring.bbox = geom::bounding_box(ring);
    std::int64_t x0 = ring.bbox.w, x1 = ring.bbox.e, y0 = ring.bbox.s, y1 = ring.bbox.n;
    std::size_t bad = 0, count = 0;
    const auto stride = static_cast<std::int64_t>(step * 1000.0);
    std::vector<double> w(s.approaches.size());
    for (std::int64_t yy = (y0 / stride) * stride; yy <= y1; yy += stride)
        for (std::int64_t xx = (x0 / stride) * stride; xx <= x1; xx += stride) {
            if (!geom::contains(ring, {xx, yy})) continue;
            const double x = static_cast<double>(xx) / 1000.0, y = static_cast<double>(yy) / 1000.0;
            double lo = 1e18, hi = -1e18;
            for (std::size_t i = 0; i < s.approaches.size(); ++i) { const double z = junction_approach_height_m(s, tile, i, x, y); lo = std::min(lo, z); hi = std::max(hi, z); }
            const double z = junction_height_m(s, tile, x, y);
            ++count;
            if (z < lo - 1e-9 || z > hi + 1e-9) ++bad;
        }
    if (tested) *tested = count;
    return bad;
}

// The largest |z_J(p) - z_J(q)| between lattice neighbours p, q (step metres apart, both inside the outer ring): the
// blend's own slope, in m per `step`. The G3-D criterion on cross-slope (+-0.2 %) belongs to the carved cores; this
// is the junction-internal analogue reported next to it.
double max_lattice_step(const JunctionSurface& s, const RoadGeomTile& tile, const RoadJunction& j, double step) {
    geom::RingSet ring; ring.outers.push_back(j.rings[2].vertices); ring.bbox = geom::bounding_box(ring);
    const auto stride = static_cast<std::int64_t>(step * 1000.0);
    double worst = 0;
    for (std::int64_t yy = (ring.bbox.s / stride) * stride; yy <= ring.bbox.n; yy += stride)
        for (std::int64_t xx = (ring.bbox.w / stride) * stride; xx <= ring.bbox.e; xx += stride) {
            if (!geom::contains(ring, {xx, yy})) continue;
            const double z = junction_height_m(s, tile, static_cast<double>(xx) / 1000.0, static_cast<double>(yy) / 1000.0);
            for (const auto& d : {geom::PointMm{stride, 0}, geom::PointMm{0, stride}}) {
                if (!geom::contains(ring, {xx + d.x, yy + d.y})) continue;
                worst = std::max(worst, std::abs(junction_height_m(s, tile, static_cast<double>(xx + d.x) / 1000.0, static_cast<double>(yy + d.y) / 1000.0) - z));
            }
        }
    return worst;
}
void check_surface_properties(const Scene& scene, std::int64_t node, int points) {
    const auto j = junction_of(scene, node);
    REQUIRE(j.decline == JunctionDecline::None);
    const auto surface = make_junction_surface(scene.tile, j); REQUIRE(surface);
    REQUIRE(surface->approaches.size() == j.approaches.size());
    // z_J = z_i on trim segment i, and the weights are exactly the unit vector there.
    CHECK(worst_match(*surface, scene.tile, real_weights, points) <= 1e-9);
    // C1: the normal slope of z_J equals that of z_i across the trim line.
    CHECK(worst_slope_mismatch(*surface, scene.tile, real_weights, 8) <= 1e-4);
    // No overshoot.
    std::size_t tested = 0;
    CHECK(bound_violations(*surface, scene.tile, j, 0.25, &tested) == 0);
    CHECK(tested > 100);
}
}

TEST_CASE("T5: junction heights match the approaches on every trim line, are C1 across it and never overshoot", "[layer][junction][height]") {
    SECTION("X") { check_surface_properties(*x_scene(), 1, 64); }
    SECTION("T") { check_surface_properties(*t_scene(), 1, 64); }
    SECTION("Y 30 degrees") { check_surface_properties(*y_scene(30.0), 1, 64); }
    SECTION("curved side arm") {
        // A chord polyline of a curved arm differs from the arc by millimetres, so the projection is not exact: the
        // match is checked against the same approach function z_i (which T5 defines through the projection), so the
        // identity itself stays exact.
        check_surface_properties(*curved_scene(), 1, 64);
    }
}

TEST_CASE("T5 sabotage: plain distance weights break the slope condition, a plain average breaks the match", "[layer][junction][height]") {
    const auto scene = x_scene();
    const auto j = junction_of(*scene, 1);
    const auto surface = make_junction_surface(scene->tile, j); REQUIRE(surface);
    REQUIRE(worst_slope_mismatch(*surface, scene->tile, real_weights, 8) <= 1e-4);
    const double plain_slope = worst_slope_mismatch(*surface, scene->tile, plain_distance_weights, 8);
    const double average_match = worst_match(*surface, scene->tile, average_weights, 16);
    std::printf("T5 sabotage: plain distances -> slope mismatch %.4f (limit 1e-4); plain average -> match error %.4f m (limit 1e-9)\n", plain_slope, average_match);
    CHECK(plain_slope > 1e-4);
    CHECK(average_match > 1e-9);
    // The plain-distance weights still match exactly on the line (their product also holds a zero factor): only the slope sees it.
    CHECK(worst_match(*surface, scene->tile, plain_distance_weights, 16) <= 1e-9);
}

TEST_CASE("T5: weights sum to one, are unit on their own trim segment and handle the corner shared by two segments", "[layer][junction][height]") {
    const auto scene = x_scene();
    const auto j = junction_of(*scene, 1);
    const auto surface = make_junction_surface(scene->tile, j); REQUIRE(surface);
    std::vector<double> w(surface->approaches.size());
    for (double x = -12; x <= 12; x += 1.7) for (double y = -12; y <= 12; y += 1.9) {
        junction_weights(*surface, kOriginE + x, kOriginN + y, w);
        double sum = 0; for (const double v : w) { CHECK(v >= 0.0); sum += v; }
        CHECK(sum == Approx(1.0).margin(1e-12));
    }
    // At the end of trim segment 0 (a ring corner is not shared by two trim segments of a straight X, but the endpoints
    // of neighbouring segments are close): the weight of segment 0 is 1 on it.
    const auto& a = surface->approaches[0];
    junction_weights(*surface, a.trim_ax, a.trim_ay, w);
    CHECK(w[0] == Approx(1.0).margin(1e-12));
    // Two segments at once: the average over the two (constructed: a surface of two approaches touching at a point).
    JunctionSurface two; two.approaches.resize(2);
    two.approaches[0].trim_ax = 0; two.approaches[0].trim_ay = 0; two.approaches[0].trim_bx = 1; two.approaches[0].trim_by = 0;
    two.approaches[1].trim_ax = 1; two.approaches[1].trim_ay = 0; two.approaches[1].trim_bx = 1; two.approaches[1].trim_by = 1;
    std::array<double, 2> w2{};
    junction_weights(two, 1.0, 0.0, w2);
    CHECK(w2[0] == 0.5); CHECK(w2[1] == 0.5);
}

TEST_CASE("T5: junction_height_m allocates nothing", "[layer][junction][height][alloc]") {
    const auto scene = x_scene();
    const auto j = junction_of(*scene, 1);
    const auto surface = make_junction_surface(scene->tile, j); REQUIRE(surface);
    if (!g2m_test::kAllocCounterAvailable) SKIP("allocation counting unavailable under a sanitizer build (G2M_SANITIZER_BUILD)");
    double sink = 0;
    g2m_test::ScopedAllocCounter counter;
    for (int i = 0; i < 500; ++i) sink += junction_height_m(*surface, scene->tile, kOriginE - 6.0 + 0.024 * i, kOriginN - 5.0 + 0.02 * i);
    CHECK(counter.count() == 0);
    CHECK(std::isfinite(sink));
}

TEST_CASE("T5: a stub's z_J at an approach's far side equals the tangent extension of the profile before station 0", "[layer][junction][height]") {
    // A point on the reference line behind station 0 (inside the junction) takes p(0) + grade * sigma of the approach.
    const auto scene = x_scene();
    const auto j = junction_of(*scene, 1);
    const auto surface = make_junction_surface(scene->tile, j); REQUIRE(surface);
    for (std::size_t i = 0; i < surface->approaches.size(); ++i) {
        const auto& a = surface->approaches[i];
        const auto& e = scene->tile.entries[a.entry_index];
        const double s0 = a.at_end ? a.length_m : 0.0;
        const auto pose = e.profile->at(s0); REQUIRE(pose);
        const double dx = a.ref_x[0] - a.ref_x[1], dy = a.ref_y[0] - a.ref_y[1];   // from the node, away from the arm
        const double len = std::hypot(dx, dy);
        const double back = 2.0;                                                    // 2 m behind station 0
        const double z = junction_approach_height_m(*surface, scene->tile, i, a.ref_x[0] + back * dx / len, a.ref_y[0] + back * dy / len);
        // grade is d(height)/d(s); behind the end of an at_end approach s grows, behind a start s falls.
        const double expect = pose->height_m + pose->grade * (a.at_end ? back : -back);
        CHECK(z == Approx(expect).margin(1e-9));
    }
}

// ---- T6 and T5 on the fixtures ----

namespace {
struct FixtureNumbers {
    std::size_t junctions = 0, surfaces = 0, no_surface = 0, points = 0;
    double worst_match = 0, worst_slope = 0;
    std::size_t bound_bad = 0, bound_points = 0, bound_junctions = 0;
    std::size_t nodes_checked = 0, stubs_checked = 0;
    double worst_node_mismatch = 0, worst_step_1m = 0;
    std::size_t node_over_limit = 0;
    std::size_t raised = 0, ignored = 0, ramps = 0, suppressed = 0;
};
FixtureNumbers fixture_numbers(const std::string& pbf, const std::function<double(double, double)>& dem) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const auto builder = RoadReferenceBuilder::make(*world); REQUIRE(builder);
    std::set<TileKey> keys;
    for (const auto& n : world->nodes) {
        const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0);
        if (k) keys.insert(*k);
    }
    FixtureNumbers out;
    for (const auto& key : keys) {
        const auto probe = select_road_graph_tile(*world, key);
        if (!probe || probe->owned_edges.empty()) continue;
        auto g = g2m_test::geometry_in_tile(*world, key, true, dem); REQUIRE(g);
        JunctionBuildStats stats;
        auto js = build_road_junctions(*g, *builder, builtin_road_rules_de(), &stats); REQUIRE(js);
        out.raised += stats.raised_tables; out.ignored += stats.calming_ignored;
        const auto adjust = adjust_features_for_junctions(*g, *js);
        out.ramps += adjust.ramps.size(); out.suppressed += adjust.suppressed;
        RoadGeomTile kept = *g; kept.features = adjust.features;
        for (const auto& j : *js) {
            const auto node = std::find_if(world->nodes.begin(), world->nodes.end(), [&](const auto& n) { return n.osm_id == j.nodes.front(); });
            const auto home = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(node->position.x) / 1000.0, static_cast<double>(node->position.y) / 1000.0);
            if (*home != key || j.decline != JunctionDecline::None) continue;   // counted in the owner tile only
            ++out.junctions;
            const auto surface = make_junction_surface(kept, j);
            if (!surface) { ++out.no_surface; continue; }
            ++out.surfaces;
            out.worst_match = std::max(out.worst_match, worst_match(*surface, kept, real_weights, 16));
            out.worst_slope = std::max(out.worst_slope, worst_slope_mismatch(*surface, kept, real_weights, 4));
            out.points += surface->approaches.size() * 16;
            std::size_t tested = 0;
            const std::size_t bad = bound_violations(*surface, kept, j, 2.0, &tested);
            out.bound_bad += bad; out.bound_points += tested; if (bad) ++out.bound_junctions;
            out.worst_step_1m = std::max(out.worst_step_1m, max_lattice_step(*surface, kept, j, 1.0));
            // T6: every stub that ends at one node of the junction reads the same height at that node.
            std::map<std::int64_t, std::vector<double>> at_node;
            for (std::size_t i = 0; i < surface->approaches.size(); ++i) {
                const auto& a = surface->approaches[i];
                const auto& e = kept.entries[a.entry_index];
                const auto* way = &*std::find_if(kept.source.graph.ways.begin(), kept.source.graph.ways.end(), [&](const auto& w) { return w.osm_id == e.way_id; });
                const std::int64_t id = way->node_ids[a.at_end ? e.stretch.end_ref : e.stretch.start_ref];
                const auto np = std::find_if(kept.source.graph.nodes.begin(), kept.source.graph.nodes.end(), [&](const auto& n) { return n.osm_id == id; })->position;
                at_node[id].push_back(junction_approach_height_m(*surface, kept, i, static_cast<double>(np.x) / 1000.0, static_cast<double>(np.y) / 1000.0));
                ++out.stubs_checked;
            }
            for (const auto& [id, zs] : at_node) {
                (void)id; ++out.nodes_checked;
                if (zs.size() < 2) continue;
                const auto [lo, hi] = std::minmax_element(zs.begin(), zs.end());
                out.worst_node_mismatch = std::max(out.worst_node_mismatch, *hi - *lo);
                if (*hi - *lo > 1.0 / 256.0) ++out.node_over_limit;
            }
        }
    }
    return out;
}
void print_numbers(const char* name, const char* dem, const FixtureNumbers& n) {
    std::printf("[junction heights] %s (%s): %zu junctions, %zu surfaces (%zu without), worst |z_J - z_i| on %zu trim points %.3e m, worst slope mismatch %.3e, bound violations %zu in %zu junctions of %zu lattice points; "
        "nodes %zu (stubs %zu) worst approach-to-node mismatch %.3e m (> 1/256 m: %zu), largest z_J step between 1 m lattice neighbours %.4f m (report only: where two diverging carriageways have their own cores 1-2 m apart at the gore, the data differ by that much); raised tables %zu, ignored calming nodes %zu, ramps %zu, suppressed features %zu\n",
        name, dem, n.junctions, n.surfaces, n.no_surface, n.points, n.worst_match, n.worst_slope, n.bound_bad, n.bound_junctions, n.bound_points,
        n.nodes_checked, n.stubs_checked, n.worst_node_mismatch, n.node_over_limit, n.worst_step_1m, n.raised, n.ignored, n.ramps, n.suppressed);
}
}

TEST_CASE("T5/T6 fixtures: matches, C1, no overshoot and approach-to-node agreement on every built junction", "[layer][junction][height][roads]") {
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto flat = fixture_numbers(pbf, {});
        print_numbers(pbf, "flat DEM", flat);
        const auto tilt = fixture_numbers(pbf, tilted);
        print_numbers(pbf, "tilted bumpy DEM", tilt);
        for (const auto& n : {flat, tilt}) {
            CHECK(n.surfaces > 200);
            CHECK(n.no_surface == 0);
            CHECK(n.worst_match <= 1e-9);
            CHECK(n.worst_slope <= 1e-4);
            CHECK(n.bound_bad == 0);
            // T6: the G3 continuity criterion (< 1 cm) and the quantisation bound (1/256 m).
            CHECK(n.worst_node_mismatch <= 1.0 / 256.0);
            CHECK(n.node_over_limit == 0);
        }
    }
}

// ---- raised junction tables and feature suppression (design 4.4) ----

namespace {
bool has(const RoadJunction& j, JunctionFlags f) { return (j.flags & static_cast<std::uint16_t>(f)) != 0; }
const Tags kTable = {{"traffic_calming", "table"}};
}

TEST_CASE("T5: a table or hump on a junction node raises the junction; other vertical kinds raise nothing", "[layer][junction][height]") {
    {
        JunctionBuildStats stats;
        const auto j = junction_of(*x_scene(kTable), 1, &stats);
        CHECK(has(j, JunctionFlags::RaisedTable));
        CHECK(j.raised_height_m == Approx(0.08));
        CHECK(j.raised_node == 1);
        CHECK(stats.raised_tables == 1);
    }
    {
        const auto j = junction_of(*x_scene({{"traffic_calming", "hump"}}), 1);
        CHECK(has(j, JunctionFlags::RaisedTable)); CHECK(j.raised_height_m == Approx(0.07));
    }
    for (const char* kind : {"bump", "cushion", "mini_bumps", "rumble_strip", "dip"}) {
        INFO(kind);
        JunctionBuildStats stats;
        const auto j = junction_of(*x_scene({{"traffic_calming", kind}}), 1, &stats);
        CHECK_FALSE(has(j, JunctionFlags::RaisedTable)); CHECK(j.raised_height_m == 0.0);
        CHECK(stats.raised_tables == 0); CHECK(stats.calming_ignored == 1);
    }
    {   // Render-only kinds are no vertical feature at all: not counted.
        JunctionBuildStats stats;
        const auto j = junction_of(*x_scene({{"traffic_calming", "chicane"}}), 1, &stats);
        CHECK_FALSE(has(j, JunctionFlags::RaisedTable)); CHECK(stats.calming_ignored == 0);
    }
    {   // The switch of the tests.
        JunctionBuildOptions off; off.raised_tables = false;
        const auto j = junction_of(*x_scene(kTable), 1, nullptr, off);
        CHECK_FALSE(has(j, JunctionFlags::RaisedTable));
    }
}

namespace {
// Points across the whole outer width at the trim line of approach `a`.
struct TrimProbe { double s, t, x, y; };
std::vector<TrimProbe> trim_probes(const RoadGeomTile& tile, const JunctionSurfaceApproach& a, int count) {
    const auto& e = tile.entries[a.entry_index];
    const double s = a.at_end ? a.length_m - a.trim_m : a.trim_m;
    const auto f = carve_footprint(*e.cross_section, s);
    std::vector<TrimProbe> out;
    for (int k = 0; k < count; ++k) {
        const double u = static_cast<double>(k) / static_cast<double>(count - 1);
        const double t = f.outer_right + u * (f.outer_left - f.outer_right);
        const auto p = lateral_point(e.profile->reference, s, t); REQUIRE(p);
        out.push_back({s, t, p->x, p->y});
    }
    return out;
}
}

TEST_CASE("T5: a raised junction meets the raised approach cores on every trim line, sidewalks included", "[layer][junction][height]") {
    const auto scene = x_scene(kTable);
    const auto j = junction_of(*scene, 1);
    REQUIRE(has(j, JunctionFlags::RaisedTable));
    const auto adjust = adjust_features_for_junctions(scene->tile, std::span<const RoadJunction>(&j, 1));
    REQUIRE(adjust.ramps.size() == 4);
    CHECK(adjust.suppressed == 0);
    RoadGeomTile kept = scene->tile; kept.features = adjust.features;
    RoadGeomTile core = scene->tile; core.features = adjust.core_features();
    REQUIRE(core.features.size() == adjust.features.size() + 4);
    for (std::size_t k = 1; k < core.features.size(); ++k)
        CHECK(std::tie(core.features[k - 1].entry_index, core.features[k - 1].station_m) <= std::tie(core.features[k].entry_index, core.features[k].station_m));
    const auto surface = make_junction_surface(kept, j); REQUIRE(surface);
    const auto unraised_scene = x_scene({});
    const auto unraised_junction = junction_of(*unraised_scene, 1);
    const auto plain = make_junction_surface(unraised_scene->tile, unraised_junction); REQUIRE(plain);
    double worst = 0, worst_if_constant = 0; std::size_t carriageway_points = 0, edge_points = 0;
    for (const auto& a : surface->approaches) {
        REQUIRE(a.has_ramp);
        const auto& e = scene->tile.entries[a.entry_index];
        for (const auto& p : trim_probes(core, a, 9)) {
            const auto f = carve_footprint(*e.cross_section, p.s);
            const double z_core = road_surface_height_m(core, a.entry_index, p.s, p.t);
            const double z_j = junction_height_m(*surface, kept, p.x, p.y);
            worst = std::max(worst, std::abs(z_j - z_core));
            const bool on_carriageway = p.t >= f.carriageway_right && p.t <= f.carriageway_left;
            carriageway_points += on_carriageway ? 1 : 0;
            if (!on_carriageway) {
                // The design's literal rule (the whole junction raised by a constant) would step by the full height here.
                ++edge_points;
                worst_if_constant = std::max(worst_if_constant, std::abs(z_j + j.raised_height_m - z_core));
            }
        }
    }
    std::printf("T5 raised table: worst |z_J - raised core| over the 4 trim lines %.3e m; %zu carriageway and %zu outer points; a constant raise would step %.3f m on the sidewalk\n",
        worst, carriageway_points, edge_points, worst_if_constant);
    CHECK(worst <= 1e-5);     // the projection of a mm-rounded segment: weights differ by ~1e-7 at 0.5 mm off the line
    CHECK(carriageway_points > 8); CHECK(edge_points >= 8);
    CHECK(worst_if_constant == Approx(0.08).margin(1e-5));   // the sabotage: would fail the match by the table height
    // The raise itself: the junction centre sits one table height above the unraised junction.
    const double dz = junction_height_m(*surface, kept, kOriginE, kOriginN) - junction_height_m(*plain, unraised_scene->tile, kOriginE, kOriginN);
    CHECK(dz == Approx(0.08).margin(1e-6));
}

TEST_CASE("T5: the ramp record is a plateau up to the trim line and one 10 x height ramp beyond it", "[layer][junction][height]") {
    const auto scene = x_scene(kTable);
    const auto j = junction_of(*scene, 1);
    const auto adjust = adjust_features_for_junctions(scene->tile, std::span<const RoadJunction>(&j, 1));
    REQUIRE(adjust.ramps.size() == j.approaches.size());
    for (const auto& a : j.approaches) {
        const double length = scene->tile.entries[a.entry_index].profile->reference.length_m;
        // The ramp of this approach end: the record on this entry whose centre lies on the matching side.
        const ProfileFeature* found = nullptr;
        for (const auto& f : adjust.ramps) if (f.entry_index == a.entry_index && (f.station_m < length / 2) == !a.at_end) found = &f;
        REQUIRE(found != nullptr);
        const ProfileFeature& f = *found;
        CHECK(f.kind == ProfileFeatureKind::Table); CHECK(f.carve); CHECK(f.node_id == 1); CHECK(f.height_mm == 80);
        const auto at = [&](double sigma) { return feature_height_contribution_m(f, a.at_end ? length - sigma : sigma); };
        CHECK(at(0.0) == Approx(0.08).margin(1e-9));
        CHECK(at(a.trim_m * 0.5) == Approx(0.08).margin(1e-9));
        CHECK(at(a.trim_m) == Approx(0.08).margin(1e-9));              // the plateau ends exactly at the trim line
        CHECK(at(a.trim_m + 0.5) < 0.08); CHECK(at(a.trim_m + 0.5) > 0.0);
        CHECK(at(a.trim_m + 1.0) == Approx(0.0).margin(1e-9));          // 10 x 0.08 m = 0.8 m, clamped to the 1 m minimum ramp
    }
}

TEST_CASE("T5: a feature on an approach inside the ring is suppressed, one beyond it is kept", "[layer][junction][height]") {
    // The trim of the west arm of the X comes from a first build; the bump node is then placed 0.3 m inside the trim
    // (suppressed: its 2 m footprint reaches into the ring) or 4 m beyond it (kept).
    const auto probe = x_scene();
    const auto jp = junction_of(*probe, 1);
    double trim = 0;
    for (const auto& a : jp.approaches) trim = std::max(trim, a.trim_m);
    REQUIRE(trim > 2.0);
    for (const bool close_in : {true, false}) {
        const double distance = close_in ? trim - 0.3 : trim + 4.0;
        const auto scene = make_scene({{1, 0, 0}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}, {5, 0, 300}, {6, -distance, 0, {{"traffic_calming", "bump"}}}},
            {{10, {2, 6, 1, 3}}, {11, {4, 1, 5}}});
        const auto j = junction_of(*scene, 1);
        REQUIRE(scene->tile.features.size() == 1);
        const auto adjust = adjust_features_for_junctions(scene->tile, std::span<const RoadJunction>(&j, 1));
        INFO((close_in ? "inside the ring" : "beyond the ring"));
        CHECK(adjust.suppressed == (close_in ? 1u : 0u));
        CHECK(adjust.features.size() == (close_in ? 0u : 1u));
        // Without the suppression the feature would leave a step on the trim line: its height at the trim station.
        const ProfileFeature& f = scene->tile.features[0];
        const auto& entry = scene->tile.entries[f.entry_index];
        const double length = entry.profile->reference.length_m;
        double trim_of_entry = 0;
        for (const auto& a : j.approaches) if (a.entry_index == f.entry_index) trim_of_entry = a.trim_m;
        const double at_trim = feature_height_contribution_m(f, f.station_m < length / 2 ? trim_of_entry : length - trim_of_entry);
        if (close_in) { CHECK(at_trim > 0.01); } else { CHECK(at_trim == 0.0); }
    }
}

TEST_CASE("T5 fixtures: tables and calming on the real junction nodes", "[layer][junction][height][roads]") {
    // Counts: the fixtures' junction nodes with a table or hump, with another vertical kind, the ramps and the suppressed features.
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto n = fixture_numbers(pbf, {});
        std::printf("[junction calming] %s: raised tables %zu, other vertical kinds ignored %zu, ramp records %zu, suppressed features %zu\n", pbf, n.raised, n.ignored, n.ramps, n.suppressed);
        CHECK(n.ramps >= n.raised);
    }
}

TEST_CASE("T6: the node agreement check sees a stretch whose profile is off by 2 cm", "[layer][junction][height]") {
    auto scene = x_scene();
    const auto j = junction_of(*scene, 1);
    const auto spread = [&](const RoadGeomTile& tile) {
        const auto surface = make_junction_surface(tile, j); REQUIRE(surface);
        double lo = 1e18, hi = -1e18;
        for (std::size_t i = 0; i < surface->approaches.size(); ++i) {
            const auto& a = surface->approaches[i];
            const double z = junction_approach_height_m(*surface, tile, i, a.ref_x[0], a.ref_y[0]);
            lo = std::min(lo, z); hi = std::max(hi, z);
        }
        return hi - lo;
    };
    CHECK(spread(scene->tile) <= 1e-9);
    // Sabotage: lift one approach's whole profile by 2 cm (what a completion refit of one stretch only would do).
    RoadGeomTile lifted = scene->tile;
    for (auto& segment : lifted.entries[j.approaches[0].entry_index].profile->vertical.segments) { segment.height0_m += 0.02; segment.height1_m += 0.02; }
    CHECK(spread(lifted) == Approx(0.02).margin(1e-9));
}
