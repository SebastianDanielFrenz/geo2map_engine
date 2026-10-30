// Lattice drape of the road ribbons (vault G2M-025): ribbon triangles are cut on the terrain mesh's own triangulation (the 1 m
// lattice of DGM1 cell centres at absolute x.5 m, every cell split along the diagonal joining (c+1, r) and (c, r+1)) so that no
// terrain triangle stands above the road. The reference below is an independent evaluation of that triangle surface.
//
// Recorded sabotages (2026-10-20; each makes the named tests fail, then reverted):
//   S1 lattice_drape.cpp: do not split a cell along its diagonal (the whole cell is the SW triangle)      -> 6 of 11 cases fail
//        (surface, raise, tolerance bound, seam, strip heights, ribbons-never-below)
//   S2 lattice_drape.cpp: drop the straight-edge height override in the cut path                           -> "no crack on the shared edge"
//   S3 lattice_drape.cpp: omit the interior-node test of the uncut shortcut                               -> "a bump no edge sees", "no crack"
//   S4 lattice_drape.cpp: read the ground 1 cm off the lattice node                                       -> 6 cases fail (incl. "only at lattice nodes")
//   S6 lattice_drape.cpp add_ramp: a piece vertex takes the plane height, not max(plane, terrain)                 -> "deck-to-ground ramp"
//   S5 road_ribbons.cpp: build the strips with the point-sampled drape whatever the option says           -> "ribbons never sit below"
#include "road_mesh_test_data.h"

#include "world/lattice_drape.h"

#include "g2m/world/road_ribbons.h"
#include "g2m/world/road_structures.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <tuple>
#include <vector>

namespace {

using g2m::world::RoadRibbonTile;
using g2m::world::detail::DrapeVertex;
using g2m::world::detail::LatticeDrape;

// Bumpy terrain: wavelength ~5 m, 0.8 m amplitude, so every 1 m cell bends and the banks are steep.
double bumpy(double e, double n) { return 100.0 + 0.8 * std::sin(e * 1.3) * std::cos(n * 0.9) + 0.05 * (e - std::floor(e / 50.0) * 50.0); }

// The terrain triangle surface, evaluated independently of the implementation (nodes at absolute x.5, y.5).
template <class Ground>
double exact_height(const Ground& g, double e, double n) {
    const double c = std::floor(e - 0.5), r = std::floor(n - 0.5);
    const double tx = e - (c + 0.5), ty = n - (r + 0.5);
    const auto h = [&](double dc, double dr) { return g(c + dc + 0.5, r + dr + 0.5); };
    if (tx + ty <= 1.0) return h(0, 0) + tx * (h(1, 0) - h(0, 0)) + ty * (h(0, 1) - h(0, 0));
    return h(1, 1) + (1.0 - tx) * (h(0, 1) - h(1, 1)) + (1.0 - ty) * (h(1, 0) - h(1, 1));
}

DrapeVertex dv(double x, double y) {
    DrapeVertex v;
    v.x = x;
    v.y = y;
    v.a = {x * 0.1, y * 0.1, 3.5, 0.0, 2.0, 1.0, 0.0, 0.0};
    return v;
}

constexpr double kOx = 456000.0, kOy = 5421000.0;
const g2m::world::RoadRibbonGround kBumpy = [](double e, double n) -> std::optional<double> { return bumpy(e, n); };

// Barycentric probe set of a triangle: corners excluded (shared, tested through the neighbours), 12 interior points.
template <class Fn>
void probe_triangle(const RoadRibbonTile& m, std::size_t i, Fn&& fn) {
    const auto& p = m.vertices[static_cast<std::size_t>(m.indices[i])];
    const auto& q = m.vertices[static_cast<std::size_t>(m.indices[i + 1])];
    const auto& r = m.vertices[static_cast<std::size_t>(m.indices[i + 2])];
    for (const double a : {0.1, 0.3, 0.5, 0.7}) {
        for (const double b : {0.1, 0.25, 0.4}) {
            if (a + b >= 0.95) continue;
            const double c = 1.0 - a - b;
            fn(a * p.x + b * q.x + c * r.x, a * p.y + b * q.y + c * r.y, a * p.z + b * q.z + c * r.z);
        }
    }
}

std::vector<std::array<double, 6>> random_triangles(int count, std::uint32_t seed, double span) {
    std::vector<std::array<double, 6>> out;
    std::uint32_t s = seed;
    const auto rnd = [&] { s = s * 1664525u + 1013904223u; return static_cast<double>(s >> 8) / 16777216.0; };
    for (int i = 0; i < count; ++i) {
        const double x = 100.0 + rnd() * 400.0, y = 100.0 + rnd() * 400.0;
        std::array<double, 6> t{x, y, x + (rnd() - 0.2) * span, y + (rnd() - 0.5) * span, x + (rnd() - 0.5) * span, y + (rnd() - 0.2) * span};
        const double det = (t[2] - t[0]) * (t[5] - t[1]) - (t[4] - t[0]) * (t[3] - t[1]);
        if (det < 0) { std::swap(t[2], t[4]); std::swap(t[3], t[5]); }
        out.push_back(t);
    }
    return out;
}

} // namespace

TEST_CASE("Lattice drape: every piece lies on the terrain triangle surface", "[world][road_ribbons][ribbon_drape]") {
    LatticeDrape drape(kBumpy, kOx, kOy, 0.0); // tolerance 0: everything is cut to the exact surface
    RoadRibbonTile mesh;
    for (const auto& t : random_triangles(60, 7, 14.0)) drape.add(mesh, dv(t[0], t[1]), dv(t[2], t[3]), dv(t[4], t[5]), 0.0);
    REQUIRE(mesh.indices.size() > 60 * 3);
    CHECK(drape.stats().pieces > drape.stats().triangles_in); // cuts happened
    double worst = 0;
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
        probe_triangle(mesh, i, [&](double x, double y, double z) { worst = std::max(worst, std::abs(z - exact_height(bumpy, kOx + x, kOy + y))); });
    CHECK(worst < 2e-4); // float vertices at ~100 m
    // Winding is kept: every piece is counter-clockwise like the input.
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3) {
        const auto& p = mesh.vertices[static_cast<std::size_t>(mesh.indices[i])], &q = mesh.vertices[static_cast<std::size_t>(mesh.indices[i + 1])],
                    &r = mesh.vertices[static_cast<std::size_t>(mesh.indices[i + 2])];
        CHECK((q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x) > 0.0f);
    }
}

TEST_CASE("Lattice drape: the raise lifts every vertex by exactly that much", "[world][road_ribbons][ribbon_drape]") {
    LatticeDrape drape(kBumpy, kOx, kOy, 0.0);
    RoadRibbonTile mesh;
    drape.add(mesh, dv(120.2, 130.3), dv(131.7, 133.1), dv(125.4, 141.9), 0.018);
    REQUIRE_FALSE(mesh.indices.empty());
    for (const auto& v : mesh.vertices) CHECK(std::abs(static_cast<double>(v.z) - 0.018 - exact_height(bumpy, kOx + v.x, kOy + v.y)) < 2e-4);
}

TEST_CASE("Lattice drape: a quad strip is watertight (no cracks, no T-junctions)", "[world][road_ribbons][ribbon_drape]") {
    // A skewed strip of quads, each cut into two triangles the way the ribbon builder does; neighbours share their edges.
    LatticeDrape drape(kBumpy, kOx, kOy, 0.0);
    RoadRibbonTile mesh;
    std::vector<std::pair<DrapeVertex, DrapeVertex>> rail;
    for (int i = 0; i <= 12; ++i) {
        const double s = 150.0 + i * 3.37;
        rail.push_back({dv(s, 200.0 + 0.41 * (s - 150.0) + std::sin(i * 0.7)), dv(s + 0.6, 207.3 + 0.41 * (s - 150.0) + std::sin(i * 0.7))});
    }
    for (std::size_t i = 0; i + 1 < rail.size(); ++i) {
        const auto &a = rail[i].first, &b = rail[i].second, &a1 = rail[i + 1].first, &b1 = rail[i + 1].second;
        drape.add(mesh, a, b, a1, 0.0);
        drape.add(mesh, a1, b, b1, 0.0);
    }
    // Every undirected edge is used by exactly two triangles (opposite directions), except the strip's outline.
    std::map<std::pair<int, int>, int> directed;
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
        for (int k = 0; k < 3; ++k) ++directed[{mesh.indices[i + static_cast<std::size_t>(k)], mesh.indices[i + static_cast<std::size_t>((k + 1) % 3)]}];
    int open_edges = 0, interior_open = 0;
    for (const auto& [e, n] : directed) {
        CHECK(n == 1); // no edge used twice in the same direction
        if (directed.count({e.second, e.first})) continue;
        ++open_edges;
        // An open edge must lie on the strip's outline: its midpoint is within 1e-4 of a rail segment or an end cap.
        const auto& p = mesh.vertices[static_cast<std::size_t>(e.first)], &q = mesh.vertices[static_cast<std::size_t>(e.second)];
        const double mx = (static_cast<double>(p.x) + q.x) / 2, my = (static_cast<double>(p.y) + q.y) / 2;
        bool on_outline = false;
        for (std::size_t i = 0; i + 1 < rail.size() && !on_outline; ++i)
            for (const auto& [u, w] : {std::pair{rail[i].first, rail[i + 1].first}, std::pair{rail[i].second, rail[i + 1].second}, std::pair{rail[i].first, rail[i].second}, std::pair{rail[i + 1].first, rail[i + 1].second}}) {
                const double dx = w.x - u.x, dy = w.y - u.y, len2 = dx * dx + dy * dy;
                const double t = std::clamp(((mx - u.x) * dx + (my - u.y) * dy) / len2, 0.0, 1.0);
                if (std::hypot(u.x + t * dx - mx, u.y + t * dy - my) < 1e-3) { on_outline = true; break; }
            }
        if (!on_outline) ++interior_open;
    }
    CHECK(open_edges > 0);
    CHECK(interior_open == 0);
}

TEST_CASE("Lattice drape: tiles agree on a shared edge, however each side cuts it", "[world][road_ribbons][ribbon_drape]") {
    // Tile A (origin 0) and tile B (origin 1024 east) both drape the same world-space edge x = kOx + 1024 from different
    // triangles. The vertices they put on that edge must have the same heights.
    const auto ground = kBumpy;
    LatticeDrape a(ground, kOx, kOy, 0.0), b(ground, kOx + 1024.0, kOy, 0.0);
    RoadRibbonTile ma, mb;
    // A: triangle left of the edge, with a corner on it and one far away; B: a triangle to the right sharing the edge.
    a.add(ma, dv(1020.3, 100.2), dv(1024.0, 100.2), dv(1024.0, 109.7), 0.0);
    a.add(ma, dv(1019.1, 100.2), dv(1024.0, 109.7), dv(1018.4, 111.0), 0.0);
    b.add(mb, dv(0.0, 100.2), dv(5.1, 103.0), dv(0.0, 109.7), 0.0);
    std::map<long long, double> on_a, on_b;
    for (const auto& v : ma.vertices) if (std::abs(v.x - 1024.0f) < 1e-4f) on_a[std::llround(v.y * 1e4)] = v.z;
    for (const auto& v : mb.vertices) if (std::abs(v.x) < 1e-4f) on_b[std::llround(v.y * 1e4)] = v.z;
    REQUIRE_FALSE(on_a.empty());
    REQUIRE_FALSE(on_b.empty());
    // Every vertex of one side sits on the other side's edge polyline: interpolate B's line at A's y and compare.
    const auto line_at = [](const std::map<long long, double>& m, long long y) -> std::optional<double> {
        const auto hi = m.lower_bound(y);
        if (hi == m.end() || hi == m.begin()) return hi != m.end() && hi->first == y ? std::optional<double>(hi->second) : std::nullopt;
        if (hi->first == y) return hi->second;
        const auto lo = std::prev(hi);
        return lo->second + (hi->second - lo->second) * static_cast<double>(y - lo->first) / static_cast<double>(hi->first - lo->first);
    };
    int compared = 0;
    for (const auto& [y, z] : on_a)
        if (const auto other = line_at(on_b, y)) { CHECK(*other == Catch::Approx(z).margin(5e-5)); ++compared; }
    CHECK(compared >= 2);
    // And both equal the exact terrain along the edge.
    for (const auto& [y, z] : on_a) CHECK(z == Catch::Approx(exact_height(bumpy, kOx + 1024.0, kOy + static_cast<double>(y) * 1e-4)).margin(2e-4));
}

namespace {
// Largest height gap at a T-junction: a vertex that lies on the interior of another triangle's edge must lie on that edge's
// line, or the two surfaces leave a crack there.
double worst_t_junction_gap(const RoadRibbonTile& m) {
    double worst = 0;
    for (std::size_t i = 0; i < m.indices.size(); i += 3)
        for (int k = 0; k < 3; ++k) {
            const auto ia = m.indices[i + static_cast<std::size_t>(k)], ib = m.indices[i + static_cast<std::size_t>((k + 1) % 3)];
            const auto &p = m.vertices[static_cast<std::size_t>(ia)], &q = m.vertices[static_cast<std::size_t>(ib)];
            const double dx = static_cast<double>(q.x) - p.x, dy = static_cast<double>(q.y) - p.y, len2 = dx * dx + dy * dy;
            if (len2 < 1e-12) continue;
            for (std::size_t v = 0; v < m.vertices.size(); ++v) {
                if (static_cast<std::int32_t>(v) == ia || static_cast<std::int32_t>(v) == ib) continue;
                const auto& w = m.vertices[v];
                const double t = ((static_cast<double>(w.x) - p.x) * dx + (static_cast<double>(w.y) - p.y) * dy) / len2;
                if (t <= 1e-6 || t >= 1 - 1e-6) continue;
                if (std::abs((static_cast<double>(w.x) - p.x) * dy - (static_cast<double>(w.y) - p.y) * dx) / std::sqrt(len2) > 1e-4) continue;
                worst = std::max(worst, std::abs(static_cast<double>(w.z) - (p.z + (static_cast<double>(q.z) - p.z) * t)));
            }
        }
    return worst;
}
} // namespace

TEST_CASE("Lattice drape: a cut neighbour of an uncut triangle leaves no crack on the shared edge", "[world][road_ribbons][ribbon_drape]") {
    // Mild terrain (amplitude 12 mm): every edge is straight within the 30 mm tolerance. One spike node inside the first
    // triangle forces it to be cut; the second, across the shared edge, stays whole. The cut side's vertices on the shared
    // edge must carry the edge's line height, not their own terrain height, or a gap of up to the tolerance opens.
    const double spike_e = kOx + 400.5, spike_n = kOy + 403.5;
    const g2m::world::RoadRibbonGround mild = [&](double e, double n) -> std::optional<double> {
        const double base = 100.0 + 0.012 * std::sin(e * 1.3) * std::cos(n * 0.9);
        return (std::abs(e - spike_e) < 1e-9 && std::abs(n - spike_n) < 1e-9) ? base + 0.5 : base;
    };
    LatticeDrape drape(mild, kOx, kOy, 0.03);
    RoadRibbonTile mesh;
    const auto a = dv(394.3, 398.2), b = dv(406.9, 399.1), c = dv(399.4, 409.6), d = dv(412.3, 410.2);
    drape.add(mesh, a, b, c, 0.0); // contains the spike node (400.5, 403.5) strictly inside
    drape.add(mesh, c, b, d, 0.0); // shares the edge b-c, flat
    REQUIRE(drape.stats().pieces > 2);
    CHECK(drape.stats().uncut >= 1);
    CHECK(worst_t_junction_gap(mesh) < 1e-4);
}

TEST_CASE("Lattice drape: a bump no edge sees is still cut out of an otherwise straight triangle", "[world][road_ribbons][ribbon_drape]") {
    // Flat terrain except one node 0.5 m high well inside the triangle; the three edges stay clear of it, so only the
    // interior-node test can see it.
    const double spike_e = kOx + 400.5, spike_n = kOy + 403.5;
    const g2m::world::RoadRibbonGround spike = [&](double e, double n) -> std::optional<double> {
        return (std::abs(e - spike_e) < 1e-9 && std::abs(n - spike_n) < 1e-9) ? 100.5 : 100.0;
    };
    LatticeDrape drape(spike, kOx, kOy, 0.008);
    RoadRibbonTile mesh;
    drape.add(mesh, dv(394.3, 398.2), dv(406.9, 399.1), dv(399.4, 409.6), 0.0);
    CHECK(drape.stats().uncut == 0);
    double worst = 0;
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
        probe_triangle(mesh, i, [&](double x, double y, double z) { worst = std::max(worst, std::abs(z - exact_height(
            [&](double e, double n) { return *spike(e, n); }, kOx + x, kOy + y))); });
    CHECK(worst <= 0.008 + 2e-4);
    bool peak = false;
    for (const auto& v : mesh.vertices) peak = peak || std::abs(v.z - 100.5f) < 1e-3f;
    CHECK(peak); // the spike node itself became a vertex
}

TEST_CASE("Lattice drape: a deck-to-ground ramp keeps the deck edge and never falls below the terrain", "[world][road_ribbons][ribbon_drape]") {
    // A quad one metre long whose first row (y = 200.3) is on a deck pinned at 101.0 m and whose second row (y = 201.3) is on the
    // ground. A ridge of nodes at y = 200.5 (101.0 m, the rest 100.0 m) stands over the straight line between the two rows.
    const auto ridge = [](double, double n) { return std::abs(n - (kOy + 200.5)) < 1e-9 ? 101.0 : 100.0; };
    const g2m::world::RoadRibbonGround ground = [&](double e, double n) -> std::optional<double> { return ridge(e, n); };
    LatticeDrape drape(ground, kOx, kOy, 0.008);
    RoadRibbonTile mesh;
    std::vector<DrapeVertex> row0, row1;
    for (int j = 0; j <= 4; ++j) {
        auto d = dv(150.3 + j, 200.3);
        d.pin = 101.0;
        row0.push_back(d);
        row1.push_back(dv(150.3 + j, 201.3));
    }
    for (std::size_t j = 0; j < 4; ++j) {
        drape.add_ramp(mesh, row0[j], row1[j], row0[j + 1], 0.0);
        drape.add_ramp(mesh, row0[j + 1], row1[j], row1[j + 1], 0.0);
    }
    REQUIRE_FALSE(mesh.indices.empty());
    double buried = -1e9;
    for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
        probe_triangle(mesh, i, [&](double x, double y, double z) { buried = std::max(buried, exact_height(ridge, kOx + x, kOy + y) - z); });
    CHECK(buried <= 2e-4); // with the plane through the corners the ridge would stand 0.16 m over the ramp
    for (const auto& v : mesh.vertices) {
        if (std::abs(v.y - 200.3f) < 1e-4f) CHECK(v.z == Catch::Approx(101.0).margin(1e-4)); // the deck edge is the deck's line
        if (std::abs(v.y - 201.3f) < 1e-4f) CHECK(v.z == Catch::Approx(exact_height(ridge, kOx + v.x, kOy + v.y)).margin(2e-4)); // the ground edge is the terrain
    }
}

TEST_CASE("Lattice drape: the ground is read only at lattice nodes", "[world][road_ribbons][ribbon_drape]") {
    std::set<std::pair<long long, long long>> offenders;
    long long reads = 0;
    const g2m::world::RoadRibbonGround counting = [&](double e, double n) -> std::optional<double> {
        ++reads;
        const double fe = e - std::floor(e), fn = n - std::floor(n);
        if (fe != 0.5 || fn != 0.5) offenders.insert({std::llround(e * 1000), std::llround(n * 1000)});
        return bumpy(e, n);
    };
    LatticeDrape drape(counting, kOx, kOy, 0.004);
    RoadRibbonTile mesh;
    for (const auto& t : random_triangles(40, 11, 12.0)) drape.add(mesh, dv(t[0], t[1]), dv(t[2], t[3]), dv(t[4], t[5]), 0.0);
    CHECK(reads > 0);
    CHECK(offenders.empty());
    CHECK(drape.stats().node_reads == static_cast<std::size_t>(reads)); // the counter agrees with the reads
}

TEST_CASE("Lattice drape: the tolerance bound holds and relaxes the cut", "[world][road_ribbons][ribbon_drape]") {
    const auto tris = random_triangles(80, 3, 10.0);
    const auto run = [&](double tol, double& worst) {
        LatticeDrape drape(kBumpy, kOx, kOy, tol);
        RoadRibbonTile mesh;
        for (const auto& t : tris) drape.add(mesh, dv(t[0], t[1]), dv(t[2], t[3]), dv(t[4], t[5]), 0.0);
        worst = 0;
        for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
            probe_triangle(mesh, i, [&](double x, double y, double z) { worst = std::max(worst, std::abs(z - exact_height(bumpy, kOx + x, kOy + y))); });
        return mesh.indices.size() / 3;
    };
    double w0 = 0, w8 = 0, w50 = 0;
    const auto n0 = run(0.0, w0), n8 = run(0.008, w8), n50 = run(0.05, w50);
    CHECK(w8 <= 0.008 + 2e-4);
    CHECK(w50 <= 0.05 + 2e-4);
    CHECK(n8 <= n0);
    CHECK(n50 <= n8);
    // Flat ground: nothing to cut at any tolerance, every input triangle comes out whole.
    const g2m::world::RoadRibbonGround flat = [](double, double) -> std::optional<double> { return 100.0; };
    LatticeDrape drape(flat, kOx, kOy, 0.008);
    RoadRibbonTile mesh;
    for (const auto& t : tris) drape.add(mesh, dv(t[0], t[1]), dv(t[2], t[3]), dv(t[4], t[5]), 0.0);
    CHECK(drape.stats().pieces == drape.stats().triangles_in);
    CHECK(drape.stats().uncut == drape.stats().pieces);
    // A per-call tolerance tighter than the constructor's wins (marking layers).
    LatticeDrape tight(kBumpy, kOx, kOy, 0.05);
    RoadRibbonTile m2;
    for (const auto& t : tris) tight.add(m2, dv(t[0], t[1]), dv(t[2], t[3]), dv(t[4], t[5]), 0.0, 0.001);
    double w1 = 0;
    for (std::size_t i = 0; i < m2.indices.size(); i += 3)
        probe_triangle(m2, i, [&](double x, double y, double z) { w1 = std::max(w1, std::abs(z - exact_height(bumpy, kOx + x, kOy + y))); });
    CHECK(w1 <= 0.001 + 2e-4);
}

TEST_CASE("Lattice drape: pieces over a terrain triangle without ground are dropped, the rest stays", "[world][road_ribbons][ribbon_drape]") {
    // NoData at the nodes of one cell column (x in [200.5, 201.5)): triangles touching it lose that part only.
    const g2m::world::RoadRibbonGround holed = [](double e, double n) -> std::optional<double> {
        if (std::abs(e - (kOx + 200.5)) < 1e-9 && n > kOy + 100.0 && n < kOy + 120.0) return std::nullopt;
        return bumpy(e, n);
    };
    LatticeDrape drape(holed, kOx, kOy, 0.0);
    RoadRibbonTile mesh;
    drape.add(mesh, dv(195.2, 105.1), dv(207.8, 108.3), dv(199.9, 116.6), 0.0);
    CHECK(drape.stats().dropped > 0);
    CHECK_FALSE(mesh.indices.empty());
    for (const auto& v : mesh.vertices) CHECK(std::isfinite(v.z));
    // Fully outside the hole: nothing dropped.
    LatticeDrape clear(holed, kOx, kOy, 0.0);
    RoadRibbonTile m2;
    clear.add(m2, dv(300.2, 105.1), dv(307.8, 108.3), dv(299.9, 116.6), 0.0);
    CHECK(clear.stats().dropped == 0);
}

namespace {
// Terrain minus ribbon over every probe of the ribbon's triangles (absolute heights; ribbon z is tile-local absolute).
double worst_buried(const RoadRibbonTile& m, double ox, double oy) {
    double worst = -1e9;
    for (std::size_t i = 0; i < m.indices.size(); i += 3)
        probe_triangle(m, i, [&](double x, double y, double z) { worst = std::max(worst, exact_height(bumpy, ox + x, oy + y) - z); });
    return worst;
}
} // namespace

TEST_CASE("Lattice drape: ribbons never sit below the terrain surface by more than a centimetre", "[world][road_ribbons][ribbon_drape]") {
    auto tile = g2m_test::junction_scene_tile();
    REQUIRE(static_cast<bool>(tile));
    const auto& key = g2m_test::kJunctionSceneTile;
    REQUIRE(g2m::world::build_road_decks(*tile).empty()); // no bridges: every ribbon triangle is on the ground
    const double ox = static_cast<double>(key.min_easting()), oy = static_cast<double>(key.min_northing());
    g2m::world::RoadRibbonOptions point;
    point.drape = g2m::world::RibbonDrape::PointSampled;
    const auto before = g2m::world::build_road_ribbons(*tile, key, kBumpy, 0.0, g2m::world::RoadSeparation{}, point);
    const auto after = g2m::world::build_road_ribbons(*tile, key, kBumpy, 0.0, g2m::world::RoadSeparation{});
    REQUIRE_FALSE(after.indices.empty());
    // The legacy drape lets the bumpy terrain stand well above the road; the lattice drape holds the 1 cm bound
    // (bed 8 mm, marking layers 3 mm, plus float rounding).
    CHECK(worst_buried(before, ox, oy) > 0.05);
    CHECK(worst_buried(after, ox, oy) <= 0.010);
    CHECK(after.drape_node_reads > 0);
    CHECK(after.drape_out >= after.drape_in - after.drape_dropped);
    CHECK(before.drape_in == 0); // the legacy path leaves the counters alone
    // The junction patches meet the same bound.
    g2m::world::RoadRibbonTile jmesh;
    g2m::world::add_junction_surfaces(jmesh, *tile, key, kBumpy, 0.0);
    if (!jmesh.indices.empty()) CHECK(worst_buried(jmesh, ox, oy) <= 0.010);
}

TEST_CASE("Lattice drape: separated decks stay untouched by the drape", "[world][road_ribbons][ribbon_drape]") {
    // The overpass scene has a deck (bridge); the drape must not change the deck set and must not pull ribbon triangles of the
    // deck stretch onto the terrain: those are not in the ribbon mesh at all (they are carried by the deck).
    for (const char* name : {"overpass.h10.sep", "overpass.h25.sep"}) {
        INFO(name);
        for (const auto& c : g2m_mesh_test::mesh_cases()) {
            if (c.name != name) continue;
            g2m::world::RoadSeparation sep;
            sep.enabled = c.separated;
            g2m::world::RoadRibbonOptions point;
            point.drape = g2m::world::RibbonDrape::PointSampled;
            const auto ground = [&](double e, double n) { return g2m_mesh_test::case_ground(c, e, n); };
            const auto before = g2m::world::build_road_ribbons(c.tile, c.key, ground, c.lift, sep, point);
            const auto after = g2m::world::build_road_ribbons(c.tile, c.key, ground, c.lift, sep);
            // The deck stretches are not draped on the terrain: the highest vertex above the ground (the deck's crown) and the
            // number of vertices of the elevated rows are unchanged by the drape.
            const auto crown = [&](const RoadRibbonTile& m) {
                double top = -1e9;
                for (const auto& v : m.vertices) {
                    const auto g = ground(static_cast<double>(c.key.min_easting()) + v.x, static_cast<double>(c.key.min_northing()) + v.y);
                    if (g) top = std::max(top, static_cast<double>(v.z) - *g);
                }
                return top;
            };
            CHECK(crown(after) > 1.0);
            CHECK(crown(after) == Catch::Approx(crown(before)).margin(1e-3));
        }
    }
}
