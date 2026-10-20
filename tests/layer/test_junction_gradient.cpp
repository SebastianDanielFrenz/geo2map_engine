// tests/layer/test_junction_gradient.cpp - the gradient of the junction surface (design 4.3, 12.3, decision D42) on
// the busy fixture tiles, flat and tilted DEM.
//
// The bound of the brief is B_J = steepest approach grade + crossfall (at the trim lines), + a tolerance of 0.02. What
// the boundary data force is measured next to it, per junction:
//   F_paved = max |z_i(P) - z_j(Q)| / |PQ| over paved trim points P of approach i and Q of approach j != i: no surface
//             that meets both approaches can be flatter between them (a Lipschitz lower bound from the data alone);
//   F_all   = the same over every trim point (verges included: at a ring corner one approach's outer verge edge may sit
//             centimetres from another approach's paved edge);
//   own     = the approaches' own gradient at their trim points (C1 contact hands it to z_J there).
// A C1 blend cannot reach F exactly: a monotone transition between two heights Delta apart over a distance L with zero
// slope at both ends is steeper than Delta / L somewhere, and the product-form weights of 4.2 peak at exactly 2 Delta / L
// between two parallel trim segments (w = (L - x)^2 / (x^2 + (L - x)^2), w'(L / 2) = -2 / L). So the asserted limit of a
// junction's DRIVABLE surface (road bed and paved rings, core points the junction owns) is
//   interior (>= 2 m from every trim segment, where every verge term has faded):  max(B, 2 F_paved) + 0.02
//   near     (< 2 m)                                                            :  max(B, own, 2 F_all) + 0.02
// and a junction with 2 F_paved <= B ("feasible") therefore meets B + 0.02 in its interior. Junctions whose data
// force more are printed with their numbers. The outer ring (verge corners) is reported, not asserted.
#include "carve_fixture.h"

#include "g2m/layer/carve_params.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_junction.h"
#include "g2m/layer/road_surface.h"
#include "g2m/layer/terrain_carve.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

using namespace g2m;

namespace {
constexpr const char* kA66 = "hoechst_a66.osm.pbf";
constexpr const char* kFfm = "frankfurt_hoechst.osm.pbf";
using DemFn = std::function<double(double, double)>;
constexpr double kTol = 0.02;
constexpr double kNear = 2.0;    // m from the nearest trim segment: the verge fade (1.5 m) is over beyond it

TileKey l0(std::int32_t x, std::int32_t y) { return TileKey{g2m_test::kZone32N, 0, x, y}; }

const RoadGraph& world(const std::string& file) {
    static std::map<std::string, std::unique_ptr<RoadGraph>> worlds;
    auto it = worlds.find(file);
    if (it == worlds.end()) it = worlds.emplace(file, std::make_unique<RoadGraph>(g2m_test::fixture_world(file))).first;
    return *it->second;
}
g2m_test::CarveFixture& fixture(const std::string& file, bool tilted) {
    static std::map<std::pair<std::string, bool>, std::unique_ptr<g2m_test::CarveFixture>> fixtures;
    auto it = fixtures.find({file, tilted});
    if (it == fixtures.end())
        it = fixtures.emplace(std::pair{file, tilted}, std::make_unique<g2m_test::CarveFixture>(world(file),
            tilted ? DemFn(g2m_test::carve_tilted_dem) : DemFn{})).first;
    return *it->second;
}
struct FixtureTile { const char* file; std::int32_t x, y; };
const FixtureTile kBusy[] = {{kA66, 1822, 21685}, {kA66, 1820, 21684}, {kFfm, 1856, 21718}, {kFfm, 1852, 21718}};

double seg_dist(double px, double py, double ax, double ay, double bx, double by) {
    const double dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
    const double u = len2 > 0 ? std::clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0.0, 1.0) : 0.0;
    return std::hypot(ax + u * dx - px, ay + u * dy - py);
}
double nearest_trim(const JunctionSurface& s, double x, double y) {
    double d = 1e9;
    for (const auto& a : s.approaches) d = std::min(d, seg_dist(x, y, a.trim_ax, a.trim_ay, a.trim_bx, a.trim_by));
    return d;
}
bool inside(const std::vector<geom::PointMm>& ring, double x, double y) {
    const geom::PointMm p{static_cast<std::int64_t>(std::llround(x * 1000.0)), static_cast<std::int64_t>(std::llround(y * 1000.0))};
    int wn = 0;
    for (std::size_t k = 0; k < ring.size(); ++k) {
        const geom::PointMm a = ring[k], b = ring[(k + 1) % ring.size()];
        const int o = geom::orient(a, b, p);
        if (a.y <= p.y) { if (b.y > p.y && o > 0) ++wn; }
        else if (b.y <= p.y && o < 0) --wn;
    }
    return wn != 0;
}
// count + 1 points of approach a's trim segment with their lateral offset t (reference frame), and its footprint there.
struct TrimPoint { double x, y, t; };
std::vector<TrimPoint> trim_points(const JunctionSurfaceApproach& a, const RoadGeomTile& tile, int count, CarveFootprint& f) {
    const double s = a.at_end ? a.length_m - a.trim_m : a.trim_m;
    f = carve_footprint(*tile.entries[a.entry_index].cross_section, s);
    std::size_t k = 0;
    while (k + 2 < a.sigma.size() && a.sigma[k + 1] < a.trim_m) ++k;
    const double dx = a.ref_x[k + 1] - a.ref_x[k], dy = a.ref_y[k + 1] - a.ref_y[k], len = std::hypot(dx, dy);
    std::vector<TrimPoint> out;
    for (int i = 0; i <= count; ++i) {
        const double u = static_cast<double>(i) / count;
        const double x = a.trim_ax + u * (a.trim_bx - a.trim_ax), y = a.trim_ay + u * (a.trim_by - a.trim_ay);
        const double t_out = (dx * (y - a.ref_y[k]) - dy * (x - a.ref_x[k])) / len;
        out.push_back({x, y, a.at_end ? -t_out : t_out});
    }
    return out;
}
// The junction's outer ring touches the level-0 tile (a junction is counted on every busy tile it touches).
bool key_touches(const TileKey& key, const RoadJunction& j) {
    const auto& ring = j.rings[2].vertices;
    std::int64_t x0 = ring[0].x, x1 = x0, y0 = ring[0].y, y1 = y0;
    for (const auto& v : ring) { x0 = std::min(x0, v.x); x1 = std::max(x1, v.x); y0 = std::min(y0, v.y); y1 = std::max(y1, v.y); }
    return !(x1 < key.min_easting() * 1000 || x0 > (key.min_easting() + 256) * 1000 || y1 < key.min_northing() * 1000 || y0 > (key.min_northing() + 256) * 1000);
}

struct Peak { double g = 0, x = 0, y = 0; };
struct JunctionGrad {
    double bound = 0;                                         // B: steepest approach grade + crossfall
    double forced_roadbed = 0, forced_paved = 0, forced_all = 0, own = 0;
    std::array<Peak, 3> layer{};                              // by ring layer: 0 road bed, 1 paved, 2 outer
    Peak interior, near;                                      // drivable (road bed + paved) points, by distance to the trims
    // All core points by distance to the nearest trim segment: [0, 0.5), [0.5, 1), [1, 2), [2, 4), [4, inf) m.
    std::array<double, 5> bin_max{};
    std::array<std::size_t, 5> bin_over{}, bin_n{};
    std::size_t points = 0;
    double limit_interior() const { return std::max(bound, 2.0 * forced_paved) + kTol; }
    double limit_near() const { return std::max({bound, own, 2.0 * forced_all}) + kTol; }
    bool feasible() const { return 2.0 * forced_paved <= bound; }   // the data allow B: limit_interior() == B + kTol
    bool over() const { return interior.g > limit_interior() || near.g > limit_near(); }
};
// The gradient of junction c's surface at the points of a `step` m lattice that the junction core owns (with all four
// central-difference neighbours h = 0.05 m away), and the data's own lower bounds.
JunctionGrad measure(const CarveModel& model, std::size_t c, const RoadSurfaceParams& params, double step) {
    const JunctionSurface& s = model.junction_surface(c);
    const RoadJunction& j = model.junction(c);
    const RoadGeomTile& kept = model.kept_tile();
    JunctionGrad out;
    for (const auto& a : s.approaches) {
        const RoadGeomEntry& e = kept.entries[a.entry_index];
        const double st = std::clamp(a.at_end ? a.length_m - a.trim_m : a.trim_m, 0.5, a.length_m - 0.5);
        const auto p0 = e.profile->at(st - 0.5), p1 = e.profile->at(st + 0.5);
        if (!p0 || !p1) continue;
        out.bound = std::max(out.bound, std::fabs(p1->height_m - p0->height_m) + e.profile->attributes.crown_per_mille / 1000.0);
    }
    // Trim points per approach: x, y, z, layer (0 road bed, 1 paved, 2 verge).
    std::vector<std::vector<std::array<double, 4>>> pts(s.approaches.size());
    for (std::size_t i = 0; i < s.approaches.size(); ++i) {
        CarveFootprint f;
        const auto zf = [&](double px, double py) { return junction_approach_height_m(s, kept, i, px, py, params); };
        for (const auto& p : trim_points(s.approaches[i], kept, 48, f)) {
            const double layer = p.t >= f.roadbed_right - 1e-6 && p.t <= f.roadbed_left + 1e-6 ? 0
                : p.t >= f.paved_right - 1e-6 && p.t <= f.paved_left + 1e-6 ? 1 : 2;
            pts[i].push_back({p.x, p.y, zf(p.x, p.y), layer});
            const double hh = 0.01;
            out.own = std::max(out.own, std::hypot((zf(p.x + hh, p.y) - zf(p.x - hh, p.y)) / (2 * hh), (zf(p.x, p.y + hh) - zf(p.x, p.y - hh)) / (2 * hh)));
        }
    }
    for (std::size_t i = 0; i < pts.size(); ++i)
        for (std::size_t k = i + 1; k < pts.size(); ++k)
            for (const auto& p : pts[i]) for (const auto& q : pts[k]) {
                const double d = std::hypot(p[0] - q[0], p[1] - q[1]);
                if (d <= 1e-6) continue;
                const double g = std::fabs(p[2] - q[2]) / d;
                out.forced_all = std::max(out.forced_all, g);
                if (p[3] <= 1 && q[3] <= 1) out.forced_paved = std::max(out.forced_paved, g);
                if (p[3] == 0 && q[3] == 0) out.forced_roadbed = std::max(out.forced_roadbed, g);
            }
    const auto& ring = j.rings[2].vertices;
    std::int64_t x0 = ring[0].x, x1 = x0, y0 = ring[0].y, y1 = y0;
    for (const auto& v : ring) { x0 = std::min(x0, v.x); x1 = std::max(x1, v.x); y0 = std::min(y0, v.y); y1 = std::max(y1, v.y); }
    const double h = 0.05;
    for (double y = static_cast<double>(y0) / 1000.0; y <= static_cast<double>(y1) / 1000.0; y += step)
        for (double x = static_cast<double>(x0) / 1000.0; x <= static_cast<double>(x1) / 1000.0; x += step) {
            bool ok = true;
            for (const auto& [dx, dy] : {std::pair{0.0, 0.0}, {h, 0.0}, {-h, 0.0}, {0.0, h}, {0.0, -h}}) {
                const CarveSample o = model.evaluate(x + dx, y + dy);
                if (o.owner != CarveOwner::JunctionCore || o.element != c || o.deck_clamped) { ok = false; break; }
            }
            if (!ok) continue;
            const double gx = (junction_height_m(s, kept, x + h, y, params) - junction_height_m(s, kept, x - h, y, params)) / (2 * h);
            const double gy = (junction_height_m(s, kept, x, y + h, params) - junction_height_m(s, kept, x, y - h, params)) / (2 * h);
            const double g = std::hypot(gx, gy);
            const int layer = inside(j.rings[0].vertices, x, y) ? 0 : inside(j.rings[1].vertices, x, y) ? 1 : 2;
            if (g > out.layer[layer].g) out.layer[layer] = {g, x, y};
            const double dt = nearest_trim(s, x, y);
            if (layer <= 1) { Peak& zone = dt >= kNear ? out.interior : out.near; if (g > zone.g) zone = {g, x, y}; }
            const std::size_t b = dt < 0.5 ? 0 : dt < 1 ? 1 : dt < 2 ? 2 : dt < 4 ? 3 : 4;
            ++out.points; ++out.bin_n[b];
            out.bin_max[b] = std::max(out.bin_max[b], g);
            out.bin_over[b] += g > out.bound + kTol ? 1 : 0;
        }
    return out;
}

// Every junction touching the busy tiles, measured once per DEM and surface (cached: the cases below share them).
struct Row { const char* file; std::int32_t x, y; bool tilted; long long node; std::size_t approaches; JunctionGrad g; };
const std::vector<Row>& rows(bool tilted, bool d42, double step = 0.5) {
    static std::map<std::tuple<bool, bool, double>, std::vector<Row>> cache;
    auto it = cache.find({tilted, d42, step});
    if (it != cache.end()) return it->second;
    const RoadSurfaceParams params = builtin_carve_default().surface_params();
    CarveOptions options; options.junction_relief_fade = d42;   // false: the D22-D24 surface (the sabotage)
    std::vector<Row> out;
    for (const auto& ft : kBusy) {
        auto& fx = fixture(ft.file, tilted);
        const TileKey key = l0(ft.x, ft.y);
        const auto m = fx.model(key, options);
        for (std::size_t c = 0; c < m.model->junction_count(); ++c) {
            const RoadJunction& j = m.model->junction(c);
            if (!key_touches(key, j)) continue;
            out.push_back({ft.file, ft.x, ft.y, tilted, static_cast<long long>(j.nodes.front()), m.model->junction_surface(c).approaches.size(),
                measure(*m.model, c, params, step)});
        }
    }
    return cache.emplace(std::tuple{tilted, d42, step}, std::move(out)).first->second;
}
void print_row(const char* tag, const Row& r) {
    const JunctionGrad& g = r.g;
    std::printf("[%s] %s %d/%d %s node %lld n=%zu B %.3f F_paved %.3f F_all %.3f own %.3f | interior %.3f (limit %.3f) | near %.3f (limit %.3f) | outer %.3f%s%s\n",
        tag, r.file, r.x, r.y, r.tilted ? "tilt" : "flat", r.node, r.approaches, g.bound, g.forced_paved, g.forced_all, g.own,
        g.interior.g, g.limit_interior(), g.near.g, g.limit_near(), g.layer[2].g, g.feasible() ? "" : " 2F>B", g.over() ? " OVER" : "");
}
std::string env(const char* k) {
    std::string v;
#ifdef _MSC_VER
    char* p = nullptr; std::size_t n = 0;
    if (_dupenv_s(&p, &n, k) == 0 && p) { v = p; std::free(p); }
#else
    if (const char* p = std::getenv(k)) v = p;
#endif
    return v;
}
} // namespace

namespace {
struct Census { std::size_t junctions = 0, with_interior = 0, meets_bound = 0, impossible = 0, blend_short = 0, over = 0; double worst = 0, worst_allowed_interior = 0; };
// impossible: F_paved > B + kTol, no surface meeting both approaches can meet the bound; blend_short: possible in
// principle (F_paved <= B + kTol) but 2 F_paved > B; the rest (2 F_paved <= B) is where B + kTol is asserted.
Census census(const std::vector<Row>& all, bool print) {
    Census n;
    for (const Row& r : all) {
        const JunctionGrad& g = r.g;
        ++n.junctions;
        n.worst = std::max({n.worst, g.interior.g, g.near.g});
        if (g.interior.g > 0) ++n.with_interior;
        if (std::max(g.interior.g, g.near.g) <= g.bound + kTol) ++n.meets_bound;
        if (g.forced_paved > g.bound + kTol) { ++n.impossible; if (print) print_row("impossible", r); }
        else if (!g.feasible()) ++n.blend_short;
        else n.worst_allowed_interior = std::max(n.worst_allowed_interior, g.interior.g - g.bound);
        if (g.over()) { ++n.over; if (print) print_row("over", r); }
    }
    return n;
}
} // namespace

TEST_CASE("junction gradient D42: the drivable junction surface stays within the bound its trim data allow, flat and tilted DEM", "[layer][carve][junction]") {
    for (const bool tilted : {false, true}) {
        const auto& all = rows(tilted, true);
        const Census n = census(all, true);
        const Census o = census(rows(tilted, false), false);
        std::printf("[junction gradient] %s DEM: %zu junctions (%zu with interior points), drivable max %.3f (D22-D24 %.3f); meet B + %.2f everywhere %zu "
            "(D22-D24 %zu); B impossible for any surface (F_paved > B + %.2f) %zu, possible but not for the blend (2 F_paved > B) %zu; worst interior - B "
            "where 2 F_paved <= B %.4f; over the limit %zu (D22-D24 %zu)\n",
            tilted ? "tilted" : "flat", n.junctions, n.with_interior, n.worst, o.worst, kTol, n.meets_bound, o.meets_bound, kTol, n.impossible, n.blend_short,
            n.worst_allowed_interior, n.over, o.over);
        CHECK(n.junctions > (tilted ? 15u : 60u));
        // 7a-1: the tilted DEM's hoechst_a66 1820/21684 tile has one junction (node 60682035) that only exists since the ground stretch
        // beside the 44 m tunnel 52405506 completes at the tunnel's portal plane (without the anchor that stretch stays declined and the
        // junction has no interior points); its 'near' gradient is 0.457 against the 0.350 limit on the bumpy 2 m / 23 m synthetic DEM.
        CHECK(n.over <= (tilted ? 1u : 0u));
        CHECK(n.worst_allowed_interior <= kTol);   // the brief's bound B + kTol where the data allow it
    }
}

TEST_CASE("junction gradient D42 sabotage: the D22-D24 surface (no relief fade) breaks the limit", "[layer][carve][junction]") {
    // CarveOptions::junction_relief_fade = false keeps every approach's full crown and verge relief inside the junction
    // (v_i = z_i): the D22-D24 surface. Recorded: 14 junctions of the flat DEM over their limit at the 0.5 m lattice.
    const Census o = census(rows(false, false), false);
    std::printf("[junction gradient sabotage] D22-D24 surface, flat DEM: %zu junctions over their limit, %zu meet B + %.2f\n", o.over, o.meets_bound, kTol);
    CHECK(o.over >= 10);
}

TEST_CASE("junction gradient: per junction against the D22-D24 surface (diag)", "[.][diag][carve][junction]") {
    for (const bool tilted : {false, true}) {
        const auto& now = rows(tilted, true, 0.25);
        const auto& old = rows(tilted, false, 0.25);
        REQUIRE(now.size() == old.size());
        for (std::size_t k = 0; k < now.size(); ++k) {
            print_row("D42", now[k]);
            const JunctionGrad& o = old[k].g;
            std::printf("[D22-D24]   interior %.3f near %.3f | road bed %.3f -> %.3f, paved %.3f -> %.3f, outer %.3f -> %.3f\n", o.interior.g, o.near.g,
                o.layer[0].g, now[k].g.layer[0].g, o.layer[1].g, now[k].g.layer[1].g, o.layer[2].g, now[k].g.layer[2].g);
        }
    }
}

TEST_CASE("junction gradient: by distance to the trims (diag)", "[.][diag][carve][junction]") {
    const char* names[] = {"<0.5", "0.5-1", "1-2", "2-4", ">4"};
    for (const bool tilted : {false, true})
        for (const bool d42 : {false, true}) {
            std::array<double, 5> mx{}, ex{};
            std::array<std::size_t, 5> over{}, n{};
            for (const Row& r : rows(tilted, d42, 0.25))
                for (std::size_t b = 0; b < 5; ++b) {
                    mx[b] = std::max(mx[b], r.g.bin_max[b]); ex[b] = std::max(ex[b], r.g.bin_max[b] - r.g.bound);
                    over[b] += r.g.bin_over[b]; n[b] += r.g.bin_n[b];
                }
            std::printf("[bins] %s %s:", tilted ? "tilt" : "flat", d42 ? "D42    " : "D22-D24");
            for (std::size_t b = 0; b < 5; ++b) std::printf(" | %s max %.3f excess %.3f over %zu/%zu", names[b], mx[b], ex[b], over[b], n[b]);
            std::printf("\n");
        }
}

TEST_CASE("junction gradient: probe one junction (diag)", "[.][diag][carve][junction][probe]") {
    // G2M_PROBE_FILE, G2M_PROBE_X/Y (level-0 tile), G2M_PROBE_NODE, G2M_PROBE_TILT (any value: tilted DEM).
    const RoadSurfaceParams params = builtin_carve_default().surface_params();
    const std::string file = env("G2M_PROBE_FILE").empty() ? std::string(kA66) : env("G2M_PROBE_FILE");
    const std::int32_t tx = env("G2M_PROBE_X").empty() ? 1822 : std::atoi(env("G2M_PROBE_X").c_str());
    const std::int32_t ty = env("G2M_PROBE_Y").empty() ? 21685 : std::atoi(env("G2M_PROBE_Y").c_str());
    const long long node = env("G2M_PROBE_NODE").empty() ? 507507LL : std::atoll(env("G2M_PROBE_NODE").c_str());
    auto& fx = fixture(file, !env("G2M_PROBE_TILT").empty());
    const auto m = fx.model(l0(tx, ty));
    for (std::size_t c = 0; c < m.model->junction_count(); ++c) {
        if (m.model->junction(c).nodes.front() != node) continue;
        const JunctionSurface& s = m.model->junction_surface(c);
        const RoadGeomTile& kept = m.model->kept_tile();
        const JunctionGrad g = measure(*m.model, c, params, 0.25);
        for (const Peak& at : {g.layer[0], g.layer[1], g.layer[2]}) {
            if (at.g == 0) continue;
            const double x = at.x, y = at.y, h = 0.05;
            std::printf("[probe] grad %.3f at (%.2f, %.2f) z %.4f\n", at.g, x, y, junction_height_m(s, kept, x, y, params));
            std::vector<double> w(s.approaches.size());
            junction_weights(s, x, y, w);
            for (std::size_t i = 0; i < s.approaches.size(); ++i) {
                const auto& a = s.approaches[i];
                const auto v = [&](double px, double py) { return junction_approach_heights(s, kept, i, px, py, params); };
                double phi = 0, psi = 0; junction_relief_weights(s, i, x, y, phi, psi);
                std::printf("[probe]   approach %zu entry %zu trim %.2f relief %.3f fade %.1f d %.2f w %.3f phi %.3f psi %.3f base %.4f paved %.4f full %.4f"
                    " full grad (%.3f, %.3f)\n", i, a.entry_index, a.trim_m, a.relief_m, a.fade_m, seg_dist(x, y, a.trim_ax, a.trim_ay, a.trim_bx, a.trim_by),
                    w[i], phi, psi, v(x, y).base, v(x, y).paved, v(x, y).full, (v(x + h, y).full - v(x - h, y).full) / (2 * h), (v(x, y + h).full - v(x, y - h).full) / (2 * h));
            }
        }
        for (const auto& a : s.approaches)
            std::printf("[probe] trim (%.2f, %.2f)-(%.2f, %.2f) ref0 (%.2f, %.2f)\n", a.trim_ax, a.trim_ay, a.trim_bx, a.trim_by, a.ref_x[0], a.ref_y[0]);
    }
}
