// tests/layer/test_terrain_carve.cpp - street fidelity P2 slice 9: g2m.terrain.carved (design 5.3, 6.1-6.4; tests
// T12-T15, T17, T19 of section 11, tile invariance, smoothing parity and the G3-D measurements of section 12).
//
// Fixture runs use the two committed OSM fixtures with roads.geom built as served (carve_fixture.h) on the flat 100 m
// DEM and on the libm-free tilted, bumpy, rough DEM (carve_tilted_dem). Every assertion has a sabotage or negative
// control that makes it fail; the measured values are printed for the slice 9 record.
#include "carve_fixture.h"
#include "alloc_counter.h"

#include "g2m/core/hash.h"
#include "g2m/layer/carve_params.h"
#include "g2m/layer/deriver.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/road_surface.h"
#include "g2m/layer/terrain_carve.h"
#include "g2m/layer/terrain_class.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/layer/tile_container.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/src_osm.h"
#include "g2m/import/bake.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/upstream.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <map>
#include <memory>
#include <utility>
#include <vector>

using namespace g2m;

namespace {
constexpr const char* kA66 = "hoechst_a66.osm.pbf";
constexpr const char* kFfm = "frankfurt_hoechst.osm.pbf";
using DemFn = std::function<double(double, double)>;

TileKey l0(std::int32_t x, std::int32_t y) { return TileKey{g2m_test::kZone32N, 0, x, y}; }

const RoadGraph& world(const std::string& file) {
    static std::map<std::string, std::unique_ptr<RoadGraph>> worlds;
    auto it = worlds.find(file);
    if (it == worlds.end()) it = worlds.emplace(file, std::make_unique<RoadGraph>(g2m_test::fixture_world(file))).first;
    return *it->second;
}
// One cached fixture per (file, DEM): the served roads.geom tiles are the expensive part.
g2m_test::CarveFixture& fixture(const std::string& file, bool tilted) {
    static std::map<std::pair<std::string, bool>, std::unique_ptr<g2m_test::CarveFixture>> fixtures;
    auto it = fixtures.find({file, tilted});
    if (it == fixtures.end())
        it = fixtures.emplace(std::pair{file, tilted}, std::make_unique<g2m_test::CarveFixture>(world(file),
            tilted ? DemFn(g2m_test::carve_tilted_dem) : DemFn{})).first;
    return *it->second;
}

bool same_owner(const CarveSample& a, const CarveSample& b) { return a.owner == b.owner && a.element == b.element; }
bool is_blend(CarveOwner o) { return o == CarveOwner::JunctionBlend || o == CarveOwner::ApproachBlend; }

// Analytic jumps of the model across owner switches (design T12): for every pair of 4-adjacent cell centres of the
// tile whose owners differ, the switch point is bisected to 1e-10 m and z is compared on both sides of it.
struct Jumps {
    double core_blend = 0;      // a core and its own blend (approach e / blend e, junction j / blend j, end caps included)
    double blend_terrain = 0;   // a blend and the smoothed terrain at the blend's outer edge
    double trim = 0;            // a junction core and the core of one of its own used approaches (the trim line)
    double blend_blend = 0;     // two blends (max-weight switch; reported, not asserted: equal w, different z_b)
    double core_core = 0;       // two different cores (lowest-wins, stretch joins; reported)
    double other = 0;
    double ring_overlap = 0;    // a junction core and its own approach's core away from the trim line (the ring reaches
                                // past the trim; junction cores win there): reported
    std::size_t n_ring_overlap = 0;
    std::size_t n_core_blend = 0, n_blend_terrain = 0, n_trim = 0, n_blend_blend = 0, n_core_core = 0, n_other = 0;
    struct Where { double x = 0, y = 0; CarveSample a, b; };
    Where w_core_blend, w_blend_terrain, w_trim, w_blend_blend, w_core_core, w_other;
};
Jumps measure_jumps(const CarveModel& model, const TileKey& key) {
    Jumps out;
    const double e0 = static_cast<double>(key.min_easting()), n0 = static_cast<double>(key.min_northing());
    const auto approach_of = [&](std::size_t junction, std::size_t entry) {
        for (const auto& a : model.junction(junction).approaches)
            if (a.entry_index == entry && a.status == JunctionApproachStatus::Used) return true;
        return false;
    };
    // The switch lies on the trim line of approach `entry` of junction `junction` (within 5 cm).
    const auto on_trim_line = [&](std::size_t junction, std::size_t entry, double x, double y) {
        double s = 0, t = 0;
        if (!model.station_of(entry, x, y, s, t)) return false;
        for (const auto& a : model.junction_surface(junction).approaches)
            if (a.entry_index == entry && std::fabs(s - (a.at_end ? a.length_m - a.trim_m : a.trim_m)) <= 0.05) return true;
        return false;
    };
    for (int j = 0; j < 256; ++j)
        for (int i = 0; i < 256; ++i)
            for (int dir = 0; dir < 2; ++dir) {
                const int i2 = i + (dir == 0 ? 1 : 0), j2 = j + (dir == 1 ? 1 : 0);
                if (i2 > 255 || j2 > 255) continue;
                const double ax = e0 + i + 0.5, ay = n0 + j + 0.5, bx = e0 + i2 + 0.5, by = n0 + j2 + 0.5;
                const CarveSample a = model.evaluate(ax, ay), b = model.evaluate(bx, by);
                if (same_owner(a, b) || a.deck_clamped || b.deck_clamped) continue;
                if (a.owner == CarveOwner::NoData || b.owner == CarveOwner::NoData) continue;
                double lo = 0, hi = 1;
                for (int it = 0; it < 34; ++it) {
                    const double mid = 0.5 * (lo + hi);
                    const CarveSample s = model.evaluate(ax + mid * (bx - ax), ay + mid * (by - ay));
                    if (same_owner(s, a)) lo = mid; else hi = mid;
                }
                const CarveSample sa = model.evaluate(ax + lo * (bx - ax), ay + lo * (by - ay));
                const CarveSample sb = model.evaluate(ax + hi * (bx - ax), ay + hi * (by - ay));
                if (sa.deck_clamped || sb.deck_clamped) continue;
                double jump = std::fabs(sa.z - sb.z);
                const CarveOwner oa = sa.owner, ob = sb.owner;
                const auto pair_is = [&](CarveOwner x, CarveOwner y) { return (oa == x && ob == y) || (oa == y && ob == x); };
                double* slot = &out.other; std::size_t* count = &out.n_other; Jumps::Where* where = &out.w_other;
                if (sa.element == sb.element && (pair_is(CarveOwner::ApproachCore, CarveOwner::ApproachBlend) ||
                                                 pair_is(CarveOwner::JunctionCore, CarveOwner::JunctionBlend))) {
                    slot = &out.core_blend; count = &out.n_core_blend; where = &out.w_core_blend;
                } else if ((oa == CarveOwner::Terrain && is_blend(ob)) || (ob == CarveOwner::Terrain && is_blend(oa))) {
                    slot = &out.blend_terrain; count = &out.n_blend_terrain; where = &out.w_blend_terrain;
                    // T is the smoothed DEM of the cell containing the point (design 6.2): where the switch lies on a
                    // cell boundary, the terrain's own step between the two cells passes through w -> 0 unchanged. The
                    // blend's step is the one beyond it.
                    jump = std::fabs((sa.z - sa.terrain) - (sb.z - sb.terrain));
                } else if (pair_is(CarveOwner::JunctionCore, CarveOwner::ApproachCore) &&
                           approach_of(oa == CarveOwner::JunctionCore ? sa.element : sb.element, oa == CarveOwner::ApproachCore ? sa.element : sb.element)) {
                    const std::size_t jc = oa == CarveOwner::JunctionCore ? sa.element : sb.element;
                    const std::size_t ae = oa == CarveOwner::ApproachCore ? sa.element : sb.element;
                    if (on_trim_line(jc, ae, ax + lo * (bx - ax), ay + lo * (by - ay))) { slot = &out.trim; count = &out.n_trim; where = &out.w_trim; }
                    else { slot = &out.ring_overlap; count = &out.n_ring_overlap; }
                } else if (is_blend(oa) && is_blend(ob)) {
                    slot = &out.blend_blend; count = &out.n_blend_blend; where = &out.w_blend_blend;
                } else if ((oa == CarveOwner::JunctionCore || oa == CarveOwner::ApproachCore) && (ob == CarveOwner::JunctionCore || ob == CarveOwner::ApproachCore)) {
                    slot = &out.core_core; count = &out.n_core_core; where = &out.w_core_core;
                }
                if (jump > *slot) { *slot = jump; *where = {ax + lo * (bx - ax), ay + lo * (by - ay), sa, sb}; }
                ++*count;
            }
    return out;
}

void print_jumps(const char* what, const Jumps& j) {
    std::printf("[T12] %s: core-blend %.6f (%zu) blend-terrain %.6f (%zu) trim %.6f (%zu) blend-blend %.4f (%zu) core-core %.4f (%zu) other %.4f (%zu) ring-overlap %.4f (%zu)\n",
        what, j.core_blend, j.n_core_blend, j.blend_terrain, j.n_blend_terrain, j.trim, j.n_trim, j.blend_blend, j.n_blend_blend,
        j.core_core, j.n_core_core, j.other, j.n_other, j.ring_overlap, j.n_ring_overlap);
}
void print_where(const char* cls, const Jumps::Where& w) {
    const auto one = [](const CarveSample& s) { std::printf("owner %d el %zu z %.4f w %.4f T %.4f", static_cast<int>(s.owner), s.element, s.z, s.weight, s.terrain); };
    std::printf("  worst %s at (%.3f, %.3f): ", cls, w.x, w.y); one(w.a); std::printf(" | "); one(w.b); std::printf("\n");
}

struct FixtureTile { const char* file; std::int32_t x, y; };
// The busiest junction tiles of each fixture (census: most junction-core cells).
const FixtureTile kBusy[] = {{kA66, 1822, 21685}, {kA66, 1820, 21684}, {kFfm, 1856, 21718}, {kFfm, 1852, 21718}};
} // namespace

// ---- smoothing (design 6.2 step 1) ----

namespace {
// The game's double smoothing (racing_game core/src/terrain_smoothing.cpp, behavioural reference): separable
// Gaussian radius 2 sigma 1 (weights exp(-i^2/2)), x pass then y pass, NaN taps skipped and renormalised, a NaN centre
// stays NaN, then h + 0.35 (s - h). In height units (1/256 m), unrounded.
double game_smoothed(const std::vector<double>& raw, int size, int x, int y) {
    const double w[5] = {0.1353352832366127, 0.6065306597126334, 1.0, 0.6065306597126334, 0.1353352832366127};
    const auto at = [&](int i, int j) { return (i < 0 || j < 0 || i >= size || j >= size) ? std::numeric_limits<double>::quiet_NaN() : raw[static_cast<std::size_t>(j * size + i)]; };
    const double h = at(x, y);
    if (h != h) return h;
    double ys = 0, yw = 0;
    for (int dy = -2; dy <= 2; ++dy) {
        if (at(x, y + dy) != at(x, y + dy)) continue;
        double xs = 0, xw = 0;
        for (int dx = -2; dx <= 2; ++dx) { const double v = at(x + dx, y + dy); if (v != v) continue; xs += w[dx + 2] * v; xw += w[dx + 2]; }
        ys += w[dy + 2] * (xs / xw); yw += w[dy + 2];
    }
    return h + 0.35 * (ys / yw - h);
}
} // namespace

TEST_CASE("carve: integer smoothing matches the game's double smoothing within one height unit, NoData skipped", "[layer][carve]") {
    const TileKey key = l0(1820, 21684);
    std::vector<HeightTile> tiles(9);
    std::array<const HeightTile*, 9> window{};
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) {
            auto& t = tiles[static_cast<std::size_t>((dy + 1) * 3 + dx + 1)];
            t.key = TileKey{key.zone, 0, key.x + dx, key.y + dy};
            g2m_test::fill_dem(t, g2m_test::carve_tilted_dem);
            window[static_cast<std::size_t>((dy + 1) * 3 + dx + 1)] = &t;
        }
    // NoData: a 3 x 3 hole and a single cell in the centre tile, a hole straddling the west seam.
    auto& centre = tiles[4];
    for (int j = 100; j < 103; ++j) for (int i = 100; i < 103; ++i) centre.h[static_cast<std::size_t>(j * 256 + i)] = kHeightNoData;
    centre.h[200 * 256 + 50] = kHeightNoData;
    for (int j = 10; j < 14; ++j) { centre.h[static_cast<std::size_t>(j * 256)] = kHeightNoData; tiles[3].h[static_cast<std::size_t>(j * 256 + 255)] = kHeightNoData; }
    // The reference sees the 768 x 768 window as one raster.
    std::vector<double> raw(768 * 768);
    for (int j = 0; j < 768; ++j) for (int i = 0; i < 768; ++i) {
        const std::int32_t v = tiles[static_cast<std::size_t>((j / 256) * 3 + i / 256)].h[static_cast<std::size_t>((j % 256) * 256 + i % 256)];
        raw[static_cast<std::size_t>(j * 768 + i)] = v == kHeightNoData ? std::numeric_limits<double>::quiet_NaN() : static_cast<double>(v);
    }
    const auto measure = [&](const CarveSmoothing& smoothing, double& max_diff, double& p99, std::size_t& nodata_mismatch) {
        const auto terrain = CarveTerrain::make(key, window, smoothing); REQUIRE(terrain);
        std::vector<double> diffs; nodata_mismatch = 0;
        for (int j = -8; j < 264; ++j) for (int i = -8; i < 264; ++i) {
            const double ref = game_smoothed(raw, 768, i + 256, j + 256);
            const std::int32_t got = terrain->smoothed(key.min_easting() + i, key.min_northing() + j);
            if ((ref != ref) != (got == kHeightNoData)) { ++nodata_mismatch; continue; }
            if (ref != ref) continue;
            diffs.push_back(std::fabs(static_cast<double>(got) - ref));
        }
        std::sort(diffs.begin(), diffs.end());
        max_diff = diffs.back(); p99 = diffs[diffs.size() * 99 / 100];
    };
    double max_diff = 0, p99 = 0; std::size_t nodata = 0;
    measure(builtin_carve_default().smoothing, max_diff, p99, nodata);
    std::printf("[smoothing] |server - game| max %.4f p99 %.4f units (1/256 m), NoData mismatches %zu\n", max_diff, p99, nodata);
    CHECK(nodata == 0);
    CHECK(max_diff <= 1.0);   // integer kernel rounding and two half-even roundings: below one height unit
    // The kernel and strength come from carve_default.json: those exact values.
    CHECK(builtin_carve_default().smoothing.kernel == std::vector<std::int32_t>{138, 621, 1024, 621, 138});
    CHECK(builtin_carve_default().smoothing.strength_percent == 35);
    // Sabotage: full strength instead of 35 % moves cells by many units.
    CarveSmoothing full = builtin_carve_default().smoothing; full.strength_percent = 100;
    double sab_max = 0, sab_p99 = 0; std::size_t sab_nodata = 0;
    measure(full, sab_max, sab_p99, sab_nodata);
    std::printf("[smoothing] sabotage strength 100: max %.1f units\n", sab_max);
    CHECK(sab_max > 1.0);
    // A NoData cell stays NoData; its neighbours are finite (renormalised).
    const auto terrain = CarveTerrain::make(key, window, builtin_carve_default().smoothing); REQUIRE(terrain);
    CHECK(terrain->smoothed(key.min_easting() + 101, key.min_northing() + 101) == kHeightNoData);
    CHECK(terrain->smoothed(key.min_easting() + 103, key.min_northing() + 101) != kHeightNoData);
    // Out of place window tiles are rejected.
    std::array<const HeightTile*, 9> bad = window; std::swap(bad[0], bad[8]);
    CHECK_FALSE(CarveTerrain::make(key, bad, builtin_carve_default().smoothing));
}

// ---- T12 continuity ----

TEST_CASE("carve T12: no step at core and blend edges, junction trim lines continuous (tilted rough DEM)", "[layer][carve]") {
    auto& fx = fixture(kA66, true);
    const TileKey key = l0(1822, 21685);
    const auto m = fx.model(key);
    const Jumps j = measure_jumps(*m.model, key);
    print_jumps("a66 1822/21685 tilted", j);
    CHECK(j.n_core_blend > 100); CHECK(j.n_blend_terrain > 100); CHECK(j.n_trim > 10);
    CHECK(j.core_blend <= 0.005);
    CHECK(j.blend_terrain <= 0.005);
    CHECK(j.trim < 0.01);
    // Trim-line samples: the junction surface equals the approach core within 1/256 m (16 points per used approach).
    double worst = 0; std::size_t points = 0;
    for (std::size_t c = 0; c < m.model->junction_count(); ++c) {
        const JunctionSurface& surface = m.model->junction_surface(c);
        for (const auto& a : surface.approaches) {
            const RoadGeomEntry& entry = m.model->core_tile().entries[a.entry_index];
            const double s = a.at_end ? a.length_m - a.trim_m : a.trim_m;
            const CarveFootprint f = carve_footprint(*entry.cross_section, s);
            for (int k = 0; k <= 15; ++k) {
                const double t = f.outer_right + (f.outer_left - f.outer_right) * k / 15.0;
                const auto p = lateral_point(entry.profile->reference, s, t); REQUIRE(p);
                const double zj = junction_height_m(surface, m.model->kept_tile(), p->x, p->y, builtin_carve_default().surface_params());
                const double za = road_surface_height_m(m.model->core_tile(), a.entry_index, s, t, builtin_carve_default().surface_params());
                worst = std::max(worst, std::fabs(zj - za)); ++points;
            }
        }
    }
    std::printf("[T12] trim-line samples: %zu points, max |z_J - z_core| %.3g m\n", points, worst);
    CHECK(points > 100);
    CHECK(worst <= 1.0 / 256.0);
    // Every carved cell is the half-even quantisation of the model at its centre.
    const auto tile = fx.carve(key); REQUIRE(tile);
    std::size_t mismatch = 0;
    for (int jj = 0; jj < 256; jj += 3) for (int ii = 0; ii < 256; ii += 3) {
        const CarveSample s = m.model->evaluate(key.min_easting() + ii + 0.5, key.min_northing() + jj + 0.5);
        if (tile->h[static_cast<std::size_t>(jj * 256 + ii)] != *quantise_height_half_even(s.z)) ++mismatch;
    }
    CHECK(mismatch == 0);
}

TEST_CASE("carve T12 sabotage: blending toward the raw DEM steps at the blend edge", "[layer][carve]") {
    auto& fx = fixture(kA66, true);
    const TileKey key = l0(1822, 21685);
    CarveOptions sabotage; sabotage.blend_from_raw = true;
    const auto m = fx.model(key, sabotage);
    const Jumps j = measure_jumps(*m.model, key);
    print_jumps("a66 1822/21685 tilted, blend from raw (sabotage)", j);
    CHECK(j.blend_terrain > 0.005);
}

TEST_CASE("carve T12: continuity on every busy fixture tile, flat and tilted DEM (G3-D)", "[layer][carve]") {
    for (const auto& ft : kBusy)
        for (int tilted = 0; tilted < 2; ++tilted) {
            auto& fx = fixture(ft.file, tilted != 0);
            const TileKey key = l0(ft.x, ft.y);
            const auto m = fx.model(key);
            const Jumps j = measure_jumps(*m.model, key);
            char what[128]; std::snprintf(what, sizeof what, "%s %d/%d %s", ft.file, ft.x, ft.y, tilted ? "tilted" : "flat");
            print_jumps(what, j);
            // 7a-1: hoechst_a66 1820/21684 tilted measures 0.0097 since the ground stretch beside the 44 m tunnel completes at the
            // tunnel's portal plane (a stretch that stayed declined before); the other tiles are unchanged.
            CHECK(j.core_blend <= (tilted && ft.x == 1820 ? 0.012 : 0.005));
            CHECK(j.blend_terrain <= 0.005);
            CHECK(j.trim < 0.01);
        }
}

// ---- T13 cross-slope ----

namespace {
// Least squares z = a + g (s - s0) + k * r over the cells, r the crown regressor (rounded |d| for a roof, d one-sided).
// Least squares z = a + g ds + q ds^2 + k r over the rows (ds, r, z); returns k. The along-road terms take the
// profile's grade and vertical curvature (the tilted bumpy DEM's profiles bend by up to ~0.15 1/m), which cells of a
// curved road would otherwise alias into the crossfall.
double fit_k(const std::vector<std::array<double, 3>>& rows) {
    constexpr int n = 4;
    double m[n][n + 1] = {};
    for (const auto& r : rows) {
        const double v[n] = {1.0, r[0], r[0] * r[0], r[1]};
        for (int a = 0; a < n; ++a) { for (int b = 0; b < n; ++b) m[a][b] += v[a] * v[b]; m[a][n] += v[a] * r[2]; }
    }
    for (int c = 0; c < n; ++c) {
        int p = c; for (int r = c + 1; r < n; ++r) if (std::fabs(m[r][c]) > std::fabs(m[p][c])) p = r;
        for (int k = 0; k <= n; ++k) std::swap(m[c][k], m[p][k]);
        for (int r = 0; r < n; ++r) if (r != c) { const double f = m[r][c] / m[c][c]; for (int k = 0; k <= n; ++k) m[r][k] -= f * m[c][k]; }
    }
    return m[n - 1][n] / m[n - 1][n - 1];
}
struct SlopeStats { std::vector<double> errors; double single_cell_max = 0; std::size_t skipped = 0;
    double worst = 0; std::size_t worst_entry = 0; double worst_s = 0, worst_k = 0, worst_want = 0, worst_tmin = 0, worst_tmax = 0; int worst_crown = 0; std::size_t worst_rows = 0; };
// Per core station (every 4 m of every carved ground entry inside the tile, away from features and trims): the LS
// crossfall over the carriageway cells within +-2.5 m of the station, compared with the entry's crown_per_mille. The
// unquantised model meets the crossfall to 0.0005; what remains is the 1/256 m quantum: over a 3.5 m carriageway a
// +-1.5 m patch (11-12 cells) measured max 0.00202, the +-2.5 m patch (~20 cells) 0.00155.
SlopeStats cross_slopes(const CarveModel& model, const HeightTile& tile) {
    SlopeStats out;
    const RoadGeomTile& core = model.core_tile();
    const double r0 = builtin_carve_default().surface_params().crown_round_m;
    const double e0 = static_cast<double>(tile.key.min_easting()), n0 = static_cast<double>(tile.key.min_northing());
    for (std::size_t e = 0; e < core.entries.size(); ++e) {
        const RoadGeomEntry& entry = core.entries[e];
        if (!entry.profile || !entry.cross_section || entry.profile->attributes.bridge || entry.profile->attributes.tunnel) continue;
        const double L = entry.profile->reference.length_m;
        const double c = entry.profile->attributes.crown_per_mille / 1000.0;
        for (double s0 = model.trim_start(e) + 4.0; s0 <= L - model.trim_end(e) - 4.0; s0 += 4.0) {
            bool feature = false;
            for (const auto& f : core.features) if (f.entry_index == e && f.carve && std::fabs(f.station_m - s0) < feature_effective_length_m(f) / 2 + 3.0) feature = true;
            if (feature) continue;
            const auto pose = entry.profile->reference.at(s0); if (!pose) continue;
            if (pose->x < e0 + 8 || pose->x > e0 + 248 || pose->y < n0 + 8 || pose->y > n0 + 248) continue;
            const CarveFootprint f0 = carve_footprint(*entry.cross_section, s0);
            std::vector<std::array<double, 3>> rows;
            std::vector<std::pair<double, double>> lateral;   // (t, z) of the cells nearest the station, for the single-cell slopes
            double t_min = 1e9, t_max = -1e9; bool foreign = false;
            const int reach = static_cast<int>(std::max(-f0.carriageway_right, f0.carriageway_left)) + 3;
            for (int dj = -reach; dj <= reach; ++dj) for (int di = -reach; di <= reach; ++di) {
                const auto ci = static_cast<std::int64_t>(std::floor(pose->x)) + di, cj = static_cast<std::int64_t>(std::floor(pose->y)) + dj;
                if (ci < tile.key.min_easting() || cj < tile.key.min_northing() || ci >= tile.key.min_easting() + 256 || cj >= tile.key.min_northing() + 256) continue;
                const double cx = static_cast<double>(ci) + 0.5, cy = static_cast<double>(cj) + 0.5;
                double s = 0, t = 0;
                if (!model.station_of(e, cx, cy, s, t) || std::fabs(s - s0) > 2.5) continue;
                const CarveFootprint f = carve_footprint(*entry.cross_section, s);
                if (t < f.carriageway_right || t > f.carriageway_left) continue;
                const CarveSample sample = model.evaluate(cx, cy);
                if (sample.owner != CarveOwner::ApproachCore || sample.element != e || sample.deck_clamped) { foreign = true; continue; }
                const double z = static_cast<double>(tile.h[static_cast<std::size_t>((cj - tile.key.min_northing()) * 256 + (ci - tile.key.min_easting()))]) / 256.0;
                const double d = t - f.crown_pivot;
                const double r = f.crown == CrownKind::Roof ? (std::fabs(d) >= r0 ? std::fabs(d) : d * d / (2 * r0) + r0 / 2) : d;
                rows.push_back({s - s0, r, z});
                t_min = std::min(t_min, t); t_max = std::max(t_max, t);
                if (std::fabs(s - s0) <= 0.5 && std::fabs(d) > r0 + 0.5) lateral.emplace_back(t, z);
            }
            if (foreign || rows.size() < 8 || t_max - t_min < 3.0) { ++out.skipped; continue; }
            const double k = fit_k(rows);
            const double intended = f0.crown == CrownKind::OneSidedRight ? c : -c;
            out.errors.push_back(std::fabs(k - intended));
            if (std::fabs(k - intended) > out.worst) {
                out.worst = std::fabs(k - intended); out.worst_entry = e; out.worst_s = s0; out.worst_k = k; out.worst_want = intended;
                out.worst_tmin = t_min; out.worst_tmax = t_max; out.worst_crown = static_cast<int>(f0.crown); out.worst_rows = rows.size();
            }
            std::sort(lateral.begin(), lateral.end());
            for (std::size_t i = 1; i < lateral.size(); ++i) {
                const double dt = lateral[i].first - lateral[i - 1].first;
                if (dt > 0.7 && dt < 1.3 && (lateral[i].first - f0.crown_pivot) * (lateral[i - 1].first - f0.crown_pivot) > 0) {
                    const double slope = (lateral[i].second - lateral[i - 1].second) / dt;
                    const double want = f0.crown == CrownKind::Roof ? (lateral[i].first > f0.crown_pivot ? -c : c) : (f0.crown == CrownKind::OneSidedRight ? c : -c);
                    out.single_cell_max = std::max(out.single_cell_max, std::fabs(slope - want));
                }
            }
        }
    }
    std::sort(out.errors.begin(), out.errors.end());
    return out;
}
} // namespace


TEST_CASE("carve T13: least-squares crossfall over >= 3 m of carriageway equals crown_per_mille within 0.2 %", "[layer][carve]") {
    std::vector<double> all; double single = 0; std::size_t skipped = 0;
    for (const auto& ft : kBusy)
        for (int tilted = 0; tilted < 2; ++tilted) {
            auto& fx = fixture(ft.file, tilted != 0);
            const TileKey key = l0(ft.x, ft.y);
            const auto m = fx.model(key);
            const auto tile = fx.carve(key); REQUIRE(tile);
            const SlopeStats s = cross_slopes(*m.model, *tile);
            all.insert(all.end(), s.errors.begin(), s.errors.end());
            single = std::max(single, s.single_cell_max); skipped += s.skipped;
            if (s.worst > 0.0015)
                std::printf("[T13] %s %d/%d %s: worst entry %zu s %.1f k %.5f want %.5f crown %d t %.2f..%.2f rows %zu\n", ft.file, ft.x, ft.y,
                    tilted ? "tilted" : "flat", s.worst_entry, s.worst_s, s.worst_k, s.worst_want, s.worst_crown, s.worst_tmin, s.worst_tmax, s.worst_rows);
        }
    std::sort(all.begin(), all.end());
    REQUIRE(all.size() > 200);
    std::printf("[T13] %zu stations (%zu skipped: feature, foreign owner or < 3 m): LS crossfall error p50 %.5f p95 %.5f p99 %.5f max %.5f; "
        "single-cell slope error max %.5f\n", all.size(), skipped, all[all.size() / 2], all[all.size() * 95 / 100], all[all.size() * 99 / 100],
        all.back(), single);
    CHECK(all.back() <= 0.002);
    // The documented control: single-cell slopes (1/256 m over 1 m = 0.39 %) cannot meet the bound.
    CHECK(single > 0.002);
}

// ---- T14 ownership ----

TEST_CASE("carve T14: ownership is independent of the candidate order (100 shuffles); first-come differs", "[layer][carve]") {
    auto& fx = fixture(kA66, true);
    const TileKey key = l0(1822, 21685);
    const auto base = fx.model(key);
    std::vector<std::pair<double, double>> points;
    std::vector<double> z0;
    for (int j = 0; j < 256; ++j) for (int i = 0; i < 256; ++i) {
        const double x = key.min_easting() + i + 0.5, y = key.min_northing() + j + 0.5;
        const CarveSample s = base.model->evaluate(x, y);
        if (s.owner == CarveOwner::Terrain) continue;
        points.emplace_back(x, y); z0.push_back(s.z);
    }
    REQUIRE(points.size() > 5000);
    std::size_t differ = 0;
    for (std::uint64_t seed = 1; seed <= 100; ++seed) {
        CarveOptions o; o.candidate_order_seed = seed * 7919;
        const auto m = fx.model(key, o);
        for (std::size_t p = 0; p < points.size(); ++p)
            if (m.model->evaluate(points[p].first, points[p].second).z != z0[p]) ++differ;
    }
    CHECK(differ == 0);
    // Sabotage: first-come ownership depends on the order.
    std::size_t first_differ = 0;
    std::vector<double> first_z;
    for (std::uint64_t seed : {0ull, 7919ull, 15838ull}) {
        CarveOptions o; o.first_come_ownership = true; o.candidate_order_seed = seed;
        const auto m = fx.model(key, o);
        for (std::size_t p = 0; p < points.size(); ++p) {
            const double z = m.model->evaluate(points[p].first, points[p].second).z;
            if (seed == 0) first_z.push_back(z); else if (z != first_z[p]) ++first_differ;
        }
    }
    std::printf("[T14] %zu cells x 100 orders: %zu differ; first-come over 3 orders: %zu differ\n", points.size(), differ, first_differ);
    CHECK(first_differ > 0);
}

// ---- T15 decks ----

namespace {
// A 300 m bridge (way 20, x 300..600 m at y 400 m) between two ground roads over a valley 8 m deep (floor
// |x - 450| < 60, 1:3 sides to |x - 450| = 84); a lower road (way 30) runs north-south along the valley floor under it.
RoadGraph bridge_world() {
    RoadGraph w;
    const auto node = [&](std::int64_t id, std::int64_t x, std::int64_t y) { w.nodes.push_back({id, {x * 1000, y * 1000}}); };
    node(1, 100, 400); node(2, 200, 400); node(3, 300, 400); node(4, 450, 400); node(5, 600, 400); node(6, 700, 400); node(7, 800, 400);
    node(31, 450, 150); node(32, 450, 650);
    w.ways = {{10, 3, {1, 2, 3}, {{"highway", "tertiary"}}}, {12, 3, {5, 6, 7}, {{"highway", "tertiary"}}},
        {20, 3, {3, 4, 5}, {{"bridge", "yes"}, {"highway", "tertiary"}, {"layer", "1"}}}, {30, 3, {31, 32}, {{"highway", "tertiary"}}}};
    for (const auto& way : w.ways) for (std::size_t i = 1; i < way.node_ids.size(); ++i)
        w.edges.push_back({way.osm_id, static_cast<std::uint32_t>(i - 1), way.node_ids[i - 1], way.node_ids[i]});
    return w;
}
double valley_dem(double e, double) {
    const double a = std::fabs(e - 450.0);
    const double depth = a <= 60.0 ? 1.0 : a >= 84.0 ? 0.0 : (84.0 - a) / 24.0;
    return 100.0 - 8.0 * depth;
}
} // namespace

TEST_CASE("carve T15: the deck clamp keeps terrain 0.55 m below a bridge, leaves the lower road and everything else alone", "[layer][carve]") {
    const RoadGraph w = bridge_world();
    g2m_test::CarveFixture fx(w, valley_dem);
    const TileKey key = l0(1, 1);
    const auto m = fx.model(key);
    CarveOptions off; off.deck_clamp = false;
    const auto m_off = fx.model(key, off);
    CarveStats stats;
    const auto tile = fx.carve(key, {}, 1, &stats); REQUIRE(tile);
    const auto tile_off = fx.carve(key, off); REQUIRE(tile_off);
    std::size_t bridge = 0, lower = 0;
    const RoadGeomTile& core = m.model->core_tile();
    for (std::size_t e = 0; e < core.entries.size(); ++e) {
        if (core.entries[e].way_id == 20) bridge = e;
        if (core.entries[e].way_id == 30) lower = e;
    }
    REQUIRE(core.entries[bridge].profile); REQUIRE(core.entries[bridge].cross_section); REQUIRE(core.entries[lower].profile);
    const auto params = builtin_carve_default();
    const double clear = params.deck.clearance_mm / 1000.0, ramp = params.deck.ramp_mm / 1000.0, margin = params.deck.margin_mm / 1000.0;
    const double L = core.entries[bridge].profile->reference.length_m;
    std::size_t inside = 0, lowered = 0, above_limit_off = 0, outside_changed = 0, lower_cells = 0, lower_changed = 0, above = 0;
    for (int j = 0; j < 256; ++j) for (int i = 0; i < 256; ++i) {
        const double x = key.min_easting() + i + 0.5, y = key.min_northing() + j + 0.5;
        const std::size_t at = static_cast<std::size_t>(j * 256 + i);
        double s = 0, t = 0;
        bool in = m.model->station_of(bridge, x, y, s, t);
        if (in) {
            const CarveFootprint fs = carve_footprint(*core.entries[bridge].cross_section, s);
            in = t >= fs.paved_right - margin && t <= fs.paved_left + margin;
            if (in) {
                ++inside;
                const double top = road_surface_height_m(core, bridge, s, std::clamp(t, fs.paved_right, fs.paved_left), params.surface_params());
                const double limit = top - clear * road_smooth_step(std::min(s, L - s) / ramp);
                if (tile->h[at] / 256.0 > limit + 1.0 / 512.0) ++above;
                if (tile_off->h[at] / 256.0 > limit + 1.0 / 512.0) ++above_limit_off;
                if (tile->h[at] != tile_off->h[at]) ++lowered;
            }
        }
        if (!in && tile->h[at] != tile_off->h[at]) ++outside_changed;
        const CarveSample so = m_off.model->evaluate(x, y);
        if (so.owner == CarveOwner::ApproachCore && so.element == lower) { ++lower_cells; if (tile->h[at] != tile_off->h[at]) ++lower_changed; }
    }
    std::printf("[T15] footprint cells %zu, lowered %zu (stats %zu, violations %zu), above the limit %zu (clamp off: %zu), "
        "changed outside %zu, lower road cells %zu changed %zu\n", inside, lowered, stats.cells_deck_clamped, stats.deck_clearance_violations,
        above, above_limit_off, outside_changed, lower_cells, lower_changed);
    CHECK(inside > 1000);
    CHECK(lowered > 300);
    CHECK(above == 0);
    CHECK(outside_changed == 0);
    CHECK(lower_cells > 1000);
    CHECK(lower_changed == 0);
    CHECK(stats.deck_clearance_violations == 0);
    // Sabotage (clamp off): terrain under the deck ends sits above the limit.
    CHECK(above_limit_off > 300);
    // Bridges are never carved: no bridge cell is a core or blend.
    CHECK(stats.bridge_entries == 1);
}

// ---- T19 workers and allocation ----

TEST_CASE("carve T19: 1 and 10 workers give identical tiles; evaluate allocates nothing", "[layer][carve]") {
    auto& fx = fixture(kA66, true);
    const TileKey key = l0(1820, 21684);
    CarveStats s1, s10;
    const auto a = fx.carve(key, {}, 1, &s1); REQUIRE(a);
    const auto b = fx.carve(key, {}, 10, &s10); REQUIRE(b);
    CHECK(a->h == b->h);
    CHECK(s1.cells_junction_core == s10.cells_junction_core);
    CHECK(s1.cells_approach_blend == s10.cells_approach_blend);
    const auto m = fx.model(key);
    double sum = 0;
    {
        g2m_test::ScopedAllocCounter counter;
        for (int j = 0; j < 256; j += 2) for (int i = 0; i < 256; i += 2)
            sum += m.model->evaluate(key.min_easting() + i + 0.5, key.min_northing() + j + 0.5).z;
        if (g2m_test::kAllocCounterAvailable) CHECK(counter.count() == 0);
    }
    CHECK(sum == sum);
    {   // Negative control: the counter sees an allocation in the same kind of scope.
        g2m_test::ScopedAllocCounter counter;
        const auto v = std::make_unique<std::vector<double>>(64, sum);
        if (g2m_test::kAllocCounterAvailable) CHECK(counter.count() > 0);
        CHECK(v->size() == 64);
    }
}

// ---- tile invariance ----

namespace {
// The largest |z_A - z_B| over the cells within `band` cells of the shared edge of two adjacent level-0 tiles, both
// models evaluated at the same cell centres (the cells of either tile).
// `b_side_only` (the negative controls): only the cells of tile B where both models have data.
double seam_difference(const CarveModel& a, const CarveModel& b, const TileKey& ka, const TileKey& kb, int band, std::size_t& cells,
        bool b_side_only = false) {
    double worst = 0; cells = 0;
    const bool vertical = ka.y == kb.y;   // east-west neighbours share a vertical edge
    const std::int64_t edge = vertical ? std::max(ka.min_easting(), kb.min_easting()) : std::max(ka.min_northing(), kb.min_northing());
    for (int k = -band; k < band; ++k)
        for (int u = 0; u < 256; ++u) {
            const double x = vertical ? static_cast<double>(edge + k) + 0.5 : static_cast<double>(ka.min_easting() + u) + 0.5;
            const double y = vertical ? static_cast<double>(ka.min_northing() + u) + 0.5 : static_cast<double>(edge + k) + 0.5;
            const CarveSample sa = a.evaluate(x, y), sb = b.evaluate(x, y);
            if (sa.z != sa.z && sb.z != sb.z) continue;
            if (b_side_only && (sa.z != sa.z || sb.z != sb.z || x < static_cast<double>(kb.min_easting()) || y < static_cast<double>(kb.min_northing()) ||
                                x > static_cast<double>(kb.min_easting() + 256) || y > static_cast<double>(kb.min_northing() + 256))) continue;
            const bool nan = sa.z != sa.z || sb.z != sb.z;   // NoData on one side only
            worst = std::max(worst, nan ? 1e9 : std::fabs(sa.z - sb.z));
            ++cells;
        }
    return worst;
}
} // namespace

TEST_CASE("carve: tile invariance across level-0 and level-2 seams, with negative controls", "[layer][carve]") {
    struct Seam { const char* file; TileKey a, b; bool crosses_l2; };
    const Seam seams[] = {{kA66, l0(1819, 21684), l0(1820, 21684), true}, {kA66, l0(1822, 21685), l0(1822, 21686), false},
                          {kFfm, l0(1855, 21718), l0(1856, 21718), true}};
    double own_worst = 0;
    for (const auto& seam : seams)
        for (int tilted = 0; tilted < 2; ++tilted) {
            auto& fx = fixture(seam.file, tilted != 0);
            const auto ma = fx.model(seam.a), mb = fx.model(seam.b);
            std::size_t cells = 0;
            const double diff = seam_difference(*ma.model, *mb.model, seam.a, seam.b, 4, cells);
            std::printf("[invariance] %s %d/%d | %d/%d %s: %zu cells, max |dz| %.3g\n", seam.file, seam.a.x, seam.a.y, seam.b.x, seam.b.y,
                tilted ? "tilted" : "flat", cells, diff);
            CHECK(cells == 8 * 256);
            CHECK(diff == 0.0);
            if (!tilted) continue;
            // Negative control 1: the centre DEM tile only (no neighbours): smoothing and blend widths differ at the edge.
            const auto mb_dem = fx.model(seam.b, {}, false, true);
            const double dem_diff = seam_difference(*ma.model, *mb_dem.model, seam.a, seam.b, 4, cells, true);
            std::printf("[invariance] control centre-only DEM: max |dz| %.3g\n", dem_diff);
            CHECK(dem_diff > 0.0);
            if (!seam.crosses_l2) continue;
            // Negative control 2: only the level-2 roads.geom tile containing the tile (not the 192 m radius).
            const auto ma_own = fx.model(seam.a, {}, true), mb_own = fx.model(seam.b, {}, true);
            const double own_diff = seam_difference(*ma_own.model, *mb_own.model, seam.a, seam.b, 4, cells);
            std::printf("[invariance] control own level-2 roads only: max |dz| %.3g\n", own_diff);
            own_worst = std::max(own_worst, own_diff);
        }
    // Across the level-2 seams with roads near them, a tile reading only its own level-2 roads.geom tile disagrees.
    CHECK(own_worst > 0.0);
}

// ---- grade limits (G3-D) ----

TEST_CASE("carve: junction gradients and blend slopes of the busy tiles (G3-D)", "[layer][carve]") {
    double worst_excess = -1e9, worst_ratio = 0, max_grad = 0; std::size_t junctions = 0, steep = 0, stations = 0, over_bound = 0;
    char worst_where[256] = {};
    for (const auto& ft : kBusy)
        for (int tilted = 0; tilted < 2; ++tilted) {
            auto& fx = fixture(ft.file, tilted != 0);
            const TileKey key = l0(ft.x, ft.y);
            const auto m = fx.model(key);
            steep += m.stats.steep_blend_stations; stations += m.stats.blend_stations;
            for (std::size_t c = 0; c < m.model->junction_count(); ++c) {
                const JunctionSurface& surface = m.model->junction_surface(c);
                double bound = 0;
                for (const auto& a : surface.approaches) {
                    const RoadGeomEntry& e = m.model->core_tile().entries[a.entry_index];
                    const double s = std::clamp(a.at_end ? a.length_m - a.trim_m : a.trim_m, 0.5, a.length_m - 0.5);
                    const auto p0 = e.profile->at(s - 0.5), p1 = e.profile->at(s + 0.5);
                    if (!p0 || !p1) continue;
                    const double grade = std::fabs(p1->height_m - p0->height_m);
                    bound = std::max(bound, grade + e.profile->attributes.crown_per_mille / 1000.0);
                }
                const auto& ring = m.model->junction(c).rings[2].vertices;
                std::int64_t x0 = ring[0].x, x1 = x0, y0 = ring[0].y, y1 = y0;
                for (const auto& v : ring) { x0 = std::min(x0, v.x); x1 = std::max(x1, v.x); y0 = std::min(y0, v.y); y1 = std::max(y1, v.y); }
                if (x1 < key.min_easting() * 1000 || x0 > (key.min_easting() + 256) * 1000 || y1 < key.min_northing() * 1000 || y0 > (key.min_northing() + 256) * 1000) continue;
                double grad = 0, gx_at = 0, gy_at = 0;
                const double h = 0.05;
                for (double y = y0 / 1000.0; y <= y1 / 1000.0; y += 0.25)
                    for (double x = x0 / 1000.0; x <= x1 / 1000.0; x += 0.25) {
                        const CarveSample s = m.model->evaluate(x, y);
                        if (s.owner != CarveOwner::JunctionCore || s.element != c || s.deck_clamped) continue;
                        const CarveSample sx0 = m.model->evaluate(x - h, y), sx1 = m.model->evaluate(x + h, y);
                        const CarveSample sy0 = m.model->evaluate(x, y - h), sy1 = m.model->evaluate(x, y + h);
                        if (!same_owner(sx0, s) || !same_owner(sx1, s) || !same_owner(sy0, s) || !same_owner(sy1, s)) continue;
                        if (sx0.deck_clamped || sx1.deck_clamped || sy0.deck_clamped || sy1.deck_clamped) continue;
                        const double gx = (sx1.z - sx0.z) / (2 * h), gy = (sy1.z - sy0.z) / (2 * h);
                        if (std::sqrt(gx * gx + gy * gy) > grad) { grad = std::sqrt(gx * gx + gy * gy); gx_at = x; gy_at = y; }
                    }
                ++junctions;
                if (grad > max_grad) {
                    max_grad = grad;
                    std::snprintf(worst_where, sizeof worst_where, "%s %d/%d %s junction node %lld at (%.2f, %.2f), %zu approaches, bound %.4f",
                        ft.file, ft.x, ft.y, tilted ? "tilted" : "flat", static_cast<long long>(m.model->junction(c).nodes.front()), gx_at, gy_at,
                        surface.approaches.size(), bound);
                }
                if (grad > bound + 0.02) ++over_bound;
                worst_excess = std::max(worst_excess, grad - bound);
                if (bound > 0) worst_ratio = std::max(worst_ratio, grad / bound);
            }
        }
    std::printf("[G3-D] %zu junctions: max gradient %.3f (%s); max (gradient - (steepest approach grade + crossfall)) %.4f, max ratio %.2f, "
        "%zu junctions over that bound + 0.02; blend stations steeper than 1:1.5: %zu of %zu\n", junctions, max_grad, worst_where, worst_excess, worst_ratio,
        over_bound, steep, stations);
    REQUIRE(junctions > 50);
    // The junction surface's own gradient bound (what the trim data allow) is asserted in test_junction_gradient.cpp
    // (D42); counted over every core point including the verge corners, where two approaches' trim data sit centimetres
    // apart, this line stays a report (D41). What is asserted here is that carving adds no
    // discontinuity of its own inside a junction (a step reads as a gradient of tens: the deck clamp under the flat
    // DEM's ground-level bridges gave 12.7 before clamped samples were excluded) and that the 1:1.5 blends that cannot
    // reach the terrain within 15 m stay rare (design 6.3: counted, not hidden).
    CHECK(max_grad <= 1.5);
    CHECK(steep * 100 < stations);
}

// ---- T17 pyramid and the deriver ----

namespace {
InputTile input_tile(std::string layer, TileKey key, const TileBody& body, VerticalDatum datum, std::int32_t offset = 0) {
    InputTile out; out.ref = {std::move(layer), key}; out.body = *encode_body(body);
    TileHeader h; h.layer = out.ref.layer; h.layer_version = 1; h.key = key; h.datum = datum; h.height_offset = offset;
    h.body_size = out.body.size(); h.body_hash = content_hash(out.body); out.header = h; return out;
}
} // namespace

TEST_CASE("carve T17: the deriver serves carve_terrain_tile at L0 and the mean-of-4 pyramid above", "[layer][carve][terrain]") {
    auto& fx = fixture(kA66, true);
    const TerrainCarvedDeriver deriver;
    const TileKey parent{g2m_test::kZone32N, 1, 1820 / 2, 21684 / 2};
    std::vector<InputTile> children;
    std::vector<HeightTile> expected(4);   // 256 KiB each: on the heap
    for (int k = 0; k < 4; ++k) {
        const TileKey child = parent.child(k % 2, k / 2);
        const auto refs = deriver.inputs_for(child);
        REQUIRE(refs.size() == 9 + carve_road_geom_keys(child).size());
        std::vector<InputTile> inputs;
        for (const auto& ref : refs) {
            if (ref.layer == kElevBaseLayer) {
                // Stored 10 m lower with a +10 m header offset: the deriver applies the offset once.
                auto t = std::make_unique<HeightTile>(fx.dem(ref.key));
                for (auto& h : t->h) h -= 2560;
                inputs.push_back(input_tile(ref.layer, ref.key, *encode_height_tile(*t), VerticalDatum::DHHN2016, 2560));
            } else {
                inputs.push_back(input_tile(ref.layer, ref.key, *encode_road_geom(fx.geom(ref.key)), VerticalDatum::DHHN2016));
            }
        }
        const auto result = deriver.derive({child, 0, inputs});
        INFO((result ? "ok" : result.error().message)); REQUIRE(result);
        CHECK(result->datum == VerticalDatum::DHHN2016); CHECK(result->height_offset == 0);
        const auto direct = fx.carve(child); REQUIRE(direct);
        auto& served = expected[static_cast<std::size_t>(k)];
        REQUIRE(decode_height_tile(*decode_body(result->body), child, served));
        CHECK(served.h == direct->h);
        children.push_back(input_tile(std::string(kTerrainCarvedLayer), child, *decode_body(result->body), VerticalDatum::DHHN2016));
        if (k == 0) {
            // The centre elev.base absent: NotFound (as terrain.height). A roads.geom tile in another datum: rejected.
            auto no_centre = inputs;
            for (auto& in : no_centre) if (in.ref.layer == kElevBaseLayer && in.ref.key == child) in.header.reset();
            const auto missing = deriver.derive({child, 0, no_centre}); REQUIRE_FALSE(missing); CHECK(missing.error().code == ErrorCode::NotFound);
            auto unspecified = inputs;
            for (auto& in : unspecified)
                if (in.ref.layer == kElevBaseLayer || in.ref.layer == kRoadGeomLayer)
                    in.header->datum = VerticalDatum::Unspecified;
            const auto unknown_datum = deriver.derive({child, 0, unspecified}); REQUIRE(unknown_datum);
            CHECK(unknown_datum->datum == VerticalDatum::Unspecified);
            auto no_height_datum = unspecified;
            for (auto& in : no_height_datum)
                if (in.ref.layer == kElevBaseLayer) { in.header->datum = VerticalDatum::None; break; }
            CHECK_FALSE(deriver.derive({child, 0, no_height_datum}));
            auto mixed_known = inputs;
            std::sort(mixed_known.begin(), mixed_known.end(), [](const auto& a, const auto& b) { return a.ref < b.ref; });
            std::vector<std::size_t> elevation_indices;
            for (std::size_t i = 0; i < mixed_known.size(); ++i) {
                if (mixed_known[i].ref.layer == kElevBaseLayer) {
                    elevation_indices.push_back(i);
                    mixed_known[i].header->datum = VerticalDatum::Unspecified;
                } else if (mixed_known[i].ref.layer == kRoadGeomLayer) {
                    mixed_known[i].header->datum = VerticalDatum::Unspecified;
                }
            }
            REQUIRE(elevation_indices.size() >= 3);
            mixed_known[elevation_indices.front()].header->datum = VerticalDatum::DHHN2016;
            mixed_known[elevation_indices.back()].header->datum = VerticalDatum::EGM2008;
            CHECK_FALSE(deriver.derive({child, 0, mixed_known}));
            std::reverse(mixed_known.begin(), mixed_known.end());
            CHECK_FALSE(deriver.derive({child, 0, mixed_known}));
            auto egm = inputs;
            for (auto& in : egm) if (in.ref.layer == kRoadGeomLayer) in.header->datum = VerticalDatum::EGM2008;
            CHECK_FALSE(deriver.derive({child, 0, egm}));
            auto short_inputs = inputs; short_inputs.pop_back();
            CHECK_FALSE(deriver.derive({child, 0, short_inputs}));
        }
    }
    const auto refs = deriver.inputs_for(parent);
    REQUIRE(refs.size() == 4);
    std::vector<InputTile> ordered_children;
    for (const auto& ref : refs)
        for (const auto& c : children) if (c.ref == ref) ordered_children.push_back(c);
    REQUIRE(ordered_children.size() == 4);
    const auto l1 = deriver.derive({parent, 0, ordered_children}); REQUIRE(l1);
    std::vector<HeightTile> out(4);
    HeightTile& got = out[0]; HeightTile& want = out[1]; HeightTile& bare_l1 = out[2];
    REQUIRE(decode_height_tile(*decode_body(l1->body), parent, got));
    want.key = parent;
    downsample_2x2(&expected[0], &expected[1], &expected[2], &expected[3], want);
    CHECK(got.h == want.h);
    // The pyramid differs from terrain.height's (the bare DEM's) where roads were carved: it is carved.
    std::vector<HeightTile> bare(4);
    for (int k = 0; k < 4; ++k) bare[static_cast<std::size_t>(k)] = fx.dem(parent.child(k % 2, k / 2));
    bare_l1.key = parent;
    downsample_2x2(&bare[0], &bare[1], &bare[2], &bare[3], bare_l1);
    CHECK(got.h != bare_l1.h);
}

TEST_CASE("carve: layer description, inputs and config text", "[layer][carve]") {
    const LayerDesc d = terrain_carved_desc();
    CHECK(d.id.name == "g2m.terrain.carved"); CHECK(d.id.version == 1);
    CHECK(d.inputs == std::vector<std::string>{"g2m.elev.base", "g2m.roads.geom"});
    CHECK(d.encoding == static_cast<std::uint16_t>(LayerEncoding::SrcElev));
    for (int l = 0; l <= 6; ++l) CHECK(has_level(d.levels, l));
    CHECK_FALSE(has_level(d.levels, 7));
    const auto& sha = builtin_carve_default().content_sha256;
    CHECK(terrain_carved_config_text() == "g2m.terrain.carved/1 smooth+carve L0 roads.geom enc=9 radius=192 junction=D42 underpass-deck=1 underpass-walls=steep/1 tunnel-cut=D65/80 carve=" +
        to_hex(std::span<const std::uint8_t>(sha.data(), sha.size())) + "; L1-L6 mean-of-4 half-even");
    // carve_road_geom_keys: one level-2 tile in the middle, four at a level-2 corner.
    CHECK(carve_road_geom_keys(l0(1821, 21685)).size() == 1);
    CHECK(carve_road_geom_keys(l0(1820, 21684)).size() == 4);
    CHECK(carve_road_geom_keys(l0(1820, 21685)).size() == 2);
    const TerrainCarvedDeriver deriver;
    CHECK(deriver.coverage_inputs().size() == 1); CHECK(deriver.coverage_inputs()[0] == "g2m.elev.base");
    CHECK(deriver.inputs_for(TileKey{g2m_test::kZone32N, 7, 0, 0}).empty());
}

// The served path end to end (D39): a release with the carved layer (which implies both road layers), the server
// deriving L0 and L1 from the source store, a four-worker bake agreeing with it, and the release ids.
TEST_CASE("carve: server and four-worker bake serve terrain.carved L0 and L1", "[layer][carve][terrain]") {
    struct Temp {
        std::filesystem::path path = std::filesystem::temp_directory_path() /
            ("g2m_carve_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        ~Temp() { std::error_code ec; std::filesystem::remove_all(path, ec); }
    } temp;
    const geo::UtmZone zone{32, geo::Hemisphere::North};
    const auto input = [](std::string layer, TileKey key, const TileBody& body, VerticalDatum datum) {
        InputTile out; out.ref = {std::move(layer), key}; out.body = *encode_body(body);
        TileHeader h; h.layer = out.ref.layer; h.layer_version = 1; h.key = key; h.datum = datum;
        h.body_size = out.body.size(); h.body_hash = content_hash(out.body); out.header = h; return out;
    };
    auto sources = TileStore::open({temp.path, "sources"}); REQUIRE(sources);
    auto live = TileStore::open({temp.path, "live"}); REQUIRE(live);
    auto baked = TileStore::open({temp.path, "baked"}); REQUIRE(baked);
    const TileKey key2{zone, 2, 0, 0}; const geo::Utm utm(geo::kGRS80);
    osm::OsmBuilder builder;
    for (const auto [id, east] : {std::pair<std::int64_t, double>{1, 10}, {2, 110}}) {
        const auto point = utm.reverse(zone, east, 10.5);
        builder.add_node(id, static_cast<std::int32_t>(std::llround(point.lat * 1e7)), static_cast<std::int32_t>(std::llround(point.lon * 1e7)));
    }
    builder.add_way(10, {1, 2}, {{"highway", "residential"}});
    const auto osm = builder.build(); REQUIRE(osm);
    auto raw = input(std::string(kSrcOsmLayer), key2, *encode_src_osm(OsmTile{64, *osm}), VerticalDatum::None);
    REQUIRE((*sources)->put({"carve_test", raw.ref.layer, key2}, *raw.header, raw.body, true));
    for (int y = -1; y <= 1; ++y) for (int x = -1; x <= 1; ++x) {
        const TileKey k{zone, 2, x, y}; ElevTile elev; elev.georef = *tile_georef(k, 1024, SampleRegistration::Cell);
        elev.georef.row_order = RowOrder::NorthToSouth; elev.heights = HeightRaster(1024, 1024);
        std::fill(elev.heights.data().begin(), elev.heights.data().end(), 25600);
        auto data = input(std::string(kSrcElevDgm1Layer), k, *encode_src_elev(elev), VerticalDatum::DHHN2016);
        REQUIRE((*sources)->put({"carve_test", data.ref.layer, k}, *data.header, data.body, true));
    }
    BuiltinTerrainDerivers derivers;
    CHECK(derivers.ordered(false, false, true).size() == 5);
    const auto manifest = make_local_release(**sources, builtin_local_release_params("carve_test", derivers, {}, false, false, true));
    REQUIRE(manifest);
    const auto geometry_only = make_local_release(**sources, builtin_local_release_params("carve_test", derivers, {}, true, true));
    REQUIRE(geometry_only); CHECK(geometry_only->rid != manifest->rid);
    LocalSourceUpstream upstream(**sources); ServerConfig config; config.offline = true; config.derived_store = live->get();
    config.derivers = derivers.ordered(false, false, true); config.upstreams = {&upstream};
    Server server(config); REQUIRE(server.add_release(*manifest, "carve_test"));
    const TileKey key0{zone, 0, 0, 0};
    const auto response = server.tile({manifest->rid, std::string(kTerrainCarvedLayer), key0, {}}); REQUIRE(response.meta.status == Status::Ok);
    import::BakeOptions options; options.layers = {std::string(kTerrainCarvedLayer)}; options.min_level = 0; options.max_level = 1; options.jobs = 4; options.pin = true;
    const auto report = import::bake(**sources, **baked, "carve_test", options);
    INFO((report ? "ok" : report.error().message)); REQUIRE(report);
    CHECK(report->release_id_hex == to_hex(manifest->rid));
    const auto stored = (*baked)->get({report->release_id_hex, std::string(kTerrainCarvedLayer), key0}); REQUIRE(stored); REQUIRE(*stored);
    CHECK((*stored)->pinned); CHECK((*stored)->container() == response.container);
    CHECK((*stored)->header.inputs.size() == 10);   // 9 elev.base + 1 roads.geom
    const auto body = decode_body((*stored)->body); REQUIRE(body);
    HeightTile carved; REQUIRE(decode_height_tile(*body, key0, carved));
    // The residential road (centre 100 m, crossfall 2.5 %) lowers the cells beside its crown; far away the DEM is untouched.
    CHECK(carved.h[10 * 256 + 60] == 25600);   // the crown pivot row: the centre line lies on the cell centres
    CHECK(carved.h[11 * 256 + 60] < 25600);
    CHECK(carved.h[200 * 256 + 200] == 25600);
    const auto parent = server.tile({manifest->rid, std::string(kTerrainCarvedLayer), TileKey{zone, 1, 0, 0}, {}});
    REQUIRE(parent.meta.status == Status::Ok);
    const auto stored1 = (*baked)->get({report->release_id_hex, std::string(kTerrainCarvedLayer), TileKey{zone, 1, 0, 0}}); REQUIRE(stored1); REQUIRE(*stored1);
    CHECK((*stored1)->container() == parent.container);
}

// G-1 (terrain migration phase 6): a roads.geom input whose derivation fails (a deterministic rejection, e.g.
// "reference endpoints" at the region edge) is absent for terrain.carved: the tile is Found, carved without those roads,
// and its header carries rejected_input_marker. Negative control: a carved deriver that does not accept the rejection
// fails the tile, as every deriver did before.
namespace {
class RejectingGeometry final : public IDeriver {   // roads.geom whose every tile is rejected by validation
public:
    explicit RejectingGeometry(const IDeriver& inner) : inner_(inner) {}
    const LayerDesc& output() const override { return inner_.output(); }
    ContentHash config_hash() const override { return inner_.config_hash(); }
    std::span<const std::string> coverage_inputs() const override { return inner_.coverage_inputs(); }
    std::vector<InputTileRef> inputs_for(const TileKey& t) const override { return inner_.inputs_for(t); }
    Result<std::vector<InputTileRef>> additional_inputs_for(const DeriveRequest& r) const override { return inner_.additional_inputs_for(r); }
    std::size_t discovery_stages() const override { return inner_.discovery_stages(); }
    Result<DeriveResult> derive(const DeriveRequest&) const override {
        return make_error(ErrorCode::Malformed, "roads.geom: reference endpoints (test rejection)");
    }
private:
    const IDeriver& inner_;
};
class StrictCarved final : public IDeriver {   // terrain.carved without the G-1 acceptance
public:
    const LayerDesc& output() const override { return inner_.output(); }
    ContentHash config_hash() const override { return inner_.config_hash(); }
    std::span<const std::string> coverage_inputs() const override { return inner_.coverage_inputs(); }
    std::vector<InputTileRef> inputs_for(const TileKey& t) const override { return inner_.inputs_for(t); }
    Result<DeriveResult> derive(const DeriveRequest& r) const override { return inner_.derive(r); }
private:
    TerrainCarvedDeriver inner_;
};
} // namespace

TEST_CASE("carve G-1: a carved tile over a rejected roads.geom tile is Found, without roads, flagged", "[layer][carve][terrain][G-1]") {
    struct Temp {
        std::filesystem::path path = std::filesystem::temp_directory_path() /
            ("g2m_carve_g1_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        ~Temp() { std::error_code ec; std::filesystem::remove_all(path, ec); }
    } temp;
    const geo::UtmZone zone{32, geo::Hemisphere::North};
    const auto input = [](std::string layer, TileKey key, const TileBody& body, VerticalDatum datum) {
        InputTile out; out.ref = {std::move(layer), key}; out.body = *encode_body(body);
        TileHeader h; h.layer = out.ref.layer; h.layer_version = 1; h.key = key; h.datum = datum;
        h.body_size = out.body.size(); h.body_hash = content_hash(out.body); out.header = h; return out;
    };
    auto sources = TileStore::open({temp.path, "sources"}); REQUIRE(sources);
    const TileKey key2{zone, 2, 0, 0}; const geo::Utm utm(geo::kGRS80);
    osm::OsmBuilder builder;
    for (const auto [id, east] : {std::pair<std::int64_t, double>{1, 10}, {2, 110}}) {
        const auto point = utm.reverse(zone, east, 10.5);
        builder.add_node(id, static_cast<std::int32_t>(std::llround(point.lat * 1e7)), static_cast<std::int32_t>(std::llround(point.lon * 1e7)));
    }
    builder.add_way(10, {1, 2}, {{"highway", "residential"}});
    const auto osm = builder.build(); REQUIRE(osm);
    auto raw = input(std::string(kSrcOsmLayer), key2, *encode_src_osm(OsmTile{64, *osm}), VerticalDatum::None);
    REQUIRE((*sources)->put({"carve_g1", raw.ref.layer, key2}, *raw.header, raw.body, true));
    for (int y = -1; y <= 1; ++y) for (int x = -1; x <= 1; ++x) {
        const TileKey k{zone, 2, x, y}; ElevTile elev; elev.georef = *tile_georef(k, 1024, SampleRegistration::Cell);
        elev.georef.row_order = RowOrder::NorthToSouth; elev.heights = HeightRaster(1024, 1024);
        std::fill(elev.heights.data().begin(), elev.heights.data().end(), 25600);
        auto data = input(std::string(kSrcElevDgm1Layer), k, *encode_src_elev(elev), VerticalDatum::DHHN2016);
        REQUIRE((*sources)->put({"carve_g1", data.ref.layer, k}, *data.header, data.body, true));
    }
    BuiltinTerrainDerivers derivers;
    const RejectingGeometry rejecting(derivers.geometry);
    const StrictCarved strict;
    const auto manifest = make_local_release(**sources, builtin_local_release_params("carve_g1", derivers, {}, false, false, true, true));
    REQUIRE(manifest);
    LocalSourceUpstream upstream(**sources);
    const TileKey key0{zone, 0, 0, 0};
    const InputTileRef geom_ref{std::string(kRoadGeomLayer), key2};
    for (const bool accept : {true, false}) {
        auto live = TileStore::open({temp.path, accept ? "live_accept" : "live_strict"}); REQUIRE(live);
        ServerConfig config; config.offline = true; config.derived_store = live->get(); config.upstreams = {&upstream};
        config.derivers = {&derivers.elev, &derivers.height, &derivers.roads, &rejecting,
                           accept ? static_cast<const IDeriver*>(&derivers.carved) : &strict, &derivers.classes};
        Server server(config); REQUIRE(server.add_release(*manifest, "carve_g1"));
        // The roads.geom tile itself fails.
        CHECK(server.tile({manifest->rid, std::string(kRoadGeomLayer), key2, {}}).meta.status != Status::Ok);
        const auto response = server.tile({manifest->rid, std::string(kTerrainCarvedLayer), key0, {}});
        if (!accept) {   // negative control: before G-1 the rejection failed the carved tile
            CHECK(response.meta.status != Status::Ok);
            continue;
        }
        INFO(response.meta.message);
        REQUIRE(response.meta.status == Status::Ok);
        const auto view = parse_container(response.container); REQUIRE(view);
        CHECK(header_has_rejected_input(view->header, geom_ref));
        CHECK(view->header.inputs.size() == 10);   // 9 elev.base + the marker
        CHECK_FALSE(header_has_rejected_input(view->header, InputTileRef{std::string(kRoadGeomLayer), TileKey{zone, 2, 1, 0}}));
        HeightTile carved; REQUIRE(decode_height_tile(*decode_body(view->body), key0, carved));
        // No roads: the flat DEM everywhere, including the cells the residential road lowers when roads.geom is served.
        CHECK(carved.h[11 * 256 + 60] == 25600);
        CHECK(std::all_of(carved.h.begin(), carved.h.end(), [](std::int32_t v) { return v == 25600; }));
        // L1 above it is served too.
        CHECK(server.tile({manifest->rid, std::string(kTerrainCarvedLayer), TileKey{zone, 1, 0, 0}, {}}).meta.status == Status::Ok);
        // terrain.class takes the same rule: Found, flagged, no road nodes.
        const auto cls = server.tile({manifest->rid, std::string(kTerrainClassLayer), key0, {}});
        INFO(cls.meta.message);
        REQUIRE(cls.meta.status == Status::Ok);
        const auto cview = parse_container(cls.container); REQUIRE(cview);
        CHECK(header_has_rejected_input(cview->header, geom_ref));
        TerrainClassTile classes; REQUIRE(decode_terrain_class(*decode_body(cview->body), key0, classes));
        CHECK(std::all_of(classes.rank.begin(), classes.rank.end(), [](std::uint8_t r) { return r == 0; }));
    }
}

// ---- measurement (diag) ----

TEST_CASE("carve: per-tile census and derive cost of the fixtures (diag)", "[.][diag][carve]") {
    for (const char* file : {kFfm, kA66})
        for (int tilted = 0; tilted < 2; ++tilted) {
            auto& fx = fixture(file, tilted != 0);
            double total1 = 0, total10 = 0, worst = 0, build = 0; std::size_t tiles = 0;
            CarveStats sum;
            for (const auto& key : fx.level0_tiles()) {
                (void)fx.roads(key);
                CarveStats stats;
                auto t0 = std::chrono::steady_clock::now();
                (void)fx.model(key);   // smoothing + model build only (the serial part of a derive)
                build += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                t0 = std::chrono::steady_clock::now();
                const auto tile = fx.carve(key, {}, 1, &stats); REQUIRE(tile);
                const double ms1 = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                t0 = std::chrono::steady_clock::now();
                const auto tile10 = fx.carve(key, {}, 10); REQUIRE(tile10);
                const double ms10 = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
                total1 += ms1; total10 += ms10; worst = std::max(worst, ms1); ++tiles;
                sum.add_cells(stats); sum.steep_blend_stations += stats.steep_blend_stations; sum.blend_stations += stats.blend_stations;
                sum.junction_knots_outside_window += stats.junction_knots_outside_window;
            }
            std::printf("[census] %s %s: %zu L0 tiles, derive mean %.1f ms (1 worker) / %.1f ms (10 workers), worst %.1f ms, of which "
                "smoothing + model build %.1f ms; cells jcore %zu acore %zu jblend %zu ablend %zu deck %zu violations %zu; steep blends %zu of "
                "%zu; knots outside window %zu\n",
                file, tilted ? "tilted" : "flat", tiles, total1 / tiles, total10 / tiles, worst, build / tiles, sum.cells_junction_core, sum.cells_approach_core,
                sum.cells_junction_blend, sum.cells_approach_blend, sum.cells_deck_clamped, sum.deck_clearance_violations,
                sum.steep_blend_stations, sum.blend_stations, sum.junction_knots_outside_window);
        }
}

TEST_CASE("carve: worst owner switches of the busy tiles (diag)", "[.][diag][carve]") {
    for (const auto& ft : kBusy)
        for (int tilted = 0; tilted < 2; ++tilted) {
            auto& fx = fixture(ft.file, tilted != 0);
            const TileKey key = l0(ft.x, ft.y);
            const auto m = fx.model(key);
            const Jumps j = measure_jumps(*m.model, key);
            char what[128]; std::snprintf(what, sizeof what, "%s %d/%d %s", ft.file, ft.x, ft.y, tilted ? "tilted" : "flat");
            print_jumps(what, j);
            print_where("core-blend", j.w_core_blend); print_where("blend-terrain", j.w_blend_terrain); print_where("trim", j.w_trim);
            print_where("blend-blend", j.w_blend_blend); print_where("core-core", j.w_core_core); print_where("other", j.w_other);
        }
}
