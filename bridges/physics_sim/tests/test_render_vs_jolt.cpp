// bridges/physics_sim/tests/test_render_vs_jolt.cpp - render vs Jolt
// (PLAN.md R2.2 section 4b, G7): the g2m_mesh L0 render chunk
// (build_chunk) and physics_sim's Jolt heightfield tiles (filled through
// G2mTerrainSource by a real World's TileManager) must be the SAME surface -
// same sample lattice (physics_grid.h's anchoring) and the same quad
// diagonal (terrain_chunk.h's build_chunk derivation) - up to Jolt's own
// height quantisation (vault PHYS-044).
//
// Method: prime a World on the terrain, then at 4096 fixed LCG points
// strictly inside L0 quads (u, v in [0.01, 0.99]) compare
//   - Jolt: backend().ray_cast straight down through the point, and
//   - render: barycentric interpolation inside the build_chunk triangle that
//     contains the point, found through that chunk's OWN index buffer
//     (positions + origin), so a wrong render diagonal is caught too.
//
// Checkerboard saddle: +-51/256 m (~+-0.2 m) about 100 m, alternating per
// DGM1 cell - the worst case for the diagonal, because the two
// triangulations of one quad differ by up to 0.4 m at its centre (the
// mean difference over a quad is 0.4/3 m). Negative control: the same
// vertices triangulated along the OTHER diagonal must disagree with Jolt by
// far more than the tolerance. Fixture: the real PKT DGM1 fixture release
// (tests/support/fixture_release), bounds max <= 5 mm, p99 <= 2 mm.
#include "bridge_test_support.h"

#include "fixture_release.h"

#include "g2m/mesh/terrain_chunk.h"
#include "g2m/phys/height_tile_loader.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"
#include "g2m/world/height_sampler.h"

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

using namespace g2m_bridge_test;
using g2m::TileKey;
using g2m::mesh::ChunkKey;
using g2m::mesh::HeightWindow;
using g2m::mesh::TerrainChunkMesh;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;
using g2m::phys::ResidentHeightSet;
using g2m::ps_bridge::G2mTerrainSource;
using g2m::ps_bridge::make_terrain_config;

namespace {

constexpr int kQuads = g2m::mesh::kChunkQuads; // 64
constexpr int kVerts = g2m::mesh::kChunkVerts; // 65

const g2m::HeightTile* lookup_resident(void* ctx, const TileKey& key) {
    const g2m::HeightTile* tile = nullptr;
    static_cast<const ResidentHeightSet*>(ctx)->find(key, &tile);
    return tile;
}

// Lazily builds the L0 render chunks a set of sample points needs, from the
// same ResidentHeightSet the physics source reads.
class L0MeshSampler {
public:
    L0MeshSampler(const ResidentHeightSet& set, g2m::geo::UtmZone zone)
        : set_(set), zone_(zone), window_(std::make_unique<HeightWindow>()) {}

    struct Sample {
        bool found = false;        // the chunk's own index buffer has a triangle containing the point
        double z_render = 0.0;     // barycentric in that triangle
        double z_wrong_diag = 0.0; // same vertices, the OTHER diagonal
        int bad_quad_triangles = 0; // index-buffer triangles that do not lie on exactly one quad
    };

    // (c, r): global DGM1 cell column/row of the quad's SW vertex; (u, v):
    // fractional position inside the quad, strictly in (0, 1).
    Sample sample(std::int64_t c, std::int64_t r, double u, double v) {
        const std::int64_t cx = g2m::floor_div(c, kQuads);
        const std::int64_t cy = g2m::floor_div(r, kQuads);
        const int i = static_cast<int>(c - cx * kQuads);
        const int j = static_cast<int>(r - cy * kQuads);
        Chunk& chunk = get(static_cast<std::int32_t>(cx), static_cast<std::int32_t>(cy));
        const TerrainChunkMesh& m = *chunk.mesh;

        Sample out;
        out.bad_quad_triangles = chunk.bad_quad_triangles;
        // Chunk-local point: vertex (i, j) sits at local (i, j) relative to
        // origin (terrain_chunk.h: origin = vertex (0,0), 1 m spacing at L0).
        const double px = static_cast<double>(i) + u;
        const double py = static_cast<double>(j) + v;
        auto vx = [&](int idx, int k) { return static_cast<double>(m.positions[static_cast<std::size_t>(3 * idx + k)]); };

        const std::size_t q = static_cast<std::size_t>(j * kQuads + i) * 2;
        for (int t = 0; t < 2; ++t) {
            const std::int32_t tri = chunk.quad_tris[q + static_cast<std::size_t>(t)];
            if (tri < 0) continue;
            const int a = m.indices[static_cast<std::size_t>(3 * tri)];
            const int b = m.indices[static_cast<std::size_t>(3 * tri + 1)];
            const int cc = m.indices[static_cast<std::size_t>(3 * tri + 2)];
            const double ax = vx(a, 0), ay = vx(a, 1), bx = vx(b, 0), by = vx(b, 1), cxv = vx(cc, 0), cyv = vx(cc, 1);
            const double d = (by - cyv) * (ax - cxv) + (cxv - bx) * (ay - cyv);
            const double wa = ((by - cyv) * (px - cxv) + (cxv - bx) * (py - cyv)) / d;
            const double wb = ((cyv - ay) * (px - cxv) + (ax - cxv) * (py - cyv)) / d;
            const double wc = 1.0 - wa - wb;
            constexpr double kEps = 1e-12;
            if (wa < -kEps || wb < -kEps || wc < -kEps) continue;
            out.found = true;
            out.z_render = m.origin[2] + wa * vx(a, 2) + wb * vx(b, 2) + wc * vx(cc, 2);
            break;
        }

        // Negative control: the (i,j)-(i+1,j+1) diagonal over the same four
        // vertex heights.
        const double z00 = vx(j * kVerts + i, 2);
        const double z10 = vx(j * kVerts + i + 1, 2);
        const double z01 = vx((j + 1) * kVerts + i, 2);
        const double z11 = vx((j + 1) * kVerts + i + 1, 2);
        out.z_wrong_diag = m.origin[2] + (u >= v ? z00 + u * (z10 - z00) + v * (z11 - z10)
                                                 : z00 + v * (z01 - z00) + u * (z11 - z01));
        return out;
    }

    std::size_t chunk_count() const { return chunks_.size(); }

private:
    struct Chunk {
        std::unique_ptr<TerrainChunkMesh> mesh;
        std::vector<std::int32_t> quad_tris; // 2 per quad, -1 = none
        int bad_quad_triangles = 0;
    };

    Chunk& get(std::int32_t cx, std::int32_t cy) {
        const auto k = std::make_pair(cx, cy);
        auto it = chunks_.find(k);
        if (it != chunks_.end()) return it->second;
        const ChunkKey key{zone_, 0, cx, cy};
        g2m::mesh::gather_window(key, &lookup_resident, const_cast<ResidentHeightSet*>(&set_), *window_);
        Chunk chunk;
        chunk.mesh = std::make_unique<TerrainChunkMesh>();
        g2m::mesh::build_chunk(key, *window_, nullptr, *chunk.mesh);
        chunk.quad_tris.assign(static_cast<std::size_t>(kQuads) * kQuads * 2, -1);
        const TerrainChunkMesh& m = *chunk.mesh;
        const std::size_t tri_count = m.indices.size() / 3;
        for (std::size_t t = 0; t < tri_count; ++t) {
            int mi = kVerts, mj = kVerts, xi = -1, xj = -1;
            bool skirt = false;
            for (int k = 0; k < 3; ++k) {
                const int idx = m.indices[3 * t + static_cast<std::size_t>(k)];
                if (static_cast<std::uint32_t>(idx) >= m.skirt_first_vertex) skirt = true;
                mi = std::min(mi, idx % kVerts), xi = std::max(xi, idx % kVerts);
                mj = std::min(mj, idx / kVerts), xj = std::max(xj, idx / kVerts);
            }
            if (skirt) continue;
            if (xi - mi != 1 || xj - mj != 1) {
                ++chunk.bad_quad_triangles;
                continue;
            }
            const std::size_t q = static_cast<std::size_t>(mj * kQuads + mi) * 2;
            if (chunk.quad_tris[q] < 0) {
                chunk.quad_tris[q] = static_cast<std::int32_t>(t);
            } else if (chunk.quad_tris[q + 1] < 0) {
                chunk.quad_tris[q + 1] = static_cast<std::int32_t>(t);
            } else {
                ++chunk.bad_quad_triangles;
            }
        }
        return chunks_.emplace(k, std::move(chunk)).first->second;
    }

    const ResidentHeightSet& set_;
    g2m::geo::UtmZone zone_;
    std::unique_ptr<HeightWindow> window_; // 18 KB, heap (never a stack local)
    std::map<std::pair<std::int32_t, std::int32_t>, Chunk> chunks_;
};

struct Comparison {
    int points = 0;
    int ray_misses = 0;
    int render_misses = 0;
    int bad_quad_triangles = 0;
    double max_err = 0.0;
    double p99_err = 0.0;
    double mean_err = 0.0;
    double wrong_max = 0.0;
    double wrong_mean = 0.0;
    // g2m::world::sample_height_exact (the physics-exact sampler) against the
    // same Jolt ray hit.
    int exact_misses = 0;
    double exact_max_err = 0.0;
    double exact_p99_err = 0.0;
    double bilinear_max_diff = 0.0; // |sample_height_bilinear - Jolt|: the render-only sampler is NOT the physics surface
};

// 4096 LCG points in L0 quads whose four vertices are samples in the
// physics-tile square [ix0..ix1] x [iy0..iy1] (the World's own resident
// TileManager tiles).
Comparison compare(ps::World& world, const ResidentHeightSet& resident, const PhysicsTileGrid& grid, PhysTileIndex lo,
                   PhysTileIndex hi, std::uint64_t seed) {
    L0MeshSampler sampler(resident, grid.frame().zone());
    const std::int64_t ts = static_cast<std::int64_t>(grid.tile_size_m());
    // Sample k of physics tile ix is DGM1 cell column e0 + ts*ix + k
    // (physics_grid.h's anchoring): the square spans cells
    // [e0 + ts*lo, e0 + ts*(hi+1)], so quads start in [.., last cell - 1].
    const std::int64_t c_lo = kE0 + ts * lo.ix, c_hi = kE0 + ts * (hi.ix + 1) - 1;
    const std::int64_t r_lo = kN0 + ts * lo.iy, r_hi = kN0 + ts * (hi.iy + 1) - 1;

    Lcg lcg(seed);
    std::vector<double> errs;
    std::vector<double> exact_errs;
    errs.reserve(4096);
    exact_errs.reserve(4096);
    const g2m::world::L0TileLookupFn lookup = [&resident](const TileKey& key) -> const g2m::HeightTile* {
        const g2m::HeightTile* tile = nullptr;
        resident.find(key, &tile);
        return tile;
    };
    const g2m::geo::UtmZone zone = grid.frame().zone();
    Comparison out;
    double wrong_sum = 0.0;
    for (int n = 0; n < 4096; ++n) {
        const std::int64_t c = lcg.range(c_lo, c_hi);
        const std::int64_t r = lcg.range(r_lo, r_hi);
        const double u = lcg.uniform(0.01, 0.99);
        const double v = lcg.uniform(0.01, 0.99);
        const L0MeshSampler::Sample s = sampler.sample(c, r, u, v);
        out.bad_quad_triangles = std::max(out.bad_quad_triangles, s.bad_quad_triangles);
        if (!s.found) {
            ++out.render_misses;
            continue;
        }
        const double x = static_cast<double>(c - kE0) + 0.5 + u;
        const double y = static_cast<double>(r - kN0) + 0.5 + v;
        const ps::RayCastHit hit = world.backend().ray_cast(ps::Vec3{x, y, s.z_render + 2.0}, ps::Vec3{0.0, 0.0, -1.0}, 4.0);
        if (!hit.hit) {
            ++out.ray_misses;
            continue;
        }
        const double err = std::abs(hit.point.z - s.z_render);
        const double wrong = std::abs(hit.point.z - s.z_wrong_diag);
        const std::optional<double> exact = g2m::world::sample_height_exact(lookup, zone, kE0, kN0, x, y);
        if (!exact) {
            ++out.exact_misses;
        } else {
            const double exact_err = std::abs(hit.point.z - *exact);
            exact_errs.push_back(exact_err);
            out.exact_max_err = std::max(out.exact_max_err, exact_err);
        }
        if (const std::optional<double> bil = g2m::world::sample_height_bilinear(lookup, zone, kE0, kN0, x, y)) {
            out.bilinear_max_diff = std::max(out.bilinear_max_diff, std::abs(hit.point.z - *bil));
        }
        errs.push_back(err);
        out.max_err = std::max(out.max_err, err);
        out.mean_err += err;
        out.wrong_max = std::max(out.wrong_max, wrong);
        wrong_sum += wrong;
        ++out.points;
    }
    if (out.points > 0) {
        out.mean_err /= out.points;
        out.wrong_mean = wrong_sum / out.points;
        std::sort(errs.begin(), errs.end());
        out.p99_err = errs[static_cast<std::size_t>(std::floor(0.99 * static_cast<double>(errs.size() - 1)))];
    }
    if (!exact_errs.empty()) {
        std::sort(exact_errs.begin(), exact_errs.end());
        out.exact_p99_err =
            exact_errs[static_cast<std::size_t>(std::floor(0.99 * static_cast<double>(exact_errs.size() - 1)))];
    }
    std::printf("render-vs-jolt: %d points (%zu chunks), ray misses %d, render misses %d, max %.4f mm, p99 %.4f mm, "
                "mean %.4f mm; other diagonal: max %.1f mm, mean %.1f mm\n",
                out.points, sampler.chunk_count(), out.ray_misses, out.render_misses, out.max_err * 1e3, out.p99_err * 1e3,
                out.mean_err * 1e3, out.wrong_max * 1e3, out.wrong_mean * 1e3);
    std::printf("sample_height_exact vs Jolt: misses %d, max %.4f mm, p99 %.4f mm; sample_height_bilinear vs Jolt: max %.1f mm\n",
                out.exact_misses, out.exact_max_err * 1e3, out.exact_p99_err * 1e3, out.bilinear_max_diff * 1e3);
    return out;
}

// Steps until TileManager has the whole (2r+1)^2 square resident (at F = 1,
// K = ceil(n/F) + 1 ticks, PLAN.md R2.2 section 2 "Prime").
void prime(ps::World& world, std::size_t tiles, std::uint32_t fills_per_tick) {
    const std::size_t k = (tiles + fills_per_tick - 1) / fills_per_tick + 1;
    for (std::size_t t = 0; t < k; ++t) world.step();
}

std::int32_t checker_saddle(std::int64_t c, std::int64_t r) { return 25600 + (((c + r) & 1) != 0 ? -51 : 51); }

} // namespace

TEST_CASE("render vs Jolt: checkerboard saddle - L0 mesh and Jolt tiles are one surface", "[bridge][terrain][render]") {
    const PhysicsTileGrid grid(test_frame());
    SyntheticTileCache cache(&checker_saddle);
    auto resident = std::make_shared<ResidentHeightSet>();
    install_square(grid, *resident, PhysTileIndex{1, 1}, 2, cache);
    auto source = std::make_shared<G2mTerrainSource>(grid, resident, ps::SurfaceId{0});

    ps::WorldConfig cfg;
    cfg.job_workers = 4;
    ps::World world(cfg);
    world.set_terrain_source(source, make_terrain_config(255.0, 1));
    // Centre of physics tile (1,1), radius 255 -> TileManager square (0..2, 0..2).
    world.set_terrain_interest_point(1, ps::Vec3{382.5, 382.5, 0.0}, 255.0);
    prime(world, 9, 1);
    REQUIRE(world.terrain_resident_tile_count() >= 9);
    REQUIRE(world.terrain_starved_tile_count() == 0);
    REQUIRE(source->fill_miss_count() == 0);

    const Comparison c = compare(world, *resident, grid, PhysTileIndex{0, 0}, PhysTileIndex{2, 2}, 0x5AD1Eull);
    CHECK(c.bad_quad_triangles == 0);
    CHECK(c.render_misses == 0);
    CHECK(c.ray_misses == 0);
    CHECK(c.points == 4096);
    CHECK(c.max_err <= 0.002);
    // Negative control: the other diagonal is off by up to ~0.4 m at a quad
    // centre and ~0.13 m on average - far outside the 2 mm tolerance.
    CHECK(c.wrong_max >= 0.1);
    CHECK(c.wrong_mean >= 0.05);
    // The physics-exact sampler is that same Jolt surface (same tolerance);
    // the bilinear one is not - on a checkerboard it is ~0.1 m off.
    CHECK(c.exact_misses == 0);
    CHECK(c.exact_max_err <= 0.002);
    CHECK(c.bilinear_max_diff >= 0.05);
}

TEST_CASE("render vs Jolt: PKT DGM1 fixture - max <= 5 mm, p99 <= 2 mm", "[bridge][terrain][render][fixture]") {
    TempDir dir("render_vs_jolt");
    std::unique_ptr<g2m::test_support::ReleaseFixture> fx = g2m::test_support::make_fixture_release(dir.path(), "g7render");
    const PhysicsTileGrid grid(fx->session_frame());
    REQUIRE(fx->session_frame().e0_m() == kE0);
    REQUIRE(fx->session_frame().n0_m() == kN0);

    // Local (1000, 400): physics tile (3, 1); r = 255 -> TileManager square
    // (2..4, 0..2) = local x 510.5..1275.5, y 0.5..765.5, fully inside the
    // fixture's 2 km x 1 km coverage.
    const PhysTileIndex centre = grid.index_for_local(1000.0, 400.0);
    REQUIRE(centre == PhysTileIndex{3, 1});
    auto resident = std::make_shared<ResidentHeightSet>();
    g2m::phys::TransportHeightTileFetch fetch(*fx->transport, fx->rid);
    std::vector<TileKey> keys;
    grid.l0_keys_for_square(centre, 2, keys);
    int ready = 0, absent = 0;
    for (const TileKey& key : keys) {
        const g2m::phys::FetchResult r = fetch.fetch(key);
        if (r.status == g2m::Status::Ok) {
            REQUIRE(r.tile != nullptr);
            resident->install(key, r.tile);
            ++ready;
        } else {
            REQUIRE(r.status == g2m::Status::NotFound);
            resident->install(key, nullptr);
            ++absent;
        }
    }
    std::printf("render-vs-jolt fixture: %d L0 tiles ready, %d absent\n", ready, absent);
    REQUIRE(ready > 0);

    auto source = std::make_shared<G2mTerrainSource>(grid, resident, ps::SurfaceId{0});
    ps::WorldConfig cfg;
    cfg.job_workers = 4;
    ps::World world(cfg);
    world.set_terrain_source(source, make_terrain_config(255.0, 1));
    world.set_terrain_interest_point(1, ps::Vec3{1000.0, 400.0, 0.0}, 255.0);
    prime(world, 9, 1);
    REQUIRE(world.terrain_starved_tile_count() == 0);
    REQUIRE(world.terrain_relief_overflow_count() == 0);
    REQUIRE(source->fill_miss_count() == 0);

    const Comparison c = compare(world, *resident, grid, PhysTileIndex{2, 0}, PhysTileIndex{4, 2}, 0xF1C57ull);
    CHECK(c.bad_quad_triangles == 0);
    CHECK(c.render_misses == 0);
    CHECK(c.ray_misses == 0);
    CHECK(c.points == 4096);
    CHECK(c.max_err <= 0.005);
    CHECK(c.p99_err <= 0.002);
    CHECK(c.exact_misses == 0);
    CHECK(c.exact_max_err <= 0.005);
    CHECK(c.exact_p99_err <= 0.002);
}
