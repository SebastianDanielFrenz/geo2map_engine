// tests/mesh/test_terrain_chunk.cpp - g2m::mesh::gather_window/build_chunk
// (PLAN.md G2.3): shared-border bit-identity, z == h/256 exactness, no
// NoData-vertex indices, the skirt-depth formula, and build_chunk's
// zero-allocation second call.

#include "g2m/mesh/terrain_chunk.h"

#include "alloc_counter.h"
#include "synthetic_source.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>

using namespace g2m;
using namespace g2m::mesh;
using geo::Hemisphere;
using geo::UtmZone;

namespace {
constexpr UtmZone k32N{32, Hemisphere::North};

TerrainChunkMesh build(const ChunkKey& key, g2m_test::SyntheticHeightSource& src) {
    HeightWindow window;
    gather_window(key, &g2m_test::SyntheticHeightSource::lookup, &src, window);
    TerrainChunkMesh mesh;
    build_chunk(key, window, /*classes=*/nullptr, mesh);
    return mesh;
}

// float(kHeightNoData) / 256.0f - see the header comment above build_chunk:
// this is the ONLY z value a NoData vertex can ever produce (both are exact
// powers of two, so the division loses no bits), so checking z against this
// sentinel is an exact, not approximate, NoData test.
constexpr float kNoDataZ = static_cast<float>(kHeightNoData) / 256.0f;

} // namespace

TEST_CASE("build_chunk: every base-grid z equals h/256 exactly", "[mesh]") {
    g2m_test::SyntheticHeightSource src(g2m_test::sinusoidal_height(30.0, 150.0));
    const ChunkKey key{k32N, 3, 5, -2};
    HeightWindow window;
    gather_window(key, &g2m_test::SyntheticHeightSource::lookup, &src, window);
    TerrainChunkMesh mesh;
    build_chunk(key, window, nullptr, mesh);

    for (int j = 0; j < kChunkVerts; ++j) {
        for (int i = 0; i < kChunkVerts; ++i) {
            const int vi = j * kChunkVerts + i;
            const std::int32_t h = window.h[static_cast<std::size_t>(j + 1) * kHaloWindow + static_cast<std::size_t>(i + 1)];
            const float expected = static_cast<float>(h) / 256.0f;
            CHECK(mesh.positions[static_cast<std::size_t>(vi) * 3 + 2] == expected);
        }
    }
}

TEST_CASE("build_chunk: no index references a NoData vertex", "[mesh]") {
    // A source that is NoData for every sample whose absolute i is odd -
    // guarantees some base quads (and some skirt segments) get dropped.
    g2m_test::SyntheticHeightSource src([](int, std::int64_t ai, std::int64_t) -> std::int32_t {
        return (ai % 2 != 0) ? kHeightNoData : 1000;
    });
    const ChunkKey key{k32N, 2, 0, 0};
    TerrainChunkMesh mesh = build(key, src);

    REQUIRE(mesh.indices.size() % 3 == 0);
    for (std::uint16_t idx : mesh.indices) {
        const float z = mesh.positions[static_cast<std::size_t>(idx) * 3 + 2];
        CHECK(z != kNoDataZ);
    }
}

TEST_CASE("build_chunk: shared border is bit-identical between same-level neighbours", "[mesh]") {
    g2m_test::SyntheticHeightSource src(g2m_test::sinusoidal_height(30.0, 150.0));
    const std::uint8_t level = 2;
    const ChunkKey west{k32N, level, 10, -3};
    const ChunkKey east{k32N, level, 11, -3};

    const TerrainChunkMesh mw = build(west, src);
    const TerrainChunkMesh me = build(east, src);

    // west's east border (i = kChunkQuads) vs east's west border (i = 0),
    // both compared as ABSOLUTE double positions (origin + local), which
    // must be bit-identical - see build_chunk's own header comment on
    // exactness (origin a half-integer multiple of 2^k, local an exact
    // integer multiple of 2^k).
    for (int j = 0; j < kChunkVerts; ++j) {
        const int vw = j * kChunkVerts + kChunkQuads;
        const int ve = j * kChunkVerts + 0;

        const double xw = mw.origin[0] + static_cast<double>(mw.positions[static_cast<std::size_t>(vw) * 3 + 0]);
        const double xe = me.origin[0] + static_cast<double>(me.positions[static_cast<std::size_t>(ve) * 3 + 0]);
        const double yw = mw.origin[1] + static_cast<double>(mw.positions[static_cast<std::size_t>(vw) * 3 + 1]);
        const double ye = me.origin[1] + static_cast<double>(me.positions[static_cast<std::size_t>(ve) * 3 + 1]);
        const double zw = mw.origin[2] + static_cast<double>(mw.positions[static_cast<std::size_t>(vw) * 3 + 2]);
        const double ze = me.origin[2] + static_cast<double>(me.positions[static_cast<std::size_t>(ve) * 3 + 2]);
        CHECK(xw == xe);
        CHECK(yw == ye);
        CHECK(zw == ze);

        for (int c = 0; c < 3; ++c) {
            CHECK(mw.normals[static_cast<std::size_t>(vw) * 3 + static_cast<std::size_t>(c)] ==
                  me.normals[static_cast<std::size_t>(ve) * 3 + static_cast<std::size_t>(c)]);
        }
    }
}

TEST_CASE("build_chunk: skirt depth covers the worst 2:1 crack, synthetic sinusoid", "[mesh]") {
    // Independent re-derivation (not calling into build_chunk's own formula):
    // sample the analytic height function directly at every border pair and
    // compute the same worst-crack quantity a second, unrelated way (via
    // double subtraction in metres, not build_chunk's int64/1-256-units path).
    const double amplitude = 30.0, wavelength = 150.0;
    g2m_test::HeightFn fn = g2m_test::sinusoidal_height(amplitude, wavelength);
    g2m_test::SyntheticHeightSource src(fn);

    const ChunkKey key{k32N, 4, -2, 7};
    TerrainChunkMesh mesh = build(key, src);

    const double step_m = static_cast<double>(std::int64_t{1} << key.level);
    const std::int64_t base_i = static_cast<std::int64_t>(kChunkQuads) * key.cx;
    const std::int64_t base_j = static_cast<std::int64_t>(kChunkQuads) * key.cy;

    double worst_m = 0.0;
    auto sample_m = [&](std::int64_t ai, std::int64_t aj) {
        return static_cast<double>(fn(key.level, ai, aj)) / 256.0;
    };
    // South and north borders (fixed aj, varying ai); west/east (fixed ai, varying aj).
    for (int k = 0; k < kChunkQuads / 2; ++k) {
        const std::int64_t a = base_i + 2 * k, b = base_i + 2 * k + 1, c = base_i + 2 * k + 2;
        for (std::int64_t aj : {base_j, base_j + kChunkQuads}) {
            const double crack = std::fabs(2.0 * sample_m(b, aj) - sample_m(a, aj) - sample_m(c, aj));
            worst_m = std::max(worst_m, crack);
        }
        const std::int64_t a2 = base_j + 2 * k, b2 = base_j + 2 * k + 1, c2 = base_j + 2 * k + 2;
        for (std::int64_t ai : {base_i, base_i + kChunkQuads}) {
            const double crack = std::fabs(2.0 * sample_m(ai, b2) - sample_m(ai, a2) - sample_m(ai, c2));
            worst_m = std::max(worst_m, crack);
        }
    }
    (void)step_m;

    // build_chunk's formula: 1.0 m + worst / 2 (undoing the doubled
    // difference) - both computed independently here, both starting from
    // the same analytic height function.
    const float expected = static_cast<float>(1.0 + worst_m / 2.0);
    CHECK(mesh.skirt_depth_m == Catch::Approx(expected).margin(1e-3f));
    CHECK(mesh.skirt_depth_m >= 1.0f);

    // Geometric property: every skirt vertex sits strictly below its own
    // border vertex by exactly skirt_depth_m (both from the same call).
    for (int border = 0; border < 4; ++border) {
        for (int k = 0; k < kChunkVerts; ++k) {
            const int svi = kBaseVertexCount + border * kChunkVerts + k;
            int bi = 0, bj = 0;
            switch (border) {
                case 0: bi = 0; bj = k; break;
                case 1: bi = kChunkQuads; bj = k; break;
                case 2: bi = k; bj = 0; break;
                default: bi = k; bj = kChunkQuads; break;
            }
            const int base_vi = bj * kChunkVerts + bi;
            const float base_z = mesh.positions[static_cast<std::size_t>(base_vi) * 3 + 2];
            const float skirt_z = mesh.positions[static_cast<std::size_t>(svi) * 3 + 2];
            CHECK(base_z - skirt_z == mesh.skirt_depth_m);
        }
    }
}

TEST_CASE("build_chunk: second call with the same shape makes zero allocations", "[mesh]") {
    g2m_test::SyntheticHeightSource src(g2m_test::sinusoidal_height(10.0, 200.0));
    const ChunkKey key{k32N, 3, 1, 1};
    HeightWindow window;
    gather_window(key, &g2m_test::SyntheticHeightSource::lookup, &src, window);

    TerrainChunkMesh mesh;
    build_chunk(key, window, nullptr, mesh); // first call: warms up capacity

    g2m_test::ScopedAllocCounter counter;
    build_chunk(key, window, nullptr, mesh); // second call: must not allocate

    if (!g2m_test::kAllocCounterAvailable) {
        // ASan/TSan builds gate alloc_counter.cpp's global operator new/delete
        // overrides off entirely (they would duplicate-symbol-clash with the
        // sanitizer runtime's own overrides at link time - see that file's
        // header comment), so counter.count() would always read 0 here
        // regardless of whether build_chunk actually allocated. Say so
        // explicitly rather than let a meaningless 0 == 0 pass silently.
        SKIP("allocation counting unavailable under a sanitizer build (G2M_SANITIZER_BUILD)");
    }
    CHECK(counter.count() == 0);
}

TEST_CASE("gather_window: out-of-coverage samples are NoData", "[mesh]") {
    // A source that reports every tile as NoData for x < 0.
    g2m_test::SyntheticHeightSource src([](int, std::int64_t ai, std::int64_t) -> std::int32_t {
        return ai < 0 ? kHeightNoData : 500;
    });
    const ChunkKey key{k32N, 1, 0, 0}; // touches ai in [-1, 129] via its halo
    HeightWindow window;
    gather_window(key, &g2m_test::SyntheticHeightSource::lookup, &src, window);
    CHECK(window.h[0] == kHeightNoData); // window (0,0) -> local (-1,-1) -> absolute ai = -1
}
