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

TEST_CASE("build_chunk: every base-grid z equals h/256 exactly (level 0)", "[mesh]") {
    // PLAN.md G2.3a: this exactness is now an L0-ONLY property - vertex i IS
    // level-0 sample i, unchanged by the chunk-footprint nesting fix. Level
    // k >= 1 vertices are a bilinear interpolation of the level-k pyramid
    // (terrain_chunk.h's build_chunk comment) and are covered instead by
    // "build_chunk: level >= 1 vertex z reproduces a planar height field
    // exactly" below.
    g2m_test::SyntheticHeightSource src(g2m_test::sinusoidal_height(30.0, 150.0));
    const ChunkKey key{k32N, 0, 5, -2};
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
    // PLAN.md G2.3a: for level >= 1 the rendered vertex is no longer the raw
    // pyramid sample but the bilinear interpolation build_chunk's own header
    // comment documents (t = 0.5 + 2^-(level+1), the SAME t on both axes,
    // from sample (ai-1, aj-1) toward (ai, aj)) - re-derived here
    // independently (not calling into terrain_chunk.cpp's internal
    // sample_vertex_k1/accumulate_border_crack_k1) so the crack quantity
    // this test measures matches what is ACTUALLY rendered, per this
    // increment's own "the vertex is no longer the raw sample, so a crack
    // model based on raw samples would no longer represent what's rendered"
    // argument (see the report / build_chunk's skirt_depth branch).
    const double t = key.level == 0 ? 0.0 : 0.5 + 1.0 / static_cast<double>(std::int64_t{1} << (key.level + 1));
    auto sample_m = [&](std::int64_t ai, std::int64_t aj) {
        if (key.level == 0) return static_cast<double>(fn(key.level, ai, aj)) / 256.0;
        const double h00 = static_cast<double>(fn(key.level, ai - 1, aj - 1));
        const double h10 = static_cast<double>(fn(key.level, ai, aj - 1));
        const double h01 = static_cast<double>(fn(key.level, ai - 1, aj));
        const double h11 = static_cast<double>(fn(key.level, ai, aj));
        const double one_minus_t = 1.0 - t;
        const double z_units =
            one_minus_t * one_minus_t * h00 + t * one_minus_t * h10 + one_minus_t * t * h01 + t * t * h11;
        return z_units / 256.0;
    };
    // Level 0 measures the DOUBLED difference (2*hb - ha - hc, matching
    // build_chunk's own accumulate_border_crack, which works in raw int64
    // 1/256-unit samples and only divides by 2 - via /512 instead of /256 -
    // once, at the very end); level >= 1 measures the true (already-halved)
    // T-junction gap |vb - 0.5*(va+vc)| directly, matching
    // accumulate_border_crack_k1, which operates on real metre heights and
    // has no such raw-integer doubling trick to undo.
    // South and north borders (fixed aj, varying ai); west/east (fixed ai, varying aj).
    for (int k = 0; k < kChunkQuads / 2; ++k) {
        const std::int64_t a = base_i + 2 * k, b = base_i + 2 * k + 1, c = base_i + 2 * k + 2;
        for (std::int64_t aj : {base_j, base_j + kChunkQuads}) {
            const double crack = key.level == 0
                                      ? std::fabs(2.0 * sample_m(b, aj) - sample_m(a, aj) - sample_m(c, aj))
                                      : std::fabs(sample_m(b, aj) - 0.5 * (sample_m(a, aj) + sample_m(c, aj)));
            worst_m = std::max(worst_m, crack);
        }
        const std::int64_t a2 = base_j + 2 * k, b2 = base_j + 2 * k + 1, c2 = base_j + 2 * k + 2;
        for (std::int64_t ai : {base_i, base_i + kChunkQuads}) {
            const double crack = key.level == 0
                                      ? std::fabs(2.0 * sample_m(ai, b2) - sample_m(ai, a2) - sample_m(ai, c2))
                                      : std::fabs(sample_m(ai, b2) - 0.5 * (sample_m(ai, a2) + sample_m(ai, c2)));
            worst_m = std::max(worst_m, crack);
        }
    }
    (void)step_m;

    // build_chunk's formula: 1.0 m + worst (level >= 1, worst_m already the
    // true gap) or 1.0 m + worst/2 (level 0, worst_m the doubled raw-unit
    // difference) - both computed independently here, both starting from the
    // same analytic height function. (Level 0's form is unaffected by this
    // fix; this test's key is level 4, so the level>=1 branch above is what
    // actually executes here.)
    const float expected = static_cast<float>(key.level == 0 ? 1.0 + worst_m / 2.0 : 1.0 + worst_m);
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
            // PLAN.md G2.3a: at level >= 1, base_z is a bilinearly-interpolated
            // (no longer an exact multiple of 1/256) float, so
            // base_z - (base_z - skirt_depth_m) need not round-trip to
            // exactly skirt_depth_m bit-for-bit (double rounding) - margin a
            // handful of ULPs at this magnitude rather than requiring exact
            // equality (still an exact check at level 0, where base_z is a
            // "nice" h/256 value and this held bit-exact before this fix).
            CHECK(base_z - skirt_z == Catch::Approx(mesh.skirt_depth_m).margin(1e-4f));
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

// PLAN.md G2.3a (chunk-footprint nesting fix): a level-(k+1) chunk's own
// corner/edge vertices must coincide EXACTLY (E, N as doubles) with its 4
// level-k children's matching vertices - the whole point of the fix (before
// it, every level had a 0.5*step_finer gap/overlap at every 2:1 boundary;
// racing_game's test_world_terrain.cpp pinned that bug at exactly that
// value). Only E/N are checked, never Z: a coarse chunk's height comes from
// the COARSER (already-downsampled) pyramid level, so it need not (and in
// general does not) equal a child's height at the same point - that
// ordinary LOD height mismatch is what skirts hide, unrelated to this fix.
TEST_CASE("build_chunk: a level (k+1) chunk nests exactly with its 4 level-k children", "[mesh]") {
    g2m_test::SyntheticHeightSource src(g2m_test::sinusoidal_height(30.0, 150.0));

    struct Case {
        std::int32_t cx, cy;
    };
    // A handful of representative (cx, cy) pairs, including negative indices
    // (PLAN.md G2.3a's formulas must hold for negative cx/cy too - integer
    // division/shift pitfalls show up there first).
    const std::array<Case, 3> cases{{{5, -2}, {0, 0}, {-3, 7}}};

    for (int k = 0; k <= 5; ++k) {
        for (const Case& cs : cases) {
            const ChunkKey coarse{k32N, static_cast<std::uint8_t>(k + 1), cs.cx, cs.cy};
            const TerrainChunkMesh mc = build(coarse, src);

            TerrainChunkMesh children[2][2]; // [dcy][dcx]
            for (int dcy = 0; dcy < 2; ++dcy) {
                for (int dcx = 0; dcx < 2; ++dcx) {
                    const ChunkKey child{k32N, static_cast<std::uint8_t>(k), 2 * cs.cx + dcx, 2 * cs.cy + dcy};
                    children[dcy][dcx] = build(child, src);
                }
            }

            auto abs_e = [](const TerrainChunkMesh& m, int i, int j) {
                return m.origin[0] + static_cast<double>(m.positions[static_cast<std::size_t>(j * kChunkVerts + i) * 3 + 0]);
            };
            auto abs_n = [](const TerrainChunkMesh& m, int i, int j) {
                return m.origin[1] + static_cast<double>(m.positions[static_cast<std::size_t>(j * kChunkVerts + i) * 3 + 1]);
            };

            // Every coarse vertex (I, J), not only the border, maps to
            // exactly one child vertex - child index = I (or J) halved
            // within its half, per terrain_chunk.h's documented vertex
            // formula (a coarse chunk's own 2 halves ARE its 2 children
            // along that axis).
            for (int J = 0; J <= kChunkQuads; ++J) {
                const int dcy = (J <= kChunkQuads / 2) ? 0 : 1;
                const int j = (dcy == 0) ? 2 * J : 2 * J - kChunkQuads;
                for (int I = 0; I <= kChunkQuads; ++I) {
                    const int dcx = (I <= kChunkQuads / 2) ? 0 : 1;
                    const int i = (dcx == 0) ? 2 * I : 2 * I - kChunkQuads;

                    const TerrainChunkMesh& child = children[static_cast<std::size_t>(dcy)][static_cast<std::size_t>(dcx)];
                    INFO("k=" << k << " cx=" << cs.cx << " cy=" << cs.cy << " I=" << I << " J=" << J);
                    CHECK(abs_e(mc, I, J) == abs_e(child, i, j));
                    CHECK(abs_n(mc, I, J) == abs_n(child, i, j));
                }
            }
        }
    }
}

// PLAN.md G2.3a: L0 must stay bit-identical - these 3 hashes were recorded
// from the code BEFORE this fix (same synthetic source/keys the "shared
// border"/"z equals h/256" tests use), with L0's own code path (vertex IS
// the sample, no interpolation) untouched by the fix.
TEST_CASE("build_chunk: level 0 content_hash is unchanged by the G2.3a nesting fix", "[mesh]") {
    g2m_test::SyntheticHeightSource src(g2m_test::sinusoidal_height(30.0, 150.0));

    struct Case {
        ChunkKey key;
        std::uint64_t expected_hash;
        double expected_origin_e, expected_origin_n;
    };
    const std::array<Case, 3> cases{{
        {ChunkKey{k32N, 0, 5, -2}, 10873786206562434936ull, 320.5, -127.5},
        {ChunkKey{k32N, 0, 0, 0}, 8543663860595596267ull, 0.5, 0.5},
        {ChunkKey{k32N, 0, -3, 7}, 4905587286737275503ull, -191.5, 448.5},
    }};
    for (const Case& c : cases) {
        const TerrainChunkMesh mesh = build(c.key, src);
        INFO("cx=" << c.key.cx << " cy=" << c.key.cy);
        CHECK(mesh.origin[0] == c.expected_origin_e);
        CHECK(mesh.origin[1] == c.expected_origin_n);
        CHECK(mesh.content_hash == c.expected_hash);
    }
}

// PLAN.md G2.3a: bilinear interpolation reproduces an affine (planar) height
// field EXACTLY (within float rounding), for every level k >= 1 - a synthetic
// height field h(E, N) = a + b*E + c*N with a/b/c chosen so every sample and
// every vertex position is EXACTLY representable, so "within float rounding"
// only has to absorb the final float cast, not any input quantisation.
TEST_CASE("build_chunk: level >= 1 vertex z reproduces a planar height field exactly", "[mesh]") {
    // a=100.0 m, b=0.25 (1/256 m per metre... chosen as exact powers of two
    // so every sample - an integer multiple of 2^level metres, offset by
    // 0.5*2^level - and every vertex position (also an exact dyadic in
    // metres) map to an exactly representable plane value once scaled to
    // 1/256 m units and rounded to int32 (the height storage format itself,
    // not this test, is what quantises to whole 1/256 m - so the "plane" the
    // test checks against is the SAME quantised plane build_chunk actually
    // interpolates, not the pre-quantisation continuous one).
    constexpr double kA = 100.0, kB = 0.25, kC = -0.125;
    g2m_test::SyntheticHeightSource src([&](int level, std::int64_t ai, std::int64_t aj) -> std::int32_t {
        const double step_m = static_cast<double>(std::int64_t{1} << level);
        const double x = (static_cast<double>(ai) + 0.5) * step_m;
        const double y = (static_cast<double>(aj) + 0.5) * step_m;
        return static_cast<std::int32_t>((kA + kB * x + kC * y) * 256.0);
    });

    for (int k = 1; k <= 6; ++k) {
        const ChunkKey key{k32N, static_cast<std::uint8_t>(k), 3, -4};
        const TerrainChunkMesh mesh = build(key, src);

        for (int j = 0; j < kChunkVerts; ++j) {
            for (int i = 0; i < kChunkVerts; ++i) {
                const int vi = j * kChunkVerts + i;
                const double e = mesh.origin[0] + static_cast<double>(mesh.positions[static_cast<std::size_t>(vi) * 3 + 0]);
                const double n = mesh.origin[1] + static_cast<double>(mesh.positions[static_cast<std::size_t>(vi) * 3 + 1]);
                // The same quantise-to-1/256-then-back-to-metres the height
                // storage format itself applies (kA+kB*e+kC*n is already an
                // exact multiple of 1/65536 for these coefficients/positions,
                // well within int32's range, so this cast is exact, not a
                // rounding step).
                const double expected = static_cast<double>(static_cast<std::int32_t>((kA + kB * e + kC * n) * 256.0)) / 256.0;
                const float z = mesh.positions[static_cast<std::size_t>(vi) * 3 + 2];
                INFO("k=" << k << " i=" << i << " j=" << j << " e=" << e << " n=" << n);
                CHECK(static_cast<double>(z) == Catch::Approx(expected).margin(1e-4));

                // The analytic gradient of a plane is its own coefficients
                // everywhere - a direct check that compute_normal's new
                // bilinear-gradient path (not a finite difference) got the
                // sign/scale right.
                const float nx = mesh.normals[static_cast<std::size_t>(vi) * 3 + 0];
                const float ny = mesh.normals[static_cast<std::size_t>(vi) * 3 + 1];
                const float nz = mesh.normals[static_cast<std::size_t>(vi) * 3 + 2];
                const double len = std::sqrt((kB * kB) + (kC * kC) + 1.0);
                CHECK(static_cast<double>(nx) == Catch::Approx(-kB / len).margin(1e-4));
                CHECK(static_cast<double>(ny) == Catch::Approx(-kC / len).margin(1e-4));
                CHECK(static_cast<double>(nz) == Catch::Approx(1.0 / len).margin(1e-4));
            }
        }
    }
}
