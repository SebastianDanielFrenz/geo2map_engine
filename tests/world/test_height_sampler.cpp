// g2m::world height samplers (include/g2m/world/height_sampler.h; terrain
// migration plan phase 1, vault G2M-020).
//
//  - sample_height_bilinear: a verbatim port of racing_game's sample_l0_height
//    (a8a1b13, route_check.cpp). Parity is pinned bit for bit: the hashes and
//    values below were computed by that function over exactly these inputs
//    (world_test_data.h FieldSet, SplitMix64-drawn query points).
//  - sample_height_exact: new (the game had no physics-exact sampler). Its
//    oracle is g2m_mesh's own build_chunk - the triangles a render chunk (and,
//    by the derivation in terrain_chunk.h, Jolt's heightfield) is made of -
//    plus hand-computed values and a negative control against the bilinear
//    sampler. The Jolt comparison itself lives in the physics bridge tests.
#include "g2m/world/height_sampler.h"

#include "world_test_data.h"

#include "g2m/mesh/terrain_chunk.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <map>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

using namespace g2m_world_test;
using g2m::world::L0TileLookupFn;
using g2m::world::sample_height_bilinear;
using g2m::world::sample_height_exact;

namespace {

L0TileLookupFn lookup_of(const FieldSet& set) {
    return [&set](const g2m::TileKey& k) { return set.lookup(k); };
}

// The game-side recipe (refgen): 400 points, x/y = (draw % 650000)/1000 - 200,
// two draws per point; hashes (has_value, bit pattern) of the result.
std::uint64_t hash_bilinear_run(const FieldSet& set, int pass, int* nulls) {
    const L0TileLookupFn fn = lookup_of(set);
    g2m_test::SplitMix64 r(0xB111EAF000ull + static_cast<std::uint64_t>(pass));
    g2m::Fnv1a64 h;
    *nulls = 0;
    for (int i = 0; i < 400; ++i) {
        const std::uint64_t a = r.next();
        const std::uint64_t b = r.next();
        const double x = static_cast<double>(a % 650000) / 1000.0 - 200.0;
        const double y = static_cast<double>(b % 650000) / 1000.0 - 200.0;
        const std::optional<double> v = sample_height_bilinear(fn, kZone, kE0, kN0, x, y);
        h.u8(v ? 1 : 0).u64(v ? bits_of(*v) : 0);
        if (!v) ++*nulls;
    }
    return h.value();
}

// One tile of a hand-made 2x2 patch: tile (1000, 20000), samples (0,0) (1,0)
// (0,1) (1,1) = 10, 12, 14, 20 m, everything else NoData.
struct PatchLookup {
    std::unique_ptr<g2m::HeightTile> tile = std::make_unique<g2m::HeightTile>();
    PatchLookup() {
        tile->key = g2m::TileKey{kZone, 0, 1000, 20000};
        for (std::int32_t& v : tile->h) v = g2m::kHeightNoData;
        tile->has_nodata = true;
        tile->h[0] = 10 * 256;
        tile->h[1] = 12 * 256;
        tile->h[256] = 14 * 256;
        tile->h[257] = 20 * 256;
    }
    L0TileLookupFn fn() const {
        return [this](const g2m::TileKey& k) -> const g2m::HeightTile* {
            return k == tile->key ? tile.get() : nullptr;
        };
    }
};

} // namespace

// Pins computed by racing_game a8a1b13's sample_l0_height (extracted verbatim,
// run over FieldSet(0), FieldSet(3 permille NoData) and FieldSet(3) with tile
// (1001, 20000) missing; 400 random points each).
TEST_CASE("sample_height_bilinear matches racing_game's sample_l0_height bit for bit", "[world][height_sampler]") {
    struct Pin {
        int pass;
        int nodata_permille;
        bool missing_east;
        int nulls;
        std::uint64_t hash;
    };
    const Pin pins[] = {
        {0, 0, false, 0, 0x21c7ea3e2f8a587bull},
        {1, 3, false, 12, 0x0e93f30f5c53860bull},
        {2, 3, true, 59, 0xd3d7131138338ca6ull},
    };
    for (const Pin& p : pins) {
        INFO("pass " << p.pass);
        FieldSet set(p.nodata_permille);
        if (p.missing_east) set.missing = {1001, 20000};
        int nulls = 0;
        CHECK(hash_bilinear_run(set, p.pass, &nulls) == p.hash);
        CHECK(nulls == p.nulls);
    }

    // Single values (exact bit patterns) at fixed probe points.
    const FieldSet full(0);
    const L0TileLookupFn fn = lookup_of(full);
    struct Probe {
        double x, y;
        std::uint64_t bits;
    };
    const Probe probes[] = {
        {0, 0, 0x4056874000000000ull},           {0.5, 0.5, 0x40522c8000000000ull},
        {255.5, 255.5, 0x404f6d8000000000ull},   {255.99, 0.01, 0x40573152bb9a5bb0ull},
        {-0.001, -0.001, 0x405687f4a9563518ull}, {256.0, 100.25, 0x4056bfd000000000ull},
        {100.25, 256.0, 0x40585bb000000000ull},  {-255.5, -255.5, 0x40602ec000000000ull},
    };
    for (const Probe& p : probes) {
        INFO("x=" << p.x << " y=" << p.y);
        const std::optional<double> v = sample_height_bilinear(fn, kZone, kE0, kN0, p.x, p.y);
        REQUIRE(v.has_value());
        CHECK(bits_of(*v) == p.bits);
    }
    // With tile (1001, 20000) missing the east probes have no answer (the game
    // returned nullopt for exactly these three).
    FieldSet east_gone(0);
    east_gone.missing = {1001, 20000};
    const L0TileLookupFn fn2 = lookup_of(east_gone);
    CHECK_FALSE(sample_height_bilinear(fn2, kZone, kE0, kN0, 255.5, 255.5).has_value());
    CHECK_FALSE(sample_height_bilinear(fn2, kZone, kE0, kN0, 255.99, 0.01).has_value());
    CHECK_FALSE(sample_height_bilinear(fn2, kZone, kE0, kN0, 256.0, 100.25).has_value());
    CHECK(sample_height_bilinear(fn2, kZone, kE0, kN0, 100.25, 256.0).has_value());
}

// Ported from racing_game test_route_check.cpp: a linear field is reproduced
// exactly by bilinear interpolation. The exact sampler (two planar triangles
// per quad) reproduces a linear field exactly too, so both run the same cases.
TEST_CASE("height samplers reproduce a linear field across L0 tile borders", "[world][height_sampler]") {
    std::map<std::pair<std::int32_t, std::int32_t>, std::unique_ptr<g2m::HeightTile>> tiles;
    for (std::int32_t ty = 19999; ty <= 20001; ++ty) {
        for (std::int32_t tx = 999; tx <= 1001; ++tx) {
            auto t = std::make_unique<g2m::HeightTile>();
            t->key = g2m::TileKey{kZone, 0, tx, ty};
            for (std::int64_t j = 0; j < 256; ++j) {
                for (std::int64_t i = 0; i < 256; ++i) {
                    const std::int64_t c = std::int64_t{tx} * 256 + i;
                    const std::int64_t r = std::int64_t{ty} * 256 + j;
                    t->h[static_cast<std::size_t>(j * 256 + i)] =
                        static_cast<std::int32_t>(64 * (c - kE0) + 128 * (r - kN0));
                }
            }
            tiles.emplace(std::make_pair(tx, ty), std::move(t));
        }
    }
    int lookups = 0;
    std::pair<std::int32_t, std::int32_t> missing{-1, -1};
    const L0TileLookupFn lookup = [&](const g2m::TileKey& k) -> const g2m::HeightTile* {
        ++lookups;
        const auto key = std::make_pair(k.x, k.y);
        if (k.level != 0 || key == missing) return nullptr;
        const auto it = tiles.find(key);
        return it == tiles.end() ? nullptr : it->second.get();
    };
    const auto expected = [](double x, double y) { return 0.25 * (x - 0.5) + 0.5 * (y - 0.5); };

    struct P {
        double x, y;
    };
    for (const P& p : std::vector<P>{{100.25, 37.75}, {0.0, 0.0}, {0.3, 128.0}, {-0.2, -0.4},
                                      {-100.9, -3.3}, {255.9, 256.2}, {511.4, 300.0}}) {
        INFO("x=" << p.x << " y=" << p.y);
        const auto b = sample_height_bilinear(lookup, kZone, kE0, kN0, p.x, p.y);
        const auto e = sample_height_exact(lookup, kZone, kE0, kN0, p.x, p.y);
        REQUIRE(b.has_value());
        REQUIRE(e.has_value());
        CHECK(*b == Catch::Approx(expected(p.x, p.y)).margin(1e-9));
        CHECK(*e == Catch::Approx(expected(p.x, p.y)).margin(1e-9));
    }

    // Exactly on a cell centre: that sample, no interpolation.
    CHECK(*sample_height_bilinear(lookup, kZone, kE0, kN0, 10.5, 20.5) == Catch::Approx(0.25 * 10 + 0.5 * 20));
    CHECK(*sample_height_exact(lookup, kZone, kE0, kN0, 10.5, 20.5) == Catch::Approx(0.25 * 10 + 0.5 * 20));

    // A NoData sample at (5, 5). The bilinear sampler loses all four cells
    // around it.
    tiles.at({1000, 20000})->h[static_cast<std::size_t>(5 * 256 + 5)] = g2m::kHeightNoData;
    CHECK_FALSE(sample_height_bilinear(lookup, kZone, kE0, kN0, 5.6, 5.6).has_value());
    CHECK_FALSE(sample_height_bilinear(lookup, kZone, kE0, kN0, 5.4, 5.4).has_value());
    CHECK(sample_height_bilinear(lookup, kZone, kE0, kN0, 6.6, 6.6).has_value());
    // The exact sampler loses only the six triangles that use the sample:
    //   cell (4,4): NE triangle (uses (5,5) as c11), SW triangle intact;
    //   cell (5,5): SW triangle (c00), NE triangle intact;
    //   cells (4,5) and (5,4): both triangles (the sample is c10 / c01, which
    //   both triangles of a cell share).
    CHECK_FALSE(sample_height_exact(lookup, kZone, kE0, kN0, 5.1, 5.1).has_value()); // cell (4,4), NE
    CHECK(sample_height_exact(lookup, kZone, kE0, kN0, 4.7, 4.7).has_value());       // cell (4,4), SW
    CHECK_FALSE(sample_height_exact(lookup, kZone, kE0, kN0, 5.6, 5.6).has_value()); // cell (5,5), SW
    CHECK(sample_height_exact(lookup, kZone, kE0, kN0, 6.4, 6.4).has_value());       // cell (5,5), NE
    CHECK_FALSE(sample_height_exact(lookup, kZone, kE0, kN0, 5.0, 6.0).has_value()); // cell (4,5), SW
    CHECK_FALSE(sample_height_exact(lookup, kZone, kE0, kN0, 5.2, 6.4).has_value()); // cell (4,5), NE
    CHECK_FALSE(sample_height_exact(lookup, kZone, kE0, kN0, 6.0, 5.0).has_value()); // cell (5,4), SW
    CHECK_FALSE(sample_height_exact(lookup, kZone, kE0, kN0, 6.4, 5.2).has_value()); // cell (5,4), NE

    // A missing tile (lookup returns nullptr) is NoData too.
    missing = {999, 20000};
    CHECK_FALSE(sample_height_bilinear(lookup, kZone, kE0, kN0, -50.0, 50.0).has_value());
    CHECK_FALSE(sample_height_exact(lookup, kZone, kE0, kN0, -50.0, 50.0).has_value());
    CHECK_FALSE(sample_height_bilinear(lookup, kZone, kE0, kN0, 0.2, 50.0).has_value());
    CHECK_FALSE(sample_height_exact(lookup, kZone, kE0, kN0, 0.2, 50.0).has_value());
    CHECK(sample_height_bilinear(lookup, kZone, kE0, kN0, 0.6, 50.0).has_value());
    CHECK(sample_height_exact(lookup, kZone, kE0, kN0, 0.6, 50.0).has_value());
    CHECK(lookups > 0);
}

TEST_CASE("sample_height_exact follows the NW-SE diagonal (hand-computed)", "[world][height_sampler]") {
    // h00 = 10, h10 = 12 (east), h01 = 14 (north), h11 = 20 at samples
    // (0,0) (1,0) (0,1) (1,1); the lattice centre of sample (0,0) is (0.5, 0.5).
    const PatchLookup patch;
    const L0TileLookupFn fn = patch.fn();

    // SW triangle (c00 c10 c01), tx = ty = 0.25: 10 + 0.25*2 + 0.25*4 = 11.5.
    const auto sw = sample_height_exact(fn, kZone, kE0, kN0, 0.75, 0.75);
    REQUIRE(sw.has_value());
    CHECK(*sw == 11.5);
    // NE triangle (c11 c10 c01), tx = ty = 0.75: 20 + 0.25*(14-20) + 0.25*(12-20) = 16.5.
    const auto ne = sample_height_exact(fn, kZone, kE0, kN0, 1.25, 1.25);
    REQUIRE(ne.has_value());
    CHECK(*ne == 16.5);
    // On the diagonal itself (tx + ty == 1) both triangles agree:
    // 10 + 0.5*2 + 0.5*4 = 13 = 20 + 0.5*(14-20) + 0.5*(12-20).
    const auto diag = sample_height_exact(fn, kZone, kE0, kN0, 1.0, 1.0);
    REQUIRE(diag.has_value());
    CHECK(*diag == 13.0);
    // At a sample centre: that sample.
    CHECK(*sample_height_exact(fn, kZone, kE0, kN0, 0.5, 0.5) == 10.0);
    // A hair below sample (1,1): the point lies in cell (0,0)'s NE triangle.
    const auto near11 = sample_height_exact(fn, kZone, kE0, kN0, 1.5 - 1e-9, 1.5 - 1e-9);
    REQUIRE(near11.has_value());
    CHECK(*near11 == Catch::Approx(20.0).margin(1e-6));
    // Exactly on it the point belongs to cell (1,1), whose triangles need the
    // (missing) samples east and north of it.
    CHECK_FALSE(sample_height_exact(fn, kZone, kE0, kN0, 1.5, 1.5).has_value());

    // Negative control: the bilinear answers are 16.75 and 11.75, not 16.5 and
    // 11.5 - the two surfaces really differ, so a test that only checked
    // "close to bilinear" would not notice a wrong diagonal.
    CHECK(*sample_height_bilinear(fn, kZone, kE0, kN0, 1.25, 1.25) == 16.75);
    CHECK(*sample_height_bilinear(fn, kZone, kE0, kN0, 0.75, 0.75) == 11.75);

    // NoData: the unused fourth corner does not matter. Without h11 the SW
    // triangle is intact and the NE one is gone; bilinear needs all four.
    PatchLookup no11;
    no11.tile->h[257] = g2m::kHeightNoData;
    const L0TileLookupFn fn11 = no11.fn();
    CHECK(*sample_height_exact(fn11, kZone, kE0, kN0, 0.75, 0.75) == 11.5);
    CHECK_FALSE(sample_height_exact(fn11, kZone, kE0, kN0, 1.25, 1.25).has_value());
    CHECK_FALSE(sample_height_bilinear(fn11, kZone, kE0, kN0, 0.75, 0.75).has_value());
    // Without h00 the NE triangle is intact and the SW one gone.
    PatchLookup no00;
    no00.tile->h[0] = g2m::kHeightNoData;
    const L0TileLookupFn fn00 = no00.fn();
    CHECK(*sample_height_exact(fn00, kZone, kE0, kN0, 1.25, 1.25) == 16.5);
    CHECK_FALSE(sample_height_exact(fn00, kZone, kE0, kN0, 0.75, 0.75).has_value());

    // Not finite: no answer.
    CHECK_FALSE(sample_height_exact(fn, kZone, kE0, kN0, std::nan(""), 0.5).has_value());
    CHECK_FALSE(sample_height_exact(fn, kZone, kE0, kN0, 0.5, HUGE_VAL).has_value());
    CHECK_FALSE(sample_height_bilinear(fn, kZone, kE0, kN0, std::nan(""), 0.5).has_value());
}

namespace {

const g2m::HeightTile* field_tile_lookup(void* ctx, const g2m::TileKey& key) {
    return static_cast<const FieldSet*>(ctx)->lookup(key);
}

// The oracle: the height of the point (e, n) (absolute UTM metres) on the mesh
// build_chunk made, i.e. the triangle of the chunk's grid that holds it. nullopt
// if no (non-dropped) triangle holds it.
std::optional<double> mesh_height_at(const g2m::mesh::TerrainChunkMesh& m, double e, double n) {
    const double px = e - m.origin[0];
    const double py = n - m.origin[1];
    const std::size_t tri_count = m.indices.size() / 3;
    for (std::size_t t = 0; t < tri_count; ++t) {
        const std::uint16_t ia = m.indices[t * 3 + 0];
        const std::uint16_t ib = m.indices[t * 3 + 1];
        const std::uint16_t ic = m.indices[t * 3 + 2];
        if (ia >= m.skirt_first_vertex || ib >= m.skirt_first_vertex || ic >= m.skirt_first_vertex) continue;
        const double ax = m.positions[ia * 3], ay = m.positions[ia * 3 + 1], az = m.positions[ia * 3 + 2];
        const double bx = m.positions[ib * 3], by = m.positions[ib * 3 + 1], bz = m.positions[ib * 3 + 2];
        const double cx = m.positions[ic * 3], cy = m.positions[ic * 3 + 1], cz = m.positions[ic * 3 + 2];
        const double det = (by - cy) * (ax - cx) + (cx - bx) * (ay - cy);
        const double l1 = ((by - cy) * (px - cx) + (cx - bx) * (py - cy)) / det;
        const double l2 = ((cy - ay) * (px - cx) + (ax - cx) * (py - cy)) / det;
        const double l3 = 1.0 - l1 - l2;
        constexpr double kEps = 1e-9;
        if (l1 >= -kEps && l2 >= -kEps && l3 >= -kEps) {
            return l1 * az + l2 * bz + l3 * cz;
        }
    }
    return std::nullopt;
}

} // namespace

// build_chunk is the same triangulation as the Jolt heightfield (the diagonal
// derivation in terrain_chunk.h; the physics bridge tests pin that against
// Jolt itself), so the sampler must agree with the mesh it makes: on rough
// terrain (the quad diagonals disagree by metres), on a chunk that crosses an
// L0 tile border, and with NoData holes (dropped triangles).
TEST_CASE("sample_height_exact agrees with the triangles build_chunk emits", "[world][height_sampler]") {
    struct Case {
        int nodata_permille;
        std::int32_t cx, cy;
        double x_lo, y_lo; // session-local point range [lo, lo + 63.8]
    };
    // Session origin (kE0, kN0): chunk (4000, 80000) covers x, y in [0, 64);
    // chunk (3999, 80000) covers x in [-64, 0): its vertex 64 sits at x = 0.5
    // and its halo reaches into tile 1000 - a tile border at x = 0.
    const Case cases[] = {
        {0, 4000, 80000, 0.5, 0.5},
        {0, 3999, 80000, -63.5, 0.5},
        {0, 4000, 79999, 0.5, -63.5},
        {30, 4000, 80000, 0.5, 0.5},
        {30, 3999, 79999, -63.5, -63.5},
    };
    for (const Case& c : cases) {
        INFO("nodata_permille=" << c.nodata_permille << " chunk=" << c.cx << "," << c.cy);
        FieldSet set(c.nodata_permille);
        const g2m::mesh::ChunkKey key{kZone, 0, c.cx, c.cy};
        auto window = std::make_unique<g2m::mesh::HeightWindow>();
        g2m::mesh::gather_window(key, &field_tile_lookup, &set, *window);
        g2m::mesh::TerrainChunkMesh mesh;
        g2m::mesh::build_chunk(key, *window, nullptr, mesh);
        REQUIRE(mesh.indices.size() > 0);

        const L0TileLookupFn fn = lookup_of(set);
        g2m_test::SplitMix64 r(0xE8AC7000ull + static_cast<std::uint64_t>(c.cx) * 31 + static_cast<std::uint64_t>(c.cy));
        int agreed = 0;
        int both_none = 0;
        int bilinear_differs = 0;
        for (int i = 0; i < 600; ++i) {
            const std::uint64_t a = r.next();
            const std::uint64_t b = r.next();
            const double x = c.x_lo + static_cast<double>(a % 63800) / 1000.0;
            const double y = c.y_lo + static_cast<double>(b % 63800) / 1000.0;
            // A point on the quad's diagonal belongs to both triangles; if one
            // of them was dropped, which side of it rounding puts the point on
            // decides the answer (measure zero, and the surface is continuous
            // there). The 0.001 m grid of the draws hits it now and then.
            const double u = static_cast<double>(kE0) + x - 0.5;
            const double v = static_cast<double>(kN0) + y - 0.5;
            if (std::abs((u - std::floor(u)) + (v - std::floor(v)) - 1.0) < 1e-7) continue;
            const std::optional<double> exact = sample_height_exact(fn, kZone, kE0, kN0, x, y);
            const std::optional<double> oracle =
                mesh_height_at(mesh, static_cast<double>(kE0) + x, static_cast<double>(kN0) + y);
            INFO("x=" << x << " y=" << y);
            REQUIRE(exact.has_value() == oracle.has_value());
            if (exact) {
                // The mesh stores float positions (z ~ 100 m: 8 micrometres).
                CHECK(*exact == Catch::Approx(*oracle).margin(1e-3));
                ++agreed;
                const std::optional<double> bil = sample_height_bilinear(fn, kZone, kE0, kN0, x, y);
                if (!bil || std::abs(*bil - *exact) > 0.05) ++bilinear_differs;
            } else {
                ++both_none;
            }
        }
        CHECK(agreed > 300);
        // Negative control: on this rough terrain the bilinear surface is
        // metres off the triangle surface almost everywhere.
        CHECK(bilinear_differs > agreed / 2);
        if (c.nodata_permille > 0) {
            CHECK(both_none > 0); // the holes really are exercised
        }
    }
}
