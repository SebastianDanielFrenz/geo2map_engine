// tests/layer/test_terrain_pyramid.cpp - PLAN.md G2.2: white-box coverage of
// height_tile.h's downsample_2x2(), the mean-of-4 half-even downsample
// TerrainHeightDeriver uses for g2m.terrain.height levels 1-6. Pure unit
// tests over HeightTile/downsample_2x2 directly - no TileStore/Server/DGM1
// fixture needed here; the integration-level coverage (a Server resolve of
// an L6 tile equalling downsample_2x2 applied out of band to its own L5
// children) lives in test_derive_terrain.cpp, alongside its existing
// import -> store -> Server Fixture.
#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"

#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

using namespace g2m;

namespace {

TileKey any_key(int level) {
    TileKey k;
    k.zone = geo::UtmZone{32, geo::Hemisphere::North};
    k.level = level;
    k.x = 100;
    k.y = 100;
    return k;
}

HeightTile uniform_tile(int level, std::int32_t fill) {
    HeightTile t;
    t.key = any_key(level);
    t.h.fill(fill);
    t.has_nodata = (fill == kHeightNoData);
    return t;
}

// Sets the single 2x2 block at local (row0, row0+1) x (col0, col0+1) of
// `child` (row 0 = south, height_tile.h's own convention) - the only block
// downsample_2x2 reads to produce output sample (col0/2, row0/2) of
// whichever quadrant `child` is passed as.
void set_block(HeightTile& child, int row0, int col0, std::int32_t v00, std::int32_t v01, std::int32_t v10,
               std::int32_t v11) {
    const int row1 = row0 + 1;
    const int col1 = col0 + 1;
    child.h[static_cast<std::size_t>(row0) * kTerrainTileSamples + static_cast<std::size_t>(col0)] = v00;
    child.h[static_cast<std::size_t>(row0) * kTerrainTileSamples + static_cast<std::size_t>(col1)] = v01;
    child.h[static_cast<std::size_t>(row1) * kTerrainTileSamples + static_cast<std::size_t>(col0)] = v10;
    child.h[static_cast<std::size_t>(row1) * kTerrainTileSamples + static_cast<std::size_t>(col1)] = v11;
}

// 2x the Cell-registered world coordinate of sample `i`/`j` of `key`
// (height_tile.h: centre at key.min_easting() + (i + 0.5) * 2^level) -
// doubled so it stays an exact integer at every level, including level 0.
std::int64_t ex2(const TileKey& key, int i) {
    return 2 * key.min_easting() + (2 * static_cast<std::int64_t>(i) + 1) * (std::int64_t{1} << key.level);
}
std::int64_t nx2(const TileKey& key, int j) {
    return 2 * key.min_northing() + (2 * static_cast<std::int64_t>(j) + 1) * (std::int64_t{1} << key.level);
}

// a*E + b*N + c at sample (i, j) of `key`, via the doubled representation
// (a*ex2 == 2*a*E, so halving isn't needed as long as a, b are only ever
// used through this helper consistently).
std::int32_t ramp_value(const TileKey& key, int i, int j, std::int64_t a, std::int64_t b, std::int64_t c) {
    return static_cast<std::int32_t>(a * ex2(key, i) + b * nx2(key, j) + c);
}

} // namespace

TEST_CASE("terrain pyramid: half-even (banker's) rounding at sum mod 4 == 2 ties", "[layer][terrain]") {
    struct Case {
        std::int32_t v00, v01, v10, v11;
        std::int32_t expect;
        const char* what;
    };
    const Case cases[] = {
        {0, 0, 1, 1, 0, "mean 0.5 -> 0 (q=0, even)"},
        {1, 1, 2, 2, 2, "mean 1.5 -> 2 (q=1, odd -> rounds to even 2)"},
        {-2, -2, -1, -1, -2, "mean -1.5 -> -2 (q=-2, even)"},
        {-1, -1, 0, 0, 0, "mean -0.5 -> 0 (q=-1, odd -> rounds to even 0)"},
    };
    for (const Case& c : cases) {
        INFO(c.what);
        HeightTile sw = uniform_tile(1, 0);
        set_block(sw, 0, 0, c.v00, c.v01, c.v10, c.v11);
        HeightTile out;
        out.key = any_key(2);
        downsample_2x2(&sw, nullptr, nullptr, nullptr, out);
        CHECK(out.h[0] == c.expect);
    }
}

TEST_CASE("terrain pyramid: negative, non-tied sums round to the nearest integer (not always toward -inf)",
         "[layer][terrain]") {
    struct Case {
        std::int32_t v00, v01, v10, v11;
        std::int32_t expect;
    };
    const Case cases[] = {
        {-4, -4, -4, -4, -4}, // exact: sum=-16, mean=-4
        {-3, -3, -3, -4, -3}, // sum=-13, mean=-3.25 -> nearest -3
        {-5, -5, -5, -4, -5}, // sum=-19, mean=-4.75 -> nearest -5
    };
    for (const Case& c : cases) {
        HeightTile sw = uniform_tile(1, 0);
        set_block(sw, 0, 0, c.v00, c.v01, c.v10, c.v11);
        HeightTile out;
        out.key = any_key(2);
        downsample_2x2(&sw, nullptr, nullptr, nullptr, out);
        CHECK(out.h[0] == c.expect);
    }
}

TEST_CASE("terrain pyramid: the half-even mean sums in int64, safe near int32 extremes", "[layer][terrain]") {
    struct Case {
        std::int32_t v00, v01, v10, v11;
        std::int32_t expect;
    };
    const auto kMax = std::numeric_limits<std::int32_t>::max();
    const auto kMin = std::numeric_limits<std::int32_t>::min();
    const Case cases[] = {
        // sum = 8,000,000,000: overflows int32 (max ~2.147e9) but is an
        // exact multiple of 4, so no rounding correction applies.
        {2000000000, 2000000000, 2000000000, 2000000000, 2000000000},
        // sum = 7,999,999,999: same magnitude, r=3 -> rounds up.
        {2000000000, 2000000000, 2000000000, 1999999999, 2000000000},
        // Individual samples themselves sit at the int32 extremes (MIN + 1,
        // never MIN itself - that is kHeightNoData) and cancel to exactly 0.
        {kMax, kMax, kMin + 1, kMin + 1, 0},
    };
    for (const Case& c : cases) {
        HeightTile sw = uniform_tile(1, 0);
        set_block(sw, 0, 0, c.v00, c.v01, c.v10, c.v11);
        HeightTile out;
        out.key = any_key(2);
        downsample_2x2(&sw, nullptr, nullptr, nullptr, out);
        CHECK(out.h[0] == c.expect);
    }
}

TEST_CASE("terrain pyramid: NoData propagation - one NoData input poisons only its own output sample; a missing "
         "child poisons its whole quadrant",
         "[layer][terrain]") {
    HeightTile sw = uniform_tile(3, 5);
    // Poison exactly one 2x2 block (output sample (0,0)); every other block
    // of sw stays {5,5,5,5} -> mean 5, exact.
    set_block(sw, 0, 0, 5, kHeightNoData, 5, 5);

    HeightTile out;
    out.key = any_key(4);
    downsample_2x2(&sw, nullptr, nullptr, nullptr, out); // se/nw/ne missing entirely
    CHECK(out.has_nodata);

    CHECK(out.h[0] == kHeightNoData); // one of its 4 inputs was NoData
    CHECK(out.h[1] == 5);             // untouched block of sw

    const int half = kTerrainTileSamples / 2;
    CHECK(out.h[static_cast<std::size_t>(half)] == kHeightNoData);                      // east half: se missing
    CHECK(out.h[static_cast<std::size_t>(half) * kTerrainTileSamples] == kHeightNoData); // north half: nw missing
}

TEST_CASE("terrain pyramid: an integer-affine ramp h = a*E + b*N + c reproduces exactly across every level "
         "0->1 .. 5->6",
         "[layer][terrain]") {
    // a, b multiples of 4 guarantee every downsample_2x2 sum-of-4 is exactly
    // divisible by 4 (each of the 4 child samples differs from the others by
    // a multiple of 4*2^level, so all 4 share the same residue mod 4): the
    // half-even correction never fires, so the parent tile must come out
    // bit-for-bit equal to the same formula evaluated directly at the
    // parent's own sample centres - not merely self-consistent, exactly
    // reproducing the analytic value (Cell registration's +0.5 sample-centre
    // offset is exactly what makes a child block's mean equal the parent
    // sample's own centre value, in real arithmetic, at every level).
    constexpr std::int64_t a = 4;
    constexpr std::int64_t b = 8;
    constexpr std::int64_t c = -12345;

    for (int child_level = 0; child_level < kTerrainMaxLevel; ++child_level) {
        INFO("child_level=" << child_level);
        TileKey parent = any_key(child_level + 1);
        parent.x = 10;
        parent.y = 10;

        // Heap-allocated, not a stack array: 4 * sizeof(HeightTile) (~1 MiB,
        // HeightTile's 256x256 int32 grid is 256 KiB each) is enough to blow
        // a default 1 MiB thread stack alongside this function's other
        // locals - found the hard way (STATUS_STACK_OVERFLOW).
        std::vector<HeightTile> children(4); // SW, SE, NW, NE
        const int dx[4] = {0, 1, 0, 1};
        const int dy[4] = {0, 0, 1, 1};
        for (int ci = 0; ci < 4; ++ci) {
            const TileKey ck = parent.child(dx[ci], dy[ci]);
            children[ci].key = ck;
            children[ci].has_nodata = false;
            for (int j = 0; j < kTerrainTileSamples; ++j) {
                for (int i = 0; i < kTerrainTileSamples; ++i) {
                    children[ci].h[static_cast<std::size_t>(j) * kTerrainTileSamples + static_cast<std::size_t>(i)] =
                        ramp_value(ck, i, j, a, b, c);
                }
            }
        }

        HeightTile out;
        out.key = parent;
        downsample_2x2(&children[0], &children[1], &children[2], &children[3], out);
        CHECK_FALSE(out.has_nodata);

        for (int j = 0; j < kTerrainTileSamples; ++j) {
            for (int i = 0; i < kTerrainTileSamples; ++i) {
                const std::int32_t want = ramp_value(parent, i, j, a, b, c);
                const std::int32_t got =
                    out.h[static_cast<std::size_t>(j) * kTerrainTileSamples + static_cast<std::size_t>(i)];
                if (got != want) {
                    INFO("i=" << i << " j=" << j << " want=" << want << " got=" << got);
                    REQUIRE(got == want);
                }
            }
        }
    }
}
