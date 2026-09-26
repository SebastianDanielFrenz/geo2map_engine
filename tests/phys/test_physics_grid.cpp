// tests/phys/test_physics_grid.cpp - PhysicsTileGrid (PLAN.md R2.2 section
// 1a/4a): index/origin round trip, pack/unpack (incl. matching
// physics_sim's own ProceduralTerrainSource layout), l0_keys_for_square
// straddling and the [AMEND] DGM1-cell-centre proof.
#include "g2m/phys/physics_grid.h"

#include "g2m/core/geo/session_frame.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

using g2m::TileKey;
using g2m::geo::Hemisphere;
using g2m::geo::SessionFrame;
using g2m::geo::UtmZone;
using g2m::phys::kPhysSamples;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;

namespace {
constexpr UtmZone k32N{32, Hemisphere::North};
} // namespace

TEST_CASE("PhysicsTileGrid: samples()/tile_size_m()", "[phys][physics_grid]") {
    SessionFrame frame(k32N, 474000, 5560000);
    PhysicsTileGrid grid(frame);
    CHECK(grid.samples() == kPhysSamples);
    CHECK(grid.tile_size_m() == 255.0);
}

TEST_CASE("PhysicsTileGrid: rejects an odd or too-small sample count", "[phys][physics_grid]") {
    SessionFrame frame(k32N, 0, 0);
    CHECK_THROWS_AS(PhysicsTileGrid(frame, 3), std::invalid_argument);
    CHECK_THROWS_AS(PhysicsTileGrid(frame, 0), std::invalid_argument);
    CHECK_THROWS_AS(PhysicsTileGrid(frame, -2), std::invalid_argument);
    CHECK_NOTHROW(PhysicsTileGrid(frame, 2));
}

TEST_CASE("PhysicsTileGrid: index_for_local / origin_local round trip", "[phys][physics_grid]") {
    SessionFrame frame(k32N, 474000, 5560000);
    PhysicsTileGrid grid(frame);
    for (std::int32_t ix : {-1000, -1, 0, 1, 999}) {
        for (std::int32_t iy : {-500, 0, 500}) {
            const PhysTileIndex idx{ix, iy};
            const auto origin = grid.origin_local(idx);
            CHECK(grid.index_for_local(origin[0], origin[1]) == idx);
            // Anywhere strictly inside the tile (up to just under the next
            // tile's own origin) round-trips too.
            CHECK(grid.index_for_local(origin[0] + 10.0, origin[1] + 254.0) == idx);
        }
    }
}

TEST_CASE("PhysicsTileGrid: index_for_local at the x = 0.5 tile boundary", "[phys][physics_grid]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame);
    // A point exactly on tile 0's western/southern edge (local 0.5) lands in
    // tile 0 itself - the same "a point on an edge lands in the tile to its
    // east/north" rule SessionFrame::tile_at documents.
    CHECK(grid.index_for_local(0.5, 0.5) == PhysTileIndex{0, 0});
    CHECK(grid.index_for_local(std::nextafter(0.5, -1.0), 0.5) == PhysTileIndex{-1, 0});
    CHECK(grid.index_for_local(0.5, std::nextafter(0.5, -1.0)) == PhysTileIndex{0, -1});
    // The shared border sample at the next tile's own origin.
    CHECK(grid.index_for_local(0.5 + 255.0, 0.5) == PhysTileIndex{1, 0});
    CHECK(grid.index_for_local(std::nextafter(0.5 + 255.0, 0.0), 0.5) == PhysTileIndex{0, 0});
}

TEST_CASE("PhysicsTileGrid::pack/unpack round trip", "[phys][physics_grid]") {
    const PhysTileIndex cases[] = {
        {0, 0},
        {1, -1},
        {-1, 1},
        {12345, -6789},
        {std::numeric_limits<std::int32_t>::min(), std::numeric_limits<std::int32_t>::max()},
        {std::numeric_limits<std::int32_t>::max(), std::numeric_limits<std::int32_t>::min()},
    };
    for (const auto& idx : cases) {
        const std::int64_t packed = PhysicsTileGrid::pack(idx);
        CHECK(PhysicsTileGrid::unpack(packed) == idx);
    }
}

TEST_CASE("PhysicsTileGrid::pack matches physics_sim's ProceduralTerrainSource layout", "[phys][physics_grid]") {
    // core/src/terrain/procedural_terrain_source.cpp's pack_key (read-only
    // reference): high 32 bits = uint32(int32(tile_x)), low 32 bits =
    // uint32(int32(tile_y)).
    const std::int64_t packed = PhysicsTileGrid::pack({-1, 2});
    const auto ux = static_cast<std::uint32_t>(static_cast<std::int32_t>(-1));
    const auto uy = static_cast<std::uint32_t>(static_cast<std::int32_t>(2));
    const auto expected = static_cast<std::int64_t>((static_cast<std::uint64_t>(ux) << 32) | uy);
    CHECK(packed == expected);
}

TEST_CASE("l0_keys_for_square: straddles 1, 2 or 4 L0 tiles depending on alignment", "[phys][physics_grid]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame);

    SECTION("tile ix=0: columns [0,255] land entirely inside L0 tile x=0 - one key") {
        std::vector<TileKey> keys;
        grid.l0_keys_for_square({0, 0}, 0, keys);
        REQUIRE(keys.size() == 1);
        CHECK(keys[0].x == 0);
        CHECK(keys[0].y == 0);
    }

    SECTION("tile ix=1: columns [255,510] cross the x=256 boundary - two keys") {
        std::vector<TileKey> keys;
        grid.l0_keys_for_square({1, 0}, 0, keys);
        REQUIRE(keys.size() == 2);
        CHECK(keys[0].x == 0);
        CHECK(keys[1].x == 1);
        CHECK(keys[0].y == keys[1].y);
    }

    SECTION("tile (1,1) straddles both axes - four keys") {
        std::vector<TileKey> keys;
        grid.l0_keys_for_square({1, 1}, 0, keys);
        REQUIRE(keys.size() == 4);
    }
}

TEST_CASE("l0_keys_for_square: floor_div edge at c = 256k-1 vs 256k", "[phys][physics_grid]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame);

    // Tile ix=0's last column is c=255 (== 256*1 - 1): still tile x=0.
    std::vector<TileKey> a;
    grid.l0_keys_for_square({0, 0}, 0, a);
    REQUIRE(a.size() == 1);
    CHECK(a[0].x == 0);

    // Tile ix=1's first column is c=255 (shared with tile 0), its last is
    // c=510: c=256 (== 256*1, the edge itself) falls in x=1, so this tile
    // straddles x=0 and x=1.
    std::vector<TileKey> b;
    grid.l0_keys_for_square({1, 0}, 0, b);
    REQUIRE(b.size() == 2);
    CHECK(b[0].x == 0);
    CHECK(b[1].x == 1);
}

TEST_CASE("l0_keys_for_square: negative e0/n0 offsets", "[phys][physics_grid]") {
    SessionFrame frame(k32N, -256000, -512000); // both exactly L0-tile-aligned (multiples of 256 m)
    PhysicsTileGrid grid(frame);
    std::vector<TileKey> keys;
    grid.l0_keys_for_square({0, 0}, 0, keys);
    REQUIRE(keys.size() == 1);
    CHECK(keys[0].x == -1000);
    CHECK(keys[0].y == -2000);
}

TEST_CASE("l0_keys_for_square: sorted by packed(), unique, over a real square", "[phys][physics_grid]") {
    SessionFrame frame(k32N, 474000, 5560000);
    PhysicsTileGrid grid(frame);
    std::vector<TileKey> keys;
    grid.l0_keys_for_square({3, -2}, 2, keys); // 5x5 physics tiles
    REQUIRE(!keys.empty());
    for (std::size_t i = 1; i < keys.size(); ++i) {
        CHECK(keys[i - 1].packed() < keys[i].packed());
    }
}

TEST_CASE("[AMEND] physics sample k of tile ix sits exactly on a DGM1 cell centre", "[phys][physics_grid]") {
    SessionFrame frame(k32N, 474000, 5560000);
    PhysicsTileGrid grid(frame);
    for (std::int32_t ix : {-3, -1, 0, 1, 4}) {
        for (int k : {0, 1, 127, 254, 255}) {
            const auto origin = grid.origin_local({ix, 0});
            const double local_x = origin[0] + static_cast<double>(k);

            // grid_easting(local_x) must be exactly N + 0.5 for an integer
            // N - the DGM1 cell-centre convention (height_tile.h: sample
            // (i,j)'s centre sits at min_easting + i + 0.5 at level 0).
            const double grid_e = frame.grid_easting(local_x);
            const double floor_e = std::floor(grid_e);
            CHECK(grid_e - floor_e == 0.5);

            // Independent proof via SessionFrame's own tile lookup (not
            // this test's own arithmetic): the sample lands inside exactly
            // one valid L0 tile, at a cell-column offset in [0, 255].
            const auto tile = frame.tile_at({local_x, 0.0}, 0);
            REQUIRE(tile.has_value());
            const double cell_col = floor_e - static_cast<double>(tile->min_easting());
            CHECK(cell_col >= 0.0);
            CHECK(cell_col <= 255.0);
        }
    }
}
