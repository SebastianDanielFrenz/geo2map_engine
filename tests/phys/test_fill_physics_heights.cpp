// tests/phys/test_fill_physics_heights.cpp - fill_physics_heights (PLAN.md
// R2.2 section 1a/4a): no row flip, shared seam samples, NoData/Absent ->
// hole, Missing -> MissingTile (with priority over HadNoData), and the
// zero-alloc guarantee at samples = 256 (default) and 1024 (the seamless-
// reference tile size some validation tests use).
#include "g2m/phys/fill_physics_heights.h"

#include "alloc_counter.h"
#include "g2m/core/geo/session_frame.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>
#include <vector>

using g2m::TileKey;
using g2m::geo::Hemisphere;
using g2m::geo::SessionFrame;
using g2m::geo::UtmZone;
using g2m::phys::FillResult;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;
using g2m::phys::ResidentHeightSet;
using g2m::phys::fill_physics_heights;

namespace {
constexpr UtmZone k32N{32, Hemisphere::North};
constexpr double kHole = -9999.0;

std::shared_ptr<g2m::HeightTile> make_tile(const TileKey& key) {
    auto tile = std::make_shared<g2m::HeightTile>();
    tile->key = key;
    return tile;
}
} // namespace

TEST_CASE("fill_physics_heights: no row flip - out[j*n+k] matches HeightTile::h[j*256+k] directly",
         "[phys][fill]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame); // samples = 256, so tile (0,0) covers L0 tile (0,0) exactly
    const int n = grid.samples();

    const TileKey key{k32N, 0, 0, 0};
    auto tile = make_tile(key);
    // An asymmetric pattern: a plain south-to-north flip would swap row 0
    // and row 255, changing every value below.
    for (int j = 0; j < 256; ++j) {
        for (int i = 0; i < 256; ++i) {
            tile->h[static_cast<std::size_t>(j) * 256 + static_cast<std::size_t>(i)] =
                static_cast<std::int32_t>(j * 1000 + i);
        }
    }

    ResidentHeightSet resident;
    resident.install(key, tile);

    std::vector<double> out(static_cast<std::size_t>(n) * static_cast<std::size_t>(n));
    const FillResult result = fill_physics_heights<double>(grid, {0, 0}, resident, out.data(), kHole);
    CHECK(result == FillResult::Ok);

    for (int j = 0; j < n; ++j) {
        for (int k = 0; k < n; ++k) {
            CAPTURE(j, k);
            const double expected = static_cast<double>(j * 1000 + k) / 256.0;
            CHECK(out[static_cast<std::size_t>(j) * n + k] == expected);
        }
    }
}

TEST_CASE("fill_physics_heights: adjacent physics tiles share a bit-exact seam sample", "[phys][fill]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame); // samples = 256, ts = 255
    const int n = grid.samples();

    const TileKey key_x0{k32N, 0, 0, 0};
    const TileKey key_x1{k32N, 0, 1, 0};
    auto tile0 = make_tile(key_x0);
    auto tile1 = make_tile(key_x1);
    for (std::size_t idx = 0; idx < tile0->h.size(); ++idx) {
        tile0->h[idx] = static_cast<std::int32_t>(idx * 7 + 3);
        tile1->h[idx] = static_cast<std::int32_t>(idx * 11 + 5);
    }

    ResidentHeightSet resident;
    resident.install(key_x0, tile0);
    resident.install(key_x1, tile1);

    std::vector<double> out0(static_cast<std::size_t>(n) * n);
    std::vector<double> out1(static_cast<std::size_t>(n) * n);
    CHECK(fill_physics_heights<double>(grid, {0, 0}, resident, out0.data(), kHole) == FillResult::Ok);
    CHECK(fill_physics_heights<double>(grid, {1, 0}, resident, out1.data(), kHole) == FillResult::Ok);

    // Physics tile ix=0's last column (k = 255) and tile ix=1's first column
    // (k = 0) both read L0 tile x=0's own column 255 (see physics_grid.h's
    // anchoring derivation): they must be bit-exact, not just close.
    for (int j = 0; j < n; ++j) {
        CAPTURE(j);
        CHECK(out0[static_cast<std::size_t>(j) * n + 255] == out1[static_cast<std::size_t>(j) * n + 0]);
    }
}

TEST_CASE("fill_physics_heights: a resident tile's NoData sample writes hole and reports HadNoData",
         "[phys][fill]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame);
    const int n = grid.samples();

    const TileKey key{k32N, 0, 0, 0};
    auto tile = make_tile(key);
    tile->h.fill(1000); // every sample = 1000/256 m
    tile->h[static_cast<std::size_t>(42) * 256 + 17] = g2m::kHeightNoData;
    tile->has_nodata = true;

    ResidentHeightSet resident;
    resident.install(key, tile);

    std::vector<double> out(static_cast<std::size_t>(n) * n, 12345.0);
    const FillResult result = fill_physics_heights<double>(grid, {0, 0}, resident, out.data(), kHole);
    CHECK(result == FillResult::HadNoData);
    CHECK(out[static_cast<std::size_t>(42) * n + 17] == kHole);
    // Everything else is the ordinary resident value.
    CHECK(out[0] == 1000.0 / 256.0);
    CHECK(out[static_cast<std::size_t>(41) * n + 17] == 1000.0 / 256.0);
}

TEST_CASE("fill_physics_heights: a fully-Absent tile writes hole everywhere but stays Ok", "[phys][fill]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame);
    const int n = grid.samples();

    ResidentHeightSet resident;
    resident.install(TileKey{k32N, 0, 0, 0}, nullptr); // confirmed 404

    std::vector<double> out(static_cast<std::size_t>(n) * n, 12345.0);
    const FillResult result = fill_physics_heights<double>(grid, {0, 0}, resident, out.data(), kHole);
    CHECK(result == FillResult::Ok); // Absent alone is NOT HadNoData - see fill_physics_heights.h's design note
    for (double v : out) {
        CHECK(v == kHole);
    }
}

TEST_CASE("fill_physics_heights: an uninstalled L0 tile writes hole and reports MissingTile", "[phys][fill]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame);
    const int n = grid.samples();

    ResidentHeightSet resident; // nothing installed at all
    std::vector<double> out(static_cast<std::size_t>(n) * n, 12345.0);
    const FillResult result = fill_physics_heights<double>(grid, {0, 0}, resident, out.data(), kHole);
    CHECK(result == FillResult::MissingTile);
    for (double v : out) {
        CHECK(v == kHole);
    }
}

TEST_CASE("fill_physics_heights: MissingTile takes priority over HadNoData in the same fill", "[phys][fill]") {
    // samples = 4 (ts = 3), e0 = 0 (a valid SessionFrame origin - must be a
    // multiple of 1000) and physics tile ix = 85: col0 = ts*ix = 255, so
    // this tile's 4 columns (255, 256, 257, 258) straddle L0 tile x=0
    // (column 255 only) and x=1 (columns 256-258).
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame, 4);
    const int n = grid.samples();
    REQUIRE(n == 4);
    const PhysTileIndex index{85, 0};

    const TileKey key_x0{k32N, 0, 0, 0};
    auto tile0 = make_tile(key_x0);
    tile0->h.fill(2000);
    // Row j=0's sample (local column 255, local row 0) is NoData.
    tile0->h[static_cast<std::size_t>(0) * 256 + 255] = g2m::kHeightNoData;

    ResidentHeightSet resident;
    resident.install(key_x0, tile0);
    // L0 tile x=1 (columns 256-258, supplying k=1,2,3 of every row) is
    // deliberately left Missing.

    std::vector<double> out(static_cast<std::size_t>(n) * n, 12345.0);
    const FillResult result = fill_physics_heights<double>(grid, index, resident, out.data(), kHole);
    CHECK(result == FillResult::MissingTile);

    // Row j=0: k=0 (-> L0 x=0, the NoData sample) is a hole; k=1..3 (-> L0
    // x=1, Missing) are holes too.
    CHECK(out[0] == kHole);
    CHECK(out[1] == kHole);
    CHECK(out[2] == kHole);
    CHECK(out[3] == kHole);
    // Row j=1: k=0 (-> L0 x=0, an ordinary resident sample) is real; k=1..3
    // are still holes (Missing).
    CHECK(out[n + 0] == 2000.0 / 256.0);
    CHECK(out[n + 1] == kHole);
}

TEST_CASE("fill_physics_heights: allocates nothing, samples = 256 and 1024", "[phys][fill][alloc]") {
    if (!g2m_test::kAllocCounterAvailable) {
        SKIP("AllocCounter is disabled on a sanitizer build");
    }

    for (const int samples : {256, 1024}) {
        CAPTURE(samples);
        SessionFrame frame(k32N, 474000, 5560000);
        PhysicsTileGrid grid(frame, samples);
        const PhysTileIndex index{3, -2};

        // Pre-populate every L0 tile this physics tile can touch, so the
        // timed call below never even takes the (allocation-free anyway)
        // Missing branch - matches how a primed PhysicsTerrainStreamer feeds
        // fill_physics_heights in production.
        std::vector<TileKey> keys;
        grid.l0_keys_for_square(index, 0, keys);
        REQUIRE(!keys.empty());

        ResidentHeightSet resident;
        for (const auto& key : keys) {
            resident.install(key, make_tile(key));
        }

        std::vector<double> out(static_cast<std::size_t>(samples) * static_cast<std::size_t>(samples));

        // Warm-up call outside the counter (first-touch page faults etc.).
        CHECK(fill_physics_heights<double>(grid, index, resident, out.data(), kHole) == FillResult::Ok);

        {
            g2m_test::ScopedAllocCounter counter;
            const FillResult result = fill_physics_heights<double>(grid, index, resident, out.data(), kHole);
            CHECK(result == FillResult::Ok);
            CHECK(counter.count() == 0);
        }
    }
}
