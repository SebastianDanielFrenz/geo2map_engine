// tests/phys/test_fill_physics_surfaces.cpp - fill_physics_surfaces (PLAN.md
// G2.5a-grip section 2/3/4, commit G-b): cell-centre placement, the 2x2 L0
// straddle's owner-rule independence, the Absent/Missing/Resident-without-
// roads outcome table (never silently grass), and the zero-alloc guarantee.
#include "g2m/phys/fill_physics_surfaces.h"

#include "alloc_counter.h"
#include "g2m/core/geo/session_frame.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <initializer_list>
#include <memory>
#include <vector>

using g2m::LandClass;
using g2m::RoadBlockStats;
using g2m::RoadSegment;
using g2m::RoadValueLut;
using g2m::SurfaceKind;
using g2m::TileKey;
using g2m::geo::Hemisphere;
using g2m::geo::SessionFrame;
using g2m::geo::UtmZone;
using g2m::phys::fill_physics_surfaces;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;
using g2m::phys::ResidentHeightSet;

namespace {
constexpr UtmZone k32N{32, Hemisphere::North};

std::shared_ptr<const g2m::HeightTile> make_tile(const TileKey& key) {
    auto tile = std::make_shared<g2m::HeightTile>();
    tile->key = key;
    return tile;
}

std::shared_ptr<const std::vector<RoadSegment>> roads_of(std::initializer_list<RoadSegment> segs) {
    return std::make_shared<const std::vector<RoadSegment>>(segs);
}
} // namespace

TEST_CASE("fill_physics_surfaces: a road segment paints only its own cell's centre", "[phys][fill][roads]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame); // samples = 256, ts = 255; tile (0,0) touches L0 (0,0) only
    const PhysTileIndex index{0, 0};
    const int cells = grid.samples() - 1;
    constexpr int kX = 10, kY = 20;

    // Cell (kX, kY)'s centre (PLAN.md G2.5a-grip section 2): E_c = e0 + 0 +
    // kX + 1 = 11, N_c = n0 + 0 + kY + 1 = 21 (metres) - the midpoint between
    // two DGM1 cell-centre height samples, never a fractional cell.
    const std::int64_t ec_mm = 11 * 1000;
    const std::int64_t nc_mm = 21 * 1000;
    // A short capsule (not a degenerate point) straddling that centre;
    // half_width_mm 50 is widened by paint_road_segments' own 710 mm minimum
    // clamp - still under the 1 m sample spacing, so no neighbouring cell is
    // ever covered.
    const RoadSegment seg{{ec_mm - 10, nc_mm}, {ec_mm + 10, nc_mm}, /*half_width_mm=*/50, LandClass::PavedRoad,
                          SurfaceKind::Asphalt, /*rank=*/1};

    ResidentHeightSet resident;
    const TileKey key{k32N, 0, 0, 0};
    resident.install(key, make_tile(key), roads_of({seg}));

    const RoadValueLut lut = RoadValueLut::by_land_class(/*off_road=*/0, {{LandClass::PavedRoad, static_cast<std::uint16_t>(111)}});
    std::vector<std::uint16_t> out(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells), 0xBEEF);
    const RoadBlockStats stats = fill_physics_surfaces(grid, index, resident, lut, /*off_road=*/0, out);

    CHECK(stats.segments_blocks == 1);
    CHECK(stats.none_blocks == 0);
    CHECK(stats.missing_blocks == 0);

    auto at = [&](int x, int y) { return out[static_cast<std::size_t>(y) * static_cast<std::size_t>(cells) + static_cast<std::size_t>(x)]; };
    CHECK(at(kX, kY) == 111);
    CHECK(at(kX - 1, kY) == 0);
    CHECK(at(kX + 1, kY) == 0);
    CHECK(at(kX, kY - 1) == 0);
    CHECK(at(kX, kY + 1) == 0);
}

TEST_CASE("fill_physics_surfaces: a 2x2 L0 straddle paints each quadrant from its own owner tile only",
         "[phys][fill][roads]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame); // samples = 256, ts = 255
    const PhysTileIndex index{4, 4};
    const int cells = grid.samples() - 1;

    // col0 = row0 = ts*4 = 1020. Cell centre E_c/N_c = 1020 + x/y + 1: cells
    // x (or y) in [0, 2] -> global metre in [1021, 1023], L0 tile 3
    // (floor(1021/256) = floor(1023/256) = 3); cells in [3, 254] -> [1024,
    // 1275], L0 tile 4. Two parent() calls (owner = floor(tx/4), same
    // derivation as tests/layer/test_osm_paint.cpp's "parent() twice" test):
    // floor(3/4) = 0, floor(4/4) = 1 - four genuinely distinct L2 owners, one
    // per quadrant.
    struct Quadrant {
        TileKey l0;
        int x, y; // a representative cell well inside this quadrant
        LandClass land_class;
        std::uint16_t expected;
    };
    const Quadrant quads[4] = {
        {TileKey{k32N, 0, 3, 3}, 1, 1, LandClass::PavedRoad, 111},
        {TileKey{k32N, 0, 4, 3}, 10, 1, LandClass::UnpavedRoad, 222},
        {TileKey{k32N, 0, 3, 4}, 1, 10, LandClass::Rail, 333},
        {TileKey{k32N, 0, 4, 4}, 10, 10, LandClass::Water, 444},
    };

    ResidentHeightSet resident;
    for (const Quadrant& q : quads) {
        const std::int64_t ec_mm = (1020 + q.x + 1) * 1000;
        const std::int64_t nc_mm = (1020 + q.y + 1) * 1000;
        const RoadSegment seg{{ec_mm - 10, nc_mm}, {ec_mm + 10, nc_mm}, 50, q.land_class, SurfaceKind::Unknown, 1};
        resident.install(q.l0, make_tile(q.l0), roads_of({seg}));
    }

    const RoadValueLut lut = RoadValueLut::by_land_class(
        /*off_road=*/0,
        {{LandClass::PavedRoad, static_cast<std::uint16_t>(111)}, {LandClass::UnpavedRoad, static_cast<std::uint16_t>(222)}, {LandClass::Rail, static_cast<std::uint16_t>(333)}, {LandClass::Water, static_cast<std::uint16_t>(444)}});
    std::vector<std::uint16_t> out(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells));
    const RoadBlockStats stats = fill_physics_surfaces(grid, index, resident, lut, /*off_road=*/0, out);

    CHECK(stats.segments_blocks == 4);
    CHECK(stats.none_blocks == 0);
    CHECK(stats.missing_blocks == 0);

    auto at = [&](int x, int y) { return out[static_cast<std::size_t>(y) * static_cast<std::size_t>(cells) + static_cast<std::size_t>(x)]; };
    for (const Quadrant& q : quads) {
        CAPTURE(q.x, q.y);
        CHECK(at(q.x, q.y) == q.expected);
    }
}

TEST_CASE("fill_physics_surfaces: an Absent L0 tile paints off_road and is not counted as a miss",
         "[phys][fill][roads]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame, 8); // ts = 7, single L0 tile (0,0) touched
    const PhysTileIndex index{0, 0};
    const int cells = grid.samples() - 1;

    ResidentHeightSet resident;
    resident.install(TileKey{k32N, 0, 0, 0}, nullptr); // confirmed 404, no roads

    const RoadValueLut lut = RoadValueLut::by_land_class(/*off_road=*/7, {{LandClass::PavedRoad, static_cast<std::uint16_t>(111)}});
    std::vector<std::uint16_t> out(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells), 0xBEEF);
    const RoadBlockStats stats = fill_physics_surfaces(grid, index, resident, lut, /*off_road=*/7, out);

    CHECK(stats.segments_blocks == 0);
    CHECK(stats.none_blocks == 1);
    CHECK(stats.missing_blocks == 0); // Absent is confirmed no-coverage, not a miss
    for (std::uint16_t v : out) CHECK(v == 7);
}

TEST_CASE("fill_physics_surfaces: an uninstalled L0 tile paints off_road and is counted as a miss",
         "[phys][fill][roads]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame, 8);
    const PhysTileIndex index{0, 0};
    const int cells = grid.samples() - 1;

    ResidentHeightSet resident; // nothing installed at all

    const RoadValueLut lut = RoadValueLut::by_land_class(/*off_road=*/7, {{LandClass::PavedRoad, static_cast<std::uint16_t>(111)}});
    std::vector<std::uint16_t> out(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells), 0xBEEF);
    const RoadBlockStats stats = fill_physics_surfaces(grid, index, resident, lut, /*off_road=*/7, out);

    CHECK(stats.segments_blocks == 0);
    CHECK(stats.none_blocks == 0);
    CHECK(stats.missing_blocks == 1);
    for (std::uint16_t v : out) CHECK(v == 7);
}

TEST_CASE("fill_physics_surfaces: a resident tile with a null roads list paints off_road but is "
         "counted as a miss (loud - never silently grass)",
         "[phys][fill][roads]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame, 8);
    const PhysTileIndex index{0, 0};
    const int cells = grid.samples() - 1;
    const TileKey key{k32N, 0, 0, 0};

    ResidentHeightSet resident;
    resident.install(key, make_tile(key)); // 2-arg install: heights resident, roads left null

    const RoadValueLut lut = RoadValueLut::by_land_class(/*off_road=*/7, {{LandClass::PavedRoad, static_cast<std::uint16_t>(111)}});
    std::vector<std::uint16_t> out(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells), 0xBEEF);
    const RoadBlockStats stats = fill_physics_surfaces(grid, index, resident, lut, /*off_road=*/7, out);

    CHECK(stats.segments_blocks == 0);
    CHECK(stats.none_blocks == 0);
    CHECK(stats.missing_blocks == 1); // resident heights alone is not enough - roads must ride along too
    for (std::uint16_t v : out) CHECK(v == 7);
}

TEST_CASE("fill_physics_surfaces: a resident tile with a non-null empty roads list is a genuine "
         "no-drivable-ways tile, not a miss",
         "[phys][fill][roads]") {
    SessionFrame frame(k32N, 0, 0);
    PhysicsTileGrid grid(frame, 8);
    const PhysTileIndex index{0, 0};
    const int cells = grid.samples() - 1;
    const TileKey key{k32N, 0, 0, 0};

    ResidentHeightSet resident;
    resident.install(key, make_tile(key), std::make_shared<const std::vector<RoadSegment>>());

    const RoadValueLut lut = RoadValueLut::by_land_class(/*off_road=*/7, {{LandClass::PavedRoad, static_cast<std::uint16_t>(111)}});
    std::vector<std::uint16_t> out(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells), 0xBEEF);
    const RoadBlockStats stats = fill_physics_surfaces(grid, index, resident, lut, /*off_road=*/7, out);

    CHECK(stats.segments_blocks == 0);
    CHECK(stats.none_blocks == 1);
    CHECK(stats.missing_blocks == 0);
    for (std::uint16_t v : out) CHECK(v == 7);
}

TEST_CASE("fill_physics_surfaces: allocates nothing", "[phys][fill][roads][alloc]") {
    if (!g2m_test::kAllocCounterAvailable) {
        SKIP("AllocCounter is disabled on a sanitizer build");
    }

    SessionFrame frame(k32N, 474000, 5560000);
    PhysicsTileGrid grid(frame); // samples = 256 (default)
    const PhysTileIndex index{3, -2};
    const int cells = grid.samples() - 1;

    // Pre-populate every L0 tile this physics tile can touch, each with a
    // real (if geometrically irrelevant) roads list - matches how a primed
    // PhysicsTerrainStreamer feeds fill_physics_surfaces in production.
    std::vector<TileKey> keys;
    grid.l0_keys_for_square(index, 0, keys);
    REQUIRE(!keys.empty());

    ResidentHeightSet resident;
    const RoadSegment seg{{0, 0}, {1000, 0}, 50, LandClass::PavedRoad, SurfaceKind::Asphalt, 1};
    for (const auto& key : keys) {
        resident.install(key, make_tile(key), roads_of({seg}));
    }

    const RoadValueLut lut = RoadValueLut::by_land_class(/*off_road=*/0, {{LandClass::PavedRoad, static_cast<std::uint16_t>(111)}});
    std::vector<std::uint16_t> out(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells));

    // Warm-up call outside the counter (first-touch page faults etc.).
    fill_physics_surfaces(grid, index, resident, lut, /*off_road=*/0, out);

    {
        g2m_test::ScopedAllocCounter counter;
        fill_physics_surfaces(grid, index, resident, lut, /*off_road=*/0, out);
        CHECK(counter.count() == 0);
    }
}
