// bridges/physics_sim/tests/test_g2m_terrain_source_roads.cpp -
// G2mTerrainSource's road mode (PLAN.md G2.5a-grip section 1/5/6, commit
// G-c): the second constructor (RoadSurfaceConfig) reads per-cell grip from
// OSM roads via g2m::phys::fill_physics_surfaces instead of one flat
// SurfaceId. Covers the road-mode bullets of PLAN.md R2.2 section 4b's
// "contract"/"fill_tile zero-alloc" scope (test_g2m_terrain_source.cpp's own
// header comment) plus the cast-through-Jolt check that scope note excludes
// for the uniform-surface case (there is nothing to distinguish there; here
// the whole point is per-cell surface, so it is in scope).
#include "bridge_test_support.h"

#include "g2m/layer/osm_roads.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "ps/backend/shape_desc.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>
#include <vector>

using namespace g2m_bridge_test;
using g2m::LandClass;
using g2m::RoadSegment;
using g2m::RoadValueLut;
using g2m::SurfaceKind;
using g2m::TileKey;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;
using g2m::phys::ResidentHeightSet;
using g2m::ps_bridge::G2mTerrainSource;
using g2m::ps_bridge::make_terrain_config;
using g2m::ps_bridge::RoadSurfaceConfig;

namespace {

// Flat 100 m - the road-painting result is independent of height, so a flat
// field keeps every cast's expected z trivial.
std::int32_t flat100(std::int64_t, std::int64_t) { return static_cast<std::int32_t>(100.0 * 256.0); }

SyntheticTileCache& flat_cache() {
    static SyntheticTileCache cache(&flat100);
    return cache;
}

constexpr std::uint16_t kOffRoadId = 1;
constexpr std::uint16_t kPavedId = 5;

RoadValueLut test_lut() { return RoadValueLut::by_land_class(kOffRoadId, {{LandClass::PavedRoad, kPavedId}}); }

// One long paved capsule along local y = `y`, spanning local x in
// [x_lo, x_hi] (absolute UTM mm, kE0/kN0-anchored - bridge_test_support.h).
// Reused as-is for every L0 tile a caller installs it into
// (install_square_with_roads): paint_road_blocks (osm_roads.cpp's
// for_each_block) only ever paints a block's own sub-rectangle, so handing
// every owning tile the SAME unclipped segment is equivalent to a real
// per-tile-clipped extraction (see fill_physics_surfaces.h/osm_roads.cpp).
std::shared_ptr<const std::vector<RoadSegment>> straight_road(double x_lo, double x_hi, double y) {
    const std::int64_t y_mm = (kN0 + static_cast<std::int64_t>(y)) * 1000;
    const RoadSegment seg{{(kE0 + static_cast<std::int64_t>(x_lo)) * 1000, y_mm},
                         {(kE0 + static_cast<std::int64_t>(x_hi)) * 1000, y_mm},
                         /*half_width_mm=*/3000,
                         LandClass::PavedRoad,
                         SurfaceKind::Asphalt,
                         /*rank=*/1};
    return std::make_shared<const std::vector<RoadSegment>>(std::vector<RoadSegment>{seg});
}

// Same "square resident, prime the World" shape as
// test_render_vs_jolt.cpp's own prime() - K = ceil(tiles/F) + 1 ticks at
// F = fills_per_tick (PLAN.md R2.2 section 2 "Prime").
void prime(ps::World& world, std::size_t tiles, std::uint32_t fills_per_tick) {
    const std::size_t k = (tiles + fills_per_tick - 1) / fills_per_tick + 1;
    for (std::size_t t = 0; t < k; ++t) world.step();
}

} // namespace

TEST_CASE("G2mTerrainSource road mode: shape cast reads the paved id on-road and off_road 30 m off, through a real "
         "World/TileManager/Jolt heightfield",
         "[bridge][terrain][roads]") {
    const PhysicsTileGrid grid(test_frame());
    auto resident = std::make_shared<ResidentHeightSet>();
    // Road along local y = 382 (mid physics tile (1,1)), spanning x [0, 800]
    // - comfortably inside the r=2 install square around tile (1,1).
    install_square_with_roads(grid, *resident, PhysTileIndex{1, 1}, 2, flat_cache(), straight_road(0.0, 800.0, 382.0));

    auto source = std::make_shared<G2mTerrainSource>(grid, resident, RoadSurfaceConfig{test_lut(), kOffRoadId});

    ps::WorldConfig cfg;
    cfg.job_workers = 4;
    ps::World world(cfg);
    world.set_terrain_source(source, make_terrain_config(255.0, 1));
    // Centre of physics tile (1,1), radius 255 -> TileManager square (0..2, 0..2).
    world.set_terrain_interest_point(1, ps::Vec3{382.5, 382.5, 0.0}, 255.0);
    prime(world, 9, 1);
    REQUIRE(world.terrain_starved_tile_count() == 0);
    REQUIRE(source->fill_miss_count() == 0);

    ps::IRigidBackend& backend = world.backend();
    auto cast_surface = [&](double x, double y) {
        const ps::Pose start{ps::Vec3{x, y, 110.0}, ps::Quat::identity()};
        const ps::ShapeCastHit hit = backend.shape_cast(ps::SphereShape{0.05}, start, ps::Vec3{0, 0, -1}, 20.0);
        REQUIRE(hit.hit);
        return hit.surface;
    };

    CHECK(cast_surface(400.0, 382.0) == kPavedId);   // on the road's own line
    CHECK(cast_surface(400.0, 412.0) == kOffRoadId); // 30 m north of it
}

TEST_CASE("G2mTerrainSource road mode: fill_tile is allocation-free across concurrent jobs",
         "[bridge][terrain][roads][alloc]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        SKIP("debug_alloc hook is disabled on this build (ASan/TSan)");
    }

    const PhysicsTileGrid grid(test_frame());
    auto resident = std::make_shared<ResidentHeightSet>();

    constexpr int kNumIndices = 64;
    std::vector<PhysTileIndex> indices;
    indices.reserve(kNumIndices);
    for (int i = 0; i < kNumIndices; ++i) indices.push_back(PhysTileIndex{i % 8, i / 8});
    const auto roads = straight_road(-50.0, 2100.0, 100.0); // spans every index's own tile footprint
    for (const auto& index : indices) install_square_with_roads(grid, *resident, index, 0, flat_cache(), roads);

    auto source = std::make_shared<G2mTerrainSource>(grid, resident, RoadSurfaceConfig{test_lut(), kOffRoadId});
    const std::uint32_t n = source->samples_per_tile();

    std::vector<ps::terrain::TileKey> keys;
    keys.reserve(indices.size());
    for (const auto& index : indices) keys.push_back(ps::terrain::TileKey{PhysicsTileGrid::pack(index)});

    std::vector<ps::terrain::TileSample> samples(keys.size());
    for (auto& out : samples) {
        out.heights.assign(static_cast<std::size_t>(n) * n, 0.0);
        out.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), ps::kInvalidSurfaceId);
    }

    ps::jobs::JobSystem jobs(4);

    // Warm-up call outside the counter (first-touch page faults, thread
    // pool spin-up).
    jobs.parallel_batches(keys.size(), [&](std::size_t i) { source->fill_tile(keys[i], samples[i]); });

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    jobs.parallel_batches(keys.size(), [&](std::size_t i) { source->fill_tile(keys[i], samples[i]); });
    ps::jobs::debug_alloc::set_armed(false);

    CHECK(ps::jobs::debug_alloc::own_violation_count() == 0);
    // The road really painted something (not every cell fell back to
    // off_road) - otherwise the alloc check above would be checking a
    // vacuous heights-only path in road-mode clothing.
    bool saw_paved = false;
    for (const auto& out : samples) {
        for (const std::uint16_t id : out.cell_surface_ids) {
            if (id == kPavedId) saw_paved = true;
        }
    }
    CHECK(saw_paved);
}

// Negative control for the test above (same rationale as
// test_g2m_terrain_source.cpp's own): proves the hook really sees an
// allocation made on a worker thread inside a job while armed.
TEST_CASE("G2mTerrainSource road mode: zero-alloc negative control - an allocation inside a job IS counted",
         "[bridge][terrain][roads][alloc]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        SKIP("debug_alloc hook is disabled on this build (ASan/TSan)");
    }
    constexpr std::size_t kJobs = 64;
    std::vector<void*> sinks(kJobs, nullptr); // sized up front, outside the armed window
    ps::jobs::JobSystem jobs(4);
    jobs.parallel_batches(kJobs, [&](std::size_t) {}); // warm-up, as above

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    // A direct ::operator new call, not a new-expression: the compiler may
    // not elide it (a new-expression's allocation may be elided/merged).
    jobs.parallel_batches(kJobs, [&](std::size_t i) { sinks[i] = ::operator new(64); });
    ps::jobs::debug_alloc::set_armed(false);
    const std::uint64_t counted = ps::jobs::debug_alloc::own_violation_count();
    for (void* p : sinks) ::operator delete(p);

    CHECK(counted >= 1);
    CHECK(counted == kJobs);
}
