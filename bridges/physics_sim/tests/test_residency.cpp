// bridges/physics_sim/tests/test_residency.cpp - residency acceptance on the
// real fixture release (PLAN.md R2.2 section 4b, G7): the full production
// chain - TransportHeightTileFetch -> HeightTileLoader -> PhysicsTerrainStreamer
// -> ResidentHeightSet -> G2mTerrainSource -> World/TileManager - with
// r = 255 m and make_terrain_config(255, 1), started exactly as plan
// section 2's "Start-up in terrain mode" says (block on the gate, then prime
// K = ceil(n/F) + 1 ticks with every tick behind the gate).
//
// The interest point sits 120 m south of the fixture's northern coverage
// edge (N 5561000), so TileManager's own 3x3 square reaches one whole
// physics-tile row entirely outside coverage (404 -> Absent -> holes) and
// one row straddling the edge: the gate must still open (Absent counts as
// ready), Jolt must see holes there and ground inside coverage, and none of
// starved/relief_overflow/fill_miss may move.
#include "bridge_test_support.h"

#include "fixture_release.h"

#include "g2m/phys/height_tile_loader.h"
#include "g2m/phys/physics_streamer.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "ps/backend/shape_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <thread>
#include <vector>

using namespace g2m_bridge_test;
using g2m::phys::GateStatus;
using g2m::phys::HeightTileLoader;
using g2m::phys::InterestPoint;
using g2m::phys::PhysicsTerrainStreamer;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;
using g2m::phys::ResidentHeightSet;
using g2m::ps_bridge::G2mTerrainSource;
using g2m::ps_bridge::make_terrain_config;

TEST_CASE("residency: fixture, r = 255 - no starved/overflow/miss; a 404 area gives holes, not a frozen gate",
         "[bridge][terrain][residency][fixture]") {
    TempDir dir("residency");
    std::unique_ptr<g2m::test_support::ReleaseFixture> fx = g2m::test_support::make_fixture_release(dir.path(), "g7resid");
    const PhysicsTileGrid grid(fx->session_frame());
    REQUIRE(fx->session_frame().e0_m() == kE0);
    REQUIRE(fx->session_frame().n0_m() == kN0);

    constexpr double kRadius = 255.0;
    constexpr double kX = 1000.0, kY = 880.0; // local; coverage is x in [0, 2000), y in [0, 1000)
    REQUIRE(grid.index_for_local(kX, kY) == PhysTileIndex{3, 3});

    auto resident = std::make_shared<ResidentHeightSet>();
    auto fetch = std::make_shared<g2m::phys::TransportHeightTileFetch>(*fx->transport, fx->rid);
    HeightTileLoader loader(fetch, g2m::phys::LoaderConfig{});
    PhysicsTerrainStreamer streamer(grid, *resident, loader, g2m::phys::StreamerConfig{});
    const InterestPoint ip{0, kX, kY, 0.0, 0.0, kRadius};

    auto source = std::make_shared<G2mTerrainSource>(grid, resident, ps::SurfaceId{0});
    ps::WorldConfig cfg;
    cfg.job_workers = 4;
    ps::World world(cfg);
    world.set_terrain_source(source, make_terrain_config(kRadius, 1));

    // Start-up: block on the gate (30 s hard timeout, plan section 2).
    GateStatus status;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(30);
    int updates = 0;
    for (;;) {
        status = streamer.update({&ip, 1});
        ++updates;
        if (status.ready) break;
        REQUIRE(std::chrono::steady_clock::now() < deadline);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    world.set_terrain_interest_point(0, ps::Vec3{kX, kY, 0.0}, kRadius);

    // Prime: K = ceil(9 / F) + 1 ticks, every one behind the gate.
    int stepped = 0, frozen = 0;
    while (stepped < 10) {
        status = streamer.update({&ip, 1});
        if (!status.ready) {
            ++frozen;
            REQUIRE(frozen < 100000);
            std::this_thread::sleep_for(std::chrono::microseconds(200));
            continue;
        }
        world.step();
        ++stepped;
    }
    const g2m::phys::LoaderStats ls = loader.stats();
    std::printf("residency: gate open after %d updates; prime frozen %d; loader ready %llu absent %llu failed %llu "
                "retries %llu; set %u; TileManager resident %zu starved %zu overflow %zu; fill_miss %llu nodata %llu\n",
                updates, frozen, static_cast<unsigned long long>(ls.ready), static_cast<unsigned long long>(ls.absent),
                static_cast<unsigned long long>(ls.failed), static_cast<unsigned long long>(ls.retries), status.resident,
                world.terrain_resident_tile_count(), world.terrain_starved_tile_count(),
                world.terrain_relief_overflow_count(), static_cast<unsigned long long>(source->fill_miss_count()),
                static_cast<unsigned long long>(source->nodata_fill_count()));

    CHECK(status.ready);
    CHECK(status.failed == 0);
    CHECK(ls.failed == 0);
    CHECK(ls.absent > 0); // the 404 area really was requested and answered
    CHECK(frozen == 0);   // nothing moved: the gate, once open, stays open
    CHECK(world.terrain_resident_tile_count() >= 9);
    CHECK(world.terrain_starved_tile_count() == 0);
    CHECK(world.terrain_relief_overflow_count() == 0);
    CHECK(source->fill_miss_count() == 0);

    // Jolt: ground inside coverage, a hole in the 404 row (physics tile
    // iy = 4 spans y 1020.5..1275.5, all beyond N 5561000).
    const ps::RayCastHit inside = world.backend().ray_cast(ps::Vec3{kX, 700.0, 3000.0}, ps::Vec3{0.0, 0.0, -1.0}, 6000.0);
    const ps::RayCastHit outside = world.backend().ray_cast(ps::Vec3{kX, 1200.0, 3000.0}, ps::Vec3{0.0, 0.0, -1.0}, 6000.0);
    std::printf("residency: ray (1000, 700) hit %d z %.3f; ray (1000, 1200) hit %d\n", inside.hit ? 1 : 0, inside.point.z,
                outside.hit ? 1 : 0);
    CHECK(inside.hit);
    CHECK_FALSE(outside.hit);

    // The source itself: the straddling tile (3, 3) has holes north of the
    // edge and real heights south of it; the outside tile (3, 4) is all holes.
    const std::uint32_t n = source->samples_per_tile();
    ps::terrain::TileSample sample;
    sample.heights.assign(static_cast<std::size_t>(n) * n, 0.0);
    sample.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), ps::kInvalidSurfaceId);
    auto count_holes = [&](PhysTileIndex t) {
        source->fill_tile(ps::terrain::TileKey{PhysicsTileGrid::pack(t)}, sample);
        std::size_t holes = 0;
        for (const ps::real h : sample.heights) holes += (h == ps::kHeightfieldNoCollision) ? 1u : 0u;
        return holes;
    };
    const std::size_t holes_edge = count_holes(PhysTileIndex{3, 3});
    const std::size_t holes_out = count_holes(PhysTileIndex{3, 4});
    std::printf("residency: holes in tile (3,3) %zu of %u, tile (3,4) %zu of %u\n", holes_edge, n * n, holes_out, n * n);
    CHECK(holes_edge > 0);
    CHECK(holes_edge < static_cast<std::size_t>(n) * n);
    CHECK(holes_out == static_cast<std::size_t>(n) * n);
    CHECK(source->fill_miss_count() == 0);
}
