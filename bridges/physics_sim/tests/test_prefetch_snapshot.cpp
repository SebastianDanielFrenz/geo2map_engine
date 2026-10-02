
#include "bridge_test_support.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"
#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"
#include "ps/world/world.h"
#include "ps/backend/shape_desc.h"
#include <catch2/catch_test_macros.hpp>
#include <atomic>
#include <thread>

using namespace g2m_bridge_test;
using g2m::phys::ResidentHeightSet;
using g2m::ps_bridge::G2mTerrainSource;

namespace {
std::int32_t flat(std::int64_t, std::int64_t) { return 25600; }
ps::terrain::TileSample sample(int n) {
    ps::terrain::TileSample out;
    out.heights.resize(static_cast<std::size_t>(n) * n);
    out.cell_surface_ids.resize(static_cast<std::size_t>(n - 1) * (n - 1));
    return out;
}
}

TEST_CASE("Snapshot bridge captures heights and roads, rejects relevant changes only", "[bridge][snapshot]") {
    g2m::phys::PhysicsTileGrid grid(test_frame());
    auto set = std::make_shared<ResidentHeightSet>();
    SyntheticTileCache cache(&flat);
    auto roads = std::make_shared<const std::vector<g2m::RoadSegment>>();
    install_square_with_roads(grid, *set, {0, 0}, 2, cache, roads);
    set->publish_snapshot();
    G2mTerrainSource source(grid, set, g2m::ps_bridge::RoadSurfaceConfig{
        g2m::RoadValueLut::by_land_class(2, {{g2m::LandClass::PavedRoad, 0}}), 2});
    const auto key = source.key_for_world_xy(10, 10);
    std::shared_ptr<const void> inputs;
    REQUIRE(source.acquire_prefetch_inputs(key, inputs));
    auto live = sample(grid.samples()), captured = sample(grid.samples());
    source.fill_tile(key, live);
    source.fill_tile_from(key, inputs.get(), captured);
    CHECK(live.heights == captured.heights);
    CHECK(live.cell_surface_ids == captured.cell_surface_ids);
    auto unrelated = *g2m::tile_key_at(k32N, 0, 900000, 9000000);
    set->install(unrelated, nullptr);
    CHECK(source.prefetch_inputs_current(key, inputs.get()));
    std::vector<g2m::TileKey> footprint;
    grid.l0_keys_for_square({0, 0}, 0, footprint);
    // Pointer identity includes road-only replacement, even unchanged heights.
    auto new_roads = std::make_shared<const std::vector<g2m::RoadSegment>>();
    set->install(footprint.front(), cache.get(footprint.front()), new_roads);
    CHECK_FALSE(source.prefetch_inputs_current(key, inputs.get()));
    set->publish_snapshot();
    REQUIRE(source.acquire_prefetch_inputs(key, inputs));
    set->install(footprint.front(), nullptr);
    CHECK_FALSE(source.prefetch_inputs_current(key, inputs.get()));
    set->retain_only({});
    set->publish_snapshot();
    auto saved = inputs;
    CHECK_FALSE(source.acquire_prefetch_inputs(key, inputs));
    CHECK(inputs == saved); // failed acquisition leaves caller handle untouched
}

TEST_CASE("Snapshot fills stay immutable while owner replaces and evicts live inputs", "[bridge][snapshot]") {
    g2m::phys::PhysicsTileGrid grid(test_frame());
    auto set = std::make_shared<ResidentHeightSet>();
    SyntheticTileCache cache(&flat);
    install_square(grid, *set, {0, 0}, 1, cache);
    set->publish_snapshot();
    G2mTerrainSource source(grid, set, 0);
    const auto key = source.key_for_world_xy(10, 10);
    std::shared_ptr<const void> inputs;
    REQUIRE(source.acquire_prefetch_inputs(key, inputs));
    auto expected = sample(grid.samples());
    source.fill_tile(key, expected);
    std::atomic<bool> start{false}, ok{true};
    std::thread reader([&] {
        auto out = sample(grid.samples());
        while (!start.load(std::memory_order_acquire)) std::this_thread::yield();
        for (int i = 0; i < 32; ++i) {
            source.fill_tile_from(key, inputs.get(), out);
            if (out.heights != expected.heights || out.cell_surface_ids != expected.cell_surface_ids) ok = false;
        }
    });
    start.store(true, std::memory_order_release);
    for (int i = 0; i < 16; ++i) {
        set->retain_only({});
        install_square(grid, *set, {0, 0}, 1, cache);
        set->publish_snapshot();
        set->purge_retired_snapshots();
    }
    reader.join();
    CHECK(ok.load());
    CHECK(source.fill_miss_count() == 0);
}

TEST_CASE("Snapshot callbacks allocate nothing in a physics job", "[bridge][snapshot]") {
    if (!ps::jobs::debug_alloc::hook_available()) SKIP("allocation hook unavailable under sanitizer");
    g2m::phys::PhysicsTileGrid grid(test_frame());
    auto set = std::make_shared<ResidentHeightSet>();
    SyntheticTileCache cache(&flat);
    install_square(grid, *set, {0, 0}, 1, cache);
    set->publish_snapshot();
    G2mTerrainSource source(grid, set, 0);
    const auto key = source.key_for_world_xy(10, 10);
    ps::jobs::JobSystem jobs(1);
    std::atomic<bool> ok{false};
    jobs.parallel_batches(1, [&](std::size_t) {});
    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    jobs.parallel_batches(1, [&](std::size_t) {
        std::shared_ptr<const void> inputs;
        ok = source.acquire_prefetch_inputs(key, inputs) && source.prefetch_inputs_current(key, inputs.get());
    });
    ps::jobs::debug_alloc::set_armed(false);
    CHECK(ok.load());
    CHECK(ps::jobs::debug_alloc::own_violation_count() == 0);
}


TEST_CASE("Snapshot prefetch preserves every crossing tick hash at 1 4 24 workers", "[bridge][snapshot][determinism]") {
    const auto run = [](int workers, bool prefetch) {
        g2m::phys::PhysicsTileGrid grid(test_frame(), 16);
        auto set = std::make_shared<ResidentHeightSet>();
        SyntheticTileCache cache(&flat);
        auto roads = std::make_shared<const std::vector<g2m::RoadSegment>>();
        install_square_with_roads(grid, *set, {0, 0}, 12, cache, roads);
        set->publish_snapshot();
        auto source = std::make_shared<G2mTerrainSource>(grid, set,
            g2m::ps_bridge::RoadSurfaceConfig{g2m::RoadValueLut::by_land_class(0, {}), 0});
        ps::WorldConfig wc;
        wc.tick_rate_hz = 240;
        wc.job_workers = workers;
        ps::World world(wc);
        world.set_terrain_source(source, g2m::ps_bridge::make_terrain_config(10, 1,
            prefetch ? 1 : 0, prefetch ? 24 : 0));
        world.set_terrain_interest_point(0, {5, 5, 0}, 10);
        for (int i = 0; i < 12; ++i) world.step();
        ps::BodyDesc body;
        body.motion = ps::BodyMotionType::Dynamic;
        body.shape = ps::SphereShape{0.3};
        body.mass = 1;
        body.pose.position = {5, 5, 100.4};
        world.create_body(body);
        std::vector<std::uint64_t> hashes;
        for (int i = 0; i < 120; ++i) {
            // Streaming focus crosses five rows; build readiness never selects
            // install ticks or pool slots. Even unpaced/late work must agree.
            world.set_terrain_interest_point(0, {5.0 + (i / 20) * 15.0, 5, 0}, 10);
            world.step();
            hashes.push_back(world.state_hash());
        }
        CHECK(source->fill_miss_count() == 0);
        CHECK(world.terrain_starved_tile_count() == 0);
        if (prefetch) {
            CHECK(world.terrain_prefetch_stats().enqueued > 0);
            CHECK(world.terrain_prefetch_stats().installed > 0);
        }
        return hashes;
    };
    const auto reference = run(1, false);
    CHECK(run(1, true) == reference);
    CHECK(run(4, true) == reference);
    CHECK(run(24, true) == reference);
}
