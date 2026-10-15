// bridges/physics_sim/tests/test_carved_prefetch.cpp - class mode on the served
// carved release (terrain migration phase 6, requests G-2/G-5): the PKT fixture
// release opted into g2m.terrain.carved + g2m.terrain.class, streamed through
// TransportHeightTileFetch (carved heights + class layer) -> HeightTileLoader
// (require_classes) -> PhysicsTerrainStreamer -> ResidentHeightSet ->
// G2mTerrainSource (ClassSurfaceConfig) -> World with Snapshot prefetch.
//
// Checks: the gate opens with every resident key carrying its class tile; a
// snapshot fill equals the live fill; replacing only a class tile invalidates
// a captured snapshot, and a class-less entry declines acquisition; the World's
// per-tick state hash with prefetch on (1 / 4 workers) equals the synchronous
// run, with prefetch installs and no fill miss.
#include "bridge_test_support.h"

#include "fixture_release.h"

#include "g2m/layer/terrain_carve.h"
#include "g2m/layer/terrain_class.h"
#include "g2m/phys/height_tile_loader.h"
#include "g2m/phys/physics_streamer.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"
#include "g2m/world/surface_id_table.h"

#include "ps/backend/shape_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <string>
#include <thread>
#include <vector>

using namespace g2m_bridge_test;
using g2m::phys::HeightTileLoader;
using g2m::phys::InterestPoint;
using g2m::phys::PhysicsTerrainStreamer;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::ResidentHeightSet;
using g2m::ps_bridge::ClassSurfaceConfig;
using g2m::ps_bridge::G2mTerrainSource;

namespace {
ps::terrain::TileSample sample(int n) {
    ps::terrain::TileSample out;
    out.heights.resize(static_cast<std::size_t>(n) * n);
    out.cell_surface_ids.resize(static_cast<std::size_t>(n - 1) * (n - 1));
    return out;
}

g2m::classes::SurfaceIdTable class_table() {
    return g2m::classes::SurfaceIdTable::build([](g2m::SurfaceKind k) { return 100 + static_cast<int>(k); },
                                               [](g2m::LandClass l) { return 200 + static_cast<int>(l); });
}
} // namespace

TEST_CASE("class mode: carved + class tiles from the served release stream, prefetch keeps every tick hash",
          "[bridge][classes][snapshot][fixture]") {
    TempDir dir("carved_prefetch");
    auto fx = g2m::test_support::make_fixture_release(dir.path(), "phase6carved", /*carved=*/true);
    const PhysicsTileGrid grid(fx->session_frame());

    auto resident = std::make_shared<ResidentHeightSet>();
    auto fetch = std::make_shared<g2m::phys::TransportHeightTileFetch>(
        *fx->transport, fx->rid, std::string(g2m::kTerrainCarvedLayer), std::string(g2m::kTerrainClassLayer));
    g2m::phys::LoaderConfig lc;
    lc.require_classes = true;
    HeightTileLoader loader(fetch, lc);
    PhysicsTerrainStreamer streamer(grid, *resident, loader, g2m::phys::StreamerConfig{});
    const InterestPoint ip{0, 700.0, 400.0, 0.0, 0.0, 255.0};
    bool ready = false;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(120);
    while (!ready) {
        ready = streamer.update({&ip, 1}).ready;
        if (ready) break;
        REQUIRE(std::chrono::steady_clock::now() < deadline);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    int resident_with_class = 0, absent = 0;
    for (const g2m::TileKey& k : streamer.gate_keys()) {
        const g2m::HeightTile* h = nullptr;
        std::shared_ptr<const std::vector<g2m::RoadSegment>> roads;
        const g2m::TerrainClassTile* cls = nullptr;
        const auto state = resident->find(k, &h, &roads, &cls);
        if (state == ResidentHeightSet::Lookup::Absent) {
            ++absent;
        } else {
            REQUIRE(state == ResidentHeightSet::Lookup::Resident);
            REQUIRE(cls != nullptr);
            CHECK(cls->key == k);
            ++resident_with_class;
        }
    }
    CHECK(resident_with_class > 0);
    std::printf("class mode gate: %d resident with class tile, %d absent\n", resident_with_class, absent);

    const g2m::classes::SurfaceIdTable table = class_table();
    constexpr ps::SurfaceId kFallback = 7;
    G2mTerrainSource source(grid, resident, ClassSurfaceConfig{table, kFallback});
    const auto key = source.key_for_world_xy(700.0, 400.0);
    std::shared_ptr<const void> inputs;
    REQUIRE(source.acquire_prefetch_inputs(key, inputs));
    auto live = sample(grid.samples()), captured = sample(grid.samples());
    source.fill_tile(key, live);
    source.fill_tile_from(key, inputs.get(), captured);
    CHECK(live.heights == captured.heights);
    CHECK(live.cell_surface_ids == captured.cell_surface_ids);
    CHECK(source.fill_miss_count() == 0);
    int finite = 0;
    for (const ps::real h : live.heights) finite += std::isfinite(static_cast<double>(h)) && h > ps::real(50) ? 1 : 0;
    CHECK(finite == static_cast<int>(live.heights.size()));   // inside coverage: carved ground everywhere
    // The fixture has no OSM and no land cover: every node is terrain of class
    // Unknown, resolved through the table (never the fallback).
    for (const ps::SurfaceId id : live.cell_surface_ids) REQUIRE(id == table.for_land_class(g2m::LandClass::Unknown));

    // A class-only replacement invalidates a captured snapshot; a class-less
    // entry declines acquisition. Done on a mirror of the streamed set (the
    // streamer owns the live one), holding its own shared_ptrs.
    std::vector<g2m::TileKey> footprint;
    grid.l0_keys_for_square(grid.index_for_local(700.0, 400.0), 0, footprint);
    REQUIRE_FALSE(footprint.empty());
    const g2m::TileKey k0 = footprint.front();
    auto mirror = std::make_shared<ResidentHeightSet>();
    std::shared_ptr<const g2m::HeightTile> h0;
    std::shared_ptr<const std::vector<g2m::RoadSegment>> r0;
    std::shared_ptr<const g2m::TerrainClassTile> c0;
    for (const g2m::TileKey& k : streamer.keep_keys()) {
        const g2m::HeightTile* h = nullptr;
        std::shared_ptr<const std::vector<g2m::RoadSegment>> r;
        const g2m::TerrainClassTile* c = nullptr;
        if (resident->find(k, &h, &r, &c) == ResidentHeightSet::Lookup::Missing) continue;
        std::shared_ptr<const g2m::HeightTile> hc = h ? std::make_shared<g2m::HeightTile>(*h) : nullptr;
        std::shared_ptr<const g2m::TerrainClassTile> cc = c ? std::make_shared<g2m::TerrainClassTile>(*c) : nullptr;
        if (k == k0) {
            h0 = hc;
            r0 = r;
            c0 = cc;
        }
        mirror->install(k, std::move(hc), std::move(r), std::move(cc));
    }
    REQUIRE(h0);
    REQUIRE(c0);
    mirror->publish_snapshot();
    G2mTerrainSource mirror_source(grid, mirror, ClassSurfaceConfig{table, kFallback});
    REQUIRE(mirror_source.acquire_prefetch_inputs(key, inputs));
    CHECK(mirror_source.prefetch_inputs_current(key, inputs.get()));
    mirror->install(k0, h0, r0, c0);   // same objects: still current
    CHECK(mirror_source.prefetch_inputs_current(key, inputs.get()));
    mirror->install(k0, h0, r0, std::make_shared<g2m::TerrainClassTile>(*c0));   // new class object only
    CHECK_FALSE(mirror_source.prefetch_inputs_current(key, inputs.get()));
    mirror->install(k0, h0, r0);   // class tile dropped
    mirror->publish_snapshot();
    const auto held = inputs;
    CHECK_FALSE(mirror_source.acquire_prefetch_inputs(key, inputs));
    CHECK(inputs == held);

    // World: prefetch on vs off over the streamed carved/class set.
    const auto run = [&](int workers, bool prefetch) {
        auto src = std::make_shared<G2mTerrainSource>(grid, resident, ClassSurfaceConfig{table, kFallback});
        ps::WorldConfig wc;
        wc.tick_rate_hz = 240;
        wc.job_workers = workers;
        ps::World world(wc);
        world.set_terrain_source(src, g2m::ps_bridge::make_terrain_config(10, 1, prefetch ? 1 : 0, prefetch ? 24 : 0));
        world.set_terrain_interest_point(0, {700, 400, 0}, 10);
        for (int i = 0; i < 12; ++i) world.step();
        ps::BodyDesc body;
        body.motion = ps::BodyMotionType::Dynamic;
        body.shape = ps::SphereShape{0.3};
        body.mass = 1;
        body.pose.position = {700, 400, 400};
        world.create_body(body);
        std::vector<std::uint64_t> hashes;
        for (int i = 0; i < 120; ++i) {
            world.set_terrain_interest_point(0, {700.0 + (i / 20) * 15.0, 400, 0}, 10);
            world.step();
            hashes.push_back(world.state_hash());
        }
        CHECK(src->fill_miss_count() == 0);
        CHECK(world.terrain_starved_tile_count() == 0);
        if (prefetch) CHECK(world.terrain_prefetch_stats().installed > 0);
        return hashes;
    };
    const auto reference = run(1, false);
    CHECK(run(1, true) == reference);
    CHECK(run(4, true) == reference);
}
