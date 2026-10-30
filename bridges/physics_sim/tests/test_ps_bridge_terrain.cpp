// Phase 9 (terrain migration plan 6.6): PhysicsTerrain, the physics terrain stack moved from racing_game's
// Session::Terrain / setup_terrain / gate_check. Smoke tests on synthetic DGM1 tiles (no store, no network): install,
// the non-blocking gate update, priming, check_primed, the surface mode -> loader requirement mapping, and the release
// of a vanished interest point.
#include "bridge_test_support.h"

#include "g2m/layer/terrain_class.h"
#include "g2m/ps_bridge/physics_terrain.h"

#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

using namespace g2m_bridge_test;
using g2m::ps_bridge::PhysicsTerrain;

namespace {

constexpr double kRadius = 400.0;
constexpr double kStartX = 230.0, kStartY = 240.0;

std::int32_t gentle(std::int64_t c, std::int64_t r) {
    const double h = 150.0 + 0.05 * static_cast<double>((c - kE0) % 40) + 0.03 * static_cast<double>((r - kN0) % 30);
    return static_cast<std::int32_t>(std::lround(h * 256.0));
}

SyntheticTileCache& cache() {
    static SyntheticTileCache c(&gentle);
    return c;
}

// Serves the synthetic tiles. roads: none / an empty list (a confirmed "no drivable ways") / a null list that claims to
// be provided (a fetch that reports roads and delivers none, retried then Failed under require_roads); classes likewise.
class SyntheticFetch final : public g2m::phys::IHeightTileFetch {
public:
    enum class Roads { None, Empty, ClaimedNull };
    SyntheticFetch(Roads roads, bool classes) : roads_(roads), classes_(classes) {}
    g2m::phys::FetchResult fetch(const g2m::TileKey& key) override {
        g2m::phys::FetchResult r{g2m::Status::Ok, cache().get(key), nullptr};
        if (roads_ == Roads::Empty) r.roads = std::make_shared<const std::vector<g2m::RoadSegment>>();
        if (classes_) {
            auto c = std::make_shared<g2m::TerrainClassTile>();
            c->reset(key);
            r.classes = c;
        }
        return r;
    }
    bool provides_roads() const override { return roads_ != Roads::None; }
    bool provides_classes() const override { return classes_; }

private:
    Roads roads_;
    bool classes_;
};

PhysicsTerrain::Config config_for(std::shared_ptr<g2m::phys::IHeightTileFetch> fetch) {
    PhysicsTerrain::Config c{test_frame(), std::move(fetch), {}, {}};
    c.loader.workers = 2;
    c.loader.max_attempts = 2;
    c.loader.backoff = std::chrono::milliseconds(1);
    return c;
}

g2m::phys::InterestPoint point(std::uint32_t id, double x, double y) {
    g2m::phys::InterestPoint p;
    p.id = id;
    p.x = x;
    p.y = y;
    p.radius_m = kRadius;
    return p;
}

// update() every 1 ms until the gate is ready (false on timeout), as the game's start-up loop does.
bool open_gate(PhysicsTerrain& t, ps::World& world, std::span<const g2m::phys::InterestPoint> points, double timeout_s = 30.0,
               g2m::phys::GateStatus* last = nullptr) {
    const auto until = std::chrono::steady_clock::now() + std::chrono::duration<double>(timeout_s);
    for (;;) {
        const auto gs = t.update(world, points);
        if (last != nullptr) *last = gs;
        if (gs.ready) return true;
        if (std::chrono::steady_clock::now() > until) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

ps::WorldConfig world_config() {
    ps::WorldConfig cfg;
    cfg.job_workers = 2;
    return cfg;
}

} // namespace

TEST_CASE("ps_bridge terrain: install, gate, prime and check_primed", "[ps_bridge][terrain]") {
    ps::World world(world_config());
    PhysicsTerrain t(config_for(std::make_shared<SyntheticFetch>(SyntheticFetch::Roads::None, false)), ps::SurfaceId{0});

    // install() validates before it touches the World.
    for (const auto& bad : {std::make_pair(2u, 16u), std::make_pair(1u, 0u), std::make_pair(0u, 5u), std::make_pair(1u, 2000u)}) {
        const auto r = t.install(world, kRadius, 1, bad.first, bad.second);
        REQUIRE(!r.ok());
        CHECK(r.error().message == "Session: invalid terrain prefetch margin/cap (margin exceeds streamer gate)");
    }
    CHECK(!t.install(world, 0.0, 1, 0, 0).ok());
    CHECK(!t.install(world, kRadius, 0, 0, 0).ok());
    CHECK(!t.install(world, kRadius, 1, 0, 0, 0).ok());
    CHECK(t.check_primed(world).ok() == false); // nothing installed, nothing primed: radius 0 -> square 1, none resident

    REQUIRE(t.install(world, kRadius, 1, 0, 0).ok());
    // The game's numbers: r = ceil(400 / 255) = 2 -> a 5 x 5 square = 25 tiles, one fill per tick -> 26 ticks.
    CHECK(t.prime_ticks(kRadius, 1) == 26);
    CHECK(t.prime_ticks(kRadius, 4) == 8);  // ceil(25 / 4) + 1
    CHECK(t.prime_ticks(100.0, 1) == 10);   // r = 1: 3 x 3 = 9 tiles
    CHECK(t.prime_ticks(255.0, 25) == 2);   // r = 1: ceil(9 / 25) + 1
    const auto unprimed = t.check_primed(world);
    REQUIRE(!unprimed.ok());
    CHECK(unprimed.error().message == "terrain priming failed (resident tiles 0 of 25, starved 0, fill misses 0)");

    const g2m::phys::InterestPoint p = point(0, kStartX, kStartY);
    REQUIRE(open_gate(t, world, {&p, 1}));
    for (int k = 0; k < t.prime_ticks(kRadius, 1); ++k) {
        (void)t.update(world, {&p, 1});
        world.step();
    }
    const auto primed = t.check_primed(world);
    CHECK(primed.ok());
    if (!primed.ok()) UNSCOPED_INFO(primed.error().message);
    CHECK(world.terrain_resident_tile_count() >= 25);
    CHECK(world.terrain_starved_tile_count() == 0);
    CHECK(t.source()->fill_miss_count() == 0);
    // The installed source is the one the stack owns: a ray down at the start finds terrain around 150 m.
    const ps::RayCastHit hit = world.backend().ray_cast(ps::Vec3{kStartX, kStartY, 400.0}, ps::Vec3{0, 0, -1}, 800.0);
    REQUIRE(hit.hit);
    CHECK(std::abs(hit.point.z - 150.0) < 5.0);
}

TEST_CASE("ps_bridge terrain: the surface mode decides what the loader requires", "[ps_bridge][terrain]") {
    using Roads = SyntheticFetch::Roads;
    const auto gate_opens = [](std::shared_ptr<g2m::phys::IHeightTileFetch> fetch, auto make) {
        ps::World world(world_config());
        auto t = make(config_for(std::move(fetch)));
        REQUIRE(t->install(world, kRadius, 1, 0, 0).ok());
        const g2m::phys::InterestPoint p = point(0, kStartX, kStartY);
        g2m::phys::GateStatus gs;
        const bool open = open_gate(*t, world, {&p, 1}, 5.0, &gs);
        return std::make_pair(open, gs.failed);
    };
    const auto uniform = [](const PhysicsTerrain::Config& c) { return std::make_unique<PhysicsTerrain>(c, ps::SurfaceId{0}); };
    const auto road = [](const PhysicsTerrain::Config& c) {
        g2m::ps_bridge::RoadSurfaceConfig rc;
        rc.off_road = 1;
        rc.lut = g2m::RoadValueLut::by_land_class(1, {{g2m::LandClass::PavedRoad, 0}, {g2m::LandClass::UnpavedRoad, 2}});
        return std::make_unique<PhysicsTerrain>(c, rc);
    };
    const auto klass = [](const PhysicsTerrain::Config& c) {
        g2m::ps_bridge::ClassSurfaceConfig cc;
        cc.fallback = 1;
        return std::make_unique<PhysicsTerrain>(c, cc);
    };

    // A fetch that claims roads but returns none: only the uniform mode (heights only) can open the gate; the road
    // and class modes require the road list, so every tile ends Failed after its retries.
    const auto uniform_null = gate_opens(std::make_shared<SyntheticFetch>(Roads::ClaimedNull, false), uniform);
    CHECK(uniform_null.first);
    const auto road_null = gate_opens(std::make_shared<SyntheticFetch>(Roads::ClaimedNull, false), road);
    CHECK(!road_null.first);
    CHECK(road_null.second > 0);
    const auto class_null = gate_opens(std::make_shared<SyntheticFetch>(Roads::ClaimedNull, true), klass);
    CHECK(!class_null.first);

    // Roads delivered: road mode opens; class mode additionally needs the class tile.
    CHECK(gate_opens(std::make_shared<SyntheticFetch>(Roads::Empty, false), road).first);
    const auto class_without = gate_opens(std::make_shared<SyntheticFetch>(Roads::Empty, false), klass);
    CHECK(!class_without.first);
    CHECK(class_without.second > 0);
    CHECK(gate_opens(std::make_shared<SyntheticFetch>(Roads::Empty, true), klass).first);
}

TEST_CASE("ps_bridge terrain: a vanished interest point is released from the World", "[ps_bridge][terrain]") {
    // Pool of one square (49 tiles, pool_factor 1): two points 3 km apart need 2 x 25 tiles and starve; once the
    // second point is gone from the list the World must drop it, and the first square fits again.
    ps::World world(world_config());
    PhysicsTerrain t(config_for(std::make_shared<SyntheticFetch>(SyntheticFetch::Roads::None, false)), ps::SurfaceId{0});
    REQUIRE(t.install(world, kRadius, 4, 0, 0, /*pool_factor=*/1).ok());
    const g2m::phys::InterestPoint both[] = {point(0, kStartX, kStartY), point(1, kStartX + 3000.0, kStartY)};
    REQUIRE(open_gate(t, world, both));
    for (int k = 0; k < 80; ++k) {
        (void)t.update(world, both);
        world.step();
    }
    CHECK(world.terrain_starved_tile_count() > 0);

    const g2m::phys::InterestPoint one[] = {both[0]};
    REQUIRE(open_gate(t, world, one));
    for (int k = 0; k < 80; ++k) {
        (void)t.update(world, one);
        world.step();
    }
    CHECK(world.terrain_starved_tile_count() == 0);
    // check_primed is not asserted here: its fill-miss counter is cumulative and the far point's tiles were filled
    // before their heights were resident (the miss is the streamer gate doing its job), so it stays non-zero.
    CHECK(world.terrain_resident_tile_count() >= 25);
}
