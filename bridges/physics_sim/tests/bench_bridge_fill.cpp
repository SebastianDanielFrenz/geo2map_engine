// bridges/physics_sim/tests/bench_bridge_fill.cpp - bench_bridge_fill
// (PLAN.md R2.2 sections 1b/4b, G8): what one TileManager fill of a
// G2mTerrainSource physics tile costs, to set F (max_tile_fills_per_tick).
// Plain main (like physics_sim's own tests/bench), registered with CTest as a
// smoke only: it asserts correctness (no fill miss, no starved tile), never
// timing. Every number goes to stdout.
//
// Measured (synthetic DGM1 L0 tiles, never cache/; 256-sample / 255 m tiles;
// pool 49 = make_terrain_config(400, F)):
//   A. ms per G2mTerrainSource::fill_tile (single thread).
//   A-road. PLAN.md G2.5a-grip section 4/6 (commit G-c): the same fill_tile
//      measurement in road mode (RoadSurfaceConfig, fill_physics_surfaces
//      instead of a flat fill) over a resident set carrying a handful of
//      road segments per tile - the extra per-fill cost budget is ~0.25 ms.
//   B. ms per IRigidBackend::update_heightfield_body (Jolt SetHeights +
//      SetMaterials incl. the backend's row reversal/conversion) on a 256^2
//      heightfield body in a World that also holds the 49-slot pool.
//   C. weld: per-tick World::step overhead of 17 resting bodies on tile
//      seams over the tiled pool-49 ground vs the same bodies on ONE
//      1024-sample static heightfield (physics_sim's cross-tile edge weld,
//      vault PHYS-045, iterates every overlapping pool heightfield body),
//      each net of the same ground's body-free step.
//   D. end to end: step time of ticks that fill vs steady ticks, moving the
//      interest point one tile at a time, at F = 1 and F = 2.
// Decision rule (plan 1b): F = 2 only if one SetHeights(256^2) + weld stays
// under 1.0 ms (B mean + C), else F = 1.
#include "bridge_test_support.h"
#include "headless_env.h"

#include "g2m/layer/osm_roads.h"
#include "g2m/phys/fill_physics_heights.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "ps/backend/shape_desc.h"
#include "ps/world/world.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <vector>

using namespace g2m_bridge_test;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;
using g2m::phys::ResidentHeightSet;
using g2m::ps_bridge::G2mTerrainSource;
using g2m::ps_bridge::make_terrain_config;
using Clock = std::chrono::steady_clock;

namespace {

int g_failures = 0;
void expect(bool ok, const char* what) {
    if (!ok) {
        ++g_failures;
        std::printf("FAIL: %s\n", what);
    }
}

double ms_since(Clock::time_point t0) { return std::chrono::duration<double, std::milli>(Clock::now() - t0).count(); }

struct Stats {
    double mean = 0.0, p50 = 0.0, p99 = 0.0, max = 0.0;
    std::size_t n = 0;
};
Stats stats_of(std::vector<double> v) {
    Stats s;
    s.n = v.size();
    if (v.empty()) return s;
    std::sort(v.begin(), v.end());
    for (double x : v) s.mean += x;
    s.mean /= static_cast<double>(v.size());
    s.p50 = v[v.size() / 2];
    s.p99 = v[static_cast<std::size_t>(std::floor(0.99 * static_cast<double>(v.size() - 1)))];
    s.max = v.back();
    return s;
}
void print_stats(const char* name, const Stats& s) {
    std::printf("  %-52s n=%4zu mean %8.4f ms  p50 %8.4f  p99 %8.4f  max %8.4f\n", name, s.n, s.mean, s.p50, s.p99, s.max);
}

// 150 m +- 0.15 m, a 60 m x 47 m wave.
std::int32_t wavy(std::int64_t c, std::int64_t r) {
    constexpr double kTwoPi = 6.283185307179586;
    const double h = 150.0 + 0.15 * std::sin(kTwoPi * static_cast<double>(c - kE0) / 60.0) *
                                 std::cos(kTwoPi * static_cast<double>(r - kN0) / 47.0);
    return static_cast<std::int32_t>(std::lround(h * 256.0));
}

ps::terrain::TileSample make_sample(std::uint32_t n) {
    ps::terrain::TileSample s;
    s.heights.assign(static_cast<std::size_t>(n) * n, 0.0);
    s.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), ps::kInvalidSurfaceId);
    return s;
}

std::vector<ps::BodyId> add_seam_bodies(ps::World& world) {
    // 17 resting bodies on/near the x = 255.5, y = 255.5 and x = 510.5 seams.
    std::vector<ps::BodyId> ids;
    ps::BodyDesc b;
    b.motion = ps::BodyMotionType::Dynamic;
    b.allow_sleep = false;
    b.friction = 0.8;
    b.shape = ps::BoxShape{ps::Vec3{0.5, 0.4, 0.25}};
    b.mass = 50.0;
    b.pose.position = ps::Vec3{255.5, 255.5, 150.15 + 0.25 + 0.02};
    ids.push_back(world.create_body(b));
    b.shape = ps::SphereShape{0.3};
    b.mass = 5.0;
    for (int k = 0; k < 16; ++k) {
        const double x = (k < 8) ? 255.5 + 1.3 * (k - 4) : 510.5;
        const double y = (k < 8) ? 250.0 : 150.0 + 1.3 * (k - 8);
        b.pose.position = ps::Vec3{x, y + (k % 2 == 0 ? 0.0 : 5.5), 150.15 + 0.3 + 0.02};
        ids.push_back(world.create_body(b));
    }
    return ids;
}

std::vector<double> time_steps(ps::World& world, int settle, int measure) {
    for (int i = 0; i < settle; ++i) world.step();
    std::vector<double> ms;
    ms.reserve(static_cast<std::size_t>(measure));
    for (int i = 0; i < measure; ++i) {
        const auto t0 = Clock::now();
        world.step();
        ms.push_back(ms_since(t0));
    }
    return ms;
}

constexpr int kWorkers = 4;

} // namespace

int main() {
    g2m_test::install_headless_crt_handlers();
    const PhysicsTileGrid grid(test_frame());
    SyntheticTileCache cache(&wavy);

    // Data for every section: physics tiles along the D path (centres ix
    // 1..11, iy 1) with r 3 margin - covers A/B/C too.
    auto resident = std::make_shared<ResidentHeightSet>();
    for (int ix = 1; ix <= 11; ++ix) install_square(grid, *resident, PhysTileIndex{ix, 1}, 3, cache);
    std::printf("bench_bridge_fill: %zu synthetic L0 tiles resident, %d job workers\n", resident->size(), kWorkers);

    // ---- A. fill_tile -------------------------------------------------------
    auto source = std::make_shared<G2mTerrainSource>(grid, resident, ps::SurfaceId{0});
    const std::uint32_t n = source->samples_per_tile();
    ps::terrain::TileSample sample = make_sample(n);
    std::vector<double> fill_ms;
    for (int rep = 0; rep < 5; ++rep) {
        for (int iy = -2; iy <= 4; ++iy) {
            for (int ix = -2; ix <= 4; ++ix) {
                const ps::terrain::TileKey key{PhysicsTileGrid::pack(PhysTileIndex{ix, iy})};
                const auto t0 = Clock::now();
                source->fill_tile(key, sample);
                fill_ms.push_back(ms_since(t0));
            }
        }
    }
    expect(source->fill_miss_count() == 0, "A: fill_tile touched a missing L0 tile");
    const Stats a = stats_of(fill_ms);

    // ---- A-road. fill_tile, road mode (PLAN.md G2.5a-grip section 4/6, G-c) ---
    // A separate resident set (same heights, same D-path footprint) that also
    // carries roads: two segments per L0 tile (one paved, one unpaved, at
    // different angles/ranks) - representative density, not a worst case
    // (paint_road_blocks' own cost is dominated by its block partitioning,
    // not segment count - see osm_roads.cpp's for_each_block).
    auto road_resident = std::make_shared<ResidentHeightSet>();
    std::vector<g2m::RoadSegment> road_segs;
    for (int ix = 1; ix <= 11; ++ix) {
        const std::int64_t x0 = kE0 + static_cast<std::int64_t>(ix - 1) * 255;
        road_segs.push_back(g2m::RoadSegment{{x0 * 1000, (kN0 + 380) * 1000},
                                             {(x0 + 255) * 1000, (kN0 + 380) * 1000},
                                             /*half_width_mm=*/3000, g2m::LandClass::PavedRoad, g2m::SurfaceKind::Asphalt,
                                             /*rank=*/3});
        road_segs.push_back(g2m::RoadSegment{{(x0 + 60) * 1000, (kN0 + 100) * 1000},
                                             {(x0 + 60) * 1000, (kN0 + 660) * 1000},
                                             /*half_width_mm=*/1500, g2m::LandClass::UnpavedRoad, g2m::SurfaceKind::Gravel,
                                             /*rank=*/1});
    }
    auto road_segs_ptr = std::make_shared<const std::vector<g2m::RoadSegment>>(std::move(road_segs));
    for (int ix = 1; ix <= 11; ++ix) install_square_with_roads(grid, *road_resident, PhysTileIndex{ix, 1}, 3, cache, road_segs_ptr);

    const g2m::RoadValueLut road_lut = g2m::RoadValueLut::by_land_class(
        /*off_road=*/0, {{g2m::LandClass::PavedRoad, static_cast<std::uint16_t>(5)}, {g2m::LandClass::UnpavedRoad, static_cast<std::uint16_t>(9)}});
    auto road_source = std::make_shared<G2mTerrainSource>(grid, road_resident,
                                                           g2m::ps_bridge::RoadSurfaceConfig{road_lut, /*off_road=*/0});
    ps::terrain::TileSample road_sample = make_sample(n);
    std::vector<double> road_fill_ms;
    for (int rep = 0; rep < 5; ++rep) {
        for (int iy = -2; iy <= 4; ++iy) {
            for (int ix = -2; ix <= 4; ++ix) {
                const ps::terrain::TileKey key{PhysicsTileGrid::pack(PhysTileIndex{ix, iy})};
                const auto t0 = Clock::now();
                road_source->fill_tile(key, road_sample);
                road_fill_ms.push_back(ms_since(t0));
            }
        }
    }
    expect(road_source->fill_miss_count() == 0, "A-road: fill_tile touched a missing L0 tile");
    const Stats a_road = stats_of(road_fill_ms);

    // ---- B. update_heightfield_body (SetHeights) at pool 49 -------------------
    Stats b;
    {
        ps::WorldConfig cfg;
        cfg.job_workers = kWorkers;
        ps::World world(cfg);
        world.set_terrain_source(source, make_terrain_config(400.0, 1));
        world.set_terrain_interest_point(1, ps::Vec3{382.5, 382.5, 0.0}, 400.0);
        for (int i = 0; i < 30; ++i) world.step();
        expect(world.terrain_resident_tile_count() >= 25, "B: pool not primed");

        // Two real tiles' content (relative to a 128 m datum, like TileManager).
        std::vector<std::vector<ps::real>> content(2);
        for (int k = 0; k < 2; ++k) {
            source->fill_tile(ps::terrain::TileKey{PhysicsTileGrid::pack(PhysTileIndex{k, 0})}, sample);
            content[static_cast<std::size_t>(k)] = sample.heights;
            for (ps::real& h : content[static_cast<std::size_t>(k)]) h -= 128.0;
        }
        const std::vector<ps::SurfaceId> ids(static_cast<std::size_t>(n - 1) * (n - 1), ps::SurfaceId{0});
        ps::HeightfieldShape hf; // TileManager's own pool-body construction
        hf.sample_spacing = 1.0;
        hf.sample_count = n;
        hf.heights.assign(static_cast<std::size_t>(n) * n, 0.0);
        hf.heights.front() = -200.0;
        hf.heights.back() = 200.0;
        hf.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), ps::kInvalidSurfaceId);
        ps::BodyDesc d;
        d.shape = hf;
        d.motion = ps::BodyMotionType::Static;
        d.friction = 0.8;
        d.pose.position = ps::Vec3{0.0, 0.0, -1.0e6};
        const ps::BodyId body = world.create_body(d);
        for (int i = 0; i < 4; ++i) world.backend().update_heightfield_body(body, content[static_cast<std::size_t>(i % 2)], ids); // warm-up
        std::vector<double> set_ms;
        for (int i = 0; i < 200; ++i) {
            const auto t0 = Clock::now();
            world.backend().update_heightfield_body(body, content[static_cast<std::size_t>(i % 2)], ids);
            set_ms.push_back(ms_since(t0));
        }
        b = stats_of(set_ms);
        expect(world.terrain_starved_tile_count() == 0, "B: starved tile");
    }

    // ---- C. weld: tiled pool 49 vs one seamless 1024 heightfield --------------
    Stats c_tiled0, c_tiled, c_seam0, c_seam;
    {
        auto tiled_world = [&](bool bodies) {
            ps::WorldConfig cfg;
            cfg.job_workers = kWorkers;
            auto world = std::make_unique<ps::World>(cfg);
            world->set_terrain_source(source, make_terrain_config(400.0, 1));
            world->set_terrain_interest_point(1, ps::Vec3{382.5, 382.5, 0.0}, 400.0);
            for (int i = 0; i < 30; ++i) world->step();
            if (bodies) add_seam_bodies(*world);
            return world;
        };
        auto seamless_world = [&](bool bodies) {
            ps::WorldConfig cfg;
            cfg.job_workers = kWorkers;
            auto world = std::make_unique<ps::World>(cfg);
            const PhysicsTileGrid grid1024(test_frame(), 1024);
            ps::HeightfieldShape hf;
            hf.sample_spacing = 1.0;
            hf.sample_count = 1024;
            hf.heights.assign(static_cast<std::size_t>(1024) * 1024, 0.0);
            const g2m::phys::FillResult fr = g2m::phys::fill_physics_heights<double>(
                grid1024, PhysTileIndex{0, 0}, *resident, hf.heights.data(), ps::kHeightfieldNoCollision);
            expect(fr == g2m::phys::FillResult::Ok, "C: seamless fill");
            for (ps::real& h : hf.heights) h -= 128.0;
            hf.cell_surface_ids.assign(static_cast<std::size_t>(1023) * 1023, ps::SurfaceId{0});
            ps::BodyDesc d;
            d.shape = std::move(hf);
            d.motion = ps::BodyMotionType::Static;
            d.friction = 0.8;
            d.pose.position = ps::Vec3{0.5, 0.5, 128.0};
            world->create_body(d);
            if (bodies) add_seam_bodies(*world);
            return world;
        };
        {
            auto w = tiled_world(false);
            c_tiled0 = stats_of(time_steps(*w, 120, 960));
        }
        {
            auto w = tiled_world(true);
            c_tiled = stats_of(time_steps(*w, 120, 960));
            expect(w->terrain_starved_tile_count() == 0, "C: starved tile");
        }
        {
            auto w = seamless_world(false);
            c_seam0 = stats_of(time_steps(*w, 120, 960));
        }
        {
            auto w = seamless_world(true);
            c_seam = stats_of(time_steps(*w, 120, 960));
        }
    }
    const double weld_ms = (c_tiled.mean - c_tiled0.mean) - (c_seam.mean - c_seam0.mean);

    // ---- D. end to end: fill ticks vs steady ticks, F = 1 and F = 2 ----------
    struct EndToEnd {
        Stats fill_ticks, steady_ticks;
        std::size_t fills = 0;
    } e2e[2];
    for (int fi = 0; fi < 2; ++fi) {
        const std::uint32_t f = static_cast<std::uint32_t>(fi + 1);
        ps::WorldConfig cfg;
        cfg.job_workers = kWorkers;
        ps::World world(cfg);
        auto src = std::make_shared<G2mTerrainSource>(grid, resident, ps::SurfaceId{0});
        world.set_terrain_source(src, make_terrain_config(400.0, f));
        double x = 382.5;
        world.set_terrain_interest_point(1, ps::Vec3{x, 382.5, 0.0}, 400.0);
        for (int i = 0; i < 30; ++i) world.step();
        std::vector<double> fill_t, steady_t;
        const int fill_ticks_per_move = static_cast<int>((5 + f - 1) / f); // 5 new keys per one-tile move
        for (int move = 0; move < 10; ++move) {
            x += 255.0;
            world.set_terrain_interest_point(1, ps::Vec3{x, 382.5, 0.0}, 400.0);
            for (int t = 0; t < 10; ++t) {
                const auto t0 = Clock::now();
                world.step();
                (t < fill_ticks_per_move ? fill_t : steady_t).push_back(ms_since(t0));
            }
        }
        e2e[fi].fill_ticks = stats_of(fill_t);
        e2e[fi].steady_ticks = stats_of(steady_t);
        e2e[fi].fills = 50;
        expect(src->fill_miss_count() == 0, "D: fill miss");
        expect(world.terrain_starved_tile_count() == 0, "D: starved tile");
    }

    std::printf("A. G2mTerrainSource::fill_tile (256^2, single thread)\n");
    print_stats("fill_tile", a);
    std::printf("A-road. G2mTerrainSource::fill_tile, road mode (256^2, single thread)\n");
    print_stats("fill_tile (road)", a_road);
    std::printf("  road-mode extra cost per fill: mean %.4f ms %s budget 0.25 ms\n", a_road.mean - a.mean,
                (a_road.mean - a.mean) < 0.25 ? "<" : ">=");
    std::printf("B. update_heightfield_body 256^2 (SetHeights+SetMaterials), World with pool 49\n");
    print_stats("update_heightfield_body", b);
    std::printf("C. World::step, 17 resting bodies on seams vs none (weld overhead)\n");
    print_stats("tiled pool 49, no bodies", c_tiled0);
    print_stats("tiled pool 49, 17 bodies", c_tiled);
    print_stats("seamless 1024^2, no bodies", c_seam0);
    print_stats("seamless 1024^2, 17 bodies", c_seam);
    std::printf("  weld overhead per tick (tiled bodies cost - seamless bodies cost): %.4f ms\n", weld_ms);
    for (int fi = 0; fi < 2; ++fi) {
        std::printf("D. end to end, F = %d: 10 one-tile moves (50 fills)\n", fi + 1);
        print_stats("ticks with fills", e2e[fi].fill_ticks);
        print_stats("steady ticks", e2e[fi].steady_ticks);
        std::printf("  incremental cost of a fill tick: mean %.4f ms, worst %.4f ms\n",
                    e2e[fi].fill_ticks.mean - e2e[fi].steady_ticks.mean, e2e[fi].fill_ticks.max - e2e[fi].steady_ticks.p50);
    }
    const double one_fill_budget = b.mean + std::max(0.0, weld_ms);
    const int f_decision = (one_fill_budget < 1.0) ? 2 : 1;
    std::printf("decision (plan 1b): SetHeights mean %.4f ms + weld %.4f ms = %.4f ms %s 1.0 ms -> F = %d\n", b.mean,
                std::max(0.0, weld_ms), one_fill_budget, one_fill_budget < 1.0 ? "<" : ">=", f_decision);
    std::printf("bench_bridge_fill: %s\n", g_failures == 0 ? "OK" : "FAILED");
    return g_failures == 0 ? 0 : 1;
}
