// bridges/physics_sim/tests/test_seam_crossing.cpp - seam vs seamless
// (PLAN.md R2.2 section 4b, G8): the same synthetic DGM1 field as
//   (a) G2mTerrainSource's 256-sample / 255 m physics tiles streamed by a
//       real World's TileManager (make_terrain_config(255, 1)) - separate
//       Jolt bodies, joined only by physics_sim's cross-tile edge weld
//       (vault PHYS-045), and
//   (b) ONE 1024-sample (1023 m) static heightfield body over the same
//       sample lattice (a test-local PhysicsTileGrid(frame, 1024) filled by
//       the same g2m::phys::fill_physics_heights, datum applied exactly as
//       TileManager does),
// with a sliding sphere and a sliding box crossing a seam at 5/20/40 m/s.
// Approach, body set-up and bounds are physics_sim's own
// tests/validation/test_terrain_seam_crossing.cpp (pinned 5c648a5): peak
// |az| over the run after the launch ticks, tiled <= 2*|seamless| + 30 m/s^2
// and <= 120 m/s^2. Two paths: along +x across the x = 255.5 seam, and
// diagonally through the 4-tile corner (255.5, 255.5) - an addition over
// physics_sim's test, whose absolute cap needs adapting there (abs_bound).
#include "bridge_test_support.h"

#include "g2m/phys/fill_physics_heights.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "ps/backend/shape_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

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

namespace {

constexpr double kBase = 100.0; // m; datum 128 for both grounds
constexpr double kStreamRadius = 255.0;
constexpr std::uint64_t kInterestId = 1;

std::int32_t flat_field(std::int64_t, std::int64_t) { return static_cast<std::int32_t>(kBase * 256.0); }
// 0.15 m amplitude, a 60 m x 47 m wave, rounded to the 1/256 m DGM1 grid.
std::int32_t wavy_field(std::int64_t c, std::int64_t r) {
    constexpr double kTwoPi = 6.283185307179586;
    const double h = kBase + 0.15 * std::sin(kTwoPi * static_cast<double>(c - kE0) / 60.0) *
                                 std::cos(kTwoPi * static_cast<double>(r - kN0) / 47.0);
    return static_cast<std::int32_t>(std::lround(h * 256.0));
}

enum class Shape { Flat, Wavy };
enum class Kind { SlidingSphere, SlidingBox };
enum class Ground { Tiles, Seamless };
const char* name_of(Shape s) { return s == Shape::Flat ? "flat" : "wavy"; }
const char* name_of(Kind k) { return k == Kind::SlidingSphere ? "sliding_sphere" : "sliding_box"; }

SyntheticTileCache& cache_for(Shape s) {
    static SyntheticTileCache flat(&flat_field);
    static SyntheticTileCache wavy(&wavy_field);
    return s == Shape::Flat ? flat : wavy;
}

// Exact height of the terrain at a sample point (x, y on the lattice
// 0.5 + k); between samples only used for the spawn height (+ a margin).
double field_height(Shape s, double x, double y) {
    const std::int64_t c = kE0 + static_cast<std::int64_t>(std::floor(x));
    const std::int64_t r = kN0 + static_cast<std::int64_t>(std::floor(y));
    return (s == Shape::Flat ? flat_field(c, r) : wavy_field(c, r)) / 256.0;
}

// The data both grounds are built from: every L0 tile under physics tiles
// -1..4 (covers the whole 1024-sample tile (0, 0) and every TileManager fill
// of the runs below).
std::shared_ptr<ResidentHeightSet> resident_for(Shape s) {
    auto resident = std::make_shared<ResidentHeightSet>();
    install_square(PhysicsTileGrid(test_frame()), *resident, PhysTileIndex{2, 2}, 3, cache_for(s));
    return resident;
}

// One 1024 x 1024 heightfield body over physics-grid-1024 tile (0, 0):
// local x, y in [0.5, 1023.5] - the same DGM1 sample lattice as the 255 m
// tiles. Heights relative to TileManager's own datum rule (mid-height
// snapped to the nearest 128 m).
struct SeamlessData {
    std::vector<double> heights;
    double datum = 0.0;
};
const SeamlessData& seamless_for(Shape s) {
    static SeamlessData data[2];
    SeamlessData& d = data[s == Shape::Flat ? 0 : 1];
    if (d.heights.empty()) {
        const PhysicsTileGrid grid1024(test_frame(), 1024);
        const std::shared_ptr<ResidentHeightSet> resident = resident_for(s);
        d.heights.assign(static_cast<std::size_t>(1024) * 1024, 0.0);
        const g2m::phys::FillResult fr =
            g2m::phys::fill_physics_heights<double>(grid1024, PhysTileIndex{0, 0}, *resident, d.heights.data(), ps::kHeightfieldNoCollision);
        REQUIRE(fr == g2m::phys::FillResult::Ok);
        const auto [mn, mx] = std::minmax_element(d.heights.begin(), d.heights.end());
        d.datum = std::round(((*mn + *mx) * 0.5) / 128.0) * 128.0;
        for (double& h : d.heights) h -= d.datum;
    }
    return d;
}

void install_ground(ps::World& world, Ground g, Shape s, const ps::Vec3& start) {
    if (g == Ground::Tiles) {
        auto source = std::make_shared<G2mTerrainSource>(PhysicsTileGrid(test_frame()), resident_for(s), ps::SurfaceId{0});
        world.set_terrain_source(source, make_terrain_config(kStreamRadius, 1));
        world.set_terrain_interest_point(kInterestId, start, kStreamRadius);
        for (int i = 0; i < 10; ++i) world.step(); // prime: K = ceil(9 / F) + 1
        REQUIRE(world.terrain_resident_tile_count() >= 9);
        REQUIRE(world.terrain_starved_tile_count() == 0);
        REQUIRE(source->fill_miss_count() == 0);
    } else {
        const SeamlessData& d = seamless_for(s);
        ps::HeightfieldShape hf;
        hf.sample_spacing = 1.0;
        hf.sample_count = 1024;
        hf.heights = d.heights;
        hf.cell_surface_ids.assign(static_cast<std::size_t>(1023) * 1023, ps::SurfaceId{0});
        ps::BodyDesc b;
        b.shape = std::move(hf);
        b.motion = ps::BodyMotionType::Static;
        b.friction = 0.8; // TileManager's own pool-body friction
        b.pose.position = ps::Vec3{0.5, 0.5, d.datum};
        world.create_body(b);
    }
}

struct BodyRun {
    double peak_az = 0.0;
    double peak_az_x = 0.0, peak_az_y = 0.0;
};

// physics_sim's run_body, generalised to a travel direction (dx, dy).
BodyRun run_body(Ground g, Shape s, Kind kind, double speed, double x0, double y0, double dx, double dy, double distance) {
    ps::WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.job_workers = 4;
    ps::World world(cfg);
    install_ground(world, g, s, ps::Vec3{x0, y0, 0.0});

    ps::BodyDesc b;
    b.motion = ps::BodyMotionType::Dynamic;
    b.allow_sleep = false;
    b.restitution = 0.0;
    b.max_angular_velocity = 1000.0;
    b.friction = 0.0;
    double rest_h = 0.3;
    if (kind == Kind::SlidingSphere) {
        b.shape = ps::SphereShape{0.3};
        b.mass = 5.0;
    } else {
        b.shape = ps::BoxShape{ps::Vec3{0.5, 0.4, 0.25}};
        b.mass = 50.0;
        rest_h = 0.25;
    }
    b.pose.position = ps::Vec3{x0, y0, field_height(s, x0, y0) + 0.15 + rest_h + 0.005};
    const ps::BodyId id = world.create_body(b);

    auto follow = [&] {
        if (g == Ground::Tiles) world.set_terrain_interest_point(kInterestId, world.get_pose(id).position, kStreamRadius);
    };
    for (int i = 0; i < 120; ++i) {
        follow();
        world.step();
    }
    {
        ps::Motion m = world.get_motion(id);
        m.linear = ps::Vec3{speed * dx, speed * dy, 0.0};
        world.backend().set_motion(id, m);
    }
    BodyRun r;
    ps::Motion prev = world.get_motion(id);
    const std::uint64_t n = static_cast<std::uint64_t>(distance / speed * cfg.tick_rate_hz);
    for (std::uint64_t t = 0; t < n; ++t) {
        follow();
        world.step();
        const ps::Motion m = world.get_motion(id);
        const double az = (m.linear.z - prev.linear.z) * cfg.tick_rate_hz;
        prev = m;
        if (t < 2) continue; // launch ticks
        if (std::abs(az) > std::abs(r.peak_az)) {
            const ps::Vec3 p = world.get_pose(id).position;
            r.peak_az = az;
            r.peak_az_x = p.x;
            r.peak_az_y = p.y;
        }
    }
    if (g == Ground::Tiles) REQUIRE(world.terrain_starved_tile_count() == 0);
    const ps::Vec3 end = world.get_pose(id).position;
    // The run really crossed its seam(s) and stayed on the ground.
    REQUIRE(end.z > kBase - 1.0);
    if (dx > 0.0) REQUIRE(end.x > 255.5 + 5.0);
    if (dy > 0.0) REQUIRE(end.y > 255.5 + 5.0);
    return r;
}

double body_seam_bound(double seamless_peak_az) { return 2.0 * std::abs(seamless_peak_az) + 30.0; }
constexpr double kBodyAbsBound = 120.0;
// The absolute 120 m/s^2 cap is physics_sim's, calibrated on its own +x
// paths - applied literally on the +x path here. On the diagonal corner
// path (not in physics_sim's test) the SEAMLESS reference itself exceeds it
// on the wavy field at 40 m/s (a 0.3 m sphere at 40 m/s over 1 m cells of
// 1/256 m-quantised heights; measured ~175 m/s^2), so there the cap
// becomes max(120, |seamless| + 30): still far tighter than the relative
// 2*|seamless| + 30 bound, and still a pure "no seam spike" statement.
double abs_bound(bool corner_path, double seamless_peak_az) {
    return corner_path ? std::max(kBodyAbsBound, std::abs(seamless_peak_az) + 30.0) : kBodyAbsBound;
}

} // namespace

TEST_CASE("seam vs seamless: bodies cross 255 m physics-tile seams without a contact spike",
         "[bridge][terrain][seam]") {
    const auto t0 = std::chrono::steady_clock::now();
    struct Path {
        const char* name;
        double x0, y0, dx, dy;
        bool corner;
    };
    constexpr double kInvSqrt2 = 0.7071067811865476;
    const Path paths[] = {{"x", 247.5, 100.5, 1.0, 0.0, false}, {"corner", 249.5, 249.5, kInvSqrt2, kInvSqrt2, true}};
    for (const Path& path : paths) {
        for (Shape shape : {Shape::Flat, Shape::Wavy}) {
            for (Kind kind : {Kind::SlidingSphere, Kind::SlidingBox}) {
                for (double v : {5.0, 20.0, 40.0}) {
                    const double dist = std::max(40.0, 2.0 * v);
                    const BodyRun tiles = run_body(Ground::Tiles, shape, kind, v, path.x0, path.y0, path.dx, path.dy, dist);
                    const BodyRun seamless =
                        run_body(Ground::Seamless, shape, kind, v, path.x0, path.y0, path.dx, path.dy, dist);
                    std::printf("seam %-6s %-4s %-14s v=%4.0f: peak |az| tiled %7.3f at (%.2f, %.2f), seamless %7.3f "
                                "(bound %.3f)\n",
                                path.name, name_of(shape), name_of(kind), v, std::abs(tiles.peak_az), tiles.peak_az_x,
                                tiles.peak_az_y, std::abs(seamless.peak_az), std::min(body_seam_bound(seamless.peak_az), abs_bound(path.corner, seamless.peak_az)));
                    CAPTURE(path.name, name_of(shape), name_of(kind), v, tiles.peak_az, tiles.peak_az_x, seamless.peak_az);
                    CHECK(std::abs(tiles.peak_az) <= body_seam_bound(seamless.peak_az));
                    CHECK(std::abs(tiles.peak_az) <= abs_bound(path.corner, seamless.peak_az));
                }
            }
        }
    }
    std::printf("seam vs seamless: %.2f s\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count());
}
