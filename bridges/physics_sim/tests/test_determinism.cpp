// bridges/physics_sim/tests/test_determinism.cpp - determinism acceptance
// (PLAN.md R2.2 section 4b, G7): 16 spheres and a box pushed across physics
// tile seams on G2mTerrainSource terrain for 2400 ticks. The per-tick
// World::state_hash sequence (which also folds every TileManager pool
// heightfield body's pose, i.e. which slot got which tile at which datum)
// must be identical
//   - at 1, 4 and 10 job workers, and
//   - between "every L0 tile installed up front" and "gated delayed
//     installs": a PhysicsTerrainStreamer + HeightTileLoader over a fetch
//     that releases one tile per 3 frozen attempts, behind a minimal
//     test-local version of racing_game's gate (plan section 2,
//     Session::try_step): build the interest point from the World's own box
//     pose/velocity, streamer.update(), set_terrain_interest_point, and
//     withhold World::step while the gate is not ready.
// This is plan section 2's determinism argument made executable: residency
// only changes WHEN a tick runs, never WHAT it computes.
// fill_miss_count() == 0 is the runtime witness.
#include "bridge_test_support.h"

#include "g2m/phys/height_tile_loader.h"
#include "g2m/phys/physics_streamer.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "ps/backend/shape_desc.h"
#include "ps/world/world.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

using namespace g2m_bridge_test;
using g2m::TileKey;
using g2m::phys::GateStatus;
using g2m::phys::HeightTileLoader;
using g2m::phys::InterestPoint;
using g2m::phys::PhysicsTerrainStreamer;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;
using g2m::phys::ResidentHeightSet;
using g2m::ps_bridge::G2mTerrainSource;
using g2m::ps_bridge::make_terrain_config;

namespace {

constexpr double kRadius = 400.0;      // plan section 2's default: pool 49, TileManager square 5x5
constexpr int kTicks = 2400;           // stepped ticks per run (10 s at 240 Hz)
constexpr int kPrimeTicks = 26;        // K = ceil(25 / F) + 1 at F = 1 (plan section 2 "Prime")
constexpr int kSettleTicks = 60;
constexpr double kStartX = 230.0, kStartY = 240.0; // physics tile (0, 0); seams at x/y = 255.5, 510.5

// 150 m +- 0.15 m, a 60 m x 47 m wave (never aligned with the 255 m tiles).
std::int32_t wavy(std::int64_t c, std::int64_t r) {
    constexpr double kTwoPi = 6.283185307179586;
    const double h = 150.0 + 0.15 * std::sin(kTwoPi * static_cast<double>(c - kE0) / 60.0) *
                                 std::cos(kTwoPi * static_cast<double>(r - kN0) / 47.0);
    return static_cast<std::int32_t>(std::lround(h * 256.0));
}

SyntheticTileCache& wavy_cache() {
    static SyntheticTileCache cache(&wavy);
    return cache;
}

// Serves synthetic tiles, but only one per grant(): fetch() blocks until a
// token is available (or open_all()). Thread-safe (every loader worker calls
// fetch concurrently). No Catch2 macro in here - it runs on loader workers
// (vault TOOL-031).
class TokenFetch final : public g2m::phys::IHeightTileFetch {
public:
    explicit TokenFetch(SyntheticTileCache& cache) : cache_(cache) {}

    g2m::phys::FetchResult fetch(const TileKey& key) override {
        {
            std::unique_lock<std::mutex> lk(m_);
            cv_.wait(lk, [&] { return open_ || tokens_ > 0; });
            if (!open_) --tokens_;
            ++served_;
        }
        return g2m::phys::FetchResult{g2m::Status::Ok, cache_.get(key), nullptr};
    }
    void grant(int n) {
        {
            std::lock_guard<std::mutex> lk(m_);
            tokens_ += n;
        }
        cv_.notify_all();
    }
    void open_all() {
        {
            std::lock_guard<std::mutex> lk(m_);
            open_ = true;
        }
        cv_.notify_all();
    }
    std::uint64_t served() const {
        std::lock_guard<std::mutex> lk(m_);
        return served_;
    }

private:
    SyntheticTileCache& cache_;
    mutable std::mutex m_;
    std::condition_variable cv_;
    int tokens_ = 0;
    bool open_ = false;
    std::uint64_t served_ = 0;
};

// Unblocks every fetch before the loader's destructor joins its workers
// (declared after the loader, so it is destroyed first - also on a REQUIRE
// unwinding the run).
struct OpenOnExit {
    TokenFetch* fetch = nullptr;
    ~OpenOnExit() {
        if (fetch != nullptr) fetch->open_all();
    }
};

void spin_for(std::chrono::microseconds d) {
    const auto until = std::chrono::steady_clock::now() + d;
    while (std::chrono::steady_clock::now() < until) std::this_thread::yield();
}

enum class Mode { UpFront, Gated };

struct RunResult {
    std::vector<std::uint64_t> hashes; // one per stepped tick
    std::uint64_t fill_miss = 0;
    std::size_t max_starved = 0;
    int frozen_attempts = 0;
    int freezes_after_spawn = 0; // frozen episodes (maximal runs of frozen attempts) once bodies exist
    std::uint64_t tiles_served = 0;
    PhysTileIndex box_start{}, box_end{};
    double min_body_z = 1e9;
    double box_end_x = 0.0, box_end_y = 0.0;
};

RunResult run(int workers, Mode mode) {
    RunResult out;
    const PhysicsTileGrid grid(test_frame());
    auto resident = std::make_shared<ResidentHeightSet>();

    std::shared_ptr<TokenFetch> fetch;
    std::unique_ptr<HeightTileLoader> loader;
    std::unique_ptr<PhysicsTerrainStreamer> streamer;
    OpenOnExit open_guard;
    if (mode == Mode::UpFront) {
        // Radius 5 around the start tile: covers every TileManager fill the
        // run can make (the box ends in physics tile ix = 2 or 3).
        install_square(grid, *resident, grid.index_for_local(kStartX, kStartY), 5, wavy_cache());
    } else {
        fetch = std::make_shared<TokenFetch>(wavy_cache());
        loader = std::make_unique<HeightTileLoader>(fetch, g2m::phys::LoaderConfig{});
        streamer = std::make_unique<PhysicsTerrainStreamer>(grid, *resident, *loader, g2m::phys::StreamerConfig{});
    }
    open_guard.fetch = fetch.get();

    auto source = std::make_shared<G2mTerrainSource>(grid, resident, ps::SurfaceId{0});
    ps::WorldConfig cfg;
    cfg.tick_rate_hz = 240.0;
    cfg.job_workers = workers;
    ps::World world(cfg);
    world.set_terrain_source(source, make_terrain_config(kRadius, 1));

    bool have_bodies = false;
    ps::BodyId box{};
    std::vector<ps::BodyId> spheres;

    // The gate (plan section 2, Session::try_step, minimal): the interest
    // point comes from sim state only - the start point until the box
    // exists, then the box's own pose/velocity read back from the World.
    auto try_step = [&]() -> bool {
        ps::Vec3 p{kStartX, kStartY, 0.0}, v{};
        if (have_bodies) {
            p = world.get_pose(box).position;
            v = world.get_motion(box).linear;
        }
        bool ready = true;
        if (streamer) {
            const InterestPoint ip{0, p.x, p.y, v.x, v.y, kRadius};
            const GateStatus status = streamer->update({&ip, 1});
            ready = status.ready;
        }
        world.set_terrain_interest_point(0, ps::Vec3{p.x, p.y, 0.0}, kRadius);
        if (!ready) return false; // frozen: the World is untouched
        world.step();
        return true;
    };

    auto spawn = [&]() {
        ps::BodyDesc b;
        b.motion = ps::BodyMotionType::Dynamic;
        b.allow_sleep = false;
        b.restitution = 0.0;
        b.friction = 0.0; // sliding, as in physics_sim's test_terrain_seam_crossing.cpp
        b.max_angular_velocity = 1000.0;
        b.shape = ps::BoxShape{ps::Vec3{0.5, 0.4, 0.25}};
        b.mass = 50.0;
        b.pose.position = ps::Vec3{kStartX, kStartY, 150.15 + 0.25 + 0.02};
        box = world.create_body(b);
        b.shape = ps::SphereShape{0.3};
        b.mass = 5.0;
        for (int j = 0; j < 4; ++j) {
            for (int i = 0; i < 4; ++i) {
                b.pose.position = ps::Vec3{kStartX - 16.0 + 1.6 * i, kStartY - 8.0 + 1.6 * j, 150.15 + 0.3 + 0.02};
                spheres.push_back(world.create_body(b));
            }
        }
        have_bodies = true;
        out.box_start = grid.index_for_local(kStartX, kStartY);
    };

    auto push = [&]() {
        ps::Motion m = world.get_motion(box);
        m.linear = ps::Vec3{34.0, 9.0, m.linear.z};
        world.backend().set_motion(box, m);
        for (int k = 0; k < 16; ++k) {
            const int i = k % 4, j = k / 4;
            ps::Motion s = world.get_motion(spheres[static_cast<std::size_t>(k)]);
            s.linear = ps::Vec3{24.0 + 2.5 * i + 0.5 * j, -6.0 + 5.0 * j + 0.25 * i, s.linear.z};
            world.backend().set_motion(spheres[static_cast<std::size_t>(k)], s);
        }
    };

    out.hashes.reserve(kTicks);
    bool in_freeze = false;
    int stepped = 0;
    while (stepped < kTicks) {
        // Bodies appear and get pushed between the same two stepped ticks in
        // every mode (keyed on `stepped`, never on wall time or attempts).
        if (stepped == kPrimeTicks && !have_bodies) spawn();
        if (try_step()) {
            ++stepped;
            out.hashes.push_back(world.state_hash());
            out.max_starved = std::max(out.max_starved, world.terrain_starved_tile_count());
            in_freeze = false;
            if (stepped == kPrimeTicks + kSettleTicks) push();
        } else {
            ++out.frozen_attempts;
            if (!in_freeze) {
                in_freeze = true;
                if (have_bodies) ++out.freezes_after_spawn;
            }
            if (out.frozen_attempts % 3 == 0) fetch->grant(1); // one tile per 3 frozen attempts
            spin_for(std::chrono::microseconds(150));
            if (out.frozen_attempts > 200000) break; // reported by the caller (hashes.size() < kTicks)
        }
    }

    out.fill_miss = source->fill_miss_count();
    if (fetch) out.tiles_served = fetch->served();
    if (have_bodies) {
        const ps::Vec3 bp = world.get_pose(box).position;
        out.box_end_x = bp.x;
        out.box_end_y = bp.y;
        out.box_end = grid.index_for_local(bp.x, bp.y);
        out.min_body_z = bp.z;
        for (const ps::BodyId s : spheres) out.min_body_z = std::min(out.min_body_z, world.get_pose(s).position.z);
    }
    return out;
}

// Index of the first differing tick, or -1 if the sequences are identical.
long first_mismatch(const std::vector<std::uint64_t>& a, const std::vector<std::uint64_t>& b) {
    const std::size_t n = std::min(a.size(), b.size());
    for (std::size_t i = 0; i < n; ++i) {
        if (a[i] != b[i]) return static_cast<long>(i);
    }
    return a.size() == b.size() ? -1 : static_cast<long>(n);
}

void report(const char* name, const RunResult& r) {
    std::printf("determinism %-10s: ticks %zu, final hash %016llx, fill_miss %llu, max starved %zu, frozen attempts %d, "
                "freezes after spawn %d, tiles served %llu, box (%.1f, %.1f) tile (%d,%d) -> (%d,%d), min body z %.3f\n",
                name, r.hashes.size(), static_cast<unsigned long long>(r.hashes.empty() ? 0 : r.hashes.back()),
                static_cast<unsigned long long>(r.fill_miss), r.max_starved, r.frozen_attempts, r.freezes_after_spawn,
                static_cast<unsigned long long>(r.tiles_served), r.box_end_x, r.box_end_y, r.box_start.ix, r.box_start.iy,
                r.box_end.ix, r.box_end.iy, r.min_body_z);
}

void check_run(const RunResult& r) {
    CHECK(r.hashes.size() == static_cast<std::size_t>(kTicks));
    CHECK(r.fill_miss == 0);
    CHECK(r.max_starved == 0);
    CHECK(r.min_body_z > 149.0);        // nothing fell through the terrain
    CHECK(r.box_end.ix >= r.box_start.ix + 2); // crossed x = 255.5 and x = 510.5
    CHECK(r.box_end.iy >= r.box_start.iy + 1); // crossed y = 255.5
}

} // namespace

TEST_CASE("determinism: per-tick state_hash identical at 1/4/10 workers and up-front vs gated delayed installs",
         "[bridge][terrain][determinism]") {
    const auto t0 = std::chrono::steady_clock::now();
    const RunResult up1 = run(1, Mode::UpFront);
    const RunResult up4 = run(4, Mode::UpFront);
    const RunResult up10 = run(10, Mode::UpFront);
    const RunResult gated4 = run(4, Mode::Gated);
    const RunResult gated10 = run(10, Mode::Gated);
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    report("upfront/1", up1);
    report("upfront/4", up4);
    report("upfront/10", up10);
    report("gated/4", gated4);
    report("gated/10", gated10);
    std::printf("determinism: 5 runs in %.2f s; first mismatch vs upfront/1: up4 %ld, up10 %ld, gated4 %ld, gated10 %ld\n",
                secs, first_mismatch(up1.hashes, up4.hashes), first_mismatch(up1.hashes, up10.hashes),
                first_mismatch(up1.hashes, gated4.hashes), first_mismatch(up1.hashes, gated10.hashes));

    check_run(up1);
    check_run(up4);
    check_run(up10);
    check_run(gated4);
    check_run(gated10);
    CHECK(first_mismatch(up1.hashes, up4.hashes) == -1);
    CHECK(first_mismatch(up1.hashes, up10.hashes) == -1);
    CHECK(first_mismatch(up1.hashes, gated4.hashes) == -1);
    CHECK(first_mismatch(up1.hashes, gated10.hashes) == -1);
    // The gate really did withhold ticks, also mid-run as the box crossed
    // into tiles whose data had not arrived yet.
    CHECK(gated4.freezes_after_spawn >= 1);
    CHECK(gated10.freezes_after_spawn >= 1);
    CHECK(up1.frozen_attempts == 0);
}
