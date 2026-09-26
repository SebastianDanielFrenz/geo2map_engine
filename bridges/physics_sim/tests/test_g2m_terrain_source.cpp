// bridges/physics_sim/tests/test_g2m_terrain_source.cpp - G2mTerrainSource
// (PLAN.md R2.2 section 1b/4b), bridge-release leg only. Covers exactly the
// two 4b bullets this bridge scaffolding is scoped to: "contract" and
// "fill_tile zero-alloc". The render-vs-Jolt/determinism/seam bullets need
// racing_game's rendering stack and are out of scope here.
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "g2m/core/geo/session_frame.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"

#include "ps/jobs/debug_alloc.h"
#include "ps/jobs/job_system.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>
#include <memory>
#include <set>
#include <vector>

using g2m::TileKey;
using g2m::geo::Hemisphere;
using g2m::geo::SessionFrame;
using g2m::geo::UtmZone;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;
using g2m::phys::ResidentHeightSet;
using g2m::ps_bridge::G2mTerrainSource;
using g2m::ps_bridge::make_terrain_config;

namespace {
constexpr UtmZone k32N{32, Hemisphere::North};

std::shared_ptr<g2m::HeightTile> make_tile(const TileKey& key) {
    auto tile = std::make_shared<g2m::HeightTile>();
    tile->key = key;
    return tile;
}

// Installs every L0 tile any physics tile in the (2r+1)x(2r+1) square
// centred on `centre` can touch, so fill_tile never takes the (still
// allocation-free, but otherwise-untested-here) Missing branch.
void install_square(PhysicsTileGrid& grid, ResidentHeightSet& resident, PhysTileIndex centre, int r) {
    std::vector<TileKey> keys;
    grid.l0_keys_for_square(centre, r, keys);
    for (const auto& key : keys) {
        resident.install(key, make_tile(key));
    }
}
} // namespace

TEST_CASE("G2mTerrainSource: contract - samples/tile_size/key_offset/origin round-trip, fill_tile never resizes",
         "[bridge][terrain]") {
    SessionFrame frame(k32N, 474000, 5560000);
    PhysicsTileGrid grid(frame); // default kPhysSamples = 256
    auto resident = std::make_shared<ResidentHeightSet>();
    install_square(grid, *resident, PhysTileIndex{0, 0}, 1);

    G2mTerrainSource source(grid, resident, /*surface=*/ps::SurfaceId{0});

    const std::uint32_t n = source.samples_per_tile();
    CHECK(n == 256);
    CHECK(n % 2 == 0); // Jolt's own HeightFieldShape sample-count constraint
    CHECK(source.tile_size_m() == 255.0);

    const ps::terrain::TileKey key0 = source.key_for_world_xy(10.0, 10.0);
    const ps::terrain::TileKey east = source.key_offset(key0, 1, 0);
    const ps::terrain::TileKey north = source.key_offset(key0, 0, 1);
    const ps::terrain::TileKey back = source.key_offset(source.key_offset(key0, 3, -2), -3, 2);
    CHECK(back.packed == key0.packed);

    const ps::Vec3 origin0 = source.tile_origin_xy(key0);
    const ps::Vec3 origin_east = source.tile_origin_xy(east);
    const ps::Vec3 origin_north = source.tile_origin_xy(north);
    CHECK(origin_east.x == origin0.x + source.tile_size_m());
    CHECK(origin_east.y == origin0.y);
    CHECK(origin_north.x == origin0.x);
    CHECK(origin_north.y == origin0.y + source.tile_size_m());

    // out's vectors are pre-sized by the caller exactly like TileManager
    // sizes them (samples_per_tile()^2 / (samples_per_tile()-1)^2) - the
    // contract fill_tile must never violate.
    ps::terrain::TileSample out;
    out.heights.assign(static_cast<std::size_t>(n) * n, -1.0);
    out.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), ps::kInvalidSurfaceId);
    const std::size_t heights_capacity = out.heights.capacity();
    const std::size_t surfaces_capacity = out.cell_surface_ids.capacity();

    source.fill_tile(key0, out);

    CHECK(out.heights.size() == static_cast<std::size_t>(n) * n);
    CHECK(out.cell_surface_ids.size() == static_cast<std::size_t>(n - 1) * (n - 1));
    // Never resized - not merely "same size after" (which a resize-to-same
    // vector could also produce), but the exact same allocation.
    CHECK(out.heights.capacity() == heights_capacity);
    CHECK(out.cell_surface_ids.capacity() == surfaces_capacity);
    CHECK(out.samples_per_tile == n);
    CHECK(out.tile_size_m == source.tile_size_m());
}

TEST_CASE("G2mTerrainSource: make_terrain_config - suppression off, pool sized per PLAN.md's worked example",
         "[bridge][terrain]") {
    const ps::terrain::TerrainConfig cfg = make_terrain_config(400.0, 1);
    CHECK_FALSE(cfg.suppress_terrain_under_static_geometry);
    CHECK_FALSE(cfg.precompute_suppression_at_load);
    CHECK(cfg.max_resident_tiles == 49); // (2*ceil(400/255)+3)^2 = (2*2+3)^2 = 49
    CHECK(cfg.max_tile_fills_per_tick == 1);
}

TEST_CASE("G2mTerrainSource: fill_tile is allocation-free across concurrent jobs", "[bridge][terrain][alloc]") {
    if (!ps::jobs::debug_alloc::hook_available()) {
        SKIP("debug_alloc hook is disabled on this build (ASan/TSan)");
    }

    SessionFrame frame(k32N, 474000, 5560000);
    PhysicsTileGrid grid(frame);
    auto resident = std::make_shared<ResidentHeightSet>();

    constexpr int kNumKeys = 64;
    std::vector<PhysTileIndex> indices;
    indices.reserve(kNumKeys);
    for (int i = 0; i < kNumKeys; ++i) {
        indices.push_back(PhysTileIndex{i % 8, i / 8});
    }
    for (const auto& index : indices) {
        install_square(grid, *resident, index, 0);
    }

    G2mTerrainSource source(grid, resident, /*surface=*/ps::SurfaceId{0});
    const std::uint32_t n = source.samples_per_tile();

    std::vector<ps::terrain::TileKey> keys;
    keys.reserve(indices.size());
    for (const auto& index : indices) {
        keys.push_back(ps::terrain::TileKey{PhysicsTileGrid::pack(index)});
    }

    std::vector<ps::terrain::TileSample> samples(keys.size());
    for (auto& out : samples) {
        out.heights.assign(static_cast<std::size_t>(n) * n, 0.0);
        out.cell_surface_ids.assign(static_cast<std::size_t>(n - 1) * (n - 1), ps::kInvalidSurfaceId);
    }

    ps::jobs::JobSystem jobs(4);

    // Warm-up call outside the counter (first-touch page faults, thread
    // pool spin-up).
    jobs.parallel_batches(keys.size(), [&](std::size_t i) { source.fill_tile(keys[i], samples[i]); });

    ps::jobs::debug_alloc::reset();
    ps::jobs::debug_alloc::set_armed(true);
    jobs.parallel_batches(keys.size(), [&](std::size_t i) { source.fill_tile(keys[i], samples[i]); });
    ps::jobs::debug_alloc::set_armed(false);

    CHECK(ps::jobs::debug_alloc::own_violation_count() == 0);
}

// Negative control for the test above (PLAN.md R2.2 4b, G7): the same
// armed-window/parallel_batches/JobSystem(4) shape, with one deliberate heap
// allocation per job. Proves the hook really sees an allocation made on a
// worker thread inside a job while armed - without it, the 0 above could
// just as well mean "the hook never fires in this executable" (e.g. another
// operator new override winning at link time, or jobs not counted as
// in-job), and would be vacuous.
TEST_CASE("G2mTerrainSource: zero-alloc negative control - an allocation inside a job IS counted",
         "[bridge][terrain][alloc]") {
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
    // Every job allocated exactly once, and none of them ran outside a job
    // (parallel_batches runs every index as a job, inline fallback included).
    CHECK(counted == kJobs);
}
