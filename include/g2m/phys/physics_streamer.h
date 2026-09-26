// g2m/phys/physics_streamer.h - PhysicsTerrainStreamer (PLAN.md R2.2 section
// 1a): decides which DGM1 L0 height tiles must be resident for physics_sim's
// TileManager to fill its physics tiles, keeps a ResidentHeightSet in step
// with a HeightTileLoader, and reports whether the World may step (the
// clock-freeze gate racing_game's fixed-rate loop owns, PLAN.md section 2).
// ps-free, like everything under include/g2m/phys.
//
// Threading contract: update() runs on the set's OWNER thread, between World
// steps (resident_heights.h's own contract) - it is the only code that
// installs into or evicts from the set. The loader's workers never see the
// set; their completions arrive through HeightTileLoader::poll() inside
// update().
//
// update(), in order:
//   1. poll the loader; install completions in key order (poll() already
//      sorts them): Ready -> the tile, Absent (404) -> nullptr (holes).
//      Failed is never installed - it is remembered (so it is not
//      re-requested every tick) and reported in GateStatus::failed.
//   2. gate set = the L0 keys of TileManager's own needed square around each
//      interest point, widened by gate_margin_tiles: centre
//      index_for_local(x, y), r = ceil(radius_m / tile_size_m) +
//      gate_margin_tiles (TileManager's r is exactly ceil(radius_m /
//      tile_size_m) - see physics_streamer.cpp for the cited line). A
//      function of position and radius only - never of velocity or timing.
//   3. keep set = gate set + the prefetch square (r = ceil(radius_m /
//      tile_size_m) + prefetch_margin_tiles around the same centre) + a
//      look-ahead square (the gate radius, around p + v * lookahead_s, that
//      displacement capped to lookahead_cap_m in length).
//   4. evict: retain_only(keep) - only when the set holds more than
//      max_resident_l0 entries. keep always contains the whole gate set, so a
//      gate key is NEVER evicted; max_resident_l0 is therefore a soft cap
//      (when keep alone is larger, the set holds exactly keep). Remembered
//      Failed keys that left keep are forgotten, so they are retried if they
//      are ever needed again.
//   5. request every keep key that is neither in the set nor remembered as
//      Failed: gate keys first, then the rest, each group by ascending
//      squared distance from the L0 tile's centre to the nearest interest
//      point, ties by TileKey::packed() (a total, timing-independent order).
//   6. ready = every gate key is Resident or Absent.
//
// Hold-before-evict invariant (why step 2's +gate_margin_tiles matters):
// TileManager fills queued keys up to Q ticks after it queued them (at most
// max_tile_fills_per_tick per tick, stale keys never pruned), so a key it
// fills now may come from its square around a position visited up to Q ticks
// ago. At F = 1 Q <= 26 ticks (~0.11 s at 240 Hz): even at 100 m/s the car
// moves far less than one 255 m physics tile in that time, so every queued
// key lies inside the square one tile wider than TileManager's own - the
// gate set - which is never evicted and must be ready before any tick runs.
// tests/phys/test_physics_streamer.cpp simulates exactly that queue.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/phys/height_tile_loader.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"

#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace g2m::phys {

// One streaming centre, in the grid's session-local frame (physics_sim's
// world XY). vx/vy (m/s) only steer the look-ahead prefetch; the gate never
// depends on them. radius_m is the SAME radius the owner passes to
// World::set_terrain_interest_point.
struct InterestPoint {
    std::uint32_t id = 0;
    double x = 0.0, y = 0.0;
    double vx = 0.0, vy = 0.0;
    double radius_m = 0.0;
};

struct StreamerConfig {
    int gate_margin_tiles = 1;     // >= 0: extra physics tiles around TileManager's own square (see above)
    int prefetch_margin_tiles = 2; // >= 0: prefetch square radius over TileManager's own
    double lookahead_s = 3.0;      // >= 0
    double lookahead_cap_m = 510.0; // >= 0: max look-ahead displacement (2 physics tiles)
    std::size_t max_resident_l0 = 160; // soft cap on the set's size (see step 4)
};

struct GateStatus {
    bool ready = false;                 // every gate key is Resident or Absent
    std::uint32_t missing_required = 0; // gate keys not yet Resident/Absent (incl. Failed ones)
    std::uint32_t inflight = 0;         // loader: queued + being fetched
    std::uint32_t resident = 0;         // ResidentHeightSet::size() after this update
    std::uint32_t failed = 0;           // remembered Failed keys (all inside the keep set)
};

class PhysicsTerrainStreamer {
public:
    // `set` and `loader` must outlive the streamer; `grid` is copied. Throws
    // std::invalid_argument on a negative margin, look-ahead time or cap.
    PhysicsTerrainStreamer(const PhysicsTileGrid& grid, ResidentHeightSet& set, HeightTileLoader& loader,
                           StreamerConfig config);

    // Owner thread, between World steps. See the header comment for the
    // steps. An empty span gives an empty gate (ready) and evicts down to
    // nothing once over the cap.
    GateStatus update(std::span<const InterestPoint> points);

    // Forgets every remembered Failed key, so the next update() requests them
    // again (e.g. once the owner knows the server is back).
    void retry_failed();

    // The last update()'s gate and keep sets, sorted by TileKey::packed()
    // (diagnostics/tests).
    const std::vector<TileKey>& gate_keys() const { return gate_; }
    const std::vector<TileKey>& keep_keys() const { return keep_; }

    const StreamerConfig& config() const { return config_; }

private:
    void add_square(PhysTileIndex centre, int r, std::vector<TileKey>& into);
    static void sort_unique(std::vector<TileKey>& keys);

    PhysicsTileGrid grid_;
    ResidentHeightSet& set_;
    HeightTileLoader& loader_;
    StreamerConfig config_;

    std::vector<TileKey> gate_;   // sorted by packed(), unique
    std::vector<TileKey> keep_;   // sorted by packed(), unique; superset of gate_
    std::vector<TileKey> failed_; // sorted by packed(), unique
    // Reused scratch.
    std::vector<LoadedHeightTile> completions_;
    std::vector<TileKey> square_;
    struct Ranked {
        double d2;
        std::uint64_t packed;
        TileKey key;
    };
    std::vector<Ranked> ranked_gate_, ranked_rest_;
    std::vector<TileKey> request_;
};

} // namespace g2m::phys
