// g2m/world/ring_streamer.h - which level-2 (1024 m) tiles of a ring product (road ribbons, buildings) to build,
// show, downgrade and drop around a moving focus (terrain migration plan phase 3b).
//
// The policy of racing_game's game/scripts/road_stream.gd and building_stream.gd (5a3baed), moved out of GDScript;
// tests/world/test_ring_streamer.cpp pins it against those scripts run headless. It only decides; the consumer
// builds the tiles (on its own worker, through next_job()) and owns the render resources.
//
// Two tiers, in absolute UTM metres from the focus to a tile's centre (x*1024+512, y*1024+512):
//   d <= full_m + show_margin_m          the tile is wanted in full (and its near part is shown)
//   d <= far_m + show_margin_m           the tile is wanted at least partially (its far part is shown)
//   d >  full_m + evict_margin_m         a resident full tile is downgraded to partial (near part dropped)
//   d >  far_m + evict_margin_m          a resident tile is evicted (and forgotten, so it is wanted again later)
// A one-tier product (road ribbons) sets full_m == far_m: every tile is full and never downgraded.
//
// Scheduling runs when the focus enters another tile: every tile of the square of radius ceil(far_m/1024)+1 around
// the focus tile that is wanted and not yet known at that tier becomes a job, nearest first; a tile more than
// behind_m behind a focus moving at >= min_speed_mps goes after everything ahead (+ behind_penalty_m). The job list
// is replaced, not appended to. A known tile (built, even if empty) is not wanted again until it is evicted.
//
// Threading: update(), accept(), frame() and the queries from one consumer thread; next_job() and pending_jobs() from
// any thread (one internal mutex). No engine type anywhere.
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <utility>
#include <vector>

namespace g2m::world {

struct RingStreamerConfig {
    double tile_m = 1024.0;
    double full_m = 2500.0;
    double far_m = 2500.0;
    double show_margin_m = 725.0;
    double evict_margin_m = 2048.0;
    double behind_m = 1024.0;
    double behind_penalty_m = 5000.0;
    double min_speed_mps = 2.0;
    // An empty result keeps a resident tile of the same key as it is (road_stream.gd) instead of dropping it
    // (building_stream.gd).
    bool empty_keeps_resident = false;
};

struct RingJob {
    std::int32_t x = 0, y = 0; // level-2 tile indices
    bool full = true;
    double order = 0.0;        // distance in metres, + behind_penalty_m behind a moving focus
};

enum class RingAccept : std::uint8_t {
    Rejected, // a partial result for a tile resident in full: ignored, nothing changes
    Empty,    // known now, nothing to show (an old resident tile is dropped unless empty_keeps_resident)
    Added,    // resident now (replaces an old resident tile of the same key)
};

// One per resident tile whose state changed in a frame() call.
struct RingChange {
    std::int32_t x = 0, y = 0;
    bool near_visible = false;
    bool far_visible = false;
    bool downgraded = false; // the near (full) part was dropped; the tile stays as partial
    bool evicted = false;    // the tile is gone (free it)
};

class RingStreamer {
public:
    explicit RingStreamer(RingStreamerConfig config);

    [[nodiscard]] const RingStreamerConfig& config() const { return config_; }

    // The focus (absolute UTM) and its velocity. When the focus tile changed since the last call, rebuilds the job
    // list and returns true.
    bool update(double easting, double northing, double vx = 0.0, double vy = 0.0);

    // The nearest queued job, removed from the queue; nullopt when none. Any thread.
    std::optional<RingJob> next_job();
    [[nodiscard]] std::size_t pending_jobs() const; // any thread
    // A copy of the queue in serving order (nearest first).
    [[nodiscard]] std::vector<RingJob> jobs() const;

    // A built result for tile (x, y) at tier `full`; `has_content` = anything to show.
    RingAccept accept(std::int32_t x, std::int32_t y, bool full, bool has_content);

    // Visibility, downgrades and evictions for the focus (absolute UTM): appends one RingChange per resident tile
    // whose visibility changed (a newly accepted tile always reports once), was downgraded or evicted.
    void frame(double easting, double northing, std::vector<RingChange>& out);

    // Known tiles (built at some tier, possibly empty), sorted by (y, x): {x, y, full}.
    struct Known {
        std::int32_t x, y;
        bool full;
    };
    [[nodiscard]] std::vector<Known> known() const;
    [[nodiscard]] std::size_t resident_count() const { return resident_.size(); }
    // Seconds the longest-waiting wanted tile (scheduled, not yet accepted) has waited; 0 when none.
    [[nodiscard]] double oldest_wanted_age_s() const;

private:
    struct Resident {
        bool full = true;
        bool reported = false;
        bool near_visible = false;
        bool far_visible = false;
    };
    using Key = std::pair<std::int32_t, std::int32_t>; // (y, x): sorted south to north, west to east
    void schedule(std::int32_t cx, std::int32_t cy, double e, double n, double vx, double vy);

    RingStreamerConfig config_;
    bool has_center_ = false;
    std::int32_t center_x_ = 0, center_y_ = 0;
    mutable std::mutex mutex_;
    std::vector<RingJob> jobs_; // reverse serving order: next_job() pops the back
    std::map<Key, bool> known_; // -> full
    std::map<Key, Resident> resident_;
    std::map<Key, std::chrono::steady_clock::time_point> wanted_since_;
};

} // namespace g2m::world
