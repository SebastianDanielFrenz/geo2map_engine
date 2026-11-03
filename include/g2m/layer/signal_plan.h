// Traffic signals as a shared, engine-neutral subsystem: fixed-time signal plans derived from the served junction
// furniture (roads.geom Signal and StopLine records), a pure state function of (plan, time), a spatial index and the
// stop-or-go rule every vehicle that obeys signals uses (NPC traffic, farm helpers, a future server).
//
// What is NOT here: lamp meshes (a renderer stands a head on each SignalHead and colours it by SignalPlan::aspect_of),
// pedestrian signals and green waves. See docs/traffic_signals.md.
//
// Determinism (PLAN.md 2.4): no libm transcendental, integer millisecond arithmetic for the plan clock, no iteration of an
// unordered container that decides a result. A plan is a pure function of the furniture records of its junction; the
// state at time t is a pure function of (plan, t), so every client or server given the same clock sees the same lamps
// and nothing needs networking.
//
// Where a plan lives: a junction's furniture is emitted only by its owner tile and a mid-block signal's by every tile
// that holds its entry, identical in all of them (road_furniture.h), so plans of different tiles never disagree.
// SignalIndex::add skips a plan whose node is already present.
#pragma once
#include "g2m/core/result.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/tile_container.h"
#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <unordered_map>
#include <vector>

namespace g2m {
enum class SignalAspect : std::uint8_t { Red = 0, RedAmber = 1, Green = 2, Amber = 3 };
const char* to_string(SignalAspect aspect);

// The stop-or-go rule's parameters. A consumer with a heavy vehicle (a loaded tractor) passes its own values.
struct SignalDriverRules {
    double comfortable_decel_mps2 = 3.0;
    double max_decel_mps2 = 6.0;
    double reaction_s = 0.5;
    double lateral_tolerance_m = 2.0;   // an agent this far outside the stop line's ends still counts as on the approach
    double heading_cos_min = 0.7;       // the agent's heading must agree with the approach's travel direction to this cosine
    friend bool operator==(const SignalDriverRules&, const SignalDriverRules&) = default;
};

// data/signals/signal_timing.json (embedded). Integer milliseconds and per-mille like the other rule files.
struct SignalTiming {
    std::int32_t amber_ms = 0, red_amber_ms = 0, clearance_ms = 0;
    std::int32_t green_base_ms = 0, green_per_lane_ms = 0, green_min_ms = 0, green_max_ms = 0;
    std::int32_t single_axis_red_ms = 0, max_cycle_ms = 0;
    std::int32_t axis_cross_max_permille = 0;
    SignalDriverRules driver;
    ContentHash content_sha256{};
    friend bool operator==(const SignalTiming&, const SignalTiming&) = default;
};
Result<SignalTiming> load_signal_timing(std::span<const std::byte> bytes);
std::span<const std::byte> builtin_signal_timing_bytes();
const SignalTiming& builtin_signal_timing();

struct SignalHead {
    double x = 0, y = 0;            // UTM metres (stand the head on the ground surface; no height stored)
    double face_x = 0, face_y = 0;  // unit vector the lamps face (towards approaching traffic)
    bool left = false;              // the second head on the travel-left side
    friend bool operator==(const SignalHead&, const SignalHead&) = default;
};

// One controlled direction of one road: a stop line (the union of its lane-group lines) and its signal heads.
struct SignalApproach {
    std::int64_t node_id = 0;         // the plan's OSM node
    std::int64_t way_id = 0;
    LaneDirection travel = LaneDirection::Forward;
    double station_m = 0;             // of the stop line on the entry's reference line
    double x = 0, y = 0;              // stop line centre, UTM metres
    double dir_x = 0, dir_y = 0;      // unit travel direction at the line
    double ax = 0, ay = 0, bx = 0, by = 0;   // stop line ends: a on the t_from side, b on the t_to side
    std::uint32_t lanes = 0;
    std::uint32_t phase = 0;          // index into SignalPlan::phases
    std::vector<SignalHead> heads;
    // Stable identity: FNV-1a 64 of (node, way, travel, station in mm).
    std::uint64_t id() const;
    friend bool operator==(const SignalApproach&, const SignalApproach&) = default;
};

// One phase: the red-amber that precedes it, its green and the amber after it, then the clearance; start_ms is where the
// red-amber begins inside the cycle. A phase without approaches is the virtual cross phase of a single-axis plan.
struct SignalPhase {
    std::int32_t start_ms = 0, green_ms = 0;
    friend bool operator==(const SignalPhase&, const SignalPhase&) = default;
};
struct SignalState {
    SignalAspect aspect = SignalAspect::Red;
    double seconds_to_change = 0;   // until the aspect of this phase changes
};
struct SignalPlan {
    std::int64_t node_id = 0;
    std::vector<SignalApproach> approaches;   // ascending id
    std::vector<SignalPhase> phases;
    std::int32_t cycle_ms = 0, offset_ms = 0;
    std::int32_t amber_ms = 0, red_amber_ms = 0, clearance_ms = 0;

    // The aspect of a phase at absolute time t_s (any epoch, any sign) and the time until it next changes.
    SignalState state(std::uint32_t phase, double t_s) const;
    SignalState state_of(const SignalApproach& approach, double t_s) const { return state(approach.phase, t_s); }
    friend bool operator==(const SignalPlan&, const SignalPlan&) = default;
};

struct SignalPlanStats {
    std::size_t plans = 0, approaches = 0, heads = 0;
    std::size_t single_axis_plans = 0;   // plans with a virtual cross phase
    std::size_t scaled_plans = 0;        // plans whose cycle was scaled down to max_cycle_ms
    std::size_t unmatched_signals = 0;   // Signal records without a stop line of the same approach (skipped)
};
// The plans of a tile's signalised nodes, ascending node id. Pure function of tile.furniture, tile.entries and the timing.
Result<std::vector<SignalPlan>> build_signal_plans(const RoadGeomTile& tile, const SignalTiming& timing = builtin_signal_timing(),
    SignalPlanStats* stats = nullptr);

enum class SignalAdvice : std::uint8_t { Go = 0, Stop = 1 };
// A signal an agent is approaching.
struct SignalHit {
    const SignalPlan* plan = nullptr;
    const SignalApproach* approach = nullptr;
    double distance_m = 0;          // along the travel direction from the agent to the stop line (negative: past it)
    double lateral_m = 0;           // across: positive to the left of the travel direction
    SignalAspect aspect = SignalAspect::Red;
    double seconds_to_change = 0;
};
// The stop-or-go rule: Green goes; Amber stops when the agent can still stop at the line with the comfortable
// deceleration after its reaction time (otherwise it goes: the dilemma zone); Red and RedAmber stop. A line already
// passed (distance < 0) always goes on.
SignalAdvice signal_advice(const SignalHit& hit, double speed_mps, const SignalDriverRules& rules = builtin_signal_timing().driver);
// The speed from which the agent can still stop at the line with the comfortable deceleration: sqrt(2 a d), 0 at or past it.
double signal_stop_speed_cap(double distance_m, const SignalDriverRules& rules = builtin_signal_timing().driver);

// Every plan of the loaded tiles, with a 64 m grid for the position queries. add() is the only mutator; reads are const.
class SignalIndex {
public:
    explicit SignalIndex(SignalTiming timing = builtin_signal_timing()) : timing_(std::move(timing)) {}
    // Adds the plans; a plan whose node id is already present is skipped. Returns how many were added.
    std::size_t add(std::vector<SignalPlan> plans);
    std::size_t plan_count() const { return plans_.size(); }
    const SignalPlan* plan_of_node(std::int64_t node_id) const;
    const SignalTiming& timing() const { return timing_; }

    // The nearest controlled stop line ahead of an agent at (x, y) heading (hx, hy) (unit), within horizon_m: the agent
    // is on the approach when it lies behind the line (distance in [-0.5, horizon]) within the line's ends plus the
    // driver's lateral tolerance, and its heading agrees with the travel direction. Ties by approach id.
    std::optional<SignalHit> next_ahead(double x, double y, double hx, double hy, double horizon_m, double t_s,
        const SignalDriverRules& rules = builtin_signal_timing().driver) const;
    // Calls fn(plan, approach) for every approach with its stop line centre within radius_m of (x, y), ascending id
    // (a renderer's "heads near the camera").
    template <class Fn> void for_each_near(double x, double y, double radius_m, Fn&& fn) const;

private:
    struct Ref { std::uint32_t plan, approach; };
    static std::int64_t cell_key(std::int32_t cx, std::int32_t cy) { return (static_cast<std::int64_t>(cx) << 32) ^ static_cast<std::uint32_t>(cy); }
    void collect(double x, double y, double radius_m, std::vector<Ref>& out) const;
    SignalTiming timing_;
    std::vector<SignalPlan> plans_;                              // insertion order
    std::unordered_map<std::int64_t, std::uint32_t> by_node_;    // lookup only
    std::unordered_map<std::int64_t, std::vector<Ref>> grid_;    // lookup only
};

template <class Fn> void SignalIndex::for_each_near(double x, double y, double radius_m, Fn&& fn) const {
    std::vector<Ref> refs;
    collect(x, y, radius_m, refs);
    for (const Ref& r : refs) fn(plans_[r.plan], plans_[r.plan].approaches[r.approach]);
}
} // namespace g2m
