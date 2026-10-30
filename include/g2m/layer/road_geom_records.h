// Street fidelity P2: the junction and furniture records that roads.geom encoding 7 stores (design section 2, 9.1).
// Plain data, shared by road_geom_tile.h (RoadGeomTile carries them), road_junction.h (builds the junctions) and
// road_furniture.h (builds the furniture). The builders and their contracts live in those two headers.
#pragma once
#include "g2m/core/geom/exact.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_rules.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace g2m {
// ---- junctions (design section 3) ----
enum class JunctionApproachStatus : std::uint8_t {
    Used = 0,
    NoProfile = 1,        // entry declined: excluded from the ring and the heights
    NoCrossSection = 2,   // profile present, cross-section declined
    Tunnel = 3,
};
struct JunctionApproach {
    std::size_t entry_index = 0;      // into RoadGeomTile::entries of the tile holding the junction
    bool at_end = false;              // the junction is at the stretch's end_ref (else start_ref)
    JunctionApproachStatus status = JunctionApproachStatus::Used;
    double trim_m = 0;                // distance from the junction-side stretch end to the trim line
    friend bool operator==(const JunctionApproach&, const JunctionApproach&) = default;
};
enum class JunctionRingLayer : std::uint8_t { RoadBed = 0, Paved = 1, Outer = 2 };
struct JunctionRing {
    std::vector<geom::PointMm> vertices;  // counter-clockwise, simple, no repeated vertex
    friend bool operator==(const JunctionRing&, const JunctionRing&) = default;
};
enum class JunctionFlags : std::uint16_t {
    Cluster = 1,          // more than one node (slice 4)
    OnStructure = 2,      // every used approach is a bridge: polygon served, never carved
    HullFallback = 4,     // an offset ring was not simple, or the offset rings did not nest; the convex hull of the trim segments is used
    Partial = 8,          // at least one approach is excluded, or a corner lies beyond the trim clamp
    RaisedTable = 16,     // a vertical traffic_calming feature on a junction node (slice 5)
};
inline constexpr std::uint16_t kJunctionFlagsDefinedMask = 31;
enum class JunctionDecline : std::uint16_t {
    None = 0,
    ClusterTooLarge = 1,  // the linked set exceeds cluster_max_nodes or cluster_max_extent (decision D19)
    ReachExceeded = 2,    // a ring vertex > reach_max from the nearest cluster node
    SingleApproach = 3,   // fewer than two used approaches
    Degenerate = 4,       // even the hull fallback has < 3 non-collinear vertices (or an edge cannot be placed, or the hull rings do not nest)
    Underground = 5,      // every approach is a tunnel
    MergeLane = 6,        // a merge lane or an exit lands on this node (JoinKind::Landing): the carriageways meet in their joins, no patch (G2M-028)
};
inline constexpr std::size_t kJunctionDeclineCount = 7;
struct RoadJunction {
    std::vector<std::int64_t> nodes;            // ascending OSM ids; nodes.front() is the owner node
    std::vector<JunctionApproach> approaches;   // used approaches (stubs) in ring order starting at the smallest
                                                // (way id, stretch start_ref, at_end), then the excluded ones in that key order
    std::vector<std::size_t> absorbed;          // entry indices of the stretches inside a cluster, ascending
    std::array<JunctionRing, 3> rings;          // RoadBed, Paved, Outer; empty when declined
    std::uint16_t flags = 0;
    JunctionDecline decline = JunctionDecline::None;
    double raised_height_m = 0;                 // RaisedTable only: the table or hump height; it enters the surface through the approach ramps (D24)
    std::int64_t raised_node = 0;               // RaisedTable only: the node whose table/hump tag sets the height
    friend bool operator==(const RoadJunction&, const RoadJunction&) = default;
};
// Ring nesting (design 9.2, decision D21): every vertex of the road bed ring lies inside or on the paved ring or at
// most this far outside it, and likewise paved in outer. Each layer's vertices are rounded to mm independently, so a
// vertex on an edge two layers share can lie up to ~1.5 mm outside the next ring.
inline constexpr double kJunctionNestingAllowanceMm = 2.0;
// The most vertices one ring may have (encoding 7 budget; the ring checks are quadratic). A builder ring above it takes
// the all-layer hull (D34). The fixtures' largest is 246.
inline constexpr std::size_t kJunctionMaxRingVertices = 1024;

// ---- furniture (design section 7) ----
enum class FurnitureKind : std::uint8_t { Crossing = 0, StopLine = 1, GiveWayLine = 2, Signal = 3 };
enum class FurnitureDecline : std::uint16_t {
    None = 0,
    StretchDeclined = 1,       // the node's stretch has no profile or no cross-section
    Tunnel = 2,
    JunctionNode = 3,          // a crossing on a degree >= 3 node: the crossing direction is unknown
    DirectionUnresolved = 4,   // a stop or give-way sign without a direction and without a junction end within reach
    NoLanes = 5,               // driving_lanes_at returns 0 for the travel direction
    Ambiguous = 6,             // a stop/give_way on a junction node whose approaches all have the same rank
    JunctionDeclined = 7,      // a junction-node signal or sign at a declined junction
};
inline constexpr std::size_t kFurnitureDeclineCount = 8;
enum FurnitureFlag : std::uint8_t {
    kFurnitureSignalised = 1,      // a crossing with crossing=traffic_signals
    kFurnitureLoweredKerb = 2,     // kerb=lowered or flush
    kFurnitureSpansJoin = 4,       // a crossing at a degree-two way join: the quad extends back over the join
    kFurnitureInsideJunction = 8,  // reserved, never set and rejected by the roads.geom validation (D28, D33): a consumer
                                   // derives "inside a junction ring" from the owner junction's approach trims
    kFurnitureLeftPost = 16,       // the second signal post, on the travel-left side
    kFurnitureDefinedMask = 31,
};
// One item. A declined record keeps its identity (kind, node, entry, travel) and has no extent (t_from = t_to = length = 0).
struct RoadFurniture {
    FurnitureKind kind = FurnitureKind::Crossing;
    FurnitureDecline decline = FurnitureDecline::None;
    std::int64_t node_id = 0;          // the source OSM node; the junction's owner node for a line or signal derived from a junction
    std::size_t entry_index = 0;       // the stretch it sits on (0 for a record of a declined junction)
    double station_m = 0;              // centre along the entry's reference line
    LaneDirection travel = LaneDirection::Forward;  // lines and signals: the direction controlled (Forward or Backward)
    double t_from = 0, t_to = 0;       // lateral extent in the reference frame, mm-rounded (signal: both = t of the post)
    double length_m = 0;               // along the road: crossing width, line width (a signal: 0), mm-rounded
    CrossingPattern pattern = CrossingPattern::None;
    std::uint8_t flags = 0;
    std::uint32_t junction_index = 0;  // 1 + the junction's index in the list given to build_road_furniture, else 0
    friend bool operator==(const RoadFurniture&, const RoadFurniture&) = default;
};
} // namespace g2m
