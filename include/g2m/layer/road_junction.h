// Street fidelity P2 slice 3 (design sections 2 and 3): single-node junction polygons by the osm2streets
// offset-intersect-trim algorithm, ported to integer-mm P1 edge polylines with exact predicates. Pure functions of a
// RoadGeomTile (its entries' references and cross-sections), the context graph's node degrees and the rules.
//
// Slice 3 builds junctions of ONE node. Clusters of trim-overlapping nodes (3.2), junction heights (4.2), raised
// tables (4.4), the ownership filter (5.1) and the roads.geom encoding of the records (section 9) come in later
// slices; RoadGeomTile therefore does not carry the records yet and no served byte changes.
#pragma once
#include "g2m/core/geom/exact.h"
#include "g2m/core/result.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_rules.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

namespace g2m {
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
    HullFallback = 4,     // an offset ring was not simple; the convex hull of the trim segments is used
    Partial = 8,          // at least one approach is excluded, or a corner lies beyond the trim clamp
    RaisedTable = 16,     // a vertical traffic_calming feature on a junction node (slice 5)
};
enum class JunctionDecline : std::uint16_t {
    None = 0,
    ClusterTooLarge = 1,  // slice 4
    ReachExceeded = 2,    // a ring vertex > reach_max from the nearest cluster node
    SingleApproach = 3,   // fewer than two used approaches
    Degenerate = 4,       // even the hull fallback has < 3 non-collinear vertices (or an edge cannot be placed)
    Underground = 5,      // every approach is a tunnel
};
inline constexpr std::size_t kJunctionDeclineCount = 6;
struct RoadJunction {
    std::vector<std::int64_t> nodes;            // ascending OSM ids; nodes.front() is the owner node
    std::vector<JunctionApproach> approaches;   // used approaches in counter-clockwise order starting at the smallest
                                                // (way id, stretch start_ref, at_end), then the excluded ones in that key order
    std::vector<std::size_t> absorbed;          // entry indices of the stretches inside a cluster (slice 4)
    std::array<JunctionRing, 3> rings;          // RoadBed, Paved, Outer; empty when declined
    std::uint16_t flags = 0;
    JunctionDecline decline = JunctionDecline::None;
    double raised_height_m = 0;                 // RaisedTable only (slice 5)
    friend bool operator==(const RoadJunction&, const RoadJunction&) = default;
};

// Counters of one build, for the measurement tables and the tests.
struct JunctionBuildStats {
    std::size_t junction_nodes = 0;      // nodes of the tile graph with degree >= 3 in the context graph
    std::size_t skipped_incomplete = 0;  // fewer approaches in the tile than the node's degree (the tile is not the owner)
    std::size_t built = 0;               // records produced (declined ones included)
    std::array<std::size_t, kJunctionDeclineCount> declines{};   // by JunctionDecline value (index 0 = built without a decline)
    std::size_t partial = 0, on_structure = 0, hull_fallbacks = 0;
    std::size_t bowtie_repaired = 0;     // junctions that needed at least one trim-extension round and then had simple rings
    std::size_t bowtie_rounds = 0;       // total trim-extension rounds spent
    std::size_t excluded_approaches = 0;
    std::size_t approaches_used = 0;
    std::size_t max_ring_vertices = 0;   // largest outer ring
};

// True iff `ring` has at least 3 vertices, no repeated vertex, positive (counter-clockwise) area and no two
// non-adjacent edges touching, and no adjacent pair folding back on itself. Exact. This is the ring validity of
// design 3.5 / 9.2.
bool junction_ring_valid(const JunctionRing& ring);

// The junction of the node `node_id`, which needs degree >= 3 in `context` and one entry of `tile` per stretch end
// at the node (else an error). A declined junction is still returned with its approaches and `decline` set.
Result<RoadJunction> build_node_junction(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    std::int64_t node_id, const RoadRules& rules = builtin_road_rules_de(), JunctionBuildStats* stats = nullptr);

// Every node of the tile's graph whose degree in `context` is >= 3 and whose approaches all have an entry in the
// tile, ascending by node id. Nodes seen only partially (their other ways lie outside the tile's graph) are
// skipped and counted: some other tile owns them. No ownership filter yet (design 5.1, slice 8).
Result<std::vector<RoadJunction>> build_node_junctions(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    const RoadRules& rules = builtin_road_rules_de(), JunctionBuildStats* stats = nullptr);
} // namespace g2m
