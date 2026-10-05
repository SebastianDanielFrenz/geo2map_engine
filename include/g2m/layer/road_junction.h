// Street fidelity P2 slices 3-4 (design sections 2 and 3): junction polygons by the osm2streets
// offset-intersect-trim algorithm, ported to integer-mm P1 edge polylines with exact predicates. Pure functions of a
// RoadGeomTile (its entries' references and cross-sections), the context graph's node degrees and the rules.
//
// Slice 3 builds junctions of ONE node; slice 4 adds clusters of trim-overlapping nodes (3.2) with one ring around
// the whole cluster (the boundary walk of decision D16). Slice 5 adds the junction surface (4.2: junction_height_m),
// raised junction tables (4.4: the RaisedTable flag, raised_height_m, the approach ramp records) and the suppression
// of approach features inside a ring. The ownership
// filter (5.1) and the roads.geom encoding of the records (section 9) come in later slices; RoadGeomTile therefore
// does not carry the records yet and no served byte changes.
#pragma once
#include "g2m/core/geom/exact.h"
#include "g2m/core/result.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_rules.h"
#include "g2m/layer/road_surface.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
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
    ClusterTooLarge = 1,  // the linked set exceeds cluster_max_nodes or cluster_max_extent (decision D19)
    ReachExceeded = 2,    // a ring vertex > reach_max from the nearest cluster node
    SingleApproach = 3,   // fewer than two used approaches
    Degenerate = 4,       // even the hull fallback has < 3 non-collinear vertices (or an edge cannot be placed)
    Underground = 5,      // every approach is a tunnel
};
inline constexpr std::size_t kJunctionDeclineCount = 6;
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

// Counters of one build, for the measurement tables and the tests.
struct JunctionBuildStats {
    std::size_t junction_nodes = 0;      // nodes of the tile graph with degree >= 3 in the context graph
    std::size_t skipped_incomplete = 0;  // fewer approaches in the tile than the node's degree (the tile is not the owner)
    std::size_t skipped_unresolved = 0;  // complete, but a possible link leads to an incomplete node (slice 4)
    std::size_t built = 0;               // records produced (declined ones included)
    std::array<std::size_t, kJunctionDeclineCount> declines{};   // by JunctionDecline value (index 0 = built without a decline)
    std::size_t partial = 0, on_structure = 0, hull_fallbacks = 0;
    std::size_t bowtie_repaired = 0;     // junctions that needed at least one trim-extension round and then had simple rings
    std::size_t bowtie_rounds = 0;       // total trim-extension rounds spent
    std::size_t excluded_approaches = 0;
    std::size_t approaches_used = 0;
    std::size_t max_ring_vertices = 0;   // largest outer ring
    std::size_t clusters = 0;            // built records with more than one node (ClusterTooLarge records excluded)
    std::size_t cluster_nodes = 0;       // nodes in those records
    std::size_t absorbed = 0;            // absorbed stretches in those records
    std::size_t face_fallbacks = 0;      // a stub off the cluster's outer boundary walk: hull rings (D16)
    std::size_t max_cluster_nodes = 0;
    std::size_t raised_tables = 0;       // built records with the RaisedTable flag (slice 5)
    std::size_t calming_ignored = 0;     // junction nodes with another vertical kind (bump, cushion, ...): raises nothing (4.4)
};

// Switches for the tests' sabotages and negative controls. Production uses the defaults.
struct JunctionBuildOptions {
    bool corner_caps = true;               // D17: a pair may close on the cap segments at the node
    bool hull_all_layers = true;           // D17: one non-simple layer sends every layer to the hull
    bool roundabouts_never_link = true;    // 3.2 rule 2
    bool cluster_caps = true;              // 3.2 rule 4 (node count and extent)
    bool unknown_links_are_absent = false; // a possible link to an incomplete node ends discovery (D20)
    bool raised_tables = true;             // slice 5: a table or hump tag on a junction node raises the junction (4.4)
};

// True iff `ring` has at least 3 vertices, no repeated vertex, positive (counter-clockwise) area and no two
// non-adjacent edges touching, and no adjacent pair folding back on itself. Exact. This is the ring validity of
// design 3.5 / 9.2.
bool junction_ring_valid(const JunctionRing& ring);

// The single-node junction of `node_id`, which needs degree >= 3 in `context` and one entry of `tile` per stretch
// end at the node (else an error). A declined junction is still returned with its approaches and `decline` set.
Result<RoadJunction> build_node_junction(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    std::int64_t node_id, const RoadRules& rules = builtin_road_rules_de(), JunctionBuildStats* stats = nullptr,
    const JunctionBuildOptions& options = {});

// The single-node junction of every node of the tile's graph whose degree in `context` is >= 3 and whose approaches
// all have an entry in the tile, ascending by node id, without clustering. Nodes seen only partially (their other
// ways lie outside the tile's graph) are skipped and counted: some other tile owns them.
Result<std::vector<RoadJunction>> build_node_junctions(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    const RoadRules& rules = builtin_road_rules_de(), JunctionBuildStats* stats = nullptr,
    const JunctionBuildOptions& options = {});

// The junctions of the tile with clustering (design 3.2, decisions D16-D20): per complete junction node either its
// single-node junction or, from the cluster's smallest node, the cluster junction. A node whose bounded discovery
// exceeds the caps keeps its single-node junction, and the smallest node of its discovery set also stores a
// ClusterTooLarge record (nodes, untrimmed approaches, linking entries; no rings). A node whose discovery reaches
// a possible link to an incomplete node is skipped and counted (the owner tile resolves it). Ascending by
// nodes.front(); for one owner the single-node record precedes the ClusterTooLarge one. Tile-invariant: every
// record depends only on nodes within cluster_max_extent + link_trim_cap * 2 of its owner, inside the halo.
Result<std::vector<RoadJunction>> build_road_junctions(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    const RoadRules& rules = builtin_road_rules_de(), JunctionBuildStats* stats = nullptr,
    const JunctionBuildOptions& options = {});


// ---- Slice 5: junction heights (design 4.2) ----

// One used approach of a junction surface: its reference polyline (the 0.5 m stations out to trim + 1, in metres),
// its trim segment (the outer-ring segment between the approach's two outer trim points, the mm-rounded points of
// the ring itself, in metres) and, for a RaisedTable junction, the ramp record of design 4.4.
struct JunctionSurfaceApproach {
    std::size_t entry_index = 0;
    bool at_end = false;
    double length_m = 0;                 // the entry's reference length
    double trim_m = 0;
    std::vector<double> sigma;           // ascending, sigma.front() == 0, distance from the junction-side stretch end
    std::vector<double> ref_x, ref_y;    // the reference line at those stations
    double trim_ax = 0, trim_ay = 0, trim_bx = 0, trim_by = 0;
    bool has_ramp = false;
    ProfileFeature ramp;                 // plateau [0, trim] at the raised height, one ramp beyond the trim line
};
// The data junction_height_m needs. Built once per junction; evaluation is pure and allocation-free.
struct JunctionSurface {
    std::vector<JunctionSurfaceApproach> approaches;   // the junction's used approaches, in RoadJunction::approaches order
};

// The surface of a built junction (decline None, at least two used approaches). Nullopt otherwise or when a point
// cannot be placed. `tile` is the tile the junction was built from.
std::optional<JunctionSurface> make_junction_surface(const RoadGeomTile& tile, const RoadJunction& junction,
    const RoadRules& rules = builtin_road_rules_de());

// The approach surface z_i(X) of design 4.2 for X = (x, y) in the UTM frame (metres): X is projected onto the
// approach's reference polyline (nearest segment, parameter clamped to the polyline, except that it may run before
// station 0), the entry's own height road_surface_height_m is read at the projected station and lateral offset
// (plus the approach's ramp record over the carriageway), and a projection before station 0 continues the profile
// linearly with its end grade (the crown, verge and ramp terms stay at station 0). The features of `tile` are used
// as they are: pass the kept features of adjust_features_for_junctions (the ramps are the surface's own).
double junction_approach_height_m(const JunctionSurface& surface, const RoadGeomTile& tile, std::size_t approach,
    double x, double y, const RoadSurfaceParams& params = {}) noexcept;

// The blend weights w_i of design 4.2 (product form of inverse squared distances to the trim segments; they sum to
// 1, w_i = 1 on trim segment i). `weights` has one slot per approach.
void junction_weights(const JunctionSurface& surface, double x, double y, std::span<double> weights) noexcept;

// z_J(X) = sum_i w_i z_i(X). The raise of a table or hump junction is part of the z_i (decision D24), so over the
// approaches' carriageways it equals the raised core on every trim line and it does not step on a sidewalk. X is
// assumed to lie inside the junction's outer ring (the caller decides ownership). Fixed approach order, only
// + - * / and det::sqrt: identical on every platform.
double junction_height_m(const JunctionSurface& surface, const RoadGeomTile& tile, double x, double y,
    const RoadSurfaceParams& params = {}) noexcept;

// ---- Slice 5: calming at junctions (design 4.4) ----

struct JunctionFeatureAdjustment {
    std::vector<ProfileFeature> features;   // the tile's features minus the suppressed ones, in the tile's order
    std::vector<ProfileFeature> ramps;      // one Table-kind record per used approach of a RaisedTable junction
    std::size_t suppressed = 0;             // features whose footprint reaches into a junction ring (not carved)
    // features + ramps, sorted by (entry, station, node id): the list a carver evaluates the approach cores with.
    std::vector<ProfileFeature> core_features() const;
};
// Every feature of an approach whose footprint reaches into the approach's ring (centre - effective length / 2 <
// trim, measured from the junction-side stretch end) is dropped (decision D23), and every used approach of a
// RaisedTable junction gets a ramp record: kind Table, plateau [0, trim], one ramp of max(10 x height, 1 m) beyond
// the trim line (decision D22), carried by the table node. `junctions` are records built from `tile`.
JunctionFeatureAdjustment adjust_features_for_junctions(const RoadGeomTile& tile, std::span<const RoadJunction> junctions);
} // namespace g2m
