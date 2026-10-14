// Street fidelity P2 slices 3-4 (design sections 2 and 3): junction polygons by the osm2streets
// offset-intersect-trim algorithm, ported to integer-mm P1 edge polylines with exact predicates. Pure functions of a
// RoadGeomTile (its entries' references and cross-sections), the context graph's node degrees and the rules.
//
// Slice 3 builds junctions of ONE node; slice 4 adds clusters of trim-overlapping nodes (3.2) with one ring around
// the whole cluster (the boundary walk of decision D16). Slice 5 adds the junction surface (4.2: junction_height_m),
// raised junction tables (4.4: the RaisedTable flag, raised_height_m, the approach ramp records) and the suppression
// of approach features inside a ring. Slice 8 serves the records: build_road_geom_tile builds them with the world
// context and stores in RoadGeomTile::junctions only the junctions whose owner node (nodes.front()) lies in the tile
// (road_tile_contains, 5.1); roads.geom encoding 7 section 5 carries them (docs/formats/road_geom.md). D34: rings that
// do not nest within 2 mm, or have more than 1024 vertices, take the all-layer hull; if that fails the junction is
// declined Degenerate. The junction heights stay a library function for the carver.
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
// The record types (RoadJunction, JunctionApproach, JunctionRing, flags, declines) are in road_geom_records.h:
// RoadGeomTile carries the junctions its tile owns (encoding 7, slice 8).

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
    std::size_t nesting_hulls = 0;       // simple offset rings that did not nest within 2 mm: all-layer hull (D34, slice 8)
    std::size_t nesting_declined = 0;    // ... whose hull rings did not nest either: Degenerate (D34)
    std::size_t ring_cap_hulls = 0;      // an offset ring above kJunctionMaxRingVertices: all-layer hull (D34)
    std::size_t fillets = 0;             // block corners rounded with a kerb return (all layers of one corner count once)
    std::size_t filleted_junctions = 0;  // junctions with at least one fillet
    std::size_t star_fallbacks = 0;      // single nodes whose broken offset rings took the star union of their strips (D45)
    std::size_t fillet_fallbacks = 0;    // corners left sharp because their fillet broke ring simplicity or nesting
};

// Switches for the tests' sabotages and negative controls. Production uses the defaults.
struct JunctionBuildOptions {
    bool corner_caps = true;               // D17: a pair may close on the cap segments at the node
    bool hull_all_layers = true;           // D17: one non-simple layer sends every layer to the hull
    bool roundabouts_never_link = true;    // 3.2 rule 2
    bool cluster_caps = true;              // 3.2 rule 4 (node count and extent)
    bool unknown_links_are_absent = false; // a possible link to an incomplete node ends discovery (D20)
    bool raised_tables = true;             // slice 5: a table or hump tag on a junction node raises the junction (4.4)
    bool hull_when_not_nested = true;      // D34 (slice 8): simple offset rings that do not nest take the all-layer hull
    bool star_union_fallback = true;     // D45: false = the convex hull for every broken ring (pre-D45)
    bool corner_fillets = true;          // block corners take a kerb-return radius (rules.junction.corner_radius_mm)
};

// True iff `ring` has at least 3 vertices, no repeated vertex, positive (counter-clockwise) area and no two
// non-adjacent edges touching, and no adjacent pair folding back on itself. Exact. This is the ring validity of
// design 3.5 / 9.2.
bool junction_ring_valid(const JunctionRing& ring);
// True iff every vertex of the road bed ring lies inside, on, or within kJunctionNestingAllowanceMm of the paved ring,
// and likewise paved in outer (design 9.2, D21/D34). Point-in-polygon exact, the allowance by + - * / on coordinate
// differences. The caller bounds the coordinates (the roads.geom validation keeps them within 1 km of the owner node).
bool junction_rings_nested(const RoadJunction& junction);

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
    double relief_m = 0;                 // max |z_i - b_i| over the paved band at the trim station (decision D42)
    double fade_m = 0;                   // the crown fade length l_i (D42); +inf keeps the full relief everywhere
    double verge_fade_m = 0;             // the verge fade length (D42); +inf keeps the full verge everywhere
};
// The data junction_height_m needs. Built once per junction; evaluation is pure and allocation-free.
struct JunctionSurface {
    std::vector<JunctionSurfaceApproach> approaches;   // the junction's used approaches, in RoadJunction::approaches order
};

// The relief fade of decision D42: approach i's cross-section relief (crown, verge, the raise's sidewalk step) fades
// out over l_i = clamp(2 * relief_i / slope, min_m, max_m) from its trim segment, so the fade itself is never steeper
// than `slope` (the smooth step's peak slope is 2). Read from data/carve/carve_default.json ("junction_fade").
struct JunctionFadeParams {
    double slope = 0.010;
    double min_m = 3.0;
    double max_m = 15.0;
    double verge_m = 1.5;     // the verge term (outside the paved band) fades over this fixed length
    friend bool operator==(const JunctionFadeParams&, const JunctionFadeParams&) = default;
};

// The surface of a built junction (decline None, at least two used approaches). Nullopt otherwise or when a point
// cannot be placed. `tile` is the tile the junction was built from; `params` and `fade` set each approach's relief
// and fade length (D42).
std::optional<JunctionSurface> make_junction_surface(const RoadGeomTile& tile, const RoadJunction& junction,
    const RoadRules& rules = builtin_road_rules_de(), const RoadSurfaceParams& params = {}, const JunctionFadeParams& fade = {});

// The approach surface z_i(X) of design 4.2 for X = (x, y) in the UTM frame (metres): X is projected onto the
// approach's reference polyline (nearest segment, parameter clamped to the polyline, except that it may run before
// station 0), the entry's own height road_surface_height_m is read at the projected station and lateral offset
// (plus the approach's ramp record over the carriageway), and a projection before station 0 continues the profile
// linearly with its end grade (the crown, verge and ramp terms stay at station 0). The features of `tile` are used
// as they are: pass the kept features of adjust_features_for_junctions (the ramps are the surface's own).
double junction_approach_height_m(const JunctionSurface& surface, const RoadGeomTile& tile, std::size_t approach,
    double x, double y, const RoadSurfaceParams& params = {}) noexcept;

// The three readings of approach i at X that decision D42 blends, from one projection (that of
// junction_approach_height_m): base b_i at lateral offset 0 (profile height, the raise of a table junction where t = 0
// lies on the carriageway, the tangent extension before station 0), paved p_i at t clamped to the paved band, and the
// full surface z_i. All three are heights the approach's own cross-section takes at the projected station.
struct JunctionApproachHeights { double base = 0, paved = 0, full = 0; };
JunctionApproachHeights junction_approach_heights(const JunctionSurface& surface, const RoadGeomTile& tile, std::size_t approach,
    double x, double y, const RoadSurfaceParams& params = {}) noexcept;

// The relief weights of D42: phi_i(X) = road_smooth_step(1 - d_i / l_i) for the paved relief and psi_i(X) the same
// with the verge fade length (d_i the distance to trim segment i): 1 on the trim segment with zero slope there, 0
// from the fade length on. psi_i <= phi_i (the verge fade is never longer than the crown fade), so v_i is a convex
// combination of b_i, p_i and z_i.
void junction_relief_weights(const JunctionSurface& surface, std::size_t approach, double x, double y, double& phi, double& psi) noexcept;

// The blend weights w_i of design 4.2 (product form of inverse squared distances to the trim segments; they sum to
// 1, w_i = 1 on trim segment i). `weights` has one slot per approach.
void junction_weights(const JunctionSurface& surface, double x, double y, std::span<double> weights) noexcept;

// z_J(X) = sum_i w_i v_i(X), v_i = b_i + phi_i (p_i - b_i) + psi_i (z_i - p_i) (decision D42; D22-D24 had v_i = z_i),
// p_i the approach surface with t clamped to its paved band: each approach's paved relief (crown, the sidewalk step of a
// raise) fades out over l_i and its verge term over the verge fade length from its trim line, so inside the junction
// the weights mix the approach bases, which agree at the node. v_i = z_i on trim i, so z_J matches the approach
// surface there with C1 contact. The raise of a table or hump junction is part of b_i (decision D24) over the
// carriageway. X is assumed to lie inside the junction's outer ring (the caller decides ownership). Fixed approach
// order, only + - * / and det::sqrt: identical on every platform.
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
