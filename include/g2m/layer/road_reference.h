#pragma once
#include "g2m/core/geom/clothoid.h"
#include "g2m/layer/road_graph.h"
#include <optional>
namespace g2m {
struct RoadReferenceSegment {
    std::uint32_t original_pair_index = 0;
    double station_m = 0;
    geom::Clothoid curve;
    std::uint32_t piece_index = 0; // 0 reference piece, 1 rounded corner
    bool break_before = false; // a topological junction separates G1 chains
};
struct RoadReferenceAnchor { std::int64_t node_id = 0; double station_m = 0; };
struct RoadReferenceLine {
    std::int64_t way_id = 0;
    double length_m = 0;
    std::size_t coincident_pairs = 0;
    std::vector<RoadReferenceSegment> segments;
    std::vector<RoadReferenceAnchor> anchors; // endpoints and branching junctions
    bool curvature_exception = false;
    // The chord bound (ReferenceFitLimits::chord_*) made the stretch unfittable, so it keeps its unbounded fit.
    bool chord_bound_relaxed = false;
    double requested_min_radius_m = 0;
    double fitted_min_radius_m = 0; // certified bound, not a measured radius
    Result<geom::CurvePose> at(double station) const;
};
struct ReferenceFitLimits {
    // Zero selects the physical class policy. Major roads use a 0.5 m
    // deviation corridor, local roads 3 m. Bounds fail instead of relaxing.
    double min_radius_m = 0;
    double max_deviation_m = 0;
    double max_bow_m = 32; // curve/chord bow, separate from original vertex tolerance
    std::size_t max_segments = 100000;
    double max_length_m = 100000;
    bool allow_curvature_exceptions = false; // bounded retries; corridor/budgets stay fixed
    // Bound on how far a fitted clothoid may leave the OSM polyline (vault G2M-024). The clothoid of a knot pair takes
    // its end headings from the neighbours (join bisectors, extrapolation) and may bow far from the chord the mapper
    // drew; max_bow_m (32 m) only stops absurd cases. Per segment the curve may be at most
    //   chord_half_width_factor * (half the way's resolved width) + chord_margin_m
    // away from the chord (perpendicular distance to the segment), else the stretch takes the rounded straight/arc
    // fallback. Both zero = no bound (the fitter's behaviour before G2M-024). Set from the road rules (reference block).
    double chord_half_width_factor = 0;
    double chord_margin_m = 0;
};
struct RoadStretch { std::uint32_t start_ref = 0, end_ref = 0; };
// The single other way at a degree-two stretch end (a way join), as seen in the
// builder's graph. `ref` is the index of the join node in the neighbour way's
// node_ids. `leaves_node`: the neighbour's refs run away from the node
// (increasing index), otherwise they arrive at it. The neighbour may be the
// stretch's own way (a ring that is split elsewhere).
struct RoadStretchJoin {
    std::int64_t way_id = 0;
    std::uint32_t ref = 0;
    bool leaves_node = false;
    friend bool operator==(const RoadStretchJoin&, const RoadStretchJoin&) = default;
};
struct RoadStretchFit { RoadStretch stretch; Result<RoadReferenceLine> reference; };
// Fits a G1 chain through a complete way with shared bisector tangents.
// Stable original ref-pair ids survive coincident/repeated refs. This is the
// Interpolation falls back to minimum-radius circular corner fitting inside
// the same corridor. Infeasible overlapping corners still fail explicitly.
Result<RoadReferenceLine> fit_road_reference(const RoadGraph& graph, std::int64_t way_id,
    const ReferenceFitLimits& limits = {});
// Validates topology once and reuses its immutable incidence view for many ways.
// The caller-owned graph must outlive the builder and must not be mutated.
// Node degree is the number of graph edges ending at the node, so the graph
// must hold EVERY way incident to the nodes of the ways being fitted. A tile
// graph alone does not (it stops at its halo); the roads.geom deriver therefore
// builds the builder over the merge of the tile graph and the graphs of the
// tiles that contain the nodes (merge_road_graphs). Stretch extents and join
// headings then depend only on the world, never on the tile.
class RoadReferenceBuilder {
public:
    static Result<RoadReferenceBuilder> make(const RoadGraph& graph);
    const RoadGraph& graph() const { return *graph_; }
    Result<std::vector<RoadStretch>> stretches(std::int64_t way_id) const;
    Result<RoadReferenceLine> fit_stretch(std::int64_t way_id, RoadStretch stretch,
        const ReferenceFitLimits& limits = {}) const;
    Result<std::vector<RoadStretchFit>> fit_stretches(std::int64_t way_id,
        const ReferenceFitLimits& limits = {}) const;
    Result<RoadReferenceLine> fit(std::int64_t way_id, const ReferenceFitLimits& limits = {}) const;
    // Edges ending at a node of the builder's graph (0 when the node is absent).
    std::uint32_t degree(std::int64_t node_id) const;
    // The neighbour way at a stretch end whose node has degree two and whose
    // second edge is not part of the stretch itself; nullopt at junctions, dead
    // ends and closed rings. `end` selects the stretch end (false = start).
    Result<std::optional<RoadStretchJoin>> join(std::int64_t way_id, RoadStretch stretch, bool end) const;
private:
    RoadReferenceBuilder() = default;
    std::size_t node_index(std::int64_t node_id) const; // graph nodes size when absent
    const RoadGraph* graph_ = nullptr;
    std::vector<std::uint32_t> incident_begin_, incident_edges_; // CSR over graph nodes
};
}
