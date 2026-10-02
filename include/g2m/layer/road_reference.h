#pragma once
#include "g2m/core/geom/clothoid.h"
#include "g2m/layer/road_graph.h"
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
};
struct RoadStretch { std::uint32_t start_ref = 0, end_ref = 0; };
struct RoadStretchFit { RoadStretch stretch; Result<RoadReferenceLine> reference; };
// Fits a G1 chain through a complete way with shared bisector tangents.
// Stable original ref-pair ids survive coincident/repeated refs. This is the
// Interpolation falls back to minimum-radius circular corner fitting inside
// the same corridor. Infeasible overlapping corners still fail explicitly.
Result<RoadReferenceLine> fit_road_reference(const RoadGraph& graph, std::int64_t way_id,
    const ReferenceFitLimits& limits = {});
// Validates topology once and reuses its immutable degree view for many ways.
// The caller-owned graph must outlive the builder and must not be mutated.
class RoadReferenceBuilder {
public:
    static Result<RoadReferenceBuilder> make(const RoadGraph& graph);
    Result<std::vector<RoadStretch>> stretches(std::int64_t way_id) const;
    Result<RoadReferenceLine> fit_stretch(std::int64_t way_id, RoadStretch stretch,
        const ReferenceFitLimits& limits = {}) const;
    Result<std::vector<RoadStretchFit>> fit_stretches(std::int64_t way_id,
        const ReferenceFitLimits& limits = {}) const;
    Result<RoadReferenceLine> fit(std::int64_t way_id, const ReferenceFitLimits& limits = {}) const;
private:
    RoadReferenceBuilder() = default;
    const RoadGraph* graph_ = nullptr;
    std::vector<std::uint32_t> degree_;
};
}
