#pragma once
#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_dem_sampler.h"
#include "g2m/layer/road_attributes.h"
#include "g2m/layer/vertical_profile.h"
namespace g2m {
class RoadUnderpassFinder;   // road_underpass.h
struct RoadProfile {
    RoadStretch stretch;
    RoadAttributes attributes;
    RoadReferenceLine reference;
    VerticalProfile vertical;
    bool start_anchor_inferred = false, end_anchor_inferred = false;
    bool start_anchor_adjusted = false, end_anchor_adjusted = false;
    Result<VerticalPose> at(double station_m) const;
};
struct RoadProfileLimits {
    ReferenceFitLimits reference;
    double sample_spacing_m = 5;
    std::size_t max_samples = 2048;
    int max_iterations = 4096;
    // Underpass mask (D53): with a finder, the DEM samples of a ground stretch over a served underpass span are replaced
    // by the rim chord (RoadUnderpassFinder::mask). Null: the DEM as before. Not owned.
    const RoadUnderpassFinder* underpass = nullptr;
};
// Complete topological stretch, never a tile-clipped fragment. Ground endpoints
// use shared-node robust DEM planes; bridge endpoints use those same
// medians and a constrained abutment interpolation without sampling the
// bare-earth interior. Endpoint grades project the common plane onto each
// reference tangent, preserving slope without forcing junctions flat.
// All required DEM samples must exist. Tunnels remain explicitly unsupported.
Result<RoadProfile> build_road_profile(const RoadReferenceBuilder& builder,
    const RoadDemSampler& dem, geo::UtmZone zone, std::int64_t way_id, RoadStretch stretch,
    const RoadProfileLimits& limits = {});
// The anchor build_road_profile uses at a stretch end node: the shared junction plane of a node without a bridge, or
// at a degree-two bridge boundary the plane inferred from the supported ground approaches, its height corrected by at
// most 0.25 m to the feasible cubic rise of the bridge span (inferred = true). Errors where build_road_profile would
// decline at that end. The completion tier reads it at bridge nodes so a completed profile meets a strict one there
// at the same height (decision D30). `limits` = the strict pass's RoadProfileLimits::reference.
struct RoadStrictAnchor { RoadJunctionPlane plane; bool inferred = false, adjusted = false; };
Result<RoadStrictAnchor> road_strict_anchor(const RoadReferenceBuilder& builder, const RoadDemSampler& dem,
    geo::UtmZone zone, std::int64_t node, const ReferenceFitLimits& limits);
}
