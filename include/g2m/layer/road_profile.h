#pragma once
#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_dem_sampler.h"
#include "g2m/layer/road_attributes.h"
#include "g2m/layer/vertical_profile.h"
namespace g2m {
struct RoadProfile {
    RoadStretch stretch;
    RoadAttributes attributes;
    RoadReferenceLine reference;
    VerticalProfile vertical;
    Result<VerticalPose> at(double station_m) const;
};
struct RoadProfileLimits {
    ReferenceFitLimits reference;
    double sample_spacing_m = 5;
    std::size_t max_samples = 2048;
    int max_iterations = 4096;
};
// Complete topological stretch, never a tile-clipped fragment. Ground endpoints
// use shared-node DEM medians and zero grade; bridge endpoints use those same
// medians and a constrained abutment interpolation without sampling the
// bare-earth interior. Both ground and deck endpoint grades are zero, avoiding
// a grade jump at their shared boundary before junction surfaces are built.
// All required DEM samples must exist. Tunnels remain explicitly unsupported.
Result<RoadProfile> build_road_profile(const RoadReferenceBuilder& builder,
    const RoadDemSampler& dem, geo::UtmZone zone, std::int64_t way_id, RoadStretch stretch,
    const RoadProfileLimits& limits = {});
}
