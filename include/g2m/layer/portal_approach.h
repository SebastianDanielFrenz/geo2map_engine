#pragma once
// Terrain migration phase 7a-1 (docs/structure_classification.md section 5): the approach level of a tunnel portal.
// A tunnel floor must start where the ground road that joins it IS, not where the DEM at the portal node happens to
// sit (the node often lies on a bank face or an embankment foot: 0.84 m too high at Unterliederbach, up to 8.2 m on
// rail). The approach level is the median DEM 2-8 m out along the joining ground way, carried back to the node along its
// measured grade (docs say "approach height"; a rising road would otherwise start 0.3 m high at 6 %).
// Pure and tile-invariant: it reads the way's polyline within 8 m of the node and the DEM within 8.5 m of it.
#include "g2m/layer/road_dem_sampler.h"
#include <optional>
#include <span>
namespace g2m {
inline constexpr double kPortalApproachNearM = 2, kPortalApproachFarM = 8;
// The grade is clamped to this (1:10) in a road plane: the same end-grade clamp as the completion tier's ground fit.
inline constexpr double kPortalApproachMaxGrade = 0.10;
struct PortalApproach {
    std::int32_t height = 0;     // 1/256 m, the level of the road AT the node: the median of the grade-detrended samples
    double grade = 0;            // m/m, rising away from the node (last third mean - first third mean), clamped to +-kPortalApproachMaxGrade
    double dir_x = 0, dir_y = 0; // unit vector from the node toward the far samples
    int samples = 0;
};
// `polyline` runs from the portal node outward along the ground way (first point = the node, further points are the
// way's following nodes in order, the way's end included). Samples the DEM at 2, 3, ... 8 m of arc length (as far as the
// polyline reaches), bilinearly. nullopt: the polyline is shorter than 2 m, or a sample is missing from the DEM.
std::optional<PortalApproach> portal_approach(const RoadDemSampler& dem, geo::UtmZone zone, std::span<const geom::PointMm> polyline);
}
