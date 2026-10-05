#pragma once
#include "g2m/layer/road_graph.h"
#include "g2m/layer/osm_roads.h"

namespace g2m {
// Shared physical policy for geometry, carving and ribbons. Dimensions are
// integer millimetres, slopes are parts per thousand. Routing is separate.
struct RoadAttributes {
    std::int32_t width_mm = 0;
    std::int32_t lanes = 0; // zero when absent/invalid; never inferred from width
    LandClass land_class = LandClass::Unknown;
    SurfaceKind surface = SurfaceKind::Unknown;
    std::int32_t max_grade_per_mille = 0;
    std::int32_t min_vertical_radius_m = 0;
    std::int32_t min_horizontal_radius_m = 0;
    std::optional<std::int32_t> incline_per_mille;
    std::int32_t crown_per_mille = 25;
    std::int32_t shoulder_mm = 1500;
    std::int32_t blend_mm = 0;
    bool bridge = false;
    bool tunnel = false;
    bool width_inferred = true;
    bool surface_inferred = true;
    friend bool operator==(const RoadAttributes&, const RoadAttributes&) = default;
};
// The width parser shared with the cross-section resolver: clean metres or a
// trailing " m", at most six fractional digits, 0.5..100 m, result in mm.
std::optional<std::int32_t> parse_road_width_mm(std::string_view text);
// Resolves preserved tags against the existing highway/surface data tables.
// Rejects unsupported highways and invalid canonical tags. Unusable optional
// values fall back deterministically. width: clean metres or a trailing " m";
// lanes: integer 1..30; incline: signed numeric percent (-100..100).
// Width range 0.5..100 m, with explicit width > lanes*3.25 > class default.
Result<RoadAttributes> resolve_road_attributes(const RoadGraphWay& way,
    const RoadStyle& style = RoadStyle::default_style());
} // namespace g2m
