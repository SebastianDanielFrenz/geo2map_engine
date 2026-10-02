#include "g2m/layer/road_profile.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <array>
#include <cmath>
namespace g2m {
Result<VerticalPose> RoadProfile::at(double station) const {
    if (!std::isfinite(station) || station < 0 || station > reference.length_m || vertical.segments.empty())
        return make_error(ErrorCode::InvalidArgument, "road profile: station outside profile");
    auto it = std::upper_bound(vertical.segments.begin(), vertical.segments.end(), station,
        [](auto value, const auto& segment) { return value < segment.station_m; });
    if (it == vertical.segments.begin()) return make_error(ErrorCode::Malformed, "road profile: station ordering");
    --it; return it->at(std::clamp(station, it->station_m, it->station_m + it->length_m));
}
Result<RoadProfile> build_road_profile(const RoadReferenceBuilder& builder,
    const RoadDemSampler& dem, geo::UtmZone zone, std::int64_t id, RoadStretch stretch,
    const RoadProfileLimits& limits) {
    if (!std::isfinite(limits.sample_spacing_m) || limits.sample_spacing_m < 0.5 ||
        limits.sample_spacing_m > 100 || limits.max_samples < 2 || limits.max_samples > 2048 ||
        limits.max_iterations < 1 || limits.max_iterations > 16384)
        return make_error(ErrorCode::InvalidArgument, "road profile: construction limits");
    const auto& graph = builder.graph();
    const auto wi = std::lower_bound(graph.ways.begin(), graph.ways.end(), id,
        [](const auto& way, auto search) { return way.osm_id < search; });
    if (wi == graph.ways.end() || wi->osm_id != id) return make_error(ErrorCode::NotFound, "road profile: way absent");
    const auto attrs = resolve_road_attributes(*wi); if (!attrs) return attrs.error();
    if (attrs->tunnel) return make_error(ErrorCode::Unsupported, "road profile: tunnel not driveable in v1");
    auto reference = builder.fit_stretch(id, stretch, limits.reference); if (!reference) return reference.error();
    if (reference->length_m < 0.5) return make_error(ErrorCode::LimitExceeded, "road profile: stretch shorter than sample spacing");
    const auto endpoint = [&](std::uint32_t index) -> Result<std::int32_t> {
        const auto ni = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), wi->node_ids[index],
            [](const auto& node, auto search) { return node.osm_id < search; });
        if (ni == graph.nodes.end() || ni->osm_id != wi->node_ids[index])
            return make_error(ErrorCode::Malformed, "road profile: endpoint absent");
        return dem.junction_height(zone, ni->position);
    };
    const auto first = endpoint(stretch.start_ref); if (!first) return first.error();
    const auto last = endpoint(stretch.end_ref); if (!last) return last.error();
    const double h0 = *first / 256.0, h1 = *last / 256.0;
    RoadProfile out; out.stretch = stretch; out.attributes = *attrs; out.reference = std::move(*reference);
    VerticalFitLimits fit; fit.max_grade = attrs->max_grade_per_mille / 1000.0;
    fit.min_radius_m = attrs->min_vertical_radius_m; fit.max_iterations = limits.max_iterations;
    if (attrs->bridge) {
        const std::array<VerticalSample, 2> abutments{{{0, h0}, {out.reference.length_m, h1}}};
        auto vertical = fit_vertical_profile(abutments, fit); if (!vertical) return vertical.error();
        out.vertical = std::move(*vertical);
        return out;
    }
    const auto intervals = static_cast<std::size_t>(std::ceil(out.reference.length_m / limits.sample_spacing_m));
    if (intervals + 1 > limits.max_samples)
        return make_error(ErrorCode::LimitExceeded, "road profile: complete stretch sample budget");
    const auto count = std::max(std::size_t{1}, intervals);
    // Redistribution avoids a final sub-0.5 m interval without dropping endpoints.
    const auto step = out.reference.length_m / static_cast<double>(count);
    if (step < 0.5) return make_error(ErrorCode::LimitExceeded, "road profile: sample interval below solver limit");
    std::vector<VerticalSample> samples; samples.reserve(count + 1); samples.push_back({0, h0});
    for (std::size_t i = 1; i < count; ++i) {
        const auto station = step * static_cast<double>(i); const auto pose = out.reference.at(station);
        if (!pose) return pose.error();
        const auto nx = -det::sin(pose->heading), ny = det::cos(pose->heading);
        std::array<std::int32_t, 7> heights{};
        for (int offset = -3; offset <= 3; ++offset) {
            const geom::PointMm point{std::llround((pose->x + offset * nx) * 1000),
                std::llround((pose->y + offset * ny) * 1000)};
            const auto height = dem.sample(zone, point); if (!height) return height.error();
            heights[static_cast<std::size_t>(offset + 3)] = *height;
        }
        std::sort(heights.begin(), heights.end()); samples.push_back({station, heights[3] / 256.0});
    }
    samples.push_back({out.reference.length_m, h1});
    auto vertical = fit_vertical_profile(samples, fit); if (!vertical) return vertical.error();
    out.vertical = std::move(*vertical); return out;
}
}
