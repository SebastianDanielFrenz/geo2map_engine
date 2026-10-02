#include "g2m/layer/road_profile.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <array>
#include <cmath>
namespace g2m {
namespace {
struct Anchor { RoadJunctionPlane plane; bool inferred = false, adjusted = false; };
// A degree-two bridge boundary is a structure/tag transition, not a road
// junction. Bare-earth heights there may belong to the road passing below.
// Infer one common anchor from supported ground approach data, and use it
// for both the bridge and its adjoining ground way.
Result<Anchor> raw_anchor(const RoadGraph& graph, const RoadDemSampler& dem,
    geo::UtmZone zone, std::int64_t id) {
    const auto point = [&](std::int64_t node) {
        return std::lower_bound(graph.nodes.begin(), graph.nodes.end(), node,
            [](const auto& n, auto search) { return n.osm_id < search; })->position;
    };
    const auto attrs = [&](const RoadGraphEdge& edge) {
        const auto wi = std::lower_bound(graph.ways.begin(), graph.ways.end(), edge.way_id,
            [](const auto& way, auto search) { return way.osm_id < search; });
        return resolve_road_attributes(*wi);
    };
    const auto incident = [&](std::int64_t node) {
        std::vector<const RoadGraphEdge*> edges;
        for (const auto& edge : graph.edges) if (edge.from_node == node || edge.to_node == node) edges.push_back(&edge);
        return edges;
    };
    const auto edges = incident(id);
    std::size_t bridge_count = 0;
    std::vector<const RoadGraphEdge*> approaches;
    for (const auto* edge : edges) {
        const auto a = attrs(*edge); if (!a) return a.error();
        if (a->bridge) ++bridge_count;
        else if (!a->tunnel) approaches.push_back(edge);
    }
    if (!bridge_count) {
        const auto plane = dem.junction_plane(zone, point(id)); if (!plane) return plane.error();
        return Anchor{*plane, false};
    }
    if (bridge_count != 1 || approaches.empty() || approaches.size() > 8)
        return make_error(ErrorCode::Unsupported, "road profile: bridge boundary requires ground approach context");
    const auto from_approach = [&](const RoadGraphEdge* approach) -> Result<Anchor> {
        constexpr double support_distance = 32; // metres away from the bare-earth underpass footprint
    double remaining = support_distance; auto node = id;
    for (std::size_t work = 0; work < 1000; ++work) {
        const auto other = approach->from_node == node ? approach->to_node : approach->from_node;
        const auto a = point(node), b = point(other);
        const auto dx = (b.x - a.x) / 1000.0, dy = (b.y - a.y) / 1000.0;
        const auto length = det::sqrt(dx * dx + dy * dy);
        if (length >= remaining) {
            const auto fraction = remaining / length;
            const geom::PointMm support{std::llround(a.x + fraction * (b.x - a.x)), std::llround(a.y + fraction * (b.y - a.y))};
            auto plane = dem.junction_plane(zone, support); if (!plane) return plane.error();
            const auto target = point(id);
            const auto height = plane->height + (plane->grade_x * ((target.x - support.x) / 1000.0) +
                plane->grade_y * ((target.y - support.y) / 1000.0)) * 256;
            if (!std::isfinite(height) || height <= -2147483647.0 || height > 2147483647.0)
                return make_error(ErrorCode::LimitExceeded, "road profile: inferred anchor height range");
            plane->height = static_cast<std::int32_t>(std::llround(height)); return Anchor{*plane, true};
        }
        remaining -= length; const auto next = incident(other);
        if (next.size() != 2) return make_error(ErrorCode::Unsupported, "road profile: bridge approach reaches junction before support");
        const auto* candidate = next[0] == approach ? next[1] : next[0];
        const auto a_next = attrs(*candidate); if (!a_next) return a_next.error();
        if (a_next->bridge || a_next->tunnel)
            return make_error(ErrorCode::Unsupported, "road profile: insufficient ground approach support");
        approach = candidate; node = other;
    }
    return make_error(ErrorCode::LimitExceeded, "road profile: approach walk budget");
    };
    std::vector<RoadJunctionPlane> planes;
    for (const auto* approach : approaches) {
        const auto anchor = from_approach(approach); if (!anchor) return anchor.error();
        planes.push_back(anchor->plane);
    }
    // At a true merge, all supported ground approaches must agree within the
    // same 0.25 m uncertainty per estimate before a shared junction plane is used.
    const auto [low, high] = std::minmax_element(planes.begin(), planes.end(),
        [](const auto& a, const auto& b) { return a.height < b.height; });
    if (static_cast<std::int64_t>(high->height) - low->height > 128)
        return make_error(ErrorCode::LimitExceeded, "road profile: junction approach estimates disagree by more than 0.5 m");
    RoadJunctionPlane plane;
    std::int64_t height_sum = 0;
    for (const auto& estimate : planes) {
        height_sum += estimate.height;
        plane.grade_x += estimate.grade_x;
        plane.grade_y += estimate.grade_y;
    }
    plane.height = static_cast<std::int32_t>(std::llround(static_cast<double>(height_sum) / static_cast<double>(planes.size())));
    plane.grade_x /= static_cast<double>(planes.size());
    plane.grade_y /= static_cast<double>(planes.size());
    return Anchor{plane, true};
}

Result<Anchor> surface_anchor(const RoadReferenceBuilder& builder, const RoadDemSampler& dem,
    geo::UtmZone zone, std::int64_t id, const ReferenceFitLimits& limits) {
    const auto& graph = builder.graph(); auto raw = raw_anchor(graph, dem, zone, id);
    if (!raw || !raw->inferred) return raw;
    const RoadGraphWay* span = nullptr;
    for (const auto& edge : graph.edges) if (edge.from_node == id || edge.to_node == id) {
        const auto wi = std::lower_bound(graph.ways.begin(), graph.ways.end(), edge.way_id,
            [](const auto& way, auto search) { return way.osm_id < search; });
        const auto a = resolve_road_attributes(*wi); if (!a) return a.error();
        if (a->bridge) span = &*wi;
    }
    if (!span || (id != span->node_ids.front() && id != span->node_ids.back()))
        return make_error(ErrorCode::Unsupported, "road profile: bridge boundary is not a complete span endpoint");
    const auto a = raw_anchor(graph, dem, zone, span->node_ids.front()); if (!a) return a.error();
    const auto b = raw_anchor(graph, dem, zone, span->node_ids.back()); if (!b) return b.error();
    const auto reference = builder.fit(span->osm_id, limits); if (!reference) return reference.error();
    const auto attributes = resolve_road_attributes(*span); if (!attributes) return attributes.error();
    const auto start = reference->at(0), end = reference->at(reference->length_m);
    if (!start) return start.error(); if (!end) return end.error();
    const auto project = [](const auto& plane, double heading) { return plane.grade_x * det::cos(heading) + plane.grade_y * det::sin(heading); };
    const auto m0 = project(a->plane, start->heading), m1 = project(b->plane, end->heading);
    const auto grade = attributes->max_grade_per_mille / 1000.0, k = 1.0 / attributes->min_vertical_radius_m;
    if (std::fabs(m0) > grade || std::fabs(m1) > grade)
        return make_error(ErrorCode::LimitExceeded, "road profile: bridge approach slope exceeds class bound");
    const auto length = reference->length_m, scale = length * length / 6;
    const auto low = std::max({(m0 + m1 - grade) * length / 3,
        ((4 * m0 + 2 * m1) / length - k) * scale, ((2 * m0 + 4 * m1) / length - k) * scale});
    const auto high = std::min({(m0 + m1 + grade) * length / 3,
        ((4 * m0 + 2 * m1) / length + k) * scale, ((2 * m0 + 4 * m1) / length + k) * scale});
    const auto lower = static_cast<std::int64_t>(std::ceil(low * 256 - 1e-10));
    const auto upper = static_cast<std::int64_t>(std::floor(high * 256 + 1e-10));
    if (lower > upper) return make_error(ErrorCode::LimitExceeded, "road profile: bridge approach slopes incompatible with span curvature");
    const auto original_delta = static_cast<std::int64_t>(b->plane.height) - a->plane.height;
    const auto delta = std::clamp(original_delta, lower, upper);
    // Project the two uncertain height estimates onto the feasible cubic rise,
    // preserving their mean. Corrections are limited to 0.25 m per anchor and
    // shared by the deck and both approaches; measured junctions are untouched.
    const auto first_height = floor_div(static_cast<std::int64_t>(a->plane.height) + b->plane.height - delta, 2);
    const auto last_height = first_height + delta;
    if (std::abs(first_height - a->plane.height) > 64 || std::abs(last_height - b->plane.height) > 64)
        return make_error(ErrorCode::LimitExceeded, "road profile: bridge anchor correction exceeds 0.25 m");
    const auto adjusted_height = id == span->node_ids.front() ? first_height : last_height;
    if (adjusted_height <= -2147483647LL || adjusted_height > 2147483647LL)
        return make_error(ErrorCode::LimitExceeded, "road profile: corrected anchor height range");
    raw->adjusted = adjusted_height != raw->plane.height;
    raw->plane.height = static_cast<std::int32_t>(adjusted_height); return raw;
}
}
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
    const auto endpoint = [&](std::uint32_t index) -> Result<Anchor> {
        const auto ni = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), wi->node_ids[index],
            [](const auto& node, auto search) { return node.osm_id < search; });
        if (ni == graph.nodes.end() || ni->osm_id != wi->node_ids[index])
            return make_error(ErrorCode::Malformed, "road profile: endpoint absent");
        return surface_anchor(builder, dem, zone, ni->osm_id, limits.reference);
    };
    const auto first = endpoint(stretch.start_ref); if (!first) return first.error();
    const auto last = endpoint(stretch.end_ref); if (!last) return last.error();
    const double h0 = first->plane.height / 256.0, h1 = last->plane.height / 256.0;
    RoadProfile out; out.stretch = stretch; out.attributes = *attrs; out.reference = std::move(*reference);
    out.start_anchor_inferred = first->inferred; out.end_anchor_inferred = last->inferred;
    out.start_anchor_adjusted = first->adjusted; out.end_anchor_adjusted = last->adjusted;
    VerticalFitLimits fit; fit.max_grade = attrs->max_grade_per_mille / 1000.0;
    if (attrs->incline_per_mille) fit.target_grade = *attrs->incline_per_mille / 1000.0;
    fit.min_radius_m = attrs->min_vertical_radius_m; fit.max_iterations = limits.max_iterations;
    const auto start = out.reference.at(0), end = out.reference.at(out.reference.length_m);
    if (!start) return start.error(); if (!end) return end.error();
    fit.start_grade = first->plane.grade_x * det::cos(start->heading) + first->plane.grade_y * det::sin(start->heading);
    fit.end_grade = last->plane.grade_x * det::cos(end->heading) + last->plane.grade_y * det::sin(end->heading);
    if (std::fabs(fit.start_grade) > fit.max_grade || std::fabs(fit.end_grade) > fit.max_grade)
        return make_error(ErrorCode::LimitExceeded, "road profile: estimated endpoint slope exceeds class bound");
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
        // The bridge approach footprint may contain the underlying road or
        // embankment. Use the supported anchor plane through that 32 m zone;
        // the support footprint itself was fully validated by raw_anchor.
        const bool near_start = first->inferred && station <= 32;
        const bool near_end = last->inferred && out.reference.length_m - station <= 32;
        if (near_start || near_end) {
            const auto height_from = [&](const Anchor& anchor, const geom::CurvePose& at) {
                return anchor.plane.height / 256.0 + anchor.plane.grade_x * (pose->x - at.x) + anchor.plane.grade_y * (pose->y - at.y);
            };
            const auto height = near_start && near_end ?
                std::lerp(height_from(*first, *start), height_from(*last, *end), station / out.reference.length_m) :
                near_start ? height_from(*first, *start) : height_from(*last, *end);
            samples.push_back({station, height}); continue;
        }
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
