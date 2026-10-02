#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_attributes.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <cmath>
namespace g2m {
namespace {
struct Knot { geom::PointMm p; std::uint32_t index; bool junction = false; };
Result<RoadReferenceLine> fit_geometry(const std::vector<Knot>& knots, bool closed, std::int64_t id,
    double radius, double deviation, const ReferenceFitLimits& limits, std::size_t coincident_pairs) {
    RoadReferenceLine out; out.way_id = id; out.coincident_pairs = coincident_pairs;
    std::vector<double> headings(knots.size());
    const auto unit = [&](std::size_t a, std::size_t b) {
        const auto dx = (knots[b].p.x - knots[a].p.x) / 1000.0, dy = (knots[b].p.y - knots[a].p.y) / 1000.0;
        const auto length = det::sqrt(dx * dx + dy * dy); return std::pair{dx / length, dy / length};
    };

    for (std::size_t i = 0; i < knots.size(); ++i) {
        auto incoming = i ? unit(i - 1, i) : closed ? unit(knots.size() - 2, knots.size() - 1) : unit(0, 1);
        auto outgoing = i + 1 < knots.size() ? unit(i, i + 1) : closed ? unit(0, 1) : incoming;
        const auto x = incoming.first + outgoing.first, y = incoming.second + outgoing.second;
        if (x * x + y * y < 1e-12) return make_error(ErrorCode::Malformed, "road reference: reversing tangent");
        headings[i] = det::atan2(y, x);
        if (closed || (i && i + 1 < knots.size())) {
            const auto previous = i ? i - 1 : knots.size() - 2;
            const auto current = i + 1 == knots.size() ? std::size_t{0} : i;
            const auto length = [&](std::size_t a, std::size_t b) {
                const auto dx = (knots[b].p.x - knots[a].p.x) / 1000.0, dy = (knots[b].p.y - knots[a].p.y) / 1000.0;
                return det::sqrt(dx * dx + dy * dy);
            };
            const auto l0 = length(previous, previous + 1), l1 = length(current, current + 1);
            const auto angle = det::atan2(incoming.first * outgoing.second - incoming.second * outgoing.first,
                incoming.first * outgoing.first + incoming.second * outgoing.second);
            headings[i] = det::atan2(incoming.second, incoming.first) + angle * l0 / (l0 + l1);
        }
    }
    if (!closed && knots.size() >= 3) {
        const auto extrapolate = [&](std::size_t first, bool end) {
            const auto a = unit(first, first + 1), b = unit(first + 1, first + 2);
            const auto angle = det::atan2(a.first * b.second - a.second * b.first, a.first * b.first + a.second * b.second);
            const auto distance = [&](std::size_t i) {
                const auto dx = (knots[i + 1].p.x - knots[i].p.x) / 1000.0, dy = (knots[i + 1].p.y - knots[i].p.y) / 1000.0;
                return det::sqrt(dx * dx + dy * dy);
            };
            const auto l0 = distance(first), l1 = distance(first + 1);
            return end ? det::atan2(b.second, b.first) + angle * l1 / (l0 + l1)
                : det::atan2(a.second, a.first) - angle * l0 / (l0 + l1);
        };
        headings.front() = extrapolate(0, false); headings.back() = extrapolate(knots.size() - 3, true);
    }
    const auto rounded = [&]() -> Result<RoadReferenceLine> {
        struct Corner { double trim = 0, angle = 0, entry_x = 0, entry_y = 0, exit_x = 0, exit_y = 0, heading = 0; };
        std::vector<Corner> corners(knots.size());
        for (std::size_t i = 0; i < knots.size(); ++i) {
            auto& c = corners[i];
            c.entry_x = c.exit_x = knots[i].p.x / 1000.0; c.entry_y = c.exit_y = knots[i].p.y / 1000.0;
            if ((!i || i + 1 == knots.size()) && !closed) continue;
            if (closed && i + 1 == knots.size()) { c = corners.front(); continue; }
            const auto incoming = i ? unit(i - 1, i) : unit(knots.size() - 2, knots.size() - 1);
            const auto outgoing = unit(i, i + 1);
            c.heading = det::atan2(incoming.second, incoming.first);
            c.angle = det::atan2(incoming.first * outgoing.second - incoming.second * outgoing.first,
                incoming.first * outgoing.first + incoming.second * outgoing.second);
            const auto angle = std::fabs(c.angle);
            if (angle < 1e-8) { c.angle = 0; continue; }
            if (angle >= det::kPi - 1e-6 || radius * (1 / det::cos(angle / 2) - 1) > deviation + 1e-10)
                return make_error(ErrorCode::LimitExceeded, "road reference: rounded corner corridor");
            c.trim = radius * det::tan(angle / 2);
            c.entry_x -= incoming.first * c.trim; c.entry_y -= incoming.second * c.trim;
            c.exit_x += outgoing.first * c.trim; c.exit_y += outgoing.second * c.trim;
        }
        RoadReferenceLine result; result.way_id = id; result.coincident_pairs = out.coincident_pairs;
        const auto append = [&](std::uint32_t index, std::uint32_t piece, const geom::Clothoid& curve) -> Result<void> {
            if (curve.length < 1e-8) return {};
            const auto pose = curve.at(0); if (!pose) return pose.error();
            if (result.length_m + curve.length > limits.max_length_m || result.segments.size() >= limits.max_segments)
                return make_error(ErrorCode::LimitExceeded, "road reference: rounded length/segment budget");
            result.segments.push_back({index, result.length_m, curve, piece}); result.length_m += curve.length; return {};
        };
        for (std::size_t i = 0; i + 1 < knots.size(); ++i) {
            const auto& a = corners[i]; const auto& b = corners[i + 1];
            const auto dx = (knots[i + 1].p.x - knots[i].p.x) / 1000.0;
            const auto dy = (knots[i + 1].p.y - knots[i].p.y) / 1000.0;
            const auto original_length = det::sqrt(dx * dx + dy * dy);
            if (a.trim + b.trim > original_length + 1e-10)
                return make_error(ErrorCode::LimitExceeded, "road reference: overlapping rounded corners");
            const auto heading = det::atan2(dy, dx);
            const auto straight = append(knots[i].index, 0,
                {a.exit_x, a.exit_y, heading, std::max(0.0, original_length - a.trim - b.trim), 0, 0});
            if (!straight) return straight.error();
            if (b.angle != 0) {
                const auto arc = append(knots[i].index, 1,
                    {b.entry_x, b.entry_y, b.heading, radius * std::fabs(b.angle), b.angle > 0 ? 1 / radius : -1 / radius, 0});
                if (!arc) return arc.error();
            }
        }
        if (result.segments.empty()) return make_error(ErrorCode::Malformed, "road reference: empty rounded line");
        return result;
    };
    std::size_t corridor_work = 0;
    for (std::size_t i = 0; i + 1 < knots.size(); ++i) {
        const auto& a = knots[i].p; const auto& b = knots[i + 1].p;
        const auto curve = geom::fit_clothoid(a.x / 1000.0, a.y / 1000.0, headings[i], b.x / 1000.0, b.y / 1000.0, headings[i + 1]);
        if (!curve) return rounded();
        if (out.length_m + curve->length > limits.max_length_m)
            return make_error(ErrorCode::LimitExceeded, "road reference: length budget");
        const auto curvature = std::max(std::fabs(curve->curvature0),
            std::fabs(curve->curvature0 + curve->curvature_rate * curve->length));
        if (curvature > 1 / radius + 1e-12)
            return rounded();
        if (curvature > 1e-14) {
            // The deviation between a twice-differentiable unit-speed curve
            // and endpoint linear interpolation is bounded between samples by
            // max_curvature * step^2 / 8. Certify the entire corridor.
            const auto step = std::min(2.0, det::sqrt(8 * limits.max_bow_m / (100 * curvature)));
            const auto count_d = std::ceil(curve->length / step);
            if (count_d > 1000000) return make_error(ErrorCode::LimitExceeded, "road reference: corridor work budget");
            const auto count = static_cast<std::size_t>(std::max(1.0, count_d));
            if (count + 1 > 100000 - corridor_work)
                return make_error(ErrorCode::LimitExceeded, "road reference: total corridor work budget");
            corridor_work += count + 1;
            const auto interval = curve->length / count;
            const auto allowance = curvature * interval * interval / 8;
            for (std::size_t sample = 0; sample <= count; ++sample) {
                const auto t = static_cast<double>(sample) / count;
                const auto pose = curve->at(t * curve->length); if (!pose) return pose.error();
                const auto dx = pose->x - (a.x / 1000.0 + (b.x - a.x) / 1000.0 * t);
                const auto dy = pose->y - (a.y / 1000.0 + (b.y - a.y) / 1000.0 * t);
                if (det::sqrt(dx * dx + dy * dy) + allowance > limits.max_bow_m)
                    return rounded();
            }
        }
        out.segments.push_back({knots[i].index, out.length_m, *curve}); out.length_m += curve->length;
        if (out.length_m > limits.max_length_m) return make_error(ErrorCode::LimitExceeded, "road reference: length budget");
    }
    return out;
}
}

namespace {
Result<RoadReferenceLine> fit_shape(const std::vector<Knot>& knots, bool closed, std::int64_t id,
    double radius, double deviation, const ReferenceFitLimits& limits, std::size_t coincident_pairs) {
    auto original = fit_geometry(knots, closed, id, radius, deviation, limits, coincident_pairs);
    if (original || original.error().code != ErrorCode::LimitExceeded) return original;
    // Smooth only shape vertices. Shared branch nodes and endpoints stay fixed.
    // Each moved vertex remains within half the original corridor; the fitted
    // curve uses the other half, so their Hausdorff bounds compose.

    auto smoothed = knots; bool changed = false;
    const auto budget_mm = deviation * 500;
    const auto distance = [](geom::PointMm a, geom::PointMm b) {
        const auto dx = static_cast<double>(a.x - b.x), dy = static_cast<double>(a.y - b.y);
        return det::sqrt(dx * dx + dy * dy);
    };
    for (int iteration = 0; iteration < 64; ++iteration) {
        auto next = smoothed; bool moved = false;
        for (std::size_t i = 1; i + 1 < smoothed.size(); ++i) {
            if (knots[i].junction) continue;
            const auto& a = smoothed[i - 1].p; const auto& p = smoothed[i].p; const auto& b = smoothed[i + 1].p;
            const auto gap = std::min(distance(a, p), distance(p, b)); if (gap < 4) continue;
            const auto dx = static_cast<double>(b.x - a.x), dy = static_cast<double>(b.y - a.y);
            const auto norm = dx * dx + dy * dy; if (norm == 0) continue;
            const auto t = std::clamp(((p.x - a.x) * dx + (p.y - a.y) * dy) / norm, 0.0, 1.0);
            double mx = 0.5 * (a.x + t * dx - p.x), my = 0.5 * (a.y + t * dy - p.y);
            const auto step = det::sqrt(mx * mx + my * my);
            if (step > gap * 0.25) { mx *= gap * 0.25 / step; my *= gap * 0.25 / step; }
            double cx = p.x + mx, cy = p.y + my;
            const auto ox = cx - knots[i].p.x, oy = cy - knots[i].p.y;
            const auto displacement = det::sqrt(ox * ox + oy * oy);
            if (displacement > budget_mm) {
                if (budget_mm <= 1) continue;
                cx = knots[i].p.x + ox * (budget_mm - 1) / displacement;
                cy = knots[i].p.y + oy * (budget_mm - 1) / displacement;
            }
            const geom::PointMm candidate{std::llround(cx), std::llround(cy)};
            if (candidate == p || candidate == a || candidate == b || distance(candidate, knots[i].p) > budget_mm) continue;
            next[i].p = candidate; moved = true;
        }
        if (!moved) break;
        // Quantisation must not create a zero-length segment.
        bool valid = true;
        for (std::size_t i = 1; i < next.size(); ++i) if (next[i].p == next[i - 1].p) valid = false;
        if (!valid) break;
        smoothed = std::move(next); changed = true;
    }
    if (!changed) return original;
    return fit_geometry(smoothed, closed, id, radius, deviation * 0.5, limits, coincident_pairs);
}
}
Result<geom::CurvePose> RoadReferenceLine::at(double station) const {
    if (!std::isfinite(station) || station < 0 || station > length_m || segments.empty())
        return make_error(ErrorCode::InvalidArgument, "road reference: station outside line");
    auto it = std::upper_bound(segments.begin(), segments.end(), station,
        [](auto value, const auto& segment) { return value < segment.station_m; });
    if (it == segments.begin()) return make_error(ErrorCode::Malformed, "road reference: invalid station ordering");
    --it; return it->curve.at(std::clamp(station - it->station_m, 0.0, it->curve.length));
}
Result<RoadReferenceBuilder> RoadReferenceBuilder::make(const RoadGraph& graph) {
    const auto valid = validate_road_graph(graph); if (!valid) return valid.error();
    RoadReferenceBuilder out; out.graph_ = &graph; out.degree_.resize(graph.nodes.size());
    const auto index = [&](auto id) {
        return static_cast<std::size_t>(std::lower_bound(graph.nodes.begin(), graph.nodes.end(), id,
            [](const auto& node, auto search) { return node.osm_id < search; }) - graph.nodes.begin());
    };
    for (const auto& edge : graph.edges) { ++out.degree_[index(edge.from_node)]; ++out.degree_[index(edge.to_node)]; }
    return out;
}
Result<RoadReferenceLine> fit_road_reference(const RoadGraph& graph, std::int64_t id, const ReferenceFitLimits& limits) {
    const auto builder = RoadReferenceBuilder::make(graph); if (!builder) return builder.error();
    return builder->fit(id, limits);
}
Result<std::vector<RoadStretch>> RoadReferenceBuilder::stretches(std::int64_t id) const {
    const auto& graph = *graph_;
    const auto wi = std::lower_bound(graph.ways.begin(), graph.ways.end(), id,
        [](const auto& way, auto search) { return way.osm_id < search; });
    if (wi == graph.ways.end() || wi->osm_id != id) return make_error(ErrorCode::NotFound, "road reference: way absent");
    std::vector<RoadStretch> out; std::size_t start = 0;
    for (std::size_t end = 1; end < wi->node_ids.size(); ++end) {
        const auto ni = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), wi->node_ids[end],
            [](const auto& node, auto search) { return node.osm_id < search; });
        if (end + 1 < wi->node_ids.size() && degree_[static_cast<std::size_t>(ni - graph.nodes.begin())] == 2) continue;
        if (out.size() == 100000) return make_error(ErrorCode::LimitExceeded, "road reference: stretch count budget");
        out.push_back({static_cast<std::uint32_t>(start), static_cast<std::uint32_t>(end)}); start = end;
    }
    return out;
}
Result<RoadReferenceLine> RoadReferenceBuilder::fit_stretch(std::int64_t id, RoadStretch stretch, const ReferenceFitLimits& limits) const {
    const auto& graph = *graph_;
    const auto wi = std::lower_bound(graph.ways.begin(), graph.ways.end(), id,
        [](const auto& way, auto search) { return way.osm_id < search; });
    if (wi == graph.ways.end() || wi->osm_id != id) return make_error(ErrorCode::NotFound, "road reference: way absent");
    if (stretch.start_ref >= stretch.end_ref || stretch.end_ref >= wi->node_ids.size())
        return make_error(ErrorCode::InvalidArgument, "road reference: invalid stretch range");
    const auto attrs = resolve_road_attributes(*wi); if (!attrs) return attrs.error();
    const auto radius = limits.min_radius_m == 0 ? attrs->min_horizontal_radius_m : limits.min_radius_m;
    const auto deviation = limits.max_deviation_m == 0 ? (wi->rank >= 5 ? 0.5 : 3.0) : limits.max_deviation_m;
    if (!std::isfinite(radius) || radius < 1 || radius > 100000 || !std::isfinite(deviation) || deviation <= 0 ||
        deviation > 100 || !limits.max_segments || limits.max_segments > 100000 ||
        !std::isfinite(limits.max_bow_m) || limits.max_bow_m <= 0 || limits.max_bow_m > 100 ||
        !std::isfinite(limits.max_length_m) || limits.max_length_m <= 0 || limits.max_length_m > 100000)
        return make_error(ErrorCode::InvalidArgument, "road reference: fitting limits");
    std::vector<Knot> knots; std::size_t coincident = 0;
    for (std::size_t i = stretch.start_ref; i <= stretch.end_ref; ++i) {
        const auto ni = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), wi->node_ids[i],
            [](const auto& node, auto search) { return node.osm_id < search; });
        const bool junction = degree_[static_cast<std::size_t>(ni - graph.nodes.begin())] != 2;
        if ((i == stretch.start_ref && i != 0 && !junction) ||
            (i == stretch.end_ref && i + 1 != wi->node_ids.size() && !junction) ||
            (i != stretch.start_ref && i != stretch.end_ref && junction))
            return make_error(ErrorCode::InvalidArgument, "road reference: incomplete topological stretch");
        if (!knots.empty() && knots.back().p == ni->position) {
            ++coincident; knots.back().index = static_cast<std::uint32_t>(i);
            knots.back().junction = knots.back().junction || junction; continue;
        }
        knots.push_back({ni->position, static_cast<std::uint32_t>(i), junction});
        if (knots.size() > limits.max_segments + 1)
            return make_error(ErrorCode::LimitExceeded, "road reference: segment budget");
    }
    if (knots.size() < 2) return make_error(ErrorCode::Malformed, "road reference: all points coincide");
    const bool closed = wi->node_ids[stretch.start_ref] == wi->node_ids[stretch.end_ref] && knots.size() > 3 &&
        std::none_of(knots.begin(), knots.end(), [](const auto& knot) { return knot.junction; });
    auto out = fit_shape(knots, closed, id, radius, deviation, limits, coincident);
    double fitted_radius = radius;
    // Only geometric infeasibility permits a retry. Resource/invalid-data errors
    // must never be hidden by the exception policy. Each retry keeps the original
    // corridor, junctions, bow and construction budgets unchanged.
    const auto retryable = [](const Error& error) {
        return error.code == ErrorCode::LimitExceeded &&
            (error.message == "road reference: rounded corner corridor" ||
             error.message == "road reference: overlapping rounded corners");
    };
    for (unsigned retry = 0; limits.allow_curvature_exceptions && !out &&
         retryable(out.error()) && fitted_radius > 1 && retry < 17; ++retry) {
        fitted_radius = std::max(1.0, fitted_radius * 0.5);
        out = fit_shape(knots, closed, id, fitted_radius, deviation, limits, coincident);
    }
    if (!out) return out.error();
    out->requested_min_radius_m = radius;
    out->fitted_min_radius_m = fitted_radius;
    out->curvature_exception = fitted_radius < radius;
    out->anchors = {{wi->node_ids[stretch.start_ref], 0}, {wi->node_ids[stretch.end_ref], out->length_m}};
    return out;
}
Result<std::vector<RoadStretchFit>> RoadReferenceBuilder::fit_stretches(std::int64_t id, const ReferenceFitLimits& limits) const {
    const auto ranges = stretches(id); if (!ranges) return ranges.error();
    if (ranges->size() > limits.max_segments) return make_error(ErrorCode::LimitExceeded, "road reference: complete-way budget");
    std::vector<RoadStretchFit> out; out.reserve(ranges->size());
    std::size_t segments = 0; double length = 0;
    for (const auto& range : *ranges) {
        auto reference = fit_stretch(id, range, limits);
        if (reference) {
            segments += reference->segments.size(); length += reference->length_m;
            if (segments > limits.max_segments || length > limits.max_length_m)
                return make_error(ErrorCode::LimitExceeded, "road reference: complete-way budget");
        }
        out.push_back({range, std::move(reference)});
    }
    return out;
}
Result<RoadReferenceLine> RoadReferenceBuilder::fit(std::int64_t id, const ReferenceFitLimits& limits) const {
    const auto ranges = stretches(id); if (!ranges) return ranges.error();
    RoadReferenceLine out; out.way_id = id;
    for (const auto& range : *ranges) {
        auto part = fit_stretch(id, range, limits); if (!part) return part.error();
        if (out.segments.size() + part->segments.size() > limits.max_segments || out.length_m + part->length_m > limits.max_length_m)
            return make_error(ErrorCode::LimitExceeded, "road reference: complete-way budget");
        if (out.anchors.empty()) out.anchors.push_back(part->anchors.front());
        bool first = true;
        for (auto& segment : part->segments) {
            segment.station_m += out.length_m; segment.break_before = first && !out.segments.empty();
            out.segments.push_back(segment); first = false;
        }
        out.length_m += part->length_m; out.coincident_pairs += part->coincident_pairs;
        out.curvature_exception = out.curvature_exception || part->curvature_exception;
        out.requested_min_radius_m = part->requested_min_radius_m;
        out.fitted_min_radius_m = out.fitted_min_radius_m == 0 ? part->fitted_min_radius_m :
            std::min(out.fitted_min_radius_m, part->fitted_min_radius_m);
        out.anchors.push_back({part->anchors.back().node_id, out.length_m});
    }
    return out;
}
}
