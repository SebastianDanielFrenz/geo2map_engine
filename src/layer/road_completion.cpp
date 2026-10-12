#include "g2m/layer/road_completion.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <map>
namespace g2m {
namespace {
// Read access to a RoadGraph for the walk: nodes and ways by id, the edges incident to a node in graph.edges order
// (ascending way id, segment index: the order every tie of the walk breaks in).
class Walk {
public:
    Walk(const RoadGraph& graph, const RoadDemSampler& dem, geo::UtmZone zone, const RoadCompletionOptions& options)
        : graph_(graph), dem_(dem), zone_(zone), options_(options) {
        incident_.reserve(graph.edges.size() * 2);
        for (std::uint32_t i = 0; i < graph.edges.size(); ++i) {
            incident_.push_back({graph.edges[i].from_node, i});
            incident_.push_back({graph.edges[i].to_node, i});
        }
        std::sort(incident_.begin(), incident_.end());
    }
    const geom::PointMm& position(std::int64_t id) const {
        return std::lower_bound(graph_.nodes.begin(), graph_.nodes.end(), id,
            [](const auto& n, auto v) { return n.osm_id < v; })->position;
    }
    const RoadGraphWay* way(std::int64_t id) const {
        const auto it = std::lower_bound(graph_.ways.begin(), graph_.ways.end(), id,
            [](const auto& w, auto v) { return w.osm_id < v; });
        return it == graph_.ways.end() || it->osm_id != id ? nullptr : &*it;
    }
    // The resolved attributes of a way (nullptr when it does not resolve), memoised.
    const RoadAttributes* attributes(std::int64_t way_id) {
        const auto it = attrs_.find(way_id);
        if (it != attrs_.end()) return it->second ? &*it->second : nullptr;
        const auto* w = way(way_id);
        std::optional<RoadAttributes> value;
        if (w) { auto a = resolve_road_attributes(*w); if (a) value = *a; }
        return (attrs_[way_id] = std::move(value)) ? &*attrs_[way_id] : nullptr;
    }
    // Edge indices incident to a node, ascending.
    template <class F> void for_each_edge(std::int64_t node, F&& f) const {
        for (auto it = std::lower_bound(incident_.begin(), incident_.end(), std::pair<std::int64_t, std::uint32_t>{node, 0});
             it != incident_.end() && it->first == node; ++it) f(graph_.edges[it->second]);
    }
    std::optional<RoadJunctionPlane> abutment(std::int64_t node, double out_x, double out_y);
    RoadBridgeAnchor anchor(std::int64_t node);
    const RoadGraph& graph() const { return graph_; }
private:
    const RoadGraph& graph_;
    const RoadDemSampler& dem_;
    geo::UtmZone zone_;
    const RoadCompletionOptions& options_;
    std::vector<std::pair<std::int64_t, std::uint32_t>> incident_;
    std::map<std::int64_t, std::optional<RoadAttributes>> attrs_;
    std::map<std::int64_t, RoadBridgeAnchor> anchors_;
};

// racing_game's `abutment`, bounded: supported ground away from the bare-earth hole under a structure. Degree-two
// bridges can be split over several OSM ways, so the walk continues along the most aligned ground approach and
// through bridge edges. Differences from the game: libm-free (det::sqrt), and the TOTAL path (bridge edges included)
// is bounded by options.abutment_path_m, so every node read and every DEM sample lies within that distance of `node`
// (the 65 m margin of road_dem_dependencies and the 128 m halo of the node tiles in the context cover it).
std::optional<RoadJunctionPlane> Walk::abutment(std::int64_t node, double out_x, double out_y) {
    const auto origin = position(node);
    auto current = node; std::int64_t previous = 0;
    double remaining = options_.abutment_ground_m, path = 0;
    for (int work = 0; work < 100; ++work) {
        const RoadGraphEdge* selected = nullptr; double score = 0.25;
        for_each_edge(current, [&](const RoadGraphEdge& edge) {
            const auto other = edge.from_node == current ? edge.to_node : edge.from_node;
            if (other == previous) return;
            const auto* a = attributes(edge.way_id);
            if (!a || a->tunnel) return;
            const auto p = position(current), q = position(other);
            const double dx = static_cast<double>(q.x - p.x) / 1000.0, dy = static_cast<double>(q.y - p.y) / 1000.0;
            const double len = det::sqrt(dx * dx + dy * dy);
            if (len < 0.001) return;
            const double dot = (dx * out_x + dy * out_y) / len;
            if (dot > score) { score = dot; selected = &edge; }
        });
        if (!selected) return std::nullopt;
        const auto other = selected->from_node == current ? selected->to_node : selected->from_node;
        const auto a = position(current), b = position(other);
        const double dx = static_cast<double>(b.x - a.x) / 1000.0, dy = static_cast<double>(b.y - a.y) / 1000.0;
        const double len = det::sqrt(dx * dx + dy * dy);
        const bool on_bridge = attributes(selected->way_id)->bridge;
        if (!on_bridge && len >= remaining) {
            if (path + remaining > options_.abutment_path_m) return std::nullopt;
            const double t = remaining / len;
            const geom::PointMm support{std::llround(static_cast<double>(a.x) + t * static_cast<double>(b.x - a.x)),
                std::llround(static_cast<double>(a.y) + t * static_cast<double>(b.y - a.y))};
            auto plane = dem_.junction_plane(zone_, support); if (!plane) return std::nullopt;
            const double h = plane->height / 256.0 + plane->grade_x * static_cast<double>(origin.x - support.x) / 1000.0 +
                plane->grade_y * static_cast<double>(origin.y - support.y) / 1000.0;
            if (!std::isfinite(h) || std::fabs(h * 256.0) > 2147483000.0) return std::nullopt;
            plane->height = static_cast<std::int32_t>(std::llround(h * 256.0));
            return *plane;
        }
        if (path + len > options_.abutment_path_m) return std::nullopt;   // the next node would lie beyond the budget
        if (!on_bridge) remaining -= len;
        path += len; out_x = dx / len; out_y = dy / len; previous = current; current = other;
    }
    return std::nullopt;
}

RoadBridgeAnchor Walk::anchor(std::int64_t node) {
    const auto cached = anchors_.find(node);
    if (cached != anchors_.end()) return cached->second;
    RoadBridgeAnchor result;
    const RoadGraphEdge* first = nullptr;
    double out_x = 0, out_y = 0;
    for_each_edge(node, [&](const RoadGraphEdge& edge) {
        if (first) return;
        const auto* a = attributes(edge.way_id);
        if (!a || !a->bridge || a->tunnel) return;
        const auto other = edge.from_node == node ? edge.to_node : edge.from_node;
        const auto p = position(node), q = position(other);
        // Away from the bridge: from its neighbour node to this one.
        const double dx = static_cast<double>(p.x - q.x) / 1000.0, dy = static_cast<double>(p.y - q.y) / 1000.0;
        const double len = det::sqrt(dx * dx + dy * dy);
        if (len < 0.001) return;
        first = &edge; out_x = dx / len; out_y = dy / len;
    });
    if (first) { result.bridging = true; result.plane = abutment(node, out_x, out_y); }
    return anchors_[node] = result;
}

// D43: the largest |end grade - chord| the ease accepts. The deck-edge misread it exists for is a few per cent (the A 648
// underpass: 0.021); the bumpy test terrain's genuine departures start near 0.10 (G3-D, design 6.6).
constexpr double kGroundEaseMismatch = 0.05;
bool contains(const std::string& text, const char* part) { return text.find(part) != std::string::npos; }
// fit_vertical_profile's two declines that only the fixed end values cause (heights and grades at both ends).
bool boundary_infeasible(const std::string& message) {
    return contains(message, "endpoint curvature infeasible") || contains(message, "fixed boundary constraints infeasible");
}
double project(const RoadJunctionPlane& plane, double heading) {
    return plane.grade_x * det::cos(heading) + plane.grade_y * det::sin(heading);
}
}

RoadBridgeAnchor road_bridge_anchor(const RoadGraph& graph, const RoadDemSampler& dem, geo::UtmZone zone,
    std::int64_t node, const RoadCompletionOptions& options) {
    Walk walk(graph, dem, zone, options);
    return walk.anchor(node);
}

Result<void> complete_road_profiles(std::vector<RoadGeomEntry>& entries, std::vector<RoadProfileOrigin>& origins,
    const RoadReferenceBuilder& builder, const RoadDemSampler& dem, geo::UtmZone zone,
    const RoadProfileLimits& limits, const RoadCompletionOptions& options, RoadCompletionStats* stats) {
    RoadCompletionStats local; if (!stats) stats = &local;
    origins.assign(entries.size(), RoadProfileOrigin{});
    if (!options.enabled) return {};
    if (!std::isfinite(options.abutment_ground_m) || !std::isfinite(options.abutment_path_m) || options.abutment_ground_m <= 0 ||
        options.abutment_path_m < options.abutment_ground_m || options.abutment_path_m > kRoadCompletionMaxAbutmentPathM || options.retry_iterations < 1 ||
        options.retry_iterations > 16384 || options.retry_spacing_m < 0.5 || options.retry_spacing_m > 100 ||
        options.ground_spacing_m < 0.5 || options.ground_spacing_m > 100 || !std::isfinite(options.ground_ramp_m) || options.ground_ramp_m < 0)
        return make_error(ErrorCode::InvalidArgument, "road completion: options");
    const auto& graph = builder.graph();
    const auto way_of = [&](std::int64_t id) {
        const auto it = std::lower_bound(graph.ways.begin(), graph.ways.end(), id, [](const auto& w, auto v) { return w.osm_id < v; });
        return it == graph.ways.end() || it->osm_id != id ? nullptr : &*it;
    };
    ReferenceFitLimits reference = limits.reference; reference.allow_curvature_exceptions = true;
    const auto mark = [&](std::size_t i, ProfileProvenance kind) {
        origins[i] = {kind, static_cast<std::uint16_t>(entries[i].decline->code)};
        entries[i].decline.reset();
    };

    // Step 1: the game's retry. Explicit numerical declines are re-run at 20 m spacing and 16384 iterations with the
    // same certified limits.
    for (std::size_t i = 0; i < entries.size(); ++i) {
        auto& entry = entries[i]; if (entry.profile || !entry.decline) continue;
        if (!contains(entry.decline->message, "bounded solver") && !contains(entry.decline->message, "endpoint curvature")) continue;
        ++stats->retry_candidates;
        RoadProfileLimits retry = limits; retry.sample_spacing_m = options.retry_spacing_m; retry.max_iterations = options.retry_iterations;
        retry.reference = reference;
        auto profile = build_road_profile(builder, dem, zone, entry.way_id, entry.stretch, retry);
        if (!profile) continue;
        ++stats->retry_accepted;
        entry.profile = std::move(*profile); mark(i, ProfileProvenance::Retry);
    }

    Walk walk(graph, dem, zone, options);
    // The anchor of an end node (decision D30): at a bridging node the strict pass's own anchor where the strict pass has
    // one (road_strict_anchor: the supported approach plane with its <= 0.25 m feasibility correction), so a completed
    // profile meets every strict profile ending there at the same height and grade plane; else the walk's abutment plane
    // (D25). A pure function of the node either way; memoised per call.
    std::map<std::int64_t, RoadBridgeAnchor> anchors;
    const auto anchor_of = [&](std::int64_t node) {
        const auto cached = anchors.find(node); if (cached != anchors.end()) return cached->second;
        auto result = walk.anchor(node);
        if (result.bridging) {
            const auto strict = road_strict_anchor(builder, dem, zone, node, limits.reference);
            if (strict && strict->inferred) { result.plane = strict->plane; ++stats->strict_anchors; }
        }
        return anchors[node] = result;
    };
    // Step 2: declined bridge spans, re-fitted between the abutment planes of their two end nodes at relaxed limits.
    for (std::size_t i = 0; i < entries.size(); ++i) {
        auto& entry = entries[i]; if (entry.profile || !entry.decline) continue;
        const auto* w = way_of(entry.way_id); if (!w) continue;
        const auto* attrs = walk.attributes(entry.way_id); if (!attrs) continue;
        if (attrs->tunnel) { ++stats->tunnel_skipped; continue; }
        if (!attrs->bridge) continue;
        ++stats->bridge_candidates;
        auto ref = builder.fit_stretch(entry.way_id, entry.stretch, reference); if (!ref) { ++stats->bridge_fit_failed; continue; }
        const auto start = ref->at(0), end = ref->at(ref->length_m); if (!start || !end) { ++stats->bridge_fit_failed; continue; }
        const auto a = anchor_of(w->node_ids[entry.stretch.start_ref]), b = anchor_of(w->node_ids[entry.stretch.end_ref]);
        if (!a.plane || !b.plane) { ++stats->bridge_no_support; continue; }
        VerticalFitLimits fit; fit.max_grade = std::max(0.12, attrs->max_grade_per_mille / 1000.0);
        fit.min_radius_m = std::min<double>(250, attrs->min_vertical_radius_m); fit.max_iterations = 16384;
        fit.start_grade = project(*a.plane, start->heading); fit.end_grade = project(*b.plane, end->heading);
        const std::array<VerticalSample, 2> samples{{{0, a.plane->height / 256.0}, {ref->length_m, b.plane->height / 256.0}}};
        auto vertical = fit_vertical_profile(samples, fit);
        if (!vertical) {
            // Estimated DEM gradients at a short span can disagree. The abutment rise stays measured: the common secant grade.
            fit.start_grade = fit.end_grade = static_cast<double>(b.plane->height - a.plane->height) / 256.0 / ref->length_m;
            vertical = fit_vertical_profile(samples, fit);
            if (!vertical) { ++stats->bridge_fit_failed; continue; }
        }
        RoadProfile p; p.stretch = entry.stretch; p.attributes = *attrs; p.reference = std::move(*ref); p.vertical = std::move(*vertical);
        p.start_anchor_inferred = p.end_anchor_inferred = true;
        entry.profile = std::move(p); mark(i, ProfileProvenance::BridgeCompleted); ++stats->bridge_completed;
    }

    // Step 3: still-declined ground stretches, re-fitted at relaxed limits, 40 m samples and 32 m ramps from the
    // bridge abutment planes at their ends (the DEM plane at the node where no bridge ends).
    for (std::size_t i = 0; i < entries.size(); ++i) {
        auto& entry = entries[i]; if (entry.profile || !entry.decline) continue;
        const auto* w = way_of(entry.way_id); if (!w) continue;
        const auto* attrs = walk.attributes(entry.way_id); if (!attrs || attrs->bridge || attrs->tunnel) continue;
        const auto first_id = w->node_ids[entry.stretch.start_ref], last_id = w->node_ids[entry.stretch.end_ref];
        ++stats->ground_candidates;
        const auto fail = [&] { ++stats->ground_failed; };
        auto ref = builder.fit_stretch(entry.way_id, entry.stretch, reference); if (!ref || ref->length_m < 0.5) { fail(); continue; }
        const auto anchor_first = anchor_of(first_id), anchor_last = anchor_of(last_id);
        const bool bridge_first = anchor_first.plane.has_value(), bridge_last = anchor_last.plane.has_value();
        const auto plane_at = [&](const RoadBridgeAnchor& anchor, std::int64_t id) -> std::optional<RoadJunctionPlane> {
            if (anchor.plane) return anchor.plane;
            auto p = dem.junction_plane(zone, walk.position(id)); if (!p) return std::nullopt; return *p;
        };
        const auto a = plane_at(anchor_first, first_id), b = plane_at(anchor_last, last_id);
        if (!a || !b) { fail(); continue; }
        const auto start = ref->at(0), end = ref->at(ref->length_m); if (!start || !end) { fail(); continue; }
        VerticalFitLimits fit; fit.max_grade = std::max(0.12, attrs->max_grade_per_mille / 1000.0);
        fit.min_radius_m = std::min<double>(500, attrs->min_vertical_radius_m); fit.max_iterations = 16384;
        fit.start_grade = std::clamp(project(*a, start->heading), -0.10, 0.10);
        fit.end_grade = std::clamp(project(*b, end->heading), -0.10, 0.10);
        const auto count = static_cast<std::size_t>(std::max(1.0, std::ceil(ref->length_m / options.ground_spacing_m)));
        if (count > 2047) { fail(); continue; }
        std::vector<VerticalSample> samples{{0, a->height / 256.0}}; bool complete = true;
        for (std::size_t k = 1; k < count; ++k) {
            const double station = ref->length_m * static_cast<double>(k) / static_cast<double>(count);
            const auto xy = ref->at(station); if (!xy) { complete = false; break; }
            const auto height = dem.junction_height(zone, {std::llround(xy->x * 1000), std::llround(xy->y * 1000)});
            if (!height) { complete = false; break; }
            double z = *height / 256.0;
            // Carry the abutments to supported ground, keeping the ramps out of the DEM's underpass depression.
            if (bridge_first && station < options.ground_ramp_m) z = a->height / 256.0 + fit.start_grade * station;
            if (bridge_last && ref->length_m - station < options.ground_ramp_m) z = b->height / 256.0 - fit.end_grade * (ref->length_m - station);
            samples.push_back({station, z});
        }
        if (!complete) { fail(); continue; }
        samples.push_back({ref->length_m, b->height / 256.0});
        auto vertical = fit_vertical_profile(samples, fit);
        // D43: end planes that contradict the stretch's own measured rise (a node at the edge of a deck, where the bare
        // earth is interpolated and catches up within metres; a short way split between two such reads) leave no cubic
        // under the curvature bound. The end heights stay measured; both end grades are eased toward the chord by the
        // same fraction k/8, the smallest k that fits (k = 8 is the chord itself, as the bridge step's fallback above).
        // Only a local misread is eased: an end plane further than kGroundEaseMismatch from the chord is terrain that
        // really departs from the stretch (a road forced onto the chord there rides off it), so it stays declined.
        if (!vertical && boundary_infeasible(vertical.error().message)) {
            const double chord = static_cast<double>(b->height - a->height) / 256.0 / ref->length_m;
            const double g0 = fit.start_grade, g1 = fit.end_grade;
            const bool near_chord = std::fabs(chord) <= fit.max_grade && std::fabs(g0 - chord) <= kGroundEaseMismatch && std::fabs(g1 - chord) <= kGroundEaseMismatch;
            for (int k = 1; near_chord && k <= 8; ++k) {
                const double t = static_cast<double>(k) / 8.0;
                fit.start_grade = g0 + t * (chord - g0); fit.end_grade = g1 + t * (chord - g1);
                vertical = fit_vertical_profile(samples, fit);
                if (vertical) { ++stats->ground_grades_eased; break; }
                if (!boundary_infeasible(vertical.error().message)) break;
            }
        }
        if (!vertical) { fail(); continue; }
        RoadProfile p; p.stretch = entry.stretch; p.attributes = *attrs; p.reference = std::move(*ref); p.vertical = std::move(*vertical);
        p.start_anchor_inferred = bridge_first; p.end_anchor_inferred = bridge_last;
        entry.profile = std::move(p); mark(i, ProfileProvenance::GroundCompleted);
        ++stats->ground_completed; if (bridge_first || bridge_last) ++stats->ground_bridge_anchored;
    }
    return {};
}
}
