#include "g2m/layer/road_completion.h"
#include "g2m/layer/road_underpass.h"
#include "g2m/layer/portal_approach.h"
#include "g2m/layer/structure_floor.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <deque>
#include <limits>
#include <map>
#include <set>
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
    RoadBridgeAnchor anchor(std::int64_t node, bool* other_side = nullptr);
    const RoadGraph& graph() const { return graph_; }
private:
    const RoadGraph& graph_;
    const RoadDemSampler& dem_;
    geo::UtmZone zone_;
    const RoadCompletionOptions& options_;
    std::vector<std::pair<std::int64_t, std::uint32_t>> incident_;
    std::map<std::int64_t, std::optional<RoadAttributes>> attrs_;
    std::map<std::int64_t, std::pair<RoadBridgeAnchor, bool>> anchors_;
};

// racing_game's `abutment`, bounded: supported ground away from the bare-earth hole under a structure. Degree-two
// bridges can be split over several OSM ways, so the walk continues along the most aligned ground approach and
// through bridge edges. Differences from the game: libm-free (det::sqrt), and the TOTAL path (bridge edges included)
// is bounded by options.abutment_path_m, so every node read and every DEM sample lies within that distance of `node`
// (the 205 m squares road_bridge_dependencies adds around every node of the incident bridges cover it, D48).
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

RoadBridgeAnchor Walk::anchor(std::int64_t node, bool* other_side) {
    if (other_side) *other_side = false;
    const auto cached = anchors_.find(node);
    if (cached != anchors_.end()) return cached->second.first;
    RoadBridgeAnchor result;
    bool later = false;
    // Away from each incident bridge edge in (way id, segment) order: from its neighbour node to this one. D25 walked
    // away from the first edge only; D47 (bridge_anchor_reuse) takes the first direction that finds support, so a
    // short bridge way next to a long one is crossed instead of the long one (the game walks both, one per stretch end).
    for_each_edge(node, [&](const RoadGraphEdge& edge) {
        if (result.plane || (result.bridging && !options_.bridge_anchor_reuse)) return;
        const auto* a = attributes(edge.way_id);
        if (!a || !a->bridge || a->tunnel) return;
        const auto other = edge.from_node == node ? edge.to_node : edge.from_node;
        const auto p = position(node), q = position(other);
        const double dx = static_cast<double>(p.x - q.x) / 1000.0, dy = static_cast<double>(p.y - q.y) / 1000.0;
        const double len = det::sqrt(dx * dx + dy * dy);
        if (len < 0.001) return;
        later = result.bridging;
        result.bridging = true; result.plane = abutment(node, dx / len, dy / len);
    });
    const bool found_later = later && result.plane.has_value();
    anchors_[node] = {result, found_later};
    if (other_side) *other_side = found_later;
    return result;
}

// D43: the largest |end grade - chord| the ease accepts. The deck-edge misread it exists for is a few per cent (the A 648
// underpass: 0.021); the bumpy test terrain's genuine departures start near 0.10 (G3-D, design 6.6).
constexpr double kGroundEaseMismatch = 0.05;
bool contains(const std::string& text, const char* part) { return text.find(part) != std::string::npos; }
// fit_vertical_profile's two declines that only the fixed end values cause (heights and grades at both ends).
bool boundary_infeasible(const std::string& message) {
    return contains(message, "endpoint curvature infeasible") || contains(message, "fixed boundary constraints infeasible");
}
// 7a-1: how far (in grade) a straight portal-to-portal floor may leave the ground road's grade at a portal. The
// continuity requirement is 0.02; the margin keeps the stored (quantised, projected) grades inside it.
constexpr double kPortalGradeSlack = 0.015;
// Share of tunnel_max_grade a tunnel floor is asked to climb between two anchored portals (the rest is the curvature's room).
constexpr double kPortalReach = 0.9;
double project(const RoadJunctionPlane& plane, double heading) {
    return plane.grade_x * det::cos(heading) + plane.grade_y * det::sin(heading);
}
}

RoadBridgeAnchor road_bridge_anchor(const RoadGraph& graph, const RoadDemSampler& dem, geo::UtmZone zone,
    std::int64_t node, const RoadCompletionOptions& options) {
    // bridge_anchor_reuse = false reproduces D25 (first incident bridge edge only).
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
        options.ground_spacing_m < 0.5 || options.ground_spacing_m > 100 || !std::isfinite(options.ground_ramp_m) || options.ground_ramp_m < 0 ||
        !(options.tunnel_spacing_m >= 0.5 && options.tunnel_spacing_m <= 100) ||
        !(options.tunnel_max_grade > 0 && options.tunnel_max_grade <= kRoadCompletionMaxTunnelGrade) ||
        !(options.tunnel_min_radius_m >= kRoadCompletionMinTunnelRadiusM && options.tunnel_min_radius_m <= 1e6) ||
        !(options.tunnel_chord_min_length_m >= 0 && options.tunnel_chord_min_length_m <= 1e6) ||
        !(options.tunnel_cut_max_length_m >= 0 && options.tunnel_cut_max_length_m <= 1e6) ||
        !(options.tunnel_cut_hump_min_m >= 0 && options.tunnel_cut_hump_min_m <= options.tunnel_cut_hump_max_m && options.tunnel_cut_hump_max_m <= 1e6) ||
        (options.tunnel_design_floor && !(options.tunnel_floors && options.tunnel_portal_anchor &&
            options.structure_clearance_m >= 1 && options.structure_clearance_m <= 20 && options.structure_roof_m >= 0 && options.structure_roof_m <= 10 &&
            options.structure_max_grade > 0 && options.structure_max_grade <= kRoadCompletionMaxTunnelGrade)))
        return make_error(ErrorCode::InvalidArgument, "road completion: options");
    const auto& graph = builder.graph();
    const auto way_of = [&](std::int64_t id) {
        const auto it = std::lower_bound(graph.ways.begin(), graph.ways.end(), id, [](const auto& w, auto v) { return w.osm_id < v; });
        return it == graph.ways.end() || it->osm_id != id ? nullptr : &*it;
    };
    ReferenceFitLimits reference = limits.reference; reference.allow_curvature_exceptions = true;
    const auto mark = [&](std::size_t i, ProfileProvenance kind) {
        if (!entries[i].decline) { origins[i].provenance = kind; return; }   // a retry-accepted stretch re-fitted to a moved portal plane keeps its strict decline code
        origins[i] = {kind, static_cast<std::uint16_t>(entries[i].decline->code)};
        entries[i].decline.reset();
    };
    const auto retryable = [](const Error& e) { return contains(e.message, "bounded solver") || contains(e.message, "endpoint curvature"); };
    RoadProfileLimits retry = limits; retry.sample_spacing_m = options.retry_spacing_m; retry.max_iterations = options.retry_iterations;
    retry.reference = reference;

    // Step 1: the game's retry. Explicit numerical declines are re-run at 20 m spacing and 16384 iterations with the
    // same certified limits.
    for (std::size_t i = 0; i < entries.size(); ++i) {
        auto& entry = entries[i]; if (entry.profile || !entry.decline) continue;
        if (!retryable(*entry.decline)) continue;
        ++stats->retry_candidates;
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
        bool other_side = false;
        auto result = walk.anchor(node, &other_side);
        if (result.bridging) {
            const auto strict = road_strict_anchor(builder, dem, zone, node, limits.reference);
            if (strict && strict->inferred) { result.plane = strict->plane; ++stats->strict_anchors; }
            else if (other_side) ++stats->walk_other_side;
        }
        return anchors[node] = result;
    };
    // A bridge span fitted between two abutment planes at relaxed limits, with the common secant grade as fallback.
    const auto fit_bridge = [&](RoadStretch stretch, const RoadAttributes& attrs, RoadReferenceLine ref,
                                const RoadJunctionPlane& a, const RoadJunctionPlane& b) -> std::optional<RoadProfile> {
        const auto start = ref.at(0), end = ref.at(ref.length_m); if (!start || !end) return std::nullopt;
        VerticalFitLimits fit; fit.max_grade = std::max(0.12, attrs.max_grade_per_mille / 1000.0);
        fit.min_radius_m = std::min<double>(250, attrs.min_vertical_radius_m); fit.max_iterations = 16384;
        fit.start_grade = project(a, start->heading); fit.end_grade = project(b, end->heading);
        const std::array<VerticalSample, 2> samples{{{0, a.height / 256.0}, {ref.length_m, b.height / 256.0}}};
        auto vertical = fit_vertical_profile(samples, fit);
        if (!vertical) {
            // Estimated DEM gradients at a short span can disagree. The abutment rise stays measured: the common secant grade.
            fit.start_grade = fit.end_grade = static_cast<double>(b.height - a.height) / 256.0 / ref.length_m;
            vertical = fit_vertical_profile(samples, fit);
            if (!vertical) return std::nullopt;
        }
        RoadProfile p; p.stretch = stretch; p.attributes = attrs; p.reference = std::move(ref); p.vertical = std::move(*vertical);
        p.start_anchor_inferred = p.end_anchor_inferred = true;
        return p;
    };
    // D47, the game's anchor map (O3 of the terrain migration plan): the end plane an ACCEPTED bridge stretch ending at a
    // node leaves there (its end height, its end grade along its end heading) is authoritative for the stretches that
    // continue from it. Kept tile-invariant (D25): the accepted outcome of a bridge stretch is recomputed from the node,
    // never read from this tile's entries - strict, else the retry, else the step-2 span between the base anchors of its
    // two end nodes (one level: no accepted plane feeds another). The bridges incident to the tile's nodes are in the
    // context with their DEM and node graph tiles (road_bridge_dependencies, D35). The last accepted stretch in
    // (way id, start ref) order wins, as in the game's entry-ordered map. The asking bridge stretch itself is excluded.
    struct Accepted { std::int64_t way_id; std::uint32_t start_ref; std::optional<RoadJunctionPlane> plane; };
    std::map<std::int64_t, std::vector<Accepted>> accepted_cache;
    const auto accepted_at = [&](std::int64_t node) -> const std::vector<Accepted>& {
        const auto cached = accepted_cache.find(node); if (cached != accepted_cache.end()) return cached->second;
        std::vector<Accepted> found;
        std::vector<std::int64_t> ways;
        walk.for_each_edge(node, [&](const RoadGraphEdge& edge) {
            const auto* a = walk.attributes(edge.way_id);
            if (a && a->bridge && !a->tunnel && (ways.empty() || ways.back() != edge.way_id)) ways.push_back(edge.way_id);
        });
        std::sort(ways.begin(), ways.end()); ways.erase(std::unique(ways.begin(), ways.end()), ways.end());
        for (const auto way_id : ways) {
            const auto* w = way_of(way_id); const auto* attrs = walk.attributes(way_id); if (!w || !attrs) continue;
            const auto stretches = builder.stretches(way_id); if (!stretches) continue;
            for (const auto stretch : *stretches) {
                const bool at_start = w->node_ids[stretch.start_ref] == node, at_end = w->node_ids[stretch.end_ref] == node;
                if (!at_start && !at_end) continue;
                Accepted item{way_id, stretch.start_ref, std::nullopt};
                std::optional<RoadProfile> profile;
                auto strict = build_road_profile(builder, dem, zone, way_id, stretch, limits);
                if (strict) profile = std::move(*strict);
                else if (retryable(strict.error())) { auto again = build_road_profile(builder, dem, zone, way_id, stretch, retry); if (again) profile = std::move(*again); }
                if (!profile) {
                    auto ref = builder.fit_stretch(way_id, stretch, reference);
                    const auto a = anchor_of(w->node_ids[stretch.start_ref]), b = anchor_of(w->node_ids[stretch.end_ref]);
                    if (ref && a.plane && b.plane) profile = fit_bridge(stretch, *attrs, std::move(*ref), *a.plane, *b.plane);
                }
                if (profile) {
                    // The game writes the start end first, then the far end: a closed span (both ends at the node) keeps the far end.
                    const double station = at_end ? profile->reference.length_m : 0;
                    const auto xy = profile->reference.at(station); const auto z = profile->at(station);
                    if (xy && z && std::isfinite(z->height_m) && std::fabs(z->height_m * 256.0) < 2147483000.0)
                        item.plane = RoadJunctionPlane{static_cast<std::int32_t>(std::llround(z->height_m * 256.0)),
                            z->grade * det::cos(xy->heading), z->grade * det::sin(xy->heading)};
                }
                found.push_back(std::move(item));
            }
        }
        return accepted_cache[node] = std::move(found);
    };
    std::set<std::int64_t> reused_nodes;
    // The plane at an end node: the accepted bridge plane (D47) ahead of the base anchor (D30/D25). For a ground stretch
    // (`ground`) a bridging node where no incident bridge stretch is accepted has no anchor: the stretch ends on the DEM
    // plane at the node, as in the game (its map holds accepted bridges only), not on the walk plane of a structure that
    // has no profile.
    const auto end_plane = [&](std::int64_t node, std::int64_t own_way, std::uint32_t own_start, bool ground) {
        auto base = anchor_of(node);
        if (!options.bridge_anchor_reuse || !base.bridging) return base;
        const auto& list = accepted_at(node);
        for (auto it = list.rbegin(); it != list.rend(); ++it) {
            if (it->way_id == own_way && it->start_ref == own_start) continue;
            if (!it->plane) continue;
            if (reused_nodes.insert(node).second) ++stats->bridge_anchor_reused;
            base.plane = it->plane; return base;
        }
        if (ground) base.plane.reset();
        return base;
    };

    // 7a-1 (docs/structure_classification.md section 5): the plane a tunnel stretch end meets at `node`, a pure function of
    // the node (memoised): (1) the end height and grade of an accepted ground profile (strict, else the 20 m retry) that
    // ends at the node, the first in (way id, start ref) order; (2) else the approach level of the joining ground way
    // (the lowest (way id, segment) edge): median DEM 2-8 m out along it with its measured grade; (3) else, where two
    // tunnel edges meet without a ground way, the junction plane (one value for every tunnel stretch there). nullopt:
    // a tunnel end nothing else joins (it keeps the DEM sample, as before). Reads: the ground ways incident to the node
    // with their DEM and stretch context (road_portal_dependencies), the DEM within 8.5 m of the node.
    // The ground ways at a portal are read WITHOUT the tile's underpass crossings (the crossing graph covers the tile's own
    // ways only, so a profile built here must not depend on it); a deck at a portal node itself is not expected.
    RoadProfileLimits portal_limits = limits, portal_retry = retry; portal_limits.underpass = nullptr; portal_retry.underpass = nullptr;
    // `away`: the grade of the joining ground road rising away from the node along it (rules 1 and 2); a tunnel end continues
    // that ROAD (its grade is -away leaving the node, +away arriving), which a bend at the node does not change; where
    // only the plane is known (rule 3) a tunnel end takes the plane's projection on its own heading.
    struct PortalAnchor { RoadJunctionPlane plane; std::optional<double> away; };
    std::map<std::int64_t, std::optional<PortalAnchor>> portal_cache;
    const auto portal_raw = [&](std::int64_t node) -> std::optional<PortalAnchor> {
        const auto cached = portal_cache.find(node); if (cached != portal_cache.end()) return cached->second;
        std::vector<const RoadGraphEdge*> ground; int tunnel_edges = 0;
        walk.for_each_edge(node, [&](const RoadGraphEdge& edge) {
            const auto* a = walk.attributes(edge.way_id); if (!a) return;
            if (a->tunnel) ++tunnel_edges; else if (!a->bridge) ground.push_back(&edge);
        });
        std::optional<PortalAnchor> result;
        std::int64_t previous_way = 0;
        for (const auto* edge : ground) {
            if (edge->way_id == previous_way) continue;
            previous_way = edge->way_id;
            const auto* w = way_of(edge->way_id); const auto stretches = builder.stretches(edge->way_id); if (!w || !stretches) continue;
            for (const auto stretch : *stretches) {
                const bool at_start = w->node_ids[stretch.start_ref] == node, at_end = w->node_ids[stretch.end_ref] == node;
                if (!at_start && !at_end) continue;
                auto profile = build_road_profile(builder, dem, zone, edge->way_id, stretch, portal_limits);
                if (!profile && retryable(profile.error())) profile = build_road_profile(builder, dem, zone, edge->way_id, stretch, portal_retry);
                if (!profile) continue;
                const double station = at_end ? profile->reference.length_m : 0;   // a closed stretch keeps the far end, as the bridge map
                const auto xy = profile->reference.at(station); const auto z = profile->at(station);
                if (!xy || !z || !std::isfinite(z->height_m) || std::fabs(z->height_m * 256.0) >= 2147483000.0) continue;
                result = PortalAnchor{RoadJunctionPlane{static_cast<std::int32_t>(std::llround(z->height_m * 256.0)),
                    z->grade * det::cos(xy->heading), z->grade * det::sin(xy->heading)}, at_end ? -z->grade : z->grade};
                ++stats->portal_ground; break;
            }
            if (result) break;
        }
        if (!result && !ground.empty()) {
            const auto* edge = ground.front(); const auto* w = way_of(edge->way_id);
            if (w) {
                // The way's nodes outward from the portal: along the edge's own direction away from `node`.
                std::vector<geom::PointMm> line;
                const auto push = [&](std::int64_t id) { const auto p = walk.position(id); if (line.empty() || !(line.back() == p)) line.push_back(p); };
                if (edge->from_node == node) for (std::size_t k = edge->segment_index; k < w->node_ids.size(); ++k) push(w->node_ids[k]);
                else for (std::size_t k = static_cast<std::size_t>(edge->segment_index) + 2; k-- > 0;) push(w->node_ids[k]);
                if (const auto approach = portal_approach(dem, zone, line)) {
                    const double g = std::clamp(approach->grade, -kPortalApproachMaxGrade, kPortalApproachMaxGrade);
                    result = PortalAnchor{RoadJunctionPlane{approach->height, g * approach->dir_x, g * approach->dir_y}, g};
                    ++stats->portal_approach;
                }
            }
        }
        if (!result && tunnel_edges >= 2) {
            if (const auto plane = dem.junction_plane(zone, walk.position(node))) { result = PortalAnchor{*plane, std::nullopt}; ++stats->portal_joint; }
        }
        return portal_cache[node] = result;
    };

    // 7a-1 reachability (review of ffd575d): a tunnel stretch whose two raw portal heights differ by more than its floor can climb
    // (kPortalReach of tunnel_max_grade over the stretch's reference length) cannot meet both. Neither the DEM floor nor a step is
    // allowed: the ground gives way. Each end moves toward the other by half the excess, the tunnel floor is fitted to the moved
    // heights, and the ground stretches joining there are re-fitted to the moved plane (step 3, the declined-stretch refit). A pure
    // function of the node: the tunnel stretches incident to it (graph), their reference lengths and the raw anchors of both ends.
    // A node on several tunnel stretches takes the largest shift.
    std::map<std::pair<std::int64_t, std::uint32_t>, std::optional<double>> tunnel_length_cache;
    const auto tunnel_length = [&](std::int64_t way_id, const RoadStretch& stretch) -> std::optional<double> {
        const auto key = std::make_pair(way_id, stretch.start_ref);
        const auto cached = tunnel_length_cache.find(key); if (cached != tunnel_length_cache.end()) return cached->second;
        std::optional<double> length;
        if (auto ref = builder.fit_stretch(way_id, stretch, reference); ref && ref->length_m >= 0.5) length = ref->length_m;
        return tunnel_length_cache[key] = length;
    };
    // 7a-2 (docs/structure_classification.md sections 3.2 and 5): the design floor of a connected tunnel component. The
    // component is the set of tunnel stretches joined at shared end nodes; its floor (structure_floor.h) is one function of
    // the component - the stretches in (way id, start ref) order, the portals (nodes where a non-tunnel edge joins, or a tunnel
    // end nothing continues), the anchors of the joining ground roads (portal_raw; an unjoined end is free and anchored on the
    // DEM sample at its node), the bare earth under every 1 m station - so every tile that reads any stretch of it solves the
    // same problem (road_component_dependencies puts all of it into the context) and gets the same bytes. Solved once per call.
    struct FloorComponent {
        bool ok = false;
        std::vector<RoadStretch> stretches; std::vector<std::int64_t> ways;            // canonical (way id, start ref) order
        std::vector<int> from, to; std::vector<double> length; std::vector<RoadReferenceLine> refs;
        std::vector<std::int64_t> nodes; std::map<std::int64_t, int> node_index;
        std::vector<int> tunnel_ends; std::vector<char> portal; std::vector<std::optional<PortalAnchor>> anchor;
        structure::NetworkFloor floor;
        std::map<std::int64_t, double> shift;                                          // anchored portal node -> the reach handler's move
        std::map<std::pair<std::int64_t, std::uint32_t>, std::size_t> index;
    };
    std::deque<FloorComponent> floor_components;
    std::map<std::pair<std::int64_t, std::uint32_t>, FloorComponent*> floor_of;
    const auto tunnel_ways_at = [&](std::int64_t node) {
        std::vector<std::int64_t> ways;
        walk.for_each_edge(node, [&](const RoadGraphEdge& edge) { const auto* a = walk.attributes(edge.way_id); if (a && a->tunnel) ways.push_back(edge.way_id); });
        std::sort(ways.begin(), ways.end()); ways.erase(std::unique(ways.begin(), ways.end()), ways.end());
        return ways;
    };
    const auto component_for = [&](std::int64_t way_id, const RoadStretch& first) -> FloorComponent* {
        const auto key0 = std::make_pair(way_id, first.start_ref);
        if (const auto it = floor_of.find(key0); it != floor_of.end()) return it->second;
        std::map<std::pair<std::int64_t, std::uint32_t>, RoadStretch> found{{key0, first}};
        std::vector<std::pair<std::int64_t, std::uint32_t>> work{key0};
        bool readable = true;
        while (!work.empty()) {
            const auto key = work.back(); work.pop_back();
            const auto* w = way_of(key.first); if (!w) { readable = false; continue; }
            const RoadStretch s = found.at(key);
            for (const auto id : {w->node_ids[s.start_ref], w->node_ids[s.end_ref]})
                for (const auto other : tunnel_ways_at(id)) {
                    const auto* ow = way_of(other); const auto st = builder.stretches(other); if (!ow || !st) { readable = false; continue; }
                    for (const auto& c : *st) {
                        if (ow->node_ids[c.start_ref] != id && ow->node_ids[c.end_ref] != id) continue;
                        if (found.emplace(std::make_pair(other, c.start_ref), c).second) work.push_back({other, c.start_ref});
                    }
                }
            if (found.size() > 4096) { readable = false; break; }
        }
        auto& c = floor_components.emplace_back(); c.ok = readable;
        for (const auto& [key, s] : found) { c.index[key] = c.ways.size(); c.ways.push_back(key.first); c.stretches.push_back(s); floor_of[key] = &c; }
        const auto node_of = [&](std::int64_t id) {
            const auto it = c.node_index.find(id); if (it != c.node_index.end()) return it->second;
            const int i = static_cast<int>(c.nodes.size()); c.nodes.push_back(id); c.node_index[id] = i; return i;
        };
        for (std::size_t i = 0; i < c.stretches.size(); ++i) {
            const auto* w = way_of(c.ways[i]);
            if (!w) { c.ok = false; c.from.push_back(0); c.to.push_back(0); continue; }
            c.from.push_back(node_of(w->node_ids[c.stretches[i].start_ref])); c.to.push_back(node_of(w->node_ids[c.stretches[i].end_ref]));
        }
        c.tunnel_ends.assign(c.nodes.size(), 0); c.portal.assign(c.nodes.size(), 0); c.anchor.assign(c.nodes.size(), std::nullopt);
        for (std::size_t i = 0; i < c.stretches.size(); ++i) { ++c.tunnel_ends[static_cast<std::size_t>(c.from[i])]; ++c.tunnel_ends[static_cast<std::size_t>(c.to[i])]; }
        ++stats->floor_components; stats->floor_stretches += c.stretches.size();
        RoadCompletionStats::FloorComponentReport report; report.first_way = c.ways.front(); report.first_start_ref = c.stretches.front().start_ref;
        report.stretches = c.stretches.size();
        structure::FloorNetwork net; net.node_count = static_cast<int>(c.nodes.size());
        if (c.ok) {
            for (std::size_t i = 0; i < c.stretches.size() && c.ok; ++i) {
                auto ref = builder.fit_stretch(c.ways[i], c.stretches[i], reference);
                if (!ref || !(ref->length_m >= 0.5)) { c.ok = false; break; }
                const auto* a = walk.attributes(c.ways[i]); const double half = a ? static_cast<double>(a->width_mm) / 2000.0 : 0.0;
                structure::FloorStretch fs; fs.from = c.from[i]; fs.to = c.to[i]; fs.length_m = ref->length_m;
                const std::size_t count = structure::station_count(ref->length_m);
                fs.ground.assign(count, std::numeric_limits<double>::quiet_NaN());
                for (std::size_t k = 0; k < count; ++k) {
                    const auto pose = ref->at(structure::station_m(ref->length_m, k)); if (!pose) continue;
                    const double nx = -det::sin(pose->heading) * half, ny = det::cos(pose->heading) * half;
                    double low = std::numeric_limits<double>::infinity(); bool all = true;
                    for (const double side : {0.0, 1.0, -1.0}) {
                        const auto h = dem.sample(zone, {std::llround((pose->x + side * nx) * 1000), std::llround((pose->y + side * ny) * 1000)});
                        if (!h) { all = false; break; }
                        low = std::min(low, *h / 256.0);
                    }
                    if (all) fs.ground[k] = low;
                }
                c.length.push_back(ref->length_m); c.refs.push_back(std::move(*ref)); net.stretches.push_back(std::move(fs));
                report.length_m += c.length.back(); report.stations += count;
            }
        }
        std::vector<std::size_t> portal_nodes;   // node index per net.portals entry
        for (std::size_t n = 0; c.ok && n < c.nodes.size(); ++n) {
            int nontunnel = 0;
            walk.for_each_edge(c.nodes[n], [&](const RoadGraphEdge& edge) { const auto* a = walk.attributes(edge.way_id); if (a && !a->tunnel) ++nontunnel; });
            if (nontunnel == 0 && c.tunnel_ends[n] != 1) continue;
            c.portal[n] = 1;
            if (nontunnel > 0) c.anchor[n] = portal_raw(c.nodes[n]);
            double z = 0;
            if (c.anchor[n]) z = c.anchor[n]->plane.height / 256.0;
            else { const auto h = dem.sample(zone, walk.position(c.nodes[n])); if (!h) { c.ok = false; break; } z = *h / 256.0; }
            net.portals.push_back({static_cast<int>(n), z, !c.anchor[n]}); portal_nodes.push_back(n);
            if (!c.anchor[n]) ++report.free_portals;
        }
        report.portals = net.portals.size();
        if (c.ok) {
            structure::NetworkFloorParams params; params.max_grade = options.structure_max_grade;
            params.needed = options.structure_clearance_m + options.structure_roof_m; params.reach_factor = kPortalReach; params.quantum_m = 1.0 / 256.0;
            c.floor = structure::network_design_floor(net, params);
            c.ok = c.floor.ok;
            report.pairs = c.floor.pairs; report.pairs_shifted = c.floor.pairs_shifted; report.pairs_beyond_grade = c.floor.pairs_beyond_grade;
            report.pairs_still_beyond_grade = c.floor.pairs_still_beyond_grade; report.passes = c.floor.passes;
            stats->floor_pairs += c.floor.pairs; stats->floor_pairs_shifted += c.floor.pairs_shifted; stats->floor_pairs_beyond_grade += c.floor.pairs_beyond_grade;
            stats->floor_pairs_still_beyond_grade += c.floor.pairs_still_beyond_grade; stats->floor_anchor_misses += c.floor.anchor_misses;
            if (!c.floor.converged) ++stats->floor_not_converged;
            for (std::size_t p = 0; c.ok && p < portal_nodes.size(); ++p) {
                const double move = c.floor.portal_shift_m[p];
                report.max_shift_m = std::max(report.max_shift_m, std::fabs(move)); stats->floor_max_shift_m = std::max(stats->floor_max_shift_m, std::fabs(move));
                if (c.anchor[portal_nodes[p]]) c.shift[c.nodes[portal_nodes[p]]] = move;
            }
        }
        if (!c.ok) ++stats->floor_unsolved;
        stats->floor_stations += report.stations; stats->floor_portals += report.portals; stats->floor_free_portals += report.free_portals;
        report.ok = c.ok; stats->floor_reports.push_back(report);
        return &c;
    };
    // The component a node of the tunnel network belongs to (nullptr: no tunnel stretch ends there).
    const auto component_at = [&](std::int64_t node) -> FloorComponent* {
        for (const auto way_id : tunnel_ways_at(node)) {
            const auto* w = way_of(way_id); const auto stretches = builder.stretches(way_id); if (!w || !stretches) continue;
            for (const auto& s : *stretches)
                if (w->node_ids[s.start_ref] == node || w->node_ids[s.end_ref] == node) return component_for(way_id, s);
        }
        return nullptr;
    };

    std::map<std::int64_t, double> shift_cache;
    const auto portal_shift = [&](std::int64_t node) -> double {
        const auto cached = shift_cache.find(node); if (cached != shift_cache.end()) return cached->second;
        if (options.tunnel_design_floor) {   // 7a-2: the move of the component solve (zero for a node no component anchors at)
            double move = 0;
            if (options.tunnel_portal_anchor && options.tunnel_floors)
                if (const auto* c = component_at(node); c && c->ok) { const auto it = c->shift.find(node); if (it != c->shift.end()) move = it->second; }
            return shift_cache[node] = move;
        }
        double best = 0;
        const auto here = options.tunnel_portal_anchor && options.tunnel_floors ? portal_raw(node) : std::nullopt;
        if (here) {
            std::vector<std::int64_t> ways;
            walk.for_each_edge(node, [&](const RoadGraphEdge& edge) { const auto* a = walk.attributes(edge.way_id); if (a && a->tunnel) ways.push_back(edge.way_id); });
            std::sort(ways.begin(), ways.end()); ways.erase(std::unique(ways.begin(), ways.end()), ways.end());
            for (const auto way_id : ways) {
                const auto* w = way_of(way_id); const auto stretches = builder.stretches(way_id); if (!w || !stretches) continue;
                for (const auto stretch : *stretches) {
                    const auto first = w->node_ids[stretch.start_ref], last = w->node_ids[stretch.end_ref];
                    if (first == last || (first != node && last != node)) continue;
                    // An end nothing joins (no raw anchor) is free: step 2a moves its floor end instead, so nothing shifts here.
                    const auto far = portal_raw(first == node ? last : first); const auto length = tunnel_length(way_id, stretch);
                    if (!far || !length) continue;
                    const double difference = (far->plane.height - here->plane.height) / 256.0, reach = kPortalReach * options.tunnel_max_grade * *length;
                    if (std::fabs(difference) <= reach) continue;
                    const double shift = (difference < 0 ? -1.0 : 1.0) * (std::fabs(difference) - reach) / 2.0;
                    if (std::fabs(shift) > std::fabs(best)) best = shift;
                }
            }
        }
        return shift_cache[node] = best;
    };
    std::map<std::int64_t, std::optional<PortalAnchor>> portal_moved_cache;
    const auto portal_plane = [&](std::int64_t node) -> std::optional<PortalAnchor> {
        auto raw = portal_raw(node); if (!raw) return raw;
        const double shift = portal_shift(node); if (shift == 0) return raw;
        const auto cached = portal_moved_cache.find(node); if (cached != portal_moved_cache.end()) return cached->second;
        raw->plane.height += static_cast<std::int32_t>(std::llround(shift * 256.0)); ++stats->portal_clamped;
        return portal_moved_cache[node] = raw;
    };
    // Step 2a, 7a-2: every declined tunnel entry takes its stretch of the component's design floor. The samples are the floor
    // at tunnel_spacing_m along the reference with both ends fixed at the floor's end values (a shared node therefore has one
    // height in every stretch and every tile); end grades: at a portal the joining ground road's (a tunnel end continues that
    // road, clamped to the structure grade), at a junction of exactly two tunnel stretch ends the mean of the two floor slopes
    // (the grade is continuous through it), else the floor's own slope. A fit the curvature bound refuses falls back to the
    // floor's own end slopes, then to the straight line between the end heights (counted).
    for (std::size_t i = 0; options.tunnel_design_floor && options.tunnel_floors && i < entries.size(); ++i) {
        auto& entry = entries[i]; if (entry.profile || !entry.decline) continue;
        const auto* attrs = walk.attributes(entry.way_id); if (!attrs || !attrs->tunnel) continue;
        ++stats->tunnel_candidates; ++stats->floor_entries;
        const auto fail = [&] { ++stats->tunnel_failed; ++stats->floor_entry_failed; };
        auto* comp = component_for(entry.way_id, entry.stretch);
        if (!comp || !comp->ok) { fail(); continue; }
        const std::size_t idx = comp->index.at({entry.way_id, entry.stretch.start_ref});
        const auto& floor = comp->floor.floor[idx];
        const double length = comp->length[idx], g = options.structure_max_grade;
        const int count = std::max(1, static_cast<int>(std::ceil(length / options.tunnel_spacing_m)));
        if (count > 2047) { fail(); continue; }
        const auto at_station = [&](double station) {
            const std::size_t k = std::min(static_cast<std::size_t>(station), floor.size() - 2);
            const double s0 = structure::station_m(length, k), s1 = structure::station_m(length, k + 1);
            return floor[k] + (floor[k + 1] - floor[k]) * ((station - s0) / (s1 - s0));
        };
        std::vector<VerticalSample> samples;
        for (int k = 0; k <= count; ++k) {
            const double station = k == count ? length : length * k / count;
            samples.push_back({station, k == 0 ? floor.front() : k == count ? floor.back() : at_station(station)});
        }
        // The slope leaving a stretch end into its stretch (positive uphill leaving the node).
        const auto leaving = [&](std::size_t s, bool at_start) {
            const auto& f = comp->floor.floor[s]; const double len = comp->length[s]; const std::size_t n = f.size();
            if (at_start) return (f[1] - f[0]) / (structure::station_m(len, 1) - structure::station_m(len, 0));
            return (f[n - 2] - f[n - 1]) / (structure::station_m(len, n - 1) - structure::station_m(len, n - 2));
        };
        const auto own_grade = [&](bool at_start) { const double o = leaving(idx, at_start); return std::clamp(at_start ? o : -o, -g, g); };
        const auto end_grade = [&](bool at_start) {
            const int node = at_start ? comp->from[idx] : comp->to[idx];
            const std::size_t n = static_cast<std::size_t>(node);
            if (comp->portal[n]) {
                if (const auto& anchor = comp->anchor[n]) {
                    const auto pose = comp->refs[idx].at(at_start ? 0 : length);
                    const double along = anchor->away ? (at_start ? -*anchor->away : *anchor->away) : (pose ? project(anchor->plane, pose->heading) : 0.0);
                    return std::clamp(along, -g, g);
                }
                return own_grade(at_start);
            }
            if (comp->tunnel_ends[n] != 2) return own_grade(at_start);
            std::vector<std::pair<std::size_t, bool>> ends;   // (stretch, at start), stretch order, start end first
            for (std::size_t s = 0; s < comp->stretches.size(); ++s) {
                if (comp->from[s] == node) ends.emplace_back(s, true);
                if (comp->to[s] == node) ends.emplace_back(s, false);
            }
            const double common = (leaving(ends[0].first, ends[0].second) - leaving(ends[1].first, ends[1].second)) / 2.0;
            const double target = ends[0] == std::make_pair(idx, at_start) ? common : -common;   // slope leaving the node into this stretch
            return std::clamp(at_start ? target : -target, -g, g);
        };
        const double designed_start = end_grade(true), designed_end = end_grade(false);
        VerticalFitLimits fit; fit.max_grade = g; fit.min_radius_m = options.tunnel_min_radius_m; fit.max_iterations = 16384;
        const auto solve = [&](const std::vector<VerticalSample>& heights, double start_grade, double end_grade_value) -> std::optional<VerticalProfile> {
            fit.start_grade = start_grade; fit.end_grade = end_grade_value;
            auto v = fit_vertical_profile(heights, fit); if (!v) return std::nullopt; return std::move(*v);
        };
        // A floor that is one straight line between its end heights (the chord governs it: a long tunnel, an open cut) stays
        // straight, as the owner ruling of D52/D65 keeps it and the terrain carve recognises a cut by it: exactly the chord
        // for a cut (length at most tunnel_cut_max_length_m), else the chord with the portal grades held within
        // kPortalGradeSlack over short vertical curves (ground_ramp_m, a quarter of the length at most).
        std::optional<VerticalProfile> vertical;
        {
            const double chord = (samples.back().height_m - samples.front().height_m) / length;
            bool straight = std::fabs(chord) <= g;
            for (std::size_t k = 1; straight && k + 1 < floor.size(); ++k)
                straight = std::fabs(floor[k] - (floor.front() + chord * structure::station_m(length, k))) <= 0.01;
            if (straight) {
                const bool cut = options.tunnel_cut_max_length_m > 0 && length <= options.tunnel_cut_max_length_m;
                fit.start_grade = cut ? chord : std::clamp(chord, designed_start - kPortalGradeSlack, designed_start + kPortalGradeSlack);
                fit.end_grade = cut ? chord : std::clamp(chord, designed_end - kPortalGradeSlack, designed_end + kPortalGradeSlack);
                std::vector<VerticalSample> line{samples.front(), samples.back()};
                if (fit.start_grade != chord || fit.end_grade != chord) {
                    const double ramp = std::min(std::max(options.ground_ramp_m, 1.0), length / 4.0);
                    line = {samples.front(), {ramp, samples.front().height_m + chord * ramp}, {length - ramp, samples.back().height_m - chord * ramp}, samples.back()};
                    if (ramp < 0.5 || line[2].station_m - line[1].station_m < 0.5) line = {samples.front(), samples.back()};
                }
                if (auto v = fit_vertical_profile(line, fit)) { vertical = std::move(*v); ++(cut ? stats->tunnel_cut : stats->tunnel_chord); }
            }
        }
        if (!vertical) vertical = solve(samples, designed_start, designed_end);
        if (!vertical) {
            vertical = solve(samples, own_grade(true), own_grade(false));
            if (vertical) ++stats->floor_entry_own_grades;
        }
        if (!vertical) {
            const std::vector<VerticalSample> ends{samples.front(), samples.back()};
            const double chord = std::clamp((ends[1].height_m - ends[0].height_m) / length, -g, g);
            vertical = solve(ends, chord, chord);
            if (vertical) ++stats->floor_entry_straight;
        }
        if (!vertical) { fail(); continue; }
        RoadProfile p; p.stretch = entry.stretch; p.attributes = *attrs; p.reference = comp->refs[idx]; p.vertical = std::move(*vertical);
        double deviation = 0;
        for (const auto& s : samples) if (const auto pose = p.at(s.station_m)) deviation = std::max(deviation, std::fabs(pose->height_m - s.height_m));
        stats->floor_fit_max_deviation_m = std::max(stats->floor_fit_max_deviation_m, deviation);
        if (deviation > 0.25) ++stats->floor_fit_over_250mm;
        entry.profile = std::move(p); mark(i, ProfileProvenance::TunnelFloor); ++stats->tunnel_completed;
    }

    // Step 2a (D47, the game's tunnel step): tunnel floors. The DEM along the tunnel's own reference is the floor (the
    // lower road's ground; the roof is supplied by the road above, so a floor is never carved into the terrain). A
    // tunnel at least tunnel_chord_min_length_m long takes the straight chord between its two portal samples instead
    // when that chord is within the grade bound (else the DEM floor, as below the threshold). D52 (owner ruling
    // 2026-10-15, long tunnels follow straight lines): the served threshold is de.json's 200 m. The threshold is
    // compared with the whole stretch's reference length, never with a tile's part of it. A function of the entry's
    // own way and the DEM along it (road_dem_dependencies), so tile-invariant.
    for (std::size_t i = 0; options.tunnel_floors && !options.tunnel_design_floor && i < entries.size(); ++i) {
        auto& entry = entries[i]; if (entry.profile || !entry.decline) continue;
        const auto* attrs = walk.attributes(entry.way_id); if (!attrs || !attrs->tunnel) continue;
        ++stats->tunnel_candidates;
        const auto fail = [&] { ++stats->tunnel_failed; };
        auto ref = builder.fit_stretch(entry.way_id, entry.stretch, reference); if (!ref || ref->length_m < 0.5) { fail(); continue; }
        const int count = std::max(1, static_cast<int>(std::ceil(ref->length_m / options.tunnel_spacing_m)));
        if (count > 2047) { fail(); continue; }
        std::vector<VerticalSample> samples; bool complete = true;
        for (int k = 0; k <= count; ++k) {
            const double station = ref->length_m * k / count; const auto xy = ref->at(station); if (!xy) { complete = false; break; }
            const auto h = dem.sample(zone, {std::llround(xy->x * 1000), std::llround(xy->y * 1000)}); if (!h) { complete = false; break; }
            samples.push_back({station, *h / 256.0});
        }
        if (!complete) { fail(); continue; }
        const double g = options.tunnel_max_grade;
        VerticalFitLimits fit; fit.max_grade = g; fit.min_radius_m = options.tunnel_min_radius_m; fit.max_iterations = 16384;
        // 7a-1: the portal anchors. Each end that something joins takes that join's height and grade (portal_plane); the
        // DEM samples near the end carry the height correction faded out over ground_ramp_m, so the interior keeps the DEM.
        const auto dem_samples = samples;
        std::optional<PortalAnchor> pa, pb; std::optional<double> pa_grade, pb_grade;
        if (options.tunnel_portal_anchor && ref->at(0) && ref->at(ref->length_m)) {
            const auto* w = way_of(entry.way_id);
            if (w) { pa = portal_plane(w->node_ids[entry.stretch.start_ref]); pb = portal_plane(w->node_ids[entry.stretch.end_ref]); }
            // The offsets are measured on the DEM samples before either is applied; the ramp never exceeds the length, so one
            // end's correction has faded out at the other end and both end heights come out exact.
            const double ramp = std::min(std::max(options.ground_ramp_m, 1.0), ref->length_m);
            double offset_a = pa ? pa->plane.height / 256.0 - samples.front().height_m : 0, offset_b = pb ? pb->plane.height / 256.0 - samples.back().height_m : 0;
            // An end nothing joins is free (review of ffd575d): it keeps the DEM sample unless that is out of the floor's reach of the
            // anchored end, then it moves to the nearest reachable height - nothing there to be continuous with.
            const double reach = kPortalReach * g * ref->length_m;
            bool free_a = false, free_b = false;
            if (pb && !pa) { const double h = pb->plane.height / 256.0, target = std::clamp(samples.front().height_m, h - reach, h + reach); free_a = target != samples.front().height_m; offset_a = target - samples.front().height_m; }
            if (pa && !pb) { const double h = pa->plane.height / 256.0, target = std::clamp(samples.back().height_m, h - reach, h + reach); free_b = target != samples.back().height_m; offset_b = target - samples.back().height_m; }
            if (free_a || free_b) ++stats->portal_free_end;
            for (auto& sample : samples) {
                const double da = sample.station_m, db = ref->length_m - sample.station_m;
                if ((pa || free_a) && da < ramp) sample.height_m += offset_a * (1.0 - da / ramp);
                if ((pb || free_b) && db < ramp) sample.height_m += offset_b * (1.0 - db / ramp);
            }
            if (pa) { pa_grade = std::clamp(pa->away ? -*pa->away : project(pa->plane, ref->at(0)->heading), -g, g); ++stats->portal_tunnel_ends; }
            if (pb) { pb_grade = std::clamp(pb->away ? *pb->away : project(pb->plane, ref->at(ref->length_m)->heading), -g, g); ++stats->portal_tunnel_ends; }
        }
        // One solve of the floor over `heights` with the given portal grades (nullopt = the DEM's own end slope).
        const auto solve = [&](const std::vector<VerticalSample>& heights, std::optional<double> start_grade, std::optional<double> end_grade) {
            std::optional<VerticalProfile> vertical;
            // D65: a short tunnel through a bank (the DEM rises over the straight portal chord by a bounded hump) is an open cut.
            bool cut = false;
            if (options.tunnel_cut_max_length_m > 0 && ref->length_m <= options.tunnel_cut_max_length_m &&
                !(options.tunnel_chord_min_length_m > 0 && ref->length_m >= options.tunnel_chord_min_length_m)) {
                double hump = 0;
                for (const auto& s : heights)
                    hump = std::max(hump, s.height_m - (heights.front().height_m + (heights.back().height_m - heights.front().height_m) * s.station_m / ref->length_m));
                cut = hump >= options.tunnel_cut_hump_min_m && hump <= options.tunnel_cut_hump_max_m;
            }
            if (cut || (options.tunnel_chord_min_length_m > 0 && ref->length_m >= options.tunnel_chord_min_length_m)) {
                const double chord = (heights.back().height_m - heights.front().height_m) / ref->length_m;
                if (std::fabs(chord) <= g) {
                    // A portal end holds the ground road's grade within kPortalGradeSlack, so a straight floor stays straight where the
                    // ground grade is near the chord and bends only as much as the continuity needs.
                    // D65 cut: exactly the chord (the terrain carve recognises a cut by a floor straight within 1 cm); the portal HEIGHT
                    // is anchored, the grade is the chord's.
                    fit.start_grade = (start_grade && !cut) ? std::clamp(chord, *start_grade - kPortalGradeSlack, *start_grade + kPortalGradeSlack) : chord;
                    fit.end_grade = (end_grade && !cut) ? std::clamp(chord, *end_grade - kPortalGradeSlack, *end_grade + kPortalGradeSlack) : chord;
                    const std::array<VerticalSample, 2> portals{{heights.front(), heights.back()}};
                    std::vector<VerticalSample> line(portals.begin(), portals.end());
                    if (fit.start_grade != chord || fit.end_grade != chord) {
                        // The ground grade at a portal differs from the chord by more than the slack: the floor stays on the chord
                        // and meets the portal grades over short vertical curves (ground_ramp_m, a quarter of the length at most)
                        // instead of bowing over the whole length (D52: long tunnels follow straight lines).
                        const double ramp = std::min(std::max(options.ground_ramp_m, 1.0), ref->length_m / 4.0);
                        line = {portals[0], {ramp, portals[0].height_m + chord * ramp}, {ref->length_m - ramp, portals[1].height_m - chord * ramp}, portals[1]};
                        if (ramp < 0.5 || line[2].station_m - line[1].station_m < 0.5) line.assign(portals.begin(), portals.end());
                    }
                    auto v = fit_vertical_profile(line, fit); if (v) { vertical = std::move(*v); ++(cut ? stats->tunnel_cut : stats->tunnel_chord); }
                }
            }
            if (!vertical) {
                fit.start_grade = std::clamp((heights[1].height_m - heights[0].height_m) / (heights[1].station_m - heights[0].station_m), -g, g);
                fit.end_grade = std::clamp((heights.back().height_m - heights[heights.size() - 2].height_m) /
                    (heights.back().station_m - heights[heights.size() - 2].station_m), -g, g);
                if (start_grade) fit.start_grade = *start_grade;
                if (end_grade) fit.end_grade = *end_grade;
                auto v = fit_vertical_profile(heights, fit); if (v) vertical = std::move(*v);
            }
            return vertical;
        };
        auto vertical = solve(samples, pa_grade, pb_grade);
        if (!vertical && (pa || pb)) {
            // The DEM between the portals does not fit with the portal heights and grades held: the floor between the two portal
            // anchors alone (the grades held), else the DEM floor as before 7a-1.
            const std::vector<VerticalSample> ends{samples.front(), samples.back()};
            vertical = solve(ends, pa_grade, pb_grade);
            if (!vertical) {
                // The portal grades contradict the portal heights over this length (curvature bound): the straight floor between the
                // portal heights keeps the height continuity and gives up the grade's.
                const double chord = std::clamp((ends[1].height_m - ends[0].height_m) / ref->length_m, -g, g);
                vertical = solve(ends, chord, chord);
                if (vertical) ++stats->portal_tunnel_straight;
            }
            if (!vertical) { ++stats->portal_tunnel_unfit; vertical = solve(dem_samples, std::nullopt, std::nullopt); }
        }
        if (!vertical) { fail(); continue; }
        RoadProfile p; p.stretch = entry.stretch; p.attributes = *attrs; p.reference = std::move(*ref); p.vertical = std::move(*vertical);
        entry.profile = std::move(p); mark(i, ProfileProvenance::TunnelFloor); ++stats->tunnel_completed;
    }

    // Step 2: declined bridge spans, re-fitted between the abutment planes of their two end nodes at relaxed limits.
    for (std::size_t i = 0; i < entries.size(); ++i) {
        auto& entry = entries[i]; if (entry.profile || !entry.decline) continue;
        const auto* w = way_of(entry.way_id); if (!w) continue;
        const auto* attrs = walk.attributes(entry.way_id); if (!attrs || attrs->tunnel || !attrs->bridge) continue;
        ++stats->bridge_candidates;
        auto ref = builder.fit_stretch(entry.way_id, entry.stretch, reference); if (!ref) { ++stats->bridge_fit_failed; continue; }
        const auto a = end_plane(w->node_ids[entry.stretch.start_ref], entry.way_id, entry.stretch.start_ref, false);
        const auto b = end_plane(w->node_ids[entry.stretch.end_ref], entry.way_id, entry.stretch.start_ref, false);
        if (!a.plane || !b.plane) { ++stats->bridge_no_support; continue; }
        auto p = fit_bridge(entry.stretch, *attrs, std::move(*ref), *a.plane, *b.plane);
        if (!p) { ++stats->bridge_fit_failed; continue; }
        entry.profile = std::move(*p); mark(i, ProfileProvenance::BridgeCompleted); ++stats->bridge_completed;
    }

    // Step 3: still-declined ground stretches, re-fitted at relaxed limits, 40 m samples and 32 m ramps from the
    // bridge abutment planes at their ends (the DEM plane at the node where no bridge ends). 7a-1: a retry-accepted ground stretch ending
    // at a portal the ground gave way at (portal_shift) is re-fitted to the moved plane as well.
    for (std::size_t i = 0; i < entries.size(); ++i) {
        auto& entry = entries[i];
        const bool refit = options.tunnel_portal_anchor && entry.profile && !entry.decline && origins[i].provenance == ProfileProvenance::Retry;
        if ((entry.profile || !entry.decline) && !refit) continue;
        const auto* w = way_of(entry.way_id); if (!w) continue;
        const auto* attrs = walk.attributes(entry.way_id); if (!attrs || attrs->bridge || attrs->tunnel) continue;
        const auto first_id = w->node_ids[entry.stretch.start_ref], last_id = w->node_ids[entry.stretch.end_ref];
        if (refit && portal_shift(first_id) == 0 && portal_shift(last_id) == 0) continue;
        ++stats->ground_candidates;
        const auto fail = [&] { ++stats->ground_failed; };
        auto ref = builder.fit_stretch(entry.way_id, entry.stretch, reference); if (!ref || ref->length_m < 0.5) { fail(); continue; }
        struct GroundFit { VerticalProfile vertical; bool bridge_first, bridge_last; };
        bool use_portal = false, waive_mismatch = false, raw_portal = false;
        // The portal plane at a node a tunnel floor ends on (touches_tunnel), else nothing.
        const auto tunnel_portal = [&](std::int64_t id) -> std::optional<RoadJunctionPlane> {
            bool tunnel = false;
            walk.for_each_edge(id, [&](const RoadGraphEdge& edge) { const auto* a = walk.attributes(edge.way_id); if (a && a->tunnel) tunnel = true; });
            if (!tunnel) return std::nullopt;
            const auto anchor = raw_portal ? portal_raw(id) : portal_plane(id); if (!anchor) return std::nullopt;
            return anchor->plane;
        };
        use_portal = options.tunnel_portal_anchor && (tunnel_portal(first_id) || tunnel_portal(last_id));
        const auto try_fit = [&](const RoadBridgeAnchor& anchor_first, const RoadBridgeAnchor& anchor_last) -> std::optional<GroundFit> {
            const RoadReferenceLine& line = *ref;
            const bool bridge_first = anchor_first.plane.has_value(), bridge_last = anchor_last.plane.has_value();
            const auto plane_at = [&](const RoadBridgeAnchor& anchor, std::int64_t id, bool at_start) -> std::optional<RoadJunctionPlane> {
                if (anchor.plane) return anchor.plane;
                if (limits.underpass) if (auto join = limits.underpass->join(id)) return join->plane;   // D63
                if (limits.underpass) if (auto over = limits.underpass->overpass_end_plane(entry.way_id, entry.stretch, line, at_start)) return over;   // D64
                if (use_portal) if (const auto tunnel = tunnel_portal(id)) return tunnel;   // 7a-1: the tunnel floor that starts here
                auto p = dem.junction_plane(zone, walk.position(id)); if (!p) return std::nullopt; return *p;
            };
            const auto a = plane_at(anchor_first, first_id, true), b = plane_at(anchor_last, last_id, false);
            if (!a || !b) return std::nullopt;
            const auto start = line.at(0), end = line.at(line.length_m); if (!start || !end) return std::nullopt;
            VerticalFitLimits fit; fit.max_grade = std::max(0.12, attrs->max_grade_per_mille / 1000.0);
            fit.min_radius_m = std::min<double>(500, attrs->min_vertical_radius_m); fit.max_iterations = 16384;
            fit.start_grade = std::clamp(project(*a, start->heading), -0.10, 0.10);
            fit.end_grade = std::clamp(project(*b, end->heading), -0.10, 0.10);
            const auto count = static_cast<std::size_t>(std::max(1.0, std::ceil(line.length_m / options.ground_spacing_m)));
            if (count > 2047) return std::nullopt;
            std::vector<VerticalSample> samples{{0, a->height / 256.0}}; bool complete = true;
            for (std::size_t k = 1; k < count; ++k) {
                const double station = line.length_m * static_cast<double>(k) / static_cast<double>(count);
                const auto xy = line.at(station); if (!xy) { complete = false; break; }
                const auto height = dem.junction_height(zone, {std::llround(xy->x * 1000), std::llround(xy->y * 1000)});
                if (!height) { complete = false; break; }
                double z = *height / 256.0;
                // Carry the abutments to supported ground, keeping the ramps out of the DEM's underpass depression.
                if (bridge_first && station < options.ground_ramp_m) z = a->height / 256.0 + fit.start_grade * station;
                if (bridge_last && line.length_m - station < options.ground_ramp_m) z = b->height / 256.0 - fit.end_grade * (line.length_m - station);
                samples.push_back({station, z});
            }
            if (!complete) return std::nullopt;
            samples.push_back({line.length_m, b->height / 256.0});
            if (limits.underpass) limits.underpass->mask_samples(entry.way_id, entry.stretch, line, samples);   // D54
            auto vertical = fit_vertical_profile(samples, fit);
            // D43: end planes that contradict the stretch's own measured rise (a node at the edge of a deck, where the bare
            // earth is interpolated and catches up within metres; a short way split between two such reads) leave no cubic
            // under the curvature bound. The end heights stay measured; both end grades are eased toward the chord by the
            // same fraction k/8, the smallest k that fits (k = 8 is the chord itself, as the bridge step's fallback above).
            // Only a local misread is eased: an end plane further than kGroundEaseMismatch from the chord is terrain that
            // really departs from the stretch (a road forced onto the chord there rides off it), so it stays declined.
            if (!vertical && boundary_infeasible(vertical.error().message)) {
                const double chord = static_cast<double>(b->height - a->height) / 256.0 / line.length_m;
                const double g0 = fit.start_grade, g1 = fit.end_grade;
                // waive_mismatch (7a-1, last resort on a stretch that touches a tunnel portal): the portal height is the requirement, so
                // the stretch may leave the DEM's grade and ride the chord from the portal to its other end.
                const bool near_chord = std::fabs(chord) <= fit.max_grade && (waive_mismatch || (std::fabs(g0 - chord) <= kGroundEaseMismatch && std::fabs(g1 - chord) <= kGroundEaseMismatch));
                for (int k = 1; near_chord && k <= 8; ++k) {
                    const double t = static_cast<double>(k) / 8.0;
                    fit.start_grade = g0 + t * (chord - g0); fit.end_grade = g1 + t * (chord - g1);
                    vertical = fit_vertical_profile(samples, fit);
                    if (vertical) { ++stats->ground_grades_eased; break; }
                    if (!boundary_infeasible(vertical.error().message)) break;
                }
            }
            if (!vertical) return std::nullopt;
            return GroundFit{std::move(*vertical), bridge_first, bridge_last};
        };
        // D47: the accepted bridge planes first. Where they leave no cubic (an end plane of a re-fitted span that the
        // stretch's own DEM rise contradicts), the base anchors of the previous release (D25/D30) are tried, so the tier
        // never declines a ground stretch the previous release completed.
        auto result = try_fit(end_plane(first_id, entry.way_id, entry.stretch.start_ref, true), end_plane(last_id, entry.way_id, entry.stretch.start_ref, true));
        if (!result && options.bridge_anchor_reuse) { result = try_fit(anchor_of(first_id), anchor_of(last_id)); if (result) ++stats->ground_base_fallback; }
        if (!result && use_portal && true) {   // the portal plane alone left no cubic: eased to the chord whatever the mismatch
            waive_mismatch = true;
            result = try_fit(end_plane(first_id, entry.way_id, entry.stretch.start_ref, true), end_plane(last_id, entry.way_id, entry.stretch.start_ref, true));
            if (!result && options.bridge_anchor_reuse) result = try_fit(anchor_of(first_id), anchor_of(last_id));
            waive_mismatch = false;
            if (result) ++stats->ground_tunnel_chord;
        }
        if (!result && use_portal && (portal_shift(first_id) != 0 || portal_shift(last_id) != 0)) {
            // The ground stretch cannot reach the moved plane within its own class bound (a stub too short to climb the shift): it keeps
            // the unmoved portal plane of the first 7a-1 build and a step of the shift remains, reported by the census as unreachable.
            raw_portal = true;
            result = try_fit(end_plane(first_id, entry.way_id, entry.stretch.start_ref, true), end_plane(last_id, entry.way_id, entry.stretch.start_ref, true));
            if (!result && options.bridge_anchor_reuse) result = try_fit(anchor_of(first_id), anchor_of(last_id));
            raw_portal = false;
            if (result) ++stats->ground_portal_unreachable;
        }
        if (!result && use_portal) {
            // The tunnel portal plane left no cubic: the DEM planes of the previous release, counted.
            use_portal = false; ++stats->ground_tunnel_unfit;
            result = try_fit(end_plane(first_id, entry.way_id, entry.stretch.start_ref, true), end_plane(last_id, entry.way_id, entry.stretch.start_ref, true));
            if (!result && options.bridge_anchor_reuse) result = try_fit(anchor_of(first_id), anchor_of(last_id));
        } else if (result && use_portal && (tunnel_portal(first_id) || tunnel_portal(last_id))) ++stats->ground_tunnel_anchored;
        if (!result && refit) continue;   // the accepted profile stays (its portal is reported unreachable by the census)
        if (!result) { fail(); continue; }
        if (refit) ++stats->ground_portal_refit;
        const bool bridge_first = result->bridge_first, bridge_last = result->bridge_last;
        RoadProfile p; p.stretch = entry.stretch; p.attributes = *attrs; p.reference = std::move(*ref); p.vertical = std::move(result->vertical);
        p.start_anchor_inferred = bridge_first; p.end_anchor_inferred = bridge_last;
        entry.profile = std::move(p); mark(i, ProfileProvenance::GroundCompleted);
        ++stats->ground_completed; if (bridge_first || bridge_last) ++stats->ground_bridge_anchored;
    }
    return {};
}
}
