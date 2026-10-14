// src/route/routing_graph.cpp - see g2m/route/routing_graph.h.
#include "g2m/route/routing_graph.h"

#include "g2m/core/det_math.h"
#include "g2m/layer/osm_roads.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace g2m::route {
namespace {

std::optional<std::string_view> tag_of(const RoadGraphWay& way, std::string_view key) {
    const auto it = std::lower_bound(way.tags.begin(), way.tags.end(), key,
                                     [](const auto& tag, std::string_view k) { return tag.first < k; });
    if (it != way.tags.end() && it->first == key) return std::string_view(it->second);
    // Tags are canonical (sorted) in a RoadGraph; fall back to a scan for hand-built input.
    for (const auto& [k, v] : way.tags) if (k == key) return std::string_view(v);
    return std::nullopt;
}

bool contains(const std::vector<std::string>& list, std::string_view value) {
    return std::find(list.begin(), list.end(), value) != list.end();
}

// Speed for one direction ("forward"/"backward"): the direction tag, else the
// plain maxspeed, else the class default; symbolic values via the zone table.
std::uint16_t way_speed(const RoadGraphWay& way, std::string_view direction_key, std::uint16_t class_default,
                        const RouteConfig& config) {
    auto text = tag_of(way, direction_key);
    if (!text) text = tag_of(way, "maxspeed");
    std::int64_t kmh = class_default;
    if (text) {
        const RoadSpeedLimit limit = parse_road_speed_limit(*text);
        if (limit.kind == SpeedLimitKind::Numeric) {
            kmh = std::llround(limit.kph);
        } else if (limit.kind == SpeedLimitKind::Unknown) {
            for (const auto& [zone, speed] : config.zone_speeds) if (zone == *text) kmh = speed;
        }
    }
    kmh = std::clamp<std::int64_t>(kmh, config.min_speed_kmh, config.max_speed_kmh);
    return static_cast<std::uint16_t>(kmh);
}

// Most specific of motorcar > motor_vehicle > vehicle > access decides.
bool access_blocked(const RoadGraphWay& way, const RouteConfig& config) {
    for (const std::string_view key : {"motorcar", "motor_vehicle", "vehicle", "access"}) {
        if (const auto value = tag_of(way, key)) return contains(config.blocked_access, *value);
    }
    return false;
}

struct Directions {
    bool forward = true, backward = true;
};
Directions way_directions(const RoadGraphWay& way, const RouteConfig& config) {
    const auto oneway = tag_of(way, "oneway");
    if (oneway) {
        if (*oneway == "yes" || *oneway == "true" || *oneway == "1") return {true, false};
        if (*oneway == "-1" || *oneway == "reverse") return {false, true};
        if (*oneway == "reversible" || *oneway == "alternating") return {false, false};
        if (*oneway == "no" || *oneway == "false" || *oneway == "0") return {true, true};
    }
    const auto junction = tag_of(way, "junction");
    if (junction && (*junction == "roundabout" || *junction == "circular")) return {true, false};
    const auto highway = tag_of(way, "highway");
    if (highway && contains(config.implied_oneway, *highway)) return {true, false};
    return {true, true};
}

std::uint32_t ceil_length_mm(const geom::PointMm& a, const geom::PointMm& b) {
    const double dx = static_cast<double>(b.x - a.x), dy = static_cast<double>(b.y - a.y);
    return static_cast<std::uint32_t>(std::ceil(det::sqrt(dx * dx + dy * dy)));
}

struct Edge {
    std::uint32_t from, to;
    std::size_t way; // index into graph.ways
    std::uint32_t seg_first, seg_last;
    std::uint64_t length_mm;
    std::int64_t first_dx, first_dy, last_dx, last_dy;
};

} // namespace

const RouteConfig& default_route_config() {
    static const RouteConfig config = [] {
        RouteConfig c;
        c.classes = {
            {"motorway", 120, true},      {"motorway_link", 60, true}, {"trunk", 100, true},
            {"trunk_link", 50, true},     {"primary", 70, true},       {"primary_link", 40, true},
            {"secondary", 60, true},      {"secondary_link", 40, true}, {"tertiary", 50, true},
            {"tertiary_link", 30, true},  {"unclassified", 40, true},  {"residential", 30, true},
            {"living_street", 7, true},   {"service", 15, true},       {"track", 10, false},
        };
        c.zone_speeds = {{"DE:urban", 50}, {"DE:rural", 100}, {"DE:living_street", 7}, {"DE:zone30", 30},
                         {"DE:zone:30", 30}, {"DE:bicycle_road", 30}, {"walk", 7}};
        c.blocked_access = {"no", "private", "agricultural", "forestry", "emergency", "use_sidepath"};
        c.implied_oneway = {"motorway"};
        return c;
    }();
    return config;
}

std::uint32_t RoutingGraph::find_node(std::int64_t osm_id) const {
    const auto real = nodes();
    const auto it = std::lower_bound(real.begin(), real.end(), osm_id,
                                     [](const RouteGraphNode& n, std::int64_t id) { return n.osm_id < id; });
    if (it == real.end() || it->osm_id != osm_id) return kNoArc;
    return static_cast<std::uint32_t>(it - real.begin());
}

std::uint32_t RoutingGraph::turn_cost_ms(const RouteArc& in, const RouteArc& out) const {
    if (nodes_[out.from].degree < 3) return 0;
    const double ax = static_cast<double>(in.out_dx), ay = static_cast<double>(in.out_dy);
    const double bx = static_cast<double>(out.in_dx), by = static_cast<double>(out.in_dy);
    const double cross = ax * by - ay * bx, dot = ax * bx + ay * by;
    if (cross == 0.0 && dot >= 0.0) return 0;
    const double deg = det::atan2(cross, dot) * (180.0 / det::kPi);
    const double mag = std::fabs(deg);
    const bool left = deg > 0.0;
    if (mag <= config_.straight_max_deg) return 0;
    if (mag <= config_.slight_max_deg) return config_.slight_ms;
    if (mag <= config_.turn_max_deg) return left ? config_.left_ms : config_.right_ms;
    return left ? config_.sharp_left_ms : config_.sharp_right_ms;
}

void RoutingGraph::successors(std::uint32_t arc_index, std::vector<Transition>& out) const {
    out.clear();
    const RouteArc& in = arcs_[arc_index];
    const std::uint32_t v = in.to;
    // Rules at v that start on the arriving way.
    const auto rules_begin = rules_.begin() + nodes_[v].first_rule, rules_end = rules_.begin() + nodes_[v + 1].first_rule;
    auto first = std::lower_bound(rules_begin, rules_end, in.way_id,
                                  [](const RouteRule& r, std::int64_t w) { return r.from_way < w; });
    auto last = first;
    bool has_only = false;
    while (last != rules_end && last->from_way == in.way_id) {
        has_only = has_only || last->kind == RestrictionKind::Only;
        ++last;
    }
    const auto allowed_onto = [&](std::int64_t way, bool u_turn) {
        bool only_match = false;
        for (auto r = first; r != last; ++r) {
            if (r->to_way != way) continue;
            // A no_* rule whose from and to are the same way (no_u_turn) forbids only the U-turn,
            // never going straight on along that way through an interior via node.
            if (r->kind == RestrictionKind::No && r->from_way == r->to_way && !u_turn) continue;
            if (r->kind == RestrictionKind::No) return false;
            only_match = true;
        }
        return !has_only || only_match;
    };
    const std::uint32_t begin = nodes_[v].first_out, end = nodes_[v + 1].first_out;
    for (std::uint32_t b = begin; b < end; ++b) {
        if (b == in.reverse) continue;
        if (!allowed_onto(arcs_[b].way_id, false)) continue;
        out.push_back({b, turn_cost_ms(in, arcs_[b])});
    }
    // U-turn back along the same edge: only where nothing else may leave (dead end).
    if (out.empty() && in.reverse != kNoArc && allowed_onto(in.way_id, true))
        out.push_back({in.reverse, config_.u_turn_ms});
}

Result<RoutingGraph> build_routing_graph(const RoadGraph& graph, std::span<const TurnRestriction> restrictions,
                                         const RouteConfig& config) {
    if (config.min_speed_kmh == 0 || config.max_speed_kmh < config.min_speed_kmh)
        return make_error(ErrorCode::InvalidArgument, "route config: speed bounds");
    for (std::size_t i = 1; i < graph.nodes.size(); ++i)
        if (graph.nodes[i - 1].osm_id >= graph.nodes[i].osm_id)
            return make_error(ErrorCode::Malformed, "routing graph: nodes not strictly ascending");
    for (std::size_t i = 1; i < graph.ways.size(); ++i)
        if (graph.ways[i - 1].osm_id >= graph.ways[i].osm_id)
            return make_error(ErrorCode::Malformed, "routing graph: ways not strictly ascending");
    const auto node_index = [&](std::int64_t id) -> std::int64_t {
        const auto it = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), id,
                                         [](const RoadGraphNode& n, std::int64_t x) { return n.osm_id < x; });
        return it == graph.nodes.end() || it->osm_id != id ? -1 : static_cast<std::int64_t>(it - graph.nodes.begin());
    };

    RoutingGraph out;
    out.config_ = config;
    RouteBuildStats& stats = out.stats_;

    // 1. Which ways, in which directions, at which speeds.
    struct WayUse {
        bool used = false;
        Directions dirs;
        std::uint16_t fwd_kmh = 0, bwd_kmh = 0;
    };
    std::vector<WayUse> use(graph.ways.size());
    std::vector<std::vector<std::uint32_t>> refs(graph.ways.size()); // node indices, repeats dropped
    for (std::size_t w = 0; w < graph.ways.size(); ++w) {
        const RoadGraphWay& way = graph.ways[w];
        const auto highway = tag_of(way, "highway");
        const auto cls = highway ? std::find_if(config.classes.begin(), config.classes.end(),
                                                [&](const RouteClassSpeed& c) { return c.highway == *highway; })
                                 : config.classes.end();
        if (cls == config.classes.end() || !cls->routable) { ++stats.ways_not_routable; continue; }
        if (access_blocked(way, config)) { ++stats.ways_access_blocked; continue; }
        WayUse& u = use[w];
        u.dirs = way_directions(way, config);
        if (!u.dirs.forward && !u.dirs.backward) { ++stats.ways_no_direction; continue; }
        if (way.node_ids.size() < 2) return make_error(ErrorCode::Malformed, "routing graph: way with fewer than 2 refs");
        for (const std::int64_t id : way.node_ids) {
            const std::int64_t n = node_index(id);
            if (n < 0) return make_error(ErrorCode::Malformed, "routing graph: way references a missing node");
            refs[w].push_back(static_cast<std::uint32_t>(n));
        }
        u.used = true;
        u.fwd_kmh = way_speed(way, "maxspeed:forward", cls->default_kmh, config);
        u.bwd_kmh = way_speed(way, "maxspeed:backward", cls->default_kmh, config);
        ++stats.ways_used;
    }

    // 2. Decision nodes: way ends, nodes at two or more way positions, via nodes.
    std::vector<std::uint32_t> occurrences(graph.nodes.size(), 0);
    std::vector<char> keep(graph.nodes.size(), 0);
    for (std::size_t w = 0; w < graph.ways.size(); ++w) {
        if (!use[w].used) continue;
        const auto& r = refs[w];
        keep[r.front()] = keep[r.back()] = 1;
        for (std::size_t i = 0; i < r.size(); ++i)
            if (i == 0 || r[i] != r[i - 1]) ++occurrences[r[i]];
    }
    for (const TurnRestriction& tr : restrictions) {
        if (tr.via_node == 0) continue;
        const std::int64_t n = node_index(tr.via_node);
        if (n >= 0 && occurrences[static_cast<std::size_t>(n)] > 0) keep[static_cast<std::size_t>(n)] = 1;
    }
    std::vector<std::uint32_t> compact(graph.nodes.size(), kNoArc);
    for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
        if (occurrences[i] >= 2) keep[i] = 1;
        if (keep[i] && occurrences[i] > 0) {
            compact[i] = static_cast<std::uint32_t>(out.nodes_.size());
            RouteGraphNode n;
            n.osm_id = graph.nodes[i].osm_id;
            n.position = graph.nodes[i].position;
            out.nodes_.push_back(n);
        }
    }
    if (out.nodes_.size() > config.max_nodes) return make_error(ErrorCode::LimitExceeded, "routing graph: max_nodes");

    // 3. Edges between consecutive decision nodes of each way.
    std::vector<Edge> edges;
    for (std::size_t w = 0; w < graph.ways.size(); ++w) {
        if (!use[w].used) continue;
        const auto& r = refs[w];
        const auto pos = [&](std::size_t i) { return graph.nodes[r[i]].position; };
        std::size_t start = 0;
        std::uint64_t length = 0;
        std::int64_t first_dx = 0, first_dy = 0;
        bool have_first = false;
        for (std::size_t i = 1; i < r.size(); ++i) {
            if (r[i] == r[i - 1]) continue;
            const geom::PointMm a = pos(i - 1), b = pos(i);
            length += ceil_length_mm(a, b);
            if (!have_first) { first_dx = b.x - a.x; first_dy = b.y - a.y; have_first = true; }
            if (compact[r[i]] == kNoArc) continue;
            // OSM ref-pair index of the edge's first segment: the first i' > start with a distinct ref.
            std::size_t seg_first = start;
            while (r[seg_first + 1] == r[seg_first]) ++seg_first;
            Edge e{compact[r[start]], compact[r[i]], w, static_cast<std::uint32_t>(seg_first),
                   static_cast<std::uint32_t>(i - 1), length, first_dx, first_dy, b.x - a.x, b.y - a.y};
            if (e.length_mm > std::numeric_limits<std::uint32_t>::max())
                return make_error(ErrorCode::LimitExceeded, "routing graph: edge longer than 4294 km");
            edges.push_back(e);
            start = i;
            length = 0;
            have_first = false;
        }
    }

    // 4. Arcs, canonical order (from node, way id, first segment, forward first).
    std::vector<RouteArc>& arcs = out.arcs_;
    for (const Edge& e : edges) {
        const WayUse& u = use[e.way];
        ++out.nodes_[e.from].degree;
        ++out.nodes_[e.to].degree;
        for (const bool forward : {true, false}) {
            if (forward ? !u.dirs.forward : !u.dirs.backward) continue;
            RouteArc a;
            a.from = forward ? e.from : e.to;
            a.to = forward ? e.to : e.from;
            a.way_id = graph.ways[e.way].osm_id;
            a.seg_first = e.seg_first;
            a.seg_last = e.seg_last;
            a.forward = forward;
            a.rank = static_cast<std::uint8_t>(std::clamp(graph.ways[e.way].rank, 0, 255));
            a.speed_kmh = forward ? u.fwd_kmh : u.bwd_kmh;
            a.length_mm = static_cast<std::uint32_t>(e.length_mm);
            const std::uint64_t num = e.length_mm * 36u, den = std::uint64_t{a.speed_kmh} * 10u;
            a.cost_ms = static_cast<std::uint32_t>((num + den - 1) / den);
            if (forward) { a.in_dx = e.first_dx; a.in_dy = e.first_dy; a.out_dx = e.last_dx; a.out_dy = e.last_dy; }
            else { a.in_dx = -e.last_dx; a.in_dy = -e.last_dy; a.out_dx = -e.first_dx; a.out_dy = -e.first_dy; }
            arcs.push_back(a);
        }
    }
    if (arcs.size() > config.max_arcs) return make_error(ErrorCode::LimitExceeded, "routing graph: max_arcs");
    std::sort(arcs.begin(), arcs.end(), [](const RouteArc& a, const RouteArc& b) {
        if (a.from != b.from) return a.from < b.from;
        if (a.way_id != b.way_id) return a.way_id < b.way_id;
        if (a.seg_first != b.seg_first) return a.seg_first < b.seg_first;
        return a.forward > b.forward;
    });
    {
        // Pair the two directions of each edge: (way, seg_first) identifies the edge.
        std::vector<std::uint32_t> order(arcs.size());
        for (std::uint32_t i = 0; i < order.size(); ++i) order[i] = i;
        std::sort(order.begin(), order.end(), [&](std::uint32_t a, std::uint32_t b) {
            if (arcs[a].way_id != arcs[b].way_id) return arcs[a].way_id < arcs[b].way_id;
            if (arcs[a].seg_first != arcs[b].seg_first) return arcs[a].seg_first < arcs[b].seg_first;
            return arcs[a].forward > arcs[b].forward;
        });
        for (std::size_t i = 0; i + 1 < order.size(); ++i) {
            RouteArc& a = arcs[order[i]];
            RouteArc& b = arcs[order[i + 1]];
            if (a.way_id == b.way_id && a.seg_first == b.seg_first) {
                a.reverse = order[i + 1];
                b.reverse = order[i];
            }
        }
    }
    {
        std::uint32_t at = 0;
        for (std::uint32_t n = 0; n < out.nodes_.size(); ++n) {
            out.nodes_[n].first_out = at;
            while (at < arcs.size() && arcs[at].from == n) ++at;
        }
    }

    // 5. Via-node rules, sorted by (node, from way, to way, kind).
    std::vector<std::pair<std::uint32_t, RouteRule>> rules;
    std::unordered_map<std::int64_t, char> used_way;
    for (std::size_t w = 0; w < graph.ways.size(); ++w) if (use[w].used) used_way.emplace(graph.ways[w].osm_id, 1);
    std::vector<std::pair<std::uint32_t, std::int64_t>> arrivals; // (head node, way) of every arc
    arrivals.reserve(arcs.size());
    for (const RouteArc& a : arcs) arrivals.emplace_back(a.to, a.way_id);
    std::sort(arrivals.begin(), arrivals.end());
    for (const TurnRestriction& tr : restrictions) {
        if (tr.via_node == 0) { ++stats.restrictions_via_way; continue; }
        const std::uint32_t v = out.find_node(tr.via_node);
        bool from_arrives = false, to_leaves = false;
        if (v != kNoArc && used_way.count(tr.from_way) && used_way.count(tr.to_way)) {
            from_arrives = std::binary_search(arrivals.begin(), arrivals.end(), std::make_pair(v, tr.from_way));
            for (std::uint32_t b = out.nodes_[v].first_out; b < arcs.size() && arcs[b].from == v; ++b)
                if (arcs[b].way_id == tr.to_way) to_leaves = true;
        }
        if (!from_arrives || !to_leaves) { ++stats.restrictions_unmatched; continue; }
        rules.push_back({v, RouteRule{tr.from_way, tr.to_way, tr.kind}});
        ++stats.restrictions_applied;
    }
    std::sort(rules.begin(), rules.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first < b.first;
        if (a.second.from_way != b.second.from_way) return a.second.from_way < b.second.from_way;
        if (a.second.to_way != b.second.to_way) return a.second.to_way < b.second.to_way;
        return a.second.kind < b.second.kind;
    });
    {
        std::size_t at = 0;
        for (std::uint32_t n = 0; n < out.nodes_.size(); ++n) {
            out.nodes_[n].first_rule = static_cast<std::uint32_t>(at);
            while (at < rules.size() && rules[at].first == n) ++at;
        }
        for (const auto& [node, rule] : rules) out.rules_.push_back(rule);
    }
    RouteGraphNode sentinel;
    sentinel.first_out = static_cast<std::uint32_t>(arcs.size());
    sentinel.first_rule = static_cast<std::uint32_t>(out.rules_.size());
    out.nodes_.push_back(sentinel);
    return out;
}

} // namespace g2m::route
