// g2m/route/routing_graph.h - the in-memory routable car graph (G4 slice 1,
// docs/routing_design.md).
//
// Built from a complete RoadGraph (one tile's graph, or several merged with
// merge_road_graphs) plus turn restrictions. Nodes are the decision points
// of the road graph (way ends, nodes shared by two or more way positions and
// restriction via nodes); every OSM shape node in between is collapsed into
// the edge, which keeps its identity as (way id, first and last OSM
// ref-pair index). Each traversable direction of an edge is one ARC. The
// search runs on arcs (edge-based), so turn restrictions and turn costs are
// transitions between arcs.
//
// Determinism: integer millimetres and integer milliseconds throughout; the
// only floating-point steps are det::sqrt per segment (rounded up to whole
// mm) and det::atan2 for the turn-angle class. Arc order is canonical
// (from node index, way id, first segment, direction), so the same input
// gives bit-identical graphs and routes on every platform.
#pragma once

#include "g2m/core/geom/exact.h"
#include "g2m/core/result.h"
#include "g2m/layer/road_graph.h"
#include "g2m/route/turn_restriction.h"

#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace g2m::route {

inline constexpr std::uint32_t kNoArc = 0xffffffffu;

struct RouteClassSpeed {
    std::string highway;        // OSM highway value
    std::uint16_t default_kmh = 0; // speed when maxspeed is absent or symbolic
    bool routable = true;
};

// Every number is configuration (defaults in default_route_config()).
struct RouteConfig {
    std::vector<RouteClassSpeed> classes;                       // unlisted highway values are not routable
    std::vector<std::pair<std::string, std::uint16_t>> zone_speeds; // symbolic maxspeed values, e.g. DE:urban -> 50
    std::vector<std::string> blocked_access;                    // access/motor_vehicle/motorcar values that close a way
    std::vector<std::string> implied_oneway;                    // highway values that are one-way without a oneway tag
    std::uint16_t min_speed_kmh = 5;
    std::uint16_t max_speed_kmh = 130;                          // cap on every arc; the A* heuristic divides by it
    // Turn costs (ms) at nodes with three or more incident edge ends. The
    // angle is the heading change from the arriving to the leaving segment,
    // positive = left (counter-clockwise). Right-hand traffic: lefts cost more.
    std::int32_t straight_max_deg = 20;
    std::int32_t slight_max_deg = 60;
    std::int32_t turn_max_deg = 120;
    std::uint32_t slight_ms = 500;
    std::uint32_t right_ms = 2000;
    std::uint32_t left_ms = 4000;
    std::uint32_t sharp_right_ms = 4000;
    std::uint32_t sharp_left_ms = 6000;
    std::uint32_t u_turn_ms = 20000; // only where nothing else leaves the node (dead end)
    // Limits.
    std::size_t max_nodes = 4u << 20;
    std::size_t max_arcs = 16u << 20;
    std::size_t max_settled = 8u << 20; // per query; LimitExceeded beyond
};
const RouteConfig& default_route_config();

struct RouteGraphNode {
    std::int64_t osm_id = 0;
    geom::PointMm position;
    std::uint32_t first_out = 0;  // arcs [first_out, next node's first_out) leave this node
    std::uint32_t first_rule = 0; // rules [first_rule, next node's first_rule) have this via node
    std::uint16_t degree = 0;     // incident edge ends (a loop counts twice), any direction
};

struct RouteArc {
    std::uint32_t from = 0, to = 0; // node indices
    std::int64_t way_id = 0;
    std::uint32_t seg_first = 0, seg_last = 0; // OSM ref-pair index range of the way, inclusive
    bool forward = true;                       // along the way's ref order
    std::uint8_t rank = 0;                     // RoadStyle rank of the way's class
    std::uint16_t speed_kmh = 0;
    std::uint32_t length_mm = 0;
    std::uint32_t cost_ms = 0;
    std::uint32_t reverse = kNoArc;            // the same edge in the other direction, if traversable
    // Direction of the first and last segment (mm, unnormalised) for turn angles.
    std::int64_t in_dx = 0, in_dy = 0, out_dx = 0, out_dy = 0;
};

// A via-node rule as the search uses it: arriving on from_way at the node.
struct RouteRule {
    std::int64_t from_way = 0, to_way = 0;
    RestrictionKind kind = RestrictionKind::No;
};

struct RouteBuildStats {
    std::size_t ways_used = 0, ways_not_routable = 0, ways_access_blocked = 0, ways_no_direction = 0;
    std::size_t restrictions_applied = 0;
    std::size_t restrictions_via_way = 0;  // not supported yet (design doc, slice 3)
    std::size_t restrictions_unmatched = 0; // a member way or the via node is not in the routable graph
};

struct Transition {
    std::uint32_t arc = kNoArc;
    std::uint32_t turn_ms = 0;
};

class RoutingGraph {
public:
    std::span<const RouteGraphNode> nodes() const { return {nodes_.data(), nodes_.size() - 1}; }
    std::span<const RouteArc> arcs() const { return arcs_; }
    std::span<const RouteRule> rules() const { return rules_; }
    const RouteConfig& config() const { return config_; }
    const RouteBuildStats& stats() const { return stats_; }
    // Node index of an OSM node id, or kNoArc.
    std::uint32_t find_node(std::int64_t osm_id) const;
    std::span<const RouteArc> out_arcs(std::uint32_t node) const {
        return {arcs_.data() + nodes_[node].first_out, nodes_[node + 1].first_out - nodes_[node].first_out};
    }
    // The arcs a vehicle on `arc` may continue onto at its head node, with
    // the turn cost, in arc order. Applies via-node restrictions and the
    // dead-end-only U-turn rule. `out` is cleared first.
    void successors(std::uint32_t arc, std::vector<Transition>& out) const;
    // Turn cost of arriving on `in` and leaving on `out` (no restriction check).
    std::uint32_t turn_cost_ms(const RouteArc& in, const RouteArc& out) const;

private:
    friend Result<RoutingGraph> build_routing_graph(const RoadGraph&, std::span<const TurnRestriction>, const RouteConfig&);
    RouteConfig config_;
    std::vector<RouteGraphNode> nodes_; // + one sentinel
    std::vector<RouteArc> arcs_;
    std::vector<RouteRule> rules_;
    RouteBuildStats stats_;
};

// `graph` must be canonical (sorted nodes and ways, every ref present), as
// build_road_graph and merge_road_graphs produce. Restrictions whose ways or
// via node are not in the routable graph are counted and ignored.
Result<RoutingGraph> build_routing_graph(const RoadGraph& graph, std::span<const TurnRestriction> restrictions,
                                         const RouteConfig& config = default_route_config());

} // namespace g2m::route
