// g2m/route/route_search.h - edge-based A* over a RoutingGraph, optionally
// with ALT landmark bounds (G4 slices 1-2, docs/routing_design.md).
//
// The search state is an arc, so restrictions and turn costs are exact.
// Cost = integer milliseconds (arc travel time + turn costs; a partial first
// or last arc is charged by RoutingGraph::partial_cost_ms).
// Heuristic = the larger of
//  - the straight line from the arc's head to the target at
//    RouteConfig::max_speed_kmh (rounded down, 2 mm slack), and
//  - with RouteLandmarks: the ALT bound to the target node(s).
// Both are admissible, so the result is optimal; states are reopened when a
// cheaper g appears, so no consistency assumption is needed. The search stops
// when the smallest f in the open list reaches the best goal cost found.
// Ties: the open list orders by (f, g, arc index); a parent or the goal
// changes only on a strictly cheaper cost, so equal-cost alternatives
// resolve the same way on every run and platform.
#pragma once

#include "g2m/route/routing_graph.h"

#include <cstdint>
#include <span>
#include <vector>

namespace g2m::route {

// ALT (A*, landmarks, triangle inequality) lower bounds: per landmark L the
// node-graph travel cost L -> v and v -> L over arc costs (no turn costs,
// no restrictions; both only add cost, so the bound stays admissible for the
// edge-based search). Landmarks are chosen by farthest-point selection
// starting from node 0, preferring nodes no landmark reaches yet (so each
// connected component gets one); deterministic. Immutable after build; share
// it between searchers and threads.
class RouteLandmarks {
public:
    static constexpr std::uint32_t kUnreachable = 0xffffffffu;
    // count 1..64 (capped at the node count). Cost: 2 * count Dijkstras.
    static Result<RouteLandmarks> build(const RoutingGraph& graph, std::size_t count = 16);
    std::size_t count() const { return count_; }
    std::span<const std::uint32_t> landmark_nodes() const { return nodes_; }
    // Lower bound on the travel cost from node v to node t.
    std::uint64_t bound(std::uint32_t v, std::uint32_t t) const;
    const RoutingGraph* graph() const { return graph_; }

private:
    const RoutingGraph* graph_ = nullptr;
    std::size_t count_ = 0;
    std::vector<std::uint32_t> nodes_;
    std::vector<std::uint32_t> from_, to_; // [node * count + landmark]
};

struct RouteQuery {
    std::int64_t from_node = 0; // OSM node ids of graph nodes
    std::int64_t to_node = 0;
};

struct Route {
    bool reachable = false;
    std::uint64_t cost_ms = 0;       // travel + turn costs
    std::uint64_t length_mm = 0;     // travelled, partial first/last arc included
    std::vector<std::uint32_t> arcs; // in travel order; empty for a node query with from == to
    std::uint32_t start_offset_mm = 0; // where travel starts on arcs.front()
    std::uint32_t end_offset_mm = 0;   // where travel ends on arcs.back()
};

struct RouteSearchStats {
    std::size_t settled = 0; // states popped and expanded
    std::size_t pushed = 0;
    std::size_t reopened = 0;
};

// Reusable search workspace for one graph (not thread-safe; one per thread).
// The graph (and landmarks) must outlive it. After the first queries have
// grown the open list, a query allocates nothing except the returned route
// (generation-stamped per-arc arrays, reused buffers).
class RouteSearcher {
public:
    explicit RouteSearcher(const RoutingGraph& graph, const RouteLandmarks* landmarks = nullptr);
    // Node to node. InvalidArgument for an unknown node id or landmarks of
    // another graph; LimitExceeded past RouteConfig::max_settled. An
    // unreachable target is a valid Route with reachable = false.
    Result<Route> find(const RouteQuery& query);
    // Point to point (RoutingGraph::snap results). Travel starts on one of
    // from's candidates at its offset and ends on one of to's candidates.
    Result<Route> find(const RoutePoint& from, const RoutePoint& to);
    const RouteSearchStats& stats() const { return stats_; }

private:
    struct Open {
        std::uint64_t f, g;
        std::uint32_t arc;
        friend bool operator>(const Open& a, const Open& b) {
            if (a.f != b.f) return a.f > b.f;
            if (a.g != b.g) return a.g > b.g;
            return a.arc > b.arc;
        }
    };
    struct Goal {
        std::uint32_t arc = kNoArc;
        std::uint32_t offset_mm = 0;
    };
    void begin();
    Result<Route> run();
    std::uint64_t heuristic(std::uint32_t node) const;
    void relax(std::uint32_t arc, std::uint64_t pre, std::uint32_t parent);
    void offer_goal(std::uint64_t cost, std::uint32_t arc, std::uint32_t parent, std::uint32_t end_offset, bool same_arc);

    const RoutingGraph* graph_;
    const RouteLandmarks* landmarks_;
    std::vector<std::uint64_t> g_;
    std::vector<std::uint32_t> parent_;
    std::vector<std::uint32_t> stamp_;
    std::uint32_t generation_ = 0;
    std::vector<Open> heap_;
    std::vector<Transition> succ_;
    RouteSearchStats stats_;
    // The current query.
    geom::PointMm target_{};
    bool node_mode_ = true;
    std::uint32_t target_node_ = kNoArc;
    Goal goals_[2]{};
    std::uint32_t goal_count_ = 0;
    std::uint32_t alt_targets_[2]{};
    std::uint32_t alt_target_count_ = 0;
    Goal starts_[2]{};
    std::uint32_t same_start_offset_ = 0;
    std::uint64_t best_goal_ = 0;
    std::uint32_t best_arc_ = kNoArc, best_parent_ = kNoArc, best_end_offset_ = 0;
    bool best_same_arc_ = false;
};

} // namespace g2m::route
