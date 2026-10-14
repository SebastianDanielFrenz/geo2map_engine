// g2m/route/route_search.h - A* over a RoutingGraph (G4 slice 1,
// docs/routing_design.md).
//
// Edge-based: the search state is an arc, so restrictions and turn costs are
// exact. Cost = integer milliseconds (arc travel time + turn costs).
// Heuristic = straight-line mm from the arc's head to the target at
// RouteConfig::max_speed_kmh, rounded down: admissible, so the first time
// the target is popped the route is optimal. States are reopened when a
// cheaper g appears, so no consistency assumption is needed.
// Ties: the open list orders by (f, g, arc index) and a parent changes only
// on a strictly cheaper g, so equal-cost alternatives resolve the same way
// on every run and platform.
#pragma once

#include "g2m/route/routing_graph.h"

#include <cstdint>
#include <vector>

namespace g2m::route {

struct RouteQuery {
    std::int64_t from_node = 0; // OSM node ids of graph nodes
    std::int64_t to_node = 0;
};

struct Route {
    bool reachable = false;
    std::uint64_t cost_ms = 0;       // travel + turn costs
    std::uint64_t length_mm = 0;
    std::vector<std::uint32_t> arcs; // in travel order; empty when from == to
};

struct RouteSearchStats {
    std::size_t settled = 0; // states popped and expanded
    std::size_t pushed = 0;
    std::size_t reopened = 0;
};

// Reusable search workspace for one graph (not thread-safe; use one per
// thread). The graph must outlive it. Per-query cost is proportional to the
// states touched, not the graph size (generation-stamped arrays).
class RouteSearcher {
public:
    explicit RouteSearcher(const RoutingGraph& graph);
    // InvalidArgument for an unknown node id; LimitExceeded past
    // RouteConfig::max_settled. An unreachable target is a valid Route with
    // reachable = false.
    Result<Route> find(const RouteQuery& query);
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
    std::uint64_t heuristic(std::uint32_t node) const;
    const RoutingGraph* graph_;
    std::vector<std::uint64_t> g_;
    std::vector<std::uint32_t> parent_;
    std::vector<std::uint32_t> stamp_;
    std::uint32_t generation_ = 0;
    std::vector<Open> heap_;
    std::vector<Transition> succ_;
    geom::PointMm target_{};
    RouteSearchStats stats_;
};

} // namespace g2m::route
