// tests/route/route_reference.h - test-only references for g2m_route:
// a plain Dijkstra over the same edge-based states (no heuristic, std::set
// open list), a route validator, and a small synthetic RoadGraph builder.
#pragma once

#include "g2m/route/route_search.h"

#include <algorithm>
#include <climits>
#include <cstdint>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace g2m_test {

// Shortest cost from from_node to to_node; nullopt-like: reachable=false.
inline g2m::route::Route dijkstra_reference(const g2m::route::RoutingGraph& graph, std::int64_t from_node,
                                            std::int64_t to_node) {
    using namespace g2m::route;
    Route route;
    const std::uint32_t s = graph.find_node(from_node), t = graph.find_node(to_node);
    if (s == kNoArc || t == kNoArc) return route;
    if (s == t) { route.reachable = true; return route; }
    const auto arcs = graph.arcs();
    std::vector<std::uint64_t> dist(arcs.size(), UINT64_MAX);
    std::vector<std::uint32_t> parent(arcs.size(), kNoArc);
    std::set<std::pair<std::uint64_t, std::uint32_t>> open;
    for (std::uint32_t a = 0; a < arcs.size(); ++a) {
        if (arcs[a].from != s) continue;
        dist[a] = arcs[a].cost_ms;
        open.insert({dist[a], a});
    }
    std::vector<Transition> succ;
    while (!open.empty()) {
        const auto [d, a] = *open.begin();
        open.erase(open.begin());
        if (arcs[a].to == t) {
            route.reachable = true;
            route.cost_ms = d;
            for (std::uint32_t x = a; x != kNoArc; x = parent[x]) route.arcs.insert(route.arcs.begin(), x);
            for (const auto x : route.arcs) route.length_mm += arcs[x].length_mm;
            return route;
        }
        graph.successors(a, succ);
        for (const Transition& tr : succ) {
            const std::uint64_t nd = d + tr.turn_ms + arcs[tr.arc].cost_ms;
            if (nd < dist[tr.arc]) {
                if (dist[tr.arc] != UINT64_MAX) open.erase({dist[tr.arc], tr.arc});
                dist[tr.arc] = nd;
                parent[tr.arc] = a;
                open.insert({nd, tr.arc});
            }
        }
    }
    return route;
}

// Checks connectivity, allowed transitions and the cost sum; returns "" or a reason.
inline std::string validate_route(const g2m::route::RoutingGraph& graph, const g2m::route::Route& route,
                                  std::int64_t from_node, std::int64_t to_node) {
    using namespace g2m::route;
    if (!route.reachable) return "";
    const auto arcs = graph.arcs();
    if (route.arcs.empty()) return from_node == to_node ? "" : "empty route";
    if (graph.nodes()[arcs[route.arcs.front()].from].osm_id != from_node) return "wrong start";
    if (graph.nodes()[arcs[route.arcs.back()].to].osm_id != to_node) return "wrong end";
    std::uint64_t cost = arcs[route.arcs.front()].cost_ms, length = arcs[route.arcs.front()].length_mm;
    std::vector<Transition> succ;
    for (std::size_t i = 1; i < route.arcs.size(); ++i) {
        graph.successors(route.arcs[i - 1], succ);
        bool found = false;
        for (const auto& tr : succ)
            if (tr.arc == route.arcs[i]) { cost += tr.turn_ms + arcs[tr.arc].cost_ms; found = true; }
        if (!found) return "forbidden or disconnected transition at step " + std::to_string(i);
        length += arcs[route.arcs[i]].length_mm;
    }
    if (cost != route.cost_ms) return "cost mismatch";
    if (length != route.length_mm) return "length mismatch";
    return "";
}

// Synthetic graphs in plain millimetre coordinates (no projection).
struct GraphBuilder {
    g2m::RoadGraph graph;
    void node(std::int64_t id, std::int64_t x_m, std::int64_t y_m) {
        graph.nodes.push_back({id, {x_m * 1000, y_m * 1000}, {}});
    }
    void way(std::int64_t id, std::vector<std::int64_t> refs, std::vector<std::pair<std::string, std::string>> tags = {}) {
        bool has_highway = false;
        for (const auto& t : tags) has_highway = has_highway || t.first == "highway";
        if (!has_highway) tags.emplace_back("highway", "residential");
        std::sort(tags.begin(), tags.end());
        graph.ways.push_back({id, 3, std::move(refs), std::move(tags)});
    }
    g2m::RoadGraph build() {
        auto g = graph;
        std::sort(g.nodes.begin(), g.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
        std::sort(g.ways.begin(), g.ways.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
        return g;
    }
};

inline g2m::route::TurnRestriction via_node_rule(std::int64_t from, std::int64_t via, std::int64_t to, const char* value) {
    g2m::route::TurnRestriction r;
    r.relation_id = 1;
    r.kind = std::string(value).rfind("only_", 0) == 0 ? g2m::route::RestrictionKind::Only : g2m::route::RestrictionKind::No;
    r.value = value;
    r.from_way = from;
    r.via_node = via;
    r.to_way = to;
    return r;
}

} // namespace g2m_test
