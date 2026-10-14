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

// Plain Dijkstra between two route points: starts on each from-candidate at its
// offset, ends on any to-candidate (the same goal semantics as RouteSearcher).
inline g2m::route::Route dijkstra_point_reference(const g2m::route::RoutingGraph& graph, const g2m::route::RoutePoint& from,
                                                  const g2m::route::RoutePoint& to) {
    using namespace g2m::route;
    const auto arcs = graph.arcs();
    Route best;
    std::uint64_t best_cost = UINT64_MAX;
    std::uint32_t best_arc = kNoArc, best_parent = kNoArc, best_end = 0, best_start = 0;
    bool same = false;
    std::vector<std::uint64_t> dist(arcs.size(), UINT64_MAX);
    std::vector<std::uint32_t> parent(arcs.size(), kNoArc), start_offset(arcs.size(), 0);
    std::set<std::pair<std::uint64_t, std::uint32_t>> open;
    for (std::uint32_t i = 0; i < from.count; ++i) {
        const auto [a, o] = from.candidates[i];
        for (std::uint32_t j = 0; j < to.count; ++j)
            if (to.candidates[j].arc == a && to.candidates[j].offset_mm >= o) {
                const auto c = graph.partial_cost_ms(a, o, to.candidates[j].offset_mm);
                if (c < best_cost) { best_cost = c; best_arc = a; best_end = to.candidates[j].offset_mm; best_start = o; same = true; }
            }
        dist[a] = graph.partial_cost_ms(a, o, arcs[a].length_mm);
        start_offset[a] = o;
        open.insert({dist[a], a});
    }
    std::vector<Transition> succ;
    while (!open.empty() && open.begin()->first < best_cost) {
        const auto [d, a] = *open.begin();
        open.erase(open.begin());
        graph.successors(a, succ);
        for (const Transition& tr : succ) {
            const std::uint64_t pre = d + tr.turn_ms;
            for (std::uint32_t j = 0; j < to.count; ++j)
                if (to.candidates[j].arc == tr.arc) {
                    const auto c = pre + graph.partial_cost_ms(tr.arc, 0, to.candidates[j].offset_mm);
                    if (c < best_cost) { best_cost = c; best_arc = tr.arc; best_parent = a; best_end = to.candidates[j].offset_mm; same = false; }
                }
            const std::uint64_t nd = pre + arcs[tr.arc].cost_ms;
            if (nd < dist[tr.arc]) {
                if (dist[tr.arc] != UINT64_MAX) open.erase({dist[tr.arc], tr.arc});
                dist[tr.arc] = nd;
                parent[tr.arc] = a;
                open.insert({nd, tr.arc});
            }
        }
    }
    if (best_arc == kNoArc) return best;
    best.reachable = true;
    best.cost_ms = best_cost;
    best.arcs.push_back(best_arc);
    if (!same) for (std::uint32_t x = best_parent; x != kNoArc; x = parent[x]) best.arcs.insert(best.arcs.begin(), x);
    best.start_offset_mm = same ? best_start : start_offset[best.arcs.front()];
    best.end_offset_mm = best_end;
    return best;
}

// Checks a point route: starts on a from-candidate at its offset, ends on a
// to-candidate, allowed transitions, cost and length; returns "" or a reason.
inline std::string validate_point_route(const g2m::route::RoutingGraph& graph, const g2m::route::Route& route,
                                        const g2m::route::RoutePoint& from, const g2m::route::RoutePoint& to) {
    using namespace g2m::route;
    if (!route.reachable) return "";
    if (route.arcs.empty()) return "empty route";
    const auto arcs = graph.arcs();
    bool start_ok = false, end_ok = false;
    for (std::uint32_t i = 0; i < from.count; ++i)
        start_ok = start_ok || (from.candidates[i].arc == route.arcs.front() && from.candidates[i].offset_mm == route.start_offset_mm);
    for (std::uint32_t i = 0; i < to.count; ++i)
        end_ok = end_ok || (to.candidates[i].arc == route.arcs.back() && to.candidates[i].offset_mm == route.end_offset_mm);
    if (!start_ok) return "start not on a from candidate";
    if (!end_ok) return "end not on a to candidate";
    std::uint64_t cost = 0, length = 0;
    if (route.arcs.size() == 1) {
        if (route.end_offset_mm < route.start_offset_mm) return "single arc runs backwards";
        cost = graph.partial_cost_ms(route.arcs[0], route.start_offset_mm, route.end_offset_mm);
        length = route.end_offset_mm - route.start_offset_mm;
    } else {
        const auto first = route.arcs.front();
        cost = graph.partial_cost_ms(first, route.start_offset_mm, arcs[first].length_mm);
        length = arcs[first].length_mm - route.start_offset_mm;
        std::vector<Transition> succ;
        for (std::size_t i = 1; i < route.arcs.size(); ++i) {
            graph.successors(route.arcs[i - 1], succ);
            bool found = false;
            for (const auto& tr : succ) {
                if (tr.arc != route.arcs[i]) continue;
                found = true;
                const bool last = i + 1 == route.arcs.size();
                cost += tr.turn_ms + (last ? graph.partial_cost_ms(tr.arc, 0, route.end_offset_mm) : arcs[tr.arc].cost_ms);
                length += last ? route.end_offset_mm : arcs[tr.arc].length_mm;
            }
            if (!found) return "forbidden or disconnected transition at step " + std::to_string(i);
        }
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
