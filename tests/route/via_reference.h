// tests/route/via_reference.h - test-only reference for via-way turn restrictions (G4 slice 3).
// Independent of the library's rule states: a path is checked by comparing way ids of its runs
// (maximal stretches on one way), and a reference Dijkstra searches (arc, last runs) states.
#pragma once

#include "route_reference.h"
#include "splitmix64.h"

#include <map>
#include <set>
#include <tuple>
#include <vector>

namespace g2m_test {

using g2m::RoadGraph;
using g2m::route::kNoArc;
using g2m::route::RestrictionKind;
using g2m::route::Route;
using g2m::route::RouteArc;
using g2m::route::RoutingGraph;
using g2m::route::Transition;
using g2m::route::TurnRestriction;

inline TurnRestriction via_way_rule(std::int64_t relation, const char* value, std::int64_t from, std::vector<std::int64_t> via,
                                    std::int64_t to) {
    TurnRestriction r;
    r.relation_id = relation;
    r.value = value;
    std::size_t code = 0;
    g2m::restriction_value_code(value, code);
    r.kind = g2m::restriction_value_kind(code);
    r.from_way = from;
    r.via_ways = std::move(via);
    r.to_way = to;
    return r;
}

struct RunStep {
    std::int64_t way;
    std::int64_t entry; // OSM id of the node the run was entered at
    friend bool operator<(const RunStep& a, const RunStep& b) { return std::tie(a.way, a.entry) < std::tie(b.way, b.entry); }
};

struct ViaReference {
    std::vector<TurnRestriction> rules; // via-way rules only
    std::map<std::int64_t, std::set<std::int64_t>> nodes_of_way;
    ViaReference(const RoadGraph& graph, const std::vector<TurnRestriction>& all) {
        for (const auto& r : all) if (r.via_node == 0) rules.push_back(r);
        for (const auto& w : graph.ways) nodes_of_way[w.osm_id].insert(w.node_ids.begin(), w.node_ids.end());
    }
    bool shared(std::int64_t a, std::int64_t b, std::int64_t node) const {
        return nodes_of_way.at(a).count(node) && nodes_of_way.at(b).count(node);
    }
    // May a vehicle whose runs so far are `runs` (the last one is the current way) and which is at
    // node `at` leave onto way `next`?
    bool allows(const std::vector<RunStep>& runs, std::int64_t at, std::int64_t next) const {
        if (next == runs.back().way) return true; // same run
        // Several Only rules that apply at one boundary allow the union of their next ways (as the
        // via-node rules do); a No rule forbids on its own.
        bool only_seen = false, only_ok = false;
        for (const auto& r : rules) {
            std::vector<std::int64_t> ways{r.from_way};
            ways.insert(ways.end(), r.via_ways.begin(), r.via_ways.end());
            ways.push_back(r.to_way);
            const std::size_t m = ways.size();
            for (std::size_t p = 1; p + 1 <= m; ++p) { // the last p runs are the prefix ways[0..p-1]
                if (runs.size() < p) break;
                bool match = true;
                for (std::size_t q = 0; q < p && match; ++q) {
                    const RunStep& run = runs[runs.size() - p + q];
                    match = run.way == ways[q] && (q == 0 || shared(ways[q - 1], ways[q], run.entry));
                }
                if (!match) continue;
                if (r.kind == RestrictionKind::No) {
                    if (p == m - 1 && next == ways[p] && shared(ways[p - 1], ways[p], at)) return false;
                } else if (p >= 2 || shared(ways[0], ways[1], at)) {
                    only_seen = true;
                    only_ok = only_ok || next == ways[p];
                }
            }
        }
        return !only_seen || only_ok;
    }
};

inline constexpr std::size_t kHistory = 4;

// Dijkstra over (arc, last runs). Node-to-node, same goal semantics as RouteSearcher::find(RouteQuery).
inline Route route_via_reference(const RoutingGraph& g, const ViaReference& ref, std::int64_t from, std::int64_t to) {
    using State = std::pair<std::uint32_t, std::vector<RunStep>>;
    Route route;
    const std::uint32_t s = g.find_node(from), t = g.find_node(to);
    if (s == kNoArc || t == kNoArc) return route;
    if (s == t) { route.reachable = true; return route; }
    const auto arcs = g.arcs();
    std::map<State, std::uint64_t> dist;
    std::set<std::pair<std::uint64_t, State>> open;
    for (std::uint32_t a = 0; a < arcs.size(); ++a) {
        if (arcs[a].from != s) continue;
        State st{a, {RunStep{arcs[a].way_id, from}}};
        dist[st] = arcs[a].cost_ms;
        open.insert({arcs[a].cost_ms, st});
    }
    std::vector<Transition> succ;
    while (!open.empty()) {
        const auto [d, st] = *open.begin();
        open.erase(open.begin());
        const auto& [a, runs] = st;
        if (arcs[a].to == t) { route.reachable = true; route.cost_ms = d; return route; }
        g.successors(a, succ);
        const std::int64_t at = g.nodes()[arcs[a].to].osm_id;
        for (const Transition& tr : succ) {
            const auto next_way = arcs[tr.arc].way_id;
            if (!ref.allows(runs, at, next_way)) continue;
            auto next_runs = runs;
            if (next_way != runs.back().way) next_runs.push_back({next_way, at});
            if (next_runs.size() > kHistory) next_runs.erase(next_runs.begin());
            State ns{tr.arc, std::move(next_runs)};
            const std::uint64_t nd = d + tr.turn_ms + arcs[tr.arc].cost_ms;
            const auto it = dist.find(ns);
            if (it != dist.end() && it->second <= nd) continue;
            if (it != dist.end()) open.erase({it->second, ns});
            dist[ns] = nd;
            open.insert({nd, std::move(ns)});
        }
    }
    return route;
}

// Does the route break a via-way rule? (reference semantics)
inline bool route_violates(const RoutingGraph& g, const ViaReference& ref, const Route& route) {
    const auto arcs = g.arcs();
    std::vector<RunStep> runs;
    for (std::size_t i = 0; i < route.arcs.size(); ++i) {
        const RouteArc& b = arcs[route.arcs[i]];
        if (i == 0) { runs.push_back({b.way_id, g.nodes()[b.from].osm_id}); continue; }
        const std::int64_t at = g.nodes()[arcs[route.arcs[i - 1]].to].osm_id;
        if (!ref.allows(runs, at, b.way_id)) return true;
        if (b.way_id != runs.back().way) runs.push_back({b.way_id, at});
    }
    return false;
}

// Appends via-way rules chained at way ends of `graph` (F, one or two V, T), only along directions a car can drive: F, V (and sometimes V2), T chained at way ends.
inline void add_via_way_rules(const RoadGraph& graph, std::vector<TurnRestriction>& rules, std::uint64_t seed, double share,
                              std::int64_t first_relation = 100000) {
    SplitMix64 rng(seed ^ 0x5eed);
    const auto& ways = graph.ways;
    std::map<std::int64_t, std::vector<std::size_t>> at_end; // node -> ways with that node as first/last ref
    for (std::size_t w = 0; w < ways.size(); ++w) {
        at_end[ways[w].node_ids.front()].push_back(w);
        at_end[ways[w].node_ids.back()].push_back(w);
    }
    // Only ways the router uses at all (others are ignored by the library, by design).
    std::set<std::int64_t> routable;
    if (const auto plain = g2m::route::build_routing_graph(graph, {}); plain)
        for (const auto& a : plain->arcs()) routable.insert(a.way_id);
    const auto dirs = [&](std::size_t w) {
        auto d = g2m::route::way_directions(ways[w]);
        if (!routable.count(ways[w].osm_id)) d = {false, false};
        return d;
    };
    // The car can enter way w at node `start` and leave it at its other end.
    const auto runs_from = [&](std::size_t w, std::int64_t start) {
        const bool forward = ways[w].node_ids.front() == start;
        return forward ? dirs(w).forward : dirs(w).backward;
    };
    const auto other_end = [&](std::size_t w, std::int64_t e) {
        return ways[w].node_ids.front() == e ? ways[w].node_ids.back() : ways[w].node_ids.front();
    };
    // Only chains whose consecutive ways meet at exactly one node are unambiguous.
    const auto meets_once = [&](std::size_t a, std::size_t b) {
        std::set<std::int64_t> na(ways[a].node_ids.begin(), ways[a].node_ids.end());
        std::size_t common = 0;
        for (const std::int64_t n : std::set<std::int64_t>(ways[b].node_ids.begin(), ways[b].node_ids.end())) common += na.count(n);
        return common == 1;
    };
    std::int64_t relation = first_relation;
    for (std::size_t f = 0; f < ways.size(); ++f) {
        for (const std::int64_t e : {ways[f].node_ids.front(), ways[f].node_ids.back()}) {
            if (ways[f].node_ids.front() == ways[f].node_ids.back() || rng.unit() >= share) continue;
            // F must arrive at e.
            const bool arrives = e == ways[f].node_ids.back() ? dirs(f).forward : dirs(f).backward;
            if (!arrives) continue;
            std::vector<std::int64_t> via;
            std::int64_t node = e;
            std::size_t previous = f;
            const std::size_t via_count = 1 + (rng.next() % 4 == 0 ? 1 : 0);
            bool ok = true;
            for (std::size_t k = 0; k < via_count && ok; ++k) {
                std::vector<std::size_t> options;
                for (const std::size_t w : at_end[node])
                    if (w != previous && w != f && other_end(w, node) != node && runs_from(w, node) && meets_once(previous, w)) options.push_back(w);
                if (options.empty()) { ok = false; break; }
                const std::size_t v = options[rng.next() % options.size()];
                via.push_back(ways[v].osm_id);
                node = other_end(v, node);
                previous = v;
            }
            if (!ok) continue;
            std::vector<std::size_t> targets;
            for (const std::size_t w : at_end[node])
                if (w != previous && runs_from(w, node) && meets_once(previous, w)) targets.push_back(w);
            if (targets.empty()) continue;
            const std::size_t t = targets[rng.next() % targets.size()];
            const bool only = rng.next() % 2 == 0;
            rules.push_back(via_way_rule(relation++, only ? "only_straight_on" : "no_straight_on", ways[f].osm_id, via, ways[t].osm_id));
        }
    }
}


} // namespace g2m_test
