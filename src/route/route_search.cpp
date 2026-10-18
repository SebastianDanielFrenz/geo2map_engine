// src/route/route_search.cpp - see g2m/route/route_search.h.
#include "g2m/route/route_search.h"

#include "g2m/core/det_math.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <limits>
#include <utility>

namespace g2m::route {
namespace {

constexpr std::uint64_t kInf = std::numeric_limits<std::uint64_t>::max();

// One node-graph Dijkstra over arc costs; `adj_first`/`adj` is a CSR of
// (neighbour, cost). Writes column k of `out` ([node * count + k]).
void node_dijkstra(std::uint32_t source, const std::vector<std::uint32_t>& adj_first,
                   const std::vector<std::pair<std::uint32_t, std::uint32_t>>& adj, std::vector<std::uint64_t>& dist,
                   std::vector<std::pair<std::uint64_t, std::uint32_t>>& heap, std::vector<std::uint32_t>& out,
                   std::size_t count, std::size_t k) {
    std::fill(dist.begin(), dist.end(), kInf);
    heap.clear();
    const auto greater = std::greater<std::pair<std::uint64_t, std::uint32_t>>{};
    dist[source] = 0;
    heap.push_back({0, source});
    while (!heap.empty()) {
        std::pop_heap(heap.begin(), heap.end(), greater);
        const auto [d, u] = heap.back();
        heap.pop_back();
        if (d != dist[u]) continue;
        for (std::uint32_t i = adj_first[u]; i < adj_first[u + 1]; ++i) {
            const auto [v, c] = adj[i];
            if (d + c < dist[v]) {
                dist[v] = d + c;
                heap.push_back({dist[v], v});
                std::push_heap(heap.begin(), heap.end(), greater);
            }
        }
    }
    for (std::size_t v = 0; v < dist.size(); ++v)
        out[v * count + k] = dist[v] >= RouteLandmarks::kUnreachable ? RouteLandmarks::kUnreachable
                                                                     : static_cast<std::uint32_t>(dist[v]);
}

} // namespace

Result<RouteLandmarks> RouteLandmarks::build(const RoutingGraph& graph, std::size_t count) {
    if (count == 0 || count > 64) return make_error(ErrorCode::InvalidArgument, "landmarks: count must be 1..64");
    RouteLandmarks out;
    out.graph_ = &graph;
    const std::size_t n = graph.nodes().size();
    out.count_ = std::min(count, n);
    if (n == 0) return out;
    const auto arcs = graph.arcs();
    // Forward and backward CSR adjacency over arc costs.
    std::vector<std::uint32_t> fwd_first(n + 1, 0), bwd_first(n + 1, 0);
    for (const RouteArc& a : arcs) { ++fwd_first[a.from + 1]; ++bwd_first[a.to + 1]; }
    for (std::size_t i = 0; i < n; ++i) { fwd_first[i + 1] += fwd_first[i]; bwd_first[i + 1] += bwd_first[i]; }
    std::vector<std::pair<std::uint32_t, std::uint32_t>> fwd(arcs.size()), bwd(arcs.size());
    {
        std::vector<std::uint32_t> fi(fwd_first.begin(), fwd_first.end() - 1), bi(bwd_first.begin(), bwd_first.end() - 1);
        for (const RouteArc& a : arcs) {
            fwd[fi[a.from]++] = {a.to, a.cost_ms};
            bwd[bi[a.to]++] = {a.from, a.cost_ms};
        }
    }
    out.from_.assign(n * out.count_, kUnreachable);
    out.to_.assign(n * out.count_, kUnreachable);
    std::vector<std::uint64_t> dist(n);
    std::vector<std::pair<std::uint64_t, std::uint32_t>> heap;
    // Farthest-point selection: the first landmark is the node farthest from node 0.
    std::vector<std::uint32_t> probe(n);
    node_dijkstra(0, fwd_first, fwd, dist, heap, probe, 1, 0);
    std::uint32_t next = 0;
    for (std::uint32_t v = 0; v < n; ++v)
        if (probe[v] != kUnreachable && probe[v] > probe[next]) next = v;
    std::vector<std::uint64_t> nearest(n, kInf);
    std::vector<char> chosen(n, 0);
    for (std::size_t k = 0; k < out.count_; ++k) {
        out.nodes_.push_back(next);
        chosen[next] = 1;
        node_dijkstra(next, fwd_first, fwd, dist, heap, out.from_, out.count_, k);
        node_dijkstra(next, bwd_first, bwd, dist, heap, out.to_, out.count_, k);
        // Next: the lowest-index node no landmark reaches in either direction, else the farthest one.
        std::uint32_t unreached = kNoArc, far = kNoArc;
        for (std::uint32_t v = 0; v < n; ++v) {
            const std::uint32_t a = out.from_[v * out.count_ + k], b = out.to_[v * out.count_ + k];
            const std::uint64_t d = std::min<std::uint64_t>(a == kUnreachable ? kInf : a, b == kUnreachable ? kInf : b);
            nearest[v] = std::min(nearest[v], d);
            if (chosen[v]) continue;
            if (nearest[v] == kInf) { if (unreached == kNoArc) unreached = v; continue; }
            if (far == kNoArc || nearest[v] > nearest[far]) far = v;
        }
        next = unreached != kNoArc ? unreached : far;
        if (next == kNoArc) break;
    }
    out.count_ = out.nodes_.size();
    if (out.count_ < std::min(count, n)) {
        // Fewer distinct landmarks than asked (tiny graph): compact the tables.
        // Not reached for count <= node count, kept for safety.
        return make_error(ErrorCode::Malformed, "landmarks: selection ran out of nodes");
    }
    return out;
}

std::uint64_t RouteLandmarks::bound(std::uint32_t v, std::uint32_t t) const {
    std::uint64_t h = 0;
    const std::uint32_t* fv = from_.data() + std::size_t{v} * count_;
    const std::uint32_t* ft = from_.data() + std::size_t{t} * count_;
    const std::uint32_t* tv = to_.data() + std::size_t{v} * count_;
    const std::uint32_t* tt = to_.data() + std::size_t{t} * count_;
    for (std::size_t k = 0; k < count_; ++k) {
        // d(v, t) >= d(L, t) - d(L, v) and d(v, t) >= d(v, L) - d(t, L).
        if (fv[k] != kUnreachable && ft[k] != kUnreachable && ft[k] > fv[k]) h = std::max<std::uint64_t>(h, ft[k] - fv[k]);
        if (tv[k] != kUnreachable && tt[k] != kUnreachable && tv[k] > tt[k]) h = std::max<std::uint64_t>(h, tv[k] - tt[k]);
    }
    return h;
}

RouteSearcher::RouteSearcher(const RoutingGraph& graph, const RouteLandmarks* landmarks)
    : graph_(&graph), landmarks_(landmarks), g_(graph.state_count(), 0), parent_(graph.state_count(), kNoArc),
      stamp_(graph.state_count(), 0) {}

std::uint64_t RouteSearcher::heuristic(std::uint32_t node) const {
    const geom::PointMm p = graph_->nodes()[node].position;
    const double dx = static_cast<double>(target_.x - p.x), dy = static_cast<double>(target_.y - p.y);
    const auto chord_mm = static_cast<std::uint64_t>(std::floor(det::sqrt(dx * dx + dy * dy)));
    std::uint64_t h = (chord_mm > 2 ? chord_mm - 2 : 0) * 36u / (std::uint64_t{graph_->config().max_speed_kmh} * 10u);
    if (landmarks_ && landmarks_->count() > 0) {
        std::uint64_t alt = kInf;
        for (std::uint32_t i = 0; i < alt_target_count_; ++i) alt = std::min(alt, landmarks_->bound(node, alt_targets_[i]));
        if (alt != kInf) h = std::max(h, alt);
    }
    return h;
}

void RouteSearcher::begin() {
    stats_ = {};
    if (++generation_ == 0) {
        std::fill(stamp_.begin(), stamp_.end(), 0u);
        generation_ = 1;
    }
    heap_.clear();
    best_goal_ = kInf;
    best_arc_ = best_parent_ = kNoArc;
    best_end_offset_ = 0;
    best_same_arc_ = false;
    goal_count_ = alt_target_count_ = 0;
    starts_[0] = starts_[1] = {};
    same_start_offset_ = 0;
}

void RouteSearcher::offer_goal(std::uint64_t cost, std::uint32_t arc, std::uint32_t parent, std::uint32_t end_offset,
                               bool same_arc) {
    if (cost >= best_goal_) return;
    best_goal_ = cost;
    best_arc_ = arc;
    best_parent_ = parent;
    best_end_offset_ = end_offset;
    best_same_arc_ = same_arc;
}

// `arc` is a search state: a real arc or a via-way ghost of one (RoutingGraph::state_count()).
void RouteSearcher::relax(std::uint32_t arc, std::uint64_t pre, std::uint32_t parent) {
    const RouteArc& a = graph_->state_arc(arc);
    // Goal first: reaching the goal on this arc does not depend on the arc's own g.
    if (node_mode_) {
        if (a.to == target_node_) offer_goal(pre + a.cost_ms, arc, parent, a.length_mm, false);
    } else {
        const std::uint32_t real = graph_->state_origin(arc);
        for (std::uint32_t i = 0; i < goal_count_; ++i)
            if (goals_[i].arc == real) offer_goal(pre + graph_->partial_cost_ms(real, 0, goals_[i].offset_mm), arc, parent, goals_[i].offset_mm, false);
    }
    const std::uint64_t g = pre + a.cost_ms;
    if (stamp_[arc] == generation_) {
        if (g >= g_[arc]) return;
        ++stats_.reopened;
    }
    stamp_[arc] = generation_;
    g_[arc] = g;
    parent_[arc] = parent;
    heap_.push_back({g + heuristic(a.to), g, arc});
    std::push_heap(heap_.begin(), heap_.end(), std::greater<Open>{});
    ++stats_.pushed;
}

Result<Route> RouteSearcher::run() {
    const auto arcs = graph_->arcs();
    const std::size_t limit = graph_->config().max_settled;
    while (!heap_.empty()) {
        if (heap_.front().f >= best_goal_) break;
        std::pop_heap(heap_.begin(), heap_.end(), std::greater<Open>{});
        const Open top = heap_.back();
        heap_.pop_back();
        if (top.g != g_[top.arc]) continue; // stale entry
        if (++stats_.settled > limit) return make_error(ErrorCode::LimitExceeded, "route: max_settled exceeded");
        graph_->state_successors(top.arc, succ_);
        for (const Transition& tr : succ_) relax(tr.arc, top.g + tr.turn_ms, top.arc);
    }
    Route route;
    if (best_arc_ == kNoArc) return route;
    route.reachable = true;
    route.cost_ms = best_goal_;
    // States map back to the real arcs they stand for.
    route.arcs.push_back(best_same_arc_ ? best_arc_ : graph_->state_origin(best_arc_));
    if (!best_same_arc_)
        for (std::uint32_t a = best_parent_; a != kNoArc; a = parent_[a]) route.arcs.push_back(graph_->state_origin(a));
    std::reverse(route.arcs.begin(), route.arcs.end());
    if (best_same_arc_) {
        route.start_offset_mm = same_start_offset_;
    } else if (!node_mode_) {
        // The start candidate the chain began on.
        for (std::uint32_t i = 0; i < 2; ++i)
            if (starts_[i].arc == route.arcs.front()) route.start_offset_mm = starts_[i].offset_mm;
    }
    route.end_offset_mm = best_end_offset_;
    for (const std::uint32_t a : route.arcs) route.length_mm += arcs[a].length_mm;
    route.length_mm -= route.start_offset_mm;
    route.length_mm -= arcs[route.arcs.back()].length_mm - route.end_offset_mm;
    return route;
}

Result<Route> RouteSearcher::find(const RouteQuery& query) {
    if (landmarks_ && landmarks_->graph() != graph_) return make_error(ErrorCode::InvalidArgument, "route: landmarks of another graph");
    const std::uint32_t s = graph_->find_node(query.from_node), t = graph_->find_node(query.to_node);
    if (s == kNoArc || t == kNoArc) return make_error(ErrorCode::InvalidArgument, "route: node id not in the routing graph");
    begin();
    if (s == t) {
        Route route;
        route.reachable = true;
        return route;
    }
    node_mode_ = true;
    target_node_ = t;
    target_ = graph_->nodes()[t].position;
    alt_targets_[0] = t;
    alt_target_count_ = 1;
    const auto arcs = graph_->arcs();
    for (const RouteArc& a : graph_->out_arcs(s)) relax(static_cast<std::uint32_t>(&a - arcs.data()), 0, kNoArc);
    return run();
}

Result<Route> RouteSearcher::find(const RoutePoint& from, const RoutePoint& to) {
    if (landmarks_ && landmarks_->graph() != graph_) return make_error(ErrorCode::InvalidArgument, "route: landmarks of another graph");
    const auto arcs = graph_->arcs();
    const auto valid = [&](const RoutePoint& p) {
        if (p.count == 0 || p.count > 2) return false;
        for (std::uint32_t i = 0; i < p.count; ++i)
            if (p.candidates[i].arc >= arcs.size() || p.candidates[i].offset_mm > arcs[p.candidates[i].arc].length_mm) return false;
        return true;
    };
    if (!valid(from) || !valid(to)) return make_error(ErrorCode::InvalidArgument, "route: route point without valid candidates");
    begin();
    node_mode_ = false;
    target_node_ = kNoArc;
    target_ = to.position;
    for (std::uint32_t i = 0; i < to.count; ++i) {
        goals_[goal_count_++] = {to.candidates[i].arc, to.candidates[i].offset_mm};
        alt_targets_[alt_target_count_++] = arcs[to.candidates[i].arc].from;
    }
    for (std::uint32_t i = 0; i < from.count; ++i) starts_[i] = {from.candidates[i].arc, from.candidates[i].offset_mm};
    // Destination ahead on the start arc itself.
    for (std::uint32_t i = 0; i < from.count; ++i)
        for (std::uint32_t j = 0; j < goal_count_; ++j)
            if (starts_[i].arc == goals_[j].arc && goals_[j].offset_mm >= starts_[i].offset_mm) {
                const auto cost = graph_->partial_cost_ms(goals_[j].arc, starts_[i].offset_mm, goals_[j].offset_mm);
                if (cost < best_goal_) same_start_offset_ = starts_[i].offset_mm;
                offer_goal(cost, goals_[j].arc, kNoArc, goals_[j].offset_mm, true);
            }
    // Seeds: the rest of each start arc.
    for (std::uint32_t i = 0; i < from.count; ++i) {
        const std::uint32_t a = starts_[i].arc;
        const std::uint64_t g = graph_->partial_cost_ms(a, starts_[i].offset_mm, arcs[a].length_mm);
        if (stamp_[a] == generation_ && g >= g_[a]) continue;
        stamp_[a] = generation_;
        g_[a] = g;
        parent_[a] = kNoArc;
        heap_.push_back({g + heuristic(arcs[a].to), g, a});
        std::push_heap(heap_.begin(), heap_.end(), std::greater<Open>{});
        ++stats_.pushed;
    }
    return run();
}

} // namespace g2m::route
