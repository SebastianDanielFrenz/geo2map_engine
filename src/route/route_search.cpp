// src/route/route_search.cpp - see g2m/route/route_search.h.
#include "g2m/route/route_search.h"

#include "g2m/core/det_math.h"

#include <algorithm>
#include <cmath>
#include <functional>

namespace g2m::route {

RouteSearcher::RouteSearcher(const RoutingGraph& graph)
    : graph_(&graph), g_(graph.arcs().size(), 0), parent_(graph.arcs().size(), kNoArc), stamp_(graph.arcs().size(), 0) {}

std::uint64_t RouteSearcher::heuristic(std::uint32_t node) const {
    const geom::PointMm p = graph_->nodes()[node].position;
    const double dx = static_cast<double>(target_.x - p.x), dy = static_cast<double>(target_.y - p.y);
    const auto chord_mm = static_cast<std::uint64_t>(std::floor(det::sqrt(dx * dx + dy * dy)));
    return chord_mm * 36u / (std::uint64_t{graph_->config().max_speed_kmh} * 10u);
}

Result<Route> RouteSearcher::find(const RouteQuery& query) {
    stats_ = {};
    const std::uint32_t s = graph_->find_node(query.from_node), t = graph_->find_node(query.to_node);
    if (s == kNoArc || t == kNoArc) return make_error(ErrorCode::InvalidArgument, "route: node id not in the routing graph");
    Route route;
    if (s == t) {
        route.reachable = true;
        return route;
    }
    if (++generation_ == 0) {
        std::fill(stamp_.begin(), stamp_.end(), 0u);
        generation_ = 1;
    }
    target_ = graph_->nodes()[t].position;
    heap_.clear();
    const auto arcs = graph_->arcs();
    const auto greater = std::greater<Open>{};
    const auto relax = [&](std::uint32_t arc, std::uint64_t g, std::uint32_t parent) {
        if (stamp_[arc] == generation_) {
            if (g >= g_[arc]) return;
            ++stats_.reopened;
        }
        stamp_[arc] = generation_;
        g_[arc] = g;
        parent_[arc] = parent;
        heap_.push_back({g + heuristic(arcs[arc].to), g, arc});
        std::push_heap(heap_.begin(), heap_.end(), greater);
        ++stats_.pushed;
    };
    for (const RouteArc& a : graph_->out_arcs(s)) {
        const auto index = static_cast<std::uint32_t>(&a - arcs.data());
        relax(index, a.cost_ms, kNoArc);
    }
    const std::size_t limit = graph_->config().max_settled;
    while (!heap_.empty()) {
        std::pop_heap(heap_.begin(), heap_.end(), greater);
        const Open top = heap_.back();
        heap_.pop_back();
        if (top.g != g_[top.arc]) continue; // stale entry
        if (arcs[top.arc].to == t) {
            route.reachable = true;
            route.cost_ms = top.g;
            for (std::uint32_t a = top.arc; a != kNoArc; a = parent_[a]) {
                route.arcs.push_back(a);
                route.length_mm += arcs[a].length_mm;
            }
            std::reverse(route.arcs.begin(), route.arcs.end());
            return route;
        }
        if (++stats_.settled > limit) return make_error(ErrorCode::LimitExceeded, "route: max_settled exceeded");
        graph_->successors(top.arc, succ_);
        for (const Transition& tr : succ_) relax(tr.arc, top.g + tr.turn_ms + arcs[tr.arc].cost_ms, top.arc);
    }
    return route; // unreachable
}

} // namespace g2m::route
