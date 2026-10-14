// src/route/route_guidance.cpp - see g2m/route/route_guidance.h.
#include "g2m/route/route_guidance.h"

#include "g2m/core/det_math.h"

#include <algorithm>
#include <cmath>
#include <string_view>

namespace g2m::route {
namespace {

struct WayInfo {
    std::string_view name, ref, highway, destination, destination_ref;
    bool roundabout = false;
};

std::string_view tag_value(const RoadGraphWay& way, std::string_view key) {
    for (const auto& [k, v] : way.tags) if (k == key) return v;
    return {};
}

double heading_change_deg(const RouteArc& in, const RouteArc& out) {
    const double ax = static_cast<double>(in.out_dx), ay = static_cast<double>(in.out_dy);
    const double bx = static_cast<double>(out.in_dx), by = static_cast<double>(out.in_dy);
    const double cross = ax * by - ay * bx, dot = ax * bx + ay * by;
    if (cross == 0.0 && dot >= 0.0) return 0.0;
    return det::atan2(cross, dot) * (180.0 / det::kPi);
}

const RoadGraphWay* find_way(const RoadGraph& roads, std::int64_t id) {
    const auto it = std::lower_bound(roads.ways.begin(), roads.ways.end(), id,
                                     [](const RoadGraphWay& w, std::int64_t x) { return w.osm_id < x; });
    return it != roads.ways.end() && it->osm_id == id ? &*it : nullptr;
}

bool is_ring(const RoadGraphWay* way) {
    if (!way) return false;
    const auto junction = tag_value(*way, "junction");
    return junction == "roundabout" || junction == "circular";
}

bool is_link(std::string_view highway) { return highway.ends_with("_link"); }
bool is_major(std::string_view highway) { return highway == "motorway" || highway == "trunk"; }

} // namespace

const char* to_string(ManeuverKind kind) {
    switch (kind) {
    case ManeuverKind::Depart: return "depart";
    case ManeuverKind::Continue: return "continue";
    case ManeuverKind::SlightLeft: return "slight_left";
    case ManeuverKind::SlightRight: return "slight_right";
    case ManeuverKind::Left: return "left";
    case ManeuverKind::Right: return "right";
    case ManeuverKind::SharpLeft: return "sharp_left";
    case ManeuverKind::SharpRight: return "sharp_right";
    case ManeuverKind::KeepLeft: return "keep_left";
    case ManeuverKind::KeepRight: return "keep_right";
    case ManeuverKind::UTurn: return "u_turn";
    case ManeuverKind::RoundaboutEnter: return "roundabout_enter";
    case ManeuverKind::RoundaboutExit: return "roundabout_exit";
    case ManeuverKind::RampOff: return "ramp_off";
    case ManeuverKind::RampOn: return "ramp_on";
    case ManeuverKind::Merge: return "merge";
    case ManeuverKind::Arrive: return "arrive";
    }
    return "?";
}

Result<std::vector<Maneuver>> build_guidance(const RoutingGraph& graph, const RoadGraph& roads, const Route& route,
                                             const GuidanceConfig& config) {
    if (!route.reachable || route.arcs.empty()) return make_error(ErrorCode::InvalidArgument, "guidance: empty or unreachable route");
    const auto arcs = graph.arcs();
    const RouteConfig& rc = graph.config();
    const std::size_t n = route.arcs.size();
    std::vector<WayInfo> info(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (route.arcs[i] >= arcs.size()) return make_error(ErrorCode::InvalidArgument, "guidance: arc index out of range");
        const RoadGraphWay* way = find_way(roads, arcs[route.arcs[i]].way_id);
        if (!way) return make_error(ErrorCode::InvalidArgument, "guidance: way not in roads");
        info[i] = {tag_value(*way, "name"),        tag_value(*way, "ref"), tag_value(*way, "highway"),
                   tag_value(*way, "destination"), tag_value(*way, "destination:ref"), is_ring(way)};
    }
    // Distance travelled at the start of each arc.
    std::vector<std::uint64_t> at(n + 1, 0);
    for (std::size_t i = 0; i < n; ++i)
        at[i + 1] = at[i] + (i == 0 ? arcs[route.arcs[0]].length_mm - route.start_offset_mm : arcs[route.arcs[i]].length_mm);

    std::vector<Maneuver> out;
    const auto emit = [&](ManeuverKind kind, std::size_t step, std::uint32_t node, geom::PointMm pos, std::uint64_t dist,
                          double angle, std::uint8_t exit_number) {
        Maneuver m;
        m.kind = kind;
        m.step = static_cast<std::uint32_t>(step);
        m.node = node;
        m.position = pos;
        m.distance_from_start_mm = dist;
        m.angle_deg = static_cast<std::int32_t>(std::llround(angle));
        m.exit_number = exit_number;
        m.name = std::string(info[step].name);
        m.ref = std::string(info[step].ref);
        m.destination = std::string(info[step].destination);
        m.destination_ref = std::string(info[step].destination_ref);
        out.push_back(std::move(m));
    };
    // Exits passed on a roundabout from step `first` on: nodes where an allowed exit leaves the ring.
    std::vector<Transition> succ;
    const auto count_exits = [&](std::size_t first, std::size_t& leave_step) {
        std::uint8_t passed = 0;
        std::size_t j = first;
        while (j + 1 < n && info[j + 1].roundabout) {
            graph.successors(route.arcs[j], succ);
            for (const auto& t : succ) {
                if (!is_ring(find_way(roads, arcs[t.arc].way_id))) { ++passed; break; }
            }
            ++j;
        }
        leave_step = j + 1; // n when the route ends on the ring
        return passed;
    };

    emit(ManeuverKind::Depart, 0, kNoArc, graph.point_at(route.arcs[0], route.start_offset_mm), 0, 0.0, 0);
    bool on_entered_ring = false;
    if (info[0].roundabout) {
        std::size_t leave = 0;
        const std::uint8_t passed = count_exits(0, leave);
        if (leave < n) {
            const std::uint32_t v = arcs[route.arcs[leave - 1]].to;
            emit(ManeuverKind::RoundaboutExit, leave, v, graph.nodes()[v].position, at[leave],
                 heading_change_deg(arcs[route.arcs[leave - 1]], arcs[route.arcs[leave]]), static_cast<std::uint8_t>(passed + 1));
        }
        on_entered_ring = true;
    }
    for (std::size_t i = 1; i < n; ++i) {
        const RouteArc& a = arcs[route.arcs[i - 1]];
        const RouteArc& b = arcs[route.arcs[i]];
        const std::uint32_t v = a.to;
        const geom::PointMm pos = graph.nodes()[v].position;
        const double angle = heading_change_deg(a, b);
        if (route.arcs[i] == a.reverse) {
            emit(ManeuverKind::UTurn, i, v, pos, at[i], angle, 0);
            continue;
        }
        if (info[i].roundabout) {
            if (!info[i - 1].roundabout) {
                std::size_t leave = 0;
                const std::uint8_t passed = count_exits(i, leave);
                emit(ManeuverKind::RoundaboutEnter, i, v, pos, at[i], angle, leave < n ? static_cast<std::uint8_t>(passed + 1) : 0);
                on_entered_ring = true;
            }
            continue;
        }
        if (info[i - 1].roundabout && on_entered_ring) continue; // the exit was announced on entry
        if (graph.nodes()[v].degree < 3) continue;
        const bool renamed = info[i].name != info[i - 1].name || info[i].ref != info[i - 1].ref;
        if (!is_link(info[i - 1].highway) && is_link(info[i].highway)) {
            emit(is_major(info[i - 1].highway) ? ManeuverKind::RampOff : ManeuverKind::RampOn, i, v, pos, at[i], angle, 0);
            continue;
        }
        if (is_link(info[i - 1].highway) && is_major(info[i].highway)) {
            emit(ManeuverKind::Merge, i, v, pos, at[i], angle, 0);
            continue;
        }
        graph.successors(route.arcs[i - 1], succ);
        const double mag = std::fabs(angle);
        const bool left = angle > 0.0;
        // The nearest other allowed exit (by heading).
        bool has_other = false;
        double nearest_gap = 0.0, nearest_angle = 0.0;
        for (const auto& t : succ) {
            if (t.arc == route.arcs[i]) continue;
            // Staying on a main road past a slip road needs no fork announcement.
            if (!renamed && !is_link(info[i].highway)) {
                const RoadGraphWay* other_way = find_way(roads, arcs[t.arc].way_id);
                if (other_way && is_link(tag_value(*other_way, "highway"))) continue;
            }
            const double other = heading_change_deg(a, arcs[t.arc]);
            const double gap = std::fabs(other - angle);
            if (!has_other || gap < nearest_gap) { nearest_gap = gap; nearest_angle = other; }
            has_other = true;
        }
        const auto turn_kind = [&]() {
            if (mag <= rc.slight_max_deg) return left ? ManeuverKind::SlightLeft : ManeuverKind::SlightRight;
            if (mag <= rc.turn_max_deg) return left ? ManeuverKind::Left : ManeuverKind::Right;
            return left ? ManeuverKind::SharpLeft : ManeuverKind::SharpRight;
        };
        if (mag <= rc.slight_max_deg) {
            if (has_other && nearest_gap <= config.fork_max_deg) {
                emit(angle >= nearest_angle ? ManeuverKind::KeepLeft : ManeuverKind::KeepRight, i, v, pos, at[i], angle, 0);
            } else if (has_other && mag > rc.straight_max_deg) {
                emit(turn_kind(), i, v, pos, at[i], angle, 0);
            } else if (renamed) {
                emit(ManeuverKind::Continue, i, v, pos, at[i], angle, 0);
            }
            continue;
        }
        emit(turn_kind(), i, v, pos, at[i], angle, 0);
    }
    const std::size_t last = n - 1;
    emit(ManeuverKind::Arrive, last, kNoArc, graph.point_at(route.arcs[last], route.end_offset_mm), route.length_mm, 0.0, 0);
    for (std::size_t k = 0; k + 1 < out.size(); ++k)
        out[k].distance_to_next_mm = out[k + 1].distance_from_start_mm - out[k].distance_from_start_mm;
    return out;
}

} // namespace g2m::route
