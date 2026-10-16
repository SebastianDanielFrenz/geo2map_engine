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

std::uint16_t lane_token(std::string_view t) {
    using namespace lane_dir;
    if (t == "left") return kLeft;
    if (t == "slight_left") return kSlightLeft;
    if (t == "through") return kThrough;
    if (t == "slight_right") return kSlightRight;
    if (t == "right") return kRight;
    if (t == "sharp_left") return kSharpLeft;
    if (t == "sharp_right") return kSharpRight;
    if (t == "merge_to_left") return kMergeToLeft;
    if (t == "merge_to_right") return kMergeToRight;
    if (t == "reverse") return kReverse;
    return 0;
}

// A tier of acceptable directions; the first tier that any lane satisfies decides. For kThrough,
// edge_through restricts it to the outer lane on that side (-1 left, +1 right, 0 either outer lane
// for an untagged lane / any lane for a tagged one).
struct Tier {
    std::uint16_t mask = 0;
    int edge_through = 0;
};
struct Wanted {
    Tier tier[3];
    int count = 0;
    void add(std::uint16_t mask, int edge = 0) { tier[count++] = {mask, edge}; }
};

// cls: 0 straight, 1 slight, 2 turn, 3 sharp; left: side of the turn. fork: keep left / keep right.
Wanted wanted_for(int cls, bool left, bool fork) {
    using namespace lane_dir;
    const std::uint16_t sl = left ? kSlightLeft : kSlightRight, l = left ? kLeft : kRight, sh = left ? kSharpLeft : kSharpRight;
    const int edge = left ? -1 : 1;
    Wanted w;
    if (fork) {
        w.add(static_cast<std::uint16_t>(sl | l));
        w.add(kThrough);
    } else if (cls == 0) {
        w.add(kThrough);
    } else if (cls == 1) {
        w.add(sl);
        w.add(static_cast<std::uint16_t>(l | kThrough), edge);
    } else if (cls == 2) {
        w.add(l);
        w.add(static_cast<std::uint16_t>(sl | sh));
    } else {
        w.add(sh);
        w.add(static_cast<std::uint16_t>(l | sl));
    }
    return w;
}

bool lane_matches(std::size_t idx, std::size_t count, std::uint16_t dir, const Tier& t) {
    const bool outer_left = idx == 0, outer_right = idx + 1 == count;
    const bool edge_ok = t.edge_through == 0 ? true : (t.edge_through < 0 ? outer_left : outer_right);
    if (dir == 0) return (t.mask & lane_dir::kThrough) != 0 && (outer_left || outer_right) && edge_ok;
    if ((dir & t.mask & ~lane_dir::kThrough) != 0) return true;
    return (dir & t.mask & lane_dir::kThrough) != 0 && edge_ok;
}

// turn:lanes of the way in the travelled direction, or empty.
std::string_view approach_turn_lanes(const RoadGraphWay& way, bool forward, std::string_view& lane_count_tag) {
    const auto oneway = tag_value(way, "oneway");
    const auto highway = tag_value(way, "highway");
    int dir = 0; // +1 one-way along the ref order, -1 against it
    if (oneway == "yes" || oneway == "1" || oneway == "true") dir = 1;
    else if (oneway == "-1" || oneway == "reverse") dir = -1;
    else if (oneway.empty() && (highway == "motorway" || highway == "motorway_link")) dir = 1;
    const auto split = tag_value(way, forward ? "turn:lanes:forward" : "turn:lanes:backward");
    if (!split.empty()) {
        lane_count_tag = tag_value(way, forward ? "lanes:forward" : "lanes:backward");
        return split;
    }
    if (dir != 0 && (dir > 0) == forward) {
        lane_count_tag = tag_value(way, "lanes");
        return tag_value(way, "turn:lanes");
    }
    return {};
}

bool parse_count(std::string_view s, std::size_t& out) {
    if (s.empty() || s.size() > 3) return false;
    std::size_t v = 0;
    for (const char c : s) {
        if (c < '0' || c > '9') return false;
        v = v * 10 + static_cast<std::size_t>(c - '0');
    }
    out = v;
    return true;
}

void build_lane_hints(const RoadGraphWay& way, bool forward, ManeuverKind kind, double angle, const RouteConfig& rc,
                      std::vector<LaneHint>& out) {
    out.clear();
    std::string_view count_tag;
    const auto text = approach_turn_lanes(way, forward, count_tag);
    if (text.empty()) return;
    std::vector<std::uint16_t> dirs;
    if (!parse_turn_lanes(text, dirs)) return;
    std::size_t tagged = 0;
    if (parse_count(count_tag, tagged) && tagged != dirs.size()) return; // inconsistent tags: say nothing
    const double mag = std::fabs(angle);
    const bool left = angle > 0.0;
    int cls = 0;
    bool fork = false, ramp_off = false;
    switch (kind) {
    case ManeuverKind::Continue: cls = 0; break;
    case ManeuverKind::SlightLeft: case ManeuverKind::SlightRight: cls = 1; break;
    case ManeuverKind::Left: case ManeuverKind::Right: cls = 2; break;
    case ManeuverKind::SharpLeft: case ManeuverKind::SharpRight: cls = 3; break;
    case ManeuverKind::KeepLeft: case ManeuverKind::KeepRight: fork = true; break;
    case ManeuverKind::RampOff: case ManeuverKind::RampOn: case ManeuverKind::Merge:
        ramp_off = kind == ManeuverKind::RampOff;
        cls = mag <= rc.straight_max_deg ? 0 : mag <= rc.slight_max_deg ? 1 : mag <= rc.turn_max_deg ? 2 : 3;
        break;
    case ManeuverKind::UTurn: {
        Wanted w;
        w.add(lane_dir::kReverse);
        w.add(lane_dir::kLeft);
        for (int t = 0; t < w.count; ++t) {
            bool any = false;
            for (std::size_t i = 0; i < dirs.size(); ++i) any = any || (dirs[i] & w.tier[t].mask) != 0;
            if (!any) continue;
            out.resize(dirs.size());
            for (std::size_t i = 0; i < dirs.size(); ++i) out[i] = {dirs[i], (dirs[i] & w.tier[t].mask) != 0};
            return;
        }
        return;
    }
    default: return; // Depart, Arrive, roundabouts: no hint
    }
    const bool is_left = fork ? kind == ManeuverKind::KeepLeft : left;
    Wanted w;
    if (ramp_off) {
        // The exit lane is tagged by side (slight_right / right ...) even when the ramp leaves nearly straight.
        const bool exit_left = mag > 0.0 && left;
        using namespace lane_dir;
        w.add(exit_left ? static_cast<std::uint16_t>(kSlightLeft | kLeft | kSharpLeft)
                        : static_cast<std::uint16_t>(kSlightRight | kRight | kSharpRight));
        const Wanted by_angle = wanted_for(cls, exit_left, false);
        for (int t = 0; t < by_angle.count; ++t) w.tier[w.count++] = by_angle.tier[t];
    } else {
        w = wanted_for(cls, is_left, fork);
    }
    for (int t = 0; t < w.count; ++t) {
        bool any = false;
        for (std::size_t i = 0; i < dirs.size() && !any; ++i) any = lane_matches(i, dirs.size(), dirs[i], w.tier[t]);
        if (!any) continue;
        out.resize(dirs.size());
        for (std::size_t i = 0; i < dirs.size(); ++i) out[i] = {dirs[i], lane_matches(i, dirs.size(), dirs[i], w.tier[t])};
        return;
    }
}

} // namespace

std::string lane_directions_to_string(std::uint16_t directions) {
    using namespace lane_dir;
    static constexpr std::pair<std::uint16_t, const char*> names[] = {
        {kLeft, "left"}, {kSlightLeft, "slight_left"}, {kSharpLeft, "sharp_left"}, {kThrough, "through"},
        {kSlightRight, "slight_right"}, {kRight, "right"}, {kSharpRight, "sharp_right"},
        {kMergeToLeft, "merge_to_left"}, {kMergeToRight, "merge_to_right"}, {kReverse, "reverse"}};
    std::string s;
    for (const auto& [bit, name] : names) {
        if (!(directions & bit)) continue;
        if (!s.empty()) s += ';';
        s += name;
    }
    return s.empty() ? "none" : s;
}

bool parse_turn_lanes(std::string_view value, std::vector<std::uint16_t>& lanes) {
    lanes.clear();
    if (value.empty()) return false;
    std::size_t pos = 0;
    while (true) {
        const std::size_t bar = value.find('|', pos);
        const std::string_view lane = value.substr(pos, bar == std::string_view::npos ? std::string_view::npos : bar - pos);
        std::uint16_t mask = 0;
        std::size_t p = 0;
        while (p <= lane.size()) {
            const std::size_t semi = lane.find(';', p);
            const std::string_view tok = lane.substr(p, semi == std::string_view::npos ? std::string_view::npos : semi - p);
            if (!tok.empty() && tok != "none") {
                const std::uint16_t bit = lane_token(tok);
                if (bit == 0) { lanes.clear(); return false; }
                mask = static_cast<std::uint16_t>(mask | bit);
            }
            if (semi == std::string_view::npos) break;
            p = semi + 1;
        }
        lanes.push_back(mask);
        if (bar == std::string_view::npos) break;
        pos = bar + 1;
    }
    return true;
}

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
        if (step > 0 && kind != ManeuverKind::Depart && kind != ManeuverKind::Arrive) {
            const RouteArc& approach = arcs[route.arcs[step - 1]];
            if (const RoadGraphWay* aw = find_way(roads, approach.way_id))
                build_lane_hints(*aw, approach.forward, kind, angle, rc, m.lanes);
        }
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
