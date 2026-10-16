// src/route/routing_graph.cpp - see g2m/route/routing_graph.h.
#include "g2m/route/routing_graph.h"

#include "g2m/core/det_math.h"
#include "g2m/layer/osm_roads.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <map>
#include <optional>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace g2m::route {
namespace {

std::optional<std::string_view> tag_of(const RoadGraphWay& way, std::string_view key) {
    const auto it = std::lower_bound(way.tags.begin(), way.tags.end(), key,
                                     [](const auto& tag, std::string_view k) { return tag.first < k; });
    if (it != way.tags.end() && it->first == key) return std::string_view(it->second);
    // Tags are canonical (sorted) in a RoadGraph; fall back to a scan for hand-built input.
    for (const auto& [k, v] : way.tags) if (k == key) return std::string_view(v);
    return std::nullopt;
}

bool contains(const std::vector<std::string>& list, std::string_view value) {
    return std::find(list.begin(), list.end(), value) != list.end();
}

// Speed for one direction ("forward"/"backward"): the direction tag, else the
// plain maxspeed, else the class default; symbolic values via the zone table.
std::uint16_t way_speed(const RoadGraphWay& way, std::string_view direction_key, std::uint16_t class_default,
                        const RouteConfig& config) {
    auto text = tag_of(way, direction_key);
    if (!text) text = tag_of(way, "maxspeed");
    std::int64_t kmh = class_default;
    if (text) {
        const RoadSpeedLimit limit = parse_road_speed_limit(*text);
        if (limit.kind == SpeedLimitKind::Numeric) {
            kmh = std::llround(limit.kph);
        } else if (limit.kind == SpeedLimitKind::Unknown) {
            for (const auto& [zone, speed] : config.zone_speeds) if (zone == *text) kmh = speed;
        }
    }
    kmh = std::clamp<std::int64_t>(kmh, config.min_speed_kmh, config.max_speed_kmh);
    return static_cast<std::uint16_t>(kmh);
}

// Most specific of motorcar > motor_vehicle > vehicle > access decides.
bool access_blocked(const RoadGraphWay& way, const RouteConfig& config) {
    for (const std::string_view key : {"motorcar", "motor_vehicle", "vehicle", "access"}) {
        if (const auto value = tag_of(way, key)) return contains(config.blocked_access, *value);
    }
    return false;
}

std::uint32_t ceil_length_mm(const geom::PointMm& a, const geom::PointMm& b) {
    const double dx = static_cast<double>(b.x - a.x), dy = static_cast<double>(b.y - a.y);
    return static_cast<std::uint32_t>(std::ceil(det::sqrt(dx * dx + dy * dy)));
}

struct Edge {
    std::uint32_t from, to;
    std::size_t way; // index into graph.ways
    std::uint32_t seg_first, seg_last;
    std::uint64_t length_mm;
    std::int64_t first_dx, first_dy, last_dx, last_dy;
    std::uint32_t shape_first, shape_count;
};

std::int64_t floor_div(std::int64_t a, std::int64_t b) {
    const std::int64_t q = a / b;
    return (a % b != 0 && ((a < 0) != (b < 0))) ? q - 1 : q;
}
std::uint64_t cell_key(std::int64_t cx, std::int64_t cy) {
    return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(cx)) << 32) | static_cast<std::uint32_t>(cy);
}


// ---- Via-way rules (G4 slice 3) -------------------------------------------------------------
//
// A rule F -> V1 .. Vk -> T (k >= 1) resolves to node chains n0 .. nk, n_j = a decision node
// both ways[j] and ways[j+1] pass through (F = ways[0], T = ways[k+1]). Arriving on F at n0,
// leaving onto V1, running along V1 (through its inner decision nodes) to n1, onto V2, ..., onto
// T at nk is "the sequence". No: the last step (leaving V_k onto T at n_k) is forbidden after
// the whole sequence. Only: after arriving on F at n0 only V1 may follow, and so on up to T
// (a vehicle on V_j may only continue along V_j to n_j, then must take ways[j+1]).
//
// The search state is an arc; the part of the sequence already driven is history. States with
// history are "ghost" copies of the arc, one per (arc, set of rule positions): a position
// (rule, j) means the path so far ends with the driven part of the sequence up to ways[j]
// (j = 0: just arrived on F at n0, which is implied by the arc itself and needs no ghost).
// Ghosts are found by a breadth-first walk from the arcs that arrive on a rule's F at n0.
constexpr std::size_t kMaxViaWays = 16;       // via ways of one rule
constexpr std::size_t kMaxInstancesPerRule = 8; // node chains of one rule
constexpr std::size_t kMaxGhostStates = 1u << 20;

struct ViaRule {
    RestrictionKind kind = RestrictionKind::No;
    std::vector<std::int64_t> ways;  // F, V1..Vk, T
    std::vector<std::uint32_t> nodes; // n0..nk (routing node indices)
};

} // namespace

struct ViaBuilder {
    using Position = std::uint32_t; // rule << 8 | j
    static Position pack(std::size_t rule, std::size_t j) { return static_cast<Position>(rule << 8 | j); }

    // Resolves one via-way restriction to its node chains and appends them to `out`.
    static void instances(const RoutingGraph& g, const RoadGraph& graph, const std::vector<std::vector<std::uint32_t>>& refs,
                          const std::vector<std::uint32_t>& compact, const std::unordered_map<std::int64_t, char>& used_way,
                          const TurnRestriction& tr, std::vector<ViaRule>& out) {
        std::vector<std::int64_t> ways;
        ways.push_back(tr.from_way);
        ways.insert(ways.end(), tr.via_ways.begin(), tr.via_ways.end());
        ways.push_back(tr.to_way);
        // Routing nodes each way passes through.
        std::vector<std::vector<std::uint32_t>> pass(ways.size());
        for (std::size_t i = 0; i < ways.size(); ++i) {
            if (!used_way.count(ways[i])) return;
            const auto it = std::lower_bound(graph.ways.begin(), graph.ways.end(), ways[i],
                                             [](const RoadGraphWay& w, std::int64_t id) { return w.osm_id < id; });
            const auto w = static_cast<std::size_t>(it - graph.ways.begin());
            for (const std::uint32_t n : refs[w]) if (compact[n] != kNoArc) pass[i].push_back(compact[n]);
            std::sort(pass[i].begin(), pass[i].end());
            pass[i].erase(std::unique(pass[i].begin(), pass[i].end()), pass[i].end());
        }
        std::vector<std::vector<std::uint32_t>> shared(ways.size() - 1);
        for (std::size_t j = 0; j + 1 < ways.size(); ++j)
            std::set_intersection(pass[j].begin(), pass[j].end(), pass[j + 1].begin(), pass[j + 1].end(), std::back_inserter(shared[j]));
        const auto arrives = [&](std::int64_t way, std::uint32_t node) {
            for (std::size_t a = 0; a < g.real_arcs_; ++a) if (g.arcs_[a].way_id == way && g.arcs_[a].to == node) return true;
            return false;
        };
        const auto leaves = [&](std::int64_t way, std::uint32_t node) {
            for (std::uint32_t a = g.nodes_[node].first_out; a < g.nodes_[node + 1].first_out; ++a) if (g.arcs_[a].way_id == way) return true;
            return false;
        };
        std::vector<std::uint32_t> chain;
        std::size_t found = 0;
        const auto walk = [&](auto&& self, std::size_t j) -> void {
            if (found >= kMaxInstancesPerRule) return;
            if (j == shared.size()) {
                if (!arrives(ways.front(), chain.front()) || !leaves(ways.back(), chain.back())) return;
                // Every via way must start at the node before it.
                for (std::size_t v = 1; v + 1 < ways.size(); ++v) if (!leaves(ways[v], chain[v - 1])) return;
                out.push_back({tr.kind, ways, chain});
                ++found;
                return;
            }
            for (const std::uint32_t n : shared[j]) {
                if (j > 0 && n == chain.back()) continue; // a via way runs from one node to another
                chain.push_back(n);
                self(self, j + 1);
                chain.pop_back();
            }
        };
        walk(walk, 0);
    }

    // The next positions for a path in positions `set` on arc `a` leaving onto `b`; false = forbidden.
    static bool step(const std::vector<ViaRule>& rules, const std::vector<Position>& set, const RouteArc& a, const RouteArc& b,
                     std::vector<Position>& next) {
        next.clear();
        bool only_constrained = false, only_ok = false;
        for (const Position p : set) {
            const std::size_t i = p >> 8, j = p & 255;
            const ViaRule& r = rules[i];
            const bool only = r.kind == RestrictionKind::Only;
            only_constrained = only_constrained || only;
            if (a.to == r.nodes[j]) {
                if (b.way_id != r.ways[j + 1]) continue;
                if (j + 2 == r.ways.size()) { // b leaves onto T: the sequence is complete
                    if (!only) return false;
                    only_ok = true;
                } else {
                    next.push_back(pack(i, j + 1));
                    only_ok = only_ok || only;
                }
            } else if (b.way_id == r.ways[j]) { // inside via way j: keep going along it
                next.push_back(pack(i, j));
                only_ok = only_ok || only;
            }
        }
        if (only_constrained && !only_ok) return false;
        std::sort(next.begin(), next.end());
        next.erase(std::unique(next.begin(), next.end()), next.end());
        return true;
    }

    static Result<void> build(RoutingGraph& g, const std::vector<ViaRule>& rules) {
        if (rules.size() >= (1u << 24)) return make_error(ErrorCode::LimitExceeded, "routing graph: via-way rules");
        // Positions implied by an arc alone: arriving on F at n0.
        std::map<std::uint32_t, std::vector<Position>> implied;
        for (std::size_t i = 0; i < rules.size(); ++i)
            for (std::uint32_t a = 0; a < g.real_arcs_; ++a)
                if (g.arcs_[a].way_id == rules[i].ways.front() && g.arcs_[a].to == rules[i].nodes.front()) implied[a].push_back(pack(i, 0));
        std::map<std::pair<std::uint32_t, std::vector<Position>>, std::uint32_t> ghost_id;
        std::vector<std::pair<std::uint32_t, std::vector<Position>>> ghosts; // (arc, positions beyond the implied ones)
        std::vector<std::pair<std::uint32_t, std::vector<Transition>>> lists; // (state, successors)
        std::vector<Transition> base;
        std::vector<Position> set, next;
        const auto expand = [&](std::uint32_t state, std::uint32_t arc, const std::vector<Position>& history) -> Result<void> {
            set = history;
            if (const auto it = implied.find(arc); it != implied.end()) set.insert(set.end(), it->second.begin(), it->second.end());
            std::sort(set.begin(), set.end());
            set.erase(std::unique(set.begin(), set.end()), set.end());
            g.successors(arc, base);
            std::vector<Transition> list;
            for (const Transition& t : base) {
                if (!step(rules, set, g.arcs_[arc], g.arcs_[t.arc], next)) continue;
                std::uint32_t target = t.arc;
                if (!next.empty()) {
                    const auto key = std::make_pair(t.arc, next);
                    auto [it, inserted] = ghost_id.emplace(key, 0u);
                    if (inserted) {
                        if (ghosts.size() >= kMaxGhostStates) return make_error(ErrorCode::LimitExceeded, "routing graph: via-way states");
                        it->second = static_cast<std::uint32_t>(g.real_arcs_ + ghosts.size());
                        ghosts.push_back(key);
                    }
                    target = it->second;
                }
                list.push_back({target, t.turn_ms});
            }
            lists.emplace_back(state, std::move(list));
            return {};
        };
        for (const auto& [arc, positions] : implied) {
            const auto done = expand(arc, arc, {});
            if (!done) return done;
        }
        for (std::size_t k = 0; k < ghosts.size(); ++k) { // ghosts grows while we walk it
            const auto copy = ghosts[k];
            const auto done = expand(static_cast<std::uint32_t>(g.real_arcs_ + k), copy.first, copy.second);
            if (!done) return done;
        }
        // Ghost arcs: copies of their real arc.
        g.ghost_origin_.reserve(ghosts.size());
        for (const auto& [arc, positions] : ghosts) {
            g.ghost_origin_.push_back(arc);
            g.arcs_.push_back(g.arcs_[arc]);
        }
        g.special_slot_.assign(g.arcs_.size(), kNoArc);
        g.special_first_.push_back(0);
        for (const auto& [state, list] : lists) {
            g.special_slot_[state] = static_cast<std::uint32_t>(g.special_first_.size() - 1);
            g.special_succ_.insert(g.special_succ_.end(), list.begin(), list.end());
            g.special_first_.push_back(static_cast<std::uint32_t>(g.special_succ_.size()));
        }
        return {};
    }
};


WayDirections way_directions(const RoadGraphWay& way, const RouteConfig& config) {
    const auto oneway = tag_of(way, "oneway");
    if (oneway) {
        if (*oneway == "yes" || *oneway == "true" || *oneway == "1") return {true, false};
        if (*oneway == "-1" || *oneway == "reverse") return {false, true};
        if (*oneway == "reversible" || *oneway == "alternating") return {false, false};
        if (*oneway == "no" || *oneway == "false" || *oneway == "0") return {true, true};
    }
    const auto junction = tag_of(way, "junction");
    if (junction && (*junction == "roundabout" || *junction == "circular")) return {true, false};
    const auto highway = tag_of(way, "highway");
    if (highway && contains(config.implied_oneway, *highway)) return {true, false};
    return {true, true};
}


const RouteConfig& default_route_config() {
    static const RouteConfig config = [] {
        RouteConfig c;
        c.classes = {
            {"motorway", 120, true},      {"motorway_link", 60, true}, {"trunk", 100, true},
            {"trunk_link", 50, true},     {"primary", 70, true},       {"primary_link", 40, true},
            {"secondary", 60, true},      {"secondary_link", 40, true}, {"tertiary", 50, true},
            {"tertiary_link", 30, true},  {"unclassified", 40, true},  {"residential", 30, true},
            {"living_street", 7, true},   {"service", 15, true},       {"track", 15, true},
        };
        // Typed helper: pair's converting constructor from an int literal trips MSVC C4244 (warnings are errors).
        const auto zone = [](const char* tag, std::uint16_t kmh) { return std::pair<std::string, std::uint16_t>{tag, kmh}; };
        c.zone_speeds = {zone("DE:urban", 50), zone("DE:rural", 100), zone("DE:living_street", 7), zone("DE:zone30", 30),
                         zone("DE:zone:30", 30), zone("DE:bicycle_road", 30), zone("walk", 7)};
        c.blocked_access = {"no", "private", "agricultural", "forestry", "emergency", "use_sidepath"};
        c.implied_oneway = {"motorway"};
        return c;
    }();
    return config;
}

std::uint32_t RoutingGraph::find_node(std::int64_t osm_id) const {
    const auto real = nodes();
    const auto it = std::lower_bound(real.begin(), real.end(), osm_id,
                                     [](const RouteGraphNode& n, std::int64_t id) { return n.osm_id < id; });
    if (it == real.end() || it->osm_id != osm_id) return kNoArc;
    return static_cast<std::uint32_t>(it - real.begin());
}

std::uint32_t RoutingGraph::turn_cost_ms(const RouteArc& in, const RouteArc& out) const {
    if (nodes_[out.from].degree < 3) return 0;
    const double ax = static_cast<double>(in.out_dx), ay = static_cast<double>(in.out_dy);
    const double bx = static_cast<double>(out.in_dx), by = static_cast<double>(out.in_dy);
    const double cross = ax * by - ay * bx, dot = ax * bx + ay * by;
    if (cross == 0.0 && dot >= 0.0) return 0;
    const double deg = det::atan2(cross, dot) * (180.0 / det::kPi);
    const double mag = std::fabs(deg);
    const bool left = deg > 0.0;
    if (mag <= config_.straight_max_deg) return 0;
    if (mag <= config_.slight_max_deg) return config_.slight_ms;
    if (mag <= config_.turn_max_deg) return left ? config_.left_ms : config_.right_ms;
    return left ? config_.sharp_left_ms : config_.sharp_right_ms;
}

void RoutingGraph::successors(std::uint32_t arc_index, std::vector<Transition>& out) const {
    out.clear();
    const RouteArc& in = arcs_[arc_index];
    const std::uint32_t v = in.to;
    // Rules at v that start on the arriving way.
    const auto rules_begin = rules_.begin() + nodes_[v].first_rule, rules_end = rules_.begin() + nodes_[v + 1].first_rule;
    auto first = std::lower_bound(rules_begin, rules_end, in.way_id,
                                  [](const RouteRule& r, std::int64_t w) { return r.from_way < w; });
    auto last = first;
    bool has_only = false;
    while (last != rules_end && last->from_way == in.way_id) {
        has_only = has_only || last->kind == RestrictionKind::Only;
        ++last;
    }
    const auto allowed_onto = [&](std::int64_t way, bool u_turn) {
        bool only_match = false;
        for (auto r = first; r != last; ++r) {
            if (r->to_way != way) continue;
            // A no_* rule whose from and to are the same way (no_u_turn) forbids only the U-turn,
            // never going straight on along that way through an interior via node.
            if (r->kind == RestrictionKind::No && r->from_way == r->to_way && !u_turn) continue;
            if (r->kind == RestrictionKind::No) return false;
            only_match = true;
        }
        return !has_only || only_match;
    };
    const std::uint32_t begin = nodes_[v].first_out, end = nodes_[v + 1].first_out;
    for (std::uint32_t b = begin; b < end; ++b) {
        if (b == in.reverse) continue;
        if (!allowed_onto(arcs_[b].way_id, false)) continue;
        out.push_back({b, turn_cost_ms(in, arcs_[b])});
    }
    // U-turn back along the same edge: only where nothing else may leave (dead end).
    if (out.empty() && in.reverse != kNoArc && allowed_onto(in.way_id, true))
        out.push_back({in.reverse, config_.u_turn_ms});
}

void RoutingGraph::state_successors(std::uint32_t state, std::vector<Transition>& out) const {
    if (!special_slot_.empty() && special_slot_[state] != kNoArc) {
        const std::uint32_t slot = special_slot_[state];
        out.assign(special_succ_.begin() + special_first_[slot], special_succ_.begin() + special_first_[slot + 1]);
        return;
    }
    successors(state, out); // an ordinary state is a real arc without history
}

std::uint64_t RoutingGraph::partial_cost_ms(std::uint32_t arc, std::uint32_t a, std::uint32_t b) const {
    const RouteArc& r = arcs_[arc];
    if (r.length_mm == 0 || b <= a) return 0;
    const std::uint64_t num = std::uint64_t{r.cost_ms} * (b - a);
    return (num + r.length_mm - 1) / r.length_mm;
}

geom::PointMm RoutingGraph::point_at(std::uint32_t arc, std::uint32_t offset_mm) const {
    const RouteArc& r = arcs_[arc];
    const auto pts = shape(arc);
    // Walk in way order; a backward arc's offset counts from the way's end.
    std::uint64_t target = r.forward ? offset_mm : (r.length_mm >= offset_mm ? r.length_mm - offset_mm : 0);
    for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
        const std::uint64_t seg = ceil_length_mm(pts[k], pts[k + 1]);
        if (target <= seg || k + 2 == pts.size()) {
            if (seg == 0) return pts[k];
            const double t = std::min(1.0, static_cast<double>(target) / static_cast<double>(seg));
            return {pts[k].x + std::llround(t * static_cast<double>(pts[k + 1].x - pts[k].x)),
                    pts[k].y + std::llround(t * static_cast<double>(pts[k + 1].y - pts[k].y))};
        }
        target -= seg;
    }
    return pts.empty() ? geom::PointMm{} : pts.back();
}

Result<RoutePoint> RoutingGraph::snap(geom::PointMm point, const SnapOptions& options) const {
    const std::int64_t cell = config_.snap_cell_mm;
    const std::int64_t r = options.max_distance_mm;
    const std::int64_t x0 = floor_div(point.x - r, cell), x1 = floor_div(point.x + r, cell);
    const std::int64_t y0 = floor_div(point.y - r, cell), y1 = floor_div(point.y + r, cell);
    const double hx = options.heading_dx, hy = options.heading_dy;
    const double hlen = det::sqrt(hx * hx + hy * hy);
    const bool use_heading = hlen > 0.0;
    const double cos_tol = det::cos(options.heading_tolerance_deg * (det::kPi / 180.0));
    struct Best {
        double distance = 0.0;
        std::uint32_t arc = kNoArc, segment = 0, chosen = kNoArc;
        double t = 0.0;
    };
    Best any, headed;
    // Order: distance, edge (its lower arc index), segment, chosen direction.
    const auto better = [](const Best& cur, double d, std::uint32_t arc, std::uint32_t seg, std::uint32_t dir) {
        if (cur.arc == kNoArc || d < cur.distance) return true;
        if (d > cur.distance) return false;
        if (arc != cur.arc) return arc < cur.arc;
        if (seg != cur.segment) return seg < cur.segment;
        return dir < cur.chosen;
    };
    for (std::int64_t cx = x0; cx <= x1; ++cx) {
        for (std::int64_t cy = y0; cy <= y1; ++cy) {
            const std::uint64_t key = cell_key(cx, cy);
            auto it = std::lower_bound(snap_index_.begin(), snap_index_.end(), key,
                                       [](const SnapEntry& e, std::uint64_t k) { return e.cell < k; });
            for (; it != snap_index_.end() && it->cell == key; ++it) {
                const RouteArc& a = arcs_[it->arc];
                const geom::PointMm p = shape_[a.shape_first + it->segment], q = shape_[a.shape_first + it->segment + 1];
                const double dx = static_cast<double>(q.x - p.x), dy = static_cast<double>(q.y - p.y);
                const double px = static_cast<double>(point.x - p.x), py = static_cast<double>(point.y - p.y);
                const double len2 = dx * dx + dy * dy;
                const double t = len2 > 0.0 ? std::clamp((px * dx + py * dy) / len2, 0.0, 1.0) : 0.0;
                const double ex = px - t * dx, ey = py - t * dy;
                const double d = det::sqrt(ex * ex + ey * ey);
                if (d > static_cast<double>(r)) continue;
                if (better(any, d, it->arc, it->segment, kNoArc)) any = {d, it->arc, it->segment, kNoArc, t};
                if (!use_heading || len2 == 0.0) continue;
                // Directions of travel of this segment: the edge's arcs.
                for (const std::uint32_t dir : {it->arc, a.reverse}) {
                    if (dir == kNoArc) continue;
                    // The shape is in way order: a backward arc travels the segment from q to p.
                    const double sign = arcs_[dir].forward ? 1.0 : -1.0;
                    const double c = sign * (dx * hx + dy * hy) / (det::sqrt(len2) * hlen);
                    if (c < cos_tol) continue;
                    if (better(headed, d, it->arc, it->segment, dir)) headed = {d, it->arc, it->segment, dir, t};
                }
            }
        }
    }
    if (any.arc == kNoArc) return make_error(ErrorCode::NotFound, "snap: no routable edge within max_distance_mm");
    const Best& pick = headed.arc != kNoArc ? headed : any;
    const RouteArc& a = arcs_[pick.arc];
    RoutePoint out;
    out.heading_matched = headed.arc != kNoArc;
    out.distance_mm = static_cast<std::uint64_t>(std::ceil(pick.distance));
    const geom::PointMm p = shape_[a.shape_first + pick.segment], q = shape_[a.shape_first + pick.segment + 1];
    out.position = {p.x + std::llround(pick.t * static_cast<double>(q.x - p.x)),
                    p.y + std::llround(pick.t * static_cast<double>(q.y - p.y))};
    // Way-order distance to the projected point: whole segments before it plus the part of this one.
    std::uint64_t along = 0;
    for (std::uint32_t k = 0; k < pick.segment; ++k)
        along += ceil_length_mm(shape_[a.shape_first + k], shape_[a.shape_first + k + 1]);
    along += static_cast<std::uint64_t>(std::llround(pick.t * static_cast<double>(ceil_length_mm(p, q))));
    along = std::min<std::uint64_t>(along, a.length_mm);
    for (const std::uint32_t dir : {pick.arc, a.reverse}) {
        if (dir == kNoArc || (out.heading_matched && dir != pick.chosen)) continue;
        const auto offset = arcs_[dir].forward ? along : a.length_mm - along;
        out.candidates[out.count++] = {dir, static_cast<std::uint32_t>(offset)};
    }
    return out;
}

Result<RoutingGraph> build_routing_graph(const RoadGraph& graph, std::span<const TurnRestriction> restrictions,
                                         const RouteConfig& config) {
    if (config.min_speed_kmh == 0 || config.max_speed_kmh < config.min_speed_kmh)
        return make_error(ErrorCode::InvalidArgument, "route config: speed bounds");
    for (std::size_t i = 1; i < graph.nodes.size(); ++i)
        if (graph.nodes[i - 1].osm_id >= graph.nodes[i].osm_id)
            return make_error(ErrorCode::Malformed, "routing graph: nodes not strictly ascending");
    for (std::size_t i = 1; i < graph.ways.size(); ++i)
        if (graph.ways[i - 1].osm_id >= graph.ways[i].osm_id)
            return make_error(ErrorCode::Malformed, "routing graph: ways not strictly ascending");
    const auto node_index = [&](std::int64_t id) -> std::int64_t {
        const auto it = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), id,
                                         [](const RoadGraphNode& n, std::int64_t x) { return n.osm_id < x; });
        return it == graph.nodes.end() || it->osm_id != id ? -1 : static_cast<std::int64_t>(it - graph.nodes.begin());
    };

    RoutingGraph out;
    out.config_ = config;
    RouteBuildStats& stats = out.stats_;

    // 1. Which ways, in which directions, at which speeds.
    struct WayUse {
        bool used = false;
        WayDirections dirs;
        std::uint16_t fwd_kmh = 0, bwd_kmh = 0;
    };
    std::vector<WayUse> use(graph.ways.size());
    std::vector<std::vector<std::uint32_t>> refs(graph.ways.size()); // node indices, repeats dropped
    for (std::size_t w = 0; w < graph.ways.size(); ++w) {
        const RoadGraphWay& way = graph.ways[w];
        const auto highway = tag_of(way, "highway");
        const auto cls = highway ? std::find_if(config.classes.begin(), config.classes.end(),
                                                [&](const RouteClassSpeed& c) { return c.highway == *highway; })
                                 : config.classes.end();
        if (cls == config.classes.end() || !cls->routable) { ++stats.ways_not_routable; continue; }
        if (access_blocked(way, config)) { ++stats.ways_access_blocked; continue; }
        WayUse& u = use[w];
        u.dirs = way_directions(way, config);
        if (!u.dirs.forward && !u.dirs.backward) { ++stats.ways_no_direction; continue; }
        if (way.node_ids.size() < 2) return make_error(ErrorCode::Malformed, "routing graph: way with fewer than 2 refs");
        for (const std::int64_t id : way.node_ids) {
            const std::int64_t n = node_index(id);
            if (n < 0) return make_error(ErrorCode::Malformed, "routing graph: way references a missing node");
            refs[w].push_back(static_cast<std::uint32_t>(n));
        }
        u.used = true;
        u.fwd_kmh = way_speed(way, "maxspeed:forward", cls->default_kmh, config);
        u.bwd_kmh = way_speed(way, "maxspeed:backward", cls->default_kmh, config);
        ++stats.ways_used;
    }

    // 2. Decision nodes: way ends, nodes at two or more way positions, via nodes.
    std::vector<std::uint32_t> occurrences(graph.nodes.size(), 0);
    std::vector<char> keep(graph.nodes.size(), 0);
    for (std::size_t w = 0; w < graph.ways.size(); ++w) {
        if (!use[w].used) continue;
        const auto& r = refs[w];
        keep[r.front()] = keep[r.back()] = 1;
        for (std::size_t i = 0; i < r.size(); ++i)
            if (i == 0 || r[i] != r[i - 1]) ++occurrences[r[i]];
    }
    for (const TurnRestriction& tr : restrictions) {
        if (tr.via_node == 0) continue;
        const std::int64_t n = node_index(tr.via_node);
        if (n >= 0 && occurrences[static_cast<std::size_t>(n)] > 0) keep[static_cast<std::size_t>(n)] = 1;
    }
    std::vector<std::uint32_t> compact(graph.nodes.size(), kNoArc);
    for (std::size_t i = 0; i < graph.nodes.size(); ++i) {
        if (occurrences[i] >= 2) keep[i] = 1;
        if (keep[i] && occurrences[i] > 0) {
            compact[i] = static_cast<std::uint32_t>(out.nodes_.size());
            RouteGraphNode n;
            n.osm_id = graph.nodes[i].osm_id;
            n.position = graph.nodes[i].position;
            out.nodes_.push_back(n);
        }
    }
    if (out.nodes_.size() > config.max_nodes) return make_error(ErrorCode::LimitExceeded, "routing graph: max_nodes");

    // 3. Edges between consecutive decision nodes of each way.
    std::vector<Edge> edges;
    std::vector<geom::PointMm>& shape = out.shape_;
    for (std::size_t w = 0; w < graph.ways.size(); ++w) {
        if (!use[w].used) continue;
        const auto& r = refs[w];
        const auto pos = [&](std::size_t i) { return graph.nodes[r[i]].position; };
        std::size_t start = 0;
        std::size_t shape_start = shape.size();
        shape.push_back(pos(0));
        std::uint64_t length = 0;
        std::int64_t first_dx = 0, first_dy = 0;
        bool have_first = false;
        for (std::size_t i = 1; i < r.size(); ++i) {
            if (r[i] == r[i - 1]) continue;
            const geom::PointMm a = pos(i - 1), b = pos(i);
            length += ceil_length_mm(a, b);
            shape.push_back(b);
            if (!have_first) { first_dx = b.x - a.x; first_dy = b.y - a.y; have_first = true; }
            if (compact[r[i]] == kNoArc) continue;
            // OSM ref-pair index of the edge's first segment: the first i' > start with a distinct ref.
            std::size_t seg_first = start;
            while (r[seg_first + 1] == r[seg_first]) ++seg_first;
            Edge e{compact[r[start]], compact[r[i]], w, static_cast<std::uint32_t>(seg_first),
                   static_cast<std::uint32_t>(i - 1), length, first_dx, first_dy, b.x - a.x, b.y - a.y,
                   static_cast<std::uint32_t>(shape_start), static_cast<std::uint32_t>(shape.size() - shape_start)};
            if (e.length_mm > std::numeric_limits<std::uint32_t>::max())
                return make_error(ErrorCode::LimitExceeded, "routing graph: edge longer than 4294 km");
            edges.push_back(e);
            start = i;
            length = 0;
            have_first = false;
            shape_start = shape.size();
            shape.push_back(b);
        }
        shape.resize(shape_start); // the way's last ref only opened an empty edge
    }

    // 4. Arcs, canonical order (from node, way id, first segment, forward first).
    std::vector<RouteArc>& arcs = out.arcs_;
    for (const Edge& e : edges) {
        const WayUse& u = use[e.way];
        ++out.nodes_[e.from].degree;
        ++out.nodes_[e.to].degree;
        for (const bool forward : {true, false}) {
            if (forward ? !u.dirs.forward : !u.dirs.backward) continue;
            RouteArc a;
            a.from = forward ? e.from : e.to;
            a.to = forward ? e.to : e.from;
            a.way_id = graph.ways[e.way].osm_id;
            a.seg_first = e.seg_first;
            a.seg_last = e.seg_last;
            a.forward = forward;
            a.rank = static_cast<std::uint8_t>(std::clamp(graph.ways[e.way].rank, 0, 255));
            a.speed_kmh = forward ? u.fwd_kmh : u.bwd_kmh;
            a.length_mm = static_cast<std::uint32_t>(e.length_mm);
            a.shape_first = e.shape_first;
            a.shape_count = e.shape_count;
            const std::uint64_t num = e.length_mm * 36u, den = std::uint64_t{a.speed_kmh} * 10u;
            a.cost_ms = static_cast<std::uint32_t>((num + den - 1) / den);
            if (forward) { a.in_dx = e.first_dx; a.in_dy = e.first_dy; a.out_dx = e.last_dx; a.out_dy = e.last_dy; }
            else { a.in_dx = -e.last_dx; a.in_dy = -e.last_dy; a.out_dx = -e.first_dx; a.out_dy = -e.first_dy; }
            arcs.push_back(a);
        }
    }
    if (arcs.size() > config.max_arcs) return make_error(ErrorCode::LimitExceeded, "routing graph: max_arcs");
    std::sort(arcs.begin(), arcs.end(), [](const RouteArc& a, const RouteArc& b) {
        if (a.from != b.from) return a.from < b.from;
        if (a.way_id != b.way_id) return a.way_id < b.way_id;
        if (a.seg_first != b.seg_first) return a.seg_first < b.seg_first;
        return a.forward > b.forward;
    });
    {
        // Pair the two directions of each edge: (way, seg_first) identifies the edge.
        std::vector<std::uint32_t> order(arcs.size());
        for (std::uint32_t i = 0; i < order.size(); ++i) order[i] = i;
        std::sort(order.begin(), order.end(), [&](std::uint32_t a, std::uint32_t b) {
            if (arcs[a].way_id != arcs[b].way_id) return arcs[a].way_id < arcs[b].way_id;
            if (arcs[a].seg_first != arcs[b].seg_first) return arcs[a].seg_first < arcs[b].seg_first;
            return arcs[a].forward > arcs[b].forward;
        });
        for (std::size_t i = 0; i + 1 < order.size(); ++i) {
            RouteArc& a = arcs[order[i]];
            RouteArc& b = arcs[order[i + 1]];
            if (a.way_id == b.way_id && a.seg_first == b.seg_first) {
                a.reverse = order[i + 1];
                b.reverse = order[i];
            }
        }
    }
    {
        std::uint32_t at = 0;
        for (std::uint32_t n = 0; n < out.nodes_.size(); ++n) {
            out.nodes_[n].first_out = at;
            while (at < arcs.size() && arcs[at].from == n) ++at;
        }
    }

    // 5. Via-node rules, sorted by (node, from way, to way, kind).
    std::vector<std::pair<std::uint32_t, RouteRule>> rules;
    std::unordered_map<std::int64_t, char> used_way;
    for (std::size_t w = 0; w < graph.ways.size(); ++w) if (use[w].used) used_way.emplace(graph.ways[w].osm_id, char{1});
    std::vector<std::pair<std::uint32_t, std::int64_t>> arrivals; // (head node, way) of every arc
    arrivals.reserve(arcs.size());
    for (const RouteArc& a : arcs) arrivals.emplace_back(a.to, a.way_id);
    std::sort(arrivals.begin(), arrivals.end());
    for (const TurnRestriction& tr : restrictions) {
        if (tr.via_node == 0) continue; // via-way rules: step 7
        const std::uint32_t v = out.find_node(tr.via_node);
        bool from_arrives = false, to_leaves = false;
        if (v != kNoArc && used_way.count(tr.from_way) && used_way.count(tr.to_way)) {
            from_arrives = std::binary_search(arrivals.begin(), arrivals.end(), std::make_pair(v, tr.from_way));
            for (std::uint32_t b = out.nodes_[v].first_out; b < arcs.size() && arcs[b].from == v; ++b)
                if (arcs[b].way_id == tr.to_way) to_leaves = true;
        }
        if (!from_arrives || !to_leaves) { ++stats.restrictions_unmatched; continue; }
        rules.push_back({v, RouteRule{tr.from_way, tr.to_way, tr.kind}});
        ++stats.restrictions_applied;
    }
    std::sort(rules.begin(), rules.end(), [](const auto& a, const auto& b) {
        if (a.first != b.first) return a.first < b.first;
        if (a.second.from_way != b.second.from_way) return a.second.from_way < b.second.from_way;
        if (a.second.to_way != b.second.to_way) return a.second.to_way < b.second.to_way;
        return a.second.kind < b.second.kind;
    });
    {
        std::size_t at = 0;
        for (std::uint32_t n = 0; n < out.nodes_.size(); ++n) {
            out.nodes_[n].first_rule = static_cast<std::uint32_t>(at);
            while (at < rules.size() && rules[at].first == n) ++at;
        }
        for (const auto& [node, rule] : rules) out.rules_.push_back(rule);
    }
    // 6. Snapping index: every segment of every edge, in each cell its bounding box touches.
    {
        const std::int64_t cell = config.snap_cell_mm;
        if (cell <= 0) return make_error(ErrorCode::InvalidArgument, "route config: snap_cell_mm");
        for (std::uint32_t a = 0; a < arcs.size(); ++a) {
            if (arcs[a].reverse != kNoArc && arcs[a].reverse < a) continue;
            for (std::uint32_t k = 0; k + 1 < arcs[a].shape_count; ++k) {
                const geom::PointMm p = shape[arcs[a].shape_first + k], q = shape[arcs[a].shape_first + k + 1];
                const std::int64_t x0 = floor_div(std::min(p.x, q.x), cell), x1 = floor_div(std::max(p.x, q.x), cell);
                const std::int64_t y0 = floor_div(std::min(p.y, q.y), cell), y1 = floor_div(std::max(p.y, q.y), cell);
                for (std::int64_t cx = x0; cx <= x1; ++cx)
                    for (std::int64_t cy = y0; cy <= y1; ++cy) out.snap_index_.push_back({cell_key(cx, cy), a, k});
            }
        }
        std::sort(out.snap_index_.begin(), out.snap_index_.end(), [](const auto& x, const auto& y) {
            if (x.cell != y.cell) return x.cell < y.cell;
            if (x.arc != y.arc) return x.arc < y.arc;
            return x.segment < y.segment;
        });
    }
    RouteGraphNode sentinel;
    sentinel.first_out = static_cast<std::uint32_t>(arcs.size());
    sentinel.first_rule = static_cast<std::uint32_t>(out.rules_.size());
    out.nodes_.push_back(sentinel);
    out.real_arcs_ = arcs.size();

    // 7. Via-way rules: search states with history (see ViaRule above).
    {
        std::vector<ViaRule> via;
        for (const TurnRestriction& tr : restrictions) {
            if (tr.via_node != 0) continue;
            ++stats.restrictions_via_way;
            if (tr.via_ways.empty() || tr.via_ways.size() > kMaxViaWays) { ++stats.restrictions_unmatched; continue; }
            const std::size_t before = via.size();
            ViaBuilder::instances(out, graph, refs, compact, used_way, tr, via);
            if (via.size() == before) ++stats.restrictions_unmatched;
            else ++stats.restrictions_via_way_applied;
        }
        stats.via_way_instances = via.size();
        if (!via.empty()) {
            auto built = ViaBuilder::build(out, via);
            if (!built) return built.error();
        }
        stats.via_way_states = out.arcs_.size() - out.real_arcs_;
    }
    if (out.arcs_.size() > config.max_arcs) return make_error(ErrorCode::LimitExceeded, "routing graph: max_arcs");
    return out;
}

} // namespace g2m::route
