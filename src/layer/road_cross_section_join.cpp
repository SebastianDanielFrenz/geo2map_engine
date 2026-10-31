// Street fidelity P1 slice 4: join resolution (design section 4).
//
// A join is computed in ONE canonical frame (the lower way id's own direction) so that the two stretches meeting
// at a node, resolved independently in different tiles, derive the identical outcome from the identical inputs:
// matching, join widths, residual, absorber and corrections. Each stretch then reads its own side of that
// outcome, mapped into its own frame (band order reversed and offsets negated when its direction is opposite).
#include "g2m/layer/road_cross_section.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>

namespace g2m {
namespace {
constexpr int kBwd = 0, kBoth = 1, kFwd = 2;

int group_of(LaneDirection d) {
    switch (d) {
    case LaneDirection::Backward: return kBwd;
    case LaneDirection::Forward: return kFwd;
    default: return kBoth;
    }
}
int provenance_rank(Provenance p) {
    switch (p) {
    case Provenance::Official: return 3;
    case Provenance::Tagged: return 2;
    case Provenance::Derived: return 1;
    default: return 0;
    }
}
// Verge and sidewalk are the same outermost position: a sidewalk that begins replaces the verge.
int match_class(LaneKind kind) { return kind == LaneKind::Verge || kind == LaneKind::Sidewalk ? 100 : static_cast<int>(kind); }

RoadCrossSection mirrored(const RoadCrossSection& in) {
    RoadCrossSection out = in;
    std::reverse(out.bands.begin(), out.bands.end());
    for (auto& b : out.bands) {
        if (b.direction == LaneDirection::Forward) b.direction = LaneDirection::Backward;
        else if (b.direction == LaneDirection::Backward) b.direction = LaneDirection::Forward;
        std::swap(b.start, b.end);
    }
    out.anchor.band = static_cast<std::uint8_t>(in.bands.size() - 1 - in.anchor.band);
    if (in.anchor.edge == AnchorEdge::Left) out.anchor.edge = AnchorEdge::Right;
    else if (in.anchor.edge == AnchorEdge::Right) out.anchor.edge = AnchorEdge::Left;
    out.offset.start_mm = -in.offset.start_mm; out.offset.end_mm = -in.offset.end_mm;
    return out;
}

struct Side {
    RoadCrossSection xs;
    const RoadClassRules* cls = nullptr;
    std::int64_t id = 0;
    std::vector<int> group[3]; // counted band indices in band order
    int first_counted = 0, last_counted = 0;
    int counted() const { return static_cast<int>(group[0].size() + group[1].size() + group[2].size()); }
    int groups() const { return int(!group[0].empty()) + int(!group[1].empty()) + int(!group[2].empty()); }
};
Side make_side(RoadCrossSection xs, const RoadClassRules* cls, std::int64_t id) {
    Side s; s.xs = std::move(xs); s.cls = cls; s.id = id;
    s.first_counted = -1;
    for (std::size_t i = 0; i < s.xs.bands.size(); ++i) {
        if (!s.xs.bands[i].counted) continue;
        if (s.first_counted < 0) s.first_counted = static_cast<int>(i);
        s.last_counted = static_cast<int>(i);
        s.group[group_of(s.xs.bands[i].direction)].push_back(static_cast<int>(i));
    }
    return s;
}
std::vector<int> travel_order(const Side& s, int g) {
    std::vector<int> v = s.group[g];
    if (g == kBwd) std::reverse(v.begin(), v.end());
    return v;
}

struct Plan {
    std::vector<int> pu, pd; // partner band index in the other side, -1 unmatched
    bool split = false, split_u = false;
};
struct SideEval { std::vector<std::int32_t> jw, len; }; // width at the join, ramp length (0 = none)

void pair_up(Plan& plan, int u, int d) { plan.pu[static_cast<std::size_t>(u)] = d; plan.pd[static_cast<std::size_t>(d)] = u; }

void match_group(const Side& U, const Side& D, int g, bool choice_right, Plan& plan) {
    const auto tu = travel_order(U, g), td = travel_order(D, g);
    const std::size_t nu = tu.size(), nd = td.size(), n = std::min(nu, nd);
    if (n == 0) return;
    const std::size_t ou = (nu > n && !choice_right) ? nu - n : 0, od = (nd > n && !choice_right) ? nd - n : 0;
    for (std::size_t i = 0; i < n; ++i) pair_up(plan, tu[ou + i], td[od + i]);
}
void match_outer(const Side& U, const Side& D, Plan& plan) {
    for (int side = 0; side < 2; ++side) {
        const auto outward = [&](const Side& s) {
            std::vector<int> v;
            if (side == 0) for (int i = s.first_counted - 1; i >= 0; --i) v.push_back(i);
            else for (int i = s.last_counted + 1; i < static_cast<int>(s.xs.bands.size()); ++i) v.push_back(i);
            return v;
        };
        const auto lu = outward(U), ld = outward(D);
        std::size_t next = 0;
        for (const int u : lu) {
            for (std::size_t j = next; j < ld.size(); ++j)
                if (match_class(U.xs.bands[static_cast<std::size_t>(u)].kind) == match_class(D.xs.bands[static_cast<std::size_t>(ld[j])].kind)) {
                    pair_up(plan, u, ld[j]); next = j + 1; break;
                }
        }
    }
}

bool incompatible(const Side& U, const Side& D) {
    return U.group[kBwd].empty() != D.group[kBwd].empty() || U.group[kFwd].empty() != D.group[kFwd].empty();
}

// Join width and ramp length of every band of both sides for a plan.
void evaluate_widths(const Side& U, const Side& D, const Plan& plan, SideEval& eu, SideEval& ed) {
    const std::size_t nu = U.xs.bands.size(), nd = D.xs.bands.size();
    eu.jw.assign(nu, 0); eu.len.assign(nu, 0); ed.jw.assign(nd, 0); ed.len.assign(nd, 0);
    const auto ramp_len = [](const Side& s, bool counted, bool matched) {
        const auto& t = s.cls->taper;
        return (counted ? (matched ? t.width_change_m : t.lane_add_m) : t.edge_band_m) * 1000;
    };
    for (std::size_t i = 0; i < nu; ++i) {
        const auto& b = U.xs.bands[i];
        const int j = plan.pu[i];
        if (plan.split && b.counted) continue;
        if (j < 0) { eu.jw[i] = 0; eu.len[i] = ramp_len(U, b.counted, false); continue; }
        const auto& o = D.xs.bands[static_cast<std::size_t>(j)];
        std::int32_t fu, fd;
        if (b.width_mm == o.width_mm) fu = fd = b.width_mm;
        else {
            const int ru = provenance_rank(b.provenance.width), rd = provenance_rank(o.provenance.width);
            if (ru < rd) fu = fd = o.width_mm;
            else if (rd < ru) fu = fd = b.width_mm;
            else fu = fd = (b.width_mm + o.width_mm) / 2;
        }
        eu.jw[i] = fu; ed.jw[static_cast<std::size_t>(j)] = fd;
        eu.len[i] = fu != b.width_mm ? ramp_len(U, b.counted, true) : 0;
        ed.len[static_cast<std::size_t>(j)] = fd != o.width_mm ? ramp_len(D, o.counted, true) : 0;
    }
    for (std::size_t j = 0; j < nd; ++j) {
        const auto& o = D.xs.bands[j];
        if (plan.split && o.counted) continue;
        if (plan.pd[j] < 0) { ed.jw[j] = 0; ed.len[j] = ramp_len(D, o.counted, false); }
    }
    if (plan.split) {
        const Side& S = plan.split_u ? U : D;
        const Side& T = plan.split_u ? D : U;
        SideEval& es = plan.split_u ? eu : ed;
        SideEval& et = plan.split_u ? ed : eu;
        std::int64_t total_t = 0, sum_s = 0;
        for (int i = T.first_counted; i <= T.last_counted; ++i) total_t += T.xs.bands[static_cast<std::size_t>(i)].width_mm;
        for (int i = S.first_counted; i <= S.last_counted; ++i) sum_s += S.xs.bands[static_cast<std::size_t>(i)].width_mm;
        std::int64_t given = 0;
        for (int i = S.first_counted; i <= S.last_counted; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            const std::int64_t w = S.xs.bands[idx].width_mm;
            const std::int64_t part = total_t * w / sum_s;
            es.jw[idx] = static_cast<std::int32_t>(part); given += part;
        }
        es.jw[static_cast<std::size_t>(S.last_counted)] += static_cast<std::int32_t>(total_t - given); // remainder on the right lane
        for (int i = S.first_counted; i <= S.last_counted; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            es.len[idx] = es.jw[idx] != S.xs.bands[idx].width_mm ? ramp_len(S, true, true) : 0;
        }
        for (int i = T.first_counted; i <= T.last_counted; ++i) {
            const auto idx = static_cast<std::size_t>(i);
            et.jw[idx] = T.xs.bands[idx].width_mm; et.len[idx] = 0;
        }
    }
}

// Position (half millimetres, + = left of the way line) of the left edge of band i with the join widths and no
// correction.
std::int64_t left_edge2(const Side& s, const SideEval& e, int i) {
    const auto a = static_cast<int>(s.xs.anchor.band);
    const std::int64_t wa = e.jw[static_cast<std::size_t>(a)];
    std::int64_t left = s.xs.anchor.edge == AnchorEdge::Left ? 0 : s.xs.anchor.edge == AnchorEdge::Centre ? wa : 2 * wa;
    for (int k = a - 1; k >= i; --k) left += 2 * static_cast<std::int64_t>(e.jw[static_cast<std::size_t>(k)]);
    for (int k = a; k < i; ++k) left -= 2 * static_cast<std::int64_t>(e.jw[static_cast<std::size_t>(k)]);
    return left;
}

std::int32_t residual(const Side& U, const Side& D, const Plan& plan, const SideEval& eu, const SideEval& ed) {
    int iu, id;
    if (plan.split) { iu = U.first_counted; id = D.first_counted; }
    else {
        iu = -1; id = -1;
        for (int i = U.first_counted; i <= U.last_counted && iu < 0; ++i) {
            const int j = plan.pu[static_cast<std::size_t>(i)];
            if (j >= 0 && D.xs.bands[static_cast<std::size_t>(j)].counted) { iu = i; id = j; }
        }
        if (iu < 0) return 0; // cannot happen for compatible sides
    }
    return static_cast<std::int32_t>((left_edge2(D, ed, id) - left_edge2(U, eu, iu)) / 2);
}

constexpr std::uint16_t kRightIsh = kTurnRight | kTurnSlightRight | kTurnSharpRight | kTurnMergeToLeft;
constexpr std::uint16_t kLeftIsh = kTurnLeft | kTurnSlightLeft | kTurnSharpLeft | kTurnReverse | kTurnMergeToRight;

struct JoinOutcome {
    SideEval u, d;
    std::int32_t delta = 0;
    JoinKind kind = JoinKind::Consistent;
    std::int32_t corr_u = 0, corr_d = 0;
    bool split = false;   // the sides' direction groups do not match (section 4.7)
};

std::int32_t floor_div2(std::int32_t v) { return v >= 0 ? v / 2 : -((-v + 1) / 2); }

JoinOutcome resolve_join(const Side& U, const Side& D, const RoadRules& rules) {
    JoinOutcome out;
    Plan plan;
    plan.pu.assign(U.xs.bands.size(), -1); plan.pd.assign(D.xs.bands.size(), -1);
    bool any_k = false;
    if (incompatible(U, D)) {
        plan.split = true;
        const int cu = U.counted(), cd = D.counted();
        plan.split_u = cu > cd || (cu == cd && U.groups() > D.groups());
    } else {
        std::array<bool, 3> choice_right{rules.added_lane_side == AddedLaneSide::Right, rules.added_lane_side == AddedLaneSide::Right,
                                         rules.added_lane_side == AddedLaneSide::Right};
        std::array<bool, 3> open{false, false, false};
        const bool any_explicit = U.xs.anchor.source == PlacementSource::Explicit || D.xs.anchor.source == PlacementSource::Explicit;
        for (int g = 0; g < 3; ++g) {
            const std::size_t nu = U.group[g].size(), nd = D.group[g].size();
            if (nu != nd) any_k = true;
            if (nu == nd || std::min(nu, nd) == 0) continue;
            const Side& W = nu > nd ? U : D;
            const std::size_t k = std::max(nu, nd) - std::min(nu, nd);
            const auto tw = travel_order(W, g);
            const auto all_ish = [&](std::size_t from, std::uint16_t mask) {
                for (std::size_t i = from; i < from + k; ++i)
                    if (!(W.xs.bands[static_cast<std::size_t>(tw[i])].turn & mask)) return false;
                return true;
            };
            const bool right_tokens = all_ish(tw.size() - k, kRightIsh), left_tokens = all_ish(0, kLeftIsh);
            const bool right_ok = right_tokens && !left_tokens, left_ok = left_tokens && !right_tokens;
            if (right_ok) choice_right[g] = true;
            else if (left_ok) choice_right[g] = false;
            else open[g] = true;
        }
        const auto build = [&](const std::array<bool, 3>& c) {
            Plan p; p.pu.assign(U.xs.bands.size(), -1); p.pd.assign(D.xs.bands.size(), -1);
            for (int g = 0; g < 3; ++g) match_group(U, D, g, c[static_cast<std::size_t>(g)], p);
            match_outer(U, D, p);
            return p;
        };
        if (any_explicit)
            for (int g = 0; g < 3; ++g) {
                if (!open[g]) continue;
                int accepted = 0; bool accepted_right = choice_right[g];
                for (const bool right : {true, false}) {
                    auto c = choice_right; c[static_cast<std::size_t>(g)] = right;
                    const Plan p = build(c);
                    SideEval eu, ed; evaluate_widths(U, D, p, eu, ed);
                    if (std::abs(residual(U, D, p, eu, ed)) < 1) { ++accepted; accepted_right = right; }
                }
                if (accepted == 1) choice_right[g] = accepted_right;
            }
        plan = build(choice_right);
    }
    if (plan.split) match_outer(U, D, plan);
    evaluate_widths(U, D, plan, out.u, out.d);
    out.split = plan.split;
    out.delta = residual(U, D, plan, out.u, out.d);

    const int prio_u0 = U.xs.anchor.source == PlacementSource::Transition ? 0 : U.xs.anchor.source == PlacementSource::Absent ? 1 : 2;
    const int prio_d0 = D.xs.anchor.source == PlacementSource::Transition ? 0 : D.xs.anchor.source == PlacementSource::Absent ? 1 : 2;
    int prio_u = prio_u0, prio_d = prio_d0;
    if (prio_u == 0 && prio_d == 0) prio_u = prio_d = 1;
    if (out.delta == 0) {
        out.kind = any_k && !plan.split ? JoinKind::LaneChange : JoinKind::Consistent;
        return out;
    }
    if (prio_u == 2 && prio_d == 2) {
        out.kind = JoinKind::PlacementConflict;
        out.corr_d = -floor_div2(out.delta);
        out.corr_u = out.delta - floor_div2(out.delta);
        return out;
    }
    out.kind = JoinKind::InferredAlignment;
    bool u_absorbs;
    if (prio_u != prio_d) u_absorbs = prio_u < prio_d;
    else {
        const int cu = U.counted(), cd = D.counted();
        if (cu != cd) u_absorbs = cu > cd;
        else {
            const auto carriageway = [](const Side& s, const SideEval& e) {
                std::int64_t sum = 0;
                for (std::size_t i = 0; i < s.xs.bands.size(); ++i) {
                    const auto k = s.xs.bands[i].kind;
                    if (s.xs.bands[i].counted || k == LaneKind::Cycle || k == LaneKind::Parking) sum += e.jw[i];
                }
                return sum;
            };
            const auto wu = carriageway(U, out.u), wd = carriageway(D, out.d);
            if (wu != wd) u_absorbs = wu > wd;
            else u_absorbs = U.id > D.id;
        }
    }
    if (u_absorbs) out.corr_u = out.delta; else out.corr_d = -out.delta;
    return out;
}

std::string_view highway_of(const RoadGraphWay& way) {
    for (const auto& t : way.tags) if (t.first == "highway") return t.second;
    return {};
}
const RoadGraphWay* find_way(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.ways.begin(), g.ways.end(), id, [](const RoadGraphWay& w, std::int64_t v) { return w.osm_id < v; });
    return it != g.ways.end() && it->osm_id == id ? &*it : nullptr;
}

// ---- Merge lanes and lane drops at a node of degree three (vault G2M-028) ----
//
// A one-way link that meets two one-way carriageways at a node, where the carriageway on the other side has exactly the
// link's counted lanes more than the through arm on the link's side, models a parallel lane (Einfaedelungs- or
// Ausfaedelungsstreifen): the two carriageways (the through arm T and the single arm S) join like at a degree-two node
// (a lane-add / lane-drop join, the width ramp of the added lane from the class taper), and the link R lands on the
// centre line of the added lanes instead of the way line. The classification reads the three ways' tags, the node
// positions and the rules only, and is symmetric in the three arms, so each of the three stretches, derived in any
// tile, reaches the same verdict.
struct MergeNode {
    bool merge = true;                       // false: a diverge (a lane drop with an exit)
    std::int64_t through_id = 0, single_id = 0, ramp_id = 0;
    bool through_leaves = false, single_leaves = false; // direction of each arm at the node
    std::int32_t landing_mm = 0;             // the ramp's offset at its node end (its own frame, + = left)
    std::int32_t landing_length_mm = 0;
};

const RoadGraphNode* find_node(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.nodes.begin(), g.nodes.end(), id, [](const RoadGraphNode& n, std::int64_t v) { return n.osm_id < v; });
    return it != g.nodes.end() && it->osm_id == id ? &*it : nullptr;
}

struct NodeArm {
    RoadNodeArm arm;
    const RoadGraphWay* way = nullptr;
    const RoadClassRules* cls = nullptr;
    RoadCrossSection xs;
    int counted = 0;
    double fx = 0, fy = 0;     // from the node to a point of the way at least direction_probe_m away (metres, UTM)
    double dx = 0, dy = 0;     // unit travel direction at the node
};

bool make_arm(const RoadReferenceBuilder& builder, const RoadRules& rules, const RoadNodeArm& arm, const RoadGraphNode& node, NodeArm& out) {
    const RoadGraph& g = builder.graph();
    out.arm = arm;
    out.way = find_way(g, arm.way_id);
    if (!out.way) return false;
    out.cls = rules.find_class(highway_of(*out.way));
    if (!out.cls) return false;
    auto base = resolve_cross_section_base(*out.way, rules);
    if (!base || !base->xs) return false;
    out.xs = std::move(*base->xs);
    if (!(out.xs.flags & kXsOneway) || (out.xs.flags & kXsReversedOneway)) return false;   // a one-way carriageway driven along its refs
    for (const auto& b : out.xs.bands) if (b.counted) out.counted += 1;
    if (out.counted < 1) return false;
    // The point at least direction_probe_m away along the way (the last node if the way ends sooner).
    const double probe = static_cast<double>(rules.merge_lane.direction_probe_m);
    const std::ptrdiff_t step = arm.leaves_node ? 1 : -1;
    std::optional<geom::PointMm> far;
    for (auto k = static_cast<std::ptrdiff_t>(arm.ref) + step; k >= 0 && k < static_cast<std::ptrdiff_t>(out.way->node_ids.size()); k += step) {
        const auto* n = find_node(g, out.way->node_ids[static_cast<std::size_t>(k)]);
        if (!n) return false;
        if (n->position == node.position) continue;
        far = n->position;
        const double fx = static_cast<double>(far->x - node.position.x) / 1000.0, fy = static_cast<double>(far->y - node.position.y) / 1000.0;
        if (std::sqrt(fx * fx + fy * fy) >= probe) break;
    }
    if (!far) return false;
    out.fx = static_cast<double>(far->x - node.position.x) / 1000.0;
    out.fy = static_cast<double>(far->y - node.position.y) / 1000.0;
    const double len = std::sqrt(out.fx * out.fx + out.fy * out.fy);
    if (!(len > 0)) return false;
    out.dx = (arm.leaves_node ? out.fx : -out.fx) / len;
    out.dy = (arm.leaves_node ? out.fy : -out.fy) / len;
    return true;
}

double angle_deg(double cos_angle) { return det::acos(std::clamp(cos_angle, -1.0, 1.0)) * 180.0 / det::kPi; }

std::optional<MergeNode> classify_merge_node(const RoadReferenceBuilder& builder, std::int64_t node_id, const RoadRules& rules) {
    const MergeLaneRules& ml = rules.merge_lane;
    if (!ml.enabled) return {};
    auto arms = builder.arms(node_id);
    if (arms.size() != 3) return {};
    std::sort(arms.begin(), arms.end(), [](const RoadNodeArm& a, const RoadNodeArm& b) { return a.way_id != b.way_id ? a.way_id < b.way_id : a.ref < b.ref; });
    if (arms[0].way_id == arms[1].way_id || arms[1].way_id == arms[2].way_id) return {};   // a way passing through is no merge lane
    const RoadGraphNode* node = find_node(builder.graph(), node_id);
    if (!node) return {};
    std::array<NodeArm, 3> a;
    for (std::size_t i = 0; i < 3; ++i) if (!make_arm(builder, rules, arms[i], *node, a[i])) return {};
    const int leaving = static_cast<int>(a[0].arm.leaves_node) + static_cast<int>(a[1].arm.leaves_node) + static_cast<int>(a[2].arm.leaves_node);
    if (leaving != 1 && leaving != 2) return {};
    MergeNode out;
    out.merge = leaving == 1;                  // one arm leaves: two roads merge
    // The single arm is the one alone in its direction.
    std::size_t si = 0;
    for (std::size_t i = 0; i < 3; ++i) if (a[i].arm.leaves_node == out.merge) si = i;
    const std::size_t xi = (si + 1) % 3, yi = (si + 2) % 3;
    const NodeArm& S = a[si];
    const double cos_x = a[xi].dx * S.dx + a[xi].dy * S.dy, cos_y = a[yi].dx * S.dx + a[yi].dy * S.dy;
    const bool x_through = cos_x > cos_y || (cos_x == cos_y && a[xi].arm.way_id < a[yi].arm.way_id);
    const NodeArm& T = x_through ? a[xi] : a[yi];
    const NodeArm& R = x_through ? a[yi] : a[xi];
    if (angle_deg(x_through ? cos_x : cos_y) > static_cast<double>(ml.max_through_deflection_deg)) return {};
    if (angle_deg(R.dx * S.dx + R.dy * S.dy) > static_cast<double>(ml.max_ramp_angle_deg)) return {};
    const auto hw = highway_of(*R.way);
    if (hw.size() < 5 || hw.substr(hw.size() - 5) != "_link") return {};
    if (S.counted != T.counted + R.counted) return {};
    // The ramp's side of the carriageway: the left normal of S's travel direction against the ramp's far point.
    const bool ramp_right = R.fx * -S.dy + R.fy * S.dx < 0;
    // The through pair as a join in travel order: upstream = the arriving one.
    const NodeArm& upstream = out.merge ? T : S;
    const NodeArm& downstream = out.merge ? S : T;
    const Side U = make_side(upstream.xs, upstream.cls, upstream.arm.way_id), D = make_side(downstream.xs, downstream.cls, downstream.arm.way_id);
    const JoinOutcome join = resolve_join(U, D, rules);
    if (join.split) return {};
    // The added lanes: the counted bands of the single arm that meet no band of the through arm (join width 0).
    const Side& wide = out.merge ? D : U;
    const SideEval& wide_eval = out.merge ? join.d : join.u;
    int first_added = -1, last_added = -1, added = 0;
    for (int i = wide.first_counted; i <= wide.last_counted; ++i) {
        if (!wide.xs.bands[static_cast<std::size_t>(i)].counted || wide_eval.jw[static_cast<std::size_t>(i)] != 0) continue;
        if (first_added < 0) first_added = i;
        last_added = i;
        ++added;
    }
    if (added != R.counted || last_added - first_added + 1 != added) return {};   // one contiguous group, the ramp's lanes
    // ... on the ramp's side: no matched counted band beyond it.
    for (int i = wide.first_counted; i <= wide.last_counted; ++i) {
        if (!wide.xs.bands[static_cast<std::size_t>(i)].counted || (i >= first_added && i <= last_added)) continue;
        if (ramp_right ? i > first_added : i < last_added) return {};
    }
    // The centre line of the added lanes seen from the single arm's way line, with the join's own correction of that arm.
    std::array<double, 33> edge{};
    if (wide.xs.bands.size() > 32 || !wide.xs.boundaries_at(0.0, std::span<double>(edge.data(), wide.xs.bands.size() + 1))) return {};
    const double centre_m = (edge[static_cast<std::size_t>(first_added)] + edge[static_cast<std::size_t>(last_added) + 1]) / 2;
    const std::int32_t correction = out.merge ? join.corr_d : join.corr_u;
    const double target_mm = std::round(centre_m * 1000.0) + static_cast<double>(correction);
    const double cos_phi = R.dx * S.dx + R.dy * S.dy;
    if (!(cos_phi > 0)) return {};
    const double landing = std::round(target_mm / cos_phi);
    if (std::fabs(landing) > static_cast<double>(ml.max_landing_offset_mm)) return {};
    out.through_id = T.arm.way_id; out.single_id = S.arm.way_id; out.ramp_id = R.arm.way_id;
    out.through_leaves = T.arm.leaves_node; out.single_leaves = S.arm.leaves_node;
    out.landing_mm = static_cast<std::int32_t>(landing);
    const std::int64_t length = std::clamp<std::int64_t>(static_cast<std::int64_t>(ml.landing_m_per_m) * std::abs(out.landing_mm),
        static_cast<std::int64_t>(ml.landing_min_m) * 1000, static_cast<std::int64_t>(ml.landing_max_m) * 1000);
    out.landing_length_mm = static_cast<std::int32_t>(out.landing_mm == 0 ? 0 : length);
    return out;
}

struct EndResult {
    bool present = false;
    std::vector<WidthRamp> ramps;    // per band of A (A's frame)
    JoinRecord record;
    std::int32_t correction = 0;     // lateral shift of A at this end (A's frame, mm)
};

// Scales a length by num/den, flooring, at least 1 mm.
std::int32_t scaled(std::int32_t len, std::int64_t num, std::int64_t den) {
    if (len <= 0) return 0;
    return static_cast<std::int32_t>(std::max<std::int64_t>(1, static_cast<std::int64_t>(len) * num / den));
}
} // namespace

Result<CrossSectionBase> resolve_cross_section(const RoadReferenceBuilder& builder, std::int64_t way_id,
        RoadStretch stretch, double length_m, const RoadRules& rules) {
    const RoadGraph& graph = builder.graph();
    const auto* way = find_way(graph, way_id);
    if (!way) return make_error(ErrorCode::NotFound, "cross section: way not in graph");
    auto base = resolve_cross_section_base(*way, rules);
    if (!base) return base.error();
    if (!base->xs) return base;
    RoadCrossSection xs = *base->xs;
    xs.length_m = length_m;
    const auto* cls_a = rules.find_class(highway_of(*way));
    if (!cls_a) return make_error(ErrorCode::InvalidArgument, "cross section: no rules for the way's highway");
    const std::size_t n = xs.bands.size();

    std::array<EndResult, 2> ends; // 0 = start, 1 = end
    std::array<std::int32_t, 2> landing{0, 0}, landing_length{0, 0};   // the end of a merge ramp (G2M-028)
    std::array<std::int64_t, 2> landing_single{0, 0};                    // ... and the single arm it lands beside
    for (int e = 0; e < 2; ++e) {
        const bool at_end = e == 1;
        auto join = builder.join(way_id, stretch, at_end);
        if (!join) return join.error();
        if (!*join && rules.merge_lane.enabled) {
            // A node of degree three: the through pair of a merge lane / lane drop joins like a degree-two node, the ramp lands.
            const auto node = way->node_ids[at_end ? stretch.end_ref : stretch.start_ref];
            if (const auto merge = classify_merge_node(builder, node, rules)) {
                if (merge->through_id == way_id) join = std::optional<RoadStretchJoin>{RoadStretchJoin{merge->single_id, 0, merge->single_leaves}};
                else if (merge->single_id == way_id) join = std::optional<RoadStretchJoin>{RoadStretchJoin{merge->through_id, 0, merge->through_leaves}};
                else if (merge->ramp_id == way_id) {
                    landing[static_cast<std::size_t>(e)] = merge->landing_mm;
                    landing_length[static_cast<std::size_t>(e)] = merge->landing_length_mm;
                    landing_single[static_cast<std::size_t>(e)] = merge->single_id;
                }
            }
        }
        if (!*join) continue;
        const auto* other = find_way(graph, (*join)->way_id);
        if (!other) continue;
        auto other_base = resolve_cross_section_base(*other, rules);
        if (!other_base || !other_base->xs) continue; // an unsupported or declined neighbour has no section to meet
        const auto* cls_b = rules.find_class(highway_of(*other));
        if (!cls_b) continue;
        const bool same = at_end ? (*join)->leaves_node : !(*join)->leaves_node; // B has A's direction at the node
        const std::int64_t id_a = way_id, id_b = (*join)->way_id;
        const bool owner_a = id_a <= id_b;
        const bool a_mirrored = !owner_a && !same;
        const bool b_mirrored = owner_a && !same;
        Side side_a = make_side(a_mirrored ? mirrored(xs) : xs, cls_a, id_a);
        Side side_b = make_side(b_mirrored ? mirrored(*other_base->xs) : *other_base->xs, cls_b, id_b);
        // Upstream in the canonical frame: the owner's own end is upstream; B arriving at the node (its end) makes B
        // upstream, B leaving (its start) makes A upstream.
        const bool a_is_u = owner_a ? at_end : (*join)->leaves_node;
        const Side& U = a_is_u ? side_a : side_b;
        const Side& D = a_is_u ? side_b : side_a;
        const JoinOutcome out = resolve_join(U, D, rules);
        auto& r = ends[static_cast<std::size_t>(e)];
        r.present = true;
        r.record = {out.kind, id_b, out.delta};
        const SideEval& mine = a_is_u ? out.u : out.d;
        const std::int32_t corr = a_is_u ? out.corr_u : out.corr_d;
        r.correction = a_mirrored ? -corr : corr;
        r.ramps.assign(n, WidthRamp{});
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t ci = a_mirrored ? n - 1 - i : i;
            if (mine.len[ci] > 0) r.ramps[i] = {mine.len[ci], mine.jw[ci]};
        }
    }

    // Offset correction of A (4.4).
    const auto shift_len = [&](std::int32_t mm) {
        const std::int64_t want = static_cast<std::int64_t>(cls_a->taper.shift_m_per_m) * std::abs(mm);
        return static_cast<std::int32_t>(std::clamp<std::int64_t>(want, static_cast<std::int64_t>(cls_a->taper.shift_min_m) * 1000,
                                                                  static_cast<std::int64_t>(cls_a->taper.shift_max_m) * 1000));
    };
    const std::int32_t c0 = ends[0].correction, c1 = ends[1].correction;
    if (c0 != 0 || c1 != 0) {
        if (xs.anchor.source == PlacementSource::Transition) {
            xs.offset = {OffsetMode::Transition, c0, 0, c1, 0};
        } else if ((c0 != 0 && !ends[1].present) || (c1 != 0 && !ends[0].present)) {
            // The other end is a junction or a dead end: shift the whole stretch (c0 and c1 cannot both be set).
            xs.offset = {OffsetMode::Hold, c0 != 0 ? c0 : c1, 0, 0, 0};
        } else {
            xs.offset = {OffsetMode::Ramps, c0, c0 != 0 ? shift_len(c0) : 0, c1, c1 != 0 ? shift_len(c1) : 0};
        }
    }

    // A merge ramp's landing (G2M-028): the end of the link shifts to the centre line of the added lane over its own ramp. It
    // fills a free end of the ramp offsets; a hold or transition offset (a whole stretch shifted) keeps its own.
    for (int e = 0; e < 2; ++e) {
        const auto k = static_cast<std::size_t>(e);
        if (landing[k] == 0 || landing_length[k] <= 0) continue;
        if (xs.offset.mode == OffsetMode::None) xs.offset = {OffsetMode::Ramps, 0, 0, 0, 0};
        if (xs.offset.mode != OffsetMode::Ramps) continue;
        if (e == 0 && xs.offset.start_mm == 0) { xs.offset.start_mm = landing[0]; xs.offset.start_length_mm = landing_length[0]; ends[0].present = true; ends[0].record = {JoinKind::Landing, landing_single[0], landing[0]}; }
        if (e == 1 && xs.offset.end_mm == 0) { xs.offset.end_mm = landing[1]; xs.offset.end_length_mm = landing_length[1]; ends[1].present = true; ends[1].record = {JoinKind::Landing, landing_single[1], landing[1]}; }
    }

    // Band ramps (4.2) and exactness clipping (4.5).
    for (std::size_t i = 0; i < n; ++i) { xs.bands[i].start = ends[0].ramps.empty() ? WidthRamp{} : ends[0].ramps[i];
                                          xs.bands[i].end = ends[1].ramps.empty() ? WidthRamp{} : ends[1].ramps[i]; }
    xs.start_join = ends[0].present ? ends[0].record : JoinRecord{};
    xs.end_join = ends[1].present ? ends[1].record : JoinRecord{};
    std::int64_t a_start = 0, a_end = 0;
    for (const auto& b : xs.bands) { a_start = std::max<std::int64_t>(a_start, b.start.length_mm); a_end = std::max<std::int64_t>(a_end, b.end.length_mm); }
    if (xs.offset.mode == OffsetMode::Ramps) { a_start = std::max<std::int64_t>(a_start, xs.offset.start_length_mm); a_end = std::max<std::int64_t>(a_end, xs.offset.end_length_mm); }
    const auto length_mm = static_cast<std::int64_t>(std::floor(length_m * 1000.0));
    if (a_start > 0 && a_end > 0 && a_start + a_end > length_mm) {
        const std::int64_t den = a_start + a_end;
        for (auto& b : xs.bands) { b.start.length_mm = scaled(b.start.length_mm, length_mm, den); b.end.length_mm = scaled(b.end.length_mm, length_mm, den); }
        xs.offset.start_length_mm = scaled(xs.offset.start_length_mm, length_mm, den);
        xs.offset.end_length_mm = scaled(xs.offset.end_length_mm, length_mm, den);
        xs.flags |= kXsTaperShortenedStart | kXsTaperShortenedEnd;
    } else if (a_start > length_mm) {
        for (auto& b : xs.bands) b.start.length_mm = scaled(b.start.length_mm, length_mm, a_start);
        xs.offset.start_length_mm = scaled(xs.offset.start_length_mm, length_mm, a_start);
        xs.flags |= kXsTaperShortenedStart;
    } else if (a_end > length_mm) {
        for (auto& b : xs.bands) b.end.length_mm = scaled(b.end.length_mm, length_mm, a_end);
        xs.offset.end_length_mm = scaled(xs.offset.end_length_mm, length_mm, a_end);
        xs.flags |= kXsTaperShortenedEnd;
    }
    CrossSectionBase result;
    result.xs = std::move(xs);
    return result;
}

Result<LateralPoint> lateral_point(const RoadReferenceLine& reference, double s, double t) {
    const auto pose = reference.at(s);
    if (!pose) return pose.error();
    double sn, cs;
    det::sincos(pose->heading, sn, cs);
    return LateralPoint{pose->x - t * sn, pose->y + t * cs};
}
} // namespace g2m
