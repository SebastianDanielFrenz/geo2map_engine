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
#include <limits>
#include <optional>
#include <span>

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
    std::vector<int> pu, pd; // diagnostics only (CrossSectionTrace): the plan's band partners
};

std::int32_t floor_div2(std::int32_t v) { return v >= 0 ? v / 2 : -((-v + 1) / 2); }

// added_right (ROADS stage 8a-4): at a lane-add merge/diverge node the ramp's side in travel order. The through arm's lanes then meet
// the single arm's lanes on the side away from the ramp, so the added lanes lie on the ramp's side; it decides every group with a
// lane-count change before the placement residual and the default side, unless its turn tokens name the other side. Absent: the
// join as before.
JoinOutcome resolve_join(const Side& U, const Side& D, const RoadRules& rules, std::optional<bool> added_right = std::nullopt) {
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
            // The ramp-side hint wins over everything but turn tokens that name the other side (tagged data: the node stays refused).
            if (added_right && !(*added_right ? left_ok : right_ok)) { choice_right[g] = *added_right; continue; }
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
    out.pu = plan.pu; out.pd = plan.pd;
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
    // ROADS stage 3b: what the ramp's landed end takes over from the single arm at the node
    bool ramp_right = false;                 // the ramp lies right of the single arm (its outer side)
    std::int32_t added_mm = 0;               // the single arm's added counted lanes, total width
    std::int32_t s_shoulder_mm = 0;          // the single arm's shoulder beyond them at the node (0: none)
    // ROADS stage 6: a taper node - no added lane (added_mm 0); the ramp's one counted lane lands with width 0 on the single arm's
    // outermost counted lane edge and widens over landing_length_mm (the taper length before the clearance landing lengthens it).
    bool taper = false;
    // ROADS stage 6r: what the link's stretch leaves for the taper (mm; -1: not measured - a classification without the room gate)
    std::int64_t taper_avail_mm = -1;
    // ROADS stage 9: the through pair joins with the ramp-side hint (stage 8a-4) - a lane-add node whose hint passed the gap check
    bool hinted = false;
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

MergeNodeReason make_arm(const RoadReferenceBuilder& builder, const RoadRules& rules, const RoadNodeArm& arm, const RoadGraphNode& node, NodeArm& out) {
    const RoadGraph& g = builder.graph();
    out.arm = arm;
    out.way = find_way(g, arm.way_id);
    if (!out.way) return MergeNodeReason::ArmWayMissing;
    out.cls = rules.find_class(highway_of(*out.way));
    if (!out.cls) return MergeNodeReason::ArmNoClass;
    auto base = resolve_cross_section_base(*out.way, rules);
    if (!base || !base->xs) return MergeNodeReason::ArmNoSection;
    out.xs = std::move(*base->xs);
    if (!(out.xs.flags & kXsOneway) || (out.xs.flags & kXsReversedOneway)) return MergeNodeReason::ArmNotOneway;   // a one-way carriageway driven along its refs
    for (const auto& b : out.xs.bands) if (b.counted) out.counted += 1;
    if (out.counted < 1) return MergeNodeReason::ArmNoCountedLane;
    // The point at least direction_probe_m away along the way (the last node if the way ends sooner).
    const double probe = static_cast<double>(rules.merge_lane.direction_probe_m);
    const std::ptrdiff_t step = arm.leaves_node ? 1 : -1;
    std::optional<geom::PointMm> far;
    for (auto k = static_cast<std::ptrdiff_t>(arm.ref) + step; k >= 0 && k < static_cast<std::ptrdiff_t>(out.way->node_ids.size()); k += step) {
        const auto* n = find_node(g, out.way->node_ids[static_cast<std::size_t>(k)]);
        if (!n) return MergeNodeReason::ArmNoDirection;
        if (n->position == node.position) continue;
        far = n->position;
        const double fx = static_cast<double>(far->x - node.position.x) / 1000.0, fy = static_cast<double>(far->y - node.position.y) / 1000.0;
        if (std::sqrt(fx * fx + fy * fy) >= probe) break;
    }
    if (!far) return MergeNodeReason::ArmNoDirection;
    out.fx = static_cast<double>(far->x - node.position.x) / 1000.0;
    out.fy = static_cast<double>(far->y - node.position.y) / 1000.0;
    const double len = std::sqrt(out.fx * out.fx + out.fy * out.fy);
    if (!(len > 0)) return MergeNodeReason::ArmNoDirection;
    out.dx = (arm.leaves_node ? out.fx : -out.fx) / len;
    out.dy = (arm.leaves_node ? out.fy : -out.fy) / len;
    return MergeNodeReason::Ok;
}

double angle_deg(double cos_angle) { return det::acos(std::clamp(cos_angle, -1.0, 1.0)) * 180.0 / det::kPi; }

// ROADS stage 6r: the angle (deg, 0..90) between the line of the link's served reference fit at the node and the single arm's
// travel direction (sx, sy); -1 when the stretch cannot be fitted. The arm direction alone is a chord over direction_probe_m: a
// link whose last metres kink toward the node meets it far steeper than the chord says (one region node: chord 14 deg, fit 60).
double ramp_fit_angle_deg(const RoadReferenceBuilder& builder, const ReferenceFitLimits& fit_limits, const RoadNodeArm& ramp,
        double sx, double sy) {
    const auto stretches = builder.stretches(ramp.way_id);
    if (!stretches) return -1;
    for (const auto& st : *stretches) {
        if (ramp.leaves_node ? st.start_ref != ramp.ref : st.end_ref != ramp.ref) continue;
        const auto fit = builder.fit_stretch(ramp.way_id, st, fit_limits);
        if (!fit) return -1;
        const auto p = fit->at(ramp.leaves_node ? 0.0 : fit->length_m);
        if (!p) return -1;
        double sn, cs; det::sincos(p->heading, sn, cs);
        return angle_deg(std::fabs(cs * sx + sn * sy));
    }
    return -1;
}

// ROADS stage 6r: the room a taper has on the link's stretch at a node (taper_room below).
struct TaperRoom {
    bool fitted = false, two_tapers = false, held = false;
    std::int64_t stretch_mm = 0, far_clear_mm = 0, claim_mm = 0, avail_mm = 0;
};
TaperRoom taper_room(const RoadReferenceBuilder& builder, const RoadRules& rules, const ReferenceFitLimits& fit_limits,
        std::int64_t node_id, const RoadNodeArm& ramp);
// ROADS stage 9: the worst lane-edge gap (mm) of the ramp's landing at the node, its stretch resolved with the room gates and the
// hint but no hint check (no recursion); nullopt when the end does not land on a Ramps offset or the stretch does not fit.
// ROADS stage 10r: a flag on `room` for that nested resolve: its merge nodes take geometry's roles (the geometric fallback's own
// hint check must measure the geometric landing); classify_merge_node strips it.
constexpr int kRoomGeomRoles = 8;
std::optional<std::int32_t> hint_landing_gap_mm(const RoadReferenceBuilder& builder, const RoadRules& rules, const ReferenceFitLimits& fit_limits,
        const RoadNodeArm& ramp, bool geom_roles = false);
// Two joins of the through pair are the same join (widths, ramps, kind, deltas, corrections).
bool same_join(const JoinOutcome& a, const JoinOutcome& b) {
    return a.kind == b.kind && a.delta == b.delta && a.corr_u == b.corr_u && a.corr_d == b.corr_d && a.split == b.split &&
           a.u.jw == b.u.jw && a.u.len == b.u.len && a.d.jw == b.d.jw && a.d.len == b.d.len;
}

// The served planar reference fit (curvature exceptions on, the rules' chord bound), as the served tiles fit.
ReferenceFitLimits served_fit_limits(const RoadRules& rules) {
    ReferenceFitLimits served;
    served.allow_curvature_exceptions = true;
    served.chord_half_width_factor = static_cast<double>(rules.reference.chord_half_width_percent) / 100.0;
    served.chord_margin_m = static_cast<double>(rules.reference.chord_margin_mm) / 1000.0;
    return served;
}
// The fitted reference's pose of an arm's stretch at the node (ROADS stage 3c-1); nullopt when the stretch does not fit.
std::optional<geom::CurvePose> arm_pose(const RoadReferenceBuilder& builder, const NodeArm& a, const ReferenceFitLimits& limits) {
    const auto stretches = builder.stretches(a.arm.way_id); if (!stretches) return std::nullopt;
    for (const auto& st : *stretches) {
        if (a.arm.leaves_node ? st.start_ref != a.arm.ref : st.end_ref != a.arm.ref) continue;
        const auto f = builder.fit_stretch(a.arm.way_id, st, limits); if (!f) return std::nullopt;
        const auto pose = f->at(a.arm.leaves_node ? 0.0 : f->length_m); if (!pose) return std::nullopt;
        return *pose;
    }
    return std::nullopt;
}

// ROADS stage 10: which of the candidate arms X and Y each role signal names the through arm: 1 = X, 2 = Y, 0 = the signal does
// not decide. Turn: at a fork S's tagged counted lanes - lanes with through/none and lanes whose only tokens turn to one side
// (left/slight/sharp, all the same side): the arm on that side is the ramp; at a merge the arm whose tagged lanes all carry only
// merge_to tokens is the ramp when the other's do not. Ref / name: the one arm sharing a ';'-separated value with S.
struct RoleSignals { int turn = 0, ref = 0, name = 0; };

std::string_view way_tag(const RoadGraphWay& w, std::string_view key) {
    for (const auto& [k, v] : w.tags) if (k == key) return v;
    return {};
}

bool shares_value(std::string_view a, std::string_view b) {
    const auto trim_sv = [](std::string_view t) {
        while (!t.empty() && t.front() == ' ') t.remove_prefix(1);
        while (!t.empty() && t.back() == ' ') t.remove_suffix(1);
        return t;
    };
    for (std::size_t i = 0; i <= a.size();) {
        const auto j = std::min(a.find(';', i), a.size());
        const auto ta = trim_sv(a.substr(i, j - i));
        if (!ta.empty())
            for (std::size_t k = 0; k <= b.size();) {
                const auto l = std::min(b.find(';', k), b.size());
                if (trim_sv(b.substr(k, l - k)) == ta) return true;
                k = l + 1;
            }
        i = j + 1;
    }
    return false;
}

RoleSignals fork_role_signals(const NodeArm& S, const NodeArm& X, const NodeArm& Y, bool merge) {
    RoleSignals r;
    const double lx = X.fx * -S.dy + X.fy * S.dx, ly = Y.fx * -S.dy + Y.fy * S.dx;   // left of S's travel
    const auto tagged = [](const CrossSectionBand& b) { return b.counted && b.provenance.turn == Provenance::Tagged && b.turn != 0; };
    if (!merge) {
        constexpr std::uint16_t kL = kTurnLeft | kTurnSlightLeft | kTurnSharpLeft, kR = kTurnRight | kTurnSlightRight | kTurnSharpRight;
        bool thr = false, tl = false, tr = false;
        for (const auto& b : S.xs.bands) {
            if (!tagged(b)) continue;
            if (b.turn & (kTurnThrough | kTurnNone)) thr = true;
            else if ((b.turn & kL) && !(b.turn & kR)) tl = true;
            else if ((b.turn & kR) && !(b.turn & kL)) tr = true;
        }
        if (thr && tl != tr && lx != ly) {
            const bool x_on_ramp_side = tl ? lx > ly : lx < ly;
            r.turn = x_on_ramp_side ? 2 : 1;
        }
    } else {
        const auto merging = [&](const NodeArm& n) {
            bool any = false;
            for (const auto& b : n.xs.bands) {
                if (!tagged(b)) continue;
                if (!(b.turn & (kTurnMergeToLeft | kTurnMergeToRight)) || (b.turn & (kTurnThrough | kTurnNone))) return false;
                any = true;
            }
            return any;
        };
        const bool mx = merging(X), my = merging(Y);
        if (mx != my) r.turn = mx ? 2 : 1;
    }
    const auto continuity = [&](std::string_view key) {
        const auto sv = way_tag(*S.way, key);
        if (sv.empty()) return 0;
        const bool sx = shares_value(sv, way_tag(*X.way, key)), sy = shares_value(sv, way_tag(*Y.way, key));
        return sx == sy ? 0 : (sx ? 1 : 2);
    };
    r.ref = continuity("ref");
    r.name = continuity("name");
    return r;
}

std::optional<MergeNode> classify_merge_node(const RoadReferenceBuilder& builder, std::int64_t node_id, const RoadRules& rules,
        const ReferenceFitLimits& fit_limits, MergeNodeReason* why = nullptr, MergeNodeDiagnosis* detail = nullptr, int room = 2,
        bool hint = true, int roles = 2) {
    // Every refusal names its reason (diagnostics only, g2m_xs_probe --merge-census). A node a way passes through keeps the reason
    // way_passes_through for every refusal but the taper ones (ROADS stage 6/6r), so the census still tells those nodes apart.
    // `room` false: the taper gates of ROADS stage 6r that need the link's stretch (room, held offset) are skipped and an accepted
    // taper carries taper_avail_mm -1 - only for the classifications inside taper_room's own resolve, which must not recurse.
    // `room` 2 (ROADS stage 9) also checks the ramp-side hint's landing gap; 1 keeps the hint unchecked (the check's own resolve).
    // `hint` false: the through pair joins without the ramp-side hint (the fallback when the check fails).
    // `roles` (ROADS stage 10r): 2 the role-tag rule applies and, at room 2, a tag role differing from geometry's is checked; 1 the
    // tag role applies unchecked (the check's own resolve); 0 geometry's roles (the fallback when the check fails).
    if (room & kRoomGeomRoles) { roles = 0; room &= ~kRoomGeomRoles; }
    bool passing = false;
    const auto refuse = [why, &passing](MergeNodeReason r) -> std::optional<MergeNode> {
        const bool taper_reason = r == MergeNodeReason::TaperRampLanes || r == MergeNodeReason::TaperAngle ||
                                  r == MergeNodeReason::TaperNoRoom || r == MergeNodeReason::TaperOffsetHeld ||
                                  r == MergeNodeReason::TaperSide;
        if (why) *why = passing && !taper_reason ? MergeNodeReason::WayPassesThrough : r;
        return std::nullopt;
    };
    if (why) *why = MergeNodeReason::Ok;
    const MergeLaneRules& ml = rules.merge_lane;
    if (!ml.enabled) return refuse(MergeNodeReason::Disabled);
    auto arms = builder.arms(node_id);
    if (arms.size() != 3) return refuse(MergeNodeReason::ArmsNot3);
    std::sort(arms.begin(), arms.end(), [](const RoadNodeArm& a, const RoadNodeArm& b) { return a.way_id != b.way_id ? a.way_id < b.way_id : a.ref < b.ref; });
    // A way passing through the node gives two arms (ROADS stage 6): it can only be the single and the through arm of a taper node.
    if (arms[0].way_id == arms[1].way_id && arms[1].way_id == arms[2].way_id) return refuse(MergeNodeReason::WayPassesThrough);
    passing = arms[0].way_id == arms[1].way_id || arms[1].way_id == arms[2].way_id;
    if (passing && ml.taper_length_m <= 0) return refuse(MergeNodeReason::WayPassesThrough);   // no taper rule: as before
    const RoadGraphNode* node = find_node(builder.graph(), node_id);
    if (!node) return refuse(MergeNodeReason::NodeMissing);
    std::array<NodeArm, 3> a;
    for (std::size_t i = 0; i < 3; ++i)
        if (const auto r = make_arm(builder, rules, arms[i], *node, a[i]); r != MergeNodeReason::Ok) return refuse(r);
    const int leaving = static_cast<int>(a[0].arm.leaves_node) + static_cast<int>(a[1].arm.leaves_node) + static_cast<int>(a[2].arm.leaves_node);
    if (leaving != 1 && leaving != 2) return refuse(MergeNodeReason::AllSameDirection);
    MergeNode out;
    out.merge = leaving == 1;                  // one arm leaves: two roads merge
    // The single arm is the one alone in its direction.
    std::size_t si = 0;
    for (std::size_t i = 0; i < 3; ++i) if (a[i].arm.leaves_node == out.merge) si = i;
    const std::size_t xi = (si + 1) % 3, yi = (si + 2) % 3;
    const NodeArm& S = a[si];
    const double cos_x = a[xi].dx * S.dx + a[xi].dy * S.dy, cos_y = a[yi].dx * S.dx + a[yi].dy * S.dy;
    // Passing through: the single arm is one of the passing way's arms (one leaves, one arrives) and its other arm is the through one.
    if (passing && a[xi].arm.way_id != S.arm.way_id && a[yi].arm.way_id != S.arm.way_id) return refuse(MergeNodeReason::WayPassesThrough);
    bool x_through = passing ? a[xi].arm.way_id == S.arm.way_id
                             : cos_x > cos_y || (cos_x == cos_y && a[xi].arm.way_id < a[yi].arm.way_id);
    // ROADS stage 7b: a fork of two non-link carriageways (neither arm a _link) has no ramp by class; the through arm is the one
    // bending less, and where both bend alike (deflections within merge_lane.fork_tie_deg) the one with more counted lanes,
    // then the left one in S's travel direction. Nodes with a link arm keep the choice above.
    const auto is_link_arm = [](const NodeArm& n) { const auto h = highway_of(*n.way); return h.size() >= 5 && h.substr(h.size() - 5) == "_link"; };
    if (!passing && !is_link_arm(a[xi]) && !is_link_arm(a[yi]) &&
        std::fabs(angle_deg(cos_x) - angle_deg(cos_y)) <= static_cast<double>(ml.fork_tie_deg)) {
        if (a[xi].counted != a[yi].counted) x_through = a[xi].counted > a[yi].counted;
        else {
            const double lx = a[xi].fx * -S.dy + a[xi].fy * S.dx, ly = a[yi].fx * -S.dy + a[yi].fy * S.dx;   // left of S's travel
            if (lx != ly) x_through = lx > ly;
        }
    }
    // ROADS stage 10/10r: where both candidate arms stay within role_tag_max_deflection_deg of S, the roles follow the turn tokens
    // (forks and merges), else at a fork ref continuity, where it decides; at a merge ref decides nothing. Ref never makes a _link
    // arm the through arm when the other arm is no link. Tokens and ref naming different arms (forks) leave the geometry's choice.
    const bool geom_x_through = x_through;
    const RoleSignals sig = fork_role_signals(S, a[xi], a[yi], out.merge);
    const int ref_use = out.merge || (sig.ref != 0 && is_link_arm(sig.ref == 1 ? a[xi] : a[yi]) && !is_link_arm(sig.ref == 1 ? a[yi] : a[xi]))
                            ? 0 : sig.ref;
    const bool role_conflict = sig.turn != 0 && ref_use != 0 && sig.turn != ref_use;
    bool role_rule = false;
    if (roles != 0 && !passing && ml.role_tag_max_deflection_deg > 0 &&
        angle_deg(cos_x) <= static_cast<double>(ml.role_tag_max_deflection_deg) &&
        angle_deg(cos_y) <= static_cast<double>(ml.role_tag_max_deflection_deg) && !role_conflict) {
        if (const int pick = sig.turn != 0 ? sig.turn : ref_use; pick != 0) { x_through = pick == 1; role_rule = true; }
    }
    // ROADS stage 10r: a tag role that differs from geometry's is kept only if the node, resolved with it, lands and its ramp's
    // landing keeps a worst lane gap of at least -accept_min_gap_mm (as the stage 9 hint check: the ramp's stretch resolved with
    // the room gates and unchecked roles and hint; an unknown gap keeps it); else the node is exactly the geometric roles' result.
    // One level: the two resolves below run with roles 1 and 0. A function of the node alone, so no landed node is lost by it.
    if (roles == 2 && room >= 2 && role_rule && x_through != geom_x_through) {
        MergeNodeReason tag_why = MergeNodeReason::Ok;
        MergeNodeDiagnosis tag_detail;
        auto tag = classify_merge_node(builder, node_id, rules, fit_limits, &tag_why, detail ? &tag_detail : nullptr, room, hint, 1);
        std::optional<std::int32_t> g;
        if (tag) g = hint_landing_gap_mm(builder, rules, fit_limits, (x_through ? a[yi] : a[xi]).arm);
        if (tag && (!g || ml.accept_min_gap_mm <= 0 || *g >= -ml.accept_min_gap_mm)) {
            if (why) *why = tag_why;
            if (detail) { *detail = tag_detail; detail->role_checked = true; detail->role_gap_known = g.has_value(); detail->role_gap_mm = g.value_or(0); }
            return tag;
        }
        if (detail) *detail = MergeNodeDiagnosis{};
        auto r = classify_merge_node(builder, node_id, rules, fit_limits, why, detail, room, hint, 0);
        if (detail) {
            detail->role_checked = true; detail->role_dropped = true; detail->role_gap_known = g.has_value(); detail->role_gap_mm = g.value_or(0);
            detail->role_turn_through_id = sig.turn == 1 ? a[xi].arm.way_id : sig.turn == 2 ? a[yi].arm.way_id : 0;
            detail->role_ref_through_id = sig.ref == 1 ? a[xi].arm.way_id : sig.ref == 2 ? a[yi].arm.way_id : 0;
            detail->role_name_through_id = sig.name == 1 ? a[xi].arm.way_id : sig.name == 2 ? a[yi].arm.way_id : 0;
        }
        return r;
    }
    const NodeArm& T = x_through ? a[xi] : a[yi];
    const NodeArm& R = x_through ? a[yi] : a[xi];
    if (detail) {   // diagnostics only (ROADS stage 10)
        const auto id_of = [&](int k) { return k == 1 ? a[xi].arm.way_id : k == 2 ? a[yi].arm.way_id : std::int64_t{0}; };
        detail->role_x_id = a[xi].arm.way_id; detail->role_y_id = a[yi].arm.way_id;
        detail->role_x_defl_deg = angle_deg(cos_x); detail->role_y_defl_deg = angle_deg(cos_y);
        detail->role_geom_through_id = id_of(geom_x_through ? 1 : 2);
        detail->role_turn_through_id = id_of(sig.turn); detail->role_ref_through_id = id_of(sig.ref);
        detail->role_name_through_id = id_of(sig.name);
        detail->role_rule_applied = role_rule; detail->role_conflict = role_conflict;
    }
    if (detail) {   // diagnostics only (ROADS stage 5a)
        detail->roles = true; detail->merge = out.merge;
        detail->through_id = T.arm.way_id; detail->single_id = S.arm.way_id; detail->ramp_id = R.arm.way_id;
        detail->single_counted = S.counted; detail->through_counted = T.counted; detail->ramp_counted = R.counted;
        detail->through_deflection_deg = angle_deg(x_through ? cos_x : cos_y);
        detail->ramp_angle_deg = angle_deg(R.dx * S.dx + R.dy * S.dy);
    }
    if (angle_deg(x_through ? cos_x : cos_y) > static_cast<double>(ml.max_through_deflection_deg)) return refuse(MergeNodeReason::ThroughDeflection);
    if (angle_deg(R.dx * S.dx + R.dy * S.dy) > static_cast<double>(ml.max_ramp_angle_deg)) return refuse(MergeNodeReason::RampAngle);
    // ROADS stage 7b: a non-link arm is a merge/diverge arm only where the lanes balance (S = T + R) - a carriageway splitting into,
    // or joining from, two carriageways; it gets the lane-add landing, gore and markings of a link, never a taper.
    const bool ramp_link = is_link_arm(R);
    if (!ramp_link) {
        const auto listed = [&](const NodeArm& n) {
            if (is_link_arm(n)) return true;
            const auto h = highway_of(*n.way);
            return std::find(ml.non_link_arm_classes.begin(), ml.non_link_arm_classes.end(), h) != ml.non_link_arm_classes.end();
        };
        if (!listed(S) || !listed(T) || !listed(R) || S.counted != T.counted + R.counted) return refuse(MergeNodeReason::RampNotLink);
    }
    // A taper node (ROADS stage 6): the carriageway keeps its lanes across the node and a one-lane link lands on its outer lane edge.
    const bool taper = ramp_link && ml.taper_length_m > 0 && S.counted == T.counted;
    if (taper && R.counted != 1) return refuse(MergeNodeReason::TaperRampLanes);
    // ROADS stage 6r: a short or steep link meeting the carriageway is a junction, not a taper - the angle gate here, the room gate
    // (and the held offset) once the landing is known. The census measures the room also where the angle refuses.
    TaperRoom room_at;
    bool room_done = false;
    const auto measure_room = [&] {
        room_at = taper_room(builder, rules, fit_limits, node_id, R.arm);
        room_done = true;
        if (detail) {
            detail->room_known = room_at.fitted; detail->two_tapers = room_at.two_tapers; detail->offset_held = room_at.held;
            detail->stretch_mm = room_at.stretch_mm; detail->far_clear_mm = room_at.far_clear_mm;
            detail->claim_mm = room_at.claim_mm; detail->avail_mm = room_at.avail_mm;
        }
    };
    if (taper) {
        if (detail) { detail->taper_candidate = true; if (room) measure_room(); }
        const double fa = ramp_fit_angle_deg(builder, fit_limits, R.arm, S.dx, S.dy);
        if (detail) detail->ramp_fit_angle_deg = fa;
        if (ml.taper_max_angle_deg > 0 && std::max(angle_deg(R.dx * S.dx + R.dy * S.dy), fa) > static_cast<double>(ml.taper_max_angle_deg))
            return refuse(MergeNodeReason::TaperAngle);
    }
    if (!taper && S.counted != T.counted + R.counted) return refuse(MergeNodeReason::LaneSum);
    // The ramp's side of the carriageway: the left normal of S's travel direction against the ramp's far point.
    const bool ramp_right = R.fx * -S.dy + R.fy * S.dx < 0;
    // ROADS stage 6r: a taper lands the link on one edge of a carriageway that runs on across the node; where that carriageway
    // bends at the node and the link lies between its two lines (right of one arm, left of the other), either edge puts the
    // link's landing across the through lanes. Such a node stays a junction.
    if (taper && (R.fx * -T.dy + R.fy * T.dx < 0) != ramp_right) return refuse(MergeNodeReason::TaperSide);
    // The through pair as a join in travel order: upstream = the arriving one.
    const NodeArm& upstream = out.merge ? T : S;
    const NodeArm& downstream = out.merge ? S : T;
    const Side U = make_side(upstream.xs, upstream.cls, upstream.arm.way_id), D = make_side(downstream.xs, downstream.cls, downstream.arm.way_id);
    const bool use_hint = !taper && hint;
    const JoinOutcome join = resolve_join(U, D, rules, use_hint ? std::optional<bool>(ramp_right) : std::nullopt);
    if (join.split) return refuse(MergeNodeReason::JoinSplit);
    // The added lanes: the counted bands of the single arm that meet no band of the through arm (join width 0).
    const Side& wide = out.merge ? D : U;
    const SideEval& wide_eval = out.merge ? join.d : join.u;
    int first_added = -1, last_added = -1, added = 0;
    if (!taper) {
        for (int i = wide.first_counted; i <= wide.last_counted; ++i) {
            if (!wide.xs.bands[static_cast<std::size_t>(i)].counted || wide_eval.jw[static_cast<std::size_t>(i)] != 0) continue;
            if (first_added < 0) first_added = i;
            last_added = i;
            ++added;
        }
        if (detail) detail->added_count = added;
        if (added != R.counted) return refuse(MergeNodeReason::AddedCountMismatch);
        if (last_added - first_added + 1 != added) return refuse(MergeNodeReason::AddedNotContiguous);   // one contiguous group, the ramp's lanes
        // ... on the ramp's side: no matched counted band beyond it.
        for (int i = wide.first_counted; i <= wide.last_counted; ++i) {
            if (!wide.xs.bands[static_cast<std::size_t>(i)].counted || (i >= first_added && i <= last_added)) continue;
            if (ramp_right ? i > first_added : i < last_added) return refuse(MergeNodeReason::AddedNotOnRampSide);
        }
    } else if (detail) {
        detail->added_count = 0;
    }
    // The centre line of the added lanes seen from the single arm's way line, with the join's own correction of that arm. The
    // single arm's bands take their widths AT the node (a matched band's join width; the added lanes stay full width, G2M-032), so
    // the landed lanes meet the added lanes' edges as the single arm serves them there (ROADS stage 3b).
    RoadCrossSection at_node = wide.xs;
    for (std::size_t i = 0; i < at_node.bands.size(); ++i) {
        const int ii = static_cast<int>(i);
        if (wide_eval.len[i] > 0 && !(ii >= first_added && ii <= last_added)) at_node.bands[i].width_mm = wide_eval.jw[i];
    }
    std::array<double, 33> edge{};
    if (at_node.bands.size() > 32 || !at_node.boundaries_at(0.0, std::span<double>(edge.data(), at_node.bands.size() + 1))) return refuse(MergeNodeReason::Boundaries);
    // The target: the added lanes' centre line, or at a taper node the single arm's outermost counted lane edge on the ramp's side
    // (S and T join at the node, so it is T's edge there too).
    const int edge_lo = taper ? (ramp_right ? wide.last_counted + 1 : wide.first_counted) : first_added;
    const int edge_hi = taper ? edge_lo - 1 : last_added;
    const double centre_m = (edge[static_cast<std::size_t>(edge_lo)] + edge[static_cast<std::size_t>(edge_hi) + 1]) / 2;
    const std::int32_t correction = out.merge ? join.corr_d : join.corr_u;
    const double target_mm = std::round(centre_m * 1000.0) + static_cast<double>(correction);
    // The landing is R's lateral offset in R's own frame that puts its counted-lane centre on the target in S's frame at the node.
    // Both frames are the fitted references' poses at the node (ROADS stage 3c-1): R's offset reaches S's frame scaled by cos phi of
    // the two headings, and R's reference point lies `lat` left of S's. A stretch that does not fit falls back to the chords
    // (direction_probe_m) and coincident reference points.
    double cos_phi = R.dx * S.dx + R.dy * S.dy, lat_mm = 0;
    if (const auto ps = arm_pose(builder, S, fit_limits), pr = ps ? arm_pose(builder, R, fit_limits) : std::nullopt; ps && pr) {
        double ss, cs, sr, cr; det::sincos(ps->heading, ss, cs); det::sincos(pr->heading, sr, cr);
        cos_phi = cr * cs + sr * ss;
        lat_mm = (-(pr->x - ps->x) * ss + (pr->y - ps->y) * cs) * 1000.0;
    }
    if (!(cos_phi > 0)) return refuse(MergeNodeReason::RampBackward);
    const double landing = std::round((target_mm - lat_mm) / cos_phi);
    if (detail && std::fabs(landing) < 2.0e9) { detail->landing_known = true; detail->landing_needed_mm = static_cast<std::int32_t>(landing); }
    if (std::fabs(landing) > static_cast<double>(ml.max_landing_offset_mm)) return refuse(MergeNodeReason::LandingTooFar);
    out.through_id = T.arm.way_id; out.single_id = S.arm.way_id; out.ramp_id = R.arm.way_id;
    out.through_leaves = T.arm.leaves_node; out.single_leaves = S.arm.leaves_node;
    out.landing_mm = static_cast<std::int32_t>(landing);
    out.ramp_right = ramp_right;
    out.taper = taper;
    for (int i = first_added; i >= 0 && i <= last_added; ++i) out.added_mm += wide.xs.bands[static_cast<std::size_t>(i)].width_mm;
    const int outer_lo = taper ? wide.first_counted : first_added, outer_hi = taper ? wide.last_counted : last_added;
    if (const int sh = ramp_right ? outer_hi + 1 : outer_lo - 1; sh >= 0 && sh < static_cast<int>(wide.xs.bands.size())) {
        const auto k = static_cast<std::size_t>(sh);
        if (wide.xs.bands[k].kind == LaneKind::Shoulder) out.s_shoulder_mm = wide_eval.len[k] > 0 ? wide_eval.jw[k] : wide.xs.bands[k].width_mm;
    }
    const std::int64_t length = std::clamp<std::int64_t>(static_cast<std::int64_t>(ml.landing_m_per_m) * std::abs(out.landing_mm),
        static_cast<std::int64_t>(ml.landing_min_m) * 1000, static_cast<std::int64_t>(ml.landing_max_m) * 1000);
    // A taper always ramps its lane width from 0, so it has a length even with no lateral offset to land.
    out.landing_length_mm = static_cast<std::int32_t>(taper ? static_cast<std::int64_t>(ml.taper_length_m) * 1000 : out.landing_mm == 0 ? 0 : length);
    // ROADS stage 8a-3: a link class's landing S-ramp is checked against its min_edge_radius_m once, by the loader; a non-link ramp
    // (stage 7b) is checked here, per node, with its own class radius: 4 W / a^2 <= 1 / R, W the landing offset, a the slope-derived
    // landing length (in mm: 4 * W_mm * R_m * 1000 <= a_mm^2).
    if (!ramp_link && out.landing_mm != 0) {
        const auto* cls = rules.find_class(highway_of(*R.way));
        const std::int64_t radius_m = cls ? cls->min_edge_radius_m : 0;
        if (4 * static_cast<std::int64_t>(std::abs(out.landing_mm)) * radius_m * 1000 > length * length)
            return refuse(MergeNodeReason::LandingCurvature);
    }
    if (taper && room) {
        if (!room_done) measure_room();
        if (!room_at.fitted || (ml.taper_min_length_m > 0 && room_at.avail_mm < static_cast<std::int64_t>(ml.taper_min_length_m) * 1000))
            return refuse(MergeNodeReason::TaperNoRoom);
        if (room_at.held) return refuse(MergeNodeReason::TaperOffsetHeld);
        out.taper_avail_mm = room_at.avail_mm;
    }
    // ROADS stage 9: where the ramp-side hint changed the through pair's join, it is kept only if the ramp's landed lanes then keep
    // a gap of at least -accept_min_gap_mm to the through lanes over the landing; else the node is what it is without the hint (landed
    // or refused). A function of the node alone: every arm's resolve, in any tile, reaches the same verdict.
    if (use_hint) {
        out.hinted = true;
        if (room >= 2 && ml.accept_min_gap_mm > 0 && !same_join(join, resolve_join(U, D, rules))) {
            const auto g = hint_landing_gap_mm(builder, rules, fit_limits, R.arm, roles == 0);
            if (detail) { detail->hint_checked = true; detail->hint_gap_known = g.has_value(); detail->hint_gap_mm = g.value_or(0); }
            if (g && *g < -ml.accept_min_gap_mm) {
                if (detail) *detail = MergeNodeDiagnosis{};
                auto r = classify_merge_node(builder, node_id, rules, fit_limits, why, detail, room, false, roles);
                if (detail) { detail->hint_checked = true; detail->hint_gap_known = true; detail->hint_gap_mm = *g; detail->hint_dropped = true; }
                return r;
            }
        }
    }
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

const char* merge_node_reason_name(MergeNodeReason reason) {
    static const char* const n[] = {"ok", "disabled", "arms_not_3", "way_passes_through", "node_missing",
        "arm_way_missing", "arm_no_class", "arm_no_section", "arm_not_oneway", "arm_no_counted_lane", "arm_no_direction",
        "all_same_direction", "through_deflection", "ramp_angle", "ramp_not_link", "lane_sum", "join_split",
        "added_count_mismatch", "added_not_contiguous", "added_not_on_ramp_side", "boundaries", "ramp_backward", "landing_too_far", "taper_ramp_lanes",
        "taper_angle", "taper_no_room", "taper_offset_held", "taper_side", "landing_curvature"};
    const auto i = static_cast<std::size_t>(reason);
    return i < sizeof(n) / sizeof(n[0]) ? n[i] : "?";
}

const char* cross_section_end_note_name(CrossSectionEndNote note) {
    static const char* const n[] = {"no_join", "joined", "neighbour_missing", "neighbour_no_section", "neighbour_no_class",
        "landed", "landing_zero", "landing_offset_hold", "landing_offset_transition", "landing_end_taken"};
    const auto i = static_cast<std::size_t>(note);
    return i < sizeof(n) / sizeof(n[0]) ? n[i] : "?";
}

MergeNodeDiagnosis diagnose_merge_node(const RoadReferenceBuilder& builder, std::int64_t node_id, const RoadRules& rules) {
    MergeNodeDiagnosis d;
    if (const auto m = classify_merge_node(builder, node_id, rules, served_fit_limits(rules), &d.reason, &d)) {
        d.merge = m->merge; d.through_id = m->through_id; d.single_id = m->single_id; d.ramp_id = m->ramp_id;
        d.landing_mm = m->landing_mm; d.landing_length_mm = m->landing_length_mm; d.taper = m->taper;
    }
    return d;
}

namespace {
// ROADS stage 3c-3: an estimate of the junction trim the far node of a landed ramp will give the ramp's approach, from the
// node's arms alone (resolve time, any tile: the arms and their ways are in every builder that holds the node). Each arm's
// outward direction is the chord to the way point at least direction_probe_m away, its edge offsets the base cross-section's
// carve footprint at the node; the ramp's arm L meets its angular neighbours: the counter-clockwise one M at angle theta with
// L's outward-left edge against M's outward-right edge, the clockwise one mirrored, the junction builder's corner pairs. The
// sharp corner on the outer layer is at station (b + a cos theta) / sin theta along L (a, b the two edges' offsets); a kerb
// return (the builder's fillet rule: the lower class corner radius, none below corner_min_wedge_deg, a one-way pair's nose
// radius below corner_oneway_pair_max_wedge_deg, none between two roundabout ways, each layer concentric) reaches its tangent
// point r / tan(theta / 2) (capped at corner_tangent_max_percent of r) beyond that layer's corner. The estimate is the
// farthest of them, at least trim_min, plus trim_margin; nullopt when the node or an arm cannot be read. The builder's
// bow-tie rounds and clusters are not modelled (the caller's margin and cap take them).
std::optional<double> far_trim_estimate_m(const RoadReferenceBuilder& builder, const RoadRules& rules, std::int64_t node_id,
        std::int64_t way_id, std::uint32_t ref) {
    const RoadGraph& g = builder.graph();
    const RoadGraphNode* node = find_node(g, node_id);
    if (!node) return std::nullopt;
    struct Arm { double ang = 0; std::array<double, 3> left{}, right{}; std::int32_t radius_mm = 0; bool oneway = false, roundabout = false, self = false; };
    std::vector<Arm> arms;
    const double probe = static_cast<double>(rules.merge_lane.direction_probe_m);
    bool found_self = false;
    for (const auto& arm : builder.arms(node_id)) {
        const RoadGraphWay* w = find_way(g, arm.way_id);
        if (!w) return std::nullopt;
        auto base = resolve_cross_section_base(*w, rules);
        if (!base || !base->xs) return std::nullopt;
        std::optional<geom::PointMm> far;
        const std::ptrdiff_t step = arm.leaves_node ? 1 : -1;
        for (auto k = static_cast<std::ptrdiff_t>(arm.ref) + step; k >= 0 && k < static_cast<std::ptrdiff_t>(w->node_ids.size()); k += step) {
            const auto* n = find_node(g, w->node_ids[static_cast<std::size_t>(k)]);
            if (!n) return std::nullopt;
            if (n->position == node->position) continue;
            far = n->position;
            const double fx = static_cast<double>(far->x - node->position.x) / 1000.0, fy = static_cast<double>(far->y - node->position.y) / 1000.0;
            if (std::sqrt(fx * fx + fy * fy) >= probe) break;
        }
        if (!far) return std::nullopt;
        Arm a;
        a.ang = det::atan2(static_cast<double>(far->y - node->position.y), static_cast<double>(far->x - node->position.x));
        const CarveFootprint f = carve_footprint(*base->xs, 0.0);
        // outward-left / outward-right offsets per layer (0 road bed, 1 paved, 2 outer); an arm entering the node is mirrored
        const std::array<double, 3> l{f.roadbed_left, f.paved_left, f.outer_left}, r{-f.roadbed_right, -f.paved_right, -f.outer_right};
        a.left = arm.leaves_node ? l : r;
        a.right = arm.leaves_node ? r : l;
        const auto hw = highway_of(*w);
        a.radius_mm = rules.junction.corner_radius_for(hw);
        std::string_view ow, jn;
        for (const auto& t : w->tags) { if (t.first == "oneway") ow = t.second; else if (t.first == "junction") jn = t.second; }
        a.roundabout = jn == "roundabout" || jn == "circular";
        a.oneway = ow == "yes" || ow == "1" || ow == "true" || ow == "-1" || (ow != "no" && (a.roundabout || hw == "motorway"));
        a.self = !found_self && arm.way_id == way_id && arm.ref == ref;
        found_self = found_self || a.self;
        arms.push_back(a);
    }
    if (!found_self) return std::nullopt;
    const Arm* L = nullptr;
    for (const auto& a : arms) if (a.self) L = &a;
    const double two_pi = 2.0 * det::kPi;
    const Arm* ccw = nullptr; const Arm* cw = nullptr; double t_ccw = two_pi, t_cw = two_pi;
    for (const auto& a : arms) {
        if (a.self) continue;
        double d = a.ang - L->ang;
        while (d <= 0) d += two_pi;
        while (d > two_pi) d -= two_pi;
        if (d < t_ccw) { t_ccw = d; ccw = &a; }
        if (two_pi - d < t_cw) { t_cw = two_pi - d; cw = &a; }
    }
    const auto& jr = rules.junction;
    double est = static_cast<double>(jr.trim_min_mm) / 1000.0;
    // one corner pair: L's edge offsets `la` on the facing side, M's `mb` on its facing side, at angle theta
    const auto corner = [&](const std::array<double, 3>& la, const std::array<double, 3>& mb, const Arm& M, double theta) {
        const double sn = det::sin(theta), cs = det::cos(theta);
        if (!(sn > 1e-6)) return;
        const auto station = [&](std::size_t layer) { return (mb[layer] + la[layer] * cs) / sn; };
        est = std::max(est, station(2));
        const double wedge_deg = theta * 180.0 / det::kPi;
        if (wedge_deg < static_cast<double>(jr.corner_min_wedge_deg) || (L->roundabout && M.roundabout)) return;
        double r = static_cast<double>(std::min(L->radius_mm, M.radius_mm));
        if (L->oneway && M.oneway && wedge_deg < static_cast<double>(jr.corner_oneway_pair_max_wedge_deg))
            r = std::min(r, static_cast<double>(jr.corner_oneway_pair_radius_mm));
        if (r < 250.0) return;
        const double tan_half = det::tan(theta / 2.0), cap = static_cast<double>(jr.corner_tangent_max_percent) / 100.0;
        for (std::size_t layer = 0; layer < 3; ++layer) {
            if (layer > 0) {
                r = std::max(r - std::min(std::abs(la[layer] - la[layer - 1]), std::abs(mb[layer] - mb[layer - 1])) * 1000.0, 0.0);
                if (r > 0 && r < 250.0) r = 250.0;
                if (r <= 0) break;
            }
            est = std::max(est, station(layer) + std::min(r / tan_half, cap * r) / 1000.0);
        }
    };
    if (ccw) corner(L->left, ccw->right, *ccw, t_ccw);
    if (cw) corner(L->right, cw->left, *cw, t_cw);
    return est + static_cast<double>(jr.trim_margin_mm) / 1000.0;
}

// The far-end clearance of the landing widths at one landed end (ROADS stage 3c-3): 0 unless the stretch's other end is a
// junction (a node of degree > 2 with no join there); then that junction's estimated trim (far_trim_estimate_m) plus
// landing_width_far_margin_m, at most landing_width_far_max_m - the bound for a far node in a junction cluster, whose trim the
// node's own arms cannot tell - and landing_width_far_max_m where no estimate can be made. `estimate_mm` receives the estimate
// (0 when none).
std::int64_t far_clearance_mm(const RoadReferenceBuilder& builder, const RoadRules& rules, const RoadGraphWay& way, std::int64_t way_id,
        RoadStretch stretch, bool at_end, bool far_junction, std::int32_t* estimate_mm = nullptr) {
    if (estimate_mm) *estimate_mm = 0;
    if (!far_junction) return 0;
    const MergeLaneRules& ml = rules.merge_lane;
    const std::int64_t max_mm = static_cast<std::int64_t>(ml.landing_width_far_max_m) * 1000;
    const std::uint32_t far_ref = at_end ? stretch.start_ref : stretch.end_ref;
    const auto est = far_trim_estimate_m(builder, rules, way.node_ids[far_ref], way_id, far_ref);
    if (!est) return max_mm;
    const auto e = static_cast<std::int64_t>(std::llround(*est * 1000.0));
    if (estimate_mm) *estimate_mm = static_cast<std::int32_t>(e);
    return std::min(max_mm, e + static_cast<std::int64_t>(ml.landing_width_far_margin_m) * 1000);
}
// Landing widths (ROADS stage 3b): the longest the landed lane and outer shoulder ramps may run at one end of a ramp: the stretch
// less the far-end clearance, so a junction at the other end meets the ramp's own widths at its trim.
std::int64_t landing_width_cap_mm(double length_m, std::int64_t far_clear_mm) {
    return static_cast<std::int64_t>(std::floor(length_m * 1000.0)) - far_clear_mm;
}

// `junction_ends` (bit 0 start, bit 1 end): ends treated as a junction whatever their node is (ROADS stage 6r: the section a taper
// end is laid onto); `room`: the classifications' gate level (2: room gates and the hint gap check; 1: room gates only, inside that check; 0: none, inside taper_room).
Result<CrossSectionBase> resolve_impl(const RoadReferenceBuilder& builder, std::int64_t way_id, RoadStretch stretch, double length_m,
        const RoadRules& rules, CrossSectionTrace* trace, const ReferenceFitLimits& fit_limits, bool gore, int depth,
        unsigned junction_ends = 0, int room = 2);
void apply_clearance_landing(const RoadReferenceBuilder& builder, const RoadGraphWay& way, std::int64_t way_id, RoadStretch stretch,
        const RoadRules& rules, const ReferenceFitLimits& fit_limits, const std::array<std::optional<MergeNode>, 2>& merge_at,
        RoadCrossSection& xs, CrossSectionTrace& tr, int depth, int room);
void apply_gore_shoulders(const RoadReferenceBuilder& builder, const RoadGraphWay& way, std::int64_t way_id, RoadStretch stretch,
        const RoadRules& rules, const ReferenceFitLimits& fit_limits, const std::array<std::optional<MergeNode>, 2>& merge_at,
        RoadCrossSection& xs, CrossSectionTrace& tr, int depth, int room);
std::int64_t far_ramp_max(const RoadCrossSection& xs, bool at_end);

Result<CrossSectionBase> resolve_impl(const RoadReferenceBuilder& builder, std::int64_t way_id, RoadStretch stretch, double length_m,
        const RoadRules& rules, CrossSectionTrace* trace, const ReferenceFitLimits& fit_limits, bool gore, int depth,
        unsigned junction_ends, int room) {
    CrossSectionTrace local_trace;   // diagnostics only: never read back into the result
    CrossSectionTrace& tr = trace ? *trace : local_trace;
    tr = CrossSectionTrace{};
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
    std::array<bool, 2> single_at_merge{false, false};                    // this end is the single arm's end at a merge-lane node
    std::array<std::optional<MergeNode>, 2> merge_at;                    // the merge node classified at this end (gore shoulders)
    for (int e = 0; e < 2; ++e) {
        const bool at_end = e == 1;
        auto join = builder.join(way_id, stretch, at_end);
        if (!join) return join.error();
        if (!*join && rules.merge_lane.enabled && !(junction_ends & (1u << e))) {
            // A node of degree three: the through pair of a merge lane / lane drop joins like a degree-two node, the ramp lands.
            const auto node = way->node_ids[at_end ? stretch.end_ref : stretch.start_ref];
            tr.ends[e].merge_ran = true;
            if (const auto merge = classify_merge_node(builder, node, rules, fit_limits, &tr.ends[e].merge_reason, nullptr, room)) {
                merge_at[static_cast<std::size_t>(e)] = *merge;
                // A way passing through a taper node is the single and the through arm: its stretch leaving the node (this end is
                // its start) has the direction of the arm it is (ROADS stage 6).
                const bool is_through = merge->through_id == way_id && (merge->single_id != way_id || merge->through_leaves == !at_end);
                if (is_through) join = std::optional<RoadStretchJoin>{RoadStretchJoin{merge->single_id, 0, merge->single_leaves}};
                else if (merge->single_id == way_id) {
                    join = std::optional<RoadStretchJoin>{RoadStretchJoin{merge->through_id, 0, merge->through_leaves}};
                    single_at_merge[static_cast<std::size_t>(e)] = true;
                }
                else if (merge->ramp_id == way_id) {
                    landing[static_cast<std::size_t>(e)] = merge->landing_mm;
                    landing_length[static_cast<std::size_t>(e)] = merge->landing_length_mm;
                    landing_single[static_cast<std::size_t>(e)] = merge->single_id;
                    tr.ends[e].note = CrossSectionEndNote::LandingZero;   // until the landing below applies
                }
            }
        }
        if (!*join) continue;
        const auto* other = find_way(graph, (*join)->way_id);
        if (!other) { tr.ends[e].note = CrossSectionEndNote::NeighbourMissing; continue; }
        auto other_base = resolve_cross_section_base(*other, rules);
        if (!other_base || !other_base->xs) { tr.ends[e].note = CrossSectionEndNote::NeighbourNoSection; continue; } // an unsupported or declined neighbour has no section to meet
        const auto* cls_b = rules.find_class(highway_of(*other));
        if (!cls_b) { tr.ends[e].note = CrossSectionEndNote::NeighbourNoClass; continue; }
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
        // The through pair at a lane-add merge node joins with the ramp-side hint the classifier used (ROADS stage 8a-4).
        const auto& mn = merge_at[static_cast<std::size_t>(e)];
        const bool hinted = mn && mn->hinted && mn->ramp_id != way_id && (*join)->way_id != mn->ramp_id;
        const JoinOutcome out = resolve_join(U, D, rules, hinted ? std::optional<bool>(mn->ramp_right) : std::nullopt);
        auto& r = ends[static_cast<std::size_t>(e)];
        r.present = true;
        r.record = {out.kind, id_b, out.delta};
        tr.ends[e].note = CrossSectionEndNote::Joined;
        tr.ends[e].kind = out.kind;
        tr.ends[e].split = out.split;
        tr.ends[e].delta_mm = a_mirrored ? -out.delta : out.delta;
        tr.ends[e].neighbour_same = same;
        {
            const auto& partners = a_is_u ? out.pu : out.pd;
            const std::size_t nb = other_base->xs->bands.size();
            for (std::size_t i = 0; i < n && i < 32; ++i) {
                const std::size_t ci = a_mirrored ? n - 1 - i : i;
                const int cj = partners[ci];
                tr.ends[e].partner[i] = static_cast<std::int8_t>(cj < 0 ? -1 : b_mirrored ? static_cast<int>(nb) - 1 - cj : cj);
            }
        }
        const SideEval& mine = a_is_u ? out.u : out.d;
        const std::int32_t corr = a_is_u ? out.corr_u : out.corr_d;
        r.correction = a_mirrored ? -corr : corr;
        r.ramps.assign(n, WidthRamp{});
        for (std::size_t i = 0; i < n; ++i) {
            const std::size_t ci = a_mirrored ? n - 1 - i : i;
            if (mine.len[ci] > 0) r.ramps[i] = {mine.len[ci], mine.jw[ci]};
            // The added lane of the single arm is the lane the link lands on (merge) or leaves from (exit): at the node it is
            // already full width, the link's own end or start continues it. A taper to zero there left the link's square end
            // standing beside the carriageway (G2M-032, B 8 west of Hoechst, exit to the Main-Taunus-Zentrum). The far end of
            // the single arm keeps its own join.
            if (single_at_merge[static_cast<std::size_t>(e)] && xs.bands[i].counted && mine.jw[ci] == 0) r.ramps[i] = WidthRamp{};
            if (r.ramps[i].length_mm > 0) tr.ends[e].tapered = true;
        }
    }

    // A taper end of this link (ROADS stage 6r): the stretch is resolved with that end a junction - so the other end (a lane-add
    // landing, a join, a junction) is exactly what it was before tapers existed - and the taper is laid onto that end alone, within
    // the room the rest leaves: the link's counted lane starts at width 0 on the single arm's outer lane edge and widens with its
    // offset over L_t = min(max(taper_length_m, L_clear), landing_clear_max_m, A) (apply_clearance_landing), its outer shoulder
    // starts at the single arm's.
    unsigned taper_mask = 0;
    for (int e = 0; e < 2; ++e)
        if (const auto& m = merge_at[static_cast<std::size_t>(e)]; m && m->taper && m->ramp_id == way_id) taper_mask |= 1u << e;
    if (taper_mask != 0) {
        auto r0 = resolve_impl(builder, way_id, stretch, length_m, rules, &tr, fit_limits, gore, depth, junction_ends | taper_mask, room);
        if (!r0 || !r0->xs) return r0;
        RoadCrossSection& txs = *r0->xs;
        const auto len_mm = static_cast<std::int64_t>(std::floor(length_m * 1000.0));
        std::array<std::optional<MergeNode>, 2> taper_at;
        for (int e = 0; e < 2; ++e) {
            if (!(taper_mask & (1u << e))) continue;
            const auto k = static_cast<std::size_t>(e);
            MergeNode m = *merge_at[k];
            const bool at_end = e == 1;
            auto& t = tr.ends[e];
            t = CrossSectionEndTrace{};
            t.merge_ran = true; t.merge_reason = MergeNodeReason::Ok;
            // The room: half the stretch beside a second taper of this link, else the stretch less the other end's longest ramp and
            // its junction clearance - never more than the classification measured.
            std::int64_t avail = 0;
            if (taper_mask == 3u) {
                avail = len_mm / 2;
            } else {
                const bool far_junction = (at_end ? txs.start_join : txs.end_join).kind == JoinKind::None &&
                                          builder.degree(way->node_ids[at_end ? stretch.start_ref : stretch.end_ref]) > 2;
                avail = len_mm - far_clearance_mm(builder, rules, *way, way_id, stretch, at_end, far_junction, &t.far_trim_estimate_mm) -
                        far_ramp_max(txs, at_end);
            }
            if (m.taper_avail_mm >= 0) avail = std::min(avail, m.taper_avail_mm);
            m.taper_avail_mm = avail;
            t.far_clear_mm = static_cast<std::int32_t>(len_mm - avail);
            if (avail <= 0) { t.note = CrossSectionEndNote::LandingZero; continue; }
            if (txs.offset.mode == OffsetMode::None) txs.offset = {OffsetMode::Ramps, 0, 0, 0, 0};
            if (txs.offset.mode != OffsetMode::Ramps) {   // the classification refuses these (taper_offset_held)
                t.note = txs.offset.mode == OffsetMode::Hold ? CrossSectionEndNote::LandingOffsetHold : CrossSectionEndNote::LandingOffsetTransition;
                continue;
            }
            const auto len = static_cast<std::int32_t>(std::min<std::int64_t>(m.landing_length_mm, avail));
            (at_end ? txs.offset.end_mm : txs.offset.start_mm) = m.landing_mm;
            (at_end ? txs.offset.end_length_mm : txs.offset.start_length_mm) = len;
            JoinRecord rec{JoinKind::Landing, m.single_id, m.landing_mm};
            t.note = CrossSectionEndNote::Landed;
            t.landing_width_mm = len;
            int first = -1, last = -1;
            for (std::size_t i = 0; i < txs.bands.size(); ++i)
                if (txs.bands[i].counted) { if (first < 0) first = static_cast<int>(i); last = static_cast<int>(i); }
            if (first < 0) { t.note = CrossSectionEndNote::LandingZero; continue; }
            for (int i = first; i <= last; ++i)
                if (auto& b = txs.bands[static_cast<std::size_t>(i)]; b.counted) (at_end ? b.end : b.start) = WidthRamp{len, 0};
            const int sh = m.ramp_right ? last + 1 : first - 1;
            if (sh >= 0 && sh < static_cast<int>(txs.bands.size()) && txs.bands[static_cast<std::size_t>(sh)].kind == LaneKind::Shoulder) {
                auto& b = txs.bands[static_cast<std::size_t>(sh)];
                if (b.width_mm != m.s_shoulder_mm) (at_end ? b.end : b.start) = WidthRamp{len, m.s_shoulder_mm};
            } else {
                t.landing_no_outer_shoulder = true;
                if (m.s_shoulder_mm > 0 && txs.bands.size() < 32) {   // as at a lane-add landing (ROADS stage 3c-2)
                    CrossSectionBand b;
                    b.kind = LaneKind::Shoulder; b.direction = LaneDirection::None; b.counted = false; b.width_mm = 0;
                    b.paved = txs.bands[static_cast<std::size_t>(m.ramp_right ? last : first)].paved;
                    b.provenance.kind = Provenance::Inferred; b.provenance.direction = Provenance::Inferred; b.provenance.width = Provenance::Inferred;
                    b.provenance.turn = Provenance::Inferred; b.provenance.change = Provenance::Inferred;
                    (at_end ? b.end : b.start) = WidthRamp{len, m.s_shoulder_mm};
                    const int at = m.ramp_right ? last + 1 : first;
                    txs.bands.insert(txs.bands.begin() + at, b);
                    if (!m.ramp_right) { ++first; ++last; }
                    t.landing_shoulder_added = true;
                }
            }
            // The counted lane's centre at the node with no landing offset (the link's frame): the landing shifts it, not the way line.
            RoadCrossSection probe = txs;
            (at_end ? probe.offset.end_mm : probe.offset.start_mm) = 0;
            std::array<double, 33> bt{};
            if (probe.bands.size() <= 32 && probe.boundaries_at(at_end ? txs.length_m : 0.0, std::span<double>(bt.data(), probe.bands.size() + 1))) {
                const auto c0 = static_cast<std::int32_t>(std::lround((bt[static_cast<std::size_t>(first)] + bt[static_cast<std::size_t>(last) + 1]) / 2 * 1000.0));
                auto& off = at_end ? txs.offset.end_mm : txs.offset.start_mm;
                off -= c0;
                rec.raw_delta_mm = off;
                t.landing_centre_mm = c0;
            }
            (at_end ? txs.end_join : txs.start_join) = rec;
            taper_at[k] = m;
        }
        if (depth <= 1 && rules.merge_lane.enabled && (taper_at[0] || taper_at[1]))
            apply_clearance_landing(builder, *way, way_id, stretch, rules, fit_limits, taper_at, txs, tr, depth, room);
        if (gore && rules.merge_lane.enabled && (taper_at[0] || taper_at[1]))
            apply_gore_shoulders(builder, *way, way_id, stretch, rules, fit_limits, taper_at, txs, tr, depth, room);
        return r0;
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
        if (landing[k] == 0 || landing_length[k] <= 0) continue;   // trace: LandingZero (set above) or no ramp role
        if (xs.offset.mode == OffsetMode::None) xs.offset = {OffsetMode::Ramps, 0, 0, 0, 0};
        if (xs.offset.mode != OffsetMode::Ramps) {
            tr.ends[e].note = xs.offset.mode == OffsetMode::Hold ? CrossSectionEndNote::LandingOffsetHold : CrossSectionEndNote::LandingOffsetTransition;
            continue;
        }
        tr.ends[e].note = CrossSectionEndNote::LandingEndTaken;   // until it lands below
        if (e == 0 && xs.offset.start_mm == 0) { xs.offset.start_mm = landing[0]; xs.offset.start_length_mm = landing_length[0]; ends[0].present = true; ends[0].record = {JoinKind::Landing, landing_single[0], landing[0]}; tr.ends[0].note = CrossSectionEndNote::Landed; }
        if (e == 1 && xs.offset.end_mm == 0) { xs.offset.end_mm = landing[1]; xs.offset.end_length_mm = landing_length[1]; ends[1].present = true; ends[1].record = {JoinKind::Landing, landing_single[1], landing[1]}; tr.ends[1].note = CrossSectionEndNote::Landed; }
    }

    // Band ramps (4.2) and exactness clipping (4.5).
    for (std::size_t i = 0; i < n; ++i) { xs.bands[i].start = ends[0].ramps.empty() ? WidthRamp{} : ends[0].ramps[i];
                                          xs.bands[i].end = ends[1].ramps.empty() ? WidthRamp{} : ends[1].ramps[i]; }
    // Landing widths (ROADS stage 3b): at a landed end the ramp's counted lanes start at the total width of the single arm's added
    // lanes (shared in proportion to their own widths) and its outer shoulder at the single arm's shoulder beyond them, both ramping
    // to the ramp's own widths over the landing length (the offset's); the landing then puts the counted lanes' centre, not the way
    // line, on the added lanes' centre, so the lane edges meet at the node up to the angle's cos residue.
    for (int e = 0; e < 2; ++e) {
        const auto k = static_cast<std::size_t>(e);
        if (tr.ends[e].note != CrossSectionEndNote::Landed || !merge_at[k] || merge_at[k]->added_mm <= 0) continue;
        const MergeNode& m = *merge_at[k];
        const bool at_end = e == 1;
        const std::int32_t len_offset = at_end ? xs.offset.end_length_mm : xs.offset.start_length_mm;
        if (len_offset <= 0) continue;
        // The width ramps end at min(L, the far-end clearance); a remainder below gore_step_m makes none (the end keeps its stage-2
        // widths, the centring below still applies).
        const bool far_junction = !ends[1 - k].present && builder.degree(way->node_ids[at_end ? stretch.start_ref : stretch.end_ref]) > 2;
        const std::int64_t far_clear = far_clearance_mm(builder, rules, *way, way_id, stretch, at_end, far_junction, &tr.ends[e].far_trim_estimate_mm);
        tr.ends[e].far_clear_mm = static_cast<std::int32_t>(far_clear);
        const std::int64_t width_cap = landing_width_cap_mm(xs.length_m, far_clear);
        const auto len = static_cast<std::int32_t>(std::min<std::int64_t>(len_offset, width_cap));
        tr.ends[e].landing_width_mm = len;
        const bool widths = len >= static_cast<std::int64_t>(rules.merge_lane.gore_step_m) * 1000;
        if (!widths) tr.ends[e].landing_widths_skipped = true;
        std::int64_t total = 0; int first = -1, last = -1;
        for (std::size_t i = 0; i < xs.bands.size(); ++i)
            if (xs.bands[i].counted) { total += xs.bands[i].width_mm; if (first < 0) first = static_cast<int>(i); last = static_cast<int>(i); }
        if (total <= 0) continue;
        std::int64_t given = 0;
        for (int i = first; i <= last; ++i) {
            auto& b = xs.bands[static_cast<std::size_t>(i)];
            if (!b.counted) continue;
            const std::int64_t w = i == last ? m.added_mm - given : (static_cast<std::int64_t>(b.width_mm) * m.added_mm + total / 2) / total;
            given += w;
            if (widths && w != b.width_mm) (at_end ? b.end : b.start) = WidthRamp{len, static_cast<std::int32_t>(w)};
        }
        const int sh = m.ramp_right ? last + 1 : first - 1;
        if (!widths) {
        } else if (sh >= 0 && sh < static_cast<int>(xs.bands.size()) && xs.bands[static_cast<std::size_t>(sh)].kind == LaneKind::Shoulder) {
            auto& b = xs.bands[static_cast<std::size_t>(sh)];
            if (b.width_mm != m.s_shoulder_mm) (at_end ? b.end : b.start) = WidthRamp{len, m.s_shoulder_mm};
        } else {
            tr.ends[e].landing_no_outer_shoulder = true;
            // ROADS stage 3c-2: no outer shoulder band on the ramp: one is added beside its counted lanes on the outer side (the
            // single arm's ramp side), width 0 away from this end and starting at the single arm's shoulder at the node, ramping to
            // 0 over the same length as the lane widths (the mirror of the gore shoulder, which ramps from 0). Nowhere else.
            if (m.s_shoulder_mm > 0 && xs.bands.size() < 32) {
                CrossSectionBand b;
                b.kind = LaneKind::Shoulder; b.direction = LaneDirection::None; b.counted = false; b.width_mm = 0;
                b.paved = xs.bands[static_cast<std::size_t>(m.ramp_right ? last : first)].paved;
                b.provenance.kind = Provenance::Inferred; b.provenance.direction = Provenance::Inferred; b.provenance.width = Provenance::Inferred;
                b.provenance.turn = Provenance::Inferred; b.provenance.change = Provenance::Inferred;
                (at_end ? b.end : b.start) = WidthRamp{len, m.s_shoulder_mm};
                const int at = m.ramp_right ? last + 1 : first;
                xs.bands.insert(xs.bands.begin() + at, b);
                if (!m.ramp_right) { ++first; ++last; }
                tr.ends[e].landing_shoulder_added = true;
            }
        }
        // The counted lanes' centre at the node with no landing offset (the ramp's frame): the landing shifts it, not the way line.
        RoadCrossSection probe = xs;
        (at_end ? probe.offset.end_mm : probe.offset.start_mm) = 0;
        std::array<double, 33> t{};
        if (probe.bands.size() <= 32 && probe.boundaries_at(at_end ? xs.length_m : 0.0, std::span<double>(t.data(), probe.bands.size() + 1))) {
            const auto centre_mm = static_cast<std::int32_t>(std::lround((t[static_cast<std::size_t>(first)] + t[static_cast<std::size_t>(last) + 1]) / 2 * 1000.0));
            auto& off = at_end ? xs.offset.end_mm : xs.offset.start_mm;
            off -= centre_mm;
            ends[k].record.raw_delta_mm = off;
            tr.ends[e].landing_centre_mm = centre_mm;
        }
    }
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
    // The clearance landing (ROADS stage 2a) is part of the ramp's section whoever asks for it - the through way's gore rule
    // measures against the lengthened landing; `depth` bounds the mutual resolves (ramp -> through -> ...).
    if (depth <= 1 && rules.merge_lane.enabled && (merge_at[0] || merge_at[1]))
        apply_clearance_landing(builder, *way, way_id, stretch, rules, fit_limits, merge_at, xs, tr, depth, room);
    if (gore && rules.merge_lane.enabled && (merge_at[0] || merge_at[1]))
        apply_gore_shoulders(builder, *way, way_id, stretch, rules, fit_limits, merge_at, xs, tr, depth, room);
    CrossSectionBase result;
    result.xs = std::move(xs);
    return result;
}

// Gore shoulders (ROADS stage 1). One way of a landed merge node: its stretch at the node, reference and section.
struct GoreArm {
    RoadReferenceLine ref;
    RoadCrossSection xs;
    bool at_end = false;              // the node is the stretch's end
    std::size_t first = 0, last = 0;  // counted bands
    double length = 0;
    double station(double d) const { return at_end ? length - d : d; }   // d = distance from the node
    std::optional<double> boundary(double d, std::size_t i) const {
        std::array<double, 33> b{};
        if (!xs.boundaries_at(station(d), std::span<double>(b.data(), xs.bands.size() + 1))) return std::nullopt;
        return b[i];
    }
    // Lateral t of the left (true) or right edge of the counted lanes at distance d from the node.
    std::optional<double> lane_edge(double d, bool left) const { return boundary(d, left ? first : last + 1); }
};
bool counted_range(const RoadCrossSection& xs, std::size_t& first, std::size_t& last) {
    bool any = false;
    for (std::size_t i = 0; i < xs.bands.size(); ++i) if (xs.bands[i].counted) { if (!any) first = i; last = i; any = true; }
    return any && xs.bands.size() <= 32;
}
// Lateral t of point (x, y) in the frame of `pose` (positive left).
double lateral_of(const geom::CurvePose& pose, double x, double y) {
    double sn, cs; det::sincos(pose.heading, sn, cs);
    return -(x - pose.x) * sn + (y - pose.y) * cs;
}
// On which side of `self` the other way lies: the centre of its counted lanes 10 m from the node, seen in self's frame from the
// centre of self's own counted lanes there (ROADS 6r: a taper's link line ends on the through way's centre line, so its reference
// still lies inside the through lanes 10 m out while its landed lanes are beside them; measured from the reference the through
// way read as lying on the wrong side and the gore rule ramped the link's outer shoulder).
std::optional<bool> other_on_left(const GoreArm& self, const GoreArm& other) {
    const double d_self = std::min(10.0, self.length), d_other = std::min(10.0, other.length);
    const auto l = other.lane_edge(d_other, true), r = other.lane_edge(d_other, false); if (!l || !r) return std::nullopt;
    const auto q = lateral_point(other.ref, other.station(d_other), (*l + *r) / 2); if (!q) return std::nullopt;
    const auto ps = self.ref.at(self.station(d_self)); if (!ps) return std::nullopt;
    const auto sl = self.lane_edge(d_self, true), sr = self.lane_edge(d_self, false); if (!sl || !sr) return std::nullopt;
    const double t = lateral_of(*ps, q->x, q->y) - (*sl + *sr) / 2;
    if (t == 0) return std::nullopt;
    return t > 0;
}

// The facing gap at distance d from the node: a's facing lane edge (b lies on a's left when `b_left`) against b's facing lane edge
// abreast of it (a lies on b's left when `a_left`), positive when the two are apart. nullopt beyond b's far end.
std::optional<double> facing_gap(const GoreArm& a, const GoreArm& b, bool b_left, bool a_left, double d) {
    const auto es = a.lane_edge(d, b_left); if (!es) return std::nullopt;
    const auto p = lateral_point(a.ref, a.station(d), *es); if (!p) return std::nullopt;
    double u = std::min(d, b.length);
    for (int it = 0; it < 8; ++it) {   // the foot of the perpendicular on b's reference
        const auto po = b.ref.at(b.station(u)); if (!po) return std::nullopt;
        double sn, cs; det::sincos(po->heading, sn, cs);
        const double along = (p->x - po->x) * cs + (p->y - po->y) * sn;   // along increasing station
        const double nu = std::clamp(u + (b.at_end ? -along : along), 0.0, b.length);
        if (std::fabs(nu - u) < 1e-6) { u = nu; break; }
        u = nu;
    }
    const auto po = b.ref.at(b.station(u)); if (!po) return std::nullopt;
    double sn, cs; det::sincos(po->heading, sn, cs);
    // Beyond b's far end: no gap. Beyond its node end (u = 0) the point lies past the node, where b continues as the single arm;
    // its tangent at the node stands in.
    if (u > 0 && std::fabs((p->x - po->x) * cs + (p->y - po->y) * sn) > 0.5) return std::nullopt;
    const auto eo = b.lane_edge(u, a_left); if (!eo) return std::nullopt;
    const double tp = lateral_of(*po, p->x, p->y);
    return a_left ? tp - *eo : *eo - tp;   // a lies on b's left: apart when further left than b's edge
}

// The ramp and the through arm of a landed merge node at one end of `way`: `self` carries this way's section `xs`, `other` the other
// arm's section resolved without gore shoulders. nullopt for the single arm, a node without the Landing, or unreadable geometry.
struct GorePair { GoreArm self, other; bool self_left = false, other_left = false; bool self_is_ramp = false; };
std::optional<GorePair> gore_pair(const RoadReferenceBuilder& builder, const RoadGraphWay& way, std::int64_t way_id, RoadStretch stretch,
        const RoadRules& rules, const ReferenceFitLimits& fit_limits, const MergeNode& m, bool at_end, const RoadCrossSection& xs, int depth, int room) {
    if (way_id != m.ramp_id && way_id != m.through_id) return std::nullopt;   // the single arm is untouched
    // A way passing through a taper node (ROADS stage 6) is the through arm only on its stretch running the through arm's way.
    const bool passing = m.through_id == m.single_id;
    if (passing && way_id == m.through_id && m.through_leaves != !at_end) return std::nullopt;
    const std::int64_t node = way.node_ids[at_end ? stretch.end_ref : stretch.start_ref];
    const std::int64_t other_id = way_id == m.ramp_id ? m.through_id : m.ramp_id;
    // The other way's stretch at the node, its reference and its section before the gore rule (the through arm's direction when the
    // through way passes through).
    const auto* other_way = find_way(builder.graph(), other_id); if (!other_way) return std::nullopt;
    const auto stretches = builder.stretches(other_id); if (!stretches) return std::nullopt;
    const bool want_dir = passing && other_id == m.through_id;
    std::optional<RoadStretch> ost; bool o_end = false;
    for (const auto& st : *stretches) {
        if (other_way->node_ids[st.start_ref] == node && (!want_dir || m.through_leaves)) { ost = st; o_end = false; break; }
        if (other_way->node_ids[st.end_ref] == node && (!want_dir || !m.through_leaves)) { ost = st; o_end = true; break; }
    }
    if (!ost) return std::nullopt;
    GorePair p;
    { auto f = builder.fit_stretch(way_id, stretch, fit_limits); if (!f) return std::nullopt; p.self.ref = std::move(*f); }
    { auto f = builder.fit_stretch(other_id, *ost, fit_limits); if (!f) return std::nullopt; p.other.ref = std::move(*f); }
    auto other_xs = resolve_impl(builder, other_id, *ost, p.other.ref.length_m, rules, nullptr, fit_limits, false, depth + 1, 0, room);
    if (!other_xs || !other_xs->xs) return std::nullopt;
    p.self.xs = xs; p.other.xs = std::move(*other_xs->xs);
    p.self.at_end = at_end; p.other.at_end = o_end;
    p.self.length = xs.length_m; p.other.length = p.other.ref.length_m;
    if (!counted_range(p.self.xs, p.self.first, p.self.last) || !counted_range(p.other.xs, p.other.first, p.other.last)) return std::nullopt;
    // Only where the landing applies: the ramp's end at this node carries the Landing record.
    p.self_is_ramp = way_id == m.ramp_id;
    const RoadCrossSection& ramp_xs = p.self_is_ramp ? p.self.xs : p.other.xs;
    if (((p.self_is_ramp ? at_end : o_end) ? ramp_xs.end_join : ramp_xs.start_join).kind != JoinKind::Landing) return std::nullopt;
    const auto sl = other_on_left(p.self, p.other), ol = other_on_left(p.other, p.self);
    if (!sl || !ol) return std::nullopt;
    p.self_left = *sl; p.other_left = *ol;
    return p;
}

// The longest ramp (any band, or the offset) reaching in from the end opposite `at_end`.
std::int64_t far_ramp_max(const RoadCrossSection& xs, bool at_end) {
    std::int64_t far_max = at_end ? xs.offset.start_length_mm : xs.offset.end_length_mm;
    for (const auto& b : xs.bands) far_max = std::max<std::int64_t>(far_max, (at_end ? b.start : b.end).length_mm);
    return far_max;
}

// ROADS stage 6r: the room a taper of `ramp` (the link's arm at the node) has on the link's stretch: the stretch length (the served
// fit) less what its other end keeps - half the stretch when that end is a taper of the same link too (both get half); else the
// longest ramp reaching in from there and, when it is a junction, its clearance (3c-3), read off the stretch resolved with this
// end a junction (no room gates inside, so no recursion). held: that section shifts the whole stretch (a hold or transition
// offset), so this end cannot land.
TaperRoom taper_room(const RoadReferenceBuilder& builder, const RoadRules& rules, const ReferenceFitLimits& fit_limits,
        std::int64_t node_id, const RoadNodeArm& ramp) {
    TaperRoom out;
    const RoadGraphWay* way = find_way(builder.graph(), ramp.way_id);
    const auto stretches = builder.stretches(ramp.way_id);
    if (!way || !stretches) return out;
    std::optional<RoadStretch> st;
    for (const auto& x : *stretches) if (ramp.leaves_node ? x.start_ref == ramp.ref : x.end_ref == ramp.ref) { st = x; break; }
    if (!st) return out;
    const bool at_end = !ramp.leaves_node;
    const auto fit = builder.fit_stretch(ramp.way_id, *st, fit_limits);
    if (!fit) return out;
    out.stretch_mm = static_cast<std::int64_t>(std::floor(fit->length_m * 1000.0));
    const std::int64_t far_node = way->node_ids[at_end ? st->start_ref : st->end_ref];
    if (far_node != node_id)
        if (const auto fm = classify_merge_node(builder, far_node, rules, fit_limits, nullptr, nullptr, 0); fm && fm->taper && fm->ramp_id == ramp.way_id) {
            out.fitted = true; out.two_tapers = true;
            out.avail_mm = out.stretch_mm / 2; out.claim_mm = out.stretch_mm - out.avail_mm;
            return out;
        }
    const auto r = resolve_impl(builder, ramp.way_id, *st, fit->length_m, rules, nullptr, fit_limits, true, 0, at_end ? 2u : 1u, 0);
    if (!r || !r->xs) return out;
    const RoadCrossSection& xs0 = *r->xs;
    out.fitted = true;
    out.claim_mm = far_ramp_max(xs0, at_end);
    const bool far_junction = (at_end ? xs0.start_join : xs0.end_join).kind == JoinKind::None && builder.degree(far_node) > 2;
    out.far_clear_mm = far_clearance_mm(builder, rules, *way, ramp.way_id, *st, at_end, far_junction);
    out.avail_mm = out.stretch_mm - out.far_clear_mm - out.claim_mm;
    out.held = xs0.offset.mode == OffsetMode::Hold || xs0.offset.mode == OffsetMode::Transition;
    return out;
}

std::optional<std::int32_t> hint_landing_gap_mm(const RoadReferenceBuilder& builder, const RoadRules& rules, const ReferenceFitLimits& fit_limits,
        const RoadNodeArm& ramp, bool geom_roles) {
    const auto stretches = builder.stretches(ramp.way_id);
    if (!stretches) return std::nullopt;
    for (const auto& st : *stretches) {
        if (ramp.leaves_node ? st.start_ref != ramp.ref : st.end_ref != ramp.ref) continue;
        const auto fit = builder.fit_stretch(ramp.way_id, st, fit_limits);
        if (!fit) return std::nullopt;
        CrossSectionTrace tr;
        const auto r = resolve_impl(builder, ramp.way_id, st, fit->length_m, rules, &tr, fit_limits, true, 0, 0u, geom_roles ? 1 | kRoomGeomRoles : 1);
        if (!r || !r->xs) return std::nullopt;
        const auto& t = tr.ends[ramp.leaves_node ? 0 : 1];
        if (!t.landing_clear) return std::nullopt;
        return t.landing_min_gap_mm;
    }
    return std::nullopt;
}

// Clearance landing (ROADS stage 2a). The ramp's landing offset decays over a slope-derived length (landing_m_per_m, at most
// landing_max_m) whatever the ramp's own line does; where that line still runs inside the through carriageway beyond it, the
// landed lanes snap back across the through lanes. The landing lasts at least to the first station (every gore_step_m) where the
// UNSHIFTED ramp already keeps its facing lane edge outside the through lanes, and grows in gore_step_m steps while the landed lanes
// still cross by more than 0.05 m and a step still narrows the crossing (a crossing at the node itself no length mends), up to
// landing_clear_max_m and what the stretch fits.
// The landing length at one end of a ramp: the offset's and that of every band ramp there (before the gore rule the only band
// ramps at a landed end are the landing widths of stage 3b, which share the offset's length).
// The width ramps keep their far-end clearance (landing_width_cap_mm).
void set_landing_length(RoadCrossSection& xs, bool at_end, std::int32_t len, std::int64_t far_clear_mm) {
    (at_end ? xs.offset.end_length_mm : xs.offset.start_length_mm) = len;
    const auto w = static_cast<std::int32_t>(std::min<std::int64_t>(len, landing_width_cap_mm(xs.length_m, far_clear_mm)));
    for (auto& b : xs.bands) if (auto& r = at_end ? b.end : b.start; r.length_mm > 0) r.length_mm = w;
}

void apply_clearance_landing(const RoadReferenceBuilder& builder, const RoadGraphWay& way, std::int64_t way_id, RoadStretch stretch,
        const RoadRules& rules, const ReferenceFitLimits& fit_limits, const std::array<std::optional<MergeNode>, 2>& merge_at,
        RoadCrossSection& xs, CrossSectionTrace& tr, int depth, int room) {
    const MergeLaneRules& ml = rules.merge_lane;
    if (ml.landing_clear_max_m <= 0 || xs.offset.mode != OffsetMode::Ramps) return;   // 0: off (rules built in code; the loader requires it)
    const auto length_mm = static_cast<std::int64_t>(std::floor(xs.length_m * 1000.0));
    const std::int64_t step = static_cast<std::int64_t>(ml.gore_step_m) * 1000;
    constexpr double kCrossTolM = 0.05;   // the crossing the gore trace reports
    for (int e = 0; e < 2; ++e) {
        const auto& m = merge_at[static_cast<std::size_t>(e)];
        if (!m || m->ramp_id != way_id) continue;
        const bool at_end = e == 1;
        if ((at_end ? xs.end_join : xs.start_join).kind != JoinKind::Landing) continue;
        const bool far_junction = (at_end ? xs.start_join : xs.end_join).kind == JoinKind::None &&
                                  builder.degree(way.node_ids[at_end ? stretch.start_ref : stretch.end_ref]) > 2;
        // A taper end (ROADS stage 6r): its room A stands for the far-end clearance (the widths end at A).
        const std::int64_t far_clear = m->taper ? length_mm - m->taper_avail_mm : far_clearance_mm(builder, rules, way, way_id, stretch, at_end, far_junction);
        const std::int32_t landing_mm = at_end ? xs.offset.end_mm : xs.offset.start_mm;
        std::int32_t base = at_end ? xs.offset.end_length_mm : xs.offset.start_length_mm;
        if ((landing_mm == 0 && !m->taper) || base <= 0) continue;
        // A taper (ROADS stage 6r): L_t = min(max(taper_length_m, L_clear), landing_clear_max_m, A), its widths and offset
        // sharing it; A is the room the rest of the stretch leaves (resolve_impl's taper layering).
        const std::int64_t taper_cap = std::min<std::int64_t>(static_cast<std::int64_t>(ml.landing_clear_max_m) * 1000, m->taper_avail_mm);
        if (m->taper) {
            if (taper_cap <= 0) continue;
            base = static_cast<std::int32_t>(std::min<std::int64_t>(base, taper_cap));
        }
        auto pair = gore_pair(builder, way, way_id, stretch, rules, fit_limits, *m, at_end, xs, depth, room);
        if (!pair) continue;
        GoreArm& r = pair->self;
        const auto set_landing = [&](std::int32_t mm, std::int64_t len) {
            (at_end ? r.xs.offset.end_mm : r.xs.offset.start_mm) = mm;
            set_landing_length(r.xs, at_end, static_cast<std::int32_t>(len), far_clear);
        };
        const auto gap = [&](double d) { return facing_gap(r, pair->other, pair->self_left, pair->other_left, d); };
        const std::int64_t cap = m->taper ? taper_cap
            : std::max<std::int64_t>(base, std::min<std::int64_t>(static_cast<std::int64_t>(ml.landing_clear_max_m) * 1000, length_mm - far_ramp_max(xs, at_end)));
        auto& t = tr.ends[e];
        t.landing_clear = true; t.landing_base_mm = base;
        // L_clear: the unshifted ramp's lanes are out of the through lanes (no through way abreast counts as clear).
        set_landing(0, base);
        std::int64_t clear = cap; bool reached = false;
        for (std::int64_t d = 0; d <= cap; d += step) {
            const auto g = gap(static_cast<double>(d) / 1000.0);
            if (!g || *g >= 0) { clear = d; reached = true; break; }
        }
        t.landing_clear_mm = static_cast<std::int32_t>(clear); t.landing_clear_reached = reached;
        // The served landing: at least L_clear, lengthened while the landed lanes still cross and a step still helps.
        const auto min_gap = [&](std::int64_t len, std::int64_t& at) {
            set_landing(landing_mm, len);
            double worst = 0; at = 0; bool any = false;
            for (std::int64_t d = 0; d <= len; d += step) {
                const auto g = gap(static_cast<double>(d) / 1000.0);
                if (!g) break;
                if (!any || *g < worst) { worst = *g; at = d; any = true; }
            }
            return worst;
        };
        std::int64_t len = std::max<std::int64_t>(base, std::min(clear, cap)), at = 0;
        double worst = min_gap(len, at);
        while (worst < -kCrossTolM && len < cap) {
            const std::int64_t next = std::min(cap, len + step);
            std::int64_t next_at = 0;
            const double w = min_gap(next, next_at);
            if (w <= worst + 0.001) break;   // no progress: the crossing is not the landing's to mend
            len = next; worst = w; at = next_at;
        }
        set_landing_length(xs, at_end, static_cast<std::int32_t>(len), far_clear);
        t.landing_width_mm = static_cast<std::int32_t>(std::min<std::int64_t>(len, landing_width_cap_mm(xs.length_m, far_clear)));
        t.landing_final_mm = static_cast<std::int32_t>(len);
        t.landing_min_gap_mm = static_cast<std::int32_t>(std::round(worst * 1000.0));
        t.landing_min_gap_at_mm = static_cast<std::int32_t>(at);
        set_landing(0, len);   // diagnostics only: r is the gore pair's copy, never read back into xs
        const auto raw = gap(static_cast<double>(at) / 1000.0);
        t.landing_raw_gap_at_worst_mm = raw ? static_cast<std::int32_t>(std::round(*raw * 1000.0)) : std::numeric_limits<std::int32_t>::min();
    }
}

void apply_gore_shoulders(const RoadReferenceBuilder& builder, const RoadGraphWay& way, std::int64_t way_id, RoadStretch stretch,
        const RoadRules& rules, const ReferenceFitLimits& fit_limits, const std::array<std::optional<MergeNode>, 2>& merge_at,
        RoadCrossSection& xs, CrossSectionTrace& tr, int depth, int room) {
    const MergeLaneRules& ml = rules.merge_lane;
    const auto length_mm = static_cast<std::int64_t>(std::floor(xs.length_m * 1000.0));
    const RoadCrossSection xs0 = xs;   // pre-gore, as the other way sees this one: both ends of a node compute the same L_g
    for (int e = 0; e < 2; ++e) {
        const auto& m = merge_at[static_cast<std::size_t>(e)];
        if (!m) continue;
        const bool at_end = e == 1;
        auto pair = gore_pair(builder, way, way_id, stretch, rules, fit_limits, *m, at_end, xs0, depth, room);
        if (!pair) continue;
        GoreArm& self = pair->self; GoreArm& other = pair->other;
        const std::optional<bool> self_left = pair->self_left, other_left = pair->other_left;
        // The facing shoulders: the shoulder band right outside the counted lanes toward the other way (none: width 0).
        const auto facing_shoulder = [](const GoreArm& a, bool left) -> std::optional<std::size_t> {
            if (left) { if (a.first > 0 && a.xs.bands[a.first - 1].kind == LaneKind::Shoulder) return a.first - 1; }
            else if (a.last + 1 < a.xs.bands.size() && a.xs.bands[a.last + 1].kind == LaneKind::Shoulder) return a.last + 1;
            return std::nullopt;
        };
        const auto sb = facing_shoulder(self, *self_left), ob = facing_shoulder(other, *other_left);
        // ROADS stage 8a-1: at a taper node the whole facing bed takes the gore - the shoulder and the verge beyond it (a verge right
        // outside the lanes when there is no shoulder) - so the two beds do not overlap where the link runs beside the carriageway.
        const auto facing_bed = [&](const GoreArm& a, bool left, const std::optional<std::size_t>& sh) {
            std::vector<std::size_t> out;
            if (!m->taper) { if (sh) out.push_back(*sh); return out; }
            const auto n = static_cast<std::ptrdiff_t>(a.xs.bands.size());
            std::ptrdiff_t i = left ? static_cast<std::ptrdiff_t>(a.first) - 1 : static_cast<std::ptrdiff_t>(a.last) + 1;
            if (i >= 0 && i < n && a.xs.bands[static_cast<std::size_t>(i)].kind == LaneKind::Shoulder) { out.push_back(static_cast<std::size_t>(i)); i += left ? -1 : 1; }
            if (i >= 0 && i < n && a.xs.bands[static_cast<std::size_t>(i)].kind == LaneKind::Verge) out.push_back(static_cast<std::size_t>(i));
            return out;
        };
        const auto bed_self = facing_bed(self, *self_left, sb), bed_other = facing_bed(other, *other_left, ob);
        std::int32_t sh_self = 0, sh_other = 0;
        for (const auto i : bed_self) sh_self += self.xs.bands[i].width_mm;
        for (const auto i : bed_other) sh_other += other.xs.bands[i].width_mm;
        auto& t = tr.ends[e];
        t.gore = true; t.gore_shoulder_mm = sh_self;
        const double need = static_cast<double>(sh_self + sh_other) / 1000.0;
        if (need <= 0) continue;
        // gap(d): self's facing lane edge at distance d from the node against the other's facing lane edge abreast of it,
        // positive when they are apart.
        const auto gap_at = [&](double d) { return facing_gap(self, other, *self_left, *other_left, d); };
        const double step = static_cast<double>(ml.gore_step_m), cap = std::min(static_cast<double>(ml.gore_max_m), self.length);
        double nose = 0, prev_d = 0, prev_g = 0, min_g = 0, min_at = 0; bool reached = false, first = true;
        for (int k = 0;; ++k) {
            const double d = std::min(cap, step * k);
            const auto g = gap_at(d);
            if (!g) break;
            if (*g >= need) {
                nose = first ? 0.0 : prev_d + (need - prev_g) / (*g - prev_g) * (d - prev_d);
                reached = true; break;
            }
            if (first || *g < min_g) { min_g = *g; min_at = d; }
            prev_d = d; prev_g = *g; nose = d; first = false;
            if (d >= cap) break;
        }
        t.gore_nose_mm = static_cast<std::int32_t>(std::round(nose * 1000.0));
        t.gore_nose_reached = reached;
        t.gore_min_gap_mm = static_cast<std::int32_t>(std::round(min_g * 1000.0));
        t.gore_min_gap_at_mm = static_cast<std::int32_t>(std::round(min_at * 1000.0));
        if (bed_self.empty()) continue;
        // The ramp length L_g (ROADS stage 2b), one per node: both ways compute it from the LINK's gap profile and both pre-gore
        // sections, so the two facing shoulders share it. L_g is the smallest multiple of gore_step_m at or past the link's nose
        // with smoothstep(s / L_g) * need <= max(gap(s), 0) + gore_overlap_tol at every sample s in [0, L_g] (no through way
        // abreast: no limit), capped by gore_max_m and what both stretches allow (L plus the longest far-end ramp fits the stretch).
        const bool ramp_self = pair->self_is_ramp;
        const GoreArm& ra = ramp_self ? self : other; const GoreArm& ta = ramp_self ? other : self;
        const bool t_on_r_left = ramp_self ? *self_left : *other_left, r_on_t_left = ramp_self ? *other_left : *self_left;
        const std::int64_t step_mm = static_cast<std::int64_t>(ml.gore_step_m) * 1000;
        const auto allowed = [](const GoreArm& a) { return static_cast<std::int64_t>(std::floor(a.length * 1000.0)) - far_ramp_max(a.xs, a.at_end); };
        const std::int64_t allow_self = allowed(self), allow_other = allowed(other);
        const std::int64_t cap_g = std::min<std::int64_t>({static_cast<std::int64_t>(ml.gore_max_m) * 1000, allow_self, allow_other,
                                                           static_cast<std::int64_t>(std::floor(ra.length * 1000.0))});
        if (cap_g <= 0) continue;
        std::vector<std::optional<double>> prof;   // the link's gap every gore_step_m from the node, up to the cap
        for (std::int64_t d = 0; d <= cap_g; d += step_mm) prof.push_back(facing_gap(ra, ta, t_on_r_left, r_on_t_left, static_cast<double>(d) / 1000.0));
        double nose_r = 0; bool nose_r_reached = false;
        for (std::size_t k = 0; k < prof.size(); ++k) {
            if (!prof[k]) break;
            if (*prof[k] >= need) {
                nose_r = k == 0 || !prof[k - 1] ? 0.0
                       : (static_cast<double>(k - 1) + (need - *prof[k - 1]) / (*prof[k] - *prof[k - 1])) * static_cast<double>(ml.gore_step_m);
                nose_r_reached = true; break;
            }
            nose_r = static_cast<double>(k) * static_cast<double>(ml.gore_step_m);
        }
        const double tol = static_cast<double>(ml.gore_overlap_tol_mm) / 1000.0;
        const auto overlap = [&](std::int64_t len, std::int64_t& at) {   // the worst shoulder overlap with ramp length len
            double worst = -1e9; at = 0;
            for (std::size_t k = 0; k < prof.size() && static_cast<std::int64_t>(k) * step_mm <= len; ++k) {
                if (!prof[k]) continue;
                const double s = static_cast<double>(static_cast<std::int64_t>(k) * step_mm) / static_cast<double>(len);
                const double o = road_smooth_step(s) * need - std::max(*prof[k], 0.0);
                if (o > worst) { worst = o; at = static_cast<std::int64_t>(k) * step_mm; }
            }
            return worst;
        };
        t.gore_link_nose_mm = static_cast<std::int32_t>(std::round(nose_r * 1000.0)); t.gore_link_nose_reached = nose_r_reached;
        // A gap that holds both shoulders at the node already (or none measured there): no ramp, as in stage 1.
        std::int64_t lg = (static_cast<std::int64_t>(t.gore_link_nose_mm) + step_mm - 1) / step_mm * step_mm;
        if (lg <= 0) continue;
        bool capped = false;
        std::int64_t at = 0;
        double worst = 0;
        if (lg >= cap_g) { lg = cap_g; worst = overlap(lg, at); capped = worst > tol; }
        else for (;; lg += step_mm) {
            if (lg >= cap_g) { lg = cap_g; worst = overlap(lg, at); capped = worst > tol; break; }
            worst = overlap(lg, at);
            if (worst <= tol) break;
        }
        // Would a longer ramp have met the condition where the stretches (not gore_max_m) stopped it?
        const std::int64_t gore_max_mm = static_cast<std::int64_t>(ml.gore_max_m) * 1000;
        t.gore_ramp_mm = static_cast<std::int32_t>(lg);
        t.gore_overlap_mm = static_cast<std::int32_t>(std::round(std::max(worst, 0.0) * 1000.0));
        t.gore_overlap_at_mm = static_cast<std::int32_t>(at);
        t.gore_capped = capped;
        t.gore_stretch_bound = capped && std::min(allow_self, allow_other) < gore_max_mm;
        t.gore_other_shorter = allow_other < allow_self && allow_other < gore_max_mm;
        // The section format bounds the longest ramp at one end plus the longest at the other (every band and the offset) by the
        // stretch length: the gore ramp yields to whatever reaches in from the far end of the served section (its other end may
        // carry a gore ramp the pre-gore length above did not see).
        const std::int64_t len = std::min<std::int64_t>(lg, length_mm - far_ramp_max(xs, at_end));
        t.gore_ramp_trimmed = len < lg;
        if (len <= 0) continue;
        for (const auto i : bed_self) (at_end ? xs.bands[i].end : xs.bands[i].start) = WidthRamp{static_cast<std::int32_t>(len), 0};
    }
}
} // namespace

Result<CrossSectionBase> resolve_cross_section(const RoadReferenceBuilder& builder, std::int64_t way_id,
        RoadStretch stretch, double length_m, const RoadRules& rules, CrossSectionTrace* trace, const ReferenceFitLimits* fit_limits) {
    return resolve_impl(builder, way_id, stretch, length_m, rules, trace, fit_limits ? *fit_limits : served_fit_limits(rules), true, 0);
}

Result<LateralPoint> lateral_point(const RoadReferenceLine& reference, double s, double t) {
    const auto pose = reference.at(s);
    if (!pose) return pose.error();
    double sn, cs;
    det::sincos(pose->heading, sn, cs);
    return LateralPoint{pose->x - t * sn, pose->y + t * cs};
}
} // namespace g2m
