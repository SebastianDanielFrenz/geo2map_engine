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
    for (int e = 0; e < 2; ++e) {
        const bool at_end = e == 1;
        const auto join = builder.join(way_id, stretch, at_end);
        if (!join) return join.error();
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
