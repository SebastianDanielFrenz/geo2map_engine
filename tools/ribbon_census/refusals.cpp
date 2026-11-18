// tools/ribbon_census/refusals.cpp - g2m_ribbon_census --refusals E0 N0 E1 N1 (ROADS stage 5a, diagnostics only): every
// degree-3 node in the box (owner tile) whose three arms are one-way ways of a major class or its link (motorway, trunk,
// primary, secondary, tertiary and their _link; one-way read from the way's base cross-section, oneway=-1 counts as
// one-way), with the merge classification of the served rules and whether a ramp actually landed there in the served tile.
// Prints
//
//   RF node E N landed reason S T R hwS hwT hwR lanesS lanesT lanesR addedFound defl rampAngle landingMm landingLenMm
//      one line per qualifying node. landed = 1 when a served entry of the ramp way has a Landing join at the node; reason =
//      the classifier's refusal (ok when it accepted); S/T/R and their classes/counted lanes are 0/- until the roles were
//      found (then the arms in way-id order stand in, written as A/B/C and flagged by roles=0 in the reason field suffix).
//      defl / rampAngle in degrees, landingMm = the landing offset the node needs (when reached), landingLenMm the
//      slope-derived landing length clamp(landing_m_per_m*|landing|, landing_min_m, landing_max_m).
//   CH node way first_stop first_len_m chain_stop chain_len_m chain_stretches clear_est_m served_clear_reached served_clear_mm stretch_len_m note
//      (landed = the ramp's resolved trace has note Landed at the node: the served entry when the tile holds it, else
//      the stretch fitted with the served limits, flagged /fit after the note)
//      for every node whose roles were found: the ramp R walked away from the node. first_stop = what ends the ramp's stretch
//      (junction: a node of degree >= 3; dead_end; join_same: a degree-two way join to a one-way way in the same direction
//      with the same counted lanes; join_lanes: ... with a different counted lane count; join_dir: ... to a way that is not
//      one-way in the same direction), first_len_m its distance from the node; chain_* = the same walk continued through
//      every join_same node, stopping at the first other stop (chain_stretches = the stretches passed). clear_est_m: an
//      ESTIMATE of the first station along the ramp chain where the ramp's own way line is at least the half counted-lane
//      widths of R and T away from T's way line (T also walked away from the node through its own join_same nodes), -1 when
//      not within the chain; it assumes each way line is its counted-lane centre. served_*: the served resolve's trace of the
//      ramp entry at the node (landing_clear_reached, landing_clear_mm; -1 when not run), the served stretch length and the
//      end note of that trace (cross_section_end_note_name, - when no served entry ends there).
//   RFSUM reason count landed_count
#include "g2m/core/tile_key.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_rules.h"
#include "g2m/world/terrain_service.h"
#include "g2m/world/ticket.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

using namespace g2m;
using namespace g2m::world;

namespace {
const RoadGraphNode* node_of(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.nodes.begin(), g.nodes.end(), id, [](const RoadGraphNode& n, std::int64_t v) { return n.osm_id < v; });
    return it != g.nodes.end() && it->osm_id == id ? &*it : nullptr;
}
const RoadGraphWay* way_of(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.ways.begin(), g.ways.end(), id, [](const RoadGraphWay& w, std::int64_t v) { return w.osm_id < v; });
    return it != g.ways.end() && it->osm_id == id ? &*it : nullptr;
}
std::string hw_of(const RoadGraphWay* w) {
    if (!w) return "-";
    for (const auto& [k, v] : w->tags) if (k == "highway") return v;
    return "-";
}
bool major(const std::string& hw) {
    static const std::set<std::string> m{"motorway", "trunk", "primary", "secondary", "tertiary", "motorway_link", "trunk_link",
                                         "primary_link", "secondary_link", "tertiary_link"};
    return m.count(hw) != 0;
}
struct Base { bool oneway = false, reversed = false; int counted = 0; double counted_w = 0; };
std::optional<Base> base_of(const RoadGraphWay* w, const RoadRules& rules) {
    if (!w) return std::nullopt;
    auto b = resolve_cross_section_base(*w, rules);
    if (!b || !b->xs) return std::nullopt;
    Base out;
    out.oneway = (b->xs->flags & kXsOneway) != 0;
    out.reversed = (b->xs->flags & kXsReversedOneway) != 0;
    for (const auto& band : b->xs->bands)
        if (band.counted) { out.counted += 1; out.counted_w += static_cast<double>(band.width_mm) / 1000.0; }
    return out;
}
struct P { double x, y; };
P pos(const RoadGraphNode& n) { return {static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0}; }

// A walk away from a node along one arm, continued through degree-two same-direction one-way joins with the same counted
// lanes (when `chain`). `pts` is the polyline from the node.
struct Walk { std::vector<P> pts; std::string first_stop, stop; double first_len = 0, len = 0; int stretches = 1; };
Walk walk(const RoadReferenceBuilder& builder, const RoadRules& rules, const RoadNodeArm& start, bool chain, double max_len = 1000.0) {
    const RoadGraph& g = builder.graph();
    Walk out;
    const RoadGraphWay* w = way_of(g, start.way_id);
    const auto b0 = base_of(w, rules);
    const int lanes = b0 ? b0->counted : -1;
    std::int64_t ref = start.ref;
    int step = start.leaves_node ? 1 : -1;
    const RoadGraphNode* n0 = w ? node_of(g, w->node_ids[static_cast<std::size_t>(ref)]) : nullptr;
    if (!w || !n0) { out.first_stop = out.stop = "missing"; return out; }
    out.pts.push_back(pos(*n0));
    bool first_done = false;
    const auto stop_at = [&](const std::string& why) {
        if (!first_done) { out.first_stop = why; out.first_len = out.len; first_done = true; }
        out.stop = why;
    };
    for (int guard = 0; guard < 100000; ++guard) {
        const std::int64_t nref = ref + step;
        if (nref < 0 || nref >= static_cast<std::int64_t>(w->node_ids.size())) {
            // the way ends at the node at `ref`
            const std::int64_t nid = w->node_ids[static_cast<std::size_t>(ref)];
            const auto deg = builder.degree(nid);
            if (deg >= 3) { stop_at("junction"); return out; }
            if (deg <= 1) { stop_at("dead_end"); return out; }
            const auto arms = builder.arms(nid);
            const RoadNodeArm* next = nullptr;
            for (const auto& a : arms) if (!(a.way_id == w->osm_id && a.ref == static_cast<std::uint32_t>(ref))) next = &a;
            if (!next) { stop_at("ring"); return out; }
            const RoadGraphWay* nw = way_of(g, next->way_id);
            const auto nb = base_of(nw, rules);
            std::string kind;
            if (!nb || !nb->oneway || nb->reversed || !b0 || b0->reversed || next->leaves_node != (step == 1)) kind = "join_dir";
            else if (nb->counted != lanes) kind = "join_lanes";
            else kind = "join_same";
            if (!first_done) { out.first_stop = kind; out.first_len = out.len; first_done = true; }
            if (!chain || kind != "join_same" || out.len >= max_len) { out.stop = kind; return out; }
            w = nw; ref = next->ref; ++out.stretches;
            continue;
        }
        const RoadGraphNode* a = node_of(g, w->node_ids[static_cast<std::size_t>(ref)]);
        const RoadGraphNode* b = node_of(g, w->node_ids[static_cast<std::size_t>(nref)]);
        if (!a || !b) { stop_at("missing"); return out; }
        const P pa = pos(*a), pb = pos(*b);
        out.len += std::hypot(pb.x - pa.x, pb.y - pa.y);
        out.pts.push_back(pb);
        ref = nref;
        if (builder.degree(b->osm_id) >= 3) { stop_at("junction"); return out; }
        if (out.len >= max_len) { stop_at("max_len"); return out; }
    }
    stop_at("guard");
    return out;
}
double dist_to(const std::vector<P>& line, P p) {
    double best = 1e18;
    for (std::size_t i = 1; i < line.size(); ++i) {
        const double ax = line[i - 1].x, ay = line[i - 1].y, bx = line[i].x, by = line[i].y;
        const double vx = bx - ax, vy = by - ay, l2 = vx * vx + vy * vy;
        const double t = l2 > 0 ? std::clamp(((p.x - ax) * vx + (p.y - ay) * vy) / l2, 0.0, 1.0) : 0.0;
        best = std::min(best, std::hypot(p.x - (ax + t * vx), p.y - (ay + t * vy)));
    }
    return best;
}
// The first station (every 1 m) along `ramp` where it is `need` away from `through`; -1 when never.
double clear_station(const std::vector<P>& ramp, const std::vector<P>& through, double need) {
    double s = 0;
    for (std::size_t i = 1; i < ramp.size(); ++i) {
        const double seg = std::hypot(ramp[i].x - ramp[i - 1].x, ramp[i].y - ramp[i - 1].y);
        for (double u = 0; u < seg; u += 1.0) {
            const P p{ramp[i - 1].x + (ramp[i].x - ramp[i - 1].x) * u / seg, ramp[i - 1].y + (ramp[i].y - ramp[i - 1].y) * u / seg};
            if (s + u > 0.5 && dist_to(through, p) >= need) return s + u;
        }
        s += seg;
    }
    return -1;
}
} // namespace

int run_refusals(TerrainService& svc, const geo::UtmZone& zone, const double box[4]) {
    const RoadRules& rules = builtin_road_rules_de();
    const MergeLaneRules& ml = rules.merge_lane;
    ReferenceFitLimits fit_limits;   // the served planar fit (as served_fit_limits in road_cross_section_join.cpp)
    fit_limits.allow_curvature_exceptions = true;
    fit_limits.chord_half_width_factor = static_cast<double>(rules.reference.chord_half_width_percent) / 100.0;
    fit_limits.chord_margin_m = static_cast<double>(rules.reference.chord_margin_mm) / 1000.0;
    const auto k0 = tile_key_at(zone, 2, box[0], box[1]), k1 = tile_key_at(zone, 2, box[2], box[3]);
    if (!k0 || !k1) return 1;
    std::map<std::string, std::pair<int, int>> sum;
    for (std::int32_t ty = k0->y; ty <= k1->y; ++ty)
        for (std::int32_t tx = k0->x; tx <= k1->x; ++tx) {
            TileKey key = *k0; key.x = tx; key.y = ty;
            auto ticket = svc.road_geom(key, Priority::Render);
            ticket.wait_for(std::chrono::minutes(30));
            const auto geometry = ticket.value();
            if (!geometry) {
                if (ticket.state() == TileState::Failed) std::cerr << "tile " << tx << ' ' << ty << " failed: " << ticket.message() << '\n';
                continue;
            }
            const RoadGraph& G = geometry->source.graph;
            const auto builder = RoadReferenceBuilder::make(G);
            if (!builder) { std::cerr << "builder failed " << tx << ' ' << ty << '\n'; continue; }
            for (const auto& node : G.nodes) {
                const P np = pos(node);
                if (std::floor(np.x / 1024) != key.x || std::floor(np.y / 1024) != key.y) continue;
                if (np.x < box[0] || np.x > box[2] || np.y < box[1] || np.y > box[3]) continue;
                if (builder->degree(node.osm_id) != 3) continue;
                const auto arms = builder->arms(node.osm_id);
                if (arms.size() != 3) continue;
                bool qual = true;
                for (const auto& a : arms) {
                    const RoadGraphWay* w = way_of(G, a.way_id);
                    const auto b = base_of(w, rules);
                    if (!major(hw_of(w)) || !b || !b->oneway) { qual = false; break; }
                }
                if (!qual) continue;
                const MergeNodeDiagnosis d = diagnose_merge_node(*builder, node.osm_id, rules);
                // landed: the ramp's resolved cross-section (the served entry's stretch and length when this tile holds it, else
                // the stretch fitted with the served limits) reports Landed at this node
                bool landed = false, via_fit = false;
                int reached = -1, clear_mm = -1;
                std::string note = "-";
                double stretch_len = -1;
                if (d.roles) {
                    const RoadGraphWay* w = way_of(G, d.ramp_id);
                    std::optional<RoadStretch> st;
                    bool at_end = false;
                    for (const auto& en : geometry->entries) {
                        if (!w || en.way_id != d.ramp_id || !en.cross_section || !en.profile) continue;
                        for (int e = 0; e < 2; ++e) {
                            const std::uint32_t r = e ? en.stretch.end_ref : en.stretch.start_ref;
                            if (r < w->node_ids.size() && w->node_ids[r] == node.osm_id) { st = en.stretch; at_end = e == 1; stretch_len = en.profile->reference.length_m; }
                        }
                    }
                    if (!st && w)
                        if (const auto sts = builder->stretches(d.ramp_id))
                            for (const auto& x : *sts)
                                for (int e = 0; e < 2; ++e) {
                                    const std::uint32_t r = e ? x.end_ref : x.start_ref;
                                    if (r < w->node_ids.size() && w->node_ids[r] == node.osm_id)
                                        if (const auto f = builder->fit_stretch(d.ramp_id, x, fit_limits)) { st = x; at_end = e == 1; stretch_len = f->length_m; via_fit = true; }
                                }
                    CrossSectionTrace tr;
                    if (st && resolve_cross_section(*builder, d.ramp_id, *st, stretch_len, rules, &tr)) {
                        const auto& t = tr.ends[at_end ? 1 : 0];
                        note = cross_section_end_note_name(t.note);
                        landed = t.note == CrossSectionEndNote::Landed;
                        if (t.landing_clear) { reached = t.landing_clear_reached ? 1 : 0; clear_mm = t.landing_clear_mm; }
                    }
                }
                std::string reason = merge_node_reason_name(d.reason);
                if (d.reason == MergeNodeReason::Ok && !landed) reason = "ok_not_landed";
                auto& s = sum[landed ? std::string("landed") : reason];
                s.first += 1; s.second += landed ? 1 : 0;
                std::int64_t ids[3];
                int cnt[3] = {0, 0, 0};
                if (d.roles) {
                    ids[0] = d.single_id; ids[1] = d.through_id; ids[2] = d.ramp_id;
                    cnt[0] = d.single_counted; cnt[1] = d.through_counted; cnt[2] = d.ramp_counted;
                } else {
                    for (int i = 0; i < 3; ++i) {
                        ids[i] = arms[static_cast<std::size_t>(i)].way_id;
                        const auto b = base_of(way_of(G, ids[i]), rules);
                        cnt[i] = b ? b->counted : 0;
                    }
                }
                const double land_len = d.landing_known
                    ? std::clamp(static_cast<double>(ml.landing_m_per_m) * std::fabs(d.landing_needed_mm) / 1000.0,
                                 static_cast<double>(ml.landing_min_m), static_cast<double>(ml.landing_max_m)) : 0.0;
                std::printf("RF %lld %.2f %.2f %d %s%s %lld %lld %lld %s %s %s %d %d %d %d %.1f %.1f %d %.0f\n",
                            static_cast<long long>(node.osm_id), np.x, np.y, landed ? 1 : 0, reason.c_str(), d.roles ? (d.taper ? "/taper" : "") : "/noroles",
                            static_cast<long long>(ids[0]), static_cast<long long>(ids[1]), static_cast<long long>(ids[2]),
                            hw_of(way_of(G, ids[0])).c_str(), hw_of(way_of(G, ids[1])).c_str(), hw_of(way_of(G, ids[2])).c_str(),
                            cnt[0], cnt[1], cnt[2], d.added_count, d.through_deflection_deg, d.ramp_angle_deg,
                            d.landing_known ? d.landing_needed_mm : 0, land_len * 1000.0);
                // ROADS stage 6r: a taper candidate's room (mm): stretch, far-end clearance, the other end's claim, what is left
                if (d.taper_candidate)
                    std::printf("TR %lld %s %.2f %d %lld %lld %lld %lld %d %d %.2f\n", static_cast<long long>(node.osm_id), reason.c_str(),
                                d.ramp_angle_deg, d.room_known ? 1 : 0, static_cast<long long>(d.stretch_mm), static_cast<long long>(d.far_clear_mm),
                                static_cast<long long>(d.claim_mm), static_cast<long long>(d.avail_mm), d.two_tapers ? 1 : 0, d.offset_held ? 1 : 0, d.ramp_fit_angle_deg);
                if (!d.roles) continue;
                const RoadNodeArm* ra = nullptr;
                const RoadNodeArm* ta = nullptr;
                for (const auto& a : arms) { if (a.way_id == d.ramp_id) ra = &a; if (a.way_id == d.through_id) ta = &a; }
                if (!ra || !ta) continue;
                const Walk first = walk(*builder, rules, *ra, false);
                const Walk ch = walk(*builder, rules, *ra, true);
                const Walk tw = walk(*builder, rules, *ta, true);
                const auto rb = base_of(way_of(G, d.ramp_id), rules), tb = base_of(way_of(G, d.through_id), rules);
                const double need = (rb ? rb->counted_w : 0) / 2 + (tb ? tb->counted_w : 0) / 2;
                const double cl = clear_station(ch.pts, tw.pts, need);
                std::printf("CH %lld %lld %s %.2f %s %.2f %d %.1f %d %d %.2f %s%s\n", static_cast<long long>(node.osm_id),
                            static_cast<long long>(d.ramp_id), first.first_stop.c_str(), first.first_len, ch.stop.c_str(), ch.len,
                            ch.stretches, cl, reached, clear_mm, stretch_len, note.c_str(), via_fit ? "/fit" : "");
            }
        }
    for (const auto& [r, c] : sum) std::printf("RFSUM %s %d %d\n", r.c_str(), c.first, c.second);
    return 0;
}
