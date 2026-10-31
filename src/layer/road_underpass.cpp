// Street fidelity P2 slice 12, request G-6: see g2m/layer/road_underpass.h.
#include "g2m/layer/road_underpass.h"

#include "g2m/core/det_math.h"
#include "g2m/core/geom/exact.h"
#include "g2m/layer/road_attributes.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/core/tile_key.h"

#include <algorithm>
#include <charconv>
#include <array>
#include <limits>
#include <set>
#include <string_view>
#include <tuple>

namespace g2m {
namespace {
int layer_of(const RoadGraphWay& way) {
    for (const auto& [k, v] : way.tags) {
        if (k != "layer") continue;
        int value = 0;
        const auto [p, ec] = std::from_chars(v.data(), v.data() + v.size(), value);
        if (ec == std::errc{} && p == v.data() + v.size() && value >= -5 && value <= 5) return value;
        return 0;
    }
    return 0;
}
bool tag_is_yes(const RoadGraphWay& way, std::string_view key) {
    for (const auto& [k, v] : way.tags)
        if (k == key) return v != "no";
    return false;
}
// D61: a way that is a storey of a parking structure (a parking aisle, or any way with a level=* tag). Its OSM layer
// says nothing about the ground; the bare-earth DEM shows neither the deck nor the space under it.
bool in_structure(const RoadGraphWay& way) {
    for (const auto& [k, v] : way.tags) {
        if (k == "service" && v == "parking_aisle") return true;
        if (k == "level") return true;
    }
    return false;
}
const RoadGraphNode* node_of(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.nodes.begin(), g.nodes.end(), id, [](const RoadGraphNode& n, std::int64_t v) { return n.osm_id < v; });
    return it != g.nodes.end() && it->osm_id == id ? &*it : nullptr;
}
const RoadGraphWay* way_of(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.ways.begin(), g.ways.end(), id, [](const RoadGraphWay& w, std::int64_t v) { return w.osm_id < v; });
    return it != g.ways.end() && it->osm_id == id ? &*it : nullptr;
}
// Intersection point of segments ab and cd (exactly intersecting) in metres.
void intersection_m(geom::PointMm a, geom::PointMm b, geom::PointMm c, geom::PointMm d, double& x, double& y) {
    const double ax = static_cast<double>(a.x) / 1000, ay = static_cast<double>(a.y) / 1000;
    const double rx = static_cast<double>(b.x - a.x) / 1000, ry = static_cast<double>(b.y - a.y) / 1000;
    const double sx = static_cast<double>(d.x - c.x) / 1000, sy = static_cast<double>(d.y - c.y) / 1000;
    const double qx = static_cast<double>(c.x) / 1000 - ax, qy = static_cast<double>(c.y) / 1000 - ay;
    const double den = rx * sy - ry * sx;
    const double u = den != 0 ? std::clamp((qx * sy - qy * sx) / den, 0.0, 1.0) : 0.0;
    x = ax + u * rx;
    y = ay + u * ry;
}
double nearest_station(const RoadReferenceLine& ref, double x, double y) {
    const auto dist2 = [&](double s) {
        const auto p = ref.at(s);
        if (!p) return std::numeric_limits<double>::infinity();
        return (p->x - x) * (p->x - x) + (p->y - y) * (p->y - y);
    };
    double best = 0, best_d = dist2(0);
    for (double s = 1; s < ref.length_m; s += 1) {
        const double d = dist2(s);
        if (d < best_d) { best_d = d; best = s; }
    }
    if (const double d = dist2(ref.length_m); d < best_d) { best_d = d; best = ref.length_m; }
    for (double s = std::max(0.0, best - 1); s <= std::min(ref.length_m, best + 1); s += 0.01) {
        const double d = dist2(s);
        if (d < best_d) { best_d = d; best = s; }
    }
    return best;
}
} // namespace

UnderpassCensus road_underpass_census(const RoadGeomTile& tile, const UnderpassCensusOptions& options) {
    UnderpassCensus out;
    const RoadGraph& g = tile.source.graph;
    // Per-way bounding boxes (mm), a prefilter for the exact segment tests.
    struct Box { std::int64_t x0, y0, x1, y1; };
    std::vector<Box> boxes(g.ways.size(), Box{std::numeric_limits<std::int64_t>::max(), std::numeric_limits<std::int64_t>::max(),
                                              std::numeric_limits<std::int64_t>::min(), std::numeric_limits<std::int64_t>::min()});
    for (std::size_t w = 0; w < g.ways.size(); ++w) {
        for (const std::int64_t id : g.ways[w].node_ids) {
            if (const RoadGraphNode* n = node_of(g, id)) {
                boxes[w].x0 = std::min<std::int64_t>(boxes[w].x0, n->position.x); boxes[w].y0 = std::min<std::int64_t>(boxes[w].y0, n->position.y);
                boxes[w].x1 = std::max<std::int64_t>(boxes[w].x1, n->position.x); boxes[w].y1 = std::max<std::int64_t>(boxes[w].y1, n->position.y);
            }
        }
    }
    for (std::size_t e = 0; e < tile.entries.size(); ++e) {
        const RoadGeomEntry& entry = tile.entries[e];
        if (!entry.profile || entry.profile->attributes.bridge || entry.profile->attributes.tunnel) continue;
        const RoadGraphWay* upper = way_of(g, entry.way_id);
        if (!upper) continue;
        ++out.ground_entries;
        const int upper_layer = layer_of(*upper);
        const auto& prof = *entry.profile;
        const double length = prof.reference.length_m;
        for (std::uint32_t i = entry.stretch.start_ref; i < entry.stretch.end_ref; ++i) {
            const RoadGraphNode* a = node_of(g, upper->node_ids[i]);
            const RoadGraphNode* b = node_of(g, upper->node_ids[i + 1]);
            if (!a || !b) continue;
            const std::int64_t sx0 = std::min<std::int64_t>(a->position.x, b->position.x), sx1 = std::max<std::int64_t>(a->position.x, b->position.x);
            const std::int64_t sy0 = std::min<std::int64_t>(a->position.y, b->position.y), sy1 = std::max<std::int64_t>(a->position.y, b->position.y);
            for (std::size_t w = 0; w < g.ways.size(); ++w) {
                const RoadGraphWay& lower = g.ways[w];
                if (lower.osm_id == upper->osm_id) continue;
                if (boxes[w].x1 < sx0 || boxes[w].x0 > sx1 || boxes[w].y1 < sy0 || boxes[w].y0 > sy1) continue;
                for (std::size_t k = 0; k + 1 < lower.node_ids.size(); ++k) {
                    const std::int64_t c_id = lower.node_ids[k], d_id = lower.node_ids[k + 1];
                    if (c_id == a->osm_id || c_id == b->osm_id || d_id == a->osm_id || d_id == b->osm_id) continue;
                    const RoadGraphNode* c = node_of(g, c_id);
                    const RoadGraphNode* d = node_of(g, d_id);
                    if (!c || !d || !geom::segments_intersect(a->position, b->position, c->position, d->position)) continue;
                    const bool tunnel = tag_is_yes(lower, "tunnel") && !tag_is_yes(lower, "bridge");
                    const int lower_layer = layer_of(lower);
                    if (tag_is_yes(lower, "bridge")) continue;   // above, not below
                    if (!tunnel && lower_layer >= upper_layer) {
                        if (lower_layer == upper_layer) ++out.same_layer;
                        continue;
                    }
                    UnderpassCrossing x;
                    x.upper_entry = e; x.upper_way = upper->osm_id; x.lower_way = lower.osm_id;
                    x.upper_layer = upper_layer; x.lower_layer = lower_layer; x.lower_tunnel = tunnel;
                    intersection_m(a->position, b->position, c->position, d->position, x.x_m, x.y_m);
                    x.station_m = nearest_station(prof.reference, x.x_m, x.y_m);
                    const double s0 = std::max(0.0, x.station_m - options.chord_half_m);
                    const double s1 = std::min(length, x.station_m + options.chord_half_m);
                    if (x.station_m - s0 < options.min_side_m || s1 - x.station_m < options.min_side_m) { ++out.short_side; continue; }
                    const auto p = prof.at(x.station_m), p0 = prof.at(s0), p1 = prof.at(s1);
                    if (!p || !p0 || !p1) continue;
                    x.profile_m = p->height_m;
                    x.chord_m = p0->height_m + (p1->height_m - p0->height_m) * (x.station_m - s0) / (s1 - s0);
                    x.dip_m = x.chord_m - x.profile_m;
                    out.crossings.push_back(x);
                }
            }
        }
    }
    std::sort(out.crossings.begin(), out.crossings.end(), [](const UnderpassCrossing& l, const UnderpassCrossing& r) {
        return std::tie(l.upper_way, l.upper_entry, l.station_m, l.lower_way) < std::tie(r.upper_way, r.upper_entry, r.station_m, r.lower_way);
    });
    for (const auto& x : out.crossings) {
        out.max_dip_m = std::max(out.max_dip_m, x.dip_m);
        if (x.dip_m > options.threshold_m) ++out.over_threshold;
    }
    return out;
}
// ---- Underpass decks (D53-D58) ----

namespace {
constexpr std::int64_t kCellMm = 64000;
std::uint64_t cell_key(std::int64_t cx, std::int64_t cy) {
    return (static_cast<std::uint64_t>(cx + (std::int64_t{1} << 31)) << 32) | static_cast<std::uint64_t>(cy + (std::int64_t{1} << 31));
}
// A pose at station s of the reference; beyond an end on the tangent of the end pose (the window may reach past a
// stretch end, D55).
std::optional<geom::CurvePose> pose_at(const RoadReferenceLine& ref, double s) {
    if (s >= 0 && s <= ref.length_m) { const auto p = ref.at(s); if (!p) return std::nullopt; return *p; }
    const bool before = s < 0;
    const auto e = ref.at(before ? 0.0 : ref.length_m); if (!e) return std::nullopt;
    const double d = before ? s : s - ref.length_m;
    geom::CurvePose p = *e; p.x += d * det::cos(e->heading); p.y += d * det::sin(e->heading); p.curvature = 0;
    return p;
}
// A window sample point: position and unit lateral normal (zone metres).
struct Frame { double x, y, nx, ny; };
// Step 2 of D55: h(k), k = -W..W, at stations centre + k: the median of 7 lateral samples 1 m apart (the profile's own
// sampling). False where the DEM or the line is missing.
template <class FrameAt>
bool sample_window(const RoadDemSampler& dem, geo::UtmZone zone, int W, double centre, const FrameAt& frame_at, std::vector<std::int32_t>& h) {
    for (int k = -W; k <= W; ++k) {
        const std::optional<Frame> f = frame_at(centre + k); if (!f) return false;
        std::array<std::int32_t, 7> lateral{};
        for (int o = -3; o <= 3; ++o) {
            const auto v = dem.sample(zone, {std::llround((f->x + o * f->nx) * 1000), std::llround((f->y + o * f->ny) * 1000)});
            if (!v) return false;
            lateral[static_cast<std::size_t>(o + 3)] = *v;
        }
        std::sort(lateral.begin(), lateral.end());
        h[static_cast<std::size_t>(k + W)] = lateral[3];
    }
    return true;
}
// D63: one way's node polyline walked away from a join node (raw nodes, so shared-node tests keep every id; cum =
// distance from the node along the polyline, non-decreasing).
struct JoinSide { std::vector<geom::PointMm> p; std::vector<std::int64_t> id; std::vector<double> cum; };
// The frame at distance d >= 0 from the node along a side; beyond its last node straight on along its last segment.
std::optional<Frame> side_frame(const JoinSide& s, double d) {
    std::size_t seg = s.p.size();
    for (std::size_t i = 0; i + 1 < s.p.size(); ++i) {
        if (!(s.cum[i + 1] > s.cum[i])) continue;
        seg = i; if (d < s.cum[i + 1]) break;
    }
    if (seg == s.p.size()) return std::nullopt;
    const double l = s.cum[seg + 1] - s.cum[seg];
    const double ux = static_cast<double>(s.p[seg + 1].x - s.p[seg].x) / 1000 / l, uy = static_cast<double>(s.p[seg + 1].y - s.p[seg].y) / 1000 / l;
    const double along = d - s.cum[seg];
    return Frame{static_cast<double>(s.p[seg].x) / 1000 + along * ux, static_cast<double>(s.p[seg].y) / 1000 + along * uy, -uy, ux};
}
} // namespace

RoadUnderpassFinder::RoadUnderpassFinder(const RoadGraph& crossings, const RoadDemSampler& dem, geo::UtmZone zone, const UnderpassRules& rules,
    const RoadGraph* context)
    : graph_(crossings), context_(context ? *context : crossings), dem_(dem), zone_(zone), rules_(rules) {
    if (!rules_.enabled) return;
    for (std::size_t w = 0; w < graph_.ways.size(); ++w) {
        const auto& way = graph_.ways[w];
        for (std::size_t k = 0; k + 1 < way.node_ids.size(); ++k) {
            const RoadGraphNode* c = node_of(graph_, way.node_ids[k]);
            const RoadGraphNode* d = node_of(graph_, way.node_ids[k + 1]);
            if (!c || !d) continue;
            const auto x0 = floor_div(std::min(c->position.x, d->position.x), kCellMm), x1 = floor_div(std::max(c->position.x, d->position.x), kCellMm);
            const auto y0 = floor_div(std::min(c->position.y, d->position.y), kCellMm), y1 = floor_div(std::max(c->position.y, d->position.y), kCellMm);
            if ((x1 - x0 + 1) * (y1 - y0 + 1) > 4096) continue;   // a segment over 4 km is no road segment
            for (auto y = y0; y <= y1; ++y) for (auto x = x0; x <= x1; ++x) cells_.push_back({cell_key(x, y), Seg{w, k}});
        }
    }
    std::sort(cells_.begin(), cells_.end(), [](const auto& l, const auto& r) {
        return std::tie(l.first, l.second.way, l.second.index) < std::tie(r.first, r.second.way, r.second.index);
    });
    // D63: node -> incident edges of the context graph.
    incident_.reserve(2 * context_.edges.size());
    for (std::size_t e = 0; e < context_.edges.size(); ++e) {
        incident_.push_back({context_.edges[e].from_node, static_cast<std::uint32_t>(e)});
        incident_.push_back({context_.edges[e].to_node, static_cast<std::uint32_t>(e)});
    }
    std::sort(incident_.begin(), incident_.end());
}

void RoadUnderpassFinder::crossings_of(geom::PointMm a, geom::PointMm b, std::int64_t a_id, std::int64_t b_id, std::int64_t upper_a,
    std::int64_t upper_b, int upper_layer, std::vector<Seg>& candidates, std::vector<Crossing>& out, RoadUnderpassStats& stats, bool over) const {
    candidates.clear();
    const auto x0 = floor_div(std::min(a.x, b.x), kCellMm), x1 = floor_div(std::max(a.x, b.x), kCellMm);
    const auto y0 = floor_div(std::min(a.y, b.y), kCellMm), y1 = floor_div(std::max(a.y, b.y), kCellMm);
    for (auto y = y0; y <= y1; ++y) for (auto x = x0; x <= x1; ++x) {
        const auto key = cell_key(x, y);
        auto it = std::lower_bound(cells_.begin(), cells_.end(), key, [](const auto& e, std::uint64_t v) { return e.first < v; });
        for (; it != cells_.end() && it->first == key; ++it) candidates.push_back(it->second);
    }
    std::sort(candidates.begin(), candidates.end(), [](const Seg& l, const Seg& r) { return std::tie(l.way, l.index) < std::tie(r.way, r.index); });
    candidates.erase(std::unique(candidates.begin(), candidates.end(), [](const Seg& l, const Seg& r) { return l.way == r.way && l.index == r.index; }),
                     candidates.end());
    for (const Seg& c_seg : candidates) {
        const RoadGraphWay& lower = graph_.ways[c_seg.way];
        if (lower.osm_id == upper_a || lower.osm_id == upper_b) continue;
        const std::int64_t c_id = lower.node_ids[c_seg.index], d_id = lower.node_ids[c_seg.index + 1];
        if (c_id == a_id || c_id == b_id || d_id == a_id || d_id == b_id) continue;
        const RoadGraphNode* c = node_of(graph_, c_id);
        const RoadGraphNode* d = node_of(graph_, d_id);
        if (!c || !d || !geom::segments_intersect(a, b, c->position, d->position)) continue;
        // D64 (over): the crossing way is an upper bridge (on a higher layer, never a tunnel) - the stretch is the lower road.
        if (over) {
            if (!tag_is_yes(lower, "bridge") || tag_is_yes(lower, "tunnel") || layer_of(lower) <= upper_layer) continue;
            if (in_structure(lower)) { ++stats.in_structure; continue; }
        } else {
            if (tag_is_yes(lower, "bridge")) continue;
        }
        const bool tunnel = !over && tag_is_yes(lower, "tunnel");
        if (!over && !tunnel && layer_of(lower) >= upper_layer) continue;
        if (!over && in_structure(lower)) { ++stats.in_structure; continue; }   // D61
        Crossing x{}; x.way = lower.osm_id; x.covered = tunnel;
        intersection_m(a, b, c->position, d->position, x.px, x.py);
        x.dx = static_cast<double>(d->position.x - c->position.x) / 1000; x.dy = static_cast<double>(d->position.y - c->position.y) / 1000;
        const auto lower_attrs = resolve_road_attributes(lower);
        x.width_mm = lower_attrs ? lower_attrs->width_mm : 3000;
        out.push_back(x);
    }
}

std::optional<RoadUnderpassSpanFit> RoadUnderpassFinder::fit_crossing(const Crossing& x, double centre, const std::vector<std::int32_t>& h,
    std::int32_t upper_width_mm, double ux, double uy, RoadUnderpassStats& stats) const {
    const int W = rules_.window_m;
    const std::int64_t R2 = 2 * static_cast<std::int64_t>(rules_.sag_radius_m);
    const double t = x.station - centre;   // 0 for a stretch's own crossings
    const int kc = static_cast<int>(std::llround(t));
    if (kc <= -W || kc >= W) { ++stats.no_rim; return std::nullopt; }
    // 3. The rims: upper hull of H(k) = h(k) 2R - 256 k^2 (h(k) without a sag radius), collinear points kept.
    const auto H = [&](int k) -> std::int64_t {
        const std::int64_t v = h[static_cast<std::size_t>(k + W)];
        return R2 > 0 ? v * R2 - 256 * static_cast<std::int64_t>(k) * k : v;
    };
    std::vector<int> hull;
    for (int k = -W; k <= W; ++k) {
        while (hull.size() >= 2) {
            const int o = hull[hull.size() - 2], q = hull.back();
            const std::int64_t cross = static_cast<std::int64_t>(q - o) * (H(k) - H(o)) - (H(q) - H(o)) * static_cast<std::int64_t>(k - o);
            if (cross > 0) hull.pop_back(); else break;
        }
        hull.push_back(k);
    }
    if (std::binary_search(hull.begin(), hull.end(), kc)) { ++stats.shallow; return std::nullopt; }
    const auto above = std::upper_bound(hull.begin(), hull.end(), kc);
    const int a = *(above - 1), b = *above;
    if (a == -W || b == W) { ++stats.no_rim; return std::nullopt; }
    // 4. Depth under the straight rim chord, 1/256 m.
    const double ha = h[static_cast<std::size_t>(a + W)], hb = h[static_cast<std::size_t>(b + W)];
    const auto depth = [&](int k) { return ha + (hb - ha) * static_cast<double>(k - a) / static_cast<double>(b - a) - h[static_cast<std::size_t>(k + W)]; };
    const double d0 = depth(kc);
    if (d0 * 1000.0 < static_cast<double>(rules_.min_depth_mm) * 256.0) { ++stats.shallow; return std::nullopt; }
    // 5. The deck: lower footprint united with the depression, abutments, clipped to the rims.
    const double tol = static_cast<double>(rules_.depth_tol_mm) * 256.0 / 1000.0;
    int r_lo = kc, r_hi = kc;
    while (r_lo - 1 > a && depth(r_lo - 1) > tol) --r_lo;
    while (r_hi + 1 < b && depth(r_hi + 1) > tol) ++r_hi;
    const double len = det::sqrt(x.dx * x.dx + x.dy * x.dy);
    const double sin_t = len > 0 ? std::fabs(ux * x.dy - uy * x.dx) / len : 1.0;
    const double cos_t = len > 0 ? std::fabs(ux * x.dx + uy * x.dy) / len : 0.0;
    const double f = (static_cast<double>(x.width_mm) / 2000.0 + static_cast<double>(rules_.lower_margin_mm) / 1000.0 +
                      static_cast<double>(upper_width_mm) / 2000.0 * cos_t) / std::max(sin_t, 0.25);
    const double abut = static_cast<double>(rules_.abutment_mm) / 1000.0;
    const double lo = std::max(static_cast<double>(a), std::min(static_cast<double>(r_lo), t - f) - abut);
    const double hi = std::min(static_cast<double>(b), std::max(static_cast<double>(r_hi), t + f) + abut);
    RoadUnderpassSpanFit fit;
    fit.start_m = static_cast<double>(std::llround((centre + lo) * 1000.0)) / 1000.0;
    fit.end_m = static_cast<double>(std::llround((centre + hi) * 1000.0)) / 1000.0;
    if (fit.end_m - fit.start_m > static_cast<double>(rules_.max_span_m)) { ++stats.too_long; return std::nullopt; }
    fit.depth_mm = static_cast<std::int32_t>(std::llround(d0 * 1000.0 / 256.0));
    fit.lower_ways = {x.way};
    fit.covered = x.covered;
    fit.chords.push_back({centre + a, centre + b, ha / 256.0, hb / 256.0});
    return fit;
}

std::vector<RoadUnderpassSpanFit> RoadUnderpassFinder::merge(std::vector<RoadUnderpassSpanFit> fits, RoadUnderpassStats& stats) {
    // 6. Merge overlapping decks.
    std::vector<RoadUnderpassSpanFit> out;
    std::sort(fits.begin(), fits.end(), [](const RoadUnderpassSpanFit& l, const RoadUnderpassSpanFit& r) {
        return std::tie(l.start_m, l.end_m, l.lower_ways) < std::tie(r.start_m, r.end_m, r.lower_ways);
    });
    for (auto& fit : fits) {
        if (!out.empty() && fit.start_m <= out.back().end_m) {
            auto& m = out.back();
            m.end_m = std::max(m.end_m, fit.end_m);
            m.depth_mm = std::max(m.depth_mm, fit.depth_mm);
            m.covered = m.covered && fit.covered;   // one open lower road makes the span a deck
            m.lower_ways.insert(m.lower_ways.end(), fit.lower_ways.begin(), fit.lower_ways.end());
            m.chords.insert(m.chords.end(), fit.chords.begin(), fit.chords.end());
            ++stats.merged;
            continue;
        }
        out.push_back(std::move(fit));
    }
    for (auto& m : out) {
        std::sort(m.lower_ways.begin(), m.lower_ways.end());
        m.lower_ways.erase(std::unique(m.lower_ways.begin(), m.lower_ways.end()), m.lower_ways.end());
    }
    return out;
}

std::optional<RoadUnderpassJoin> RoadUnderpassFinder::join(std::int64_t node) const {
    if (!rules_.enabled) return std::nullopt;
    // 1. Exactly two incident edges, of two different ground ways on one layer.
    auto it = std::lower_bound(incident_.begin(), incident_.end(), std::pair<std::int64_t, std::uint32_t>{node, 0});
    std::array<std::uint32_t, 2> edge{}; std::size_t n = 0;
    for (; it != incident_.end() && it->first == node; ++it) { if (n == 2) return std::nullopt; edge[n++] = it->second; }
    if (n != 2) return std::nullopt;
    const RoadGraphEdge& e0 = context_.edges[edge[0]];
    const RoadGraphEdge& e1 = context_.edges[edge[1]];
    if (e0.way_id == e1.way_id) return std::nullopt;
    const RoadGraphWay* ways[2] = {way_of(context_, e0.way_id), way_of(context_, e1.way_id)};
    std::int32_t widths[2] = {0, 0};
    for (int i = 0; i < 2; ++i) {
        if (!ways[i]) return std::nullopt;
        const auto attrs = resolve_road_attributes(*ways[i]);
        if (!attrs || attrs->bridge || attrs->tunnel || in_structure(*ways[i])) return std::nullopt;
        widths[i] = attrs->width_mm;
    }
    const int layer = layer_of(*ways[0]);
    if (layer_of(*ways[1]) != layer) return std::nullopt;
    const int W = rules_.window_m;
    // The node polylines away from the node, up to W along each.
    JoinSide sides[2];
    const RoadGraphEdge* edges[2] = {&e0, &e1};
    for (int i = 0; i < 2; ++i) {
        const RoadGraphEdge& e = *edges[i];
        const RoadGraphWay& way = *ways[i];
        const bool forward = e.from_node == node;
        std::ptrdiff_t k = static_cast<std::ptrdiff_t>(e.segment_index) + (forward ? 0 : 1);
        if (k < 0 || k >= static_cast<std::ptrdiff_t>(way.node_ids.size()) || way.node_ids[static_cast<std::size_t>(k)] != node) return std::nullopt;
        const std::ptrdiff_t step = forward ? 1 : -1;
        JoinSide& s = sides[i];
        double cum = 0;
        for (; k >= 0 && k < static_cast<std::ptrdiff_t>(way.node_ids.size()); k += step) {
            const RoadGraphNode* nd = node_of(context_, way.node_ids[static_cast<std::size_t>(k)]); if (!nd) return std::nullopt;
            if (!s.p.empty()) {
                const double dx = static_cast<double>(nd->position.x - s.p.back().x) / 1000, dy = static_cast<double>(nd->position.y - s.p.back().y) / 1000;
                cum += det::sqrt(dx * dx + dy * dy);
            }
            s.p.push_back(nd->position); s.id.push_back(nd->osm_id); s.cum.push_back(cum);
            if (cum >= W) break;
        }
        if (!(s.cum.back() > 0)) return std::nullopt;
    }
    // 2a. The lower crossings of the path within W of the node (before any DEM read: most joins have none).
    struct PathCrossing { Crossing x; std::int32_t upper_width_mm; double ux, uy; };
    std::vector<PathCrossing> found;
    std::vector<Crossing> scratch;
    std::vector<Seg> candidates;
    RoadUnderpassStats stats;
    for (int i = 0; i < 2; ++i) {
        const JoinSide& s = sides[i];
        const double sign = i == 0 ? -1.0 : 1.0;
        for (std::size_t k = 0; k + 1 < s.p.size() && s.cum[k] < W; ++k) {
            const double l = s.cum[k + 1] - s.cum[k];
            if (!(l > 0)) continue;
            scratch.clear();
            crossings_of(s.p[k], s.p[k + 1], s.id[k], s.id[k + 1], ways[0]->osm_id, ways[1]->osm_id, layer, candidates, scratch, stats);
            for (auto& x : scratch) {
                const double dx = x.px - static_cast<double>(s.p[k].x) / 1000, dy = x.py - static_cast<double>(s.p[k].y) / 1000;
                x.station = sign * (s.cum[k] + det::sqrt(dx * dx + dy * dy));
                if (!(std::fabs(x.station) < W)) continue;
                found.push_back({x, widths[i], static_cast<double>(s.p[k + 1].x - s.p[k].x) / 1000 / l, static_cast<double>(s.p[k + 1].y - s.p[k].y) / 1000 / l});
            }
        }
    }
    if (found.empty()) return std::nullopt;
    std::sort(found.begin(), found.end(), [](const PathCrossing& l, const PathCrossing& r) {
        return std::tie(l.x.station, l.x.way, l.x.dx, l.x.dy) < std::tie(r.x.station, r.x.way, r.x.dx, r.x.dy);
    });
    // 2b. The window around the node: p < 0 along the first way, p > 0 along the second.
    std::vector<std::int32_t> h(static_cast<std::size_t>(2 * W + 1));
    const auto frame_at = [&](double p) { return p < 0 ? side_frame(sides[0], -p) : side_frame(sides[1], p); };
    if (!sample_window(dem_, zone_, W, 0.0, frame_at, h)) return std::nullopt;
    std::vector<RoadUnderpassSpanFit> fits;
    for (const auto& c : found)
        if (auto fit = fit_crossing(c.x, 0.0, h, c.upper_width_mm, c.ux, c.uy, stats)) fits.push_back(std::move(*fit));
    const auto merged = merge(std::move(fits), stats);
    // 3. The spans reaching the node: a covered one is the support, a deck leaves the node unsupported.
    const double clear = static_cast<double>(rules_.end_clearance_mm) / 1000.0;
    RoadUnderpassJoin out;
    out.node = node; out.negative_way = ways[0]->osm_id; out.positive_way = ways[1]->osm_id;
    bool any_chord = false; double z0 = 0, slope = 0;
    for (const auto& m : merged) {
        if (m.end_m - m.start_m > static_cast<double>(rules_.max_span_m)) continue;   // a merged span outgrown the limit
        if (!(m.start_m - clear <= 0 && m.end_m + clear >= 0)) continue;
        bool spans_node = false;
        for (const auto& c : m.chords) {
            if (!(c.from_m <= 0 && c.to_m >= 0 && c.to_m > c.from_m)) continue;   // the node may be a rim itself
            spans_node = true;
            const double s = (c.to_height_m - c.from_height_m) / (c.to_m - c.from_m);
            const double z = c.from_height_m + s * (0 - c.from_m);
            if (m.covered && (!any_chord || z > z0)) { z0 = z; slope = s; any_chord = true; }
        }
        if (!spans_node) continue;
        if (!m.covered) return std::nullopt;
        out.spans.push_back(m);
    }
    if (out.spans.empty() || !any_chord) return std::nullopt;
    // The +p direction at the node: the shared-bisector heading of the reference lines' degree-two join (road_reference).
    const auto first_away = [](const JoinSide& s) {
        std::size_t k = 1; while (k + 1 < s.p.size() && !(s.cum[k] > 0)) ++k; return k;
    };
    const geom::PointMm here = sides[0].p.front();
    const geom::PointMm before = sides[0].p[first_away(sides[0])], after = sides[1].p[first_away(sides[1])];
    const auto dir = [](geom::PointMm a, geom::PointMm b, double& x, double& y, double& l) {
        const double dx = static_cast<double>(b.x - a.x) / 1000, dy = static_cast<double>(b.y - a.y) / 1000;
        l = det::sqrt(dx * dx + dy * dy); x = dx / l; y = dy / l;
    };
    double ix = 0, iy = 0, il = 0, ox = 0, oy = 0, ol = 0;
    dir(before, here, ix, iy, il); dir(here, after, ox, oy, ol);
    if ((ix + ox) * (ix + ox) + (iy + oy) * (iy + oy) < 1e-12) return std::nullopt;
    const double heading = det::atan2(iy, ix) + det::atan2(ix * oy - iy * ox, ix * ox + iy * oy) * il / (il + ol);
    out.ux = det::cos(heading); out.uy = det::sin(heading);
    out.plane.height = static_cast<std::int32_t>(std::llround(z0 * 256.0));
    out.plane.grade_x = slope * out.ux; out.plane.grade_y = slope * out.uy;
    return out;
}

std::vector<RoadUnderpassSpanFit> RoadUnderpassFinder::spans(std::int64_t way_id, RoadStretch stretch, const RoadReferenceLine& ref,
    RoadUnderpassStats* stats) const {
    std::vector<RoadUnderpassSpanFit> out;
    if (!rules_.enabled) return out;
    const RoadGraphWay* upper = way_of(graph_, way_id);
    if (!upper || stretch.end_ref >= upper->node_ids.size() || stretch.start_ref >= stretch.end_ref) return out;
    const auto upper_attrs = resolve_road_attributes(*upper);
    if (!upper_attrs || upper_attrs->bridge || upper_attrs->tunnel) return out;
    RoadUnderpassStats local; if (!stats) stats = &local;
    if (in_structure(*upper)) { ++stats->in_structure; return out; }   // D61
    ++stats->entries;
    const int upper_layer = layer_of(*upper);
    const double L = ref.length_m;
    const int W = rules_.window_m;

    // D63: the way-boundary supports at the stretch's end nodes; station of a support point p = s_node + sigma p.
    struct JoinEnd { RoadUnderpassJoin join; double s_node; double sigma; };
    std::vector<JoinEnd> ends;
    for (const bool at_start : {true, false}) {
        const std::int64_t node = upper->node_ids[at_start ? stretch.start_ref : stretch.end_ref];
        if (!at_start && !ends.empty() && ends.front().join.node == node) continue;
        auto j = join(node);
        if (!j || (j->negative_way != way_id && j->positive_way != way_id)) continue;
        const double sigma = (j->positive_way == way_id ? 1.0 : -1.0) * (at_start ? 1.0 : -1.0);
        ends.push_back({std::move(*j), at_start ? 0.0 : L, sigma});
    }
    const auto joined = [&](const Crossing& x) {
        for (const auto& e : ends) {
            if (std::fabs(x.station - e.s_node) > W) continue;
            for (const auto& span : e.join.spans)
                if (std::binary_search(span.lower_ways.begin(), span.lower_ways.end(), x.way)) return true;
        }
        return false;
    };

    // The lower crossings of the stretch's node polyline.
    std::vector<Crossing> found;
    std::vector<Seg> candidates;
    for (std::uint32_t i = stretch.start_ref; i < stretch.end_ref; ++i) {
        const RoadGraphNode* a = node_of(graph_, upper->node_ids[i]);
        const RoadGraphNode* b = node_of(graph_, upper->node_ids[i + 1]);
        if (!a || !b) continue;
        const std::size_t before = found.size();
        crossings_of(a->position, b->position, a->osm_id, b->osm_id, upper->osm_id, upper->osm_id, upper_layer, candidates, found, *stats);
        for (std::size_t k = before; k < found.size(); ++k) found[k].station = nearest_station(ref, found[k].px, found[k].py);
    }
    std::sort(found.begin(), found.end(), [](const Crossing& l, const Crossing& r) { return std::tie(l.station, l.way, l.dx, l.dy) < std::tie(r.station, r.way, r.dx, r.dy); });

    std::vector<RoadUnderpassSpanFit> fits;
    std::vector<std::int32_t> h(static_cast<std::size_t>(2 * W + 1));
    const auto frame_at = [&](double s) -> std::optional<Frame> {
        const auto pose = pose_at(ref, s); if (!pose) return std::nullopt;
        return Frame{pose->x, pose->y, -det::sin(pose->heading), det::cos(pose->heading)};
    };
    for (const Crossing& x : found) {
        if (joined(x)) { ++stats->joined; continue; }   // D63: the end node's support serves it
        ++stats->crossings;
        if (!sample_window(dem_, zone_, W, x.station, frame_at, h)) { ++stats->dem_missing; continue; }
        const auto pose = ref.at(std::clamp(x.station, 0.0, L));
        if (!pose) { ++stats->dem_missing; continue; }
        auto fit = fit_crossing(x, x.station, h, upper_attrs->width_mm, det::cos(pose->heading), det::sin(pose->heading), *stats);
        if (!fit) continue;
        const double clear = static_cast<double>(rules_.end_clearance_mm) / 1000.0;
        if (fit->start_m < clear || fit->end_m > L - clear) { ++stats->reaches_end; continue; }
        ++stats->served;
        fits.push_back(std::move(*fit));
    }
    out = merge(std::move(fits), *stats);
    // A merged deck may have outgrown the span limit.
    std::erase_if(out, [&](const RoadUnderpassSpanFit& m) {
        if (m.end_m - m.start_m <= static_cast<double>(rules_.max_span_m)) return false;
        ++stats->too_long; return true;
    });
    for (const auto& m : out) { if (m.covered) ++stats->covered; else ++stats->decks; }
    // D63: the support spans of the end nodes on this stretch's side, in its stations (covered: masked, never a deck).
    for (const auto& e : ends) {
        for (const auto& span : e.join.spans) {
            RoadUnderpassSpanFit m = span;
            const auto station = [&](double p) { return e.s_node + e.sigma * p; };
            const double s0 = station(span.start_m), s1 = station(span.end_m);
            m.start_m = static_cast<double>(std::llround(std::max(0.0, std::min(s0, s1)) * 1000.0)) / 1000.0;
            m.end_m = static_cast<double>(std::llround(std::min(L, std::max(s0, s1)) * 1000.0)) / 1000.0;
            if (!(m.end_m > m.start_m)) continue;
            for (auto& c : m.chords) {
                const double f = station(c.from_m), t = station(c.to_m);
                if (e.sigma > 0) { c.from_m = f; c.to_m = t; }
                else { c.from_m = t; c.to_m = f; std::swap(c.from_height_m, c.to_height_m); }
            }
            ++stats->join_spans;
            out.push_back(std::move(m));
        }
    }
    // D64: the hump spans under upper bridges (covered: mask only).
    auto over = overpass_spans(way_id, stretch, ref, *stats);
    const bool sort_out = !ends.empty() || !over.empty();
    for (auto& m : over) out.push_back(std::move(m));
    if (sort_out)
        std::sort(out.begin(), out.end(), [](const RoadUnderpassSpanFit& l, const RoadUnderpassSpanFit& r) {
            return std::tie(l.start_m, l.end_m, l.covered, l.lower_ways) < std::tie(r.start_m, r.end_m, r.covered, r.lower_ways);
        });
    return out;
}

std::vector<RoadUnderpassSpanFit> RoadUnderpassFinder::overpass_spans(std::int64_t way_id, RoadStretch stretch, const RoadReferenceLine& ref,
    RoadUnderpassStats& stats) const {
    std::vector<RoadUnderpassSpanFit> out;
    if (!rules_.enabled || !rules_.overpass) return out;
    const RoadGraphWay* lower = way_of(graph_, way_id);
    if (!lower || stretch.end_ref >= lower->node_ids.size() || stretch.start_ref >= stretch.end_ref) return out;
    const auto attrs = resolve_road_attributes(*lower);
    if (!attrs || attrs->bridge || attrs->tunnel || in_structure(*lower)) return out;
    const int layer = layer_of(*lower);
    const double L = ref.length_m;
    const int W = rules_.window_m;
    std::vector<Crossing> found;
    std::vector<Seg> candidates;
    for (std::uint32_t i = stretch.start_ref; i < stretch.end_ref; ++i) {
        const RoadGraphNode* a = node_of(graph_, lower->node_ids[i]);
        const RoadGraphNode* b = node_of(graph_, lower->node_ids[i + 1]);
        if (!a || !b) continue;
        const std::size_t before = found.size();
        crossings_of(a->position, b->position, a->osm_id, b->osm_id, lower->osm_id, lower->osm_id, layer, candidates, found, stats, true);
        for (std::size_t k = before; k < found.size(); ++k) found[k].station = nearest_station(ref, found[k].px, found[k].py);
    }
    if (found.empty()) return out;
    std::sort(found.begin(), found.end(), [](const Crossing& l, const Crossing& r) { return std::tie(l.station, l.way, l.dx, l.dy) < std::tie(r.station, r.way, r.dx, r.dy); });
    std::vector<std::int32_t> h(static_cast<std::size_t>(2 * W + 1));
    const auto frame_at = [&](double s) -> std::optional<Frame> {
        const auto pose = pose_at(ref, s); if (!pose) return std::nullopt;
        return Frame{pose->x, pose->y, -det::sin(pose->heading), det::cos(pose->heading)};
    };
    std::vector<RoadUnderpassSpanFit> fits;
    for (const Crossing& x : found) {
        ++stats.over_crossings;
        if (!sample_window(dem_, zone_, W, x.station, frame_at, h)) { ++stats.dem_missing; continue; }
        const auto pose = ref.at(std::clamp(x.station, 0.0, L));
        if (!pose) { ++stats.dem_missing; continue; }
        // The hump is the depression of the negated window: its rims, depth and extent come from the D55 steps unchanged.
        for (auto& v : h) v = -v;
        auto fit = fit_crossing(x, x.station, h, attrs->width_mm, det::cos(pose->heading), det::sin(pose->heading), stats);
        if (!fit) continue;
        for (auto& c : fit->chords) { c.from_height_m = -c.from_height_m; c.to_height_m = -c.to_height_m; }
        fit->covered = true;   // mask only: the bridge is not a deck of this road
        // The span ends where the stretch does (the hump may well reach past its end node).
        fit->start_m = std::max(0.0, fit->start_m);
        fit->end_m = std::min(L, fit->end_m);
        if (!(fit->end_m > fit->start_m)) continue;
        ++stats.over_masked;
        fits.push_back(std::move(*fit));
    }
    out = merge(std::move(fits), stats);
    std::erase_if(out, [&](const RoadUnderpassSpanFit& m) {
        if (m.end_m - m.start_m <= static_cast<double>(rules_.max_span_m)) return false;
        ++stats.too_long; return true;
    });
    return out;
}

std::optional<RoadJunctionPlane> RoadUnderpassFinder::overpass_end_plane(std::int64_t way_id, RoadStretch stretch,
    const RoadReferenceLine& ref, bool at_start, RoadUnderpassStats* stats) const {
    RoadUnderpassStats local; if (!stats) stats = &local;
    const double L = ref.length_m, s = at_start ? 0.0 : L;
    const double clear = static_cast<double>(rules_.end_clearance_mm) / 1000.0;
    const auto pose = ref.at(s); if (!pose) return std::nullopt;
    bool any = false; double z = 0, slope = 0;
    for (const auto& m : overpass_spans(way_id, stretch, ref, *stats)) {
        if (at_start ? !(m.start_m <= clear) : !(m.end_m >= L - clear)) continue;
        for (const auto& c : m.chords) {
            if (!(c.from_m <= s && c.to_m >= s && c.to_m > c.from_m)) continue;
            const double g = (c.to_height_m - c.from_height_m) / (c.to_m - c.from_m);
            const double v = c.from_height_m + g * (s - c.from_m);
            if (!any || v > z) { z = v; slope = g; any = true; }
        }
    }
    if (!any) return std::nullopt;
    RoadJunctionPlane plane;
    plane.height = static_cast<std::int32_t>(std::llround(z * 256.0));
    plane.grade_x = slope * det::cos(pose->heading); plane.grade_y = slope * det::sin(pose->heading);
    return plane;
}

void RoadUnderpassFinder::mask(const std::vector<RoadUnderpassSpanFit>& spans, std::vector<VerticalSample>& samples) {
    if (spans.empty() || samples.size() < 2) return;
    const auto chord_at = [](const RoadUnderpassSpanFit& span, double s, double fallback) {
        bool any = false; double z = 0;
        for (const auto& c : span.chords) {
            if (s < c.from_m || s > c.to_m || !(c.to_m > c.from_m)) continue;
            const double v = c.from_height_m + (c.to_height_m - c.from_height_m) * (s - c.from_m) / (c.to_m - c.from_m);
            if (!any || v > z) { z = v; any = true; }
        }
        return any ? z : fallback;
    };
    for (const auto& span : spans) {
        bool interior = false;
        for (std::size_t i = 1; i + 1 < samples.size(); ++i) {
            auto& sample = samples[i];
            if (sample.station_m < span.start_m || sample.station_m > span.end_m) continue;
            sample.height_m = chord_at(span, sample.station_m, sample.height_m); interior = true;
        }
        if (interior) continue;
        const double mid = 0.5 * (span.start_m + span.end_m);
        const auto it = std::upper_bound(samples.begin(), samples.end(), mid, [](double v, const VerticalSample& x) { return v < x.station_m; });
        if (it == samples.begin() || it == samples.end()) continue;
        if (mid - (it - 1)->station_m < 0.5 || it->station_m - mid < 0.5) continue;
        samples.insert(it, VerticalSample{mid, chord_at(span, mid, 0.5 * ((it - 1)->height_m + it->height_m))});
    }
}

void RoadUnderpassFinder::mask_samples(std::int64_t way_id, RoadStretch stretch, const RoadReferenceLine& reference,
    std::vector<VerticalSample>& samples) const {
    mask(spans(way_id, stretch, reference), samples);
}

Result<std::vector<InputTileRef>> road_crossing_dependencies(const RoadGraph& tile_graph, geo::UtmZone zone, const TileKey* exclude,
    std::size_t max_tiles, std::int64_t join_margin_mm) {
    TileKey key{zone, 2, 0, 0};
    if (!key.valid() || !max_tiles || max_tiles > 4096) return make_error(ErrorCode::InvalidArgument, "road crossings: invalid zone or budget");
    if (join_margin_mm < 0 || join_margin_mm > 1000000) return make_error(ErrorCode::InvalidArgument, "road crossings: invalid join margin");
    const std::int64_t size = key.size_m() * 1000;
    std::set<TileKey> keys;
    for (const auto& way : tile_graph.ways) {
        for (std::size_t k = 0; k + 1 < way.node_ids.size(); ++k) {
            const RoadGraphNode* a = node_of(tile_graph, way.node_ids[k]);
            const RoadGraphNode* b = node_of(tile_graph, way.node_ids[k + 1]);
            if (!a || !b) return make_error(ErrorCode::Malformed, "road crossings: way node absent");
            const auto x0 = floor_div(std::min(a->position.x, b->position.x) - 1, size), x1 = floor_div(std::max(a->position.x, b->position.x), size);
            const auto y0 = floor_div(std::min(a->position.y, b->position.y) - 1, size), y1 = floor_div(std::max(a->position.y, b->position.y), size);
            if (x0 < TileKey::kIndexMin || x1 > TileKey::kIndexMax || y0 < TileKey::kIndexMin || y1 > TileKey::kIndexMax)
                return make_error(ErrorCode::LimitExceeded, "road crossings: coordinates outside tile grid");
            for (auto y = y0; y <= y1; ++y) for (auto x = x0; x <= x1; ++x) {
                if (!geom::segment_touches_box(a->position.x, a->position.y, b->position.x, b->position.y, x * size, y * size, (x + 1) * size, (y + 1) * size))
                    continue;
                key.x = static_cast<std::int32_t>(x); key.y = static_cast<std::int32_t>(y);
                if (exclude && key == *exclude) continue;
                keys.insert(key);
                if (keys.size() > max_tiles) return make_error(ErrorCode::LimitExceeded, "road crossings: tile dependency budget");
            }
        }
    }
    // D63: the tiles touching the square of half-width join_margin_mm around each tile way's first and last node.
    if (join_margin_mm > 0) for (const auto& way : tile_graph.ways) {
        if (way.node_ids.empty()) continue;
        for (const std::int64_t id : {way.node_ids.front(), way.node_ids.back()}) {
            const RoadGraphNode* n = node_of(tile_graph, id);
            if (!n) return make_error(ErrorCode::Malformed, "road crossings: way node absent");
            const auto x0 = floor_div(n->position.x - join_margin_mm, size), x1 = floor_div(n->position.x + join_margin_mm, size);
            const auto y0 = floor_div(n->position.y - join_margin_mm, size), y1 = floor_div(n->position.y + join_margin_mm, size);
            if (x0 < TileKey::kIndexMin || x1 > TileKey::kIndexMax || y0 < TileKey::kIndexMin || y1 > TileKey::kIndexMax)
                return make_error(ErrorCode::LimitExceeded, "road crossings: coordinates outside tile grid");
            for (auto y = y0; y <= y1; ++y) for (auto x = x0; x <= x1; ++x) {
                key.x = static_cast<std::int32_t>(x); key.y = static_cast<std::int32_t>(y);
                if (exclude && key == *exclude) continue;
                keys.insert(key);
                if (keys.size() > max_tiles) return make_error(ErrorCode::LimitExceeded, "road crossings: tile dependency budget");
            }
        }
    }
    std::vector<InputTileRef> out; out.reserve(keys.size());
    for (const auto& k : keys) out.push_back({std::string(kRoadGraphLayer), k});
    return out;
}
} // namespace g2m
