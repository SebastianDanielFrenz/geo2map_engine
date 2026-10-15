// Street fidelity P2 slice 12, request G-6: see g2m/layer/road_underpass.h.
#include "g2m/layer/road_underpass.h"

#include "g2m/core/det_math.h"
#include "g2m/core/geom/exact.h"
#include "g2m/layer/road_attributes.h"

#include <algorithm>
#include <charconv>
#include <limits>
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
} // namespace g2m
