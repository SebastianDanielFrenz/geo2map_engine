// src/route/route_map.cpp - see g2m/route/route_map.h.
#include "g2m/route/route_map.h"

#include "g2m/core/det_math.h"
#include "g2m/layer/osm_roads.h"

#include <algorithm>
#include <cmath>
#include <string_view>
#include <utility>

namespace g2m::route {
namespace {

std::string_view tag_value(const RoadGraphWay& way, std::string_view key) {
    for (const auto& [k, v] : way.tags) if (k == key) return v;
    return {};
}

bool inside(TileKey tile, const TileKey& key) {
    if (tile.zone != key.zone || tile.level > key.level) return false;
    while (tile.level < key.level) tile = tile.parent();
    return tile == key;
}

const RoadGraphNode* find_node(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.nodes.begin(), g.nodes.end(), id,
                                     [](const RoadGraphNode& n, std::int64_t x) { return n.osm_id < x; });
    return it != g.nodes.end() && it->osm_id == id ? &*it : nullptr;
}

const RoadGraphWay* find_way(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.ways.begin(), g.ways.end(), id,
                                     [](const RoadGraphWay& w, std::int64_t x) { return w.osm_id < x; });
    return it != g.ways.end() && it->osm_id == id ? &*it : nullptr;
}

std::uint8_t line_flags(const RoadGraphWay& way, const RouteConfig& directions) {
    std::uint8_t flags = 0;
    const WayDirections d = way_directions(way, directions);
    if (d.forward && !d.backward) flags |= kMapOneway;
    if (!d.forward && d.backward) flags |= kMapOnewayReverse;
    if (!d.forward && !d.backward) flags |= kMapNoCars;
    const auto bridge = tag_value(way, "bridge");
    if (!bridge.empty() && bridge != "no") flags |= kMapBridge;
    const auto tunnel = tag_value(way, "tunnel");
    if (!tunnel.empty() && tunnel != "no") flags |= kMapTunnel;
    if (tag_value(way, "highway").ends_with("_link")) flags |= kMapLink;
    const auto junction = tag_value(way, "junction");
    if (junction == "roundabout" || junction == "circular") flags |= kMapRoundabout;
    return flags;
}

// Squared distance from p to the segment a-b (mm^2, double; exact products
// for coordinate differences below 2^26 mm).
double segment_distance2(const geom::PointMm& p, const geom::PointMm& a, const geom::PointMm& b) {
    const double vx = static_cast<double>(b.x - a.x), vy = static_cast<double>(b.y - a.y);
    const double wx = static_cast<double>(p.x - a.x), wy = static_cast<double>(p.y - a.y);
    const double len2 = vx * vx + vy * vy;
    const double t = vx * wx + vy * wy;
    if (len2 == 0.0 || t <= 0.0) return wx * wx + wy * wy;
    if (t >= len2) {
        const double ux = static_cast<double>(p.x - b.x), uy = static_cast<double>(p.y - b.y);
        return ux * ux + uy * uy;
    }
    const double cross = vx * wy - vy * wx;
    return cross * cross / len2;
}

std::uint64_t ceil_length_mm(const geom::PointMm& a, const geom::PointMm& b) {
    const double dx = static_cast<double>(b.x - a.x), dy = static_cast<double>(b.y - a.y);
    return static_cast<std::uint64_t>(std::ceil(det::sqrt(dx * dx + dy * dy)));
}

struct Item {
    std::int64_t way_id;
    std::uint32_t seg;
    std::int64_t from_id, to_id;
    geom::PointMm from, to;
    const RoadGraphWay* way;
};

} // namespace

const char* to_string(MapLod lod) {
    switch (lod) {
    case MapLod::Detail: return "detail";
    case MapLod::Mid: return "mid";
    case MapLod::Far: return "far";
    }
    return "?";
}

const MapConfig& default_map_config() {
    static const MapConfig config = [] {
        const RoadStyle& style = RoadStyle::default_style();
        const auto rank_of = [&](std::string_view highway, int fallback) {
            const HighwayStyleEntry* e = style.find_highway(highway);
            return e != nullptr ? e->rank : fallback;
        };
        MapConfig c;
        c.lods[0] = {2, 0, 1000};
        c.lods[1] = {4, rank_of("tertiary", 5), 8000};
        c.lods[2] = {6, rank_of("primary", 7), 40000};
        return c;
    }();
    return config;
}

std::size_t MapTile::byte_size() const {
    std::size_t bytes = lines.size() * sizeof(MapLine) + points.size() * sizeof(MapPoint);
    for (const auto& s : strings) bytes += s.size();
    return bytes;
}

std::vector<geom::PointMm> simplify_polyline(std::span<const geom::PointMm> points, std::int64_t tolerance_mm) {
    if (points.size() <= 2) return {points.begin(), points.end()};
    const double tol = static_cast<double>(std::max<std::int64_t>(tolerance_mm, 0));
    const double tol2 = tol * tol;
    std::vector<bool> keep(points.size(), false);
    keep.front() = keep.back() = true;
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, points.size() - 1}};
    while (!stack.empty()) {
        const auto [i, j] = stack.back();
        stack.pop_back();
        if (j <= i + 1) continue;
        double best = -1.0;
        std::size_t at = i;
        for (std::size_t k = i + 1; k < j; ++k) {
            const double d2 = segment_distance2(points[k], points[i], points[j]);
            if (d2 > best) { best = d2; at = k; }
        }
        if (best > tol2) {
            keep[at] = true;
            stack.push_back({at, j});
            stack.push_back({i, at});
        }
    }
    std::vector<geom::PointMm> out;
    for (std::size_t k = 0; k < points.size(); ++k) if (keep[k]) out.push_back(points[k]);
    return out;
}

Result<MapTile> build_map_tile(const TileKey& key, MapLod lod, std::span<const RoadGraphTile* const> tiles,
                               const MapConfig& config, MapTileStats* stats_out) {
    const auto lod_index = static_cast<std::size_t>(lod);
    if (lod_index >= kMapLodCount) return make_error(ErrorCode::InvalidArgument, "map tile: unknown LOD");
    const MapLodConfig& lc = config.lods[lod_index];
    if (!key.valid() || key.level != lc.level)
        return make_error(ErrorCode::InvalidArgument, "map tile: key level does not match the LOD level");
    MapTileStats stats;
    std::vector<Item> items;
    for (const RoadGraphTile* tile : tiles) {
        if (tile == nullptr) continue;
        if (tile->key.level != 2) return make_error(ErrorCode::InvalidArgument, "map tile: source tile is not level 2");
        if (!inside(tile->key, key)) continue;
        ++stats.source_tiles;
        const RoadGraph& g = tile->graph;
        for (const std::uint32_t index : tile->owned_edges) {
            if (index >= g.edges.size()) return make_error(ErrorCode::Malformed, "map tile: owned edge out of range");
            ++stats.owned_edges;
            const RoadGraphEdge& e = g.edges[index];
            const RoadGraphWay* way = find_way(g, e.way_id);
            if (way == nullptr) return make_error(ErrorCode::Malformed, "map tile: way of an owned edge missing");
            if (way->rank < lc.min_rank) continue;
            const RoadGraphNode* a = find_node(g, e.from_node);
            const RoadGraphNode* b = find_node(g, e.to_node);
            if (a == nullptr || b == nullptr) return make_error(ErrorCode::Malformed, "map tile: node of an owned edge missing");
            items.push_back({e.way_id, e.segment_index, e.from_node, e.to_node, a->position, b->position, way});
        }
    }
    std::sort(items.begin(), items.end(), [](const Item& x, const Item& y) {
        return x.way_id != y.way_id ? x.way_id < y.way_id : x.seg < y.seg;
    });
    for (std::size_t k = 1; k < items.size(); ++k)
        if (items[k].way_id == items[k - 1].way_id && items[k].seg == items[k - 1].seg)
            return make_error(ErrorCode::Malformed, "map tile: an edge is owned by two tiles");
    stats.drawn_edges = items.size();

    const std::int64_t sw_x = key.min_easting() * 1000, sw_y = key.min_northing() * 1000;
    const auto quantize = [](std::int64_t mm, std::int64_t origin) {
        return static_cast<std::int32_t>(floor_div(mm - origin + 50, 100));
    };
    struct Pending {
        MapLine line;
        std::string_view name, ref;
    };
    std::vector<Pending> pending;
    MapTile out;
    out.key = key;
    out.lod = lod;
    std::vector<geom::PointMm> run;
    std::size_t k = 0;
    while (k < items.size()) {
        std::size_t end = k + 1;
        while (end < items.size() && items[end].way_id == items[k].way_id && items[end].from_id == items[end - 1].to_id) ++end;
        run.clear();
        run.push_back(items[k].from);
        for (std::size_t m = k; m < end; ++m) run.push_back(items[m].to);
        stats.points_in += run.size();
        const auto simple = simplify_polyline(run, lc.tolerance_mm);
        const auto first = static_cast<std::uint32_t>(out.points.size());
        for (const auto& p : simple) {
            const MapPoint q{quantize(p.x, sw_x), quantize(p.y, sw_y)};
            if (out.points.size() > first && out.points.back() == q) continue;
            out.points.push_back(q);
        }
        const auto count = static_cast<std::uint32_t>(out.points.size() - first);
        if (count < 2) {
            out.points.resize(first);
            ++stats.collapsed;
            k = end;
            continue;
        }
        stats.points_out += count;
        if (out.points.size() > config.max_points) return make_error(ErrorCode::LimitExceeded, "map tile: too many points");
        Pending p;
        MapLine& line = p.line;
        line.way_id = items[k].way_id;
        line.seg_first = items[k].seg;
        line.seg_last = items[end - 1].seg;
        line.point_first = first;
        line.point_count = count;
        line.rank = static_cast<std::int8_t>(items[k].way->rank);
        line.flags = line_flags(*items[k].way, config.directions);
        std::int64_t best = -1;
        for (std::uint32_t m = first; m + 1 < first + count; ++m) {
            const std::int64_t dx = std::int64_t{out.points[m + 1].x} - out.points[m].x;
            const std::int64_t dy = std::int64_t{out.points[m + 1].y} - out.points[m].y;
            const std::int64_t len2 = dx * dx + dy * dy;
            if (len2 > best) {
                best = len2;
                line.anchor = {static_cast<std::int32_t>(floor_div(std::int64_t{out.points[m].x} + out.points[m + 1].x, 2)),
                               static_cast<std::int32_t>(floor_div(std::int64_t{out.points[m].y} + out.points[m + 1].y, 2))};
                line.anchor_dx = static_cast<std::int32_t>(dx);
                line.anchor_dy = static_cast<std::int32_t>(dy);
            }
        }
        p.name = tag_value(*items[k].way, "name");
        p.ref = tag_value(*items[k].way, "ref");
        pending.push_back(std::move(p));
        if (pending.size() > config.max_lines) return make_error(ErrorCode::LimitExceeded, "map tile: too many lines");
        k = end;
    }
    for (const auto& p : pending) {
        if (!p.name.empty()) out.strings.emplace_back(p.name);
        if (!p.ref.empty()) out.strings.emplace_back(p.ref);
    }
    std::sort(out.strings.begin(), out.strings.end());
    out.strings.erase(std::unique(out.strings.begin(), out.strings.end()), out.strings.end());
    const auto index_of = [&](std::string_view s) {
        if (s.empty()) return kMapNoString;
        const auto it = std::lower_bound(out.strings.begin(), out.strings.end(), s,
                                         [](const std::string& a, std::string_view b) { return std::string_view(a) < b; });
        return static_cast<std::uint32_t>(it - out.strings.begin());
    };
    out.lines.reserve(pending.size());
    for (auto& p : pending) {
        p.line.name = index_of(p.name);
        p.line.ref = index_of(p.ref);
        out.lines.push_back(p.line);
    }
    if (stats_out != nullptr) *stats_out = stats;
    return out;
}

std::vector<geom::PointMm> route_polyline(const RoutingGraph& graph, const Route& route) {
    std::vector<geom::PointMm> out;
    if (!route.reachable) return out;
    const auto arcs = graph.arcs();
    const std::size_t n = route.arcs.size();
    std::vector<geom::PointMm> piece;
    for (std::size_t i = 0; i < n; ++i) {
        const std::uint32_t arc = route.arcs[i];
        const RouteArc& r = arcs[arc];
        const std::uint64_t a = i == 0 ? route.start_offset_mm : 0;
        const std::uint64_t b = i + 1 == n ? route.end_offset_mm : r.length_mm;
        // The travelled range in way order.
        const std::uint64_t wa = r.forward ? a : r.length_mm - std::min<std::uint64_t>(b, r.length_mm);
        const std::uint64_t wb = r.forward ? b : r.length_mm - std::min<std::uint64_t>(a, r.length_mm);
        const auto pts = graph.shape(arc);
        piece.clear();
        piece.push_back(graph.point_at(arc, static_cast<std::uint32_t>(a)));
        std::vector<geom::PointMm> interior;
        std::uint64_t cum = 0;
        for (std::size_t k = 0; k + 2 < pts.size(); ++k) {
            cum += ceil_length_mm(pts[k], pts[k + 1]);
            if (cum > wa && cum < wb) interior.push_back(pts[k + 1]);
        }
        if (!r.forward) std::reverse(interior.begin(), interior.end());
        piece.insert(piece.end(), interior.begin(), interior.end());
        piece.push_back(graph.point_at(arc, static_cast<std::uint32_t>(b)));
        for (const auto& p : piece)
            if (out.empty() || !(out.back() == p)) out.push_back(p);
    }
    return out;
}

} // namespace g2m::route
