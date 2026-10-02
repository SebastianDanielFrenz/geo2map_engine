#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/terrain_layers.h"
#include <algorithm>
#include <set>
namespace g2m {
Result<std::vector<InputTileRef>> road_dem_dependencies(const RoadGraph& graph,
    geo::UtmZone zone, std::int64_t margin_mm, std::size_t max_tiles) {
    const auto valid = validate_road_graph(graph); if (!valid) return valid.error();
    TileKey key{zone, 0, 0, 0};
    if (!key.valid() || margin_mm < 0 || margin_mm > 100000 || !max_tiles || max_tiles > 4096)
        return make_error(ErrorCode::InvalidArgument, "road DEM: invalid zone, margin or budget");
    std::set<TileKey> keys;
    const auto node = [&](auto id) -> const geom::PointMm& {
        return std::lower_bound(graph.nodes.begin(), graph.nodes.end(), id,
            [](const auto& value, auto search) { return value.osm_id < search; })->position;
    };
    constexpr std::int64_t size = 256000;
    for (const auto& edge : graph.edges) {
        const auto& a = node(edge.from_node); const auto& b = node(edge.to_node);
        const auto x0 = floor_div(std::min(a.x, b.x) - margin_mm - 1, size);
        const auto x1 = floor_div(std::max(a.x, b.x) + margin_mm, size);
        const auto y0 = floor_div(std::min(a.y, b.y) - margin_mm - 1, size);
        const auto y1 = floor_div(std::max(a.y, b.y) + margin_mm, size);
        if (x0 < TileKey::kIndexMin || x1 > TileKey::kIndexMax ||
            y0 < TileKey::kIndexMin || y1 > TileKey::kIndexMax)
            return make_error(ErrorCode::LimitExceeded, "road DEM: coordinates outside tile grid");
        // Bound candidate work before iterating. Exact intersection below
        // avoids claiming a whole diagonal bounding box as DEM dependencies.
        if (x1 - x0 + 1 > 65536 || y1 - y0 + 1 > 65536 ||
            (x1 - x0 + 1) * (y1 - y0 + 1) > 65536)
            return make_error(ErrorCode::LimitExceeded, "road DEM: segment search budget");
        for (auto y = y0; y <= y1; ++y) for (auto x = x0; x <= x1; ++x) {
            if (!geom::segment_touches_box(a.x, a.y, b.x, b.y,
                x * size - margin_mm, y * size - margin_mm,
                (x + 1) * size + margin_mm, (y + 1) * size + margin_mm)) continue;
            key.x = static_cast<std::int32_t>(x); key.y = static_cast<std::int32_t>(y); keys.insert(key);
            if (keys.size() > max_tiles)
                return make_error(ErrorCode::LimitExceeded, "road DEM: tile working-set budget");
        }
    }
    std::vector<InputTileRef> out; out.reserve(keys.size());
    for (const auto& tile : keys) out.push_back({std::string(kElevBaseLayer), tile});
    return out;
}
}
