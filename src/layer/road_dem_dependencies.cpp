#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_attributes.h"
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
Result<std::vector<InputTileRef>> road_bridge_dependencies(const RoadGraph& tile_graph, const RoadGraph& context,
    geo::UtmZone zone, const TileKey* exclude, std::int64_t walk_margin_mm) {
    if (walk_margin_mm < 0 || walk_margin_mm > 1000000) return make_error(ErrorCode::InvalidArgument, "road bridge dependencies: invalid walk margin");
    const auto valid_tile = validate_road_graph(tile_graph); if (!valid_tile) return valid_tile.error();
    const auto valid_context = validate_road_graph(context); if (!valid_context) return valid_context.error();
    std::set<std::int64_t> tile_nodes;
    for (const auto& way : tile_graph.ways) tile_nodes.insert(way.node_ids.begin(), way.node_ids.end());
    // The bridge ways incident to a tile-way node, in way-id order (context.ways is sorted), as a canonical sub-graph.
    RoadGraph bridges; std::set<std::int64_t> used;
    for (const auto& way : context.ways) {
        const auto attributes = resolve_road_attributes(way);
        if (!attributes || !attributes->bridge) continue;
        if (std::none_of(way.node_ids.begin(), way.node_ids.end(), [&](auto id) { return tile_nodes.count(id) != 0; })) continue;
        bridges.ways.push_back(way); used.insert(way.node_ids.begin(), way.node_ids.end());
        for (std::size_t i = 1; i < way.node_ids.size(); ++i) {
            if (way.node_ids[i - 1] == way.node_ids[i]) { ++bridges.repeated_ref_pairs; continue; }
            bridges.edges.push_back({way.osm_id, static_cast<std::uint32_t>(i - 1), way.node_ids[i - 1], way.node_ids[i]});
        }
    }
    if (bridges.ways.empty()) return std::vector<InputTileRef>{};
    for (const auto& node : context.nodes) if (used.count(node.osm_id)) bridges.nodes.push_back(node);
    auto dem = road_dem_dependencies(bridges, zone); if (!dem) return dem.error();
    auto graph = road_context_dependencies(bridges, zone, exclude); if (!graph) return graph.error();
    dem->insert(dem->end(), graph->begin(), graph->end());
    if (walk_margin_mm > 0) {
        // D48: the squares the abutment walk from any node of a listed bridge can read (DEM cells and node tiles).
        std::set<TileKey> squares;
        const auto add = [&](const geom::PointMm& p, int level) -> Result<void> {
            TileKey key{zone, level, 0, 0}; const auto size = key.size_m() * 1000;
            const auto x0 = floor_div(p.x - walk_margin_mm - 1, size), x1 = floor_div(p.x + walk_margin_mm, size);
            const auto y0 = floor_div(p.y - walk_margin_mm - 1, size), y1 = floor_div(p.y + walk_margin_mm, size);
            if (x0 < TileKey::kIndexMin || x1 > TileKey::kIndexMax || y0 < TileKey::kIndexMin || y1 > TileKey::kIndexMax)
                return make_error(ErrorCode::LimitExceeded, "road bridge dependencies: coordinates outside tile grid");
            for (auto y = y0; y <= y1; ++y) for (auto x = x0; x <= x1; ++x) {
                key.x = static_cast<std::int32_t>(x); key.y = static_cast<std::int32_t>(y);
                if (level == 2 && exclude && key == *exclude) continue;
                squares.insert(key);
                if (squares.size() > 4096) return make_error(ErrorCode::LimitExceeded, "road bridge dependencies: walk tile budget");
            }
            return {};
        };
        for (const auto& node : bridges.nodes) {
            if (auto r = add(node.position, 0); !r) return r.error();
            if (auto r = add(node.position, 2); !r) return r.error();
        }
        for (const auto& key : squares) dem->push_back({std::string(key.level == 0 ? kElevBaseLayer : kRoadGraphLayer), key});
    }
    std::sort(dem->begin(), dem->end()); dem->erase(std::unique(dem->begin(), dem->end()), dem->end());
    return dem;
}
}
