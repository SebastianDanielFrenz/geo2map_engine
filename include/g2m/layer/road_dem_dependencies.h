#pragma once
#include "g2m/layer/road_graph.h"
#include "g2m/layer/deriver.h"
namespace g2m {
// All level-0 elev.base tiles intersecting complete road segments expanded
// by margin_mm, sorted and unique. Includes geometry outside the graph tile.
// The default 65 m covers the maximum 100 m road width plus 15 m blend.
// Enforces a bounded DEM working set; never returns a silently clipped list.
Result<std::vector<InputTileRef>> road_dem_dependencies(const RoadGraph& graph,
    geo::UtmZone zone, std::int64_t margin_mm = 65000, std::size_t max_tiles = 1024);
}
