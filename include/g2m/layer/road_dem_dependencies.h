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
// Decision D35 (P2 design section 15): the strict pass's anchor at a bridge boundary node (surface_anchor) projects
// both raw anchors of the bridge span onto its feasible cubic rise, so it reads the span's FAR end: the ways around
// it (the 32 m walk along its ground approach) and the DEM within 35.5 m of it. A tile whose way ends at the near end
// need not hold the bridge, so its own DEM and context dependencies may miss both. This lists, for every bridge way
// of `context` (the merged stretch context of the tile, road_context_dependencies) incident to a node of
// `tile_graph`'s ways: the elev.base tiles of its segments widened by the same 65 m margin (road_dem_dependencies)
// and the roads.graph tiles of its nodes (road_context_dependencies, `exclude` omitted). Every way incident to a node
// within 32 m of the far end lies in the far end's roads.graph tile, and the walk's DEM reads lie within 65 m of the
// span's end. Sorted and unique; a way whose attributes do not resolve is not a bridge here (the anchor declines on
// it anyway). One hop only: the far end's own neighbours are complete in its tile, no chain is followed.
Result<std::vector<InputTileRef>> road_bridge_dependencies(const RoadGraph& tile_graph, const RoadGraph& context,
    geo::UtmZone zone, const TileKey* exclude = nullptr);
}
