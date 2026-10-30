// g2m/world/buildings.h - OSM building footprints of one level-2 tile (terrain migration plan phase 3b): the
// BuildingSet product. Ported from racing_game core/src/buildings.cpp (0c027dc / 5a3baed, unchanged since) statement
// for statement; the projection is g2m::geo::Utm (det math). Engine-neutral: the consumer tessellates and colours.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/src_osm.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace g2m::world {

struct BuildingPoint {
    double e, n; // absolute UTM metres
};
// One building (a closed way, or a multipolygon relation, tagged building=* or building:part=*, not "no"). Heights
// in metres: height from `height`, else building:levels x storey + roof (roof:height or roof:levels x storey), else the
// fallback; min_height from `min_height`, else building:min_level x storey. base = the highest ground sample under the
// footprint (centre and every outer vertex), bottom = base + min_height when min_height > 0, else the lowest sample - 1
// (the walls reach into sloping ground). Rings are open (the closing vertex dropped), as projected from OSM.
struct Building {
    std::int64_t id;
    bool relation = false;
    double height = 0, min_height = 0, base = 0, bottom = 0;
    std::string height_source, colour, type; // "height" | "building:levels" | "fallback"; building:colour; building=*
    // Appearance facts, separate from the unchanged footprint/height contract.
    std::string roof_shape, roof_colour, roof_material, roof_orientation, roof_direction, material;
    double roof_height = 0, levels = 0;
    std::vector<std::vector<BuildingPoint>> outers, inners;
};
// Ground height (absolute metres) at absolute UTM (e, n); nullopt = no ground there.
using BuildingGround = std::function<std::optional<double>(double, double)>;

// Every building of `tile` owned by `key` (the level-2 tile holding the centre of its outer bounding box, so a
// footprint in two tiles' halos is emitted once), in source order: relations first, then ways not used by a relation.
// Dropped: height below min_height, min_height >= height, a height outside (0, 2000] m, an outer ring with fewer than
// three projected vertices, a relation with a missing or unclosable member ring, and a footprint with no ground sample.
// Numbers accept a decimal comma and the suffix "m" (heights also "ft", "feet", "'"); anything else is not a number.
std::vector<Building> extract_buildings(const OsmTile& tile, const TileKey& key, double min_height, double fallback_height,
                                        double storey_height, const BuildingGround& ground);

// What a BuildingSet is built with (the game's settings: ordinary tiles 0 / fallback 8 / storey 3; skyline tiles a
// min_height of 50).
struct BuildingParams {
    double min_height_m = 0.0;
    double fallback_height_m = 8.0;
    double storey_height_m = 3.0;
};

// The BuildingSet product of one level-2 tile: extract_buildings of the tile's g2m.src.osm data.
struct BuildingSet {
    TileKey key; // level-2 tile
    std::vector<Building> buildings;
};
BuildingSet build_building_set(const OsmTile& tile, const TileKey& key, const BuildingParams& params, const BuildingGround& ground);

} // namespace g2m::world
