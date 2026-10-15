// g2m/world/road_ribbons.h - the road ribbon mesh of one level-2 tile (terrain migration plan phase 3a): ribbons per
// paved stretch at 1 m stations, draped on the ground or on a structure deck, lane/marking flags from the served
// cross-section and the tags, turn arrows, and (owner tile only) the served junctions: asphalt patches over the road bed
// rings cut on the terrain lattice, the gravel verge around them, the through road's markings continued over them,
// gores, turn guide lines and the stop, give-way and crossing lines. Ported from racing_game core/src/road_visual.cpp
// (0c027dc) with det math. Render only; colours and the GPU upload stay with the consumer. Engine-neutral.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/world/road_structures.h"

#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace g2m::world {

// flags (bits, read by the consumer's road shader): 1 lane markings, 2 edge lines, 4 solid separator, 8 fast road,
// 16 solid paint, 32 overlay (only the paint is drawn), 64 / 128 left / right edge line broken (a junction mouth),
// 512 / 1024 left / right edge line not drawn, 2048 paving stones (no painted lines), 4096 concrete, 8192 unpaved (a
// junction patch piece at a track's mouth, drawn like the terrain's dirt). Bit 256 is left free for the consumer (the
// game's adapter sets it for a verge vertex).
// x, y: metres from the tile's south-west corner; z: absolute height. u: lateral offset from the centre of the counted
// lanes, v: station, width: the counted lanes' width; verge 1 = the gravel strip beside the road bed (no asphalt, no
// paint).
struct RoadRibbonVertex {
    float x, y, z, u, v, width, fade, lanes, forward, flags, verge = 0;
};
// The RoadRibbonTile product: one mesh per level-2 tile (indices: triangles into `vertices`).
struct RoadRibbonTile {
    TileKey key;
    std::vector<RoadRibbonVertex> vertices;
    std::vector<std::int32_t> indices;
};
// Ground height (absolute metres) at absolute UTM (easting, northing); nullopt = no ground there (NoData, not loaded).
using RoadRibbonGround = std::function<std::optional<double>(double, double)>;

// The ribbons of `geometry` inside `key` (each 1 m row and each marking is kept by the tile holding its centre), lifted
// `lift_m` above the surface. Structure decks come from build_road_decks(geometry, separation); a stretch is drawn on a
// deck of its own (way, stretch start_ref) where one covers the station.
RoadRibbonTile build_road_ribbons(const RoadGeomTile& geometry, const TileKey& key, const RoadRibbonGround& ground, double lift_m,
                                  const RoadSeparation& separation = {});
// The junction patches alone (appended to `mesh`), in tile-local metres like build_road_ribbons.
void add_junction_surfaces(RoadRibbonTile& mesh, const RoadGeomTile& geometry, const TileKey& key, const RoadRibbonGround& ground, double lift_m);

// Priority rank of a road for right of way (StVO 18(3) / 8): motorway 14, trunk 12, primary 10, secondary 8,
// tertiary 6, unclassified / road 4, residential 2, living_street and anything else 0, every *_link 1. The ribbons pick
// the through road and the implied give-way lines with it.
int road_priority_rank(const RoadGraphWay& way);

} // namespace g2m::world
