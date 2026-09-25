// g2m/layer/osm_clip.h - cuts one g2m.src.osm tile out of a larger OSM set
// (docs/formats/src_osm.md, "Selection contract").
//
// With B = the tile's square on its UTM grid, grown by halo_m on every side
// (a closed box, integer millimetres):
//  1. a node is IN if its projected position lies in B;
//  2. a way is included if any of its segments touches B (a one-node way:
//     if the node is IN); it keeps its full ref list;
//  3. the tile's nodes are the IN nodes plus every node any included way
//     refers to (so every way is complete and has its full geometry);
//  4. a relation is included if any member is an included node, way or
//     relation (closed transitively: a parent of an included relation is
//     included); its member list is kept complete, but members that are
//     not otherwise included are NOT added;
//  5. strings are re-interned to what the tile uses.
// Positions are OSM's 1e-7 degree values projected with g2m's own
// deterministic transverse Mercator (GRS80, the tile's zone) and floored to
// whole millimetres; all tests after that are exact integer arithmetic. A
// node more than 30 degrees of longitude from the zone's central meridian
// has no position ("far"): it is never IN, and a segment ending in it
// touches B only if its other end is IN.
//
// Deterministic: the output depends only on (set, key, halo_m).
#pragma once

#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_data.h"
#include "g2m/layer/src_osm.h"

namespace g2m {

inline constexpr std::uint32_t kDefaultOsmHaloM = 128;

// `set` must satisfy OsmData::validate() (in particular: ways complete).
Result<OsmTile> clip_osm(const osm::OsmData& set, const TileKey& key, std::uint32_t halo_m = kDefaultOsmHaloM);

// Exposed for tests: does segment (ax,ay)-(bx,by) touch the closed box
// [x0,x1] x [y0,y1]? Exact for all int64 inputs with |value| < 2^62.
bool segment_touches_box(std::int64_t ax, std::int64_t ay, std::int64_t bx, std::int64_t by, std::int64_t x0,
                         std::int64_t y0, std::int64_t x1, std::int64_t y1);

} // namespace g2m
