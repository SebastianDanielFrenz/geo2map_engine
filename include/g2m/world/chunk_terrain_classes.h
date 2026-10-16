// g2m/world/chunk_terrain_classes.h - the render ClassWindow of one chunk read from the served g2m.terrain.class layer
// (street fidelity P2 slice 11, design section 10.1 "Render colour per vertex"; decision D62).
//
// The served alternative to chunk_road_classes.h: no OSM segments and no rasteriser run in the client, the class comes
// from the same ownership the carved heights and the physics grip use (CarveModel::classify on the server).
//
// Vertex to node: a render vertex (i, j) of chunk (L, cx, cy) sits at E = (64 cx + i) 2^L + 0.5 m (a cell centre, see
// mesh/terrain_chunk.h); a class node of level L sits at E = k 2^L m. The vertex takes the node at floor of its
// position, its south-west corner node: k = 64 cx + i (same for N). That node lies in the class tile
// (L, floor(cx / 4), floor(cy / 4)) at local index 64 (cx mod 4) + i in 0..256, so a chunk reads exactly one class
// tile (its east / north edge vertices read the tile's own edge nodes, equal to the neighbour's first column / row by
// construction). Physics reads the node at its own cell centre (fill_physics_classes), so render and grip differ by at
// most half a cell at a class border (design 10.1).
//
// Level policy: the rank plane carries the road (0 = no road, else the way's rank clamped to 1..255). A road node
// (rank > 0) whose chunk level has no policy (raster_params_for_render_level: disabled, or past the policy's levels, so
// L4-L6 with the default policy), or whose rank is below the level's min_rank, is written
// Unknown (the "no class data" value build_chunk and the game's colour ramp already handle); nodes with rank 0 keep
// their land class. min_half_width_mm is not used: the server's rank-max pyramid already keeps roads at coarse levels.
// Note: rank 0 tracks are stored as rank 1, so min_rank 0 and 1 select the same nodes.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/layer/terrain_class.h"
#include "g2m/mesh/terrain_chunk.h"

#include <memory>

namespace g2m::world {

// Returns the g2m.terrain.class tile `key` (any level), or null when it is absent or not loaded (the window is then
// all-Unknown). Bare function pointer plus context, like ClassLookup's RoadSegmentsFn; must be callable from several
// threads at once if the caller builds chunks in parallel.
using TerrainClassTileFn = std::shared_ptr<const TerrainClassTile> (*)(void* ctx, const TileKey& key);

// A TerrainClassTileFn with its ctx. A null `fn` means "not used" (ViewSource then falls back to its ClassLookup).
struct TerrainClassLookup {
    TerrainClassTileFn fn = nullptr;
    void* ctx = nullptr;
    // The render road LOD policy (osm_roads.h, the game's settings); null = default_render_road_lod(). Not owned;
    // must outlive the call. A caller caching chunk classes keys them with render_road_lod_key(policy, chunk level).
    const RenderRoadLod* lod = nullptr;
};

// The class tile holding every node chunk `key` reads: (zone, level, floor(cx / 4), floor(cy / 4)).
TileKey terrain_class_tile_for_chunk(const mesh::ChunkKey& key);

// The window of chunk `key` from `tile` (null = all-Unknown). `tile->key` must equal terrain_class_tile_for_chunk(key);
// a tile for another key also leaves the window all-Unknown. Allocation-free.
void terrain_class_window(const mesh::ChunkKey& key, const TerrainClassTile* tile, const RenderRoadLod& lod,
                          mesh::ClassWindow& out) noexcept;

// lookup.fn(ctx, terrain_class_tile_for_chunk(key)) then terrain_class_window. `out` stays all-Unknown (no call) when
// lookup.fn is null.
void fill_chunk_terrain_classes(const mesh::ChunkKey& key, const TerrainClassLookup& lookup, mesh::ClassWindow& out);

} // namespace g2m::world
