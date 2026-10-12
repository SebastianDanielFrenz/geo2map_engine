// g2m/world/chunk_road_classes.h - OSM road classes projected onto one render
// chunk's vertices (terrain migration plan phase 1; ported from racing_game's
// road_classes.cpp, roads_plan.md R-2 / G2.5a-grip R-b).
//
// A caller supplies the road segments of whole g2m.src.osm tiles (level 2,
// 1024 m + 128 m halo) through a RoadSegmentsFn; rasterize_chunk_road_classes
// paints them with rasterize_road_blocks' OWNER RULE onto the chunk's L0
// cell-centre lattice (the same lattice a TerrainChunkMesh's vertices sit on,
// see mesh/terrain_chunk.h's vertex-placement formula) and writes the
// LandClass per vertex into a mesh::ClassWindow. The level policy is
// raster_params_for_render_level (osm_roads.h; level >= 4 has no road classes).
//
// Render path only: grip / physics surface ids are filled elsewhere
// (phys/fill_physics_surfaces.h) from the same segments and rasteriser.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/mesh/terrain_chunk.h"

#include <memory>
#include <vector>

namespace g2m::world {

// Returns the road segments of one g2m.src.osm tile (level 2), or an empty or
// null list on any failure ("no roads" is a valid, non-fatal answer). A bare
// function pointer plus context, like mesh::TileLookup; must be callable from
// several threads at once if the caller builds chunks in parallel.
using RoadSegmentsFn = std::shared_ptr<const std::vector<RoadSegment>> (*)(void* ctx, const TileKey& key);

// A RoadSegmentsFn with its ctx. A null `fn` means "no road classes": the
// ClassWindow stays all-Unknown and nothing is fetched.
struct ClassLookup {
    RoadSegmentsFn fn = nullptr;
    void* ctx = nullptr;
};

// The Lattice matching render chunk `key`'s 65x65 vertex grid: vertex (i, j)
// at absolute E = (64*cx + i) * 2^level + 0.5 (same for N).
Lattice road_class_lattice(const mesh::ChunkKey& key);

// The g2m.src.osm tile keys (level 2, 1024 m) whose footprint overlaps the
// chunk's footprint (inclusive of a shared edge): at most 4 for a level <= 3
// chunk. Order: y ascending, then x ascending.
std::vector<TileKey> src_osm_tiles_for_chunk(const mesh::ChunkKey& key);

// Rasterises the chunk's road classes with the owner rule: `lookup` is called
// once per aligned level-2 block the lattice straddles, and each block is
// painted from only its own owner tile's segments, so a sample never depends
// on a neighbouring tile's data or on the lookup call order. `out` is first
// reset to all-Unknown and left that way (no fetch) if lookup.fn is null or
// the chunk's level has no road policy. An empty or null list reads as
// RoadBlockStatus::None (the lookup contract does not separate "fetched,
// empty" from "failed").
void rasterize_chunk_road_classes(const mesh::ChunkKey& key, const ClassLookup& lookup, mesh::ClassWindow& out);

} // namespace g2m::world
