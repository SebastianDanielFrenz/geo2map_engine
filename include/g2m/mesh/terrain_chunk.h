// g2m/mesh/terrain_chunk.h - engine-neutral terrain render chunks (PLAN.md
// G2.3): a fixed-size (kChunkQuads x kChunkQuads quads, 65 x 65 vertices)
// piece of one terrain.height/terrain.class pyramid level, triangulated with
// the SAME diagonal physics_sim's Jolt backend uses for its own heightfield
// (see build_chunk's header comment below for the derivation) so a vehicle's
// wheels sit visually on the rendered ground, plus a data-driven skirt that
// hides the crack against a coarser or finer LOD neighbour.
//
// Every type here is POD-shaped (arrays, spans, a double origin[3]) so a
// later C API (g2m_c, PLAN.md 7.5) can map onto it 1:1 without an
// intermediate marshalling layer - see PLAN.md section 4 decision 2.
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"

#include <array>
#include <compare>
#include <cstdint>
#include <vector>

namespace g2m::mesh {

// 64 quads per side -> 65 x 65 vertices. One chunk (k, cx, cy) covers
// absolute level-k samples [64*cx, 64*cx + 64] x [64*cy, 64*cy + 64] of the
// terrain.height pyramid's own absolute lattice (Cell-registered, PLAN.md
// G2.1's height_tile.h) - it lives inside terrain.height tile
// {level=k, x=cx>>2, y=cy>>2} (256 samples per tile / 64 samples per chunk =
// 4 chunks per tile per axis).
inline constexpr int kChunkQuads = 64;
inline constexpr int kChunkVerts = kChunkQuads + 1;      // 65
inline constexpr int kHaloWindow = kChunkVerts + 2;      // 67 (1-sample halo each side)
inline constexpr int kBaseVertexCount = kChunkVerts * kChunkVerts;      // 65*65 = 4225
inline constexpr int kSkirtVertexCount = 4 * kChunkVerts;               // 4*65 = 260
inline constexpr int kTotalVertexCount = kBaseVertexCount + kSkirtVertexCount; // 4485

struct ChunkKey {
    geo::UtmZone zone;
    std::uint8_t level;
    std::int32_t cx, cy;
    auto operator<=>(const ChunkKey&) const = default;
};

// 67 x 67 int32 heights (row-major, row 0 = south, same convention as
// HeightTile::h), a 1-sample halo around the chunk's own 65 x 65 vertices so
// build_chunk's central-difference normals and skirt-depth formula never run
// off the edge. Index: window offset (a, b) in [0, 66] holds the absolute
// sample at local chunk offset (a - 1, b - 1); kHeightNoData (INT32_MIN)
// where no tile covers that sample.
struct HeightWindow {
    std::array<std::int32_t, kHaloWindow * kHaloWindow> h{};
};

// LandClass per vertex of the 65 x 65 base grid (row-major, row 0 = south, no
// halo - G2.5; every ClassWindow is Unknown (0) before that increment).
struct ClassWindow {
    std::array<std::uint8_t, kChunkVerts * kChunkVerts> c{};
};

// Returns the HeightTile covering `key` (a terrain.height key, any level),
// or nullptr if absent (out of coverage / not resident) - gather_window then
// treats every one of that tile's samples as NoData. `ctx` is opaque,
// threaded through unchanged (no TileStore/Server dependency here - g2m_mesh
// links only g2m_core and g2m_layers).
using TileLookup = const HeightTile* (*)(void* ctx, const TileKey&);

// Fills `out` with the 67 x 67 halo window for chunk `key`, fetching from
// `lookup` only the (at most 9, typically 1 or 4) distinct terrain.height
// tiles the halo can touch - a chunk that is not on the western/eastern or
// southern/northern edge of its own tile never crosses a tile boundary at
// all; only an edge chunk's 1-sample halo reaches into exactly one
// neighbouring tile per crossed axis.
void gather_window(const ChunkKey& key, TileLookup lookup, void* ctx, HeightWindow& out);

// A built render chunk. `positions`/`normals` are float triples (x, y, z),
// Z-up, x east / y north, local to `origin`; `indices` index into
// `positions`/`normals`/`land_class` (three per triangle). Base vertices
// (kBaseVertexCount of them) come first, row-major (row 0 = south);
// `skirt_first_vertex` is the index of the first of the kSkirtVertexCount
// skirt vertices that follow (west, then east, then south, then north
// border, each kChunkVerts vertices, south-to-north / west-to-east order,
// duplicating a shared corner once per border it belongs to).
struct TerrainChunkMesh {
    ChunkKey key;
    double origin[3] = {0.0, 0.0, 0.0}; // absolute UTM E, N of vertex (0,0) (a level-k sample centre); z = 0
    std::vector<float> positions;       // 3 floats per vertex, x,y,z local (Z-up, x east, y north); z = h/256 exact
    std::vector<float> normals;         // 3 floats per vertex, unit, Z-up
    std::vector<std::uint8_t> land_class; // 1 byte per vertex
    std::vector<std::uint16_t> indices;   // 3 per triangle, < 65536 vertices total
    std::uint32_t skirt_first_vertex = 0;
    float skirt_depth_m = 0.0f;
    std::uint64_t content_hash = 0;
};

// Builds `out` from `window` (and `classes`, or an implicit all-Unknown
// ClassWindow if null - G2.5 has not run yet). Reuses `out`'s vector
// capacity (resize, not reallocate-from-empty): a second call with the same
// or smaller shape makes zero allocations - see tests/mesh/test_terrain_chunk.cpp.
//
// Triangulation and the Jolt diagonal (derivation, PLAN.md G2.3):
//   Jolt's own JPH::HeightFieldShape (physics_sim/.../HeightFieldShape.cpp)
//   splits every quad, in ITS OWN grid index space (x, y), along the
//   (x, y)-(x+1, y+1) diagonal.
//
//   physics_sim's jolt_backend.cpp wraps the inner (Y-up) HeightFieldShape in
//   a RotatedTranslatedShape (+90 degree rotation about local X, position
//   heightfield_wrapper_position()) specifically so that, from OUTSIDE that
//   file, body-local sample (ix, iy) - the same (ix, iy) the ITerrainSource
//   caller used to fill `heights[iy*n+ix]` - lands at body-local point
//   (ix*spacing, iy*spacing, height): x = east index, y = north index, row 0
//   = south, exactly g2m's own HeightTile/HeightWindow convention. The file's
//   own header comment on kHeightfieldYupToZup derives this by working out
//   what its ROW-REVERSAL (reverse_heightfield_rows_into: Jolt's internal
//   row `iy_jolt` <- our row `n-1-iy_jolt`) must be, given the rotation's
//   effect, to land our (ix, iy) samples at the documented positions; that
//   same algebra gives iy_jolt = n-1-iy_src.
//
//   So Jolt's own quad cell (x=ix_jolt, y=iy_jolt) has, in OUR (i=east,
//   j=north, row0=south) index space:
//     Jolt corner (ix_jolt,   iy_jolt)   -> our (i,   j+1)   [west, NORTH]
//     Jolt corner (ix_jolt+1, iy_jolt)   -> our (i+1, j+1)   [east, north]
//     Jolt corner (ix_jolt,   iy_jolt+1) -> our (i,   j)     [west, SOUTH]
//     Jolt corner (ix_jolt+1, iy_jolt+1) -> our (i+1, j)     [east, south]
//   (increasing Jolt-y maps to DEcreasing our j, because of the row
//   reversal). Jolt's own diagonal, (ix_jolt,iy_jolt)-(ix_jolt+1,iy_jolt+1),
//   therefore maps to our (i, j+1)-(i+1, j): the NW corner of the quad to
//   its SE corner - the "anti-diagonal", not the (i,j)-(i+1,j+1) one.
//
//   build_chunk triangulates every quad (i, j) as two CCW (seen from +Z)
//   triangles sharing exactly that diagonal:
//     T1 = {(i,j), (i+1,j), (i,j+1)}, T2 = {(i+1,j), (i+1,j+1), (i,j+1)}.
//   A triangle is dropped (no index emitted) if any of its 3 vertices is
//   NoData.
void build_chunk(const ChunkKey& key, const HeightWindow& window, const ClassWindow* classes, TerrainChunkMesh& out);

} // namespace g2m::mesh
