// g2m/mesh/lod_select.h - camera-driven LOD chunk selection (PLAN.md G2.3).
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/mesh/terrain_chunk.h"

#include <vector>

namespace g2m::mesh {

struct LodParams {
    // PLAN.md deviation, documented (see the G2.3 report / commit message):
    // the plan's own LodParams has no `zone` field, but select_chunks fills
    // a std::vector<ChunkKey>, and ChunkKey requires one (a session has
    // exactly one UTM zone throughout - PLAN.md 4 decision 5 / 8). Adding it
    // here keeps ChunkKey exactly as specified.
    geo::UtmZone zone{32, geo::Hemisphere::North};
    double range0_m = 250.0;
    int max_level = 6;
    // 6000 m (not e.g. 20000): with range0_m=250 and max_level=6, the
    // per-level chunk count is roughly constant (~36, geometric LOD's whole
    // point - each level's footprint area grows 4x while its own chunk area
    // also grows 4x), except the OUTERMOST (root) level, whose ring has no
    // further split threshold above it and so grows with max_distance_m^2 -
    // 6000 keeps that ring (and the total, PLAN.md G2.3's own "count <= 400"
    // bound - see tests/mesh/test_lod_select.cpp) comfortably bounded; a
    // caller wanting a longer view distance sets this explicitly.
    double max_distance_m = 6000.0;
};

// Fills `out` with the set of chunks to render around (cam_e, cam_n) (an
// absolute UTM point in `params.zone`):
//   - roots are level `params.max_level` chunks within `params.max_distance_m`
//     of the camera (2D distance from the camera point to the chunk's own
//     footprint rectangle);
//   - a level-k chunk (k > 0) is replaced by its 4 level-(k-1) children
//     (same footprint, double resolution: child index = 2*cx + {0,1},
//     2*cy + {0,1}) while the 2D distance from the camera to the chunk's
//     footprint is less than range_{k-1} = params.range0_m * 2^(k-1);
//   - a restricted-quadtree 2:1 balance pass then splits any remaining
//     chunk that is more than 1 level coarser than an edge-adjacent chunk,
//     until no such pair remains;
//   - `out` is sorted by (level descending, cy, cx).
void select_chunks(double cam_e, double cam_n, const LodParams& params, std::vector<ChunkKey>& out);

} // namespace g2m::mesh
