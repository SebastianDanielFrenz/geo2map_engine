// g2m/layer/height_tile.h - HeightTile, the in-memory shape shared by
// g2m.elev.base and g2m.terrain.height (PLAN.md G2.1/G2.2): a 256 x 256
// (kTerrainTileSamples^2) grid of int32 heights (1/256 m, kHeightNoData for
// missing samples), Cell-registered, row 0 = SOUTH (g2m's own row order,
// unlike the NorthToSouth DGM1/GeoTIFF convention). Sample (i, j)'s centre
// sits at (key.min_easting() + (i + 0.5) * 2^level, key.min_northing() +
// (j + 0.5) * 2^level) metres.
//
// Both g2m.elev.base and g2m.terrain.height reuse body encoding 1 (SrcElev,
// docs/formats/src_elev.md, src_elev.h) at this georef - no new codec.
#pragma once

#include "g2m/core/raster.h"
#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/layer/tile_container.h"

#include <array>
#include <cstdint>

namespace g2m {

struct HeightTile {
    TileKey key;
    // Row-major, row 0 = south. kTerrainTileSamples * kTerrainTileSamples.
    std::array<std::int32_t, 65536> h{};
    bool has_nodata = false;
};

static_assert(kTerrainTileSamples * kTerrainTileSamples == 65536,
             "HeightTile::h is sized for kTerrainTileSamples == 256");

// The Cell-registered, SouthToNorth, kTerrainTileSamples x kTerrainTileSamples
// georef of the terrain tile `key` (valid for any valid TileKey - the
// footprint always divides evenly into kTerrainTileSamples cells at an
// integer-millimetre spacing).
RasterGeoref height_tile_georef(const TileKey& key);

// Encodes `t` as an SrcElev (encoding 1) TileBody at height_tile_georef(t.key).
Result<TileBody> encode_height_tile(const HeightTile& t);
// Decodes `body` (an SrcElev TileBody) into `out`; fails if its georef does
// not match height_tile_georef(expect) or its sample count is not
// kTerrainTileSamples^2.
Result<void> decode_height_tile(const TileBody& body, const TileKey& expect, HeightTile& out);

// The mean-of-4 half-even downsample TerrainHeightDeriver uses for levels
// 1-6 (PLAN.md G2.2). Output sample (i, j) is covered by whichever of
// sw/se/nw/ne is its quadrant (i, j < kTerrainTileSamples/2 -> the
// west/south half, else east/north) at local child indices
// (2*ci, 2*cj)..(2*ci+1, 2*cj+1) (ci/cj = i/j offset into the half). The
// value is round-half-even(mean of those 4 int64 samples): s = sum,
// q = floor_div(s, 4), r = s - 4*q, q += 1 if r > 2 or (r == 2 and q is
// odd). NoData (kHeightNoData) if any of the 4 samples is NoData; a null
// child pointer (missing/out-of-coverage input) counts as all-NoData for
// its whole quadrant. `out.key` must already be set by the caller (this
// only fills `h` and `has_nodata`).
void downsample_2x2(const HeightTile* sw, const HeightTile* se, const HeightTile* nw, const HeightTile* ne,
                    HeightTile& out);

} // namespace g2m
