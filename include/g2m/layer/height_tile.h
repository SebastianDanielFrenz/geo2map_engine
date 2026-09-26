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

} // namespace g2m
