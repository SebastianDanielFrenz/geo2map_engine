// g2m/layer/src_landcover.h - body encoding 2 (SrcLandcover), version 1:
// raw land-cover classes of g2m.src.landcover (docs/formats/src_landcover.md).
//
// One uint8 class per cell, in the source's own class scheme (named, e.g.
// "esa_worldcover_v200"); mapping to g2m::LandClass is an IClassifier's job,
// not the encoding's.
#pragma once

#include "g2m/core/raster.h"
#include "g2m/core/result.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/tile_container.h"

#include <string>

namespace g2m {

inline constexpr std::uint16_t kSrcLandcoverEncodingVersion = 1;
inline constexpr std::size_t kMaxClassSchemeLength = 64;

struct LandcoverTile {
    RasterGeoref georef;
    ClassRaster classes;
    std::string class_scheme;        // [a-z0-9_.], 1..64 bytes
    std::uint8_t nodata_class = 0;   // the scheme's "no data" value

    friend bool operator==(const LandcoverTile&, const LandcoverTile&) = default;
};

Result<TileBody> encode_src_landcover(const LandcoverTile& tile);
Result<LandcoverTile> decode_src_landcover(const TileBody& body);

namespace src_landcover {
inline constexpr std::uint16_t kMeta = 1;
inline constexpr std::uint16_t kClasses = 2;
} // namespace src_landcover

} // namespace g2m
