// g2m/layer/src_elev.h - body encoding 1 (SrcElev), version 1: raw DEM
// samples of g2m.src.elev.<source> (docs/formats/src_elev.md).
//
// Heights are int32 in 1/256 m (g2m/core/raster.h), kHeightNoData marks a
// missing sample. The encoding is lossless for every int32 value: decode
// returns exactly the raster and georef that were encoded, NoData included.
#pragma once

#include "g2m/core/raster.h"
#include "g2m/core/result.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/tile_container.h"

namespace g2m {

inline constexpr std::uint16_t kSrcElevEncodingVersion = 1;

enum class ElevPredictor : std::uint8_t {
    Left = 1,      // p = west neighbour (column 0: the sample below/above in the previous row)
    Gradient = 2,  // p = west + previous-row - previous-row-west (planar), edges as Left
};

struct ElevTile {
    RasterGeoref georef;  // width/height == heights' dimensions
    HeightRaster heights; // row r / column c as georef.row_order says

    friend bool operator==(const ElevTile&, const ElevTile&) = default;
};

Result<TileBody> encode_src_elev(const ElevTile& tile, ElevPredictor predictor = ElevPredictor::Gradient);
Result<ElevTile> decode_src_elev(const TileBody& body);

// Section types.
namespace src_elev {
inline constexpr std::uint16_t kMeta = 1;
inline constexpr std::uint16_t kSamples = 2;
inline constexpr std::uint16_t kNoData = 3;
} // namespace src_elev

} // namespace g2m
