// src/layer/height_tile.cpp - see g2m/layer/height_tile.h.

#include "g2m/layer/height_tile.h"

#include "g2m/layer/src_elev.h"

#include <algorithm>
#include <cassert>

namespace g2m {

RasterGeoref height_tile_georef(const TileKey& key) {
    assert(key.valid());
    const std::optional<RasterGeoref> g =
        tile_georef(key, kTerrainTileSamples, SampleRegistration::Cell, RowOrder::SouthToNorth);
    // A valid key always yields a valid georef here: size_m() is
    // kLevel0SizeM << level (an integer number of metres), so the spacing
    // (size_m() / kTerrainTileSamples, in mm) is always an integer.
    assert(g.has_value());
    return g.value_or(RasterGeoref{});
}

Result<TileBody> encode_height_tile(const HeightTile& t) {
    ElevTile e;
    e.georef = height_tile_georef(t.key);
    e.heights = HeightRaster(kTerrainTileSamples, kTerrainTileSamples);
    const std::span<std::int32_t> dst = e.heights.data();
    std::copy(t.h.begin(), t.h.end(), dst.begin());
    return encode_src_elev(e);
}

Result<void> decode_height_tile(const TileBody& body, const TileKey& expect, HeightTile& out) {
    Result<ElevTile> decoded = decode_src_elev(body);
    if (!decoded) return decoded.error();
    const RasterGeoref want = height_tile_georef(expect);
    if (!(decoded->georef == want)) {
        return make_error(ErrorCode::Malformed, "height_tile: georef does not match the expected key");
    }
    if (decoded->heights.width() != kTerrainTileSamples || decoded->heights.height() != kTerrainTileSamples) {
        return make_error(ErrorCode::Malformed, "height_tile: unexpected sample count");
    }
    out.key = expect;
    const std::span<const std::int32_t> src = decoded->heights.data();
    std::copy(src.begin(), src.end(), out.h.begin());
    out.has_nodata = std::any_of(out.h.begin(), out.h.end(), [](std::int32_t v) { return v == kHeightNoData; });
    return {};
}

} // namespace g2m
