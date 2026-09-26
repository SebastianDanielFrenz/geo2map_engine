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

void downsample_2x2(const HeightTile* sw, const HeightTile* se, const HeightTile* nw, const HeightTile* ne,
                    HeightTile& out) {
    constexpr int kHalf = kTerrainTileSamples / 2;
    bool any_nodata = false;
    for (int j = 0; j < kTerrainTileSamples; ++j) {
        const bool north = j >= kHalf;
        const int cj = north ? j - kHalf : j;
        for (int i = 0; i < kTerrainTileSamples; ++i) {
            const bool east = i >= kHalf;
            const int ci = east ? i - kHalf : i;
            const HeightTile* child = north ? (east ? ne : nw) : (east ? se : sw);
            const std::size_t idx =
                static_cast<std::size_t>(j) * static_cast<std::size_t>(kTerrainTileSamples) + static_cast<std::size_t>(i);
            if (child == nullptr) {
                out.h[idx] = kHeightNoData;
                any_nodata = true;
                continue;
            }
            const int col0 = 2 * ci;
            const int col1 = col0 + 1;
            const int row0 = 2 * cj;
            const int row1 = row0 + 1;
            const std::int32_t v00 =
                child->h[static_cast<std::size_t>(row0) * static_cast<std::size_t>(kTerrainTileSamples) +
                        static_cast<std::size_t>(col0)];
            const std::int32_t v01 =
                child->h[static_cast<std::size_t>(row0) * static_cast<std::size_t>(kTerrainTileSamples) +
                        static_cast<std::size_t>(col1)];
            const std::int32_t v10 =
                child->h[static_cast<std::size_t>(row1) * static_cast<std::size_t>(kTerrainTileSamples) +
                        static_cast<std::size_t>(col0)];
            const std::int32_t v11 =
                child->h[static_cast<std::size_t>(row1) * static_cast<std::size_t>(kTerrainTileSamples) +
                        static_cast<std::size_t>(col1)];
            if (v00 == kHeightNoData || v01 == kHeightNoData || v10 == kHeightNoData || v11 == kHeightNoData) {
                out.h[idx] = kHeightNoData;
                any_nodata = true;
                continue;
            }
            const std::int64_t sum = static_cast<std::int64_t>(v00) + static_cast<std::int64_t>(v01) +
                                     static_cast<std::int64_t>(v10) + static_cast<std::int64_t>(v11);
            std::int64_t q = floor_div(sum, std::int64_t{4});
            const std::int64_t r = sum - 4 * q;
            if (r > 2 || (r == 2 && (q & 1) != 0)) ++q;
            out.h[idx] = static_cast<std::int32_t>(q);
        }
    }
    out.has_nodata = any_nodata;
}

} // namespace g2m
