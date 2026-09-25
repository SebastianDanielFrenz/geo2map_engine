// src/layer/georef_io.h - INTERNAL: RasterGeoref serialisation shared by the
// raster encodings (docs/formats/src_elev.md, "Georef block").
#pragma once

#include "g2m/core/bytes.h"
#include "g2m/core/raster.h"
#include "g2m/core/result.h"

#include <algorithm>

namespace g2m::layer_detail {

// Samples per raster, bounded so a sample section stays <= 64 MiB at 4 bytes.
inline constexpr std::int64_t kMaxRasterSamples = std::int64_t{1} << 24;
inline constexpr int kMaxRasterSide = 1 << 24;

inline void write_georef(ByteWriter& w, const RasterGeoref& g) {
    w.u8(static_cast<std::uint8_t>(g.zone.zone));
    w.u8(static_cast<std::uint8_t>(g.zone.hemisphere));
    w.i64(g.west_mm);
    w.i64(g.south_mm);
    w.i64(g.spacing_mm);
    w.u8(static_cast<std::uint8_t>(g.registration));
    w.u8(static_cast<std::uint8_t>(g.row_order));
    w.u32(static_cast<std::uint32_t>(g.width));
    w.u32(static_cast<std::uint32_t>(g.height));
}

// Validates everything RasterGeoref::valid() checks, without any
// intermediate overflow on hostile input, plus the sample-count limit.
inline Result<RasterGeoref> read_georef(ByteReader& r) {
    RasterGeoref g;
    const std::uint8_t zone = r.u8();
    const std::uint8_t hemi = r.u8();
    g.west_mm = r.i64();
    g.south_mm = r.i64();
    g.spacing_mm = r.i64();
    const std::uint8_t reg = r.u8();
    const std::uint8_t order = r.u8();
    const std::uint32_t w = r.u32();
    const std::uint32_t h = r.u32();
    if (!r.ok()) return make_error(ErrorCode::Truncated, "georef truncated");
    if (hemi > 1 || reg > 1 || order > 1) return make_error(ErrorCode::Malformed, "georef: bad enum value");
    if (w == 0 || h == 0 || w > static_cast<std::uint32_t>(kMaxRasterSide) || h > static_cast<std::uint32_t>(kMaxRasterSide) ||
        static_cast<std::int64_t>(w) * static_cast<std::int64_t>(h) > kMaxRasterSamples) {
        return make_error(ErrorCode::LimitExceeded, "georef: raster size out of range");
    }
    constexpr std::int64_t kLimit = std::int64_t{1} << 40;
    const std::int64_t side = std::max<std::int64_t>(w, h);
    if (g.spacing_mm <= 0 || g.spacing_mm >= kLimit / side) return make_error(ErrorCode::Malformed, "georef: bad spacing");
    if (g.west_mm <= -kLimit || g.west_mm >= kLimit || g.south_mm <= -kLimit || g.south_mm >= kLimit) {
        return make_error(ErrorCode::Malformed, "georef: origin out of range");
    }
    g.zone = geo::UtmZone{zone, static_cast<geo::Hemisphere>(hemi)};
    g.registration = static_cast<SampleRegistration>(reg);
    g.row_order = static_cast<RowOrder>(order);
    g.width = static_cast<int>(w);
    g.height = static_cast<int>(h);
    if (!g.valid()) return make_error(ErrorCode::Malformed, "georef invalid");
    return g;
}

} // namespace g2m::layer_detail
