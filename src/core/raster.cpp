// src/core/raster.cpp - see g2m/core/raster.h.

#include "g2m/core/raster.h"

#include <climits>
#include <cmath>

namespace g2m {
namespace {

std::int64_t reg_half(SampleRegistration r) { return r == SampleRegistration::Cell ? 1 : 0; }

// Row index counted from the south edge.
std::int64_t south_index(const RasterGeoref& g, std::int64_t row) {
    return g.row_order == RowOrder::SouthToNorth ? row : static_cast<std::int64_t>(g.height) - 1 - row;
}

} // namespace

std::optional<std::int32_t> quantise_height(double metres) {
    if (!std::isfinite(metres)) return std::nullopt;
    const double scaled = metres * kHeightUnitsPerMetre; // exact (power of two)
    // INT32_MIN is reserved for NoData.
    if (!(scaled > static_cast<double>(INT32_MIN) + 0.5 && scaled < static_cast<double>(INT32_MAX) + 0.5)) {
        return std::nullopt;
    }
    return static_cast<std::int32_t>(std::llround(scaled));
}

std::optional<std::int32_t> quantise_height_half_even(double metres) {
    if (!std::isfinite(metres)) return std::nullopt;
    const double x = metres * kHeightUnitsPerMetre; // exact (power of two)
    const double r = std::floor(x);
    const double d = x - r;
    const bool r_odd = std::fmod(r, 2.0) != 0.0;
    const double result = (d > 0.5 || (d == 0.5 && r_odd)) ? r + 1.0 : r;
    // INT32_MIN is reserved for NoData.
    if (!(result > static_cast<double>(INT32_MIN) + 0.5 && result < static_cast<double>(INT32_MAX) + 0.5)) {
        return std::nullopt;
    }
    return static_cast<std::int32_t>(result);
}

double dequantise_height(std::int32_t units) { return static_cast<double>(units) / kHeightUnitsPerMetre; }

bool RasterGeoref::valid() const {
    if (!zone.valid() || spacing_mm <= 0 || width <= 0 || height <= 0) return false;
    // Keep every half-mm position far inside int64 (and exact in double
    // metres for dyadic positions): |coordinates| < 2^40 mm (~1.1e6 km).
    constexpr std::int64_t kLimit = std::int64_t{1} << 40;
    auto in = [&](std::int64_t v) { return v > -kLimit && v < kLimit; };
    return in(west_mm) && in(south_mm) && in(spacing_mm) && in(spacing_mm * width) && in(spacing_mm * height) &&
           in(west_mm + spacing_mm * width) && in(south_mm + spacing_mm * height);
}

std::int64_t RasterGeoref::cells_x() const {
    return registration == SampleRegistration::Node ? static_cast<std::int64_t>(width) - 1 : width;
}
std::int64_t RasterGeoref::cells_y() const {
    return registration == SampleRegistration::Node ? static_cast<std::int64_t>(height) - 1 : height;
}

std::int64_t RasterGeoref::sample_easting_half_mm(int col) const {
    return 2 * west_mm + (2 * static_cast<std::int64_t>(col) + reg_half(registration)) * spacing_mm;
}

std::int64_t RasterGeoref::sample_northing_half_mm(int row) const {
    return 2 * south_mm + (2 * south_index(*this, row) + reg_half(registration)) * spacing_mm;
}

double RasterGeoref::sample_easting_m(int col) const {
    return static_cast<double>(sample_easting_half_mm(col)) / 2000.0;
}

double RasterGeoref::sample_northing_m(int row) const {
    return static_cast<double>(sample_northing_half_mm(row)) / 2000.0;
}

std::optional<RasterGeoref> tile_georef(const TileKey& tile, int cells_per_side, SampleRegistration registration,
                                        RowOrder row_order) {
    if (!tile.valid() || cells_per_side <= 0) return std::nullopt;
    const std::int64_t size_mm = tile.size_m() * 1000;
    if (size_mm % cells_per_side != 0) return std::nullopt;
    RasterGeoref g;
    g.zone = tile.zone;
    g.west_mm = tile.min_easting() * 1000;
    g.south_mm = tile.min_northing() * 1000;
    g.spacing_mm = size_mm / cells_per_side;
    g.registration = registration;
    g.row_order = row_order;
    const int samples = registration == SampleRegistration::Node ? cells_per_side + 1 : cells_per_side;
    g.width = samples;
    g.height = samples;
    if (!g.valid()) return std::nullopt;
    return g;
}

std::optional<LatticeOffset> lattice_offset(const RasterGeoref& a, const RasterGeoref& b) {
    if (!a.valid() || !b.valid()) return std::nullopt;
    if (a.zone != b.zone || a.spacing_mm != b.spacing_mm) return std::nullopt;
    const std::int64_t step = 2 * a.spacing_mm; // one spacing, in half-mm
    const std::int64_t de = b.sample_easting_half_mm(0) - a.sample_easting_half_mm(0);
    // South-most sample row of each, in half-mm.
    const auto south_row = [](const RasterGeoref& g) {
        return g.row_order == RowOrder::SouthToNorth ? 0 : g.height - 1;
    };
    const std::int64_t dn = b.sample_northing_half_mm(south_row(b)) - a.sample_northing_half_mm(south_row(a));
    if (de % step != 0 || dn % step != 0) return std::nullopt;
    const std::int64_t dcol = de / step;
    const std::int64_t d = dn / step; // b's south index s maps to a's south index s + d
    const auto ha = static_cast<std::int64_t>(a.height);
    const auto hb = static_cast<std::int64_t>(b.height);
    const bool a_sn = a.row_order == RowOrder::SouthToNorth;
    const bool b_sn = b.row_order == RowOrder::SouthToNorth;
    if (a_sn && b_sn) return LatticeOffset{dcol, d, false};
    if (!a_sn && !b_sn) return LatticeOffset{dcol, ha - hb - d, false};
    if (a_sn && !b_sn) return LatticeOffset{dcol, hb - 1 + d, true};
    return LatticeOffset{dcol, ha - 1 - d, true};
}

} // namespace g2m
