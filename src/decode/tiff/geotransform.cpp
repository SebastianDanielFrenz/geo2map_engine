// src/decode/tiff/geotransform.cpp - see g2m/decode/tiff/geotransform.h.

#include "g2m/decode/tiff/geotransform.h"

#include <cmath>

namespace g2m::decode {
namespace {
double reg_half(SampleRegistration r) { return r == SampleRegistration::Cell ? 0.5 : 0.0; }

// Rounds `value_m * 1000` to the nearest integer millimetre, returning
// nullopt if it is not within `tol_mm` of an integer (a genuine sub-mm
// offset - out of RasterGeoref's exact-integer-mm model).
std::optional<std::int64_t> round_to_mm(double value_m) {
    constexpr double kTolMm = 1e-6; // generous: DGM1's own values are exact at double precision
    const double mm = value_m * 1000.0;
    const double rounded = std::round(mm);
    if (std::fabs(mm - rounded) > kTolMm) return std::nullopt;
    return static_cast<std::int64_t>(rounded);
}
} // namespace

std::optional<RasterGeoref> to_raster_georef(const GeoTransform& transform, int width, int height,
                                              SampleRegistration registration, RowOrder row_order,
                                              geo::UtmZone zone) {
    if (!transform.valid() || width <= 0 || height <= 0 || !zone.valid()) return std::nullopt;
    // GeoTransform's row-0-is-north convention only maps cleanly onto a
    // RasterGeoref whose own row order is also NorthToSouth (TIFF/.tfw's
    // native storage order - see RowOrder's own doc comment). A
    // SouthToNorth target would need the caller to flip rows first.
    if (row_order != RowOrder::NorthToSouth) return std::nullopt;

    constexpr double kSpacingTolM = 1e-9;
    if (std::fabs(transform.pixel_width - transform.pixel_height) > kSpacingTolM) return std::nullopt;
    const double spacing_m = transform.pixel_width;

    const double half = reg_half(registration);
    const double west_m = transform.origin_x - half * spacing_m;
    const double south_m = transform.origin_y - (static_cast<double>(height) - 1.0 + half) * spacing_m;

    const auto west_mm = round_to_mm(west_m);
    const auto south_mm = round_to_mm(south_m);
    const auto spacing_mm = round_to_mm(spacing_m);
    if (!west_mm || !south_mm || !spacing_mm || *spacing_mm <= 0) return std::nullopt;

    RasterGeoref g;
    g.zone = zone;
    g.west_mm = *west_mm;
    g.south_mm = *south_mm;
    g.spacing_mm = *spacing_mm;
    g.registration = registration;
    g.row_order = row_order;
    g.width = width;
    g.height = height;
    if (!g.valid()) return std::nullopt;
    return g;
}

} // namespace g2m::decode
