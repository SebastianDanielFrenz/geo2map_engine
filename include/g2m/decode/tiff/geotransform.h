// g2m/decode/tiff/geotransform.h - generic affine georeference (PLAN.md 13,
// milestone G1b), for sources whose georeference is not directly a g2m
// RasterGeoref (non-UTM sources: Copernicus GLO-30 and ESA WorldCover are
// geographic lat/lon; reprojection to UTM is out of scope here, G1c/G2's
// job).
#pragma once

#include "g2m/core/raster.h"

#include <cstdint>
#include <optional>
#include <string>

namespace g2m::decode {

// Axis-aligned (no rotation/shear - every source this reader targets has
// none) affine pixel-to-world transform, sample-CENTRE convention (matches
// a .tfw file's own convention, and GeoTIFF's ModelTiepoint combined with
// RasterType PixelIsPoint; PixelIsArea tiepoints are converted to this
// centre convention at parse time - see tfw.h / tiff_reader.h).
//
//   world_x(col) = origin_x + col * pixel_width
//   world_y(row) = origin_y - row * pixel_height   (row 0 = north edge; pixel_height > 0)
struct GeoTransform {
    double origin_x = 0.0;   // world X of sample (col=0, row=0)'s centre
    double origin_y = 0.0;   // world Y of sample (col=0, row=0)'s centre
    double pixel_width = 0.0;  // > 0
    double pixel_height = 0.0; // > 0 (row spacing magnitude; rows run north to south)

    bool valid() const { return pixel_width > 0.0 && pixel_height > 0.0; }
};

// Where a raster's CRS comes from, in priority order this library applies
// (PLAN.md: "some DGM1 copies lack GeoKeys ... let the caller fall back to
// the filename/.tfw"):
//  - Utm: g2m::geo::UtmZone known (from GeoKeys, or supplied by the caller
//    from the filename/tile key convention) - a RasterGeoref can be built.
//  - GeographicEpsg: geographic (lat/lon) CRS by EPSG code (e.g. 4326) -
//    generic GeoTransform only, no RasterGeoref (reprojection is not this
//    library's job).
//  - ProjectedEpsg: a projected CRS by EPSG code that is not recognised as
//    a plain UTM zone - generic GeoTransform only.
//  - Absent: no georeference info found in the TIFF at all (some DGM1
//    copies) - caller must supply one (filename convention / a sibling
//    .tfw file).
enum class CrsKind : std::uint8_t { Absent, Utm, GeographicEpsg, ProjectedEpsg };

struct Crs {
    CrsKind kind = CrsKind::Absent;
    std::uint16_t epsg = 0; // ProjectedCSTypeGeoKey or GeographicTypeGeoKey verbatim, when kind != Absent/Utm
    int utm_zone = 0;       // 1..60, only when kind == Utm
    bool utm_south = false; // only when kind == Utm
};

// Builds a g2m::RasterGeoref for a UTM-CRS raster, given the pixel-centre
// affine transform, the sample registration/row order the caller already
// knows the source uses (DGM1: Cell, NorthToSouth), and the UTM zone.
// Requires every sample position to land on an exact integer millimetre
// (true for DGM1's 1 m grid at half-metre centres; also true for any other
// metre-based UTM raster on a sane grid) - returns nullopt otherwise
// (would need resampling, out of scope).
std::optional<RasterGeoref> to_raster_georef(const GeoTransform& transform, int width, int height,
                                              SampleRegistration registration, RowOrder row_order,
                                              geo::UtmZone zone);

} // namespace g2m::decode
