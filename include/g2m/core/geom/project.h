// g2m/core/geom/project.h - project an OSM-style fixed-point lat/lon into
// integer-millimetre UTM coordinates (G1c plan I1). The exact rule
// osm_clip.cpp's clip_osm() has always used for its own node projection,
// pulled out here so ring assembly (region resolution, I2+) can share it
// byte-for-byte.
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/core/geom/exact.h"

#include <cmath>
#include <cstdint>
#include <optional>

namespace g2m::geom {

// lat_e7/lon_e7: OSM's fixed point, degrees * 1e7. Empty if the point is
// more than 30 degrees of longitude from `zone`'s central meridian (the
// projection is not meant to be used that far out - "far", matching
// clip_osm's own comment) or the projection is not finite. Otherwise the
// projected easting/northing, floored to whole millimetres.
inline std::optional<PointMm> project_e7_to_mm(const geo::Utm& utm, geo::UtmZone zone, std::int32_t lat_e7,
                                                std::int32_t lon_e7) {
    const double lon0 = geo::utm_central_meridian(zone.zone);
    const double lat = static_cast<double>(lat_e7) / 1e7;
    const double lon = static_cast<double>(lon_e7) / 1e7;
    double dlon = lon - lon0;
    if (dlon > 180.0) dlon -= 360.0;
    if (dlon < -180.0) dlon += 360.0;
    if (dlon > 30.0 || dlon < -30.0) return std::nullopt; // far
    const geo::UtmForward f = utm.forward(zone, lat, lon);
    if (!std::isfinite(f.easting) || !std::isfinite(f.northing)) return std::nullopt;
    return PointMm{static_cast<std::int64_t>(std::floor(f.easting * 1000.0)),
                   static_cast<std::int64_t>(std::floor(f.northing * 1000.0))};
}

} // namespace g2m::geom
