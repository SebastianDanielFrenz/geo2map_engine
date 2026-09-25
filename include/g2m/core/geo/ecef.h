// g2m/core/geo/ecef.h - geodetic -> earth-centred earth-fixed coordinates
// and local east/north/up frames (PLAN.md 2.7, 4.6: g2m provides ECEF and
// ENU frames for a later curved world shape). Forward only; the inverse is
// not needed yet.
#pragma once

#include "g2m/core/geo/ellipsoid.h"

namespace g2m::geo {

struct Vec3d {
    double x;
    double y;
    double z;
};

// lat, lon in degrees; h = height above the ellipsoid (m). Mirrors
// GeographicLib Geocentric::Forward.
Vec3d geodetic_to_ecef(const Ellipsoid& ellipsoid, double lat, double lon, double h);

// Unit vectors of the local east/north/up frame at (lat, lon), in ECEF.
// up is the ellipsoid normal.
struct EnuFrame {
    Vec3d east;
    Vec3d north;
    Vec3d up;
};
EnuFrame enu_frame(double lat, double lon);

} // namespace g2m::geo
