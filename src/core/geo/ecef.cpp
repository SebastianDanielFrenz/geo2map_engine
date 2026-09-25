// src/core/geo/ecef.cpp - see g2m/core/geo/ecef.h.

#include "g2m/core/geo/ecef.h"

#include "g2m/core/geo/angle.h"

#include <cmath>

namespace g2m::geo {

Vec3d geodetic_to_ecef(const Ellipsoid& ellipsoid, double lat, double lon, double h) {
    double sphi = 0, cphi = 0, slam = 0, clam = 0;
    sincosd(lat, sphi, cphi);
    sincosd(lon, slam, clam);
    const double e2 = ellipsoid.f * (2 - ellipsoid.f);
    const double e2m = 1 - e2;
    const double n = ellipsoid.a / std::sqrt(1 - e2 * (sphi * sphi)); // prime vertical radius
    Vec3d out{};
    out.z = (e2m * n + h) * sphi;
    const double r = (n + h) * cphi;
    out.y = r * slam;
    out.x = r * clam;
    return out;
}

EnuFrame enu_frame(double lat, double lon) {
    double sphi = 0, cphi = 0, slam = 0, clam = 0;
    sincosd(lat, sphi, cphi);
    sincosd(lon, slam, clam);
    EnuFrame f{};
    f.east = {-slam, clam, 0.0};
    f.north = {-sphi * clam, -sphi * slam, cphi};
    f.up = {cphi * clam, cphi * slam, sphi};
    return f;
}

} // namespace g2m::geo
