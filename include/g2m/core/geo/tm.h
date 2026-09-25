// g2m/core/geo/tm.h - transverse Mercator projection (PLAN.md section 4.1).
//
// Krueger's series to 6th order in the third flattening n, evaluated with
// Clenshaw summation as in Karney (2011), "Transverse Mercator with an
// accuracy of a few nanometers", J. Geodesy 85:475-485, and GeographicLib's
// TransverseMercator (MIT). Truncation error <= 5 nm within 35 degrees of
// the central meridian; do not use beyond about 75 degrees from it.
// Built entirely on g2m::det, so every result is bit-identical on every
// supported compiler/OS.
//
// Units: degrees for angles, metres for x/y. x is the easting and y the
// northing relative to the central meridian / equator (no false offsets;
// utm.h adds those).
#pragma once

#include "g2m/core/geo/ellipsoid.h"

namespace g2m::geo {

struct TmForward {
    double x;      // easting (m), 0 on the central meridian
    double y;      // northing (m), 0 on the equator
    double gamma;  // meridian convergence (degrees): angle from grid north to true north, clockwise
    double k;      // point scale factor
};

struct TmReverse {
    double lat;    // degrees
    double lon;    // degrees, in [-180, 180]
    double gamma;  // meridian convergence (degrees)
    double k;      // point scale factor
};

class TransverseMercator {
public:
    static constexpr int kOrder = 6;

    // k0: central scale factor (0.9996 for UTM).
    TransverseMercator(const Ellipsoid& ellipsoid, double k0);

    // lat in [-90, 90]; lon and lon0 any finite value (degrees).
    TmForward forward(double lon0, double lat, double lon) const;
    TmReverse reverse(double lon0, double x, double y) const;

    const Ellipsoid& ellipsoid() const { return ellipsoid_; }
    double k0() const { return k0_; }
    // Rectifying radius A = a/(1+n) (1 + n^2/4 + n^4/64 + n^6/256).
    double rectifying_radius() const { return a1_; }

private:
    Ellipsoid ellipsoid_;
    double k0_;
    double e2_;   // e^2
    double es_;   // e (signed as GeographicLib: negative for a prolate ellipsoid)
    double e2m_;  // 1 - e^2
    double c_;    // sqrt(1 - e^2) * exp(e atanh(e)): scale at the pole / k0
    double n_;
    double b1_;   // A / a
    double a1_;   // A
    double alp_[kOrder + 1]; // alpha_1..alpha_6 (forward), index 0 unused
    double bet_[kOrder + 1]; // beta_1..beta_6 (reverse), index 0 unused
};

} // namespace g2m::geo
