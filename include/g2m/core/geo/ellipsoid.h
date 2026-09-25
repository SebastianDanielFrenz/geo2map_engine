// g2m/core/geo/ellipsoid.h - reference ellipsoids (PLAN.md section 4.1).
#pragma once

namespace g2m::geo {

// An oblate ellipsoid of revolution: equatorial radius a (m) and flattening
// f. Derived quantities are computed where they are used (tm.cpp, ecef.cpp),
// always with the same fixed operation order.
struct Ellipsoid {
    double a;
    double f;

    constexpr double e2() const { return f * (2 - f); }       // first eccentricity squared
    constexpr double n() const { return f / (2 - f); }        // third flattening
    constexpr double b() const { return a * (1 - f); }        // polar radius
};

// GRS80: the ellipsoid of ETRS89, and thus of EPSG:25832 (ETRS89 / UTM 32N),
// the official German grid. The default everywhere in g2m.
inline constexpr Ellipsoid kGRS80{6378137.0, 1.0 / 298.257222101};
// WGS84. Differs from GRS80 only in f (b differs by 0.1 mm); the metre-level
// ETRS89/WGS84 datum offset is ignored (PLAN.md 4.1).
inline constexpr Ellipsoid kWGS84{6378137.0, 1.0 / 298.257223563};

} // namespace g2m::geo
