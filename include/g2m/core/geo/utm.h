// g2m/core/geo/utm.h - Universal Transverse Mercator on top of tm.h
// (PLAN.md section 4). Zones 1..60, north/south hemisphere; UPS (the polar
// caps) is out of scope.
#pragma once

#include "g2m/core/geo/ellipsoid.h"
#include "g2m/core/geo/tm.h"

#include <compare>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>

namespace g2m::geo {

enum class Hemisphere : std::uint8_t { North = 0, South = 1 };

struct UtmZone {
    int zone = 0;                          // 1..60
    Hemisphere hemisphere = Hemisphere::North;

    bool valid() const { return zone >= 1 && zone <= 60; }
    friend constexpr bool operator==(const UtmZone&, const UtmZone&) = default;
    friend constexpr auto operator<=>(const UtmZone&, const UtmZone&) = default;
};

inline constexpr double kUtmK0 = 0.9996;
inline constexpr double kUtmFalseEasting = 500000.0;
inline constexpr double kUtmFalseNorthingSouth = 10000000.0;
inline constexpr double kUtmMinLat = -80.0; // inclusive
inline constexpr double kUtmMaxLat = 84.0;  // exclusive

// Central meridian of a zone, degrees: 6 * zone - 183.
constexpr double utm_central_meridian(int zone) { return 6.0 * zone - 183.0; }

// False northing of a hemisphere (0 or 10,000 km).
constexpr double utm_false_northing(Hemisphere h) {
    return h == Hemisphere::South ? kUtmFalseNorthingSouth : 0.0;
}

// MGRS latitude band index, -10 (C) .. 9 (X), as GeographicLib
// MGRS::LatitudeBand. Only meaningful for lat in [-80, 84).
int utm_latitude_band(double lat);

// The standard UTM zone of a point, including the Norway (32V) and
// Svalbard (31X/33X/35X/37X) exceptions, as GeographicLib
// UTMUPS::StandardZone. Hemisphere: North for lat >= 0. Empty outside
// [-80, 84) (UPS territory) or for non-finite input.
std::optional<UtmZone> utm_standard_zone(double lat, double lon);

// "utm32n" / "utm33s": the tile scheme name of a zone (PLAN.md 3.4).
std::string utm_scheme_name(UtmZone zone);
// Its exact inverse: "utm" + zone 1..60 without leading zero + n|s.
std::optional<UtmZone> parse_utm_scheme(std::string_view name);

struct UtmForward {
    double easting;   // m, false easting included
    double northing;  // m, false northing included
    double gamma;     // meridian convergence, degrees
    double k;         // point scale factor
};

class Utm {
public:
    explicit Utm(const Ellipsoid& ellipsoid = kGRS80);

    // Any zone may be used for any point ("extended zone", PLAN.md 4.5):
    // the series stays accurate far outside the 6-degree strip.
    UtmForward forward(UtmZone zone, double lat, double lon) const;
    TmReverse reverse(UtmZone zone, double easting, double northing) const;

    const TransverseMercator& tm() const { return tm_; }

private:
    TransverseMercator tm_;
};

} // namespace g2m::geo
