// src/core/geo/utm.cpp - see g2m/core/geo/utm.h.

#include "g2m/core/geo/utm.h"

#include "g2m/core/geo/angle.h"

#include <algorithm>
#include <cmath>

namespace g2m::geo {

int utm_latitude_band(double lat) {
    const int ilat = static_cast<int>(std::floor(lat));
    return std::max(-10, std::min(9, (ilat + 80) / 8 - 10));
}

std::optional<UtmZone> utm_standard_zone(double lat, double lon) {
    if (!std::isfinite(lat) || !std::isfinite(lon)) return std::nullopt;
    if (!(lat >= kUtmMinLat && lat < kUtmMaxLat)) return std::nullopt;
    int ilon = static_cast<int>(std::floor(ang_normalize(lon)));
    if (ilon == 180) ilon = -180; // ilon now in [-180, 180)
    int zone = (ilon + 186) / 6;
    const int band = utm_latitude_band(lat);
    if (band == 7 && zone == 31 && ilon >= 3) {
        zone = 32; // Norway exception
    } else if (band == 9 && ilon >= 0 && ilon < 42) {
        zone = 2 * ((ilon + 183) / 12) + 1; // Svalbard exception: 31X, 33X, 35X, 37X
    }
    return UtmZone{zone, lat >= 0 ? Hemisphere::North : Hemisphere::South};
}

std::string utm_scheme_name(UtmZone zone) {
    return "utm" + std::to_string(zone.zone) + (zone.hemisphere == Hemisphere::North ? "n" : "s");
}

std::optional<UtmZone> parse_utm_scheme(std::string_view s) {
    if (s.size() < 5 || s.size() > 6 || !s.starts_with("utm")) return std::nullopt;
    const char h = s.back();
    if (h != 'n' && h != 's') return std::nullopt;
    const std::string_view digits = s.substr(3, s.size() - 4);
    if (digits[0] == '0') return std::nullopt; // canonical: no leading zero
    int zone = 0;
    for (char c : digits) {
        if (c < '0' || c > '9') return std::nullopt;
        zone = zone * 10 + (c - '0');
    }
    const UtmZone z{zone, h == 's' ? Hemisphere::South : Hemisphere::North};
    if (!z.valid()) return std::nullopt;
    return z;
}

Utm::Utm(const Ellipsoid& ellipsoid) : tm_(ellipsoid, kUtmK0) {}

UtmForward Utm::forward(UtmZone zone, double lat, double lon) const {
    const TmForward t = tm_.forward(utm_central_meridian(zone.zone), lat, lon);
    return {t.x + kUtmFalseEasting, t.y + utm_false_northing(zone.hemisphere), t.gamma, t.k};
}

TmReverse Utm::reverse(UtmZone zone, double easting, double northing) const {
    return tm_.reverse(utm_central_meridian(zone.zone), easting - kUtmFalseEasting,
                       northing - utm_false_northing(zone.hemisphere));
}

} // namespace g2m::geo
