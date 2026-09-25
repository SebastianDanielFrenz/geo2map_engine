// src/core/geo/session_frame.cpp - see g2m/core/geo/session_frame.h.

#include "g2m/core/geo/session_frame.h"

#include <cmath>
#include <stdexcept>

namespace g2m::geo {

SessionFrame::SessionFrame(UtmZone zone, std::int64_t e0_m, std::int64_t n0_m, const Ellipsoid& ellipsoid)
    : zone_(zone), e0_(e0_m), n0_(n0_m), lon0_(utm_central_meridian(zone.zone)),
      off_x_(static_cast<double>(e0_m - static_cast<std::int64_t>(kUtmFalseEasting))),
      off_y_(static_cast<double>(n0_m - static_cast<std::int64_t>(utm_false_northing(zone.hemisphere)))),
      tm_(ellipsoid, kUtmK0) {
    if (!zone.valid()) throw std::invalid_argument("SessionFrame: invalid UTM zone");
    if (e0_m % 1000 != 0 || n0_m % 1000 != 0) {
        throw std::invalid_argument("SessionFrame: E0/N0 must be multiples of 1000 m");
    }
    constexpr std::int64_t kLimit = std::int64_t{1} << 40;
    if (e0_m <= -kLimit || e0_m >= kLimit || n0_m <= -kLimit || n0_m >= kLimit) {
        throw std::invalid_argument("SessionFrame: origin out of range");
    }
}

std::optional<SessionFrame> SessionFrame::around(double lat, double lon, const Ellipsoid& ellipsoid) {
    const std::optional<UtmZone> zone = utm_standard_zone(lat, lon);
    if (!zone) return std::nullopt;
    const TransverseMercator tm(ellipsoid, kUtmK0);
    const TmForward f = tm.forward(utm_central_meridian(zone->zone), lat, lon);
    const double e = f.x + kUtmFalseEasting;
    const double n = f.y + utm_false_northing(zone->hemisphere);
    // Exact: floor to an integer metre, then an integer floor division.
    const std::int64_t e0 = floor_div(static_cast<std::int64_t>(std::floor(e)), 1000) * 1000;
    const std::int64_t n0 = floor_div(static_cast<std::int64_t>(std::floor(n)), 1000) * 1000;
    return SessionFrame(*zone, e0, n0, ellipsoid);
}

LocalXY SessionFrame::from_geodetic(double lat, double lon) const {
    const TmForward f = tm_.forward(lon0_, lat, lon);
    return {f.x - off_x_, f.y - off_y_};
}

TmReverse SessionFrame::to_geodetic(LocalXY p) const { return tm_.reverse(lon0_, p.x + off_x_, p.y + off_y_); }

TmForward SessionFrame::tm_at(double lat, double lon) const { return tm_.forward(lon0_, lat, lon); }

LocalXY SessionFrame::from_grid(double easting, double northing) const {
    return {easting - static_cast<double>(e0_), northing - static_cast<double>(n0_)};
}

double SessionFrame::grid_easting(double x) const { return x + static_cast<double>(e0_); }
double SessionFrame::grid_northing(double y) const { return y + static_cast<double>(n0_); }

std::optional<TileKey> SessionFrame::tile_at(LocalXY p, int level) const {
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) return std::nullopt;
    constexpr double kLimit = 4503599627370496.0; // 2^52
    if (std::fabs(p.x) >= kLimit || std::fabs(p.y) >= kLimit) return std::nullopt;
    // floor(x) + E0 is an exact integer grid easting whose tile equals that
    // of x + E0 (the fractional part cannot cross an integer tile edge).
    const std::int64_t ie = static_cast<std::int64_t>(std::floor(p.x)) + e0_;
    const std::int64_t in = static_cast<std::int64_t>(std::floor(p.y)) + n0_;
    return tile_key_at(zone_, level, static_cast<double>(ie), static_cast<double>(in));
}

LocalXY SessionFrame::tile_origin(const TileKey& tile) const {
    return {static_cast<double>(tile.min_easting() - e0_), static_cast<double>(tile.min_northing() - n0_)};
}

} // namespace g2m::geo
