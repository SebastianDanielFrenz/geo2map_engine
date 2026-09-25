// g2m/core/geo/session_frame.h - the flat local frame of one session
// (PLAN.md 4.2): x = grid E - E0, y = grid N - N0 on the session's UTM
// zone, with E0/N0 integer kilometres. z (orthometric height) passes
// through unchanged and is not handled here.
#pragma once

#include "g2m/core/geo/tm.h"
#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"

#include <cstdint>
#include <optional>

namespace g2m::geo {

struct LocalXY {
    double x; // m east of E0
    double y; // m north of N0
};

class SessionFrame {
public:
    // e0_m/n0_m: the origin on the zone's UTM grid (false offsets included),
    // in metres; must be multiples of 1000. Throws std::invalid_argument
    // otherwise or for an invalid zone.
    SessionFrame(UtmZone zone, std::int64_t e0_m, std::int64_t n0_m, const Ellipsoid& ellipsoid = kGRS80);

    // The frame of a spawn point: its standard zone and the kilometre corner
    // it lies in. Empty outside the UTM latitude range.
    static std::optional<SessionFrame> around(double lat, double lon, const Ellipsoid& ellipsoid = kGRS80);

    UtmZone zone() const { return zone_; }
    std::int64_t e0_m() const { return e0_; }
    std::int64_t n0_m() const { return n0_; }
    const TransverseMercator& tm() const { return tm_; }

    // Geodetic <-> local. Computed from the TM x/y directly minus integer
    // offsets (E0 - 500 km, N0 - false northing), never via the full grid
    // northing, so a point near the origin keeps full double precision.
    // The session keeps its zone everywhere (extended zone, PLAN.md 4.5).
    LocalXY from_geodetic(double lat, double lon) const;
    TmReverse to_geodetic(LocalXY p) const;
    // Meridian convergence and scale at a geodetic point.
    TmForward tm_at(double lat, double lon) const;

    // Grid <-> local: plain subtraction/addition of E0/N0.
    LocalXY from_grid(double easting, double northing) const;
    double grid_easting(double x) const;
    double grid_northing(double y) const;

    // The tile at local point p. Exact integer arithmetic (floor of the local
    // coordinate, add the integer origin, floor-divide by the tile size), so
    // a point exactly on a tile edge always lands in the tile to its
    // east/north. Empty as tile_key_at.
    std::optional<TileKey> tile_at(LocalXY p, int level) const;
    // South-west corner of a tile of this frame's zone, in local metres
    // (exact: integers well inside double's exact range).
    LocalXY tile_origin(const TileKey& tile) const;

private:
    UtmZone zone_;
    std::int64_t e0_;
    std::int64_t n0_;
    double lon0_;
    double off_x_; // E0 - false easting: tm.x = x + off_x_
    double off_y_; // N0 - false northing: tm.y = y + off_y_
    TransverseMercator tm_;
};

} // namespace g2m::geo
