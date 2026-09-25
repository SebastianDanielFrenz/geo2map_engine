// g2m/core/tile_key.h - raster/vector tile keys on the UTM grid
// (PLAN.md 0.5, 3.4, 4.4).
//
// A tile of level L covers 256 m * 2^L on a side:
//   x = floor(E / size), y = floor(N / size)
// on the zone's false-easting/false-northing grid. Levels are shared by all
// layers (level 2 = 1024 m, the vector tile size). Indices are computed with
// integer arithmetic only (floor of the coordinate, then a floor division),
// so the result is exact for every finite input.
#pragma once

#include "g2m/core/geo/utm.h"

#include <compare>
#include <cstdint>
#include <optional>
#include <string>

namespace g2m {

struct TileKey {
    static constexpr int kMaxLevel = 20;            // 256 m * 2^20 = 268,435 km: far beyond any zone
    static constexpr int kIndexBits = 24;           // signed x/y range: [-2^23, 2^23)
    static constexpr std::int32_t kIndexMin = -(1 << (kIndexBits - 1));
    static constexpr std::int32_t kIndexMax = (1 << (kIndexBits - 1)) - 1;
    static constexpr std::int64_t kLevel0SizeM = 256;

    geo::UtmZone zone;
    int level = 0;
    std::int32_t x = 0;  // easting index
    std::int32_t y = 0;  // northing index

    bool valid() const;

    std::int64_t size_m() const { return kLevel0SizeM << level; }
    // South-west corner on the UTM grid (false offsets included), metres.
    std::int64_t min_easting() const { return static_cast<std::int64_t>(x) * size_m(); }
    std::int64_t min_northing() const { return static_cast<std::int64_t>(y) * size_m(); }

    // The tile one level up containing this one. Requires level < kMaxLevel.
    TileKey parent() const;
    // One of the 4 tiles one level down; dx, dy in {0, 1} (1 = east/north).
    // Requires level > 0.
    TileKey child(int dx, int dy) const;

    // 64-bit packing (PLAN.md 4.4; also the physics bridge's TileKey.packed
    // source and the cache key):
    //   bits 54..59 zone (1..60), bit 53 hemisphere (1 = south),
    //   bits 48..52 level, bits 24..47 y + 2^23, bits 0..23 x + 2^23.
    // Numeric order of packed() equals the (zone, hemisphere, level, y, x)
    // order of operator<=>.
    std::uint64_t packed() const;
    static std::optional<TileKey> unpack(std::uint64_t packed);

    friend bool operator==(const TileKey& a, const TileKey& b) {
        return a.zone == b.zone && a.level == b.level && a.x == b.x && a.y == b.y;
    }
    friend std::strong_ordering operator<=>(const TileKey& a, const TileKey& b) {
        if (auto c = a.zone.zone <=> b.zone.zone; c != 0) return c;
        if (auto c = a.zone.hemisphere <=> b.zone.hemisphere; c != 0) return c;
        if (auto c = a.level <=> b.level; c != 0) return c;
        if (auto c = a.y <=> b.y; c != 0) return c;
        return a.x <=> b.x;
    }
};

// Floor division for b > 0 (rounds toward negative infinity).
constexpr std::int64_t floor_div(std::int64_t a, std::int64_t b) {
    const std::int64_t q = a / b;
    return (a % b != 0 && a < 0) ? q - 1 : q;
}

// The tile containing grid point (easting, northing) of zone at level.
// Exact: floor(E) is an exact integer, and floor(E / size) ==
// floor_div(floor(E), size) because size is an integer. Empty if a
// coordinate is not finite, the level is out of range, or an index does not
// fit the 24-bit field.
std::optional<TileKey> tile_key_at(geo::UtmZone zone, int level, double easting, double northing);

// "utm32n/0/1953/21582": scheme/level/x/y as used in the tile URLs (PLAN.md 3.4).
std::string to_string(const TileKey& key);

} // namespace g2m
