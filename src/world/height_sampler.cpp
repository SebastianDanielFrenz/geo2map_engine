// g2m/world/height_sampler.cpp - see include/g2m/world/height_sampler.h.
#include "g2m/world/height_sampler.h"

#include "g2m/layer/terrain_layers.h"

#include <cmath>
#include <limits>

namespace g2m::world {

namespace {

constexpr std::int64_t kN = kTerrainTileSamples;

// One lattice corner: h in metres when ok. ok is false for NoData, a missing
// tile or an invalid key.
struct Corner {
    double h = 0.0;
    bool ok = false;
};

// The tile holding global cell (c, r), or nullptr if it has no valid key or
// the lookup has no such tile. `cached` short-circuits the common case where
// the four corners share one tile.
const HeightTile* tile_for(const L0TileLookupFn& lookup, geo::UtmZone zone, std::int64_t c, std::int64_t r,
                           const HeightTile* cached, bool& key_valid) {
    TileKey key;
    key.zone = zone;
    key.level = 0;
    key.x = static_cast<std::int32_t>(floor_div(c, kN));
    key.y = static_cast<std::int32_t>(floor_div(r, kN));
    key_valid = key.valid();
    if (!key_valid) {
        return nullptr;
    }
    return (cached != nullptr && cached->key == key) ? cached : lookup(key);
}

} // namespace

std::optional<double> sample_height_bilinear(const L0TileLookupFn& lookup, geo::UtmZone zone, std::int64_t e0,
                                             std::int64_t n0, double x, double y) {
    // Global cell-centre lattice: column c's centre is at easting c + 0.5.
    const double u = static_cast<double>(e0) + x - 0.5;
    const double v = static_cast<double>(n0) + y - 0.5;
    if (!std::isfinite(u) || !std::isfinite(v)) {
        return std::nullopt;
    }
    const double fu = std::floor(u);
    const double fv = std::floor(v);
    const auto c0 = static_cast<std::int64_t>(fu);
    const auto r0 = static_cast<std::int64_t>(fv);
    const double tx = u - fu;
    const double ty = v - fv;

    const HeightTile* cached = nullptr; // the 4 corners usually share one tile
    double h[2][2] = {};
    for (int dj = 0; dj < 2; ++dj) {
        for (int di = 0; di < 2; ++di) {
            const std::int64_t c = c0 + di;
            const std::int64_t r = r0 + dj;
            TileKey key;
            key.zone = zone;
            key.level = 0;
            key.x = static_cast<std::int32_t>(floor_div(c, kN));
            key.y = static_cast<std::int32_t>(floor_div(r, kN));
            if (!key.valid()) {
                return std::nullopt;
            }
            const HeightTile* tile = (cached != nullptr && cached->key == key) ? cached : lookup(key);
            if (tile == nullptr) {
                return std::nullopt;
            }
            cached = tile;
            const std::int64_t i = c - static_cast<std::int64_t>(key.x) * kN;
            const std::int64_t j = r - static_cast<std::int64_t>(key.y) * kN;
            const std::int32_t raw = tile->h[static_cast<std::size_t>(j * kN + i)];
            if (raw == kHeightNoData) {
                return std::nullopt;
            }
            h[dj][di] = static_cast<double>(raw) / 256.0;
        }
    }
    const double south = h[0][0] + tx * (h[0][1] - h[0][0]);
    const double north = h[1][0] + tx * (h[1][1] - h[1][0]);
    return south + ty * (north - south);
}

std::optional<double> sample_height_exact(const L0TileLookupFn& lookup, geo::UtmZone zone, std::int64_t e0,
                                          std::int64_t n0, double x, double y) {
    const double u = static_cast<double>(e0) + x - 0.5;
    const double v = static_cast<double>(n0) + y - 0.5;
    if (!std::isfinite(u) || !std::isfinite(v)) {
        return std::nullopt;
    }
    const double fu = std::floor(u);
    const double fv = std::floor(v);
    const auto c0 = static_cast<std::int64_t>(fu);
    const auto r0 = static_cast<std::int64_t>(fv);
    const double tx = u - fu;
    const double ty = v - fv;

    // corner[dj][di]: dj = row offset (north), di = column offset (east).
    Corner corner[2][2];
    const HeightTile* cached = nullptr;
    for (int dj = 0; dj < 2; ++dj) {
        for (int di = 0; di < 2; ++di) {
            const std::int64_t c = c0 + di;
            const std::int64_t r = r0 + dj;
            bool key_valid = false;
            const HeightTile* tile = tile_for(lookup, zone, c, r, cached, key_valid);
            if (tile == nullptr) {
                continue;
            }
            cached = tile;
            const std::int64_t i = c - floor_div(c, kN) * kN;
            const std::int64_t j = r - floor_div(r, kN) * kN;
            const std::int32_t raw = tile->h[static_cast<std::size_t>(j * kN + i)];
            if (raw == kHeightNoData) {
                continue;
            }
            corner[dj][di].h = static_cast<double>(raw) / 256.0;
            corner[dj][di].ok = true;
        }
    }

    const Corner& c00 = corner[0][0];
    const Corner& c10 = corner[0][1]; // east
    const Corner& c01 = corner[1][0]; // north
    const Corner& c11 = corner[1][1];
    if (tx + ty <= 1.0) {
        if (!c00.ok || !c10.ok || !c01.ok) {
            return std::nullopt;
        }
        return c00.h + tx * (c10.h - c00.h) + ty * (c01.h - c00.h);
    }
    if (!c11.ok || !c10.ok || !c01.ok) {
        return std::nullopt;
    }
    return c11.h + (1.0 - tx) * (c01.h - c11.h) + (1.0 - ty) * (c10.h - c11.h);
}

} // namespace g2m::world
