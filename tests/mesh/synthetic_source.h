// tests/mesh/synthetic_source.h - test-only synthetic HeightTile source for
// g2m_mesh tests (PLAN.md G2.3): every height is an ANALYTIC, deterministic
// function of the absolute (level, i, j) sample index, so two chunks built
// from overlapping windows always see bit-identical samples in the overlap
// (no fixture I/O, no G2.2 pyramid dependency).
#pragma once

#include "g2m/core/det_math.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/terrain_layers.h"

#include <cstdint>
#include <functional>
#include <unordered_map>

namespace g2m_test {

// A pure function of the ABSOLUTE sample index (level, ai, aj) -> height in
// 1/256 m units (or g2m::kHeightNoData). Absolute index: ai = tile.x *
// kTerrainTileSamples + local_i (matches gather_window's own decomposition
// and HeightTile's own Cell-registration doc comment).
using HeightFn = std::function<std::int32_t(int level, std::int64_t ai, std::int64_t aj)>;

// Flat terrain at a fixed height (metres).
inline HeightFn flat_height(double metres) {
    const std::int32_t units = static_cast<std::int32_t>(metres * 256.0);
    return [units](int, std::int64_t, std::int64_t) { return units; };
}

// Sinusoidal terrain: h(x, y) = amplitude_m * sin(2*pi*x/wavelength_m) *
// sin(2*pi*y/wavelength_m), x/y the sample's absolute Cell-registered world
// position at `level` (metres) - g2m::det::sin, so the result is bit-identical
// Windows/Linux (load-bearing for tests/golden/golden_g2mesh.cpp).
inline HeightFn sinusoidal_height(double amplitude_m, double wavelength_m) {
    return [amplitude_m, wavelength_m](int level, std::int64_t ai, std::int64_t aj) {
        const double step_m = static_cast<double>(std::int64_t{1} << level);
        const double x = (static_cast<double>(ai) + 0.5) * step_m;
        const double y = (static_cast<double>(aj) + 0.5) * step_m;
        const double k = 2.0 * g2m::det::kPi / wavelength_m;
        const double h = amplitude_m * g2m::det::sin(k * x) * g2m::det::sin(k * y);
        return static_cast<std::int32_t>(h * 256.0);
    };
}

// Adapts a HeightFn to g2m::mesh::TileLookup (a plain function pointer +
// opaque ctx): construct one instance, pass &instance as ctx and &lookup as
// the TileLookup. Tiles are generated lazily and cached (a chunk's halo
// window touches at most 9 distinct tiles).
class SyntheticHeightSource {
public:
    explicit SyntheticHeightSource(HeightFn fn) : fn_(std::move(fn)) {}

    static const g2m::HeightTile* lookup(void* ctx, const g2m::TileKey& key) {
        return static_cast<SyntheticHeightSource*>(ctx)->tile_for(key);
    }

private:
    const g2m::HeightTile* tile_for(const g2m::TileKey& key) {
        const std::uint64_t packed = key.packed();
        auto it = cache_.find(packed);
        if (it != cache_.end()) return &it->second;

        g2m::HeightTile tile{};
        tile.key = key;
        tile.has_nodata = false;
        for (int j = 0; j < g2m::kTerrainTileSamples; ++j) {
            const std::int64_t aj = static_cast<std::int64_t>(key.y) * g2m::kTerrainTileSamples + j;
            for (int i = 0; i < g2m::kTerrainTileSamples; ++i) {
                const std::int64_t ai = static_cast<std::int64_t>(key.x) * g2m::kTerrainTileSamples + i;
                tile.h[static_cast<std::size_t>(j) * g2m::kTerrainTileSamples + static_cast<std::size_t>(i)] =
                    fn_(key.level, ai, aj);
            }
        }
        auto [inserted, ok] = cache_.emplace(packed, tile);
        (void)ok;
        return &inserted->second;
    }

    HeightFn fn_;
    std::unordered_map<std::uint64_t, g2m::HeightTile> cache_;
};

} // namespace g2m_test
