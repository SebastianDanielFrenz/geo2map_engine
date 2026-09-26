// bridges/physics_sim/tests/bridge_test_support.h - shared helpers for the
// g2m_bridge_tests acceptance tests and bench_bridge_fill (PLAN.md R2.2
// section 4b, G7/G8): synthetic DGM1 L0 height tiles generated from a
// per-cell height function (deterministic, no store files, never reads
// cache/), installing a square of them into a ResidentHeightSet, a fixed
// 64-bit LCG, and a TempDir for the fixture-release tests.
//
// HeightTile is 256 KiB (vault TOOL-039): every tile here lives on the heap
// behind a shared_ptr, never by value or as a stack local.
#pragma once

#include "g2m/core/geo/session_frame.h"
#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <system_error>
#include <vector>

namespace g2m_bridge_test {

inline constexpr g2m::geo::UtmZone k32N{32, g2m::geo::Hemisphere::North};
// The R2.2 fixture's session frame (PLAN.md R2.2 section 1c); the synthetic
// tests use the same anchor so every coordinate below is comparable.
inline constexpr std::int64_t kE0 = 474000;
inline constexpr std::int64_t kN0 = 5560000;

inline g2m::geo::SessionFrame test_frame() { return g2m::geo::SessionFrame(k32N, kE0, kN0); }

// Height in 1/256 m of the DGM1 cell at global grid column `c` (= floor(E))
// and row `r` (= floor(N)). The cell's sample sits at session-local
// (c - kE0 + 0.5, r - kN0 + 0.5).
using CellHeightFn = std::int32_t (*)(std::int64_t c, std::int64_t r);

// Writes the L0 tile `key` of `fn` into `out` (row-major, row 0 = south,
// HeightTile's own layout).
inline void fill_synthetic_tile(const g2m::TileKey& key, CellHeightFn fn, g2m::HeightTile& out) {
    out.key = key;
    out.has_nodata = false;
    const std::int64_t c0 = key.min_easting();
    const std::int64_t r0 = key.min_northing();
    for (int j = 0; j < g2m::kTerrainTileSamples; ++j) {
        for (int i = 0; i < g2m::kTerrainTileSamples; ++i) {
            out.h[static_cast<std::size_t>(j) * g2m::kTerrainTileSamples + static_cast<std::size_t>(i)] = fn(c0 + i, r0 + j);
        }
    }
}

// Thread-safe, lazily filled cache of synthetic L0 tiles for one height
// function (a loader worker may call get() concurrently with the test
// thread). Tiles are immutable once built and shared by every consumer.
class SyntheticTileCache {
public:
    explicit SyntheticTileCache(CellHeightFn fn) : fn_(fn) {}

    std::shared_ptr<const g2m::HeightTile> get(const g2m::TileKey& key) {
        std::lock_guard<std::mutex> lk(m_);
        auto it = tiles_.find(key.packed());
        if (it != tiles_.end()) return it->second;
        auto tile = std::make_shared<g2m::HeightTile>();
        fill_synthetic_tile(key, fn_, *tile);
        std::shared_ptr<const g2m::HeightTile> shared = tile;
        tiles_.emplace(key.packed(), shared);
        return shared;
    }

    CellHeightFn fn() const { return fn_; }

private:
    CellHeightFn fn_;
    std::mutex m_;
    std::map<std::uint64_t, std::shared_ptr<const g2m::HeightTile>> tiles_;
};

// Installs every L0 tile any sample of the (2r+1)^2 square of physics tiles
// around `centre` can touch.
inline void install_square(const g2m::phys::PhysicsTileGrid& grid, g2m::phys::ResidentHeightSet& resident,
                           g2m::phys::PhysTileIndex centre, int r, SyntheticTileCache& cache) {
    std::vector<g2m::TileKey> keys;
    grid.l0_keys_for_square(centre, r, keys);
    for (const auto& key : keys) resident.install(key, cache.get(key));
}

// Knuth's MMIX 64-bit LCG; the top 53 bits give a uniform double in [0, 1).
// Fixed sequence on every compiler/OS (no std:: distribution involved).
class Lcg {
public:
    explicit Lcg(std::uint64_t seed) : s_(seed) {}
    std::uint64_t next() {
        s_ = s_ * 6364136223846793005ull + 1442695040888963407ull;
        return s_;
    }
    double unit() { return static_cast<double>(next() >> 11) * 0x1.0p-53; }
    double uniform(double lo, double hi) { return lo + (hi - lo) * unit(); }
    // Uniform integer in [lo, hi] (the modulo bias is irrelevant here).
    std::int64_t range(std::int64_t lo, std::int64_t hi) {
        return lo + static_cast<std::int64_t>((next() >> 16) % static_cast<std::uint64_t>(hi - lo + 1));
    }

private:
    std::uint64_t s_;
};

// Same collision-avoidance shape as tests/layer/test_derive_terrain.cpp's
// own TempDir (duplicated: that one is private to tests/layer).
class TempDir {
public:
    explicit TempDir(const std::string& tag) {
        static std::atomic<std::uint64_t> counter{0};
        const auto now = static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        path_ = std::filesystem::temp_directory_path() /
                ("g2m_bridge_" + tag + "_" + std::to_string(now) + "_" + std::to_string(counter.fetch_add(1)));
        std::filesystem::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

} // namespace g2m_bridge_test
