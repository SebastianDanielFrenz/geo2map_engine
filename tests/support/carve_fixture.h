// tests/support/carve_fixture.h - street fidelity P2 slice 9: the carved terrain of a fixture world, built the way
// TerrainCarvedDeriver does it (the 3 x 3 elev.base window around a level-0 tile and the served roads.geom level-2
// tiles of carve_road_geom_keys), from a DEM function (flat 100 m, or a terrain function as in road_fixture.h).
// The roads.geom and DEM tiles are cached per key. Header-only, test-only.
#pragma once

#include "road_fixture.h"

#include "g2m/layer/terrain_carve.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

namespace g2m_test {

class CarveFixture {
public:
    CarveFixture(const g2m::RoadGraph& world, std::function<double(double, double)> height_m = {})
        : world_(&world), height_m_(std::move(height_m)) {}

    // The served roads.geom tile of a level-2 key (an empty tile where the world has no ways).
    const g2m::RoadGeomTile& geom(const g2m::TileKey& key) {
        auto it = geom_.find(key);
        if (it != geom_.end()) return *it->second;
        auto tile = served_geometry_in_tile(*world_, key, height_m_);
        if (!tile) throw std::runtime_error("carve fixture: geometry " + tile.error().message);
        return *geom_.emplace(key, std::make_unique<g2m::RoadGeomTile>(std::move(*tile))).first->second;
    }
    const g2m::HeightTile& dem(const g2m::TileKey& key) {
        auto it = dem_.find(key);
        if (it != dem_.end()) return *it->second;
        auto tile = std::make_unique<g2m::HeightTile>(); tile->key = key; fill_dem(*tile, height_m_);
        return *dem_.emplace(key, std::move(tile)).first->second;
    }
    // The 3 x 3 elev.base window of a level-0 tile; `centre_only` leaves the 8 neighbours absent (a negative control).
    std::array<const g2m::HeightTile*, 9> window(const g2m::TileKey& key0, bool centre_only = false) {
        std::array<const g2m::HeightTile*, 9> w{};
        for (int dy = -1; dy <= 1; ++dy)
            for (int dx = -1; dx <= 1; ++dx) {
                if (centre_only && (dx != 0 || dy != 0)) continue;
                w[static_cast<std::size_t>((dy + 1) * 3 + (dx + 1))] = &dem(g2m::TileKey{key0.zone, 0, key0.x + dx, key0.y + dy});
            }
        return w;
    }
    // The roads.geom inputs of a level-0 tile; `own_only` keeps only the level-2 tile containing it (a negative control).
    std::vector<g2m::CarveGeomInput> roads(const g2m::TileKey& key0, bool own_only = false) {
        std::vector<g2m::CarveGeomInput> in;
        const g2m::TileKey own = key0.parent().parent();
        for (const auto& k : g2m::carve_road_geom_keys(key0)) {
            if (own_only && k != own) continue;
            in.push_back({k, &geom(k)});
        }
        return in;
    }
    g2m::Result<g2m::HeightTile> carve(const g2m::TileKey& key0, const g2m::CarveOptions& options = {}, unsigned workers = 1,
            g2m::CarveStats* stats = nullptr) {
        const auto w = window(key0);
        const auto r = roads(key0);
        return g2m::carve_terrain_tile(key0, w, r, g2m::builtin_carve_default(), options, workers, stats);
    }

    // A model of one level-0 tile, with the terrain it reads (kept alive together).
    struct Model {
        std::unique_ptr<g2m::CarveTerrain> terrain;
        std::optional<g2m::CarveModel> model;
        g2m::CarveStats stats;
    };
    Model model(const g2m::TileKey& key0, const g2m::CarveOptions& options = {}, bool own_roads_only = false,
            bool centre_dem_only = false) {
        Model m;
        const auto w = window(key0, centre_dem_only);
        auto terrain = g2m::CarveTerrain::make(key0, w, g2m::builtin_carve_default().smoothing);
        if (!terrain) throw std::runtime_error("carve fixture: terrain " + terrain.error().message);
        m.terrain = std::make_unique<g2m::CarveTerrain>(std::move(*terrain));
        const auto r = roads(key0, own_roads_only);
        auto model = g2m::CarveModel::build(r, g2m::builtin_carve_default(), *m.terrain, options, g2m::builtin_road_rules_de(), &m.stats);
        if (!model) throw std::runtime_error("carve fixture: model " + model.error().message);
        m.model.emplace(std::move(*model));
        return m;
    }

    // Level-0 tiles touched by a node of the world (ascending).
    std::vector<g2m::TileKey> level0_tiles() const {
        std::set<g2m::TileKey> keys;
        for (const auto& node : world_->nodes) {
            const auto key = g2m::tile_key_at(kZone32N, 0, static_cast<double>(node.position.x) / 1000.0, static_cast<double>(node.position.y) / 1000.0);
            if (key) keys.insert(*key);
        }
        return {keys.begin(), keys.end()};
    }

private:
    const g2m::RoadGraph* world_;
    std::function<double(double, double)> height_m_;
    std::map<g2m::TileKey, std::unique_ptr<g2m::RoadGeomTile>> geom_;
    std::map<g2m::TileKey, std::unique_ptr<g2m::HeightTile>> dem_;
};

// The committed OSM fixtures as worlds.
inline g2m::RoadGraph fixture_world(const std::string& file) {
    const auto data = read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + file);
    auto world = g2m::build_road_graph(data, kZone32N);
    if (!world) throw std::runtime_error("fixture world: " + world.error().message);
    return std::move(*world);
}

// The tilted bumpy rough DEM of slice 9: a 3 % grade east, +-2 m bumps (wavelengths 23 m and 31 m, as the slice 6-8
// world tests' sin/cos DEM) and +-0.1 m of cell noise (DGM1-like roughness, so raw and smoothed terrain differ). Built
// from + - * / and an integer hash only, so the DEM - and the g3d golden on it - is the same on every platform.
inline double carve_wave(double u) {
    const double f = u - std::floor(u), x = 2.0 * f - 1.0, q = x * x - 1.0;
    return 2.0 * q * q - 1.0;   // 1 at f = 0.5, -1 at f = 0 and 1, C1
}
inline double carve_noise(double e, double n) {
    const auto ie = static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(e)));
    const auto in = static_cast<std::uint64_t>(static_cast<std::int64_t>(std::floor(n)));
    std::uint64_t h = (ie * 0x9E3779B97F4A7C15ull) ^ (in * 0xC2B2AE3D27D4EB4Full);
    h ^= h >> 29; h *= 0xBF58476D1CE4E5B9ull; h ^= h >> 32;
    return (static_cast<double>(h % 201) - 100.0) / 1000.0;
}
inline double carve_tilted_dem(double e, double n) {
    return 100.0 + 0.03 * (e - 455000.0) + 2.0 * carve_wave(e / 23.0) * carve_wave(n / 31.0) + carve_noise(e, n);
}

} // namespace g2m_test
