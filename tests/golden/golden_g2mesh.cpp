// tests/golden/golden_g2mesh.cpp - G2mesh golden hashes (PLAN.md G2.3): a
// SEPARATE golden set from "g2" (G2.2 extends "g2" in parallel on its own
// branch) - g2m_mesh's select_chunks + build_chunk over 2 fixed camera
// positions and a deterministic SYNTHETIC (analytic, g2m::det::sin-based)
// HeightTile source, so this file needs neither testdata/fixtures nor
// G2.2's real height pyramid. Each entry hashes, in select_chunks' own
// output order, every selected chunk's ChunkKey (zone/level/cx/cy) plus its
// built TerrainChunkMesh::content_hash; "g2mesh.cam0"/"g2mesh.cam1" plus a
// combined "all" entry mirror golden_g2.cpp's own shape.

#include "golden.h"

#include "g2m/core/det_math.h"
#include "g2m/core/hash.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/mesh/lod_select.h"
#include "g2m/mesh/terrain_chunk.h"

#include <cstdint>
#include <unordered_map>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;
using g2m::HeightTile;
using g2m::TileKey;
namespace mesh = g2m::mesh;
namespace geo = g2m::geo;

// Same shape as golden_g2.cpp's own file-local Acc (deliberately not shared -
// see that file's header comment).
class Acc {
public:
    void u(std::uint64_t v) {
        h_.u64(v);
        ++n_;
    }
    Entry done(std::string name) const { return {std::move(name), n_, h_.value()}; }

private:
    Fnv1a64 h_;
    std::uint64_t n_ = 0;
};

// Deterministic analytic terrain: h(x, y) = 25 m * sin(2*pi*x/300) *
// sin(2*pi*y/300), Cell-registered sample positions (height_tile.h's own
// documented formula) - g2m::det::sin only, so this golden is bit-identical
// Windows/Linux (same reason core/src never calls the platform libm).
std::int32_t synthetic_height(int level, std::int64_t ai, std::int64_t aj) {
    const double step_m = static_cast<double>(std::int64_t{1} << level);
    const double x = (static_cast<double>(ai) + 0.5) * step_m;
    const double y = (static_cast<double>(aj) + 0.5) * step_m;
    constexpr double kAmplitudeM = 25.0;
    constexpr double kWavelengthM = 300.0;
    const double k = 2.0 * g2m::det::kPi / kWavelengthM;
    const double h = kAmplitudeM * g2m::det::sin(k * x) * g2m::det::sin(k * y);
    return static_cast<std::int32_t>(h * 256.0);
}

class SyntheticSource {
public:
    static const HeightTile* lookup(void* ctx, const TileKey& key) {
        return static_cast<SyntheticSource*>(ctx)->tile_for(key);
    }

private:
    const HeightTile* tile_for(const TileKey& key) {
        const std::uint64_t packed = key.packed();
        auto it = cache_.find(packed);
        if (it != cache_.end()) return &it->second;
        HeightTile tile{};
        tile.key = key;
        for (int j = 0; j < g2m::kTerrainTileSamples; ++j) {
            const std::int64_t aj = static_cast<std::int64_t>(key.y) * g2m::kTerrainTileSamples + j;
            for (int i = 0; i < g2m::kTerrainTileSamples; ++i) {
                const std::int64_t ai = static_cast<std::int64_t>(key.x) * g2m::kTerrainTileSamples + i;
                tile.h[static_cast<std::size_t>(j) * g2m::kTerrainTileSamples + static_cast<std::size_t>(i)] =
                    synthetic_height(key.level, ai, aj);
            }
        }
        auto [inserted, ok] = cache_.emplace(packed, tile);
        (void)ok;
        return &inserted->second;
    }
    std::unordered_map<std::uint64_t, HeightTile> cache_;
};

Entry one_camera(const char* name, double cam_e, double cam_n) {
    const geo::UtmZone zone{32, geo::Hemisphere::North};
    mesh::LodParams params;
    params.zone = zone;
    params.range0_m = 250.0;
    params.max_level = 6;
    params.max_distance_m = 6000.0;

    std::vector<mesh::ChunkKey> chunks;
    mesh::select_chunks(cam_e, cam_n, params, chunks);

    SyntheticSource src;
    Acc a;
    a.u(chunks.size());
    for (const mesh::ChunkKey& key : chunks) {
        a.u(static_cast<std::uint64_t>(key.zone.zone));
        a.u(static_cast<std::uint64_t>(key.zone.hemisphere));
        a.u(key.level);
        a.u(static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.cx)));
        a.u(static_cast<std::uint64_t>(static_cast<std::uint32_t>(key.cy)));

        mesh::HeightWindow window;
        mesh::gather_window(key, &SyntheticSource::lookup, &src, window);
        mesh::TerrainChunkMesh out;
        mesh::build_chunk(key, window, /*classes=*/nullptr, out);
        a.u(out.content_hash);
    }
    return a.done(name);
}

} // namespace

std::vector<Entry> compute_g2mesh() {
    std::vector<Entry> out;
    out.push_back(one_camera("g2mesh.cam0", 12345.5, -6789.25));
    out.push_back(one_camera("g2mesh.cam1", -50000.0, 30500.75));

    Fnv1a64 all;
    std::uint64_t n = 0;
    for (const Entry& e : out) {
        all.str(e.name).u64(e.count).u64(e.hash);
        n += e.count;
    }
    out.push_back({"all", n, all.value()});
    return out;
}

} // namespace g2m_golden
