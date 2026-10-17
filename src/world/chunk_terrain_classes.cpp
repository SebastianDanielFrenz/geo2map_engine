// g2m/world/chunk_terrain_classes.cpp - see include/g2m/world/chunk_terrain_classes.h.
#include "g2m/world/chunk_terrain_classes.h"

#include "g2m/layer/classifier.h"

#include <cstddef>
#include <cstdint>
#include <optional>

namespace g2m::world {

namespace {
constexpr int kChunksPerTile = kTerrainClassSamples / mesh::kChunkQuads; // 256 / 64 = 4
static_assert(kChunksPerTile * mesh::kChunkQuads + 1 == kTerrainClassSamples);
} // namespace

TileKey terrain_class_tile_for_chunk(const mesh::ChunkKey& key) {
    return TileKey{key.zone, key.level, static_cast<std::int32_t>(floor_div(key.cx, kChunksPerTile)),
                   static_cast<std::int32_t>(floor_div(key.cy, kChunksPerTile))};
}

void terrain_class_window(const mesh::ChunkKey& key, const TerrainClassTile* tile, const RenderRoadLod& lod,
                          mesh::ClassWindow& out) noexcept {
    out = mesh::ClassWindow{}; // all-Unknown (LandClass::Unknown == 0)
    if (tile == nullptr || !(tile->key == terrain_class_tile_for_chunk(key)) ||
        tile->land_class.size() != kTerrainClassNodes || tile->rank.size() != kTerrainClassNodes)
        return;

    // The road filter of this chunk level (the same lookup chunk_road_classes uses): no policy = every road node
    // Unknown; else rank >= min_rank kept.
    const std::optional<RasterParams> params = raster_params_for_render_level(key.level, lod);
    const bool roads_enabled = params.has_value();
    const int min_rank = roads_enabled ? params->min_rank : 0;

    const int i0 = static_cast<int>(key.cx - floor_div(key.cx, kChunksPerTile) * kChunksPerTile) * mesh::kChunkQuads;
    const int j0 = static_cast<int>(key.cy - floor_div(key.cy, kChunksPerTile) * kChunksPerTile) * mesh::kChunkQuads;
    for (int j = 0; j < mesh::kChunkVerts; ++j) {
        for (int i = 0; i < mesh::kChunkVerts; ++i) {
            const std::size_t node = TerrainClassTile::index(i0 + i, j0 + j);
            const std::uint8_t rank = tile->rank[node];
            const bool keep = rank == 0 || (roads_enabled && static_cast<int>(rank) >= min_rank);
            out.c[static_cast<std::size_t>(j) * static_cast<std::size_t>(mesh::kChunkVerts) + static_cast<std::size_t>(i)] =
                keep ? tile->land_class[node] : static_cast<std::uint8_t>(LandClass::Unknown);
        }
    }
}

void fill_chunk_terrain_classes(const mesh::ChunkKey& key, const TerrainClassLookup& lookup, mesh::ClassWindow& out) {
    if (lookup.fn == nullptr) {
        out = mesh::ClassWindow{};
        return;
    }
    const std::shared_ptr<const TerrainClassTile> tile = lookup.fn(lookup.ctx, terrain_class_tile_for_chunk(key));
    terrain_class_window(key, tile.get(), lookup.lod != nullptr ? *lookup.lod : default_render_road_lod(), out);
}

} // namespace g2m::world
