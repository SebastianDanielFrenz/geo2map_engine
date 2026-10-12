// g2m/world/chunk_road_classes.cpp - see include/g2m/world/chunk_road_classes.h.
// Ported from racing_game core/src/road_classes.cpp (a8a1b13) statement for
// statement.
#include "g2m/world/chunk_road_classes.h"

#include <algorithm>
#include <cstddef>
#include <optional>
#include <span>

namespace g2m::world {

namespace {

// Allocation-free RoadBlockLookupFn adapter over a ClassLookup: ctx is the
// ClassLookup itself. An empty or null result is RoadBlockStatus::None.
RoadBlockLookupResult class_lookup_block_fn(void* ctx, const TileKey& l2_key) {
    const auto* lookup = static_cast<const ClassLookup*>(ctx);
    std::shared_ptr<const std::vector<RoadSegment>> segs = lookup->fn(lookup->ctx, l2_key);
    if (!segs || segs->empty()) {
        return RoadBlockLookupResult{RoadBlockStatus::None, {}};
    }
    return RoadBlockLookupResult{RoadBlockStatus::Segments, std::span<const RoadSegment>(*segs)};
}

} // namespace

Lattice road_class_lattice(const mesh::ChunkKey& key) {
    const std::int64_t spacing_mm = (std::int64_t{1} << key.level) * 1000;
    // terrain_chunk.h's vertex-placement formula: vertex (i, j) sits at
    // absolute E = (64*cx + i) * 2^level + 0.5 (same for N) - a FIXED
    // 500 mm offset, not scaled by 2^level.
    const std::int64_t origin_e_mm = static_cast<std::int64_t>(key.cx) * 64 * spacing_mm + 500;
    const std::int64_t origin_n_mm = static_cast<std::int64_t>(key.cy) * 64 * spacing_mm + 500;
    return Lattice{origin_e_mm, origin_n_mm, spacing_mm, mesh::kChunkVerts, mesh::kChunkVerts};
}

std::vector<TileKey> src_osm_tiles_for_chunk(const mesh::ChunkKey& key) {
    const std::int64_t size_m = 64 * (std::int64_t{1} << key.level);
    const std::int64_t e_min = static_cast<std::int64_t>(key.cx) * size_m;
    const std::int64_t n_min = static_cast<std::int64_t>(key.cy) * size_m;
    const std::int64_t e_max = e_min + size_m;
    const std::int64_t n_max = n_min + size_m;

    constexpr int kOsmLevel = 2; // g2m.src.osm tiles are level 2 (1024 m), docs/formats/src_osm.md
    constexpr std::int64_t kOsmTileSizeM = TileKey::kLevel0SizeM << kOsmLevel;

    const std::int64_t x0 = floor_div(e_min, kOsmTileSizeM);
    const std::int64_t x1 = floor_div(e_max, kOsmTileSizeM);
    const std::int64_t y0 = floor_div(n_min, kOsmTileSizeM);
    const std::int64_t y1 = floor_div(n_max, kOsmTileSizeM);

    std::vector<TileKey> out;
    out.reserve(static_cast<std::size_t>((x1 - x0 + 1) * (y1 - y0 + 1)));
    for (std::int64_t ty = y0; ty <= y1; ++ty) {
        for (std::int64_t tx = x0; tx <= x1; ++tx) {
            out.push_back(TileKey{key.zone, kOsmLevel, static_cast<std::int32_t>(tx), static_cast<std::int32_t>(ty)});
        }
    }
    return out;
}

void rasterize_chunk_road_classes(const mesh::ChunkKey& key, const ClassLookup& lookup, mesh::ClassWindow& out) {
    out = mesh::ClassWindow{}; // all-Unknown (LandClass::Unknown == 0)
    if (lookup.fn == nullptr) {
        return;
    }
    const std::optional<RasterParams> params = raster_params_for_render_level(key.level);
    if (!params.has_value()) {
        return;
    }

    const Lattice lattice = road_class_lattice(key);
    const std::size_t sample_count = static_cast<std::size_t>(lattice.nx) * static_cast<std::size_t>(lattice.ny);
    std::vector<std::uint8_t> land_class(sample_count, static_cast<std::uint8_t>(LandClass::Unknown));
    std::vector<std::uint8_t> surface(sample_count, static_cast<std::uint8_t>(SurfaceKind::Unknown));

    // Owner rule: partition the chunk's lattice into aligned level-2 (1024 m)
    // g2m.src.osm blocks and paint each from only its own owner tile's
    // segments. `ctx` is `&lookup`, consumed synchronously.
    constexpr int kOsmBlockLevel = 2;
    rasterize_road_blocks(lattice, kOsmBlockLevel, key.zone, *params, &class_lookup_block_fn,
                          const_cast<void*>(static_cast<const void*>(&lookup)), land_class, surface);

    // land_class is row-major j*nx+i, matching ClassWindow's layout (row 0 =
    // south, nx == ny == kChunkVerts): a straight copy. `surface` has no
    // ClassWindow home and is discarded.
    std::copy(land_class.begin(), land_class.end(), out.c.begin());
}

} // namespace g2m::world
