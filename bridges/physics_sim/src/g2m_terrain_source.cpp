#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "g2m/phys/fill_physics_heights.h"

#include "ps/backend/shape_desc.h"

#include <algorithm>
#include <cmath>

namespace g2m::ps_bridge {

G2mTerrainSource::G2mTerrainSource(g2m::phys::PhysicsTileGrid grid,
                                   std::shared_ptr<const g2m::phys::ResidentHeightSet> resident, ps::SurfaceId surface)
    : grid_(std::move(grid)), resident_(std::move(resident)), surface_(surface) {}

ps::real G2mTerrainSource::tile_size_m() const { return static_cast<ps::real>(grid_.tile_size_m()); }

std::uint32_t G2mTerrainSource::samples_per_tile() const { return static_cast<std::uint32_t>(grid_.samples()); }

ps::terrain::TileKey G2mTerrainSource::key_for_world_xy(ps::real x, ps::real y) const {
    const g2m::phys::PhysTileIndex index = grid_.index_for_local(static_cast<double>(x), static_cast<double>(y));
    return ps::terrain::TileKey{g2m::phys::PhysicsTileGrid::pack(index)};
}

ps::terrain::TileKey G2mTerrainSource::key_offset(ps::terrain::TileKey key, std::int32_t dx, std::int32_t dy) const {
    g2m::phys::PhysTileIndex index = g2m::phys::PhysicsTileGrid::unpack(key.packed);
    index.ix += dx;
    index.iy += dy;
    return ps::terrain::TileKey{g2m::phys::PhysicsTileGrid::pack(index)};
}

ps::Vec3 G2mTerrainSource::tile_origin_xy(ps::terrain::TileKey key) const {
    const g2m::phys::PhysTileIndex index = g2m::phys::PhysicsTileGrid::unpack(key.packed);
    const std::array<double, 2> origin = grid_.origin_local(index);
    return ps::Vec3(static_cast<ps::real>(origin[0]), static_cast<ps::real>(origin[1]), 0.0);
}

void G2mTerrainSource::fill_tile(ps::terrain::TileKey key, ps::terrain::TileSample& out) const {
    const g2m::phys::PhysTileIndex index = g2m::phys::PhysicsTileGrid::unpack(key.packed);

    const g2m::phys::FillResult result =
        g2m::phys::fill_physics_heights<ps::real>(grid_, index, *resident_, out.heights.data(), ps::kHeightfieldNoCollision);

    // Every cell gets the same single surface (PLAN.md R2.2 1b) - `out`'s
    // vector is already sized by the caller, this only overwrites content.
    std::fill(out.cell_surface_ids.begin(), out.cell_surface_ids.end(), surface_);

    out.samples_per_tile = samples_per_tile();
    out.tile_size_m = tile_size_m();
    out.tile_origin_xy = tile_origin_xy(key);

    // Priority matches g2m::phys::FillResult's own documented priority:
    // MissingTile over HadNoData.
    if (result == g2m::phys::FillResult::MissingTile) {
        fill_miss_count_.fetch_add(1, std::memory_order_relaxed);
    } else if (result == g2m::phys::FillResult::HadNoData) {
        nodata_fill_count_.fetch_add(1, std::memory_order_relaxed);
    }
}

std::uint64_t G2mTerrainSource::fill_miss_count() const noexcept {
    return fill_miss_count_.load(std::memory_order_relaxed);
}

std::uint64_t G2mTerrainSource::nodata_fill_count() const noexcept {
    return nodata_fill_count_.load(std::memory_order_relaxed);
}

ps::terrain::TerrainConfig make_terrain_config(double interest_radius_m, std::uint32_t fills_per_tick) {
    ps::terrain::TerrainConfig cfg;
    cfg.suppress_terrain_under_static_geometry = false;
    cfg.precompute_suppression_at_load = false;

    // Literal 255 m, not grid_.tile_size_m() - this is a free function (no
    // grid instance), and the plan's own worked example (r=400 -> 49) is
    // stated against the fixed 255 m physics-tile size every
    // G2mTerrainSource uses (PhysicsTileGrid's default kPhysSamples=256).
    const int r = static_cast<int>(std::ceil(interest_radius_m / 255.0));
    const std::size_t side = static_cast<std::size_t>(2 * r + 3);
    cfg.max_resident_tiles = side * side;
    cfg.max_tile_fills_per_tick = fills_per_tick;
    return cfg;
}

} // namespace g2m::ps_bridge
