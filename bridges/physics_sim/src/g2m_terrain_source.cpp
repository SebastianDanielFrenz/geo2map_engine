#include "g2m/ps_bridge/g2m_terrain_source.h"

#include "g2m/phys/fill_physics_heights.h"
#include "g2m/phys/fill_physics_surfaces.h"

#include "ps/backend/shape_desc.h"
#include "ps/types.h"

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace g2m::ps_bridge {

// fill_physics_surfaces (g2m/phys/fill_physics_surfaces.h) writes into a
// std::span<std::uint16_t>; passing out.cell_surface_ids (a
// std::vector<ps::SurfaceId>) straight through relies on the two types being
// the same width - guarded here rather than in ps/types.h itself, since this
// is the one place g2m_phys's road painting meets ps::SurfaceId.
static_assert(std::is_same_v<ps::SurfaceId, std::uint16_t>, "ps::SurfaceId must stay uint16_t for fill_physics_surfaces");

G2mTerrainSource::G2mTerrainSource(g2m::phys::PhysicsTileGrid grid,
                                   std::shared_ptr<const g2m::phys::ResidentHeightSet> resident, ps::SurfaceId surface)
    : grid_(std::move(grid)), resident_(std::move(resident)), surface_(surface) {}

G2mTerrainSource::G2mTerrainSource(g2m::phys::PhysicsTileGrid grid,
                                   std::shared_ptr<const g2m::phys::ResidentHeightSet> resident, RoadSurfaceConfig road)
    : grid_(std::move(grid)), resident_(std::move(resident)), surface_(road.off_road), road_(std::move(road)) {}

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
    fill_from(key, *resident_, out);
}

void G2mTerrainSource::fill_tile_from(ps::terrain::TileKey key, const void* inputs,
                                     ps::terrain::TileSample& out) const {
    fill_from(key, *static_cast<const g2m::phys::ResidentHeightSet*>(inputs), out);
}

bool G2mTerrainSource::acquire_prefetch_inputs(ps::terrain::TileKey key,
                                              std::shared_ptr<const void>& out) const {
    auto view = resident_->snapshot();
    if (!view || !footprint_ready(key, *view, false)) return false;
    out = std::move(view);
    return true;
}

bool G2mTerrainSource::prefetch_inputs_current(ps::terrain::TileKey key, const void* inputs) const {
    return inputs && footprint_ready(key, *static_cast<const g2m::phys::ResidentHeightSet*>(inputs), true);
}

bool G2mTerrainSource::footprint_ready(ps::terrain::TileKey key,
                                       const g2m::phys::ResidentHeightSet& view, bool compare_live) const {
    const auto index = g2m::phys::PhysicsTileGrid::unpack(key.packed);
    const std::int64_t ts = grid_.samples() - 1;
    const auto col = grid_.frame().e0_m() + ts * index.ix;
    const auto row = grid_.frame().n0_m() + ts * index.iy;
    // Include both border height samples; cell-centre road inputs are a subset.
    for (auto y = g2m::floor_div(row, 256); y <= g2m::floor_div(row + ts, 256); ++y) {
        for (auto x = g2m::floor_div(col, 256); x <= g2m::floor_div(col + ts, 256); ++x) {
            g2m::TileKey l0;
            l0.zone = grid_.frame().zone();
            l0.level = 0;
            l0.x = static_cast<std::int32_t>(x);
            l0.y = static_cast<std::int32_t>(y);
            const g2m::HeightTile* height = nullptr;
            std::shared_ptr<const std::vector<g2m::RoadSegment>> roads;
            const auto state = view.find(l0, &height, &roads);
            if (state == g2m::phys::ResidentHeightSet::Lookup::Missing) return false;
            if (road_ && state == g2m::phys::ResidentHeightSet::Lookup::Resident && !roads) return false;
            if (compare_live) {
                const g2m::HeightTile* live_height = nullptr;
                std::shared_ptr<const std::vector<g2m::RoadSegment>> live_roads;
                if (resident_->find(l0, &live_height, &live_roads) != state || height != live_height ||
                    (road_ && roads.get() != live_roads.get())) return false;
            }
        }
    }
    return true;
}

void G2mTerrainSource::fill_from(ps::terrain::TileKey key, const g2m::phys::ResidentHeightSet& resident,
                                 ps::terrain::TileSample& out) const {
    const g2m::phys::PhysTileIndex index = g2m::phys::PhysicsTileGrid::unpack(key.packed);

    const g2m::phys::FillResult result =
        g2m::phys::fill_physics_heights<ps::real>(grid_, index, resident, out.heights.data(), ps::kHeightfieldNoCollision);

    if (road_) {
        // Road mode (PLAN.md G2.5a-grip section 5, commit G-c): per-cell
        // grip from OSM roads instead of one flat surface. `out`'s vector is
        // already sized by the caller (ITerrainSource::fill_tile's own
        // contract) - fill_physics_surfaces only overwrites content, same as
        // the std::fill below in uniform mode.
        const g2m::RoadBlockStats road_stats =
            g2m::phys::fill_physics_surfaces(grid_, index, resident, road_->lut, road_->off_road, out.cell_surface_ids);
        if (road_stats.missing_blocks > 0) fill_miss_count_.fetch_add(1, std::memory_order_relaxed);
    } else {
        // Every cell gets the same single surface (PLAN.md R2.2 1b) - `out`'s
        // vector is already sized by the caller, this only overwrites content.
        std::fill(out.cell_surface_ids.begin(), out.cell_surface_ids.end(), surface_);
    }

    out.samples_per_tile = samples_per_tile();
    out.tile_size_m = tile_size_m();
    out.tile_origin_xy = tile_origin_xy(key);

    // Priority matches g2m::phys::FillResult's own documented priority:
    // MissingTile over HadNoData. Road mode's own missing_blocks fold into
    // the SAME fill_miss_count_ above (fill_tile's doc comment) - a
    // MissingTile height result on top of a road-side miss double-counts by
    // one, which is fine: this counter is a witness ("did anything go
    // missing this fill"), not an exact per-cause tally.
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

ps::terrain::TerrainConfig make_terrain_config(double interest_radius_m, std::uint32_t fills_per_tick,
                                                 std::uint32_t prefetch_margin_tiles,
                                                 std::uint32_t prefetch_max_tiles) {
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
    cfg.prefetch_margin_tiles = prefetch_margin_tiles;
    cfg.prefetch_max_tiles = prefetch_max_tiles;
    return cfg;
}

} // namespace g2m::ps_bridge
