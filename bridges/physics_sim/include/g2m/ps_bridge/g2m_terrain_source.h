// g2m/ps_bridge/g2m_terrain_source.h - G2mTerrainSource (PLAN.md R2.2
// section 1b, G6): the ps::terrain::ITerrainSource implementation that
// reads geo2map_engine's own DGM1-derived physics-tile data
// (g2m::phys::PhysicsTileGrid + g2m::phys::ResidentHeightSet) instead of
// physics_sim's own procedural terrain.json spec.
//
// Every method below is a thin, allocation-free translation between the two
// projects' tile-key spaces:
//   - g2m::phys::PhysicsTileGrid::pack/unpack(PhysTileIndex) is the physics
//     tile's OWN opaque numbering (a *different* grid from g2m::TileKey's
//     256 m DGM1 data-tile keys - see physics_grid.h's header comment);
//     ps::terrain::TileKey::packed is exactly this value, opaque to
//     physics_sim itself (PLAN.md section 6's "opaque keys" rule - nothing
//     outside this file ever decodes a ps::terrain::TileKey).
//   - fill_tile defers all the actual sample-copying work to
//     g2m::phys::fill_physics_heights (header-only, allocation-free for any
//     sample count, no row flip) against the ResidentHeightSet this source
//     was constructed with.
//
// No physics_sim type appears in g2m_phys's own public headers
// (include/g2m/phys) - this header is the only place the two projects'
// terrain representations meet.
#pragma once

#include "g2m/layer/osm_roads.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"

#include "ps/terrain/terrain_config.h"
#include "ps/terrain/terrain_source.h"
#include "ps/types.h"

#include <atomic>
#include <cstdint>
#include <memory>
#include <optional>

namespace g2m::ps_bridge {

// Road mode's own config (PLAN.md G2.5a-grip section 5, commit G-c): the
// road-value LUT fill_tile resolves every painted cell through
// (g2m::phys::fill_physics_surfaces, itself g2m::phys::detail::surface_lookup
// over the ResidentHeightSet's road lists) plus the id an off-road cell
// gets. No RasterParams field - fill_physics_surfaces hardcodes
// RasterParams{} (its own paint_road_blocks call), so there is nothing here
// for a caller to override yet.
struct RoadSurfaceConfig {
    g2m::RoadValueLut lut;
    ps::SurfaceId off_road = ps::kInvalidSurfaceId;
};

class G2mTerrainSource final : public ps::terrain::ITerrainSource {
public:
    // `grid` anchors the 255 m physics-tile lattice on a session; `resident`
    // is the shared, owner-thread-mutated cache fill_tile reads from (kept
    // alive for as long as this source is - the streamer/World keeps
    // installing/evicting keys in it between steps, PLAN.md R2.2 section
    // 1a/2). `surface` is the single ps::SurfaceId every cell of every tile
    // this source produces gets (PLAN.md R2.2 1b: "a single SurfaceId,
    // resolved by the caller through SurfaceTable" - land-class surfaces
    // are R2.3).
    G2mTerrainSource(g2m::phys::PhysicsTileGrid grid, std::shared_ptr<const g2m::phys::ResidentHeightSet> resident,
                      ps::SurfaceId surface);

    // Road mode (PLAN.md G2.5a-grip section 1/5, commit G-c): fill_tile
    // resolves cell_surface_ids from `resident`'s own road lists via
    // g2m::phys::fill_physics_surfaces instead of one flat SurfaceId - per-
    // cell grip from OSM roads. Heights are unaffected (still
    // g2m::phys::fill_physics_heights, same as the constructor above).
    // `road.off_road` is what every non-road cell gets, whether or not the
    // height side of the same fill_tile call also missed.
    G2mTerrainSource(g2m::phys::PhysicsTileGrid grid, std::shared_ptr<const g2m::phys::ResidentHeightSet> resident,
                      RoadSurfaceConfig road);

    [[nodiscard]] ps::real tile_size_m() const override;
    [[nodiscard]] std::uint32_t samples_per_tile() const override;
    [[nodiscard]] ps::terrain::TileKey key_for_world_xy(ps::real x, ps::real y) const override;
    [[nodiscard]] ps::terrain::TileKey key_offset(ps::terrain::TileKey key, std::int32_t dx,
                                                   std::int32_t dy) const override;
    [[nodiscard]] ps::Vec3 tile_origin_xy(ps::terrain::TileKey key) const override;

    // Allocation-free and lock-free (PLAN.md R2.2 1b): writes only into
    // `out`'s already-sized heights/cell_surface_ids vectors (never
    // resizes, per ITerrainSource::fill_tile's own contract), reading the
    // ResidentHeightSet this source holds a shared_ptr to. Heights are
    // always ABSOLUTE (ITerrainSource's own contract - TileManager applies
    // the per-tile datum, not this source). A NoData sample, or one from a
    // confirmed-absent (404) L0 tile, becomes ps::kHeightfieldNoCollision
    // (a hole) - see g2m::phys::fill_physics_heights's own FillResult
    // documentation for exactly which case is which. cell_surface_ids comes
    // from g2m::phys::fill_physics_surfaces in road mode (constructed with a
    // RoadSurfaceConfig), or is filled uniformly with `surface_` otherwise -
    // both branches are the same allocation-free/lock-free contract.
    void fill_tile(ps::terrain::TileKey key, ps::terrain::TileSample& out) const override;

    [[nodiscard]] ps::terrain::PrefetchSupportKind prefetch_support() const override {
        return ps::terrain::PrefetchSupportKind::Snapshot;
    }
    bool acquire_prefetch_inputs(ps::terrain::TileKey key, std::shared_ptr<const void>& out) const override;
    void fill_tile_from(ps::terrain::TileKey key, const void* inputs, ps::terrain::TileSample& out) const override;
    [[nodiscard]] bool prefetch_inputs_current(ps::terrain::TileKey key, const void* inputs) const override;

    // Diagnostic counters (PLAN.md R2.2 1b), relaxed atomics incremented
    // from inside the const fill_tile above: fill_miss_count() counts a
    // fill that touched at least one L0 key never installed in the
    // ResidentHeightSet (g2m::phys::FillResult::MissingTile - the runtime
    // witness that streaming/gating failed to keep up, see the plan's
    // "Determinism argument"). In road mode it ALSO counts a fill whose
    // surface side touched a Missing L0 key (a non-resident key, or a
    // resident one with no roads list at all - g2m::phys::RoadBlockStats::
    // missing_blocks > 0, fill_physics_surfaces.h's own Outcomes table) -
    // one shared counter, never silently grass either way.
    // nodata_fill_count() counts a fill that hit a
    // genuine in-coverage NoData sample (FillResult::HadNoData) -
    // informational only, never a correctness signal on its own (a
    // confirmed-absent/404 L0 tile does NOT count here, see
    // fill_physics_heights.h's own "Design note on Absent vs. HadNoData").
    [[nodiscard]] std::uint64_t fill_miss_count() const noexcept;
    [[nodiscard]] std::uint64_t nodata_fill_count() const noexcept;

private:
    void fill_from(ps::terrain::TileKey key, const g2m::phys::ResidentHeightSet& resident,
                   ps::terrain::TileSample& out) const;
    bool footprint_ready(ps::terrain::TileKey key, const g2m::phys::ResidentHeightSet& snapshot,
                         bool compare_live) const;
    g2m::phys::PhysicsTileGrid grid_;
    std::shared_ptr<const g2m::phys::ResidentHeightSet> resident_;
    ps::SurfaceId surface_;
    std::optional<RoadSurfaceConfig> road_; // road mode iff set (PLAN.md G2.5a-grip section 5)
    mutable std::atomic<std::uint64_t> fill_miss_count_{0};
    mutable std::atomic<std::uint64_t> nodata_fill_count_{0};
};

// A ps::terrain::TerrainConfig sized for one interest point of radius
// `interest_radius_m` on a G2mTerrainSource's 255 m physics-tile grid,
// filling at most `fills_per_tick` tiles per World::step() (PLAN.md R2.2
// 1b):
//   - suppress_terrain_under_static_geometry / precompute_suppression_at_load
//     are both OFF - R2.2 has no static geometry in terrain mode, so
//     there is nothing for the suppression pass to probe against.
//   - max_resident_tiles = (2*ceil(interest_radius_m/255)+3)^2: a
//     (2r+1)x(2r+1) square of physics tiles covering the interest radius,
//     plus one tile of margin on every side (the "+3" over "+1") so a tile
//     just inside the gate at the start of a tick is never evicted before
//     TileManager can re-derive that it is still needed (r=400 -> 49,
//     matching the plan's own worked example).
//   - fills_per_tick: use 1. PLAN.md R2.2 1b's rule allows 2 only if one
//     SetHeights(256^2) plus the weld stays under 1.0 ms; bench_bridge_fill
//     (G8, bridge-release, dev box) measured update_heightfield_body(256^2)
//     at ~2.26 ms mean - over budget on its own - against ~0.03 ms per
//     fill_tile and ~0.006 ms/tick of weld overhead at pool 49.
[[nodiscard]] ps::terrain::TerrainConfig make_terrain_config(double interest_radius_m, std::uint32_t fills_per_tick,
                    std::uint32_t prefetch_margin_tiles = 0, std::uint32_t prefetch_max_tiles = 0);

} // namespace g2m::ps_bridge
