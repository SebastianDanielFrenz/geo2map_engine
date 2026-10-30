// g2m/ps_bridge/physics_terrain.h - PhysicsTerrain: geo2map's physics terrain stack, composed for a physics_sim World
// (terrain migration phase 9, moved from racing_game's Session::Terrain, Session::setup_terrain and
// Session::gate_check; vault G2M-027).
//
// It owns, in the destruction order the game relied on, the pieces that stream DGM1 height (and road / class) tiles
// into a ps::World's TileManager:
//
//   grid      PhysicsTileGrid         the 255 m physics-tile lattice on the session frame
//   resident  ResidentHeightSet       shared with the source (the World keeps its own shared_ptr to the source)
//   loader    HeightTileLoader        worker threads; the destructor joins them (waiting out a fetch in flight)
//   streamer  PhysicsTerrainStreamer  holds references to resident and loader, so it is destroyed first
//   source    G2mTerrainSource        the ps::terrain::ITerrainSource installed into the World
//
// The World keeps a shared_ptr to `source` (and through it to `resident`), so both outlive this object when it is
// destroyed before the World. The loader's workers start in the constructor, before install().
//
// What stays with the caller (the game): the blocking start-up loop (sleeping, timeouts, cancel, progress), env
// overrides, tracing, the interest-point list itself and the stepping of the priming ticks. The calls below are
// non-blocking and run on the sim thread between World steps. Typical use:
//
//   PhysicsTerrain t(config, surface);          // or a RoadSurfaceConfig / ClassSurfaceConfig (surface_binding.h)
//   t.install(world, radius, fills, margin, cap);           // Result<void>
//   while (!t.update(world, points).ready) { if (failed) t.retry_failed(); sleep }
//   for (int k = 0; k < t.prime_ticks(radius, fills); ++k) world.step();
//   if (auto r = t.check_primed(world); !r) fail(r.error().message);
#pragma once

#include "g2m/core/result.h"
#include "g2m/core/geo/session_frame.h"
#include "g2m/phys/height_tile_loader.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/physics_streamer.h"
#include "g2m/phys/resident_heights.h"
#include "g2m/ps_bridge/g2m_terrain_source.h"

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace ps {
class World;
}

namespace g2m::ps_bridge {

class PhysicsTerrain {
public:
    struct Config {
        g2m::geo::SessionFrame frame;
        std::shared_ptr<g2m::phys::IHeightTileFetch> fetch;
        // loader.require_roads / require_classes are set by the constructor from the surface mode (uniform: neither,
        // road mode: roads, class mode: roads and classes), whatever they hold here.
        g2m::phys::LoaderConfig loader;
        g2m::phys::StreamerConfig streamer;
    };

    // Uniform mode: one SurfaceId for every cell (the game's physics.terrain_surface).
    PhysicsTerrain(const Config& config, ps::SurfaceId surface);
    // Road mode: grip from per-cell OSM road classes; roads are required alongside heights (bind_road_surfaces).
    PhysicsTerrain(const Config& config, RoadSurfaceConfig road);
    // Served class mode: grip from the g2m.terrain.class tile; roads and classes are required (bind_class_surfaces).
    PhysicsTerrain(const Config& config, ClassSurfaceConfig classes);
    PhysicsTerrain(const PhysicsTerrain&) = delete;
    PhysicsTerrain& operator=(const PhysicsTerrain&) = delete;

    // World::set_terrain_source(source, make_terrain_config(radius_m, fills_per_tick, prefetch_margin_tiles,
    // prefetch_max_tiles) with max_resident_tiles multiplied by pool_factor (the game's pool holds two interest
    // points: the car and a followed vehicle). Rejects, before touching the World, a prefetch margin above the
    // streamer's gate margin (the loader gate must cover every footprint a background shape may read), a cap above
    // 1024, and exactly one of margin / cap being zero; and radius_m <= 0, fills_per_tick < 1, pool_factor < 1.
    // Remembers radius_m / fills_per_tick for check_primed.
    [[nodiscard]] Result<void> install(ps::World& world, double radius_m, std::uint32_t fills_per_tick,
                                       std::uint32_t prefetch_margin_tiles, std::uint32_t prefetch_max_tiles,
                                       std::uint32_t pool_factor = 2);

    // The game's gate_check without its timing and tracing: stream_update() then apply_interest_points(). They are
    // public separately so a caller can time or trace between the two.
    //   stream_update: PhysicsTerrainStreamer::update(points).
    //   apply_interest_points: World::set_terrain_interest_point(id, {x, y, 0}, radius) for every point, then
    //   remove_terrain_interest_point for every id set by an earlier call that is no longer in `points` (a followed
    //   vehicle that went away). Allocation-free once warm for up to 8 distinct live ids.
    g2m::phys::GateStatus update(ps::World& world, std::span<const g2m::phys::InterestPoint> points);
    g2m::phys::GateStatus stream_update(std::span<const g2m::phys::InterestPoint> points);
    void apply_interest_points(ps::World& world, std::span<const g2m::phys::InterestPoint> points);

    // Forgets every remembered Failed key; the next update requests them again.
    void retry_failed() { streamer_.retry_failed(); }

    // Ticks to step with no actor so TileManager fills its whole square (side (2*ceil(radius/tile)+1)^2 tiles, at
    // most fills_per_tick per tick) before anything touches the ground: ceil(square / fills) + 1.
    [[nodiscard]] int prime_ticks(double radius_m, std::uint32_t fills_per_tick) const;

    // After priming: every tile of the install() radius's square is resident, none starved, no fill ever missed an
    // L0 tile. The error text names the three counts.
    [[nodiscard]] Result<void> check_primed(const ps::World& world) const;

    [[nodiscard]] const g2m::phys::PhysicsTileGrid& grid() const { return grid_; }
    [[nodiscard]] g2m::phys::ResidentHeightSet& resident() { return *resident_; }
    [[nodiscard]] const g2m::phys::ResidentHeightSet& resident() const { return *resident_; }
    [[nodiscard]] g2m::phys::HeightTileLoader& loader() { return loader_; }
    [[nodiscard]] g2m::phys::PhysicsTerrainStreamer& streamer() { return streamer_; }
    [[nodiscard]] const std::shared_ptr<G2mTerrainSource>& source() const { return source_; }

private:
    // Declaration order is the destruction contract (see the file comment).
    g2m::phys::PhysicsTileGrid grid_;
    std::shared_ptr<g2m::phys::ResidentHeightSet> resident_;
    g2m::phys::HeightTileLoader loader_;
    g2m::phys::PhysicsTerrainStreamer streamer_;
    std::shared_ptr<G2mTerrainSource> source_;

    double installed_radius_m_ = 0.0;
    std::vector<std::uint32_t> active_ids_; // interest-point ids currently set in the World
};

} // namespace g2m::ps_bridge
