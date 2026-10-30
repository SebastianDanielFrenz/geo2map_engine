#include "g2m/ps_bridge/physics_terrain.h"

#include "ps/world/world.h"

#include <algorithm>
#include <cmath>
#include <string>

namespace g2m::ps_bridge {

namespace {

g2m::phys::LoaderConfig loader_for(const PhysicsTerrain::Config& c, bool require_roads, bool require_classes) {
    g2m::phys::LoaderConfig lc = c.loader;
    lc.require_roads = require_roads;
    lc.require_classes = require_classes;
    return lc;
}

constexpr std::size_t kTrackedIds = 8;

} // namespace

PhysicsTerrain::PhysicsTerrain(const Config& config, ps::SurfaceId surface)
    : grid_(config.frame),
      resident_(std::make_shared<g2m::phys::ResidentHeightSet>()),
      loader_(config.fetch, loader_for(config, /*require_roads=*/false, /*require_classes=*/false)),
      streamer_(grid_, *resident_, loader_, config.streamer),
      source_(std::make_shared<G2mTerrainSource>(grid_, resident_, surface)) {
    active_ids_.reserve(kTrackedIds);
}

PhysicsTerrain::PhysicsTerrain(const Config& config, RoadSurfaceConfig road)
    : grid_(config.frame),
      resident_(std::make_shared<g2m::phys::ResidentHeightSet>()),
      loader_(config.fetch, loader_for(config, /*require_roads=*/true, /*require_classes=*/false)),
      streamer_(grid_, *resident_, loader_, config.streamer),
      source_(std::make_shared<G2mTerrainSource>(grid_, resident_, std::move(road))) {
    active_ids_.reserve(kTrackedIds);
}

PhysicsTerrain::PhysicsTerrain(const Config& config, ClassSurfaceConfig classes)
    : grid_(config.frame),
      resident_(std::make_shared<g2m::phys::ResidentHeightSet>()),
      loader_(config.fetch, loader_for(config, /*require_roads=*/true, /*require_classes=*/true)),
      streamer_(grid_, *resident_, loader_, config.streamer),
      source_(std::make_shared<G2mTerrainSource>(grid_, resident_, std::move(classes))) {
    active_ids_.reserve(kTrackedIds);
}

Result<void> PhysicsTerrain::install(ps::World& world, double radius_m, std::uint32_t fills_per_tick,
                                     std::uint32_t prefetch_margin_tiles, std::uint32_t prefetch_max_tiles,
                                     std::uint32_t pool_factor) {
    if (!(radius_m > 0.0)) return make_error(ErrorCode::InvalidArgument, "PhysicsTerrain::install: radius_m must be > 0");
    if (fills_per_tick < 1) {
        return make_error(ErrorCode::InvalidArgument, "PhysicsTerrain::install: fills_per_tick must be >= 1");
    }
    if (pool_factor < 1) return make_error(ErrorCode::InvalidArgument, "PhysicsTerrain::install: pool_factor must be >= 1");
    // The loader gate must cover every footprint a background shape may read (the game's check, verbatim).
    if (prefetch_margin_tiles > static_cast<std::uint32_t>(g2m::phys::StreamerConfig{}.gate_margin_tiles) ||
        prefetch_max_tiles > 1024 || ((prefetch_margin_tiles == 0) != (prefetch_max_tiles == 0))) {
        return make_error(ErrorCode::InvalidArgument,
                          "Session: invalid terrain prefetch margin/cap (margin exceeds streamer gate)");
    }
    ps::terrain::TerrainConfig terrain_config =
        make_terrain_config(radius_m, fills_per_tick, prefetch_margin_tiles, prefetch_max_tiles);
    terrain_config.max_resident_tiles *= pool_factor;
    world.set_terrain_source(source_, terrain_config);
    installed_radius_m_ = radius_m;
    return {};
}

g2m::phys::GateStatus PhysicsTerrain::stream_update(std::span<const g2m::phys::InterestPoint> points) {
    return streamer_.update(points);
}

void PhysicsTerrain::apply_interest_points(ps::World& world, std::span<const g2m::phys::InterestPoint> points) {
    // TileManager keeps one interest point per id; the ids are the streamer's own.
    for (const g2m::phys::InterestPoint& ip : points) {
        world.set_terrain_interest_point(ip.id, ps::Vec3{ip.x, ip.y, 0.0}, ip.radius_m);
    }
    // A point that disappeared (a followed vehicle, a lost actor) releases its TileManager point.
    for (const std::uint32_t id : active_ids_) {
        const bool still_there = std::any_of(points.begin(), points.end(), [id](const auto& p) { return p.id == id; });
        if (!still_there) world.remove_terrain_interest_point(id);
    }
    active_ids_.clear();
    for (const g2m::phys::InterestPoint& ip : points) {
        if (std::find(active_ids_.begin(), active_ids_.end(), ip.id) == active_ids_.end()) active_ids_.push_back(ip.id);
    }
}

g2m::phys::GateStatus PhysicsTerrain::update(ps::World& world, std::span<const g2m::phys::InterestPoint> points) {
    const g2m::phys::GateStatus gs = stream_update(points);
    apply_interest_points(world, points);
    return gs;
}

int PhysicsTerrain::prime_ticks(double radius_m, std::uint32_t fills_per_tick) const {
    const int r_tm = static_cast<int>(std::ceil(radius_m / grid_.tile_size_m()));
    const std::size_t side = static_cast<std::size_t>(2 * r_tm + 1);
    const std::size_t square = side * side;
    return static_cast<int>((square + fills_per_tick - 1) / fills_per_tick) + 1;
}

Result<void> PhysicsTerrain::check_primed(const ps::World& world) const {
    const int r_tm = static_cast<int>(std::ceil(installed_radius_m_ / grid_.tile_size_m()));
    const std::size_t side = static_cast<std::size_t>(2 * r_tm + 1);
    const std::size_t square = side * side;
    const std::size_t resident = world.terrain_resident_tile_count();
    const std::size_t starved = world.terrain_starved_tile_count();
    const std::uint64_t misses = source_->fill_miss_count();
    if (resident < square || starved != 0 || misses != 0) {
        return make_error(ErrorCode::Malformed, "terrain priming failed (resident tiles " + std::to_string(resident) +
                                                    " of " + std::to_string(square) + ", starved " +
                                                    std::to_string(starved) + ", fill misses " + std::to_string(misses) + ")");
    }
    return {};
}

} // namespace g2m::ps_bridge
