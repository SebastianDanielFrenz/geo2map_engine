// g2m/world/products.h - the payload types TerrainService tickets deliver
// (terrain migration plan phase 2, vault G2M-020), and Outcome<T>, what a
// product producer returns. Engine-neutral: no physics_sim, Jolt or Godot type.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/tile_container.h"
#include "g2m/world/buildings.h"
#include "g2m/world/road_ribbons.h"
#include "g2m/world/road_structures.h"
#include "g2m/world/ticket.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace g2m::world {

// What a producer (and the service's own fetchers) returns: Ready with a value,
// Absent (final: the release has no such tile), or Failed (transient or a bug;
// not cached). `bytes` is the value's weight in the service's memory cache.
template <class T>
struct Outcome {
    TileState state = TileState::Failed;
    std::shared_ptr<const T> value;
    std::string message;
    std::size_t bytes = 0;

    static Outcome ready(std::shared_ptr<const T> v, std::size_t weight) {
        Outcome o;
        o.state = TileState::Ready;
        o.value = std::move(v);
        o.bytes = weight;
        return o;
    }
    static Outcome absent(std::string why) {
        Outcome o;
        o.state = TileState::Absent;
        o.message = std::move(why);
        return o;
    }
    static Outcome failed(std::string why) {
        Outcome o;
        o.state = TileState::Failed;
        o.message = std::move(why);
        return o;
    }
    bool ready() const noexcept { return state == TileState::Ready; }
};

// g2m.terrain.height (or g2m.terrain.carved when the service opted into the carved
// layer) decoded, header height_offset applied (tile_fetch.h). A HeightTile is 256 KiB.
using HeightTilePtr = std::shared_ptr<const HeightTile>;

// g2m.roads.geom, decoded (g2m::RoadGeomTile; `source.key` is the level-2 tile). The
// served profiles (completion included) are the release's truth: a consumer does
// not re-fit them.
using RoadGeomTile = g2m::RoadGeomTile;

// The drivable road segments of one g2m.src.osm tile (extract_road_segments with
// the default RoadStyle), the list grip, speed limit and road-ahead read.
struct RoadSegmentsTile {
    TileKey key; // the level-2 tile
    std::vector<RoadSegment> segments;
};

// g2m.src.osm of one level-2 tile is delivered as a g2m::OsmTile.

// Any layer's tile container as served (header + body bytes), for layers without a
// typed ticket and for tools.
struct RawTile {
    TileKey key;
    std::string layer;
    TileHeader header;
    std::vector<std::uint8_t> container; // HEADER || BODY, exactly as a TileResponse carries it
};

// ---- Products of phase 3 ----------------------------------------------------------
//
// StructureSet (road_structures.h) and RoadRibbonTile (road_ribbons.h) are filled in by
// phase 3a: build_structure_set / build_road_ribbons over a RoadGeomTile the caller
// hands in. The service delivers them through the producer the caller installs
// (TerrainService::set_*_producer); until then the matching request is Failed.
// BuildingSet (buildings.h) is filled in by phase 3b: build_building_set over the OsmTile of a level-2 tile and a
// ground sampler, delivered the same way (TerrainService::set_buildings_producer).
// Hash of the producer's option struct (part of the job key and the cache key), 0 when it has none.
struct ProductOptions {
    std::uint64_t hash = 0;
};
using RibbonOptions = ProductOptions;
using BuildingOptions = ProductOptions;

} // namespace g2m::world
