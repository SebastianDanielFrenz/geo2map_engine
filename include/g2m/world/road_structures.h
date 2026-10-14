// g2m/world/road_structures.h - structure meshes of one roads.geom tile (terrain migration plan phase 3a): bridge
// decks, tunnel floors and the tunnel roofs inferred where a tunnel passes under a higher road. Ported from racing_game
// core/src/road_structures.cpp (0c027dc: `deck()`, `road_deck_height`, `build_road_decks`) with det math; the output
// feeds physics (the game installs every deck mesh as a static collider), so it is a pure, deterministic function of
// the RoadGeomTile it is handed. Engine-neutral: no physics_sim, Jolt or Godot type.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/mesh/terrain_chunk.h"

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

namespace g2m::world {

// One structure: the road surface of a profiled bridge or tunnel stretch (or an inferred roof span of an upper road)
// between start_station and end_station of its profile, as a mesh. Mesh layout: 5 vertices across per 1 m station
// (rounded crown), then the same vertices 0.5 m lower (the slab's underside, normals down), then the two fascia sides.
// Positions are float metres relative to mesh.origin (absolute UTM of the first station's reference point), indices
// counter-clockwise from above on the top. bounds are absolute UTM metres over the mesh's vertices.
struct RoadDeck {
    std::int64_t way_id = 0;
    double start_station = 0, end_station = 0;
    std::vector<VerticalCubic> vertical; // the profile's vertical segments (a roof: one cubic over [start, end])
    double half_width = 0, crown = 0;    // metres, crossfall fraction (crown_per_mille / 1000)
    bool inferred = false;               // a roof span inferred over a lower tunnel way
    bool tunnel_floor = false;           // the stretch is a tunnel (its floor)
    double min_easting = 0, max_easting = 0, min_northing = 0, max_northing = 0;
    LandClass land_class = LandClass::PavedRoad;
    mesh::TerrainChunkMesh mesh;
};

// Height of the deck's top surface at (station, lateral) - the profile at the station less the crown drop at the
// lateral offset; nullopt outside [start_station, end_station] (1e-6 tolerance) or where the profile has no value.
std::optional<double> road_deck_height(const RoadDeck& deck, double station, double lateral);

// Every deck of the tile, in a fixed order: first one per entry whose profile is a bridge or a tunnel (whole stretch,
// entry order), then the inferred roofs (lower tunnel way in graph order, upper entry in entry order, crossing in
// reference-segment order). A roof spans max(20, (w_lower / 2 + 4) / sin(angle)) metres either side of the crossing,
// over an upper road of rank >= 5 on a higher OSM layer, with the upper profile's own heights and grades at its ends.
// Stretches shorter than 0.5 m or longer than 1000 m have no deck.
std::vector<std::shared_ptr<const RoadDeck>> build_road_decks(const RoadGeomTile& geometry);

// The StructureSet product of one level-2 tile: build_road_decks over `geometry` (key = geometry.source.key).
struct StructureSet {
    TileKey key; // level-2 tile
    std::vector<std::shared_ptr<const RoadDeck>> decks;
};
StructureSet build_structure_set(const RoadGeomTile& geometry);

} // namespace g2m::world
