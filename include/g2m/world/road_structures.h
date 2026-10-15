// g2m/world/road_structures.h - structure meshes of one roads.geom tile (terrain migration plan phase 3a): bridge
// decks, tunnel floors and the tunnel roofs inferred where a tunnel passes under a higher road. Ported from racing_game
// core/src/road_structures.cpp (0c027dc: `deck()`, `road_deck_height`, `build_road_decks`; 945455a: the approach decks of
// grade-separated crossings, RoadSeparation) with det math; the output
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
    std::uint32_t start_ref = 0;         // the stretch of way_id the stations belong to (a way can hold several stretches)
    double start_station = 0, end_station = 0;
    std::vector<VerticalCubic> vertical; // the profile's vertical segments (a roof: one cubic over [start, end])
    double half_width = 0, crown = 0;    // metres, crossfall fraction (crown_per_mille / 1000)
    bool inferred = false;               // inferred: a roof span over a lower tunnel way, or a bridge approach deck
    bool tunnel_floor = false;           // the stretch is a tunnel (its floor)
    double min_easting = 0, max_easting = 0, min_northing = 0, max_northing = 0;
    LandClass land_class = LandClass::PavedRoad;
    mesh::TerrainChunkMesh mesh;
};

// Grade separation between ground roads, set by the caller (the game: world config "road_separation"). Two roads whose
// surfaces differ by more than clearance_m cannot share terrain: one of them stands on a structure there.
// build_road_decks continues a bridge onto its ground approach (an inferred deck) from the bridge end as far as the
// approach's footprint (half width + shoulder) still comes within margin_m of the footprint of a road more than
// clearance_m below it, at most approach_reach_m along the approach, so the upper road is carried by the structure and
// never by terrain carved for the road underneath (an OSM bridge way often ends over the lower road's verge). The
// consumer's terrain patch uses the same numbers. enabled = false adds no approach decks (the 0c027dc behaviour).
struct RoadSeparation {
    double clearance_m = 2.5;
    double margin_m = 3.0;
    double approach_reach_m = 60.0;
    bool enabled = true;
};

// Height of the deck's top surface at (station, lateral) - the profile at the station less the crown drop at the
// lateral offset; nullopt outside [start_station, end_station] (1e-6 tolerance) or where the profile has no value.
std::optional<double> road_deck_height(const RoadDeck& deck, double station, double lateral);

// Every deck of the tile, in a fixed order: first one per entry whose profile is a bridge or a tunnel (whole stretch,
// entry order), then the inferred roofs (lower tunnel way in graph order, upper entry in entry order, crossing in
// reference-segment order). A roof spans max(20, (w_lower / 2 + 4) / sin(angle)) metres either side of the crossing,
// over an upper road of rank >= 5 on a higher OSM layer, with the upper profile's own heights and grades at its ends;
// then (separation.enabled) the approach decks: per paved ground entry (entry order) whose stretch starts or ends on an
// end node of a bridge way, start end first, a deck from that end to 1 m past the last whole-metre station (within
// approach_reach_m) whose footprint overlaps a lower road, carrying the approach's own profile.
// Stretches shorter than 0.5 m or longer than 1000 m have no deck.
std::vector<std::shared_ptr<const RoadDeck>> build_road_decks(const RoadGeomTile& geometry, const RoadSeparation& separation = {});

// The StructureSet product of one level-2 tile: build_road_decks over `geometry` (key = geometry.source.key).
struct StructureSet {
    TileKey key; // level-2 tile
    std::vector<std::shared_ptr<const RoadDeck>> decks;
};
StructureSet build_structure_set(const RoadGeomTile& geometry, const RoadSeparation& separation = {});

} // namespace g2m::world
