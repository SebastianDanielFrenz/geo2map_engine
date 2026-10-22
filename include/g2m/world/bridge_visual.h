#pragma once
#include "g2m/layer/road_graph.h"
#include "g2m/mesh/terrain_chunk.h"
#include <string>
namespace g2m::world {
struct RoadDeck;
enum class BridgeForm {
  Generic,
  Beam,
  Truss,
  Arch,
  Suspension,
  CableStayed,
  Movable
};
struct BridgeFacts {
  BridgeForm form = BridgeForm::Generic;
  std::string source_id, wikidata_id, inventory_id, structure_tag;
  bool explicit_form = false;
};
struct BridgeVisual {
  mesh::TerrainChunkMesh mesh; // render only, same origin as the physical deck
  BridgeFacts facts;
  std::uint32_t rgba = 0x737b7e00u;
  bool unsupported = false;
};
BridgeFacts bridge_facts(const RoadGraphWay &);
// Generic parapets or an explicitly tagged structure family, never an inventory
// proximity guess. Procedural member dimensions are visual approximations, not
// surveyed engineering facts. No piers are invented in lower road/water space.
// The authoritative deck mesh is read, never modified or used as a new
// collider.
BridgeVisual build_bridge_visual(const RoadDeck &, const BridgeFacts &);
} // namespace g2m::world
