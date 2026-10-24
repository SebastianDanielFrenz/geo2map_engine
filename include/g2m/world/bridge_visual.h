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
enum class BridgeParapetStyle {
  RoadGuardrail,
  ConcreteParapet,
  PedestrianRailing,
  RailwayParapet
};
struct BridgeFacts {
  BridgeForm form = BridgeForm::Generic;
  BridgeParapetStyle parapet_style = BridgeParapetStyle::RoadGuardrail;
  std::string source_id, wikidata_id, inventory_id, structure_tag;
  std::string highway_tag, railway_tag, bridge_tag, bridge_material_tag,
      barrier_tag;
  bool explicit_form = false;
  bool parapet_style_inferred = true;
};
BridgeFacts bridge_facts(const RoadGraphWay &);
struct BridgeVisual {
  mesh::TerrainChunkMesh mesh; // render only, same origin as the physical deck
  BridgeFacts facts;
  std::uint32_t rgba = 0x737b7e00u;
  bool unsupported = false;
};
// Source-tag-selected parapets or an explicitly tagged structure family,
// never an inventory proximity guess. Procedural dimensions are visual
// approximations, not surveyed facts. An unqualified arch retains source facts
// but produces parapets only because its topology is unknown. No piers are
// invented in lower road or water space. The authoritative deck mesh is read,
// never modified or used as a new collider.
BridgeVisual build_bridge_visual(const RoadDeck &, const BridgeFacts &);
} // namespace g2m::world
