#pragma once
// Street fidelity P2 slice 6: the profile completion tier of design 6.6, a port of racing_game's
// complete_road_profiles and its 20 m retry that runs after the strict pass of build_road_geom_tile and flags every
// profile it adds (ProfileProvenance). Terrain migration phase 4 (decision D47) adds the game's tunnel floors
// (TunnelFloor: DEM floor, or the portal chord above RoadCompletionOptions::tunnel_chord_min_length_m; never carved)
// and the game's accepted-bridge anchors (an accepted bridge stretch's end plane ahead of the walk, recomputed from
// the node so the result stays tile-invariant), plus a walk that tries every incident bridge edge.
//
// Tile invariance (decision D25). The result for one entry is a function of
//   - the entry's own strict outcome and the strict pass's anchor at its two end nodes (road_strict_anchor, D30: the
//     same reads the strict pass of this very entry makes, including P1's latent far-end read of a bridge span),
//   - the context graph within abutment_path_m (served 200 m, D48) of the entry's two end nodes and of the far end
//     nodes of the bridges incident to them (D47's accepted bridges), and
//   - the DEM within abutment_path_m + 3.5 m per axis of those nodes (walk + 3 m footprint + 0.5 m to the far edge of
//     a bilinear cell; at most 205 m with kRoadCompletionMaxAbutmentPathM) and along the entry's own way,
// and of nothing else: not of other entries, not of their strict or completed outcome, not of the tile. The reads fit
// what every tile holding the entry already has: every node the walk starts from is a node of a bridge way incident
// to a node of the tile's ways, and road_bridge_dependencies (second discovery stage, D35) adds the DEM and roads.graph
// tiles of a 205 m square around every node of those bridge ways (D48); a way incident to a node in that square lies
// in that node's roads.graph tile.
// Anchors are therefore NOT read from other entries' profiles (the game's anchor map, which would see a bridge only
// in a tile that holds it) but recomputed from the node itself by the same pure function (road_bridge_anchor).
#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/layer/road_geom_tile.h"
namespace g2m {
// The largest RoadCompletionOptions::abutment_path_m complete_road_profiles accepts (decisions D29, D48). The walk's
// support point lies at most abutment_path_m from its start node (per axis too); junction_plane reads a 3 m footprint
// around it and a bilinear sample reaches a cell whose square ends up to 0.5 m beyond the sample point per axis. Every
// start node is a node of a bridge way road_bridge_dependencies lists, and around each of those nodes it adds the DEM
// and roads.graph tiles of a kRoadBridgeWalkMarginMm (205 m) square: 201.5 + 3 + 0.5 = 205 m, so every node and DEM
// cell the walk reads is in an input every tile holding the entry loads. (D29's bound was 61.5 m against the 65 m
// margin around the entry's own way; slice 6 accepted 62 m.)
inline constexpr std::int64_t kRoadCompletionDemMarginMm = kRoadBridgeWalkMarginMm;
inline constexpr double kRoadCompletionMaxAbutmentPathM = 201.5;
static_assert(kRoadCompletionMaxAbutmentPathM + 3 + 0.5 <= static_cast<double>(kRoadCompletionDemMarginMm) / 1000.0);
// What a bridge abutment plane at `node` is, as the completion tier computes it.
struct RoadBridgeAnchor {
    bool bridging = false;                   // an incident bridge way (not a tunnel) ends or passes at the node
    std::optional<RoadJunctionPlane> plane;  // nullopt when bridging but no ground support within the walk budget
};
// The anchor of a node, a pure function of the node: the walk of racing_game's `abutment` (the most aligned ground
// edge, dot > 0.25, on through bridge edges, support 32 m of ground away, bounded by options.abutment_path_m of
// total path) starting AWAY from each incident bridge edge in (way id, segment index) order until one finds support
// (D47; with options.bridge_anchor_reuse = false only the first, D25), so that two bridge stretches meeting at the
// node read the same plane whichever asks. The plane is the DEM plane at the support point
// carried back to the node. `graph` must hold every way incident to every node within the walk budget of `node`.
RoadBridgeAnchor road_bridge_anchor(const RoadGraph& graph, const RoadDemSampler& dem, geo::UtmZone zone,
    std::int64_t node, const RoadCompletionOptions& options = {});
// Steps 1-3 of design 6.6 (step 2a: tunnel floors, D47) over entries that carry only their strict outcome (profile or decline, no cross-section
// yet). Fills `origins` (one per entry). `builder` is built over the context graph. Declined entries only are
// changed; accepted strict profiles are never touched (decision D26).
Result<void> complete_road_profiles(std::vector<RoadGeomEntry>& entries, std::vector<RoadProfileOrigin>& origins,
    const RoadReferenceBuilder& builder, const RoadDemSampler& dem, geo::UtmZone zone,
    const RoadProfileLimits& limits, const RoadCompletionOptions& options, RoadCompletionStats* stats = nullptr);
}
