#pragma once
// Street fidelity P2 slice 6: the profile completion tier of design 6.6, a port of racing_game's
// complete_road_profiles and its 20 m retry that runs after the strict pass of build_road_geom_tile and flags every
// profile it adds (ProfileProvenance). No tunnel completion (not carved, not drivable).
//
// Tile invariance (decision D25). The result for one entry is a function of
//   - the entry's own strict outcome (the strict pass, as before),
//   - the context graph within 60 m of the entry's two end nodes, and
//   - the DEM within 63.5 m per axis of those end nodes (60 m walk + 3 m footprint + 0.5 m to the far edge of a
//     bilinear cell; at most 65 m with kRoadCompletionMaxAbutmentPathM) and along the entry's own way,
// and of nothing else: not of other entries, not of their strict or completed outcome, not of the tile. The reads fit
// what every tile holding the entry already has: a way incident to a node within 60 m of an end node n touches the
// level-2 tile of n widened by the 128 m halo, so it is in the roads.graph tile of n, which road_context_dependencies
// lists (n is a node of the entry's way); the DEM margin of road_dem_dependencies is 65 m around the way's segments.
// Anchors are therefore NOT read from other entries' profiles (the game's anchor map, which would see a bridge only
// in a tile that holds it) but recomputed from the node itself by the same pure function (road_bridge_anchor).
#include "g2m/layer/road_geom_tile.h"
namespace g2m {
// The largest RoadCompletionOptions::abutment_path_m complete_road_profiles accepts (decision D29). The walk's support
// point lies at most abutment_path_m from the end node (per axis too); junction_plane reads a 3 m footprint around it
// and a bilinear sample reaches a cell whose square ends up to 0.5 m beyond the sample point per axis. 61.5 + 3 + 0.5
// = 65 m, the road_dem_dependencies margin around the entry's own way, so every DEM cell the walk reads lies in a DEM
// tile that every tile holding the entry loads. (Slice 6 accepted 62 m: such a walk reads up to 65.5 m away.)
inline constexpr std::int64_t kRoadCompletionDemMarginMm = 65000;   // == road_dem_dependencies' default margin_mm
inline constexpr double kRoadCompletionMaxAbutmentPathM = 61.5;
static_assert(kRoadCompletionMaxAbutmentPathM + 3 + 0.5 <= static_cast<double>(kRoadCompletionDemMarginMm) / 1000.0);
// What a bridge abutment plane at `node` is, as the completion tier computes it.
struct RoadBridgeAnchor {
    bool bridging = false;                   // an incident bridge way (not a tunnel) ends or passes at the node
    std::optional<RoadJunctionPlane> plane;  // nullopt when bridging but no ground support within the walk budget
};
// The anchor of a node, a pure function of the node: the walk of racing_game's `abutment` (the most aligned ground
// edge, dot > 0.25, on through bridge edges, support 32 m of ground away, bounded by options.abutment_path_m of
// total path) starting AWAY from the first incident bridge edge in (way id, segment index) order, so that two bridge
// stretches meeting at the node read the same plane whichever asks. The plane is the DEM plane at the support point
// carried back to the node. `graph` must hold every way incident to every node within the walk budget of `node`.
RoadBridgeAnchor road_bridge_anchor(const RoadGraph& graph, const RoadDemSampler& dem, geo::UtmZone zone,
    std::int64_t node, const RoadCompletionOptions& options = {});
// Steps 1-3 of design 6.6 over entries that carry only their strict outcome (profile or decline, no cross-section
// yet). Fills `origins` (one per entry). `builder` is built over the context graph. Declined entries only are
// changed; accepted strict profiles are never touched (decision D26).
Result<void> complete_road_profiles(std::vector<RoadGeomEntry>& entries, std::vector<RoadProfileOrigin>& origins,
    const RoadReferenceBuilder& builder, const RoadDemSampler& dem, geo::UtmZone zone,
    const RoadProfileLimits& limits, const RoadCompletionOptions& options, RoadCompletionStats* stats = nullptr);
}
