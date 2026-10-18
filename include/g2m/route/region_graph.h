// g2m/route/region_graph.h - the routing graph of a whole region, assembled
// from g2m.roads.graph tiles (G4 slice 2, docs/routing_design.md).
//
// A roads.graph tile holds every complete way touching it (plus its halo),
// so neighbouring tiles repeat ways and nodes. merge_region_graphs keeps one
// copy of each (by OSM id), rejects conflicting copies, and sorts the result,
// so the output does not depend on the tile order. Unlike
// merge_road_graphs it applies no per-tile size limits.
//
// Intended use (racing_game R4): at world load, on a worker thread, call
// assemble_region_graph over the loaded store's level-2 keys, then
// build_region_routing (graph, then ALT landmarks). Both are pure functions
// of their inputs.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/route/route_search.h"

#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <vector>

namespace g2m::route {

struct RegionGraphStats {
    std::size_t tiles_requested = 0, tiles_present = 0, tiles_absent = 0;
    std::size_t node_copies = 0, way_copies = 0; // before dedup
    std::size_t nodes = 0, ways = 0;             // after
    std::size_t restriction_copies = 0, restrictions = 0; // encoding-6 tiles only; before and after dedup
};

// Unions complete graphs. Malformed when two copies of one node or way
// differ; LimitExceeded past max_ways ways or max_nodes nodes.
Result<RoadGraph> merge_region_graphs(std::span<const RoadGraph* const> graphs, RegionGraphStats* stats = nullptr,
                                      std::size_t max_nodes = 32u << 20, std::size_t max_ways = 8u << 20);

// Returns the decoded tile, std::nullopt for an absent tile (outside the
// data), or an error (which aborts the assembly).
using RoadGraphTileFetch = std::function<Result<std::optional<RoadGraphTile>>(const TileKey&)>;

// Fetches every key (sorted, duplicates dropped; the fetch order is the
// sorted order) and merges the graphs of the tiles (restrictions are ignored:
// see assemble_region). Keys may be any level-2 keys.
Result<RoadGraph> assemble_region_graph(std::span<const TileKey> keys, const RoadGraphTileFetch& fetch,
                                        RegionGraphStats* stats = nullptr);

// A region: the merged graph and the merged turn restrictions of the tiles
// (encoding 6; tiles of encoding 4 or 5 contribute none).
struct RegionData {
    RoadGraph graph;
    std::vector<TurnRestriction> restrictions; // sorted by (relation, from, to), one copy each
};

// Unions the restriction lists of tiles: one copy per (relation, from, to),
// sorted; Malformed when two copies of one rule differ.
Result<std::vector<TurnRestriction>> merge_tile_restrictions(std::span<const RoadGraphTile* const> tiles,
                                                             RegionGraphStats* stats = nullptr);

// As assemble_region_graph, plus the restrictions the tiles carry. Feed
// `restrictions` to build_region_routing: the result equals a routing graph
// built from the whole OSM extract of the region.
Result<RegionData> assemble_region(std::span<const TileKey> keys, const RoadGraphTileFetch& fetch,
                                   RegionGraphStats* stats = nullptr);

// Union of per-tile restriction sets: sorted by (relation, from, to), one copy each.
std::vector<TurnRestriction> merge_turn_restrictions(std::span<const RestrictionSet> sets);

// The routing graph and its landmarks; heap-held so the landmarks' graph
// pointer stays valid when the struct moves.
struct RegionRouting {
    std::unique_ptr<RoutingGraph> graph;
    std::unique_ptr<RouteLandmarks> landmarks; // null when landmark_count == 0
};
Result<RegionRouting> build_region_routing(const RoadGraph& graph, std::span<const TurnRestriction> restrictions,
                                           const RouteConfig& config = default_route_config(),
                                           std::size_t landmark_count = 16);

} // namespace g2m::route
