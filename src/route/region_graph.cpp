// src/route/region_graph.cpp - see g2m/route/region_graph.h.
#include "g2m/route/region_graph.h"

#include <algorithm>
#include <utility>

namespace g2m::route {

Result<RoadGraph> merge_region_graphs(std::span<const RoadGraph* const> graphs, RegionGraphStats* stats,
                                      std::size_t max_nodes, std::size_t max_ways) {
    std::vector<const RoadGraphNode*> nodes;
    std::vector<const RoadGraphWay*> ways;
    for (const RoadGraph* g : graphs) {
        for (const auto& n : g->nodes) nodes.push_back(&n);
        for (const auto& w : g->ways) ways.push_back(&w);
    }
    if (stats) { stats->node_copies = nodes.size(); stats->way_copies = ways.size(); }
    std::sort(nodes.begin(), nodes.end(), [](const auto* a, const auto* b) { return a->osm_id < b->osm_id; });
    std::sort(ways.begin(), ways.end(), [](const auto* a, const auto* b) { return a->osm_id < b->osm_id; });
    RoadGraph out;
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        if (i && nodes[i]->osm_id == nodes[i - 1]->osm_id) {
            if (!(*nodes[i] == *nodes[i - 1])) return make_error(ErrorCode::Malformed, "region graph: conflicting node copies");
            continue;
        }
        if (out.nodes.size() >= max_nodes) return make_error(ErrorCode::LimitExceeded, "region graph: max_nodes");
        out.nodes.push_back(*nodes[i]);
    }
    for (std::size_t i = 0; i < ways.size(); ++i) {
        if (i && ways[i]->osm_id == ways[i - 1]->osm_id) {
            if (!(*ways[i] == *ways[i - 1])) return make_error(ErrorCode::Malformed, "region graph: conflicting way copies");
            continue;
        }
        if (out.ways.size() >= max_ways) return make_error(ErrorCode::LimitExceeded, "region graph: max_ways");
        const RoadGraphWay& way = *ways[i];
        out.ways.push_back(way);
        for (std::size_t k = 1; k < way.node_ids.size(); ++k) {
            if (way.node_ids[k - 1] == way.node_ids[k]) { ++out.repeated_ref_pairs; continue; }
            out.edges.push_back({way.osm_id, static_cast<std::uint32_t>(k - 1), way.node_ids[k - 1], way.node_ids[k]});
        }
    }
    // Every ref must resolve: a tile holds the nodes of its complete ways.
    for (const auto& way : out.ways)
        for (const std::int64_t id : way.node_ids) {
            const auto it = std::lower_bound(out.nodes.begin(), out.nodes.end(), id,
                                             [](const RoadGraphNode& n, std::int64_t x) { return n.osm_id < x; });
            if (it == out.nodes.end() || it->osm_id != id) return make_error(ErrorCode::Malformed, "region graph: way references a missing node");
        }
    if (stats) { stats->nodes = out.nodes.size(); stats->ways = out.ways.size(); }
    return out;
}

Result<RoadGraph> assemble_region_graph(std::span<const TileKey> keys, const RoadGraphTileFetch& fetch,
                                        RegionGraphStats* stats) {
    std::vector<TileKey> sorted(keys.begin(), keys.end());
    std::sort(sorted.begin(), sorted.end());
    sorted.erase(std::unique(sorted.begin(), sorted.end()), sorted.end());
    RegionGraphStats local;
    local.tiles_requested = sorted.size();
    std::vector<RoadGraphTile> tiles;
    for (const TileKey& key : sorted) {
        auto tile = fetch(key);
        if (!tile) return tile.error();
        if (!tile->has_value()) { ++local.tiles_absent; continue; }
        if (!((*tile)->key == key)) return make_error(ErrorCode::Malformed, "region graph: fetched tile has another key");
        tiles.push_back(std::move(**tile));
        ++local.tiles_present;
    }
    std::vector<const RoadGraph*> graphs;
    for (const auto& t : tiles) graphs.push_back(&t.graph);
    auto merged = merge_region_graphs(graphs, &local);
    if (stats) *stats = local;
    return merged;
}

std::vector<TurnRestriction> merge_turn_restrictions(std::span<const RestrictionSet> sets) {
    std::vector<TurnRestriction> out;
    for (const auto& set : sets) out.insert(out.end(), set.restrictions.begin(), set.restrictions.end());
    const auto key_less = [](const TurnRestriction& a, const TurnRestriction& b) {
        if (a.relation_id != b.relation_id) return a.relation_id < b.relation_id;
        if (a.from_way != b.from_way) return a.from_way < b.from_way;
        return a.to_way < b.to_way;
    };
    std::stable_sort(out.begin(), out.end(), key_less);
    out.erase(std::unique(out.begin(), out.end(), [&](const auto& a, const auto& b) { return !key_less(a, b) && !key_less(b, a); }),
              out.end());
    return out;
}

Result<RegionRouting> build_region_routing(const RoadGraph& graph, std::span<const TurnRestriction> restrictions,
                                           const RouteConfig& config, std::size_t landmark_count) {
    auto built = build_routing_graph(graph, restrictions, config);
    if (!built) return built.error();
    RegionRouting out;
    out.graph = std::make_unique<RoutingGraph>(std::move(*built));
    if (landmark_count > 0 && !out.graph->nodes().empty()) {
        auto lm = RouteLandmarks::build(*out.graph, landmark_count);
        if (!lm) return lm.error();
        out.landmarks = std::make_unique<RouteLandmarks>(std::move(*lm));
    }
    return out;
}

} // namespace g2m::route
