// tests/support/road_junction_scene.h - a small synthetic world whose served roads.geom tile carries junctions and
// furniture of every kind (street fidelity P2 slice 8: the encoding-7 tests and the roads.geom fuzz seeds). Header-only,
// test-only, no Catch2 and no PBF reader (the fuzz seed library includes it).
//
// Around the centre of tile (455, 5421), positions in metres from the centre:
//   way 10 (secondary)   1 (-200,0) - 2 (-100,0) zebra crossing - 3 (0,0) traffic signals - 4 (100,0) - 5 (200,0) - 10 (300,0)
//   way 11 (residential) 6 (0,-150) - 3 - 7 (0,150)
//   way 12 (residential) 8 (200,-150) - 5 - 9 (200,150)            node 5: stop sign and a raised table
//   way 13 (residential) 7 - 20 (0,700), leaving the tile             node 7: a zebra crossing on a junction node
//   way 14 (residential) 7 - 21 (-150,150)
// Junctions at 3 (signals: stop lines and signal posts), 5 (raised table; the stop sign lines the side road) and 7
// (a declined JunctionNode crossing); a placed crossing at 2. Node 20 lies outside the tile. Flat 100 m DEM.
#pragma once

#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_graph_tile.h"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace g2m_test {

inline const g2m::TileKey kJunctionSceneTile{g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North}, 2, 455, 5421};

inline g2m::RoadGraph junction_scene_graph() {
    using Tags = std::vector<std::pair<std::string, std::string>>;
    struct Node { std::int64_t id; double x, y; Tags tags; };
    struct Way { std::int64_t id; int rank; std::vector<std::int64_t> nodes; std::string highway; };
    constexpr double e0 = 455.0 * 1024.0 + 512.0, n0 = 5421.0 * 1024.0 + 512.0;
    const std::vector<Node> nodes = {
        {1, -200, 0, {}}, {2, -100, 0, {{"crossing", "zebra"}, {"highway", "crossing"}}}, {3, 0, 0, {{"highway", "traffic_signals"}}},
        {4, 100, 0, {}}, {5, 200, 0, {{"highway", "stop"}, {"traffic_calming", "table"}}}, {6, 0, -150, {}},
        {7, 0, 150, {{"crossing", "zebra"}, {"highway", "crossing"}}}, {8, 200, -150, {}}, {9, 200, 150, {}}, {10, 300, 0, {}},
        {20, 0, 700, {}}, {21, -150, 150, {}}};
    const std::vector<Way> ways = {{10, 6, {1, 2, 3, 4, 5, 10}, "secondary"}, {11, 3, {6, 3, 7}, "residential"},
        {12, 3, {8, 5, 9}, "residential"}, {13, 3, {7, 20}, "residential"}, {14, 3, {7, 21}, "residential"}};
    g2m::RoadGraph graph;
    for (const auto& n : nodes) graph.nodes.push_back({n.id, {std::llround((e0 + n.x) * 1000.0), std::llround((n0 + n.y) * 1000.0)}, n.tags});
    for (const auto& w : ways) {
        graph.ways.push_back({w.id, w.rank, w.nodes, {{"highway", w.highway}}});
        for (std::size_t i = 1; i < w.nodes.size(); ++i) graph.edges.push_back({w.id, static_cast<std::uint32_t>(i - 1), w.nodes[i - 1], w.nodes[i]});
    }
    return graph;
}

// The served tile of the scene, built the way RoadGeomDeriver does it: tile graph, context graph from the tiles that
// hold every incident way, flat DEM, curvature exceptions on, the served completion options.
inline g2m::Result<g2m::RoadGeomTile> junction_scene_tile() {
    using namespace g2m;
    const auto world = junction_scene_graph(); const auto key = kJunctionSceneTile;
    auto tile = select_road_graph_tile(world, key); if (!tile) return tile.error();
    const auto dem_deps = road_dem_dependencies(tile->graph, key.zone); if (!dem_deps) return dem_deps.error();
    std::vector<HeightTile> heights(dem_deps->size()); std::vector<const HeightTile*> views;
    for (std::size_t i = 0; i < heights.size(); ++i) { heights[i].key = (*dem_deps)[i].key; heights[i].h.fill(25600); views.push_back(&heights[i]); }
    const auto dem = RoadDemSampler::make(views); if (!dem) return dem.error();
    RoadProfileLimits limits; limits.reference.allow_curvature_exceptions = true;
    const auto deps = road_context_dependencies(tile->graph, key.zone, &key); if (!deps) return deps.error();
    std::vector<RoadGraphTile> others;
    for (const auto& dep : *deps) { auto t = select_road_graph_tile(world, dep.key); if (!t) return t.error(); others.push_back(std::move(*t)); }
    std::vector<const RoadGraph*> graphs{&tile->graph};
    for (const auto& t : others) graphs.push_back(&t.graph);
    const auto context = merge_road_graphs(graphs); if (!context) return context.error();
    return build_road_geom_tile(*tile, *dem, limits, &*context, builtin_road_rules_de(), served_road_completion_options());
}

} // namespace g2m_test
