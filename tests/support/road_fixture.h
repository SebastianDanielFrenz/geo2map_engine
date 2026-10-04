// tests/support/road_fixture.h - road-test helpers shared by the street
// fidelity P1 tests: load a committed OSM fixture PBF as a RoadGraph (the
// complete world of that file), derive a roads.geom tile of it exactly the way
// the deriver does (tile graph + context graph from the tiles that hold every
// incident way), and pick ways by tag. Header-only, test-only.
#pragma once

#include "synthetic.h"

#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_graph_tile.h"

#include <algorithm>
#include <map>
#include <string>
#include <vector>

namespace g2m_test {

// Reads nodes and ways of a PBF into canonical OSM data (relations ignored).
inline g2m::osm::OsmData read_osm_fixture(const std::string& path) {
    struct Reader final : g2m::decode::osm::PbfHandler {
        g2m::osm::OsmBuilder builder;
        void on_node(const g2m::decode::osm::OsmNode& n) override {
            g2m::osm::OsmBuilder::TagList tags;
            for (const auto& tag : n.tags) tags.emplace_back(tag.key, tag.value);
            builder.add_node(n.id, n.lat_1e7, n.lon_1e7, std::move(tags));
        }
        void on_way(const g2m::decode::osm::OsmWay& w) override {
            g2m::osm::OsmBuilder::TagList tags;
            for (const auto& tag : w.tags) tags.emplace_back(tag.key, tag.value);
            builder.add_way(w.id, w.node_refs, std::move(tags));
        }
    } reader;
    g2m::decode::osm::read_pbf(path, reader, nullptr,
        g2m::decode::osm::PbfEntities::Nodes | g2m::decode::osm::PbfEntities::Ways);
    auto data = reader.builder.build();
    return data ? std::move(*data) : g2m::osm::OsmData{};
}

inline std::string way_tag(const g2m::RoadGraphWay& way, const std::string& key) {
    for (const auto& [k, v] : way.tags) if (k == key) return v;
    return {};
}

// Keeps the ways satisfying `keep` and the nodes they use: a smaller, still
// self-consistent world (its own node degrees are the truth).
template <class Pred>
inline g2m::RoadGraph filter_road_graph(const g2m::RoadGraph& world, Pred keep) {
    g2m::RoadGraph out;
    std::map<std::int64_t, g2m::geom::PointMm> nodes;
    for (const auto& n : world.nodes) nodes.emplace(n.osm_id, n.position);
    std::map<std::int64_t, bool> used;
    for (const auto& way : world.ways) if (keep(way)) {
        out.ways.push_back(way);
        for (std::size_t i = 0; i < way.node_ids.size(); ++i) {
            used[way.node_ids[i]] = true;
            if (i && way.node_ids[i - 1] != way.node_ids[i])
                out.edges.push_back({way.osm_id, static_cast<std::uint32_t>(i - 1), way.node_ids[i - 1], way.node_ids[i]});
            else if (i) ++out.repeated_ref_pairs;
        }
    }
    for (const auto& [id, position] : nodes) if (used.count(id)) out.nodes.push_back({id, position});
    return out;
}

// roads.geom of one level-2 tile of `world`, the way RoadGeomDeriver does it.
// `with_context` = false builds from the (partial) tile graph alone, the
// pre-slice-1 behaviour, for negative controls.
inline g2m::Result<g2m::RoadGeomTile> geometry_in_tile(const g2m::RoadGraph& world, const g2m::TileKey& key,
        bool with_context = true) {
    using namespace g2m;
    auto tile = select_road_graph_tile(world, key); if (!tile) return tile.error();
    const auto dem_deps = road_dem_dependencies(tile->graph, key.zone); if (!dem_deps) return dem_deps.error();
    std::vector<HeightTile> heights(dem_deps->size());
    std::vector<const HeightTile*> views;
    for (std::size_t i = 0; i < heights.size(); ++i) { heights[i].key = (*dem_deps)[i].key; heights[i].h.fill(25600); views.push_back(&heights[i]); }
    const auto dem = RoadDemSampler::make(views); if (!dem) return dem.error();
    RoadProfileLimits limits; limits.reference.allow_curvature_exceptions = true;
    if (!with_context) return build_road_geom_tile(*tile, *dem, limits);
    const auto deps = road_context_dependencies(tile->graph, key.zone, &key); if (!deps) return deps.error();
    std::vector<RoadGraphTile> others;
    for (const auto& dep : *deps) { auto t = select_road_graph_tile(world, dep.key); if (!t) return t.error(); others.push_back(std::move(*t)); }
    std::vector<const RoadGraph*> graphs{&tile->graph};
    for (const auto& t : others) graphs.push_back(&t.graph);
    const auto context = merge_road_graphs(graphs); if (!context) return context.error();
    return build_road_geom_tile(*tile, *dem, limits, &*context);
}

} // namespace g2m_test
