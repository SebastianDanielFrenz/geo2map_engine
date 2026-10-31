// tests/support/road_fixture.h - road-test helpers shared by the street
// fidelity P1 tests: load a committed OSM fixture PBF as a RoadGraph (the
// complete world of that file), derive a roads.geom tile of it exactly the way
// the deriver does (tile graph + context graph from the tiles that hold every
// incident way), and pick ways by tag. Header-only, test-only.
#pragma once

#include "synthetic.h"

#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/layer/road_dem_dependencies.h"
#include "g2m/layer/road_completion.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_underpass.h"
#include "g2m/layer/road_graph_tile.h"

#include <algorithm>
#include <cmath>
#include <functional>
#include <map>
#include <set>
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

// Fills a DEM tile: flat 100 m, or `height_m(easting, northing)` at every cell centre (1/256 m, rounded).
inline void fill_dem(g2m::HeightTile& tile, const std::function<double(double, double)>& height_m) {
    if (!height_m) { tile.h.fill(25600); return; }
    const double cell = static_cast<double>(tile.key.size_m()) / 256.0;
    const double e0 = static_cast<double>(tile.key.min_easting()), n0 = static_cast<double>(tile.key.min_northing());
    for (std::size_t j = 0; j < 256; ++j)
        for (std::size_t i = 0; i < 256; ++i)
            tile.h[j * 256 + i] = static_cast<std::int32_t>(std::llround(height_m(e0 + (static_cast<double>(i) + 0.5) * cell, n0 + (static_cast<double>(j) + 0.5) * cell) * 256.0));
}

// The inputs RoadGeomDeriver resolves for one level-2 tile of `world`: the tile graph, the merged context (stage 1:
// the tiles of every node of the tile's ways; stage 2, decision D35: the tiles of every node of each bridge incident
// to those nodes) and the DEM tile keys (the tile's ways and those bridges, 65 m margin). `bridge_far_end` = false
// stops after stage 1 (the pre-D35 dependency set, a negative control).
struct RoadGeomInputs {
    g2m::RoadGraphTile tile;
    g2m::RoadGraph context;
    std::vector<g2m::TileKey> dem_keys;
    g2m::RoadGraph crossings;   // D57: the tile graph plus the roads.graph tiles of road_crossing_dependencies
};
// `walk_margin_mm`: the D48 squares of road_bridge_dependencies (0 = the pre-D48 stage 2, a negative control).
inline g2m::Result<RoadGeomInputs> road_geom_inputs(const g2m::RoadGraph& world, const g2m::TileKey& key, bool bridge_far_end = true,
        std::int64_t walk_margin_mm = g2m::kRoadBridgeWalkMarginMm) {
    using namespace g2m;
    RoadGeomInputs in;
    auto tile = select_road_graph_tile(world, key); if (!tile) return tile.error();
    in.tile = std::move(*tile);
    const auto dem_deps = road_dem_dependencies(in.tile.graph, key.zone); if (!dem_deps) return dem_deps.error();
    std::set<TileKey> dem_keys; for (const auto& d : *dem_deps) dem_keys.insert(d.key);
    const auto deps = road_context_dependencies(in.tile.graph, key.zone, &key); if (!deps) return deps.error();
    std::set<TileKey> graph_keys; for (const auto& d : *deps) graph_keys.insert(d.key);
    const auto merge = [&]() -> Result<RoadGraph> {
        std::vector<RoadGraphTile> others;
        for (const auto& k : graph_keys) { auto t = select_road_graph_tile(world, k); if (!t) return t.error(); others.push_back(std::move(*t)); }
        std::vector<const RoadGraph*> graphs{&in.tile.graph};
        for (const auto& t : others) graphs.push_back(&t.graph);
        return merge_road_graphs(graphs);
    };
    auto context = merge(); if (!context) return context.error();
    if (bridge_far_end) {
        const auto bridges = road_bridge_dependencies(in.tile.graph, *context, key.zone, &key, walk_margin_mm); if (!bridges) return bridges.error();
        for (const auto& d : *bridges) (d.layer == kRoadGraphLayer ? graph_keys : dem_keys).insert(d.key);
        context = merge(); if (!context) return context.error();
    }
    in.context = std::move(*context);
    in.dem_keys.assign(dem_keys.begin(), dem_keys.end());
    {   // D57: the crossing graph, as RoadGeomDeriver builds it.
        const auto refs = road_crossing_dependencies(in.tile.graph, key.zone, &key); if (!refs) return refs.error();
        std::vector<RoadGraphTile> others;
        for (const auto& r : *refs) { auto t = select_road_graph_tile(world, r.key); if (!t) return t.error(); others.push_back(std::move(*t)); }
        std::vector<const RoadGraph*> graphs{&in.tile.graph};
        for (const auto& t : others) graphs.push_back(&t.graph);
        auto crossings = merge_road_graphs(graphs); if (!crossings) return crossings.error();
        in.crossings = std::move(*crossings);
    }
    return in;
}

// roads.geom of one level-2 tile of `world`, the way RoadGeomDeriver does it.
// `with_context` = false builds from the (partial) tile graph alone, the
// pre-slice-1 behaviour, for negative controls. `height_m(easting, northing)` replaces the flat 100 m DEM
// (25600 / 256) by a terrain function, for the tilted and bumpy cases.
inline g2m::Result<g2m::RoadGeomTile> geometry_in_tile(const g2m::RoadGraph& world, const g2m::TileKey& key,
        bool with_context = true, const std::function<double(double, double)>& height_m = {}) {
    using namespace g2m;
    auto in = road_geom_inputs(world, key, with_context); if (!in) return in.error();
    std::vector<HeightTile> heights(in->dem_keys.size());
    std::vector<const HeightTile*> views;
    for (std::size_t i = 0; i < heights.size(); ++i) { heights[i].key = in->dem_keys[i]; fill_dem(heights[i], height_m); views.push_back(&heights[i]); }
    const auto dem = RoadDemSampler::make(views); if (!dem) return dem.error();
    RoadProfileLimits limits; limits.reference.allow_curvature_exceptions = true;
    if (!with_context) return build_road_geom_tile(in->tile, *dem, limits);
    return build_road_geom_tile(in->tile, *dem, limits, &in->context);
}

// Everything geometry_in_tile can vary (the completion tier of slice 6 and its inputs).
struct GeomOptions {
    bool with_context = true;
    bool bridge_far_end = true;                               // D35's second discovery stage (false: negative control)
    std::function<double(double, double)> height_m = {};      // replaces the flat 100 m DEM
    g2m::RoadCompletionOptions completion = {};
    g2m::RoadCompletionStats* stats = nullptr;
    // Called on the built tile with the tile's own DEM sampler before it is returned (e.g. the game's completion code).
    std::function<void(g2m::RoadGeomTile&, const g2m::RoadDemSampler&)> post = {};
    // Underpass decks (D53-D58): the pass on, its crossing graph (false: the tile graph alone, a negative control),
    // its stats, and the rules it runs with (null: the built-in de.json).
    bool underpass = false;
    bool crossing_context = true;
    g2m::RoadUnderpassStats* underpass_stats = nullptr;
    const g2m::RoadRules* rules = nullptr;
};
inline g2m::Result<g2m::RoadGeomTile> geometry_in_tile_with(const g2m::RoadGraph& world, const g2m::TileKey& key, const GeomOptions& o) {
    using namespace g2m;
    auto in = road_geom_inputs(world, key, o.with_context && o.bridge_far_end); if (!in) return in.error();
    std::vector<HeightTile> heights(in->dem_keys.size());
    std::vector<const HeightTile*> views;
    for (std::size_t i = 0; i < heights.size(); ++i) { heights[i].key = in->dem_keys[i]; fill_dem(heights[i], o.height_m); views.push_back(&heights[i]); }
    const auto dem = RoadDemSampler::make(views); if (!dem) return dem.error();
    RoadProfileLimits limits; limits.reference.allow_curvature_exceptions = true;
    RoadUnderpassOptions underpass; underpass.enabled = o.underpass; underpass.stats = o.underpass_stats;
    underpass.crossings = o.crossing_context ? &in->crossings : &in->tile.graph;
    auto tile = build_road_geom_tile(in->tile, *dem, limits, o.with_context ? &in->context : nullptr, o.rules ? *o.rules : builtin_road_rules_de(),
        o.completion, o.stats, underpass);
    if (tile && o.post) o.post(*tile, *dem);
    return tile;
}
// The tile RoadGeomDeriver serves (slice 8): context graph, the served completion options, the underpass decks (D53).
inline g2m::Result<g2m::RoadGeomTile> served_geometry_in_tile(const g2m::RoadGraph& world, const g2m::TileKey& key,
        const std::function<double(double, double)>& height_m = {}, const g2m::RoadRules* rules = nullptr) {
    GeomOptions o; o.height_m = height_m; o.completion = g2m::served_road_completion_options(); o.underpass = true; o.rules = rules;
    return geometry_in_tile_with(world, key, o);
}

} // namespace g2m_test
