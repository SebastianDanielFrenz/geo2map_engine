// tests/world/road_mesh_test_data.h - inputs and hashes for the phase-3a mesh pins (terrain migration plan phase 3a):
// the structure decks (road_structures.h) and the road ribbons (road_ribbons.h) of a set of roads.geom tiles.
//
// The pins in road_mesh_pins.inc were produced by racing_game's own code (0c027dc: road_structures.cpp and
// road_visual.cpp, extracted verbatim by refgen/extract_mesh_ref.sh) run over exactly these inputs by
// refgen/refgen_meshes.cpp; test_road_meshes.cpp runs the ported code over the same inputs. Every input is built by
// geo2map's own deterministic code (served roads.geom tiles of the committed OSM fixtures and two synthetic scenes) and
// plain arithmetic, so every compiler and OS builds the same tiles. Exceptions instead of Catch2 assertions: the
// reference generator includes this header and links no Catch2. The hash helpers are templates so they hash the game's
// rg:: types and the ported g2m::world:: types (same members, same order) the same way.
#pragma once

#include "road_fixture.h"
#include "road_junction_scene.h"

#include "g2m/core/det_math.h"
#include "g2m/core/hash.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_rules.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef G2M_SOURCE_DIR
#error "G2M_SOURCE_DIR must name the source tree (the OSM fixtures)"
#endif

namespace g2m_mesh_test {

// A synthetic world with every kind of structure, around the centre of tile (456, 5421) (metres from the centre):
//   way 100 (primary, rank 7)        101 (-300,0) - 102 (-100,0) - 103 (100,0) - 104 (300,0)
//   way 110 (residential, tunnel=yes, layer=-1)   111 (-40,-150) - 112 (30,150): passes under way 100 (an inferred roof)
//   way 120 (secondary, bridge=yes, layer=1)      122 (0,250) - 123 (200,250), approached by
//   way 121 (secondary)  124 (-250,250) - 122,    way 126 (secondary)  123 - 125 (400,250)
//   way 130 (motorway, rank 9, layer=1)           131 (-300,-300) - 132 (300,-260): over
//   way 140 (service, tunnel=yes, layer=-1)       141 (0,-400) - 142 (10,-150)
// The DEM is tilted (2 % east, -1 % north) so profiles carry grades.
inline const g2m::TileKey kStructureSceneTile{g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North}, 2, 456, 5421};
inline g2m::RoadGraph structure_scene_graph() {
    using Tags = std::vector<std::pair<std::string, std::string>>;
    struct Node { std::int64_t id; double x, y; };
    struct Way { std::int64_t id; int rank; std::vector<std::int64_t> nodes; Tags tags; };
    constexpr double e0 = 456.0 * 1024.0 + 512.0, n0 = 5421.0 * 1024.0 + 512.0;
    const std::vector<Node> nodes = {{101, -300, 0}, {102, -100, 0}, {103, 100, 0}, {104, 300, 0}, {111, -40, -150}, {112, 30, 150},
        {122, 0, 250}, {123, 200, 250}, {124, -250, 250}, {125, 400, 250}, {131, -300, -300}, {132, 300, -260}, {141, 0, -400}, {142, 10, -150}};
    const std::vector<Way> ways = {
        {100, 7, {101, 102, 103, 104}, {{"highway", "primary"}}},
        {110, 3, {111, 112}, {{"highway", "residential"}, {"layer", "-1"}, {"tunnel", "yes"}}},
        {120, 6, {122, 123}, {{"bridge", "yes"}, {"highway", "secondary"}, {"layer", "1"}}},
        {121, 6, {124, 122}, {{"highway", "secondary"}}},
        {126, 6, {123, 125}, {{"highway", "secondary"}}},
        {130, 9, {131, 132}, {{"highway", "motorway"}, {"layer", "1"}}},
        {140, 1, {141, 142}, {{"highway", "service"}, {"layer", "-1"}, {"tunnel", "yes"}}}};
    g2m::RoadGraph graph;
    for (const auto& n : nodes) graph.nodes.push_back({n.id, {std::llround((e0 + n.x) * 1000.0), std::llround((n0 + n.y) * 1000.0)}, {}});
    for (const auto& w : ways) {
        graph.ways.push_back({w.id, w.rank, w.nodes, w.tags});
        for (std::size_t i = 1; i < w.nodes.size(); ++i) graph.edges.push_back({w.id, static_cast<std::uint32_t>(i - 1), w.nodes[i - 1], w.nodes[i]});
    }
    return graph;
}
inline double structure_scene_dem(double e, double n) {
    constexpr double e0 = 456.0 * 1024.0 + 512.0, n0 = 5421.0 * 1024.0 + 512.0;
    return 120.0 + 0.02 * (e - e0) - 0.01 * (n - n0);
}

// A grade-separated crossing (racing_game tests/unit/test_road_overpass.cpp's scene, 945455a, built here as a road graph
// instead of OSM): a motorway (two one-way 3-lane carriageways 7.5 m either side of an east-west axis through the centre
// of tile (457, 5422), 600 m each way) under a secondary road at 70 deg (approach 200 - bridge 201 (bridge=yes,
// layer=1, +-bridge_half_m around the crossing) - approach 202, nodes every 10 m, 300 m each way). The DEM is flat 100 m;
// the overpass road stands on an embankment that rises to 7.5 m at the abutments and falls back to grade 150 m further
// out, 6 m top half width, 1:1.5 side slopes; bare earth under the span.
inline const g2m::TileKey kOverpassTile{g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North}, 2, 457, 5422};
constexpr double kOverpassE = 457.0 * 1024.0 + 512.0, kOverpassN = 5422.0 * 1024.0 + 512.0;
constexpr double kOverpassHeadingRad = 70.0 * 3.14159265358979323846 / 180.0;
inline double overpass_dem(double bridge_half_m, double e, double n) {
    const double ux = g2m::det::cos(kOverpassHeadingRad), uy = g2m::det::sin(kOverpassHeadingRad);
    const double along = (e - kOverpassE) * ux + (n - kOverpassN) * uy, across = std::abs(-(e - kOverpassE) * uy + (n - kOverpassN) * ux);
    const double a = std::abs(along);
    if (a < bridge_half_m) return 100;
    const double crest = 7.5 * std::clamp(1 - (a - bridge_half_m) / 150, 0.0, 1.0);
    return 100 + std::max(0.0, crest - std::max(0.0, across - 6.0) / 1.5);
}
inline g2m::RoadGraph overpass_graph(double bridge_half_m) {
    using Tags = std::vector<std::pair<std::string, std::string>>;
    g2m::RoadGraph graph;
    std::int64_t next = 1;
    const auto node = [&](double e, double n) {
        graph.nodes.push_back({next, {std::llround(e * 1000.0), std::llround(n * 1000.0)}, {}});
        return next++;
    };
    const auto way = [&](std::int64_t id, int rank, std::vector<std::int64_t> refs, Tags tags) {
        for (std::size_t i = 1; i < refs.size(); ++i) graph.edges.push_back({id, static_cast<std::uint32_t>(i - 1), refs[i - 1], refs[i]});
        graph.ways.push_back({id, rank, std::move(refs), std::move(tags)});
    };
    for (int side : {-1, 1}) {
        std::vector<std::int64_t> refs;
        for (int i = -12; i <= 12; ++i) refs.push_back(node(kOverpassE + i * 50.0 * (side < 0 ? 1 : -1), kOverpassN + side * 7.5));
        way(side < 0 ? 100 : 101, 9, std::move(refs), {{"highway", "motorway"}, {"lanes", "3"}, {"oneway", "yes"}, {"ref", "A 66"}});
    }
    const double ux = g2m::det::cos(kOverpassHeadingRad), uy = g2m::det::sin(kOverpassHeadingRad);
    const auto at = [&](double t) { return node(kOverpassE + ux * t, kOverpassN + uy * t); };
    std::vector<std::int64_t> south, bridge, north;
    for (double t = -300; t < -bridge_half_m; t += 10) south.push_back(at(t));
    const auto a0 = at(-bridge_half_m);
    south.push_back(a0);
    bridge.push_back(a0);
    bridge.push_back(at(0));
    const auto a1 = at(bridge_half_m);
    bridge.push_back(a1);
    north.push_back(a1);
    for (double t = bridge_half_m + 10; t <= 300; t += 10) north.push_back(at(t));
    const Tags road{{"highway", "secondary"}, {"lanes", "2"}};
    way(200, 6, south, road);
    way(201, 6, bridge, {{"bridge", "yes"}, {"highway", "secondary"}, {"lanes", "2"}, {"layer", "1"}});
    way(202, 6, north, road);
    std::sort(graph.nodes.begin(), graph.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    std::sort(graph.ways.begin(), graph.ways.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    std::sort(graph.edges.begin(), graph.edges.end(), [](const auto& a, const auto& b) {
        return a.way_id != b.way_id ? a.way_id < b.way_id : a.segment_index < b.segment_index;
    });
    return graph;
}

// One mesh case: a roads.geom tile, the key the ribbons are built for, the ground model, the ribbon lift and the grade
// separation. ground 0: smooth (a tilted plane with a twist, relative to the key's south-west corner); 1: the same with
// periodic holes (nullopt in about one 37 m x 53 m block in eleven); 2: the overpass scene's DEM (bridge_half_m).
// separated false: no approach decks (racing_game 0c027dc, or 945455a with clearance 1e9 / margin 0 / reach 0);
// true: the default RoadSeparation (clearance 2.5 m, margin 3 m, reach 60 m).
struct MeshCase {
    std::string name;
    g2m::RoadGeomTile tile;
    g2m::TileKey key;
    int ground = 0;
    double lift = 0.0;
    bool separated = false;
    double bridge_half_m = 0.0;
};
inline std::optional<double> case_ground(const MeshCase& c, double e, double n) {
    if (c.ground == 2) return overpass_dem(c.bridge_half_m, e, n);
    const double x = e - static_cast<double>(c.key.min_easting()), y = n - static_cast<double>(c.key.min_northing());
    if (c.ground == 1) {
        const long long bx = static_cast<long long>(std::floor(e / 37.0)), by = static_cast<long long>(std::floor(n / 53.0));
        if (((bx + by) % 11 + 11) % 11 == 0) return std::nullopt;
    }
    return 100.0 + 0.01 * x - 0.006 * y + 1e-5 * x * y;
}

inline void fixture_cases(std::vector<MeshCase>& out, const std::string& name, const std::string& file, bool holed, bool separated = false) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + file);
    auto world = g2m::build_road_graph(data, g2m_test::kZone32N);
    if (!world) throw std::runtime_error(name + ": graph: " + world.error().message);
    std::set<g2m::TileKey> keys;
    for (const auto& node : world->nodes) {
        const auto key = g2m::tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(node.position.x) / 1000.0, static_cast<double>(node.position.y) / 1000.0);
        if (key) keys.insert(*key);
    }
    for (const auto& key : keys) {
        const auto probe = g2m::select_road_graph_tile(*world, key);
        if (!probe || probe->owned_edges.empty()) continue;
        // The pins are racing_game 945455a's output, which knows no merge lane (vault G2M-028): the fixtures are built without it.
        static const g2m::RoadRules no_merge = [] { auto r = g2m::builtin_road_rules_de(); r.merge_lane.enabled = false; return r; }();
        auto tile = g2m_test::served_geometry_in_tile(*world, key, {}, &no_merge);
        if (!tile) throw std::runtime_error(name + ": geometry: " + tile.error().message);
        const std::string base = name + "." + std::to_string(key.x) + "." + std::to_string(key.y);
        if (separated) {
            out.push_back({base + ".sep", std::move(*tile), key, 0, 0.018, true});
            continue;
        }
        out.push_back({base, *tile, key, 0, 0.018});
        if (holed) out.push_back({base + ".holed", std::move(*tile), key, 1, 0.0});
    }
}

// Every case, in a fixed order.
inline std::vector<MeshCase> mesh_cases() {
    std::vector<MeshCase> out;
    {
        g2m_test::GeomOptions o;
        o.height_m = structure_scene_dem;
        o.completion = g2m::served_road_completion_options();
        auto tile = g2m_test::geometry_in_tile_with(structure_scene_graph(), kStructureSceneTile, o);
        if (!tile) throw std::runtime_error("structure scene: " + tile.error().message);
        out.push_back({"structure_scene", *tile, kStructureSceneTile, 0, 0.018});
        out.push_back({"structure_scene.holed", std::move(*tile), kStructureSceneTile, 1, 0.0});
    }
    {
        auto tile = g2m_test::junction_scene_tile();
        if (!tile) throw std::runtime_error("junction scene: " + tile.error().message);
        out.push_back({"junction_scene", *tile, g2m_test::kJunctionSceneTile, 0, 0.018});
        out.push_back({"junction_scene.holed", std::move(*tile), g2m_test::kJunctionSceneTile, 1, 0.0});
    }
    fixture_cases(out, "frankfurt_hoechst", "frankfurt_hoechst.osm.pbf", false);
    fixture_cases(out, "hoechst_a66", "hoechst_a66.osm.pbf", true);
    // Grade separation on (racing_game 945455a): the overpass scenes with a short and a long span, each on and off,
    // the structure scene and every hoechst_a66 tile on.
    for (const double half : {10.0, 25.0}) {
        g2m_test::GeomOptions o;
        o.height_m = [half](double e, double n) { return overpass_dem(half, e, n); };
        o.completion = g2m::served_road_completion_options();
        auto tile = g2m_test::geometry_in_tile_with(overpass_graph(half), kOverpassTile, o);
        if (!tile) throw std::runtime_error("overpass scene: " + tile.error().message);
        const std::string base = "overpass.h" + std::to_string(static_cast<int>(half));
        out.push_back({base + ".sep", *tile, kOverpassTile, 2, 0.018, true, half});
        out.push_back({base, std::move(*tile), kOverpassTile, 2, 0.018, false, half});
    }
    {
        MeshCase c = out.front();
        c.name = "structure_scene.sep";
        c.separated = true;
        out.push_back(std::move(c));
    }
    fixture_cases(out, "hoechst_a66", "hoechst_a66.osm.pbf", false, true);
    return out;
}

inline std::uint32_t f32_bits(float v) { std::uint32_t b; std::memcpy(&b, &v, 4); return b; }
inline std::uint64_t f64_bits(double v) { std::uint64_t b; std::memcpy(&b, &v, 8); return b; }

// Every member of a deck (bounds, flags, vertical segments, mesh origin, positions, normals, classes, indices) and its
// height at a fixed probe set (stations from 0.5 m before the start to 0.5 m past the end, four laterals).
template <class Deck, class HeightFn>
void hash_deck(g2m::Fnv1a64& h, const Deck& d, HeightFn height) {
    h.i64(d.way_id).f64(d.start_station).f64(d.end_station).f64(d.half_width).f64(d.crown).u8(d.inferred ? 1 : 0).u8(d.tunnel_floor ? 1 : 0)
        .f64(d.min_easting).f64(d.max_easting).f64(d.min_northing).f64(d.max_northing).u8(static_cast<std::uint8_t>(d.land_class));
    h.u64(d.vertical.size());
    for (const auto& c : d.vertical) h.f64(c.station_m).f64(c.length_m).f64(c.height0_m).f64(c.height1_m).f64(c.grade0).f64(c.grade1);
    const auto& m = d.mesh;
    h.f64(m.origin[0]).f64(m.origin[1]).f64(m.origin[2]).u64(m.positions.size()).u64(m.indices.size());
    for (float v : m.positions) h.u32(f32_bits(v));
    for (float v : m.normals) h.u32(f32_bits(v));
    for (auto v : m.land_class) h.u8(v);
    for (auto v : m.indices) h.u16(v);
    const double span = d.end_station - d.start_station + 1.0;
    for (int i = 0; i <= 7; ++i)
        for (double lateral : {-3.0, -0.7, 0.0, 1.9}) {
            const auto z = height(d, d.start_station - 0.5 + span * i / 7.0, lateral);
            h.u8(z ? 1 : 0).u64(z ? f64_bits(*z) : 0);
        }
}
// Every vertex member and every index of a ribbon mesh.
template <class Mesh>
std::uint64_t hash_ribbon(const Mesh& m) {
    g2m::Fnv1a64 h;
    h.u64(m.vertices.size()).u64(m.indices.size());
    for (const auto& v : m.vertices)
        for (float f : {v.x, v.y, v.z, v.u, v.v, v.width, v.fade, v.lanes, v.forward, v.flags, v.verge}) h.u32(f32_bits(f));
    for (auto i : m.indices) h.i32(i);
    return h.value();
}

} // namespace g2m_mesh_test
