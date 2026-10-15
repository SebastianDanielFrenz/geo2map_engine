// tests/world/building_test_data.h - inputs and hashes for the phase-3b BuildingSet pins (terrain migration plan phase
// 3b): extract_buildings over a synthetic OSM scene with every tag form the game reads, plus the committed OSM fixtures.
//
// The pins in building_pins.inc were produced by racing_game's own core/src/buildings.cpp (5a3baed, extracted verbatim
// by refgen/extract_view_ref.sh) run over exactly these inputs by refgen/refgen_view.cpp; test_buildings.cpp runs the
// ported g2m::world::extract_buildings over the same inputs. Exceptions instead of Catch2 assertions (the reference
// generator links no Catch2). The hash helper is a template so it hashes rg::Building and g2m::world::Building alike.
#pragma once

#include "road_fixture.h"
#include "world_test_data.h"

#include "g2m/core/hash.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/src_osm.h"

#include <cmath>
#include <cstdint>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#ifndef G2M_SOURCE_DIR
#error "G2M_SOURCE_DIR must name the source tree (the OSM fixtures)"
#endif

namespace g2m_building_test {

using g2m_world_test::kOsmKey; // level 2, 32N x 350 y 5450: E 358400..359424, N 5580800..5581824

// A square footprint way: nodes id0..id0+3 around centre (e, n) with half side h (closed by repeating id0).
inline void square(g2m::osm::OsmBuilder& b, std::int64_t way, std::int64_t id0, double e, double n, double h,
                   g2m::osm::OsmBuilder::TagList tags) {
    g2m_world_test::add_utm_node(b, id0, e - h, n - h);
    g2m_world_test::add_utm_node(b, id0 + 1, e + h, n - h);
    g2m_world_test::add_utm_node(b, id0 + 2, e + h, n + h);
    g2m_world_test::add_utm_node(b, id0 + 3, e - h, n + h);
    b.add_way(way, {id0, id0 + 1, id0 + 2, id0 + 3, id0}, std::move(tags));
}

// Every height/tag form extract_buildings distinguishes, inside kOsmKey (and two footprints owned by its neighbours).
inline g2m::OsmTile building_scene() {
    g2m::osm::OsmBuilder b;
    const double e = 358400.0, n = 5580800.0;
    std::int64_t id = 1000;
    std::int64_t way = 1;
    const auto sq = [&](double x, double y, double h, g2m::osm::OsmBuilder::TagList tags) {
        square(b, way++, id, e + x, n + y, h, std::move(tags));
        id += 4;
    };
    sq(100, 100, 6, {{"building", "yes"}});                                                // fallback
    sq(130, 100, 6, {{"building", "house"}, {"height", "12.5"}});                          // height
    sq(160, 100, 6, {{"building", "house"}, {"height", "7,5 m"}});                         // decimal comma + m
    sq(190, 100, 6, {{"building", "yes"}, {"height", "30 ft"}});                           // ft
    sq(220, 100, 6, {{"building", "yes"}, {"height", "20'"}});                             // '
    sq(250, 100, 6, {{"building", "yes"}, {"height", "tall"}});                            // not a number -> fallback
    sq(280, 100, 6, {{"building", "apartments"}, {"building:levels", "5"}});               // levels
    sq(310, 100, 6, {{"building", "yes"}, {"building:levels", "4"}, {"roof:height", "2.5"}});
    sq(340, 100, 6, {{"building", "yes"}, {"building:levels", "3"}, {"roof:levels", "1"}});
    sq(370, 100, 6, {{"building", "yes"}, {"building:levels", "3 m"}});                    // levels with a unit -> fallback
    sq(400, 100, 6, {{"building", "no"}});                                                 // not a building
    sq(430, 100, 6, {{"building:part", "yes"}, {"height", "20"}, {"min_height", "8"}});    // part, min_height
    sq(460, 100, 6, {{"building:part", "yes"}, {"building:levels", "6"}, {"building:min_level", "2"}});
    sq(490, 100, 6, {{"building", "yes"}, {"height", "10"}, {"min_height", "10"}});        // min >= height: dropped
    sq(520, 100, 6, {{"building", "yes"}, {"height", "2500"}});                            // > 2000 -> fallback
    sq(550, 100, 6, {{"building", "yes"}, {"height", "-4"}});                              // negative -> fallback
    sq(580, 100, 6, {{"building", "yes"}, {"height", "0"}});                               // zero -> fallback
    sq(610, 100, 6, {{"building", "church"}, {"height", "60"}, {"building:colour", "#c0b090"}});
    sq(640, 100, 6, {{"building", "yes"}, {"height", "4"}});                               // below a min_height case
    sq(700, 700, 40, {{"building", "commercial"}, {"height", "120"}});                     // big, sloping ground
    sq(-50, 300, 10, {{"building", "yes"}});                                               // owned by the west neighbour
    sq(10, 500, 30, {{"building", "yes"}});                                                // straddles the west edge, owned here
    sq(300, 1030, 10, {{"building", "yes"}});                                              // owned by the north neighbour
    sq(900, 300, 6, {{"building", "yes"}});                                                // over the ground hole
    // A multipolygon: outer ring split into two ways, one inner ring; its member ways carry no building tag.
    g2m_world_test::add_utm_node(b, 5000, e + 500, n + 400);
    g2m_world_test::add_utm_node(b, 5001, e + 560, n + 400);
    g2m_world_test::add_utm_node(b, 5002, e + 560, n + 460);
    g2m_world_test::add_utm_node(b, 5003, e + 500, n + 460);
    g2m_world_test::add_utm_node(b, 5004, e + 520, n + 420);
    g2m_world_test::add_utm_node(b, 5005, e + 540, n + 420);
    g2m_world_test::add_utm_node(b, 5006, e + 540, n + 440);
    g2m_world_test::add_utm_node(b, 5007, e + 520, n + 440);
    b.add_way(200, {5000, 5001, 5002});
    b.add_way(201, {5003, 5002}); // reversed fragment
    b.add_way(202, {5003, 5000});
    b.add_way(203, {5004, 5005, 5006, 5007, 5004});
    b.add_relation(300, {{g2m::osm::MemberType::Way, 200, "outer"}, {g2m::osm::MemberType::Way, 201, "outer"},
                         {g2m::osm::MemberType::Way, 202, ""}, {g2m::osm::MemberType::Way, 203, "inner"}},
                   {{"type", "multipolygon"}, {"building", "retail"}, {"building:levels", "2"}});
    // A building relation over a building way: the way is consumed by the relation.
    g2m_world_test::add_utm_node(b, 5100, e + 600, n + 600);
    g2m_world_test::add_utm_node(b, 5101, e + 620, n + 600);
    g2m_world_test::add_utm_node(b, 5102, e + 620, n + 620);
    b.add_way(210, {5100, 5101, 5102, 5100}, {{"building", "yes"}, {"height", "9"}});
    b.add_relation(310, {{g2m::osm::MemberType::Way, 210, "outer"}}, {{"type", "multipolygon"}, {"building", "office"}});
    // A relation with an unclosable outer ring (dropped) and one with a missing member way (dropped).
    g2m_world_test::add_utm_node(b, 5200, e + 800, n + 800);
    g2m_world_test::add_utm_node(b, 5201, e + 820, n + 800);
    g2m_world_test::add_utm_node(b, 5202, e + 820, n + 820);
    b.add_way(220, {5200, 5201, 5202});
    b.add_relation(320, {{g2m::osm::MemberType::Way, 220, "outer"}}, {{"building", "yes"}});
    b.add_relation(330, {{g2m::osm::MemberType::Way, 999, "outer"}}, {{"building", "yes"}});
    // An open way tagged building (dropped) and a way with a missing node is impossible (OsmBuilder refuses).
    b.add_way(230, {5200, 5202}, {{"building", "yes"}});
    auto data = b.build();
    if (!data) throw std::runtime_error("building scene: " + data.error().message);
    g2m::OsmTile tile;
    tile.halo_m = 128;
    tile.data = std::move(data).value();
    return tile;
}

// Ground: a tilted, curved plane with a hole (no ground) around (E 359300, N 5581100).
inline std::optional<double> ground(double e, double n) {
    const double x = e - 358400.0, y = n - 5580800.0;
    if (x > 860 && x < 940 && y > 260 && y < 340) return std::nullopt;
    return 120.0 + 0.03 * x - 0.02 * y + 2e-5 * x * y;
}

struct Case {
    std::string name;
    g2m::OsmTile tile;
    g2m::TileKey key;
    double min_height, fallback, storey;
};

inline std::vector<Case> cases() {
    std::vector<Case> out;
    const auto scene = building_scene();
    out.push_back({"scene", scene, kOsmKey, 0.0, 8.0, 3.0});
    out.push_back({"scene.skyline", scene, kOsmKey, 50.0, 8.0, 3.0});
    out.push_back({"scene.storey35", scene, kOsmKey, 0.0, 6.0, 3.5});
    g2m::TileKey west = kOsmKey;
    west.x -= 1;
    out.push_back({"scene.west", scene, west, 0.0, 8.0, 3.0});
    g2m::TileKey north = kOsmKey;
    north.y += 1;
    out.push_back({"scene.north", scene, north, 0.0, 8.0, 3.0});
    for (const char* file : {"frankfurt_hoechst", "hoechst_a66"}) {
        g2m::OsmTile t;
        t.halo_m = 128;
        t.data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + file + ".osm.pbf");
        const g2m::geo::Utm utm(g2m::geo::kGRS80);
        std::set<g2m::TileKey> keys;
        for (const auto& node : t.data.nodes) {
            const auto p = utm.forward(g2m_test::kZone32N, node.lat * 1e-7, node.lon * 1e-7);
            const auto k = g2m::tile_key_at(g2m_test::kZone32N, 2, p.easting, p.northing);
            if (k) keys.insert(*k);
        }
        for (const auto& k : keys) out.push_back({std::string(file) + "." + std::to_string(k.x) + "." + std::to_string(k.y), t, k, 0.0, 8.0, 3.0});
    }
    return out;
}

template <class B> std::uint64_t hash_buildings(const std::vector<B>& buildings) {
    g2m::Fnv1a64 h;
    h.u64(buildings.size());
    for (const auto& b : buildings) {
        h.i64(b.id);
        h.u32(b.relation ? 1u : 0u);
        h.f64(b.height);
        h.f64(b.min_height);
        h.f64(b.base);
        h.f64(b.bottom);
        h.str(b.height_source);
        h.str(b.colour);
        h.str(b.type);
        for (const auto* rings : {&b.outers, &b.inners}) {
            h.u64(rings->size());
            for (const auto& r : *rings) {
                h.u64(r.size());
                for (const auto& p : r) {
                    h.f64(p.e);
                    h.f64(p.n);
                }
            }
        }
    }
    return h.value();
}

} // namespace g2m_building_test
