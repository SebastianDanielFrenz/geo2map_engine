// tests/golden/golden_g3j.cpp - G3j golden hashes (street fidelity P2 slice 8, design section 11 "Goldens"): the
// junction, furniture and provenance sections of roads.geom encoding 7 over the committed OSM fixtures
// (frankfurt_hoechst and hoechst_a66), built as served (golden_g3xs.cpp's tiles, flat synthetic DEM).
//
// Per fixture and tile: one entry per section (5 junctions, 6 furniture, 7 provenance) whose count is the record count
// and whose hash is the FNV-1a 64 of the section's raw (pre-Zstd) bytes; per fixture the totals and a record-level hash
// over every owned junction (owner, flags, decline, node count, approach count, outer ring vertex count) and every
// furniture record (kind, decline, node, junction owner, station in mm, travel, t in mm); one "all" line.
// testdata/golden/g3j_golden.txt; compared byte for byte between the Windows and Linux legs by tools/ci.ps1.

#include "golden.h"

#include "road_fixture.h"

#include "g2m/core/hash.h"
#include "g2m/core/tile_key.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <string>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;

[[noreturn]] void fail(const std::string& what) {
    std::fprintf(stderr, "golden g3j: %s\n", what.c_str());
    std::abort();
}

void fixture(std::vector<Entry>& out, const std::string& name, const std::string& file) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + file);
    auto world = g2m::build_road_graph(data, g2m_test::kZone32N);
    if (!world) fail(name + ": graph: " + world.error().message);
    std::set<g2m::TileKey> keys;
    for (const auto& node : world->nodes) {
        const auto key = g2m::tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(node.position.x) / 1000.0, static_cast<double>(node.position.y) / 1000.0);
        if (key) keys.insert(*key);
    }
    Fnv1a64 junction_hash, furniture_hash; std::uint64_t junctions = 0, furniture = 0, completed = 0, entries = 0;
    for (const auto& key : keys) {
        const auto probe = g2m::select_road_graph_tile(*world, key);
        if (!probe || probe->owned_edges.empty()) continue;
        const auto tile = g2m_test::served_geometry_in_tile(*world, key);
        if (!tile) fail(name + ": geometry: " + tile.error().message);
        const auto body = g2m::encode_road_geom(*tile);
        if (!body) fail(name + ": encode: " + body.error().message);
        if (body->sections.size() != 7) fail(name + ": not an encoding-7 body");
        const std::string prefix = "g3j." + name + ".tile." + std::to_string(key.x) + "." + std::to_string(key.y);
        const std::uint64_t counts[3] = {tile->junctions.size(), tile->furniture.size(), tile->origins.size()};
        const char* kinds[3] = {".junctions", ".furniture", ".provenance"};
        for (std::size_t s = 0; s < 3; ++s) {
            const auto& bytes = body->sections[4 + s].data;
            out.push_back({prefix + kinds[s], counts[s], Fnv1a64().bytes(bytes.data(), bytes.size()).value()});
        }
        for (const auto& j : tile->junctions) {
            junction_hash.i64(j.nodes.front()).u16(j.flags).u16(static_cast<std::uint16_t>(j.decline)).u64(j.nodes.size()).u64(j.approaches.size())
                .u64(j.rings[2].vertices.size());
            ++junctions;
        }
        for (const auto& f : tile->furniture) {
            const std::int64_t owner = f.junction_index ? tile->junctions[f.junction_index - 1].nodes.front() : 0;
            furniture_hash.u8(static_cast<std::uint8_t>(f.kind)).u16(static_cast<std::uint16_t>(f.decline)).i64(f.node_id).i64(owner)
                .i64(static_cast<std::int64_t>(std::llround(f.station_m * 1000.0))).u8(static_cast<std::uint8_t>(f.travel))
                .i64(static_cast<std::int64_t>(std::llround(f.t_from * 1000.0))).i64(static_cast<std::int64_t>(std::llround(f.t_to * 1000.0)));
            ++furniture;
        }
        for (const auto& o : tile->origins) { ++entries; if (o.provenance != g2m::ProfileProvenance::Strict) ++completed; }
    }
    out.push_back({"g3j." + name + ".junctions", junctions, junction_hash.value()});
    out.push_back({"g3j." + name + ".furniture", furniture, furniture_hash.value()});
    out.push_back({"g3j." + name + ".completed_entries", completed, entries});
}

} // namespace

std::vector<Entry> compute_g3j() {
    std::vector<Entry> out;
    fixture(out, "frankfurt_hoechst", "frankfurt_hoechst.osm.pbf");
    fixture(out, "hoechst_a66", "hoechst_a66.osm.pbf");
    Fnv1a64 all; std::uint64_t n = 0;
    for (const Entry& e : out) { all.str(e.name).u64(e.count).u64(e.hash); n += e.count; }
    out.push_back({"all", n, all.value()});
    return out;
}

} // namespace g2m_golden
