// tests/golden/golden_g3xs.cpp - G3xs golden hashes (street fidelity P1, design 7.5): roads.geom
// encoding 6 over the committed OSM fixtures (frankfurt_hoechst and hoechst_a66), derived exactly the
// way RoadGeomDeriver does it (tile graph plus the stretch context of the tiles that hold every
// incident way) with a synthetic flat DEM - profiles need a DEM, cross-sections do not.
//
// Per fixture: the level-2 tile list, the FNV-1a 64 of every tile's encoded container body (the design
// asks for SHA-256; the golden file's own entry format is an FNV count/hash pair, and the dump is
// compared byte for byte between the Windows and Linux legs, which is the requirement), and one hash
// over every stretch line: way id, start/end ref, band count, join kinds and the boundary t values at
// s = 0, L/4, L/2, 3L/4, L in units of 0.1 mm (llround(t * 1e4)). testdata/golden/g3xs_golden.txt.

#include "golden.h"

#include "road_fixture.h"

#include "g2m/core/hash.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/tile_container.h"

#include <array>
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
    std::fprintf(stderr, "golden g3xs: %s\n", what.c_str());
    std::abort();
}

void fixture(std::vector<Entry>& out, const std::string& name, const std::string& file) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + file);
    auto world = g2m::build_road_graph(data, g2m_test::kZone32N);
    if (!world) fail(name + ": graph: " + world.error().message);
    // Every level-2 tile that owns an edge: the tile of each node, kept when select_road_graph_tile finds owned edges.
    std::set<g2m::TileKey> keys;
    for (const auto& node : world->nodes) {
        const auto key = g2m::tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(node.position.x) / 1000.0, static_cast<double>(node.position.y) / 1000.0);
        if (key) keys.insert(*key);
    }
    Fnv1a64 tiles_hash; std::uint64_t tiles = 0;
    Fnv1a64 stretch_hash; std::uint64_t stretches = 0;
    for (const auto& key : keys) {
        const auto probe = g2m::select_road_graph_tile(*world, key);
        if (!probe || probe->owned_edges.empty()) continue;
        const auto geometry = g2m_test::geometry_in_tile(*world, key);
        if (!geometry) fail(name + ": geometry: " + geometry.error().message);
        const auto body = g2m::encode_road_geom(*geometry);
        if (!body) fail(name + ": encode: " + body.error().message);
        const auto bytes = g2m::encode_body(*body);
        if (!bytes) fail(name + ": body: " + bytes.error().message);
        Entry e; e.name = "g3xs." + name + ".tile." + std::to_string(key.x) + "." + std::to_string(key.y);
        e.count = bytes->size();
        e.hash = Fnv1a64().bytes(bytes->data(), bytes->size()).value();
        out.push_back(std::move(e));
        tiles_hash.i32(key.x).i32(key.y); ++tiles;
        for (const auto& entry : geometry->entries) {
            stretch_hash.i64(entry.way_id).u32(entry.stretch.start_ref).u32(entry.stretch.end_ref);
            ++stretches;
            if (!entry.cross_section) {
                // Absent (declined profile) or a cross-section decline: record which.
                stretch_hash.u8(entry.cross_section_decline ? 2 : 0);
                if (entry.cross_section_decline) stretch_hash.u16(static_cast<std::uint16_t>(entry.cross_section_decline->code));
                continue;
            }
            const auto& xs = *entry.cross_section;
            stretch_hash.u8(1).u64(xs.bands.size()).u8(static_cast<std::uint8_t>(xs.start_join.kind)).u8(static_cast<std::uint8_t>(xs.end_join.kind));
            std::array<double, 33> t{};
            const std::span<double> span(t.data(), xs.bands.size() + 1);
            for (int i = 0; i <= 4; ++i) {
                if (!xs.boundaries_at(xs.length_m * static_cast<double>(i) / 4.0, span)) fail(name + ": boundaries_at");
                for (const double v : span) stretch_hash.i64(static_cast<std::int64_t>(std::llround(v * 1e4)));
            }
        }
    }
    out.push_back({"g3xs." + name + ".tiles", tiles, tiles_hash.value()});
    out.push_back({"g3xs." + name + ".stretches", stretches, stretch_hash.value()});
}

} // namespace

std::vector<Entry> compute_g3xs() {
    std::vector<Entry> out;
    fixture(out, "frankfurt_hoechst", "frankfurt_hoechst.osm.pbf");
    fixture(out, "hoechst_a66", "hoechst_a66.osm.pbf");
    Fnv1a64 all; std::uint64_t n = 0;
    for (const Entry& e : out) { all.str(e.name).u64(e.count).u64(e.hash); n += e.count; }
    out.push_back({"all", n, all.value()});
    return out;
}

} // namespace g2m_golden
