// tests/golden/golden_g3f.cpp - G3f golden hashes (farm fields): g2m.fields.geom over the committed OSM fixtures
// (frankfurt_hoechst and hoechst_a66), derived the way the server does it (the tile's own g2m.src.osm clip plus the
// discovery stages), for every level-2 tile that holds a field or a decline.
//
// Per tile: one entry whose count is the encoded container size and whose hash is the FNV-1a 64 of the container
// bytes, and one entry whose count is the number of fields and whose hash covers every record (id, flags, area,
// row direction, anchor, vertex count of the outline). Per fixture one line with the totals (fields, owned fields,
// subdivided blocks, declines). One "all" line. testdata/golden/g3f_golden.txt; compared byte for byte between the
// Windows and Linux legs by tools/ci.ps1 (integer arithmetic only: the raster, the capsule tests and the
// Douglas-Peucker comparisons are exact).

#include "golden.h"

#include "road_fixture.h"

#include "g2m/core/geom/project.h"
#include "g2m/core/hash.h"
#include "g2m/layer/field_tile.h"
#include "g2m/layer/osm_clip.h"
#include "g2m/layer/src_osm.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <string>
#include <vector>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;

[[noreturn]] void fail(const std::string& what) {
    std::fprintf(stderr, "golden g3f: %s\n", what.c_str());
    std::abort();
}

g2m::InputTile source_tile(const g2m::OsmClipIndex& index, const g2m::InputTileRef& ref) {
    const g2m::OsmTile clipped{index.halo_m(), index.clip(ref.key)};
    const auto body = g2m::encode_src_osm(clipped);
    if (!body) fail("encode_src_osm: " + body.error().message);
    const auto bytes = g2m::encode_body(*body);
    if (!bytes) fail("encode_body: " + bytes.error().message);
    g2m::TileHeader header; header.layer = ref.layer; header.layer_version = 1; header.key = ref.key;
    header.body_size = bytes->size(); header.body_hash = g2m::content_hash(*bytes);
    return {ref, header, *bytes};
}

void fixture(std::vector<Entry>& out, const std::string& name, const std::string& file) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + file);
    const auto index = g2m::OsmClipIndex::build(data, g2m_test::kZone32N);
    const g2m::geo::Utm utm(g2m::geo::kGRS80);
    std::set<g2m::TileKey> keys;
    for (const auto& n : data.nodes) {
        const auto p = g2m::geom::project_e7_to_mm(utm, g2m_test::kZone32N, n.lat, n.lon);
        if (!p) continue;
        const auto k = g2m::tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(p->x) / 1000.0, static_cast<double>(p->y) / 1000.0);
        if (k) keys.insert(*k);
    }
    const g2m::FieldsGeomDeriver deriver;
    std::uint64_t fields = 0, owned = 0, subdivided = 0, declines = 0;
    for (const auto& key : keys) {
        std::vector<g2m::InputTile> inputs;
        for (const auto& ref : deriver.inputs_for(key)) inputs.push_back(source_tile(index, ref));
        for (std::size_t stage = 0; stage < deriver.discovery_stages(); ++stage) {
            const auto more = deriver.additional_inputs_for({key, 0, inputs});
            if (!more) fail(name + ": discovery: " + more.error().message);
            std::size_t added = 0;
            for (const auto& ref : *more) {
                if (std::any_of(inputs.begin(), inputs.end(), [&](const auto& in) { return in.ref == ref; })) continue;
                inputs.push_back(source_tile(index, ref)); ++added;
            }
            std::sort(inputs.begin(), inputs.end(), [](const auto& a, const auto& b) { return a.ref < b.ref; });
            if (added == 0) break;
        }
        const auto result = deriver.derive({key, 0, inputs});
        if (!result) fail(name + ": derive: " + result.error().message);
        const auto body = g2m::decode_body(result->body);
        if (!body) fail(name + ": body: " + body.error().message);
        const auto tile = g2m::decode_fields_geom(*body);
        if (!tile) fail(name + ": decode: " + tile.error().message);
        fields += tile->fields.size(); declines += tile->declines.size(); subdivided += tile->counts.subdivided;
        for (const auto& f : tile->fields) if ((f.flags & g2m::field_flag::Owned) != 0) ++owned;
        if (tile->fields.empty() && tile->declines.empty()) continue;
        const std::string prefix = "g3f." + name + ".tile." + std::to_string(key.x) + "." + std::to_string(key.y);
        out.push_back({prefix + ".container", result->body.size(), Fnv1a64().bytes(result->body.data(), result->body.size()).value()});
        Fnv1a64 records;
        for (const auto& f : tile->fields)
            records.u64(f.id).u32(f.flags).u64(f.area_mm2).u32(f.row_dir_cdeg).i64(f.anchor.x).i64(f.anchor.y)
                .u64(f.polygon.outer.size()).u64(f.polygon.holes.size()).u32(f.landuse).str(f.crop);
        for (const auto& d : tile->declines) records.u8(static_cast<std::uint8_t>(d.source)).i64(d.osm_id).u8(static_cast<std::uint8_t>(d.reason));
        out.push_back({prefix + ".records", tile->fields.size(), records.value()});
    }
    Fnv1a64 totals; totals.u64(fields).u64(owned).u64(subdivided).u64(declines);
    out.push_back({"g3f." + name + ".totals", fields, totals.value()});
}

} // namespace

std::vector<Entry> compute_g3f() {
    std::vector<Entry> out;
    fixture(out, "frankfurt_hoechst", "frankfurt_hoechst.osm.pbf");
    fixture(out, "hoechst_a66", "hoechst_a66.osm.pbf");
    Fnv1a64 all; std::uint64_t n = 0;
    for (const Entry& e : out) { all.str(e.name).u64(e.count).u64(e.hash); n += e.count; }
    out.push_back({"all", n, all.value()});
    return out;
}

} // namespace g2m_golden
