// tests/golden/golden_g3rail.cpp - G3rail golden hashes (railways R1/R2): the g2m.rail.geom tiles of the committed OSM
// fixtures (frankfurt_hoechst and hoechst_a66) on the tilted rough synthetic DEM of carve_fixture.h. One entry per
// level-2 tile that owns a piece: count = pieces, hash = FNV-1a 64 of the encoded tile body bytes (so the ordering,
// the chain anchoring, the ground profile and the codec are all pinned), plus one "all" line.
// testdata/golden/g3rail_golden.txt.
#include "golden.h"

#include "carve_fixture.h"
#include "road_fixture.h"
#include "synthetic.h"

#include "g2m/core/hash.h"
#include "g2m/layer/rail_geom_tile.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <set>
#include <string>
#include <vector>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;

[[noreturn]] void fail(const std::string& what) {
    std::fprintf(stderr, "golden g3rail: %s\n", what.c_str());
    std::abort();
}

void fixture(std::vector<Entry>& out, const std::string& name, const std::string& file) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + file);
    const g2m::osm::OsmData* tiles[] = {&data};
    const auto& rules = g2m::builtin_rail_rules_de();
    const auto world = g2m::collect_rail_world(tiles, g2m_test::kZone32N, rules);
    if (!world) fail("collect " + name + ": " + world.error().message);
    std::set<g2m::TileKey> keys;
    for (const auto& [id, way] : world->ways) {
        for (const auto& p : way.pts) {
            const auto size = static_cast<std::int64_t>(1024000);
            keys.insert(g2m::TileKey{g2m_test::kZone32N, 2, static_cast<std::int32_t>(g2m::floor_div(p.x, size)), static_cast<std::int32_t>(g2m::floor_div(p.y, size))});
        }
    }
    for (const auto& key : keys) {
        if (plan_owned_pieces(*world, key, rules).empty()) continue;
        const auto deps = g2m::rail_dem_dependencies(*world, key, rules);
        if (!deps) fail("dependencies " + g2m::to_string(key) + ": " + deps.error().message);
        std::vector<std::unique_ptr<g2m::HeightTile>> dem;
        std::vector<const g2m::HeightTile*> views;
        for (const auto& ref : *deps) {
            dem.push_back(std::make_unique<g2m::HeightTile>()); dem.back()->key = ref.key;
            g2m_test::fill_dem(*dem.back(), g2m_test::carve_tilted_dem);
            views.push_back(dem.back().get());
        }
        const auto sampler = g2m::RoadDemSampler::make(views);
        if (!sampler) fail("sampler: " + sampler.error().message);
        const auto tile = g2m::build_rail_geom_tile(*world, key, rules, *sampler);
        if (!tile) fail("build " + g2m::to_string(key) + ": " + tile.error().message);
        const auto body = g2m::encode_rail_geom(*tile);
        if (!body) fail("encode: " + body.error().message);
        const auto bytes = g2m::encode_body(*body);
        if (!bytes) fail("encode body: " + bytes.error().message);
        Fnv1a64 h;
        h.bytes(std::span<const std::byte>(reinterpret_cast<const std::byte*>(bytes->data()), bytes->size()));
        out.push_back({"g3rail." + name + "." + std::to_string(key.x) + "." + std::to_string(key.y), tile->pieces.size(), h.value()});
    }
}

} // namespace

std::vector<Entry> compute_g3rail() {
    std::vector<Entry> out;
    fixture(out, "frankfurt_hoechst", "frankfurt_hoechst.osm.pbf");
    fixture(out, "hoechst_a66", "hoechst_a66.osm.pbf");
    Fnv1a64 all;
    std::uint64_t n = 0;
    for (const Entry& e : out) {
        all.str(e.name).u64(e.count).u64(e.hash);
        n += e.count;
    }
    out.push_back({"all", n, all.value()});
    return out;
}

} // namespace g2m_golden
