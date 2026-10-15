// tests/golden/golden_g3c.cpp - G3c golden hashes (street fidelity P2 slice 10, terrain migration phase 6): the surface
// classification g2m.terrain.class over the committed OSM fixtures (frankfurt_hoechst and hoechst_a66) on the tilted
// rough synthetic DEM of carve_fixture.h and the synthetic WorldCover-like land cover of synthetic.h
// (make_landcover_tile, SplitMix64 seeded by the level-2 key) - the four level-0 tiles of the g3d set plus one level-1
// tile (910, 10842) of hoechst_a66 from its 4 x 4 level-0 window (D49).
//
// Per tile: one entry per plane (land_class, surface, role, rank) whose count is the node count and whose hash is the
// FNV-1a 64 of the plane bytes, and one entry whose count is the road nodes (rank > 0) and whose hash covers the role
// histogram. One "all" line. testdata/golden/g3c_golden.txt; a separate set so g3d (carved heights) keeps its lines.
#include "golden.h"

#include "carve_fixture.h"
#include "synthetic.h"

#include "g2m/core/hash.h"
#include "g2m/layer/landcover_classifier.h"
#include "g2m/layer/terrain_class.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;

[[noreturn]] void fail(const std::string& what) {
    std::fprintf(stderr, "golden g3c: %s\n", what.c_str());
    std::abort();
}

std::unique_ptr<g2m::TerrainClassTile> classify(g2m_test::CarveFixture& fx, const g2m::TileKey& key) {
    std::vector<g2m::LandcoverTile> lc;
    for (const auto& k : g2m::terrain_class_landcover_keys(key)) lc.push_back(g2m_test::make_landcover_tile(k.packed(), k));
    const g2m::LandcoverBase base(lc, g2m::builtin_esa_worldcover_classifier());
    const auto w = fx.window(key);
    const auto r = fx.roads(key);
    auto out = std::make_unique<g2m::TerrainClassTile>();
    const auto ok = g2m::classify_terrain_tile(*out, key, w, r, base);
    if (!ok) fail("classify " + g2m::to_string(key) + ": " + ok.error().message);
    return out;
}

void emit(std::vector<Entry>& out, const std::string& prefix, const g2m::TerrainClassTile& t) {
    const std::pair<const char*, const std::vector<std::uint8_t>*> planes[] = {
        {"land_class", &t.land_class}, {"surface", &t.surface}, {"role", &t.role}, {"rank", &t.rank}};
    for (const auto& [name, plane] : planes) {
        Fnv1a64 h;
        for (const std::uint8_t v : *plane) h.u8(v);
        out.push_back({prefix + "." + name, plane->size(), h.value()});
    }
    std::array<std::uint64_t, 8> roles{};
    std::uint64_t road = 0;
    for (std::size_t i = 0; i < t.role.size(); ++i) {
        ++roles[t.role[i]];
        if (t.rank[i] > 0) ++road;
    }
    Fnv1a64 h;
    for (const std::uint64_t n : roles) h.u64(n);
    out.push_back({prefix + ".roles", road, h.value()});
}

void fixture(std::vector<Entry>& out, const std::string& name, const std::string& file,
             std::initializer_list<std::pair<std::int32_t, std::int32_t>> tiles, bool with_l1) {
    const g2m::RoadGraph world = g2m_test::fixture_world(file);
    g2m_test::CarveFixture fx(world, g2m_test::carve_tilted_dem);
    for (const auto& [x, y] : tiles) {
        const auto t = classify(fx, g2m::TileKey{g2m_test::kZone32N, 0, x, y});
        emit(out, "g3c." + name + ".tile." + std::to_string(x) + "." + std::to_string(y), *t);
    }
    if (!with_l1) return;
    constexpr std::int32_t px = 910, py = 10842;
    std::map<std::pair<std::int32_t, std::int32_t>, std::unique_ptr<g2m::TerrainClassTile>> l0s;
    std::array<const g2m::TerrainClassTile*, 16> window{};
    for (int dy = -1; dy <= 2; ++dy) {
        for (int dx = -1; dx <= 2; ++dx) {
            auto t = classify(fx, g2m::TileKey{g2m_test::kZone32N, 0, 2 * px + dx, 2 * py + dy});
            window[static_cast<std::size_t>((dy + 1) * 4 + (dx + 1))] = t.get();
            l0s[{dx, dy}] = std::move(t);
        }
    }
    g2m::TerrainClassTile l1;
    g2m::downsample_terrain_class(g2m::TileKey{g2m_test::kZone32N, 1, px, py}, window, l1);
    emit(out, "g3c." + name + ".l1." + std::to_string(px) + "." + std::to_string(py), l1);
}

} // namespace

std::vector<Entry> compute_g3c() {
    std::vector<Entry> out;
    fixture(out, "frankfurt_hoechst", "frankfurt_hoechst.osm.pbf", {{1856, 21718}, {1852, 21718}}, false);
    fixture(out, "hoechst_a66", "hoechst_a66.osm.pbf", {{1822, 21685}, {1820, 21684}}, true);
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
