// tests/unit/test_osm_clip_index.cpp - g2m::OsmClipIndex (osm_clip.h, PLAN.md
// G1c I4) against clip_osm(), its own byte-for-byte oracle: a 6x6 tile
// neighbourhood at several halos on synthetic data, the same equivalence
// against the whole committed fixture PBF (testdata/fixtures/osm/
// frankfurt_hoechst.osm.pbf), and an 8-thread concurrent-clip check.

#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/import/region_resolve.h"
#include "g2m/layer/osm_clip.h"
#include "g2m/layer/region.h"
#include "g2m/layer/src_osm.h"

#include "synthetic.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

using namespace g2m;
using osm::OsmBuilder;
using osm::OsmData;

namespace {

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in.is_open());
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

RegionSpec find_region(const std::vector<RegionSpec>& specs, std::string_view name) {
    for (const auto& s : specs) {
        if (s.name == name) return s;
    }
    FAIL("region '" << name << "' not found in regions.json");
    return {};
}

// Encodes an OsmTile through the real g2m.src.osm pipeline down to raw
// bytes, matching what a store would actually persist - the "byte-for-byte"
// equivalence contract osm_clip.h's OsmClipIndex doc comment promises.
std::vector<std::uint8_t> encode_bytes(const OsmTile& tile) {
    const Result<TileBody> body = encode_src_osm(tile);
    REQUIRE(body.ok());
    const Result<std::vector<std::uint8_t>> bytes = encode_body(*body);
    REQUIRE(bytes.ok());
    return *bytes;
}

// index.clip(key), re-wrapped and encoded exactly as osm_clip.h's own doc
// comment specifies.
std::vector<std::uint8_t> encode_index_clip(const OsmClipIndex& index, const TileKey& key) {
    return encode_bytes(OsmTile{index.halo_m(), index.clip(key)});
}

std::vector<std::uint8_t> encode_oracle_clip(const OsmData& set, const TileKey& key, std::uint32_t halo_m) {
    const Result<OsmTile> oracle = clip_osm(set, key, halo_m);
    REQUIRE(oracle.ok());
    return encode_bytes(*oracle);
}

// Loads a whole .osm.pbf, unfiltered, into a canonical OsmData - the "S_full"
// this test's fixture-PBF equivalence check needs (no such helper is public:
// PLAN D20 confines libosmium's PBF *writer* to tests/support/fixture_cut,
// but the *reader* is g2m_decode's own public read_pbf, freely usable here).
class FullLoader final : public decode::osm::PbfHandler {
public:
    void on_node(const decode::osm::OsmNode& n) override {
        OsmBuilder::TagList tags;
        tags.reserve(n.tags.size());
        for (const auto& t : n.tags) tags.emplace_back(t.key, t.value);
        builder.add_node(n.id, n.lat_1e7, n.lon_1e7, std::move(tags));
    }
    void on_way(const decode::osm::OsmWay& w) override {
        OsmBuilder::TagList tags;
        tags.reserve(w.tags.size());
        for (const auto& t : w.tags) tags.emplace_back(t.key, t.value);
        builder.add_way(w.id, w.node_refs, std::move(tags));
    }
    void on_relation(const decode::osm::OsmRelation& r) override {
        std::vector<OsmBuilder::MemberIn> members;
        members.reserve(r.members.size());
        for (const auto& m : r.members) {
            osm::MemberType type = osm::MemberType::Node;
            if (m.type == decode::osm::OsmMemberType::Way) {
                type = osm::MemberType::Way;
            } else if (m.type == decode::osm::OsmMemberType::Relation) {
                type = osm::MemberType::Relation;
            }
            members.push_back(OsmBuilder::MemberIn{type, m.ref, m.role});
        }
        OsmBuilder::TagList tags;
        tags.reserve(r.tags.size());
        for (const auto& t : r.tags) tags.emplace_back(t.key, t.value);
        builder.add_relation(r.id, std::move(members), std::move(tags));
    }
    OsmBuilder builder;
};

} // namespace

TEST_CASE("OsmClipIndex: matches clip_osm over a 6x6 neighbourhood, several halos, synthetic data",
          "[unit][osm][layer]") {
    for (const std::uint64_t seed : {1u, 3u}) {
        const OsmData set = g2m_test::make_osm_set(seed, g2m_test::home_tile(), 500, 70, 10);
        REQUIRE(set.validate());
        for (const std::uint32_t halo_m : {0u, 128u, 1024u}) {
            const OsmClipIndex index = OsmClipIndex::build(set, g2m_test::kZone32N, halo_m);
            CHECK(index.halo_m() == halo_m);
            CHECK(index.zone() == g2m_test::kZone32N);
            for (std::int32_t dy = -2; dy <= 3; ++dy) {
                for (std::int32_t dx = -2; dx <= 3; ++dx) {
                    const TileKey key = g2m_test::home_tile(dx, dy);
                    const std::vector<std::uint8_t> want = encode_oracle_clip(set, key, halo_m);
                    const std::vector<std::uint8_t> got = encode_index_clip(index, key);
                    CHECK(got == want);
                }
            }
        }
    }
    // Also a tile far from every element: both sides agree it is empty.
    const OsmData set = g2m_test::make_osm_set(5, g2m_test::home_tile());
    const OsmClipIndex index = OsmClipIndex::build(set, g2m_test::kZone32N, kDefaultOsmHaloM);
    const TileKey far = g2m_test::home_tile(500, 500);
    CHECK(encode_index_clip(index, far) == encode_oracle_clip(set, far, kDefaultOsmHaloM));
}

TEST_CASE("OsmClipIndex: matches clip_osm over the whole committed fixture PBF", "[unit][osm][layer][io]") {
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";

    FullLoader loader;
    decode::osm::read_pbf(pbf_path, loader);
    const Result<OsmData> full = loader.builder.build();
    REQUIRE(full.ok());
    REQUIRE(full->nodes.size() > 0);

    // The fixture region's own level-2 footprint (test_import_resolve.cpp:
    // tile_count == 24) - a real, non-trivial set of query tiles actually
    // covering the fixture's content, not an arbitrarily chosen handful.
    const std::string regions_json = read_file(fixtures_dir() + "/regions.json");
    const Result<std::vector<RegionSpec>> specs = parse_regions(regions_json);
    REQUIRE(specs.ok());
    const RegionSpec spec = find_region(*specs, "fixture");
    const Result<ResolvedRegion> resolved = import::resolve_region(spec, pbf_path);
    REQUIRE(resolved.ok());
    const std::vector<TileKey> keys = tile_keys(*resolved);
    REQUIRE(keys.size() == 24);

    for (const std::uint32_t halo_m : {0u, 128u, 1024u}) {
        const OsmClipIndex index = OsmClipIndex::build(*full, g2m_test::kZone32N, halo_m);
        for (const TileKey& key : keys) {
            CHECK(encode_index_clip(index, key) == encode_oracle_clip(*full, key, halo_m));
        }
    }
}

TEST_CASE("OsmClipIndex: clip() is safe to call concurrently from 8 threads", "[unit][osm][layer]") {
    const OsmData set = g2m_test::make_osm_set(7, g2m_test::home_tile(), 400, 60, 8);
    const OsmClipIndex index = OsmClipIndex::build(set, g2m_test::kZone32N, kDefaultOsmHaloM);

    std::vector<TileKey> keys;
    for (std::int32_t dy = -2; dy <= 3; ++dy) {
        for (std::int32_t dx = -2; dx <= 3; ++dx) keys.push_back(g2m_test::home_tile(dx, dy));
    }
    std::vector<std::vector<std::uint8_t>> want(keys.size());
    for (std::size_t i = 0; i < keys.size(); ++i) want[i] = encode_oracle_clip(set, keys[i], kDefaultOsmHaloM);

    // vector<char>, never vector<bool> (TOOL-019: bit-packed vector<bool> is
    // not safe for concurrent writes to distinct elements).
    std::vector<char> agree(keys.size(), 0);
    constexpr int kThreads = 8;
    std::vector<std::thread> threads;
    threads.reserve(kThreads);
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&, t]() {
            for (std::size_t i = static_cast<std::size_t>(t); i < keys.size(); i += kThreads) {
                agree[i] = (encode_index_clip(index, keys[i]) == want[i]) ? 1 : 0;
            }
        });
    }
    for (std::thread& th : threads) th.join();
    for (std::size_t i = 0; i < keys.size(); ++i) CHECK(agree[i] == 1);
}
