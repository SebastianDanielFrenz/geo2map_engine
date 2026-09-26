// tests/unit/test_bake.cpp - PLAN.md G2.2: g2m::import::bake() is
// deterministic regardless of --jobs. Imports the committed DGM1 fixture
// (same fixture-scale bbox as test_import_e2e.cpp/test_derive_terrain.cpp)
// into a `sources` store, bakes g2m.terrain.height levels 0-6 into two
// separate `derived` stores - once with jobs=1, once with jobs=8 - and
// checks both derived stores' index-hash (same per-layer FNV-1a64-over-
// sorted-(key,body_hash) algorithm g2m_tiler's own `index-hash` command
// uses, duplicated here rather than shared: that logic is a small static
// function private to apps/g2m_tiler/main.cpp) comes out byte-for-byte
// identical.

#include "g2m/core/hash.h"
#include "g2m/core/tile_key.h"
#include "g2m/decode/dgm1.h"
#include "g2m/import/bake.h"
#include "g2m/import/dem.h"
#include "g2m/import/region_resolve.h"
#include "g2m/import/sink.h"
#include "g2m/server/tile_store.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

using namespace g2m;
using namespace g2m::import;
namespace dgm1 = g2m::decode::dgm1;

namespace {

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

// Same collision-avoidance TempDir shape as test_import_e2e.cpp/
// test_derive_terrain.cpp's own (each duplicated privately - see those
// files' own comments for why this isn't a shared header).
class TempDir {
public:
    explicit TempDir(const std::string& tag) {
        static std::atomic<std::uint64_t> counter{0};
        const auto now = static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        path_ = std::filesystem::temp_directory_path() /
                ("g2m_test_" + tag + "_" + std::to_string(now) + "_" + std::to_string(counter.fetch_add(1)));
        std::filesystem::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

Result<ResolvedRegion> resolve_fixture_region() {
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    RegionSpec spec;
    spec.name = "g22_bake";
    spec.scheme = "utm32n";
    spec.halo_m = 0;
    UtmBboxPart bbox;
    bbox.west = 474000;
    bbox.south = 5560000;
    bbox.east = 476000;
    bbox.north = 5561000;
    spec.parts.push_back(bbox);
    return resolve_region(spec, pbf_path);
}

std::unique_ptr<TileStore> open_store(const TempDir& d, const std::string& name) {
    TileStoreConfig c;
    c.dir = d.path();
    c.name = name;
    Result<std::unique_ptr<TileStore>> s = TileStore::open(c);
    REQUIRE(s.ok());
    return std::move(*s);
}

// Per-layer + overall FNV-1a64 over a store's (scope) index, in the index's
// own (layer, key) sort order - the exact algorithm cmd_index_hash (apps/
// g2m_tiler/main.cpp) uses, duplicated here (a test binary, not linking
// apps/g2m_tiler's own .cpp).
struct IndexHash {
    std::uint64_t all = 0;
    std::map<std::string, std::uint64_t> per_layer;
};

Result<IndexHash> index_hash(TileStore& store, const std::string& scope) {
    Result<std::vector<TileIndexEntry>> index = store.index(scope);
    if (!index) return index.error();

    IndexHash out;
    Fnv1a64 all_hasher;
    std::string current_layer;
    bool have_layer = false;
    Fnv1a64 layer_hasher;
    auto flush_layer = [&]() { out.per_layer[current_layer] = layer_hasher.value(); };
    for (const auto& entry : *index) {
        if (!have_layer || entry.layer != current_layer) {
            if (have_layer) flush_layer();
            current_layer = entry.layer;
            have_layer = true;
            layer_hasher = Fnv1a64{};
        }
        layer_hasher.u64(entry.key.packed());
        layer_hasher.bytes(entry.body_hash.data(), entry.body_hash.size());
        all_hasher.u64(entry.key.packed());
        all_hasher.bytes(entry.body_hash.data(), entry.body_hash.size());
    }
    if (have_layer) flush_layer();
    out.all = all_hasher.value();
    return out;
}

} // namespace

TEST_CASE("bake: --jobs 1 and --jobs 8 produce byte-identical derived stores (index-hash equality)",
         "[import][bake]") {
    TempDir dir("bake_jobs");
    std::unique_ptr<TileStore> sources = open_store(dir, "sources");
    const std::string scope = "g22_bake_fixture";

    Result<ResolvedRegion> region = resolve_fixture_region();
    REQUIRE(region.ok());
    Result<dgm1::Catalog> catalog = dgm1::Catalog::open({fixtures_dir() + "/dgm1/PKT_fixture.zip"});
    REQUIRE(catalog.ok());

    StoreSink sink(*sources, scope, /*pin=*/true);
    Result<DemImportReport> imported = import_dem(*region, *catalog, sink, scope, DemImportOptions{});
    REQUIRE(imported.ok());
    REQUIRE(imported->tiles_written > 0);

    std::unique_ptr<TileStore> derived_j1 = open_store(dir, "derived_j1");
    std::unique_ptr<TileStore> derived_j8 = open_store(dir, "derived_j8");

    BakeOptions options;
    options.layers = {"g2m.terrain.height"};
    options.min_level = 0;
    options.max_level = 6;

    options.jobs = 1;
    Result<BakeReport> report_j1 = bake(*sources, *derived_j1, scope, options);
    REQUIRE(report_j1.ok());

    options.jobs = 8;
    Result<BakeReport> report_j8 = bake(*sources, *derived_j8, scope, options);
    REQUIRE(report_j8.ok());

    // Same `sources` snapshot -> the same local release, so both bakes must
    // address `derived` under the same release id.
    REQUIRE_FALSE(report_j1->release_id_hex.empty());
    CHECK(report_j1->release_id_hex == report_j8->release_id_hex);

    REQUIRE(report_j1->layers.size() == 1);
    REQUIRE(report_j8->layers.size() == 1);
    CHECK(report_j1->layers[0].tiles_total > 0);
    CHECK(report_j1->layers[0].tiles_total == report_j8->layers[0].tiles_total);
    CHECK(report_j1->layers[0].container_bytes_total == report_j8->layers[0].container_bytes_total);
    REQUIRE(report_j1->layers[0].levels.size() == report_j8->layers[0].levels.size());
    for (std::size_t i = 0; i < report_j1->layers[0].levels.size(); ++i) {
        CAPTURE(i);
        CHECK(report_j1->layers[0].levels[i].level == report_j8->layers[0].levels[i].level);
        CHECK(report_j1->layers[0].levels[i].tiles == report_j8->layers[0].levels[i].tiles);
        CHECK(report_j1->layers[0].levels[i].container_bytes == report_j8->layers[0].levels[i].container_bytes);
    }

    Result<IndexHash> hash_j1 = index_hash(*derived_j1, report_j1->release_id_hex);
    Result<IndexHash> hash_j8 = index_hash(*derived_j8, report_j8->release_id_hex);
    REQUIRE(hash_j1.ok());
    REQUIRE(hash_j8.ok());
    REQUIRE_FALSE(hash_j1->per_layer.empty());
    CHECK(hash_j1->all == hash_j8->all);
    CHECK(hash_j1->per_layer == hash_j8->per_layer);
}
