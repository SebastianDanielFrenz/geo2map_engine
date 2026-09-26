// tests/unit/test_import_e2e.cpp - PLAN.md G1c I6 item E: the whole
// import -> store -> release -> server -> client path exercised together,
// not just each stage's own unit tests. Imports DEM against the committed
// DGM1 fixture (testdata/fixtures/dgm1/PKT_fixture.zip, same fixture-scale
// bbox as test_import_dem.cpp) into a pinned TileStore scope (StoreSink's
// pin flag, G1c I6 item E) as well as a plain MemorySink (the reference
// body to compare against), builds a local release over that store,
// serves it through an in-process Server, fetches one elev tile through
// the in-process transport, puts it into a client cache and reads it back
// from there ("the client's get"), and checks it decodes to the same
// bytes import_dem wrote into the reference MemorySink. Also checks every
// imported tile ends up pinned and that evicting under a tiny cap
// therefore leaves them all in place.

#include "g2m/client/cache.h"
#include "g2m/client/in_process_transport.h"
#include "g2m/core/tile_key.h"
#include "g2m/decode/dgm1.h"
#include "g2m/import/dem.h"
#include "g2m/import/region_resolve.h"
#include "g2m/import/sink.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/layer_registry.h"
#include "g2m/layer/region.h"
#include "g2m/layer/release.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/server.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <variant>
#include <vector>

using namespace g2m;
using namespace g2m::import;
namespace dgm1 = g2m::decode::dgm1;

namespace {

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

// A fresh directory under the system temp dir, unique per instance
// (steady-clock timestamp + a process-wide counter, so two TempDir
// instances - even across concurrently-running CTest processes - never
// collide; the same collision-avoidance shape tests/server/
// server_test_util.h's own TempDir uses, duplicated here rather than
// pulled into g2m_unit_tests from the g2m_server_tests-only helper
// header), removed (best effort) on destruction.
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

// Same bbox-only region helper as test_import_dem.cpp's resolve_bbox_region:
// resolved against the committed OSM fixture purely for its sha256 pass (a
// UtmBboxPart's own geometry does not depend on PBF content), sized to
// overlap testdata/fixtures/dgm1/PKT_fixture.zip's two 1 km squares
// (474/5560, 475/5560).
Result<ResolvedRegion> resolve_bbox_region() {
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    RegionSpec spec;
    spec.name = "e2e_fixture";
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

} // namespace

TEST_CASE("import e2e: pinned import -> local release -> server -> client cache", "[import][server][client]") {
    const std::string kElev(kSrcElevDgm1Layer);
    const std::string kScope = "e2e";

    Result<ResolvedRegion> resolved = resolve_bbox_region();
    REQUIRE(resolved.ok());
    Result<dgm1::Catalog> catalog = dgm1::Catalog::open({fixtures_dir() + "/dgm1/PKT_fixture.zip"});
    REQUIRE(catalog.ok());

    // Reference: the same import into a plain in-memory sink.
    MemorySink mem_sink;
    Result<DemImportReport> mem_report = import_dem(*resolved, *catalog, mem_sink, kScope, DemImportOptions{});
    REQUIRE(mem_report.ok());
    REQUIRE(mem_report->tiles_written > 0);

    // The same import again, into a real TileStore this time, pinned
    // (G1c I6 item E's StoreSink pin flag) - an importer building a
    // pinned, never-evicted local release.
    TempDir dir("import_e2e");
    std::unique_ptr<TileStore> sources = open_store(dir, "sources");
    std::unique_ptr<TileStore> derived = open_store(dir, "derived");
    {
        StoreSink store_sink(*sources, kScope, /*pin=*/true);
        Result<DemImportReport> store_report =
            import_dem(*resolved, *catalog, store_sink, kScope, DemImportOptions{});
        REQUIRE(store_report.ok());
        CHECK(store_report->tiles_written == mem_report->tiles_written);
    }

    REQUIRE(sources->check_consistency().ok());
    Result<TileStoreStats> stats = sources->stats();
    REQUIRE(stats.ok());
    CHECK(stats->tiles == mem_report->tiles_written);
    CHECK(stats->pinned_tiles == stats->tiles); // every imported tile is pinned

    // Build a local release over the pinned source store, serve it through
    // an in-process Server (raw elev tiles come from the local source
    // upstream, same as test_server.cpp's "raw tiles from the local source
    // upstream" case).
    LocalReleaseParams params;
    params.source_scope = kScope;
    params.sources = {ReleaseSource{"dgm1_fixture", "2026-09-26", "dl-de/zero-2-0", "test", false}};
    Result<ReleaseManifest> manifest = make_local_release(*sources, params);
    REQUIRE(manifest.ok());
    REQUIRE_FALSE(manifest->layers.empty());

    LocalSourceUpstream local(*sources);
    ServerConfig config;
    config.server_id = "test-e2e";
    config.contact = "nobody@example.invalid";
    config.derived_store = derived.get();
    config.upstreams = {&local};
    Server server(config);
    REQUIRE(server.add_release(*manifest, kScope).ok());
    const ReleaseId rid = manifest->rid;

    // Pick one elev tile the DEM import actually wrote, from the reference
    // sink (so it is guaranteed to exist and to have a known body).
    std::optional<TileKey> elev_key;
    std::vector<std::uint8_t> reference_body;
    for (const auto& [key, entry] : mem_sink.entries()) {
        if (key.first != kElev) continue;
        elev_key = key.second;
        reference_body = entry.body;
        break;
    }
    REQUIRE(elev_key.has_value());

    // Fetch it through the in-process transport (server -> local source
    // upstream), same request shape a real client would send.
    InProcessTransport transport(server);
    const Response response = transport.send(Request{TileRequest{rid, kElev, *elev_key, std::nullopt}});
    REQUIRE(std::holds_alternative<TileResponse>(response));
    const TileResponse& tile_response = std::get<TileResponse>(response);
    REQUIRE(tile_response.meta.status == Status::Ok);

    // "The client's get": the client puts what it fetched into its own
    // cache (PLAN.md 7.2, g2m::ClientCache) and reads it back from there,
    // rather than comparing the transport response bytes directly.
    TempDir client_dir("import_e2e_client");
    ClientCacheConfig cache_config;
    cache_config.dir = client_dir.path();
    Result<std::unique_ptr<ClientCache>> cache = ClientCache::open(cache_config);
    REQUIRE(cache.ok());
    REQUIRE((*cache)->put(tile_response.container).ok());
    Result<std::optional<std::vector<std::uint8_t>>> cached = (*cache)->get(rid, kElev, *elev_key);
    REQUIRE(cached.ok());
    REQUIRE(cached->has_value());

    Result<ContainerView> parsed = parse_container(**cached);
    REQUIRE(parsed.ok());
    CHECK(std::vector<std::uint8_t>(parsed->body.begin(), parsed->body.end()) == reference_body);

    // Eviction under a tiny cap: every source-store tile is pinned, so
    // nothing is evicted and every one is still there afterwards.
    Result<std::uint64_t> evicted = sources->evict_to(0);
    REQUIRE(evicted.ok());
    CHECK(*evicted == 0);
    Result<TileStoreStats> after = sources->stats();
    REQUIRE(after.ok());
    CHECK(after->tiles == stats->tiles);
    CHECK(after->evicted_tiles == 0);
    // The evicted tile is still there and still decodes.
    Result<std::optional<StoredTile>> still_there = sources->get(TileAddress{kScope, kElev, *elev_key});
    REQUIRE(still_there.ok());
    REQUIRE(still_there->has_value());
    CHECK((*still_there)->body == reference_body);
}
