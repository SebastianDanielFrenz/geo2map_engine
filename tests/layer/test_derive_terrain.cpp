// tests/layer/test_derive_terrain.cpp - PLAN.md G2.1: g2m.elev.base and
// g2m.terrain.height, level 0 only.
//
// Exercises the real import -> store -> local release -> Server path (same
// fixture-scale shape as tests/unit/test_import_e2e.cpp), then verifies
// every level-0 elev.base/terrain.height tile the fixture covers against an
// INDEPENDENT recomputation straight from g2m::decode::dgm1::Catalog (never
// reusing g2m::import::import_dem's own KmCache/assembly code, src/import/
// dem.cpp) - so this is a correctness check against the raw DGM1 data, not
// just a round-trip-through-the-same-pipeline self-consistency check.

#include "g2m/core/raster.h"
#include "g2m/core/tile_key.h"
#include "g2m/decode/dgm1.h"
#include "g2m/import/dem.h"
#include "g2m/import/region_resolve.h"
#include "g2m/import/sink.h"
#include "g2m/layer/derive_terrain.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/layer_registry.h"
#include "g2m/layer/region.h"
#include "g2m/layer/release.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

using namespace g2m;
using namespace g2m::import;
namespace dgm1 = g2m::decode::dgm1;

namespace {

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

// Same collision-avoidance shape as tests/unit/test_import_e2e.cpp's own
// TempDir (duplicated rather than shared - that one is private to
// tests/unit, this test lives in tests/layer).
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

// The fixture region (testdata/fixtures/regions.json's "fixture" entry
// covers more than we need here): a literal utm_bbox chosen, like
// test_import_dem.cpp's and test_import_e2e.cpp's own resolve_bbox_region,
// to overlap exactly the DGM1 fixture's two 1 km squares 474/5560 and
// 475/5560 (testdata/fixtures/dgm1/PKT_fixture.zip).
Result<ResolvedRegion> resolve_fixture_region() {
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    RegionSpec spec;
    spec.name = "g21_derive_terrain";
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

// Every level-0 descendant of every level-2 tile the resolved region
// touches (2 levels down: 4 x 4 = 16 children each) - PLAN.md G2.1 is level
// 0 only, so this is exactly the set g2m.elev.base/g2m.terrain.height
// should answer 200 for.
std::vector<TileKey> level0_targets(const ResolvedRegion& region) {
    std::vector<TileKey> out;
    for (const TileKey& t2 : tile_keys(region)) {
        for (int dx1 = 0; dx1 < 2; ++dx1) {
            for (int dy1 = 0; dy1 < 2; ++dy1) {
                const TileKey t1 = t2.child(dx1, dy1);
                for (int dx0 = 0; dx0 < 2; ++dx0) {
                    for (int dy0 = 0; dy0 < 2; ++dy0) out.push_back(t1.child(dx0, dy0));
                }
            }
        }
    }
    return out;
}

// A minimal, test-only re-decode of the DGM1 fixture, independent of
// src/import/dem.cpp's own KmCache: decodes each 1 km square through
// dgm1::Catalog at most once and remembers the result (or its absence).
class KmDecodedCache {
public:
    explicit KmDecodedCache(const dgm1::Catalog& catalog) : catalog_(catalog) {}

    // nullptr: no DGM1 coverage at all for this 1 km square (expect NoData).
    const std::vector<float>* get(int e_km, int n_km) {
        const auto key = std::make_pair(e_km, n_km);
        const auto it = cache_.find(key);
        if (it != cache_.end()) return it->second.empty() ? nullptr : &it->second;
        const std::vector<dgm1::CopyRef> refs = catalog_.copies(e_km, n_km);
        if (refs.empty()) {
            cache_.emplace(key, std::vector<float>{});
            return nullptr;
        }
        Result<std::vector<float>> decoded = catalog_.decode(refs.front());
        REQUIRE(decoded.ok());
        auto ins = cache_.emplace(key, std::move(decoded).value());
        return &ins.first->second;
    }

private:
    const dgm1::Catalog& catalog_;
    std::map<std::pair<int, int>, std::vector<float>> cache_;
};

// Independent expectation for level-0 sample (i, j) of `target` (row 0 =
// south, see height_tile.h's own doc comment: sample (i, j)'s centre sits
// at (target.min_easting() + i + 0.5, target.min_northing() + j + 0.5) at
// level 0). std::nullopt means NoData is expected. See dgm1.h's own header
// comment for the km square's Cell/NorthToSouth/1 m-spacing convention this
// mirrors by hand.
std::optional<std::int32_t> expected_sample(KmDecodedCache& km, const TileKey& target, int i, int j) {
    const std::int64_t easting_i = target.min_easting() + i;
    const std::int64_t northing_i = target.min_northing() + j;
    const int e_km = static_cast<int>(floor_div(easting_i, 1000));
    const int n_km = static_cast<int>(floor_div(northing_i, 1000));
    const std::vector<float>* square = km.get(e_km, n_km);
    if (!square) return std::nullopt;
    const int col = static_cast<int>(easting_i - static_cast<std::int64_t>(e_km) * 1000);
    const int row_from_south = static_cast<int>(northing_i - static_cast<std::int64_t>(n_km) * 1000);
    REQUIRE(col >= 0);
    REQUIRE(col < 1000);
    REQUIRE(row_from_south >= 0);
    REQUIRE(row_from_south < 1000);
    const int row_from_north = 999 - row_from_south;
    const float v = (*square)[static_cast<std::size_t>(row_from_north) * 1000 + static_cast<std::size_t>(col)];
    if (v == -9999.0f) return std::nullopt;
    return quantise_height_half_even(static_cast<double>(v));
}

// Every sample of `tile` against expected_sample(); exact equality plus the
// PLAN.md G2.1-mandated |h/256 - tif| <= 1/512 m tolerance restated in
// metres (redundant with exact equality here, since expected_sample()
// itself quantises the same way, but asked for explicitly).
void verify_tile_samples(KmDecodedCache& km, const HeightTile& tile) {
    for (int j = 0; j < kTerrainTileSamples; ++j) {
        for (int i = 0; i < kTerrainTileSamples; ++i) {
            const std::optional<std::int32_t> want = expected_sample(km, tile.key, i, j);
            const std::int32_t got =
                tile.h[static_cast<std::size_t>(j) * static_cast<std::size_t>(kTerrainTileSamples) +
                      static_cast<std::size_t>(i)];
            if (!want) {
                REQUIRE(got == kHeightNoData);
            } else {
                INFO("target level=" << tile.key.level << " x=" << tile.key.x << " y=" << tile.key.y
                                     << " easting=" << tile.key.min_easting() << " northing=" << tile.key.min_northing()
                                     << " i=" << i << " j=" << j);
                REQUIRE(got == *want);
                REQUIRE(std::fabs(dequantise_height(got) - dequantise_height(*want)) <= 1.0 / 512.0);
            }
        }
    }
}

// The whole import -> store -> local release -> Server fixture, shared by
// every TEST_CASE below (each constructs its own, same shape as
// tests/server/test_server.cpp's own World).
struct Fixture {
    TempDir dir{"derive_terrain"};
    std::unique_ptr<TileStore> sources = open_store(dir, "sources");
    std::unique_ptr<TileStore> derived = open_store(dir, "derived");
    // Raw g2m.src.elev.dgm1 tiles are served from `sources` through this
    // upstream (must outlive the Server, hence a member here) - the same
    // wiring tests/unit/test_import_e2e.cpp and tests/server/test_server.cpp
    // use for "raw tiles from the local source upstream". Without this the
    // Server has no store, deriver or upstream for a RAW layer at all, so
    // every ElevBaseDeriver::inputs_for() resolution 404s and every derived
    // elev.base tile comes back entirely NoData - found the hard way: with
    // this missing, 5 of 6 TEST_CASEs below still passed (almost every
    // sample in this fixture's region legitimately expects NoData, since
    // the fixture covers only 2 of the many 1 km squares the region's
    // level-2 tiling touches), and the 6th only failed once REQUIRE hit the
    // very first sample that expects real data.
    LocalSourceUpstream local_upstream{*sources};
    std::optional<dgm1::Catalog> catalog;
    ResolvedRegion region;
    BuiltinTerrainDerivers derivers;
    ReleaseManifest manifest;
    std::unique_ptr<Server> server;
    std::string scope = "g21";
    ReleaseId rid{};

    Fixture() {
        Result<ResolvedRegion> r = resolve_fixture_region();
        REQUIRE(r.ok());
        region = *r;

        Result<dgm1::Catalog> c = dgm1::Catalog::open({fixtures_dir() + "/dgm1/PKT_fixture.zip"});
        REQUIRE(c.ok());
        catalog.emplace(std::move(c).value());

        StoreSink sink(*sources, scope, /*pin=*/true);
        Result<DemImportReport> report = import_dem(region, *catalog, sink, scope, DemImportOptions{});
        REQUIRE(report.ok());
        REQUIRE(report->tiles_written > 0);

        std::vector<ReleaseSource> sources_attr = {
            ReleaseSource{"dgm1_fixture", "2026-09-26", "dl-de/zero-2-0", "test", false}};
        LocalReleaseParams params = builtin_local_release_params(scope, derivers, std::move(sources_attr));
        Result<ReleaseManifest> m = make_local_release(*sources, params);
        REQUIRE(m.ok());
        manifest = *m;
        rid = manifest.rid;

        ServerConfig config;
        config.server_id = "test-derive-terrain";
        config.derived_store = derived.get();
        config.derivers = derivers.ordered();
        config.upstreams = {&local_upstream};
        server = std::make_unique<Server>(config);
        REQUIRE(server->add_release(manifest, scope).ok());
    }
};

TileResponse fetch(Server& s, const ReleaseId& rid, const std::string& layer, const TileKey& key) {
    return s.tile(TileRequest{rid, layer, key, std::nullopt});
}

} // namespace

TEST_CASE("terrain: g2m.elev.base matches an independent DGM1 decode, sample for sample", "[layer][terrain]") {
    Fixture fx;
    KmDecodedCache km(*fx.catalog);

    const std::vector<TileKey> targets = level0_targets(fx.region);
    REQUIRE_FALSE(targets.empty());

    bool saw_nodata = false;
    for (const TileKey& target : targets) {
        const TileResponse resp = fetch(*fx.server, fx.rid, std::string(kElevBaseLayer), target);
        REQUIRE(resp.meta.status == Status::Ok);
        Result<ContainerView> parsed = parse_container(resp.container);
        REQUIRE(parsed.ok());
        Result<TileBody> body = decode_body(parsed->body);
        REQUIRE(body.ok());

        HeightTile tile;
        Result<void> decoded = decode_height_tile(*body, target, tile);
        REQUIRE(decoded.ok());
        verify_tile_samples(km, tile);
        saw_nodata = saw_nodata || tile.has_nodata;
    }
    // The fixture's own bbox was chosen to exactly match the two DGM1 km
    // squares (see resolve_fixture_region()'s comment), but level-2 tiling
    // (1024 m) does not divide evenly into that 2000 x 1000 m box, so at
    // least one level-0 tile straddling the outer edge is expected to carry
    // some NoData samples - failing this would mean the fixture/tiling
    // shape silently changed under this test.
    CHECK(saw_nodata);
}

TEST_CASE("terrain: g2m.terrain.height L0 is byte-identical to g2m.elev.base (body hash equality)",
         "[layer][terrain]") {
    Fixture fx;
    const std::vector<TileKey> targets = level0_targets(fx.region);
    REQUIRE_FALSE(targets.empty());

    for (const TileKey& target : targets) {
        const TileResponse elev_resp = fetch(*fx.server, fx.rid, std::string(kElevBaseLayer), target);
        const TileResponse height_resp = fetch(*fx.server, fx.rid, std::string(kTerrainHeightLayer), target);
        REQUIRE(elev_resp.meta.status == Status::Ok);
        REQUIRE(height_resp.meta.status == Status::Ok);
        REQUIRE(elev_resp.meta.etag.has_value());
        REQUIRE(height_resp.meta.etag.has_value());
        // PLAN.md decision 9: terrain.height's level 0 is a byte-identical
        // passthrough of elev.base, so the store's content-hash dedup
        // collapses the two tiles' bodies - checked here at the body-hash
        // (ETag) level, the observable proof of that dedup from a client's
        // point of view.
        CHECK(*elev_resp.meta.etag == *height_resp.meta.etag);

        Result<ContainerView> elev_view = parse_container(elev_resp.container);
        Result<ContainerView> height_view = parse_container(height_resp.container);
        REQUIRE(elev_view.ok());
        REQUIRE(height_view.ok());
        CHECK(std::vector<std::uint8_t>(elev_view->body.begin(), elev_view->body.end()) ==
              std::vector<std::uint8_t>(height_view->body.begin(), height_view->body.end()));
        CHECK(height_view->header.datum == elev_view->header.datum);
        CHECK(height_view->header.height_offset == elev_view->header.height_offset);
    }
}

TEST_CASE("terrain: Server::tile() 200 inside coverage, 404 well outside it (both layers)",
         "[layer][terrain][server]") {
    Fixture fx;
    const std::vector<TileKey> targets = level0_targets(fx.region);
    REQUIRE_FALSE(targets.empty());

    // In coverage: 200.
    CHECK(fetch(*fx.server, fx.rid, std::string(kElevBaseLayer), targets.front()).meta.status == Status::Ok);
    CHECK(fetch(*fx.server, fx.rid, std::string(kTerrainHeightLayer), targets.front()).meta.status == Status::Ok);

    // Well outside the fixture's own footprint (~1.28 million m away, same
    // zone): definitively 404, never a 500/503 - proves the coverage check
    // (not a deriver failure) is what answers here.
    TileKey outside = targets.front();
    outside.x += 5000;
    outside.y += 5000;
    const TileResponse elev_outside = fetch(*fx.server, fx.rid, std::string(kElevBaseLayer), outside);
    const TileResponse height_outside = fetch(*fx.server, fx.rid, std::string(kTerrainHeightLayer), outside);
    CHECK(elev_outside.meta.status == Status::NotFound);
    CHECK(height_outside.meta.status == Status::NotFound);

    // An unknown layer / unknown release: also 404, exercising the same
    // resolver entry point from the other definitive-check branches.
    CHECK(fetch(*fx.server, fx.rid, "g2m.does.not.exist", targets.front()).meta.status == Status::NotFound);
    ReleaseId unknown_rid{};
    unknown_rid[0] = 0xab;
    CHECK(fetch(*fx.server, unknown_rid, std::string(kElevBaseLayer), targets.front()).meta.status ==
         Status::NotFound);
}

TEST_CASE("terrain: ElevBaseDeriver's decode memo does not change the result (warm vs. always-cold)",
         "[layer][terrain]") {
    Fixture fx;
    const std::vector<TileKey> targets = level0_targets(fx.region);
    REQUIRE(targets.size() >= 2);

    // Fetch the raw g2m.src.elev.dgm1 input tiles directly from the source
    // store (bypassing the Server) so ElevBaseDeriver::derive() can be
    // called directly - a white-box comparison of "memo warm" (repeated
    // calls on ONE shared deriver instance, so its internal
    // DecodedElevMemo actually gets reused across sibling level-0 tiles)
    // against "memo always cold" (a FRESH ElevBaseDeriver, and therefore a
    // fresh, empty memo, constructed for every single call).
    ElevBaseDeriver warm;
    std::vector<std::vector<std::uint8_t>> warm_bodies;
    std::vector<std::vector<std::uint8_t>> cold_bodies;
    for (const TileKey& target : targets) {
        std::vector<InputTileRef> refs = warm.inputs_for(target);
        REQUIRE(refs.size() == 1);
        Result<std::optional<StoredTile>> stored = fx.sources->get(TileAddress{fx.scope, refs[0].layer, refs[0].key});
        REQUIRE(stored.ok());

        std::vector<InputTile> inputs;
        if (stored->has_value()) {
            InputTile in;
            in.ref = refs[0];
            in.header = (*stored)->header;
            in.body = (*stored)->body;
            inputs.push_back(std::move(in));
        }
        DeriveRequest req{target, fx.manifest.world_seed, inputs};

        Result<DeriveResult> warm_result = warm.derive(req);
        REQUIRE(warm_result.ok());
        warm_bodies.push_back(warm_result->body);

        ElevBaseDeriver cold; // fresh memo every call: never a cache hit
        Result<DeriveResult> cold_result = cold.derive(req);
        REQUIRE(cold_result.ok());
        cold_bodies.push_back(cold_result->body);
    }

    for (std::size_t i = 0; i < targets.size(); ++i) {
        CHECK(warm_bodies[i] == cold_bodies[i]);
    }
}

TEST_CASE("terrain: ElevBaseDeriver::derive is deterministic and thread-safe (1 vs. 8 workers)",
         "[layer][terrain]") {
    Fixture fx;
    const std::vector<TileKey> targets = level0_targets(fx.region);
    REQUIRE_FALSE(targets.empty());

    // Build every DeriveRequest's inputs up front (single-threaded, main
    // thread only) - only derive() itself is exercised concurrently below.
    std::vector<std::vector<InputTile>> all_inputs(targets.size());
    ElevBaseDeriver probe;
    for (std::size_t idx = 0; idx < targets.size(); ++idx) {
        std::vector<InputTileRef> refs = probe.inputs_for(targets[idx]);
        REQUIRE(refs.size() == 1);
        Result<std::optional<StoredTile>> stored = fx.sources->get(TileAddress{fx.scope, refs[0].layer, refs[0].key});
        REQUIRE(stored.ok());
        if (stored->has_value()) {
            InputTile in;
            in.ref = refs[0];
            in.header = (*stored)->header;
            in.body = (*stored)->body;
            all_inputs[idx].push_back(std::move(in));
        }
    }

    // Reference: one deriver instance, called sequentially on the main
    // thread (1 worker).
    ElevBaseDeriver reference_deriver;
    std::vector<std::vector<std::uint8_t>> reference(targets.size());
    for (std::size_t idx = 0; idx < targets.size(); ++idx) {
        DeriveRequest req{targets[idx], fx.manifest.world_seed, all_inputs[idx]};
        Result<DeriveResult> r = reference_deriver.derive(req);
        REQUIRE(r.ok());
        reference[idx] = r->body;
    }

    // 8 workers sharing ONE deriver instance (so its DecodedElevMemo is
    // genuinely exercised concurrently), each pulling the next index from a
    // shared atomic counter and writing ONLY into its own preallocated
    // slot - no vector<bool> (TOOL-030), and every Catch2 assertion stays
    // on the main thread (TOOL-031): workers only record Result::ok()/error
    // into plain data, asserted on after every thread has joined.
    ElevBaseDeriver shared_deriver;
    std::vector<std::vector<std::uint8_t>> parallel_bodies(targets.size());
    std::vector<std::uint8_t> parallel_ok(targets.size(), 0); // 1 = derive() returned ok, plain bytes not bits
    std::atomic<std::size_t> next{0};

    const auto worker = [&]() {
        for (;;) {
            const std::size_t idx = next.fetch_add(1);
            if (idx >= targets.size()) break;
            DeriveRequest req{targets[idx], fx.manifest.world_seed, all_inputs[idx]};
            Result<DeriveResult> r = shared_deriver.derive(req);
            if (r.ok()) {
                parallel_ok[idx] = 1;
                parallel_bodies[idx] = std::move(r).value().body;
            }
        }
    };

    constexpr int kWorkers = 8;
    std::vector<std::thread> threads;
    threads.reserve(kWorkers);
    for (int t = 0; t < kWorkers; ++t) threads.emplace_back(worker);
    for (std::thread& t : threads) t.join();

    for (std::size_t idx = 0; idx < targets.size(); ++idx) {
        REQUIRE(parallel_ok[idx] == 1);
        CHECK(parallel_bodies[idx] == reference[idx]);
    }
}

namespace {

// A deriver whose only input is its OWN output layer, at a strictly higher
// TileKey level every hop (never the same key twice, so this is not a
// literal no-op self-loop, just an unbounded self-referencing chain) -
// PLAN.md G2.1's risk item: does the resolver (or make_local_release)
// special-case a deriver reading its own layer name? Reading
// src/server/server.cpp's Server::Impl::resolve() shows the only guard is
// `kMaxDeriveDepth = 16`, a plain recursion-depth counter - no same-layer-
// name cycle detection anywhere. This deriver empirically confirms that:
// self-input is structurally ACCEPTED (not rejected up front), and an
// actually-cyclic/unbounded chain fails only once depth exceeds
// kMaxDeriveDepth, with Resolved::Kind::Error -> Status::Internal (500),
// never a stack overflow or a silent wrong answer. Directly relevant to
// PLAN.md G2.2, which plans a genuinely self-referencing mean-of-4 pyramid
// (terrain.height level N reading terrain.height level N-1) on top of this
// exact same resolver behaviour.
class SelfChainDeriver final : public IDeriver {
public:
    SelfChainDeriver() {
        desc_.id = LayerId{"test.selfchain", 1};
        desc_.kind = LayerKind::Baked;
        desc_.encoding = 0x8010;
        desc_.encoding_version = 1;
        for (int lvl = 0; lvl <= 18; ++lvl) desc_.levels |= level_bit(lvl);
        desc_.inputs = {"test.selfchain"};
        desc_.content = "test-only: unbounded self-referencing chain (PLAN.md G2.1 self-input finding)";
    }
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override { return ContentHash{}; }
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override {
        ++calls_inputs_for;
        return {InputTileRef{"test.selfchain", target.parent()}};
    }
    Result<DeriveResult> derive(const DeriveRequest&) const override {
        ++calls_derive;
        TileBody b;
        b.encoding = desc_.encoding;
        b.encoding_version = 1;
        Result<std::vector<std::uint8_t>> body = encode_body(b);
        if (!body) return body.error();
        DeriveResult r;
        r.body = std::move(body).value();
        return r;
    }
    mutable std::atomic<int> calls_inputs_for{0};
    mutable std::atomic<int> calls_derive{0};

private:
    LayerDesc desc_;
};

} // namespace

TEST_CASE("terrain: self-input (a deriver reading its own output layer) is accepted, bounded only by "
         "kMaxDeriveDepth",
         "[layer][terrain][server]") {
    TempDir dir("selfchain");
    std::unique_ptr<TileStore> derived = open_store(dir, "derived");

    SelfChainDeriver self_deriver;

    // Coverage set BY HAND (global), not computed via make_local_release:
    // that computation unions each input layer's ALREADY-COMPUTED coverage
    // (see g2m/server/builtin_release.h's own ordering comment), which for
    // a layer whose only input is itself would just be empty on the first
    // pass - a make_local_release limitation orthogonal to what this test
    // is checking (the resolver's own cycle behaviour, not coverage
    // bootstrapping). tests/server/test_server.cpp's own
    // World::add_network_layer() sets a ReleaseLayer's coverage by hand the
    // same way.
    ReleaseLayer layer;
    layer.desc = self_deriver.output();
    layer.config_hash = self_deriver.config_hash();
    layer.availability = Availability::OnDemand;
    layer.coverage.global = true;

    ReleaseManifest manifest;
    manifest.world_seed = 1;
    manifest.sources.push_back(ReleaseSource{"test", "2026-09-26", "none", "test", false});
    manifest.layers = {layer};

    ServerConfig config;
    config.server_id = "test-selfchain";
    config.derived_store = derived.get();
    config.derivers = {&self_deriver};
    Server server(config);
    REQUIRE(server.add_release(manifest, "selfchain_scope").ok());

    TileKey start;
    start.zone = geo::UtmZone{32, geo::Hemisphere::North};
    start.level = 0;
    start.x = 1000;
    start.y = 1000;
    REQUIRE(start.valid());

    const TileResponse resp = fetch(server, manifest.rid, "test.selfchain", start);
    CHECK(resp.meta.status == Status::Internal);
    CHECK(resp.meta.message.find("too deep") != std::string::npos);
    // kMaxDeriveDepth == 16 (server.cpp): depths 0..16 (17 calls) each ask
    // inputs_for() before recursing into their own input; depth 17 is
    // rejected BEFORE calling inputs_for() or derive() at all, and that
    // Error propagates straight back up through every enclosing depth
    // without any of them ever reaching their own derive() call (each
    // level's Derive step resolves its input first - depth-first - so a
    // chain that never bottoms out never computes a single body).
    CHECK(self_deriver.calls_inputs_for.load() == 17);
    CHECK(self_deriver.calls_derive.load() == 0);
}

TEST_CASE("terrain: Server resolve of an L6 g2m.terrain.height tile (depth 7: 6 same-layer hops plus one to "
         "g2m.elev.base) equals downsample_2x2 applied out of band to its own L5 children (PLAN.md G2.2)",
         "[layer][terrain][server]") {
    Fixture fx;
    const std::vector<TileKey> targets = level0_targets(fx.region);
    REQUIRE_FALSE(targets.empty());

    TileKey l6 = targets.front();
    while (l6.level < kTerrainMaxLevel) l6 = l6.parent();
    REQUIRE(l6.level == kTerrainMaxLevel);

    const TileResponse l6_resp = fetch(*fx.server, fx.rid, std::string(kTerrainHeightLayer), l6);
    REQUIRE(l6_resp.meta.status == Status::Ok);
    Result<ContainerView> l6_view = parse_container(l6_resp.container);
    REQUIRE(l6_view.ok());
    Result<TileBody> l6_body = decode_body(l6_view->body);
    REQUIRE(l6_body.ok());
    HeightTile l6_tile;
    REQUIRE(decode_height_tile(*l6_body, l6, l6_tile).ok());

    // Independently fetch each of the 4 same-layer L5 children (from the
    // SAME Server - already warm in cache after the L6 fetch above, since
    // every successful Derive step caches unconditionally, server.cpp) and
    // rebuild L6 from them by calling downsample_2x2 directly, out of band.
    // A child outside the fixture's own tiny footprint 404s - treated the
    // same way TerrainHeightDeriver::derive() itself treats a missing input
    // (nullptr, i.e. "this whole quadrant is NoData"), never as a test
    // failure: only at least one of the 4 is guaranteed covered (whichever
    // one Coverage::covers() used to answer the L6 fetch above Ok).
    // Heap-allocated: 4 stack-local HeightTiles (~1 MiB) plus this
    // function's other HeightTile locals is enough to blow a default 1 MiB
    // thread stack (STATUS_STACK_OVERFLOW, found the hard way - see
    // test_terrain_pyramid.cpp's own version of this comment).
    std::vector<HeightTile> children(4);
    const int dx[4] = {0, 1, 0, 1};
    const int dy[4] = {0, 0, 1, 1};
    const HeightTile* child_ptrs[4] = {nullptr, nullptr, nullptr, nullptr};
    for (int i = 0; i < 4; ++i) {
        const TileKey ck = l6.child(dx[i], dy[i]);
        const TileResponse resp = fetch(*fx.server, fx.rid, std::string(kTerrainHeightLayer), ck);
        if (resp.meta.status == Status::NotFound) continue;
        REQUIRE(resp.meta.status == Status::Ok);
        Result<ContainerView> view = parse_container(resp.container);
        REQUIRE(view.ok());
        Result<TileBody> body = decode_body(view->body);
        REQUIRE(body.ok());
        REQUIRE(decode_height_tile(*body, ck, children[i]).ok());
        child_ptrs[i] = &children[i];
    }
    REQUIRE((child_ptrs[0] || child_ptrs[1] || child_ptrs[2] || child_ptrs[3]));

    HeightTile rebuilt;
    rebuilt.key = l6;
    downsample_2x2(child_ptrs[0], child_ptrs[1], child_ptrs[2], child_ptrs[3], rebuilt);

    CHECK(rebuilt.has_nodata == l6_tile.has_nodata);
    CHECK(rebuilt.h == l6_tile.h);
}
