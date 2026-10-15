// tests/support/fixture_release.cpp - see fixture_release.h. Verbatim same
// pipeline as tests/layer/test_derive_terrain.cpp's own (now removed)
// private Fixture struct: import the DGM1 fixture into a source TileStore,
// build a local release over the built-in terrain derivers, and stand up a
// Server plus an in-process transport over it.

#include "fixture_release.h"

#include "g2m/import/dem.h"
#include "g2m/import/sink.h"

#include <catch2/catch_test_macros.hpp>

#include <utility>

namespace g2m::test_support {

namespace {

std::unique_ptr<TileStore> open_store(const std::filesystem::path& dir, const std::string& name) {
    TileStoreConfig c;
    c.dir = dir;
    c.name = name;
    Result<std::unique_ptr<TileStore>> s = TileStore::open(c);
    REQUIRE(s.ok());
    return std::move(*s);
}

// Same literal bbox as tests/layer/test_derive_terrain.cpp's
// resolve_fixture_region(): it overlaps exactly the DGM1 fixture's two
// 1 km squares 474/5560 and 475/5560 (testdata/fixtures/dgm1/PKT_fixture.zip).
Result<ResolvedRegion> resolve_fixture_region(const std::string& fixtures_dir, const std::string& scope) {
    const std::string pbf_path = fixtures_dir + "/osm/frankfurt_hoechst.osm.pbf";
    RegionSpec spec;
    spec.name = scope;
    spec.scheme = "utm32n";
    spec.halo_m = 0;
    UtmBboxPart bbox;
    bbox.west = 474000;
    bbox.south = 5560000;
    bbox.east = 476000;
    bbox.north = 5561000;
    spec.parts.push_back(bbox);
    return import::resolve_region(spec, pbf_path);
}

} // namespace

geo::SessionFrame ReleaseFixture::session_frame() const {
    return geo::SessionFrame(geo::UtmZone{32, geo::Hemisphere::North}, /*e0_m=*/474000, /*n0_m=*/5560000);
}

std::unique_ptr<ReleaseFixture> make_fixture_release(const std::filesystem::path& dir, const std::string& scope,
                                                     bool carved) {
    const std::string fixtures_dir = std::string(G2M_SOURCE_DIR) + "/testdata/fixtures";

    auto fx = std::make_unique<ReleaseFixture>(open_store(dir, "sources"), open_store(dir, "derived"));
    fx->scope = scope;

    Result<ResolvedRegion> r = resolve_fixture_region(fixtures_dir, scope);
    REQUIRE(r.ok());
    fx->region = *r;

    Result<decode::dgm1::Catalog> c = decode::dgm1::Catalog::open({fixtures_dir + "/dgm1/PKT_fixture.zip"});
    REQUIRE(c.ok());
    fx->catalog.emplace(std::move(c).value());

    import::StoreSink sink(*fx->sources, fx->scope, /*pin=*/true);
    Result<import::DemImportReport> report =
        import::import_dem(fx->region, *fx->catalog, sink, fx->scope, import::DemImportOptions{});
    REQUIRE(report.ok());
    REQUIRE(report->tiles_written > 0);

    std::vector<ReleaseSource> sources_attr = {
        ReleaseSource{"dgm1_fixture", "2026-09-26", "dl-de/zero-2-0", "test", false}};
    LocalReleaseParams params = builtin_local_release_params(fx->scope, fx->derivers, std::move(sources_attr), false, false, carved, carved);
    Result<ReleaseManifest> m = make_local_release(*fx->sources, params);
    REQUIRE(m.ok());
    fx->manifest = *m;
    fx->rid = fx->manifest.rid;

    ServerConfig config;
    config.server_id = "test-" + fx->scope;
    config.derived_store = fx->derived.get();
    config.derivers = fx->derivers.ordered(false, false, carved, carved);
    config.upstreams = {&fx->local_upstream};
    fx->server = std::make_unique<Server>(std::move(config));
    REQUIRE(fx->server->add_release(fx->manifest, fx->scope).ok());

    fx->transport = std::make_unique<InProcessTransport>(*fx->server);

    return fx;
}

} // namespace g2m::test_support
