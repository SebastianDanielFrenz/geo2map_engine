// tests/support/fixture_release.h - factors the import -> store -> local
// release -> Server pipeline out of tests/layer/test_derive_terrain.cpp's
// own private Fixture (PLAN.md R2.2 section 1c), so the bridge tests
// (bridges/physics_sim/tests, G7+) can stand up the exact same fixture
// release without duplicating the DGM1-fixture-import/Server wiring.
//
// The fixture's data footprint is fixed, matching
// tests/layer/test_derive_terrain.cpp's own resolve_fixture_region(): PKT
// DGM1 easting 474000-476000, northing 5560000-5561000 (UTM zone 32N,
// testdata/fixtures/dgm1/PKT_fixture.zip's own two 1 km squares). The
// physics bridge's own session frame for this same footprint is
// e0=474000, n0=5560000 (session_frame() below).
#pragma once

#include "g2m/client/in_process_transport.h"
#include "g2m/core/geo/session_frame.h"
#include "g2m/decode/dgm1.h"
#include "g2m/import/region_resolve.h"
#include "g2m/layer/region.h"
#include "g2m/layer/release.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace g2m::test_support {

// Owns every piece the returned Server/InProcessTransport need kept alive
// (the two TileStores, the local-source upstream, the decoded DGM1
// catalog, the built-in terrain derivers). Declared in construction order
// so the implicit destructor tears them down in reverse - transport and
// server first, the stores they read from last.
struct ReleaseFixture {
    std::unique_ptr<TileStore> sources;
    std::unique_ptr<TileStore> derived;
    LocalSourceUpstream local_upstream;
    std::optional<decode::dgm1::Catalog> catalog;
    ResolvedRegion region;
    BuiltinTerrainDerivers derivers;
    ReleaseManifest manifest;
    std::unique_ptr<Server> server;
    std::unique_ptr<InProcessTransport> transport;
    std::string scope;
    ReleaseId rid{};

    ReleaseFixture(std::unique_ptr<TileStore> sources_store, std::unique_ptr<TileStore> derived_store)
        : sources(std::move(sources_store)), derived(std::move(derived_store)), local_upstream(*sources) {}

    // This fixture's own session frame (PLAN.md R2.2 section 1c):
    // e0=474000, n0=5560000, zone 32N - the same footprint
    // resolve_fixture_region() resolves in tests/layer/test_derive_terrain.cpp.
    geo::SessionFrame session_frame() const;
};

// Builds the fixture's stores inside `dir`, which must already exist - the
// caller owns `dir`'s lifetime/cleanup (e.g. its own TempDir), this
// function never removes it. `scope` names the raw tiles in the source
// store (LocalSourceUpstream) and the local release's source_scope.
//
// Any pipeline failure aborts the current Catch2 test via REQUIRE, exactly
// as tests/layer/test_derive_terrain.cpp's own Fixture did before this was
// factored out - callers must invoke this from inside a running TEST_CASE.
std::unique_ptr<ReleaseFixture> make_fixture_release(const std::filesystem::path& dir,
                                                     const std::string& scope = "fixture_release");

} // namespace g2m::test_support
