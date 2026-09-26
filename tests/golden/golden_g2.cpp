// tests/golden/golden_g2.cpp - G2 golden hashes: PLAN.md G2.1's terrain L0
// layers (g2m.elev.base, g2m.terrain.height) run through the real
// import -> store -> local release -> Server path (same fixture-scale
// shape as tests/layer/test_derive_terrain.cpp's own Fixture, duplicated
// here rather than shared - see golden_g1c.cpp's header comment for why
// this file family never reuses another test binary's helpers) over
// testdata/fixtures/dgm1's fixture DGM1 catalogue.
//
// Each entry hashes, per level-0 target tile in a fixed order, the
// Server's own response status plus (on 200) the tile's ETag - the
// store's content-addressed body hash (ContentHash = SHA-256, PLAN.md
// 3.4) - rather than re-hashing tile bytes here, so this golden also pins
// the ETag/body-hash computation itself, not just ElevBaseDeriver's
// output. compute_g2() returns "g2.g2m.elev.base", "g2.g2m.terrain.height"
// and a combined "all" entry, mirroring golden_g1c.cpp's own shape.

#include "golden.h"

#include "g2m/core/hash.h"
#include "g2m/core/tile_key.h"
#include "g2m/decode/dgm1.h"
#include "g2m/import/dem.h"
#include "g2m/import/region_resolve.h"
#include "g2m/import/sink.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/region.h"
#include "g2m/layer/release.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"

#include <array>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;
namespace dgm1 = g2m::decode::dgm1;

// Same shape as golden_g1c.cpp's own file-local Acc (deliberately not
// shared - see that file's header comment).
class Acc {
public:
    void bytes(std::span<const std::uint8_t> b) {
        h_.u64(b.size()).bytes(b.data(), b.size());
        ++n_;
    }
    void u(std::uint64_t v) {
        h_.u64(v);
        ++n_;
    }
    Entry done(std::string name) const { return {std::move(name), n_, h_.value()}; }

private:
    Fnv1a64 h_;
    std::uint64_t n_ = 0;
};

template <class T>
T must(g2m::Result<T> r, const char* what) {
    if (!r) {
        std::fprintf(stderr, "g2 golden: %s failed: %s\n", what, r.error().message.c_str());
        std::abort();
    }
    return std::move(r).value();
}

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

// Same collision-avoidance shape as tests/layer/test_derive_terrain.cpp's
// own TempDir (duplicated, not shared - see this file's header comment).
class TempDir {
public:
    explicit TempDir(const std::string& tag) {
        static std::atomic<std::uint64_t> counter{0};
        const auto now = static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        path_ = std::filesystem::temp_directory_path() /
                ("g2m_golden_" + tag + "_" + std::to_string(now) + "_" + std::to_string(counter.fetch_add(1)));
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

// Same literal bbox as tests/layer/test_derive_terrain.cpp's own
// resolve_fixture_region(): chosen to overlap exactly the DGM1 fixture's
// two 1 km squares 474/5560 and 475/5560
// (testdata/fixtures/dgm1/PKT_fixture.zip).
g2m::ResolvedRegion resolve_fixture_region() {
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    g2m::RegionSpec spec;
    spec.name = "g2_golden";
    spec.scheme = "utm32n";
    spec.halo_m = 0;
    g2m::UtmBboxPart bbox;
    bbox.west = 474000;
    bbox.south = 5560000;
    bbox.east = 476000;
    bbox.north = 5561000;
    spec.parts.push_back(bbox);
    return must(g2m::import::resolve_region(spec, pbf_path), "resolve_region");
}

// Every level-0 descendant of every level-2 tile the resolved region
// touches - same construction as
// tests/layer/test_derive_terrain.cpp's own level0_targets().
std::vector<g2m::TileKey> level0_targets(const g2m::ResolvedRegion& region) {
    std::vector<g2m::TileKey> out;
    for (const g2m::TileKey& t2 : g2m::tile_keys(region)) {
        for (int dx1 = 0; dx1 < 2; ++dx1) {
            for (int dy1 = 0; dy1 < 2; ++dy1) {
                const g2m::TileKey t1 = t2.child(dx1, dy1);
                for (int dx0 = 0; dx0 < 2; ++dx0) {
                    for (int dy0 = 0; dy0 < 2; ++dy0) out.push_back(t1.child(dx0, dy0));
                }
            }
        }
    }
    return out;
}

std::unique_ptr<g2m::TileStore> open_store(const TempDir& d, const std::string& name) {
    g2m::TileStoreConfig c;
    c.dir = d.path();
    c.name = name;
    return must(g2m::TileStore::open(c), "TileStore::open");
}

} // namespace

std::vector<Entry> compute_g2() {
    using namespace g2m;
    using namespace g2m::import;

    TempDir dir("g2");
    std::unique_ptr<TileStore> sources = open_store(dir, "sources");
    std::unique_ptr<TileStore> derived = open_store(dir, "derived");
    LocalSourceUpstream local_upstream(*sources);

    const ResolvedRegion region = resolve_fixture_region();
    const dgm1::Catalog catalog =
        must(dgm1::Catalog::open({fixtures_dir() + "/dgm1/PKT_fixture.zip"}), "dgm1::Catalog::open");

    const std::string scope = "g2_golden";
    StoreSink sink(*sources, scope, /*pin=*/true);
    const DemImportReport dreport = must(import_dem(region, catalog, sink, scope, DemImportOptions{}), "import_dem");
    if (dreport.tiles_written == 0) {
        std::fprintf(stderr, "g2 golden: import_dem wrote 0 tiles\n");
        std::abort();
    }

    BuiltinTerrainDerivers derivers;
    std::vector<ReleaseSource> sources_attr = {
        ReleaseSource{"dgm1_fixture", "2026-09-26", "dl-de/zero-2-0", "test", false}};
    LocalReleaseParams params = builtin_local_release_params(scope, derivers, std::move(sources_attr));
    const ReleaseManifest manifest = must(make_local_release(*sources, params), "make_local_release");

    ServerConfig config;
    config.server_id = "golden-g2";
    config.derived_store = derived.get();
    config.derivers = derivers.ordered();
    config.upstreams = {&local_upstream};
    Server server(config);
    if (!server.add_release(manifest, scope).ok()) {
        std::fprintf(stderr, "g2 golden: add_release failed\n");
        std::abort();
    }

    const std::vector<TileKey> targets = level0_targets(region);
    if (targets.empty()) {
        std::fprintf(stderr, "g2 golden: no level-0 targets\n");
        std::abort();
    }

    std::vector<Entry> out;
    for (std::string_view layer : {kElevBaseLayer, kTerrainHeightLayer}) {
        Acc a;
        for (const TileKey& target : targets) {
            const TileResponse resp = server.tile(TileRequest{manifest.rid, std::string(layer), target, std::nullopt});
            a.u(static_cast<std::uint64_t>(resp.meta.status));
            if (resp.meta.etag) {
                a.bytes(std::span<const std::uint8_t>(resp.meta.etag->data(), resp.meta.etag->size()));
            } else {
                a.u(0);
            }
        }
        out.push_back(a.done("g2." + std::string(layer)));
    }

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
