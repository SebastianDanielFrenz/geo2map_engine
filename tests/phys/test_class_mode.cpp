// tests/phys/test_class_mode.cpp - class mode of the physics path (terrain
// migration phase 6, requests G-2/G-5): fill_physics_classes (cell-centre
// placement over a 2 x 2 L0 straddle, the Absent/Missing/no-class outcome
// table, zero allocation), ResidentHeightSet's class tile beside the road
// metadata, the loader's require_classes rule and the streamer install, and
// TransportHeightTileFetch's class layer.
#include "g2m/phys/fill_physics_classes.h"
#include "g2m/phys/height_tile_loader.h"
#include "g2m/phys/physics_streamer.h"
#include "g2m/phys/resident_heights.h"

#include "alloc_counter.h"
#include "g2m/core/geo/session_frame.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/terrain_class.h"
#include "g2m/layer/tile_container.h"
#include "g2m/world/surface_id_table.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <variant>
#include <vector>

using g2m::LandClass;
using g2m::RoadSegment;
using g2m::Status;
using g2m::SurfaceKind;
using g2m::TerrainClassTile;
using g2m::TileKey;
using g2m::classes::SurfaceIdTable;
using g2m::geo::Hemisphere;
using g2m::geo::SessionFrame;
using g2m::geo::UtmZone;
using g2m::phys::ClassFillStats;
using g2m::phys::FetchResult;
using g2m::phys::fill_physics_classes;
using g2m::phys::HeightTileLoader;
using g2m::phys::LoadedHeightTile;
using g2m::phys::LoaderConfig;
using g2m::phys::LoadOutcome;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;
using g2m::phys::ResidentHeightSet;
using namespace std::chrono_literals;

namespace {
constexpr UtmZone k32N{32, Hemisphere::North};

TileKey l0_at(std::int64_t x, std::int64_t y) {
    TileKey k;
    k.zone = k32N;
    k.level = 0;
    k.x = static_cast<std::int32_t>(x);
    k.y = static_cast<std::int32_t>(y);
    return k;
}

// A class field that is a function of the zone-metre node position only, so
// neighbouring tiles agree on their shared edge nodes as served tiles do.
std::uint8_t lc_at(std::int64_t e, std::int64_t n) { return static_cast<std::uint8_t>((e * 7 + n * 3) % 14); }
std::uint8_t sk_at(std::int64_t e, std::int64_t n) { return static_cast<std::uint8_t>((e + 2 * n) % 12); }

std::shared_ptr<const TerrainClassTile> make_classes(const TileKey& key) {
    auto t = std::make_shared<TerrainClassTile>();
    t->reset(key);
    for (int j = 0; j < g2m::kTerrainClassSamples; ++j) {
        for (int i = 0; i < g2m::kTerrainClassSamples; ++i) {
            const std::int64_t e = static_cast<std::int64_t>(key.x) * 256 + i;
            const std::int64_t n = static_cast<std::int64_t>(key.y) * 256 + j;
            const std::size_t at = TerrainClassTile::index(i, j);
            t->land_class[at] = lc_at(e, n);
            t->surface[at] = sk_at(e, n);
        }
    }
    return t;
}

std::shared_ptr<const g2m::HeightTile> make_heights(const TileKey& key, std::int32_t v = 0) {
    auto tile = std::make_shared<g2m::HeightTile>();
    tile->key = key;
    tile->h.fill(v);
    return tile;
}

SurfaceIdTable test_table() {
    return SurfaceIdTable::build([](SurfaceKind k) { return 100 + static_cast<int>(k); },
                                 [](LandClass l) { return 200 + static_cast<int>(l); });
}

// The physics tile used below straddles 2 x 2 L0 tiles: cell centres
// E 474766..475020 (L0 x 1854, 1855), N 5559491..5559745 (L0 y 21716, 21717).
const SessionFrame kFrame(k32N, 474000, 5560000);
constexpr PhysTileIndex kIndex{3, -2};
} // namespace

TEST_CASE("fill_physics_classes: every cell reads its centre node through the full table", "[phys][fill][classes]") {
    const PhysicsTileGrid grid(kFrame);
    std::vector<TileKey> keys;
    grid.l0_keys_for_square(kIndex, 0, keys);
    REQUIRE(keys.size() == 4);
    ResidentHeightSet resident;
    for (const TileKey& k : keys) resident.install(k, make_heights(k), nullptr, make_classes(k));

    const int ts = grid.samples() - 1;
    std::vector<std::uint16_t> out(static_cast<std::size_t>(ts) * ts, 0);
    const SurfaceIdTable table = test_table();
    const ClassFillStats stats = fill_physics_classes(grid, kIndex, resident, table, 9, out);
    CHECK(stats.resident_tiles == 4);
    CHECK(stats.absent_tiles == 0);
    CHECK(stats.missing_tiles == 0);

    const std::int64_t col0 = 474000 + 255 * 3, row0 = 5560000 - 255 * 2;
    int mismatches = 0, roads_by_kind = 0;
    for (int y = 0; y < ts; ++y) {
        for (int x = 0; x < ts; ++x) {
            const std::int64_t e = col0 + x + 1, n = row0 + y + 1;
            const auto lc = static_cast<LandClass>(lc_at(e, n));
            const auto sk = static_cast<SurfaceKind>(sk_at(e, n));
            if (SurfaceIdTable::is_road(lc) && sk != SurfaceKind::Unknown) ++roads_by_kind;
            if (out[static_cast<std::size_t>(y) * ts + x] != table.resolve(lc, sk)) ++mismatches;
        }
    }
    CHECK(mismatches == 0);
    CHECK(roads_by_kind > 0);   // the road-by-kind branch of resolve is exercised

    // Negative control: reading the node one metre west (the cell's own SW
    // sample, a plausible off-by-one) changes many cells.
    int shifted = 0;
    for (int y = 0; y < ts; ++y) {
        for (int x = 0; x < ts; ++x) {
            const std::int64_t e = col0 + x, n = row0 + y + 1;
            if (out[static_cast<std::size_t>(y) * ts + x] !=
                table.resolve(static_cast<LandClass>(lc_at(e, n)), static_cast<SurfaceKind>(sk_at(e, n)))) {
                ++shifted;
            }
        }
    }
    CHECK(shifted > ts * ts / 2);
}

TEST_CASE("fill_physics_classes: absent, missing and class-less tiles take the fallback", "[phys][fill][classes]") {
    const PhysicsTileGrid grid(kFrame);
    ResidentHeightSet resident;
    resident.install(l0_at(1854, 21716), make_heights(l0_at(1854, 21716)), nullptr, make_classes(l0_at(1854, 21716)));
    resident.install(l0_at(1855, 21716), nullptr);                                   // Absent
    resident.install(l0_at(1854, 21717), make_heights(l0_at(1854, 21717)), nullptr);  // Resident, no class tile
    // (1855, 21717) not installed: Missing.

    const int ts = grid.samples() - 1;
    std::vector<std::uint16_t> out(static_cast<std::size_t>(ts) * ts, 0);
    const SurfaceIdTable table = test_table();
    const ClassFillStats stats = fill_physics_classes(grid, kIndex, resident, table, 9, out);
    CHECK(stats.resident_tiles == 1);
    CHECK(stats.absent_tiles == 1);
    CHECK(stats.missing_tiles == 2);

    const std::int64_t col0 = 474000 + 255 * 3, row0 = 5560000 - 255 * 2;
    int fallback = 0, resolved = 0, wrong = 0;
    for (int y = 0; y < ts; ++y) {
        for (int x = 0; x < ts; ++x) {
            const std::int64_t e = col0 + x + 1, n = row0 + y + 1;
            const bool sw = e < 1855 * 256 && n < 21717 * 256;
            const std::uint16_t v = out[static_cast<std::size_t>(y) * ts + x];
            if (sw) {
                ++resolved;
                if (v != table.resolve(static_cast<LandClass>(lc_at(e, n)), static_cast<SurfaceKind>(sk_at(e, n)))) ++wrong;
            } else {
                ++fallback;
                if (v != 9) ++wrong;
            }
        }
    }
    CHECK(wrong == 0);
    CHECK(resolved > 0);
    CHECK(fallback > 0);
}

TEST_CASE("fill_physics_classes: allocates nothing", "[phys][fill][classes][alloc]") {
    if (!g2m_test::kAllocCounterAvailable) SKIP("AllocCounter is disabled on a sanitizer build");
    const PhysicsTileGrid grid(kFrame);
    std::vector<TileKey> keys;
    grid.l0_keys_for_square(kIndex, 0, keys);
    ResidentHeightSet resident;
    for (const TileKey& k : keys) resident.install(k, make_heights(k), nullptr, make_classes(k));
    const int ts = grid.samples() - 1;
    std::vector<std::uint16_t> out(static_cast<std::size_t>(ts) * ts);
    const SurfaceIdTable table = test_table();
    fill_physics_classes(grid, kIndex, resident, table, 9, out);
    g2m_test::ScopedAllocCounter counter;
    fill_physics_classes(grid, kIndex, resident, table, 9, out);
    CHECK(counter.count() == 0);
}

TEST_CASE("ResidentHeightSet: the class tile rides beside the road metadata", "[phys][resident_heights][classes]") {
    ResidentHeightSet set;
    const TileKey k = l0_at(10, 20);
    const auto heights = make_heights(k);
    const auto roads = std::make_shared<const std::vector<RoadSegment>>(
        std::vector<RoadSegment>{RoadSegment{{0, 0}, {1000, 0}, 50, LandClass::PavedRoad, SurfaceKind::Asphalt, 1}});
    const auto classes = make_classes(k);
    set.install(k, heights, roads, classes);

    const g2m::HeightTile* h = nullptr;
    std::shared_ptr<const std::vector<RoadSegment>> r;
    const TerrainClassTile* c = nullptr;
    REQUIRE(set.find(k, &h, &r, &c) == ResidentHeightSet::Lookup::Resident);
    CHECK(h == heights.get());
    CHECK(r == roads);   // G-2: class mode keeps the RoadSegment metadata
    CHECK(c == classes.get());

    // The 3-arg install replaces the whole entry: the class tile goes.
    set.install(k, heights, roads);
    REQUIRE(set.find(k, &h, &r, &c) == ResidentHeightSet::Lookup::Resident);
    CHECK(c == nullptr);
    CHECK(r == roads);

    CHECK(set.find(l0_at(11, 20), &h, &r, &c) == ResidentHeightSet::Lookup::Missing);
    CHECK(c == nullptr);
}

namespace {
// Answers 200 with heights, roads and (unless `with_classes` is false) the
// class tile of the key.
class ClassFetch final : public g2m::phys::IHeightTileFetch {
public:
    explicit ClassFetch(bool with_classes) : with_classes_(with_classes) {}
    FetchResult fetch(const TileKey& key) override {
        {
            std::lock_guard<std::mutex> lk(m_);
            ++calls_;
        }
        auto roads = std::make_shared<const std::vector<RoadSegment>>();
        return FetchResult{Status::Ok, make_heights(key, key.x), std::move(roads),
                           with_classes_ ? make_classes(key) : nullptr};
    }
    bool provides_roads() const override { return true; }
    bool provides_classes() const override { return with_classes_; }
    int calls() const {
        std::lock_guard<std::mutex> lk(m_);
        return calls_;
    }
private:
    bool with_classes_;
    mutable std::mutex m_;
    int calls_ = 0;
};

std::vector<LoadedHeightTile> poll_until(HeightTileLoader& loader, std::size_t want) {
    std::vector<LoadedHeightTile> out;
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (out.size() < want && std::chrono::steady_clock::now() < deadline) {
        if (loader.poll(out) == 0) std::this_thread::sleep_for(1ms);
    }
    return out;
}

LoaderConfig class_config() {
    LoaderConfig c;
    c.workers = 2;
    c.max_attempts = 3;
    c.backoff = 1ms;
    c.require_roads = true;
    c.require_classes = true;
    return c;
}
} // namespace

TEST_CASE("HeightTileLoader: require_classes keeps roads and classes; a fetch without classes fails",
          "[phys][loader][classes]") {
    const TileKey k = l0_at(10, 20);
    {
        auto fetch = std::make_shared<ClassFetch>(true);
        HeightTileLoader loader(fetch, class_config());
        const std::vector<TileKey> want{k};
        loader.request(want);
        const auto done = poll_until(loader, 1);
        REQUIRE(done.size() == 1);
        CHECK(done[0].outcome == LoadOutcome::Ready);
        REQUIRE(done[0].roads);
        REQUIRE(done[0].classes);
        CHECK(done[0].classes->key == k);
    }
    {
        auto fetch = std::make_shared<ClassFetch>(false);
        HeightTileLoader loader(fetch, class_config());
        const std::vector<TileKey> want{k};
        loader.request(want);
        const auto done = poll_until(loader, 1);
        REQUIRE(done.size() == 1);
        CHECK(done[0].outcome == LoadOutcome::Failed);
        CHECK(done[0].attempts == 3);
        CHECK(fetch->calls() == 3);
    }
    {
        // require_classes off: the same class-less fetch is Ready (the default is unchanged).
        auto fetch = std::make_shared<ClassFetch>(false);
        LoaderConfig c = class_config();
        c.require_classes = false;
        HeightTileLoader loader(fetch, c);
        const std::vector<TileKey> want{k};
        loader.request(want);
        const auto done = poll_until(loader, 1);
        REQUIRE(done.size() == 1);
        CHECK(done[0].outcome == LoadOutcome::Ready);
        CHECK_FALSE(done[0].classes);
    }
}

TEST_CASE("PhysicsTerrainStreamer: installs the class tile and the roads of every gate key", "[phys][streamer][classes]") {
    const PhysicsTileGrid grid(SessionFrame(k32N, 474000, 5560000));
    ResidentHeightSet set;
    auto fetch = std::make_shared<ClassFetch>(true);
    HeightTileLoader loader(fetch, class_config());
    g2m::phys::PhysicsTerrainStreamer streamer(grid, set, loader, g2m::phys::StreamerConfig{});
    const std::vector<g2m::phys::InterestPoint> points{g2m::phys::InterestPoint{0, 100.0, 100.0, 0.0, 0.0, 255.0}};
    bool ready = false;
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (!ready && std::chrono::steady_clock::now() < deadline) {
        ready = streamer.update(points).ready;
        if (!ready) std::this_thread::sleep_for(1ms);
    }
    REQUIRE(ready);
    int with_both = 0;
    for (const TileKey& k : streamer.gate_keys()) {
        const g2m::HeightTile* h = nullptr;
        std::shared_ptr<const std::vector<RoadSegment>> r;
        const TerrainClassTile* c = nullptr;
        REQUIRE(set.find(k, &h, &r, &c) == ResidentHeightSet::Lookup::Resident);
        if (r && c && c->key == k) ++with_both;
    }
    CHECK(with_both == static_cast<int>(streamer.gate_keys().size()));
}

namespace {
// Serves one container per layer name, or a status.
class LayerTransport final : public g2m::ITransport {
public:
    g2m::Response send(const g2m::Request& request) override {
        std::lock_guard<std::mutex> lk(m_);
        ++sends_;
        g2m::TileResponse resp;
        resp.meta.status = Status::NotFound;
        if (const auto* tr = std::get_if<g2m::TileRequest>(&request)) {
            const auto it = layers_.find(tr->layer);
            if (it != layers_.end()) {
                resp.meta.status = it->second.first;
                if (it->second.first == Status::Ok) resp.container = it->second.second;
            }
        }
        return g2m::Response{resp};
    }
    void set(const std::string& layer, Status s, std::vector<std::uint8_t> container = {}) {
        std::lock_guard<std::mutex> lk(m_);
        layers_[layer] = {s, std::move(container)};
    }
    int sends() const {
        std::lock_guard<std::mutex> lk(m_);
        return sends_;
    }
private:
    mutable std::mutex m_;
    std::map<std::string, std::pair<Status, std::vector<std::uint8_t>>> layers_;
    int sends_ = 0;
};

std::vector<std::uint8_t> container_of(const std::string& layer, const TileKey& key, const g2m::TileBody& body) {
    const auto bytes = g2m::encode_body(body);
    g2m::TileHeader header;
    header.layer = layer;
    header.layer_version = 1;
    header.key = key;
    auto c = g2m::assemble_container(header, *bytes);
    return c ? *c : std::vector<std::uint8_t>{};
}
} // namespace

TEST_CASE("TransportHeightTileFetch: class layer beside the carved heights", "[phys][loader][classes]") {
    const TileKey k = l0_at(1854, 21716);
    const std::string carved(g2m::kTerrainCarvedLayer), cls(g2m::kTerrainClassLayer);
    LayerTransport transport;
    const auto heights = make_heights(k, 25600);
    const auto classes = make_classes(k);
    const auto height_body = g2m::encode_height_tile(*heights);
    const auto class_body = g2m::encode_terrain_class(*classes);
    REQUIRE(height_body);
    REQUIRE(class_body);
    transport.set(carved, Status::Ok, container_of(carved, k, *height_body));
    transport.set(cls, Status::Ok, container_of(cls, k, *class_body));

    g2m::ReleaseId rid{};
    g2m::phys::TransportHeightTileFetch fetch(transport, rid, carved, cls);
    CHECK(fetch.provides_classes());
    FetchResult r = fetch.fetch(k);
    REQUIRE(r.status == Status::Ok);
    REQUIRE(r.tile);
    REQUIRE(r.classes);
    CHECK(*r.classes == *classes);
    CHECK(r.tile->h[0] == 25600);

    // A class tile missing beside a present height tile is a server fault.
    transport.set(cls, Status::NotFound);
    CHECK(fetch.fetch(k).status == Status::Internal);
    // A class tile for another key is rejected too.
    const TileKey other = l0_at(1855, 21716);
    const auto other_body = g2m::encode_terrain_class(*make_classes(other));
    transport.set(cls, Status::Ok, container_of(cls, other, *other_body));
    CHECK(fetch.fetch(k).status == Status::Internal);
    // A 404 height stays a 404 (no class request needed).
    transport.set(carved, Status::NotFound);
    const int before = transport.sends();
    CHECK(fetch.fetch(k).status == Status::NotFound);
    CHECK(transport.sends() == before + 1);

    // Without a class layer: heights only, one request, no classes.
    transport.set(carved, Status::Ok, container_of(carved, k, *height_body));
    g2m::phys::TransportHeightTileFetch plain(transport, rid, carved);
    CHECK_FALSE(plain.provides_classes());
    const int before_plain = transport.sends();
    FetchResult p = plain.fetch(k);
    CHECK(p.status == Status::Ok);
    CHECK_FALSE(p.classes);
    CHECK(transport.sends() == before_plain + 1);
}
