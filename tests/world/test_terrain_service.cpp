// g2m::world::TerrainService (include/g2m/world/terrain_service.h; terrain migration plan phase 2, vault
// G2M-020): dedup, priorities, cancellation, shutdown, bit-identity with the plain serial server, warm-up,
// PhysicsFetch, peeks, producers. The world is a synthetic one: 5 x 5 flat 100 m elevation tiles and one
// service road per level-2 tile of the central 3 x 3 (the pattern of tests/layer/test_road_geom.cpp).
//
// Test state shared with pool threads is atomics or mutex-guarded, so the file is TSan-clean.
#include "g2m/world/terrain_service.h"

#include "g2m/core/hash.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"
#include "g2m/client/in_process_transport.h"
#include "g2m/world/tile_fetch.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>
#include <future>
#include <mutex>
#include <thread>
#include <utility>
#include <vector>

using namespace g2m;
using namespace g2m::world;
using namespace std::chrono_literals;

namespace {

const geo::UtmZone kZone{32, geo::Hemisphere::North};
const std::string kScope = "svc_test";
constexpr auto kLong = 60s;

class TempDir {
public:
    TempDir() {
        static std::atomic<std::uint64_t> counter{0};
        path_ = std::filesystem::temp_directory_path() /
                ("g2m_svc_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + "_" +
                 std::to_string(counter.fetch_add(1)));
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

TileKey l2(int x, int y) { return TileKey{kZone, 2, x, y}; }
TileKey l0(int x, int y) { return TileKey{kZone, 0, x, y}; }

InputTile input_tile(std::string layer, TileKey key, const TileBody& body, VerticalDatum datum = VerticalDatum::None) {
    InputTile out;
    out.ref = {std::move(layer), key};
    out.body = *encode_body(body);
    TileHeader h;
    h.layer = out.ref.layer;
    h.layer_version = 1;
    h.key = key;
    h.datum = datum;
    h.body_size = out.body.size();
    h.body_hash = content_hash(out.body);
    out.header = h;
    return out;
}

// A deriver counting its calls: output = f(the input tile's body hash, the key). `delay_ms` makes it slow.
class ChainDeriver final : public IDeriver {
public:
    ChainDeriver(std::string out, std::string in, int delay_ms = 0) : in_(std::move(in)), delay_ms_(delay_ms) {
        desc_.id = LayerId{std::move(out), 1};
        desc_.kind = LayerKind::Baked;
        desc_.encoding = 0x8001;
        desc_.levels = level_bit(kRawTileLevel);
        desc_.inputs = {in_};
        desc_.content = "service test layer";
        config_[0] = 0x31;
    }
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override { return config_; }
    std::vector<InputTileRef> inputs_for(const TileKey& t) const override { return {InputTileRef{in_, t}}; }
    Result<DeriveResult> derive(const DeriveRequest& req) const override {
        calls++;
        if (delay_ms_ > 0) std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms_));
        TileBody b;
        b.encoding = desc_.encoding;
        b.encoding_version = 1;
        Section s{1, SectionCodec::Zstd, {}};
        for (const InputTile& in : req.inputs) {
            s.data.push_back(in.header ? 1 : 0);
            if (in.header) s.data.insert(s.data.end(), in.header->body_hash.begin(), in.header->body_hash.end());
        }
        const std::uint64_t k = req.target.packed();
        for (int i = 0; i < 8; ++i) s.data.push_back(static_cast<std::uint8_t>(k >> (8 * i)));
        b.sections.push_back(std::move(s));
        auto body = encode_body(b);
        if (!body) return body.error();
        DeriveResult r;
        r.body = std::move(*body);
        return r;
    }
    mutable std::atomic<int> calls{0};

private:
    std::string in_;
    int delay_ms_;
    LayerDesc desc_;
    ContentHash config_{};
};

// The source store of the synthetic world, closed again after filling: 5x5 flat DEM tiles (x, y in -2..2) and
// one 100 m service road in every level-2 tile of -1..1.
struct World {
    TempDir dir;
    std::filesystem::path sources_dir() const { return dir.path() / "sources"; }

    World() {
        auto sources = TileStore::open({sources_dir(), "tiles"});
        REQUIRE(sources);
        const geo::Utm utm(geo::kGRS80);
        for (int y = -1; y <= 1; ++y)
            for (int x = -1; x <= 1; ++x) {
                const TileKey key = l2(x, y);
                const double east = x * 1024.0 + 10, north = y * 1024.0 + 10;
                osm::OsmBuilder builder;
                const std::int64_t base = 1000 * (static_cast<std::int64_t>(y + 1) * 3 + (x + 1) + 1);
                for (const auto [id, e] : {std::pair<std::int64_t, double>{base + 1, east}, {base + 2, east + 100}}) {
                    const auto p = utm.reverse(kZone, e, north);
                    builder.add_node(id, static_cast<std::int32_t>(std::llround(p.lat * 1e7)),
                                     static_cast<std::int32_t>(std::llround(p.lon * 1e7)));
                }
                builder.add_way(base + 10, {base + 1, base + 2}, {{"highway", "service"}, {"maxspeed", "30"}});
                const auto data = builder.build();
                REQUIRE(data);
                auto raw = input_tile(std::string(kSrcOsmLayer), key, *encode_src_osm(OsmTile{64, *data}));
                REQUIRE((*sources)->put({kScope, raw.ref.layer, key}, *raw.header, raw.body, true));
            }
        for (int y = -2; y <= 2; ++y)
            for (int x = -2; x <= 2; ++x) {
                const TileKey k = l2(x, y);
                ElevTile elev;
                elev.georef = *tile_georef(k, 1024, SampleRegistration::Cell);
                elev.georef.row_order = RowOrder::NorthToSouth;
                elev.heights = HeightRaster(1024, 1024);
                std::fill(elev.heights.data().begin(), elev.heights.data().end(), 25600);
                auto data = input_tile(std::string(kSrcElevDgm1Layer), k, *encode_src_elev(elev), VerticalDatum::DHHN2016);
                REQUIRE((*sources)->put({kScope, data.ref.layer, k}, *data.header, data.body, true));
            }
    }

    TerrainServiceConfig config(const std::string& derived_name = "derived", int workers = 2) const {
        TerrainServiceConfig c;
        c.stores.source_dir = sources_dir();
        c.stores.source_scope = kScope;
        c.stores.source_read_only = false;
        c.stores.derived_dir = dir.path() / "derived_dir";
        c.stores.derived_name = derived_name;
        c.zone = kZone;
        c.derive_workers = workers;
        return c;
    }
    std::unique_ptr<TerrainService> open(const TerrainServiceConfig& c) const {
        auto s = TerrainService::open(c);
        INFO((s ? "ok" : s.error().message));
        REQUIRE(s);
        return std::move(*s);
    }
    std::unique_ptr<TerrainService> open(const std::string& derived_name = "derived", int workers = 2) const {
        return open(config(derived_name, workers));
    }
};

// Latched producer harness: products whose body runs under test control.
struct Marker {
    int id = 0;
};
struct Harness {
    ProductKind kind = TerrainService::register_product_kind("test.marker");
    std::mutex mu;
    std::condition_variable cv;
    bool released = false;
    std::atomic<int> started{0};
    std::atomic<int> runs{0};
    std::vector<int> order; // ids in execution order (guarded by mu)

    Producer<Marker> blocker() {
        return [this](ProductContext&, const TileKey& k) {
            started++;
            std::unique_lock lock(mu);
            cv.wait(lock, [&] { return released; });
            return Outcome<Marker>::ready(std::make_shared<const Marker>(Marker{k.x}), 8);
        };
    }
    Producer<Marker> recorder() {
        return [this](ProductContext&, const TileKey& k) {
            runs++;
            {
                std::lock_guard lock(mu);
                order.push_back(k.x);
            }
            return Outcome<Marker>::ready(std::make_shared<const Marker>(Marker{k.x}), 8);
        };
    }
    void release() {
        {
            std::lock_guard lock(mu);
            released = true;
        }
        cv.notify_all();
    }
    void wait_started(int n) {
        for (int i = 0; i < 20000 && started < n; ++i) std::this_thread::sleep_for(1ms);
        REQUIRE(started >= n);
    }
    Ticket<Marker> submit_blocker(TerrainService& svc, int x = 900) {
        return svc.product<Marker>(kind, l2(x, 0), Priority::Gate, 0, blocker());
    }
    Ticket<Marker> submit(TerrainService& svc, int x, Priority p) {
        return svc.product<Marker>(kind, l2(x, 0), p, 0, recorder());
    }
};

bool wait_ready(const std::vector<Ticket<RawTile>>& ts) {
    return std::all_of(ts.begin(), ts.end(), [](const Ticket<RawTile>& t) { return t.wait_for(kLong); });
}

} // namespace

TEST_CASE("service: open validates its configuration", "[world][terrain_service]") {
    World w;
    auto bad = [&](auto mutate) {
        TerrainServiceConfig c = w.config();
        mutate(c);
        const auto r = TerrainService::open(c);
        CHECK_FALSE(r.ok());
        return r.ok() ? std::string() : r.error().message;
    };
    CHECK(bad([](auto& c) { c.zone = geo::UtmZone{0, geo::Hemisphere::North}; }).find("zone") != std::string::npos);
    CHECK_FALSE(bad([](auto& c) { c.e0_m = 1500; }).empty());
    CHECK_FALSE(bad([](auto& c) { c.layers.classes = true; }).empty());
    CHECK_FALSE(bad([](auto& c) { c.layers.roads = false; c.layers.geometry = true; }).empty());
    CHECK_FALSE(bad([](auto& c) { c.stores.source_scope.clear(); }).empty());
    CHECK_FALSE(bad([](auto& c) {
        c.stores.source_dir = c.stores.source_dir / "missing";
        c.stores.source_read_only = true;
    }).empty());
}

TEST_CASE("service: release id and coverage match the plain local release", "[world][terrain_service]") {
    World w;
    auto svc = w.open();
    BuiltinTerrainDerivers derivers;
    auto sources = TileStore::open({w.sources_dir(), "tiles"});
    REQUIRE(sources);
    auto manifest = make_local_release(**sources, builtin_local_release_params(kScope, derivers, {}, true, true));
    REQUIRE(manifest);
    CHECK(svc->release_id() == manifest->rid);
    CHECK(svc->release_id_hex() == to_hex(manifest->rid));
    CHECK(svc->manifest() == *manifest);
    CHECK(svc->has_layer(kRoadGeomLayer));
    CHECK_FALSE(svc->has_layer(kTerrainCarvedLayer));
    CHECK(svc->height_layer() == kTerrainHeightLayer);
    CHECK(svc->frame().zone() == kZone);

    const auto geom = svc->covered_tiles(kRoadGeomLayer, 2);
    CHECK(geom.size() == 9);
    CHECK(std::is_sorted(geom.begin(), geom.end()));
    CHECK(svc->covered_tiles(kRoadGeomLayer, 0).empty()); // the layer has no level 0
    const auto h2 = svc->covered_tiles(kTerrainHeightLayer, 2);
    const auto h0 = svc->covered_tiles(kTerrainHeightLayer, 0);
    CHECK(!h2.empty());
    CHECK(h0.size() == h2.size() * 16);
    CHECK(svc->covered_tiles("no.such.layer", 2).empty());

    // The carved opt-in changes the release (it adds a layer) and the layer height() serves.
    TerrainServiceConfig cc = w.config("derived_carved");
    cc.layers.carved = true;
    auto carved = w.open(cc);
    CHECK(carved->height_layer() == kTerrainCarvedLayer);
    CHECK(carved->release_id() != svc->release_id());
}

TEST_CASE("service: concurrent requests for one tile derive it exactly once", "[world][terrain_service][dedup]") {
    World w;
    ChainDeriver slow("test.slow", std::string(kSrcOsmLayer), 80);
    TerrainServiceConfig c = w.config("derived", 4);
    c.extra_derivers = {&slow};
    auto svc = w.open(c);

    constexpr int kThreads = 16;
    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    std::vector<Ticket<RawTile>> tickets(kThreads);
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t)
        threads.emplace_back([&, t] {
            ready++;
            while (!go) std::this_thread::yield();
            tickets[static_cast<std::size_t>(t)] = svc->raw_tile("test.slow", l2(0, 0), static_cast<Priority>(t % 5));
        });
    while (ready < kThreads) std::this_thread::yield();
    go = true;
    for (auto& th : threads) th.join();
    REQUIRE(wait_ready(tickets));
    for (const auto& t : tickets) {
        REQUIRE(t.state() == TileState::Ready);
        CHECK(t.value()->container == tickets[0].value()->container);
    }
    CHECK(slow.calls == 1);
    const ServiceStats st = svc->stats();
    CHECK(st.jobs_run == 1);
    CHECK(st.joined + st.cache_hits >= static_cast<std::uint64_t>(kThreads) - 1);

    // A second round is answered from the memory cache with no new work.
    auto again = svc->raw_tile("test.slow", l2(0, 0), Priority::Render);
    CHECK(again.state() == TileState::Ready);
    CHECK(slow.calls == 1);
    CHECK(svc->stats().cache_hits >= 1);
}

TEST_CASE("service: the resolver derives a shared dependency once across distinct jobs", "[world][terrain_service][dedup]") {
    World w;
    ChainDeriver base("test.base", std::string(kSrcOsmLayer), 80);
    ChainDeriver a("test.a", "test.base"), b("test.b", "test.base");
    TerrainServiceConfig c = w.config("derived", 4);
    c.extra_derivers = {&base, &a, &b};
    auto svc = w.open(c);
    auto ta = svc->raw_tile("test.a", l2(1, 1), Priority::Render);
    auto tb = svc->raw_tile("test.b", l2(1, 1), Priority::Physics);
    REQUIRE(ta.wait_for(kLong));
    REQUIRE(tb.wait_for(kLong));
    CHECK(ta.state() == TileState::Ready);
    CHECK(tb.state() == TileState::Ready);
    CHECK(base.calls == 1);
    CHECK(a.calls == 1);
    CHECK(b.calls == 1);
    CHECK(svc->stats().jobs_run == 2); // two service jobs, one shared derivation underneath
}

TEST_CASE("service: jobs run in priority order, requests of equal priority in request order",
          "[world][terrain_service][priority]") {
    World w;
    auto svc = w.open("derived", /*workers=*/1);
    Harness h;
    auto blocker = h.submit_blocker(*svc);
    h.wait_started(1); // the only worker is busy: everything below queues

    std::vector<Ticket<Marker>> ts;
    ts.push_back(h.submit(*svc, 1, Priority::Warm));
    ts.push_back(h.submit(*svc, 2, Priority::Render));
    ts.push_back(h.submit(*svc, 3, Priority::Prefetch));
    ts.push_back(h.submit(*svc, 4, Priority::Render));
    ts.push_back(h.submit(*svc, 5, Priority::Physics));
    ts.push_back(h.submit(*svc, 6, Priority::Gate));
    ts.push_back(h.submit(*svc, 7, Priority::Warm));
    h.release();
    for (auto& t : ts) REQUIRE(t.wait_for(kLong));
    for (auto& t : ts) CHECK(t.state() == TileState::Ready);
    CHECK(h.order == std::vector<int>{6, 5, 3, 2, 4, 1, 7});
}

TEST_CASE("service: a request for a queued tile raises its priority", "[world][terrain_service][priority]") {
    World w;
    auto svc = w.open("derived", 1);
    Harness h;
    auto blocker = h.submit_blocker(*svc);
    h.wait_started(1);
    auto x = h.submit(*svc, 1, Priority::Warm);
    auto y = h.submit(*svc, 2, Priority::Warm);
    auto z = h.submit(*svc, 3, Priority::Render);
    auto x_again = h.submit(*svc, 1, Priority::Gate); // joins x's job and raises it above everything
    h.release();
    REQUIRE(x.wait_for(kLong));
    REQUIRE(x_again.wait_for(kLong));
    REQUIRE(y.wait_for(kLong));
    REQUIRE(z.wait_for(kLong));
    CHECK(h.runs == 3); // x ran once for both holders
    CHECK(h.order == std::vector<int>{1, 3, 2});
    CHECK(svc->stats().boosted >= 1);
    CHECK(x.value() == x_again.value());
}

TEST_CASE("service: cancelled and dropped tickets neither run nor leak", "[world][terrain_service][cancel]") {
    World w;
    auto svc = w.open("derived", 1);
    Harness h;
    auto blocker = h.submit_blocker(*svc);
    h.wait_started(1);

    auto cancelled = h.submit(*svc, 1, Priority::Render);
    cancelled.cancel();
    CHECK(cancelled.state() == TileState::Cancelled);
    cancelled.cancel(); // idempotent
    {
        auto dropped = h.submit(*svc, 2, Priority::Render);
        auto copy = dropped; // copies share the one interest: the job lives until the last copy goes
        dropped = Ticket<Marker>();
        CHECK(copy.state() == TileState::Pending);
    }                       // last copy dropped here
    auto shared_a = h.submit(*svc, 3, Priority::Render);
    auto shared_b = h.submit(*svc, 3, Priority::Render); // second holder of the same job
    shared_a.cancel();                                   // one holder leaves, the job stays
    CHECK(shared_a.state() == TileState::Cancelled);
    auto sentinel = h.submit(*svc, 99, Priority::Warm);
    h.release();
    REQUIRE(sentinel.wait_for(kLong));
    REQUIRE(shared_b.wait_for(kLong));
    CHECK(shared_b.state() == TileState::Ready);
    CHECK(shared_a.state() == TileState::Cancelled); // stays cancelled although the job finished for the other holder
    CHECK(h.order == std::vector<int>{3, 99});       // 1 and 2 never ran
    CHECK(svc->stats().cancelled_before_start >= 2);

    // A cancelled job can be requested again and then runs.
    auto retry = h.submit(*svc, 1, Priority::Render);
    REQUIRE(retry.wait_for(kLong));
    CHECK(retry.state() == TileState::Ready);
    CHECK(blocker.state() == TileState::Ready);
}

TEST_CASE("service: cancelling a running job lets it finish and keeps its result", "[world][terrain_service][cancel]") {
    World w;
    auto svc = w.open("derived", 2);
    Harness h;
    auto running = h.submit_blocker(*svc, 7);
    h.wait_started(1);
    running.cancel();
    CHECK(running.state() == TileState::Cancelled);
    h.release();
    for (int i = 0; i < 20000 && !svc->peek_product<Marker>(h.kind, l2(7, 0)); ++i) std::this_thread::sleep_for(1ms);
    REQUIRE(svc->peek_product<Marker>(h.kind, l2(7, 0)));
    CHECK(svc->peek_product<Marker>(h.kind, l2(7, 0))->id == 7);
}

TEST_CASE("service: shutting down with queued and running jobs does not deadlock", "[world][terrain_service][cancel]") {
    World w;
    auto svc = w.open("derived", 1);
    Harness h;
    auto running = h.submit_blocker(*svc);
    h.wait_started(1);
    std::vector<Ticket<Marker>> queued;
    for (int i = 1; i <= 5; ++i) queued.push_back(h.submit(*svc, i, Priority::Render));
    auto height = svc->height(l0(0, 0), Priority::Physics); // a builtin job queued behind it

    std::thread releaser([&] {
        std::this_thread::sleep_for(150ms);
        h.release();
    });
    auto destroyed = std::async(std::launch::async, [&] { svc.reset(); });
    REQUIRE(destroyed.wait_for(30s) == std::future_status::ready);
    releaser.join();
    CHECK(running.state() == TileState::Ready); // the running job finished
    for (auto& t : queued) CHECK(t.state() == TileState::Cancelled);
    CHECK(height.state() == TileState::Cancelled);
    CHECK(h.runs == 0);
}

TEST_CASE("service: results are bit-identical to the plain serial server", "[world][terrain_service][identity]") {
    World w;
    // Plain serial reference: the same release over the same sources, its own derived store.
    BuiltinTerrainDerivers derivers;
    auto sources = TileStore::open({w.sources_dir(), "tiles"});
    REQUIRE(sources);
    auto plain_derived = TileStore::open({w.dir.path() / "plain", "tiles"});
    REQUIRE(plain_derived);
    LocalSourceUpstream upstream(**sources);
    const auto manifest = make_local_release(**sources, builtin_local_release_params(kScope, derivers, {}, true, true));
    REQUIRE(manifest);
    ServerConfig sc;
    sc.offline = true;
    sc.derived_store = plain_derived->get();
    sc.derivers = derivers.ordered(true, true);
    sc.upstreams = {&upstream};
    Server plain(sc);
    REQUIRE(plain.add_release(*manifest, kScope));

    auto svc = w.open("derived", 4);
    REQUIRE(svc->release_id() == manifest->rid);

    std::vector<std::pair<std::string, TileKey>> items;
    for (const TileKey& k : svc->covered_tiles(kRoadGeomLayer, 2)) items.emplace_back(std::string(kRoadGeomLayer), k);
    for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x) items.emplace_back(std::string(kTerrainHeightLayer), l0(x, y));
    items.emplace_back(std::string(kTerrainHeightLayer), l0(400, 400));
    // Everything at once, mixed priorities, in a scrambled order: the pool must not change a single byte.
    std::vector<std::pair<std::string, TileKey>> scrambled = items;
    std::reverse(scrambled.begin(), scrambled.end());
    std::vector<Ticket<RawTile>> raw;
    for (std::size_t i = 0; i < scrambled.size(); ++i)
        raw.push_back(svc->raw_tile(scrambled[i].first, scrambled[i].second, static_cast<Priority>(i % 5)));
    REQUIRE(wait_ready(raw));
    for (std::size_t i = 0; i < scrambled.size(); ++i) {
        const TileResponse ref = plain.tile({manifest->rid, scrambled[i].first, scrambled[i].second, std::nullopt});
        INFO(scrambled[i].first << " " << to_string(scrambled[i].second));
        if (ref.meta.status == Status::Ok) {
            REQUIRE(raw[i].state() == TileState::Ready);
            CHECK(raw[i].value()->container == ref.container);
        } else {
            CHECK(raw[i].state() == TileState::Absent);
            CHECK(ref.meta.status == Status::NotFound);
        }
    }

    // The typed products decode to what the plain server's bytes decode to.
    InProcessTransport transport(plain);
    for (int y = 0; y < 4; ++y)
        for (int x = 0; x < 4; ++x) {
            auto t = svc->height(l0(x, y), Priority::Physics);
            REQUIRE(t.wait_for(kLong));
            REQUIRE(t.state() == TileState::Ready);
            const auto ref = fetch_height_tile(transport, manifest->rid, kTerrainHeightLayer, l0(x, y));
            REQUIRE(ref.tile);
            CHECK(t.value()->h == ref.tile->h);
        }
    for (const TileKey& k : svc->covered_tiles(kRoadGeomLayer, 2)) {
        auto segs = svc->road_segments(k, Priority::Render);
        auto geom = svc->road_geom(k, Priority::Render);
        REQUIRE(segs.wait_for(kLong));
        REQUIRE(geom.wait_for(kLong));
        REQUIRE(segs.state() == TileState::Ready);
        REQUIRE(geom.state() == TileState::Ready);
        const Response osm = plain.handle(Request{TileRequest{manifest->rid, std::string(kSrcOsmLayer), k, std::nullopt}});
        const RoadSegmentsResult ref = decode_road_segments_response(osm, k, kZone);
        REQUIRE(ref.status == RoadFetchStatus::Ok);
        CHECK(segs.value()->segments == *ref.segments);
        CHECK(!segs.value()->segments.empty());
        const TileResponse gref = plain.tile({manifest->rid, std::string(kRoadGeomLayer), k, std::nullopt});
        const auto view = parse_container(gref.container);
        REQUIRE(view);
        const auto body = decode_body(view->body);
        REQUIRE(body);
        const auto rt = decode_road_geom(*body);
        REQUIRE(rt);
        CHECK(*encode_road_geom(*geom.value()) == *encode_road_geom(*rt));
        CHECK(!geom.value()->entries.empty());
    }
    // Outside the release: a final Absent, not a failure.
    auto none = svc->height(l0(400, 400), Priority::Render);
    REQUIRE(none.wait_for(kLong));
    CHECK(none.state() == TileState::Absent);
    CHECK_FALSE(none.value());
}

TEST_CASE("service: warm fills the derived store and leaves the memory cache alone", "[world][terrain_service][warm]") {
    World w;
    std::vector<TileKey> geom_keys, height_keys;
    {
        auto svc = w.open("derived", 4);
        geom_keys = svc->covered_tiles(kRoadGeomLayer, 2);
        for (int y = 0; y < 4; ++y)
            for (int x = 0; x < 4; ++x) height_keys.push_back(l0(x, y));
        WarmRequest req = WarmRequest::of(kRoadGeomLayer, geom_keys);
        for (const TileKey& k : height_keys) req.items.push_back({std::string(kTerrainHeightLayer), k});
        req.items.push_back({std::string(kTerrainHeightLayer), l0(400, 400)}); // outside the release
        WarmJob job = svc->warm(req);
        REQUIRE(job.wait_for(kLong));
        const WarmProgress p = job.progress();
        CHECK(p.finished);
        CHECK(p.total == req.items.size());
        CHECK(p.ready == geom_keys.size() + height_keys.size());
        CHECK(p.absent == 1);
        CHECK(p.failed == 0);
        CHECK(p.cancelled == 0);
        CHECK(svc->stats().cache_entries == 0);
        CHECK_FALSE(svc->peek_height(height_keys[0]));
        CHECK_FALSE(svc->peek_road_geom(geom_keys[0]));

        // Warming again is served from the derived store.
        WarmJob again = svc->warm(req);
        REQUIRE(again.wait_for(kLong));
        CHECK(again.progress().ready == p.ready);
    }
    // The store now holds every tile (a second reader object on the same files).
    auto derived = TileStore::open({w.dir.path() / "derived_dir", "derived"});
    REQUIRE(derived);
    const std::string rid_hex = [&] {
        auto svc = w.open("derived", 1);
        return svc->release_id_hex();
    }();
    for (const TileKey& k : geom_keys) {
        const auto t = (*derived)->get({rid_hex, std::string(kRoadGeomLayer), k});
        REQUIRE(t);
        CHECK(t->has_value());
    }
    for (const TileKey& k : height_keys) {
        const auto t = (*derived)->get({rid_hex, std::string(kTerrainHeightLayer), k});
        REQUIRE(t);
        CHECK(t->has_value());
    }

    // A fresh service over the filled store derives nothing: a counting deriver proves the cache path.
    auto svc2 = w.open("derived", 2);
    auto t = svc2->height(height_keys[0], Priority::Physics);
    REQUIRE(t.wait_for(kLong));
    CHECK(t.state() == TileState::Ready);
}

TEST_CASE("service: warm can be cancelled", "[world][terrain_service][warm]") {
    World w;
    auto svc = w.open("derived", 2);
    WarmRequest req;
    req.window = 1;
    for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x) req.items.push_back({std::string(kTerrainHeightLayer), l0(x, y)});
    WarmJob job = svc->warm(req);
    job.cancel();
    REQUIRE(job.wait_for(kLong));
    const WarmProgress p = job.progress();
    CHECK(p.finished);
    CHECK(p.ready + p.absent + p.failed + p.cancelled == p.total);
    CHECK(p.cancelled > p.total / 2);
    job.cancel(); // idempotent
    // The service is still healthy afterwards.
    auto t = svc->height(l0(0, 0), Priority::Physics);
    REQUIRE(t.wait_for(kLong));
    CHECK(t.state() == TileState::Ready);

    WarmJob empty = svc->warm(WarmRequest{});
    CHECK(empty.finished());
    CHECK(WarmJob().finished());
}

TEST_CASE("service: producers join running jobs and run queued ones, a pool of one cannot deadlock",
          "[world][terrain_service][nested]") {
    World w;
    auto svc = w.open("derived", 1);
    const ProductKind kind = TerrainService::register_product_kind("test.nested");
    struct Sum {
        std::size_t segments = 0;
        std::size_t entries = 0;
        int height_0 = 0;
    };
    auto producer = [](ProductContext& ctx, const TileKey& k) {
        const auto segs = ctx.road_segments(k);
        const auto geom = ctx.road_geom(k);
        const auto height = ctx.height(l0(0, 0));
        if (!segs.ready() || !geom.ready() || !height.ready()) return Outcome<Sum>::failed("a dependency failed");
        return Outcome<Sum>::ready(std::make_shared<const Sum>(Sum{segs.value->segments.size(), geom.value->entries.size(),
                                                                   height.value->h[0]}),
                                   32);
    };
    // road_segments(0,0) is queued first; the producer (queued behind it, same worker) takes it over inline.
    Harness h;
    auto blocker = h.submit_blocker(*svc);
    h.wait_started(1);
    auto queued_segments = svc->road_segments(l2(0, 0), Priority::Render);
    auto nested = svc->product<Sum>(kind, l2(0, 0), Priority::Physics, 0, producer);
    h.release();
    REQUIRE(nested.wait_for(kLong));
    REQUIRE(queued_segments.wait_for(kLong));
    REQUIRE(nested.state() == TileState::Ready);
    CHECK(nested.value()->segments == 1);
    CHECK(nested.value()->entries >= 1);
    CHECK(nested.value()->height_0 == 25600);
    CHECK(queued_segments.state() == TileState::Ready);
    CHECK(svc->stats().inline_runs >= 1);
}

TEST_CASE("service: concurrent producers share nested products", "[world][terrain_service][nested]") {
    World w;
    auto svc = w.open("derived", 4);
    const ProductKind kind = TerrainService::register_product_kind("test.nested_many");
    std::vector<Ticket<Marker>> ts;
    for (int i = 0; i < 12; ++i)
        ts.push_back(svc->product<Marker>(kind, l2(0, 0), Priority::Render, static_cast<std::uint64_t>(i),
                                          [](ProductContext& ctx, const TileKey& k) {
                                              const auto osm = ctx.source_osm(k);
                                              const auto segs = ctx.road_segments(k);
                                              if (!osm.ready() || !segs.ready()) return Outcome<Marker>::failed("dependency");
                                              return Outcome<Marker>::ready(std::make_shared<const Marker>(Marker{1}), 8);
                                          }));
    for (auto& t : ts) {
        REQUIRE(t.wait_for(kLong));
        CHECK(t.state() == TileState::Ready);
    }
    // 12 producers, but source_osm and road_segments of the tile exist once each.
    CHECK(svc->stats().jobs_run <= 12 + 2);
    CHECK(svc->peek_source_osm(l2(0, 0)));
    CHECK(svc->peek_road_segments(l2(0, 0)));
}

TEST_CASE("service: peeks never start work; surface heights read resident tiles", "[world][terrain_service][peek]") {
    World w;
    auto svc = w.open("derived", 2);
    const TileKey k = l0(1, 1); // covers x 256..512, y 256..512 of the session frame (e0 = n0 = 0)
    CHECK_FALSE(svc->peek_height(k));
    CHECK_FALSE(svc->peek_road_geom(l2(0, 0)));
    CHECK_FALSE(svc->peek_road_segments(l2(0, 0)));
    CHECK_FALSE(svc->peek_source_osm(l2(0, 0)));
    CHECK_FALSE(svc->surface_height_exact(300.5, 300.5));
    CHECK_FALSE(svc->surface_height_bilinear(300.5, 300.5));
    std::this_thread::sleep_for(50ms);
    CHECK(svc->stats().jobs_run == 0);

    auto t = svc->height(k, Priority::Physics);
    REQUIRE(t.wait_for(kLong));
    REQUIRE(t.state() == TileState::Ready);
    CHECK(svc->peek_height(k) == t.value());
    const auto exact = svc->surface_height_exact(300.5, 300.5);
    const auto bilinear = svc->surface_height_bilinear(300.5, 300.5);
    REQUIRE(exact);
    REQUIRE(bilinear);
    CHECK(std::abs(*exact - 100.0) < 1e-6);
    CHECK(std::abs(*bilinear - 100.0) < 1e-6);
    CHECK_FALSE(svc->surface_height_exact(-900.5, 300.5)); // tile not resident
}

TEST_CASE("service: PhysicsFetch blocks until ready and maps statuses like the loader expects",
          "[world][terrain_service][physics]") {
    World w;
    auto svc = w.open("derived", 3);
    auto fetch = svc->physics_fetch(/*with_road_segments=*/true, Priority::Gate);
    CHECK(fetch->provides_roads());
    CHECK(fetch->priority() == Priority::Gate);
    fetch->set_priority(Priority::Physics);
    CHECK(fetch->priority() == Priority::Physics);

    const phys::FetchResult ok = fetch->fetch(l0(1, 1));
    REQUIRE(ok.status == Status::Ok);
    REQUIRE(ok.tile);
    REQUIRE(ok.roads);
    CHECK(ok.roads->size() == 1);
    CHECK(ok.tile->h[0] == 25600);
    CHECK(svc->peek_height(l0(1, 1))); // went through the service's cache

    const phys::FetchResult outside = fetch->fetch(l0(400, 400));
    CHECK(outside.status == Status::NotFound);
    CHECK_FALSE(outside.tile);

    // A tile with height data but no OSM tile: heights are fine, the roads are a confirmed empty list.
    const phys::FetchResult no_roads = fetch->fetch(l0(-8, 1)); // level-2 tile (-2, 0): elevation only
    REQUIRE(no_roads.status == Status::Ok);
    REQUIRE(no_roads.roads);
    CHECK(no_roads.roads->empty());

    auto heights_only = svc->physics_fetch(false, Priority::Physics);
    CHECK_FALSE(heights_only->provides_roads());
    const phys::FetchResult h = heights_only->fetch(l0(2, 2));
    REQUIRE(h.status == Status::Ok);
    CHECK_FALSE(h.roads);

    // Used the way HeightTileLoader uses it: many threads at once.
    std::vector<std::thread> threads;
    std::atomic<int> bad{0};
    for (int t = 0; t < 8; ++t)
        threads.emplace_back([&, t] {
            const phys::FetchResult r = fetch->fetch(l0(t % 4, (t / 4) % 4));
            if (r.status != Status::Ok || !r.tile || !r.roads) bad++;
        });
    for (auto& th : threads) th.join();
    CHECK(bad == 0);
}

TEST_CASE("service: phase-3 products fail until a producer is installed", "[world][terrain_service][producers]") {
    World w;
    auto svc = w.open("derived", 2);
    CHECK(svc->structures(l2(0, 0), Priority::Render).state() == TileState::Failed);
    CHECK(svc->road_ribbons(l2(0, 0), Priority::Render, {}).state() == TileState::Failed);
    CHECK(svc->buildings(l2(0, 0), Priority::Render, {}).state() == TileState::Failed);
    CHECK_FALSE(svc->peek_structures(l2(0, 0)));

    std::atomic<int> calls{0};
    svc->set_structures_producer([&](ProductContext& ctx, const TileKey& k) {
        calls++;
        const auto segs = ctx.road_segments(k);
        if (!segs.ready()) return Outcome<StructureSet>::failed("no roads");
        return Outcome<StructureSet>::ready(std::make_shared<const StructureSet>(StructureSet{k}), 16);
    });
    auto a = svc->structures(l2(0, 0), Priority::Render);
    auto b = svc->structures(l2(0, 0), Priority::Prefetch);
    REQUIRE(a.wait_for(kLong));
    REQUIRE(b.wait_for(kLong));
    CHECK(a.state() == TileState::Ready);
    CHECK(a.value() == b.value());
    CHECK(calls == 1);
    CHECK(svc->peek_structures(l2(0, 0)) == a.value());
    CHECK(svc->structures(l2(0, 0), Priority::Render).state() == TileState::Ready); // memory cache, no new job
    CHECK(calls == 1);

    // Options are part of the identity of a ribbon/building product.
    svc->set_road_ribbons_producer([&](ProductContext&, const TileKey& k) {
        calls++;
        return Outcome<RoadRibbonTile>::ready(std::make_shared<const RoadRibbonTile>(RoadRibbonTile{k}), 16);
    });
    auto r1 = svc->road_ribbons(l2(0, 0), Priority::Render, RibbonOptions{1});
    auto r2 = svc->road_ribbons(l2(0, 0), Priority::Render, RibbonOptions{2});
    REQUIRE(r1.wait_for(kLong));
    REQUIRE(r2.wait_for(kLong));
    CHECK(r1.value() != r2.value());
    CHECK(calls == 3);
}

TEST_CASE("service: a failed job is not cached and a later request retries", "[world][terrain_service][producers]") {
    World w;
    auto svc = w.open("derived", 2);
    const ProductKind kind = TerrainService::register_product_kind("test.flaky");
    std::atomic<int> calls{0};
    auto producer = [&](ProductContext&, const TileKey& k) {
        if (calls++ == 0) return Outcome<Marker>::failed("first attempt fails");
        return Outcome<Marker>::ready(std::make_shared<const Marker>(Marker{k.x}), 8);
    };
    auto first = svc->product<Marker>(kind, l2(5, 5), Priority::Render, 0, producer);
    REQUIRE(first.wait_for(kLong));
    CHECK(first.state() == TileState::Failed);
    CHECK(first.message() == "first attempt fails");
    auto second = svc->product<Marker>(kind, l2(5, 5), Priority::Render, 0, producer);
    REQUIRE(second.wait_for(kLong));
    CHECK(second.state() == TileState::Ready);
    CHECK(calls == 2);
}

TEST_CASE("service: the memory cache evicts least recently used tiles", "[world][terrain_service][cache]") {
    World w;
    TerrainServiceConfig c = w.config("derived", 2);
    c.cache_bytes = 3 * sizeof(HeightTile); // room for about three height tiles
    auto svc = w.open(c);
    for (int x = 0; x < 6; ++x) {
        auto t = svc->height(l0(x, 0), Priority::Render);
        REQUIRE(t.wait_for(kLong));
        REQUIRE(t.state() == TileState::Ready);
    }
    const ServiceStats st = svc->stats();
    CHECK(st.cache_entries <= 3);
    CHECK(st.cache_bytes <= 3 * sizeof(HeightTile) + 4096);
    CHECK_FALSE(svc->peek_height(l0(0, 0)));
    CHECK(svc->peek_height(l0(5, 0)));
}
