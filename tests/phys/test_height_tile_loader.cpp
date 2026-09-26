// tests/phys/test_height_tile_loader.cpp - HeightTileLoader, DelayedFetch and
// TransportHeightTileFetch (PLAN.md R2.2 section 1a/4a "loader" bullets):
// 404 -> Absent, 503 retried then Ready, 500 x max_attempts -> Failed,
// 400/410 -> Failed at once, dedup against in-flight and done, request()
// replacing the want-list, poll() sorted by key, and the destructor joining
// with requests queued, in flight and in a backoff wait.
//
// Everything runs against a scripted in-memory IHeightTileFetch (no network,
// no store files) and must stay clean under linux-tsan. Catch2 assertions
// run ONLY on the test's own thread (vault TOOL-031): the scripted fetch,
// called on loader worker threads, just records into mutex-guarded state
// that the test reads after the fact.
#include "g2m/phys/height_tile_loader.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/raster.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/tile_container.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <variant>
#include <vector>

using g2m::Status;
using g2m::TileKey;
using g2m::geo::Hemisphere;
using g2m::geo::UtmZone;
using g2m::phys::DelayedFetch;
using g2m::phys::FetchResult;
using g2m::phys::HeightTileLoader;
using g2m::phys::IHeightTileFetch;
using g2m::phys::LoadedHeightTile;
using g2m::phys::LoaderConfig;
using g2m::phys::LoadOutcome;
using namespace std::chrono_literals;

namespace {
constexpr UtmZone k32N{32, Hemisphere::North};

TileKey key_at(std::int32_t x, std::int32_t y) {
    TileKey k;
    k.zone = k32N;
    k.level = 0;
    k.x = x;
    k.y = y;
    return k;
}

std::shared_ptr<const g2m::HeightTile> make_tile(const TileKey& key, std::int32_t fill_value) {
    auto tile = std::make_shared<g2m::HeightTile>();
    tile->key = key;
    tile->h.fill(fill_value);
    return tile;
}

// Scripted, thread-safe fetch. Each key has a queue of statuses, consumed
// one per call; once empty (or never scripted) the key answers 200 with a
// tile filled with its own x. Optionally every call blocks until open().
class ScriptedFetch final : public IHeightTileFetch {
public:
    explicit ScriptedFetch(bool start_closed = false) : open_(!start_closed) {}

    void script(const TileKey& key, std::vector<Status> statuses) {
        std::lock_guard<std::mutex> lk(m_);
        scripts_[key.packed()] = std::move(statuses);
    }
    void open() {
        {
            std::lock_guard<std::mutex> lk(m_);
            open_ = true;
        }
        cv_.notify_all();
    }
    int calls(const TileKey& key) const {
        std::lock_guard<std::mutex> lk(m_);
        const auto it = calls_.find(key.packed());
        return it == calls_.end() ? 0 : it->second;
    }
    int total_calls() const {
        std::lock_guard<std::mutex> lk(m_);
        int n = 0;
        for (const auto& [k, c] : calls_) n += c;
        return n;
    }
    // Calls that have entered fetch() (blocked or not).
    int entered() const {
        std::lock_guard<std::mutex> lk(m_);
        return entered_;
    }

    FetchResult fetch(const TileKey& key) override {
        std::unique_lock<std::mutex> lk(m_);
        ++entered_;
        cv_.wait(lk, [this] { return open_; });
        ++calls_[key.packed()];
        auto it = scripts_.find(key.packed());
        if (it != scripts_.end() && !it->second.empty()) {
            const Status s = it->second.front();
            it->second.erase(it->second.begin());
            if (s != Status::Ok) return FetchResult{s, nullptr};
        }
        lk.unlock();
        return FetchResult{Status::Ok, make_tile(key, key.x)};
    }

private:
    mutable std::mutex m_;
    std::condition_variable cv_;
    bool open_;
    int entered_ = 0;
    std::map<std::uint64_t, std::vector<Status>> scripts_;
    std::map<std::uint64_t, int> calls_;
};

// Polls until `want` completions have arrived in total (or 20 s pass).
std::vector<LoadedHeightTile> poll_until(HeightTileLoader& loader, std::size_t want) {
    std::vector<LoadedHeightTile> out;
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (out.size() < want && std::chrono::steady_clock::now() < deadline) {
        if (loader.poll(out) == 0) std::this_thread::sleep_for(1ms);
    }
    return out;
}

template <class Pred>
bool wait_for(Pred pred) {
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (!pred()) {
        if (std::chrono::steady_clock::now() >= deadline) return false;
        std::this_thread::sleep_for(1ms);
    }
    return true;
}

LoaderConfig fast_config(int workers = 2) {
    LoaderConfig c;
    c.workers = workers;
    c.max_attempts = 4;
    c.backoff = 1ms;
    return c;
}
} // namespace

TEST_CASE("HeightTileLoader: 200 -> Ready with the tile", "[phys][loader]") {
    auto fetch = std::make_shared<ScriptedFetch>();
    HeightTileLoader loader(fetch, fast_config());
    const TileKey k = key_at(7, 3);
    const std::vector<TileKey> want{k};
    loader.request(want);
    const auto out = poll_until(loader, 1);
    REQUIRE(out.size() == 1);
    CHECK(out[0].key == k);
    CHECK(out[0].outcome == LoadOutcome::Ready);
    REQUIRE(out[0].tile);
    CHECK(out[0].tile->h[0] == 7);
    CHECK(out[0].attempts == 1);
    const auto st = loader.stats();
    CHECK(st.ready == 1);
    CHECK(st.requested == 1);
    CHECK(st.retries == 0);
}

TEST_CASE("HeightTileLoader: 404 -> Absent, final, one attempt", "[phys][loader]") {
    auto fetch = std::make_shared<ScriptedFetch>();
    const TileKey k = key_at(1, 1);
    fetch->script(k, {Status::NotFound, Status::NotFound, Status::NotFound, Status::NotFound});
    HeightTileLoader loader(fetch, fast_config());
    const std::vector<TileKey> want{k};
    loader.request(want);
    const auto out = poll_until(loader, 1);
    REQUIRE(out.size() == 1);
    CHECK(out[0].outcome == LoadOutcome::Absent);
    CHECK_FALSE(out[0].tile);
    CHECK(out[0].attempts == 1);
    CHECK(fetch->calls(k) == 1);
    CHECK(loader.stats().absent == 1);
}

TEST_CASE("HeightTileLoader: 503 retried then Ready", "[phys][loader]") {
    auto fetch = std::make_shared<ScriptedFetch>();
    const TileKey k = key_at(2, 5);
    fetch->script(k, {Status::Unavailable, Status::Unavailable});
    HeightTileLoader loader(fetch, fast_config());
    const std::vector<TileKey> want{k};
    loader.request(want);
    const auto out = poll_until(loader, 1);
    REQUIRE(out.size() == 1);
    CHECK(out[0].outcome == LoadOutcome::Ready);
    REQUIRE(out[0].tile);
    CHECK(out[0].attempts == 3);
    CHECK(fetch->calls(k) == 3);
    const auto st = loader.stats();
    CHECK(st.retries == 2);
    CHECK(st.ready == 1);
}

TEST_CASE("HeightTileLoader: 500 x max_attempts -> Failed, never Absent", "[phys][loader]") {
    auto fetch = std::make_shared<ScriptedFetch>();
    const TileKey k = key_at(-3, 9);
    fetch->script(k, {Status::Internal, Status::Internal, Status::Internal, Status::Internal, Status::Ok});
    HeightTileLoader loader(fetch, fast_config());
    const std::vector<TileKey> want{k};
    loader.request(want);
    const auto out = poll_until(loader, 1);
    REQUIRE(out.size() == 1);
    CHECK(out[0].outcome == LoadOutcome::Failed);
    CHECK_FALSE(out[0].tile);
    CHECK(out[0].attempts == 4);
    CHECK(out[0].last_status == Status::Internal);
    CHECK(fetch->calls(k) == 4); // exactly max_attempts, the scripted 5th (Ok) never reached
    const auto st = loader.stats();
    CHECK(st.failed == 1);
    CHECK(st.absent == 0);
    CHECK(st.retries == 3);
}

TEST_CASE("HeightTileLoader: 400 and 410 -> Failed at once", "[phys][loader]") {
    auto fetch = std::make_shared<ScriptedFetch>();
    const TileKey bad = key_at(0, 1);
    const TileKey gone = key_at(0, 2);
    fetch->script(bad, {Status::BadRequest, Status::Ok});
    fetch->script(gone, {Status::Gone, Status::Ok});
    HeightTileLoader loader(fetch, fast_config());
    const std::vector<TileKey> want{bad, gone};
    loader.request(want);
    const auto out = poll_until(loader, 2);
    REQUIRE(out.size() == 2);
    for (const auto& d : out) {
        CHECK(d.outcome == LoadOutcome::Failed);
        CHECK(d.attempts == 1);
    }
    CHECK(fetch->calls(bad) == 1);
    CHECK(fetch->calls(gone) == 1);
    CHECK(loader.stats().retries == 0);
}

TEST_CASE("HeightTileLoader: dedup against in-flight and done; forgotten after poll", "[phys][loader]") {
    auto fetch = std::make_shared<ScriptedFetch>(/*start_closed=*/true);
    HeightTileLoader loader(fetch, fast_config(1));
    const TileKey a = key_at(10, 10);
    const TileKey b = key_at(11, 10);

    const std::vector<TileKey> want_a{a};
    loader.request(want_a);
    REQUIRE(wait_for([&] { return fetch->entered() == 1; })); // a is in flight, blocked
    CHECK(loader.stats().inflight == 1);

    // a again (in flight -> not queued), b once despite the duplicate.
    const std::vector<TileKey> want_ab{a, b, b, a};
    loader.request(want_ab);
    CHECK(loader.stats().queued == 1);

    fetch->open();
    auto out = poll_until(loader, 2);
    REQUIRE(out.size() == 2);
    CHECK(out[0].key == a); // sorted by packed(): same y, x 10 < 11
    CHECK(out[1].key == b);
    CHECK(fetch->calls(a) == 1);
    CHECK(fetch->calls(b) == 1);

    // Completed but not yet polled -> not fetched again.
    const std::vector<TileKey> want_b{b};
    loader.request(want_b);
    REQUIRE(wait_for([&] { return loader.stats().ready == 3; }));
    loader.request(want_b); // b is now done-not-polled: dedup against done
    std::this_thread::sleep_for(20ms);
    CHECK(fetch->calls(b) == 2);
    out.clear();
    CHECK(loader.poll(out) == 1);
    CHECK(fetch->calls(b) == 2);

    // Polled -> forgotten: a later request fetches again (e.g. after eviction).
    loader.request(want_b);
    out = poll_until(loader, 1);
    REQUIRE(out.size() == 1);
    CHECK(fetch->calls(b) == 3);
}

TEST_CASE("HeightTileLoader: request replaces the want-list", "[phys][loader]") {
    auto fetch = std::make_shared<ScriptedFetch>(/*start_closed=*/true);
    HeightTileLoader loader(fetch, fast_config(1));
    const TileKey a = key_at(1, 0), b = key_at(2, 0), c = key_at(3, 0), d = key_at(4, 0);
    const std::vector<TileKey> first{a, b, c};
    loader.request(first);
    REQUIRE(wait_for([&] { return fetch->entered() == 1; })); // a taken, b/c queued
    CHECK(loader.stats().queued == 2);
    const std::vector<TileKey> second{d};
    loader.request(second); // drops b and c
    CHECK(loader.stats().queued == 1);
    fetch->open();
    const auto out = poll_until(loader, 2);
    REQUIRE(out.size() == 2);
    CHECK(out[0].key == a);
    CHECK(out[1].key == d);
    std::this_thread::sleep_for(20ms);
    CHECK(fetch->calls(b) == 0);
    CHECK(fetch->calls(c) == 0);
    CHECK(fetch->total_calls() == 2);
}

TEST_CASE("HeightTileLoader: poll output is sorted by key whatever the arrival order", "[phys][loader]") {
    // DelayedFetch jitter makes completion order differ from request order.
    auto inner = std::make_shared<ScriptedFetch>();
    auto fetch = std::make_shared<DelayedFetch>(inner, 2ms, 12345u);
    HeightTileLoader loader(fetch, fast_config(4));
    std::vector<TileKey> want;
    for (std::int32_t y = 3; y >= -3; --y) {
        for (std::int32_t x = 3; x >= -3; --x) want.push_back(key_at(x, y));
    }
    loader.request(want);
    std::vector<LoadedHeightTile> all;
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    std::vector<std::vector<std::uint64_t>> batches;
    while (all.size() < want.size() && std::chrono::steady_clock::now() < deadline) {
        std::vector<LoadedHeightTile> batch;
        if (loader.poll(batch) == 0) {
            std::this_thread::sleep_for(1ms);
            continue;
        }
        std::vector<std::uint64_t> packed;
        for (const auto& d : batch) packed.push_back(d.key.packed());
        batches.push_back(packed);
        for (auto& d : batch) all.push_back(std::move(d));
    }
    REQUIRE(all.size() == want.size());
    for (const auto& p : batches) CHECK(std::is_sorted(p.begin(), p.end()));
    std::vector<std::uint64_t> got;
    for (const auto& d : all) got.push_back(d.key.packed());
    std::sort(got.begin(), got.end());
    CHECK(std::adjacent_find(got.begin(), got.end()) == got.end()); // each key exactly once
    for (const auto& k : want) CHECK(inner->calls(k) == 1);
}

TEST_CASE("HeightTileLoader: destructor joins with requests queued and in flight", "[phys][loader]") {
    auto inner = std::make_shared<ScriptedFetch>();
    auto fetch = std::make_shared<DelayedFetch>(inner, 30ms, 7u);
    std::vector<TileKey> want;
    for (std::int32_t x = 0; x < 64; ++x) want.push_back(key_at(x, 0));
    const auto t0 = std::chrono::steady_clock::now();
    {
        HeightTileLoader loader(fetch, fast_config(2));
        loader.request(want);
        std::this_thread::sleep_for(5ms); // both workers inside a 30-60 ms fetch
    }
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    CHECK(elapsed < 2s);                  // 64 * 45 ms / 2 workers would be ~1.4 s+
    CHECK(inner->total_calls() <= 4);     // queued keys were dropped, not fetched
}

TEST_CASE("HeightTileLoader: destructor cuts a retry backoff short", "[phys][loader]") {
    auto fetch = std::make_shared<ScriptedFetch>();
    const TileKey k = key_at(5, 5);
    fetch->script(k, {Status::Unavailable, Status::Unavailable, Status::Unavailable, Status::Unavailable});
    LoaderConfig config;
    config.workers = 1;
    config.max_attempts = 4;
    config.backoff = 60s;
    const auto t0 = std::chrono::steady_clock::now();
    {
        HeightTileLoader loader(fetch, config);
        const std::vector<TileKey> want{k};
        loader.request(want);
        REQUIRE(wait_for([&] { return fetch->calls(k) == 1; }));
    }
    CHECK(std::chrono::steady_clock::now() - t0 < 10s);
    CHECK(fetch->calls(k) == 1);
}

TEST_CASE("HeightTileLoader: rejects invalid configuration", "[phys][loader]") {
    auto fetch = std::make_shared<ScriptedFetch>();
    LoaderConfig c;
    c.workers = 0;
    CHECK_THROWS_AS(HeightTileLoader(fetch, c), std::invalid_argument);
    c = LoaderConfig{};
    c.max_attempts = 0;
    CHECK_THROWS_AS(HeightTileLoader(fetch, c), std::invalid_argument);
    CHECK_THROWS_AS(HeightTileLoader(nullptr, LoaderConfig{}), std::invalid_argument);
}

TEST_CASE("DelayedFetch: deterministic per-key jitter in [base, 2*base]", "[phys][loader]") {
    auto inner = std::make_shared<ScriptedFetch>();
    const DelayedFetch f1(inner, 100ms, 42u);
    const DelayedFetch f2(inner, 100ms, 42u);
    const DelayedFetch f3(inner, 100ms, 43u);
    const DelayedFetch zero(inner, 0ms, 42u);
    bool any_differs_by_key = false;
    bool any_differs_by_seed = false;
    const auto first = f1.delay_for(key_at(0, 0));
    for (std::int32_t x = -20; x < 20; ++x) {
        const TileKey k = key_at(x, x * 3);
        const auto d = f1.delay_for(k);
        CHECK(d >= 100ms);
        CHECK(d <= 200ms);
        CHECK(d == f2.delay_for(k));
        CHECK(zero.delay_for(k) == 0ms);
        any_differs_by_key = any_differs_by_key || d != first;
        any_differs_by_seed = any_differs_by_seed || d != f3.delay_for(k);
    }
    CHECK(any_differs_by_key);
    CHECK(any_differs_by_seed);
}

// ---- TransportHeightTileFetch over a scripted ITransport ----

namespace {
// Serves one pre-built container (or a status) for any tile request.
class FakeTransport final : public g2m::ITransport {
public:
    g2m::Response send(const g2m::Request& request) override {
        std::lock_guard<std::mutex> lk(m_);
        ++sends_;
        if (const auto* tr = std::get_if<g2m::TileRequest>(&request)) last_ = *tr;
        g2m::TileResponse resp;
        resp.meta.status = status_;
        if (status_ == Status::Ok) resp.container = container_;
        return g2m::Response{resp};
    }
    void set(Status s, std::vector<std::uint8_t> container = {}) {
        std::lock_guard<std::mutex> lk(m_);
        status_ = s;
        container_ = std::move(container);
    }
    g2m::TileRequest last() const {
        std::lock_guard<std::mutex> lk(m_);
        return last_;
    }

private:
    mutable std::mutex m_;
    Status status_ = Status::Ok;
    std::vector<std::uint8_t> container_;
    g2m::TileRequest last_;
    int sends_ = 0;
};

std::vector<std::uint8_t> make_container(const TileKey& header_key, const TileKey& body_key, std::int32_t offset,
                                         std::string layer = std::string(g2m::kTerrainHeightLayer)) {
    auto tile = std::make_unique<g2m::HeightTile>(); // heap: 256 KiB (vault TOOL-039)
    tile->key = body_key;
    for (std::size_t i = 0; i < tile->h.size(); ++i) tile->h[i] = static_cast<std::int32_t>(i % 1000) * 256;
    tile->h[5] = g2m::kHeightNoData;
    const auto body = g2m::encode_height_tile(*tile);
    const auto body_bytes = g2m::encode_body(*body);
    g2m::TileHeader header;
    header.layer = std::move(layer);
    header.layer_version = 1;
    header.key = header_key;
    header.height_offset = offset;
    auto container = g2m::assemble_container(header, *body_bytes);
    return container ? *container : std::vector<std::uint8_t>{};
}
} // namespace

TEST_CASE("TransportHeightTileFetch: decodes a 200 container, passes other statuses through", "[phys][loader]") {
    FakeTransport transport;
    const TileKey k = key_at(1850, 21720);
    g2m::ReleaseId rid{};
    rid[0] = 0xAB;
    g2m::phys::TransportHeightTileFetch fetch(transport, rid);

    auto container = make_container(k, k, 0);
    REQUIRE_FALSE(container.empty());
    transport.set(Status::Ok, container);
    FetchResult r = fetch.fetch(k);
    REQUIRE(r.status == Status::Ok);
    REQUIRE(r.tile);
    CHECK(r.tile->key == k);
    CHECK(r.tile->h[3] == 3 * 256);
    CHECK(r.tile->h[5] == g2m::kHeightNoData);
    CHECK(r.tile->has_nodata);
    CHECK(transport.last().rid == rid);
    CHECK(transport.last().layer == std::string(g2m::kTerrainHeightLayer));
    CHECK(transport.last().key == k);
    CHECK_FALSE(transport.last().if_none_match.has_value());

    SECTION("height_offset is applied to every non-NoData sample") {
        transport.set(Status::Ok, make_container(k, k, 512));
        r = fetch.fetch(k);
        REQUIRE(r.status == Status::Ok);
        CHECK(r.tile->h[3] == 3 * 256 + 512);
        CHECK(r.tile->h[5] == g2m::kHeightNoData);
    }
    SECTION("statuses pass through without a tile") {
        for (Status s : {Status::NotFound, Status::Unavailable, Status::Internal, Status::BadRequest, Status::Gone}) {
            transport.set(s);
            r = fetch.fetch(k);
            CHECK(r.status == s);
            CHECK_FALSE(r.tile);
        }
    }
    SECTION("a corrupt container is Internal") {
        container[container.size() - 1] ^= 0x5A; // body SHA-256 no longer matches
        transport.set(Status::Ok, container);
        r = fetch.fetch(k);
        CHECK(r.status == Status::Internal);
        CHECK_FALSE(r.tile);
    }
    SECTION("a container for another key or layer is Internal") {
        const TileKey other = key_at(1851, 21720);
        transport.set(Status::Ok, make_container(other, other, 0));
        CHECK(fetch.fetch(k).status == Status::Internal);
        transport.set(Status::Ok, make_container(k, k, 0, std::string(g2m::kElevBaseLayer)));
        CHECK(fetch.fetch(k).status == Status::Internal);
    }
}
