// g2m::world tile decode and fetch-cache helpers (include/g2m/world/
// tile_fetch.h; terrain migration plan phase 1, vault G2M-020).
//
// Parity with racing_game a8a1b13 (world_terrain.cpp): the pinned hashes below
// were produced by that code's decode_height_tile_container and
// decode_road_segments_response (extracted verbatim, run over the cases in
// world_test_data.h: decode_cases() / road_response_cases()). The cache cases
// are ported from the game's tests/unit/test_world_terrain.cpp. A second
// parity line ties the decoder to g2m::phys::TransportHeightTileFetch, which
// implements the same arithmetic in geo2map.
#include "g2m/world/tile_fetch.h"

#include "world_test_data.h"

#include "g2m/phys/height_tile_loader.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace g2m_world_test;
using g2m::world::decode_height_tile_container;
using g2m::world::decode_road_segments_response;
using g2m::world::fetch_height_tile;
using g2m::world::fetch_height_tile_cached;
using g2m::world::fetch_road_segments;
using g2m::world::fetch_road_segments_cached;
using g2m::world::HeightTileFetchResult;
using g2m::world::RoadFetchStatus;
using g2m::world::RoadSegmentsResult;

namespace {

// A transport answering every request with a fixed response; records them.
class FixedTransport final : public g2m::ITransport {
public:
    explicit FixedTransport(g2m::Response response) : response_(std::move(response)) {}
    g2m::Response send(const g2m::Request& request) override {
        requests.push_back(request);
        return response_;
    }
    std::vector<g2m::Request> requests;

private:
    g2m::Response response_;
};

struct DecodePin {
    const char* name;
    g2m::Status status;
    std::uint64_t tile_hash; // 0 = null tile
};

// Pins from racing_game a8a1b13's decode_height_tile_container (refgen output).
constexpr DecodePin kDecodePins[] = {
    {"offset_zero", g2m::Status::Ok, 0x2c72b981996527d9ull},
    {"offset_plus", g2m::Status::Ok, 0x4a37a0f82d55b922ull},
    {"offset_minus", g2m::Status::Ok, 0x18ae67dd3f0b4825ull},
    {"offset_int_max", g2m::Status::Internal, 0},
    {"nodata_kept", g2m::Status::Ok, 0x5f55af5cfe5f1195ull},
    {"offset_underflow", g2m::Status::Internal, 0},
    {"wrong_layer", g2m::Status::Internal, 0},
    {"wrong_key", g2m::Status::Internal, 0},
    {"expect_other_layer", g2m::Status::Internal, 0},
    {"truncated", g2m::Status::Internal, 0},
    {"corrupt_body", g2m::Status::Internal, 0},
    {"garbage", g2m::Status::Internal, 0},
    {"empty", g2m::Status::Internal, 0},
};

const DecodePin& pin_for(const std::string& name) {
    for (const DecodePin& p : kDecodePins) {
        if (name == p.name) return p;
    }
    throw std::runtime_error("no pin for case " + name);
}

} // namespace

TEST_CASE("decode_height_tile_container matches racing_game's decoder on every case", "[world][tile_fetch]") {
    const std::vector<DecodeCase> cases = decode_cases();
    REQUIRE(cases.size() == std::size(kDecodePins));
    for (const DecodeCase& c : cases) {
        INFO("case " << c.name);
        const DecodePin& pin = pin_for(c.name);
        const HeightTileFetchResult r = decode_height_tile_container(c.container, c.expect_layer, c.expect_key);
        CHECK(r.status == pin.status);
        if (pin.tile_hash != 0) {
            REQUIRE(r.tile != nullptr);
            CHECK(hash_height_tile(*r.tile) == pin.tile_hash);
        } else {
            CHECK(r.tile == nullptr);
        }
    }
}

TEST_CASE("decode_height_tile_container applies height_offset to every non-NoData sample", "[world][tile_fetch]") {
    // Independent of the pins: the arithmetic itself, sample by sample.
    const auto pattern = make_pattern_tile();
    const std::string layer(g2m::kTerrainHeightLayer);

    const HeightTileFetchResult plus =
        decode_height_tile_container(make_height_container(*pattern, 12345), layer, kOffsetTestKey);
    REQUIRE(plus.tile != nullptr);
    for (std::size_t i = 0; i < pattern->h.size(); ++i) {
        REQUIRE(plus.tile->h[i] == pattern->h[i] + 12345);
    }
    CHECK(plus.tile->key == kOffsetTestKey);
    CHECK_FALSE(plus.tile->has_nodata);

    auto nodata = make_pattern_tile();
    nodata->h[0] = g2m::kHeightNoData;
    nodata->h[300] = g2m::kHeightNoData;
    nodata->has_nodata = true;
    const HeightTileFetchResult kept =
        decode_height_tile_container(make_height_container(*nodata, -500), layer, kOffsetTestKey);
    REQUIRE(kept.tile != nullptr);
    CHECK(kept.tile->h[0] == g2m::kHeightNoData);
    CHECK(kept.tile->h[300] == g2m::kHeightNoData);
    CHECK(kept.tile->h[1] == nodata->h[1] - 500);
    CHECK(kept.tile->has_nodata);
}

TEST_CASE("decode_height_tile_container agrees with phys::TransportHeightTileFetch", "[world][tile_fetch]") {
    // The two implementations of the same arithmetic: same status and same
    // tile for every case (a failed decode is Internal in both).
    for (const DecodeCase& c : decode_cases()) {
        INFO("case " << c.name);
        FixedTransport transport(make_tile_response(g2m::Status::Ok, c.container));
        g2m::phys::TransportHeightTileFetch phys_fetch(transport, g2m::ReleaseId{}, c.expect_layer);
        const g2m::phys::FetchResult theirs = phys_fetch.fetch(c.expect_key);
        const HeightTileFetchResult ours = decode_height_tile_container(c.container, c.expect_layer, c.expect_key);
        CHECK(ours.status == theirs.status);
        REQUIRE((ours.tile != nullptr) == (theirs.tile != nullptr));
        if (ours.tile) {
            CHECK(hash_height_tile(*ours.tile) == hash_height_tile(*theirs.tile));
        }
    }
}

TEST_CASE("fetch_height_tile sends the tile request and classifies the response", "[world][tile_fetch]") {
    const auto pattern = make_pattern_tile();
    const std::string layer(g2m::kTerrainHeightLayer);
    g2m::ReleaseId rid{};
    rid[0] = 0xAB;
    rid[31] = 0xCD;

    SECTION("Ok decodes and sends {rid, layer, key}") {
        FixedTransport t(make_tile_response(g2m::Status::Ok, make_height_container(*pattern, 256)));
        const HeightTileFetchResult r = fetch_height_tile(t, rid, layer, kOffsetTestKey);
        REQUIRE(r.status == g2m::Status::Ok);
        REQUIRE(r.tile != nullptr);
        CHECK(r.tile->h[0] == pattern->h[0] + 256);
        REQUIRE(t.requests.size() == 1);
        const auto* req = std::get_if<g2m::TileRequest>(&t.requests[0]);
        REQUIRE(req != nullptr);
        CHECK(req->rid == rid);
        CHECK(req->layer == layer);
        CHECK(req->key == kOffsetTestKey);
    }
    SECTION("a non-Ok status passes straight through with a null tile") {
        FixedTransport t(make_tile_response(g2m::Status::NotFound, {}, "outside coverage"));
        const HeightTileFetchResult r = fetch_height_tile(t, rid, layer, kOffsetTestKey);
        CHECK(r.status == g2m::Status::NotFound);
        CHECK(r.tile == nullptr);
    }
    SECTION("a non-tile response is Internal") {
        FixedTransport t(g2m::Response{g2m::PackResponse{}});
        const HeightTileFetchResult r = fetch_height_tile(t, rid, layer, kOffsetTestKey);
        CHECK(r.status == g2m::Status::Internal);
        CHECK(r.tile == nullptr);
    }
    SECTION("an Ok response with a corrupt container is Internal") {
        std::vector<std::uint8_t> bad = make_height_container(*pattern, 0);
        bad.back() ^= 0x11;
        FixedTransport t(make_tile_response(g2m::Status::Ok, std::move(bad)));
        const HeightTileFetchResult r = fetch_height_tile(t, rid, layer, kOffsetTestKey);
        CHECK(r.status == g2m::Status::Internal);
        CHECK(r.tile == nullptr);
    }
}

// ---- height cache (ported from racing_game test_world_terrain.cpp) --------------

TEST_CASE("fetch_height_tile_cached: hit, miss, and a failure is not cached", "[world][tile_fetch]") {
    std::mutex mutex;
    std::map<g2m::TileKey, std::shared_ptr<const g2m::HeightTile>> cache;
    int calls = 0;
    bool fail = true;
    const g2m::world::HeightTileFetchFn fn = [&](const g2m::TileKey& key) -> HeightTileFetchResult {
        ++calls;
        if (fail) return HeightTileFetchResult{g2m::Status::Internal, nullptr};
        auto tile = std::make_shared<g2m::HeightTile>();
        tile->key = key;
        return HeightTileFetchResult{g2m::Status::Ok, std::move(tile)};
    };

    bool hit = true;
    HeightTileFetchResult a = fetch_height_tile_cached(mutex, cache, kOffsetTestKey, fn, &hit);
    CHECK(a.status == g2m::Status::Internal);
    CHECK_FALSE(hit);
    CHECK(cache.empty());
    a = fetch_height_tile_cached(mutex, cache, kOffsetTestKey, fn, &hit);
    CHECK(calls == 2); // the failure was retried

    fail = false;
    const HeightTileFetchResult ok = fetch_height_tile_cached(mutex, cache, kOffsetTestKey, fn, &hit);
    CHECK(ok.status == g2m::Status::Ok);
    CHECK_FALSE(hit);
    CHECK(calls == 3);
    REQUIRE(cache.size() == 1);

    const HeightTileFetchResult again = fetch_height_tile_cached(mutex, cache, kOffsetTestKey, fn, &hit);
    CHECK(again.status == g2m::Status::Ok);
    CHECK(hit);
    CHECK(calls == 3); // served from the cache
    CHECK(again.tile == ok.tile);

    // A fetch that reports Ok without a tile is a failure too (not inserted).
    const g2m::world::HeightTileFetchFn lying = [&](const g2m::TileKey&) {
        return HeightTileFetchResult{g2m::Status::Ok, nullptr};
    };
    const g2m::TileKey other{kZone, 0, 1, 2};
    const HeightTileFetchResult bad = fetch_height_tile_cached(mutex, cache, other, lying, &hit);
    CHECK(bad.tile == nullptr);
    CHECK(cache.count(other) == 0);
}

TEST_CASE("fetch_height_tile_cached: 8 threads hammering overlapping keys observe one identical tile per key",
          "[world][tile_fetch]") {
    // No Catch2 assertion on a worker thread (TOOL-031): each thread writes only
    // its own row; everything is checked after the join.
    constexpr int kThreadCount = 8;
    constexpr int kIterationsPerThread = 200;
    constexpr int kKeyCount = 4;

    std::array<g2m::TileKey, kKeyCount> keys;
    for (int i = 0; i < kKeyCount; ++i) {
        keys[static_cast<std::size_t>(i)] = g2m::TileKey{kZone, /*level=*/3, /*x=*/i, /*y=*/i * 2};
    }

    std::atomic<int> fetch_call_count{0};
    const g2m::world::HeightTileFetchFn fetch_fn = [&](const g2m::TileKey& key) -> HeightTileFetchResult {
        fetch_call_count.fetch_add(1, std::memory_order_relaxed);
        // Widens the race window between the miss check and the insert.
        std::this_thread::sleep_for(std::chrono::microseconds(200));
        auto tile = std::make_shared<g2m::HeightTile>(); // heap: TOOL-039
        tile->key = key;
        tile->h.fill(key.x * 1000 + key.y);
        return HeightTileFetchResult{g2m::Status::Ok, std::move(tile)};
    };

    std::mutex cache_mutex;
    std::map<g2m::TileKey, std::shared_ptr<const g2m::HeightTile>> cache;
    std::vector<std::vector<std::shared_ptr<const g2m::HeightTile>>> per_thread_tile(
        kThreadCount, std::vector<std::shared_ptr<const g2m::HeightTile>>(kIterationsPerThread));
    std::vector<std::vector<int>> per_thread_key(kThreadCount, std::vector<int>(kIterationsPerThread));

    std::vector<std::thread> workers;
    for (int t = 0; t < kThreadCount; ++t) {
        workers.emplace_back([&, t]() {
            for (int i = 0; i < kIterationsPerThread; ++i) {
                const int key_index = (t + i) % kKeyCount;
                HeightTileFetchResult result =
                    fetch_height_tile_cached(cache_mutex, cache, keys[static_cast<std::size_t>(key_index)], fetch_fn);
                per_thread_tile[static_cast<std::size_t>(t)][static_cast<std::size_t>(i)] = result.tile;
                per_thread_key[static_cast<std::size_t>(t)][static_cast<std::size_t>(i)] = key_index;
            }
        });
    }
    for (std::thread& w : workers) w.join();

    std::array<const g2m::HeightTile*, kKeyCount> winning{};
    for (int t = 0; t < kThreadCount; ++t) {
        for (int i = 0; i < kIterationsPerThread; ++i) {
            const int key_index = per_thread_key[static_cast<std::size_t>(t)][static_cast<std::size_t>(i)];
            const auto& tile = per_thread_tile[static_cast<std::size_t>(t)][static_cast<std::size_t>(i)];
            const g2m::TileKey& key = keys[static_cast<std::size_t>(key_index)];
            REQUIRE(tile != nullptr);
            CHECK(tile->key == key);
            CHECK(tile->h[0] == key.x * 1000 + key.y);
            const g2m::HeightTile*& w = winning[static_cast<std::size_t>(key_index)];
            if (w == nullptr) w = tile.get();
            CHECK(tile.get() == w); // every caller got the SAME tile object
        }
    }
    CHECK(cache.size() == static_cast<std::size_t>(kKeyCount));
    CHECK(fetch_call_count.load() >= kKeyCount);
    CHECK(fetch_call_count.load() <= kThreadCount * kKeyCount); // redundant fetches are bounded
}

// ---- road segments ---------------------------------------------------------------

namespace {

struct RoadPin {
    const char* name;
    RoadFetchStatus status;
    long long count;          // -1 = null list
    std::uint64_t list_hash;  // 0 = null list
};

// Pins from racing_game a8a1b13's decode_road_segments_response (refgen output).
constexpr RoadPin kRoadPins[] = {
    {"ok", RoadFetchStatus::Ok, 7, 0xc7434ae4a95a6a09ull},
    {"ok_empty_tile", RoadFetchStatus::Ok, 0, 0xa8c7f832281a39c5ull},
    {"not_found_outside_coverage", RoadFetchStatus::Absent, 0, 0xa8c7f832281a39c5ull},
    {"not_found_other", RoadFetchStatus::Failed, -1, 0},
    {"not_found_no_message", RoadFetchStatus::Failed, -1, 0},
    {"server_error", RoadFetchStatus::Failed, -1, 0},
    {"ok_status_empty_container", RoadFetchStatus::Failed, -1, 0},
    {"wrong_layer", RoadFetchStatus::Failed, -1, 0},
    {"wrong_key", RoadFetchStatus::Failed, -1, 0},
    {"corrupt_body", RoadFetchStatus::Failed, -1, 0},
    {"height_container_for_osm", RoadFetchStatus::Failed, -1, 0},
    {"non_tile_response", RoadFetchStatus::Failed, -1, 0},
};

} // namespace

TEST_CASE("decode_road_segments_response matches racing_game's decoder on every case", "[world][tile_fetch]") {
    const std::vector<RoadResponseCase> cases = road_response_cases();
    REQUIRE(cases.size() == std::size(kRoadPins));
    for (std::size_t i = 0; i < cases.size(); ++i) {
        const RoadResponseCase& c = cases[i];
        const RoadPin& pin = kRoadPins[i];
        INFO("case " << c.name);
        REQUIRE(c.name == pin.name);
        const RoadSegmentsResult r = decode_road_segments_response(c.response, c.expect_key, kZone);
        CHECK(r.status == pin.status);
        if (pin.count < 0) {
            CHECK(r.segments == nullptr);
        } else {
            REQUIRE(r.segments != nullptr);
            CHECK(static_cast<long long>(r.segments->size()) == pin.count);
            CHECK(hash_road_segments(*r.segments) == pin.list_hash);
        }
    }
}

TEST_CASE("decode_road_segments_response: Absent shares one empty list", "[world][tile_fetch]") {
    const g2m::Response r = make_tile_response(g2m::Status::NotFound, {}, "outside coverage");
    const RoadSegmentsResult a = decode_road_segments_response(r, kOsmKey, kZone);
    const RoadSegmentsResult b = decode_road_segments_response(r, kOsmKey, kZone);
    CHECK(a.status == RoadFetchStatus::Absent);
    REQUIRE(a.segments != nullptr);
    CHECK(a.segments->empty());
    CHECK(a.segments == b.segments); // the shared empty list, as in the game
}

TEST_CASE("fetch_road_segments sends the osm tile request", "[world][tile_fetch]") {
    g2m::ReleaseId rid{};
    rid[5] = 0x77;
    FixedTransport t(make_tile_response(g2m::Status::Ok, make_osm_container(make_road_osm_tile())));
    const RoadSegmentsResult r = fetch_road_segments(t, rid, kOsmKey, kZone);
    REQUIRE(r.status == RoadFetchStatus::Ok);
    REQUIRE(r.segments != nullptr);
    CHECK(r.segments->size() == 7);
    REQUIRE(t.requests.size() == 1);
    const auto* req = std::get_if<g2m::TileRequest>(&t.requests[0]);
    REQUIRE(req != nullptr);
    CHECK(req->rid == rid);
    CHECK(req->layer == g2m::kSrcOsmLayer);
    CHECK(req->key == kOsmKey);
}

TEST_CASE("fetch_road_segments_cached: a Failed fetch is not cached - a second call refetches", "[world][tile_fetch]") {
    std::mutex cache_mutex;
    std::map<g2m::TileKey, RoadSegmentsResult> cache;
    int fetch_calls = 0;
    const g2m::world::RoadSegmentsFetchFn fetch_fn = [&](const g2m::TileKey&) -> RoadSegmentsResult {
        ++fetch_calls;
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    };

    bool was_cache_hit = true;
    RoadSegmentsResult first = fetch_road_segments_cached(cache_mutex, cache, kOsmKey, fetch_fn, &was_cache_hit);
    CHECK(first.status == RoadFetchStatus::Failed);
    CHECK_FALSE(was_cache_hit);
    CHECK(cache.empty());

    RoadSegmentsResult second = fetch_road_segments_cached(cache_mutex, cache, kOsmKey, fetch_fn, &was_cache_hit);
    CHECK(second.status == RoadFetchStatus::Failed);
    CHECK_FALSE(was_cache_hit);
    CHECK(cache.empty());
    CHECK(fetch_calls == 2);
}

TEST_CASE("fetch_road_segments_cached: Ok and Absent results are cached", "[world][tile_fetch]") {
    SECTION("Absent") {
        std::mutex cache_mutex;
        std::map<g2m::TileKey, RoadSegmentsResult> cache;
        int fetch_calls = 0;
        const g2m::world::RoadSegmentsFetchFn fetch_fn = [&](const g2m::TileKey&) -> RoadSegmentsResult {
            ++fetch_calls;
            return RoadSegmentsResult{RoadFetchStatus::Absent, std::make_shared<const std::vector<g2m::RoadSegment>>()};
        };
        bool was_cache_hit = true;
        const RoadSegmentsResult first = fetch_road_segments_cached(cache_mutex, cache, kOsmKey, fetch_fn, &was_cache_hit);
        CHECK(first.status == RoadFetchStatus::Absent);
        CHECK_FALSE(was_cache_hit);
        REQUIRE(cache.count(kOsmKey) == 1);
        const RoadSegmentsResult second = fetch_road_segments_cached(cache_mutex, cache, kOsmKey, fetch_fn, &was_cache_hit);
        CHECK(second.status == RoadFetchStatus::Absent);
        CHECK(second.segments == first.segments);
        CHECK(was_cache_hit);
        CHECK(fetch_calls == 1);
    }
    SECTION("Ok") {
        std::mutex cache_mutex;
        std::map<g2m::TileKey, RoadSegmentsResult> cache;
        int fetch_calls = 0;
        auto segments = std::make_shared<const std::vector<g2m::RoadSegment>>(make_road_segments());
        const g2m::world::RoadSegmentsFetchFn fetch_fn = [&](const g2m::TileKey&) -> RoadSegmentsResult {
            ++fetch_calls;
            return RoadSegmentsResult{RoadFetchStatus::Ok, segments};
        };
        bool was_cache_hit = true;
        const RoadSegmentsResult first = fetch_road_segments_cached(cache_mutex, cache, kOsmKey, fetch_fn, &was_cache_hit);
        CHECK(first.status == RoadFetchStatus::Ok);
        CHECK_FALSE(was_cache_hit);
        const RoadSegmentsResult second = fetch_road_segments_cached(cache_mutex, cache, kOsmKey, fetch_fn, &was_cache_hit);
        CHECK(second.status == RoadFetchStatus::Ok);
        CHECK(second.segments == first.segments);
        CHECK(was_cache_hit);
        CHECK(fetch_calls == 1);
    }
}

TEST_CASE("fetch_road_segments_cached: racing threads agree on one list", "[world][tile_fetch]") {
    std::mutex cache_mutex;
    std::map<g2m::TileKey, RoadSegmentsResult> cache;
    std::atomic<int> calls{0};
    const g2m::world::RoadSegmentsFetchFn fetch_fn = [&](const g2m::TileKey&) -> RoadSegmentsResult {
        calls.fetch_add(1);
        std::this_thread::sleep_for(std::chrono::microseconds(300));
        return RoadSegmentsResult{RoadFetchStatus::Ok, std::make_shared<const std::vector<g2m::RoadSegment>>(make_road_segments())};
    };
    constexpr int kThreads = 8;
    std::vector<RoadSegmentsResult> results(kThreads);
    std::vector<std::thread> workers;
    for (int t = 0; t < kThreads; ++t) {
        workers.emplace_back([&, t] { results[static_cast<std::size_t>(t)] = fetch_road_segments_cached(cache_mutex, cache, kOsmKey, fetch_fn); });
    }
    for (std::thread& w : workers) w.join();
    for (const RoadSegmentsResult& r : results) {
        REQUIRE(r.segments != nullptr);
        CHECK(r.segments == results[0].segments);
    }
    CHECK(calls.load() >= 1);
    CHECK(cache.size() == 1);
}
