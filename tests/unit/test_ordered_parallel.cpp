// tests/unit/test_ordered_parallel.cpp - g2m::import::ordered_for
// (g2m/import/ordered_parallel.h, G1c I2): delivery order independent of
// `jobs`, exception propagation, and the in-flight window bound.

#include "g2m/import/ordered_parallel.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <mutex>
#include <random>
#include <stdexcept>
#include <thread>
#include <vector>

using g2m::import::ordered_for;

namespace {

// A splitmix64-seeded, deterministic-per-index sleep so producer finish
// order is scrambled independent of the actual OS thread scheduler - if
// consume() ever saw an out-of-order index, this would very likely catch
// it (physics_sim CLAUDE.md's splitmix64.h pattern, reused here as a
// simple counter-based jitter instead of a shared PRNG object, since we
// only need "some values are cheap, some are not", not statistical
// quality).
void jitter_sleep(std::size_t i) {
    const std::size_t bucket = (i * 2654435761u) % 5;
    if (bucket == 0) std::this_thread::sleep_for(std::chrono::milliseconds(2));
}

} // namespace

TEST_CASE("ordered_parallel: delivery order matches index order for jobs=1,2,8", "[import]") {
    constexpr std::size_t kCount = 200;
    for (const int jobs : {1, 2, 8}) {
        std::vector<std::size_t> delivered;
        delivered.reserve(kCount);
        std::mutex produce_calls_mutex;
        std::size_t produce_calls = 0;

        ordered_for(
            kCount, jobs,
            [&](std::size_t i) -> std::size_t {
                jitter_sleep(i);
                {
                    std::lock_guard<std::mutex> lock(produce_calls_mutex);
                    ++produce_calls;
                }
                return i * 2; // arbitrary payload derived from i
            },
            [&](std::size_t i, std::size_t&& payload) {
                CHECK(payload == i * 2);
                delivered.push_back(i);
            });

        REQUIRE(delivered.size() == kCount);
        for (std::size_t i = 0; i < kCount; ++i) CHECK(delivered[i] == i);
        CHECK(produce_calls == kCount);
    }
}

TEST_CASE("ordered_parallel: each index is produced exactly once", "[import]") {
    constexpr std::size_t kCount = 500;
    std::vector<std::atomic<int>> seen(kCount);
    for (auto& c : seen) c = 0;

    std::vector<std::size_t> delivered;
    ordered_for(
        kCount, 6,
        [&](std::size_t i) -> int {
            seen[i].fetch_add(1);
            return static_cast<int>(i);
        },
        [&](std::size_t i, int&& v) {
            CHECK(v == static_cast<int>(i));
            delivered.push_back(i);
        });

    REQUIRE(delivered.size() == kCount);
    for (std::size_t i = 0; i < kCount; ++i) CHECK(seen[i].load() == 1);
}

TEST_CASE("ordered_parallel: a producer exception is rethrown once consume reaches its index", "[import]") {
    constexpr std::size_t kCount = 50;
    constexpr std::size_t kFailAt = 17;
    std::vector<std::size_t> delivered;

    // CLAUDE.md "Windows ASan caveat" (TOOL-019): a hand-written
    // `catch (const T& e)` that then touches `e.what()` reads garbage
    // under clang-cl ASan (stack instrumentation moves the catch
    // parameter). CHECK_THROWS_AS only catches BY TYPE (never touches the
    // exception object), which the same doc note calls out as safe - the
    // pattern every other exception check in this codebase already uses
    // (test_decode_osm.cpp/test_decode_tiff.cpp's own CHECK_THROWS_AS
    // calls), so the exception's message is deliberately not asserted
    // here.
    const auto run = [&]() {
        ordered_for(
            kCount, 4,
            [&](std::size_t i) -> std::size_t {
                if (i == kFailAt) throw std::runtime_error("boom");
                return i;
            },
            [&](std::size_t i, std::size_t&& v) {
                CHECK(v == i);
                delivered.push_back(i);
            });
    };
    CHECK_THROWS_AS(run(), std::runtime_error);

    // Every index BEFORE the failing one was delivered, in order; nothing
    // at or after it was.
    REQUIRE(delivered.size() == kFailAt);
    for (std::size_t i = 0; i < kFailAt; ++i) CHECK(delivered[i] == i);
}

TEST_CASE("ordered_parallel: at most `jobs` results are ever buffered ahead of consume", "[import]") {
    // A producer that is only fast for indices already within the window
    // would deadlock/starve if the window bound were broken; here every
    // producer just records how many OTHER produce() calls are
    // concurrently in flight, which must never exceed `jobs`.
    constexpr std::size_t kCount = 100;
    constexpr int kJobs = 3;
    std::atomic<int> in_flight{0};
    std::atomic<int> max_in_flight{0};

    ordered_for(
        kCount, kJobs,
        [&](std::size_t) -> int {
            const int now = in_flight.fetch_add(1) + 1;
            int prev = max_in_flight.load();
            while (now > prev && !max_in_flight.compare_exchange_weak(prev, now)) {
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            in_flight.fetch_sub(1);
            return 0;
        },
        [](std::size_t, int&&) {});

    CHECK(max_in_flight.load() <= kJobs);
}

TEST_CASE("ordered_parallel: count == 0 calls neither produce nor consume", "[import]") {
    bool produce_called = false;
    bool consume_called = false;
    ordered_for(
        0, 4, [&](std::size_t) -> int { produce_called = true; return 0; },
        [&](std::size_t, int&&) { consume_called = true; });
    CHECK_FALSE(produce_called);
    CHECK_FALSE(consume_called);
}

TEST_CASE("ordered_parallel: jobs < 1 is treated as 1", "[import]") {
    constexpr std::size_t kCount = 10;
    std::vector<std::size_t> delivered;
    ordered_for(
        kCount, 0, [](std::size_t i) -> std::size_t { return i; },
        [&](std::size_t i, std::size_t&& v) {
            CHECK(v == i);
            delivered.push_back(i);
        });
    REQUIRE(delivered.size() == kCount);
    for (std::size_t i = 0; i < kCount; ++i) CHECK(delivered[i] == i);
}
