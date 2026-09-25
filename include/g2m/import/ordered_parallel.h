// g2m/import/ordered_parallel.h - ordered_for: fan `count` independent
// `produce(i)` calls out over up to `jobs` worker threads, but always
// deliver them to `consume(i, ...)` in ascending index order on the calling
// thread - so an importer's OUTPUT (tile writes, an ImportReport's message
// order, ...) is identical whatever `--jobs` was (G1c I2; PLAN.md's
// determinism-independent-of-jobs requirement, the same rule
// jobs/job_system.h's own parallel_for/gather split follows in
// physics_sim, applied here at the process level instead of the thread-pool
// level since g2m_tiler's importers are their own top-level parallel loop).
#pragma once

#include <condition_variable>
#include <cstddef>
#include <exception>
#include <mutex>
#include <optional>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace g2m::import {

// `produce`: (std::size_t index) -> T, called concurrently from up to
// `jobs` worker threads (never index-reentrant: each index is produced
// exactly once, by exactly one thread). `consume`: (std::size_t index, T&&)
// -> void, called only from THIS function's own caller thread, strictly in
// ascending index order, never concurrently with any `produce` call for a
// DIFFERENT purpose (the two may overlap in time - that is the point - but
// `consume` itself is never called from more than one thread and never
// re-entered).
//
// At most `jobs` indices are ever "in flight" ahead of the next index
// `consume` is waiting for (a producer whose claimed index would exceed
// that window blocks until `consume` advances) - bounded memory (at most
// `jobs` results ever buffered at once), not "produce everything, then
// deliver everything".
//
// If any `produce(i)` throws, no index after the first (lowest) failing one
// is ever passed to `consume`; once `consume` reaches that index, the
// exception is rethrown from ordered_for itself (in index order, which may
// not be the order the failures actually occurred in - deterministic
// reporting matching the deterministic success path). Every worker thread
// is joined before ordered_for returns or rethrows, in every case.
//
// `jobs < 1` is treated as 1. `jobs = 1` still uses one dedicated worker
// thread (kept simple - a single code path for every `jobs` value); the
// window bound then degenerates to "produce index i, consume index i,
// repeat".
template <typename Produce, typename Consume>
void ordered_for(std::size_t count, int jobs, Produce produce, Consume consume) {
    if (count == 0) return;
    if (jobs < 1) jobs = 1;
    const std::size_t window = static_cast<std::size_t>(jobs);

    using T = std::invoke_result_t<Produce&, std::size_t>;
    struct Slot {
        std::optional<T> value;
        std::exception_ptr error;
        bool ready = false;
    };
    std::vector<Slot> slots(count);

    std::mutex mutex;
    std::condition_variable cv;
    std::size_t next_index = 0; // next index a worker may claim
    std::size_t consumed = 0;   // indices [0, consumed) already delivered to consume()

    auto worker = [&]() {
        for (;;) {
            std::size_t i;
            {
                std::unique_lock<std::mutex> lock(mutex);
                cv.wait(lock, [&] { return next_index >= count || next_index < consumed + window; });
                if (next_index >= count) return;
                i = next_index++;
            }
            Slot result;
            try {
                result.value.emplace(produce(i));
            } catch (...) {
                result.error = std::current_exception();
            }
            result.ready = true;
            {
                std::lock_guard<std::mutex> lock(mutex);
                slots[i] = std::move(result);
            }
            cv.notify_all();
        }
    };

    std::vector<std::thread> threads;
    threads.reserve(static_cast<std::size_t>(jobs));
    for (int t = 0; t < jobs; ++t) threads.emplace_back(worker);

    auto join_all = [&]() {
        for (auto& th : threads) {
            if (th.joinable()) th.join();
        }
    };

    for (std::size_t i = 0; i < count; ++i) {
        Slot got;
        {
            std::unique_lock<std::mutex> lock(mutex);
            cv.wait(lock, [&] { return slots[i].ready; });
            got = std::move(slots[i]);
            consumed = i + 1;
        }
        cv.notify_all(); // window may have opened up for waiting producers
        if (got.error) {
            {
                std::lock_guard<std::mutex> lock(mutex);
                next_index = count; // stop handing out new work
            }
            cv.notify_all();
            join_all();
            std::rethrow_exception(got.error);
        }
        consume(i, std::move(*got.value));
    }
    join_all();
}

} // namespace g2m::import
