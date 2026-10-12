// src/world/job_scheduler.h - INTERNAL. The worker pool, priority queue and per-job
// registry behind g2m::world::TerrainService (terrain migration phase 2).
//
// One mutex (`mu_`) guards the registry, the queue and every JobCore's interest, runner and
// hooks; a JobCore's own done_mu only serialises terminal waits. Lock order: mu_ then done_mu.
// Job functions and hooks always run with NO lock held.
//
// A job exists in the registry only while it is Queued or Running, so "is something already
// computing this?" is one lookup. A Finished or Cancelled job leaves the registry in the same
// critical section that publishes its phase.
#pragma once

#include "g2m/world/ticket.h"

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <queue>
#include <thread>
#include <vector>

namespace g2m::world::detail {

struct SchedulerStats {
    std::uint64_t jobs_run = 0, inline_runs = 0, joined = 0, boosted = 0, cancelled_before_start = 0;
    int peak_running = 0;
};

class JobScheduler final : public IInterestSink, public std::enable_shared_from_this<JobScheduler> {
public:
    // Starts `workers` (>= 1) threads.
    static std::shared_ptr<JobScheduler> create(int workers);
    ~JobScheduler() override;

    // Cancels every queued job, lets running ones finish, joins the workers. Idempotent. Later
    // submissions return Cancelled jobs.
    void shutdown();
    bool stopped() const noexcept { return stop_.load(std::memory_order_acquire); }
    int workers() const noexcept { return static_cast<int>(threads_.size()); }

    // Re-check of a result cache, run under the scheduler lock when no job exists for a key.
    using Probe = std::function<std::optional<ErasedOutcome>()>;

    struct Submitted {
        std::shared_ptr<JobCore> job;
        std::shared_ptr<Interest> interest; // null for a job that was already finished/cancelled
    };
    // Async request: joins the job of `key` (raising its priority if queued lower) or queues a new one.
    Submitted submit(const JobKey& key, const void* type, Priority priority, const std::function<ErasedOutcome()>& fn,
                     const Probe& probe);
    // Blocking request for a producer: joins a running job, runs a queued one inline, or runs `fn` inline.
    ErasedOutcome join_or_run(const JobKey& key, const void* type, const std::function<ErasedOutcome()>& fn,
                              const Probe& probe);
    // Runs `hook` once when `job` ends (immediately, on this thread, if it already has).
    void add_hook(const std::shared_ptr<JobCore>& job, std::function<void()> hook);

    // A job that is already finished with `o` (a ticket that never needed the pool).
    static std::shared_ptr<JobCore> make_finished(const JobKey& key, const void* type, ErasedOutcome o);

    SchedulerStats stats() const;
    void release_interest(JobCore& job) noexcept override;

private:
    explicit JobScheduler(int workers);
    struct Entry {
        std::uint8_t priority;
        std::uint64_t seq;
        std::shared_ptr<JobCore> job;
        // priority_queue is a max-heap: "less" = later = lower priority value last.
        bool operator<(const Entry& o) const noexcept {
            if (priority != o.priority) return priority > o.priority;
            return seq > o.seq;
        }
    };

    void worker_main();
    // Runs job->fn (phase already Running), publishes the result. Returns a copy of the outcome.
    ErasedOutcome execute(const std::shared_ptr<JobCore>& job, bool inline_run);
    static std::shared_ptr<JobCore> make_cancelled(const JobKey& key, const void* type);
    // Both under mu_; collect the hooks to run after unlocking.
    void cancel_locked(const std::shared_ptr<JobCore>& job, std::vector<std::function<void()>>& hooks);
    static void run_hooks(std::vector<std::function<void()>>& hooks);

    mutable std::mutex mu_;
    std::condition_variable cv_;
    std::atomic<bool> stop_{false};
    std::map<JobKey, std::shared_ptr<JobCore>> registry_;
    std::priority_queue<Entry> queue_;
    std::uint64_t seq_ = 0;
    SchedulerStats stats_;
    int running_ = 0;
    std::vector<std::thread> threads_;
};

} // namespace g2m::world::detail
