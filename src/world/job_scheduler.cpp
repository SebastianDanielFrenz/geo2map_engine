// src/world/job_scheduler.cpp - see job_scheduler.h.

#include "job_scheduler.h"

#include <algorithm>
#include <utility>

namespace g2m::world::detail {

namespace {
ErasedOutcome failed_outcome(std::string why) {
    ErasedOutcome o;
    o.state = TileState::Failed;
    o.message = std::move(why);
    return o;
}
} // namespace

std::shared_ptr<JobScheduler> JobScheduler::create(int workers) {
    auto s = std::shared_ptr<JobScheduler>(new JobScheduler(std::max(1, workers)));
    return s;
}

JobScheduler::JobScheduler(int workers) {
    threads_.reserve(static_cast<std::size_t>(workers));
    for (int i = 0; i < workers; ++i) threads_.emplace_back([this] { worker_main(); });
}

JobScheduler::~JobScheduler() { shutdown(); }

void JobScheduler::shutdown() {
    std::vector<std::function<void()>> hooks;
    std::vector<std::thread> to_join;
    {
        std::lock_guard lock(mu_);
        if (!stop_.exchange(true)) {
            // Cancel everything that has not started; a running job finishes normally.
            std::vector<std::shared_ptr<JobCore>> queued;
            for (auto& [k, job] : registry_)
                if (job->phase.load(std::memory_order_relaxed) == JobCore::Queued) queued.push_back(job);
            for (auto& job : queued) cancel_locked(job, hooks);
            queue_ = {};
        }
        to_join = std::move(threads_);
        threads_.clear();
    }
    cv_.notify_all();
    for (auto& t : to_join) {
        if (t.get_id() == std::this_thread::get_id()) t.detach(); // never join ourselves
        else t.join();
    }
    run_hooks(hooks);
}

void JobScheduler::run_hooks(std::vector<std::function<void()>>& hooks) {
    for (auto& h : hooks)
        if (h) h();
    hooks.clear();
}

void JobScheduler::cancel_locked(const std::shared_ptr<JobCore>& job, std::vector<std::function<void()>>& hooks) {
    {
        std::lock_guard dl(job->done_mu);
        job->phase.store(JobCore::Cancelled, std::memory_order_release);
    }
    job->fn = nullptr;
    for (auto& h : job->hooks) hooks.push_back(std::move(h));
    job->hooks.clear();
    const auto it = registry_.find(job->key);
    if (it != registry_.end() && it->second == job) registry_.erase(it);
    ++stats_.cancelled_before_start;
    job->done_cv.notify_all();
}

std::shared_ptr<JobCore> JobScheduler::make_finished(const JobKey& key, const void* type, ErasedOutcome o) {
    auto job = std::make_shared<JobCore>();
    job->key = key;
    job->type = type;
    job->outcome = std::move(o);
    job->phase.store(JobCore::Finished, std::memory_order_release);
    return job;
}

std::shared_ptr<JobCore> JobScheduler::make_cancelled(const JobKey& key, const void* type) {
    auto job = std::make_shared<JobCore>();
    job->key = key;
    job->type = type;
    job->phase.store(JobCore::Cancelled, std::memory_order_release);
    return job;
}

JobScheduler::Submitted JobScheduler::submit(const JobKey& key, const void* type, Priority priority,
                                             const std::function<ErasedOutcome()>& fn, const Probe& probe) {
    std::lock_guard lock(mu_);
    if (stop_.load(std::memory_order_relaxed)) return {make_cancelled(key, type), nullptr};
    const auto prio = static_cast<std::uint8_t>(priority);
    const auto it = registry_.find(key);
    std::shared_ptr<JobCore> job;
    if (it != registry_.end()) {
        job = it->second;
        if (job->type != type) return {make_finished(key, type, failed_outcome("product type mismatch")), nullptr};
        ++job->interest;
        ++stats_.joined;
        if (job->phase.load(std::memory_order_relaxed) == JobCore::Queued && prio < job->priority.load()) {
            // Raise: queue a second entry at the higher priority; the stale one is skipped when popped.
            job->priority.store(prio);
            queue_.push(Entry{prio, ++seq_, job});
            ++stats_.boosted;
            cv_.notify_one();
        }
    } else {
        if (probe) {
            if (auto cached = probe()) return {make_finished(key, type, std::move(*cached)), nullptr};
        }
        job = std::make_shared<JobCore>();
        job->key = key;
        job->type = type;
        job->fn = fn;
        job->priority.store(prio);
        job->interest = 1;
        registry_.emplace(key, job);
        queue_.push(Entry{prio, ++seq_, job});
        cv_.notify_one();
    }
    auto interest = std::make_shared<Interest>();
    interest->job = job;
    interest->sink = shared_from_this();
    return {std::move(job), std::move(interest)};
}

void JobScheduler::release_interest(JobCore& job) noexcept {
    std::vector<std::function<void()>> hooks;
    {
        std::lock_guard lock(mu_);
        if (--job.interest > 0) return;
        if (job.phase.load(std::memory_order_relaxed) != JobCore::Queued) return;
        // Last holder gone before the job started: it never runs.
        const auto it = registry_.find(job.key);
        if (it != registry_.end() && it->second.get() == &job) {
            const auto owner = it->second; // keeps the JobCore alive while cancelled
            cancel_locked(owner, hooks);
        }
    }
    run_hooks(hooks);
}

void JobScheduler::add_hook(const std::shared_ptr<JobCore>& job, std::function<void()> hook) {
    {
        std::lock_guard lock(mu_);
        if (job->phase.load(std::memory_order_relaxed) < JobCore::Finished) {
            job->hooks.push_back(std::move(hook));
            return;
        }
    }
    hook();
}

ErasedOutcome JobScheduler::execute(const std::shared_ptr<JobCore>& job, bool inline_run) {
    std::function<ErasedOutcome()> fn;
    {
        std::lock_guard lock(mu_);
        fn = std::move(job->fn);
        job->fn = nullptr;
        ++running_;
        stats_.peak_running = std::max(stats_.peak_running, running_);
        ++stats_.jobs_run;
        if (inline_run) ++stats_.inline_runs;
    }
    ErasedOutcome out;
    try {
        out = fn ? fn() : failed_outcome("job has no function");
    } catch (...) {
        out = failed_outcome("job threw an exception");
    }
    fn = nullptr; // drop what the job captured before anyone can observe it as finished
    std::vector<std::function<void()>> hooks;
    {
        std::lock_guard lock(mu_);
        --running_;
        const auto it = registry_.find(job->key);
        if (it != registry_.end() && it->second == job) registry_.erase(it);
        job->outcome = std::move(out);
        hooks = std::move(job->hooks);
        job->hooks.clear();
        std::lock_guard dl(job->done_mu);
        job->phase.store(JobCore::Finished, std::memory_order_release);
    }
    job->done_cv.notify_all();
    ErasedOutcome copy = job->outcome;
    run_hooks(hooks);
    return copy;
}

void JobScheduler::worker_main() {
    std::unique_lock lock(mu_);
    for (;;) {
        cv_.wait(lock, [&] { return stop_.load(std::memory_order_relaxed) || !queue_.empty(); });
        if (stop_.load(std::memory_order_relaxed)) return;
        Entry e = queue_.top();
        queue_.pop();
        const std::shared_ptr<JobCore> job = std::move(e.job);
        if (job->phase.load(std::memory_order_relaxed) != JobCore::Queued) continue; // stale: cancelled, boosted twin, or run inline
        job->phase.store(JobCore::Running, std::memory_order_relaxed);
        job->runner = std::this_thread::get_id();
        lock.unlock();
        execute(job, /*inline_run=*/false);
        lock.lock();
    }
}

ErasedOutcome JobScheduler::join_or_run(const JobKey& key, const void* type, const std::function<ErasedOutcome()>& fn,
                                        const Probe& probe) {
    std::shared_ptr<JobCore> job;
    {
        std::unique_lock lock(mu_);
        const auto it = registry_.find(key);
        if (it != registry_.end()) {
            job = it->second;
            if (job->type != type) return failed_outcome("product type mismatch");
            const auto phase = job->phase.load(std::memory_order_relaxed);
            ++stats_.joined;
            if (phase == JobCore::Running) {
                if (job->runner == std::this_thread::get_id()) return failed_outcome("cyclic product dependency");
                lock.unlock();
                std::unique_lock dl(job->done_mu);
                job->done_cv.wait(dl, [&] { return job->terminal(); });
                // A Running job cannot be cancelled; it ends Finished.
                return job->outcome;
            }
            // Queued: take it over and run it here (never wait for queued work).
            job->phase.store(JobCore::Running, std::memory_order_relaxed);
            job->runner = std::this_thread::get_id();
        } else {
            if (probe) {
                if (auto cached = probe()) return std::move(*cached);
            }
            if (stop_.load(std::memory_order_relaxed)) return failed_outcome("service is shutting down");
            job = std::make_shared<JobCore>();
            job->key = key;
            job->type = type;
            job->fn = fn;
            job->phase.store(JobCore::Running, std::memory_order_relaxed);
            job->runner = std::this_thread::get_id();
            registry_.emplace(key, job);
        }
    }
    return execute(job, /*inline_run=*/true);
}

SchedulerStats JobScheduler::stats() const {
    std::lock_guard lock(mu_);
    return stats_;
}

} // namespace g2m::world::detail
