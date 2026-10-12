// g2m/world/ticket.h - the asynchronous request handle of g2m::world::TerrainService
// (terrain migration plan phase 2, vault G2M-020): Priority, TileState, Ticket<T>.
//
// A Ticket is the ONLY thing a render or simulation thread touches: state() and
// value() are lock-free atomic reads and never block, never derive. A ticket is
// cheap to copy; all copies share one request. Every copy observes the same
// result, and cancel() on any copy withdraws the request (see below).
//
// Cancellation. A ticket is one holder's interest in a job. The job stops (never
// runs) once every holder has cancelled or dropped its ticket AND it has not
// started yet; a job that is already running finishes and its result is cached
// like any other (a later request is then a cache hit). Dropping the last copy of
// a ticket is the same as cancel(). A cancelled ticket reports TileState::Cancelled
// even if the job later finishes for another holder.
//
// detail:: is the type-erased machinery TerrainService and its scheduler share
// with this template; it is not for callers.
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace g2m::world {

// Highest first. Within one priority jobs run in request order. A request for a
// tile that is already queued at a lower priority raises that tile's priority.
enum class Priority : std::uint8_t { Gate = 0, Physics = 1, Prefetch = 2, Render = 3, Warm = 4 };

enum class TileState : std::uint8_t {
    Pending,   // queued or running
    Ready,     // value() is non-null
    Absent,    // the release has no such tile (outside coverage, layer not in the release): final, cached
    Failed,    // message() says why; NOT cached, a later request retries
    Cancelled, // this holder cancelled, or the service shut down before the job started
};

namespace detail {

// Identity of one job: the product kind (a builtin or a registered product), the
// packed tile key and a hash of the product's options (0 when it has none).
struct JobKey {
    std::uint32_t kind = 0;
    std::uint64_t tile = 0;
    std::uint64_t options = 0;
    friend bool operator==(const JobKey&, const JobKey&) = default;
    friend auto operator<=>(const JobKey&, const JobKey&) = default;
};

// What a job produces, type erased. `value` is a shared_ptr<const T> for the T the
// requesting call named; `bytes` is its weight in the service's memory cache.
struct ErasedOutcome {
    TileState state = TileState::Failed; // Ready, Absent or Failed
    std::shared_ptr<const void> value;
    std::string message;
    std::size_t bytes = 0;
};

class IInterestSink;

struct JobCore {
    enum Phase : std::uint8_t { Queued = 0, Running = 1, Finished = 2, Cancelled = 3 };

    JobKey key;
    const void* type = nullptr;                // typeid(T) address, checked on a join
    std::atomic<std::uint8_t> phase{Queued};
    std::atomic<std::uint8_t> priority{static_cast<std::uint8_t>(Priority::Render)};
    std::function<ErasedOutcome()> fn;         // set once at creation; cleared when the job ends

    // Written once by the thread that ran the job, before phase becomes Finished (release).
    ErasedOutcome outcome;

    // Guarded by the scheduler's mutex:
    int interest = 0;
    std::thread::id runner;
    std::vector<std::function<void()>> hooks;  // run once, outside the lock, when the job ends

    // Terminal-phase wait (tickets, nested joins).
    std::mutex done_mu;
    std::condition_variable done_cv;

    bool terminal() const noexcept { return phase.load(std::memory_order_acquire) >= Finished; }
};

class IInterestSink {
public:
    virtual ~IInterestSink() = default;
    virtual void release_interest(JobCore& job) noexcept = 0;
};

// One holder's share of a job; copies of a Ticket share it.
struct Interest {
    std::shared_ptr<JobCore> job;
    std::shared_ptr<IInterestSink> sink;
    std::atomic<bool> active{true};
    void release() noexcept {
        if (active.exchange(false) && sink && job) sink->release_interest(*job);
    }
    ~Interest() { release(); }
};

} // namespace detail

template <class T>
class Ticket {
public:
    Ticket() = default;

    // Never blocks. A default-constructed ticket is Failed("empty ticket").
    TileState state() const noexcept {
        if (!job_) return TileState::Failed;
        if (interest_ && !interest_->active.load(std::memory_order_acquire)) return TileState::Cancelled;
        const auto p = job_->phase.load(std::memory_order_acquire);
        if (p == detail::JobCore::Finished) return job_->outcome.state;
        if (p == detail::JobCore::Cancelled) return TileState::Cancelled;
        return TileState::Pending;
    }
    bool pending() const noexcept { return state() == TileState::Pending; }
    // Null unless state() == Ready. Never blocks.
    std::shared_ptr<const T> value() const noexcept {
        if (state() != TileState::Ready) return nullptr;
        return std::static_pointer_cast<const T>(job_->outcome.value);
    }
    // The reason of an Absent or Failed ticket, else empty.
    const std::string& message() const noexcept {
        static const std::string kEmpty;
        if (!job_ || job_->phase.load(std::memory_order_acquire) != detail::JobCore::Finished) return kEmpty;
        return job_->outcome.message;
    }
    // Blocks up to `d` for the job to reach a terminal phase; true if it did. For tools, tests and
    // loader threads - NEVER on a render or simulation thread.
    bool wait_for(std::chrono::milliseconds d) const {
        if (!job_) return true;
        std::unique_lock lock(job_->done_mu);
        return job_->done_cv.wait_for(lock, d, [&] { return job_->terminal(); });
    }
    // Withdraws this holder's interest (see the header comment). Idempotent.
    void cancel() noexcept {
        if (interest_) interest_->release();
    }
    explicit operator bool() const noexcept { return job_ != nullptr; }

    // Internal (TerrainService): wraps a job and the holder's interest in it.
    Ticket(std::shared_ptr<detail::JobCore> job, std::shared_ptr<detail::Interest> interest)
        : job_(std::move(job)), interest_(std::move(interest)) {}

private:
    std::shared_ptr<detail::JobCore> job_;
    std::shared_ptr<detail::Interest> interest_;
};

} // namespace g2m::world
