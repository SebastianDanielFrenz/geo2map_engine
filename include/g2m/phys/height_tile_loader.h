// g2m/phys/height_tile_loader.h - asynchronous DGM1 L0 HeightTile loading for
// the physics bridge (PLAN.md R2.2 section 1a): IHeightTileFetch (one
// blocking, thread-safe fetch), two implementations of it, and
// HeightTileLoader (a small fixed worker pool with retry/backoff that turns a
// priority-ordered want-list into completions the owner thread polls).
// ps-free, like everything under include/g2m/phys.
//
// Threading contract (together with resident_heights.h's own):
//   - HeightTileLoader's public methods (request/poll/stats) may be called
//     from any ONE owner thread (PhysicsTerrainStreamer::update() calls all
//     of them from the sim thread, between World steps); stats() is also safe
//     from any other thread.
//   - Its workers only ever call IHeightTileFetch::fetch() and touch the
//     loader's own mutex-guarded queues. They NEVER see a ResidentHeightSet -
//     completions reach the set only through poll(), on the owner thread,
//     which installs them between steps.
//   - IHeightTileFetch::fetch() is called concurrently from every worker, so
//     an implementation must be thread-safe (ITransport::send already is,
//     see g2m/server/transport.h).
//
// Status mapping (PLAN.md R2.2 section 1a, exactly):
//   200 Ok (with a tile)       -> Ready.
//   404 NotFound               -> Absent: final and deterministic (outside
//                                 coverage); the owner installs it as holes.
//   500 Internal, 503 Unavail. -> retried, up to LoaderConfig::max_attempts
//                                 fetch calls in total, then Failed.
//   400 BadRequest, 410 Gone   -> Failed at once (retrying cannot help).
//   anything else (304, ...)   -> Failed at once (never produced by a tile
//                                 request without if_none_match).
// A 200 without a tile (an IHeightTileFetch contract violation) and an
// exception thrown out of fetch() are both treated like a 500. Failed is
// reported, never turned into Absent: an Absent install would silently put
// holes where coverage exists.
//
// PLAN.md G2.5a-grip section 3: with LoaderConfig::require_roads set, a 200
// Ok with a tile but a null FetchResult::roads is ALSO treated like a 500
// (retried, then Failed after the budget) - the same "never silently
// missing" rule, extended to roads. require_roads defaults false, so
// heights-only behaviour (every existing caller) is unchanged.
//
// Class mode (terrain migration phase 6, requests G-2/G-5): a fetch may also
// carry the key's g2m.terrain.class tile (FetchResult::classes,
// provides_classes()). LoaderConfig::require_classes treats a 200 without it
// like a 500, the same rule as require_roads. The two are independent: a
// class-mode fetch keeps its RoadSegment metadata (speed limits, road_ahead)
// when it has one, and the loader, ResidentHeightSet and G2mTerrainSource
// keep it beside the class tile.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/layer/release.h"
#include "g2m/layer/terrain_class.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/server/protocol.h"
#include "g2m/server/transport.h"

#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <thread>
#include <vector>

namespace g2m::phys {

// One fetch's outcome. `tile` is set only when status == Status::Ok. Heap
// only: a HeightTile is 256 KiB (vault TOOL-039 - never a value/stack copy).
// `roads` (PLAN.md G2.5a-grip section 3) is that L0 key's owner L2 tile's own
// road segment list - null unless the fetch is road-aware (provides_roads())
// and the underlying data actually has one; a non-null-but-empty list is a
// confirmed "no drivable ways" (RoadBlockStatus::None downstream,
// fill_physics_surfaces.h), distinct from null ("never even attempted" /
// "load failed", RoadBlockStatus::Missing downstream).
struct FetchResult {
    Status status = Status::Internal;
    std::shared_ptr<const HeightTile> tile;
    std::shared_ptr<const std::vector<RoadSegment>> roads;
    // The same key's g2m.terrain.class tile (class mode); null unless the fetch
    // provides_classes(). Independent of `roads`.
    std::shared_ptr<const TerrainClassTile> classes = nullptr;
};

// Fetches one DGM1 L0 height tile. Blocking; must be thread-safe (called
// concurrently from every HeightTileLoader worker).
class IHeightTileFetch {
public:
    virtual ~IHeightTileFetch() = default;
    virtual FetchResult fetch(const TileKey& key) = 0;

    // True iff this fetch populates FetchResult::roads (PLAN.md G2.5a-grip
    // section 3) - a capability query a caller enabling LoaderConfig::
    // require_roads should check before construction (racing_game's own
    // hard-error list, out of this library's scope). Default false:
    // heights-only by default, like TransportHeightTileFetch below.
    virtual bool provides_roads() const { return false; }
    // True iff this fetch populates FetchResult::classes (class mode).
    virtual bool provides_classes() const { return false; }
};

// Fetches through a g2m ITransport (in-process server now, HTTP later):
// TileRequest{rid, layer, key} -> parse_container (header decode plus body
// size/SHA-256 check) -> decode_body -> decode_height_tile. A non-200
// response passes its status straight through; a 200 whose container does
// not parse/decode, or whose header names a different layer/key, becomes
// Status::Internal (retried by the loader - a corrupt or mismatched reply is
// treated as a transient server fault, not as "no data"). A non-zero header
// height_offset (1/256 m, "added to every stored height",
// tile_container.h) is applied to every non-NoData sample; a sample it would
// push outside int32 or onto the NoData sentinel is also Status::Internal.
// The transport must outlive this object.
//
// Class mode: a non-empty `class_layer` (g2m.terrain.class) fetches that
// layer's tile of the same key after a 200 height and decodes it into
// FetchResult::classes (decode_terrain_class). Any non-200 class reply, or a
// class tile that does not decode, makes the whole fetch Status::Internal
// (retried): the class layer has the height layer's coverage, so a missing
// class tile beside a present height tile is a server fault, not "no data".
// Pair it with layer = g2m.terrain.carved for the carved surface.
class TransportHeightTileFetch final : public IHeightTileFetch {
public:
    TransportHeightTileFetch(ITransport& transport, ReleaseId rid,
                             std::string layer = std::string(kTerrainHeightLayer), std::string class_layer = {});
    FetchResult fetch(const TileKey& key) override;
    bool provides_classes() const override { return !class_layer_.empty(); }

private:
    ITransport& transport_;
    ReleaseId rid_;
    std::string layer_;
    std::string class_layer_;
};

// Test/smoke only: sleeps base + jitter before forwarding to `inner`, where
// jitter is a deterministic function of (key, seed) in [0, base] - so the
// same key always takes the same time, but neighbouring keys complete out
// of request order (exercising the loader's sorted poll and the streamer's
// gate under realistic arrival skew). base == 0 forwards without sleeping.
class DelayedFetch final : public IHeightTileFetch {
public:
    DelayedFetch(std::shared_ptr<IHeightTileFetch> inner, std::chrono::milliseconds base, std::uint32_t seed);
    FetchResult fetch(const TileKey& key) override;
    // Forwards `inner`'s own capability (PLAN.md G2.5a-grip section 3).
    bool provides_roads() const override { return inner_->provides_roads(); }
    bool provides_classes() const override { return inner_->provides_classes(); }

    // The delay fetch(key) sleeps for (exposed for tests).
    std::chrono::milliseconds delay_for(const TileKey& key) const;

private:
    std::shared_ptr<IHeightTileFetch> inner_;
    std::chrono::milliseconds base_;
    std::uint32_t seed_;
};

enum class LoadOutcome {
    Ready,  // tile is set.
    Absent, // 404: confirmed no coverage; tile is nullptr.
    Failed, // gave up (see the status mapping above); tile is nullptr.
};

struct LoadedHeightTile {
    TileKey key;
    LoadOutcome outcome = LoadOutcome::Failed;
    std::shared_ptr<const HeightTile> tile;
    std::shared_ptr<const std::vector<RoadSegment>> roads; // set only when outcome == Ready (see FetchResult::roads)
    std::shared_ptr<const TerrainClassTile> classes;        // set only when outcome == Ready (see FetchResult::classes)
    int attempts = 0;    // fetch() calls made for this completion (>= 1)
    Status last_status = Status::Internal; // status of the last attempt (diagnostics)
};

struct LoaderConfig {
    int workers = 2;          // >= 1
    int max_attempts = 4;     // >= 1: total fetch() calls for a retryable status
    // Wait before retry n (n = 1, 2, ...) is backoff * 2^(n-1): 50/100/200 ms
    // at the defaults. Interrupted at once by the destructor.
    std::chrono::milliseconds backoff{50};
    // PLAN.md G2.5a-grip section 3: a 200 Ok with a tile but a null
    // FetchResult::roads is treated like a 500 (Retry, then Failed after
    // max_attempts) instead of Ready - the same "never silently missing"
    // rule as a 200 without a tile at all. false (the default) keeps
    // heights-only behaviour bit-identical.
    bool require_roads = false;
    // Class mode: the same rule for FetchResult::classes. false (the default)
    // keeps every existing caller unchanged.
    bool require_classes = false;
};

struct LoaderStats {
    std::uint64_t requested = 0; // fetch jobs a worker has started (cumulative; one per completion)
    std::uint32_t queued = 0;    // want-list entries not yet picked up by a worker (now)
    std::uint32_t inflight = 0;  // keys a worker is fetching or backing off on (now)
    std::uint64_t ready = 0;     // completions, cumulative, by outcome
    std::uint64_t absent = 0;
    std::uint64_t failed = 0;
    std::uint64_t retries = 0;   // fetch() calls beyond each job's first (cumulative)
};

class HeightTileLoader {
public:
    // Starts config.workers threads. Throws std::invalid_argument on a null
    // fetch, workers < 1, max_attempts < 1 or a negative backoff.
    HeightTileLoader(std::shared_ptr<IHeightTileFetch> fetch, LoaderConfig config);
    // Stops and joins every worker. Safe with requests queued or in flight:
    // queued keys are dropped, a backoff wait is cut short, and a worker
    // blocked inside fetch() is waited for (fetch() cannot be cancelled) -
    // its result is discarded.
    ~HeightTileLoader();

    HeightTileLoader(const HeightTileLoader&) = delete;
    HeightTileLoader& operator=(const HeightTileLoader&) = delete;

    // Replaces the want-list with `priority_ordered` (highest priority
    // first; workers take keys from the front). Keys already in flight keep
    // running and are not queued again; keys already completed but not yet
    // poll()ed are not queued again either (dedup against done); duplicate
    // keys within the list keep their first position. A key that was queued
    // but is absent from the new list is dropped without being fetched.
    // Once a completion has been poll()ed the loader forgets the key, so a
    // later request() for it (e.g. after eviction) fetches it again.
    void request(std::span<const TileKey> priority_ordered);

    // Appends every completion since the last poll to `out`, sorted
    // ascending by TileKey::packed() (arrival order is timing-dependent;
    // the returned order is not), and returns how many it appended.
    std::size_t poll(std::vector<LoadedHeightTile>& out);

    LoaderStats stats() const;

private:
    void worker_main();
    // True when `key` is in flight or completed-not-polled. Caller holds
    // mutex_.
    bool in_flight_or_done_locked(const TileKey& key) const;

    std::shared_ptr<IHeightTileFetch> fetch_;
    LoaderConfig config_;

    mutable std::mutex mutex_;
    std::condition_variable cv_; // work available, or stop_
    bool stop_ = false;
    std::deque<TileKey> want_;                 // priority order, front first
    std::vector<TileKey> inflight_;            // unordered; tiny (<= workers)
    std::vector<LoadedHeightTile> done_;       // completed, not yet polled
    LoaderStats stats_{};

    std::vector<std::thread> workers_;
};

} // namespace g2m::phys
