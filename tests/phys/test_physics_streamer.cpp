// tests/phys/test_physics_streamer.cpp - PhysicsTerrainStreamer (PLAN.md R2.2
// section 1a/4a "streamer" bullets): the gate set is TileManager's own square
// plus gate_margin_tiles, keep/evict (retain_only only over the cap, gate
// keys never evicted), 404 -> Absent (ready), Failed -> frozen and reported,
// request order (gate first, by distance, ties by key), and the queued-key
// invariant: a model of physics_sim's TileManager FIFO pending queue at
// F = 1, driven by a random-walk / straight-line interest point at <= 100 m/s
// that only steps while the gate is ready - every key it pops for a fill must
// have all its L0 keys Resident or Absent at that moment.
//
// Runs against an in-memory IHeightTileFetch (no network, no store files).
// Catch2 assertions only on the test's own thread (vault TOOL-031): the
// fetch, called on loader workers, records into mutex-guarded state.
#include "g2m/phys/physics_streamer.h"

#include "g2m/core/geo/session_frame.h"
#include "g2m/core/geo/utm.h"
#include "g2m/phys/height_tile_loader.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <thread>
#include <vector>

using g2m::Status;
using g2m::TileKey;
using g2m::geo::Hemisphere;
using g2m::geo::SessionFrame;
using g2m::geo::UtmZone;
using g2m::phys::FetchResult;
using g2m::phys::GateStatus;
using g2m::phys::HeightTileLoader;
using g2m::phys::IHeightTileFetch;
using g2m::phys::InterestPoint;
using g2m::phys::LoaderConfig;
using g2m::phys::PhysicsTerrainStreamer;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;
using g2m::phys::ResidentHeightSet;
using g2m::phys::StreamerConfig;
using namespace std::chrono_literals;

namespace {
constexpr UtmZone k32N{32, Hemisphere::North};

// The R2.2 fixture's session frame (PLAN.md R2.2 section 1c).
PhysicsTileGrid make_grid() { return PhysicsTileGrid(SessionFrame(k32N, 474000, 5560000)); }

// In-memory fetch: 404 outside an optional L0 coverage box, 503 for keys
// marked failing, otherwise 200 with one shared tile (content is irrelevant
// here - only residency is under test). Optionally blocks until open().
class SyntheticFetch final : public IHeightTileFetch {
public:
    explicit SyntheticFetch(bool start_closed = false)
        : open_(!start_closed), tile_(std::make_shared<g2m::HeightTile>()) {}

    void set_coverage(std::int32_t x0, std::int32_t y0, std::int32_t x1, std::int32_t y1) {
        std::lock_guard<std::mutex> lk(m_);
        has_coverage_ = true;
        x0_ = x0, y0_ = y0, x1_ = x1, y1_ = y1;
    }
    void set_failing(const TileKey& key) {
        std::lock_guard<std::mutex> lk(m_);
        failing_.insert(key.packed());
    }
    void clear_failing() {
        std::lock_guard<std::mutex> lk(m_);
        failing_.clear();
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
    std::vector<TileKey> order() const {
        std::lock_guard<std::mutex> lk(m_);
        return order_;
    }

    FetchResult fetch(const TileKey& key) override {
        std::unique_lock<std::mutex> lk(m_);
        order_.push_back(key);
        cv_.wait(lk, [this] { return open_; });
        ++calls_[key.packed()];
        if (failing_.count(key.packed()) != 0) return FetchResult{Status::Unavailable, nullptr};
        if (has_coverage_ && (key.x < x0_ || key.x > x1_ || key.y < y0_ || key.y > y1_)) {
            return FetchResult{Status::NotFound, nullptr};
        }
        return FetchResult{Status::Ok, tile_};
    }

private:
    mutable std::mutex m_;
    std::condition_variable cv_;
    bool open_;
    std::shared_ptr<const g2m::HeightTile> tile_;
    bool has_coverage_ = false;
    std::int32_t x0_ = 0, y0_ = 0, x1_ = 0, y1_ = 0;
    std::set<std::uint64_t> failing_;
    std::map<std::uint64_t, int> calls_;
    std::vector<TileKey> order_;
};

LoaderConfig fast_loader(int workers = 2) {
    LoaderConfig c;
    c.workers = workers;
    c.max_attempts = 2;
    c.backoff = 1ms;
    return c;
}

InterestPoint ip_at(double x, double y, double radius = 400.0, double vx = 0.0, double vy = 0.0) {
    InterestPoint p;
    p.id = 0;
    p.x = x;
    p.y = y;
    p.vx = vx;
    p.vy = vy;
    p.radius_m = radius;
    return p;
}

// Calls update() until the gate is ready and the loader is idle (or 20 s).
GateStatus settle(PhysicsTerrainStreamer& streamer, const InterestPoint& p) {
    const std::vector<InterestPoint> points{p};
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    GateStatus s = streamer.update(points);
    while (!(s.ready && s.inflight == 0) && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
        s = streamer.update(points);
    }
    // One more pass so completions that arrived with the last poll are installed.
    return streamer.update(points);
}

std::vector<TileKey> expected_gate(const PhysicsTileGrid& grid, double x, double y, double radius, int margin) {
    const int r = static_cast<int>(std::ceil(radius / grid.tile_size_m())) + margin;
    std::vector<TileKey> out;
    grid.l0_keys_for_square(grid.index_for_local(x, y), r, out);
    return out;
}

bool same_keys(const std::vector<TileKey>& a, const std::vector<TileKey>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!(a[i] == b[i])) return false;
    }
    return true;
}
} // namespace

TEST_CASE("PhysicsTerrainStreamer: gate = TileManager's square + gate_margin_tiles", "[phys][streamer]") {
    const PhysicsTileGrid grid = make_grid();
    ResidentHeightSet set;
    auto fetch = std::make_shared<SyntheticFetch>();
    HeightTileLoader loader(fetch, fast_loader());
    PhysicsTerrainStreamer streamer(grid, set, loader, StreamerConfig{});

    // Radii straddling TileManager's ceil(radius / 255) steps.
    for (const double radius : {0.0, 1.0, 254.0, 255.0, 255.001, 400.0, 510.0, 510.5}) {
        for (const double x : {0.5, 1000.0, -3000.25, 254.999}) {
            const double y = x * 0.5 - 17.0;
            const std::vector<InterestPoint> points{ip_at(x, y, radius, 80.0, -30.0)};
            streamer.update(points);
            CHECK(same_keys(streamer.gate_keys(), expected_gate(grid, x, y, radius, 1)));
        }
    }

    // Hand value: radius 400 -> TileManager r = 2 (5 x 5 tiles), gate r = 3
    // (7 x 7 physics tiles = 1786 m incl. the shared border sample): 7 or 8
    // L0 columns/rows, never more.
    const std::vector<InterestPoint> points{ip_at(1000.0, 1000.0)};
    streamer.update(points);
    CHECK(streamer.gate_keys().size() >= 49);
    CHECK(streamer.gate_keys().size() <= 64);

    // Velocity never changes the gate.
    const std::vector<InterestPoint> moving{ip_at(1000.0, 1000.0, 400.0, 100.0, 100.0)};
    const std::vector<TileKey> gate_still = streamer.gate_keys();
    streamer.update(moving);
    CHECK(same_keys(streamer.gate_keys(), gate_still));
}

TEST_CASE("PhysicsTerrainStreamer: settles ready; keep = gate + prefetch + look-ahead", "[phys][streamer]") {
    const PhysicsTileGrid grid = make_grid();
    ResidentHeightSet set;
    auto fetch = std::make_shared<SyntheticFetch>();
    HeightTileLoader loader(fetch, fast_loader());
    PhysicsTerrainStreamer streamer(grid, set, loader, StreamerConfig{});

    const InterestPoint p = ip_at(1000.0, 1000.0, 400.0, 100.0, 0.0);
    const GateStatus s = settle(streamer, p);
    CHECK(s.ready);
    CHECK(s.missing_required == 0);
    CHECK(s.failed == 0);
    CHECK(s.inflight == 0);

    // keep = gate(r 3) + prefetch (r 2 + 2 = 4) + gate-radius square around
    // the look-ahead point (100 m/s * 3 s = 300 m east, under the 510 m cap).
    std::vector<TileKey> keep = expected_gate(grid, 1000.0, 1000.0, 400.0, 2);
    std::vector<TileKey> ahead = expected_gate(grid, 1300.0, 1000.0, 400.0, 1);
    keep.insert(keep.end(), ahead.begin(), ahead.end());
    std::sort(keep.begin(), keep.end(), [](const TileKey& a, const TileKey& b) { return a.packed() < b.packed(); });
    keep.erase(std::unique(keep.begin(), keep.end()), keep.end());
    CHECK(same_keys(streamer.keep_keys(), keep));
    CHECK(keep.size() <= StreamerConfig{}.max_resident_l0);
    CHECK(set.size() == keep.size());
    for (const TileKey& k : keep) CHECK(set.contains(k));
    CHECK(s.resident == set.size());

    // The look-ahead displacement is capped at lookahead_cap_m (510 m).
    const std::vector<InterestPoint> fast{ip_at(1000.0, 1000.0, 400.0, 1000.0, 0.0)};
    streamer.update(fast);
    std::vector<TileKey> capped = expected_gate(grid, 1000.0, 1000.0, 400.0, 2);
    std::vector<TileKey> capped_ahead = expected_gate(grid, 1510.0, 1000.0, 400.0, 1);
    capped.insert(capped.end(), capped_ahead.begin(), capped_ahead.end());
    std::sort(capped.begin(), capped.end(),
              [](const TileKey& a, const TileKey& b) { return a.packed() < b.packed(); });
    capped.erase(std::unique(capped.begin(), capped.end()), capped.end());
    CHECK(same_keys(streamer.keep_keys(), capped));
}

TEST_CASE("PhysicsTerrainStreamer: evicts only over the cap, never a gate key", "[phys][streamer]") {
    const PhysicsTileGrid grid = make_grid();
    ResidentHeightSet set;
    auto fetch = std::make_shared<SyntheticFetch>();
    HeightTileLoader loader(fetch, fast_loader());

    SECTION("under a large cap nothing is evicted") {
        StreamerConfig c;
        c.max_resident_l0 = 100000;
        PhysicsTerrainStreamer streamer(grid, set, loader, c);
        settle(streamer, ip_at(1000.0, 1000.0));
        const std::vector<TileKey> first_keep = streamer.keep_keys();
        settle(streamer, ip_at(6000.0, 1000.0));
        for (const TileKey& k : first_keep) CHECK(set.contains(k)); // far behind, still resident
        CHECK(set.size() > streamer.keep_keys().size());
    }
    SECTION("over the default cap the set shrinks to exactly keep") {
        PhysicsTerrainStreamer streamer(grid, set, loader, StreamerConfig{});
        settle(streamer, ip_at(1000.0, 1000.0));
        const std::vector<TileKey> first_keep = streamer.keep_keys();
        settle(streamer, ip_at(6000.0, 1000.0));
        CHECK(same_keys(streamer.keep_keys(), expected_gate(grid, 6000.0, 1000.0, 400.0, 2)));
        CHECK(set.size() == streamer.keep_keys().size());
        for (const TileKey& k : first_keep) CHECK_FALSE(set.contains(k)); // 5 km apart: disjoint
    }
    SECTION("cap 0: every update evicts, yet no gate key ever leaves the set") {
        StreamerConfig c;
        c.max_resident_l0 = 0;
        c.prefetch_margin_tiles = 0; // keep == gate (+ look-ahead): the tightest eviction there is
        c.lookahead_s = 0.0;
        PhysicsTerrainStreamer streamer(grid, set, loader, c);
        std::uint64_t gate_keys_checked = 0;
        std::uint64_t evicted_after_ready = 0;
        double x = 100.0;
        for (int step = 0; step < 60; ++step) {
            // Every gate key resident before this update must still be after it.
            const std::vector<TileKey> gate = expected_gate(grid, x, 50.0, 400.0, 1);
            std::vector<TileKey> held;
            for (const TileKey& k : gate) {
                if (set.contains(k)) held.push_back(k);
            }
            const std::vector<InterestPoint> points{ip_at(x, 50.0)};
            const GateStatus s = streamer.update(points);
            for (const TileKey& k : held) {
                ++gate_keys_checked;
                if (!set.contains(k)) ++evicted_after_ready;
            }
            CHECK(set.size() <= streamer.keep_keys().size()); // cap 0: evicted down to keep every update
            if (s.ready) x += 40.0; // move only while ready, like the fixed-rate loop
            else std::this_thread::sleep_for(1ms);
        }
        CHECK(gate_keys_checked > 0);
        CHECK(evicted_after_ready == 0);
    }
}

TEST_CASE("PhysicsTerrainStreamer: 404 outside coverage installs Absent; gate still ready", "[phys][streamer]") {
    const PhysicsTileGrid grid = make_grid();
    ResidentHeightSet set;
    auto fetch = std::make_shared<SyntheticFetch>();
    // L0 coverage: only the 256 m tiles over e 474000-474768, n 5560000-5560768.
    const std::int32_t kx = 474000 / 256, ky = 5560000 / 256;
    fetch->set_coverage(kx, ky, kx + 2, ky + 2);
    HeightTileLoader loader(fetch, fast_loader());
    PhysicsTerrainStreamer streamer(grid, set, loader, StreamerConfig{});
    const GateStatus s = settle(streamer, ip_at(300.0, 300.0));
    CHECK(s.ready);
    CHECK(s.failed == 0);
    std::size_t absent = 0, resident = 0;
    for (const TileKey& k : streamer.gate_keys()) {
        const g2m::HeightTile* t = nullptr;
        const auto lookup = set.find(k, &t);
        CHECK(lookup != ResidentHeightSet::Lookup::Missing);
        if (lookup == ResidentHeightSet::Lookup::Absent) ++absent;
        if (lookup == ResidentHeightSet::Lookup::Resident) ++resident;
    }
    CHECK(absent > 0);
    CHECK(resident > 0);
    CHECK(loader.stats().absent > 0);
}

TEST_CASE("PhysicsTerrainStreamer: a Failed gate key freezes, is reported and not re-requested", "[phys][streamer]") {
    const PhysicsTileGrid grid = make_grid();
    ResidentHeightSet set;
    auto fetch = std::make_shared<SyntheticFetch>();
    const std::vector<TileKey> gate = expected_gate(grid, 1000.0, 1000.0, 400.0, 1);
    const TileKey bad = gate[gate.size() / 2];
    fetch->set_failing(bad);
    HeightTileLoader loader(fetch, fast_loader()); // max_attempts 2
    PhysicsTerrainStreamer streamer(grid, set, loader, StreamerConfig{});
    const std::vector<InterestPoint> points{ip_at(1000.0, 1000.0)};

    GateStatus s{};
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    do {
        std::this_thread::sleep_for(1ms);
        s = streamer.update(points);
    } while (!(s.failed == 1 && s.inflight == 0) && std::chrono::steady_clock::now() < deadline);
    CHECK(s.failed == 1);
    CHECK_FALSE(s.ready);
    CHECK(s.missing_required == 1);
    CHECK_FALSE(set.contains(bad)); // Failed is never installed (never turned into Absent)
    CHECK(fetch->calls(bad) == 2);

    for (int i = 0; i < 20; ++i) {
        s = streamer.update(points);
        std::this_thread::sleep_for(1ms);
    }
    CHECK(fetch->calls(bad) == 2); // remembered: not re-requested every tick
    CHECK_FALSE(s.ready);

    fetch->clear_failing();
    streamer.retry_failed();
    s = settle(streamer, points[0]);
    CHECK(s.ready);
    CHECK(s.failed == 0);
    CHECK(set.contains(bad));
    CHECK(fetch->calls(bad) == 3);
}

TEST_CASE("PhysicsTerrainStreamer: requests gate keys first, each group by distance then key", "[phys][streamer]") {
    const PhysicsTileGrid grid = make_grid();
    ResidentHeightSet set;
    auto fetch = std::make_shared<SyntheticFetch>(/*start_closed=*/true);
    HeightTileLoader loader(fetch, fast_loader(1)); // one worker: fetch order == request order
    PhysicsTerrainStreamer streamer(grid, set, loader, StreamerConfig{});
    const InterestPoint p = ip_at(1000.0, 1000.0);
    const std::vector<InterestPoint> points{p};
    streamer.update(points);
    const std::vector<TileKey> gate = streamer.gate_keys();
    const std::size_t total = streamer.keep_keys().size();
    fetch->open();
    const auto deadline = std::chrono::steady_clock::now() + 20s;
    while (loader.stats().ready < total && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(1ms);
    }
    const std::vector<TileKey> order = fetch->order();
    REQUIRE(order.size() == total);

    const double e0 = 474000.0, n0 = 5560000.0;
    const auto d2 = [&](const TileKey& k) {
        const double cx = static_cast<double>(k.min_easting()) + 128.0 - e0 - p.x;
        const double cy = static_cast<double>(k.min_northing()) + 128.0 - n0 - p.y;
        return cx * cx + cy * cy;
    };
    const auto in_gate = [&](const TileKey& k) {
        return std::find(gate.begin(), gate.end(), k) != gate.end();
    };
    for (std::size_t i = 0; i < order.size(); ++i) {
        CHECK(in_gate(order[i]) == (i < gate.size()));
        if (i + 1 < order.size() && in_gate(order[i]) == in_gate(order[i + 1])) {
            const double a = d2(order[i]), b = d2(order[i + 1]);
            CHECK((a < b || (a == b && order[i].packed() < order[i + 1].packed())));
        }
    }
}

// ---- The queued-key invariant (hold-before-evict) ----

namespace {
// Deterministic, platform-independent uniform doubles (splitmix64), so the
// walk is the same path on every leg.
class Rng {
public:
    explicit Rng(std::uint64_t seed) : s_(seed) {}
    std::uint64_t next() {
        std::uint64_t z = (s_ += 0x9E3779B97F4A7C15ull);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        return z ^ (z >> 31);
    }
    double uniform(double lo, double hi) {
        return lo + (hi - lo) * (static_cast<double>(next() >> 11) * (1.0 / 9007199254740992.0));
    }

private:
    std::uint64_t s_;
};

// A model of physics_sim's TileManager::update() steps 1-3 (core/src/terrain/
// tile_manager.cpp, read-only reference): needed square r = ceil(radius/ts)
// around key_for_world_xy; newly-needed keys not resident and not already
// queued are appended to a FIFO pending queue that is NEVER pruned of stale
// keys; at most `fills_per_tick` keys are popped per tick, each taking a free
// pool slot or else the first assigned slot no longer needed (else it stays
// queued - starved). The popped keys are the ones fill_tile() runs for.
class TileManagerModel {
public:
    TileManagerModel(const PhysicsTileGrid& grid, std::size_t pool, std::size_t fills_per_tick)
        : grid_(grid), slots_(pool), fills_per_tick_(fills_per_tick) {
        for (std::size_t i = pool; i > 0; --i) free_.push_back(i - 1);
    }

    struct Popped {
        std::int64_t key;
        std::uint64_t queued_tick;
    };

    void update(double x, double y, double radius, std::uint64_t tick, std::vector<Popped>& filled) {
        filled.clear();
        needed_.clear();
        const PhysTileIndex c = grid_.index_for_local(x, y);
        const auto r = static_cast<std::int32_t>(std::ceil(radius / grid_.tile_size_m()));
        for (std::int32_t dy = -r; dy <= r; ++dy) {
            for (std::int32_t dx = -r; dx <= r; ++dx) needed_.push_back(PhysicsTileGrid::pack({c.ix + dx, c.iy + dy}));
        }
        for (const std::int64_t k : needed_) {
            if (resident(k)) continue;
            bool queued = false;
            for (const auto& q : pending_) queued = queued || q.key == k;
            if (!queued) pending_.push_back(Popped{k, tick});
        }
        std::deque<Popped> keep;
        std::size_t processed = 0;
        for (const Popped& q : pending_) {
            if (processed >= fills_per_tick_) {
                keep.push_back(q);
                continue;
            }
            const std::size_t slot = acquire();
            if (slot == kNone) {
                ++starved_;
                keep.push_back(q);
                continue;
            }
            slots_[slot] = Slot{true, q.key};
            filled.push_back(q);
            ++processed;
        }
        pending_.swap(keep);
    }

    std::size_t pending() const { return pending_.size(); }
    std::uint64_t starved() const { return starved_; }

private:
    struct Slot {
        bool assigned = false;
        std::int64_t key = 0;
    };
    static constexpr std::size_t kNone = static_cast<std::size_t>(-1);

    bool resident(std::int64_t k) const {
        for (const Slot& s : slots_) {
            if (s.assigned && s.key == k) return true;
        }
        return false;
    }
    std::size_t acquire() {
        if (!free_.empty()) {
            const std::size_t i = free_.back();
            free_.pop_back();
            return i;
        }
        for (std::size_t i = 0; i < slots_.size(); ++i) {
            if (!slots_[i].assigned) continue;
            if (std::find(needed_.begin(), needed_.end(), slots_[i].key) == needed_.end()) return i;
        }
        return kNone;
    }

    const PhysicsTileGrid& grid_;
    std::vector<Slot> slots_;
    std::vector<std::size_t> free_;
    std::size_t fills_per_tick_;
    std::deque<Popped> pending_;
    std::vector<std::int64_t> needed_;
    std::uint64_t starved_ = 0;
};

enum class Walk { Random, Diagonal, Zigzag, SeamDither };

struct InvariantResult {
    std::uint64_t ticks = 0;
    std::uint64_t fills = 0;
    std::uint64_t violations = 0;     // popped keys with a Missing L0 key
    std::uint64_t freezes = 0;        // ready -> not-ready transitions after start-up
    std::uint64_t max_queue_lag = 0;  // ticks a popped key waited in the queue
    std::uint64_t max_queue_lag_steady = 0; // same, for keys queued from tick 26 on (start-up square drained)
    std::uint64_t starved = 0;
    bool timed_out = false;
    double distance_m = 0.0;
};

InvariantResult run_invariant(Walk walk, StreamerConfig config, std::chrono::milliseconds fetch_delay,
                              std::uint64_t ticks, std::uint64_t seed) {
    const PhysicsTileGrid grid = make_grid();
    ResidentHeightSet set;
    std::shared_ptr<IHeightTileFetch> fetch = std::make_shared<SyntheticFetch>();
    if (fetch_delay.count() > 0) fetch = std::make_shared<g2m::phys::DelayedFetch>(fetch, fetch_delay, 99u);
    HeightTileLoader loader(fetch, fast_loader(2));
    PhysicsTerrainStreamer streamer(grid, set, loader, config);
    TileManagerModel tm(grid, /*pool=*/49, /*fills_per_tick=*/1); // make_terrain_config(400, 1)

    constexpr double kRadius = 400.0;
    constexpr double kDt = 1.0 / 240.0;
    constexpr double kMaxSpeed = 100.0;
    Rng rng(seed);
    double x = 1000.0, y = -700.0, vx = 0.0, vy = 0.0;
    if (walk == Walk::Diagonal) vx = vy = kMaxSpeed / std::sqrt(2.0);
    if (walk == Walk::Zigzag) vx = kMaxSpeed;
    // SeamDither: +-1 m around the western seam of x's physics tile, at full
    // speed - a seam crossing every ~5 ticks, so keys TileManager queued on
    // one side are still queued when the car is back on the other.
    const double seam_x = grid.origin_local(grid.index_for_local(x, y))[0];
    if (walk == Walk::SeamDither) {
        x = seam_x + 0.25;
        vx = kMaxSpeed;
    }

    InvariantResult res;
    std::vector<TileManagerModel::Popped> filled;
    std::vector<TileKey> l0;
    bool was_ready = false;
    const auto deadline = std::chrono::steady_clock::now() + 120s;
    while (res.ticks < ticks) {
        const std::vector<InterestPoint> points{ip_at(x, y, kRadius, vx, vy)};
        const GateStatus s = streamer.update(points);
        if (!s.ready) {
            if (was_ready) ++res.freezes;
            was_ready = false;
            if (std::chrono::steady_clock::now() > deadline) {
                res.timed_out = true;
                break;
            }
            std::this_thread::yield();
            continue; // frozen: the World is not stepped, the car does not move
        }
        was_ready = true;

        // World::set_terrain_interest_point(p) + World::step(): TileManager
        // runs after backend_step; its fills read the set right now.
        tm.update(x, y, kRadius, res.ticks, filled);
        for (const auto& f : filled) {
            ++res.fills;
            res.max_queue_lag = std::max(res.max_queue_lag, res.ticks - f.queued_tick);
            if (f.queued_tick >= 26) res.max_queue_lag_steady = std::max(res.max_queue_lag_steady, res.ticks - f.queued_tick);
            grid.l0_keys_for_square(PhysicsTileGrid::unpack(f.key), 0, l0);
            for (const TileKey& k : l0) {
                if (!set.contains(k)) ++res.violations;
            }
        }
        ++res.ticks;

        // Move (sim state only).
        switch (walk) {
        case Walk::Random:
            vx += rng.uniform(-40.0, 40.0) * kDt * 60.0;
            vy += rng.uniform(-40.0, 40.0) * kDt * 60.0;
            if (rng.uniform(0.0, 1.0) < 1.0 / 400.0) { // occasional hard turn at full speed
                const double a = rng.uniform(0.0, 6.283185307179586);
                const double c = rng.uniform(-1.0, 1.0);
                const double sgn = a < 3.14159 ? 1.0 : -1.0;
                vx = kMaxSpeed * c;
                vy = sgn * kMaxSpeed * std::sqrt(1.0 - c * c);
            }
            break;
        case Walk::Diagonal:
            break;
        case Walk::Zigzag: // reverse every 1.5 s: repeatedly re-cross the same seams
            if (res.ticks % 360 == 0) vx = -vx;
            break;
        case Walk::SeamDither:
            if ((vx > 0.0 && x > seam_x + 1.0) || (vx < 0.0 && x < seam_x - 1.0)) vx = -vx;
            break;
        }
        const double speed = std::sqrt(vx * vx + vy * vy);
        if (speed > kMaxSpeed) {
            vx *= kMaxSpeed / speed;
            vy *= kMaxSpeed / speed;
        }
        x += vx * kDt;
        y += vy * kDt;
        res.distance_m += std::sqrt(vx * vx + vy * vy) * kDt;
    }
    res.starved = tm.starved();
    return res;
}
} // namespace

TEST_CASE("PhysicsTerrainStreamer: every key TileManager pops at F = 1 is resident (queued-key invariant)",
          "[phys][streamer]") {
    struct Case {
        const char* name;
        Walk walk;
        std::uint64_t ticks;
        std::size_t cap;
        int prefetch;
        double lookahead_s;
        std::chrono::milliseconds delay;
    };
    // 8000 ticks = 33 s at 240 Hz, up to 3.3 km. cap 0 + no prefetch/look-
    // ahead is the tightest the streamer can be: the set holds exactly the
    // gate after every update, so only the gate margin protects queued keys.
    const Case cases[] = {
        {"random walk, defaults", Walk::Random, 8000, 160, 2, 3.0, 0ms},
        {"random walk, cap 0, no prefetch/look-ahead", Walk::Random, 8000, 0, 0, 0.0, 0ms},
        {"diagonal 100 m/s, cap 0, no prefetch, delayed fetch", Walk::Diagonal, 8000, 0, 0, 0.0, 1ms},
        {"zigzag 100 m/s, cap 0, no prefetch", Walk::Zigzag, 8000, 0, 0, 0.0, 0ms},
        {"seam dither 100 m/s, cap 0, no prefetch", Walk::SeamDither, 2000, 0, 0, 0.0, 0ms},
    };
    std::uint64_t seed = 0x5EEDu;
    for (const Case& c : cases) {
        StreamerConfig config;
        config.max_resident_l0 = c.cap;
        config.prefetch_margin_tiles = c.prefetch;
        config.lookahead_s = c.lookahead_s;
        const InvariantResult r = run_invariant(c.walk, config, c.delay, c.ticks, seed++);
        INFO(c.name << ": ticks " << r.ticks << ", fills " << r.fills << ", freezes " << r.freezes
                    << ", max queue lag " << r.max_queue_lag << " (steady " << r.max_queue_lag_steady
                    << "), distance " << r.distance_m << " m");
        CHECK_FALSE(r.timed_out);
        CHECK(r.ticks == c.ticks);
        CHECK(r.violations == 0);
        CHECK(r.starved == 0);
        CHECK(r.fills > 25);                 // more than the start-up square: seams were crossed
        CHECK(r.max_queue_lag <= 26);        // PLAN.md R2.2 section 1a's Q bound at F = 1
        CHECK(r.max_queue_lag_steady <= 13); // ... and in steady state
        CHECK(r.distance_m > 150.0);
    }
}

TEST_CASE("PhysicsTerrainStreamer: queued-key invariant negative control (gate_margin_tiles = 0)",
          "[phys][streamer]") {
    // Without the +1 margin the gate is exactly TileManager's current square,
    // so keys it queued on the other side of a seam a few ticks ago are
    // evicted before it pops them - proving the invariant check above can
    // see a violation at all.
    StreamerConfig config;
    config.gate_margin_tiles = 0;
    config.max_resident_l0 = 0;
    config.prefetch_margin_tiles = 0;
    config.lookahead_s = 0.0;
    const InvariantResult r = run_invariant(Walk::SeamDither, config, 0ms, 2000, 0xBADu);
    INFO("ticks " << r.ticks << ", fills " << r.fills << ", violations " << r.violations);
    CHECK_FALSE(r.timed_out);
    CHECK(r.violations > 0);
}
