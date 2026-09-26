// src/phys/height_tile_loader.cpp - see g2m/phys/height_tile_loader.h.
#include "g2m/phys/height_tile_loader.h"

#include "g2m/core/raster.h"

#include <algorithm>
#include <limits>
#include <stdexcept>
#include <utility>
#include <variant>

namespace g2m::phys {

// ---- TransportHeightTileFetch ----

TransportHeightTileFetch::TransportHeightTileFetch(ITransport& transport, ReleaseId rid, std::string layer)
    : transport_(transport), rid_(rid), layer_(std::move(layer)) {}

FetchResult TransportHeightTileFetch::fetch(const TileKey& key) {
    const Response response = transport_.send(Request{TileRequest{rid_, layer_, key, std::nullopt}});
    const auto* tile_response = std::get_if<TileResponse>(&response);
    if (tile_response == nullptr) return FetchResult{Status::Internal, nullptr};
    if (tile_response->meta.status != Status::Ok) return FetchResult{tile_response->meta.status, nullptr};

    const Result<ContainerView> view = parse_container(tile_response->container);
    if (!view) return FetchResult{Status::Internal, nullptr};
    if (view->header.layer != layer_ || !(view->header.key == key)) return FetchResult{Status::Internal, nullptr};
    const Result<TileBody> body = decode_body(view->body);
    if (!body) return FetchResult{Status::Internal, nullptr};

    // Heap from the start: HeightTile is 256 KiB (vault TOOL-039).
    auto tile = std::make_shared<HeightTile>();
    if (!decode_height_tile(*body, key, *tile)) return FetchResult{Status::Internal, nullptr};

    const std::int32_t offset = view->header.height_offset;
    if (offset != 0) {
        for (std::int32_t& h : tile->h) {
            if (h == kHeightNoData) continue;
            const std::int64_t v = static_cast<std::int64_t>(h) + offset;
            if (v <= static_cast<std::int64_t>(kHeightNoData) ||
                v > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
                return FetchResult{Status::Internal, nullptr};
            }
            h = static_cast<std::int32_t>(v);
        }
    }
    return FetchResult{Status::Ok, std::move(tile)};
}

// ---- DelayedFetch ----

namespace {
// splitmix64 finaliser: a fixed, platform-independent integer mix.
std::uint64_t mix64(std::uint64_t z) {
    z += 0x9E3779B97F4A7C15ull;
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}
} // namespace

DelayedFetch::DelayedFetch(std::shared_ptr<IHeightTileFetch> inner, std::chrono::milliseconds base,
                           std::uint32_t seed)
    : inner_(std::move(inner)), base_(base), seed_(seed) {
    if (!inner_) throw std::invalid_argument("DelayedFetch: inner fetch is null");
    if (base_.count() < 0) throw std::invalid_argument("DelayedFetch: negative base delay");
}

std::chrono::milliseconds DelayedFetch::delay_for(const TileKey& key) const {
    if (base_.count() <= 0) return std::chrono::milliseconds{0};
    const std::uint64_t h = mix64(key.packed() ^ (static_cast<std::uint64_t>(seed_) << 32 | seed_));
    const auto range = static_cast<std::uint64_t>(base_.count()) + 1; // jitter in [0, base]
    return base_ + std::chrono::milliseconds{static_cast<std::chrono::milliseconds::rep>(h % range)};
}

FetchResult DelayedFetch::fetch(const TileKey& key) {
    const std::chrono::milliseconds delay = delay_for(key);
    if (delay.count() > 0) std::this_thread::sleep_for(delay);
    return inner_->fetch(key);
}

// ---- HeightTileLoader ----

namespace {
enum class Disposition { Ready, Absent, Retry, Fail };

Disposition classify(const FetchResult& r) {
    switch (r.status) {
    case Status::Ok:
        return r.tile ? Disposition::Ready : Disposition::Retry; // 200 without a tile: treat like a 500
    case Status::NotFound:
        return Disposition::Absent;
    case Status::Internal:
    case Status::Unavailable:
        return Disposition::Retry;
    case Status::BadRequest:
    case Status::Gone:
    case Status::NotModified:
    default:
        return Disposition::Fail;
    }
}

bool same_key(const TileKey& a, const TileKey& b) { return a.packed() == b.packed(); }
} // namespace

HeightTileLoader::HeightTileLoader(std::shared_ptr<IHeightTileFetch> fetch, LoaderConfig config)
    : fetch_(std::move(fetch)), config_(config) {
    if (!fetch_) throw std::invalid_argument("HeightTileLoader: fetch is null");
    if (config_.workers < 1) throw std::invalid_argument("HeightTileLoader: workers must be >= 1");
    if (config_.max_attempts < 1) throw std::invalid_argument("HeightTileLoader: max_attempts must be >= 1");
    if (config_.backoff.count() < 0) throw std::invalid_argument("HeightTileLoader: negative backoff");
    workers_.reserve(static_cast<std::size_t>(config_.workers));
    for (int i = 0; i < config_.workers; ++i) workers_.emplace_back([this] { worker_main(); });
}

HeightTileLoader::~HeightTileLoader() {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        stop_ = true;
        want_.clear();
    }
    cv_.notify_all();
    for (std::thread& t : workers_) {
        if (t.joinable()) t.join();
    }
}

bool HeightTileLoader::in_flight_or_done_locked(const TileKey& key) const {
    for (const TileKey& k : inflight_) {
        if (same_key(k, key)) return true;
    }
    for (const LoadedHeightTile& d : done_) {
        if (same_key(d.key, key)) return true;
    }
    return false;
}

void HeightTileLoader::request(std::span<const TileKey> priority_ordered) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        want_.clear();
        for (const TileKey& key : priority_ordered) {
            if (in_flight_or_done_locked(key)) continue;
            bool dup = false;
            for (const TileKey& q : want_) {
                if (same_key(q, key)) {
                    dup = true;
                    break;
                }
            }
            if (!dup) want_.push_back(key);
        }
        stats_.queued = static_cast<std::uint32_t>(want_.size());
    }
    cv_.notify_all();
}

std::size_t HeightTileLoader::poll(std::vector<LoadedHeightTile>& out) {
    std::vector<LoadedHeightTile> batch;
    {
        std::lock_guard<std::mutex> lk(mutex_);
        batch.swap(done_);
    }
    std::sort(batch.begin(), batch.end(),
              [](const LoadedHeightTile& a, const LoadedHeightTile& b) { return a.key.packed() < b.key.packed(); });
    for (LoadedHeightTile& d : batch) out.push_back(std::move(d));
    return batch.size();
}

LoaderStats HeightTileLoader::stats() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return stats_;
}

void HeightTileLoader::worker_main() {
    std::unique_lock<std::mutex> lk(mutex_);
    for (;;) {
        cv_.wait(lk, [this] { return stop_ || !want_.empty(); });
        if (stop_) return;

        const TileKey key = want_.front();
        want_.pop_front();
        inflight_.push_back(key);
        stats_.queued = static_cast<std::uint32_t>(want_.size());
        stats_.inflight = static_cast<std::uint32_t>(inflight_.size());
        ++stats_.requested;

        LoadedHeightTile result;
        result.key = key;
        bool abandoned = false;
        for (int attempt = 1;; ++attempt) {
            lk.unlock();
            FetchResult fr;
            try {
                fr = fetch_->fetch(key);
            } catch (...) {
                fr = FetchResult{Status::Internal, nullptr}; // an escaping exception would terminate the process
            }
            lk.lock();

            result.attempts = attempt;
            result.last_status = fr.status;
            const Disposition d = classify(fr);
            if (d == Disposition::Ready) {
                result.outcome = LoadOutcome::Ready;
                result.tile = std::move(fr.tile);
                break;
            }
            if (d == Disposition::Absent) {
                result.outcome = LoadOutcome::Absent;
                break;
            }
            if (d == Disposition::Fail || attempt >= config_.max_attempts) {
                result.outcome = LoadOutcome::Failed;
                break;
            }
            if (stop_) {
                abandoned = true;
                break;
            }
            ++stats_.retries;
            // backoff * 2^(attempt-1), shift capped so it can never overflow.
            const auto wait = config_.backoff * (std::int64_t{1} << std::min(attempt - 1, 16));
            if (cv_.wait_for(lk, wait, [this] { return stop_; })) {
                abandoned = true;
                break;
            }
        }

        inflight_.erase(std::find_if(inflight_.begin(), inflight_.end(),
                                     [&](const TileKey& k) { return same_key(k, key); }));
        stats_.inflight = static_cast<std::uint32_t>(inflight_.size());
        if (abandoned || stop_) continue; // loop top returns on stop_; nothing is reported after shutdown
        switch (result.outcome) {
        case LoadOutcome::Ready: ++stats_.ready; break;
        case LoadOutcome::Absent: ++stats_.absent; break;
        case LoadOutcome::Failed: ++stats_.failed; break;
        }
        done_.push_back(std::move(result));
    }
}

} // namespace g2m::phys
