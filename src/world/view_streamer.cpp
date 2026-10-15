// g2m/world/view_streamer.cpp - see include/g2m/world/view_streamer.h. Ported from racing_game 5a3baed
// (world_terrain.cpp, terrain_view_streamer.cpp, terrain_stream.cpp, lod_settings.cpp) statement for statement; the
// only arithmetic change is std::hypot -> det::hypot. tests/world/test_view_streamer.cpp pins it against the game.
//
// Why one diff at a time, coalesced to the latest focus: every diff is exactly target(n) vs target(n-1), so the
// "removals only after all adds" rule stays local to one diff. Queueing every due selection would upload stale
// intermediate diffs; cancel-and-restart on every move could starve at speed. commit() starts the follow-up at once.
//
// Determinism: select_chunks is pure, the key sets are sorted vectors diffed with set_difference, and
// build_view_chunks writes each chunk by index, so a diff is byte-identical for any build_threads.
#include "g2m/world/view_streamer.h"

#include "g2m/core/det_math.h"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <utility>

namespace g2m::world {

namespace {

constexpr double kMinDirectionSpeedMps = 2.0;
constexpr double kBehindPenaltyS = 1000.0;
constexpr double kTimeBinS = 0.25;

double clamp01(double v) { return std::clamp(v, 0.0, 1.0); }

void build_one_chunk(const mesh::ChunkKey& key, const ViewSource& source, ViewChunk& out_chunk) {
    mesh::HeightWindow window;
    mesh::gather_window(key, source.lookup, source.ctx, window);
    // After the gather: every tile of the window has been produced now, so its state is final for THIS build.
    out_chunk.carve = chunk_window_carve(key, source.ctx, source.carve_state);
    if (source.class_lookup.fn == nullptr) {
        mesh::build_chunk(key, window, /*classes=*/nullptr, out_chunk.mesh);
    } else {
        mesh::ClassWindow classes;
        rasterize_chunk_road_classes(key, source.class_lookup, classes);
        mesh::build_chunk(key, window, &classes, out_chunk.mesh);
    }
    out_chunk.key = key;
    out_chunk.origin_session[0] = out_chunk.mesh.origin[0] - source.e0;
    out_chunk.origin_session[1] = out_chunk.mesh.origin[1] - source.n0;
    out_chunk.origin_session[2] = out_chunk.mesh.origin[2];
    out_chunk.rgba.clear();
    if (source.colour) source.colour(out_chunk);
}

unsigned auto_initial_threads() {
    const unsigned hw = std::thread::hardware_concurrency();
    return hw == 0 ? 4u : std::min(hw, 8u);
}

} // namespace

ChunkCarve chunk_window_carve(const mesh::ChunkKey& key, void* ctx, TileCarveFn fn) {
    if (fn == nullptr) return ChunkCarve::NotCarved;
    const auto tile_index = [](std::int64_t sample) { return floor_div(sample, 256); };
    ChunkCarve worst = ChunkCarve::NotCarved;
    for (std::int64_t ty = tile_index(64LL * key.cy - 1); ty <= tile_index(64LL * key.cy + 65); ++ty)
        for (std::int64_t tx = tile_index(64LL * key.cx - 1); tx <= tile_index(64LL * key.cx + 65); ++tx) {
            const TileKey tk{key.zone, key.level, static_cast<std::int32_t>(tx), static_cast<std::int32_t>(ty)};
            const auto state = static_cast<ChunkCarve>(fn(ctx, tk));
            if (state == ChunkCarve::Fallback) return ChunkCarve::Fallback;
            if (state == ChunkCarve::Carved) worst = ChunkCarve::Carved;
        }
    return worst;
}

void select_view_keys(double cam_x, double cam_y, const mesh::LodParams& params, double e0, double n0,
                      std::vector<mesh::ChunkKey>& out) {
    mesh::select_chunks(cam_x + e0, cam_y + n0, params, out);
}

bool build_view_chunks(const std::vector<mesh::ChunkKey>& keys, const ViewSource& source, unsigned thread_count,
                       std::vector<ViewChunk>& out, const std::atomic<bool>* cancel) {
    out.clear();
    out.resize(keys.size());
    if (keys.empty()) {
        return true;
    }
    auto cancelled = [cancel]() { return cancel != nullptr && cancel->load(std::memory_order_relaxed); };

    const unsigned n_threads = std::max(1u, std::min(thread_count, static_cast<unsigned>(keys.size())));
    if (n_threads <= 1) {
        for (std::size_t i = 0; i < keys.size(); ++i) {
            if (cancelled()) {
                return false;
            }
            build_one_chunk(keys[i], source, out[i]);
        }
        return !cancelled();
    }

    const std::size_t total = keys.size();
    const std::size_t per_thread = (total + n_threads - 1) / n_threads;
    std::vector<std::thread> workers;
    workers.reserve(n_threads);
    for (unsigned t = 0; t < n_threads; ++t) {
        const std::size_t begin = static_cast<std::size_t>(t) * per_thread;
        if (begin >= total) {
            break;
        }
        const std::size_t end = std::min(total, begin + per_thread);
        workers.emplace_back([&keys, &out, &cancelled, &source, begin, end]() {
            for (std::size_t i = begin; i < end; ++i) {
                if (cancelled()) {
                    return;
                }
                build_one_chunk(keys[i], source, out[i]);
            }
        });
    }
    for (std::thread& w : workers) {
        w.join();
    }
    return !cancelled();
}

void build_static_view(double cam_x, double cam_y, const ViewSource& source, unsigned thread_count,
                       std::vector<ViewChunk>& out) {
    std::vector<mesh::ChunkKey> keys;
    select_view_keys(cam_x, cam_y, source.params, source.e0, source.n0, keys);
    build_view_chunks(keys, source, thread_count, out, /*cancel=*/nullptr);
}

double reselect_distance_for_speed(double base_m, double speed_mps, double min_fraction, double speed_lo_mps,
                                   double speed_hi_mps) {
    if (!(speed_mps > speed_lo_mps)) {
        return base_m;
    }
    const double t = clamp01((speed_mps - speed_lo_mps) / std::max(1e-9, speed_hi_mps - speed_lo_mps));
    return base_m * (1.0 - (1.0 - clamp01(min_fraction)) * t);
}

double chunk_time_to_reach_s(const mesh::ChunkKey& key, double e0, double n0, double px, double py, double vx,
                             double vy) {
    const double size = static_cast<double>(mesh::kChunkQuads) * std::ldexp(1.0, key.level);
    const double half = size * 0.5;
    // Chunk centre relative to the car, in session-local axes.
    const double cx = static_cast<double>(key.cx) * size + half - e0 - px;
    const double cy = static_cast<double>(key.cy) * size + half - n0 - py;
    const double speed = det::hypot(vx, vy);
    if (!(speed >= kMinDirectionSpeedMps)) {
        const double gx = std::max(0.0, std::abs(cx) - half);
        const double gy = std::max(0.0, std::abs(cy) - half);
        return det::hypot(gx, gy) / kMinDirectionSpeedMps;
    }
    const double ux = vx / speed;
    const double uy = vy / speed;
    const double along = cx * ux + cy * uy;
    const double lateral = std::abs(-cx * uy + cy * ux);
    const double r = half * (std::abs(ux) + std::abs(uy)); // footprint half-extent along u and across it
    const double lateral_gap = std::max(0.0, lateral - r);
    if (along + r < 0.0) {
        return kBehindPenaltyS + det::hypot(-(along + r), lateral_gap) / speed;
    }
    const double ahead = std::max(0.0, along - r);
    return det::hypot(ahead, lateral_gap) / speed;
}

void sort_adds_by_urgency(std::vector<ViewChunk>& added, double e0, double n0, double px, double py, double vx,
                          double vy) {
    if (added.size() < 2) {
        return;
    }
    struct Entry {
        double time_s;
        long long bin;
        std::size_t index;
    };
    std::vector<Entry> order(added.size());
    for (std::size_t i = 0; i < added.size(); ++i) {
        const double t = chunk_time_to_reach_s(added[i].key, e0, n0, px, py, vx, vy);
        order[i] = Entry{t, static_cast<long long>(std::floor(t / kTimeBinS)), i};
    }
    std::sort(order.begin(), order.end(), [&added](const Entry& a, const Entry& b) {
        if (a.bin != b.bin) {
            return a.bin < b.bin;
        }
        const auto& ka = added[a.index].key;
        const auto& kb = added[b.index].key;
        if (ka.level != kb.level) {
            return ka.level < kb.level; // finest level first
        }
        if (a.time_s != b.time_s) {
            return a.time_s < b.time_s;
        }
        return ka < kb;
    });
    std::vector<ViewChunk> sorted;
    sorted.reserve(added.size());
    for (const Entry& e : order) {
        sorted.push_back(std::move(added[e.index]));
    }
    added = std::move(sorted);
}

std::vector<int> changed_road_lod_levels(const RenderRoadLod* a, const RenderRoadLod* b, int max_level) {
    const RenderRoadLod& pa = a != nullptr ? *a : default_render_road_lod();
    const RenderRoadLod& pb = b != nullptr ? *b : default_render_road_lod();
    std::vector<int> out;
    for (int level = 0; level <= max_level; ++level) {
        if (render_road_lod_key(pa, level) != render_road_lod_key(pb, level)) out.push_back(level);
    }
    return out;
}

// ---- ViewStreamer ------------------------------------------------------------

ViewStreamer::ViewStreamer(ViewSource source, Options options) : source_(std::move(source)), options_(options) {
    worker_ = std::thread([this]() { worker_main(); });
}

ViewStreamer::~ViewStreamer() {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        stop_ = true;
    }
    cancel_.store(true, std::memory_order_relaxed);
    cv_.notify_all();
    if (worker_.joinable()) {
        worker_.join();
    }
}

bool ViewStreamer::build_initial(double x, double y, std::vector<ViewChunk>& out) {
    {
        std::lock_guard<std::mutex> lk(mutex_);
        if (phase_ != Phase::Idle) {
            return false;
        }
    }
    // Idle and every public method runs on the one adapter thread, so the worker cannot start a job meanwhile.
    std::vector<mesh::ChunkKey> keys;
    select_view_keys(x, y, source_.params, source_.e0, source_.n0, keys);
    const unsigned threads =
        options_.initial_build_threads == 0 ? auto_initial_threads() : options_.initial_build_threads;
    build_view_chunks(keys, source_, threads, out, /*cancel=*/nullptr);

    std::sort(keys.begin(), keys.end());
    std::lock_guard<std::mutex> lk(mutex_);
    resident_ = std::move(keys);
    resident_road_lod_ = source_.road_lod;
    source_dirty_ = false;
    has_selection_ = true;
    selection_x_ = x;
    selection_y_ = y;
    stats_.resident_count = resident_.size();
    return true;
}

void ViewStreamer::update_focus(double x, double y, double vx, double vy, double ref_x, double ref_y) {
    std::lock_guard<std::mutex> lk(mutex_);
    latest_x_ = x;
    latest_y_ = y;
    latest_vx_ = vx;
    latest_vy_ = vy;
    latest_rx_ = std::isnan(ref_x) ? x : ref_x;
    latest_ry_ = std::isnan(ref_y) ? y : ref_y;
    has_latest_ = true;
    maybe_start_locked();
}

void ViewStreamer::set_source(const ViewSource& source) {
    std::lock_guard<std::mutex> lk(mutex_);
    source_ = source;
    source_dirty_ = true;
    if (!has_latest_ && has_selection_) { // no focus update yet: reselect where the last selection was
        latest_x_ = latest_rx_ = selection_x_;
        latest_y_ = latest_ry_ = selection_y_;
        has_latest_ = true;
    }
    maybe_start_locked();
}

void ViewStreamer::maybe_start_locked() {
    if (phase_ != Phase::Idle || !has_latest_ || stop_) {
        return;
    }
    if (has_selection_ && !source_dirty_) {
        const double dx = latest_x_ - selection_x_;
        const double dy = latest_y_ - selection_y_;
        const double r = reselect_distance_for_speed(options_.reselect_distance_m, det::hypot(latest_vx_, latest_vy_));
        if (dx * dx + dy * dy <= r * r) {
            return;
        }
    }
    has_selection_ = true;
    source_dirty_ = false;
    selection_x_ = latest_x_;
    selection_y_ = latest_y_;
    job_x_ = latest_x_;
    job_y_ = latest_y_;
    job_vx_ = latest_vx_;
    job_vy_ = latest_vy_;
    job_rx_ = latest_rx_;
    job_ry_ = latest_ry_;
    job_serial_ = next_serial_++;
    job_pending_ = true;
    phase_ = Phase::Building;
    ++stats_.selections_started;
    cv_.notify_one();
}

bool ViewStreamer::poll(ViewDiff& out) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (phase_ != Phase::Ready) {
        return false;
    }
    out = std::move(ready_);
    ready_ = ViewDiff{};
    applying_serial_ = out.serial;
    phase_ = Phase::Applying;
    return true;
}

bool ViewStreamer::commit(std::uint64_t serial) {
    std::lock_guard<std::mutex> lk(mutex_);
    if (phase_ != Phase::Applying || serial != applying_serial_) {
        return false;
    }
    resident_ = std::move(target_);
    target_.clear();
    resident_road_lod_ = target_road_lod_;
    phase_ = Phase::Idle;
    ++stats_.diffs_committed;
    stats_.resident_count = resident_.size();
    maybe_start_locked();
    return true;
}

ViewStreamer::Phase ViewStreamer::phase() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return phase_;
}

ViewStreamer::Stats ViewStreamer::stats() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return stats_;
}

std::vector<mesh::ChunkKey> ViewStreamer::resident_keys() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return resident_;
}

void ViewStreamer::worker_main() {
    for (;;) {
        std::vector<mesh::ChunkKey> resident;
        double x = 0.0, y = 0.0, vx = 0.0, vy = 0.0, rx = 0.0, ry = 0.0;
        std::uint64_t serial = 0;
        ViewSource src;
        std::shared_ptr<const RenderRoadLod> resident_lod;
        {
            std::unique_lock<std::mutex> lk(mutex_);
            cv_.wait(lk, [this]() { return stop_ || job_pending_; });
            if (stop_) {
                return;
            }
            job_pending_ = false;
            resident = resident_; // resident_ never changes while Building; a copy keeps the build lock-free
            x = job_x_;
            y = job_y_;
            vx = job_vx_;
            vy = job_vy_;
            rx = job_rx_;
            ry = job_ry_;
            serial = job_serial_;
            src = source_;
            resident_lod = resident_road_lod_;
        }

        const auto start = std::chrono::steady_clock::now();

        std::vector<mesh::ChunkKey> selected;
        select_view_keys(x, y, src.params, src.e0, src.n0, selected);
        std::sort(selected.begin(), selected.end());
        selected.erase(std::unique(selected.begin(), selected.end()), selected.end());

        std::vector<mesh::ChunkKey> to_build;
        ViewDiff diff;
        diff.serial = serial;
        diff.focus_x = x;
        diff.focus_y = y;
        diff.velocity_x = vx;
        diff.velocity_y = vy;
        diff.selected_at = start;
        std::set_difference(selected.begin(), selected.end(), resident.begin(), resident.end(),
                            std::back_inserter(to_build));
        std::set_difference(resident.begin(), resident.end(), selected.begin(), selected.end(),
                            std::back_inserter(diff.removed));
        // Road LOD change: resident chunks that stay, at a level whose road-class content changed, are rebuilt.
        const std::size_t added_count = to_build.size();
        if (resident_lod != src.road_lod) {
            int max_level = 0;
            for (const auto& k : resident) max_level = std::max(max_level, static_cast<int>(k.level));
            const std::vector<int> levels = changed_road_lod_levels(resident_lod.get(), src.road_lod.get(), max_level);
            if (!levels.empty()) {
                std::vector<mesh::ChunkKey> kept;
                std::set_intersection(resident.begin(), resident.end(), selected.begin(), selected.end(),
                                      std::back_inserter(kept));
                for (const auto& k : kept) {
                    if (std::find(levels.begin(), levels.end(), static_cast<int>(k.level)) != levels.end()) {
                        to_build.push_back(k);
                    }
                }
            }
        }

        if (!build_view_chunks(to_build, src, std::max(1u, options_.build_threads), diff.added, &cancel_)) {
            return; // cancelled: only the destructor raises cancel_
        }
        if (diff.added.size() > added_count) {
            diff.replaced.assign(std::make_move_iterator(diff.added.begin() + static_cast<std::ptrdiff_t>(added_count)),
                                 std::make_move_iterator(diff.added.end()));
            diff.added.resize(added_count);
        }
        if (det::hypot(vx, vy) >= 2.0) {
            sort_adds_by_urgency(diff.added, src.e0, src.n0, rx, ry, vx, vy);
        }
        const double ms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();

        std::lock_guard<std::mutex> lk(mutex_);
        if (stop_) {
            return;
        }
        stats_.chunks_built += added_count;
        stats_.chunks_reused += selected.size() - added_count;
        stats_.chunks_rebuilt += diff.replaced.size();
        stats_.last_added = diff.added.size();
        stats_.last_removed = diff.removed.size();
        stats_.last_build_ms = ms;
        ready_ = std::move(diff);
        target_ = std::move(selected);
        target_road_lod_ = src.road_lod;
        phase_ = Phase::Ready;
    }
}

} // namespace g2m::world
