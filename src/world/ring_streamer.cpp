// g2m/world/ring_streamer.cpp - see include/g2m/world/ring_streamer.h. The policy of racing_game's road_stream.gd
// and building_stream.gd (5a3baed); the scripts' float32 Vector2 arithmetic is double here (distances differ by
// < 1 m at UTM magnitudes, see test_ring_streamer.cpp). Ties in the job order, which Godot's unstable sort leaves
// arbitrary, are broken by tile key.
#include "g2m/world/ring_streamer.h"

#include "g2m/core/det_math.h"

#include <algorithm>
#include <cmath>

namespace g2m::world {

RingStreamer::RingStreamer(RingStreamerConfig config) : config_(config) {}

bool RingStreamer::update(double easting, double northing, double vx, double vy) {
    const auto cx = static_cast<std::int32_t>(std::floor(easting / config_.tile_m));
    const auto cy = static_cast<std::int32_t>(std::floor(northing / config_.tile_m));
    if (has_center_ && cx == center_x_ && cy == center_y_) return false;
    schedule(cx, cy, easting, northing, vx, vy);
    has_center_ = true;
    center_x_ = cx;
    center_y_ = cy;
    return true;
}

void RingStreamer::schedule(std::int32_t cx, std::int32_t cy, double e, double n, double vx, double vy) {
    const auto radius = static_cast<std::int32_t>(std::ceil(config_.far_m / config_.tile_m)) + 1;
    const double speed = det::hypot(vx, vy);
    const bool moving = speed >= config_.min_speed_mps;
    const double ux = moving ? vx / speed : 0.0, uy = moving ? vy / speed : 0.0;
    const auto now = std::chrono::steady_clock::now();
    std::vector<RingJob> jobs;
    for (std::int32_t y = cy - radius; y <= cy + radius; ++y)
        for (std::int32_t x = cx - radius; x <= cx + radius; ++x) {
            const double dx = static_cast<double>(x) * config_.tile_m + config_.tile_m * 0.5 - e;
            const double dy = static_cast<double>(y) * config_.tile_m + config_.tile_m * 0.5 - n;
            const double distance = det::hypot(dx, dy);
            if (distance > config_.far_m + config_.show_margin_m) continue;
            const bool full = distance <= config_.full_m + config_.show_margin_m;
            const auto k = known_.find(Key{y, x});
            if (k != known_.end() && (!full || k->second)) continue;
            double order = distance;
            if (moving && dx * ux + dy * uy < -config_.behind_m) order += config_.behind_penalty_m;
            jobs.push_back({x, y, full, order});
            wanted_since_.emplace(Key{y, x}, now);
        }
    // Serving order nearest first (ties by key); stored reversed so next_job() pops the back.
    std::sort(jobs.begin(), jobs.end(), [](const RingJob& a, const RingJob& b) {
        if (a.order != b.order) return a.order > b.order;
        if (a.y != b.y) return a.y > b.y;
        return a.x > b.x;
    });
    std::lock_guard<std::mutex> lk(mutex_);
    jobs_ = std::move(jobs);
}

std::optional<RingJob> RingStreamer::next_job() {
    std::lock_guard<std::mutex> lk(mutex_);
    if (jobs_.empty()) return std::nullopt;
    const RingJob job = jobs_.back();
    jobs_.pop_back();
    return job;
}

std::size_t RingStreamer::pending_jobs() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return jobs_.size();
}

std::vector<RingJob> RingStreamer::jobs() const {
    std::lock_guard<std::mutex> lk(mutex_);
    return {jobs_.rbegin(), jobs_.rend()};
}

RingAccept RingStreamer::accept(std::int32_t x, std::int32_t y, bool full, bool has_content) {
    const Key key{y, x};
    const auto r = resident_.find(key);
    if (r != resident_.end() && r->second.full && !full) return RingAccept::Rejected;
    known_[key] = full;
    wanted_since_.erase(key);
    if (!has_content) {
        if (!config_.empty_keeps_resident && r != resident_.end()) resident_.erase(r);
        return RingAccept::Empty;
    }
    Resident res;
    res.full = full;
    resident_[key] = res;
    return RingAccept::Added;
}

void RingStreamer::frame(double easting, double northing, std::vector<RingChange>& out) {
    for (auto it = resident_.begin(); it != resident_.end();) {
        const auto [y, x] = it->first;
        Resident& r = it->second;
        const double dx = static_cast<double>(x) * config_.tile_m + config_.tile_m * 0.5 - easting;
        const double dy = static_cast<double>(y) * config_.tile_m + config_.tile_m * 0.5 - northing;
        const double distance = det::hypot(dx, dy);
        RingChange c;
        c.x = x;
        c.y = y;
        c.near_visible = distance <= config_.full_m + config_.show_margin_m;
        c.far_visible = distance <= config_.far_m + config_.show_margin_m;
        bool changed = !r.reported || c.near_visible != r.near_visible || c.far_visible != r.far_visible;
        r.reported = true;
        r.near_visible = c.near_visible;
        r.far_visible = c.far_visible;
        if (distance > config_.far_m + config_.evict_margin_m) {
            c.evicted = true;
            out.push_back(c);
            known_.erase(it->first);
            it = resident_.erase(it);
            continue;
        }
        if (r.full && distance > config_.full_m + config_.evict_margin_m) {
            c.downgraded = true;
            changed = true;
            r.full = false;
            known_[it->first] = false;
        }
        if (changed) out.push_back(c);
        ++it;
    }
}

std::vector<RingStreamer::Known> RingStreamer::known() const {
    std::vector<Known> out;
    out.reserve(known_.size());
    for (const auto& [k, full] : known_) out.push_back({k.second, k.first, full});
    return out;
}

double RingStreamer::oldest_wanted_age_s() const {
    if (wanted_since_.empty()) return 0.0;
    auto oldest = std::chrono::steady_clock::time_point::max();
    for (const auto& [k, t] : wanted_since_) oldest = std::min(oldest, t);
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - oldest).count();
}

} // namespace g2m::world
