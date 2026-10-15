// src/phys/physics_streamer.cpp - see g2m/phys/physics_streamer.h.
#include "g2m/phys/physics_streamer.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace g2m::phys {

namespace {
bool packed_less(const TileKey& a, const TileKey& b) { return a.packed() < b.packed(); }

bool sorted_contains(const std::vector<TileKey>& sorted, const TileKey& key) {
    return std::binary_search(sorted.begin(), sorted.end(), key, packed_less);
}

// TileManager's own square radius, in physics tiles. MUST stay the same
// expression as physics_sim's TileManager::update() (core/src/terrain/
// tile_manager.cpp, step 1, line 217 at physics_sim 5c648a5):
//     const auto radius_tiles = static_cast<std::int32_t>(std::ceil(ip.radius_m / ts));
// with ts = ITerrainSource::tile_size_m() (= PhysicsTileGrid::tile_size_m()
// in the bridge) around key_for_world_xy (= index_for_local), dx/dy in
// [-radius_tiles, radius_tiles].
std::int32_t tile_manager_radius_tiles(double radius_m, double ts) {
    return static_cast<std::int32_t>(std::ceil(radius_m / ts));
}
} // namespace

PhysicsTerrainStreamer::PhysicsTerrainStreamer(const PhysicsTileGrid& grid, ResidentHeightSet& set,
                                               HeightTileLoader& loader, StreamerConfig config)
    : grid_(grid), set_(set), loader_(loader), config_(config) {
    if (config_.gate_margin_tiles < 0 || config_.prefetch_margin_tiles < 0) {
        throw std::invalid_argument("PhysicsTerrainStreamer: margins must be >= 0");
    }
    if (!(config_.lookahead_s >= 0.0) || !(config_.lookahead_cap_m >= 0.0)) {
        throw std::invalid_argument("PhysicsTerrainStreamer: look-ahead time and cap must be >= 0");
    }
}

void PhysicsTerrainStreamer::sort_unique(std::vector<TileKey>& keys) {
    std::sort(keys.begin(), keys.end(), packed_less);
    keys.erase(std::unique(keys.begin(), keys.end(),
                           [](const TileKey& a, const TileKey& b) { return a.packed() == b.packed(); }),
               keys.end());
}

void PhysicsTerrainStreamer::add_square(PhysTileIndex centre, int r, std::vector<TileKey>& into) {
    grid_.l0_keys_for_square(centre, r, square_);
    into.insert(into.end(), square_.begin(), square_.end());
}

void PhysicsTerrainStreamer::retry_failed() { failed_.clear(); }

GateStatus PhysicsTerrainStreamer::update(std::span<const InterestPoint> points) {
    // 1. Completions, in key order.
    completions_.clear();
    loader_.poll(completions_);
    for (LoadedHeightTile& done : completions_) {
        switch (done.outcome) {
        case LoadOutcome::Ready: set_.install(done.key, std::move(done.tile), std::move(done.roads), std::move(done.classes)); break;
        case LoadOutcome::Absent: set_.install(done.key, nullptr, std::move(done.roads)); break;
        case LoadOutcome::Failed:
            if (!sorted_contains(failed_, done.key)) {
                failed_.insert(std::upper_bound(failed_.begin(), failed_.end(), done.key, packed_less), done.key);
            }
            break;
        }
    }
    completions_.clear();

    // 2./3. Gate and keep sets.
    const double ts = grid_.tile_size_m();
    gate_.clear();
    keep_.clear();
    for (const InterestPoint& p : points) {
        const PhysTileIndex centre = grid_.index_for_local(p.x, p.y);
        const std::int32_t r_tm = tile_manager_radius_tiles(p.radius_m, ts);
        const int r_gate = r_tm + config_.gate_margin_tiles;
        add_square(centre, r_gate, gate_);
        add_square(centre, r_tm + config_.prefetch_margin_tiles, keep_);

        double dx = p.vx * config_.lookahead_s;
        double dy = p.vy * config_.lookahead_s;
        const double len = std::sqrt(dx * dx + dy * dy);
        if (len > config_.lookahead_cap_m) {
            const double s = config_.lookahead_cap_m / len;
            dx *= s;
            dy *= s;
        }
        const PhysTileIndex ahead = grid_.index_for_local(p.x + dx, p.y + dy);
        if (!(ahead == centre)) add_square(ahead, r_gate, keep_);
    }
    sort_unique(gate_);
    keep_.insert(keep_.end(), gate_.begin(), gate_.end());
    sort_unique(keep_);

    // 4. Evict (never a gate key: keep_ is a superset of gate_).
    if (set_.size() > config_.max_resident_l0) set_.retain_only(keep_);
    failed_.erase(std::remove_if(failed_.begin(), failed_.end(),
                                 [this](const TileKey& k) { return !sorted_contains(keep_, k); }),
                  failed_.end());

    // 5. Request what is missing: gate first, then the rest, each by
    // distance to the nearest interest point, ties by key.
    ranked_gate_.clear();
    ranked_rest_.clear();
    const auto e0 = static_cast<double>(grid_.frame().e0_m());
    const auto n0 = static_cast<double>(grid_.frame().n0_m());
    const double half = static_cast<double>(TileKey::kLevel0SizeM) / 2.0;
    for (const TileKey& key : keep_) {
        if (set_.contains(key) || sorted_contains(failed_, key)) continue;
        // L0 tile centre in the session frame (integers: exact in double).
        const double cx = static_cast<double>(key.min_easting()) + half - e0;
        const double cy = static_cast<double>(key.min_northing()) + half - n0;
        double d2 = std::numeric_limits<double>::infinity();
        for (const InterestPoint& p : points) {
            const double ex = cx - p.x;
            const double ey = cy - p.y;
            d2 = std::min(d2, ex * ex + ey * ey);
        }
        const Ranked r{d2, key.packed(), key};
        (sorted_contains(gate_, key) ? ranked_gate_ : ranked_rest_).push_back(r);
    }
    const auto by_rank = [](const Ranked& a, const Ranked& b) {
        if (a.d2 != b.d2) return a.d2 < b.d2;
        return a.packed < b.packed;
    };
    std::sort(ranked_gate_.begin(), ranked_gate_.end(), by_rank);
    std::sort(ranked_rest_.begin(), ranked_rest_.end(), by_rank);
    request_.clear();
    for (const Ranked& r : ranked_gate_) request_.push_back(r.key);
    for (const Ranked& r : ranked_rest_) request_.push_back(r.key);
    loader_.request(request_);

    // 6. Gate status.
    GateStatus status;
    for (const TileKey& key : gate_) {
        if (!set_.contains(key)) ++status.missing_required;
    }
    status.ready = status.missing_required == 0;
    const LoaderStats ls = loader_.stats();
    status.inflight = ls.queued + ls.inflight;
    status.resident = static_cast<std::uint32_t>(set_.size());
    status.failed = static_cast<std::uint32_t>(failed_.size());
    // All allocations/reclamation occur on the owner side, outside World::step.
    set_.publish_snapshot();
    set_.purge_retired_snapshots();
    return status;
}

} // namespace g2m::phys
