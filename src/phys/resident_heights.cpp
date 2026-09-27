// src/phys/resident_heights.cpp - see g2m/phys/resident_heights.h.
#include "g2m/phys/resident_heights.h"

#include <algorithm>

namespace g2m::phys {

namespace {
std::uint64_t packed_of(const TileKey& key) { return key.packed(); }
} // namespace

ResidentHeightSet::Lookup ResidentHeightSet::find(const TileKey& key, const HeightTile** out) const noexcept {
    return find(key, out, nullptr);
}

ResidentHeightSet::Lookup ResidentHeightSet::find(const TileKey& key, const HeightTile** out,
                                                   std::shared_ptr<const std::vector<RoadSegment>>* roads_out) const
    noexcept {
    const auto packed = packed_of(key);
    const auto it = std::lower_bound(entries_.begin(), entries_.end(), packed,
                                     [](const Entry& e, std::uint64_t p) { return e.key.packed() < p; });
    if (it == entries_.end() || it->key.packed() != packed) {
        *out = nullptr;
        if (roads_out) *roads_out = nullptr;
        return Lookup::Missing;
    }
    if (roads_out) *roads_out = it->roads;
    if (!it->tile) {
        *out = nullptr;
        return Lookup::Absent;
    }
    *out = it->tile.get();
    return Lookup::Resident;
}

void ResidentHeightSet::install(const TileKey& key, std::shared_ptr<const HeightTile> tile) {
    install(key, std::move(tile), nullptr);
}

void ResidentHeightSet::install(const TileKey& key, std::shared_ptr<const HeightTile> tile,
                                 std::shared_ptr<const std::vector<RoadSegment>> roads) {
    const auto packed = packed_of(key);
    const auto it = std::lower_bound(entries_.begin(), entries_.end(), packed,
                                     [](const Entry& e, std::uint64_t p) { return e.key.packed() < p; });
    if (it != entries_.end() && it->key.packed() == packed) {
        it->tile = std::move(tile);
        it->roads = std::move(roads);
        return;
    }
    entries_.insert(it, Entry{key, std::move(tile), std::move(roads)});
}

void ResidentHeightSet::retain_only(std::span<const TileKey> keep_sorted) {
    std::vector<Entry> kept;
    kept.reserve(entries_.size());
    std::size_t ki = 0;
    for (auto& entry : entries_) {
        const auto p = entry.key.packed();
        while (ki < keep_sorted.size() && keep_sorted[ki].packed() < p) ++ki;
        if (ki < keep_sorted.size() && keep_sorted[ki].packed() == p) {
            kept.push_back(std::move(entry));
        }
    }
    entries_ = std::move(kept);
}

bool ResidentHeightSet::contains(const TileKey& key) const noexcept {
    const HeightTile* tile = nullptr;
    return find(key, &tile) != Lookup::Missing;
}

} // namespace g2m::phys
