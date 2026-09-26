// g2m/phys/resident_heights.h - ResidentHeightSet, the physics bridge's
// in-memory cache of loaded DGM1 L0 HeightTiles (PLAN.md R2.2 section 1a).
// ps-free (no physics_sim/Jolt/Godot type here or anywhere in g2m/phys) -
// this header is shared by g2m_ps_bridge (G6+) and any other physics_sim
// peer, but never links physics_sim itself.
//
// Threading contract: ONLY the owner thread ever mutates a ResidentHeightSet
// (install()/retain_only()), and only BETWEEN World steps (the same "owner
// thread, between steps" seam PhysicsTerrainStreamer::update() runs in,
// PLAN.md section 1a). During a step the set is read-only: find() takes no
// lock and never will, because no writer can run concurrently with it by
// construction - the owner serialises "poll loader -> install -> run one
// step" itself. find() returns a raw HeightTile* (the pointee's lifetime is
// this set's own shared_ptr, kept alive until a later retain_only() drops
// it), so a read-heavy per-substep caller (fill_physics_heights, called once
// per physics tile per fill) does zero refcount traffic - no shared_ptr copy
// on the hot path.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace g2m::phys {

class ResidentHeightSet {
public:
    enum class Lookup {
        Resident, // key is installed with a real tile; *out is set to it.
        Absent,   // key is installed as a confirmed 404 (no coverage); *out is set to nullptr.
        Missing,  // key has never been installed (still loading, or not yet requested); *out is set to nullptr.
    };

    // Binary search by TileKey::packed(). *out is written in every case (see
    // Lookup above) so a caller can read it unconditionally.
    Lookup find(const TileKey& key, const HeightTile** out) const noexcept;

    // Installs (or replaces) `key`'s entry. `tile == nullptr` records a
    // confirmed Absent (the loader's 404 outcome) - a deliberately different
    // state from "not installed at all" (Missing): once a key is installed,
    // find() never reports Missing for it again until a retain_only() call
    // evicts it.
    void install(const TileKey& key, std::shared_ptr<const HeightTile> tile);

    // Keeps only the entries whose key appears in `keep_sorted` (which must
    // already be sorted ascending by TileKey::packed(), duplicates allowed);
    // every other entry is dropped. O(size() + keep_sorted.size()).
    void retain_only(std::span<const TileKey> keep_sorted);

    std::size_t size() const noexcept { return entries_.size(); }
    // True whether the key is Resident or Absent (i.e. find() != Missing).
    bool contains(const TileKey& key) const noexcept;

private:
    struct Entry {
        TileKey key;
        std::shared_ptr<const HeightTile> tile; // nullptr => Absent
    };
    // Sorted ascending by key.packed(), unique by key.
    std::vector<Entry> entries_;
};

} // namespace g2m::phys
