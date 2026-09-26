// g2m/phys/physics_grid.h - PhysicsTileGrid (PLAN.md R2.2 section 1a): the
// 255 m, node-registered physics-tile grid physics_sim's ITerrainSource
// bridge (g2m_ps_bridge, G6+) fills from g2m's own 256 m, cell-registered
// DGM1 L0 data tiles (g2m::TileKey level 0, g2m::HeightTile).
//
// g2m_phys is ps-free: no physics_sim/Jolt/Godot type appears anywhere
// under include/g2m/phys, and this library never links physics_sim.
//
// Why a SEPARATE 255 m grid, not the 256 m data-tile grid itself (vault
// DGM1-lattice ruling, made during G0/G1 planning):
//   - physics_sim's IRigidBackend::HeightFieldShape needs shared border
//     samples between neighbouring tiles (NODE registration) and an EVEN
//     sample count (Jolt rounds HeightFieldShape::mSampleCount up to a
//     multiple of its block size otherwise). g2m's own L0 data tiles are
//     CELL-registered (DGM1's native lattice: samples sit at cell centres,
//     no shared borders) with an odd-friendly 256 samples per side - a
//     resolution choice, not a registration one.
//   - So a physics tile is a DIFFERENT lattice: kPhysSamples (256, even)
//     samples at exactly 1 m spacing, spanning kPhysSamples - 1 = 255 m -
//     one metre short of the data tile's 256 m, on purpose, so its samples
//     land on DGM1 CELL CENTRES (see the anchoring derivation below) while
//     still sharing a border sample between adjacent physics tiles (tile
//     ix's last sample column coincides exactly with tile ix+1's first).
//   - One physics tile therefore reads from at most 4 data tiles (2x2),
//     never more, and needs no resampling - only an integer-offset copy
//     (see fill_physics_heights.h).
//
// Anchoring derivation: physics tile (ix, iy)'s sample (k, l) sits at local
// (session-frame) coordinate (ts*ix + 0.5 + k, ts*iy + 0.5 + l), where
// ts = tile_size_m() = kPhysSamples - 1 (an integer). Since a SessionFrame's
// E0/N0 origin is an integer number of metres (session_frame.h), this
// sample's GLOBAL UTM grid easting is e0 + ts*ix + k + 0.5: the integer
// part e0 + ts*ix + k is always exact (every term is an integer), so the
// sample sits EXACTLY on the centre of DGM1 cell column e0 + ts*ix + k -
// never off by a fractional cell. See the "sample sits on a DGM1 cell
// centre" test in tests/phys/test_physics_grid.cpp for the checked proof
// (against SessionFrame::grid_easting/tile_at, not just this derivation).
#pragma once

#include "g2m/core/geo/session_frame.h"
#include "g2m/core/tile_key.h"

#include <array>
#include <compare>
#include <cstdint>
#include <vector>

namespace g2m::phys {

// tile edge = kPhysSamples - 1 = 255 m, 1 m spacing (see the header
// derivation above for why 255, not 256).
inline constexpr int kPhysSamples = 256;

struct PhysTileIndex {
    std::int32_t ix = 0;
    std::int32_t iy = 0;
    auto operator<=>(const PhysTileIndex&) const = default;
};

class PhysicsTileGrid {
public:
    // `frame` anchors the grid on a session's UTM zone/origin. `samples`
    // must be even and >= 2 (Jolt's own HeightFieldShape sample-count
    // constraint - see the header derivation above); throws
    // std::invalid_argument otherwise, matching SessionFrame's own
    // constructor-validation convention.
    explicit PhysicsTileGrid(geo::SessionFrame frame, int samples = kPhysSamples);

    int samples() const { return samples_; }
    // samples() - 1, an exact integer value returned as double.
    double tile_size_m() const { return static_cast<double>(samples_ - 1); }

    // The physics tile whose sample lattice covers session-local point
    // (x, y): floor((x - 0.5) / tile_size_m()), same for y. A point exactly
    // on a tile's western/southern edge (x == ts*ix + 0.5) lands in tile ix
    // itself (matches SessionFrame::tile_at's own "a point exactly on an
    // edge lands in the tile to its east/north" convention).
    PhysTileIndex index_for_local(double x, double y) const;

    // Session-local (x, y) of sample (0, 0) of physics tile `index`:
    // {tile_size_m()*ix + 0.5, tile_size_m()*iy + 0.5}.
    std::array<double, 2> origin_local(PhysTileIndex index) const;

    // Packs/unpacks exactly like physics_sim's own
    // ProceduralTerrainSource::pack_key/unpack_key (core/src/terrain/
    // procedural_terrain_source.cpp, read-only reference): two signed
    // 32-bit halves, (uint64(uint32(ix)) << 32) | uint32(iy). This is a
    // SEPARATE numbering from g2m::TileKey::packed() (the DGM1 data-tile
    // grid) - a physics tile key is opaque to physics_sim and must never be
    // decoded there (PLAN.md section 6's "opaque keys" rule), only compared
    // or round-tripped through this pair.
    static std::int64_t pack(PhysTileIndex index);
    static PhysTileIndex unpack(std::int64_t packed);

    // The set of L0 (level 0, 256 m) DGM1 data-tile keys any sample of the
    // (2r+1) x (2r+1) square of physics tiles centred on `centre` can touch
    // - sorted ascending by TileKey::packed(), unique. `out` is cleared
    // first. Not allocation-free (unlike fill_physics_heights): called only
    // between World steps (PLAN.md R2.2 section 1a/2), never from a job.
    void l0_keys_for_square(PhysTileIndex centre, int r, std::vector<TileKey>& out) const;

    const geo::SessionFrame& frame() const { return frame_; }

private:
    geo::SessionFrame frame_;
    int samples_;
};

} // namespace g2m::phys
