// g2m/phys/fill_physics_classes.h - class mode of the physics surface fill
// (terrain migration phase 6, requests G-2/G-3/G-5): one physics tile's cell
// surface ids from the g2m.terrain.class tiles riding on a ResidentHeightSet's
// L0 entries, resolved through a full classes::SurfaceIdTable (SurfaceKind 0-11
// and LandClass 0-13, not the paved/unpaved/off_road choice of road mode).
// ps-free, header-only, sibling of fill_physics_surfaces.h.
//
// Placement: physics cell (x, y) of tile (ix, iy) is classified at its CENTRE,
// the integer metre E = e0 + ts*ix + x + 1, N = n0 + ts*iy + y + 1 (ts =
// samples() - 1; the same point as fill_physics_surfaces.h). A level-0 class
// tile is node-registered at exactly those integer metres (terrain_class.h,
// D11), so a cell reads node (E - 256 tx, N - 256 ty) of the class tile tx =
// floor(E / 256), ty = floor(N / 256): no resampling. Edge nodes are equal in
// both neighbouring tiles by construction, so the choice of the floor tile is
// not a decision.
//
// Outcomes (loud, as fill_physics_surfaces.h):
//   - L0 key Absent (confirmed 404)                -> `fallback`, not a miss;
//   - L0 key Missing (not resident)               -> `fallback`, a miss;
//   - L0 key Resident without a class tile        -> `fallback`, a miss (a
//     contract violation in class mode, never silently the Unknown class);
//   - L0 key Resident with a class tile           -> table.resolve(land_class, surface).
//
// No allocation and no lock: up to four ResidentHeightSet::find calls (the at
// most 2 x 2 L0 tiles a physics tile touches), then one table lookup per cell.
// A pure function of (grid, index, the resident entries, table, fallback).
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/classifier.h"
#include "g2m/layer/terrain_class.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"
#include "g2m/world/surface_id_table.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <span>

namespace g2m::phys {

struct ClassFillStats {
    int resident_tiles = 0; // touched L0 keys read from a class tile
    int absent_tiles = 0;   // touched L0 keys confirmed absent (fallback, no miss)
    int missing_tiles = 0;  // touched L0 keys missing, or resident without a class tile (fallback, a miss)
};

// Fills `out` (row-major, row stride samples()-1, (samples()-1)^2 elements: a
// physics tile's cell_surface_ids) as described in the file comment.
inline ClassFillStats fill_physics_classes(const PhysicsTileGrid& grid, PhysTileIndex index,
                                            const ResidentHeightSet& resident,
                                            const classes::SurfaceIdTable& table, std::uint16_t fallback,
                                            std::span<std::uint16_t> out) noexcept {
    const std::int64_t ts = static_cast<std::int64_t>(grid.samples() - 1);
    const geo::SessionFrame& frame = grid.frame();
    const std::int64_t col0 = frame.e0_m() + ts * static_cast<std::int64_t>(index.ix);
    const std::int64_t row0 = frame.n0_m() + ts * static_cast<std::int64_t>(index.iy);
    const std::int64_t size = TileKey::kLevel0SizeM;

    // Cell centres span [col0 + 1, col0 + ts]: at most two L0 columns and rows.
    const std::int64_t tx0 = floor_div(col0 + 1, size);
    const std::int64_t ty0 = floor_div(row0 + 1, size);
    const std::int64_t txn = floor_div(col0 + ts, size) - tx0 + 1;
    const std::int64_t tyn = floor_div(row0 + ts, size) - ty0 + 1;

    ClassFillStats stats;
    const TerrainClassTile* tiles[2][2] = {{nullptr, nullptr}, {nullptr, nullptr}};
    for (std::int64_t b = 0; b < tyn; ++b) {
        for (std::int64_t a = 0; a < txn; ++a) {
            TileKey l0;
            l0.zone = frame.zone();
            l0.level = 0;
            l0.x = static_cast<std::int32_t>(tx0 + a);
            l0.y = static_cast<std::int32_t>(ty0 + b);
            const HeightTile* height = nullptr;
            const TerrainClassTile* cls = nullptr;
            const auto state = resident.find(l0, &height, nullptr, &cls);
            if (state == ResidentHeightSet::Lookup::Absent) {
                ++stats.absent_tiles;
            } else if (state == ResidentHeightSet::Lookup::Resident && cls != nullptr) {
                tiles[b][a] = cls;
                ++stats.resident_tiles;
            } else {
                ++stats.missing_tiles;
            }
        }
    }

    const std::size_t cells = static_cast<std::size_t>(ts);
    for (std::int64_t y = 0; y < ts; ++y) {
        const std::int64_t n = row0 + y + 1;
        const std::int64_t b = floor_div(n, size) - ty0;
        const int j = static_cast<int>(n - (ty0 + b) * size);
        std::uint16_t* row = out.data() + static_cast<std::size_t>(y) * cells;
        for (std::int64_t x = 0; x < ts; ++x) {
            const std::int64_t e = col0 + x + 1;
            const std::int64_t a = floor_div(e, size) - tx0;
            const TerrainClassTile* cls = tiles[b][a];
            if (cls == nullptr) {
                row[x] = fallback;
                continue;
            }
            const std::size_t at = TerrainClassTile::index(static_cast<int>(e - (tx0 + a) * size), j);
            row[x] = table.resolve(static_cast<LandClass>(cls->land_class[at]), static_cast<SurfaceKind>(cls->surface[at]));
        }
    }
    return stats;
}

} // namespace g2m::phys
