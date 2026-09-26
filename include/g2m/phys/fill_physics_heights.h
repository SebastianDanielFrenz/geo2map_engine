// g2m/phys/fill_physics_heights.h - fills one physics tile's height samples
// from a ResidentHeightSet (PLAN.md R2.2 section 1a). ps-free, header-only
// (templated on Real so both physics_sim's `double` and a `float` test
// double can use it without a second instantiation living in this library).
//
// NO ALLOCATION for any `samples` (including 1024, the seamless-reference
// tile size some validation tests use) and NO ROW FLIP: `out` is row-major
// [j * n + k] with j increasing NORTH, exactly HeightTile::h's own
// row-0-is-south convention (see height_tile.h) - sample (k, j) of the
// physics tile is read directly from HeightTile row j's own global row, no
// north/south inversion anywhere in this file.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>

namespace g2m::phys {

enum class FillResult {
    Ok,          // every sample came from a resident, fully-covered tile.
    HadNoData,   // every touched L0 key was resident or a confirmed Absent
                 // (never Missing), but at least one RESIDENT tile's own
                 // sample was NoData (a genuine data-quality hole inside
                 // coverage this session expected to be solid - see below
                 // for why a plain Absent tile does NOT set this).
    MissingTile, // at least one touched L0 key has never been installed
                 // (still loading). Takes priority over HadNoData: if a
                 // fill call touches both a genuine NoData sample and a
                 // Missing tile, the more actionable signal (retry) wins.
};

// Design note on Absent vs. HadNoData: a confirmed-404 (Absent) tile writes
// `hole` just like a NoData sample does, but by itself leaves the result at
// Ok, not HadNoData - an Absent tile at the edge of DGM1 coverage (water,
// outside the country) is the EXPECTED, common case there, and folding it
// into HadNoData would drown out the signal HadNoData exists for: a
// genuinely resident, in-coverage tile that still has a NoData sample. A
// caller wanting "does this physics tile have any hole at all, for any
// reason" combines HadNoData with its own resident.find() sweep instead.
template <class Real>
FillResult fill_physics_heights(const PhysicsTileGrid& grid, PhysTileIndex index, const ResidentHeightSet& resident,
                                Real* out, Real hole) noexcept {
    const int n = grid.samples();
    const std::int64_t ts = static_cast<std::int64_t>(n - 1);
    const geo::SessionFrame& frame = grid.frame();
    const std::int64_t e0 = frame.e0_m();
    const std::int64_t n0 = frame.n0_m();
    const geo::UtmZone zone = frame.zone();
    const std::int64_t col0 = e0 + ts * static_cast<std::int64_t>(index.ix); // global DGM1 column of sample k=0
    const std::int64_t row0 = n0 + ts * static_cast<std::int64_t>(index.iy); // global DGM1 row of sample j=0

    FillResult result = FillResult::Ok;

    for (int j = 0; j < n; ++j) {
        const std::int64_t row = row0 + j;
        const std::int64_t ty = floor_div(row, TileKey::kLevel0SizeM);
        const int local_j = static_cast<int>(row - TileKey::kLevel0SizeM * ty);

        int k = 0;
        while (k < n) {
            const std::int64_t col = col0 + k;
            const std::int64_t tx = floor_div(col, TileKey::kLevel0SizeM);
            const int local_i0 = static_cast<int>(col - TileKey::kLevel0SizeM * tx);
            // This run of columns shares one L0 key: it ends at the tile's
            // own east edge (256 - local_i0 more columns) or at this row's
            // last sample (n - k more columns), whichever comes first.
            const int run = std::min(n - k, static_cast<int>(TileKey::kLevel0SizeM) - local_i0);

            TileKey key;
            key.zone = zone;
            key.level = 0;
            key.x = static_cast<std::int32_t>(tx);
            key.y = static_cast<std::int32_t>(ty);

            const HeightTile* tile = nullptr;
            const auto lookup = resident.find(key, &tile);
            Real* row_out = out + static_cast<std::size_t>(j) * static_cast<std::size_t>(n) + static_cast<std::size_t>(k);

            if (lookup == ResidentHeightSet::Lookup::Missing) {
                result = FillResult::MissingTile;
                for (int di = 0; di < run; ++di) row_out[di] = hole;
            } else if (lookup == ResidentHeightSet::Lookup::Absent) {
                for (int di = 0; di < run; ++di) row_out[di] = hole;
            } else { // Resident
                const std::size_t tile_row_base = static_cast<std::size_t>(local_j) * kTerrainTileSamples;
                for (int di = 0; di < run; ++di) {
                    const std::int32_t hv = tile->h[tile_row_base + static_cast<std::size_t>(local_i0 + di)];
                    if (hv == kHeightNoData) {
                        row_out[di] = hole;
                        if (result == FillResult::Ok) result = FillResult::HadNoData;
                    } else {
                        row_out[di] = static_cast<Real>(hv) / static_cast<Real>(kHeightUnitsPerMetre);
                    }
                }
            }
            k += run;
        }
    }
    return result;
}

} // namespace g2m::phys
