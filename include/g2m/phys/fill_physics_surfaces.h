// g2m/phys/fill_physics_surfaces.h - fills one physics tile's cell surface
// ids from a ResidentHeightSet's road data (PLAN.md G2.5a-grip section 4).
// ps-free, header-only, sibling of fill_physics_heights.h - NOT templated
// (the output is always a fixed uint16_t SurfaceId-shaped id, unlike
// fill_physics_heights' Real sample).
//
// Cell-centre placement (PLAN.md G2.5a-grip section 2): physics cell (x, y)
// of tile (ix, iy), x/y in [0, samples()-1), is classified at its CENTRE, an
// INTEGER metre - the midpoint between two consecutive DGM1 cell-centre
// height samples (each at a half-integer metre, physics_grid.h's own
// anchoring derivation), which lands exactly on a shared corner of 4 DGM1
// cells - the same point the render L0 "one-cell proof" (osm_roads.h) checks:
//   E_c = e0 + ts*ix + x + 1, N_c = n0 + ts*iy + y + 1, ts = samples() - 1.
// This file builds a Lattice at that origin, 1 m spacing, (ts, ts) samples,
// and reuses paint_road_blocks (osm_roads.h) at block_level 0 (physics, 256
// m) for the actual paint - the SAME exact capsule loop and clamps the
// render path uses, so a road surface at a physics cell and at its render
// quad agree by construction.
//
// Outcomes (PLAN.md G2.5a-grip section 3/4 - loud, never silently grass):
//   - a touched L0 key is Absent (confirmed 404, no coverage)      -> off_road, NOT a miss.
//   - a touched L0 key is Missing (not resident)                   -> off_road, IS a miss.
//   - a touched L0 key is Resident but its roads is null            -> off_road, IS a miss
//     (a contract violation in road mode - never silently treated as "no
//     roads here"; see ResidentHeightSet::install's 3-arg overload).
//   - a touched L0 key is Resident with a non-null, empty roads list -> off_road, NOT a miss
//     (a genuine "no drivable ways" tile).
//   - a touched L0 key is Resident with a non-empty roads list       -> painted via paint_road_blocks.
//
// NO SCRATCH: besides `out` itself, the only extra state is a handful of
// stack ints/TileKeys (the up-to-2x2 L0 tile candidates this physics tile can
// touch, physics_grid.h's own "at most 4 data tiles" bound) - never a heap
// buffer, thread_local array or slot pool. Allocation-free (a ResidentHeightSet::
// find() call per touched L0 key - up to 4 - is a lock-free binary search, no
// allocation; see tests/phys/alloc_counter.h).
//
// Determinism: a pure function of (grid, index, resident's own entries, lut,
// off_road) - same worker-invariance contract as fill_physics_heights.h (no
// shared scratch, fixed paint order via paint_road_blocks' own block-then-
// segment-list-order rule).
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/phys/resident_heights.h"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace g2m::phys {

namespace detail {

// The up-to-2x2 L0 tile x/y candidates a physics tile can touch (physics_grid.h's
// own "at most 4 data tiles" bound) - precomputed once by fill_physics_surfaces,
// then scanned by surface_lookup below (called at most once per block by
// paint_road_blocks). Plain stack data, not a buffer: no heap, no
// thread_local, nothing besides these ints/pointers.
struct SurfaceLookupCtx {
    const ResidentHeightSet* resident = nullptr;
    geo::UtmZone zone{};
    std::int32_t tx_lo = 0, tx_hi = 0, ty_lo = 0, ty_hi = 0;
};

// RoadBlockLookupFn (osm_roads.h): called once per touched block with that
// block's OWNER L2 key. Reconstructs which of the (at most 4) candidate L0
// keys owns `l2_key` (l0.parent().parent() == l2_key) and resolves it via
// ResidentHeightSet::find - see this header's own Outcomes comment above for
// the Lookup -> RoadBlockStatus mapping.
inline RoadBlockLookupResult surface_lookup(void* ctx_v, const TileKey& l2_key) {
    const auto& ctx = *static_cast<const SurfaceLookupCtx*>(ctx_v);
    const std::int32_t txs[2] = {ctx.tx_lo, ctx.tx_hi};
    const std::int32_t tys[2] = {ctx.ty_lo, ctx.ty_hi};
    for (std::int32_t ty : tys) {
        for (std::int32_t tx : txs) {
            TileKey l0;
            l0.zone = ctx.zone;
            l0.level = 0;
            l0.x = tx;
            l0.y = ty;
            if (!(l0.parent().parent() == l2_key)) continue;

            const HeightTile* tile = nullptr;
            std::shared_ptr<const std::vector<RoadSegment>> roads;
            const ResidentHeightSet::Lookup lookup = ctx.resident->find(l0, &tile, &roads);
            switch (lookup) {
            case ResidentHeightSet::Lookup::Missing:
                return RoadBlockLookupResult{RoadBlockStatus::Missing, {}};
            case ResidentHeightSet::Lookup::Absent:
                // Confirmed no height coverage - no road data can exist
                // either; off_road (already pre-filled by the caller), never
                // a miss (mirrors fill_physics_heights.h's own
                // Absent-vs-HadNoData design note).
                return RoadBlockLookupResult{RoadBlockStatus::None, {}};
            case ResidentHeightSet::Lookup::Resident:
                if (!roads) {
                    // Loud: resident heights with no roads list at all is a
                    // contract violation in road mode - counted the same way
                    // as Missing, never silently painted as off_road-via-None.
                    return RoadBlockLookupResult{RoadBlockStatus::Missing, {}};
                }
                if (roads->empty()) return RoadBlockLookupResult{RoadBlockStatus::None, {}};
                return RoadBlockLookupResult{RoadBlockStatus::Segments, std::span<const RoadSegment>(*roads)};
            }
        }
    }
    // Every candidate this physics tile can touch was checked; none had
    // l2_key as its own owner. paint_road_blocks only ever queries the owner
    // of a block actually inside `lattice`, and every such block's L0 key is
    // one of the (tx_lo/tx_hi, ty_lo/ty_hi) candidates by construction (the
    // same "at most 2x2 L0 tiles" bound fill_physics_heights.h relies on) -
    // unreachable in practice; fail safe rather than assert.
    return RoadBlockLookupResult{RoadBlockStatus::Missing, {}};
}

} // namespace detail

// Fills `out` (row-major, row_stride == samples()-1, exactly (samples()-1)^2
// elements - a physics tile's cell_surface_ids, PLAN.md G2.5a-grip section 2)
// with `lut.at(seg.land_class, seg.surface)` for every cell whose CENTRE a
// road segment covers, `off_road` everywhere else. Returns paint_road_blocks'
// own per-status block counts (segments_blocks/none_blocks/missing_blocks) -
// missing_blocks > 0 iff at least one touched L0 key was not resident, or
// resident without its own roads list (this header's own Outcomes comment
// above) - a caller's own fill-miss counter, mirroring
// fill_physics_heights.h's FillResult::MissingTile.
inline RoadBlockStats fill_physics_surfaces(const PhysicsTileGrid& grid, PhysTileIndex index,
                                             const ResidentHeightSet& resident, const RoadValueLut& lut,
                                             std::uint16_t off_road, std::span<std::uint16_t> out) noexcept {
    const int n = grid.samples();
    const std::int64_t ts = static_cast<std::int64_t>(n - 1); // cells per side, e.g. 255
    const geo::SessionFrame& frame = grid.frame();
    const std::int64_t e0 = frame.e0_m();
    const std::int64_t n0 = frame.n0_m();
    const geo::UtmZone zone = frame.zone();

    const std::int64_t col0 = e0 + ts * static_cast<std::int64_t>(index.ix); // same col0 as fill_physics_heights.h
    const std::int64_t row0 = n0 + ts * static_cast<std::int64_t>(index.iy);

    // Cell x's own centre column is col0 + x + 1, x in [0, ts) -> the range
    // [col0 + 1, col0 + ts] (ts consecutive integers), which touches at most
    // 2 L0 tile columns (ts < 256) - the same "at most 2x2 L0 tiles" bound as
    // fill_physics_heights.h.
    detail::SurfaceLookupCtx ctx;
    ctx.resident = &resident;
    ctx.zone = zone;
    ctx.tx_lo = static_cast<std::int32_t>(floor_div(col0 + 1, TileKey::kLevel0SizeM));
    ctx.tx_hi = static_cast<std::int32_t>(floor_div(col0 + ts, TileKey::kLevel0SizeM));
    ctx.ty_lo = static_cast<std::int32_t>(floor_div(row0 + 1, TileKey::kLevel0SizeM));
    ctx.ty_hi = static_cast<std::int32_t>(floor_div(row0 + ts, TileKey::kLevel0SizeM));

    const std::size_t cells = static_cast<std::size_t>(ts);
    const std::size_t total = cells * cells;
    std::fill_n(out.data(), total, off_road);

    const Lattice lattice{(col0 + 1) * 1000, (row0 + 1) * 1000, 1000, static_cast<int>(ts), static_cast<int>(ts)};

    return paint_road_blocks(lattice, /*block_level=*/0, zone, RasterParams{}, lut, &detail::surface_lookup, &ctx, out,
                              cells);
}

} // namespace g2m::phys
