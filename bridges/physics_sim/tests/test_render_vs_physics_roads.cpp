// bridges/physics_sim/tests/test_render_vs_physics_roads.cpp - render-vs-
// physics road CELL INVARIANT (PLAN.md G2.5a-grip section 6, commit G-c):
// for every physics cell (g2m::phys::fill_physics_surfaces, block_level 0,
// cell-CENTRE classification) painted as road, at least one of its 4
// render-L0 corners (g2m::rasterize_road_blocks, block_level 2,
// raster_params_for_render_level(0), DGM1-sample-position classification)
// is road of the SAME class, and vice versa.
//
// A physics cell's own classification point (its centre, an integer metre)
// sits exactly at the shared corner of the 4 DGM1 cells around it
// (fill_physics_surfaces.h's own header comment - the same point the render
// L0 "one-cell proof" checks), so a physics cell (x, y)'s 4 render-L0
// corners are the 4 DGM1-sample positions at local (x, y), (x+1, y),
// (x, y+1), (x+1, y+1) relative to the same physics-tile origin.
//
// Pure g2m_phys/g2m_layer comparison - no physics_sim/Jolt type involved
// (sibling of test_render_vs_jolt.cpp, which is scoped to the HEIGHT
// render-vs-Jolt comparison via a real World/TileManager and does not touch
// roads).
#include "bridge_test_support.h"

#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/phys/fill_physics_surfaces.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

using namespace g2m_bridge_test;
using g2m::LandClass;
using g2m::Lattice;
using g2m::RasterParams;
using g2m::RoadBlockLookupResult;
using g2m::RoadBlockStatus;
using g2m::RoadSegment;
using g2m::RoadValueLut;
using g2m::SurfaceKind;
using g2m::TileKey;
using g2m::phys::fill_physics_surfaces;
using g2m::phys::PhysicsTileGrid;
using g2m::phys::PhysTileIndex;
using g2m::phys::ResidentHeightSet;

namespace {

std::shared_ptr<g2m::HeightTile> make_tile(const TileKey& key) {
    auto tile = std::make_shared<g2m::HeightTile>();
    tile->key = key;
    return tile;
}

// The lookup context/fn pair, mirroring g2m::phys::detail::surface_lookup
// (fill_physics_surfaces.h) exactly - this test builds its own render-side
// lattice/lookup rather than reusing that internal helper (which is scoped
// to the physics cell-centre lattice only), but the candidate-tile-search
// logic is identical.
struct LookupCtx {
    const ResidentHeightSet* resident;
    g2m::geo::UtmZone zone;
    std::int32_t tx_lo, tx_hi, ty_lo, ty_hi;
};

RoadBlockLookupResult lookup_roads(void* ctx_v, const TileKey& l2_key) {
    const auto& ctx = *static_cast<const LookupCtx*>(ctx_v);
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

            const g2m::HeightTile* tile = nullptr;
            std::shared_ptr<const std::vector<RoadSegment>> roads;
            const ResidentHeightSet::Lookup r = ctx.resident->find(l0, &tile, &roads);
            switch (r) {
            case ResidentHeightSet::Lookup::Missing: return RoadBlockLookupResult{RoadBlockStatus::Missing, {}};
            case ResidentHeightSet::Lookup::Absent: return RoadBlockLookupResult{RoadBlockStatus::None, {}};
            case ResidentHeightSet::Lookup::Resident:
                if (!roads) return RoadBlockLookupResult{RoadBlockStatus::Missing, {}};
                if (roads->empty()) return RoadBlockLookupResult{RoadBlockStatus::None, {}};
                return RoadBlockLookupResult{RoadBlockStatus::Segments, std::span<const RoadSegment>(*roads)};
            }
        }
    }
    return RoadBlockLookupResult{RoadBlockStatus::Missing, {}}; // unreachable in practice - fail safe
}

} // namespace

TEST_CASE("render vs physics roads: every road physics cell has a same-class render-L0 corner, and vice versa",
         "[bridge][terrain][roads][render]") {
    const PhysicsTileGrid grid(test_frame());
    const PhysTileIndex index{2, 1};
    const int cells = grid.samples() - 1; // ts, 255

    auto resident = std::make_shared<ResidentHeightSet>();
    std::vector<TileKey> keys;
    grid.l0_keys_for_square(index, 0, keys);

    const std::int64_t col0 = kE0 + static_cast<std::int64_t>(cells) * index.ix;
    const std::int64_t row0 = kN0 + static_cast<std::int64_t>(cells) * index.iy;

    // Several short capsules at assorted angles/ranks/land classes across
    // the tile, so the invariant is checked against real edge cases (partial
    // cell coverage, near-corner crossings), not just one clean straight
    // line.
    std::vector<RoadSegment> segs;
    segs.push_back(RoadSegment{{(col0 + 10) * 1000, (row0 + 5) * 1000},
                               {(col0 + 240) * 1000, (row0 + 200) * 1000},
                               /*half_width_mm=*/2500, LandClass::PavedRoad, SurfaceKind::Asphalt, /*rank=*/3});
    segs.push_back(RoadSegment{{(col0 + 5) * 1000, (row0 + 150) * 1000},
                               {(col0 + 250) * 1000, (row0 + 40) * 1000},
                               /*half_width_mm=*/800, LandClass::UnpavedRoad, SurfaceKind::Gravel, /*rank=*/1});
    segs.push_back(RoadSegment{{(col0 + 120) * 1000, row0 * 1000},
                               {(col0 + 120) * 1000, (row0 + 255) * 1000},
                               /*half_width_mm=*/1500, LandClass::PavedRoad, SurfaceKind::Asphalt, /*rank=*/5});
    auto roads = std::make_shared<const std::vector<RoadSegment>>(segs);
    for (const auto& key : keys) resident->install(key, make_tile(key), roads);

    const RoadValueLut lut =
        RoadValueLut::by_land_class(/*off_road=*/0, {{LandClass::PavedRoad, static_cast<std::uint16_t>(5)}, {LandClass::UnpavedRoad, static_cast<std::uint16_t>(9)}});

    // Physics side: fill_physics_surfaces, block_level 0, cell centres
    // (col0 + x + 1, row0 + y + 1), x/y in [0, cells)).
    std::vector<std::uint16_t> phys(static_cast<std::size_t>(cells) * static_cast<std::size_t>(cells));
    const g2m::RoadBlockStats phys_stats = fill_physics_surfaces(grid, index, *resident, lut, /*off_road=*/0, phys);
    REQUIRE(phys_stats.missing_blocks == 0);

    // Render side: rasterize_road_blocks, block_level 2, level-0 params,
    // over the DGM1-sample ("render-L0 vertex") corner grid - (cells+1)^2
    // points at (col0 + 0.5 + x', row0 + 0.5 + y'), x'/y' in [0, cells+1).
    // Corner (x', y') is physics cell (x, y)'s own corner iff x' in
    // {x, x+1} and y' in {y, y+1}.
    const int corners = cells + 1;
    // This lattice's floored-metre column/row range is [col0, col0+cells]
    // (256 consecutive integers at cells=255, one more than
    // fill_physics_surfaces' own [col0+1, col0+cells] - see this file's own
    // header comment) - a window exactly one L0 tile (256 m) wide, so it
    // still touches at most 2 tiles per axis, but its own tx_lo/tx_hi must
    // be derived from ITS range, not copied from fill_physics_surfaces.h's.
    LookupCtx ctx{resident.get(), grid.frame().zone(), 0, 0, 0, 0};
    ctx.tx_lo = static_cast<std::int32_t>(g2m::floor_div(col0, TileKey::kLevel0SizeM));
    ctx.tx_hi = static_cast<std::int32_t>(g2m::floor_div(col0 + cells, TileKey::kLevel0SizeM));
    ctx.ty_lo = static_cast<std::int32_t>(g2m::floor_div(row0, TileKey::kLevel0SizeM));
    ctx.ty_hi = static_cast<std::int32_t>(g2m::floor_div(row0 + cells, TileKey::kLevel0SizeM));

    const Lattice corner_lattice{col0 * 1000 + 500, row0 * 1000 + 500, 1000, corners, corners};
    std::vector<std::uint8_t> render_land_class(static_cast<std::size_t>(corners) * static_cast<std::size_t>(corners), 0);
    std::vector<std::uint8_t> render_surface(static_cast<std::size_t>(corners) * static_cast<std::size_t>(corners), 0);
    const std::optional<RasterParams> render_params = g2m::raster_params_for_render_level(0);
    REQUIRE(render_params.has_value());
    const g2m::RoadBlockStats render_stats = g2m::rasterize_road_blocks(
        corner_lattice, /*block_level=*/2, grid.frame().zone(), *render_params, &lookup_roads, &ctx, render_land_class,
        render_surface);
    REQUIRE(render_stats.missing_blocks == 0);

    auto render_id_at = [&](int xp, int yp) -> std::uint16_t {
        const std::size_t i = static_cast<std::size_t>(yp) * static_cast<std::size_t>(corners) + static_cast<std::size_t>(xp);
        return lut.at(static_cast<LandClass>(render_land_class[i]), static_cast<SurfaceKind>(render_surface[i]));
    };
    auto phys_id_at = [&](int x, int y) -> std::uint16_t {
        return phys[static_cast<std::size_t>(y) * static_cast<std::size_t>(cells) + static_cast<std::size_t>(x)];
    };

    int road_cells = 0, road_cells_with_match = 0;
    for (int y = 0; y < cells; ++y) {
        for (int x = 0; x < cells; ++x) {
            const std::uint16_t pid = phys_id_at(x, y);
            if (pid == 0) continue; // off_road - the invariant only speaks about road cells
            ++road_cells;
            const std::uint16_t c00 = render_id_at(x, y);
            const std::uint16_t c10 = render_id_at(x + 1, y);
            const std::uint16_t c01 = render_id_at(x, y + 1);
            const std::uint16_t c11 = render_id_at(x + 1, y + 1);
            if (c00 == pid || c10 == pid || c01 == pid || c11 == pid) ++road_cells_with_match;
        }
    }

    // ... and vice versa: every road render corner has a same-class physics
    // cell among the (up to 4) cells it is itself a corner of.
    int road_corners = 0, road_corners_with_match = 0;
    for (int yp = 0; yp < corners; ++yp) {
        for (int xp = 0; xp < corners; ++xp) {
            const std::uint16_t rid = render_id_at(xp, yp);
            if (rid == 0) continue;
            ++road_corners;
            bool matched = false;
            for (int dy = -1; dy <= 0 && !matched; ++dy) {
                for (int dx = -1; dx <= 0 && !matched; ++dx) {
                    const int cx = xp + dx, cy = yp + dy;
                    if (cx < 0 || cy < 0 || cx >= cells || cy >= cells) continue;
                    if (phys_id_at(cx, cy) == rid) matched = true;
                }
            }
            if (matched) ++road_corners_with_match;
        }
    }

    INFO("road physics cells " << road_cells << " (matched " << road_cells_with_match << "), road render corners "
                               << road_corners << " (matched " << road_corners_with_match << ")");
    REQUIRE(road_cells > 0);
    REQUIRE(road_corners > 0);
    CHECK(road_cells_with_match == road_cells);
    CHECK(road_corners_with_match == road_corners);
}
