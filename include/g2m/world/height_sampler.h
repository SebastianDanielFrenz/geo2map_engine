// g2m/world/height_sampler.h - point height queries on the level-0 cell-centre
// lattice (terrain migration plan phase 1; vault G2M-020, request G-4).
//
// Both samplers read the same lattice: DGM1 level-0 sample (c, r) of the
// session frame sits at session-local (c - e0 + 0.5, r - n0 + 0.5) metres,
// where (e0, n0) is the session origin in whole metres (SessionFrame::e0_m /
// n0_m). A query point is (x, y) in session-local metres.
//
//   sample_height_bilinear  bilinear over the four surrounding samples. Ported
//                           bit for bit from racing_game's sample_l0_height
//                           (route_check.cpp). The render-only answer: it is
//                           NOT the surface physics_sim collides with. NoData
//                           in any of the four corners, or a missing tile,
//                           gives nullopt.
//
//   sample_height_exact     the surface physics_sim's Jolt heightfield (and
//                           g2m_mesh's L0 render chunks) actually has. Every
//                           quad is split along its NW-SE diagonal, the
//                           (c, r+1)-(c+1, r) one (derivation: the comment on
//                           mesh::build_chunk, include/g2m/mesh/terrain_chunk.h);
//                           the answer is linear inside the triangle holding
//                           the point:
//                             tx + ty <= 1 (SW triangle (c,r) (c+1,r) (c,r+1)):
//                               z = h00 + tx (h10 - h00) + ty (h01 - h00)
//                             tx + ty >  1 (NE triangle (c+1,r) (c+1,r+1) (c,r+1)):
//                               z = h11 + (1 - tx) (h01 - h11) + (1 - ty) (h10 - h11)
//                           NoData: a triangle is solid only if all three of its
//                           corners are valid (Jolt's HeightFieldShape drops a
//                           triangle with a no-collision corner, and build_chunk
//                           drops it too), so the unused fourth corner may be
//                           NoData or sit in a missing tile without affecting
//                           the answer. The value is the exact piecewise-linear
//                           surface over the double heights (h / 256); Jolt
//                           stores quantised floats, so physics itself differs
//                           from it by its own quantisation (<= 5 mm on the
//                           fixture, bridges/physics_sim test_render_vs_jolt).
//
// Both are pure, allocation-free apart from the std::function call, and
// thread-safe if the lookup is. `lookup` returns the level-0 height tile with
// the given key or nullptr (not resident / no coverage).
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"

#include <cstdint>
#include <functional>
#include <optional>

namespace g2m::world {

using L0TileLookupFn = std::function<const HeightTile*(const TileKey&)>;

std::optional<double> sample_height_bilinear(const L0TileLookupFn& lookup, geo::UtmZone zone, std::int64_t e0,
                                             std::int64_t n0, double x, double y);

std::optional<double> sample_height_exact(const L0TileLookupFn& lookup, geo::UtmZone zone, std::int64_t e0,
                                          std::int64_t n0, double x, double y);

} // namespace g2m::world
