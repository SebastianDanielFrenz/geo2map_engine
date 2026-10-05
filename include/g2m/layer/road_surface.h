#pragma once
#include "g2m/layer/road_geom_tile.h"
namespace g2m {
// Street fidelity P2 slice 2 (design section 4.1): the ONE definition of a carved road height. Carving, the
// terrain.class derivation and every consumer evaluate this function; nothing re-derives the crown or the verge.
//
//   z(s, t) = p(s) + C(s, t) - C(s, 0) + V(s, t) + F(s, t)
//
// p is the entry's vertical profile on the reference line, C the crown (roof, or one-sided for a one-way
// carriageway), V the verge drop between the paved and the outer edge, F the carved calming features. Subtracting
// C(s, 0) keeps the profile exact on the reference line, where it was fitted to the DEM (design D5).
//
// The two numbers that are carving parameters rather than road geometry come from data/carve/carve_default.json
// (CarveParams::surface_params()); the defaults below equal that file's values.
struct RoadSurfaceParams {
    double crown_round_m = 0.5;   // r0: the roof ridge is rounded over |t - pivot| < r0 (game parity)
    double verge_drop_m = 0.06;   // depth below the paved edge at the outer edge
    friend bool operator==(const RoadSurfaceParams&, const RoadSurfaceParams&) = default;
};

// The crown term C of design 4.1 for the signed offset d = t - crown pivot. `crossfall` is the slope (0.025 for
// 25 per mille). Roof: -crossfall * rho(d) with rho(u) = |u| for |u| >= r0 and u^2/(2 r0) + r0/2 below (C1, no
// square root); OneSidedRight: crossfall * d; OneSidedLeft: -crossfall * d. Only + - * /.
double road_crown_m(CrownKind kind, double crossfall, double d, double crown_round_m) noexcept;

// The verge term V: 0 inside the paved band, -verge_drop * road_smooth_step(distance past the paved edge /
// verge width) between the paved and the outer edge on that side (t already clamped to the outer edges).
double road_verge_m(const CarveFootprint& f, double t, double verge_drop_m) noexcept;

// Height of the carved road surface of entry `entry` at station s and lateral offset t (metres, positive left of
// the reference direction). s is clamped to [0, reference length], t to [outer_right, outer_left]. The carved
// calming features act over their lateral extent (full carriageway for width 0 and for a cushion, else the width
// centred on the carriageway), with no lateral blend. Returns quiet NaN for an entry without both a profile and a
// cross-section, or an index out of range. Pure, allocation-free, only + - * / and the det helpers.
double road_surface_height_m(const RoadGeomTile& tile, std::size_t entry, double s, double t,
    const RoadSurfaceParams& params = {}) noexcept;
} // namespace g2m
