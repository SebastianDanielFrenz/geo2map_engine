// Street fidelity P1 slice 6: the grip-raster input of a roads.geom tile (design 8.1). One capsule chain per
// cross-section entry whose footprint is the SAME carve_footprint that carving and the render ribbon use, so
// the painted road, the carved flat area and the ribbon agree.
#pragma once
#include "g2m/layer/road_geom_tile.h"

namespace g2m {
// For every entry with a cross-section, at stations every `spacing_m` (plus s = L), one RoadSegment per
// station pair: centre line = the reference points offset by m(s) = (roadbed_left + roadbed_right) / 2 at both
// stations, half_width_mm = llround(1000 * the MEAN over the two stations of (roadbed_left - roadbed_right) / 2)
// (design 8.1 says max; the mean halves the error and makes it two-sided, see below),
// land_class/surface from the entry's RoadAttributes, rank/way_id/bridge/speed limits as extract_road_segments.
// Each Sidewalk band is emitted as its own capsule chain with surface PavingStones (paved), before the road
// bed of the same way so the asphalt overwrites it; Verge bands are not emitted (the cell keeps its terrain
// class). A capsule is kept when its bounding box widened by its half-width touches the box of `l0_key` (any
// level; a different UTM zone yields an empty list). Order: (rank, way_id, sidewalk before road bed, entry,
// station), so rasterize_road_segments' later-overwrites rule paints important roads last. Entries without a
// cross-section (declined, or an encoding-5 tile) are not emitted: a consumer falls back to
// extract_road_segments for them. Deterministic: only g2m::det math and llround.
// Error of an edge against the true envelope: at most omega * spacing_m / 4 + omega^2 * h / 2 laterally, where omega
// is the width rate of the band chain (a smooth-step ramp peaks at twice its mean rate) and h its half-width: the
// first term is the constant half-width per station pair, the second the round caps of the neighbouring capsules.
// 1.7 cm for a ramp with peak slope 1:20, 4.8 cm for the steepest shipped taper (3.5 m over 60 m, 1 m spacing);
// plus the chord sagitta spacing_m^2 * curvature / 8. A Sidewalk capsule reaches 8 cm into the road bed (painted
// later) so the two chains never leave a gap.
std::vector<RoadSegment> envelope_road_segments(const RoadGeomTile& tile, const TileKey& l0_key, double spacing_m = 1.0);
}
