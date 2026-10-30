// Render-only lane-paint meshes, independent of renderer shaders/textures.
#pragma once
#include "g2m/world/road_ribbons.h"
namespace g2m::world {
struct RoadPaintOptions {
  double line_width_m = .15;
  double lift_m = .002;
  std::size_t max_paint_triangles = 250000;
};
struct RoadPaintResult {
  RoadRibbonTile mesh;
  std::size_t paint_triangles = 0;
  bool complete = true; // false: capacity/invalid data, original mesh retained
};
// Converts the current home-region marking semantics to explicit paint
// triangles: 3/6 m dash, 6/12 m fast-road dash, broad broken edge lines and
// solid separators. Intersects paint bands with each already-draped triangle.
// UV station controls dash phase (never local tile x/y), so adjoining tiles and
// rebase agree. Existing arrows/crosswalks/stop lines (flag 16) survive
// unchanged. No collision changes. The original input is returned on
// bounded-work failure; no half-painted tile.
RoadPaintResult build_road_paint(const RoadRibbonTile &,
                                 const RoadPaintOptions & = {});
} // namespace g2m::world
