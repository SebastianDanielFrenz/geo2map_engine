#include "g2m/world/road_paint.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
namespace {
using namespace g2m::world;
RoadRibbonTile ribbon(float start, float end, int flags = 1, int lanes = 2) {
  RoadRibbonTile m;
  const auto v = [](float u, float s, int f, int n) {
    return RoadRibbonVertex{
        u, s, 100 + .01f * s,        u, s,
        8, 1, static_cast<float>(n), 1, static_cast<float>(f),
        0};
  };
  m.vertices = {v(-4, start, flags, lanes), v(4, start, flags, lanes),
                v(4, end, flags, lanes), v(-4, end, flags, lanes)};
  m.indices = {0, 1, 2, 0, 2, 3};
  return m;
}
double paint_area(const RoadPaintResult &r) {
  double a = 0;
  for (std::size_t i = 0; i < r.mesh.indices.size(); i += 3) {
    const auto &p =
        r.mesh.vertices[static_cast<std::size_t>(r.mesh.indices[i])];
    if (static_cast<int>(p.flags) != 16)
      continue;
    const auto &q =
        r.mesh.vertices[static_cast<std::size_t>(r.mesh.indices[i + 1])];
    const auto &s =
        r.mesh.vertices[static_cast<std::size_t>(r.mesh.indices[i + 2])];
    a += std::abs((q.x - p.x) * (s.y - p.y) - (s.x - p.x) * (q.y - p.y)) * .5;
  }
  return a;
}
} // namespace
TEST_CASE("native paint uses metric lane widths and station-based dash phase",
          "[world][road_paint]") {
  const auto whole = build_road_paint(ribbon(0, 18));
  REQUIRE(whole.complete);
  CHECK(whole.paint_triangles > 0);
  CHECK(std::abs(paint_area(whole) - .15 * 6) < 1e-5);
  const auto left = build_road_paint(ribbon(0, 7)),
             right = build_road_paint(ribbon(7, 18));
  CHECK(std::abs(paint_area(left) + paint_area(right) - paint_area(whole)) <
        1e-5);
  const auto negative = build_road_paint(ribbon(-9, 0));
  CHECK(std::abs(paint_area(negative) - .15 * 3) < 1e-5);
  for (std::size_t i = 4; i < whole.mesh.vertices.size(); ++i) {
    const auto &v = whole.mesh.vertices[i];
    CHECK(std::abs(v.z - (100 + .01 * v.v + .002)) < 1e-5);
    CHECK(std::abs(v.u) <= .07501);
  }
}
TEST_CASE("solid separators edges and explicit paint survive native conversion",
          "[world][road_paint]") {
  auto solid = build_road_paint(ribbon(0, 18, 1 | 4));
  CHECK(std::abs(paint_area(solid) - .15 * 18) < 1e-5);
  auto edges = build_road_paint(ribbon(0, 18, 1 | 2 | 4));
  CHECK(std::abs(paint_area(edges) - .15 * 18 * 3) < 1e-4);
  auto hidden = build_road_paint(ribbon(0, 18, 1 | 2 | 4 | 512));
  CHECK(std::abs(paint_area(hidden) - .15 * 18 * 2) < 1e-4);
  auto explicit_mark = ribbon(0, 2, 16, 1);
  auto out = build_road_paint(explicit_mark);
  CHECK(out.mesh.vertices.size() == explicit_mark.vertices.size());
  CHECK(out.mesh.indices == explicit_mark.indices);
  auto overlay = build_road_paint(ribbon(0, 18, 1 | 32));
  CHECK(std::abs(paint_area(overlay) - .15 * 6) < 1e-5);
  for (auto i : overlay.mesh.indices)
    CHECK(static_cast<int>(
              overlay.mesh.vertices[static_cast<std::size_t>(i)].flags) == 16);
}
TEST_CASE(
    "paint work limits and invalid data retain the complete original mesh",
    "[world][road_paint]") {
  auto src = ribbon(0, 100);
  RoadPaintOptions o;
  o.max_paint_triangles = 1;
  auto out = build_road_paint(src, o);
  CHECK_FALSE(out.complete);
  CHECK(out.mesh.vertices.size() == src.vertices.size());
  CHECK(out.mesh.indices == src.indices);
  CHECK(out.paint_triangles == 0);
  o = {};
  src.indices[0] = -1;
  CHECK_FALSE(build_road_paint(src, o).complete);
  src = ribbon(0, 18);
  src.vertices[0].u = std::numeric_limits<float>::infinity();
  CHECK_FALSE(build_road_paint(src).complete);
}
TEST_CASE("unmarked road paths and missing edge lines do not acquire paint",
          "[world][road_paint]") {
  auto no = build_road_paint(ribbon(0, 18, 8192, 1));
  CHECK(no.paint_triangles == 0);
  CHECK(no.mesh.indices.size() == 6);
  auto empty = build_road_paint(ribbon(3.1f, 8.9f));
  CHECK(empty.paint_triangles == 0);
}
