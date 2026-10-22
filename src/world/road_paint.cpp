// Feature reference: BeamNG.Earth roads.py/roadmesh.py (606ba0f), MIT,
// (c) 2026 TecnaGamer. Native geometry replaces stock line/decal materials.
#include "g2m/world/road_paint.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <vector>
namespace g2m::world {
namespace {
using Vertex = RoadRibbonVertex;
using Poly = std::vector<Vertex>;
Vertex lerp(const Vertex &a, const Vertex &b, double t) {
  Vertex c = a;
  const auto f = [t](float x, float y) {
    return static_cast<float>(x + (y - x) * t);
  };
  c.x = f(a.x, b.x);
  c.y = f(a.y, b.y);
  c.z = f(a.z, b.z);
  c.u = f(a.u, b.u);
  c.v = f(a.v, b.v);
  c.width = f(a.width, b.width);
  c.fade = f(a.fade, b.fade);
  return c;
}
template <class F> Poly clip(const Poly &input, F distance) {
  Poly out;
  if (input.empty())
    return out;
  auto a = input.back();
  double da = distance(a);
  for (auto b : input) {
    const double db = distance(b);
    if ((da >= 0) != (db >= 0))
      out.push_back(lerp(a, b, da / (da - db)));
    if (db >= 0)
      out.push_back(b);
    a = b;
    da = db;
  }
  return out;
}
bool finite(const Vertex &v) {
  return std::isfinite(v.x) && std::isfinite(v.y) && std::isfinite(v.z) &&
         std::isfinite(v.u) && std::isfinite(v.v) && std::isfinite(v.width) &&
         std::isfinite(v.fade) && std::isfinite(v.flags) &&
         std::isfinite(v.lanes) && std::isfinite(v.forward);
}
} // namespace
RoadPaintResult build_road_paint(const RoadRibbonTile &input,
                                 const RoadPaintOptions &options) {
  RoadPaintResult result;
  result.mesh = input;
  auto &output = result.mesh;
  const auto fail = [&]() {
    result.mesh = input;
    result.complete = false;
    result.paint_triangles = 0;
    return std::move(result);
  };
  if (!std::isfinite(options.line_width_m) || options.line_width_m < .05 ||
      options.line_width_m > .5 || !std::isfinite(options.lift_m) ||
      options.lift_m < 0 || options.lift_m > .05 ||
      input.indices.size() % 3 != 0)
    return fail();
  for (const auto &v : input.vertices)
    if (!finite(v) || v.flags < 0 || v.flags > 65535 || v.lanes < 0 ||
        v.lanes > 32)
      return fail();
  output.indices.clear();
  const auto emit = [&](const Poly &polygon) -> bool {
    if (polygon.size() < 3)
      return true;
    if (result.paint_triangles + polygon.size() - 2 >
        options.max_paint_triangles)
      return false;
    for (std::size_t i = 2; i < polygon.size(); ++i) {
      const auto &a = polygon[0];
      const auto &b = polygon[i - 1];
      const auto &c = polygon[i];
      const double area = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
      if (std::abs(area) < 1e-9)
        continue;
      if (output.vertices.size() >
          static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max() -
                                   3))
        return false;
      for (auto v : {a, b, c}) {
        v.z += static_cast<float>(options.lift_m);
        v.flags = 16;
        v.fade = 1;
        v.verge = 0;
        output.indices.push_back(
            static_cast<std::int32_t>(output.vertices.size()));
        output.vertices.push_back(v);
      }
      ++result.paint_triangles;
    }
    return true;
  };
  for (std::size_t index = 0; index < input.indices.size(); index += 3) {
    Poly triangle;
    for (std::size_t j = 0; j < 3; ++j) {
      const auto n = input.indices[index + j];
      if (n < 0 || static_cast<std::size_t>(n) >= input.vertices.size())
        return fail();
      auto v = input.vertices[static_cast<std::size_t>(n)];
      if (!finite(v) || v.flags < 0 || v.flags > 65535 || v.lanes < 0 ||
          v.lanes > 32)
        return fail();
      triangle.push_back(v);
    }
    const auto first = triangle.front();
    const int flags = static_cast<int>(first.flags);
    // Preserve non-overlay surfaces and all existing explicit paint.
    if ((flags & 32) == 0 || (flags & 16) != 0)
      for (std::size_t j = 0; j < 3; ++j)
        output.indices.push_back(input.indices[index + j]);
    if ((flags & 1) == 0 || (flags & 16) != 0 || first.verge > .5f)
      continue;
    for (const auto &v : triangle)
      if (static_cast<int>(v.flags) != flags || v.lanes != first.lanes ||
          v.forward != first.forward)
        return fail();
    triangle = clip(triangle, [](const Vertex &v) {
      return static_cast<double>(v.fade) - .5;
    });
    if (triangle.empty())
      continue;
    const auto band = [&](double fraction, double offset, double width,
                          double dash, double gap) -> bool {
      auto strip = clip(triangle, [&](const Vertex &v) {
        return static_cast<double>(v.u) - fraction * v.width - offset +
               width * .5;
      });
      strip = clip(strip, [&](const Vertex &v) {
        return fraction * v.width + offset + width * .5 -
               static_cast<double>(v.u);
      });
      if (strip.empty())
        return true;
      if (dash == 0)
        return emit(strip);
      double low = 1e30, high = -1e30;
      for (const auto &v : strip) {
        low = std::min(low, static_cast<double>(v.v));
        high = std::max(high, static_cast<double>(v.v));
      }
      const double period = dash + gap;
      const double begin = std::floor(low / period) * period;
      if (!std::isfinite(begin) || high - low > 4096 || std::abs(begin) > 1e9)
        return false;
      for (double s = begin; s <= high; s += period) {
        auto p = clip(strip, [&](const Vertex &v) { return v.v - s; });
        p = clip(p, [&](const Vertex &v) { return s + dash - v.v; });
        if (!emit(p))
          return false;
      }
      return true;
    };
    const bool fast = (flags & 8) != 0;
    if ((flags & 2) != 0)
      for (const int side : {-1, 1}) {
        if ((side > 0 && (flags & 512) != 0) ||
            (side < 0 && (flags & 1024) != 0))
          continue;
        const bool broken =
            (side > 0 && (flags & 64) != 0) || (side < 0 && (flags & 128) != 0);
        const double dash = broken ? (fast ? 6 : 1.5) : 0;
        if (!band(side * .5, -side * .25,
                  broken && fast ? .30 : options.line_width_m, dash, dash))
          return fail();
      }
    const int lanes = static_cast<int>(std::round(first.lanes));
    for (int lane = 1; lane < lanes; ++lane) {
      const bool separator = first.forward > 0 && first.forward < first.lanes &&
                             std::abs(lane - first.forward) < .25;
      const bool solid = separator && (flags & 4) != 0;
      const double dash = solid ? 0 : (fast ? 6 : 3);
      if (!band(-.5 + static_cast<double>(lane) / lanes, 0,
                options.line_width_m, dash, 2 * dash))
        return fail();
    }
  }
  // The original vertex attributes are retained for texture/material mapping;
  // only procedural shader paint is disabled on the original triangles.
  for (std::size_t i = 0; i < input.vertices.size(); ++i) {
    const int flags = static_cast<int>(output.vertices[i].flags);
    if ((flags & 16) == 0)
      output.vertices[i].flags = static_cast<float>(flags & ~1);
  }
  return result;
}
} // namespace g2m::world
