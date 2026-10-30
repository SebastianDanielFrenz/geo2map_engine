// Algorithmic reference: BeamNG.Earth server/buildings.py (606ba0f),
// Copyright (c) 2026 TecnaGamer, MIT; full notice in THIRD_PARTY_NOTICES.md.
// Reimplemented for g2m's deterministic math, mapped total heights, courtyards,
// stable feature IDs and engine-neutral materials. No BeamNG asset dependency.
#include "g2m/world/building_mesh.h"
#include "g2m/core/det_math.h"
#include "g2m/core/geo/angle.h"
#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <optional>
#include <string_view>
#include <utility>

namespace g2m::world {
namespace {
using V = std::array<double, 3>;
using C = std::array<float, 3>;
V sub(V a, V b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V cross(V a, V b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
std::uint64_t mix(std::uint64_t x) {
  x += 0x9e3779b97f4a7c15ULL;
  x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
  x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
  return x ^ (x >> 31);
}
C colour(std::string_view s, C fallback) {
  if (s.size() == 4 && s[0] == '#') {
    unsigned n = 0;
    auto p = std::from_chars(s.data() + 1, s.data() + 4, n, 16);
    if (p.ec == std::errc{} && p.ptr == s.data() + 4)
      return {static_cast<float>((n >> 8) & 15) / 15,
              static_cast<float>((n >> 4) & 15) / 15,
              static_cast<float>(n & 15) / 15};
  }
  if (s.size() == 7 && s[0] == '#') {
    unsigned n = 0;
    auto p = std::from_chars(s.data() + 1, s.data() + 7, n, 16);
    if (p.ec == std::errc{} && p.ptr == s.data() + 7)
      return {static_cast<float>((n >> 16) & 255) / 255,
              static_cast<float>((n >> 8) & 255) / 255,
              static_cast<float>(n & 255) / 255};
  }
  if (s == "white")
    return {.9f, .9f, .88f};
  if (s == "red")
    return {.6f, .16f, .12f};
  if (s == "brown")
    return {.35f, .23f, .15f};
  if (s == "grey" || s == "gray")
    return {.5f, .5f, .5f};
  if (s == "black")
    return {.08f, .08f, .08f};
  if (s == "beige")
    return {.76f, .71f, .60f};
  if (s == "yellow")
    return {.83f, .73f, .43f};
  return fallback;
}
struct Builder {
  BuildingMesh mesh;
  const BuildingMeshOptions &o;
  V uv_origin{};
  void tri(V a, V b, V c, C tint, BuildingMaterial material) {
    const auto n = cross(sub(b, a), sub(c, a));
    const double len = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    if (len < 1e-9)
      return;
    for (const auto p : {a, b, c}) {
      BuildingVertex v;
      v.position = {static_cast<float>(p[0] - o.origin_e),
                    static_cast<float>(p[1] - o.origin_n),
                    static_cast<float>(p[2])};
      v.normal = {static_cast<float>(n[0] / len),
                  static_cast<float>(n[1] / len),
                  static_cast<float>(n[2] / len)};
      v.colour = tint;
      v.material = material;
      // Metric planar coordinates on this actual face, including pitched roofs
      // and vertical walls. A building anchor avoids large-coordinate float
      // precision loss and remains independent of the renderer's floating
      // origin.
      V tangent = std::abs(n[2] / len) > .999 ? V{1, 0, 0} : V{n[1], -n[0], 0};
      const double tl =
          std::sqrt(tangent[0] * tangent[0] + tangent[1] * tangent[1]);
      for (auto &c : tangent)
        c /= tl;
      const V unit{n[0] / len, n[1] / len, n[2] / len};
      const auto bitangent = cross(unit, tangent);
      const auto q = sub(p, uv_origin);
      v.uv = {static_cast<float>(q[0] * tangent[0] + q[1] * tangent[1] +
                                 q[2] * tangent[2]),
              static_cast<float>(q[0] * bitangent[0] + q[1] * bitangent[1] +
                                 q[2] * bitangent[2])};
      mesh.vertices.push_back(v);
    }
  }
  void box(V origin, V dx, V dy, V dz, C tint, BuildingMaterial material) {
    std::array<V, 8> points;
    for (int i = 0; i < 8; ++i)
      for (int axis = 0; axis < 3; ++axis)
        points[static_cast<std::size_t>(i)][static_cast<std::size_t>(axis)] =
            origin[static_cast<std::size_t>(axis)] +
            ((i & 1) ? dx[static_cast<std::size_t>(axis)] : 0) +
            ((i & 2) ? dy[static_cast<std::size_t>(axis)] : 0) +
            ((i & 4) ? dz[static_cast<std::size_t>(axis)] : 0);
    const auto n = cross(dx, dy);
    const bool reverse = n[0] * dz[0] + n[1] * dz[1] + n[2] * dz[2] < 0;
    constexpr int faces[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                                 {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
    for (const auto &f : faces) {
      const auto a = points[static_cast<std::size_t>(f[0])],
                 b = points[static_cast<std::size_t>(f[1])],
                 c = points[static_cast<std::size_t>(f[2])],
                 d = points[static_cast<std::size_t>(f[3])];
      if (reverse) {
        tri(a, d, c, tint, material);
        tri(a, c, b, tint, material);
      } else {
        tri(a, b, c, tint, material);
        tri(a, c, d, tint, material);
      }
    }
  }
  void quad(V a, V b, V c, V d, C tint, BuildingMaterial material) {
    tri(a, b, c, tint, material);
    tri(a, c, d, tint, material);
  }
};
double area(const std::vector<BuildingPoint> &ring) {
  double a = 0;
  const auto origin = ring.front();
  for (std::size_t i = 0; i < ring.size(); ++i) {
    const auto p = ring[i], q = ring[(i + 1) % ring.size()];
    a += (p.e - origin.e) * (q.n - origin.n) -
         (q.e - origin.e) * (p.n - origin.n);
  }
  return a * .5;
}
struct Box {
  double e = 0, n = 0, ux = 0, uy = 0, hl = 0, hw = 0;
};
std::optional<double> roof_direction_degrees(std::string_view value) {
  while (!value.empty() && (value.front() == ' ' || value.front() == '\t'))
    value.remove_prefix(1);
  while (!value.empty() && (value.back() == ' ' || value.back() == '\t'))
    value.remove_suffix(1);
  if (value.empty())
    return std::nullopt;
  double degrees = 0;
  const auto parsed = std::from_chars(value.data(), value.data() + value.size(),
                                      degrees, std::chars_format::general);
  if (parsed.ec == std::errc{} && parsed.ptr == value.data() + value.size() &&
      std::isfinite(degrees) && degrees >= 0 && degrees <= 360)
    return degrees == 360 ? 0 : degrees;
  constexpr std::pair<std::string_view, double> compass[] = {
      {"N", 0},   {"NNE", 22.5},  {"NE", 45},  {"ENE", 67.5},
      {"E", 90},  {"ESE", 112.5}, {"SE", 135}, {"SSE", 157.5},
      {"S", 180}, {"SSW", 202.5}, {"SW", 225}, {"WSW", 247.5},
      {"W", 270}, {"WNW", 292.5}, {"NW", 315}, {"NNW", 337.5}};
  for (const auto &[name, angle] : compass) {
    if (value.size() != name.size())
      continue;
    bool equal = true;
    for (std::size_t i = 0; i < value.size(); ++i) {
      const char c = value[i] >= 'a' && value[i] <= 'z'
                         ? static_cast<char>(value[i] - 'a' + 'A')
                         : value[i];
      equal = equal && c == name[i];
    }
    if (equal)
      return angle;
  }
  return std::nullopt;
}
bool align_roof_direction(const Building &b, Box &box, double degrees,
                          RoofForm form) {
  double east = 0, north = 0;
  g2m::geo::sincosd(degrees, east, north);
  // A skillion's high edge is opposite the mapped downhill direction. For
  // ridge-shaped roofs, either sign names the same pair of slope faces.
  if (form == RoofForm::Skillion) {
    east = -east;
    north = -north;
  }
  const double ux = north, uy = -east;
  double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
  for (const auto &ring : b.outers)
    for (const auto p : ring) {
      const double u = p.e * ux + p.n * uy;
      const double v = -p.e * uy + p.n * ux;
      u0 = std::min(u0, u);
      u1 = std::max(u1, u);
      v0 = std::min(v0, v);
      v1 = std::max(v1, v);
    }
  if (!(u1 - u0 >= 1 && v1 - v0 >= 1))
    return false;
  box.ux = ux;
  box.uy = uy;
  box.hl = (u1 - u0) * .5;
  box.hw = (v1 - v0) * .5;
  const double um = (u0 + u1) * .5, vm = (v0 + v1) * .5;
  box.e = um * ux - vm * uy;
  box.n = um * uy + vm * ux;
  return true;
}
bool rectangle(const Building &b, Box &box, bool require_rectangle = true) {
  if (b.outers.size() != 1 ||
      (require_rectangle && (!b.inners.empty() || b.outers[0].size() != 4)))
    return false;
  const auto &r = b.outers[0];
  double longest = 0;
  for (std::size_t i = 0; i < r.size(); ++i) {
    const auto a = r[i], c = r[(i + 1) % 4];
    const double x = c.e - a.e, y = c.n - a.n, l = x * x + y * y;
    if (l > longest) {
      longest = l;
      box.ux = x;
      box.uy = y;
    }
  }
  if (longest < 1)
    return false;
  const double len = std::sqrt(longest);
  box.ux /= len;
  box.uy /= len;
  // Canonical orientation independent of winding and the first source vertex.
  if (box.ux < 0 || (box.ux == 0 && box.uy < 0)) {
    box.ux = -box.ux;
    box.uy = -box.uy;
  }
  const auto p0 = r.front();
  double a0 = 1e30, a1 = -1e30, b0 = 1e30, b1 = -1e30;
  for (auto p : r) {
    const double x = p.e - p0.e, y = p.n - p0.n, u = x * box.ux + y * box.uy,
                 v = -x * box.uy + y * box.ux;
    a0 = std::min(a0, u);
    a1 = std::max(a1, u);
    b0 = std::min(b0, v);
    b1 = std::max(b1, v);
  }
  box.hl = (a1 - a0) * .5;
  box.hw = (b1 - b0) * .5;
  if (box.hw < .5 || box.hl < .5)
    return false;
  box.e = p0.e + box.ux * (a0 + a1) * .5 - box.uy * (b0 + b1) * .5;
  box.n = p0.n + box.uy * (a0 + a1) * .5 + box.ux * (b0 + b1) * .5;
  if (!require_rectangle)
    return true;
  for (auto p : r) {
    const double x = p.e - box.e, y = p.n - box.n, u = x * box.ux + y * box.uy,
                 v = -x * box.uy + y * box.ux;
    if (std::abs(std::abs(u) - box.hl) > .03 ||
        std::abs(std::abs(v) - box.hw) > .03)
      return false;
  }
  return true;
}
void flat_roof(Builder &m, const Building &b, double z, C tint,
               const std::function<void(V, V, V)> &emit = {}) {
  // Even/odd scan slabs cover concave rings and courtyard holes without an
  // unconstrained triangulation. Same exact contours as the footprint walls.
  std::vector<double> ys;
  std::vector<const std::vector<BuildingPoint> *> rings;
  for (const auto &r : b.outers)
    rings.push_back(&r);
  for (const auto &r : b.inners)
    rings.push_back(&r);
  for (const auto *r : rings)
    for (auto p : *r)
      ys.push_back(p.n);
  std::sort(ys.begin(), ys.end());
  ys.erase(std::unique(ys.begin(), ys.end()), ys.end());
  struct Edge {
    double mid, lo, hi;
  };
  for (std::size_t j = 1; j < ys.size(); ++j) {
    const double lo = ys[j - 1], hi = ys[j], mid = (lo + hi) * .5;
    std::vector<Edge> edges;
    for (const auto *r : rings)
      for (std::size_t i = 0; i < r->size(); ++i) {
        auto a = (*r)[i], c = (*r)[(i + 1) % r->size()];
        if (mid <= std::min(a.n, c.n) || mid >= std::max(a.n, c.n))
          continue;
        const double slope = (c.e - a.e) / (c.n - a.n);
        edges.push_back({a.e + (mid - a.n) * slope, a.e + (lo - a.n) * slope,
                         a.e + (hi - a.n) * slope});
      }
    std::sort(edges.begin(), edges.end(),
              [](const auto &a, const auto &c) { return a.mid < c.mid; });
    for (std::size_t i = 1; i < edges.size(); i += 2) {
      auto a = edges[i - 1], c = edges[i];
      if (emit) {
        emit({a.lo, lo, z}, {c.lo, lo, z}, {c.hi, hi, z});
        emit({a.lo, lo, z}, {c.hi, hi, z}, {a.hi, hi, z});
      } else
        m.quad({a.lo, lo, z}, {c.lo, lo, z}, {c.hi, hi, z}, {a.hi, hi, z}, tint,
               BuildingMaterial::Roof);
    }
  }
}
struct RoofPlane {
  double u, v, z;
};
void clipped_roof(Builder &m, const Building &building, const Box &box,
                  RoofForm form, double eave, double rise, C roof, C wall,
                  bool mapped_direction) {
  const double top = eave + rise;
  std::vector<RoofPlane> planes;
  if (form == RoofForm::Skillion)
    planes = {{0, rise / (2 * box.hw), eave + rise * .5}};
  else {
    planes = {{0, rise / box.hw, top}, {0, -rise / box.hw, top}};
    if (form == RoofForm::Hipped) {
      if (mapped_direction && box.hl < box.hw) {
        // With the mapped slope axis wider than the ridge axis, the hip ridge
        // collapses to a peak. Use four planes meeting at the mapped total
        // height instead of letting the short-axis planes cut the peak down.
        planes.push_back({rise / box.hl, 0, top});
        planes.push_back({-rise / box.hl, 0, top});
      } else {
        planes.push_back({rise / box.hw, 0, eave + rise * box.hl / box.hw});
        planes.push_back({-rise / box.hw, 0, eave + rise * box.hl / box.hw});
      }
    }
    if (form == RoofForm::Pyramidal) {
      planes.push_back({rise / box.hl, 0, top});
      planes.push_back({-rise / box.hl, 0, top});
    }
  }
  const auto height = [&](const RoofPlane &p, V a) {
    const double x = a[0] - box.e, y = a[1] - box.n;
    return p.z + p.u * (x * box.ux + y * box.uy) +
           p.v * (-x * box.uy + y * box.ux);
  };
  const auto minimum = [&](V a) {
    double h = 1e30;
    for (const auto &p : planes)
      h = std::min(h, height(p, a));
    return h;
  };
  const auto emit = [&](V a, V b, V c) {
    for (std::size_t plane = 0; plane < planes.size(); ++plane) {
      std::vector<V> polygon{a, b, c};
      for (std::size_t other = 0; other < planes.size() && !polygon.empty();
           ++other) {
        if (other == plane)
          continue;
        std::vector<V> output;
        auto prev = polygon.back();
        double dp = height(planes[other], prev) - height(planes[plane], prev);
        for (auto next : polygon) {
          const double dn =
              height(planes[other], next) - height(planes[plane], next);
          if ((dp >= 0) != (dn >= 0)) {
            const double t = dp / (dp - dn);
            output.push_back({prev[0] + (next[0] - prev[0]) * t,
                              prev[1] + (next[1] - prev[1]) * t, 0});
          }
          if (dn >= 0)
            output.push_back(next);
          prev = next;
          dp = dn;
        }
        polygon = std::move(output);
      }
      for (std::size_t i = 2; i < polygon.size(); ++i) {
        auto p = polygon[0], q = polygon[i - 1], r = polygon[i];
        p[2] = height(planes[plane], p);
        q[2] = height(planes[plane], q);
        r[2] = height(planes[plane], r);
        m.tri(p, q, r, roof, BuildingMaterial::Roof);
      }
    }
  };
  // Start with the exact footprint triangulation (including holes), then split
  // at every roof-plane crease. No bounding-box triangle spans a courtyard.
  flat_roof(m, building, 0, roof, emit);
  for (const auto *rings : {&building.outers, &building.inners})
    for (const auto &ring : *rings) {
      const bool reverse = (area(ring) < 0) != (rings == &building.inners);
      for (std::size_t i = 0; i < ring.size(); ++i) {
        auto a = ring[i], b = ring[(i + 1) % ring.size()];
        if (reverse)
          std::swap(a, b);
        const V first{a.e, a.n, eave}, last{b.e, b.n, eave};
        std::vector<double> cuts{0, 1};
        for (std::size_t j = 0; j < planes.size(); ++j)
          for (std::size_t k = j + 1; k < planes.size(); ++k) {
            const double p = height(planes[j], first) -
                             height(planes[k], first),
                         q = height(planes[j], last) - height(planes[k], last);
            if ((p > 0) != (q > 0)) {
              const double t = p / (p - q);
              if (t > 0 && t < 1)
                cuts.push_back(t);
            }
          }
        std::sort(cuts.begin(), cuts.end());
        cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
        for (std::size_t j = 1; j < cuts.size(); ++j) {
          const auto at = [&](double t) {
            return V{a.e + (b.e - a.e) * t, a.n + (b.n - a.n) * t, eave};
          };
          auto p = at(cuts[j - 1]), q = at(cuts[j]), r = q, s = p;
          r[2] = minimum(r);
          s[2] = minimum(s);
          m.quad(p, q, r, s, wall, BuildingMaterial::Wall);
        }
      }
    }
}

} // namespace

BuildingMesh build_building_mesh(const Building &b,
                                 const BuildingMeshOptions &options) {
  Builder m{{}, options};
  if (!std::isfinite(b.height) || !std::isfinite(b.base) ||
      !std::isfinite(b.bottom) || b.height <= 0 || b.height > 2000 ||
      b.bottom >= b.base + b.height)
    return {};
  if (!std::isfinite(options.origin_e) || !std::isfinite(options.origin_n))
    return {};
  for (const auto *rings : {&b.outers, &b.inners})
    for (const auto &r : *rings) {
      if (r.size() < 3)
        return {};
      for (auto p : r)
        if (!std::isfinite(p.e) || !std::isfinite(p.n))
          return {};
    }
  if (b.outers.empty())
    return {};
  m.uv_origin = {b.outers.front().front().e, b.outers.front().front().n,
                 b.base};
  const auto seed = mix(static_cast<std::uint64_t>(b.id) ^ options.seed ^
                        (b.relation ? 0xa0761d6478bd642fULL : 0));
  constexpr C palette[] = {{.77f, .73f, .65f},
                           {.68f, .64f, .55f},
                           {.84f, .80f, .70f},
                           {.63f, .45f, .34f},
                           {.69f, .70f, .69f}};
  const C wall = colour(b.colour, palette[seed % 5]);
  const C roof = colour(b.roof_colour, seed % 3 == 0 ? C{.24f, .25f, .27f}
                                                     : C{.43f, .21f, .14f});
  const double top = b.base + b.height;
  Box box;
  const bool rectangular = rectangle(b, box);
  const bool valid_box = rectangular || rectangle(b, box, false);
  RoofForm form = RoofForm::Flat;
  if (b.roof_shape == "gabled")
    form = RoofForm::Gabled;
  else if (b.roof_shape == "hipped")
    form = RoofForm::Hipped;
  else if (b.roof_shape == "skillion")
    form = RoofForm::Skillion;
  else if (b.roof_shape == "pyramidal")
    form = RoofForm::Pyramidal;
  else if (!b.roof_shape.empty() && b.roof_shape != "flat")
    m.mesh.unsupported_roof = true;
  else if (b.roof_shape.empty() && options.infer_residential_roofs &&
           b.height <= 18 &&
           (b.type == "house" || b.type == "detached" ||
            b.type == "semidetached_house" || b.type == "terrace" ||
            b.type == "residential")) {
    form = seed % 4 == 0 ? RoofForm::Hipped : RoofForm::Gabled;
    m.mesh.inferred_roof = true;
  }
  const auto direction = roof_direction_degrees(b.roof_direction);
  const bool has_direction = !b.roof_direction.empty();
  // A mapped direction works for both box roofs and the clipped polygon path:
  // align the local slope axis to the compass direction, then project the
  // actual footprint to recover the envelope used by each roof form.
  if (form != RoofForm::Flat && (!valid_box || (has_direction && !direction))) {
    form = RoofForm::Flat;
    m.mesh.unsupported_roof = true;
  }
  if (form != RoofForm::Flat && has_direction && direction) {
    if (!align_roof_direction(b, box, *direction, form)) {
      form = RoofForm::Flat;
      m.mesh.unsupported_roof = true;
    }
  } else if (b.roof_orientation == "across" && valid_box) {
    std::swap(box.hl, box.hw);
    const auto u = box.ux;
    box.ux = -box.uy;
    box.uy = u;
  }
  const bool mapped_direction =
      form != RoofForm::Flat && has_direction && direction.has_value();
  double rise = 0;
  if (form != RoofForm::Flat) {
    rise = b.roof_height > 0 ? b.roof_height : std::min(box.hw * .6, 3.5);
    rise = std::min(
        rise,
        std::max(0.0, (top - std::max(b.base + b.min_height, b.bottom)) * .65));
    if (rise < .05)
      form = RoofForm::Flat;
  }
  if (form == RoofForm::Flat)
    rise = 0;
  const double eave = top - rise;
  const double storey = std::isfinite(options.storey_height_m)
                            ? std::clamp(options.storey_height_m, 2.0, 6.0)
                            : 3.0;
  const double bay = std::isfinite(options.bay_width_m)
                         ? std::clamp(options.bay_width_m, 1.5, 8.0)
                         : 2.8;
  const int floors = std::clamp(
      static_cast<int>(
          b.levels > 0 ? b.levels
                       : std::floor((eave - b.base - b.min_height) / storey)),
      1, 64);
  const double floor_h = (eave - b.base - b.min_height) / floors;
  for (const auto *rings : {&b.outers, &b.inners})
    for (const auto &r : *rings) {
      const bool inner = rings == &b.inners;
      const bool reverse = (area(r) < 0) != inner;
      for (std::size_t i = 0; i < r.size(); ++i) {
        auto a = r[i], c = r[(i + 1) % r.size()];
        if (reverse)
          std::swap(a, c);
        const double dx = c.e - a.e, dy = c.n - a.n,
                     len = std::sqrt(dx * dx + dy * dy);
        if (len < .001)
          continue;
        m.quad({a.e, a.n, b.bottom}, {c.e, c.n, b.bottom}, {c.e, c.n, eave},
               {a.e, a.n, eave}, wall, BuildingMaterial::Wall);
        if (!options.facade_detail || floor_h < 2 || b.type == "garage" ||
            b.type == "garages" || b.type == "shed")
          continue;
        const int bays =
            std::clamp(static_cast<int>(std::floor(len / bay)), 1, 64);
        const double width = len / bays;
        const double nx = dy / len, ny = -dx / len;
        for (int f = 0; f < floors; ++f)
          for (int k = 0; k < bays; ++k) {
            if (m.mesh.windows >= std::min(options.max_windows, 4096U))
              break;
            const double s0 = width * (k + .25), s1 = width * (k + .75),
                         z0 = b.base + b.min_height + floor_h * (f + .28),
                         z1 = b.base + b.min_height + floor_h * (f + .76);
            const auto at = [&](double s, double z, double off) -> V {
              return {a.e + dx / len * s + nx * off,
                      a.n + dy / len * s + ny * off, z};
            };
            if (!inner && i == 0 && f == 0 && k == 0 &&
                (b.type == "house" || b.type == "detached" ||
                 b.type == "semidetached_house" || b.type == "terrace" ||
                 b.type == "residential")) {
              const double center = (s0 + s1) * .5,
                           half = std::min(.525, (s1 - s0) * .5),
                           floor = b.base + b.min_height;
              const double door_h = std::min(2.15, floor_h * .85);
              const auto vec = [&](double width) {
                return V{dx / len * width, dy / len * width, 0};
              };
              m.box(at(center - half - .07, floor, .015), vec(2 * half + .14),
                    {nx * .075, ny * .075, 0}, {0, 0, door_h + .08},
                    {.34f, .32f, .28f}, BuildingMaterial::Frame);
              m.box(at(center - half, floor + .025, .08), vec(2 * half),
                    {nx * .035, ny * .035, 0}, {0, 0, door_h - .025},
                    {.30f, .23f, .16f}, BuildingMaterial::Frame);
              m.box(at(center + half * .65, floor + 1.0, .115), vec(.09),
                    {nx * .05, ny * .05, 0}, {0, 0, .035}, {.65f, .65f, .63f},
                    BuildingMaterial::Frame);
              ++m.mesh.doors;
              continue;
            }
            m.quad(at(s0 - .06, z0 - .06, .012), at(s1 + .06, z0 - .06, .012),
                   at(s1 + .06, z1 + .06, .012), at(s0 - .06, z1 + .06, .012),
                   {.27f, .26f, .24f}, BuildingMaterial::Frame);
            const auto variation =
                mix(seed ^ static_cast<std::uint64_t>(
                               i * 4096 + static_cast<std::size_t>(f) * 64 +
                               static_cast<std::size_t>(k)));
            const C glass =
                variation % 7 == 0 ? C{.30f, .33f, .35f} : C{.12f, .20f, .25f};
            m.quad(at(s0, z0, .018), at(s1, z0, .018), at(s1, z1, .018),
                   at(s0, z1, .018), glass, BuildingMaterial::Glass);
            if (f == 0) {
              const C sill{.65f, .64f, .60f};
              m.quad(at(s0 - .1, z0 - .08, .02), at(s1 + .1, z0 - .08, .02),
                     at(s1 + .1, z0 - .08, .22), at(s0 - .1, z0 - .08, .22),
                     sill, BuildingMaterial::Frame);
              m.quad(at(s0 - .1, z0 - .18, .22), at(s1 + .1, z0 - .18, .22),
                     at(s1 + .1, z0 - .08, .22), at(s0 - .1, z0 - .08, .22),
                     sill, BuildingMaterial::Frame);
              m.quad(at(s0 - .1, z0 - .18, .02), at(s0 - .1, z0 - .18, .22),
                     at(s0 - .1, z0 - .08, .22), at(s0 - .1, z0 - .08, .02),
                     sill, BuildingMaterial::Frame);
              m.quad(at(s1 + .1, z0 - .18, .22), at(s1 + .1, z0 - .18, .02),
                     at(s1 + .1, z0 - .08, .02), at(s1 + .1, z0 - .08, .22),
                     sill, BuildingMaterial::Frame);
            }
            ++m.mesh.windows;
          }
      }
    }
  m.mesh.roof = form;
  if (form == RoofForm::Flat) {
    flat_roof(m, b, top, roof);
    for (auto &v : m.mesh.vertices) {
      if (v.material == BuildingMaterial::Roof) {
        v.material = BuildingMaterial::FlatRoof;
        if (b.roof_colour.empty())
          v.colour = {1, 1, 1};
      }
      if (v.material == BuildingMaterial::Wall && b.material == "brick")
        v.material = BuildingMaterial::Brick;
    }
    return std::move(m.mesh);
  }
  const auto add_chimney = [&]() {
    if (!options.facade_detail || seed % 3 == 0 ||
        (b.type != "house" && b.type != "detached" &&
         b.type != "semidetached_house" && b.type != "terrace" &&
         b.type != "residential"))
      return;
    const double u = -box.hl * .25, v = box.hw * .65;
    const V center{box.e + box.ux * u - box.uy * v,
                   box.n + box.uy * u + box.ux * v, 0};
    const auto inside = [](const auto &ring, double x, double y) {
      bool value = false;
      for (std::size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const auto a = ring[i], c = ring[j];
        if ((a.n > y) != (c.n > y) &&
            x < (c.e - a.e) * (y - a.n) / (c.n - a.n) + a.e)
          value = !value;
      }
      return value;
    };
    bool support = false;
    for (const auto &ring : b.outers)
      support = support || inside(ring, center[0], center[1]);
    for (const auto &ring : b.inners)
      if (inside(ring, center[0], center[1]))
        support = false;
    if (!support)
      return;
    double z = top - rise * std::abs(v) / box.hw;
    if (form == RoofForm::Hipped) {
      if (mapped_direction && box.hl < box.hw)
        z = std::min(z, top - rise * std::abs(u) / box.hl);
      else
        z = std::min(z, eave + rise * (box.hl - std::abs(u)) / box.hw);
    }
    if (form == RoofForm::Pyramidal)
      z = std::min(z, top - rise * std::abs(u) / box.hl);
    if (form == RoofForm::Skillion)
      z = eave + rise * (v + box.hw) / (2 * box.hw);
    const double upper = std::min(top - .05, z + .95),
                 half = std::min(.25, box.hw * .2);
    if (upper - z < .25)
      return;
    const V base{center[0] - box.ux * half + box.uy * half,
                 center[1] - box.uy * half - box.ux * half, z - .15};
    m.box(base, {box.ux * half * 2, box.uy * half * 2, 0},
          {-box.uy * half * 2, box.ux * half * 2, 0}, {0, 0, upper - z + .15},
          {1, 1, 1}, BuildingMaterial::Brick);
    m.box({base[0], base[1], upper}, {box.ux * half * 2, box.uy * half * 2, 0},
          {-box.uy * half * 2, box.ux * half * 2, 0}, {0, 0, .04},
          {.55f, .54f, .52f}, BuildingMaterial::Frame);
    ++m.mesh.chimneys;
  };
  add_chimney();
  if (!rectangular || mapped_direction) {
    clipped_roof(m, b, box, form, eave, rise, roof, wall, mapped_direction);
    for (auto &v : m.mesh.vertices) {
      if (v.material == BuildingMaterial::Roof) {
        if (b.roof_material == "slate" || b.roof_colour == "grey" ||
            b.roof_colour == "gray" || b.roof_colour == "black")
          v.material = BuildingMaterial::Slate;
        if (b.roof_colour.empty())
          v.colour = {1, 1, 1};
      }
      if (v.material == BuildingMaterial::Wall && b.material == "brick")
        v.material = BuildingMaterial::Brick;
    }
    return std::move(m.mesh);
  }
  const double overhang =
      std::isfinite(options.eave_m) ? std::clamp(options.eave_m, 0.0, .5) : .2;
  const double hl = box.hl + overhang, hw = box.hw + overhang;
  const double ze = eave - overhang * rise / box.hw;
  const auto at = [&](double u, double v, double z) -> V {
    return {box.e + box.ux * u - box.uy * v, box.n + box.uy * u + box.ux * v,
            z};
  };
  if (form == RoofForm::Skillion) {
    m.quad(at(-hl, -hw, ze), at(hl, -hw, ze), at(hl, hw, top), at(-hl, hw, top),
           roof, BuildingMaterial::Roof);
    m.tri(at(-box.hl, box.hw, eave), at(-box.hl, -box.hw, eave),
          at(-box.hl, box.hw, top), wall, BuildingMaterial::Wall);
    m.tri(at(box.hl, -box.hw, eave), at(box.hl, box.hw, eave),
          at(box.hl, box.hw, top), wall, BuildingMaterial::Wall);
    m.quad(at(box.hl, box.hw, eave), at(-box.hl, box.hw, eave),
           at(-box.hl, box.hw, top), at(box.hl, box.hw, top), wall,
           BuildingMaterial::Wall);
  } else {
    const double ridge =
        form == RoofForm::Gabled
            ? hl
            : (form == RoofForm::Pyramidal ? 0 : std::max(hl - hw, 0.0));
    m.quad(at(-hl, -hw, ze), at(hl, -hw, ze), at(ridge, 0, top),
           at(-ridge, 0, top), roof, BuildingMaterial::Roof);
    m.quad(at(hl, hw, ze), at(-hl, hw, ze), at(-ridge, 0, top),
           at(ridge, 0, top), roof, BuildingMaterial::Roof);
    if (form == RoofForm::Gabled) {
      m.tri(at(-box.hl, box.hw, eave), at(-box.hl, -box.hw, eave),
            at(-box.hl, 0, top), wall, BuildingMaterial::Wall);
      m.tri(at(box.hl, -box.hw, eave), at(box.hl, box.hw, eave),
            at(box.hl, 0, top), wall, BuildingMaterial::Wall);
    } else {
      m.tri(at(hl, -hw, ze), at(hl, hw, ze), at(ridge, 0, top), roof,
            BuildingMaterial::Roof);
      m.tri(at(-hl, hw, ze), at(-hl, -hw, ze), at(-ridge, 0, top), roof,
            BuildingMaterial::Roof);
    }
  }
  for (auto &v : m.mesh.vertices) {
    if (v.material == BuildingMaterial::Roof) {
      if (b.roof_material == "slate" || b.roof_colour == "grey" ||
          b.roof_colour == "gray" || b.roof_colour == "black")
        v.material = BuildingMaterial::Slate;
      else if (b.roof_material == "metal" || b.roof_material == "metal_sheet" ||
               b.roof_material == "steel")
        v.material = BuildingMaterial::Metal;
      if (b.roof_colour.empty())
        v.colour = {1, 1, 1};
    }
    if (v.material == BuildingMaterial::Wall && b.material == "brick")
      v.material = BuildingMaterial::Brick;
  }
  return std::move(m.mesh);
}
} // namespace g2m::world
