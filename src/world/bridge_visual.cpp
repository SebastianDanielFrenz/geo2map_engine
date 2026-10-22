// Procedural bridge family reference: BeamNG.Earth bridgetypes.py (606ba0f),
// MIT, Copyright (c) 2026 TecnaGamer; THIRD_PARTY_NOTICES.md.
#include "g2m/world/bridge_visual.h"
#include "g2m/world/road_structures.h"
#include <algorithm>
#include <array>
#include <cmath>

namespace g2m::world {
namespace {
using V = std::array<double, 3>;
V add(V a, V b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V mul(V a, double k) { return {a[0] * k, a[1] * k, a[2] * k}; }
V sub(V a, V b) { return add(a, mul(b, -1)); }
V cross(V a, V b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2],
          a[0] * b[1] - a[1] * b[0]};
}
double length(V a) {
  return std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}
struct Builder {
  mesh::TerrainChunkMesh mesh;
  bool failed = false;
  void beam(V a, V b, double width) {
    const auto d = sub(b, a);
    const double len = length(d);
    if (len < .01)
      return;
    // Bound visual geometry independently of the physical deck's 16-bit mesh.
    if (mesh.positions.size() / 3 + 36 > 40000) {
      failed = true;
      return;
    }
    const auto axis = mul(d, 1 / len);
    auto side = cross(axis, std::abs(axis[2]) > .9 ? V{1, 0, 0} : V{0, 0, 1});
    side = mul(side, width * .5 / length(side));
    const auto up = mul(cross(axis, side), 1.0);
    std::array<V, 8> p;
    for (int i = 0; i < 8; ++i)
      p[static_cast<std::size_t>(i)] =
          add(i < 4 ? a : b,
              add(mul(side, (i & 1) ? 1 : -1), mul(up, (i & 2) ? 1 : -1)));
    constexpr int faces[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                                 {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
    for (const auto &f : faces) {
      auto n = cross(sub(p[static_cast<std::size_t>(f[1])],
                         p[static_cast<std::size_t>(f[0])]),
                     sub(p[static_cast<std::size_t>(f[2])],
                         p[static_cast<std::size_t>(f[0])]));
      n = mul(n, 1 / length(n));
      for (int i : {f[0], f[1], f[2], f[0], f[2], f[3]}) {
        const auto v = p[static_cast<std::size_t>(i)];
        mesh.indices.push_back(
            static_cast<std::uint16_t>(mesh.positions.size() / 3));
        for (auto c : v)
          mesh.positions.push_back(static_cast<float>(c));
        for (auto c : n)
          mesh.normals.push_back(static_cast<float>(c));
        mesh.land_class.push_back(
            static_cast<std::uint8_t>(LandClass::BuiltUp));
      }
    }
  }
};
} // namespace
BridgeFacts bridge_facts(const RoadGraphWay &way) {
  BridgeFacts facts;
  facts.source_id = "osm:way/" + std::to_string(way.osm_id);
  for (const auto &[k, v] : way.tags) {
    if (k == "wikidata")
      facts.wikidata_id = v;
    else if (k == "ref:nbi")
      facts.inventory_id = v;
    else if (k == "bridge:structure")
      facts.structure_tag = v;
  }
  const auto &s = facts.structure_tag;
  if (s == "beam" || s == "slab")
    facts.form = BridgeForm::Beam;
  else if (s == "truss")
    facts.form = BridgeForm::Truss;
  else if (s == "arch")
    facts.form = BridgeForm::Arch;
  else if (s == "suspension")
    facts.form = BridgeForm::Suspension;
  else if (s == "cable-stayed")
    facts.form = BridgeForm::CableStayed;
  else if (s == "movable")
    facts.form = BridgeForm::Movable;
  for (const auto &[k, v] : way.tags)
    if (k == "bridge:movable" && v != "no" && !v.empty())
      facts.form = BridgeForm::Movable;
  facts.explicit_form = facts.form != BridgeForm::Generic;
  return facts;
}
BridgeVisual build_bridge_visual(const RoadDeck &deck,
                                 const BridgeFacts &facts) {
  BridgeVisual out;
  out.facts = facts;
  if (deck.tunnel_floor || deck.inferred)
    return out;
  const double span = deck.end_station - deck.start_station;
  if (!std::isfinite(span) || span < .5 || span > 1000) {
    out.unsupported = true;
    return out;
  }
  const auto rows = static_cast<std::size_t>(std::ceil(span)) + 1;
  if (deck.mesh.positions.size() < rows * 15) {
    out.unsupported = true;
    return out;
  }
  for (std::size_t i = 0; i < rows * 15; ++i)
    if (!std::isfinite(deck.mesh.positions[i])) {
      out.unsupported = true;
      return out;
    }
  Builder b;
  std::copy(std::begin(deck.mesh.origin), std::end(deck.mesh.origin),
            std::begin(b.mesh.origin));
  const auto point = [&](double t, int side, double z) -> V {
    const double station =
        std::clamp(t, 0.0, 1.0) * static_cast<double>(rows - 1);
    const auto i = static_cast<std::size_t>(std::floor(station));
    const auto j = std::min(i + 1, rows - 1);
    const double f = station - static_cast<double>(i);
    const auto read = [&](std::size_t row, int col) -> V {
      const auto k = (row * 5 + static_cast<std::size_t>(col)) * 3;
      return {deck.mesh.positions[k], deck.mesh.positions[k + 1],
              deck.mesh.positions[k + 2]};
    };
    const int col = side < 0 ? 0 : 4;
    auto p = add(mul(read(i, col), 1 - f), mul(read(j, col), f));
    const auto c = add(mul(read(i, 2), 1 - f), mul(read(j, 2), f));
    auto outward = sub(p, c);
    outward[2] = 0;
    const auto w = length(outward);
    if (w > .01)
      p = add(p, mul(outward, .22 / w));
    p[2] += z;
    return p;
  };
  const int panels = std::clamp(static_cast<int>(std::ceil(span / 4)), 1, 80);
  for (int side : {-1, 1}) {
    for (int i = 0; i < panels; ++i) {
      const double t = static_cast<double>(i) / panels,
                   u = static_cast<double>(i + 1) / panels;
      b.beam(point(t, side, 1.1), point(u, side, 1.1), .12);
      b.beam(point(t, side, .55), point(u, side, .55), .08);
      b.beam(point(t, side, 0), point(t, side, 1.1), .10);
    }
    b.beam(point(1, side, 0), point(1, side, 1.1), .10);
  }
  const auto form = facts.explicit_form ? facts.form : BridgeForm::Generic;
  const double h = std::clamp(span * .12, 3.0, 18.0);
  if (form == BridgeForm::Truss) {
    const int count = std::clamp(static_cast<int>(std::ceil(span / 8)), 2, 60);
    for (int side : {-1, 1})
      for (int i = 0; i < count; ++i) {
        const double t = static_cast<double>(i) / count,
                     u = static_cast<double>(i + 1) / count;
        b.beam(point(t, side, h), point(u, side, h), .24);
        b.beam(point(t, side, .2), point(u, side, .2), .24);
        b.beam(point(t, side, i % 2 == 0 ? .2 : h),
               point(u, side, i % 2 == 0 ? h : .2), .20);
      }
  } else if (form == BridgeForm::Arch) {
    for (int side : {-1, 1})
      for (int i = 0; i < panels; ++i) {
        const double t = static_cast<double>(i) / panels,
                     u = static_cast<double>(i + 1) / panels;
        const double z = .3 + 4 * h * t * (1 - t),
                     zz = .3 + 4 * h * u * (1 - u);
        b.beam(point(t, side, z), point(u, side, zz), .3);
        if (i > 0)
          b.beam(point(t, side, .2), point(t, side, z), .10);
      }
  } else if (form == BridgeForm::Suspension ||
             form == BridgeForm::CableStayed) {
    const double tower_h = std::clamp(span * .18, 8.0, 60.0);
    for (int side : {-1, 1}) {
      for (double t : {.2, .8})
        b.beam(point(t, side, -.5), point(t, side, tower_h), .65);
      const auto cable_z = [&](double t) {
        if (t < .2)
          return 1.0 + (tower_h - 1.0) * t / .2;
        if (t > .8)
          return 1.0 + (tower_h - 1.0) * (1 - t) / .2;
        const double x = (t - .5) / .3;
        return tower_h * .3 + tower_h * .7 * x * x;
      };
      for (int i = 0; i < panels; ++i) {
        const double t = static_cast<double>(i) / panels,
                     u = static_cast<double>(i + 1) / panels;
        if (form == BridgeForm::Suspension) {
          b.beam(point(t, side, cable_z(t)), point(u, side, cable_z(u)), .14);
          if (t > .2 && t < .8)
            b.beam(point(t, side, .3), point(t, side, cable_z(t)), .055);
        } else if (i % 2 == 0)
          b.beam(point(t < .5 ? .2 : .8, side, tower_h), point(t, side, .3),
                 .065);
      }
    }
  } else if (form == BridgeForm::Movable)
    out.unsupported = true;
  if (b.failed) {
    out.unsupported = true;
    return out;
  }
  out.mesh = std::move(b.mesh);
  return out;
}
} // namespace g2m::world
