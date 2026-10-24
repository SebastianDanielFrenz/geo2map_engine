// Procedural bridge family reference: BeamNG.Earth bridgetypes.py (606ba0f),
// MIT, Copyright (c) 2026 TecnaGamer; THIRD_PARTY_NOTICES.md.
#include "g2m/world/bridge_visual.h"
#include "g2m/world/road_structures.h"
#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

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
  void box(V a, V b, double side_width, double up_width) {
    const auto d = sub(b, a);
    const double len = length(d);
    if (len < .01)
      return;
    // Bound visual geometry independently of the physical deck's 16-bit mesh.
    if (mesh.positions.size() / 3 + 24 > 40000) {
      failed = true;
      return;
    }
    const auto axis = mul(d, 1 / len);
    auto side = cross(V{0, 0, 1}, axis);
    if (length(side) < .01)
      side = cross(V{1, 0, 0}, axis);
    side = mul(side, side_width * .5 / length(side));
    const auto perpendicular = cross(axis, side);
    const auto up = mul(perpendicular, up_width * .5 / length(perpendicular));
    std::array<V, 8> p;
    for (int i = 0; i < 8; ++i) {
      const auto at = i < 4 ? a : b;
      p[static_cast<std::size_t>(i)] =
          add(at, add(mul(side, (i & 1) ? 1 : -1), mul(up, (i & 2) ? 1 : -1)));
    }
    constexpr int faces[6][4] = {{0, 2, 3, 1}, {4, 5, 7, 6}, {0, 1, 5, 4},
                                 {2, 6, 7, 3}, {0, 4, 6, 2}, {1, 3, 7, 5}};
    for (const auto &f : faces) {
      auto n = cross(sub(p[static_cast<std::size_t>(f[1])],
                         p[static_cast<std::size_t>(f[0])]),
                     sub(p[static_cast<std::size_t>(f[2])],
                         p[static_cast<std::size_t>(f[0])]));
      n = mul(n, 1 / length(n));
      const auto first = static_cast<std::uint16_t>(mesh.positions.size() / 3);
      for (int i : {f[0], f[1], f[2], f[3]}) {
        const auto v = p[static_cast<std::size_t>(i)];
        for (auto c : v)
          mesh.positions.push_back(static_cast<float>(c));
        for (auto c : n)
          mesh.normals.push_back(static_cast<float>(c));
        mesh.land_class.push_back(
            static_cast<std::uint8_t>(LandClass::BuiltUp));
      }
      for (auto i : {0, 1, 2, 0, 2, 3})
        mesh.indices.push_back(static_cast<std::uint16_t>(first + i));
    }
  }
  void beam(V a, V b, double width) { box(a, b, width, width); }
  void wall(V a, V b, double thickness, double height) {
    box(a, b, thickness, height);
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
    else if (k == "highway")
      facts.highway_tag = v;
    else if (k == "railway")
      facts.railway_tag = v;
    else if (k == "bridge")
      facts.bridge_tag = v;
    else if (k == "bridge:material")
      facts.bridge_material_tag = v;
    else if (k == "barrier")
      facts.barrier_tag = v;
    else if (k == "fence_type" && v == "railing" && facts.barrier_tag.empty())
      facts.barrier_tag = "railing";
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

  // OSM describes bridge function and load-bearing structure, but does not
  // imply a particular parapet from `bridge:structure`. Choose a conservative
  // shape from the carrying way's use; a mapped barrier on that same way is a
  // more direct statement and takes precedence.
  const bool railway_carrying_way =
      facts.railway_tag == "rail" || facts.railway_tag == "tram" ||
      facts.railway_tag == "light_rail" ||
      facts.railway_tag == "narrow_gauge" || facts.railway_tag == "subway" ||
      facts.railway_tag == "monorail" || facts.railway_tag == "funicular";
  if (railway_carrying_way)
    facts.parapet_style = BridgeParapetStyle::RailwayParapet;
  else if (facts.highway_tag == "footway" || facts.highway_tag == "cycleway" ||
           facts.highway_tag == "path" || facts.highway_tag == "steps" ||
           facts.highway_tag == "pedestrian" ||
           facts.highway_tag == "bridleway" || facts.bridge_tag == "boardwalk")
    facts.parapet_style = BridgeParapetStyle::PedestrianRailing;
  else if (facts.highway_tag == "motorway" || facts.highway_tag == "trunk")
    facts.parapet_style = BridgeParapetStyle::ConcreteParapet;
  else
    facts.parapet_style = BridgeParapetStyle::RoadGuardrail;
  if (facts.barrier_tag == "jersey_barrier") {
    facts.parapet_style = BridgeParapetStyle::ConcreteParapet;
    facts.parapet_style_inferred = false;
  } else if (facts.barrier_tag == "guard_rail") {
    facts.parapet_style = BridgeParapetStyle::RoadGuardrail;
    facts.parapet_style_inferred = false;
  } else if (facts.barrier_tag == "railing" ||
             facts.barrier_tag == "handrail") {
    facts.parapet_style = BridgeParapetStyle::PedestrianRailing;
    facts.parapet_style_inferred = false;
  }
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
  double edge_offset = .22;
  if (facts.parapet_style == BridgeParapetStyle::ConcreteParapet)
    edge_offset = .27;
  else if (facts.parapet_style == BridgeParapetStyle::RailwayParapet)
    edge_offset = .25;
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
      p = add(p, mul(outward, edge_offset / w));
    p[2] += z;
    return p;
  };
  // Subdivide where sampled deck edges depart from their chord by >5 cm.
  constexpr double max_chord_error_m = .05;
  constexpr std::size_t max_rail_panels = 512;
  const int panels = std::clamp(static_cast<int>(std::ceil(span / 4)), 1, 80);
  const auto form = facts.explicit_form ? facts.form : BridgeForm::Generic;
  std::vector<std::array<double, 2>> pending, accepted;
  for (int i = 0; i < panels; ++i)
    pending.push_back(
        {static_cast<double>(i) / panels, static_cast<double>(i + 1) / panels});
  const auto chord_error = [&](double t0, double t1) {
    const double s0 = t0 * static_cast<double>(rows - 1);
    const double s1 = t1 * static_cast<double>(rows - 1);
    double error = 0;
    for (std::size_t row = static_cast<std::size_t>(std::floor(s0)) + 1;
         row < static_cast<std::size_t>(std::ceil(s1)) && row < rows; ++row) {
      const double f = (static_cast<double>(row) - s0) / (s1 - s0);
      for (int side : {-1, 1}) {
        const auto actual = point(
            static_cast<double>(row) / static_cast<double>(rows - 1), side, 0);
        const auto chord =
            add(mul(point(t0, side, 0), 1 - f), mul(point(t1, side, 0), f));
        const auto delta = sub(actual, chord);
        error = std::max(error, length(delta));
      }
    }
    return error;
  };
  while (!pending.empty()) {
    const auto interval = pending.back();
    pending.pop_back();
    const auto [t0, t1] = interval;
    if (chord_error(t0, t1) <= max_chord_error_m) {
      accepted.push_back(interval);
      if (accepted.size() + pending.size() > max_rail_panels) {
        out.unsupported = true;
        return out;
      }
    } else {
      const double mid = (t0 + t1) * .5;
      if (mid <= t0 || mid >= t1 ||
          accepted.size() + pending.size() + 2 > max_rail_panels) {
        out.unsupported = true;
        return out;
      }
      pending.push_back({mid, t1});
      pending.push_back({t0, mid});
    }
  }
  std::sort(accepted.begin(), accepted.end(),
            [](const auto &a, const auto &b) { return a[0] < b[0]; });
  std::vector<double> rail_stations{0};
  for (const auto &interval : accepted)
    rail_stations.push_back(interval[1]);
  const bool pedestrian =
      facts.parapet_style == BridgeParapetStyle::PedestrianRailing;
  const auto pedestrian_post_count =
      static_cast<std::size_t>(std::ceil(span / .15)) + 1;
  const auto parapet_beam_count =
      2 * pedestrian_post_count + 4 * (rail_stations.size() - 1);
  if (pedestrian && parapet_beam_count > 40000 / 24) {
    out.unsupported = true;
    return out;
  }
  for (int side : {-1, 1}) {
    if (facts.parapet_style == BridgeParapetStyle::ConcreteParapet ||
        facts.parapet_style == BridgeParapetStyle::RailwayParapet) {
      const double height =
          facts.parapet_style == BridgeParapetStyle::RailwayParapet ? 1.45
                                                                    : 1.0;
      const double thickness =
          facts.parapet_style == BridgeParapetStyle::RailwayParapet ? .3 : .34;
      for (std::size_t i = 0; i + 1 < rail_stations.size(); ++i) {
        const double t = rail_stations[i], u = rail_stations[i + 1];
        b.wall(point(t, side, height * .5), point(u, side, height * .5),
               thickness, height);
      }
    } else {
      const double top = pedestrian ? 1.2 : .98;
      const double lower = pedestrian ? .58 : .5;
      const double rail_width = pedestrian ? .055 : .12;
      const double post_width = pedestrian ? .065 : .10;
      for (std::size_t i = 0; i + 1 < rail_stations.size(); ++i) {
        const double t = rail_stations[i], u = rail_stations[i + 1];
        b.beam(point(t, side, top), point(u, side, top), rail_width);
        b.beam(point(t, side, lower), point(u, side, lower), rail_width * .8);
      }
      if (pedestrian) {
        for (std::size_t i = 0; i < pedestrian_post_count; ++i) {
          const double t = static_cast<double>(i) /
                           static_cast<double>(pedestrian_post_count - 1);
          b.beam(point(t, side, 0), point(t, side, top), post_width);
        }
      } else {
        for (std::size_t i = 0; i < rail_stations.size(); ++i)
          b.beam(point(rail_stations[i], side, 0),
                 point(rail_stations[i], side, top), post_width);
      }
    }
  }
  const bool has_structural_form = form == BridgeForm::Truss ||
                                   form == BridgeForm::Suspension ||
                                   form == BridgeForm::CableStayed;
  const bool curved_structural_form =
      has_structural_form && chord_error(0, 1) > max_chord_error_m;
  const double h = std::clamp(span * .12, 3.0, 18.0);
  if (curved_structural_form) {
    // These straight member layouts can cut through a curved deck. Keep only
    // the safe adaptive parapets until a curve-aware structural layout exists.
    out.unsupported = true;
  } else if (form == BridgeForm::Truss) {
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
    // Generic arch does not establish deck/through/above topology.
    out.unsupported = true;
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
