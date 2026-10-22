#include "g2m/world/bridge_visual.h"
#include "g2m/world/road_structures.h"
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <utility>
namespace {
g2m::world::RoadDeck deck(double length = 40) {
  g2m::world::RoadDeck d;
  d.end_station = length;
  d.mesh.origin[0] = 450000;
  d.mesh.origin[1] = 5500000;
  d.way_id = 123;
  const int count = static_cast<int>(std::ceil(length));
  for (int i = 0; i <= count; ++i)
    for (int j = 0; j < 5; ++j) {
      d.mesh.positions.push_back(static_cast<float>(length * i / count));
      d.mesh.positions.push_back(static_cast<float>(j * 2 - 4));
      d.mesh.positions.push_back(static_cast<float>(100 + .01 * i));
    }
  return d;
}
} // namespace
TEST_CASE("bridge families preserve source IDs and physical deck",
          "[world][bridge_visual]") {
  auto d = deck();
  const auto original = d.mesh.positions;
  for (const auto *family : {"", "beam", "truss", "arch", "suspension",
                             "cable-stayed", "movable", "unexpected"}) {
    g2m::RoadGraphWay way;
    way.osm_id = 123;
    way.tags = {{"bridge:structure", family},
                {"wikidata", "Q12345"},
                {"ref:nbi", "NBI-001"}};
    const auto facts = g2m::world::bridge_facts(way);
    CHECK(facts.source_id == "osm:way/123");
    CHECK(facts.wikidata_id == "Q12345");
    CHECK(facts.inventory_id == "NBI-001");
    const auto visual = g2m::world::build_bridge_visual(d, facts);
    INFO(family);
    REQUIRE(!visual.mesh.positions.empty());
    CHECK(visual.mesh.origin[0] == d.mesh.origin[0]);
    CHECK(d.mesh.positions == original);
    if (facts.form == g2m::world::BridgeForm::Arch) {
      CHECK(visual.unsupported);
      CHECK(visual.facts.structure_tag == "arch");
    }
    for (auto i : visual.mesh.indices)
      CHECK(i < visual.mesh.positions.size() / 3);
    for (auto p : visual.mesh.positions)
      CHECK(std::isfinite(p));
    CHECK(visual.mesh.positions.size() == visual.mesh.normals.size());
  }
}
TEST_CASE("inferred and tunnel decks do not get invented bridge bodies",
          "[world][bridge_visual]") {
  auto d = deck();
  d.inferred = true;
  CHECK(g2m::world::build_bridge_visual(d, {}).mesh.positions.empty());
  d.inferred = false;
  d.tunnel_floor = true;
  CHECK(g2m::world::build_bridge_visual(d, {}).mesh.positions.empty());
  d.tunnel_floor = false;
  d.end_station = 1001;
  CHECK(g2m::world::build_bridge_visual(d, {}).unsupported);
}
TEST_CASE("long bridge visuals stay within the mesh index budget",
          "[world][bridge_visual]") {
  auto d = deck(1000);
  g2m::world::BridgeFacts f;
  f.explicit_form = true;
  f.form = g2m::world::BridgeForm::Truss;
  const auto v = g2m::world::build_bridge_visual(d, f);
  CHECK_FALSE(v.unsupported);
  CHECK(v.mesh.positions.size() / 3 <= 40000);
  CHECK(v.mesh.positions.size() / 3 + d.mesh.positions.size() / 3 < 65536);
}

TEST_CASE("adaptive parapets stay outside tight and long curved decks",
          "[world][bridge_visual]") {
  const auto curved = [](double length, double radius) {
    g2m::world::RoadDeck d;
    d.end_station = length;
    const int count = static_cast<int>(std::ceil(length));
    for (int i = 0; i <= count; ++i) {
      const double s = length * i / count, theta = s / radius;
      const double x = radius * std::sin(theta),
                   y = radius * (1 - std::cos(theta));
      const double nx = -std::sin(theta), ny = std::cos(theta);
      for (int j = 0; j < 5; ++j) {
        const double lateral = j * 2.0 - 4.0;
        d.mesh.positions.push_back(static_cast<float>(x + nx * lateral));
        d.mesh.positions.push_back(static_cast<float>(y + ny * lateral));
        d.mesh.positions.push_back(100.0f);
      }
    }
    return d;
  };
  for (const auto [length, radius] :
       {std::pair{40.0, 12.0}, std::pair{1000.0, 110.0}}) {
    auto d = curved(length, radius);
    const auto original = d.mesh.positions;
    const auto v = g2m::world::build_bridge_visual(d, {});
    INFO("span=" << length << " radius=" << radius);
    REQUIRE_FALSE(v.unsupported);
    REQUIRE_FALSE(v.mesh.positions.empty());
    const auto outside_deck = [&](double x, double y) {
      const double radial = std::hypot(x, y - radius);
      return (radial <= radius - 4.0 + .01) || (radial >= radius + 4.0 - .01);
    };
    for (std::size_t i = 0; i < v.mesh.positions.size() / 3; ++i)
      CHECK(outside_deck(v.mesh.positions[i * 3], v.mesh.positions[i * 3 + 1]));
    // Each beam emits 36 triangle vertices; the first six and next six
    // triangles are its two end caps, so their centroids give the beam axis.
    for (std::size_t i = 0; i + 11 < v.mesh.positions.size() / 3; i += 36) {
      double ax = 0, ay = 0, bx = 0, by = 0;
      for (std::size_t j = 0; j < 6; ++j) {
        ax += v.mesh.positions[(i + j) * 3] / 6.0;
        ay += v.mesh.positions[(i + j) * 3 + 1] / 6.0;
        bx += v.mesh.positions[(i + 6 + j) * 3] / 6.0;
        by += v.mesh.positions[(i + 6 + j) * 3 + 1] / 6.0;
      }
      CHECK(outside_deck((ax + bx) * .5, (ay + by) * .5));
    }
    // Sample the actual triangle edges, including rail thickness. Axis-only
    // checks can miss an inside face crossing a curve while its axis is clear.
    for (std::size_t i = 0; i + 2 < v.mesh.indices.size(); i += 3)
      for (int edge = 0; edge < 3; ++edge) {
        const auto a = static_cast<std::size_t>(v.mesh.indices[i + edge]) * 3;
        const auto b =
            static_cast<std::size_t>(v.mesh.indices[i + (edge + 1) % 3]) * 3;
        for (double f : {.25, .5, .75})
          CHECK(outside_deck(
              v.mesh.positions[a] * (1 - f) + v.mesh.positions[b] * f,
              v.mesh.positions[a + 1] * (1 - f) + v.mesh.positions[b + 1] * f));
      }
    CHECK(d.mesh.positions == original);
    g2m::world::BridgeFacts truss;
    truss.explicit_form = true;
    truss.form = g2m::world::BridgeForm::Truss;
    const auto structural = g2m::world::build_bridge_visual(d, truss);
    CHECK(structural.unsupported);
    CHECK(structural.mesh.positions == v.mesh.positions);
  }
}
TEST_CASE("straight long adaptive parapets stay within mesh budget",
          "[world][bridge_visual]") {
  auto d = deck(1000);
  const auto v = g2m::world::build_bridge_visual(d, {});
  CHECK_FALSE(v.unsupported);
  CHECK(v.mesh.positions.size() / 3 <= 40000);
  CHECK(v.mesh.positions.size() / 3 + d.mesh.positions.size() / 3 < 65536);
}

TEST_CASE("adaptive rail budget failure returns no partial visual",
          "[world][bridge_visual]") {
  auto d = deck(1000);
  const int rows = 1001;
  for (int row = 0; row < rows; ++row) {
    const float offset = row % 2 == 0 ? 12.0f : -12.0f;
    for (int col = 0; col < 5; ++col)
      d.mesh.positions[(static_cast<std::size_t>(row) * 5 + col) * 3 + 1] +=
          offset;
  }
  const auto v = g2m::world::build_bridge_visual(d, {});
  CHECK(v.unsupported);
  CHECK(v.mesh.positions.empty());
  CHECK(v.mesh.indices.empty());
}
TEST_CASE("adaptive parapets follow vertical deck curvature",
          "[world][bridge_visual]") {
  auto d = deck(40);
  for (int row = 0; row <= 40; ++row)
    for (int col = 0; col < 5; ++col)
      d.mesh.positions[(static_cast<std::size_t>(row) * 5 + col) * 3 + 2] =
          100.0f + .125f * row * (40 - row);
  const auto v = g2m::world::build_bridge_visual(d, {});
  REQUIRE_FALSE(v.unsupported);
  REQUIRE_FALSE(v.mesh.positions.empty());
  for (std::size_t i = 0; i + 35 < v.mesh.positions.size() / 3; i += 36) {
    double ax = 0, az = 0, bx = 0, bz = 0;
    for (std::size_t j = 0; j < 6; ++j) {
      ax += v.mesh.positions[(i + j) * 3] / 6.0;
      az += v.mesh.positions[(i + j) * 3 + 2] / 6.0;
      bx += v.mesh.positions[(i + 6 + j) * 3] / 6.0;
      bz += v.mesh.positions[(i + 6 + j) * 3 + 2] / 6.0;
    }
    if (std::abs(ax - bx) < .01)
      continue; // vertical posts
    const double x = (ax + bx) * .5;
    const auto row =
        static_cast<std::size_t>(std::clamp(std::floor(x), 0.0, 39.0));
    const double f = x - static_cast<double>(row);
    const double ground = d.mesh.positions[(row * 5 + 2) * 3 + 2] * (1 - f) +
                          d.mesh.positions[((row + 1) * 5 + 2) * 3 + 2] * f;
    const double gap = (az + bz) * .5 - ground;
    CHECK(((gap >= .499 && gap <= .551) || (gap >= 1.049 && gap <= 1.101)));
  }
}
