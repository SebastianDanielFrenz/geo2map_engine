#include "building_test_data.h"
#include "g2m/world/building_mesh.h"
#include <algorithm>
#include <bit>
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
#include <string_view>

namespace {
using namespace g2m::world;
Building house() {
  Building b;
  b.id = 42;
  b.type = "house";
  b.height = 9;
  b.base = 100;
  b.bottom = 98;
  b.outers = {{{450000, 5500000},
               {450012, 5500000},
               {450012, 5500008},
               {450000, 5500008}}};
  return b;
}
BuildingMeshOptions opts() {
  BuildingMeshOptions o;
  o.origin_e = 450000;
  o.origin_n = 5500000;
  return o;
}
double roof_area(const BuildingMesh &m) {
  double area = 0;
  for (std::size_t i = 0; i < m.vertices.size(); i += 3) {
    if (m.vertices[i].material != BuildingMaterial::Roof &&
        m.vertices[i].material != BuildingMaterial::FlatRoof &&
        m.vertices[i].material != BuildingMaterial::Slate &&
        m.vertices[i].material != BuildingMaterial::Metal)
      continue;
    const auto a = m.vertices[i].position, b = m.vertices[i + 1].position,
               c = m.vertices[i + 2].position;
    area += std::abs((b[0] - a[0]) * (c[1] - a[1]) -
                     (c[0] - a[0]) * (b[1] - a[1])) *
            .5;
  }
  return area;
}
std::uint64_t hash(const BuildingMesh &m) {
  std::uint64_t h = 14695981039346656037ULL;
  for (const auto &v : m.vertices) {
    for (const auto &field : {v.position, v.normal, v.colour})
      for (float x : field) {
        h ^= std::bit_cast<std::uint32_t>(x);
        h *= 1099511628211ULL;
      }
    h ^= static_cast<unsigned>(v.material);
    h *= 1099511628211ULL;
  }
  return h;
}
auto roof_triangles(const BuildingMesh &m) {
  using Point = std::array<float, 3>;
  using Triangle = std::array<Point, 3>;
  std::vector<Triangle> result;
  for (std::size_t i = 0; i + 2 < m.vertices.size(); i += 3) {
    const auto material = m.vertices[i].material;
    if (material != BuildingMaterial::Roof &&
        material != BuildingMaterial::FlatRoof &&
        material != BuildingMaterial::Slate &&
        material != BuildingMaterial::Metal)
      continue;
    Triangle triangle{m.vertices[i].position, m.vertices[i + 1].position,
                      m.vertices[i + 2].position};
    std::sort(triangle.begin(), triangle.end());
    result.push_back(triangle);
  }
  std::sort(result.begin(), result.end());
  return result;
}
} // namespace
TEST_CASE("procedural roof forms preserve mapped total height and finite unit "
          "normals",
          "[world][building_mesh]") {
  for (const auto *shape :
       {"flat", "gabled", "hipped", "pyramidal", "skillion"}) {
    auto b = house();
    b.roof_shape = shape;
    b.roof_height = 2.5;
    const auto m = build_building_mesh(b, opts());
    INFO(shape);
    REQUIRE(!m.vertices.empty());
    CHECK_FALSE(m.unsupported_roof);
    CHECK(m.vertices.size() % 3 == 0);
    float max_z = -10000;
    for (const auto &v : m.vertices) {
      for (auto x : v.position)
        CHECK(std::isfinite(x));
      max_z = std::max(max_z, v.position[2]);
      const auto n = v.normal;
      CHECK(std::abs(n[0] * n[0] + n[1] * n[1] + n[2] * n[2] - 1) < 1e-5);
      CHECK(v.position[2] <= 109.0001);
    }
    CHECK(std::abs(max_z - 109) < 1e-4);
    CHECK(roof_area(m) >= 95.99);
    CHECK(m.windows > 0);
  }
}
TEST_CASE(
    "mapped building metadata survives extraction without moving the footprint",
    "[world][building_mesh]") {
  g2m::osm::OsmBuilder src;
  g2m_building_test::square(src, 77, 700, 358500, 5580900, 6,
                            {{"building", "house"},
                             {"height", "11"},
                             {"roof:shape", "hipped"},
                             {"roof:height", "2,5 m"},
                             {"roof:colour", "#804020"},
                             {"roof:orientation", "across"},
                             {"roof:direction", "90"},
                             {"building:levels", "2"},
                             {"building:material", "brick"}});
  auto data = src.build();
  REQUIRE(data);
  g2m::OsmTile tile;
  tile.data = std::move(data).value();
  auto buildings = extract_buildings(tile, g2m_building_test::kOsmKey, 0, 8, 3,
                                     &g2m_building_test::ground);
  REQUIRE(buildings.size() == 1);
  const auto &b = buildings.front();
  CHECK(b.height == 11);
  CHECK(b.roof_height == 2.5);
  CHECK(b.roof_shape == "hipped");
  CHECK(b.roof_colour == "#804020");
  CHECK(b.roof_orientation == "across");
  CHECK(b.roof_direction == "90");
  CHECK(b.levels == 2);
  CHECK(b.material == "brick");
}
TEST_CASE("courtyards and concave roofs are not replaced by bounding boxes",
          "[world][building_mesh]") {
  auto b = house();
  b.roof_shape = "gabled";
  b.inners = {{{450004, 5500002},
               {450008, 5500002},
               {450008, 5500006},
               {450004, 5500006}}};
  const auto m = build_building_mesh(b, opts());
  CHECK(m.roof == RoofForm::Gabled);
  CHECK_FALSE(m.unsupported_roof);
  CHECK(std::abs(roof_area(m) - 80) < 1e-4);
  b.inners.clear();
  b.outers = {{{450000, 5500000},
               {450012, 5500000},
               {450012, 5500004},
               {450006, 5500004},
               {450006, 5500008},
               {450000, 5500008}}};
  const auto c = build_building_mesh(b, opts());
  CHECK(c.roof == RoofForm::Gabled);
  CHECK(std::abs(roof_area(c) - 72) < 1e-4);
}
TEST_CASE(
    "facade cost is bounded and skyline geometry keeps the same silhouette",
    "[world][building_mesh]") {
  auto b = house();
  b.height = 1800;
  b.levels = 600;
  auto o = opts();
  o.max_windows = 17;
  const auto full = build_building_mesh(b, o);
  CHECK(full.windows == 17);
  CHECK(full.vertices.size() < 1500);
  o.facade_detail = false;
  const auto lite = build_building_mesh(b, o);
  CHECK(lite.windows == 0);
  CHECK(lite.vertices.size() < full.vertices.size());
  auto max_z = [](const auto &m) {
    float z = -10000;
    for (const auto &v : m.vertices)
      z = std::max(z, v.position[2]);
    return z;
  };
  CHECK(max_z(full) == max_z(lite));
}
TEST_CASE(
    "feature appearance is stable across requests and render-origin shifts",
    "[world][building_mesh]") {
  auto b = house();
  auto o = opts();
  const auto first = build_building_mesh(b, o);
  CHECK(hash(first) == hash(build_building_mesh(b, o)));
  auto reversed = b;
  std::reverse(reversed.outers.front().begin(), reversed.outers.front().end());
  const auto r = build_building_mesh(reversed, o);
  CHECK(r.roof == first.roof);
  CHECK(std::abs(roof_area(r) - roof_area(first)) < 1e-4);
  o.origin_e += 1024;
  const auto shifted = build_building_mesh(b, o);
  REQUIRE(shifted.vertices.size() == first.vertices.size());
  for (std::size_t i = 0; i < first.vertices.size(); ++i) {
    CHECK(std::abs(shifted.vertices[i].position[0] + 1024 -
                   first.vertices[i].position[0]) < 1e-4);
    CHECK(shifted.vertices[i].colour == first.vertices[i].colour);
    CHECK(shifted.vertices[i].normal == first.vertices[i].normal);
  }
}
TEST_CASE("mapped roof directions align deterministic pitched geometry",
          "[world][building_mesh]") {
  for (const auto *shape : {"gabled", "hipped", "skillion", "pyramidal"}) {
    auto b = house();
    b.roof_shape = shape;
    b.roof_direction = "90";
    b.roof_height = 2.5;
    const auto directed = build_building_mesh(b, opts());
    b.roof_direction = "E";
    const auto compass = build_building_mesh(b, opts());
    INFO(shape);
    CHECK(directed.roof != RoofForm::Flat);
    CHECK_FALSE(directed.unsupported_roof);
    CHECK(hash(directed) == hash(compass));
    float max_z = -10000;
    for (const auto &v : directed.vertices) {
      max_z = std::max(max_z, v.position[2]);
      CHECK(v.position[2] <= 109.0001f);
    }
    CHECK(std::abs(max_z - 109) < 1e-4);
  }
  auto b = house();
  b.roof_shape = "gabled";
  b.roof_direction = "90";
  const auto rotated = build_building_mesh(b, opts());
  double ridge_e = 0, ridge_n_min = 10000, ridge_n_max = -10000;
  std::size_t ridge_count = 0;
  for (const auto &v : rotated.vertices)
    if ((v.material == BuildingMaterial::Roof ||
         v.material == BuildingMaterial::Slate ||
         v.material == BuildingMaterial::Metal) &&
        std::abs(v.position[2] - 109) < 1e-4) {
      ridge_e += v.position[0];
      ridge_n_min = std::min(ridge_n_min, static_cast<double>(v.position[1]));
      ridge_n_max = std::max(ridge_n_max, static_cast<double>(v.position[1]));
      ++ridge_count;
    }
  REQUIRE(ridge_count > 0);
  CHECK(std::abs(ridge_e / ridge_count - 6) < 1e-4);
  CHECK(std::abs(ridge_n_min) < 1e-4);
  CHECK(std::abs(ridge_n_max - 8) < 1e-4);

  auto diagonal = house();
  diagonal.roof_shape = "gabled";
  diagonal.roof_direction = "45";
  diagonal.roof_height = 2.5;
  constexpr double c = .86602540378443864676, s = .5;
  for (auto &p : diagonal.outers.front()) {
    const double e = p.e - 450006, n = p.n - 5500004;
    p.e = 450006 + c * e - s * n;
    p.n = 5500004 + s * e + c * n;
  }
  const auto rotated_footprint = build_building_mesh(diagonal, opts());
  CHECK(rotated_footprint.roof == RoofForm::Gabled);
  CHECK_FALSE(rotated_footprint.unsupported_roof);
  float rotated_top = -10000;
  for (const auto &v : rotated_footprint.vertices)
    rotated_top = std::max(rotated_top, v.position[2]);
  CHECK(std::abs(rotated_top - 109) < 1e-4);
  CHECK(std::abs(roof_area(rotated_footprint) - 96) < 1e-4);
  CHECK(hash(rotated_footprint) == hash(build_building_mesh(diagonal, opts())));

  auto east_slope = house();
  east_slope.roof_shape = "skillion";
  east_slope.roof_direction = "90";
  east_slope.roof_height = 2.5;
  const auto skillion = build_building_mesh(east_slope, opts());
  float west_high = -10000, east_low = 10000;
  for (const auto &v : skillion.vertices) {
    if (v.material != BuildingMaterial::Roof &&
        v.material != BuildingMaterial::Slate &&
        v.material != BuildingMaterial::Metal)
      continue;
    if (std::abs(v.position[0]) < 1e-4)
      west_high = std::max(west_high, v.position[2]);
    if (std::abs(v.position[0] - 12) < 1e-4)
      east_low = std::min(east_low, v.position[2]);
  }
  CHECK(std::abs(west_high - 109) < 1e-4);
  CHECK(std::abs(east_low - 106.5) < 1e-4);

  auto gable_north = house();
  gable_north.roof_shape = "gabled";
  gable_north.roof_direction = "0";
  auto gable_south = gable_north;
  gable_south.roof_direction = "180";
  CHECK(roof_triangles(build_building_mesh(gable_north, opts())) ==
        roof_triangles(build_building_mesh(gable_south, opts())));
  auto gable_zero = gable_north;
  gable_zero.roof_direction = "N";
  auto gable_wrap = gable_north;
  gable_wrap.roof_direction = "360";
  CHECK(hash(build_building_mesh(gable_north, opts())) ==
        hash(build_building_mesh(gable_zero, opts())));
  CHECK(hash(build_building_mesh(gable_north, opts())) ==
        hash(build_building_mesh(gable_wrap, opts())));
  gable_north.roof_direction = "90.5";
  CHECK_FALSE(build_building_mesh(gable_north, opts()).unsupported_roof);
  for (const auto *invalid : {"-1", "360.1", "nan", "inf"}) {
    gable_north.roof_direction = invalid;
    CHECK(build_building_mesh(gable_north, opts()).unsupported_roof);
  }
  auto short_ridge_hip = house();
  short_ridge_hip.roof_shape = "hipped";
  short_ridge_hip.roof_direction = "90";
  short_ridge_hip.roof_height = 2.5;
  const auto hipped = build_building_mesh(short_ridge_hip, opts());
  CHECK(hipped.roof == RoofForm::Hipped);
  CHECK_FALSE(hipped.unsupported_roof);
  float hipped_top = -10000;
  for (const auto &v : hipped.vertices)
    hipped_top = std::max(hipped_top, v.position[2]);
  CHECK(std::abs(hipped_top - 109) < 1e-4);

  auto chimney_case = short_ridge_hip;
  auto chimney_mesh = build_building_mesh(chimney_case, opts());
  for (std::uint64_t id = 1; chimney_mesh.chimneys == 0 && id <= 12; ++id) {
    chimney_case.id = id;
    chimney_mesh = build_building_mesh(chimney_case, opts());
  }
  REQUIRE(chimney_mesh.chimneys > 0);
  float chimney_bottom = 10000;
  for (const auto &v : chimney_mesh.vertices)
    if (v.material == BuildingMaterial::Brick)
      chimney_bottom = std::min(chimney_bottom, v.position[2]);
  CHECK(std::abs(chimney_bottom - 107.225) < 1e-4);
}
TEST_CASE("mapped directions clip across concave footprints and courtyards",
          "[world][building_mesh]") {
  auto b = house();
  b.roof_shape = "gabled";
  b.roof_direction = "NE";
  b.roof_height = 2.5;
  b.inners = {{{450004, 5500002},
               {450008, 5500002},
               {450008, 5500006},
               {450004, 5500006}}};
  const auto courtyard = build_building_mesh(b, opts());
  CHECK(courtyard.roof == RoofForm::Gabled);
  CHECK_FALSE(courtyard.unsupported_roof);
  CHECK(std::abs(roof_area(courtyard) - 80) < 1e-4);
  float courtyard_top = -10000;
  for (const auto &v : courtyard.vertices)
    courtyard_top = std::max(courtyard_top, v.position[2]);
  CHECK(std::abs(courtyard_top - 109) < 1e-4);

  b.inners.clear();
  b.outers = {{{450000, 5500000},
               {450012, 5500000},
               {450012, 5500004},
               {450006, 5500004},
               {450006, 5500008},
               {450000, 5500008}}};
  const auto concave = build_building_mesh(b, opts());
  CHECK(concave.roof == RoofForm::Gabled);
  CHECK_FALSE(concave.unsupported_roof);
  CHECK(std::abs(roof_area(concave) - 72) < 1e-4);
  float concave_top = -10000;
  for (const auto &v : concave.vertices)
    concave_top = std::max(concave_top, v.position[2]);
  CHECK(std::abs(concave_top - 109) < 1e-4);
}
TEST_CASE("mapped flat and unsupported roofs do not get inferred pitched "
          "replacements",
          "[world][building_mesh]") {
  auto b = house();
  b.roof_shape = "flat";
  CHECK(build_building_mesh(b, opts()).roof == RoofForm::Flat);
  b.roof_shape = "dome";
  const auto m = build_building_mesh(b, opts());
  CHECK(m.roof == RoofForm::Flat);
  CHECK(m.unsupported_roof);
  b.roof_shape = "gabled";
  b.roof_direction = "not-a-compass-direction";
  const auto invalid_direction = build_building_mesh(b, opts());
  CHECK(invalid_direction.unsupported_roof);
  CHECK(invalid_direction.roof == RoofForm::Flat);
  b.roof_direction.clear();
  b.bottom = 105;
  b.min_height = 5;
  b.roof_height = 20;
  const auto raised = build_building_mesh(b, opts());
  for (const auto &v : raised.vertices)
    CHECK(v.position[2] >= 104.5);
  b.height = std::numeric_limits<double>::quiet_NaN();
  CHECK(build_building_mesh(b, opts()).vertices.empty());
}

TEST_CASE("building faces have non-degenerate metric UVs and mapped material "
          "semantics",
          "[world][building_mesh]") {
  auto b = house();
  b.roof_shape = "gabled";
  b.roof_material = "slate";
  b.material = "brick";
  const auto m = build_building_mesh(b, opts());
  bool brick = false, slate = false, projecting_sill = false;
  for (std::size_t i = 0; i < m.vertices.size(); i += 3) {
    const auto &a = m.vertices[i];
    const auto &c = m.vertices[i + 1];
    const auto &d = m.vertices[i + 2];
    const double uv_area = (c.uv[0] - a.uv[0]) * (d.uv[1] - a.uv[1]) -
                           (d.uv[0] - a.uv[0]) * (c.uv[1] - a.uv[1]);
    CHECK(std::abs(uv_area) > 1e-7);
    brick = brick || a.material == BuildingMaterial::Brick;
    slate = slate || a.material == BuildingMaterial::Slate;
    if (a.material == BuildingMaterial::Frame &&
        (a.position[0] < -.1 || a.position[0] > 12.1 || a.position[1] < -.1 ||
         a.position[1] > 8.1))
      projecting_sill = true;
  }
  CHECK(brick);
  CHECK(slate);
  CHECK(projecting_sill);
}

TEST_CASE("residential buildings include volumetric doors and supported "
          "chimney models",
          "[world][building_mesh]") {
  auto b = house();
  b.roof_shape = "gabled";
  const auto full = build_building_mesh(b, opts());
  CHECK(full.doors == 1);
  CHECK(full.windows > 0);
  auto lite = opts();
  lite.facade_detail = false;
  const auto far = build_building_mesh(b, lite);
  CHECK(far.doors == 0);
  CHECK(far.chimneys == 0);
  CHECK(far.windows == 0);
  bool chimney = false;
  for (int id = 1; id <= 5; ++id) {
    b.id = id;
    const auto m = build_building_mesh(b, opts());
    chimney = chimney || m.chimneys > 0;
    for (const auto &v : m.vertices)
      CHECK(v.position[2] <= 109.0001);
  }
  CHECK(chimney);
}
