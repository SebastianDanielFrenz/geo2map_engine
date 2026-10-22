#include "g2m/world/bridge_visual.h"
#include "g2m/world/building_mesh.h"
#include "g2m/world/dsm_ground.h"
#include "g2m/world/road_paint.h"
#include "g2m/world/road_structures.h"
#include <bit>
#include <catch2/catch_test_macros.hpp>
#include <sstream>
namespace {
struct Digest {
  std::uint64_t hash = 14695981039346656037ULL;
  void add(std::uint32_t v) {
    for (int i = 0; i < 4; ++i) {
      hash ^= (v >> (i * 8)) & 255;
      hash *= 1099511628211ULL;
    }
  }
  void f(float v) { add(std::bit_cast<std::uint32_t>(v)); }
};
} // namespace
TEST_CASE("terrain port geometry has cross-platform golden hashes",
          "[world][port_goldens]") {
  using namespace g2m::world;
  Building house;
  house.id = 42;
  house.type = "house";
  house.height = 9;
  house.base = 100;
  house.bottom = 98;
  house.roof_shape = "hipped";
  house.material = "brick";
  house.outers = {{{450000, 5500000},
                   {450012, 5500000},
                   {450012, 5500008},
                   {450000, 5500008}}};
  BuildingMeshOptions options;
  options.origin_e = 450000;
  options.origin_n = 5500000;
  Digest building;
  const auto m = build_building_mesh(house, options);
  for (const auto &v : m.vertices) {
    for (const auto &f : {v.position, v.normal, v.colour})
      for (auto x : f)
        building.f(x);
    for (auto x : v.uv)
      building.f(x);
    building.add(static_cast<std::uint32_t>(v.material));
  }
  RoadRibbonTile road;
  road.vertices = {{-4, 0, 100, -4, 0, 8, 1, 2, 1, 3, 0},
                   {4, 0, 100, 4, 0, 8, 1, 2, 1, 3, 0},
                   {4, 18, 100.18f, 4, 18, 8, 1, 2, 1, 3, 0},
                   {-4, 18, 100.18f, -4, 18, 8, 1, 2, 1, 3, 0}};
  road.indices = {0, 1, 2, 0, 2, 3};
  const auto paint = build_road_paint(road);
  REQUIRE(paint.complete);
  Digest marking;
  for (const auto &v : paint.mesh.vertices)
    for (float x : {v.x, v.y, v.z, v.u, v.v, v.width, v.fade, v.lanes,
                    v.forward, v.flags, v.verge})
      marking.f(x);
  for (auto x : paint.mesh.indices)
    marking.add(static_cast<std::uint32_t>(x));
  RoadDeck deck;
  deck.end_station = 40;
  for (int i = 0; i <= 40; ++i)
    for (int j = 0; j < 5; ++j)
      deck.mesh.positions.insert(deck.mesh.positions.end(),
                                 {static_cast<float>(i),
                                  static_cast<float>(2 * j - 4),
                                  100 + .01f * i});
  BridgeFacts facts;
  facts.form = BridgeForm::Truss;
  facts.explicit_form = true;
  const auto visual = build_bridge_visual(deck, facts);
  Digest bridge;
  for (const auto *data : {&visual.mesh.positions, &visual.mesh.normals})
    for (auto x : *data)
      bridge.f(x);
  for (auto x : visual.mesh.indices)
    bridge.add(x);
  DsmGroundOptions dsm_options;
  const int halo = *dsm_ground_halo(dsm_options), n = 2 * halo + 16;
  g2m::HeightRaster raster(n, n);
  g2m::Raster<std::uint8_t> kinds(
      n, n, static_cast<std::uint8_t>(ElevationSourceKind::Dsm));
  for (int y = 0; y < n; ++y)
    for (int x = 0; x < n; ++x)
      raster.at(x, y) =
          256 * (x / 5 - y / 7 + ((x % 13 < 4 && y % 11 < 4) ? 20 : 0));
  const auto corrected =
      correct_dsm_ground(raster, kinds, {halo, halo, 16, 16}, dsm_options);
  REQUIRE(corrected);
  Digest dsm;
  for (auto h : corrected->data())
    dsm.add(static_cast<std::uint32_t>(h));
  std::ostringstream hashes;
  hashes << std::hex << building.hash << " " << marking.hash << " "
         << bridge.hash << " " << dsm.hash;
  CHECK(hashes.str() ==
        "d61f99178f5b8990 67907b18c44ae827 922b58ce77ec77ed fbd2872c5d41ffef");
}
