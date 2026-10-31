#include "g2m/world/dsm_ground.h"
#include <catch2/catch_test_macros.hpp>
namespace {
using namespace g2m;
using namespace g2m::world;
Raster<std::uint8_t> kinds(int n, ElevationSourceKind kind) {
  return {n, n, static_cast<std::uint8_t>(kind)};
}
HeightRaster crop(const HeightRaster &a, int x, int y, int n) {
  HeightRaster b(n, n);
  for (int j = 0; j < n; ++j)
    for (int i = 0; i < n; ++i)
      b.at(i, j) = a.at(x + i, y + j);
  return b;
}
} // namespace
TEST_CASE(
    "DSM correction preserves measured DTM water negative ground and NoData",
    "[world][dsm_ground]") {
  DsmGroundOptions o;
  const auto halo = dsm_ground_halo(o);
  REQUIRE(halo);
  const int h = *halo, n = 2 * h + 16;
  HeightRaster a(n, n, -10 * 256);
  auto k = kinds(n, ElevationSourceKind::Dsm);
  for (int y = h + 3; y < h + 7; ++y)
    for (int x = h + 3; x < h + 7; ++x)
      a.at(x, y) = 10 * 256;
  auto corrected = correct_dsm_ground(a, k, {h, h, 16, 16}, o);
  REQUIRE(corrected);
  CHECK(corrected->at(4, 4) == -10 * 256);
  CHECK(corrected->at(12, 12) == -10 * 256);
  for (auto kind : {ElevationSourceKind::Dtm, ElevationSourceKind::Water,
                    ElevationSourceKind::Unknown}) {
    auto result = correct_dsm_ground(a, kinds(n, kind), {h, h, 16, 16}, o);
    REQUIRE(result);
    CHECK(result->at(4, 4) == 10 * 256);
  }
  a.at(h, h) = kHeightNoData;
  corrected = correct_dsm_ground(a, k, {h, h, 16, 16}, o);
  REQUIRE(corrected);
  CHECK(corrected->at(0, 0) == kHeightNoData);
  CHECK(corrected->at(4, 4) == 10 * 256);
}
TEST_CASE(
    "DSM full-context and independently tiled cores agree including corners",
    "[world][dsm_ground]") {
  DsmGroundOptions o;
  o.sample_m = 20;
  const int h = *dsm_ground_halo(o), n = 2 * h + 32;
  HeightRaster a(n, n);
  auto k = kinds(n, ElevationSourceKind::Dsm);
  for (int y = 0; y < n; ++y)
    for (int x = 0; x < n; ++x)
      a.at(x, y) =
          256 * (5 + x / 5 + y / 7 + ((x % 17 < 4 && y % 19 < 4) ? 15 : 0));
  auto all = correct_dsm_ground(a, k, {h, h, 32, 32}, o);
  REQUIRE(all);
  for (int ty = 0; ty < 2; ++ty)
    for (int tx = 0; tx < 2; ++tx) {
      const auto context = crop(a, tx * 16, ty * 16, 2 * h + 16);
      auto part = correct_dsm_ground(
          context, kinds(2 * h + 16, ElevationSourceKind::Dsm), {h, h, 16, 16},
          o);
      REQUIRE(part);
      for (int y = 0; y < 16; ++y)
        for (int x = 0; x < 16; ++x)
          CHECK(part->at(x, y) == all->at(tx * 16 + x, ty * 16 + y));
    }
  CHECK_FALSE(correct_dsm_ground(a, k, {h - 1, h, 16, 16}, o));
}
TEST_CASE("DSM hill protection and mask-aware mixed sources do not lower "
          "measured ground",
          "[world][dsm_ground]") {
  DsmGroundOptions o;
  const int h = *dsm_ground_halo(o), n = 2 * h + 16;
  HeightRaster a(n, n, 0);
  auto k = kinds(n, ElevationSourceKind::Dsm);
  for (int y = h + 3; y < h + 7; ++y)
    for (int x = h + 3; x < h + 7; ++x)
      a.at(x, y) = 80 * 256;
  auto result = correct_dsm_ground(a, k, {h, h, 16, 16}, o);
  REQUIRE(result);
  CHECK(result->at(4, 4) == 80 * 256);
  a.at(h + 4, h + 4) = 20 * 256;
  k.at(h + 4, h + 4) = static_cast<std::uint8_t>(ElevationSourceKind::Dtm);
  result = correct_dsm_ground(a, k, {h, h, 16, 16}, o);
  REQUIRE(result);
  CHECK(result->at(4, 4) == 20 * 256);
  CHECK(*dsm_ground_halo(o) == 18);
  o.sample_m = 0;
  CHECK_FALSE(dsm_ground_halo(o));
}
