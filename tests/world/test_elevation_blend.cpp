#include "g2m/world/elevation_blend.h"

#include <catch2/catch_test_macros.hpp>

namespace {
using namespace g2m;
using namespace g2m::world;

HeightRaster crop(const HeightRaster &raster, int x, int y, int width,
                  int height) {
  HeightRaster result(width, height);
  for (int row = 0; row < height; ++row)
    for (int col = 0; col < width; ++col)
      result.at(col, row) = raster.at(x + col, y + row);
  return result;
}
} // namespace

TEST_CASE("elevation source priority preserves primary and fills fallback",
          "[world][elevation_blend]") {
  HeightRaster primary(8, 6, kHeightNoData), fallback(8, 6, -20 * 256);
  primary.at(2, 2) = -7 * 256;
  primary.at(3, 2) = 11 * 256;
  fallback.at(4, 2) = kHeightNoData;
  auto result =
      blend_elevation_sources(primary, fallback, {0, 0, 8, 6}, {.radius_m = 0});
  REQUIRE(result);
  CHECK(result->heights.at(2, 2) == -7 * 256);
  CHECK(result->heights.at(3, 2) == 11 * 256);
  CHECK(result->heights.at(0, 0) == -20 * 256);
  CHECK(result->heights.at(4, 2) == kHeightNoData);
  CHECK(result->diagnostics.primary_preserved == 2);
  CHECK(result->diagnostics.fallback_filled == 45);
  CHECK(result->diagnostics.no_data == 1);
  CHECK(result->diagnostics.fallback_without_overlap == 45);
}

TEST_CASE("constant overlap offset decays linearly and respects its cap",
          "[world][elevation_blend]") {
  HeightRaster primary(17, 13, kHeightNoData), fallback(17, 13, 100 * 256);
  for (int y = 4; y < 9; ++y)
    for (int x = 4; x < 7; ++x)
      primary.at(x, y) = fallback.at(x, y) + 10 * 256;
  auto result = blend_elevation_sources(
      primary, fallback, {4, 4, 9, 5},
      {.spacing_m = 1, .radius_m = 4, .max_adjust_m = 6});
  REQUIRE(result);
  CHECK(result->heights.at(3, 2) == 104 * 256 + 128);
  CHECK(result->heights.at(5, 2) == 101 * 256 + 128);
  CHECK(result->heights.at(6, 2) == 100 * 256);
  CHECK(result->heights.at(1, 2) == primary.at(5, 6));
  CHECK(result->diagnostics.fallback_adjusted > 0);
}

TEST_CASE("fractional physical radius fades using sample spacing",
          "[world][elevation_blend]") {
  HeightRaster primary(11, 9, kHeightNoData), fallback(11, 9, 0);
  primary.at(4, 4) = 12 * 256;
  auto result = blend_elevation_sources(
      primary, fallback, {2, 2, 7, 5},
      {.spacing_m = 1, .radius_m = 1.5, .max_adjust_m = 20});
  REQUIRE(result);
  CHECK(result->heights.at(3, 2) == 4 * 256);
  CHECK(result->heights.at(4, 2) == 0);
}

TEST_CASE("ramp overlaps use nearest local offset and deterministic ties",
          "[world][elevation_blend]") {
  HeightRaster primary(11, 11, kHeightNoData), fallback(11, 11, 0);
  primary.at(5, 4) = 10 * 256; // first row-major seed: +10 m
  primary.at(4, 5) = 20 * 256; // tied at (2,2), but later row-major seed
  auto result = blend_elevation_sources(primary, fallback, {2, 2, 7, 7},
                                        {.radius_m = 2, .max_adjust_m = 20});
  REQUIRE(result);
  CHECK(result->heights.at(2, 2) == 5 * 256);
  CHECK(result->heights.at(3, 2) == 10 * 256);
  CHECK(result->heights.at(2, 3) == 20 * 256);
}

TEST_CASE("full-context and halo-tiled elevation blends agree",
          "[world][elevation_blend]") {
  constexpr int radius = 3, side = 24;
  HeightRaster primary(side, side), fallback(side, side);
  for (int y = 0; y < side; ++y) {
    for (int x = 0; x < side; ++x) {
      fallback.at(x, y) = (x * 3 - y * 2) * 256;
      primary.at(x, y) =
          x < 9 ? fallback.at(x, y) + (4 + y) * 256 : kHeightNoData;
    }
  }
  const auto whole =
      blend_elevation_sources(primary, fallback, {radius, radius, 18, 18},
                              {.radius_m = static_cast<double>(radius)});
  REQUIRE(whole);
  for (int tile_y = 0; tile_y < 2; ++tile_y) {
    for (int tile_x = 0; tile_x < 2; ++tile_x) {
      const int ox = tile_x * 9;
      const int oy = tile_y * 9;
      const auto p = crop(primary, ox, oy, 9 + 2 * radius, 9 + 2 * radius);
      const auto f = crop(fallback, ox, oy, 9 + 2 * radius, 9 + 2 * radius);
      const auto tile =
          blend_elevation_sources(p, f, {radius, radius, 9, 9},
                                  {.radius_m = static_cast<double>(radius)});
      REQUIRE(tile);
      for (int y = 0; y < 9; ++y)
        for (int x = 0; x < 9; ++x)
          CHECK(tile->heights.at(x, y) ==
                whole->heights.at(tile_x * 9 + x, tile_y * 9 + y));
    }
  }
}

TEST_CASE("elevation blend rejects invalid support and sample budgets",
          "[world][elevation_blend]") {
  HeightRaster a(8, 8, 0), b(8, 8, 0);
  CHECK_FALSE(blend_elevation_sources(a, b, {1, 1, 6, 6}, {.radius_m = 2}));
  CHECK_FALSE(blend_elevation_sources(a, b, {2, 2, 4, 4}, {.spacing_m = 0}));
  CHECK_FALSE(blend_elevation_sources(a, b, {2, 2, 4, 4}, {.max_samples = 32}));
  CHECK_FALSE(blend_elevation_sources(a, HeightRaster(7, 8), {0, 0, 7, 8}));
}