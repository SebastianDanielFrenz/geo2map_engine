#include "g2m/core/geo/utm.h"
#include "g2m/world/elevation_ground.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
using namespace g2m;
using namespace g2m::world;
namespace {
RasterGeoref grid(int width = 128, int height = 128) {
  RasterGeoref g;
  g.zone = {32, geo::Hemisphere::North};
  g.west_mm = 466432000;
  g.south_mm = 5533184000;
  g.spacing_mm = 1000;
  g.width = width;
  g.height = height;
  g.registration = SampleRegistration::Cell;
  return g;
}
GroundElevationOptions options() {
  GroundElevationOptions o;
  o.spacing_m = 8;
  o.ground.opening_m = 32;
  o.ground.relief_m = 32;
  o.ground.relief_opening_m = 16;
  o.ground.threshold_m = 1;
  o.ground.max_cut_m = 30;
  return o;
}
} // namespace
TEST_CASE("ground import filters DSM before upsampling but preserves DTM and "
          "unknown detail",
          "[elevation_ground]") {
  const auto g = grid();
  const geo::Utm utm;
  const auto sample = [&](double lat, double lon) -> std::optional<double> {
    const auto p = utm.forward(g.zone, lat, lon);
    const double x = p.easting - g.west_mm / 1000.0,
                 y = p.northing - g.south_mm / 1000.0;
    return 100 + (x > 40 && x < 56 && y > 40 && y < 56 ? 12 : 0);
  };
  const auto corrected =
      prepare_ground_elevation(g, sample, ElevationSourceKind::Dsm, options());
  REQUIRE(corrected);
  REQUIRE(corrected->coarse_lowered > 0);
  REQUIRE(corrected->tile.heights.at(47, 47) < 112 * 256);
  for (auto type : {ElevationSourceKind::Dtm, ElevationSourceKind::Unknown,
                    ElevationSourceKind::Water}) {
    const auto original = prepare_ground_elevation(g, sample, type, options());
    REQUIRE(original);
    REQUIRE(original->coarse_lowered == 0);
    REQUIRE(original->tile.heights.at(47, 47) == 112 * 256);
  }
}
TEST_CASE("ground import composed halo makes independent tiles agree with "
          "whole output",
          "[elevation_ground]") {
  auto g = grid(256, 128);
  const geo::Utm utm;
  const auto sample = [&](double lat, double lon) -> std::optional<double> {
    const auto p = utm.forward(g.zone, lat, lon);
    const double x = p.easting - 466432, y = p.northing - 5533184;
    return -5 + x * .01 + (x > 120 && x < 136 && y > 32 && y < 56 ? 14 : 0);
  };
  const auto whole =
      prepare_ground_elevation(g, sample, ElevationSourceKind::Dsm, options());
  REQUIRE(whole);
  for (int tile = 0; tile < 2; ++tile) {
    auto part = g;
    part.width = 128;
    part.west_mm += tile * 128000;
    const auto result = prepare_ground_elevation(
        part, sample, ElevationSourceKind::Dsm, options());
    REQUIRE(result);
    for (int y = 0; y < 128; ++y)
      for (int x = 0; x < 128; ++x)
        REQUIRE(result->tile.heights.at(x, y) ==
                whole->tile.heights.at(x + tile * 128, y));
  }
}
TEST_CASE(
    "ground import falls back to original coverage and rejects invalid grids",
    "[elevation_ground]") {
  auto g = grid();
  const auto sample = [](double, double) -> std::optional<double> {
    return -3;
  };
  auto missing = [](double, double) -> std::optional<double> { return {}; };
  const auto result =
      prepare_ground_elevation(g, missing, ElevationSourceKind::Dsm, options());
  REQUIRE(result);
  REQUIRE(result->tile.heights.at(0, 0) == kHeightNoData);
  REQUIRE(result->raw_fallback_samples == result->tile.heights.size());
  g.west_mm += 1;
  REQUIRE_FALSE(
      prepare_ground_elevation(g, sample, ElevationSourceKind::Dsm, options()));
  g = grid();
  auto bad = options();
  bad.spacing_m = 7;
  REQUIRE_FALSE(
      prepare_ground_elevation(g, sample, ElevationSourceKind::Dsm, bad));
  REQUIRE_FALSE(
      prepare_ground_elevation(g, {}, ElevationSourceKind::Dsm, options()));
}
