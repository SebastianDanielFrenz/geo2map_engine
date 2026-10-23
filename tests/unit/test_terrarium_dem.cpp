#include "g2m/core/geo/utm.h"
#include "g2m/decode/terrarium_dem.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <vector>

using namespace g2m;
using namespace g2m::decode;

namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;

std::vector<std::uint8_t> constant_rgba(int size, std::uint8_t r,
                                        std::uint8_t g, std::uint8_t b,
                                        std::uint8_t a = 255) {
  std::vector<std::uint8_t> bytes(static_cast<std::size_t>(size) * size * 4);
  for (std::size_t i = 0; i < bytes.size(); i += 4) {
    bytes[i] = r;
    bytes[i + 1] = g;
    bytes[i + 2] = b;
    bytes[i + 3] = a;
  }
  return bytes;
}

double longitude_at_pixel(double x, double world_pixels) {
  return ((x + 0.5) / world_pixels) * 360.0 - 180.0;
}

double latitude_at_pixel(double y, double world_pixels) {
  const double mercator = kPi * (1.0 - 2.0 * (y + 0.5) / world_pixels);
  return std::atan(std::sinh(mercator)) * 180.0 / kPi;
}

TerrariumTile tile(int z, int x, int y, int size, std::uint8_t r,
                   std::uint8_t g, std::uint8_t b) {
  return {z, x, y, constant_rgba(size, r, g, b)};
}
} // namespace

TEST_CASE("Terrarium decodes RGB heights including below sea level",
          "[terrarium_dem]") {
  const TerrariumSourceBounds bounds{1, 0, 0, 0, 0};
  TerrariumDem sea_level({tile(1, 0, 0, 4, 128, 0, 0)}, bounds, 4 * 4 * 4, 4);
  const double lat = latitude_at_pixel(1, 8);
  const double lon = longitude_at_pixel(1, 8);
  REQUIRE(sea_level.sample(lat, lon) == Catch::Approx(0.0).margin(1e-10));

  TerrariumDem below_sea({tile(1, 0, 0, 4, 127, 251, 0)}, bounds, 4 * 4 * 4, 4);
  REQUIRE(below_sea.sample(lat, lon) == Catch::Approx(-5.0).margin(1e-10));
}

TEST_CASE("Terrarium bilinear interpolation crosses adjacent tile borders",
          "[terrarium_dem]") {
  auto west = constant_rgba(4, 128, 10, 0);
  auto east = constant_rgba(4, 128, 20, 0);
  TerrariumDem dem({TerrariumTile{1, 1, 0, std::move(east)},
                    TerrariumTile{1, 0, 0, std::move(west)}},
                   TerrariumSourceBounds{1, 0, 1, 0, 0}, 2 * 4 * 4 * 4, 4);
  const double lat = latitude_at_pixel(1, 8);
  const double seam_lon = ((4.0 / 8.0) * 360.0) - 180.0;
  REQUIRE(dem.sample(lat, seam_lon) == Catch::Approx(15.0).margin(1e-8));
}

TEST_CASE("Terrarium wraps longitude across the antimeridian within bounds",
          "[terrarium_dem]") {
  auto last = constant_rgba(4, 128, 10, 0);
  auto first = constant_rgba(4, 128, 20, 0);
  TerrariumDem dem({TerrariumTile{2, 0, 0, std::move(first)},
                    TerrariumTile{2, 3, 0, std::move(last)}},
                   TerrariumSourceBounds{2, 3, 0, 0, 0}, 2 * 4 * 4 * 4, 4);
  const double lat = latitude_at_pixel(1, 16);
  const auto at_dateline = dem.sample(lat, 180.0);
  const auto wrapped = dem.sample(lat, -180.0);
  REQUIRE(at_dateline);
  REQUIRE(wrapped);
  REQUIRE(*at_dateline == Catch::Approx(15.0).margin(1e-8));
  REQUIRE(*at_dateline == *wrapped);
  REQUIRE_FALSE(dem.sample(lat, 0.0));
}

TEST_CASE("Terrarium snaps subpixel roundoff at valid pixel centres",
          "[terrarium_dem]") {
  auto rgba = constant_rgba(4, 0, 0, 0, 0);
  const std::size_t center = (1 * 4 + 1) * 4;
  rgba[center] = 128;
  rgba[center + 1] = 0;
  rgba[center + 2] = 0;
  rgba[center + 3] = 255;
  TerrariumDem dem({TerrariumTile{1, 0, 0, std::move(rgba)}},
                   TerrariumSourceBounds{1, 0, 0, 0, 0}, 4 * 4 * 4, 4);
  const double lat = latitude_at_pixel(1, 8) + 1e-12;
  const double lon = longitude_at_pixel(1, 8) + 1e-12;
  const auto height = dem.sample(lat, lon);
  REQUIRE(height);
  CHECK(*height == Catch::Approx(0.0).margin(1e-10));
}
TEST_CASE(
    "Terrarium propagates NoData and does not extrapolate beyond coverage",
    "[terrarium_dem]") {
  auto rgba = constant_rgba(4, 127, 251, 0);
  rgba[(1 * 4 + 1) * 4 + 3] = 0;
  TerrariumDem dem({TerrariumTile{1, 0, 0, std::move(rgba)}},
                   TerrariumSourceBounds{1, 0, 0, 0, 0}, 4 * 4 * 4, 4);
  const double lat = latitude_at_pixel(1, 8);
  const double lon = longitude_at_pixel(1, 8);
  REQUIRE_FALSE(dem.sample(lat, lon));
  REQUIRE_FALSE(dem.sample(0.0, longitude_at_pixel(1, 8)));
  REQUIRE_FALSE(dem.sample(std::numeric_limits<double>::quiet_NaN(), lon));
  REQUIRE_FALSE(dem.sample(86.0, lon));
  REQUIRE_FALSE(dem.sample(lat, std::numeric_limits<double>::infinity()));
}

TEST_CASE("Terrarium reprojection preserves registration and quantises "
          "deterministically",
          "[terrarium_dem]") {
  // RGB(128,0,1) represents 1/256 m. Adding 1/512 m is an exact tie
  // between 1 and 2 quantisation units and half-even selects 2.
  TerrariumDem dem({tile(0, 0, 0, 8, 128, 0, 1)},
                   TerrariumSourceBounds{0, 0, 0, 0, 0}, 8 * 8 * 4, 8);
  const geo::Utm utm;
  const auto origin = utm.forward({32, geo::Hemisphere::North}, 50.0, 8.5);
  RasterGeoref georef;
  georef.zone = {32, geo::Hemisphere::North};
  georef.west_mm = static_cast<std::int64_t>(origin.easting * 1000.0);
  georef.south_mm = static_cast<std::int64_t>(origin.northing * 1000.0);
  georef.spacing_mm = 1000;
  georef.registration = SampleRegistration::Cell;
  georef.row_order = RowOrder::NorthToSouth;
  georef.width = 3;
  georef.height = 2;

  const auto a = dem.reproject(georef, 1.0 / 512.0);
  const auto b = dem.reproject(georef, 1.0 / 512.0);
  REQUIRE(a.georef == georef);
  REQUIRE(a == b);
  for (const auto height : a.heights.data())
    REQUIRE(height == 2);

  REQUIRE_THROWS_AS(
      dem.reproject(georef, std::numeric_limits<double>::infinity()),
      DecodeError);
}

TEST_CASE("Terrarium rejects malformed, duplicate, out-of-bounds and "
          "over-budget tiles",
          "[terrarium_dem]") {
  const TerrariumSourceBounds bounds{1, 0, 0, 0, 0};
  REQUIRE_THROWS_AS(TerrariumDem({}, bounds, 4096, 4), DecodeError);
  REQUIRE_THROWS_AS(TerrariumDem({tile(1, 0, 0, 4, 128, 0, 0)}, bounds, 63, 4),
                    DecodeError);
  REQUIRE_THROWS_AS(TerrariumDem({tile(1, 1, 0, 4, 128, 0, 0)}, bounds, 64, 4),
                    DecodeError);
  REQUIRE_THROWS_AS(
      TerrariumDem({TerrariumTile{1, 0, 0, std::vector<std::uint8_t>(63)}},
                   bounds, 64, 4),
      DecodeError);
  REQUIRE_THROWS_AS(
      TerrariumDem({tile(1, 0, 0, 4, 128, 0, 0), tile(1, 0, 0, 4, 128, 0, 0)},
                   bounds, 128, 4),
      DecodeError);
  REQUIRE_THROWS_AS(TerrariumDem({tile(1, 0, 0, 4, 128, 0, 0)},
                                 TerrariumSourceBounds{1, 0, 2, 0, 0}, 64, 4),
                    DecodeError);
}
