#include "g2m/core/geo/utm.h"
#include "g2m/core/hash.h"
#include "g2m/decode/geographic_dem.h"
#include "tiff_builder.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <limits>
using namespace g2m;
using namespace g2m::decode;
namespace {
std::shared_ptr<IByteSource> fixture(double west, double north, double dx,
                                     double dy, int w = 4, int h = 4,
                                     bool hole = false) {
  test_support::TiffBuilder b;
  b.add_long(256, w);
  b.add_long(257, h);
  b.add_short(258, 32);
  b.add_short(259, 1);
  b.add_short(262, 1);
  b.add_short(277, 1);
  b.add_long(278, h);
  b.add_short(339, 3);
  b.add_double_array(33550, {dx, dy, 0});
  b.add_double_array(33922, {0, 0, 0, west, north, 0});
  b.add_short_array(
      34735, {1, 1, 0, 3, 1024, 0, 1, 2, 1025, 0, 1, 2, 2048, 0, 1, 4326});
  b.add_ascii(42113, "-9999");
  std::vector<float> values(static_cast<std::size_t>(w * h));
  for (int y = 0; y < h; ++y)
    for (int x = 0; x < w; ++x)
      values[y * w + x] = static_cast<float>(10 * (west + x * dx) +
                                             20 * (north - y * dy) - 1100);
  if (hole)
    values[5] = -9999;
  std::vector<std::byte> bytes(values.size() * 4);
  std::memcpy(bytes.data(), values.data(), bytes.size());
  return std::make_shared<MemoryByteSource>(b.build(bytes));
}
} // namespace
TEST_CASE("Geographic DEM interpolates across longitude and mixed-width "
          "latitude bands",
          "[geographic_dem]") {
  GeographicDem dem({fixture(8, 51, .25, .25), fixture(9, 51, .25, .25),
                     fixture(8, 50, .125, .25, 8),
                     fixture(9, 50, .125, .25, 8)});
  for (double lat : {50.875, 50.125, 50.0, 49.875})
    for (double lon : {8.125, 8.875, 9.0, 9.125}) {
      CAPTURE(lat, lon);
      const auto h = dem.sample(lat, lon);
      REQUIRE(h);
      REQUIRE(*h == Catch::Approx(10 * lon + 20 * lat - 1100).margin(1e-6));
    }
  const auto reads = dem.decoded_blocks();
  for (int i = 0; i < 10; ++i)
    REQUIRE(dem.sample(50.5, 8.5));
  REQUIRE(dem.decoded_blocks() == reads);
  REQUIRE_FALSE(dem.sample(48, 8));
}
TEST_CASE("Geographic DEM preserves negative land and NoData without edge "
          "extrapolation",
          "[geographic_dem]") {
  GeographicDem dem({fixture(8, 50, .25, .25, 4, 4, true)});
  REQUIRE(*dem.sample(50, 8) == -20);
  REQUIRE_FALSE(dem.sample(49.75, 8.25));
  REQUIRE_FALSE(dem.sample(49.875, 8.125));
  REQUIRE_FALSE(dem.sample(49.1, 8.9));
  REQUIRE_FALSE(dem.sample(std::numeric_limits<double>::quiet_NaN(), 8));
}
TEST_CASE("Geographic DEM wraps dateline samples", "[geographic_dem]") {
  GeographicDem dem({fixture(179, 1, .25, .25), fixture(-180, 1, .25, .25)});
  REQUIRE(dem.sample(.5, 179.875));
  REQUIRE(dem.sample(.5, 180.125) == dem.sample(.5, -179.875));
}
TEST_CASE("Geographic DEM reprojects deterministically to registered UTM grids",
          "[geographic_dem]") {
  GeographicDem dem({fixture(8, 51, .25, .25)});
  const geo::Utm utm;
  const geo::UtmZone zone{32, geo::Hemisphere::North};
  const auto p = utm.forward(zone, 50.5, 8.5);
  RasterGeoref g;
  g.zone = zone;
  g.west_mm = static_cast<std::int64_t>(p.easting * 1000);
  g.south_mm = static_cast<std::int64_t>(p.northing * 1000);
  g.spacing_mm = 1000;
  g.width = 16;
  g.height = 16;
  g.registration = SampleRegistration::Cell;
  auto a = dem.reproject(g);
  auto b = dem.reproject(g);
  REQUIRE(a == b);
  for (int y = 0; y < 16; ++y)
    for (int x = 0; x < 16; ++x) {
      const auto ll =
          utm.reverse(zone, g.sample_easting_m(x), g.sample_northing_m(y));
      REQUIRE(a.heights.at(x, y) ==
              *quantise_height_half_even(10 * ll.lon + 20 * ll.lat - 1100));
    }
  auto shifted = dem.reproject(g, 1.5);
  REQUIRE(shifted.heights.at(0, 0) == a.heights.at(0, 0) + 384);
  REQUIRE_THROWS_AS(dem.reproject(g, std::numeric_limits<double>::infinity()),
                    DecodeError);
  REQUIRE_THROWS_AS(GeographicDem({}, 8), DecodeError);
  REQUIRE_THROWS_AS(GeographicDem({fixture(8, 51, .25, .25)}, 0), DecodeError);
}
