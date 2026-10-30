#include "g2m/decode/terrarium_dem.h"

#include "g2m/core/det_math.h"
#include "g2m/core/geo/utm.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>
#include <utility>

namespace g2m::decode {
namespace {
constexpr double kPi = 3.141592653589793238462643383279502884;
constexpr int kMaxZoom = 30;
constexpr int kMaxTileSize = 4096;
constexpr std::uint64_t kMaxOutputSamples = 4194304;

bool tile_less(const TerrariumTile &a, const TerrariumTile &b) {
  return std::tie(a.z, a.y, a.x) < std::tie(b.z, b.y, b.x);
}

std::int64_t positive_mod(std::int64_t value, std::int64_t modulus) {
  const std::int64_t r = value % modulus;
  return r < 0 ? r + modulus : r;
}

double normalise_longitude(double lon) {
  return lon - 360.0 * std::floor((lon + 180.0) / 360.0);
}

double snap_pixel_coordinate(double coordinate) {
  const double nearest = std::round(coordinate);
  return std::abs(coordinate - nearest) < 1e-8 ? nearest : coordinate;
}

std::optional<double> mix(std::optional<double> a, std::optional<double> b,
                          double t) {
  if (t == 0.0)
    return a;
  if (t == 1.0)
    return b;
  if (!a || !b)
    return {};
  return *a + (*b - *a) * t;
}
} // namespace

TerrariumDem::TerrariumDem(std::vector<TerrariumTile> tiles,
                           TerrariumSourceBounds bounds,
                           std::size_t memory_limit_bytes, int tile_size)
    : tiles_(std::move(tiles)), bounds_(bounds), tile_size_(tile_size) {
  if (bounds_.z < 0 || bounds_.z > kMaxZoom || tile_size_ <= 0 ||
      tile_size_ > kMaxTileSize || memory_limit_bytes == 0 || tiles_.empty())
    throw DecodeError(
        "Terrarium DEM: invalid zoom, tile size, memory limit, or empty input");

  world_tiles_ = 1 << bounds_.z;
  if (bounds_.min_x < 0 || bounds_.min_x >= world_tiles_ || bounds_.max_x < 0 ||
      bounds_.max_x >= world_tiles_ || bounds_.min_y < 0 ||
      bounds_.max_y < bounds_.min_y || bounds_.max_y >= world_tiles_)
    throw DecodeError("Terrarium DEM: invalid XYZ source bounds");

  const std::size_t side = static_cast<std::size_t>(tile_size_);
  const std::size_t rgba_per_tile = side * side * 4;
  if (rgba_per_tile / 4 / side != side ||
      tiles_.size() > memory_limit_bytes / rgba_per_tile)
    throw DecodeError("Terrarium DEM: input exceeds memory limit");

  memory_bytes_ = tiles_.size() * rgba_per_tile;
  for (const auto &tile : tiles_) {
    if (tile.z != bounds_.z || tile.x < 0 || tile.x >= world_tiles_ ||
        tile.y < bounds_.min_y || tile.y > bounds_.max_y ||
        !x_in_bounds(tile.x) || tile.rgba.size() != rgba_per_tile)
      throw DecodeError(
          "Terrarium DEM: tile key or RGBA dimensions outside source bounds");
  }

  std::sort(tiles_.begin(), tiles_.end(), tile_less);
  for (std::size_t i = 1; i < tiles_.size(); ++i)
    if (!tile_less(tiles_[i - 1], tiles_[i]))
      throw DecodeError("Terrarium DEM: duplicate tile key");
}

bool TerrariumDem::x_in_bounds(int x) const {
  if (bounds_.min_x <= bounds_.max_x)
    return x >= bounds_.min_x && x <= bounds_.max_x;
  return x >= bounds_.min_x || x <= bounds_.max_x;
}

std::optional<double> TerrariumDem::pixel(std::int64_t global_x,
                                          std::int64_t global_y) const {
  const std::int64_t world_pixels =
      static_cast<std::int64_t>(world_tiles_) * tile_size_;
  if (global_y < 0 || global_y >= world_pixels)
    return {};
  global_x = positive_mod(global_x, world_pixels);
  const int tile_x = static_cast<int>(global_x / tile_size_);
  const int tile_y = static_cast<int>(global_y / tile_size_);
  if (!x_in_bounds(tile_x) || tile_y < bounds_.min_y || tile_y > bounds_.max_y)
    return {};

  TerrariumTile key;
  key.z = bounds_.z;
  key.x = tile_x;
  key.y = tile_y;
  const auto it =
      std::lower_bound(tiles_.begin(), tiles_.end(), key, tile_less);
  if (it == tiles_.end() || it->z != bounds_.z || it->x != tile_x ||
      it->y != tile_y)
    return {};

  const int col = static_cast<int>(global_x % tile_size_);
  const int row = static_cast<int>(global_y % tile_size_);
  const std::size_t offset =
      (static_cast<std::size_t>(row) * tile_size_ + col) * 4;
  const auto *rgba = it->rgba.data() + offset;
  if (rgba[3] == 0)
    return {};
  return 256.0 * rgba[0] + rgba[1] + rgba[2] / 256.0 - 32768.0;
}

std::optional<double> TerrariumDem::sample(double latitude,
                                           double longitude) const {
  if (!std::isfinite(latitude) || !std::isfinite(longitude) ||
      latitude < -90.0 || latitude > 90.0 || std::abs(longitude) > 540.0)
    return {};

  const double max_latitude = g2m::det::atan(g2m::det::sinh(kPi)) * 180.0 / kPi;
  if (latitude < -max_latitude || latitude > max_latitude)
    return {};
  longitude = normalise_longitude(longitude);
  const double world_pixels = static_cast<double>(world_tiles_) * tile_size_;
  const double x =
      snap_pixel_coordinate((longitude + 180.0) / 360.0 * world_pixels - 0.5);
  const double latitude_rad = latitude * kPi / 180.0;
  const double mercator_y = g2m::det::asinh(g2m::det::tan(latitude_rad));
  const double y = snap_pixel_coordinate(
      (1.0 - mercator_y / kPi) * 0.5 * world_pixels - 0.5);
  if (!std::isfinite(x) || !std::isfinite(y) || y < 0.0 ||
      y > world_pixels - 1.0)
    return {};

  const auto x0 = static_cast<std::int64_t>(std::floor(x));
  const auto y0 = static_cast<std::int64_t>(std::floor(y));
  const double fx = x - static_cast<double>(x0);
  const double fy = y - static_cast<double>(y0);
  const auto north = mix(pixel(x0, y0), pixel(x0 + 1, y0), fx);
  const auto south = mix(pixel(x0, y0 + 1), pixel(x0 + 1, y0 + 1), fx);
  return mix(north, south, fy);
}

ElevTile TerrariumDem::reproject(const RasterGeoref &georef,
                                 double height_shift_m) const {
  if (!georef.valid() || georef.width <= 0 || georef.height <= 0 ||
      static_cast<std::uint64_t>(georef.width) *
              static_cast<std::uint64_t>(georef.height) >
          kMaxOutputSamples ||
      !std::isfinite(height_shift_m))
    throw DecodeError("Terrarium DEM: invalid or over-budget output grid");

  ElevTile out{georef,
               HeightRaster(georef.width, georef.height, kHeightNoData)};
  const geo::Utm utm;
  for (int y = 0; y < georef.height; ++y) {
    for (int x = 0; x < georef.width; ++x) {
      const auto ll = utm.reverse(georef.zone, georef.sample_easting_m(x),
                                  georef.sample_northing_m(y));
      const auto height = sample(ll.lat, ll.lon);
      if (!height)
        continue;
      const auto quantised =
          quantise_height_half_even(*height + height_shift_m);
      if (!quantised)
        throw DecodeError("Terrarium DEM: height outside encoded range");
      out.heights.at(x, y) = *quantised;
    }
  }
  return out;
}

} // namespace g2m::decode
