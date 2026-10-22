#include "g2m/decode/geographic_dem.h"
#include "g2m/core/geo/utm.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <limits>

namespace g2m::decode {
namespace {
double longitude(double x) {
  return x - 360.0 * std::floor((x + 180.0) / 360.0);
}
double snap(double x) {
  const double n = std::round(x);
  return std::abs(x - n) < 1e-8 ? n : x;
}
std::optional<double> mix(std::optional<double> a, std::optional<double> b,
                          double t) {
  if (t == 0)
    return a;
  if (t == 1)
    return b;
  if (!a || !b)
    return {};
  return *a + (*b - *a) * t;
}
} // namespace
GeographicDem::GeographicDem(std::vector<std::shared_ptr<IByteSource>> sources,
                             std::size_t blocks)
    : capacity_(blocks) {
  if (sources.empty() || sources.size() > 256 || blocks == 0 || blocks > 64)
    throw DecodeError("geographic DEM: invalid source/cache count");
  for (auto &bytes : sources) {
    if (!bytes)
      throw DecodeError("geographic DEM: null byte source");
    Source s;
    s.reader = std::make_unique<tiff::TiffReader>(bytes);
    if (s.reader->ifds().empty())
      throw DecodeError("geographic DEM: no IFD");
    const auto &f = s.reader->ifds()[0];
    if (f.crs.kind != CrsKind::GeographicEpsg || f.crs.epsg != 4326 ||
        !f.geo_transform || !f.geo_transform->valid() ||
        f.sample_format != tiff::SampleFormat::Float ||
        f.bits_per_sample != 32 || f.samples_per_pixel != 1 || f.width < 2 ||
        f.height < 2 || f.width > 100000 || f.height > 100000)
      throw DecodeError("geographic DEM: expected georeferenced EPSG:4326 "
                        "float32 single band");
    s.transform = *f.geo_transform;
    s.width = static_cast<int>(f.width);
    s.height = static_cast<int>(f.height);
    const auto &t = s.transform;
    if (!std::isfinite(t.origin_x) || !std::isfinite(t.origin_y) ||
        !std::isfinite(t.pixel_width) || !std::isfinite(t.pixel_height) ||
        t.origin_x < -180.001 || t.origin_x > 180 || t.origin_y < -90 ||
        t.origin_y > 90.001 || t.pixel_width * s.width > 360 ||
        t.pixel_height * s.height > 180)
      throw DecodeError("geographic DEM: invalid geographic extent");
    s.block_w =
        f.is_tiled()
            ? static_cast<int>(std::min<std::uint64_t>(f.tile_width, 1024))
            : std::min(s.width, 1024);
    s.block_h =
        f.is_tiled()
            ? static_cast<int>(std::min<std::uint64_t>(f.tile_height, 1024))
            : std::min(s.height, 128);
    if (s.block_w <= 0 || s.block_h <= 0)
      throw DecodeError("geographic DEM: invalid block shape");
    if (f.gdal_nodata) {
      float v = 0;
      const auto &text = *f.gdal_nodata;
      const auto parsed =
          std::from_chars(text.data(), text.data() + text.size(), v);
      if (parsed.ec != std::errc{} || parsed.ptr != text.data() + text.size())
        throw DecodeError("geographic DEM: invalid NoData value");
      s.nodata = v;
    }
    sources_.push_back(std::move(s));
  }
}
std::optional<std::size_t> GeographicDem::owner(double lat, double lon) const {
  lon = longitude(lon);
  for (std::size_t i = 0; i < sources_.size(); ++i) {
    const auto &s = sources_[i];
    const auto &t = s.transform;
    const double x = snap((lon - t.origin_x) / t.pixel_width);
    const double y = snap((t.origin_y - lat) / t.pixel_height);
    if (x >= 0 && x < s.width && y >= 0 && y < s.height)
      return i;
  }
  return {};
}
std::optional<double> GeographicDem::pixel(std::size_t id, int x, int y) {
  auto &s = sources_[id];
  if (x < 0 || y < 0 || x >= s.width || y >= s.height)
    return {};
  const int bx = x / s.block_w * s.block_w, by = y / s.block_h * s.block_h;
  auto found = std::find_if(cache_.begin(), cache_.end(), [&](const Block &b) {
    return b.source == id && b.x == bx && b.y == by;
  });
  if (found == cache_.end()) {
    auto raster = s.reader->decode_window_f32(
        0, bx, by, std::min(s.block_w, s.width - bx),
        std::min(s.block_h, s.height - by));
    cache_.push_front({id, bx, by, std::move(raster)});
    ++decoded_blocks_;
    if (cache_.size() > capacity_)
      cache_.pop_back();
  } else
    cache_.splice(cache_.begin(), cache_, found);
  const float v = cache_.front().samples.at(x - bx, y - by);
  if (!std::isfinite(v) || (s.nodata && v == *s.nodata))
    return {};
  return v;
}
std::optional<double> GeographicDem::row(double lat, double lon) {
  lon = longitude(lon);
  const auto id = owner(lat, lon);
  if (!id)
    return {};
  const auto &s = sources_[*id];
  const auto &t = s.transform;
  const double fy = snap((t.origin_y - lat) / t.pixel_height);
  // Adjacent latitude bands may change longitude spacing, but their rows
  // must coincide. Refuse interpolation across an unsupported row lattice.
  if (std::abs(fy - std::round(fy)) > 1e-6)
    return {};
  const int y = static_cast<int>(std::round(fy));
  const double fx = snap((lon - t.origin_x) / t.pixel_width);
  const int x = static_cast<int>(std::floor(fx));
  const auto a = pixel(*id, x, y);
  const double fraction = fx - x;
  if (fraction == 0)
    return a;
  if (x + 1 < s.width)
    return mix(a, pixel(*id, x + 1, y), fraction);
  const double east = longitude(t.origin_x + (x + 1) * t.pixel_width);
  const auto next = owner(lat, east);
  if (!next)
    return {};
  const auto &nt = sources_[*next].transform;
  const double nx = snap((east - nt.origin_x) / nt.pixel_width),
               ny = snap((nt.origin_y - lat) / nt.pixel_height);
  if (std::abs(nx - std::round(nx)) > 1e-6 ||
      std::abs(ny - std::round(ny)) > 1e-6)
    return {};
  return mix(a,
             pixel(*next, static_cast<int>(std::round(nx)),
                   static_cast<int>(std::round(ny))),
             fraction);
}
std::optional<double> GeographicDem::sample(double lat, double lon) {
  if (!std::isfinite(lat) || !std::isfinite(lon) || lat < -90 || lat > 90 ||
      std::abs(lon) > 540)
    return {};
  lon = longitude(lon);
  const auto id = owner(lat, lon);
  if (!id)
    return {};
  const auto &t = sources_[*id].transform;
  const double fy = snap((t.origin_y - lat) / t.pixel_height),
               r = std::floor(fy);
  const double north = t.origin_y - r * t.pixel_height;
  const auto a = row(north, lon);
  if (fy == r)
    return a;
  return mix(a, row(north - t.pixel_height, lon), fy - r);
}
ElevTile GeographicDem::reproject(const RasterGeoref &g, double shift) {
  if (!g.valid() || g.width <= 0 || g.height <= 0 ||
      static_cast<std::uint64_t>(g.width) *
              static_cast<std::uint64_t>(g.height) >
          4194304 ||
      !std::isfinite(shift))
    throw DecodeError("geographic DEM: invalid/beyond-budget output grid");
  ElevTile out{g, HeightRaster(g.width, g.height, kHeightNoData)};
  const geo::Utm utm;
  for (int y = 0; y < g.height; ++y)
    for (int x = 0; x < g.width; ++x) {
      const auto ll =
          utm.reverse(g.zone, g.sample_easting_m(x), g.sample_northing_m(y));
      const auto h = sample(ll.lat, ll.lon);
      if (!h)
        continue;
      const auto q = quantise_height_half_even(*h + shift);
      if (!q)
        throw DecodeError("geographic DEM: height out of range");
      out.heights.at(x, y) = *q;
    }
  return out;
}
} // namespace g2m::decode
