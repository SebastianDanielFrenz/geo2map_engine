#include "g2m/world/elevation_ground.h"
#include "g2m/core/geo/utm.h"
#include <cmath>

namespace g2m::world {
Result<GroundElevationResult> prepare_ground_elevation(
    const RasterGeoref &target, const GeographicElevationSampler &sample,
    ElevationSourceKind kind, const GroundElevationOptions &options) {
  if (!target.valid() || target.registration != SampleRegistration::Cell ||
      target.width <= 0 || target.height <= 0 ||
      static_cast<std::uint64_t>(target.width) * target.height > 4194304 ||
      !sample ||
      static_cast<unsigned>(kind) >
          static_cast<unsigned>(ElevationSourceKind::Water))
    return make_error(ErrorCode::InvalidArgument,
                      "ground elevation: invalid grid/source");
  GroundElevationResult out;
  out.tile = {target, HeightRaster(target.width, target.height, kHeightNoData)};
  const geo::Utm utm;
  auto quantised = [&](double e, double n) -> Result<std::int32_t> {
    const auto ll = utm.reverse(target.zone, e, n);
    const auto h = sample(ll.lat, ll.lon);
    if (!h)
      return kHeightNoData;
    const auto q = quantise_height_half_even(*h);
    if (!q)
      return make_error(ErrorCode::Malformed,
                        "ground elevation: invalid source height");
    return *q;
  };
  if (kind != ElevationSourceKind::Dsm) {
    for (int y = 0; y < target.height; ++y)
      for (int x = 0; x < target.width; ++x) {
        const auto q =
            quantised(target.sample_easting_m(x), target.sample_northing_m(y));
        if (!q)
          return q.error();
        out.tile.heights.at(x, y) = *q;
      }
    return out;
  }
  const int step = options.spacing_m;
  if (step < 4 || step > 128 || (step & (step - 1)) != 0)
    return make_error(
        ErrorCode::InvalidArgument,
        "ground elevation: coarse spacing must be power of two, 4-128m");
  const std::int64_t mm = static_cast<std::int64_t>(step) * 1000;
  if (target.spacing_mm > mm || mm % target.spacing_mm != 0 ||
      target.west_mm % mm != 0 || target.south_mm % mm != 0 ||
      (static_cast<std::int64_t>(target.width) * target.spacing_mm) % mm != 0 ||
      (static_cast<std::int64_t>(target.height) * target.spacing_mm) % mm != 0)
    return make_error(
        ErrorCode::InvalidArgument,
        "ground elevation: target must align to the global coarse lattice");
  auto ground = options.ground;
  ground.sample_m = step;
  const auto h = dsm_ground_halo(ground);
  if (!h)
    return h.error();
  const auto nx =
      static_cast<std::int64_t>(target.width) * target.spacing_mm / mm + 2;
  const auto ny =
      static_cast<std::int64_t>(target.height) * target.spacing_mm / mm + 2;
  const auto cw = nx + 2 * (*h), ch = ny + 2 * (*h);
  if (cw <= 0 || ch <= 0 || cw * ch > 4194304)
    return make_error(ErrorCode::InvalidArgument,
                      "ground elevation: source context exceeds sample budget");
  RasterGeoref context = target;
  context.west_mm -= static_cast<std::int64_t>(*h + 1) * mm;
  context.south_mm -= static_cast<std::int64_t>(*h + 1) * mm;
  context.spacing_mm = mm;
  context.row_order = RowOrder::SouthToNorth;
  context.width = static_cast<int>(cw);
  context.height = static_cast<int>(ch);
  HeightRaster heights(context.width, context.height, kHeightNoData);
  for (int y = 0; y < context.height; ++y)
    for (int x = 0; x < context.width; ++x) {
      const auto q =
          quantised(context.sample_easting_m(x), context.sample_northing_m(y));
      if (!q)
        return q.error();
      heights.at(x, y) = *q;
    }
  Raster<std::uint8_t> kinds(context.width, context.height,
                             static_cast<std::uint8_t>(kind));
  const auto filtered = correct_dsm_ground(
      heights, kinds, {*h, *h, static_cast<int>(nx), static_cast<int>(ny)},
      ground);
  if (!filtered)
    return filtered.error();
  for (int y = 0; y < filtered->height(); ++y)
    for (int x = 0; x < filtered->width(); ++x)
      if (filtered->at(x, y) != kHeightNoData &&
          filtered->at(x, y) < heights.at(x + *h, y + *h))
        ++out.coarse_lowered;
  const double west = target.west_mm / 1000.0, south = target.south_mm / 1000.0;
  for (int y = 0; y < target.height; ++y)
    for (int x = 0; x < target.width; ++x) {
      const double e = target.sample_easting_m(x),
                   n = target.sample_northing_m(y);
      const double u = (e - west + step * .5) / step,
                   v = (n - south + step * .5) / step;
      const int i = static_cast<int>(std::floor(u)),
                j = static_cast<int>(std::floor(v));
      const auto a = filtered->at(i, j), b = filtered->at(i + 1, j),
                 c = filtered->at(i, j + 1), d = filtered->at(i + 1, j + 1);
      if (a == kHeightNoData || b == kHeightNoData || c == kHeightNoData ||
          d == kHeightNoData) {
        const auto raw = quantised(e, n);
        if (!raw)
          return raw.error();
        out.tile.heights.at(x, y) = *raw;
        ++out.raw_fallback_samples;
        continue;
      }
      const double fx = u - i, fy = v - j;
      const double low =
          static_cast<double>(a) + (static_cast<double>(b) - a) * fx;
      const double high =
          static_cast<double>(c) + (static_cast<double>(d) - c) * fx;
      const auto q = quantise_height_half_even((low + (high - low) * fy) /
                                               kHeightUnitsPerMetre);
      if (!q)
        return make_error(ErrorCode::Malformed,
                          "ground elevation: interpolated height out of range");
      out.tile.heights.at(x, y) = *q;
    }
  return out;
}
} // namespace g2m::world
