// Morphological ground correction reference: BeamNG.Earth dsmground.py,
// 606ba0f, MIT, (c) 2026 TecnaGamer; THIRD_PARTY_NOTICES.md.
// Adapted for quantised input, explicit provenance, complete stencil support,
// conservative NoData and tiled-vs-whole equality.
#include "g2m/world/dsm_ground.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace g2m::world {
namespace {
int radius(double metres, double spacing) {
  const double n = metres / spacing, f = std::floor(n), r = n - f;
  const int rounded = static_cast<int>(f) +
                      (r > .5 || (r == .5 && std::fmod(f, 2) != 0) ? 1 : 0);
  return (std::max(3, rounded) | 1) / 2;
}
Result<void> valid(const DsmGroundOptions &o) {
  for (double v : {o.sample_m, o.opening_m, o.threshold_m, o.slope_factor,
                   o.max_cut_m, o.relief_m, o.relief_opening_m, o.relief_flat_m,
                   o.relief_hills_m, o.deep_low_m, o.deep_high_m})
    if (!std::isfinite(v) || v < 0 || v > 100000)
      return make_error(ErrorCode::InvalidArgument,
                        "dsm_ground: invalid numeric option");
  if (o.sample_m < .1 || o.opening_m / o.sample_m > 4096 ||
      o.relief_m / o.sample_m > 4096 ||
      o.relief_opening_m / o.sample_m > 4096 ||
      o.relief_hills_m <= o.relief_flat_m || o.deep_high_m <= o.deep_low_m)
    return make_error(ErrorCode::InvalidArgument,
                      "dsm_ground: invalid scale or protection interval");
  return {};
}
template <bool minimum>
HeightRaster extrema_axis(const HeightRaster &a, int r, bool vertical) {
  HeightRaster out(a.width(), a.height());
  const int lines = vertical ? a.width() : a.height(),
            n = vertical ? a.height() : a.width();
  std::vector<int> queue(static_cast<std::size_t>(n));
  for (int line = 0; line < lines; ++line) {
    int head = 0, tail = 0, next = 0;
    const auto get = [&](int i) {
      return vertical ? a.at(line, i) : a.at(i, line);
    };
    for (int i = 0; i < n; ++i) {
      const int end = std::min(n - 1, i + r);
      while (next <= end) {
        const auto v = get(next);
        while (tail > head &&
               (minimum ? get(queue[static_cast<std::size_t>(tail - 1)]) >= v
                        : get(queue[static_cast<std::size_t>(tail - 1)]) <= v))
          --tail;
        queue[static_cast<std::size_t>(tail++)] = next++;
      }
      while (tail > head && queue[static_cast<std::size_t>(head)] < i - r)
        ++head;
      const auto value = get(queue[static_cast<std::size_t>(head)]);
      if (vertical)
        out.at(line, i) = value;
      else
        out.at(i, line) = value;
    }
  }
  return out;
}
template <bool minimum> HeightRaster extrema(const HeightRaster &a, int r) {
  return extrema_axis<minimum>(extrema_axis<minimum>(a, r, false), r, true);
}
HeightRaster opening(const HeightRaster &a, int r) {
  return extrema<false>(extrema<true>(a, r), r);
}
struct Integral {
  int w, h;
  std::vector<std::int64_t> sums;
  explicit Integral(const HeightRaster &a)
      : w(a.width() + 1), h(a.height() + 1),
        sums(static_cast<std::size_t>(w) * static_cast<std::size_t>(h), 0) {
    for (int y = 1; y < h; ++y) {
      std::int64_t row = 0;
      for (int x = 1; x < w; ++x) {
        row += a.at(x - 1, y - 1);
        sums[static_cast<std::size_t>(y * w + x)] =
            sums[static_cast<std::size_t>((y - 1) * w + x)] + row;
      }
    }
  }
  std::int64_t sum(int x0, int y0, int x1, int y1) const {
    const auto at = [&](int x, int y) {
      return sums[static_cast<std::size_t>(y * w + x)];
    };
    return at(x1, y1) - at(x0, y1) - at(x1, y0) + at(x0, y0);
  }
  double mean(int x, int y, int r) const {
    const int x0 = std::max(0, x - r), y0 = std::max(0, y - r),
              x1 = std::min(w - 1, x + r + 1), y1 = std::min(h - 1, y + r + 1);
    return static_cast<double>(sum(x0, y0, x1, y1)) /
           ((x1 - x0) * (y1 - y0) * 256.0);
  }
};
} // namespace
Result<int> dsm_ground_halo(const DsmGroundOptions &o) {
  auto v = valid(o);
  if (!v)
    return v.error();
  const int r = radius(o.opening_m, o.sample_m),
            rr = radius(o.relief_m, o.sample_m),
            rs = radius(o.relief_opening_m, o.sample_m);
  const int mean_r = (std::max(3, ((2 * r + 1) / 2) | 1)) / 2;
  return std::max({3 * r, 2 * r + mean_r + 1, 2 * rs + rr});
}
Result<HeightRaster> correct_dsm_ground(const HeightRaster &context,
                                        const Raster<std::uint8_t> &kinds,
                                        RasterRect core,
                                        const DsmGroundOptions &o) {
  auto halo = dsm_ground_halo(o);
  if (!halo)
    return halo.error();
  const int h = *halo, w = context.width(), height = context.height();
  if (w <= 0 || height <= 0 ||
      static_cast<std::int64_t>(w) * height > 4194304 || kinds.width() != w ||
      kinds.height() != height || core.width <= 0 || core.height <= 0 ||
      core.x < h || core.y < h ||
      static_cast<std::int64_t>(core.x) + core.width + h > w ||
      static_cast<std::int64_t>(core.y) + core.height + h > height)
    return make_error(
        ErrorCode::InvalidArgument,
        "dsm_ground: invalid raster/core or incomplete composed halo");
  HeightRaster out(core.width, core.height);
  bool any_dsm = false;
  for (int y = 0; y < core.height; ++y)
    for (int x = 0; x < core.width; ++x) {
      out.at(x, y) = context.at(core.x + x, core.y + y);
      any_dsm =
          any_dsm || kinds.at(core.x + x, core.y + y) ==
                         static_cast<std::uint8_t>(ElevationSourceKind::Dsm);
    }
  if (!any_dsm)
    return out; // Measured DTM is bit-identical and avoids all filtering work.
  HeightRaster input = context, invalid(w, height);
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < w; ++x) {
      const auto k = kinds.at(x, y);
      if (k > static_cast<std::uint8_t>(ElevationSourceKind::Water))
        return make_error(ErrorCode::InvalidArgument,
                          "dsm_ground: invalid source kind");
      if (input.at(x, y) == kHeightNoData) {
        input.at(x, y) = 0;
        invalid.at(x, y) = 1;
      }
    }
  const Integral invalid_sum(invalid);
  const int r = radius(o.opening_m, o.sample_m),
            rr = radius(o.relief_m, o.sample_m),
            rs = radius(o.relief_opening_m, o.sample_m);
  const int mr = (std::max(3, ((2 * r + 1) / 2) | 1)) / 2;
  const auto opened = opening(input, r), small = opening(input, rs);
  const auto relief_hi = extrema<false>(small, rr),
             relief_lo = extrema<true>(small, rr);
  HeightRaster cuts(w, height);
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < w; ++x) {
      const auto cut =
          static_cast<std::int64_t>(input.at(x, y)) - opened.at(x, y);
      cuts.at(x, y) = static_cast<std::int32_t>(
          std::clamp<std::int64_t>(cut, 0, INT32_MAX));
    }
  const auto deep = extrema<false>(cuts, r);
  const Integral mean(opened);
  for (int y = 0; y < core.height; ++y)
    for (int x = 0; x < core.width; ++x) {
      const int px = core.x + x, py = core.y + y;
      if (kinds.at(px, py) !=
              static_cast<std::uint8_t>(ElevationSourceKind::Dsm) ||
          invalid_sum.sum(px - h, py - h, px + h + 1, py + h + 1) != 0)
        continue;
      const double gx =
          (mean.mean(px + 1, py, mr) - mean.mean(px - 1, py, mr)) /
          (2 * o.sample_m);
      const double gy =
          (mean.mean(px, py + 1, mr) - mean.mean(px, py - 1, mr)) /
          (2 * o.sample_m);
      const double threshold = o.threshold_m + o.slope_factor *
                                                   det::hypot(gx, gy) *
                                                   o.opening_m * .5;
      const double cut = cuts.at(px, py) / 256.0;
      double k =
          std::clamp((cut - threshold) / std::max(o.threshold_m, .5), 0.0, 1.0);
      const double relief = (static_cast<std::int64_t>(relief_hi.at(px, py)) -
                             relief_lo.at(px, py)) /
                            256.0;
      k *= std::clamp((o.relief_hills_m - relief) /
                          (o.relief_hills_m - o.relief_flat_m),
                      0.0, 1.0);
      k *= std::clamp((o.deep_high_m - deep.at(px, py) / 256.0) /
                          (o.deep_high_m - o.deep_low_m),
                      0.0, 1.0);
      const double corrected = dequantise_height(context.at(px, py)) -
                               std::min(cut * k, o.max_cut_m);
      if (const auto value = quantise_height_half_even(corrected))
        out.at(x, y) = *value;
    }
  return out;
}
} // namespace g2m::world
