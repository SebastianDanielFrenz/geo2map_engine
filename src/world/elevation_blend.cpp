#include "g2m/world/elevation_blend.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <tuple>
#include <vector>

namespace g2m::world {
namespace {

struct SeedVisit {
  int distance = 0;
  std::size_t seed = 0;
  std::size_t cell = 0;
};
struct VisitLater {
  bool operator()(const SeedVisit &a, const SeedVisit &b) const {
    return std::tie(a.distance, a.seed, a.cell) >
           std::tie(b.distance, b.seed, b.cell);
  }
};

Result<int> radius_samples(const ElevationBlendOptions &o) {
  if (!std::isfinite(o.spacing_m) || o.spacing_m <= 0 ||
      !std::isfinite(o.radius_m) || o.radius_m < 0 ||
      !std::isfinite(o.max_adjust_m) || o.max_adjust_m < 0 ||
      o.max_samples == 0)
    return make_error(ErrorCode::InvalidArgument,
                      "elevation_blend: invalid numeric option");
  const double samples = std::ceil(o.radius_m / o.spacing_m);
  if (!std::isfinite(samples) || samples > 1'000'000)
    return make_error(ErrorCode::InvalidArgument,
                      "elevation_blend: radius exceeds supported sample range");
  return static_cast<int>(samples);
}

bool valid_core(RasterRect core, int width, int height, int halo) {
  if (core.x < 0 || core.y < 0 || core.width <= 0 || core.height <= 0 ||
      core.x > width - core.width || core.y > height - core.height)
    return false;
  return core.x >= halo && core.y >= halo &&
         width - (core.x + core.width) >= halo &&
         height - (core.y + core.height) >= halo;
}

} // namespace

Result<ElevationBlendResult>
blend_elevation_sources(const HeightRaster &primary,
                        const HeightRaster &fallback, RasterRect core,
                        const ElevationBlendOptions &o) {
  auto radius_result = radius_samples(o);
  if (!radius_result)
    return radius_result.error();
  const int radius = radius_result.value();
  if (primary.width() <= 0 || primary.height() <= 0 ||
      primary.width() != fallback.width() ||
      primary.height() != fallback.height())
    return make_error(ErrorCode::InvalidArgument,
                      "elevation_blend: source grids differ or are empty");
  const auto w = static_cast<std::size_t>(primary.width());
  const auto h = static_cast<std::size_t>(primary.height());
  if (w > std::numeric_limits<std::size_t>::max() / h || w * h > o.max_samples)
    return make_error(ErrorCode::LimitExceeded,
                      "elevation_blend: raster exceeds sample budget");
  if (!valid_core(core, primary.width(), primary.height(), radius))
    return make_error(
        ErrorCode::InvalidArgument,
        "elevation_blend: core is invalid or lacks full radius halo");
  const auto max_adjust = quantise_height_half_even(o.max_adjust_m);
  if (!max_adjust || *max_adjust < 0)
    return make_error(ErrorCode::InvalidArgument,
                      "elevation_blend: adjustment limit is out of range");

  const std::size_t count = w * h;
  bool core_has_only_primary = true;
  for (int y = 0; y < core.height && core_has_only_primary; ++y) {
    for (int x = 0; x < core.width; ++x) {
      const auto i = static_cast<std::size_t>(core.y + y) * w +
                     static_cast<std::size_t>(core.x + x);
      if (primary.data()[i] == kHeightNoData) {
        core_has_only_primary = false;
        break;
      }
    }
  }
  if (core_has_only_primary) {
    ElevationBlendResult result{HeightRaster(core.width, core.height), {}};
    for (std::size_t i = 0; i < count; ++i)
      if (primary.data()[i] != kHeightNoData &&
          fallback.data()[i] != kHeightNoData)
        ++result.diagnostics.overlap_seeds;
    for (int y = 0; y < core.height; ++y) {
      for (int x = 0; x < core.width; ++x) {
        result.heights.at(x, y) = primary.at(core.x + x, core.y + y);
        ++result.diagnostics.primary_preserved;
      }
    }
    return result;
  }
  const int unreachable = std::numeric_limits<int>::max();
  std::vector<int> distances(count, unreachable);
  std::vector<std::size_t> nearest_seed(
      count, std::numeric_limits<std::size_t>::max());
  std::priority_queue<SeedVisit, std::vector<SeedVisit>, VisitLater> queue;
  ElevationBlendDiagnostics diagnostics;
  for (std::size_t i = 0; i < count; ++i) {
    if (primary.data()[i] != kHeightNoData &&
        fallback.data()[i] != kHeightNoData) {
      distances[i] = 0;
      nearest_seed[i] = i;
      queue.push({0, i, i});
      ++diagnostics.overlap_seeds;
    }
  }

  const auto visit = [&](std::size_t from, std::size_t to, int next_distance,
                         auto &pending) {
    if (next_distance > radius)
      return;
    if (next_distance < distances[to] ||
        (next_distance == distances[to] &&
         nearest_seed[from] < nearest_seed[to])) {
      distances[to] = next_distance;
      nearest_seed[to] = nearest_seed[from];
      pending.push({next_distance, nearest_seed[to], to});
    }
  };
  while (!queue.empty()) {
    const auto current = queue.top();
    queue.pop();
    if (distances[current.cell] != current.distance ||
        nearest_seed[current.cell] != current.seed ||
        current.distance >= radius)
      continue;
    const int x = static_cast<int>(current.cell % w);
    const int y = static_cast<int>(current.cell / w);
    if (x > 0)
      visit(current.cell, current.cell - 1, current.distance + 1, queue);
    if (x + 1 < primary.width())
      visit(current.cell, current.cell + 1, current.distance + 1, queue);
    if (y > 0)
      visit(current.cell, current.cell - w, current.distance + 1, queue);
    if (y + 1 < primary.height())
      visit(current.cell, current.cell + w, current.distance + 1, queue);
  }

  ElevationBlendResult result{HeightRaster(core.width, core.height), {}};
  result.diagnostics = diagnostics;

  for (int y = 0; y < core.height; ++y) {
    for (int x = 0; x < core.width; ++x) {
      const int sx = core.x + x, sy = core.y + y;
      const std::size_t i =
          static_cast<std::size_t>(sy) * w + static_cast<std::size_t>(sx);
      const auto p = primary.data()[i];
      const auto f = fallback.data()[i];
      if (p != kHeightNoData) {
        result.heights.at(x, y) = p;
        ++result.diagnostics.primary_preserved;
      } else if (f == kHeightNoData) {
        result.heights.at(x, y) = kHeightNoData;
        ++result.diagnostics.no_data;
      } else {
        std::int64_t value = f;
        const double distance_m =
            static_cast<double>(distances[i]) * o.spacing_m;
        const bool overlap_in_support =
            nearest_seed[i] != std::numeric_limits<std::size_t>::max() &&
            distances[i] <= radius && distance_m <= o.radius_m;
        if (radius > 0 && overlap_in_support && distance_m < o.radius_m) {
          const std::size_t seed = nearest_seed[i];
          const std::int64_t raw_delta =
              static_cast<std::int64_t>(primary.data()[seed]) -
              static_cast<std::int64_t>(fallback.data()[seed]);
          const auto limit = static_cast<std::int64_t>(*max_adjust);
          const std::int64_t delta = std::clamp(raw_delta, -limit, limit);
          const double remaining_fraction =
              (o.radius_m - distance_m) / o.radius_m;
          const auto adjustment_units = quantise_height_half_even(
              (static_cast<double>(delta) / kHeightUnitsPerMetre) *
              remaining_fraction);
          if (!adjustment_units)
            return make_error(ErrorCode::Malformed,
                              "elevation_blend: correction is out of range");
          value += *adjustment_units;
          if (*adjustment_units != 0)
            ++result.diagnostics.fallback_adjusted;
          else
            ++result.diagnostics.fallback_unadjusted;
        } else {
          ++result.diagnostics.fallback_unadjusted;
        }
        if (value <= kHeightNoData ||
            value > std::numeric_limits<std::int32_t>::max())
          return make_error(
              ErrorCode::Malformed,
              "elevation_blend: adjusted height exceeds int32 range");
        result.heights.at(x, y) = static_cast<std::int32_t>(value);
        ++result.diagnostics.fallback_filled;
        if (!overlap_in_support)
          ++result.diagnostics.fallback_without_overlap;
      }
    }
  }
  return result;
}

} // namespace g2m::world