#pragma once

#include "g2m/core/raster.h"
#include "g2m/core/result.h"
#include "g2m/world/dsm_ground.h" // RasterRect

#include <cstddef>
#include <cstdint>

namespace g2m::world {

struct ElevationBlendOptions {
  // Square-grid sample spacing and the physical radius over which an overlap
  // offset fades to zero. Manhattan distance is used on this grid.
  double spacing_m = 1.0;
  double radius_m = 20.0;
  // Empirical seam correction is clamped to this magnitude. Increase it
  // explicitly when the known source offset is larger.
  double max_adjust_m = 20.0;
  // Bounds all temporary arrays and the priority queue's worst-case work.
  std::size_t max_samples = 16u * 1024u * 1024u;
};

struct ElevationBlendDiagnostics {
  std::uint64_t primary_preserved = 0;
  std::uint64_t fallback_filled = 0;
  std::uint64_t fallback_adjusted = 0;
  std::uint64_t fallback_unadjusted = 0;
  // Fallback fills with no overlap seed inside the configured physical radius.
  std::uint64_t fallback_without_overlap = 0;
  std::uint64_t no_data = 0;
  std::uint64_t overlap_seeds = 0;
};

struct ElevationBlendResult {
  HeightRaster heights;
  ElevationBlendDiagnostics diagnostics;
};

// Source-priority join for rasters on the same sample grid. Every valid
// primary sample is copied unchanged. NoData primary cells use the fallback,
// optionally corrected by the nearest valid primary/fallback overlap delta.
// The correction decays linearly with Manhattan distance and is zero beyond
// radius_m. A full ceil(radius_m / spacing_m) sample halo around core is
// required so tiled results equal whole-grid results. Raster coordinates
// resolve equal-distance seeds lexicographically by row then column.
//
// No overlap means fallback samples are copied unchanged and counted in
// fallback_without_overlap. Missing samples in both sources remain NoData.
// This estimates a local seam offset; it does not convert vertical datums.
Result<ElevationBlendResult>
blend_elevation_sources(const HeightRaster &primary,
                        const HeightRaster &fallback, RasterRect core,
                        const ElevationBlendOptions & = {});

} // namespace g2m::world