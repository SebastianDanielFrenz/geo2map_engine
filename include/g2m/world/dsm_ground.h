#pragma once
#include "g2m/core/raster.h"
#include "g2m/core/result.h"
#include <cstdint>
namespace g2m::world {
enum class ElevationSourceKind : std::uint8_t { Unknown, Dtm, Dsm, Water };
struct DsmGroundOptions {
  double sample_m = 30;
  double opening_m = 400, threshold_m = 2, slope_factor = .35, max_cut_m = 45;
  double relief_m = 600, relief_opening_m = 150;
  double relief_flat_m = 40, relief_hills_m = 90;
  double deep_low_m = 35, deep_high_m = 55;
};
struct RasterRect {
  int x = 0, y = 0, width = 0, height = 0;
};
// Complete composed input support: opening, mean/gradient, local relief and
// deep-cut veto. In sample units, not a copied provider-specific pixel halo.
Result<int> dsm_ground_halo(const DsmGroundOptions &);
// Offline/import primitive. Explicit DTM/water samples and unknown provenance
// are passed through. Only labelled DSM samples can be lowered. Any NoData in
// the full dependency neighbourhood suppresses inference for that output pixel.
// Core must have the complete halo; no edge replication disguised as real data.
// Uses integer extrema/integral sums, specified double operations and half-even
// final quantisation. No provider fetching or implicit datum conversion.
Result<HeightRaster> correct_dsm_ground(const HeightRaster &context,
                                        const Raster<std::uint8_t> &source_kind,
                                        RasterRect core,
                                        const DsmGroundOptions & = {});
} // namespace g2m::world
