#pragma once
#include "g2m/layer/src_elev.h"
#include "g2m/world/dsm_ground.h"
#include <functional>
#include <optional>

namespace g2m::world {
using GeographicElevationSampler =
    std::function<std::optional<double>(double lat, double lon)>;
struct GroundElevationOptions {
  int spacing_m = 32;
  DsmGroundOptions ground;
};
struct GroundElevationResult {
  ElevTile tile;
  std::size_t coarse_lowered = 0;
  std::size_t raw_fallback_samples = 0;
};
// Offline source preparation. DSM morphology is applied on a globally aligned
// metric coarse lattice, with its COMPLETE dependency halo, before upsampling.
// Primary measured DTM and unknown source types use the original sampler and
// are not downsampled/filtered. Callback decoding exceptions propagate.
Result<GroundElevationResult> prepare_ground_elevation(
    const RasterGeoref &target, const GeographicElevationSampler &,
    ElevationSourceKind, const GroundElevationOptions & = {});
} // namespace g2m::world
