// Street fidelity P2: server-side carving parameters (data/carve/carve_default.json, design 8.2).
// Smoothing kernel and strength ported from racing_game's terrain smoothing, crown rounding, verge drop, blend
// widths per road class and the bridge deck clearance clamp. The file is hashed ONLY into the g2m.terrain.carved and
// g2m.terrain.class layer configs, never into roads.geom: re-tuning carving must not re-derive profiles (D8, D12).
// Every number is an integer (millimetres, metres or percent) like the road rules. The JSON is embedded at build time.
#pragma once
#include "g2m/core/result.h"
#include "g2m/layer/tile_container.h"
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {
struct CarveSmoothing {
    bool enabled = false;
    std::int32_t radius_m = 0;                 // the kernel has 2 * radius_m + 1 taps at 1 m spacing
    std::vector<std::int32_t> kernel;          // symmetric, positive integer weights
    std::int32_t strength_percent = 0;         // result = h + strength * (smoothed - h)
    friend bool operator==(const CarveSmoothing&, const CarveSmoothing&) = default;
};
struct CarveBlendEntry {
    std::string highway;
    std::int32_t blend_mm = 0;
    friend bool operator==(const CarveBlendEntry&, const CarveBlendEntry&) = default;
};
struct CarveDeck {
    std::int32_t clearance_mm = 0;  // terrain stays this far below the deck top
    std::int32_t ramp_mm = 0;       // smooth-step ramp from the footprint edge inwards
    std::int32_t margin_mm = 0;     // the footprint is the paved band plus this margin
    friend bool operator==(const CarveDeck&, const CarveDeck&) = default;
};
struct CarveParams {
    CarveSmoothing smoothing;
    std::int32_t crown_round_mm = 0;   // r0 of the rounded crown ridge (design 4.1)
    std::int32_t verge_drop_mm = 0;    // drop over the verge band down to the outer edge
    std::vector<CarveBlendEntry> blend; // per highway class, ascending highway value
    std::int32_t blend_default_mm = 0;  // classes without an entry
    std::int32_t blend_slope_factor = 0; // b >= factor * |terrain - road| at the edge (1:1.5 slope limit, design 6.3)
    std::int32_t blend_min_mm = 0, blend_max_mm = 0;
    CarveDeck deck;
    ContentHash content_sha256{};      // SHA-256 of the JSON bytes the parameters were loaded from
    friend bool operator==(const CarveParams&, const CarveParams&) = default;

    // The blend width class default of a highway value (the default entry for an unknown class).
    std::int32_t blend_for_class_mm(std::string_view highway) const;
};

// Strict loader: format "g2m.carve/1", integers only, unknown or missing keys rejected, every highway class of
// the default RoadStyle present in `blend`, blend widths within [blend_min_mm, blend_max_mm] and blend_max_mm
// at most 15 m (the width the junction rules' tile-invariance proof assumes).
Result<CarveParams> load_carve_params(std::span<const std::byte> json);
// The embedded data/carve/carve_default.json, parsed once. A corrupt build aborts.
const CarveParams& builtin_carve_default();
std::span<const std::byte> builtin_carve_default_bytes();
} // namespace g2m
