// g2m.src.soil: the raw soil source of a level-2 tile (docs/formats/src_soil.md).
//
// The source is a global lattice of 1/N degree cells (N = 480, about 231 m north-south, SoilGrids 250 m resampled to a
// geographic grid; cell (col, row) covers lon [col/N, (col+1)/N) and lat [row/N, (row+1)/N)). A tile holds the window of
// lattice cells that covers its 1024 m square plus one cell all around, for 42 planes: 7 properties x 6 depth
// intervals, uint16 each, 65535 = no data (the source has no value there). Values are the source's own integers:
//   sand, silt, clay  g/kg (permille of the fine earth)       soc   dg/kg (0.1 g/kg)
//   phh2o             pH * 10                                   cfvo  cm3/dm3 (volume permille of coarse fragments)
//   bdod              cg/cm3 (0.01 g/cm3)
// Depth intervals (cm): 0-5, 5-15, 15-30, 30-60, 60-100, 100-200. Nothing is interpreted here.
#pragma once
#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/tile_container.h"
#include <array>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {
inline constexpr std::uint16_t kSrcSoilEncodingVersion = 1;
inline constexpr int kSoilProps = 7, kSoilDepths = 6, kSoilPlanes = kSoilProps * kSoilDepths;
inline constexpr std::uint16_t kSoilNoData = 65535;
inline constexpr int kSoilMaxWindow = 4096; // cells per side
enum SoilProp : int { SoilSand = 0, SoilSilt = 1, SoilClay = 2, SoilSoc = 3, SoilPh = 4, SoilCfvo = 5, SoilBdod = 6 };
inline constexpr std::array<std::string_view, kSoilProps> kSoilPropNames{"sand", "silt", "clay", "soc", "phh2o", "cfvo", "bdod"};
inline constexpr std::array<std::string_view, kSoilDepths> kSoilDepthNames{"0-5cm", "5-15cm", "15-30cm", "30-60cm", "60-100cm", "100-200cm"};
// Depth interval boundaries in cm: interval d is [bounds[d], bounds[d+1]).
inline constexpr std::array<std::int32_t, kSoilDepths + 1> kSoilDepthBoundsCm{0, 5, 15, 30, 60, 100, 200};
constexpr int soil_plane(int prop, int depth) { return prop * kSoilDepths + depth; }

struct SoilSourceTile {
    TileKey key;                       // level 2
    std::string scheme;                // [a-z0-9_.], 1..64 bytes, e.g. "soilgrids_v2_mean"
    std::int32_t cells_per_degree = 0; // N
    std::int64_t col0 = 0, row0 = 0;   // lattice index of the window's west column and south row
    int width = 0, height = 0;         // cells; row 0 is the south row
    std::vector<std::uint16_t> values; // plane-major: values[plane * width * height + r * width + c]
    std::uint16_t at(int plane, int c, int r) const {
        return values[static_cast<std::size_t>(plane) * static_cast<std::size_t>(width) * static_cast<std::size_t>(height) +
                      static_cast<std::size_t>(r) * static_cast<std::size_t>(width) + static_cast<std::size_t>(c)];
    }
    friend bool operator==(const SoilSourceTile&, const SoilSourceTile&) = default;
};

struct SoilWindow {
    std::int64_t col0 = 0, row0 = 0;
    int width = 0, height = 0;
    friend bool operator==(const SoilWindow&, const SoilWindow&) = default;
};
// The lattice window a level-2 tile stores: the cells touching its square (5 x 5 boundary-and-interior sample points
// through the deterministic UTM reverse projection) plus one cell on every side. Pure.
SoilWindow soil_source_window(const TileKey& key, std::int32_t cells_per_degree);

Result<TileBody> encode_src_soil(const SoilSourceTile& tile);
Result<SoilSourceTile> decode_src_soil(const TileBody& body);

namespace src_soil {
inline constexpr std::uint16_t kMeta = 1;
inline constexpr std::uint16_t kValues = 2;
} // namespace src_soil
} // namespace g2m
