// g2m.soil.grid: the derived soil grid of a level-2 tile (docs/formats/soil_grid.md, docs/pipeline/soil.md).
//
// The library exposes FACTS: for every cell of a 32 x 32 grid (32 m cells) of the 1024 m tile, two horizons (topsoil
// 0-30 cm, subsoil 30-100 cm; the ranges are data) with sand / silt / clay (permille, sum exactly 1000), a texture class
// (USDA, 12 classes), soil organic carbon, pH, coarse fragments and bulk density, or a reason why the cell has no soil
// (no source value, built-up, water). What the soil MEANS for a crop or the water balance (field capacity, wilting
// point, ...) is the game's: derive it from the texture there.
//
// A cell holds the value of the source lattice cell that contains the cell's CENTRE (nearest, no interpolation), so a
// cell is a function of its position alone: the same position gives the same record whichever tile derives it.
// Per-field aggregates (summarise_field_soil) sample a fixed absolute lattice inside the polygon and read the cells
// through a lookup, so a field has the same summary whichever tiles the lookup is served from.
#pragma once
#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/deriver.h"
#include "g2m/layer/field_tile.h"
#include "g2m/layer/soil_rules.h"
#include "g2m/layer/tile_container.h"
#include <array>
#include <cstdint>
#include <functional>
#include <optional>
#include <string_view>
#include <vector>

namespace g2m {
inline constexpr std::string_view kSoilGridLayer = "g2m.soil.grid";
inline constexpr std::uint16_t kSoilGridEncoding = 12; // LayerEncoding::SoilGrid

enum class SoilState : std::uint8_t {
    Ok = 0,        // soil facts present
    NoSource = 1,  // the source has no value here (its own mask: water, built-up, ice) or the tile is outside the source window
    BuiltUp = 2,   // land cover says built-up (data/soil_rules/default.json "landcover.builtup")
    Water = 3,     // land cover says permanent water
};
inline constexpr std::uint8_t kSoilStateCount = 4;

// One horizon of a cell; all zero unless the cell's state is Ok.
struct SoilHorizon {
    std::uint8_t texture = 0;      // class code of data/soil_rules (USDA: 1 sand, 2 loamy sand, 3 sandy loam, 4 loam, 5 silt loam, 6 silt, 7 sandy clay loam, 8 clay loam, 9 silty clay loam, 10 sandy clay, 11 silty clay, 12 clay)
    std::uint16_t sand = 0, silt = 0, clay = 0; // permille of the fine earth, sum exactly 1000
    std::uint16_t soc = 0;         // soil organic carbon, dg/kg (0.1 g/kg)
    std::uint16_t ph10 = 0;        // pH (H2O) * 10
    std::uint16_t cfvo = 0;        // coarse fragments (> 2 mm), volume permille
    std::uint16_t bdod = 0;        // bulk density of the fine earth, cg/cm3 (0.01 g/cm3)
    friend bool operator==(const SoilHorizon&, const SoilHorizon&) = default;
};
struct SoilSample {
    SoilState state = SoilState::NoSource;
    std::array<SoilHorizon, kSoilHorizons> horizon{};
    friend bool operator==(const SoilSample&, const SoilSample&) = default;
};

struct SoilGridTile {
    TileKey key;                       // level 2
    std::int32_t cells_per_side = 0;
    std::vector<SoilSample> cells;     // cells_per_side^2, row-major, row 0 is the south row
    friend bool operator==(const SoilGridTile&, const SoilGridTile&) = default;
};

Result<TileBody> encode_soil_grid(const SoilGridTile& tile);
Result<SoilGridTile> decode_soil_grid(const TileBody& body);
// Structural rules: a valid level-2 key, cells_per_side divides 1024, state <= 3, a non-Ok cell is all zero, an Ok cell
// has texture codes 1..254, sand+silt+clay == 1000 in both horizons and ranges that fit the source units.
Result<void> validate_soil_grid(const SoilGridTile& tile);

// The cell at UTM millimetres of the tile's zone (half-open cells; nullopt outside the tile).
std::optional<SoilSample> soil_at(const SoilGridTile& tile, std::int64_t easting_mm, std::int64_t northing_mm);

// ---- per-field aggregates ----------------------------------------------------------------------------------
// A lookup answers "the soil record at this UTM position (mm)"; nullopt = not available (tile not resident or outside
// the coverage). The field functions below call it on a fixed absolute lattice.
using SoilLookup = std::function<std::optional<SoilSample>(std::int64_t easting_mm, std::int64_t northing_mm)>;

struct SoilHorizonSummary {
    std::uint8_t dominant_texture = 0;        // most frequent class among the valid samples (ties: lowest code)
    std::uint16_t dominant_share_permille = 0;// its share of the valid samples
    std::uint8_t mean_texture = 0;            // class of the mean sand/silt/clay
    std::uint32_t texture_mask = 0;           // bit (code) set for every class present, codes 1..31
    std::uint16_t sand = 0, silt = 0, clay = 0, soc = 0, ph10 = 0, cfvo = 0, bdod = 0; // means over the valid samples, round half up
    std::uint16_t sand_min = 0, sand_max = 0, clay_min = 0, clay_max = 0;              // the spread (variation within the field)
    friend bool operator==(const SoilHorizonSummary&, const SoilHorizonSummary&) = default;
};
struct SoilFieldSummary {
    std::uint32_t samples = 0;                // lattice points inside the field (or 1 for the anchor fallback)
    std::uint32_t valid = 0;                  // samples whose cell state is Ok
    std::uint32_t missing = 0;                // samples the lookup could not answer; > 0 means the aggregate is incomplete
    std::array<std::uint32_t, kSoilStateCount> state_count{}; // samples per SoilState (missing not counted)
    bool anchor_fallback = false;             // the field holds no lattice point (it is small): the anchor was sampled instead
    std::array<SoilHorizonSummary, kSoilHorizons> horizon{}; // all zero when valid == 0
    friend bool operator==(const SoilFieldSummary&, const SoilFieldSummary&) = default;
};
// Samples the polygon on the absolute lattice (spacing rules.field_lattice_spacing_mm, points at odd multiples of
// half the spacing, boundary inclusive) and aggregates. A pure function of (polygon, rules, the lookup's answers).
SoilFieldSummary summarise_field_soil(const FieldRecord& field, const SoilRules& rules, const SoilLookup& lookup);

// ---- the layer ---------------------------------------------------------------------------------------------
LayerDesc soil_grid_desc();

class SoilGridDeriver final : public IDeriver {
public:
    SoilGridDeriver() : desc_(soil_grid_desc()), coverage_{std::string(kSrcSoilLayer)} {}
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override;
    // Coverage follows the raw soil layer; land cover is optional context (absent = no built-up / water masking).
    std::span<const std::string> coverage_inputs() const override { return coverage_; }
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override;
    Result<DeriveResult> derive(const DeriveRequest& request) const override;
private:
    LayerDesc desc_;
    std::vector<std::string> coverage_;
};
} // namespace g2m
