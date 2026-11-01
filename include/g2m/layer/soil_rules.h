// Soil layer rules as DATA (data/soil_rules/default.json): how the derived soil grid reads the raw soil source
// (cell size, horizons as depth ranges, no-data rule, land cover classes that are not soil) and how a sand / silt /
// clay triple becomes a texture class (ordered clauses over integer permille, so no float and no libm). The JSON is
// embedded at build time; its SHA-256 is part of the g2m.soil.grid config hash, so editing it changes the release id.
#pragma once
#include "g2m/core/result.h"
#include "g2m/layer/tile_container.h"
#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <vector>

namespace g2m {
// One comparison on the permille triple: w[0]*sand + w[1]*silt + w[2]*clay `op` k.
enum class SoilCmp : std::uint8_t { Lt = 0, Le = 1, Gt = 2, Ge = 3 };
struct SoilClause {
    std::array<std::int32_t, 3> w{};
    SoilCmp op = SoilCmp::Lt;
    std::int64_t k = 0;
    friend bool operator==(const SoilClause&, const SoilClause&) = default;
};
struct SoilTextureClass {
    std::uint8_t code = 0;   // 1..255, stored in the tile (append-only); 0 means "no texture"
    std::string name;
    // The class matches when ANY alternative has ALL its clauses true. The first matching class in file order wins.
    std::vector<std::vector<SoilClause>> alternatives;
    friend bool operator==(const SoilTextureClass&, const SoilTextureClass&) = default;
};
// A horizon is a depth range in cm; the tile stores the thickness-weighted mean of the source depth intervals it overlaps.
struct SoilHorizonRule {
    std::string name;
    std::int32_t from_cm = 0, to_cm = 0;
    friend bool operator==(const SoilHorizonRule&, const SoilHorizonRule&) = default;
};
inline constexpr std::size_t kSoilHorizons = 2; // horizon 0 = topsoil, 1 = subsoil (the layout of the tile; the ranges are data)

struct SoilRules {
    std::string source_scheme;               // the g2m.src.soil scheme the rules were written for
    std::int32_t cells_per_degree = 0;       // the source lattice (1/N degree cells)
    std::int32_t cells_per_side = 0;         // derived grid: cells per tile edge (1024 m / cells_per_side per cell)
    bool surface_texture_sum_zero_is_nodata = false;
    std::string landcover_scheme;
    std::vector<std::uint8_t> landcover_builtup, landcover_water; // sorted
    std::array<SoilHorizonRule, kSoilHorizons> horizons;
    std::string texture_scheme;
    std::vector<SoilTextureClass> texture;   // file order = match order
    std::int32_t field_lattice_spacing_mm = 0;
    ContentHash content_sha256{};

    // Class code of a permille triple (renormalised to sum 1000 first); 0 when nothing matches or all three are 0.
    std::uint8_t texture_code(std::int32_t sand, std::int32_t silt, std::int32_t clay) const;
    const SoilTextureClass* texture_by_code(std::uint8_t code) const;
    friend bool operator==(const SoilRules&, const SoilRules&) = default;
};

// Renormalises a sand/silt/clay permille triple to sum exactly 1000 (floor for sand and silt, clay takes the remainder,
// at most 2 permille); an all-zero or negative total gives 0/0/0. Integer only.
void soil_normalise_texture(std::int32_t& sand, std::int32_t& silt, std::int32_t& clay);

Result<SoilRules> load_soil_rules(std::span<const std::byte> json);
const SoilRules& builtin_soil_rules();
std::span<const std::byte> builtin_soil_rules_bytes();
} // namespace g2m
