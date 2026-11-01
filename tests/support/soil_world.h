// tests/support/soil_world.h - a synthetic soil world for the soil layer's tests, goldens and fuzz seeds.
//
// The world is a PURE FUNCTION of the source lattice cell (col, row of the 1/480 degree lattice): raw_value() gives
// every one of the 42 source planes, texture regions of 2 x 2 lattice cells cycling through the twelve USDA class
// representatives plus a small hash jitter, with three kinds of no-data patch (all planes missing, surface texture all
// zero, one deep plane missing). Land cover is a pure function of the 4 m UTM cell (water and built-up blocks).
// Tiles (source soil, land cover) are filled from these functions, and ref_cell() evaluates ONE 32 m cell of a derived
// tile straight from them with its own arithmetic (explicit horizon weights, its own texture matcher), so a derived
// tile can be compared with a direct evaluation at any position, whichever tile it came from.
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/core/hash.h"
#include "g2m/core/raster.h"
#include "g2m/layer/soil_rules.h"
#include "g2m/layer/soil_tile.h"
#include "g2m/layer/src_landcover.h"
#include "g2m/layer/src_soil.h"
#include "g2m/layer/tile_container.h"
#include "g2m/layer/deriver.h"

#include <array>
#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace g2m_test::soil {
using namespace g2m;

inline std::uint64_t mix(std::uint64_t a, std::uint64_t b) {
    std::uint64_t z = a * 0x9E3779B97F4A7C15ull + b * 0xC2B2AE3D27D4EB4Full + 0x165667B19E3779F9ull;
    z ^= z >> 30; z *= 0xBF58476D1CE4E5B9ull; z ^= z >> 27; z *= 0x94D049BB133111EBull; z ^= z >> 31;
    return z;
}

// sand / silt / clay (permille) of a typical member of each USDA class 1..12 (data/soil_rules/default.json order).
inline constexpr int kReps[12][3] = {{920, 50, 30}, {820, 100, 80}, {650, 200, 150}, {400, 400, 200}, {200, 650, 150}, {50, 900, 50},
                                     {600, 150, 250}, {330, 330, 340}, {100, 580, 320}, {550, 50, 400}, {100, 450, 450}, {200, 200, 600}};

// The raw value of source plane (prop, depth) at lattice cell (col, row); 65535 = no data.
inline std::uint16_t raw_value(std::int64_t col, std::int64_t row, int prop, int depth) {
    if (col % 17 == 3 && row % 11 == 5) return kSoilNoData;                                  // nothing at all
    if (prop == SoilSilt && depth == 4 && (col + row) % 23 == 0) return kSoilNoData;         // a deep plane only (subsoil)
    if (prop <= SoilClay && depth == 0 && col % 19 == 7) return 0;                           // surface texture all zero
    const std::int64_t region = ((col >> 1) * 7 + (row >> 1) * 13) % 12;
    const std::uint64_t h = mix(static_cast<std::uint64_t>(col), static_cast<std::uint64_t>(row));
    const int js = static_cast<int>(h % 21) - 10, ji = static_cast<int>((h >> 8) % 21) - 10;
    const auto pos = [](int v) { return v < 1 ? 1 : v; };
    switch (prop) {
    case SoilSand: return static_cast<std::uint16_t>(pos(kReps[region][0] + js - depth * 4));
    case SoilSilt: return static_cast<std::uint16_t>(pos(kReps[region][1] + ji));
    case SoilClay: return static_cast<std::uint16_t>(pos(kReps[region][2] + depth * 4));
    case SoilSoc: return static_cast<std::uint16_t>(400 - 50 * depth + static_cast<int>(h % 7));
    case SoilPh: return static_cast<std::uint16_t>(55 + static_cast<int>((h >> 16) % 20));
    case SoilCfvo: return static_cast<std::uint16_t>(20 * depth + static_cast<int>((h >> 20) % 40));
    default: return static_cast<std::uint16_t>(110 + 7 * depth);
    }
}

inline SoilSourceTile make_src_tile(const TileKey& key, const std::string& scheme = "soilgrids_v2_mean", std::int32_t n = 480) {
    const SoilWindow w = soil_source_window(key, n);
    SoilSourceTile t; t.key = key; t.scheme = scheme; t.cells_per_degree = n; t.col0 = w.col0; t.row0 = w.row0; t.width = w.width; t.height = w.height;
    const std::size_t cells = static_cast<std::size_t>(w.width) * static_cast<std::size_t>(w.height);
    t.values.resize(cells * kSoilPlanes);
    for (int p = 0; p < kSoilProps; ++p) for (int d = 0; d < kSoilDepths; ++d)
        for (int r = 0; r < w.height; ++r) for (int c = 0; c < w.width; ++c)
            t.values[static_cast<std::size_t>(soil_plane(p, d)) * cells + static_cast<std::size_t>(r) * static_cast<std::size_t>(w.width) + static_cast<std::size_t>(c)] =
                raw_value(w.col0 + c, w.row0 + r, p, d);
    return t;
}

// Land cover class of the 4 m UTM cell (cx, cy) (absolute cell indices): 80 water (32 m blocks), 50 built-up (64 m blocks), else 30.
inline std::uint8_t landcover_class(std::int64_t cx, std::int64_t cy) {
    if ((cx >> 3) % 7 == 2 && (cy >> 3) % 5 == 1) return 80;
    if ((cx >> 4) % 5 == 1 && (cy >> 4) % 3 == 2) return 50;
    return 30;
}
inline LandcoverTile make_landcover(const TileKey& key) {
    LandcoverTile t; t.georef = *tile_georef(key, 256, SampleRegistration::Cell); t.classes = ClassRaster(256, 256);
    t.class_scheme = "esa_worldcover_v200"; t.nodata_class = 0;
    for (int y = 0; y < 256; ++y) for (int x = 0; x < 256; ++x)
        t.classes.at(x, y) = landcover_class(static_cast<std::int64_t>(key.x) * 256 + x, static_cast<std::int64_t>(key.y) * 256 + y);
    return t;
}

inline InputTile make_input(const InputTileRef& ref, const TileBody& body) {
    const auto bytes = encode_body(body);
    TileHeader h; h.layer = ref.layer; h.layer_version = 1; h.key = ref.key;
    h.body_size = bytes->size(); h.body_hash = content_hash(*bytes);
    return {ref, h, *bytes};
}
inline InputTile absent(const InputTileRef& ref) { return InputTile{ref, {}, {}}; }

// ---- the reference evaluation ---------------------------------------------------------------------------------

// The class of the (already renormalised) triple by the rules' own clauses, evaluated here independently of
// SoilRules::texture_code; returns 0 when no class or more than one class matches (a partition violation).
inline std::uint8_t ref_texture(const SoilRules& rules, int sand, int silt, int clay) {
    int matches = 0; std::uint8_t code = 0;
    for (const auto& cls : rules.texture) {
        bool any = false;
        for (const auto& alt : cls.alternatives) {
            bool all = true;
            for (const auto& c : alt) {
                const std::int64_t v = std::int64_t{c.w[0]} * sand + std::int64_t{c.w[1]} * silt + std::int64_t{c.w[2]} * clay;
                const bool ok = c.op == SoilCmp::Lt ? v < c.k : c.op == SoilCmp::Le ? v <= c.k : c.op == SoilCmp::Gt ? v > c.k : v >= c.k;
                if (!ok) { all = false; break; }
            }
            if (all) { any = true; break; }
        }
        if (any) { ++matches; code = cls.code; }
    }
    return matches == 1 ? code : std::uint8_t{0};
}

// The soil record of cell (c, r) of level-2 tile `key`, straight from the pure functions. `with_landcover` false =
// the land cover tile is absent (no built-up / water masking).
inline SoilSample ref_cell(const TileKey& key, int c, int r, bool with_landcover = true) {
    const SoilRules& rules = builtin_soil_rules();
    const std::int64_t cell_mm = key.size_m() * 1000 / rules.cells_per_side;
    const std::int64_t e_mm = key.min_easting() * 1000 + c * cell_mm + cell_mm / 2, n_mm = key.min_northing() * 1000 + r * cell_mm + cell_mm / 2;
    SoilSample out;
    if (with_landcover) {
        const std::uint8_t cls = landcover_class(floor_div(e_mm, 4000), floor_div(n_mm, 4000));
        if (cls == 80) { out.state = SoilState::Water; return out; }
        if (cls == 50) { out.state = SoilState::BuiltUp; return out; }
    }
    const geo::Utm utm;
    const geo::TmReverse rev = utm.reverse(key.zone, static_cast<double>(e_mm) / 1000.0, static_cast<double>(n_mm) / 1000.0);
    const std::int64_t col = static_cast<std::int64_t>(std::floor(rev.lon * 480.0)), row = static_cast<std::int64_t>(std::floor(rev.lat * 480.0));
    // Surface texture all zero is a no-data cell (rules.surface_texture_sum_zero_is_nodata).
    if (raw_value(col, row, SoilSand, 0) == 0 && raw_value(col, row, SoilSilt, 0) == 0 && raw_value(col, row, SoilClay, 0) == 0) return SoilSample{};
    static constexpr int kFrom[2] = {0, 30}, kTo[2] = {30, 100};
    static constexpr int kBounds[7] = {0, 5, 15, 30, 60, 100, 200};
    for (int h = 0; h < 2; ++h) {
        std::array<int, kSoilProps> mean{};
        int total = 0;
        for (int d = 0; d < kSoilDepths; ++d) total += std::max(0, std::min(kTo[h], kBounds[d + 1]) - std::max(kFrom[h], kBounds[d]));
        for (int p = 0; p < kSoilProps; ++p) {
            std::int64_t sum = 0;
            for (int d = 0; d < kSoilDepths; ++d) {
                const int wgt = std::max(0, std::min(kTo[h], kBounds[d + 1]) - std::max(kFrom[h], kBounds[d]));
                if (wgt == 0) continue;
                const std::uint16_t v = raw_value(col, row, p, d);
                if (v == kSoilNoData) return SoilSample{};
                sum += std::int64_t{wgt} * v;
            }
            mean[static_cast<std::size_t>(p)] = static_cast<int>((sum + total / 2) / total);
        }
        const int tot = mean[0] + mean[1] + mean[2];
        const int sand = static_cast<int>(std::int64_t{mean[0]} * 1000 / tot), silt = static_cast<int>(std::int64_t{mean[1]} * 1000 / tot), clay = 1000 - sand - silt;
        SoilHorizon& z = out.horizon[static_cast<std::size_t>(h)];
        z.sand = static_cast<std::uint16_t>(sand); z.silt = static_cast<std::uint16_t>(silt); z.clay = static_cast<std::uint16_t>(clay);
        z.texture = ref_texture(rules, sand, silt, clay);
        if (z.texture == 0) return SoilSample{};
        z.soc = static_cast<std::uint16_t>(mean[SoilSoc]); z.ph10 = static_cast<std::uint16_t>(mean[SoilPh]);
        z.cfvo = static_cast<std::uint16_t>(mean[SoilCfvo]); z.bdod = static_cast<std::uint16_t>(mean[SoilBdod]);
    }
    out.state = SoilState::Ok;
    return out;
}
// The record at UTM millimetres (zone of `zone`), from whichever level-2 tile contains the point.
inline SoilSample ref_at(const geo::UtmZone& zone, std::int64_t e_mm, std::int64_t n_mm, bool with_landcover = true) {
    const TileKey key{zone, 2, static_cast<std::int32_t>(floor_div(e_mm, 1024000)), static_cast<std::int32_t>(floor_div(n_mm, 1024000))};
    const std::int64_t cell_mm = 32000;
    return ref_cell(key, static_cast<int>((e_mm - key.min_easting() * 1000) / cell_mm), static_cast<int>((n_mm - key.min_northing() * 1000) / cell_mm), with_landcover);
}

} // namespace g2m_test::soil
