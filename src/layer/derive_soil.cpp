// g2m.soil.grid deriver (docs/pipeline/soil.md): one level-2 tile = 32 x 32 cells of 32 m. Every cell is a pure function
// of its centre position: the centre goes through the deterministic UTM reverse projection to a lon/lat, the source
// lattice cell containing it (floor(lon * N), floor(lat * N), N = 480) gives the 42 raw values, the horizons are
// thickness-weighted integer means, the texture class comes from the rules file. Land cover (optional input) marks
// built-up and water cells.
#include "g2m/core/geo/utm.h"
#include "g2m/core/hash.h"
#include "g2m/layer/soil_tile.h"
#include "g2m/layer/src_landcover.h"
#include "g2m/layer/src_soil.h"
#include <algorithm>
#include <cmath>

namespace g2m {
namespace {
Result<void> check_header(const InputTile& in) {
    if (in.header->key != in.ref.key || in.header->layer != in.ref.layer || in.header->layer_version != 1 ||
        in.header->body_size != in.body.size() || in.header->body_hash != content_hash(in.body))
        return make_error(ErrorCode::Malformed, "soil.grid: dependency header mismatch");
    return {};
}
// Thickness (cm) of source depth interval d inside [from, to).
std::int32_t overlap_cm(std::int32_t from, std::int32_t to, int d) {
    return std::max<std::int32_t>(0, std::min(to, kSoilDepthBoundsCm[d + 1]) - std::max(from, kSoilDepthBoundsCm[d]));
}
} // namespace

ContentHash SoilGridDeriver::config_hash() const {
    const auto& sha = builtin_soil_rules().content_sha256;
    return Sha256::of("g2m.soil.grid/1 enc=12 impl=1 rules=" + to_hex(std::span<const std::uint8_t>(sha.data(), sha.size())) +
        " cell=centre-nearest lattice=floor(lon*N),floor(lat*N) horizon=thickness-weighted-mean-half-up texture=renormalise-floor-clay-remainder"
        " nodata=any-65535|surface-texture-zero landcover=centre-cell-builtup-water-override view=same-key");
}

std::vector<InputTileRef> SoilGridDeriver::inputs_for(const TileKey& target) const {
    if (!target.valid() || target.level != 2) return {};
    return {{std::string(kSrcLandcoverLayer), target}, {std::string(kSrcSoilLayer), target}};
}

Result<DeriveResult> SoilGridDeriver::derive(const DeriveRequest& request) const {
    const TileKey& target = request.target;
    if (!target.valid() || target.level != 2) return make_error(ErrorCode::InvalidArgument, "soil.grid: level-2 target required");
    const SoilRules& rules = builtin_soil_rules();
    const auto expected = inputs_for(target);
    if (request.inputs.size() != expected.size()) return make_error(ErrorCode::InvalidArgument, "soil.grid: unexpected inputs");
    for (std::size_t i = 0; i < expected.size(); ++i)
        if (!(request.inputs[i].ref == expected[i])) return make_error(ErrorCode::InvalidArgument, "soil.grid: unexpected inputs");
    const InputTile& lc_in = request.inputs[0];
    const InputTile& soil_in = request.inputs[1];
    if (!soil_in.header) return make_error(ErrorCode::NotFound, "soil.grid: no g2m.src.soil tile");
    if (auto ok = check_header(soil_in); !ok) return ok.error();
    const auto soil_body = decode_body(soil_in.body); if (!soil_body) return soil_body.error();
    auto src = decode_src_soil(*soil_body); if (!src) return src.error();
    if (!(src->key == target)) return make_error(ErrorCode::Malformed, "soil.grid: src.soil key does not match its tile");
    if (src->scheme != rules.source_scheme || src->cells_per_degree != rules.cells_per_degree)
        return make_error(ErrorCode::Unsupported, "soil.grid: src.soil scheme or lattice does not match the rules");

    std::optional<LandcoverTile> landcover;
    if (lc_in.header) {
        if (auto ok = check_header(lc_in); !ok) return ok.error();
        const auto lb = decode_body(lc_in.body); if (!lb) return lb.error();
        auto t = decode_src_landcover(*lb); if (!t) return t.error();
        const auto want = tile_georef(target, 256, SampleRegistration::Cell);
        if (!want || !(t->georef == *want) || t->class_scheme != rules.landcover_scheme)
            return make_error(ErrorCode::Malformed, "soil.grid: src.landcover georef or scheme does not match");
        landcover = std::move(*t);
    }
    const auto in_list = [](const std::vector<std::uint8_t>& list, std::uint8_t v) { return std::binary_search(list.begin(), list.end(), v); };

    // Horizon weights: thickness of each source depth interval inside the horizon.
    std::array<std::array<std::int32_t, kSoilDepths>, kSoilHorizons> weight{};
    std::array<std::int32_t, kSoilHorizons> total{};
    for (std::size_t h = 0; h < kSoilHorizons; ++h)
        for (int d = 0; d < kSoilDepths; ++d) { weight[h][d] = overlap_cm(rules.horizons[h].from_cm, rules.horizons[h].to_cm, d); total[h] += weight[h][d]; }

    SoilGridTile tile; tile.key = target; tile.cells_per_side = rules.cells_per_side;
    const int n = rules.cells_per_side;
    tile.cells.resize(static_cast<std::size_t>(n) * static_cast<std::size_t>(n));
    const geo::Utm utm;
    const std::int64_t size_mm = target.size_m() * 1000, cell_mm = size_mm / n;
    const double ncell = static_cast<double>(rules.cells_per_degree);
    for (int r = 0; r < n; ++r) {
        for (int c = 0; c < n; ++c) {
            SoilSample& out = tile.cells[static_cast<std::size_t>(r) * static_cast<std::size_t>(n) + static_cast<std::size_t>(c)];
            const std::int64_t off_e = c * cell_mm + cell_mm / 2, off_n = r * cell_mm + cell_mm / 2; // centre offset in the tile, mm
            if (landcover) {
                const std::int64_t lc_mm = landcover->georef.spacing_mm;
                const std::uint8_t cls = landcover->classes.at(static_cast<int>(off_e / lc_mm), static_cast<int>(off_n / lc_mm));
                if (in_list(rules.landcover_water, cls)) { out.state = SoilState::Water; continue; }
                if (in_list(rules.landcover_builtup, cls)) { out.state = SoilState::BuiltUp; continue; }
            }
            const geo::TmReverse rev = utm.reverse(target.zone, static_cast<double>(target.min_easting() * 1000 + off_e) / 1000.0,
                                                    static_cast<double>(target.min_northing() * 1000 + off_n) / 1000.0);
            const std::int64_t col = static_cast<std::int64_t>(std::floor(rev.lon * ncell)) - src->col0;
            const std::int64_t row = static_cast<std::int64_t>(std::floor(rev.lat * ncell)) - src->row0;
            if (col < 0 || col >= src->width || row < 0 || row >= src->height) { out.state = SoilState::NoSource; continue; }
            bool missing = false;
            std::array<std::array<std::int32_t, kSoilProps>, kSoilHorizons> mean{};
            for (std::size_t h = 0; h < kSoilHorizons && !missing; ++h) {
                for (int p = 0; p < kSoilProps && !missing; ++p) {
                    std::int64_t sum = 0;
                    for (int d = 0; d < kSoilDepths; ++d) {
                        if (weight[h][d] == 0) continue;
                        const std::uint16_t v = src->at(soil_plane(p, d), static_cast<int>(col), static_cast<int>(row));
                        if (v == kSoilNoData) { missing = true; break; }
                        sum += static_cast<std::int64_t>(weight[h][d]) * v;
                    }
                    mean[h][p] = static_cast<std::int32_t>((sum + total[h] / 2) / total[h]);
                }
            }
            if (!missing && rules.surface_texture_sum_zero_is_nodata &&
                src->at(soil_plane(SoilSand, 0), static_cast<int>(col), static_cast<int>(row)) == 0 &&
                src->at(soil_plane(SoilSilt, 0), static_cast<int>(col), static_cast<int>(row)) == 0 &&
                src->at(soil_plane(SoilClay, 0), static_cast<int>(col), static_cast<int>(row)) == 0) missing = true;
            std::array<SoilHorizon, kSoilHorizons> hz{};
            for (std::size_t h = 0; h < kSoilHorizons && !missing; ++h) {
                std::int32_t sand = mean[h][SoilSand], silt = mean[h][SoilSilt], clay = mean[h][SoilClay];
                soil_normalise_texture(sand, silt, clay);
                if (sand + silt + clay != 1000) { missing = true; break; }
                hz[h].sand = static_cast<std::uint16_t>(sand); hz[h].silt = static_cast<std::uint16_t>(silt); hz[h].clay = static_cast<std::uint16_t>(clay);
                hz[h].texture = rules.texture_code(sand, silt, clay);
                if (hz[h].texture == 0) { missing = true; break; }
                hz[h].soc = static_cast<std::uint16_t>(mean[h][SoilSoc]);
                hz[h].ph10 = static_cast<std::uint16_t>(mean[h][SoilPh]);
                hz[h].cfvo = static_cast<std::uint16_t>(mean[h][SoilCfvo]);
                hz[h].bdod = static_cast<std::uint16_t>(mean[h][SoilBdod]);
            }
            if (missing) { out.state = SoilState::NoSource; continue; }
            out.state = SoilState::Ok; out.horizon = hz;
        }
    }
    auto encoded = encode_soil_grid(tile); if (!encoded) return encoded.error();
    auto bytes = encode_body(*encoded); if (!bytes) return bytes.error();
    return DeriveResult{std::move(*bytes), VerticalDatum::None, 0};
}
} // namespace g2m
