#include "g2m/layer/soil_tile.h"
#include "g2m/core/bytes.h"
#include "g2m/layer/src_landcover.h"
#include "g2m/layer/src_soil.h"
#include <algorithm>
#include <limits>

namespace g2m {
namespace {
Error malformed(std::string message) { return make_error(ErrorCode::Malformed, "soil.grid: " + message); }
constexpr std::uint16_t kMaxSoc = 10000, kMaxPh10 = 140, kMaxBdod = 300; // unit limits (dg/kg, pH*10, cg/cm3)
constexpr std::uint16_t kPermilleMax = 1000;
bool horizon_zero(const SoilHorizon& h) { return h == SoilHorizon{}; }
} // namespace

Result<void> validate_soil_grid(const SoilGridTile& t) {
    if (!t.key.valid() || t.key.level != 2) return malformed("key must be a valid level-2 key");
    if (t.cells_per_side < 1 || 1024 % t.cells_per_side != 0 || t.cells_per_side > 256) return malformed("cells_per_side");
    if (t.cells.size() != static_cast<std::size_t>(t.cells_per_side) * static_cast<std::size_t>(t.cells_per_side)) return malformed("cell count");
    for (const SoilSample& c : t.cells) {
        const auto state = static_cast<std::uint8_t>(c.state);
        if (state >= kSoilStateCount) return malformed("state");
        if (c.state != SoilState::Ok) {
            for (const auto& h : c.horizon) if (!horizon_zero(h)) return malformed("a cell without soil must be all zero");
            continue;
        }
        for (const auto& h : c.horizon) {
            if (h.texture == 0 || h.texture == 255) return malformed("texture code");
            if (h.sand > kPermilleMax || h.silt > kPermilleMax || h.clay > kPermilleMax ||
                static_cast<int>(h.sand) + h.silt + h.clay != 1000) return malformed("sand + silt + clay must be 1000");
            if (h.soc > kMaxSoc || h.ph10 > kMaxPh10 || h.cfvo > kPermilleMax || h.bdod > kMaxBdod) return malformed("value out of range");
        }
    }
    return {};
}

Result<TileBody> encode_soil_grid(const SoilGridTile& t) {
    const auto valid = validate_soil_grid(t); if (!valid) return valid.error();
    ByteWriter w;
    w.u64(t.key.packed());
    w.varint(static_cast<std::uint64_t>(t.cells_per_side));
    for (const auto& c : t.cells) w.u8(static_cast<std::uint8_t>(c.state));
    for (std::size_t h = 0; h < kSoilHorizons; ++h) {
        for (const auto& c : t.cells) w.u8(c.horizon[h].texture);
        for (const auto& c : t.cells) w.u16(c.horizon[h].sand);
        for (const auto& c : t.cells) w.u16(c.horizon[h].silt);
        for (const auto& c : t.cells) w.u16(c.horizon[h].clay);
        for (const auto& c : t.cells) w.u16(c.horizon[h].soc);
        for (const auto& c : t.cells) w.u16(c.horizon[h].ph10);
        for (const auto& c : t.cells) w.u16(c.horizon[h].cfvo);
        for (const auto& c : t.cells) w.u16(c.horizon[h].bdod);
    }
    return TileBody{kSoilGridEncoding, 1, {{1, SectionCodec::Zstd, w.take()}}};
}

Result<SoilGridTile> decode_soil_grid(const TileBody& body) {
    if (body.encoding != kSoilGridEncoding || body.encoding_version != 1) return make_error(ErrorCode::Unsupported, "soil.grid: encoding/version");
    if (body.sections.size() != 1 || body.sections[0].type != 1 || body.sections[0].data.size() > kMaxSectionRawSize) return malformed("sections");
    ByteReader r(body.sections[0].data);
    const auto key = TileKey::unpack(r.u64()); if (!key) return malformed("key");
    SoilGridTile t; t.key = *key;
    const auto n = r.varint();
    if (!r.ok() || n < 1 || n > 256) return malformed("cells_per_side");
    t.cells_per_side = static_cast<std::int32_t>(n);
    const std::size_t cells = static_cast<std::size_t>(n) * static_cast<std::size_t>(n);
    if (cells * (1 + kSoilHorizons * 15) != r.remaining()) return malformed("truncated or trailing bytes");
    t.cells.resize(cells);
    for (auto& c : t.cells) c.state = static_cast<SoilState>(r.u8());
    for (std::size_t h = 0; h < kSoilHorizons; ++h) {
        for (auto& c : t.cells) c.horizon[h].texture = r.u8();
        for (auto& c : t.cells) c.horizon[h].sand = r.u16();
        for (auto& c : t.cells) c.horizon[h].silt = r.u16();
        for (auto& c : t.cells) c.horizon[h].clay = r.u16();
        for (auto& c : t.cells) c.horizon[h].soc = r.u16();
        for (auto& c : t.cells) c.horizon[h].ph10 = r.u16();
        for (auto& c : t.cells) c.horizon[h].cfvo = r.u16();
        for (auto& c : t.cells) c.horizon[h].bdod = r.u16();
    }
    if (!r.at_end()) return malformed("truncated or trailing bytes");
    const auto valid = validate_soil_grid(t); if (!valid) return valid.error();
    return t;
}

std::optional<SoilSample> soil_at(const SoilGridTile& tile, std::int64_t easting_mm, std::int64_t northing_mm) {
    if (tile.cells_per_side < 1 || tile.cells.size() != static_cast<std::size_t>(tile.cells_per_side) * static_cast<std::size_t>(tile.cells_per_side))
        return std::nullopt;
    const std::int64_t size_mm = tile.key.size_m() * 1000, cell_mm = size_mm / tile.cells_per_side;
    const std::int64_t dx = easting_mm - tile.key.min_easting() * 1000, dy = northing_mm - tile.key.min_northing() * 1000;
    if (dx < 0 || dy < 0 || dx >= size_mm || dy >= size_mm) return std::nullopt;
    return tile.cells[static_cast<std::size_t>(dy / cell_mm) * static_cast<std::size_t>(tile.cells_per_side) + static_cast<std::size_t>(dx / cell_mm)];
}

SoilFieldSummary summarise_field_soil(const FieldRecord& field, const SoilRules& rules, const SoilLookup& lookup) {
    using geom::PointMm;
    SoilFieldSummary out;
    const std::int64_t s = rules.field_lattice_spacing_mm, half = s / 2;
    std::vector<PointMm> points;
    // Lattice points (i*s + s/2, j*s + s/2) of the absolute UTM millimetre grid that fall inside the polygon.
    const auto first = [&](std::int64_t lo) { return floor_div(lo - half + s - 1, s); }; // smallest i with i*s + half >= lo
    for (std::int64_t i = first(field.bbox.w); i * s + half <= field.bbox.e; ++i)
        for (std::int64_t j = first(field.bbox.s); j * s + half <= field.bbox.n; ++j) {
            const PointMm p{i * s + half, j * s + half};
            if (field_polygon_contains(field.polygon, p)) points.push_back(p);
        }
    if (points.empty()) { points.push_back(field.anchor); out.anchor_fallback = true; }
    out.samples = static_cast<std::uint32_t>(points.size());

    struct Acc {
        std::array<std::uint32_t, 256> count{};
        std::uint64_t sand = 0, silt = 0, clay = 0, soc = 0, ph = 0, cfvo = 0, bdod = 0;
        std::uint16_t sand_min = std::numeric_limits<std::uint16_t>::max(), sand_max = 0, clay_min = std::numeric_limits<std::uint16_t>::max(), clay_max = 0;
        std::uint32_t mask = 0;
    };
    std::array<Acc, kSoilHorizons> acc{};
    for (const PointMm& p : points) {
        const std::optional<SoilSample> sample = lookup(p.x, p.y);
        if (!sample) { ++out.missing; continue; }
        const auto state = static_cast<std::uint8_t>(sample->state);
        if (state < kSoilStateCount) ++out.state_count[state];
        if (sample->state != SoilState::Ok) continue;
        ++out.valid;
        for (std::size_t h = 0; h < kSoilHorizons; ++h) {
            const SoilHorizon& v = sample->horizon[h];
            Acc& a = acc[h];
            ++a.count[v.texture];
            if (v.texture < 32) a.mask |= std::uint32_t{1} << v.texture;
            a.sand += v.sand; a.silt += v.silt; a.clay += v.clay; a.soc += v.soc; a.ph += v.ph10; a.cfvo += v.cfvo; a.bdod += v.bdod;
            a.sand_min = std::min(a.sand_min, v.sand); a.sand_max = std::max(a.sand_max, v.sand);
            a.clay_min = std::min(a.clay_min, v.clay); a.clay_max = std::max(a.clay_max, v.clay);
        }
    }
    if (out.valid == 0) return out;
    const std::uint64_t n = out.valid;
    const auto mean = [&](std::uint64_t sum) { return static_cast<std::uint16_t>((sum + n / 2) / n); };
    for (std::size_t h = 0; h < kSoilHorizons; ++h) {
        const Acc& a = acc[h];
        SoilHorizonSummary& o = out.horizon[h];
        unsigned best = 0;
        for (unsigned c = 1; c < 256; ++c) if (a.count[c] > a.count[best]) best = c; // count[0] is 0 for valid samples
        o.dominant_texture = static_cast<std::uint8_t>(best);
        o.dominant_share_permille = static_cast<std::uint16_t>(static_cast<std::uint64_t>(a.count[best]) * 1000 / n);
        o.texture_mask = a.mask;
        o.sand = mean(a.sand); o.silt = mean(a.silt); o.clay = mean(a.clay);
        o.soc = mean(a.soc); o.ph10 = mean(a.ph); o.cfvo = mean(a.cfvo); o.bdod = mean(a.bdod);
        o.sand_min = a.sand_min; o.sand_max = a.sand_max; o.clay_min = a.clay_min; o.clay_max = a.clay_max;
        o.mean_texture = rules.texture_code(o.sand, o.silt, o.clay);
    }
    return out;
}

LayerDesc soil_grid_desc() {
    return {{std::string(kSoilGridLayer), 1}, LayerKind::Baked, kSoilGridEncoding, 1, level_bit(2),
        {std::string(kSrcLandcoverLayer), std::string(kSrcSoilLayer)},
        "soil facts per 32 m cell: topsoil and subsoil sand / silt / clay, texture class, organic carbon, pH, coarse fragments, bulk density"};
}
} // namespace g2m
