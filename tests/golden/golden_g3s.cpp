// tests/golden/golden_g3s.cpp - G3s golden hashes (soil): g2m.src.soil, g2m.soil.grid and the per-field aggregates over
// the synthetic soil world of tests/support/soil_world.h (a pure function of the source lattice cell), plus the texture
// table evaluated over every permille triple.
//
// Entries: "g3s.texture.partition" (count = 501501 triples, hash over every class code in (sand, silt) order), per tile
// ("g3s.tile.<x>.<y>.src", count = the number of values, hash of the encoded container; ".grid" the same for the derived
// tile; ".cells", count = the number of cells with soil, hash over every cell record), "g3s.fields" (count = fields,
// hash over every summary) and "all". testdata/golden/g3s_golden.txt; compared byte for byte between the Windows and
// Linux legs by tools/ci.ps1: the lattice cell of a cell centre goes through the deterministic UTM reverse projection and
// a floor, everything after is integer arithmetic.

#include "golden.h"

#include "soil_world.h"
#include "synthetic.h"

#include "g2m/core/hash.h"
#include "g2m/layer/field_rules.h"
#include "g2m/layer/field_tile.h"
#include "g2m/layer/soil_tile.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace g2m_golden {
namespace {
using g2m::Fnv1a64;
using namespace g2m_test::soil;

[[noreturn]] void fail(const std::string& what) {
    std::fprintf(stderr, "golden g3s: %s\n", what.c_str());
    std::abort();
}
void hash_horizon(Fnv1a64& h, const g2m::SoilHorizon& z) {
    h.u8(z.texture).u32(z.sand).u32(z.silt).u32(z.clay).u32(z.soc).u32(z.ph10).u32(z.cfvo).u32(z.bdod);
}
} // namespace

std::vector<Entry> compute_g3s() {
    std::vector<Entry> out;
    const g2m::SoilRules& rules = g2m::builtin_soil_rules();
    {
        Fnv1a64 h; std::uint64_t n = 0;
        for (int sand = 0; sand <= 1000; ++sand)
            for (int silt = 0; silt <= 1000 - sand; ++silt) { h.u8(rules.texture_code(sand, silt, 1000 - sand - silt)); ++n; }
        out.push_back({"g3s.texture.partition", n, h.value()});
    }
    const g2m::SoilGridDeriver deriver;
    std::map<std::uint64_t, g2m::SoilGridTile> tiles;
    for (const auto& [dx, dy] : {std::pair{0, 0}, std::pair{1, 0}, std::pair{0, 1}, std::pair{1, 1}}) {
        const g2m::TileKey key = g2m_test::home_tile(dx, dy);
        const std::string prefix = "g3s.tile." + std::to_string(key.x) + "." + std::to_string(key.y);
        const g2m::SoilSourceTile src = make_src_tile(key);
        const auto src_body = g2m::encode_src_soil(src); if (!src_body) fail("encode_src_soil");
        const auto src_bytes = g2m::encode_body(*src_body); if (!src_bytes) fail("encode_body");
        out.push_back({prefix + ".src", src.values.size(), Fnv1a64().bytes(src_bytes->data(), src_bytes->size()).value()});
        const auto refs = deriver.inputs_for(key);
        const auto landcover_body = g2m::encode_src_landcover(make_landcover(key)); if (!landcover_body) fail("encode_src_landcover");
        const std::vector<g2m::InputTile> inputs{make_input(refs[0], *landcover_body), make_input(refs[1], *src_body)};
        const auto result = deriver.derive({key, 0, inputs}); if (!result) fail("derive: " + result.error().message);
        out.push_back({prefix + ".grid", result->body.size(), Fnv1a64().bytes(result->body.data(), result->body.size()).value()});
        const auto body = g2m::decode_body(result->body); if (!body) fail("decode_body");
        const auto tile = g2m::decode_soil_grid(*body); if (!tile) fail("decode_soil_grid");
        Fnv1a64 cells; std::uint64_t ok = 0;
        for (const auto& c : tile->cells) {
            cells.u8(static_cast<std::uint8_t>(c.state));
            for (const auto& z : c.horizon) hash_horizon(cells, z);
            ok += c.state == g2m::SoilState::Ok;
        }
        out.push_back({prefix + ".cells", ok, cells.value()});
        tiles.emplace(key.packed(), *tile);
    }
    {   // Field aggregates over four rectangles, one across the seam of the four tiles.
        const g2m::TileKey home = g2m_test::home_tile();
        const std::int64_t e = (home.min_easting() + 1024) * 1000, n = (home.min_northing() + 1024) * 1000;
        const std::vector<std::array<std::int64_t, 4>> rects = {{e - 240000, n - 180000, e + 200000, n + 150000}, {e - 700000, n - 600000, e - 300000, n - 200000},
                                                                  {e + 100000, n + 100000, e + 600000, n + 500000}, {e - 5000, n - 5000, e + 5000, n + 5000}};
        const g2m::SoilLookup lookup = [&](std::int64_t em, std::int64_t nm) -> std::optional<g2m::SoilSample> {
            const g2m::TileKey k{home.zone, 2, static_cast<std::int32_t>(g2m::floor_div(em, 1024000)), static_cast<std::int32_t>(g2m::floor_div(nm, 1024000))};
            const auto it = tiles.find(k.packed());
            if (it == tiles.end()) return std::nullopt;
            return g2m::soil_at(it->second, em, nm);
        };
        Fnv1a64 h; std::uint64_t count = 0;
        for (const auto& r : rects) {
            g2m::FieldRecord f; f.source = g2m::FieldSource::OsmWay; f.osm_id = 1; f.id = *g2m::pack_field_id(f.source, 1, 0);
            f.polygon.outer = {{r[0], r[1]}, {r[2], r[1]}, {r[2], r[3]}, {r[0], r[3]}};
            g2m::normalise_field_ring(f.polygon.outer, true);
            g2m::finalize_field_record(f, g2m::builtin_field_rules_de());
            const g2m::SoilFieldSummary s = g2m::summarise_field_soil(f, rules, lookup);
            h.u32(s.samples).u32(s.valid).u32(s.missing).u8(s.anchor_fallback ? 1 : 0);
            for (const auto c : s.state_count) h.u32(c);
            for (const auto& z : s.horizon) {
                h.u8(z.dominant_texture).u32(z.dominant_share_permille).u8(z.mean_texture).u32(z.texture_mask)
                    .u32(z.sand).u32(z.silt).u32(z.clay).u32(z.soc).u32(z.ph10).u32(z.cfvo).u32(z.bdod)
                    .u32(z.sand_min).u32(z.sand_max).u32(z.clay_min).u32(z.clay_max);
            }
            ++count;
        }
        out.push_back({"g3s.fields", count, h.value()});
    }
    Fnv1a64 all; std::uint64_t n = 0;
    for (const Entry& e : out) { all.str(e.name).u64(e.count).u64(e.hash); n += e.count; }
    out.push_back({"all", n, all.value()});
    return out;
}

} // namespace g2m_golden
