// g2m.src.soil and g2m.soil.grid: rules, raw codec, derived codec, deriver on the synthetic soil world, per-field
// aggregates, the importer on synthetic GeoTIFFs, server/bake identity (docs/formats/src_soil.md, soil_grid.md,
// docs/pipeline/soil.md). The synthetic world is tests/support/soil_world.h: a pure function of the source lattice
// cell, so every derived cell can be compared with a direct evaluation at its position.
#include "g2m/core/bytes.h"
#include "g2m/core/geo/utm.h"
#include "g2m/core/hash.h"
#include "g2m/import/bake.h"
#include "g2m/import/sink.h"
#include "g2m/import/soil.h"
#include "g2m/layer/field_rules.h"
#include "g2m/layer/field_tile.h"
#include "g2m/layer/soil_rules.h"
#include "g2m/layer/soil_tile.h"
#include "g2m/layer/src_landcover.h"
#include "g2m/layer/src_soil.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/upstream.h"
#include "soil_world.h"
#include "synthetic.h"
#include "tiff_builder.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <string>

using namespace g2m;
using namespace g2m_test::soil;

namespace {
const TileKey kHome = g2m_test::home_tile();
const geo::UtmZone kZone = g2m_test::kZone32N;

struct Temp {
    std::filesystem::path path = std::filesystem::temp_directory_path() / ("g2m_soil_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    Temp() { std::filesystem::create_directories(path); }
    ~Temp() { std::error_code ec; std::filesystem::remove_all(path, ec); }
};

std::string rules_text() { const auto b = builtin_soil_rules_bytes(); return std::string(reinterpret_cast<const char*>(b.data()), b.size()); }
Result<SoilRules> load_text(const std::string& s) { return load_soil_rules({reinterpret_cast<const std::byte*>(s.data()), s.size()}); }

std::vector<InputTile> grid_inputs(const TileKey& key, bool landcover = true, std::int32_t window_margin = 0) {
    SoilGridDeriver d;
    std::vector<InputTile> in;
    const auto refs = d.inputs_for(key);
    in.push_back(landcover ? make_input(refs[0], *encode_src_landcover(make_landcover(key))) : absent(refs[0]));
    SoilSourceTile src = make_src_tile(key);
    if (window_margin > 0) {   // the same world through a wider window: the derived cells must not change
        src.col0 -= window_margin; src.row0 -= window_margin; src.width += 2 * window_margin; src.height += 2 * window_margin;
        src.values.assign(static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height) * kSoilPlanes, 0);
        const std::size_t cells = static_cast<std::size_t>(src.width) * static_cast<std::size_t>(src.height);
        for (int p = 0; p < kSoilProps; ++p) for (int dd = 0; dd < kSoilDepths; ++dd)
            for (int r = 0; r < src.height; ++r) for (int c = 0; c < src.width; ++c)
                src.values[static_cast<std::size_t>(soil_plane(p, dd)) * cells + static_cast<std::size_t>(r) * static_cast<std::size_t>(src.width) + static_cast<std::size_t>(c)] =
                    raw_value(src.col0 + c, src.row0 + r, p, dd);
    }
    in.push_back(make_input(refs[1], *encode_src_soil(src)));
    return in;
}
SoilGridTile derive_grid(const TileKey& key, bool landcover = true, std::int32_t window_margin = 0) {
    SoilGridDeriver d;
    const auto in = grid_inputs(key, landcover, window_margin);
    const auto result = d.derive({key, 0, in}); REQUIRE(result);
    const auto body = decode_body(result->body); REQUIRE(body);
    const auto tile = decode_soil_grid(*body); REQUIRE(tile);
    return *tile;
}
Result<DeriveResult> derive_raw(const std::vector<InputTile>& in, const TileKey& key) { SoilGridDeriver d; return d.derive({key, 0, in}); }

TileBody body_of(const std::vector<std::uint8_t>& container) { const auto b = decode_body(container); REQUIRE(b); return *b; }
} // namespace

// ---- rules ----------------------------------------------------------------------------------------------------

TEST_CASE("Soil rules: the embedded file loads, hashes its bytes and rejects malformed rule files", "[layer][soil]") {
    const SoilRules& rules = builtin_soil_rules();
    CHECK(rules.source_scheme == "soilgrids_v2_mean"); CHECK(rules.cells_per_degree == 480); CHECK(rules.cells_per_side == 32);
    CHECK(rules.texture.size() == 12); CHECK(rules.horizons[0].to_cm == 30); CHECK(rules.horizons[1].from_cm == 30); CHECK(rules.horizons[1].to_cm == 100);
    CHECK(rules.field_lattice_spacing_mm == 16000);
    CHECK(rules.content_sha256 == Sha256::of(builtin_soil_rules_bytes()));
    const std::string text = rules_text();
    CHECK(load_text(text));
    const auto replaced = [&](const std::string& from, const std::string& to) {
        auto s = text; const auto p = s.find(from); REQUIRE(p != std::string::npos); s.replace(p, from.size(), to); return s;
    };
    CHECK_FALSE(load_text(""));
    CHECK_FALSE(load_text("[]"));
    CHECK_FALSE(load_text(replaced("g2m.soil_rules/1", "g2m.soil_rules/2")));
    CHECK_FALSE(load_text(replaced("\"cells_per_side\": 32", "\"cells_per_side\": 33")));   // does not divide 1024
    CHECK_FALSE(load_text(replaced("\"cells_per_side\": 32", "\"cells_per_side\": 2")));    // below the range
    CHECK_FALSE(load_text(replaced("\"builtup\": [50]", "\"builtup\": [80]")));            // both built-up and water
    CHECK_FALSE(load_text(replaced("\"builtup\": [50]", "\"builtup\": [50, 50]")));        // duplicate
    CHECK_FALSE(load_text(replaced("\"from_cm\": 30", "\"from_cm\": 20")));                // overlapping horizons
    CHECK_FALSE(load_text(replaced("\"code\": 5,", "\"code\": 4,")));                      // duplicate code
    CHECK_FALSE(load_text(replaced("\"op\": \"<\"", "\"op\": \"~\"")));                     // unknown operator
    CHECK_FALSE(load_text(replaced("\"field_summary\"", "\"field_summery\"")));            // unknown and missing key
    CHECK_FALSE(load_text(replaced("\"lattice_spacing_mm\": 16000", "\"lattice_spacing_mm\": 100"))); // below the range
    CHECK_FALSE(load_text(replaced("\"w\": [0, 2, 3]", "\"w\": [0, 2]")));                  // clause needs 3 weights
    CHECK_FALSE(load_text(text + " x"));
    // Every edit of a value moves the hash a release id is built from.
    const auto edited = load_text(replaced("\"lattice_spacing_mm\": 16000", "\"lattice_spacing_mm\": 16001")); REQUIRE(edited);
    CHECK(edited->content_sha256 != rules.content_sha256);
}

TEST_CASE("Soil texture: the twelve classes partition the triangle and the class representatives resolve", "[layer][soil]") {
    const SoilRules& rules = builtin_soil_rules();
    // Every permille triple summing to 1000 matches exactly one class (checked with an independent matcher), and
    // SoilRules::texture_code agrees with it.
    std::uint64_t triples = 0;
    std::array<std::uint64_t, 13> per_class{};
    for (int sand = 0; sand <= 1000; ++sand)
        for (int silt = 0; silt <= 1000 - sand; ++silt) {
            const int clay = 1000 - sand - silt;
            const std::uint8_t ref = ref_texture(rules, sand, silt, clay);
            if (ref == 0 || rules.texture_code(sand, silt, clay) != ref) { FAIL("partition violated at " << sand << "/" << silt << "/" << clay << " ref " << int(ref)); }
            ++triples; ++per_class[ref];
        }
    CHECK(triples == 501501);
    for (int c = 1; c <= 12; ++c) CHECK(per_class[static_cast<std::size_t>(c)] > 0);
    for (int i = 0; i < 12; ++i) CHECK(rules.texture_code(kReps[i][0], kReps[i][1], kReps[i][2]) == i + 1);
    CHECK(rules.texture_code(0, 0, 0) == 0);
    CHECK(rules.texture_by_code(8) != nullptr); CHECK(rules.texture_by_code(8)->name == "clay_loam"); CHECK(rules.texture_by_code(99) == nullptr);
    // The normalisation: floor for sand and silt, clay takes the remainder.
    std::int32_t a = 300, b = 300, c = 300; soil_normalise_texture(a, b, c); CHECK((a == 333 && b == 333 && c == 334));
    a = 500; b = 500; c = 0; soil_normalise_texture(a, b, c); CHECK((a == 500 && b == 500 && c == 0));
    a = 1; b = 1; c = 1; soil_normalise_texture(a, b, c); CHECK((a == 333 && b == 333 && c == 334));
    a = 0; b = 0; c = 0; soil_normalise_texture(a, b, c); CHECK((a == 0 && b == 0 && c == 0));
    // A triple not summing to 1000 is renormalised first: (60, 60, 60) is the same as (333, 333, 334).
    CHECK(rules.texture_code(60, 60, 60) == rules.texture_code(333, 333, 334));
}

// ---- raw codec and window ------------------------------------------------------------------------------------

TEST_CASE("src.soil codec round-trips, is deterministic and refuses every structural violation", "[layer][soil]") {
    const SoilSourceTile t = make_src_tile(kHome);
    const auto body = encode_src_soil(t); REQUIRE(body);
    const auto back = decode_src_soil(*body); REQUIRE(back);
    CHECK(*back == t);
    CHECK(encode_body(*body).value() == encode_body(*encode_src_soil(t)).value());
    CHECK(t.at(soil_plane(SoilClay, 2), 3, 2) == raw_value(t.col0 + 3, t.row0 + 2, SoilClay, 2));
    const auto bad = [&](const char* what, auto mutate) {
        SoilSourceTile m = t; mutate(m);
        INFO(what); CHECK_FALSE(encode_src_soil(m));
    };
    bad("value above 32767 that is not no-data", [](SoilSourceTile& m) { m.values[5] = 40000; });
    bad("value count", [](SoilSourceTile& m) { m.values.pop_back(); });
    bad("empty window", [](SoilSourceTile& m) { m.width = 0; });
    bad("scheme", [](SoilSourceTile& m) { m.scheme = "Bad Scheme"; });
    bad("empty scheme", [](SoilSourceTile& m) { m.scheme.clear(); });
    bad("level", [](SoilSourceTile& m) { m.key.level = 1; });
    bad("cells per degree", [](SoilSourceTile& m) { m.cells_per_degree = 0; });
    const auto mutated = [&](auto mutate) { TileBody b = *body; mutate(b); return decode_src_soil(b); };
    CHECK_FALSE(mutated([](TileBody& b) { b.encoding = 3; }));
    CHECK_FALSE(mutated([](TileBody& b) { b.encoding_version = 2; }));
    CHECK_FALSE(mutated([](TileBody& b) { b.sections.pop_back(); }));
    CHECK_FALSE(mutated([](TileBody& b) { b.sections.push_back(b.sections[0]); }));
    CHECK_FALSE(mutated([](TileBody& b) { b.sections[0].data.push_back(0); }));                       // trailing meta byte
    CHECK_FALSE(mutated([](TileBody& b) { b.sections[1].data.resize(b.sections[1].data.size() - 2); })); // one value short
    CHECK_FALSE(mutated([](TileBody& b) { b.sections[1].data[0] = 0x40; b.sections[1].data[1] = 0x9C; })); // 40000 injected
    CHECK_FALSE(mutated([](TileBody& b) { b.sections[0].data.resize(5); }));                           // truncated meta
}

TEST_CASE("soil_source_window covers every cell centre of its tile with a margin of one lattice cell", "[layer][soil]") {
    const geo::Utm utm;
    for (const TileKey& key : {kHome, g2m_test::home_tile(7, -3), g2m_test::home_tile(-20, 40), TileKey{geo::UtmZone{33, geo::Hemisphere::North}, 2, 100, 5600}}) {
        const SoilWindow w = soil_source_window(key, 480);
        CHECK(w.width >= 6); CHECK(w.height >= 6); CHECK(w.width <= 14); CHECK(w.height <= 14);
        for (int i = 0; i <= 32; ++i) for (int j = 0; j <= 32; ++j) {
            const auto r = utm.reverse(key.zone, static_cast<double>(key.min_easting()) + 32.0 * i, static_cast<double>(key.min_northing()) + 32.0 * j);
            const std::int64_t col = static_cast<std::int64_t>(std::floor(r.lon * 480.0)), row = static_cast<std::int64_t>(std::floor(r.lat * 480.0));
            REQUIRE(col - w.col0 >= 1); REQUIRE(w.col0 + w.width - 1 - col >= 1);
            REQUIRE(row - w.row0 >= 1); REQUIRE(w.row0 + w.height - 1 - row >= 1);
        }
        CHECK(soil_source_window(key, 480) == w);
    }
}

// ---- derived codec ---------------------------------------------------------------------------------------------

TEST_CASE("soil.grid codec round-trips, validates and answers point queries", "[layer][soil]") {
    SoilGridTile t; t.key = kHome; t.cells_per_side = 32; t.cells.resize(1024);
    for (int r = 0; r < 32; ++r) for (int c = 0; c < 32; ++c) t.cells[static_cast<std::size_t>(r) * 32 + static_cast<std::size_t>(c)] = ref_cell(kHome, c, r);
    std::size_t ok = 0; for (const auto& c : t.cells) ok += c.state == SoilState::Ok; REQUIRE(ok > 100);
    const auto body = encode_soil_grid(t); REQUIRE(body);
    const auto back = decode_soil_grid(*body); REQUIRE(back);
    CHECK(*back == t);
    std::size_t ok_index = 0; while (t.cells[ok_index].state != SoilState::Ok) ++ok_index;

    const auto bad = [&](const char* what, auto mutate) { SoilGridTile m = t; mutate(m); INFO(what); CHECK_FALSE(validate_soil_grid(m)); CHECK_FALSE(encode_soil_grid(m)); };
    bad("a cell without soil carries data", [&](SoilGridTile& m) { for (auto& c : m.cells) if (c.state != SoilState::Ok) { c.horizon[0].sand = 1; break; } });
    bad("sum not 1000", [&](SoilGridTile& m) { m.cells[ok_index].horizon[0].silt += 1; });
    bad("texture 0 in a valid cell", [&](SoilGridTile& m) { m.cells[ok_index].horizon[1].texture = 0; });
    bad("texture 255", [&](SoilGridTile& m) { m.cells[ok_index].horizon[1].texture = 255; });
    bad("soc range", [&](SoilGridTile& m) { m.cells[ok_index].horizon[0].soc = 10001; });
    bad("pH range", [&](SoilGridTile& m) { m.cells[ok_index].horizon[0].ph10 = 141; });
    bad("bulk density range", [&](SoilGridTile& m) { m.cells[ok_index].horizon[0].bdod = 301; });
    bad("coarse fragment range", [&](SoilGridTile& m) { m.cells[ok_index].horizon[0].cfvo = 1001; });
    bad("state", [&](SoilGridTile& m) { m.cells[0].state = static_cast<SoilState>(4); });
    bad("cell count", [&](SoilGridTile& m) { m.cells.pop_back(); });
    bad("cells_per_side does not divide 1024", [&](SoilGridTile& m) { m.cells_per_side = 33; });
    bad("level", [&](SoilGridTile& m) { m.key.level = 1; });
    const auto mutated = [&](auto mutate) { TileBody b = *body; mutate(b); return decode_soil_grid(b); };
    CHECK_FALSE(mutated([](TileBody& b) { b.encoding = 10; }));
    CHECK_FALSE(mutated([](TileBody& b) { b.encoding_version = 2; }));
    CHECK_FALSE(mutated([](TileBody& b) { b.sections.push_back(b.sections[0]); }));
    CHECK_FALSE(mutated([](TileBody& b) { b.sections[0].data.push_back(0); }));
    CHECK_FALSE(mutated([](TileBody& b) { b.sections[0].data.pop_back(); }));
    CHECK_FALSE(mutated([](TileBody& b) { b.sections[0].data[8] = 0; }));   // cells_per_side 0

    // Point queries: half-open cells, south-west origin.
    const std::int64_t e0 = kHome.min_easting() * 1000, n0 = kHome.min_northing() * 1000;
    CHECK(soil_at(t, e0, n0) == t.cells[0]);
    CHECK(soil_at(t, e0 + 31999, n0 + 31999) == t.cells[0]);
    CHECK(soil_at(t, e0 + 32000, n0) == t.cells[1]);
    CHECK(soil_at(t, e0, n0 + 32000) == t.cells[32]);
    CHECK(soil_at(t, e0 + 1023999, n0 + 1023999) == t.cells[1023]);
    CHECK_FALSE(soil_at(t, e0 + 1024000, n0));
    CHECK_FALSE(soil_at(t, e0, n0 + 1024000));
    CHECK_FALSE(soil_at(t, e0 - 1, n0));
    CHECK_FALSE(soil_at(t, e0, n0 - 1));
}

// ---- the deriver ---------------------------------------------------------------------------------------------------

TEST_CASE("Soil deriver: every cell equals a direct evaluation of the world at its position", "[layer][soil][soil_derive]") {
    std::map<SoilState, std::size_t> states; std::set<int> top_classes, sub_classes;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        const TileKey key = g2m_test::home_tile(dx, dy);
        const SoilGridTile tile = derive_grid(key);
        REQUIRE(tile.cells_per_side == 32); REQUIRE(tile.key == key);
        for (int r = 0; r < 32; ++r) for (int c = 0; c < 32; ++c) {
            const SoilSample& got = tile.cells[static_cast<std::size_t>(r) * 32 + static_cast<std::size_t>(c)];
            const SoilSample want = ref_cell(key, c, r);
            if (!(got == want)) FAIL("tile " << dx << "," << dy << " cell " << c << "," << r << ": state " << int(got.state) << " vs " << int(want.state));
            ++states[got.state];
            if (got.state == SoilState::Ok) { top_classes.insert(got.horizon[0].texture); sub_classes.insert(got.horizon[1].texture); }
        }
    }
    // The world exercises every state and most classes (the test is not vacuous).
    CHECK(states[SoilState::Ok] > 4000); CHECK(states[SoilState::NoSource] > 100); CHECK(states[SoilState::BuiltUp] > 100); CHECK(states[SoilState::Water] > 50);
    CHECK(top_classes.size() >= 10); CHECK(sub_classes.size() >= 10);
}

TEST_CASE("Soil deriver: without a land cover tile no cell is masked", "[layer][soil][soil_derive]") {
    const SoilGridTile tile = derive_grid(kHome, false);
    std::size_t ok = 0, ok_masked = 0;
    for (const auto& c : derive_grid(kHome).cells) ok_masked += c.state == SoilState::Ok;
    for (int r = 0; r < 32; ++r) for (int c = 0; c < 32; ++c) {
        const SoilSample& got = tile.cells[static_cast<std::size_t>(r) * 32 + static_cast<std::size_t>(c)];
        CHECK(got == ref_cell(kHome, c, r, false));
        CHECK(got.state != SoilState::BuiltUp); CHECK(got.state != SoilState::Water);
        ok += got.state == SoilState::Ok;
    }
    CHECK(ok > ok_masked);
}

TEST_CASE("Soil deriver: a cell does not depend on the window its tile's source was cut with", "[layer][soil][soil_derive]") {
    for (const int margin : {1, 4}) {
        const SoilGridTile wide = derive_grid(kHome, true, margin);
        CHECK(wide == derive_grid(kHome));
    }
}

TEST_CASE("Soil deriver: no-data cells, a window that is too small and every input rule", "[layer][soil][soil_derive]") {
    SoilGridDeriver deriver;
    CHECK(deriver.output().id.name == "g2m.soil.grid");
    CHECK(deriver.coverage_inputs().size() == 1); CHECK(deriver.coverage_inputs()[0] == kSrcSoilLayer);
    CHECK(deriver.inputs_for(kHome).size() == 2);
    CHECK(deriver.inputs_for(TileKey{kZone, 1, 3, 3}).empty());
    CHECK(deriver.config_hash() == SoilGridDeriver().config_hash());

    // A window cut down to a few cells leaves the rest NoSource (never a read outside the window).
    {
        SoilSourceTile src = make_src_tile(kHome);
        SoilSourceTile cut = src; cut.width = src.width - 3;
        const std::size_t cells = static_cast<std::size_t>(cut.width) * static_cast<std::size_t>(cut.height);
        cut.values.assign(cells * kSoilPlanes, 0);
        for (int p = 0; p < kSoilPlanes; ++p) for (int r = 0; r < cut.height; ++r) for (int c = 0; c < cut.width; ++c)
            cut.values[static_cast<std::size_t>(p) * cells + static_cast<std::size_t>(r) * static_cast<std::size_t>(cut.width) + static_cast<std::size_t>(c)] = src.at(p, c, r);
        const auto refs = deriver.inputs_for(kHome);
        const auto result = derive_raw({absent(refs[0]), make_input(refs[1], *encode_src_soil(cut))}, kHome); REQUIRE(result);
        const auto tile = decode_soil_grid(body_of(result->body)); REQUIRE(tile);
        std::size_t no_source = 0, ok = 0;
        for (const auto& c : tile->cells) { no_source += c.state == SoilState::NoSource; ok += c.state == SoilState::Ok; }
        CHECK(no_source > 100); CHECK(ok > 100);
    }
    const auto refs = deriver.inputs_for(kHome);
    const auto soil_ok = make_input(refs[1], *encode_src_soil(make_src_tile(kHome)));
    const auto lc_ok = make_input(refs[0], *encode_src_landcover(make_landcover(kHome)));
    CHECK(derive_raw({lc_ok, soil_ok}, kHome));
    // Missing soil source tile: no derivation.
    CHECK_FALSE(derive_raw({lc_ok, absent(refs[1])}, kHome));
    CHECK(derive_raw({lc_ok, absent(refs[1])}, kHome).error().code == ErrorCode::NotFound);
    // A source of another scheme or lattice is refused, not misread.
    CHECK(derive_raw({lc_ok, make_input(refs[1], *encode_src_soil(make_src_tile(kHome, "other_scheme")))}, kHome).error().code == ErrorCode::Unsupported);
    CHECK_FALSE(derive_raw({lc_ok, make_input(refs[1], *encode_src_soil(make_src_tile(kHome, "soilgrids_v2_mean", 960)))}, kHome));
    // A soil body of another tile, a body that does not match its header and a foreign land cover scheme.
    CHECK_FALSE(derive_raw({lc_ok, make_input(refs[1], *encode_src_soil(make_src_tile(g2m_test::home_tile(1, 0))))}, kHome));
    { auto tampered = soil_ok; tampered.body[tampered.body.size() / 2] ^= 0x01; CHECK_FALSE(derive_raw({lc_ok, tampered}, kHome)); }
    { LandcoverTile lc = make_landcover(kHome); lc.class_scheme = "other_scheme"; CHECK_FALSE(derive_raw({make_input(refs[0], *encode_src_landcover(lc)), soil_ok}, kHome)); }
    // Wrong input list.
    CHECK_FALSE(derive_raw({soil_ok}, kHome));
    CHECK_FALSE(derive_raw({soil_ok, lc_ok}, kHome));
    // Repeatable.
    CHECK(derive_raw({lc_ok, soil_ok}, kHome)->body == derive_raw({lc_ok, soil_ok}, kHome)->body);
}

// ---- per-field aggregates ----------------------------------------------------------------------------------------

namespace {
FieldRecord rect_field(std::int64_t e0, std::int64_t n0, std::int64_t e1, std::int64_t n1, std::int64_t osm = 1) {
    FieldRecord f; f.source = FieldSource::OsmWay; f.osm_id = osm; f.piece = 0; f.id = *pack_field_id(f.source, osm, 0);
    f.polygon.outer = {{e0, n0}, {e1, n0}, {e1, n1}, {e0, n1}};
    normalise_field_ring(f.polygon.outer, true);
    finalize_field_record(f, builtin_field_rules_de());
    return f;
}
struct Expected {
    std::uint32_t samples = 0, valid = 0, missing = 0;
    std::array<std::uint32_t, kSoilStateCount> states{};
    std::array<std::map<int, int>, 2> classes;
    std::array<std::int64_t, 2> sand{}, clay{};
    std::array<int, 2> sand_min{1000, 1000}, sand_max{0, 0};
};
// The independent aggregation: enumerate the lattice points of the rectangle by hand.
Expected expect_rect(std::int64_t e0, std::int64_t n0, std::int64_t e1, std::int64_t n1, const SoilLookup& lookup) {
    Expected x;
    for (std::int64_t e = (e0 - 8000 + 15999) / 16000 * 16000 + 8000; e <= e1; e += 16000)
        for (std::int64_t n = (n0 - 8000 + 15999) / 16000 * 16000 + 8000; n <= n1; n += 16000) {
            ++x.samples;
            const auto s = lookup(e, n);
            if (!s) { ++x.missing; continue; }
            ++x.states[static_cast<std::size_t>(s->state)];
            if (s->state != SoilState::Ok) continue;
            ++x.valid;
            for (int h = 0; h < 2; ++h) {
                const auto& z = s->horizon[static_cast<std::size_t>(h)];
                ++x.classes[static_cast<std::size_t>(h)][z.texture]; x.sand[static_cast<std::size_t>(h)] += z.sand; x.clay[static_cast<std::size_t>(h)] += z.clay;
                x.sand_min[static_cast<std::size_t>(h)] = std::min<int>(x.sand_min[static_cast<std::size_t>(h)], z.sand);
                x.sand_max[static_cast<std::size_t>(h)] = std::max<int>(x.sand_max[static_cast<std::size_t>(h)], z.sand);
            }
        }
    return x;
}
void check_against(const SoilFieldSummary& s, const Expected& x) {
    CHECK(s.samples == x.samples); CHECK(s.valid == x.valid); CHECK(s.missing == x.missing); CHECK(s.state_count == x.states);
    if (x.valid == 0) return;
    for (std::size_t h = 0; h < 2; ++h) {
        int best = 0, best_n = 0; std::uint32_t mask = 0;
        for (const auto& [cls, n] : x.classes[h]) { if (n > best_n) { best = cls; best_n = n; } mask |= std::uint32_t{1} << cls; }   // map order: ties keep the lowest code
        CHECK(s.horizon[h].dominant_texture == best);
        CHECK(s.horizon[h].dominant_share_permille == static_cast<std::uint64_t>(best_n) * 1000 / x.valid);
        CHECK(s.horizon[h].texture_mask == mask);
        CHECK(s.horizon[h].sand == (x.sand[h] + x.valid / 2) / x.valid);
        CHECK(s.horizon[h].clay == (x.clay[h] + x.valid / 2) / x.valid);
        CHECK(s.horizon[h].sand_min == x.sand_min[h]); CHECK(s.horizon[h].sand_max == x.sand_max[h]);
    }
}
} // namespace

TEST_CASE("Field soil: the aggregate equals an independent enumeration whichever tiles serve the lookup", "[layer][soil][soil_field]") {
    const SoilRules& rules = builtin_soil_rules();
    // A field across the seam between two tiles and the corner of four.
    const std::int64_t e_seam = (kHome.min_easting() + 1024) * 1000, n_seam = (kHome.min_northing() + 1024) * 1000;
    const FieldRecord f = rect_field(e_seam - 240000, n_seam - 180000, e_seam + 200000, n_seam + 150000);
    const SoilLookup direct = [&](std::int64_t e, std::int64_t n) -> std::optional<SoilSample> { return ref_at(kZone, e, n); };
    const SoilFieldSummary a = summarise_field_soil(f, rules, direct);
    check_against(a, expect_rect(e_seam - 240000, n_seam - 180000, e_seam + 200000, n_seam + 150000, direct));
    {   // The lookup is asked at exactly the lattice points of the rules: odd multiples of 8 m in absolute UTM millimetres.
        std::set<std::pair<std::int64_t, std::int64_t>> asked;
        const SoilLookup spy = [&](std::int64_t e, std::int64_t n) -> std::optional<SoilSample> { asked.emplace(e, n); return direct(e, n); };
        summarise_field_soil(f, rules, spy);
        for (const auto& p : asked) { CHECK((p.first - 8000) % 16000 == 0); CHECK((p.second - 8000) % 16000 == 0); }
        CHECK(asked.size() == a.samples);
        CHECK(asked.begin()->first == e_seam - 240000 + 8000);   // the west edge is a multiple of 16 m: the first column lies half a spacing inside
    }
    // Served from derived tiles (each with its own source window) the summary is the same.
    std::map<std::uint64_t, SoilGridTile> tiles;
    for (int dx = 0; dx <= 1; ++dx) for (int dy = 0; dy <= 1; ++dy) { const TileKey k = g2m_test::home_tile(dx, dy); tiles.emplace(k.packed(), derive_grid(k, true, dx + dy)); }
    const SoilLookup served = [&](std::int64_t e, std::int64_t n) -> std::optional<SoilSample> {
        const TileKey k{kZone, 2, static_cast<std::int32_t>(floor_div(e, 1024000)), static_cast<std::int32_t>(floor_div(n, 1024000))};
        const auto it = tiles.find(k.packed()); if (it == tiles.end()) return std::nullopt; return soil_at(it->second, e, n);
    };
    CHECK(summarise_field_soil(f, rules, served) == a);
    // A tile the lookup cannot answer leaves the aggregate marked incomplete instead of silently shorter.
    const SoilLookup holed = [&](std::int64_t e, std::int64_t n) -> std::optional<SoilSample> { if (e >= e_seam) return std::nullopt; return direct(e, n); };
    const SoilFieldSummary h = summarise_field_soil(f, rules, holed);
    check_against(h, expect_rect(e_seam - 240000, n_seam - 180000, e_seam + 200000, n_seam + 150000, holed));
    CHECK(h.missing > 0); CHECK(h.valid + h.missing + h.state_count[1] + h.state_count[2] + h.state_count[3] == h.samples);
    // Nothing valid anywhere: all-zero horizons, every sample counted under its state.
    const SoilLookup none = [](std::int64_t, std::int64_t) -> std::optional<SoilSample> { return SoilSample{}; };
    const SoilFieldSummary z = summarise_field_soil(f, rules, none);
    CHECK(z.valid == 0); CHECK(z.state_count[static_cast<std::size_t>(SoilState::NoSource)] == z.samples);
    CHECK(z.horizon[0] == SoilHorizonSummary{}); CHECK(z.horizon[1] == SoilHorizonSummary{});
}

TEST_CASE("Field soil: ties go to the lowest class, means round half up, small fields fall back to the anchor", "[layer][soil][soil_field]") {
    const SoilRules& rules = builtin_soil_rules();
    const std::int64_t o = (kHome.min_easting() + 100) * 1000 / 32000 * 32000, on = (kHome.min_northing() + 100) * 1000 / 32000 * 32000;
    const FieldRecord f = rect_field(o, on, o + 320000, on + 16000);   // 20 lattice points in one row, 10 on each side of the split
    const auto make = [&](std::uint16_t sand, std::uint16_t silt, std::uint16_t clay, std::uint8_t tex) {
        SoilSample s; s.state = SoilState::Ok;
        for (auto& z : s.horizon) { z.texture = tex; z.sand = sand; z.silt = silt; z.clay = clay; z.soc = 100; z.ph10 = 65; z.cfvo = 10; z.bdod = 130; }
        return s;
    };
    const SoilSample left = make(330, 330, 340, 8), right = make(400, 400, 200, 4);
    const SoilLookup split = [&](std::int64_t e, std::int64_t) -> std::optional<SoilSample> { return e < o + 160000 ? left : right; };
    const SoilFieldSummary s = summarise_field_soil(f, rules, split);
    REQUIRE(s.samples == 20); CHECK(s.valid == 20);
    CHECK(s.horizon[0].dominant_texture == 4);                 // 10 : 10 -> the lowest code
    CHECK(s.horizon[0].dominant_share_permille == 500);
    CHECK(s.horizon[0].texture_mask == ((1u << 4) | (1u << 8)));
    CHECK(s.horizon[0].sand == 365); CHECK(s.horizon[0].silt == 365); CHECK(s.horizon[0].clay == 270);   // (3300 + 4000 + 10) / 20
    CHECK(s.horizon[0].sand_min == 330); CHECK(s.horizon[0].sand_max == 400); CHECK(s.horizon[0].clay_min == 200); CHECK(s.horizon[0].clay_max == 340);
    CHECK(s.horizon[0].mean_texture == rules.texture_code(365, 365, 270));
    CHECK(s.horizon[0].soc == 100); CHECK(s.horizon[0].ph10 == 65); CHECK(s.horizon[0].bdod == 130);
    // 11 : 9 -> the majority; a half-permille mean rounds up (11 x 331 + 9 x 330 = 6611, / 20 = 330.55 -> 331).
    const SoilSample a = make(331, 330, 339, 8), b = make(330, 330, 340, 4);
    const SoilLookup eleven = [&](std::int64_t e, std::int64_t) -> std::optional<SoilSample> { return e < o + 176000 ? a : b; };
    const SoilFieldSummary t = summarise_field_soil(f, rules, eleven);
    CHECK(t.horizon[0].dominant_texture == 8); CHECK(t.horizon[0].dominant_share_permille == 550); CHECK(t.horizon[0].sand == 331);

    // A 2 m x 2 m field holds no lattice point: the anchor is sampled.
    const FieldRecord tiny = rect_field(o + 1000, on + 1000, o + 3000, on + 3000);
    const SoilFieldSummary u = summarise_field_soil(tiny, rules, split);
    CHECK(u.anchor_fallback); CHECK(u.samples == 1); CHECK(u.valid == 1); CHECK(u.horizon[0].dominant_texture == 8);
}

// ---- importer ----------------------------------------------------------------------------------------------------------

namespace {
// A single-band 16-bit GeoTIFF of the lattice cells [col0, col0 + width) x [row0, row0 + height), filled from
// raw_value(); `is_signed` writes int16 with -32768 as the no-data value (the SoilGrids way), otherwise uint16.
struct TiffSpec {
    std::int64_t col0 = 0, row0 = 0; int width = 0, height = 0;
    bool is_signed = true; double pixel = 1.0 / 480.0; bool geokeys = true;
    std::uint16_t poison = 0; // when set, the centre pixel of sand_0-5cm holds this value (an unsigned set only)
};
void write_soil_tiff(const std::filesystem::path& path, const TiffSpec& spec, int prop, int depth) {
    std::vector<std::byte> strip(static_cast<std::size_t>(spec.width) * static_cast<std::size_t>(spec.height) * 2);
    for (int row = 0; row < spec.height; ++row) for (int col = 0; col < spec.width; ++col) {
        const std::int64_t lat_row = spec.row0 + spec.height - 1 - row;
        const std::uint16_t v = raw_value(spec.col0 + col, lat_row, prop, depth);
        std::uint16_t stored = spec.is_signed && v == kSoilNoData ? std::uint16_t{0x8000} : v;
        if (spec.poison != 0 && prop == SoilSand && depth == 0 && col == spec.width / 2 && row == spec.height / 2) stored = spec.poison;
        const std::size_t at = (static_cast<std::size_t>(row) * static_cast<std::size_t>(spec.width) + static_cast<std::size_t>(col)) * 2;
        strip[at] = static_cast<std::byte>(stored & 0xff); strip[at + 1] = static_cast<std::byte>(stored >> 8);
    }
    g2m::test_support::TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(spec.width)); b.add_short(257, static_cast<std::uint16_t>(spec.height));
    b.add_short(258, 16); b.add_short(259, 1); b.add_short(262, 1); b.add_short(277, 1);
    b.add_long(278, static_cast<std::uint32_t>(spec.height)); b.add_short(284, 1); b.add_short(339, spec.is_signed ? 2 : 1);
    b.add_double_array(33550, {spec.pixel, spec.pixel, 0.0});
    b.add_double_array(33922, {0.0, 0.0, 0.0, static_cast<double>(spec.col0) / 480.0, static_cast<double>(spec.row0 + spec.height) / 480.0, 0.0});
    if (spec.geokeys) b.add_short_array(34735, {1, 1, 0, 2, 1025, 0, 1, 1, 2048, 0, 1, 4326});
    if (spec.is_signed) b.add_ascii(42113, "-32768");
    const auto bytes = b.build(strip);
    std::ofstream out(path, std::ios::binary | std::ios::trunc); out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}
void write_soil_set(const std::filesystem::path& dir, const TiffSpec& spec) {
    std::filesystem::create_directories(dir);
    for (int p = 0; p < kSoilProps; ++p) for (int d = 0; d < kSoilDepths; ++d)
        write_soil_tiff(dir / (std::string(kSoilPropNames[static_cast<std::size_t>(p)]) + "_" + std::string(kSoilDepthNames[static_cast<std::size_t>(d)]) + "_mean.tif"), spec, p, d);
}
ResolvedRegion region_of(const std::vector<TileKey>& keys) {
    ResolvedRegion r; r.name = "test"; r.scheme = geo::utm_scheme_name(keys.front().zone); r.level = 2; r.halo_m = 0;
    std::vector<TileKey> sorted = keys;
    std::sort(sorted.begin(), sorted.end(), [](const TileKey& a, const TileKey& b) { return a.y != b.y ? a.y < b.y : a.x < b.x; });
    for (const auto& k : sorted) {   // one run per row of adjacent tiles
        if (!r.tiles.empty() && r.tiles.back().y == k.y && r.tiles.back().x1 + 1 == k.x) ++r.tiles.back().x1;
        else r.tiles.push_back(CoverageRun{k.y, k.x, k.x});
    }
    r.interior = r.tiles; r.tile_count = keys.size();
    return r;
}
// The covering box of the windows of `keys`, widened by `margin` cells.
TiffSpec covering(const std::vector<TileKey>& keys, int margin = 2) {
    std::int64_t c0 = INT64_MAX, r0 = INT64_MAX, c1 = INT64_MIN, r1 = INT64_MIN;
    for (const auto& k : keys) {
        const SoilWindow w = soil_source_window(k, 480);
        c0 = std::min(c0, w.col0); r0 = std::min(r0, w.row0); c1 = std::max(c1, w.col0 + w.width); r1 = std::max(r1, w.row0 + w.height);
    }
    TiffSpec s; s.col0 = c0 - margin; s.row0 = r0 - margin; s.width = static_cast<int>(c1 - c0 + 2 * margin); s.height = static_cast<int>(r1 - r0 + 2 * margin);
    return s;
}
std::vector<std::uint8_t> tile_bytes(const import::MemorySink& sink, const TileKey& key) {
    for (const auto& [k, e] : sink.entries()) if (k.first == kSrcSoilLayer && k.second == key) return e.body;
    return {};
}
} // namespace

TEST_CASE("Soil import: a set of GeoTIFFs becomes the raw tiles of the world, int16 or uint16, for any job count", "[layer][soil][soil_import]") {
    Temp temp;
    const std::vector<TileKey> keys = {kHome, g2m_test::home_tile(1, 0), g2m_test::home_tile(0, 1), g2m_test::home_tile(1, 1)};
    for (const bool is_signed : {true, false}) {
        const auto dir = temp.path / (is_signed ? "signed" : "unsigned");
        TiffSpec spec = covering(keys); spec.is_signed = is_signed;
        write_soil_set(dir, spec);
        import::MemorySink one, four;
        const auto r1 = import::import_soil(region_of(keys), {dir}, one, "test", {1}); REQUIRE(r1);
        const auto r4 = import::import_soil(region_of(keys), {dir}, four, "test", {4}); REQUIRE(r4);
        CHECK(r1->tiles_written == 4); CHECK(r1->tiles_skipped_nocoverage == 0); CHECK(r1->cells_nodata == 0);
        CHECK(one.entries().size() == 4);
        for (const auto& k : keys) {
            const auto bytes = tile_bytes(one, k); REQUIRE_FALSE(bytes.empty());
            CHECK(bytes == tile_bytes(four, k));
            const auto src = decode_src_soil(body_of(bytes)); REQUIRE(src);
            CHECK(*src == make_src_tile(k));   // values copied as they are; -32768 and 65535 both become the marker
        }
    }
}

TEST_CASE("Soil import: partial coverage, several sets, tiles without coverage and every rejection", "[layer][soil][soil_import]") {
    Temp temp;
    const std::vector<TileKey> keys = {kHome};
    const TiffSpec full = covering(keys);
    write_soil_set(temp.path / "full", full);
    // A set that covers only the northern part of the window.
    TiffSpec north = full; const int cut = full.height / 2; north.row0 = full.row0 + cut; north.height = full.height - cut;
    write_soil_set(temp.path / "north", north);
    const SoilWindow w = soil_source_window(kHome, 480);
    {
        import::MemorySink sink;
        const auto r = import::import_soil(region_of(keys), {temp.path / "north"}, sink, "test"); REQUIRE(r);
        CHECK(r->tiles_written == 1); CHECK(r->cells_nodata > 0);
        const auto src = decode_src_soil(body_of(tile_bytes(sink, kHome))); REQUIRE(src);
        const SoilSourceTile want = make_src_tile(kHome);
        for (int row = 0; row < w.height; ++row) for (int col = 0; col < w.width; ++col) {
            const bool covered = w.row0 + row >= north.row0;
            for (int p = 0; p < kSoilPlanes; ++p) CHECK(src->at(p, col, row) == (covered ? want.at(p, col, row) : kSoilNoData));
        }
        CHECK(r->cells_nodata == static_cast<std::uint64_t>(std::max(0, static_cast<int>(north.row0 - w.row0))) * static_cast<std::uint64_t>(w.width));
    }
    {   // The first set that covers a cell wins; the second fills the rest.
        import::MemorySink sink;
        const auto r = import::import_soil(region_of(keys), {temp.path / "north", temp.path / "full"}, sink, "test"); REQUIRE(r);
        CHECK(r->cells_nodata == 0);
        CHECK(*decode_src_soil(body_of(tile_bytes(sink, kHome))) == make_src_tile(kHome));
    }
    {   // A value above 32767 that is not the marker is no data too (it cannot be a soil value of any property).
        TiffSpec poisoned = full; poisoned.is_signed = false; poisoned.poison = 40000;
        write_soil_set(temp.path / "poison", poisoned);
        import::MemorySink sink;
        const auto r = import::import_soil(region_of(keys), {temp.path / "poison"}, sink, "test"); REQUIRE(r);
        const auto src = decode_src_soil(body_of(tile_bytes(sink, kHome))); REQUIRE(src);
        SoilSourceTile want = make_src_tile(kHome);
        const std::int64_t col = poisoned.col0 + poisoned.width / 2, lat_row = poisoned.row0 + poisoned.height - 1 - poisoned.height / 2;
        REQUIRE(col >= w.col0); REQUIRE(col < w.col0 + w.width); REQUIRE(lat_row >= w.row0); REQUIRE(lat_row < w.row0 + w.height);
        const std::size_t cells = static_cast<std::size_t>(w.width) * static_cast<std::size_t>(w.height);
        want.values[static_cast<std::size_t>(soil_plane(SoilSand, 0)) * cells + static_cast<std::size_t>(lat_row - w.row0) * static_cast<std::size_t>(w.width) + static_cast<std::size_t>(col - w.col0)] = kSoilNoData;
        CHECK(*src == want);
    }
    {   // A tile no set touches is not written.
        import::MemorySink sink;
        const auto r = import::import_soil(region_of({g2m_test::home_tile(300, 300)}), {temp.path / "full"}, sink, "test"); REQUIRE(r);
        CHECK(r->tiles_written == 0); CHECK(r->tiles_skipped_nocoverage == 1); CHECK(sink.size() == 0);
    }
    // Rejections: nothing is written, the code says why.
    const auto rejected = [&](const char* what, const std::filesystem::path& dir, ErrorCode code) {
        import::MemorySink sink; INFO(what);
        const auto r = import::import_soil(region_of(keys), {dir}, sink, "test");
        REQUIRE_FALSE(r); CHECK(r.error().code == code); CHECK(sink.size() == 0);
    };
    {
        write_soil_set(temp.path / "badpixel", full);
        TiffSpec s = full; s.pixel = 1.0 / 481.0; write_soil_tiff(temp.path / "badpixel" / "clay_0-5cm_mean.tif", s, SoilClay, 0);
        rejected("pixel size", temp.path / "badpixel", ErrorCode::Unsupported);
    }
    {
        write_soil_set(temp.path / "nocrs", full);
        TiffSpec s = full; s.geokeys = false; write_soil_tiff(temp.path / "nocrs" / "soc_30-60cm_mean.tif", s, SoilSoc, 3);
        rejected("no CRS", temp.path / "nocrs", ErrorCode::Unsupported);
    }
    {
        write_soil_set(temp.path / "size", full);
        TiffSpec s = full; s.width -= 1; write_soil_tiff(temp.path / "size" / "bdod_100-200cm_mean.tif", s, SoilBdod, 5);
        rejected("size differs", temp.path / "size", ErrorCode::Unsupported);
    }
    {
        write_soil_set(temp.path / "mixed", full);
        TiffSpec s = full; s.is_signed = false; write_soil_tiff(temp.path / "mixed" / "silt_5-15cm_mean.tif", s, SoilSilt, 1);
        rejected("sample type differs", temp.path / "mixed", ErrorCode::Unsupported);
    }
    {
        write_soil_set(temp.path / "missing", full);
        std::filesystem::remove(temp.path / "missing" / "phh2o_0-5cm_mean.tif");
        rejected("a file is missing", temp.path / "missing", ErrorCode::Io);
    }
    {
        import::MemorySink sink;
        const auto r = import::import_soil(region_of(keys), {}, sink, "test"); REQUIRE_FALSE(r); CHECK(r.error().code == ErrorCode::InvalidArgument);
    }
}

// ---- server and bake -----------------------------------------------------------------------------------------------

TEST_CASE("Soil layer: server and parallel bake produce identical pinned containers", "[layer][soil][soil_bake]") {
    Temp temp;
    auto sources = TileStore::open({temp.path, "sources"}); REQUIRE(sources);
    auto live = TileStore::open({temp.path, "live"}); REQUIRE(live);
    auto baked1 = TileStore::open({temp.path, "baked1"}); REQUIRE(baked1);
    auto baked4 = TileStore::open({temp.path, "baked4"}); REQUIRE(baked4);
    std::vector<TileKey> soil_keys;
    for (int dx = -1; dx <= 1; ++dx) for (int dy = -1; dy <= 1; ++dy) {
        const TileKey k = g2m_test::home_tile(dx, dy);
        if (dx == 1 && dy == 1) continue;                          // no soil source here: no soil tile
        soil_keys.push_back(k);
        const SoilGridDeriver d; const auto refs = d.inputs_for(k);
        const auto soil = make_input(refs[1], *encode_src_soil(make_src_tile(k)));
        REQUIRE((*sources)->put({"soil_test", soil.ref.layer, k}, *soil.header, soil.body, true));
        if (dx == 0 && dy == -1) continue;                         // and one tile without land cover (unmasked)
        const auto lc = make_input(refs[0], *encode_src_landcover(make_landcover(k)));
        REQUIRE((*sources)->put({"soil_test", lc.ref.layer, k}, *lc.header, lc.body, true));
    }
    BuiltinTerrainDerivers derivers;
    const auto manifest = make_local_release(**sources, builtin_local_release_params("soil_test", derivers, {}, false, false, false, false, false, true));
    REQUIRE(manifest);
    LocalSourceUpstream upstream(**sources);
    ServerConfig config; config.offline = true; config.derived_store = live->get();
    config.derivers = derivers.ordered(false, false, false, false, false, true); config.upstreams = {&upstream};
    Server server(config); REQUIRE(server.add_release(*manifest, "soil_test"));
    std::string release_hex;
    for (const int jobs : {1, 4}) {
        import::BakeOptions options; options.layers = {std::string(kSoilGridLayer)};
        options.min_level = options.max_level = 2; options.jobs = jobs; options.pin = true;
        auto& store = jobs == 1 ? *baked1 : *baked4;
        const auto report = import::bake(**sources, *store, "soil_test", options); REQUIRE(report);
        CHECK(report->release_id_hex == to_hex(manifest->rid)); REQUIRE(report->layers.size() == 1);
        CHECK(report->layers[0].tiles_total == soil_keys.size());   // coverage follows the raw soil layer
        for (const auto& k : soil_keys) {
            const auto response = server.tile({manifest->rid, std::string(kSoilGridLayer), k, {}});
            REQUIRE(response.meta.status == Status::Ok);
            const auto stored = store->get({report->release_id_hex, std::string(kSoilGridLayer), k});
            REQUIRE(stored); REQUIRE(*stored); CHECK((*stored)->pinned);
            CHECK((*stored)->container() == response.container);
            const auto tile = decode_soil_grid(body_of((*stored)->body)); REQUIRE(tile);
            const bool plain = k == g2m_test::home_tile(0, -1);
            for (int r = 0; r < 32; ++r) for (int c = 0; c < 32; ++c)
                REQUIRE(tile->cells[static_cast<std::size_t>(r) * 32 + static_cast<std::size_t>(c)] == ref_cell(k, c, r, !plain));
        }
        // A tile without soil source answers NotFound, not an empty tile.
        const auto missing = server.tile({manifest->rid, std::string(kSoilGridLayer), g2m_test::home_tile(1, 1), {}});
        CHECK(missing.meta.status != Status::Ok);
        release_hex = report->release_id_hex;
    }
    for (const auto& k : soil_keys)
        CHECK((*(*baked1)->get({release_hex, std::string(kSoilGridLayer), k}))->container() == (*(*baked4)->get({release_hex, std::string(kSoilGridLayer), k}))->container());
}
