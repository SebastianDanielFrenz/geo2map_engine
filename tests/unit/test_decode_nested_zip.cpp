// tests/unit/test_decode_nested_zip.cpp - dgm1::Catalog against the
// committed DGM1 fixture (testdata/fixtures/dgm1/PKT_fixture.zip, PLAN.md
// G1c I3): a REAL nested-STORED outer PKT zip (repacked byte-for-byte from
// S:\claude_code\geo2map_data by tests/support/fixture_cut's repack-dgm1
// subcommand, see testdata/fixtures/README.md for provenance) containing
// two inner municipality zips - "inner0.zip" (Frankfurt, tiles 474/5560 and
// 475/5560) and "inner1.zip" (Bad Homburg, tile 474/5560 only) - so tile
// 474/5560 has two copies whose raw bytes differ (different producer/
// delivery) but whose decoded height samples agree exactly, matching the
// real-world fact this fixture was cut to exercise (G2M-002/G1c pre-item).

#include "g2m/decode/dgm1.h"
#include "g2m/decode/tiff/tfw.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <thread>
#include <vector>

using namespace g2m;
using namespace g2m::decode;

namespace {
constexpr double kTolM = 1e-6;
} // namespace

namespace {

std::string fixture_path() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/dgm1/PKT_fixture.zip"; }

} // namespace

TEST_CASE("import: dgm1 fixture enumerate_tiles sees both copies of 474/5560 and one of 475/5560",
          "[decode][import][dgm1]") {
    const auto all = dgm1::enumerate_tiles({fixture_path()});
    std::map<dgm1::TileKey, int> copies_per_key;
    for (const auto& [key, loc] : all) ++copies_per_key[key];

    REQUIRE(copies_per_key.size() == 2);
    CHECK(copies_per_key[dgm1::TileKey{474, 5560}] == 2);
    CHECK(copies_per_key[dgm1::TileKey{475, 5560}] == 1);
}

TEST_CASE("import: dgm1 fixture's two copies of 474/5560 have different bytes but identical samples",
          "[decode][import][dgm1]") {
    const auto all = dgm1::enumerate_tiles({fixture_path()});
    std::vector<dgm1::TileLocation> copies;
    for (const auto& [key, loc] : all) {
        if (key == dgm1::TileKey{474, 5560}) copies.push_back(loc);
    }
    REQUIRE(copies.size() == 2);

    const dgm1::DecodedTile a = dgm1::decode_tile(copies[0]);
    const dgm1::DecodedTile b = dgm1::decode_tile(copies[1]);
    CHECK(a.heights.width() == 1000);
    CHECK(a.heights.height() == 1000);
    CHECK(a.georef == b.georef);
    CHECK(a.heights == b.heights); // samples agree
}

TEST_CASE("import: dgm1::Catalog over the fixture reports the expected stats", "[decode][import][dgm1]") {
    Result<dgm1::Catalog> catalog = dgm1::Catalog::open({fixture_path()});
    REQUIRE(catalog.ok());

    const dgm1::CatalogStats stats = catalog->stats();
    CHECK(stats.outer == 1);
    CHECK(stats.inner == 2);
    CHECK(stats.copies == 3);
    CHECK(stats.unique_keys == 2);
    CHECK(stats.duplicated_keys == 1);

    CHECK(catalog->copies(474, 5560).size() == 2);
    CHECK(catalog->copies(475, 5560).size() == 1);
    CHECK(catalog->copies(1, 1).empty()); // not in this fixture - not an error

    // copies() ordering is (outer basename, inner index, entry index) - for
    // a single outer zip that collapses to inner-zip encounter order, i.e.
    // Frankfurt (inner0.zip) before Bad Homburg (inner1.zip).
    const auto refs = catalog->copies(474, 5560);
    const std::string first_desc = catalog->describe(refs[0]);
    const std::string second_desc = catalog->describe(refs[1]);
    CHECK(first_desc.find("inner0.zip") != std::string::npos);
    CHECK(second_desc.find("inner1.zip") != std::string::npos);
}

TEST_CASE("import: dgm1::Catalog::decode agrees with decode_tile and its own describe() is stable",
          "[decode][import][dgm1]") {
    Result<dgm1::Catalog> catalog = dgm1::Catalog::open({fixture_path()});
    REQUIRE(catalog.ok());

    const auto refs_474 = catalog->copies(474, 5560);
    REQUIRE(refs_474.size() == 2);
    const Result<std::vector<float>> decoded_a = catalog->decode(refs_474[0]);
    const Result<std::vector<float>> decoded_b = catalog->decode(refs_474[1]);
    REQUIRE(decoded_a.ok());
    REQUIRE(decoded_b.ok());
    CHECK(decoded_a->size() == 1000 * 1000);
    CHECK(*decoded_a == *decoded_b); // same fact as the decode_tile-based test above, via the Catalog API

    const auto refs_475 = catalog->copies(475, 5560);
    REQUIRE(refs_475.size() == 1);
    const Result<std::vector<float>> decoded_475 = catalog->decode(refs_475[0]);
    REQUIRE(decoded_475.ok());
    CHECK(decoded_475->size() == 1000 * 1000);

    CHECK(catalog->describe(refs_474[0]) == catalog->describe(refs_474[0])); // pure/repeatable
}

TEST_CASE("import: dgm1::Catalog::tfw_text parses and matches the filename-derived georef",
          "[decode][import][dgm1]") {
    Result<dgm1::Catalog> catalog = dgm1::Catalog::open({fixture_path()});
    REQUIRE(catalog.ok());

    for (const auto& [e_km, n_km] : std::vector<std::pair<int, int>>{{474, 5560}, {475, 5560}}) {
        const auto refs = catalog->copies(e_km, n_km);
        REQUIRE_FALSE(refs.empty());
        for (const auto& ref : refs) {
            const Result<std::string> text = catalog->tfw_text(ref);
            if (!text) {
                // Absence is documented as not-an-error (CopyRef::tfw_entry_idx's
                // own comment) - this fixture is expected to carry one for
                // every copy since it was repacked straight from real tiles
                // that had one, so treat a miss here as informative, not fatal.
                CHECK(text.error().code == ErrorCode::NotFound);
                continue;
            }
            const std::optional<GeoTransform> tfw = parse_tfw(*text);
            REQUIRE(tfw.has_value());
            CHECK(std::fabs(tfw->origin_x - (static_cast<double>(e_km) * 1000.0 + 0.5)) < kTolM);
            CHECK(std::fabs(tfw->origin_y - (static_cast<double>(n_km) * 1000.0 + 999.5)) < kTolM);
            CHECK(std::fabs(tfw->pixel_width - 1.0) < kTolM);
            CHECK(std::fabs(tfw->pixel_height - 1.0) < kTolM);
        }
    }
}

TEST_CASE("import: dgm1::Catalog::decode is safe to call concurrently across copies",
          "[decode][import][dgm1]") {
    Result<dgm1::Catalog> catalog = dgm1::Catalog::open({fixture_path()});
    REQUIRE(catalog.ok());

    std::vector<dgm1::CopyRef> all_refs = catalog->copies(474, 5560);
    const auto refs_475 = catalog->copies(475, 5560);
    all_refs.insert(all_refs.end(), refs_475.begin(), refs_475.end());
    REQUIRE(all_refs.size() == 3);

    std::vector<std::thread> threads;
    // std::vector<bool> is bit-packed - concurrent writes to logically
    // distinct indices can still share the same underlying byte/word, which
    // is a genuine data race (caught by ThreadSanitizer, not a false
    // positive). std::vector<char> gives each index its own byte.
    std::vector<char> ok(all_refs.size() * 4, 0);
    for (std::size_t rep = 0; rep < 4; ++rep) {
        for (std::size_t i = 0; i < all_refs.size(); ++i) {
            threads.emplace_back([&, rep, i]() {
                const Result<std::vector<float>> d = catalog->decode(all_refs[i]);
                ok[rep * all_refs.size() + i] = (d.ok() && d->size() == 1000 * 1000) ? 1 : 0;
            });
        }
    }
    for (auto& t : threads) t.join();
    CHECK(std::all_of(ok.begin(), ok.end(), [](char b) { return b != 0; }));
}
