// tests/unit/test_import_dem.cpp - g2m::import::import_dem against the
// committed DGM1 fixture (testdata/fixtures/dgm1/PKT_fixture.zip) and the
// fixture region's utm_bbox part (testdata/fixtures/regions.json, PLAN.md
// G1c pre-item a: 474000/5560000 - 476000/5561000, chosen so its level-2
// tile footprint actually overlaps the DGM1 fixture's two 1 km squares,
// 474/5560 and 475/5560). Mirrors test_import_resolve.cpp's own fixture-
// reading conventions (fixtures_dir(), read_file()).

#include "g2m/core/raster.h"
#include "g2m/core/tile_key.h"
#include "g2m/decode/dgm1.h"
#include "g2m/import/dem.h"
#include "g2m/import/region_resolve.h"
#include "g2m/import/sink.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/region.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/tile_container.h"
#include "tiff_builder.h"
#include "zip_builder.h"

#include <libdeflate.h>

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

using namespace g2m;
using namespace g2m::import;
namespace dgm1 = g2m::decode::dgm1;

namespace {

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

// Resolves a bare utm_bbox-only region against the committed fixture PBF
// (reused purely for its sha256 pass, same rationale as
// test_import_resolve.cpp's own "handles a utm_bbox part" test - the PBF's
// content is unrelated to a UtmBboxPart's geometry).
Result<ResolvedRegion> resolve_bbox_region(std::int64_t west, std::int64_t south, std::int64_t east,
                                            std::int64_t north) {
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    RegionSpec spec;
    spec.name = "dgm1_fixture";
    spec.scheme = "utm32n";
    spec.halo_m = 0;
    UtmBboxPart bbox;
    bbox.west = west;
    bbox.south = south;
    bbox.east = east;
    bbox.north = north;
    spec.parts.push_back(bbox);
    return resolve_region(spec, pbf_path);
}

Result<dgm1::Catalog> open_fixture_catalog() {
    return dgm1::Catalog::open({fixtures_dir() + "/dgm1/PKT_fixture.zip"});
}

// Field-wise equality for MemorySink::entries() (Entry itself has no
// operator== - TileHeader does, std::vector<uint8_t> does via its own
// built-in operator==).
bool same_entries(const std::vector<std::pair<std::pair<std::string, TileKey>, MemorySink::Entry>>& a,
                   const std::vector<std::pair<std::pair<std::string, TileKey>, MemorySink::Entry>>& b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (!(a[i].first == b[i].first)) return false;
        if (!(a[i].second.header == b[i].second.header)) return false;
        if (a[i].second.body != b[i].second.body) return false;
    }
    return true;
}

} // namespace

// ---- PLAN.md G1c pre-item (b)/(c)/(d): DEM tests I3 skipped ----------------

namespace {

using g2m::test_support::TiffBuilder;
using g2m::test_support::ZipBuilder;

std::vector<std::byte> deflate_zlib_compress(const std::vector<std::byte>& in) {
    libdeflate_compressor* c = libdeflate_alloc_compressor(6);
    std::vector<std::byte> out(in.size() + 4096);
    const std::size_t n = libdeflate_zlib_compress(c, in.data(), in.size(), out.data(), out.size());
    libdeflate_free_compressor(c);
    out.resize(n);
    return out;
}

// Predictor 2 (horizontal differencing) over one row's raw 32-bit WORDS, in
// place - DGM1's own shape (test_decode_tiff.cpp's "DGM1 shape" test; G2M-002:
// predictor 2 over float32 Deflate data differences the raw uint32 word, not
// individual bytes).
void apply_predictor2_encode_u32(std::vector<std::byte>& row, int width) {
    auto* p = reinterpret_cast<std::uint32_t*>(row.data());
    for (int i = width - 1; i >= 1; --i) p[i] = static_cast<std::uint32_t>(p[i] - p[i - 1]);
}

// Builds one synthetic 1000x1000 float32 DGM1-shaped TIFF (Deflate +
// horizontal-differencing predictor, matching every real Hessen delivery
// tile's own encoding) from `samples` (row-major, 1000*1000 floats, metres,
// row 0 = north per dgm1.h's own NorthToSouth convention).
std::vector<std::byte> build_dgm1_tiff(const std::vector<float>& samples) {
    constexpr int w = 1000, h = 1000;
    REQUIRE(samples.size() == static_cast<std::size_t>(w) * h);
    std::vector<std::byte> raw(samples.size() * 4);
    std::memcpy(raw.data(), samples.data(), raw.size());
    std::vector<std::byte> encoded(raw.size());
    for (int y = 0; y < h; ++y) {
        std::vector<std::byte> row(raw.begin() + static_cast<std::ptrdiff_t>(y) * w * 4,
                                    raw.begin() + static_cast<std::ptrdiff_t>(y + 1) * w * 4);
        apply_predictor2_encode_u32(row, w);
        std::memcpy(encoded.data() + static_cast<std::ptrdiff_t>(y) * w * 4, row.data(), row.size());
    }
    TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(w));
    b.add_short(257, static_cast<std::uint16_t>(h));
    b.add_short(258, 32);
    b.add_short(259, 8); // Deflate
    b.add_short(277, 1);
    b.add_long(278, static_cast<std::uint32_t>(h));
    b.add_short(284, 1);
    b.add_short(317, 2); // Predictor::HorizontalDifferencing
    b.add_short(339, 3); // Float
    return b.build(deflate_zlib_compress(encoded));
}

// Searches an already-decoded ElevTile for the sample nearest a UTM point (to
// 1e-6 m - both this test file's target coordinates and RasterGeoref's own
// sample-position formulas are exact dyadic fractions of a metre, so an exact
// match is expected whenever the tile covers the point). Returns false if the
// point falls outside this tile's own footprint.
bool find_sample(const ElevTile& tile, double target_easting_m, double target_northing_m, int* out_col,
                  int* out_row) {
    int col = -1;
    for (int c = 0; c < tile.heights.width(); ++c) {
        if (std::fabs(tile.georef.sample_easting_m(c) - target_easting_m) < 1e-6) {
            col = c;
            break;
        }
    }
    if (col < 0) return false;
    int row = -1;
    for (int r = 0; r < tile.heights.height(); ++r) {
        if (std::fabs(tile.georef.sample_northing_m(r) - target_northing_m) < 1e-6) {
            row = r;
            break;
        }
    }
    if (row < 0) return false;
    *out_col = col;
    *out_row = row;
    return true;
}

} // namespace

TEST_CASE("import: a written sample matches quantise_height_half_even of the raw TIFF value directly",
          "[import][io][dgm1]") {
    // Km square 475/5560 carries exactly one copy in the fixture (no
    // duplicate-copy averaging/tie-break to reason about - see
    // test_decode_nested_zip.cpp's own catalog stats). Cell registration +
    // NorthToSouth row order (dgm1.h's own header comment): sample centre
    // (E*1000 + 0.5 + col, N*1000 + 999.5 - row); for E=475500.5,
    // N=5560500.5 that is local_col=500, local_row=499.
    Result<dgm1::Catalog> catalog = open_fixture_catalog();
    REQUIRE(catalog.ok());
    const auto refs = catalog->copies(475, 5560);
    REQUIRE(refs.size() == 1);
    const Result<std::vector<float>> decoded = catalog->decode(refs.front());
    REQUIRE(decoded.ok());
    const float raw_metres = (*decoded)[499 * 1000 + 500];
    const std::optional<std::int32_t> expected = quantise_height_half_even(static_cast<double>(raw_metres));
    REQUIRE(expected.has_value());

    Result<ResolvedRegion> resolved = resolve_bbox_region(474000, 5560000, 476000, 5561000);
    REQUIRE(resolved.ok());
    MemorySink sink;
    Result<DemImportReport> report = import_dem(*resolved, *catalog, sink, "test", DemImportOptions{});
    REQUIRE(report.ok());

    constexpr double kTargetEasting = 475500.5;
    constexpr double kTargetNorthing = 5560500.5;
    bool found = false;
    for (const auto& [key, entry] : sink.entries()) {
        if (key.first != std::string(kSrcElevDgm1Layer)) continue;
        Result<TileBody> body = decode_body(entry.body);
        REQUIRE(body.ok());
        Result<ElevTile> tile = decode_src_elev(*body);
        REQUIRE(tile.ok());
        int col = 0, row = 0;
        if (!find_sample(*tile, kTargetEasting, kTargetNorthing, &col, &row)) continue;
        CHECK(tile->heights.at(col, row) == *expected);
        found = true;
        break;
    }
    CHECK(found);
}

TEST_CASE("import: quantise_height_half_even ties to even - a tie that stays even", "[import][io][dgm1]") {
    // 600.001953125 m == 600 + 1/512 exactly (0.001953125 == 1/512), so
    // metres*256 == 153600.5 exactly - a genuine tie. Its floor r = 153600 is
    // EVEN, so half-even rounding keeps it rather than rounding up.
    const auto q = quantise_height_half_even(600.001953125);
    REQUIRE(q.has_value());
    CHECK(*q == 153600);
}

TEST_CASE("import: quantise_height_half_even ties to even - a tie that rounds up at an odd r",
          "[import][io][dgm1]") {
    // 600.005859375 m == 600 + 3/512 exactly (0.005859375 == 3/512), so
    // metres*256 == 153601.5 exactly - a tie whose floor r = 153601 is ODD,
    // so half-even rounding rounds away from r, up to 153602.
    const auto q = quantise_height_half_even(600.005859375);
    REQUIRE(q.has_value());
    CHECK(*q == 153602);
}

TEST_CASE("import: a synthetic differing duplicate is reported and copies[0] wins, import still succeeds",
          "[import][io][dgm1]") {
    // Two copies of the SAME km key (900/6000, chosen well away from the
    // real fixture's 474-475/5560 squares) inside one inner zip, entry order
    // A then B (Catalog::copies()' own "(outer basename, inner index, entry
    // index)" order - see dgm1.cpp's Catalog::open) - so copies[0] is A.
    // They differ at exactly one sample (row 0 / col 0, the tile's NW
    // corner - E=900000.5, N=6000999.5) so dup_differing sees a real
    // disagreement, not a decode artifact.
    std::vector<float> samples_a(1000 * 1000, 500.0f);
    std::vector<float> samples_b = samples_a;
    samples_b[0] = 501.0f;
    REQUIRE(samples_a[0] != samples_b[0]);

    ZipBuilder inner;
    inner.add({"dgm1_32_900_6000_1_he.tif", build_dgm1_tiff(samples_a), /*deflate=*/false});
    inner.add({"dgm1_32_900_6000_1_he_dup.tif", build_dgm1_tiff(samples_b), /*deflate=*/false});
    const std::vector<std::byte> inner_bytes = inner.build();

    ZipBuilder outer;
    outer.add({"muni.zip", inner_bytes, /*deflate=*/false});
    const std::vector<std::byte> outer_bytes = outer.build();

    const std::string path =
        (std::filesystem::temp_directory_path() / "g2m_test_dgm1_differing_dup.zip").string();
    {
        std::ofstream out(path, std::ios::binary | std::ios::trunc);
        out.write(reinterpret_cast<const char*>(outer_bytes.data()), static_cast<std::streamsize>(outer_bytes.size()));
    }

    // Scoped so the Catalog's open file handle on `path` (Windows: an
    // exclusive-ish handle via FileByteSource) is released before the
    // removal attempt below - otherwise the remove can lose a race with the
    // handle's own teardown (seen once: "Der Prozess kann nicht auf die
    // Datei zugreifen ...").
    {
        Result<dgm1::Catalog> catalog = dgm1::Catalog::open({path});
        REQUIRE(catalog.ok());
        const auto refs = catalog->copies(900, 6000);
        REQUIRE(refs.size() == 2);
        CHECK(catalog->describe(refs[0]).find("dgm1_32_900_6000_1_he.tif") != std::string::npos);
        CHECK(catalog->describe(refs[1]).find("dgm1_32_900_6000_1_he_dup.tif") != std::string::npos);

        Result<ResolvedRegion> resolved = resolve_bbox_region(900000, 6000000, 901000, 6001000);
        REQUIRE(resolved.ok());

        MemorySink sink;
        Result<DemImportReport> report = import_dem(*resolved, *catalog, sink, "test", DemImportOptions{});
        REQUIRE(report.ok()); // a differing duplicate is reported, never a failure
        CHECK(report->dup_keys == 1);
        CHECK(report->dup_identical == 0);
        CHECK(report->dup_differing == 1);
        REQUIRE_FALSE(report->differing.empty());
        CHECK(report->differing.front().find("900/6000") != std::string::npos);

        // copies[0] (A, sample 500.0) wins - the written height at the
        // differing corner matches A, not B (501.0).
        const std::optional<std::int32_t> expected_a = quantise_height_half_even(500.0);
        REQUIRE(expected_a.has_value());
        bool found = false;
        for (const auto& [key, entry] : sink.entries()) {
            if (key.first != std::string(kSrcElevDgm1Layer)) continue;
            Result<TileBody> body = decode_body(entry.body);
            REQUIRE(body.ok());
            Result<ElevTile> tile = decode_src_elev(*body);
            REQUIRE(tile.ok());
            int col = 0, row = 0;
            if (!find_sample(*tile, 900000.5, 6000999.5, &col, &row)) continue;
            CHECK(tile->heights.at(col, row) == *expected_a);
            found = true;
            break;
        }
        CHECK(found);
    }

    // Best-effort cleanup (matches test_decode_osm.cpp's own retry pattern
    // for a scratch PBF/zip file on Windows) - not a correctness assertion,
    // so a stray leftover temp file is not a test failure.
    for (int attempt = 0; attempt < 20; ++attempt) {
        std::error_code ec;
        if (std::filesystem::remove(path, ec) || ec == std::errc::no_such_file_or_directory) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

TEST_CASE("import: import_dem writes the fixture region's DGM1-covered tiles and accounts for every tile",
          "[import][io][dgm1]") {
    // Matches testdata/fixtures/regions.json's own utm_bbox part exactly.
    Result<ResolvedRegion> resolved = resolve_bbox_region(474000, 5560000, 476000, 5561000);
    REQUIRE(resolved.ok());
    REQUIRE(resolved->tile_count > 0);

    Result<dgm1::Catalog> catalog = open_fixture_catalog();
    REQUIRE(catalog.ok());
    CHECK(catalog->stats().duplicated_keys == 1); // 474/5560 has 2 copies (Frankfurt + Bad Homburg)

    MemorySink sink;
    Result<DemImportReport> report = import_dem(*resolved, *catalog, sink, "test", DemImportOptions{});
    REQUIRE(report.ok());

    // Every region tile is written, or skipped for one of the two documented
    // reasons - never silently dropped.
    CHECK(report->tiles_written + report->tiles_skipped_nocov + report->tiles_skipped_allnodata ==
          resolved->tile_count);
    // The bbox sits squarely over the fixture's two DGM1 squares, so at
    // least one tile is actually written.
    CHECK(report->tiles_written > 0);
    CHECK(sink.size() == report->tiles_written);

    // 474/5560's two copies (byte-different, sample-identical - see
    // test_decode_nested_zip.cpp) are decoded identically whenever a tile
    // touching that km square pulls in both copies for comparison.
    CHECK(report->dup_keys == 1);
    // PLAN.md G1c pre-item (a): exactly one dup decode happens against this
    // fixture (km 474/5560, the only duplicated key it carries) - not just
    // "at least one".
    CHECK(report->dup_identical == 1);
    CHECK(report->dup_differing == 0);
    CHECK(report->differing.empty());

    // Spot-check one written tile round-trips through the real encode/decode
    // pipeline (encode_body/decode_body, encode_src_elev/decode_src_elev),
    // matching the exact bytes import_dem itself hands to sink.put()
    // (dem.cpp: sink.put(header.layer, key, header, encoded_body) where
    // encoded_body == encode_body(encode_src_elev(tile)) - not a full
    // assemble_container()).
    const auto entries = sink.entries();
    REQUIRE_FALSE(entries.empty());
    const auto& first = entries.front();
    CHECK(first.first.first == std::string(kSrcElevDgm1Layer));
    CHECK(first.second.header.layer == std::string(kSrcElevDgm1Layer));
    CHECK(first.second.header.body_size == first.second.body.size());

    Result<TileBody> body = decode_body(first.second.body);
    REQUIRE(body.ok());
    Result<ElevTile> tile = decode_src_elev(*body);
    REQUIRE(tile.ok());
    CHECK(tile->heights.width() > 0);
    CHECK(tile->heights.height() > 0);
}

TEST_CASE("import: import_dem skips level-2 tiles with no DGM1 coverage at all", "[import][io][dgm1]") {
    // Extends the fixture bbox east past both DGM1 squares (474/475 x 5560)
    // into 476/5560 and beyond, which the fixture does not carry - some
    // resulting level-2 tile(s) must therefore see zero DGM1 coverage.
    Result<ResolvedRegion> resolved = resolve_bbox_region(474000, 5560000, 479000, 5561000);
    REQUIRE(resolved.ok());

    Result<dgm1::Catalog> catalog = open_fixture_catalog();
    REQUIRE(catalog.ok());

    MemorySink sink;
    Result<DemImportReport> report = import_dem(*resolved, *catalog, sink, "test", DemImportOptions{});
    REQUIRE(report.ok());

    CHECK(report->tiles_written + report->tiles_skipped_nocov + report->tiles_skipped_allnodata ==
          resolved->tile_count);
    CHECK(report->tiles_skipped_nocov > 0);
    // The near (474/475) end of the bbox is still covered.
    CHECK(report->tiles_written > 0);
}

TEST_CASE("import: import_dem's tile output is independent of --jobs and --km-cache", "[import][io][dgm1]") {
    Result<ResolvedRegion> resolved = resolve_bbox_region(474000, 5560000, 476000, 5561000);
    REQUIRE(resolved.ok());
    Result<dgm1::Catalog> catalog = open_fixture_catalog();
    REQUIRE(catalog.ok());

    MemorySink sink_a;
    DemImportOptions opts_a;
    opts_a.jobs = 1;
    opts_a.km_cache_tiles = 1; // forces eviction/re-decode churn
    Result<DemImportReport> report_a = import_dem(*resolved, *catalog, sink_a, "a", opts_a);
    REQUIRE(report_a.ok());

    MemorySink sink_b;
    DemImportOptions opts_b;
    opts_b.jobs = 4;
    opts_b.km_cache_tiles = 256;
    Result<DemImportReport> report_b = import_dem(*resolved, *catalog, sink_b, "b", opts_b);
    REQUIRE(report_b.ok());

    // The counters this run measured may differ (scheduling-dependent, see
    // DemImportReport's own doc comment), but the actual tile OUTPUT must not.
    CHECK(report_a->tiles_written == report_b->tiles_written);
    CHECK(same_entries(sink_a.entries(), sink_b.entries()));
}
