// tests/unit/test_import_landcover.cpp - g2m::import::import_landcover
// (PLAN.md G1c I5).
//
// Validation-rejection tests and the jobs-invariance / sample-point /
// all-zero-skip tests run against small SYNTHETIC GeoTIFFs built in-process
// with tests/support/tiff_builder.h - never the real 94 MB WorldCover
// product, which would make this file slow and would also require shipping
// a licensed asset just to test a rejection path. Two [real-data] TEST_CASEs
// (gated on G2M_DATA_DIR, same pattern as test_import_resolve.cpp/
// test_import_dem.cpp) exercise the real file and the checked-in
// testdata/fixtures/worldcover/*.tif fixtures.
//
// Synthetic fixtures use N = 2048 (a power of two, NOT the real product's
// N = 12000): dividing/multiplying back by a power of two never rounds in
// IEEE-754 double, so 1.0/N and any integer-numerator C/N tiepoint ratio
// round-trip validate_ifd0's "1.0/n == sx" / "tiepoint corner is an exact
// multiple of 1/N" checks exactly without having to hunt for an N=12000
// combination that happens to divide exactly - landcover.cpp's own
// validate_ifd0 derives N from the file itself and never assumes 12000.
//
// "20 fixed sample points vs brute-force reference" (the plan's own words):
// interpreted here as computing each sample's expected world pixel index
// (p = floor(lon*N), q = floor(lat*N)) twice - once in `double` (exactly
// landcover.cpp's own arithmetic) and once in `long double` - and requiring
// the two agree, since long double is not universally wider than double on
// every target this project builds for (MSVC's long double IS double) and a
// synthetic pattern keyed on the world pixel index itself is a perfectly
// good "brute-force reference" that does not depend on decoding a window at
// all. A disagreement is only tolerated within 1e-9 of a true pixel-edge
// fraction (frac(lon*N) or frac(lat*N) within 1e-9 of an integer) - D17's
// own documented "one rounding, edge resolves to the east/south pixel" edge
// rule, which a long-double recomputation can legitimately cross.

#include "g2m/core/geo/utm.h"
#include "g2m/core/raster.h"
#include "g2m/core/tile_key.h"
#include "g2m/decode/byte_source.h"
#include "g2m/decode/tiff/tiff_reader.h"
#include "g2m/import/landcover.h"
#include "g2m/import/sink.h"
#include "g2m/layer/region.h"
#include "g2m/layer/src_landcover.h"

#include "tiff_builder.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

using namespace g2m;
using namespace g2m::import;

namespace {

std::optional<std::string> data_dir() {
#if defined(_MSC_VER) || defined(_WIN32)
    char* buf = nullptr;
    std::size_t len = 0;
    if (_dupenv_s(&buf, &len, "G2M_DATA_DIR") != 0 || buf == nullptr) return std::nullopt;
    std::string value(buf);
    std::free(buf);
    if (value.empty()) return std::nullopt;
    return value;
#else
    const char* p = std::getenv("G2M_DATA_DIR");
    if (p == nullptr || *p == '\0') return std::nullopt;
    return std::string(p);
#endif
}

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

void write_bytes(const std::filesystem::path& path, const std::vector<std::byte>& bytes) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    REQUIRE(out.is_open());
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

std::filesystem::path temp_path(const std::string& name) {
    return std::filesystem::temp_directory_path() / ("g2m_test_landcover_" + name + ".tif");
}

// One arbitrary valid German-location tile, shared by every synthetic test
// below - deliberately NOT the real-data tests' own tile (32N/2/465/5440),
// so nothing in this file can be confused with real WorldCover content.
const TileKey kTestTile{geo::UtmZone{32, geo::Hemisphere::North}, /*level=*/2, /*x=*/460, /*y=*/5560};

ResolvedRegion single_tile_region(const TileKey& key) {
    ResolvedRegion r;
    r.name = "test";
    r.scheme = geo::utm_scheme_name(key.zone);
    r.level = key.level;
    r.halo_m = 0;
    r.tiles = {CoverageRun{key.y, key.x, key.x}};
    r.interior = r.tiles;
    r.tile_count = 1;
    return r;
}

constexpr std::int64_t kSynthN = 2048; // power of two: 1/N and C/N round-trip exactly

// Deterministic non-zero class pattern over WORLD pixel indices
// (p = floor(lon*N), q = floor(lat*N)). The synthetic source raster is
// filled by this function directly, so a test can predict any destination
// sample's expected class from its own independently-computed (p, q)
// without decoding anything back out of the produced tile.
std::uint8_t pattern_class(std::int64_t p, std::int64_t q) {
    const auto h = static_cast<std::uint64_t>(p) * 1000003ull + static_cast<std::uint64_t>(q) * 999983ull;
    return static_cast<std::uint8_t>(10 + 10 * (h % 10)); // 10..100, never 0
}

struct SyntheticFile {
    std::filesystem::path path;
    std::int64_t n = kSynthN;
    std::int64_t c0 = 0; // lon0 * n (west edge of pixel col 0)
    std::int64_t c1 = 0; // lat0 * n (north edge of row 0)
    std::int64_t width = 0;
    std::int64_t height = 0;
};

// Builds a synthetic WorldCover-shaped GeoTIFF covering `key`'s own full
// footprint plus `margin_px` source pixels on every side, filled by
// pattern_class (or all zero). Tag set mirrors landcover.cpp's own
// validate_ifd0 / cut-worldcover's writer exactly (this file's header
// comment).
SyntheticFile build_synthetic_tif(const std::filesystem::path& path, const TileKey& key, int margin_px,
                                   bool all_zero) {
    const std::optional<RasterGeoref> georef =
        tile_georef(key, /*cells_per_side=*/256, SampleRegistration::Cell, RowOrder::SouthToNorth);
    REQUIRE(georef.has_value());

    const geo::Utm utm;
    std::int64_t p_min = std::numeric_limits<std::int64_t>::max();
    std::int64_t p_max = std::numeric_limits<std::int64_t>::min();
    std::int64_t q_min = std::numeric_limits<std::int64_t>::max();
    std::int64_t q_max = std::numeric_limits<std::int64_t>::min();
    for (int r = 0; r < georef->height; ++r) {
        const double northing_m = georef->sample_northing_m(r);
        for (int c = 0; c < georef->width; ++c) {
            const double easting_m = georef->sample_easting_m(c);
            const geo::TmReverse rev = utm.reverse(georef->zone, easting_m, northing_m);
            const auto p = static_cast<std::int64_t>(std::floor(rev.lon * static_cast<double>(kSynthN)));
            const auto q = static_cast<std::int64_t>(std::floor(rev.lat * static_cast<double>(kSynthN)));
            p_min = std::min(p_min, p);
            p_max = std::max(p_max, p);
            q_min = std::min(q_min, q);
            q_max = std::max(q_max, q);
        }
    }
    const std::int64_t p0 = p_min - margin_px, p1 = p_max + margin_px;
    const std::int64_t q0 = q_min - margin_px, q1 = q_max + margin_px;
    const std::int64_t width = p1 - p0 + 1;
    const std::int64_t height = q1 - q0 + 1;
    (void)q0;
    const std::int64_t c0 = p0;
    const std::int64_t c1 = q1 + 1;

    std::vector<std::byte> strip(static_cast<std::size_t>(width) * static_cast<std::size_t>(height));
    for (std::int64_t row = 0; row < height; ++row) {
        const std::int64_t q = q1 - row;
        for (std::int64_t col = 0; col < width; ++col) {
            const std::int64_t p = p0 + col;
            const std::uint8_t v = all_zero ? std::uint8_t{0} : pattern_class(p, q);
            strip[static_cast<std::size_t>(row) * static_cast<std::size_t>(width) + static_cast<std::size_t>(col)] =
                static_cast<std::byte>(v);
        }
    }

    const double sx = 1.0 / static_cast<double>(kSynthN);
    const double tiepoint_lon = static_cast<double>(c0) / static_cast<double>(kSynthN);
    const double tiepoint_lat = static_cast<double>(c1) / static_cast<double>(kSynthN);

    g2m::test_support::TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(width));   // ImageWidth
    b.add_short(257, static_cast<std::uint16_t>(height));  // ImageLength
    b.add_short(258, 8);                                    // BitsPerSample
    b.add_short(259, 1);                                    // Compression: none
    b.add_short(262, 1);                                    // PhotometricInterpretation
    b.add_short(277, 1);                                    // SamplesPerPixel
    b.add_long(278, static_cast<std::uint32_t>(height));    // RowsPerStrip
    b.add_short(284, 1);                                    // PlanarConfiguration
    b.add_short(339, 1);                                    // SampleFormat: unsigned int
    b.add_double_array(33550, {sx, sx, 0.0});               // ModelPixelScaleTag
    b.add_double_array(33922, {0.0, 0.0, 0.0, tiepoint_lon, tiepoint_lat, 0.0}); // ModelTiepointTag
    b.add_short_array(34735, {1, 1, 0, 2, 1025, 0, 1, 1, 2048, 0, 1, 4326});     // GeoKeyDirectory
    b.add_ascii(42113, "0");                                                     // GDAL_NODATA
    write_bytes(path, b.build(strip));

    return SyntheticFile{path, kSynthN, c0, c1, width, height};
}

// A minimal 1x1-pixel TIFF for the three rejection tests: every field the
// caller does not override is a VALID WorldCover-shaped value (tiepoint
// fixed at lon=0/lat=0, so its "corner is an integer multiple of 1/N" check
// trivially holds for any N and never interferes with whichever single
// field a test is deliberately breaking).
void write_minimal_tif(const std::filesystem::path& path, std::uint16_t bits_per_sample, std::uint16_t sample_format,
                       std::uint16_t samples_per_pixel, double sx, double sy,
                       const std::vector<std::uint16_t>& geokeys, bool with_nodata = true) {
    g2m::test_support::TiffBuilder b;
    b.add_short(256, 1);
    b.add_short(257, 1);
    b.add_short(258, bits_per_sample);
    b.add_short(259, 1);
    b.add_short(262, 1);
    b.add_short(277, samples_per_pixel);
    b.add_long(278, 1);
    b.add_short(284, 1);
    b.add_short(339, sample_format);
    b.add_double_array(33550, {sx, sy, 0.0});
    b.add_double_array(33922, {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    if (!geokeys.empty()) b.add_short_array(34735, geokeys);
    if (with_nodata) b.add_ascii(42113, "0");
    const std::size_t bytes_per_sample = bits_per_sample / 8;
    std::vector<std::byte> strip(bytes_per_sample * samples_per_pixel, std::byte{0});
    write_bytes(path, b.build(strip));
}

const std::vector<std::uint16_t> kValidGeoKeys = {1, 1, 0, 2, 1025, 0, 1, 1, 2048, 0, 1, 4326};
const std::vector<std::uint16_t> kProjectedGeoKeys = {1, 1, 0, 2, 1025, 0, 1, 1, 3072, 0, 1, 3857};

} // namespace

TEST_CASE("import_landcover: rejects float32 IFD0", "[import][io]") {
    const auto path = temp_path("float32");
    write_minimal_tif(path, /*bits=*/32, /*sample_format=*/3 /*IEEEFP*/, /*samples=*/1, 1.0 / kSynthN, 1.0 / kSynthN,
                       kValidGeoKeys);

    const ResolvedRegion region = single_tile_region(kTestTile);
    MemorySink sink;
    const Result<LandcoverImportReport> report = import_landcover(region, {path}, sink, "test");
    REQUIRE_FALSE(report.ok());
    CHECK(report.error().code == ErrorCode::Unsupported);
    std::filesystem::remove(path);
}

TEST_CASE("import_landcover: rejects a non-1/N pixel scale", "[import][io]") {
    const auto path = temp_path("badscale");
    // 0.0004889 is not the exact reciprocal of any integer - floor(1/sx+0.5)
    // picks a candidate N but 1.0/N != sx.
    write_minimal_tif(path, 8, /*sample_format=*/1 /*UnsignedInt*/, 1, 0.0004889, 0.0004889, kValidGeoKeys);

    const ResolvedRegion region = single_tile_region(kTestTile);
    MemorySink sink;
    const Result<LandcoverImportReport> report = import_landcover(region, {path}, sink, "test");
    REQUIRE_FALSE(report.ok());
    CHECK(report.error().code == ErrorCode::Unsupported);
    std::filesystem::remove(path);
}

TEST_CASE("import_landcover: rejects a projected CRS (EPSG:3857)", "[import][io]") {
    const auto path = temp_path("epsg3857");
    write_minimal_tif(path, 8, 1, 1, 1.0 / kSynthN, 1.0 / kSynthN, kProjectedGeoKeys);

    const ResolvedRegion region = single_tile_region(kTestTile);
    MemorySink sink;
    const Result<LandcoverImportReport> report = import_landcover(region, {path}, sink, "test");
    REQUIRE_FALSE(report.ok());
    CHECK(report.error().code == ErrorCode::Unsupported);
    std::filesystem::remove(path);
}

TEST_CASE("import_landcover: an all-zero tile is skipped, not written", "[import][io]") {
    const auto path = temp_path("allzero");
    build_synthetic_tif(path, kTestTile, /*margin_px=*/2, /*all_zero=*/true);

    const ResolvedRegion region = single_tile_region(kTestTile);
    MemorySink sink;
    const Result<LandcoverImportReport> report = import_landcover(region, {path}, sink, "test");
    if (!report.ok()) UNSCOPED_INFO("error: " << report.error().message);
    REQUIRE(report.ok());
    CHECK(report->tiles_written == 0);
    CHECK(report->tiles_skipped_allzero == 1);
    CHECK(sink.size() == 0);
    std::filesystem::remove(path);
}

TEST_CASE("import_landcover: tile output is independent of --jobs", "[import][io]") {
    const auto path = temp_path("jobsinvariance");
    build_synthetic_tif(path, kTestTile, /*margin_px=*/2, /*all_zero=*/false);

    // A region wide enough for several level-2 tiles so jobs > 1 actually
    // fans out work: a 4x4 block of tiles all backed by the SAME synthetic
    // file (WorldCover's own real single-file-covers-everything case for
    // this project, landcover.h's own doc comment).
    ResolvedRegion region;
    region.name = "test";
    region.scheme = geo::utm_scheme_name(kTestTile.zone);
    region.level = kTestTile.level;
    for (std::int32_t y = kTestTile.y; y < kTestTile.y + 4; ++y) {
        region.tiles.push_back(CoverageRun{y, kTestTile.x, kTestTile.x + 3});
    }
    region.interior = region.tiles;
    region.tile_count = 16;

    // kTestTile itself is guaranteed to decode against the one synthetic
    // file (built to cover just that tile's own footprint plus a small
    // pixel margin); a handful of its immediate neighbours also pick up a
    // few non-zero samples from that same margin along their shared border
    // (an incidental consequence of the margin, not something this test
    // needs to pin down exactly) and so are written too, while tiles two or
    // more steps away fall entirely outside the file's window and resolve
    // to all-zero (skipped). This test's actual point is that
    // produce()/consume() ordering and the sink.put() sequence do not
    // depend on --jobs - which both the written and the all-zero tiles
    // exercise - not the exact written/skipped split.
    LandcoverImportOptions opt1;
    opt1.jobs = 1;
    MemorySink sink1;
    const Result<LandcoverImportReport> r1 = import_landcover(region, {path}, sink1, "test", opt1);
    REQUIRE(r1.ok());

    LandcoverImportOptions opt8;
    opt8.jobs = 8;
    MemorySink sink8;
    const Result<LandcoverImportReport> r8 = import_landcover(region, {path}, sink8, "test", opt8);
    REQUIRE(r8.ok());

    CHECK(r1->tiles_written == r8->tiles_written);
    CHECK(r1->tiles_skipped_allzero == r8->tiles_skipped_allzero);
    CHECK(r1->tiles_written + r1->tiles_skipped_allzero == 16);
    CHECK(r1->tiles_written >= 1);   // at least kTestTile itself
    CHECK(r1->tiles_written < 16);   // not every tile in the 4x4 block

    const auto entries1 = sink1.entries();
    const auto entries8 = sink8.entries();
    REQUIRE(entries1.size() == entries8.size());
    for (std::size_t i = 0; i < entries1.size(); ++i) {
        CHECK(entries1[i].first == entries8[i].first);
        CHECK(entries1[i].second.header.layer == entries8[i].second.header.layer);
        CHECK(entries1[i].second.header.key == entries8[i].second.header.key);
        CHECK(entries1[i].second.header.datum == entries8[i].second.header.datum);
        CHECK(entries1[i].second.body == entries8[i].second.body);
    }
    std::filesystem::remove(path);
}

TEST_CASE("import_landcover: 20 fixed sample points vs an independently re-derived reference", "[import][io]") {
    const auto path = temp_path("samplepoints");
    build_synthetic_tif(path, kTestTile, /*margin_px=*/2, /*all_zero=*/false);

    const ResolvedRegion region = single_tile_region(kTestTile);
    MemorySink sink;
    const Result<LandcoverImportReport> report = import_landcover(region, {path}, sink, "test");
    REQUIRE(report.ok());
    REQUIRE(report->tiles_written == 1);

    const auto entries = sink.entries();
    REQUIRE(entries.size() == 1);
    const Result<TileBody> body = decode_body(entries[0].second.body);
    REQUIRE(body.ok());
    const Result<LandcoverTile> tile = decode_src_landcover(*body);
    REQUIRE(tile.ok());

    const std::optional<RasterGeoref> georef =
        tile_georef(kTestTile, 256, SampleRegistration::Cell, RowOrder::SouthToNorth);
    REQUIRE(georef.has_value());
    CHECK(tile->georef == *georef);

    const geo::Utm utm;
    // 20 fixed (r, c) sample indices spanning the tile, including its
    // corners and centre.
    const std::vector<std::pair<int, int>> samples = {
        {0, 0},     {0, 255},   {255, 0},   {255, 255}, {128, 128}, {0, 128},   {255, 128}, {128, 0},
        {128, 255}, {17, 42},   {200, 9},   {64, 191},  {191, 64},  {3, 250},   {250, 3},   {77, 77},
        {180, 200}, {50, 210},  {210, 50},  {123, 123},
    };
    REQUIRE(samples.size() == 20);

    int edge_mismatches_tolerated = 0;
    for (const auto& [r, c] : samples) {
        const double easting_m = georef->sample_easting_m(c);
        const double northing_m = georef->sample_northing_m(r);
        const geo::TmReverse rev = utm.reverse(georef->zone, easting_m, northing_m);

        // Reference 1: double, exactly landcover.cpp's own formula.
        const auto p_double = static_cast<std::int64_t>(std::floor(rev.lon * static_cast<double>(kSynthN)));
        const auto q_double = static_cast<std::int64_t>(std::floor(rev.lat * static_cast<double>(kSynthN)));

        // Reference 2: long double cross-check of the same lon*N/lat*N
        // floor step (independent rounding path - on MSVC long double IS
        // double, so this is a no-op re-derivation there, still valid as a
        // second, textually-independent computation).
        const long double lon_ld = static_cast<long double>(rev.lon) * static_cast<long double>(kSynthN);
        const long double lat_ld = static_cast<long double>(rev.lat) * static_cast<long double>(kSynthN);
        const auto p_ld = static_cast<std::int64_t>(std::floor(lon_ld));
        const auto q_ld = static_cast<std::int64_t>(std::floor(lat_ld));

        const double lon_frac = rev.lon * static_cast<double>(kSynthN) - std::floor(rev.lon * static_cast<double>(kSynthN));
        const double lat_frac = rev.lat * static_cast<double>(kSynthN) - std::floor(rev.lat * static_cast<double>(kSynthN));
        const bool near_lon_edge = lon_frac < 1e-9 || lon_frac > 1.0 - 1e-9;
        const bool near_lat_edge = lat_frac < 1e-9 || lat_frac > 1.0 - 1e-9;

        if (p_double != p_ld) {
            REQUIRE(near_lon_edge);
            ++edge_mismatches_tolerated;
            continue; // expected class is ambiguous within 1e-9 of a pixel edge
        }
        if (q_double != q_ld) {
            REQUIRE(near_lat_edge);
            ++edge_mismatches_tolerated;
            continue;
        }

        const std::uint8_t expected = pattern_class(p_double, q_double);
        const std::uint8_t actual = tile->classes.at(c, r);
        CHECK(static_cast<int>(actual) == static_cast<int>(expected));
    }
    // Sanity: this synthetic tile's own geometry should not put every
    // sample within 1e-9 of a pixel edge - if it did, the test above would
    // be vacuous.
    CHECK(edge_mismatches_tolerated < 20);
    std::filesystem::remove(path);
}

TEST_CASE("real-data: WorldCover tile 32N/2/465/5440 zero fraction and class histogram", "[import][io][real-data][long]") {
    const auto dir = data_dir();
    if (!dir) SKIP("G2M_DATA_DIR not set");
    const std::string tif_path = *dir + "/ESA_WorldCover_10m_2021_v200_N48E006_Map.tif";
    if (!std::filesystem::exists(tif_path)) SKIP("ESA_WorldCover_10m_2021_v200_N48E006_Map.tif not found under G2M_DATA_DIR");

    const TileKey key{geo::UtmZone{32, geo::Hemisphere::North}, 2, 465, 5440};
    const ResolvedRegion region = single_tile_region(key);
    MemorySink sink;
    const Result<LandcoverImportReport> report = import_landcover(region, {tif_path}, sink, "test");
    REQUIRE(report.ok());
    REQUIRE(report->tiles_written == 1);
    REQUIRE(report->tiles_skipped_allzero == 0);

    const auto entries = sink.entries();
    REQUIRE(entries.size() == 1);
    const Result<TileBody> body = decode_body(entries[0].second.body);
    REQUIRE(body.ok());
    const Result<LandcoverTile> tile = decode_src_landcover(*body);
    REQUIRE(tile.ok());

    std::map<int, std::uint64_t> histogram;
    std::uint64_t zero_count = 0;
    const auto samples = tile->classes.data();
    for (std::uint8_t v : samples) {
        ++histogram[v];
        if (v == 0) ++zero_count;
    }
    const double zero_fraction = static_cast<double>(zero_count) / static_cast<double>(samples.size());
    std::cerr << "[real-data] tile 32N/2/465/5440: zero_fraction=" << zero_fraction << " (" << zero_count << "/"
              << samples.size() << "), classes: ";
    for (const auto& [cls, count] : histogram) std::cerr << cls << "=" << count << " ";
    std::cerr << "wall_ms=" << report->wall_ms << "\n";

    CHECK(zero_fraction < 0.01);
}

TEST_CASE("real-data: checked-in WorldCover fixtures reproduce their source window", "[import][io][real-data][long]") {
    const auto dir = data_dir();
    if (!dir) SKIP("G2M_DATA_DIR not set");
    const std::string real_tif = *dir + "/ESA_WorldCover_10m_2021_v200_N48E006_Map.tif";
    if (!std::filesystem::exists(real_tif)) SKIP("ESA_WorldCover_10m_2021_v200_N48E006_Map.tif not found under G2M_DATA_DIR");

    const std::string regions_path = fixtures_dir() + "/regions.json";
    std::ifstream regions_in(regions_path, std::ios::binary);
    REQUIRE(regions_in.is_open());
    std::ostringstream regions_ss;
    regions_ss << regions_in.rdbuf();
    const Result<std::vector<RegionSpec>> specs = parse_regions(regions_ss.str());
    REQUIRE(specs.ok());

    // The "fixture" region resolves (against the real hessen-latest.osm.pbf)
    // to two separate tile clusters ~11 km apart (Höchst admin relation,
    // utm_bbox part) - testdata/fixtures/README.md and this file's sibling
    // fixture_cut cut-worldcover invocation record the exact tile sets each
    // fixture window covers. Rather than re-resolving against the OSM PBF
    // here (this file has no PBF dependency and should not gain one just
    // for this check), this test directly re-derives each fixture's own
    // covered tile set from the checked-in fixture's OWN georeference
    // (its width/height/tiepoint fully determine which world pixels it
    // holds) and compares its content, pixel-for-pixel, against the same
    // window decoded fresh from the real source file.
    for (const char* fixture_name :
         {"worldcover/fixture_N50E008_hoechst.tif", "worldcover/fixture_N50E008_bbox.tif"}) {
        const std::string fixture_path = fixtures_dir() + "/" + fixture_name;
        REQUIRE(std::filesystem::exists(fixture_path));

        auto fixture_source = std::make_shared<g2m::decode::FileByteSource>(fixture_path);
        g2m::decode::tiff::TiffReader fixture_reader(fixture_source);
        REQUIRE_FALSE(fixture_reader.ifds().empty());
        const auto& fixture_ifd = fixture_reader.ifds()[0];
        REQUIRE(fixture_ifd.geo_transform.has_value());
        const double sx = fixture_ifd.geo_transform->pixel_width;
        const auto n = static_cast<std::int64_t>(std::floor(1.0 / sx + 0.5));
        REQUIRE(1.0 / static_cast<double>(n) == sx);
        // Exactness guard matching landcover.cpp's own open_and_validate
        // (src/import/landcover.cpp): a fixture tiepoint whose reconstructed
        // corner*N isn't bit-exactly an integer would be REJECTED by the
        // production importer as "tiepoint corner is not an integer multiple
        // of 1/N degree" - found the hard way when cut-worldcover's naive
        // `new_c0/n` division didn't round-trip for N=12000 (not a power of
        // two); fixed by aligning the cut window's corner to a multiple of
        // 375 (kTiepointAlignment in fixture_cut/main.cpp) before dividing.
        const double fixture_c0d = fixture_ifd.geo_transform->corner_x * static_cast<double>(n);
        const double fixture_c1d = fixture_ifd.geo_transform->corner_y * static_cast<double>(n);
        CHECK(std::floor(fixture_c0d) == fixture_c0d);
        CHECK(std::floor(fixture_c1d) == fixture_c1d);
        const auto fixture_c0 = static_cast<std::int64_t>(fixture_c0d);
        const auto fixture_c1 = static_cast<std::int64_t>(fixture_c1d);
        const int fixture_w = static_cast<int>(fixture_ifd.width);
        const int fixture_h = static_cast<int>(fixture_ifd.height);

        auto real_source = std::make_shared<g2m::decode::FileByteSource>(real_tif);
        g2m::decode::tiff::TiffReader real_reader(real_source);
        REQUIRE_FALSE(real_reader.ifds().empty());
        const auto& real_ifd = real_reader.ifds()[0];
        REQUIRE(real_ifd.geo_transform.has_value());
        const double real_sx = real_ifd.geo_transform->pixel_width;
        const auto real_n = static_cast<std::int64_t>(std::floor(1.0 / real_sx + 0.5));
        REQUIRE(1.0 / static_cast<double>(real_n) == real_sx);
        REQUIRE(real_n == n); // same product, same native pixel scale
        const auto real_c0 = static_cast<std::int64_t>(real_ifd.geo_transform->corner_x * static_cast<double>(real_n));
        const auto real_c1 = static_cast<std::int64_t>(real_ifd.geo_transform->corner_y * static_cast<double>(real_n));

        // The fixture's own tiepoint corner tells us exactly where its
        // window sits inside the real file's own pixel grid.
        const std::int64_t col_lo = fixture_c0 - real_c0;
        const std::int64_t row_lo = real_c1 - fixture_c1;
        REQUIRE(col_lo >= 0);
        REQUIRE(row_lo >= 0);
        REQUIRE(col_lo + fixture_w <= static_cast<std::int64_t>(real_ifd.width));
        REQUIRE(row_lo + fixture_h <= static_cast<std::int64_t>(real_ifd.height));

        const Raster<std::uint8_t> fixture_window = fixture_reader.decode_window_u8(0, 0, 0, fixture_w, fixture_h);
        const Raster<std::uint8_t> real_window =
            real_reader.decode_window_u8(0, col_lo, row_lo, fixture_w, fixture_h);
        CHECK(fixture_window.data().size() == real_window.data().size());
        bool all_equal = true;
        for (std::size_t i = 0; i < fixture_window.data().size(); ++i) {
            if (fixture_window.data()[i] != real_window.data()[i]) {
                all_equal = false;
                break;
            }
        }
        CHECK(all_equal);
        std::cerr << "[real-data] " << fixture_name << ": " << fixture_w << "x" << fixture_h
                  << " window matches source at col=" << col_lo << " row=" << row_lo << "\n";
    }
}
