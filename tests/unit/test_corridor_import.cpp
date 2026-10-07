// tests/unit/test_corridor_import.cpp - the A5/A67 corridor + Mannheim import
// (owner brief 2026-10-07): the Baden-Wuerttemberg DGM1 adapter
// (decode/dgm1_bw.h), import_dem's two-source border rule, the utm_corridor
// region part, resolve_region over several PBFs and merge_osm_data.
//
// Every check carries a recorded sabotage in its comment: the one-line source
// change that makes it fail (verified by hand when the test was written).

#include "g2m/core/hash.h"
#include "g2m/core/raster.h"
#include "g2m/core/tile_key.h"
#include "g2m/decode/dgm1.h"
#include "g2m/decode/dgm1_bw.h"
#include "g2m/import/dem.h"
#include "g2m/import/osm.h"
#include "g2m/import/region_resolve.h"
#include "g2m/import/sink.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/osm_data.h"
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
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <set>
#include <string>
#include <thread>
#include <vector>

using namespace g2m;
using namespace g2m::import;
namespace bw = g2m::decode::dgm1_bw;
namespace dgm1 = g2m::decode::dgm1;

namespace {

using g2m::test_support::TiffBuilder;
using g2m::test_support::ZipBuilder;

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

// ---- BW .xyz text helpers ------------------------------------------------

// One line "X Y Z\r\n" for the sample at (col, row) of km square (e, n).
std::string xyz_line(int e_km, int n_km, int col, int row, const char* z) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "%d.50 %d.50 %s\r\n", e_km * 1000 + col, n_km * 1000 + 999 - row, z);
    return buf;
}

// All samples of rows [0, rows) x cols [0, cols) at height z (e.g. "600.00").
std::string xyz_block(int e_km, int n_km, int cols, int rows, const char* z) {
    std::string s;
    for (int r = 0; r < rows; ++r) {
        for (int c = 0; c < cols; ++c) s += xyz_line(e_km, n_km, c, r, z);
    }
    return s;
}

std::vector<std::byte> bytes_of(const std::string& s) {
    std::vector<std::byte> v(s.size());
    std::memcpy(v.data(), s.data(), s.size());
    return v;
}

std::string write_zip(const std::string& name, const ZipBuilder& zb) {
    const std::vector<std::byte> bytes = zb.build();
    const std::string path = (std::filesystem::temp_directory_path() / name).string();
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    return path;
}

void remove_with_retry(const std::string& path) {
    for (int attempt = 0; attempt < 20; ++attempt) {
        std::error_code ec;
        if (std::filesystem::remove(path, ec) || ec == std::errc::no_such_file_or_directory) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(25));
    }
}

// ---- Hessen DGM1 tiff (copy of test_import_dem.cpp's builder) ----------------

std::vector<std::byte> deflate_zlib_compress(const std::vector<std::byte>& in) {
    libdeflate_compressor* c = libdeflate_alloc_compressor(6);
    std::vector<std::byte> out(in.size() + 4096);
    const std::size_t n = libdeflate_zlib_compress(c, in.data(), in.size(), out.data(), out.size());
    libdeflate_free_compressor(c);
    out.resize(n);
    return out;
}

std::vector<std::byte> build_dgm1_tiff(const std::vector<float>& samples) {
    constexpr int w = 1000, h = 1000;
    REQUIRE(samples.size() == static_cast<std::size_t>(w) * h);
    std::vector<std::byte> raw(samples.size() * 4);
    std::memcpy(raw.data(), samples.data(), raw.size());
    std::vector<std::byte> encoded(raw.size());
    for (int y = 0; y < h; ++y) {
        std::vector<std::byte> row(raw.begin() + static_cast<std::ptrdiff_t>(y) * w * 4,
                                    raw.begin() + static_cast<std::ptrdiff_t>(y + 1) * w * 4);
        auto* p = reinterpret_cast<std::uint32_t*>(row.data());
        for (int i = w - 1; i >= 1; --i) p[i] = static_cast<std::uint32_t>(p[i] - p[i - 1]);
        std::memcpy(encoded.data() + static_cast<std::ptrdiff_t>(y) * w * 4, row.data(), row.size());
    }
    TiffBuilder b;
    b.add_short(256, static_cast<std::uint16_t>(w));
    b.add_short(257, static_cast<std::uint16_t>(h));
    b.add_short(258, 32);
    b.add_short(259, 8);
    b.add_short(277, 1);
    b.add_long(278, static_cast<std::uint32_t>(h));
    b.add_short(284, 1);
    b.add_short(317, 2);
    b.add_short(339, 3);
    return b.build(deflate_zlib_compress(encoded));
}

Result<ResolvedRegion> resolve_bbox_region(std::int64_t west, std::int64_t south, std::int64_t east,
                                            std::int64_t north) {
    RegionSpec spec;
    spec.name = "bw_fixture";
    spec.scheme = "utm32n";
    spec.halo_m = 0;
    UtmBboxPart bbox;
    bbox.west = west;
    bbox.south = south;
    bbox.east = east;
    bbox.north = north;
    spec.parts.push_back(bbox);
    return resolve_region(spec, fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf");
}

// The height written at UTM point (e, n), looked up over every dgm1 tile in `sink`.
std::optional<std::int32_t> height_at(const MemorySink& sink, double e, double n) {
    for (const auto& [key, entry] : sink.entries()) {
        if (key.first != std::string(kSrcElevDgm1Layer)) continue;
        Result<TileBody> body = decode_body(entry.body);
        REQUIRE(body.ok());
        Result<ElevTile> tile = decode_src_elev(*body);
        REQUIRE(tile.ok());
        int col = -1, row = -1;
        for (int c = 0; c < tile->heights.width(); ++c) {
            if (std::fabs(tile->georef.sample_easting_m(c) - e) < 1e-6) col = c;
        }
        for (int r = 0; r < tile->heights.height(); ++r) {
            if (std::fabs(tile->georef.sample_northing_m(r) - n) < 1e-6) row = r;
        }
        if (col >= 0 && row >= 0) return tile->heights.at(col, row);
    }
    return std::nullopt;
}

} // namespace

// ============================================================================
// BW .xyz decoder
// ============================================================================

TEST_CASE("bw dgm1: tile names parse, 2 km and foreign names do not", "[decode][dgm1][bw]") {
    bw::KmKey k;
    CHECK(bw::try_parse_bw_tile_name("dgm1_32_465_5480_1_bw_2022.xyz", k));
    CHECK(k.e_km == 465);
    CHECK(k.n_km == 5480);
    CHECK(bw::try_parse_bw_tile_name("DGM1_32_457_5493_1_BW.XYZ", k));
    CHECK_FALSE(bw::try_parse_bw_tile_name("dgm1_32_465_5480_2_bw.xyz", k)); // 2 km tile, not 1
    CHECK_FALSE(bw::try_parse_bw_tile_name("dgm1_32_465_5480_1_bw_2022.csv", k));
    CHECK_FALSE(bw::try_parse_bw_tile_name("dgm1_32_465_5480_1_he.xyz", k));
    CHECK_FALSE(bw::try_parse_bw_tile_name("dgm1_33_465_5480_1_bw.xyz", k)); // another UTM zone
}

TEST_CASE("bw dgm1: samples are placed by their own coordinates, absent samples are NoData", "[decode][dgm1][bw]") {
    // Three samples, deliberately NOT in file order, CRLF and LF mixed, a blank line, and a
    // negative height. Sabotage: place by line index instead of coordinates -> idx checks fail.
    std::string text;
    text += xyz_line(465, 5480, 7, 3, "91.21");                 // col 7, row 3
    text += "465000.50 5480999.50 -2.50\n";                      // col 0, row 0 (LF only)
    text += "\r\n";                                              // blank line
    text += xyz_line(465, 5480, 999, 999, "100.01");             // last sample
    std::size_t listed = 0;
    const Result<std::vector<float>> r = bw::parse_xyz_tile(text, 465, 5480, &listed);
    REQUIRE(r.ok());
    REQUIRE(r->size() == 1000u * 1000u);
    CHECK(listed == 3);
    CHECK((*r)[3 * 1000 + 7] == 91.21f);
    CHECK((*r)[0] == -2.5f);
    CHECK((*r)[999 * 1000 + 999] == 100.01f);
    std::size_t nodata = 0;
    for (const float v : *r) nodata += (v == -9999.0f) ? 1 : 0;
    CHECK(nodata == 1000u * 1000u - 3);
}

TEST_CASE("bw dgm1: off-lattice, out-of-square, duplicate and malformed lines are rejected", "[decode][dgm1][bw]") {
    CHECK_FALSE(bw::parse_xyz_tile("465000.00 5480999.50 90.00\r\n", 465, 5480).ok()); // corner, not a cell centre
    CHECK_FALSE(bw::parse_xyz_tile("466000.50 5480999.50 90.00\r\n", 465, 5480).ok()); // east of the square
    CHECK_FALSE(bw::parse_xyz_tile("465000.50 5481000.50 90.00\r\n", 465, 5480).ok()); // north of the square
    CHECK_FALSE(bw::parse_xyz_tile("465000.50 5480999.50 90.00\r\n465000.50 5480999.50 91.00\r\n", 465, 5480).ok());
    CHECK_FALSE(bw::parse_xyz_tile("465000.50 5480999.50\r\n", 465, 5480).ok());              // no Z
    CHECK_FALSE(bw::parse_xyz_tile("465000.50 5480999.50 90.00 7\r\n", 465, 5480).ok());      // trailing field
    CHECK_FALSE(bw::parse_xyz_tile("465000.5005 5480999.50 90.00\r\n", 465, 5480).ok());      // finer than 1 mm
    const Result<std::vector<float>> empty = bw::parse_xyz_tile("", 465, 5480);
    REQUIRE(empty.ok()); // an empty file is a tile with no samples (all NoData), not an error
}

TEST_CASE("bw dgm1: Catalog indexes the 1 km tiles of a 2 km zip and decodes one", "[decode][dgm1][bw]") {
    ZipBuilder zb;
    zb.add({"dgm1_32_901_6000_2_bw/", {}, false});
    zb.add({"dgm1_32_901_6000_2_bw/dgm1_32_901_6000_1_bw_2022.csv", bytes_of("Kachelname;x\r\n"), false});
    zb.add({"dgm1_32_901_6000_2_bw/dgm1_32_901_6000_1_bw_2022.xyz", bytes_of(xyz_block(901, 6000, 3, 2, "12.34")),
            true});
    zb.add({"dgm1_32_901_6000_2_bw/dgm1_32_902_6001_1_bw_2022.xyz", bytes_of(xyz_block(902, 6001, 1, 1, "5.00")),
            false});
    zb.add({"dgm1_32_901_6000_2_bw/Meta-ATKIS_DGM1_ALS2.txt", bytes_of("meta"), false});
    const std::string path = write_zip("g2m_test_bw_catalog.zip", zb);
    {
        Result<bw::Catalog> cat = bw::Catalog::open({path});
        REQUIRE(cat.ok());
        CHECK(cat->stats().copies == 2);
        CHECK(cat->stats().unique_keys == 2);
        CHECK(cat->copies(903, 6000).empty());
        const auto refs = cat->copies(901, 6000);
        REQUIRE(refs.size() == 1);
        CHECK(cat->describe(refs[0]).find("dgm1_32_901_6000_1_bw_2022.xyz") != std::string::npos);
        std::size_t listed = 0;
        const Result<std::vector<float>> tile = cat->decode(refs[0], &listed);
        REQUIRE(tile.ok());
        CHECK(listed == 6);
        CHECK((*tile)[1] == 12.34f);
        CHECK((*tile)[1000 + 2] == 12.34f);
        CHECK((*tile)[3] == -9999.0f);
    }
    remove_with_retry(path);
}

// ============================================================================
// import_dem: BW only, and the Hessen/BW border rule
// ============================================================================

TEST_CASE("bw import: a BW-only square is imported as heights in the shared DHHN2016 units", "[import][io][dgm1][bw]") {
    ZipBuilder zb;
    zb.add({"dgm1_32_901_6000_2_bw/dgm1_32_901_6000_1_bw_2022.xyz", bytes_of(xyz_block(901, 6000, 1000, 1, "123.46")),
            true});
    const std::string path = write_zip("g2m_test_bw_only.zip", zb);
    {
        Result<bw::Catalog> cat = bw::Catalog::open({path});
        REQUIRE(cat.ok());
        Result<ResolvedRegion> resolved = resolve_bbox_region(901000, 6000000, 902000, 6001000);
        REQUIRE(resolved.ok());
        DemSources sources;
        sources.bw = &*cat;
        MemorySink sink;
        Result<DemImportReport> report = import_dem(*resolved, sources, sink, "test", DemImportOptions{});
        REQUIRE(report.ok());
        CHECK(report->km_bw_only >= 1);
        CHECK(report->km_merged == 0);
        const std::optional<std::int32_t> expected = quantise_height_half_even(123.46f);
        REQUIRE(expected.has_value());
        const auto got = height_at(sink, 901000.5, 6000999.5); // row 0, col 0
        REQUIRE(got.has_value());
        CHECK(*got == *expected);
        const auto missing = height_at(sink, 901000.5, 6000998.5); // row 1 is not listed
        REQUIRE(missing.has_value());
        CHECK(*missing == kHeightNoData);
    }
    remove_with_retry(path);
}

TEST_CASE("bw import: at a shared square the BW sample wins where BW lists it, Hessen elsewhere, never averaged",
          "[import][io][dgm1][bw]") {
    // Hessen square (900, 6000) is 500.0 m everywhere. BW lists only the west half (cols 0..499) of
    // the top 100 rows at 600.0 m. Sabotage: swap the merge so Hessen overwrites BW (dem.cpp
    // decode_km, `(*hessen)[i] = b;` removed) -> the 600.0 checks and the border counters fail.
    std::vector<float> hessen(1000 * 1000, 500.0f);
    ZipBuilder inner;
    inner.add({"dgm1_32_900_6000_1_he.tif", build_dgm1_tiff(hessen), false});
    ZipBuilder outer;
    outer.add({"muni.zip", inner.build(), false});
    const std::string hpath = write_zip("g2m_test_bw_border_he.zip", outer);

    ZipBuilder zb;
    zb.add({"dgm1_32_899_6000_2_bw/dgm1_32_900_6000_1_bw_2022.xyz", bytes_of(xyz_block(900, 6000, 500, 100, "600.00")),
            true});
    const std::string bpath = write_zip("g2m_test_bw_border_bw.zip", zb);
    {
        Result<dgm1::Catalog> hc = dgm1::Catalog::open({hpath});
        Result<bw::Catalog> bc = bw::Catalog::open({bpath});
        REQUIRE(hc.ok());
        REQUIRE(bc.ok());
        Result<ResolvedRegion> resolved = resolve_bbox_region(900000, 6000000, 901000, 6001000);
        REQUIRE(resolved.ok());
        DemSources sources;
        sources.hessen = &*hc;
        sources.bw = &*bc;
        MemorySink sink;
        Result<DemImportReport> report = import_dem(*resolved, sources, sink, "test", DemImportOptions{});
        REQUIRE(report.ok());
        CHECK(report->km_merged >= 1);
        CHECK(report->border_samples_both == 500u * 100u);
        CHECK(report->border_samples_differing == 500u * 100u);
        CHECK(report->border_max_abs_diff_units == 100 * kHeightUnitsPerMetre);

        const auto bw_sample = height_at(sink, 900000.5, 6000999.5);     // col 0, row 0: BW lists it
        const auto bw_edge = height_at(sink, 900499.5, 6000900.5);       // col 499, row 99: last BW sample
        const auto east_sample = height_at(sink, 900500.5, 6000999.5);   // col 500: not in BW -> Hessen
        const auto south_sample = height_at(sink, 900000.5, 6000899.5);  // row 100: not in BW -> Hessen
        REQUIRE(bw_sample.has_value());
        REQUIRE(bw_edge.has_value());
        REQUIRE(east_sample.has_value());
        REQUIRE(south_sample.has_value());
        CHECK(*bw_sample == *quantise_height_half_even(600.0));
        CHECK(*bw_edge == *quantise_height_half_even(600.0));
        CHECK(*east_sample == *quantise_height_half_even(500.0));
        CHECK(*south_sample == *quantise_height_half_even(500.0));
    }
    remove_with_retry(hpath);
    remove_with_retry(bpath);
}

TEST_CASE("bw import: import_dem without any source is an error", "[import][dgm1][bw]") {
    Result<ResolvedRegion> resolved = resolve_bbox_region(900000, 6000000, 901000, 6001000);
    REQUIRE(resolved.ok());
    MemorySink sink;
    CHECK_FALSE(import_dem(*resolved, DemSources{}, sink, "test", DemImportOptions{}).ok());
}

// ============================================================================
// utm_corridor region part
// ============================================================================

TEST_CASE("corridor: regions.json parsing accepts a corridor and rejects the malformed ones", "[io][region]") {
    const auto doc = [](const std::string& part) {
        return std::string(R"({"format":"g2m.regions/1","regions":[{"name":"c","scheme":"utm32n","halo_m":0,"parts":[)") +
               part + "]}]}";
    };
    const auto good = parse_regions(doc(R"({"kind":"utm_corridor","half_width_m":3000,"points":[[470000,5520000],[471000,5530000]]})"));
    REQUIRE(good.ok());
    const auto* c = std::get_if<UtmCorridorPart>(&good->front().parts.front());
    REQUIRE(c != nullptr);
    CHECK(c->half_width_m == 3000);
    CHECK(c->points.size() == 2);
    CHECK(c->points[1] == std::make_pair<std::int64_t, std::int64_t>(471000, 5530000));

    CHECK_FALSE(parse_regions(doc(R"({"kind":"utm_corridor","half_width_m":3000,"points":[[470000,5520000]]})")).ok());
    CHECK_FALSE(parse_regions(doc(R"({"kind":"utm_corridor","half_width_m":0,"points":[[1,2],[3,4]]})")).ok());
    CHECK_FALSE(parse_regions(doc(R"({"kind":"utm_corridor","half_width_m":10,"points":[[1,2],[1,2]]})")).ok());
    CHECK_FALSE(parse_regions(doc(R"({"kind":"utm_corridor","half_width_m":10,"points":[[1,2],[3]]})")).ok());
    CHECK_FALSE(parse_regions(doc(R"({"kind":"utm_corridor","half_width_m":10,"points":[[1,2],[3,4]],"width":5})")).ok());
}

TEST_CASE("corridor: a horizontal corridor resolves to exactly the tiles its rectangle touches", "[import][io][region]") {
    // Segment (e 450000..456000, n 5550000), half width 1500 m, caps of 1500 m: the rectangle is
    // e in [448500, 457500], n in [5548500, 5551500]. Level-2 tiles are 1024 m squares on the zone's
    // grid; no rectangle edge lies on a tile edge here (checked below), so "touches" is unambiguous.
    // Sabotage: drop the end caps in corridor_rects (ax = x0, bx = x1) -> the west/east tile columns differ.
    RegionSpec spec;
    spec.name = "corr";
    spec.scheme = "utm32n";
    spec.halo_m = 0;
    UtmCorridorPart c;
    c.half_width_m = 1500;
    c.points = {{450000, 5550000}, {456000, 5550000}};
    spec.parts.push_back(c);
    const Result<ResolvedRegion> r = resolve_region(spec, fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf");
    REQUIRE(r.ok());
    REQUIRE(r->parts.size() == 1);
    CHECK(r->parts[0].kind == ResolvedPartKind::UtmCorridor);

    std::set<std::pair<std::int64_t, std::int64_t>> got;
    for (const TileKey& k : tile_keys(*r)) got.insert({k.min_easting(), k.min_northing()});
    REQUIRE_FALSE(got.empty());

    // Brute-force expectation over the bounding range of tiles.
    const std::int64_t size = TileKey::kLevel0SizeM << kRawTileLevel;
    std::set<std::pair<std::int64_t, std::int64_t>> want;
    const std::int64_t e_lo = got.begin()->first - 4 * size, e_hi = got.rbegin()->first + 4 * size;
    std::int64_t n_lo = INT64_MAX, n_hi = INT64_MIN;
    for (const auto& g : got) {
        n_lo = std::min(n_lo, g.second);
        n_hi = std::max(n_hi, g.second);
    }
    // Tile origins are multiples of `size` (zone-relative grid): derive the phase from a known tile.
    const std::int64_t phase_e = got.begin()->first % size;
    const std::int64_t phase_n = got.begin()->second % size;
    for (std::int64_t e = e_lo - ((e_lo - phase_e) % size + size) % size; e <= e_hi; e += size) {
        for (std::int64_t n = n_lo - 4 * size; n <= n_hi + 4 * size; n += size) {
            const std::int64_t nn = n - ((n - phase_n) % size + size) % size;
            const bool overlaps_e = e < 457500 && e + size > 448500;
            const bool overlaps_n = nn < 5551500 && nn + size > 5548500;
            if (overlaps_e && overlaps_n) want.insert({e, nn});
        }
    }
    CHECK(got == want);
}

TEST_CASE("corridor: a diagonal corridor covers the tiles near its axis and none far from it", "[import][io][region]") {
    RegionSpec spec;
    spec.name = "diag";
    spec.scheme = "utm32n";
    spec.halo_m = 0;
    UtmCorridorPart c;
    c.half_width_m = 3000;
    c.points = {{450000, 5550000}, {460000, 5560000}, {460000, 5575000}}; // two segments, a 45 degree bend
    spec.parts.push_back(c);
    const Result<ResolvedRegion> r = resolve_region(spec, fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf");
    REQUIRE(r.ok());
    const double half_diag = 1024.0 * std::sqrt(2.0) / 2.0;
    const auto dist_to_segment = [](double px, double py, double ax, double ay, double bx, double by) {
        const double dx = bx - ax, dy = by - ay;
        double t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy);
        t = std::max(0.0, std::min(1.0, t));
        return std::hypot(px - (ax + t * dx), py - (ay + t * dy));
    };
    std::set<std::pair<std::int64_t, std::int64_t>> got;
    for (const TileKey& k : tile_keys(*r)) got.insert({k.min_easting(), k.min_northing()});
    // Every tile whose centre is within (3000 - half diagonal) of the polyline is in.
    for (std::int64_t e = 446000; e < 466000; e += 1024) {
        for (std::int64_t n = 5546000; n < 5580000; n += 1024) {
            // snap to the tile grid of the resolved set
            const std::int64_t gx = got.begin()->first % 1024, gy = got.begin()->second % 1024;
            const std::int64_t te = e - ((e - gx) % 1024 + 1024) % 1024;
            const std::int64_t tn = n - ((n - gy) % 1024 + 1024) % 1024;
            const double cx = static_cast<double>(te) + 512, cy = static_cast<double>(tn) + 512;
            const double d = std::min(dist_to_segment(cx, cy, 450000, 5550000, 460000, 5560000),
                                       dist_to_segment(cx, cy, 460000, 5560000, 460000, 5575000));
            if (d <= 3000.0 - half_diag - 1.0) CHECK(got.count({te, tn}) == 1);
            // Beyond the square cap's corner reach (sqrt2 * width) plus a tile, nothing is in.
            if (d > 3000.0 * std::sqrt(2.0) + 2 * half_diag) CHECK(got.count({te, tn}) == 0);
        }
    }
}

// ============================================================================
// resolve_region over several PBFs
// ============================================================================

TEST_CASE("resolve: several PBFs - the relation is taken from the first file that holds it", "[import][io][region]") {
    const std::string with_relation = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    const std::string without = fixtures_dir() + "/osm/unsorted_nodes.osm.pbf";
    RegionSpec spec;
    spec.name = "multi";
    spec.scheme = "utm32n";
    spec.halo_m = 0;
    OsmAdminPart a;
    a.relation = 4209939; // Hoechst, only in the Frankfurt fixture
    spec.parts.push_back(a);

    const Result<ResolvedRegion> single = resolve_region(spec, with_relation);
    REQUIRE(single.ok());
    const Result<ResolvedRegion> multi = resolve_region(spec, std::vector<std::string>{without, with_relation});
    REQUIRE(multi.ok());
    CHECK(multi->tiles == single->tiles);
    CHECK(multi->parts[0].geometry_hash == single->parts[0].geometry_hash);
    CHECK(multi->pbf_name == "unsorted_nodes.osm.pbf+frankfurt_hoechst.osm.pbf");

    // pbf_sha256 of several files = SHA-256 over the files' own digests in order.
    const auto file_digest = [](const std::string& path) {
        std::ifstream in(path, std::ios::binary);
        std::string bytes((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        return Sha256::of(std::string_view(bytes));
    };
    const Sha256::Digest d_without = file_digest(without);
    const Sha256::Digest d_with = file_digest(with_relation);
    CHECK(single->pbf_sha256 == d_with); // one file = its own digest
    Sha256 sha;
    sha.update(d_without.data(), d_without.size());
    sha.update(d_with.data(), d_with.size());
    CHECK(multi->pbf_sha256 == sha.finish());

    // A relation in none of the files is an error naming the last file.
    OsmAdminPart missing;
    missing.relation = 1;
    spec.parts = {missing};
    CHECK_FALSE(resolve_region(spec, std::vector<std::string>{without, with_relation}).ok());
}

// ============================================================================
// merge_osm_data
// ============================================================================

namespace {

osm::OsmData make_set(int variant) {
    osm::OsmBuilder b;
    if (variant == 0) { // extract A: nodes 1-3, way 10, relation 100 with one member
        b.add_node(1, 495000000, 85000000);
        b.add_node(2, 495100000, 85100000);
        b.add_node(3, 495200000, 85200000, {{"highway", "motorway_junction"}});
        b.add_way(10, {1, 2, 3}, {{"highway", "motorway"}, {"ref", "A 67"}});
        b.add_relation(100, {{osm::MemberType::Way, 10, "route"}}, {{"type", "route"}, {"ref", "A 67"}});
        b.add_relation(101, {{osm::MemberType::Way, 10, "outer"}}, {{"type", "multipolygon"}});
    } else { // extract B: nodes 1-5 (node 2 moved), way 10 (same), way 11, relation 100 with two members
        b.add_node(1, 495000000, 85000000);
        b.add_node(2, 495100001, 85100000); // 1e-7 degree off: a conflict
        b.add_node(3, 495200000, 85200000, {{"highway", "motorway_junction"}});
        b.add_node(4, 495300000, 85300000);
        b.add_node(5, 495400000, 85400000);
        b.add_way(10, {1, 2, 3}, {{"highway", "motorway"}, {"ref", "A 67"}});
        b.add_way(11, {3, 4, 5}, {{"highway", "motorway"}, {"ref", "A 67"}});
        b.add_relation(100, {{osm::MemberType::Way, 10, "route"}, {osm::MemberType::Way, 11, "route"}},
                       {{"type", "route"}, {"ref", "A 67"}});
        b.add_relation(101, {{osm::MemberType::Way, 10, "outer"}}, {{"type", "multipolygon"}});
    }
    const Result<osm::OsmData> d = b.build();
    REQUIRE(d.ok());
    return *d;
}

} // namespace

TEST_CASE("osm merge: ids present in both extracts are kept once, conflicts follow the documented rule",
          "[import][osm]") {
    // Sabotage 1: skip the `second.find_...` dedupe for nodes (always add) -> build() fails on a duplicate id.
    // Sabotage 2: always keep the FIRST relation (drop the longer-member-list rule) -> relation 100 has 1 member.
    const osm::OsmData a = make_set(0);
    const osm::OsmData b = make_set(1);
    OsmMergeReport rep;
    const Result<osm::OsmData> m = merge_osm_data(a, b, &rep);
    REQUIRE(m.ok());
    REQUIRE(m->validate().ok());
    CHECK(m->nodes.size() == 5);
    CHECK(m->ways.size() == 2);
    CHECK(m->relations.size() == 2);
    CHECK(rep.nodes_both == 3);
    CHECK(rep.nodes_conflicting == 1); // node 2
    CHECK(rep.ways_both == 1);
    CHECK(rep.ways_conflicting == 0);
    CHECK(rep.relations_both == 2);
    CHECK(rep.relations_conflicting == 1); // 100: members differ; 101 identical

    // Node 2 comes from the FIRST set; relation 100 from the one with more members (B).
    CHECK(m->find_node(2)->lat == 495100000);
    CHECK(m->find_relation(100)->members_count == 2);
    CHECK(m->find_relation(101)->members_count == 1);

    // With the sets swapped the first set (B) wins node 2, the longer relation is still B's.
    const Result<osm::OsmData> swapped = merge_osm_data(b, a);
    REQUIRE(swapped.ok());
    CHECK(swapped->find_node(2)->lat == 495100001);
    CHECK(swapped->find_relation(100)->members_count == 2);

    // Deterministic: the same inputs merge to equal data; a self-merge is the identity.
    CHECK(*merge_osm_data(a, b) == *m);
    const Result<osm::OsmData> self = merge_osm_data(b, b);
    REQUIRE(self.ok());
    CHECK(*self == b);
}

TEST_CASE("osm merge: a way crossing the border arrives complete and unique", "[import][osm]") {
    // Extract A holds the west part of way 20 plus all its nodes (an extract keeps crossing ways whole),
    // extract B holds the same way and a second way 21 starting at the shared node 3.
    osm::OsmBuilder ba;
    ba.add_node(1, 1, 1);
    ba.add_node(2, 2, 2);
    ba.add_node(3, 3, 3);
    ba.add_way(20, {1, 2, 3}, {{"highway", "motorway"}, {"ref", "A 6"}});
    osm::OsmBuilder bb;
    bb.add_node(2, 2, 2);
    bb.add_node(3, 3, 3);
    bb.add_node(4, 4, 4);
    bb.add_node(1, 1, 1);
    bb.add_way(20, {1, 2, 3}, {{"highway", "motorway"}, {"ref", "A 6"}});
    bb.add_way(21, {3, 4}, {{"highway", "motorway"}, {"ref", "A 6"}});
    const Result<osm::OsmData> a = ba.build();
    const Result<osm::OsmData> b = bb.build();
    REQUIRE(a.ok());
    REQUIRE(b.ok());
    OsmMergeReport rep;
    const Result<osm::OsmData> m = merge_osm_data(*a, *b, &rep);
    REQUIRE(m.ok());
    CHECK(m->ways.size() == 2);
    CHECK(m->nodes.size() == 4);
    CHECK(rep.ways_both == 1);
    CHECK(rep.ways_conflicting == 0);
    CHECK(rep.nodes_conflicting == 0);
    for (const osm::Way& w : m->ways) {
        for (const std::int64_t ref : m->refs_of(w)) CHECK(m->find_node(ref) != nullptr);
    }
}

// ============================================================================
// real data: both extracts around the Viernheimer Dreieck (A 6 / A 67, the state border)
// ============================================================================

TEST_CASE("real-data: Hessen + Karlsruhe extracts merge around the Viernheimer Dreieck without duplicates",
          "[import][io][osm][real-data][long]") {
#if defined(_WIN32)
    char* env_buf = nullptr;
    std::size_t env_len = 0;
    if (_dupenv_s(&env_buf, &env_len, "G2M_DATA_DIR") != 0 || env_buf == nullptr || *env_buf == 0) {
        std::free(env_buf);
        SKIP("G2M_DATA_DIR not set");
    }
    const std::string data_root(env_buf);
    std::free(env_buf);
#else
    const char* env = std::getenv("G2M_DATA_DIR");
    if (env == nullptr || *env == 0) SKIP("G2M_DATA_DIR not set");
    const std::string data_root(env);
#endif
    const std::string he = data_root + "/hessen-latest.osm.pbf";
    const std::string ka = data_root + "/karlsruhe-regbez-latest.osm.pbf";
    if (!std::filesystem::exists(he) || !std::filesystem::exists(ka)) SKIP("a PBF is missing under G2M_DATA_DIR");

    // 6 km x 6 km around the Dreieck (468070, 5489641): the state border runs through it.
    RegionSpec spec;
    spec.name = "viernheim";
    spec.scheme = "utm32n";
    spec.halo_m = 0;
    UtmBboxPart bbox;
    bbox.west = 465000;
    bbox.south = 5486500;
    bbox.east = 471000;
    bbox.north = 5492500;
    spec.parts.push_back(bbox);
    const Result<ResolvedRegion> resolved = resolve_region(spec, std::vector<std::string>{he, ka});
    REQUIRE(resolved.ok());

    OsmExtractReport rep_he, rep_both;
    OsmMergeReport merge;
    const Result<osm::OsmData> only_he = extract_region_set(he, *resolved, &rep_he);
    const Result<osm::OsmData> only_ka = extract_region_set(ka, *resolved, nullptr);
    const Result<osm::OsmData> both = extract_region_set(std::vector<std::string>{he, ka}, *resolved, &rep_both, &merge);
    REQUIRE(only_he.ok());
    REQUIRE(only_ka.ok());
    REQUIRE(both.ok());
    CHECK(both->validate().ok());
    // An extract clips a way crossing ITS border (counted in ways_incomplete); the other extract holds it whole.
    std::cerr << "[real-data] viernheim merge: ways he/ka/both " << only_he->ways.size() << "/" << only_ka->ways.size()
              << "/" << both->ways.size() << " ways_both=" << merge.ways_both << " ways_conflicting="
              << merge.ways_conflicting << " nodes_both=" << merge.nodes_both << " incomplete(sum)="
              << rep_both.ways_incomplete << " he_incomplete=" << rep_he.ways_incomplete << std::endl;

    // Each extract alone is a subset of the merge, and the merge is smaller than the sum (shared border objects).
    CHECK(both->ways.size() >= only_he->ways.size());
    CHECK(both->ways.size() >= only_ka->ways.size());
    CHECK(both->ways.size() < only_he->ways.size() + only_ka->ways.size());
    CHECK(merge.ways_both > 0);
    CHECK(both->ways.size() + merge.ways_both == only_he->ways.size() + only_ka->ways.size());
    CHECK(both->nodes.size() + merge.nodes_both == only_he->nodes.size() + only_ka->nodes.size());

    // Every way of both is complete and the motorways A 6 and A 67 both reach into the region.
    std::set<std::string> refs;
    for (const osm::Way& w : both->ways) {
        for (const std::int64_t r : both->refs_of(w)) REQUIRE(both->find_node(r) != nullptr);
        const auto ref = both->tag(both->tags_of(w), "ref");
        const auto hw = both->tag(both->tags_of(w), "highway");
        if (hw && *hw == "motorway" && ref) refs.insert(std::string(*ref));
    }
    CHECK(refs.count("A 6") == 1);
    CHECK(refs.count("A 67") == 1);
}
