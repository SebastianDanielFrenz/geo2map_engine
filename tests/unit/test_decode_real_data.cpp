// tests/unit/test_decode_real_data.cpp - real-data smoke tests, gated on the
// G2M_DATA_DIR environment variable (SKIP when unset). Point it at the
// directory containing dgm1/PKT_*.zip, the two Copernicus GLO-30 COGs, the
// WorldCover COG and hessen-latest.osm.pbf (this milestone's own
// S:\claude_code\geo2map_data\downloads). Not run by default CI - real,
// multi-gigabyte third-party downloads, not committed fixtures.

#include "g2m/core/geo/utm.h"
#include "g2m/decode/byte_source.h"
#include "g2m/decode/dgm1.h"
#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/decode/tiff/tiff_reader.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <vector>

using namespace g2m::decode;

namespace {

std::optional<std::string> data_dir() {
#if defined(_MSC_VER) || defined(_WIN32)
    // plain getenv() is flagged deprecated (-Werror) under the MSVC CRT;
    // _dupenv_s is its suggested replacement.
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

std::vector<std::string> find_pkt_zips(const std::string& dgm1_dir) {
    std::vector<std::string> out;
    for (const auto& entry : std::filesystem::directory_iterator(dgm1_dir)) {
        if (!entry.is_regular_file()) continue;
        std::string name = entry.path().filename().string();
        std::string lower = name;
        for (char& c : lower) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (lower.size() >= 4 && lower.compare(lower.size() - 4, 4, ".zip") == 0) {
            out.push_back(entry.path().string());
        }
    }
    return out;
}

} // namespace

TEST_CASE("real-data: DGM1 tile enumeration across all PKT_*.zip files", "[decode][real-data][dgm1]") {
    auto dir = data_dir();
    if (!dir) {
        SKIP("G2M_DATA_DIR not set");
    }
    const std::string dgm1_dir = *dir + "/dgm1";
    if (!std::filesystem::exists(dgm1_dir)) SKIP("G2M_DATA_DIR/dgm1 not found");
    auto zips = find_pkt_zips(dgm1_dir);
    INFO("found " << zips.size() << " outer PKT_*.zip files");
    REQUIRE(zips.size() > 0);

    const auto t0 = std::chrono::steady_clock::now();
    auto all = dgm1::enumerate_tiles(zips);
    const auto t1 = std::chrono::steady_clock::now();
    const double enum_s = std::chrono::duration<double>(t1 - t0).count();

    std::map<dgm1::TileKey, int> copies_per_key;
    for (const auto& [key, loc] : all) ++copies_per_key[key];

    const std::size_t unique_count = copies_per_key.size();
    std::size_t duplicated = 0;
    for (const auto& [key, n] : copies_per_key) {
        if (n > 1) ++duplicated;
    }

    std::cerr << "[real-data] DGM1 enumerate_tiles: " << zips.size() << " outer zips, " << all.size()
              << " tile copies, " << unique_count << " unique tile keys, " << duplicated
              << " keys with >1 copy, in " << enum_s << " s ("
              << (enum_s > 0 ? static_cast<double>(all.size()) / enum_s : 0.0) << " tiles/s enumerated)\n";

    CHECK(all.size() > 0);
    CHECK(unique_count > 0);
    CHECK(all.size() >= unique_count);

    // The 474/5560 tile PLAN.md/G2M-002 use as the cross-check reference
    // must be present.
    const dgm1::TileKey ref_key{474, 5560};
    REQUIRE(copies_per_key.count(ref_key) == 1);
    INFO("tile 474/5560 has " << copies_per_key[ref_key] << " copies across the 8 zips");
}

TEST_CASE("real-data: DGM1 tile 474/5560 decodes identically from every copy", "[decode][real-data][dgm1]") {
    auto dir = data_dir();
    if (!dir) SKIP("G2M_DATA_DIR not set");
    const std::string dgm1_dir = *dir + "/dgm1";
    if (!std::filesystem::exists(dgm1_dir)) SKIP("G2M_DATA_DIR/dgm1 not found");
    auto zips = find_pkt_zips(dgm1_dir);
    REQUIRE(zips.size() > 0);
    auto all = dgm1::enumerate_tiles(zips);

    const dgm1::TileKey ref_key{474, 5560};
    std::vector<dgm1::TileLocation> locations;
    for (const auto& [key, loc] : all) {
        if (key == ref_key) locations.push_back(loc);
    }
    REQUIRE(locations.size() > 0);
    INFO(locations.size() << " copies of tile 474/5560 found");

    const auto t0 = std::chrono::steady_clock::now();
    std::vector<dgm1::DecodedTile> decoded;
    for (const auto& loc : locations) decoded.push_back(dgm1::decode_tile(loc));
    const auto t1 = std::chrono::steady_clock::now();
    const double decode_s = std::chrono::duration<double>(t1 - t0).count();
    std::cerr << "[real-data] decoded tile 474/5560 from " << locations.size() << " copies in " << decode_s
              << " s (" << (decode_s > 0 ? static_cast<double>(locations.size()) / decode_s : 0.0)
              << " tiles/s)\n";

    REQUIRE(decoded.size() == locations.size());
    CHECK(decoded[0].heights.width() == 1000);
    CHECK(decoded[0].heights.height() == 1000);
    CHECK(decoded[0].georef.west_mm == 474'000'000);
    CHECK(decoded[0].georef.south_mm == 5'560'000'000);

    for (std::size_t i = 1; i < decoded.size(); ++i) {
        CHECK(decoded[i].heights == decoded[0].heights);
        CHECK(decoded[i].georef == decoded[0].georef);
    }

    // Log a couple of sample values / NoData presence for the final report -
    // not an assertion (DGM1 legitimately has NoData at some cells e.g.
    // buildings/water masks in some deliveries).
    int nodata_count = 0;
    for (int y = 0; y < decoded[0].heights.height(); ++y) {
        for (int x = 0; x < decoded[0].heights.width(); ++x) {
            if (decoded[0].heights.at(x, y) == -9999.0f) ++nodata_count;
        }
    }
    std::cerr << "[real-data] tile 474/5560 sample [0,0]=" << decoded[0].heights.at(0, 0)
              << " [500,500]=" << decoded[0].heights.at(500, 500) << " NoData cells=" << nodata_count << "/1000000\n";
}

TEST_CASE("real-data: DGM1 sampled census across every outer zip (decodes without throwing, mm-quantised)",
          "[decode][real-data][dgm1][long]") {
    auto dir = data_dir();
    if (!dir) SKIP("G2M_DATA_DIR not set");
    const std::string dgm1_dir = *dir + "/dgm1";
    if (!std::filesystem::exists(dgm1_dir)) SKIP("G2M_DATA_DIR/dgm1 not found");
    auto zips = find_pkt_zips(dgm1_dir);
    REQUIRE(zips.size() > 0);
    auto all = dgm1::enumerate_tiles(zips);

    // One copy per unique key (first copy found).
    std::map<dgm1::TileKey, dgm1::TileLocation> first_copy;
    for (const auto& [key, loc] : all) first_copy.try_emplace(key, loc);

    // decode_tile() (both overloads) fully decompresses its tile's whole
    // INNER "<municipality> - DGM1.zip" into memory on every single call -
    // it does not cache the inner ZipReader across calls (out of this
    // milestone's scope: a pure, bounded per-tile decoder, not a batch
    // pipeline). Decoding every one of ~3000 unique tiles this way would
    // mean redundantly re-decompressing the SAME multi-hundred-MB-to-GB
    // inner zip once per tile it contains - for the 3.64 GB Wetteraukreis
    // delivery alone this would be minutes-to-hours of wasted work for a
    // smoke test. Sample instead: a bounded stride through the sorted key
    // set still touches tiles spread across every outer zip (worth noting
    // for a future batch-decode API, G2M-004).
    constexpr std::size_t kSampleTarget = 60;
    const std::size_t stride = std::max<std::size_t>(1, first_copy.size() / kSampleTarget);

    std::size_t decoded_count = 0;
    const auto t0 = std::chrono::steady_clock::now();
    std::size_t idx = 0;
    for (const auto& [key, loc] : first_copy) {
        if (idx++ % stride != 0) continue;
        auto tile = dgm1::decode_tile(loc);
        CHECK(tile.heights.width() == 1000);
        CHECK(tile.heights.height() == 1000);
        CHECK(tile.georef.west_mm == static_cast<std::int64_t>(key.e_km) * 1'000'000);
        CHECK(tile.georef.south_mm == static_cast<std::int64_t>(key.n_km) * 1'000'000);
        ++decoded_count;
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double decode_s = std::chrono::duration<double>(t1 - t0).count();
    std::cerr << "[real-data] sampled-decoded " << decoded_count << " of " << first_copy.size()
              << " unique DGM1 tiles (stride " << stride << ") in " << decode_s << " s ("
              << (decode_s > 0 ? static_cast<double>(decoded_count) / decode_s : 0.0) << " tiles/s)\n";
    CHECK(decoded_count > 0);
}

TEST_CASE("real-data: GLO-30 Copernicus DSM COG window decode (float32, geographic CRS)",
          "[decode][real-data][tiff]") {
    auto dir = data_dir();
    if (!dir) SKIP("G2M_DATA_DIR not set");
    std::string path = *dir + "/Copernicus_DSM_COG_10_N49_00_E008_00_DEM.tif";
    if (!std::filesystem::exists(path)) {
        path = *dir + "/Copernicus_DSM_COG_10_N50_00_E008_00_DEM.tif";
    }
    if (!std::filesystem::exists(path)) SKIP("no GLO-30 COG found under G2M_DATA_DIR");

    auto src = std::make_shared<FileByteSource>(path);
    tiff::TiffReader reader(src);
    REQUIRE(reader.ifds().size() >= 1);
    const auto& ifd = reader.ifds().front();
    std::cerr << "[real-data] GLO-30 " << path << ": " << ifd.width << "x" << ifd.height
              << ", IFD count (incl. overviews)=" << reader.ifds().size()
              << ", crs.kind=" << static_cast<int>(ifd.crs.kind) << " epsg=" << ifd.crs.epsg
              << ", geo_transform=" << (ifd.geo_transform ? "present" : "absent") << "\n";
    REQUIRE(ifd.width >= 64);
    REQUIRE(ifd.height >= 64);

    auto window = reader.decode_window_f32(0, 0, 0, 32, 32);
    bool any_finite = false;
    for (int y = 0; y < 32; ++y) {
        for (int x = 0; x < 32; ++x) {
            const float v = window.at(x, y);
            if (std::isfinite(v)) any_finite = true;
        }
    }
    CHECK(any_finite);
}

TEST_CASE("real-data: ESA WorldCover COG window decode (uint8 land-cover classes)", "[decode][real-data][tiff]") {
    auto dir = data_dir();
    if (!dir) SKIP("G2M_DATA_DIR not set");
    const std::string path = *dir + "/ESA_WorldCover_10m_2021_v200_N48E006_Map.tif";
    if (!std::filesystem::exists(path)) SKIP("WorldCover COG not found under G2M_DATA_DIR");

    auto src = std::make_shared<FileByteSource>(path);
    tiff::TiffReader reader(src);
    REQUIRE(reader.ifds().size() >= 1);
    const auto& ifd = reader.ifds().front();
    std::cerr << "[real-data] WorldCover " << path << ": " << ifd.width << "x" << ifd.height
              << ", IFD count=" << reader.ifds().size() << ", tiled=" << ifd.is_tiled() << "\n";
    REQUIRE(ifd.width >= 64);
    REQUIRE(ifd.height >= 64);

    auto window = reader.decode_window_u8(0, 0, 0, 32, 32);
    std::set<int> distinct_values;
    for (int y = 0; y < 32; ++y)
        for (int x = 0; x < 32; ++x) distinct_values.insert(window.at(x, y));
    std::cerr << "[real-data] WorldCover 32x32 window has " << distinct_values.size() << " distinct class value(s)\n";
    CHECK(distinct_values.size() >= 1);
}

TEST_CASE("real-data: hessen-latest.osm.pbf streamed with a bbox prefilter around 50.10N/8.55E",
          "[decode][real-data][osm][long]") {
    auto dir = data_dir();
    if (!dir) SKIP("G2M_DATA_DIR not set");
    const std::string path = *dir + "/hessen-latest.osm.pbf";
    if (!std::filesystem::exists(path)) SKIP("hessen-latest.osm.pbf not found under G2M_DATA_DIR");

    osm::BoundingBox bbox{50.05, 50.15, 8.45, 8.65}; // ~0.1 deg box around 50.10N/8.55E
    class CountingHandler : public osm::PbfHandler {
    public:
        std::uint64_t nodes_seen = 0;
        void on_node(const osm::OsmNode&) override { ++nodes_seen; }
    };
    CountingHandler handler;

    const auto t0 = std::chrono::steady_clock::now();
    osm::PbfStats stats = osm::read_pbf(path, handler, &bbox);
    const auto t1 = std::chrono::steady_clock::now();
    const double s = std::chrono::duration<double>(t1 - t0).count();
    const std::uint64_t total_entities = stats.node_count + stats.way_count + stats.relation_count;
    std::cerr << "[real-data] hessen-latest.osm.pbf, bbox " << bbox.min_lat << ".." << bbox.max_lat << "N, "
              << bbox.min_lon << ".." << bbox.max_lon << "E: nodes(in bbox)=" << stats.node_count
              << " ways(all)=" << stats.way_count << " relations(all)=" << stats.relation_count << " in " << s
              << " s (" << (s > 0 ? static_cast<double>(total_entities) / s : 0.0) << " entities/s)\n";

    CHECK(stats.node_count > 0);
    CHECK(handler.nodes_seen == stats.node_count);
    CHECK(stats.way_count > 0);
}
