// tests/unit/test_import_osm.cpp - g2m::import::extract_region_set/import_osm
// (PLAN.md G1c I4) against the committed fixture PBF, plus a real-data
// companion gated on G2M_DATA_DIR (mirrors test_import_resolve.cpp's own
// pattern). extract_region_set's S_region equivalence contract
// (clip_osm(S_region, k) == clip_osm(S_full, k) for every region tile k,
// osm.h's own doc comment) is checked directly against the whole fixture
// PBF loaded unfiltered.

#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/import/osm.h"
#include "g2m/import/region_resolve.h"
#include "g2m/import/sink.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/osm_clip.h"
#include "g2m/layer/region.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/tile_store.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

#if defined(_WIN32)
#include <windows.h>

#include <psapi.h>
#endif

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

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in.is_open());
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

RegionSpec find_region(const std::vector<RegionSpec>& specs, std::string_view name) {
    for (const auto& s : specs) {
        if (s.name == name) return s;
    }
    FAIL("region '" << name << "' not found in regions.json");
    return {};
}

ResolvedRegion resolve_fixture_region() {
    const std::string regions_json = read_file(fixtures_dir() + "/regions.json");
    const Result<std::vector<RegionSpec>> specs = parse_regions(regions_json);
    REQUIRE(specs.ok());
    const RegionSpec spec = find_region(*specs, "fixture");
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    const Result<ResolvedRegion> resolved = resolve_region(spec, pbf_path);
    REQUIRE(resolved.ok());
    return *resolved;
}

// Same "load everything unfiltered" helper as test_osm_clip_index.cpp (kept
// local rather than shared - both test binaries link g2m_layer_tests/
// g2m_unit_tests independently and this is 30 lines, not worth a new shared
// header for one caller pair).
class FullLoader final : public decode::osm::PbfHandler {
public:
    void on_node(const decode::osm::OsmNode& n) override {
        osm::OsmBuilder::TagList tags;
        tags.reserve(n.tags.size());
        for (const auto& t : n.tags) tags.emplace_back(t.key, t.value);
        builder.add_node(n.id, n.lat_1e7, n.lon_1e7, std::move(tags));
    }
    void on_way(const decode::osm::OsmWay& w) override {
        osm::OsmBuilder::TagList tags;
        tags.reserve(w.tags.size());
        for (const auto& t : w.tags) tags.emplace_back(t.key, t.value);
        builder.add_way(w.id, w.node_refs, std::move(tags));
    }
    void on_relation(const decode::osm::OsmRelation& r) override {
        std::vector<osm::OsmBuilder::MemberIn> members;
        members.reserve(r.members.size());
        for (const auto& m : r.members) {
            osm::MemberType type = osm::MemberType::Node;
            if (m.type == decode::osm::OsmMemberType::Way) {
                type = osm::MemberType::Way;
            } else if (m.type == decode::osm::OsmMemberType::Relation) {
                type = osm::MemberType::Relation;
            }
            members.push_back(osm::OsmBuilder::MemberIn{type, m.ref, m.role});
        }
        osm::OsmBuilder::TagList tags;
        tags.reserve(r.tags.size());
        for (const auto& t : r.tags) tags.emplace_back(t.key, t.value);
        builder.add_relation(r.id, std::move(members), std::move(tags));
    }
    osm::OsmBuilder builder;
};

std::vector<std::uint8_t> encode_bytes(const OsmTile& tile) {
    const Result<TileBody> body = encode_src_osm(tile);
    REQUIRE(body.ok());
    const Result<std::vector<std::uint8_t>> bytes = encode_body(*body);
    REQUIRE(bytes.ok());
    return *bytes;
}

// Field-wise equality for MemorySink::entries(), same helper as
// test_import_dem.cpp's own same_entries().
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

TEST_CASE("import osm: extract_region_set against the fixture PBF has no incomplete ways", "[import][io][osm]") {
    const ResolvedRegion resolved = resolve_fixture_region();
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";

    OsmExtractReport report;
    const Result<osm::OsmData> region_set = extract_region_set(pbf_path, resolved, &report);
    REQUIRE(region_set.ok());
    REQUIRE(region_set->validate().ok());

    CHECK(report.nodes_total > 0);
    CHECK(report.nodes_in > 0);
    CHECK(report.nodes_in <= report.nodes_total);
    CHECK(report.ways_in > 0);
    CHECK(report.ways_incomplete == 0);
    CHECK(report.index_bytes == report.nodes_total * 16);
}

TEST_CASE("import osm: S_region equivalence - clip_osm(S_region,k) == clip_osm(S_full,k) for every region tile",
          "[import][io][osm]") {
    const ResolvedRegion resolved = resolve_fixture_region();
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    const std::vector<TileKey> keys = tile_keys(resolved);
    REQUIRE_FALSE(keys.empty());

    const Result<osm::OsmData> region_set = extract_region_set(pbf_path, resolved);
    REQUIRE(region_set.ok());

    FullLoader loader;
    decode::osm::read_pbf(pbf_path, loader);
    const Result<osm::OsmData> full_set = loader.builder.build();
    REQUIRE(full_set.ok());

    for (const TileKey& key : keys) {
        const Result<OsmTile> from_region = clip_osm(*region_set, key, kDefaultOsmHaloM);
        const Result<OsmTile> from_full = clip_osm(*full_set, key, kDefaultOsmHaloM);
        REQUIRE(from_region.ok());
        REQUIRE(from_full.ok());
        CHECK(encode_bytes(*from_region) == encode_bytes(*from_full));
    }
}

TEST_CASE("import osm: extract_region_set reports Malformed on an unsorted PBF", "[import][io][osm]") {
    const std::string pbf_path = fixtures_dir() + "/osm/unsorted_nodes.osm.pbf";
    REQUIRE(std::filesystem::exists(pbf_path));

    RegionSpec spec;
    spec.name = "unsorted";
    spec.scheme = "utm32n";
    spec.halo_m = 0;
    UtmBboxPart bbox;
    bbox.west = 0;
    bbox.south = 0;
    bbox.east = 1000;
    bbox.north = 1000;
    spec.parts.push_back(bbox);
    const Result<ResolvedRegion> resolved = resolve_region(spec, pbf_path);
    REQUIRE(resolved.ok());

    const Result<osm::OsmData> region_set = extract_region_set(pbf_path, *resolved);
    REQUIRE_FALSE(region_set.ok());
    CHECK(region_set.error().code == ErrorCode::Malformed);
    CHECK(region_set.error().message.find("PBF not sorted") != std::string::npos);
}

TEST_CASE("import osm: import_osm skips empty tiles, header is g2m.src.osm v1, datum None",
          "[import][io][osm]") {
    const ResolvedRegion resolved = resolve_fixture_region();
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    const Result<osm::OsmData> region_set = extract_region_set(pbf_path, resolved);
    REQUIRE(region_set.ok());

    MemorySink sink;
    const Result<OsmImportReport> report = import_osm(resolved, *region_set, sink, "test", OsmImportOptions{});
    REQUIRE(report.ok());
    CHECK(report->tiles_written > 0);
    CHECK(report->tiles_written + report->tiles_empty == tile_keys(resolved).size());
    CHECK(sink.size() == report->tiles_written);

    const auto entries = sink.entries();
    REQUIRE_FALSE(entries.empty());
    for (const auto& [addr, entry] : entries) {
        CHECK(addr.first == std::string(kSrcOsmLayer));
        CHECK(entry.header.layer == std::string(kSrcOsmLayer));
        CHECK(entry.header.layer_version == kSrcOsmEncodingVersion);
        CHECK(entry.header.datum == VerticalDatum::None);
        CHECK(entry.header.body_size == entry.body.size());

        const Result<TileBody> body = decode_body(entry.body);
        REQUIRE(body.ok());
        const Result<OsmTile> tile = decode_src_osm(*body);
        REQUIRE(tile.ok());
    }
}

TEST_CASE("import osm: import_osm's output is independent of --jobs (1 vs 8)", "[import][io][osm]") {
    const ResolvedRegion resolved = resolve_fixture_region();
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    const Result<osm::OsmData> region_set = extract_region_set(pbf_path, resolved);
    REQUIRE(region_set.ok());

    MemorySink sink_a;
    OsmImportOptions opts_a;
    opts_a.jobs = 1;
    const Result<OsmImportReport> report_a = import_osm(resolved, *region_set, sink_a, "a", opts_a);
    REQUIRE(report_a.ok());

    MemorySink sink_b;
    OsmImportOptions opts_b;
    opts_b.jobs = 8;
    const Result<OsmImportReport> report_b = import_osm(resolved, *region_set, sink_b, "b", opts_b);
    REQUIRE(report_b.ok());

    CHECK(report_a->tiles_written == report_b->tiles_written);
    CHECK(report_a->tiles_empty == report_b->tiles_empty);
    CHECK(same_entries(sink_a.entries(), sink_b.entries()));
}

TEST_CASE("real-data: extract_region_set + import_osm against hessen-latest.osm.pbf for the home region",
          "[import][io][osm][real-data][long]") {
    auto dir = data_dir();
    if (!dir) SKIP("G2M_DATA_DIR not set");
    const std::string pbf_path = *dir + "/hessen-latest.osm.pbf";
    if (!std::filesystem::exists(pbf_path)) SKIP("hessen-latest.osm.pbf not found under G2M_DATA_DIR");

    RegionSpec spec;
    spec.name = "home";
    spec.scheme = "utm32n";
    spec.halo_m = 2000;
    OsmAdminPart p1;
    p1.relation = 62382;
    p1.expect_admin_level = 6;
    OsmAdminPart p2;
    p2.relation = 62566;
    p2.expect_admin_level = 6;
    OsmAdminPart p3;
    p3.relation = 4209939;
    p3.expect_admin_level = 10;
    spec.parts = {p1, p2, p3};

    const Result<ResolvedRegion> resolved = resolve_region(spec, pbf_path);
    REQUIRE(resolved.ok());

    OsmExtractReport extract_report;
    const auto t0 = std::chrono::steady_clock::now();
    const Result<osm::OsmData> region_set = extract_region_set(pbf_path, *resolved, &extract_report);
    const auto t_extract = std::chrono::steady_clock::now();
    REQUIRE(region_set.ok());
    CHECK(extract_report.ways_incomplete == 0);

    // A scratch store, never PLAN.md I7's real S:\claude_code\geo2map_cache\home.
    const std::filesystem::path scratch_dir =
        std::filesystem::temp_directory_path() / "g2m_test_osm_real_data_scratch";
    std::error_code rm_ec;
    std::filesystem::remove_all(scratch_dir, rm_ec);
    TileStoreConfig config;
    config.dir = scratch_dir;
    const Result<std::unique_ptr<TileStore>> store = TileStore::open(config);
    REQUIRE(store.ok());

    StoreSink sink(**store, "home");
    const auto t1 = std::chrono::steady_clock::now();
    const Result<OsmImportReport> report = import_osm(*resolved, *region_set, sink, "home", OsmImportOptions{});
    const auto t2 = std::chrono::steady_clock::now();
    REQUIRE(report.ok());

    const Result<TileStoreStats> store_stats = (*store)->stats();
    REQUIRE(store_stats.ok());

    const double extract_s = std::chrono::duration<double>(t_extract - t0).count();
    const double import_s = std::chrono::duration<double>(t2 - t1).count();
    const double total_s = std::chrono::duration<double>(t2 - t0).count();
    std::cerr << "[real-data] home region osm import: nodes_total=" << extract_report.nodes_total
              << " nodes_in=" << extract_report.nodes_in << " ways_in=" << extract_report.ways_in
              << " ways_incomplete=" << extract_report.ways_incomplete
              << " relations_in=" << extract_report.relations_in << " index_bytes=" << extract_report.index_bytes
              << " (n1=" << extract_report.n1_ms << "ms w=" << extract_report.w_ms << "ms r=" << extract_report.r_ms
              << "ms n2=" << extract_report.n2_ms << "ms, extract wall=" << extract_s << "s)\n";
    std::cerr << "[real-data] home region osm import: tiles_written=" << report->tiles_written
              << " tiles_empty=" << report->tiles_empty << " import_wall=" << import_s << "s total=" << total_s
              << "s store body_bytes=" << store_stats->body_bytes << " bodies=" << store_stats->bodies << "\n";

    CHECK(extract_report.ways_incomplete == 0);
    CHECK(report->tiles_written > 0);
    CHECK(report->tiles_written + report->tiles_empty == tile_keys(*resolved).size());

    // task brief: peak RSS < 4 GB. No portable in-test peak-RSS reader is
    // shared with apps/g2m_tiler/rss.cpp (app-only, not part of any test
    // library) - reported via /proc or Windows PSAPI directly here, kept
    // local since this is this test's only use.
#if defined(_WIN32)
    {
        PROCESS_MEMORY_COUNTERS counters{};
        counters.cb = sizeof(counters);
        if (K32GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters))) {
            const auto peak_mb = static_cast<std::uint64_t>(counters.PeakWorkingSetSize) / (1024 * 1024);
            std::cerr << "[real-data] home region osm import: peak RSS " << peak_mb << " MiB\n";
            CHECK(peak_mb < 4096);
        }
    }
#else
    {
        std::ifstream status("/proc/self/status");
        std::string line;
        std::uint64_t peak_kb = 0;
        while (std::getline(status, line)) {
            if (line.rfind("VmHWM:", 0) == 0) {
                peak_kb = static_cast<std::uint64_t>(std::stoull(line.substr(6)));
                break;
            }
        }
        if (peak_kb > 0) {
            std::cerr << "[real-data] home region osm import: peak RSS " << (peak_kb / 1024) << " MiB\n";
            CHECK(peak_kb / 1024 < 4096);
        }
    }
#endif

    std::filesystem::remove_all(scratch_dir, rm_ec);
}
