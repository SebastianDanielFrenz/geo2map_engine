// tests/unit/test_import_resolve.cpp - g2m::import::resolve_region/
// find_admin against the committed fixture PBF (testdata/fixtures/, G1c I2),
// plus a real-data companion gated on G2M_DATA_DIR (skip when unset - see
// test_decode_real_data.cpp's own header comment for the pattern this
// mirrors), plus a small MemorySink smoke test (g2m/import/sink.h).

#include "g2m/import/region_resolve.h"
#include "g2m/import/sink.h"
#include "g2m/layer/region.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>

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

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    REQUIRE(in.is_open());
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

RegionSpec find_region(const std::vector<RegionSpec>& specs, std::string_view name) {
    for (const auto& s : specs) {
        if (s.name == name) return s;
    }
    FAIL("region '" << name << "' not found in regions.json");
    return {};
}

} // namespace

TEST_CASE("import: resolve_region against the committed fixture PBF", "[import][io]") {
    const std::string regions_path = fixtures_dir() + "/regions.json";
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    REQUIRE(std::filesystem::exists(pbf_path));

    const std::string regions_json = read_file(regions_path);
    Result<std::vector<RegionSpec>> specs = parse_regions(regions_json);
    REQUIRE(specs.ok());
    const RegionSpec spec = find_region(*specs, "fixture");

    Result<ResolvedRegion> resolved = resolve_region(spec, pbf_path);
    REQUIRE(resolved.ok());

    CHECK(resolved->name == "fixture");
    CHECK(resolved->scheme == "utm32n");
    CHECK(resolved->halo_m == 200);
    // Two parts (PLAN G1c pre-item a): the original Höchst osm_admin part,
    // plus a utm_bbox part (474000/5560000 - 476000/5561000) added so the
    // DEM fixture tests below have region tiles that actually overlap the
    // DGM1 fixture (tiles 474/5560, 475/5560).
    REQUIRE(resolved->parts.size() == 2);
    CHECK(resolved->parts[0].kind == ResolvedPartKind::OsmAdmin);
    CHECK(resolved->parts[0].id == 4209939);
    CHECK(resolved->parts[0].name == "H\xC3\xB6" "chst"); // "Höchst", UTF-8 (split to end the hex escape before 'c')
    REQUIRE(resolved->parts[0].admin_level.has_value());
    CHECK(*resolved->parts[0].admin_level == 10);
    CHECK(resolved->parts[0].vertices == 614);
    CHECK(resolved->parts[0].role_mismatches == 0);

    CHECK(resolved->parts[1].kind == ResolvedPartKind::UtmBbox);
    CHECK(resolved->parts[1].vertices == 4);
    CHECK(resolved->parts[1].role_mismatches == 0); // always 0 for utm_bbox - no OSM roles involved

    // Committed against the fixture PBF's actual content - a change here
    // means either the fixture or the resolve pipeline changed; re-cut or
    // investigate, don't just update the number.
    CHECK(resolved->tile_count == 24);
    CHECK(resolved->tiles.size() == 7); // 7 CoverageRuns at this footprint (union of both parts)

    // interior is a SUBSET of tiles (every interior tile is also in the
    // full footprint) - true even when, as here, the fixture's small halo
    // yields zero interior tiles.
    CHECK(resolved->interior.size() <= resolved->tiles.size());

    const std::vector<TileKey> keys = tile_keys(*resolved);
    CHECK(keys.size() == resolved->tile_count);
}

TEST_CASE("import: resolve_region output is byte-stable across repeated runs", "[import][io]") {
    const std::string regions_path = fixtures_dir() + "/regions.json";
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    const std::string regions_json = read_file(regions_path);
    Result<std::vector<RegionSpec>> specs = parse_regions(regions_json);
    REQUIRE(specs.ok());
    const RegionSpec spec = find_region(*specs, "fixture");

    Result<ResolvedRegion> r1 = resolve_region(spec, pbf_path);
    Result<ResolvedRegion> r2 = resolve_region(spec, pbf_path);
    REQUIRE(r1.ok());
    REQUIRE(r2.ok());
    CHECK(*r1 == *r2);
    CHECK(to_json(*r1) == to_json(*r2));

    // The JSON representation round-trips through parse_resolved_region.
    // NOTE: ResolvedPart::rings is deliberately in-memory only (region.h's
    // own doc comment - not part of the JSON representation), so a
    // reparsed ResolvedRegion's rings are empty even though r1's are not;
    // comparing re-serialized JSON (not struct equality) is the correct
    // round-trip check.
    const std::string json = to_json(*r1);
    Result<ResolvedRegion> reparsed = parse_resolved_region(json);
    REQUIRE(reparsed.ok());
    CHECK(to_json(*reparsed) == json);
}

TEST_CASE("import: resolve_region handles a utm_bbox part (no PBF geometry needed)", "[import][io]") {
    // UtmBboxPart resolution (ring_set_for_bbox) is pure geometry - it
    // never reads the PBF for its own ring, but resolve_region still hashes
    // the PBF file up front (pbf_sha256, part of ResolvedRegion), so a
    // valid path is still required; the committed fixture PBF is reused
    // here purely as "some real file", unrelated to this part's content.
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";

    RegionSpec spec;
    spec.name = "bbox_only";
    spec.scheme = "utm32n";
    spec.halo_m = 100;
    UtmBboxPart bbox;
    bbox.west = 474000;
    bbox.south = 5560000;
    bbox.east = 476000;
    bbox.north = 5561000;
    spec.parts.push_back(bbox);

    Result<ResolvedRegion> resolved = resolve_region(spec, pbf_path);
    REQUIRE(resolved.ok());
    REQUIRE(resolved->parts.size() == 1);
    CHECK(resolved->parts[0].kind == ResolvedPartKind::UtmBbox);
    CHECK(resolved->parts[0].id == 0);
    CHECK(resolved->parts[0].name.empty());
    CHECK_FALSE(resolved->parts[0].admin_level.has_value());
    CHECK(resolved->parts[0].vertices == 4); // one rectangular outer ring
    CHECK(resolved->tile_count > 0);
    // Every interior tile is part of the full footprint.
    CHECK(resolved->interior.size() <= resolved->tiles.size());
}

TEST_CASE("import: MemorySink collects put() calls keyed by (layer, key), last write wins", "[import]") {
    MemorySink sink;
    CHECK(sink.size() == 0);

    const geo::UtmZone zone{32, geo::Hemisphere::North};
    const TileKey key_a{zone, /*level=*/2, /*x=*/10, /*y=*/20};
    const TileKey key_b{zone, /*level=*/2, /*x=*/11, /*y=*/20};

    TileHeader h1;
    h1.layer = "dem";
    h1.layer_version = 1;
    h1.key = key_a;
    const std::vector<std::uint8_t> body1{1, 2, 3};
    REQUIRE(sink.put("dem", key_a, h1, body1).ok());

    TileHeader h2;
    h2.layer = "dem";
    h2.layer_version = 1;
    h2.key = key_b;
    const std::vector<std::uint8_t> body2{4, 5};
    REQUIRE(sink.put("dem", key_b, h2, body2).ok());

    CHECK(sink.size() == 2);

    // Overwrite key_a - last write wins, size stays 2.
    const std::vector<std::uint8_t> body1b{9, 9, 9, 9};
    REQUIRE(sink.put("dem", key_a, h1, body1b).ok());
    CHECK(sink.size() == 2);

    const auto entries = sink.entries();
    REQUIRE(entries.size() == 2);
    // Sorted by (layer, key) - both entries share layer "dem", so ordered
    // by TileKey next; key_a.x=10 < key_b.x=11.
    CHECK(entries[0].first.second == key_a);
    CHECK(entries[0].second.body == body1b);
    CHECK(entries[0].second.header.body_size == body1b.size());
    CHECK(entries[1].first.second == key_b);
    CHECK(entries[1].second.body == body2);
}

TEST_CASE("import: find_admin locates the fixture's Stadtteil relation", "[import][io]") {
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    std::vector<AdminCandidate> all = find_admin(pbf_path);
    // The bbox-mode fixture cut (G1c I4 pre-item 2, testdata/fixtures/README.md)
    // recurses relation membership upward: any boundary=administrative
    // relation sharing so much as one member way with the (re-cut) bbox is
    // now included, not just the target Stadtteil relation the earlier
    // hand-trimmed fixture happened to contain alone. Neighbouring
    // Stadtteile/Stadtbezirke whose own boundary ways run along Hoechst's
    // edge are pulled in too - assert the target relation is present among
    // however many candidates exist, not an exact singleton count.
    const auto find_target = [](const std::vector<AdminCandidate>& v) {
        return std::find_if(v.begin(), v.end(), [](const AdminCandidate& c) { return c.relation_id == 4209939; });
    };
    REQUIRE(all.size() >= 1);
    const auto it = find_target(all);
    REQUIRE(it != all.end());
    CHECK(it->name == "H\xC3\xB6" "chst"); // "Höchst", UTF-8
    REQUIRE(it->admin_level.has_value());
    CHECK(*it->admin_level == 10);

    std::vector<AdminCandidate> by_level = find_admin(pbf_path, std::nullopt, 10);
    // Committed against the fixture's actual content: 7 admin_level-10
    // Stadtteil relations (Sindlingen, Schwanheim, Bonames, Hoechst, Nied,
    // Sossenheim, Unterliederbach) whose boundary touches the bbox,
    // including the target - a change here means the fixture was re-cut,
    // not a pipeline regression; re-derive with a debug dump of `all`
    // (relation id/name/admin_level) if it moves.
    CHECK(by_level.size() == 7);
    CHECK(find_target(by_level) != by_level.end());

    // No admin_level-6 (kreisfreie Stadt / Landkreis) relation's own
    // member ways happen to touch this small bbox.
    std::vector<AdminCandidate> by_level_miss = find_admin(pbf_path, std::nullopt, 6);
    CHECK(by_level_miss.empty());
}

TEST_CASE("import: resolve_region reports NotFound for a relation absent from the PBF", "[import][io]") {
    const std::string pbf_path = fixtures_dir() + "/osm/frankfurt_hoechst.osm.pbf";
    RegionSpec spec;
    spec.name = "missing";
    spec.scheme = "utm32n";
    spec.halo_m = 0;
    OsmAdminPart part;
    part.relation = 1; // not present in the fixture
    spec.parts.push_back(part);

    Result<ResolvedRegion> resolved = resolve_region(spec, pbf_path);
    REQUIRE_FALSE(resolved.ok());
    CHECK(resolved.error().code == ErrorCode::NotFound);
}

TEST_CASE("real-data: resolve_region against the full hessen-latest.osm.pbf for the home region",
          "[import][io][real-data][long]") {
    auto dir = data_dir();
    if (!dir) SKIP("G2M_DATA_DIR not set");
    const std::string pbf_path = *dir + "/hessen-latest.osm.pbf";
    if (!std::filesystem::exists(pbf_path)) SKIP("hessen-latest.osm.pbf not found under G2M_DATA_DIR");

    // The three home administrative relations (task brief, planner-measured):
    // 62382 Main-Taunus-Kreis (admin 6), 62566 Hochtaunuskreis (admin 6),
    // 4209939 Hoechst (admin 10). Expected footprint at halo_m=2000: ~1085
    // tiles, ~533 interior (+-1%, task brief).
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

    const auto t0 = std::chrono::steady_clock::now();
    Result<ResolvedRegion> resolved = resolve_region(spec, pbf_path);
    const auto t1 = std::chrono::steady_clock::now();
    const double wall_s = std::chrono::duration<double>(t1 - t0).count();
    REQUIRE(resolved.ok());

    std::uint64_t interior_tiles = 0;
    for (const auto& run : resolved->interior) interior_tiles += static_cast<std::uint64_t>(run.x1 - run.x0 + 1);

    std::size_t total_role_mismatches = 0;
    for (const auto& part : resolved->parts) total_role_mismatches += part.role_mismatches;

    std::cerr << "[real-data] home region resolve: tile_count=" << resolved->tile_count
              << " interior_tiles=" << interior_tiles << " parts=" << resolved->parts.size()
              << " role_mismatches=" << total_role_mismatches << " (";
    for (std::size_t i = 0; i < resolved->parts.size(); ++i) {
        if (i != 0) std::cerr << ",";
        std::cerr << resolved->parts[i].role_mismatches;
    }
    std::cerr << ") wall=" << wall_s << "s\n";

    // task brief: ~1085 tiles, +-1%.
    CHECK(resolved->tile_count >= 1074);
    CHECK(resolved->tile_count <= 1096);
    // task brief: interior ~533 (~1085*0.5, +-1%).
    CHECK(interior_tiles >= 528);
    CHECK(interior_tiles <= 538);
    CHECK(resolved->parts.size() == 3);

    // PLAN G1c pre-item (b): the real Hessen relations' member ways can carry
    // OSM role tags (e.g. "outer"/"inner") that AssembledArea::role_mismatches
    // disagrees with by ring-orientation - measured on real data (see
    // std::cerr line above) before deciding whether 0 is a fact to assert or
    // just a number to report. As measured against hessen-latest.osm.pbf for
    // relations 62382/62566/4209939, every part's ring assembly agrees with
    // its members' own role tags.
    CHECK(total_role_mismatches == 0);
}
