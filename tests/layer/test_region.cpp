// tests/layer/test_region.cpp - g2m/layer/region.h (regions.json /
// g2m.regions/1, the g2m.resolved_region/1 artefact, split_selector) and
// g2m/core/geom/ring_set.h's tiles_near/classify_square (introduced by the
// same G1c I1 increment, not yet covered by any other test file).

#include "g2m/layer/region.h"

#include "g2m/core/geom/ring_set.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <string>
#include <utility>

using namespace g2m;

namespace {

geom::PointMm mm(std::int64_t x, std::int64_t y) { return geom::PointMm{x, y}; }

ResolvedRegion sample_resolved_region() {
    ResolvedRegion r;
    r.name = "home";
    r.scheme = "utm32n";
    r.level = 2;
    r.halo_m = 2000;
    ResolvedPart bbox_part;
    bbox_part.kind = ResolvedPartKind::UtmBbox;
    bbox_part.vertices = 4;
    bbox_part.geometry_hash = 0x0123456789abcdefull;
    r.parts.push_back(bbox_part);
    ResolvedPart admin_part;
    admin_part.kind = ResolvedPartKind::OsmAdmin;
    admin_part.id = 62650;
    admin_part.name = "Frankfurt am Main";
    admin_part.admin_level = 6;
    admin_part.vertices = 1842;
    admin_part.geometry_hash = 0xfedcba9876543210ull;
    r.parts.push_back(admin_part);
    r.pbf_name = "hessen-latest.osm.pbf";
    r.pbf_sha256[0] = 0xab;
    r.pbf_sha256[31] = 0xcd;
    r.tiles = {CoverageRun{100, 200, 210}, CoverageRun{101, 199, 211}};
    r.interior = {CoverageRun{100, 202, 208}};
    r.tile_count = 33;
    return r;
}

} // namespace

TEST_CASE("regions: parse OK", "[layer][region]") {
    const std::string_view text = R"({"format":"g2m.regions/1","regions":[)"
                                   R"({"name":"home","scheme":"utm32n","halo_m":2000,"parts":[)"
                                   R"({"kind":"utm_bbox","west":474000,"south":5560000,"east":480000,"north":5566000},)"
                                   R"({"kind":"osm_admin","relation":62650,"expect_name":"Frankfurt am Main",)"
                                   R"("expect_admin_level":6}]},)"
                                   R"({"name":"second","scheme":"utm33n","halo_m":0,"parts":[)"
                                   R"({"kind":"osm_admin","relation":1}]}])"
                                   R"(})";
    const auto r = parse_regions(text);
    REQUIRE(r.ok());
    REQUIRE(r->size() == 2);
    const RegionSpec& home = r->front();
    CHECK(home.name == "home");
    CHECK(home.scheme == "utm32n");
    CHECK(home.halo_m == 2000);
    REQUIRE(home.parts.size() == 2);
    const auto* bbox = std::get_if<UtmBboxPart>(&home.parts[0]);
    REQUIRE(bbox != nullptr);
    CHECK(bbox->west == 474000);
    CHECK(bbox->south == 5560000);
    CHECK(bbox->east == 480000);
    CHECK(bbox->north == 5566000);
    const auto* admin = std::get_if<OsmAdminPart>(&home.parts[1]);
    REQUIRE(admin != nullptr);
    CHECK(admin->relation == 62650);
    REQUIRE(admin->expect_name.has_value());
    CHECK(*admin->expect_name == "Frankfurt am Main");
    REQUIRE(admin->expect_admin_level.has_value());
    CHECK(*admin->expect_admin_level == 6);
    const RegionSpec& second = (*r)[1];
    CHECK(second.name == "second");
    CHECK(second.halo_m == 0);
}

TEST_CASE("regions: unknown key is rejected", "[layer][region]") {
    const auto r1 = parse_regions(R"({"format":"g2m.regions/1","regions":[],"extra":1})");
    REQUIRE_FALSE(r1.ok());
    CHECK(r1.error().code == ErrorCode::InvalidArgument);

    const auto r2 = parse_regions(R"({"format":"g2m.regions/1","regions":[)"
                                   R"({"name":"home","scheme":"utm32n","halo_m":0,"parts":[)"
                                   R"({"kind":"utm_bbox","west":0,"south":0,"east":1,"north":1}],"extra":1}]})");
    REQUIRE_FALSE(r2.ok());
    CHECK(r2.error().code == ErrorCode::InvalidArgument);

    const auto r3 = parse_regions(R"({"format":"g2m.regions/1","regions":[)"
                                   R"({"name":"home","scheme":"utm32n","halo_m":0,"parts":[)"
                                   R"({"kind":"utm_bbox","west":0,"south":0,"east":1,"north":1,"extra":1}]}]})");
    REQUIRE_FALSE(r3.ok());
    CHECK(r3.error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("regions: bad name is rejected", "[layer][region]") {
    const auto r = parse_regions(R"({"format":"g2m.regions/1","regions":[)"
                                  R"({"name":"Home","scheme":"utm32n","halo_m":0,"parts":[)"
                                  R"({"kind":"utm_bbox","west":0,"south":0,"east":1,"north":1}]}]})");
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("regions: duplicate name is rejected", "[layer][region]") {
    const std::string_view part = R"({"kind":"utm_bbox","west":0,"south":0,"east":1,"north":1})";
    const std::string text = std::string(R"({"format":"g2m.regions/1","regions":[)") +
                             R"({"name":"home","scheme":"utm32n","halo_m":0,"parts":[)" + std::string(part) + "]}," +
                             R"({"name":"home","scheme":"utm32n","halo_m":0,"parts":[)" + std::string(part) + "]}]}";
    const auto r = parse_regions(text);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("regions: utm_bbox west >= east is rejected", "[layer][region]") {
    const auto r = parse_regions(R"({"format":"g2m.regions/1","regions":[)"
                                  R"({"name":"home","scheme":"utm32n","halo_m":0,"parts":[)"
                                  R"({"kind":"utm_bbox","west":10,"south":0,"east":10,"north":1}]}]})");
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::InvalidArgument);

    const auto r2 = parse_regions(R"({"format":"g2m.regions/1","regions":[)"
                                   R"({"name":"home","scheme":"utm32n","halo_m":0,"parts":[)"
                                   R"({"kind":"utm_bbox","west":11,"south":0,"east":10,"north":1}]}]})");
    REQUIRE_FALSE(r2.ok());
    CHECK(r2.error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("regions: halo_m out of range is rejected", "[layer][region]") {
    const auto r = parse_regions(R"({"format":"g2m.regions/1","regions":[)"
                                  R"({"name":"home","scheme":"utm32n","halo_m":20001,"parts":[)"
                                  R"({"kind":"utm_bbox","west":0,"south":0,"east":1,"north":1}]}]})");
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("regions: malformed JSON and unsupported format", "[layer][region]") {
    const auto bad = parse_regions("{ not json");
    REQUIRE_FALSE(bad.ok());
    CHECK(bad.error().code == ErrorCode::Malformed);

    const auto wrong_format = parse_regions(R"({"format":"g2m.regions/2","regions":[]})");
    REQUIRE_FALSE(wrong_format.ok());
    CHECK(wrong_format.error().code == ErrorCode::Unsupported);
}

TEST_CASE("split_selector: path#name", "[layer][region]") {
    const auto r = split_selector("data/world/regions.json#home");
    REQUIRE(r.ok());
    CHECK(r->first == "data/world/regions.json");
    CHECK(r->second == "home");

    // First '#' only: a path containing '#' still splits correctly.
    const auto r2 = split_selector("a#b#c");
    REQUIRE(r2.ok());
    CHECK(r2->first == "a");
    CHECK(r2->second == "b#c");

    const auto missing = split_selector("no_hash_here");
    REQUIRE_FALSE(missing.ok());
    CHECK(missing.error().code == ErrorCode::InvalidArgument);

    const auto empty_path = split_selector("#home");
    REQUIRE_FALSE(empty_path.ok());
    const auto empty_name = split_selector("path#");
    REQUIRE_FALSE(empty_name.ok());
}

TEST_CASE("tiles_near: hand count at halo 0, 1, 1024 on an exact single tile", "[layer][region][geom]") {
    // A ring EXACTLY matching level-2 tile (0,0): [0, 1024] x [0, 1024] m,
    // i.e. [0, 1024000] x [0, 1024000] mm (level-2 tile size = 256 * 2^2 =
    // 1024 m). Every distance below is either exactly 0 (a shared vertex or
    // edge), a clean multiple of 1024 m, or 1024*sqrt(2) m - chosen so the
    // whole count can be hand-verified without any off-grid rounding.
    geom::RingSet rings;
    rings.outers.push_back(geom::Ring{mm(0, 0), mm(1024000, 0), mm(1024000, 1024000), mm(0, 1024000)});
    rings.bbox = geom::bounding_box(rings);
    const geo::UtmZone zone{32, geo::Hemisphere::North};

    const auto tile_set = [](const std::vector<std::pair<TileKey, geom::TileRelation>>& v) {
        std::set<std::pair<std::int32_t, std::int32_t>> s;
        for (const auto& [key, rel] : v) {
            REQUIRE(rel != geom::TileRelation::Outside); // tiles_near never returns Outside
            s.emplace(key.x, key.y);
        }
        return s;
    };

    // halo=0: tile(0,0) itself plus its 3 grid-aligned neighbours whose
    // shared edge/corner coincides exactly with the ring (inclusive boxes -
    // see ring_set.h/BoxMm's own "inclusive" contract): (1,0), (0,1), (1,1)
    // (the ring's own NE corner (1024000,1024000) is exactly tile(1,1)'s SW
    // corner).
    const auto at0 = tiles_near(rings, zone, /*level=*/2, /*halo_m=*/0);
    CHECK(at0.size() == 4);
    CHECK(tile_set(at0) == std::set<std::pair<std::int32_t, std::int32_t>>{{0, 0}, {1, 0}, {0, 1}, {1, 1}});

    // halo=1 (1 m): the dilated scan window grows to x,y in [-1, 1] (3x3=9
    // candidates: 1 m is enough to push cell_of(-1000mm) to index -1, but
    // not enough to reach index -2 or +2). Every one of the 5 new tiles
    // touches the ring exactly (shares an edge or a corner vertex, distance
    // 0), so all 9 are Boundary.
    const auto at1 = tiles_near(rings, zone, /*level=*/2, /*halo_m=*/1);
    CHECK(at1.size() == 9);
    CHECK(tile_set(at1) == std::set<std::pair<std::int32_t, std::int32_t>>{
                                {-1, -1}, {0, -1}, {1, -1}, {-1, 0}, {0, 0}, {1, 0}, {-1, 1}, {0, 1}, {1, 1}});

    // halo=1024 (1024 m = one tile side): the scan window grows to x,y in
    // [-1, 2] (4x4=16 candidates). Of the 12 new candidates, 11 are within
    // 1024 m of the ring (several at EXACTLY 1024 m - e.g. tile (2,0)'s
    // west edge is exactly 1024000 mm from the ring's own east edge - the
    // "corner exactly at distance h is included" case, inclusive per
    // classify_square's own <= comparison). Only tile (2,2), diagonally
    // 1024*sqrt(2) m from the ring's NE corner, exceeds halo=1024 m and is
    // excluded.
    const auto at1024 = tiles_near(rings, zone, /*level=*/2, /*halo_m=*/1024);
    CHECK(at1024.size() == 15);
    const std::set<std::pair<std::int32_t, std::int32_t>> want1024 = {
        {-1, -1}, {0, -1}, {1, -1}, {2, -1}, {-1, 0}, {0, 0}, {1, 0}, {2, 0},
        {-1, 1},  {0, 1},  {1, 1},  {2, 1},  {-1, 2}, {0, 2}, {1, 2}, // (2,2) excluded
    };
    CHECK(tile_set(at1024) == want1024);
    // The exactly-at-h corner case named above, checked explicitly:
    CHECK(want1024.count({2, 0}) == 1);
    CHECK(want1024.count({2, 2}) == 0);
}

TEST_CASE("resolved region: to_json -> parse -> to_json is byte-identical", "[layer][region]") {
    const ResolvedRegion r = sample_resolved_region();
    const std::string j1 = to_json(r);
    const auto parsed = parse_resolved_region(j1);
    REQUIRE(parsed.ok());
    CHECK(*parsed == r);
    const std::string j2 = to_json(*parsed);
    CHECK(j1 == j2);

    // Canonical: no added indentation/newlines (release_manifest's own
    // rules) - "Frankfurt am Main" legitimately contains spaces, so only
    // newlines (dump()'s pretty-print indicator) are checked here.
    CHECK(j1.find('\n') == std::string::npos);
    CHECK(j1.substr(0, 10) == R"({"format":)"); // sorted keys: "format" first
}

TEST_CASE("resolved region: unknown key is rejected", "[layer][region]") {
    const std::string j = to_json(sample_resolved_region());
    const std::string bad = j.substr(0, j.size() - 1) + R"(,"extra":1})";
    const auto r = parse_resolved_region(bad);
    REQUIRE_FALSE(r.ok());
    CHECK(r.error().code == ErrorCode::InvalidArgument);
}

TEST_CASE("resolved region: tile_keys reconstructs from coverage runs", "[layer][region]") {
    ResolvedRegion r;
    r.name = "home";
    r.scheme = "utm32n";
    r.level = 2;
    r.tiles = {CoverageRun{5, 10, 12}, CoverageRun{6, 20, 20}};
    const std::vector<TileKey> keys = tile_keys(r);
    REQUIRE(keys.size() == 4);
    const geo::UtmZone zone{32, geo::Hemisphere::North};
    CHECK(keys[0] == TileKey{zone, 2, 10, 5});
    CHECK(keys[1] == TileKey{zone, 2, 11, 5});
    CHECK(keys[2] == TileKey{zone, 2, 12, 5});
    CHECK(keys[3] == TileKey{zone, 2, 20, 6});

    // An unparseable scheme yields an empty (defensive) result.
    r.scheme = "not_a_scheme";
    CHECK(tile_keys(r).empty());
}

TEST_CASE("resolved region: interior_tile_keys reconstructs from r.interior, not r.tiles",
          "[layer][region]") {
    // PLAN.md G2.0 Task B: a verification that a region's interior carries
    // no unexpected NoData needs to enumerate exactly the interior subset,
    // not the full footprint - the two coverage-run lists can, and here do,
    // differ (r.tiles is the union including halo/boundary tiles, r.interior
    // only the tiles fully inside the region).
    ResolvedRegion r;
    r.name = "home";
    r.scheme = "utm32n";
    r.level = 2;
    r.tiles = {CoverageRun{5, 9, 13}, CoverageRun{6, 19, 21}}; // wider: includes halo
    r.interior = {CoverageRun{5, 10, 12}, CoverageRun{6, 20, 20}};
    const std::vector<TileKey> keys = interior_tile_keys(r);
    REQUIRE(keys.size() == 4);
    const geo::UtmZone zone{32, geo::Hemisphere::North};
    CHECK(keys[0] == TileKey{zone, 2, 10, 5});
    CHECK(keys[1] == TileKey{zone, 2, 11, 5});
    CHECK(keys[2] == TileKey{zone, 2, 12, 5});
    CHECK(keys[3] == TileKey{zone, 2, 20, 6});
    // tile_keys() on the same region returns the wider footprint, not this.
    CHECK(tile_keys(r).size() == 8);

    // An unparseable scheme yields an empty (defensive) result, same
    // contract as tile_keys().
    r.scheme = "not_a_scheme";
    CHECK(interior_tile_keys(r).empty());
}
