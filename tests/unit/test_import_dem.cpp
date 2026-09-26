// tests/unit/test_import_dem.cpp - g2m::import::import_dem against the
// committed DGM1 fixture (testdata/fixtures/dgm1/PKT_fixture.zip) and the
// fixture region's utm_bbox part (testdata/fixtures/regions.json, PLAN.md
// G1c pre-item a: 474000/5560000 - 476000/5561000, chosen so its level-2
// tile footprint actually overlaps the DGM1 fixture's two 1 km squares,
// 474/5560 and 475/5560). Mirrors test_import_resolve.cpp's own fixture-
// reading conventions (fixtures_dir(), read_file()).

#include "g2m/decode/dgm1.h"
#include "g2m/import/dem.h"
#include "g2m/import/region_resolve.h"
#include "g2m/import/sink.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/region.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/tile_container.h"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <string>
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
    CHECK(report->dup_identical >= 1);
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
