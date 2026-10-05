// tests/golden/golden_g3d.cpp - G3d golden hashes (street fidelity P2 slice 9, design section 11 "Goldens"): the carved
// terrain g2m.terrain.carved at level 0 over the committed OSM fixtures (frankfurt_hoechst and hoechst_a66), on the
// tilted, bumpy, rough synthetic DEM of carve_fixture.h (carve_tilted_dem: + - * / and an integer hash only, so the
// input is the same on every platform) - the four busiest level-0 tiles of the slice 9 tests.
//
// Per tile: one entry whose count is the cell count and whose hash is the FNV-1a 64 of the carved heights (int32,
// 1/256 m, row-major), and one entry whose count is the carved cell count and whose hash covers the cell counters per
// owner, the deck-clamped cells and the blend-station counters. Each tile is carved with 1 and with 4 workers; a
// difference aborts. One "all" line. testdata/golden/g3d_golden.txt; compared byte for byte between the Windows and
// Linux legs by tools/ci.ps1 (the Shepard junction surface, the blends and the smoothing in double arithmetic: no FMA,
// fixed order).

#include "golden.h"

#include "carve_fixture.h"

#include "g2m/core/hash.h"

#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;

[[noreturn]] void fail(const std::string& what) {
    std::fprintf(stderr, "golden g3d: %s\n", what.c_str());
    std::abort();
}

void fixture(std::vector<Entry>& out, const std::string& name, const std::string& file, std::initializer_list<std::pair<std::int32_t, std::int32_t>> tiles) {
    const g2m::RoadGraph world = g2m_test::fixture_world(file);
    g2m_test::CarveFixture fx(world, g2m_test::carve_tilted_dem);
    for (const auto& [x, y] : tiles) {
        const g2m::TileKey key{g2m_test::kZone32N, 0, x, y};
        g2m::CarveStats stats;
        const auto tile = fx.carve(key, {}, 1, &stats);
        if (!tile) fail(name + ": carve: " + tile.error().message);
        const auto tile4 = fx.carve(key, {}, 4);
        if (!tile4) fail(name + ": carve (4 workers): " + tile4.error().message);
        if (tile4->h != tile->h) fail(name + ": 1 and 4 workers differ");
        const std::string prefix = "g3d." + name + ".tile." + std::to_string(x) + "." + std::to_string(y);
        Fnv1a64 heights;
        for (const std::int32_t h : tile->h) heights.i32(h);
        out.push_back({prefix + ".heights", tile->h.size(), heights.value()});
        Fnv1a64 counters;
        counters.u64(stats.cells_nodata).u64(stats.cells_terrain).u64(stats.cells_junction_core).u64(stats.cells_approach_core)
            .u64(stats.cells_junction_blend).u64(stats.cells_approach_blend).u64(stats.cells_deck_clamped).u64(stats.deck_clearance_violations)
            .u64(stats.blend_stations).u64(stats.steep_blend_stations).u64(stats.junctions_carved);
        const std::uint64_t carved = stats.cells_junction_core + stats.cells_approach_core + stats.cells_junction_blend + stats.cells_approach_blend;
        out.push_back({prefix + ".owners", carved, counters.value()});
    }
}

} // namespace

std::vector<Entry> compute_g3d() {
    std::vector<Entry> out;
    fixture(out, "frankfurt_hoechst", "frankfurt_hoechst.osm.pbf", {{1856, 21718}, {1852, 21718}});
    fixture(out, "hoechst_a66", "hoechst_a66.osm.pbf", {{1822, 21685}, {1820, 21684}});
    Fnv1a64 all; std::uint64_t n = 0;
    for (const Entry& e : out) { all.str(e.name).u64(e.count).u64(e.hash); n += e.count; }
    out.push_back({"all", n, all.value()});
    return out;
}

} // namespace g2m_golden
