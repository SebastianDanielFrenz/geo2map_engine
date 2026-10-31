// tests/golden/test_golden.cpp - this build's golden hashes must equal
// testdata/golden/g0_golden.txt and g1_golden.txt (recorded once; identical
// on every leg).

#include "golden.h"

#include <catch2/catch_test_macros.hpp>

#include <fstream>
#include <map>
#include <sstream>
#include <string>

namespace {

void check_against(const std::string& file, const std::vector<g2m_golden::Entry>& entries) {
    std::ifstream in(std::string(G2M_SOURCE_DIR) + "/testdata/golden/" + file);
    REQUIRE(in.good());
    std::map<std::string, std::string> expected;
    std::string line;
    while (std::getline(in, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        std::string name, count, hash;
        ss >> name >> count >> hash;
        expected[name] = count + " " + hash;
    }
    CHECK(entries.size() == expected.size());
    for (const auto& e : entries) {
        INFO(e.name);
        const auto it = expected.find(e.name);
        REQUIRE(it != expected.end());
        std::istringstream ss(g2m_golden::format({e}));
        std::string name, count, hash;
        ss >> name >> count >> hash;
        CHECK(it->second == count + " " + hash);
    }
}

} // namespace

TEST_CASE("G0 golden hashes match testdata/golden/g0_golden.txt", "[golden]") {
    check_against("g0_golden.txt", g2m_golden::compute_g0());
}

TEST_CASE("G1 golden hashes match testdata/golden/g1_golden.txt", "[golden]") {
    check_against("g1_golden.txt", g2m_golden::compute_g1());
}

TEST_CASE("G1c golden hashes match testdata/golden/g1c_golden.txt", "[golden]") {
    check_against("g1c_golden.txt", g2m_golden::compute_g1c());
}

// PLAN.md G2.1: g2m.elev.base/g2m.terrain.height level 0, over the real
// import -> store -> local release -> Server path (golden_g2.cpp).
TEST_CASE("G2 golden hashes match testdata/golden/g2_golden.txt", "[golden]") {
    check_against("g2_golden.txt", g2m_golden::compute_g2());
}

// PLAN.md G2.3: g2m_mesh's select_chunks + build_chunk over 2 fixed cameras
// and a deterministic synthetic HeightTile source (golden_g2mesh.cpp) - a
// SEPARATE golden set from "g2" (G2.2 extends "g2" in parallel).
TEST_CASE("G2mesh golden hashes match testdata/golden/g2mesh_golden.txt", "[golden]") {
    check_against("g2mesh_golden.txt", g2m_golden::compute_g2mesh());
}

// PLAN.md 6.5 / roads_plan.md: g2m::extract_road_segments +
// g2m::rasterize_road_segments over a fixed synthetic OsmData set
// (golden_g2roads.cpp) - proves the client-side OSM road raster (segment
// extraction incl. SurfaceKind/paved-vs-unpaved LandClass, and the exact
// rasteriser) is byte-identical Windows/Linux.
TEST_CASE("G2roads golden hashes match testdata/golden/g2roads_golden.txt", "[golden]") {
    check_against("g2roads_golden.txt", g2m_golden::compute_g2roads());
}

// G2.5a-grip commit G-a: g2m::paint_road_segments/paint_road_blocks/
// rasterize_road_blocks/RoadValueLut/raster_params_for_render_level
// (golden_g2paint.cpp) - a separate golden set from "g2roads" above, for
// everything G-a added on top of it.
TEST_CASE("G2paint golden hashes match testdata/golden/g2paint_golden.txt", "[golden]") {
    check_against("g2paint_golden.txt", g2m_golden::compute_g2paint());
}

// Street fidelity P1 slice 5 (re-recorded by P2 slice 8 for the served encoding 7): roads.geom over the committed OSM
// fixtures (golden_g3xs.cpp); the same file is compared Windows vs Linux by tools/ci.ps1.
TEST_CASE("G3xs golden hashes match testdata/golden/g3xs_golden.txt", "[golden][road_cross_section]") {
    check_against("g3xs_golden.txt", g2m_golden::compute_g3xs());
}

// Street fidelity P2 slice 8: the junction, furniture and provenance sections of encoding 7 (golden_g3j.cpp).
TEST_CASE("G3j golden hashes match testdata/golden/g3j_golden.txt", "[golden][road_junction]") {
    check_against("g3j_golden.txt", g2m_golden::compute_g3j());
}

// Street fidelity P2 slice 9: the carved terrain g2m.terrain.carved level 0 (golden_g3d.cpp).
TEST_CASE("G3d golden hashes match testdata/golden/g3d_golden.txt", "[golden][carve]") {
    check_against("g3d_golden.txt", g2m_golden::compute_g3d());
}

TEST_CASE("G3rail golden hashes match testdata/golden/g3rail_golden.txt", "[golden][rail]") {
    check_against("g3rail_golden.txt", g2m_golden::compute_g3rail());
}

TEST_CASE("G3f golden hashes match testdata/golden/g3f_golden.txt", "[golden][fields]") {
    check_against("g3f_golden.txt", g2m_golden::compute_g3f());
}

TEST_CASE("G3s golden hashes match testdata/golden/g3s_golden.txt", "[golden][soil]") {
    check_against("g3s_golden.txt", g2m_golden::compute_g3s());
}

TEST_CASE("G3c golden hashes match testdata/golden/g3c_golden.txt", "[golden][terrain_class]") {
    check_against("g3c_golden.txt", g2m_golden::compute_g3c());
}

// dem.h/osm.h/landcover.h all document their sink output as independent of
// `jobs` (only DemImportReport's scheduling-dependent counters are not, and
// the fixture's 2 unique 1km DGM1 keys never evict from the default 256-
// entry km_cache regardless of jobs, so even those stay identical here) -
// this is G1c I6 item A's own explicit self-check of that claim.
TEST_CASE("G1c golden hashes are independent of jobs (1 == 3 == 8)", "[golden]") {
    const std::vector<g2m_golden::Entry> j1 = g2m_golden::compute_g1c_for_jobs(1);
    const std::vector<g2m_golden::Entry> j3 = g2m_golden::compute_g1c_for_jobs(3);
    const std::vector<g2m_golden::Entry> j8 = g2m_golden::compute_g1c_for_jobs(8);
    REQUIRE(j1.size() == j3.size());
    REQUIRE(j1.size() == j8.size());
    for (std::size_t i = 0; i < j1.size(); ++i) {
        INFO(j1[i].name);
        CHECK(j1[i].name == j3[i].name);
        CHECK(j1[i].count == j3[i].count);
        CHECK(j1[i].hash == j3[i].hash);
        CHECK(j1[i].name == j8[i].name);
        CHECK(j1[i].count == j8[i].count);
        CHECK(j1[i].hash == j8[i].hash);
    }
}
