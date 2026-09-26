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
