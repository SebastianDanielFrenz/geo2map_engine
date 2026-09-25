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
