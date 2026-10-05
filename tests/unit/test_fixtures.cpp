// tests/unit/test_fixtures.cpp - PLAN.md G1c I6 item D: testdata/fixtures/
// as a whole stays within its documented 10 MiB budget
// (testdata/fixtures/README.md), and every file in it is exactly the one
// the README's provenance table says it is (by SHA-256, g2m::Sha256) - not
// silently re-cut, truncated, or replaced. Also fails if a fixture file
// exists on disk that the table does not know about (a fixture added
// without updating the README), and if a tabled file is missing from disk.
//
// Scope is testdata/fixtures/** only (not testdata/fuzz_seeds/**, which
// the README documents separately and which fuzzing's own corpus-reset
// logic - tests/fuzz/CMakeLists.txt - already treats as read-only input,
// not a size-budgeted asset this test needs to police).

#include "g2m/core/hash.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <istream>
#include <iterator>
#include <map>
#include <span>
#include <string>
#include <vector>

namespace {

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

std::vector<std::uint8_t> read_file(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

std::string sha256_hex(const std::vector<std::uint8_t>& bytes) {
    g2m::Sha256 hasher;
    hasher.update(bytes.data(), bytes.size());
    const g2m::Sha256::Digest digest = hasher.finish();
    return g2m::to_hex(std::span<const std::uint8_t>(digest.data(), digest.size()));
}

// The provenance table testdata/fixtures/README.md documents: every
// committed fixture file's path (relative to testdata/fixtures/, forward
// slashes) and its SHA-256. Kept in lockstep with the README by hand -
// there is no machine-readable source of truth other than the README
// itself (PLAN.md G1c I6 item C).
const std::map<std::string, std::string>& expected_hashes() {
    static const std::map<std::string, std::string> table = {
        {"dgm1/PKT_fixture.zip", "8fddb6a1910e5ba90d295026ac802caab2745b7d8312d3dce5b037329c07f956"},
        {"osm/frankfurt_hoechst.osm.pbf", "dab190344af86d2feca297e68637155c99ade32df8189b0fd5e4def3c3361cae"},
        {"osm/hoechst_a66.osm.pbf", "8556244549478372cccdead6d3fd59638b5e313fc9358ccb9a1a4b7ac2947dd8"},
        {"osm/unsorted_nodes.osm.pbf", "3431f2e25aefc19947999dd657949947441e47e432a8da664befe4a372a53f22"},
        {"regions.json", "586df4ace50c7cb45a795814af1b7e7371ca4f370ccb8aeb29cc62a0b59c40a5"},
        {"worldcover/fixture_N50E008_bbox.tif", "806c442100e789bb1d55f7342199f21ed6d7060efff4c96d65558dd1aeae37f1"},
        {"worldcover/fixture_N50E008_hoechst.tif", "a81638668b11726e7a9e700af02b3f46ec6dde893461252fdf70b47f1675ff11"},
    };
    return table;
}

} // namespace

TEST_CASE("fixtures: testdata/fixtures/** matches the README provenance table, total <= 10 MiB",
          "[fixtures][io][hash]") {
    const std::filesystem::path root = fixtures_dir();
    REQUIRE(std::filesystem::exists(root));

    std::uint64_t total_bytes = 0;
    std::size_t files_checked = 0;
    for (const auto& dir_entry : std::filesystem::recursive_directory_iterator(root)) {
        if (!dir_entry.is_regular_file()) continue;
        const std::filesystem::path rel = std::filesystem::relative(dir_entry.path(), root);
        const std::string rel_str = rel.generic_string();
        if (rel_str == "README.md") continue; // documentation, not a fixture

        INFO("fixture file: " << rel_str);
        const auto it = expected_hashes().find(rel_str);
        // Fails if a fixture file exists that is not in the README's table.
        REQUIRE(it != expected_hashes().end());

        const std::vector<std::uint8_t> bytes = read_file(dir_entry.path());
        total_bytes += bytes.size();
        ++files_checked;
        CHECK(sha256_hex(bytes) == it->second);
    }

    // Every tabled file is actually present on disk.
    CHECK(files_checked == expected_hashes().size());
    // testdata/fixtures/README.md's own documented budget.
    CHECK(total_bytes <= (std::uint64_t{10} << 20));
}
