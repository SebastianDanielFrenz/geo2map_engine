// Street fidelity P1 slice 2: the country road rules data file and its loader.
#include "g2m/layer/road_rules.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/core/hash.h"
#include <catch2/catch_test_macros.hpp>
#include <cstring>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>

using namespace g2m;
namespace {
std::string embedded_text() {
    const auto bytes = builtin_road_rules_de_bytes();
    return std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}
Result<RoadRules> load_text(const std::string& text) {
    return load_road_rules({reinterpret_cast<const std::byte*>(text.data()), text.size()});
}
// Replaces the first occurrence of `from`; the test fails if the file no longer contains it.
std::string mutated(const std::string& from, const std::string& to) {
    auto text = embedded_text();
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
    return text;
}
}

TEST_CASE("The embedded rules file equals data/road_rules/de.json and its hash is pinned", "[layer][road_rules]") {
    std::ifstream in(std::string(G2M_SOURCE_DIR) + "/data/road_rules/de.json", std::ios::binary);
    REQUIRE(in);
    const std::string file((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    CHECK(file == embedded_text());
    // A data edit must come with a conscious edit of this value (it enters the roads.geom config hash in slice 5).
    const auto bytes = builtin_road_rules_de_bytes();
    const auto digest = Sha256::of(bytes);
    CHECK(to_hex(digest) == "bc58b4f720a3be30e2938da20b93b8a789b478f330d9d59d9eba1066d693b85a");
    CHECK(builtin_road_rules_de().content_sha256 == digest);
}

TEST_CASE("The built-in German rules cover every road class and reproduce today's untagged widths", "[layer][road_rules]") {
    const auto& rules = builtin_road_rules_de();
    CHECK(rules.country == "DE");
    CHECK(rules.min_lane_mm == 2500); CHECK(rules.added_lane_side == AddedLaneSide::Right);
    CHECK(rules.cycle_lane_mm == 1850); CHECK(rules.sidewalk_mm == 2500); CHECK(rules.tagged_shoulder_mm == 1500);
    CHECK(rules.parking_width_mm(ParkingOrientation::Parallel) == 2000);
    CHECK(rules.parking_width_mm(ParkingOrientation::Diagonal) == 4500);
    CHECK(rules.parking_width_mm(ParkingOrientation::Perpendicular) == 5000);
    const auto& style = RoadStyle::default_style();
    REQUIRE(rules.classes.size() == style.highways.size());
    for (std::size_t i = 1; i < rules.classes.size(); ++i) CHECK(rules.classes[i - 1].highway < rules.classes[i].highway);
    for (const auto& entry : style.highways) CHECK(rules.find_class(entry.highway) != nullptr);
    CHECK(rules.find_class("footway") == nullptr);
    // D8: an untagged two-way road keeps today's paved width (design table in section 5).
    for (const char* name : {"primary", "secondary", "tertiary", "unclassified", "residential", "living_street", "service", "track"}) {
        const auto* c = rules.find_class(name); REQUIRE(c);
        const auto* s = style.find_highway(name); REQUIRE(s);
        CHECK(c->twoway_lanes * c->lane_mm == static_cast<std::int32_t>(s->default_width_m * 1000.0 + 0.5));
    }
    const auto* link = rules.find_class("motorway_link"); REQUIRE(link);
    CHECK(link->lane_mm + link->shoulder_inner_mm + link->shoulder_outer_mm == 6000);
    const auto* motorway = rules.find_class("motorway"); REQUIRE(motorway);
    CHECK(motorway->outer_lane_mm == 3750); CHECK(motorway->lane_mm == 3500);
    CHECK(motorway->shoulder_outer_mm == 2500); CHECK(motorway->shoulder_inner_mm == 750); CHECK(motorway->verge_mm == 1500);
    CHECK(motorway->oneway_lanes == 2); CHECK(motorway->twoway_lanes == 4);
    CHECK(motorway->min_edge_radius_m == 200);
    const auto* living = rules.find_class("living_street"); REQUIRE(living);
    CHECK(living->single_lane_both); CHECK(living->single_lane_mm == 5000); CHECK(living->lane_mm == 5000);
    CHECK(living->min_edge_radius_m == 10);
    CHECK_FALSE(rules.find_class("residential")->markings_default);
}

TEST_CASE("The rules loader is strict", "[layer][road_rules]") {
    const auto good = load_text(embedded_text()); REQUIRE(good);
    CHECK(good->classes == builtin_road_rules_de().classes);
    CHECK_FALSE(load_text("not json"));
    CHECK_FALSE(load_text(""));
    CHECK_FALSE(load_text(mutated("g2m.road_rules/1", "g2m.road_rules/2")));
    CHECK_FALSE(load_text(mutated("\"country\": \"DE\",", "\"country\": \"DE\", \"bogus\": 1,")));          // unknown key
    CHECK_FALSE(load_text(mutated("\"primary_link\": {", "\"primary_link\": { \"...\": \"as primary\",")));  // abbreviation marker
    CHECK_FALSE(load_text(mutated("\"track\": {", "\"trackx\": {")));                                     // missing + unknown class
    CHECK_FALSE(load_text(mutated("\"track\": {", "\"footway\": {")));
    CHECK_FALSE(load_text(mutated("\"verge_mm\": 1500", "\"verge_mm\": 20001")));                          // width range
    CHECK_FALSE(load_text(mutated("\"verge_mm\": 1500", "\"verge_mm\": -1")));
    CHECK_FALSE(load_text(mutated("\"cycle_lane_mm\": 1850", "\"cycle_lane_mm\": 1850.5")));              // integers only
    CHECK_FALSE(load_text(mutated("\"cycle_lane_mm\": 1850", "\"cycle_lane_mm\": \"1850\"")));
    CHECK_FALSE(load_text(mutated("\"added_lane_side\": \"right\"", "\"added_lane_side\": \"middle\"")));
    CHECK_FALSE(load_text(mutated("\"shift_min_m\": 60, \"shift_max_m\": 250", "\"shift_min_m\": 300, \"shift_max_m\": 250")));
    CHECK_FALSE(load_text(mutated("\"lane_mm\": 2750", "\"lane_mm\": 2000")));                             // below min_lane_mm
    CHECK_FALSE(load_text(mutated("\"single_lane_both\": true, \"single_lane_mm\": 5000", "\"single_lane_both\": true, \"lane_mm\": 3000, \"single_lane_mm\": 5000")));
    CHECK_FALSE(load_text(mutated("\"min_edge_radius_m\": {", "\"min_edge_radius_m\": { \"footway\": 5,")));
    CHECK_FALSE(load_text(mutated("\"default\": 10,", "\"default\": 0,")));
    // A taper too short for the radius: motorway 3750 mm over 20 m needs R <= 20^2 / (4 * 3.75) = 26.7 m, not 200 m.
    CHECK_FALSE(load_text(mutated("\"lane_add_m\": 60, \"width_change_m\": 60", "\"lane_add_m\": 20, \"width_change_m\": 60")));
    // ... but the same taper is fine once the stated radius allows it.
    auto relaxed = mutated("\"lane_add_m\": 60, \"width_change_m\": 60", "\"lane_add_m\": 20, \"width_change_m\": 60");
    const auto at = relaxed.find("\"motorway\": 200"); REQUIRE(at != std::string::npos);
    relaxed.replace(at, std::strlen("\"motorway\": 200"), "\"motorway\": 25");
    CHECK(load_text(relaxed));
}
