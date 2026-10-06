// Street fidelity P2 slice 1: the carving parameter file (data/carve/carve_default.json) and its strict loader.
#include "g2m/layer/carve_params.h"
#include "g2m/layer/road_attributes.h"
#include "g2m/layer/road_rules.h"
#include "g2m/core/hash.h"
#include <algorithm>
#include <catch2/catch_test_macros.hpp>
#include <fstream>
#include <iterator>
#include <string>

using namespace g2m;
namespace {
std::string embedded_text() {
    const auto bytes = builtin_carve_default_bytes();
    return std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}
Result<CarveParams> load_text(const std::string& text) {
    return load_carve_params({reinterpret_cast<const std::byte*>(text.data()), text.size()});
}
std::string mutated(const std::string& from, const std::string& to) {
    auto text = embedded_text();
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
    return text;
}
RoadGraphWay way_of(const std::string& highway) {
    return {1, 0, {1, 2}, {{"highway", highway}}};
}
}

TEST_CASE("The embedded carve file equals data/carve/carve_default.json and its hash is pinned", "[layer][carve]") {
    std::ifstream in(std::string(G2M_SOURCE_DIR) + "/data/carve/carve_default.json", std::ios::binary);
    REQUIRE(in);
    const std::string file((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    CHECK(file == embedded_text());
    const auto digest = Sha256::of(builtin_carve_default_bytes());
    CHECK(to_hex(digest) == "db0a26a349d9ab5ec9cf62ffbaf23b0d69416e1126f7b7adf2843440afdc4020");
    CHECK(builtin_carve_default().content_sha256 == digest);
    // It is a separate file from the road rules: the roads.geom config hash never sees it (design D8, D12).
    CHECK(digest != builtin_road_rules_de().content_sha256);
}

TEST_CASE("The default carve parameters of design 8.2", "[layer][carve]") {
    const auto& c = builtin_carve_default();
    CHECK(c.smoothing.enabled); CHECK(c.smoothing.radius_m == 2); CHECK(c.smoothing.strength_percent == 35);
    REQUIRE(c.smoothing.kernel.size() == 5);
    CHECK(c.smoothing.kernel == std::vector<std::int32_t>{138, 621, 1024, 621, 138});
    CHECK(c.crown_round_mm == 500); CHECK(c.verge_drop_mm == 60);
    CHECK(c.blend_slope_factor == 3); CHECK(c.blend_min_mm == 3000); CHECK(c.blend_max_mm == 15000);
    CHECK(c.deck.clearance_mm == 550); CHECK(c.deck.ramp_mm == 4000); CHECK(c.deck.margin_mm == 1000);
    CHECK(c.blend_default_mm == 4000);
    CHECK(c.junction_fade_slope_per_mille == 10); CHECK(c.junction_fade_min_mm == 3000); CHECK(c.junction_fade_max_mm == 15000);
    CHECK(c.junction_fade_verge_mm == 1500);
    const JunctionFadeParams fade = c.junction_fade_params();
    CHECK(fade.slope == 0.010); CHECK(fade.min_m == 3.0); CHECK(fade.max_m == 15.0); CHECK(fade.verge_m == 1.5);
    CHECK(fade == JunctionFadeParams{});   // make_junction_surface's default is the shipped file
    REQUIRE(c.blend.size() == RoadStyle::default_style().highways.size());
    for (std::size_t i = 1; i < c.blend.size(); ++i) CHECK(c.blend[i - 1].highway < c.blend[i].highway);
    // The blend widths up to 15 m are what the junction rules' proof assumes.
    CHECK(c.blend_max_mm <= kCarveBlendMaxM * 1000);
}

TEST_CASE("The carve blend table records the RoadAttributes::blend_mm defaults of every class", "[layer][carve]") {
    const auto& c = builtin_carve_default();
    for (const auto& entry : RoadStyle::default_style().highways) {
        const auto attrs = resolve_road_attributes(way_of(std::string(entry.highway)));
        REQUIRE(attrs);
        INFO(entry.highway);
        CHECK(c.blend_for_class_mm(entry.highway) == attrs->blend_mm);
    }
    CHECK(c.blend_for_class_mm("footway") == c.blend_default_mm); // a class without an entry takes the default
    // Sabotage: one edited entry no longer matches resolve_road_attributes - the comparison above would fail.
    const auto edited = load_text(mutated("\"primary\": 12000", "\"primary\": 10000")); REQUIRE(edited);
    const auto primary = resolve_road_attributes(way_of("primary")); REQUIRE(primary);
    CHECK(edited->blend_for_class_mm("primary") != primary->blend_mm);
}

TEST_CASE("The carve loader is strict", "[layer][carve]") {
    const auto good = load_text(embedded_text()); REQUIRE(good);
    CHECK(*good == builtin_carve_default());
    CHECK_FALSE(load_text("not json"));
    CHECK_FALSE(load_text(""));
    CHECK_FALSE(load_text(mutated("g2m.carve/1", "g2m.carve/2")));
    CHECK_FALSE(load_text(mutated("\"crown_round_mm\": 500,", "\"crown_round_mm\": 500, \"bogus\": 1,")));  // unknown key
    CHECK_FALSE(load_text(mutated("\"crown_round_mm\": 500", "\"crown_round_mm\": 0")));
    CHECK_FALSE(load_text(mutated("\"crown_round_mm\": 500", "\"crown_round_mm\": 0.5")));                  // integers only
    CHECK_FALSE(load_text(mutated("\"verge_drop_mm\": 60", "\"verge_drop_mm\": 301")));
    CHECK_FALSE(load_text(mutated("\"verge_drop_mm\": 60", "\"verge_drop_mm\": -1")));
    CHECK_FALSE(load_text(mutated("\"enabled\": true", "\"enabled\": 1")));
    CHECK_FALSE(load_text(mutated("\"radius_m\": 2", "\"radius_m\": 3")));                                   // kernel length mismatch
    CHECK_FALSE(load_text(mutated("[138, 621, 1024, 621, 138]", "[138, 621, 1024, 621, 139]")));            // asymmetric
    CHECK_FALSE(load_text(mutated("[138, 621, 1024, 621, 138]", "[138, 621, 0, 621, 138]")));               // non-positive weight
    CHECK_FALSE(load_text(mutated("\"strength_percent\": 35", "\"strength_percent\": 101")));
    CHECK_FALSE(load_text(mutated("\"blend_slope_factor\": 3", "\"blend_slope_factor\": 0")));
    CHECK_FALSE(load_text(mutated("\"blend_min_mm\": 3000", "\"blend_min_mm\": 16000")));                     // min above max
    CHECK_FALSE(load_text(mutated("\"blend_max_mm\": 15000", "\"blend_max_mm\": 16000")));                  // above the junction proof's 15 m
    CHECK_FALSE(load_text(mutated("\"motorway\": 15000", "\"motorway\": 2000")));                           // below blend_min_mm
    CHECK_FALSE(load_text(mutated("\"motorway_link\": 15000, ", "")));                                      // missing class
    CHECK_FALSE(load_text(mutated("\"track\": 3000,", "\"footway\": 3000,")));                               // unknown + missing class
    CHECK_FALSE(load_text(mutated("\"default\": 4000,", "")));
    CHECK_FALSE(load_text(mutated("\"clearance_mm\": 550", "\"clearance_mm\": 0")));
    CHECK_FALSE(load_text(mutated("\"ramp_mm\": 4000", "\"ramp_mm\": 0")));
    CHECK_FALSE(load_text(mutated("\"margin_mm\": 1000", "\"margin_mm\": -1")));
    CHECK_FALSE(load_text(mutated("\"deck\": {", "\"deckx\": {")));
    CHECK_FALSE(load_text(mutated("\"sources\": {", "\"sourcesx\": {")));
    // junction_fade (D42)
    CHECK_FALSE(load_text(mutated("\"junction_fade\": {", "\"junction_fadex\": {")));
    CHECK_FALSE(load_text(mutated("\"slope_per_mille\": 10", "\"slope_per_mille\": 0")));
    CHECK_FALSE(load_text(mutated("\"slope_per_mille\": 10", "\"slope_per_mille\": 10, \"bogus\": 1")));
    CHECK_FALSE(load_text(mutated("\"min_mm\": 3000, \"max_mm\": 15000", "\"min_mm\": 16000, \"max_mm\": 15000")));   // min above max
    CHECK_FALSE(load_text(mutated("\"verge_mm\": 1500", "\"verge_mm\": 3001")));                                     // verge above min
    CHECK_FALSE(load_text(mutated("\"verge_mm\": 1500", "\"verge_mm\": 0")));
    CHECK(load_text(mutated("\"verge_mm\": 1500", "\"verge_mm\": 3000")));                                           // verge == min is fine
}
