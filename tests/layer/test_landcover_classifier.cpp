// Street fidelity P2 slice 1: the ESA WorldCover classifier table (data/classifiers/esa_worldcover_v200.json).
#include "g2m/layer/landcover_classifier.h"
#include "g2m/core/hash.h"
#include <catch2/catch_test_macros.hpp>
#include <fstream>
#include <iterator>
#include <string>

using namespace g2m;
namespace {
std::string embedded_text() {
    const auto bytes = builtin_esa_worldcover_classifier_bytes();
    return std::string(reinterpret_cast<const char*>(bytes.data()), bytes.size());
}
Result<LandcoverClassifier> load_text(const std::string& text) {
    return LandcoverClassifier::load({reinterpret_cast<const std::byte*>(text.data()), text.size()});
}
std::string mutated(const std::string& from, const std::string& to) {
    auto text = embedded_text();
    const auto at = text.find(from);
    REQUIRE(at != std::string::npos);
    text.replace(at, from.size(), to);
    return text;
}
ClassifierFacts facts(const char* scheme, std::uint8_t code) {
    ClassifierFacts f; f.landcover_scheme = scheme; f.landcover_class = code; return f;
}
}

TEST_CASE("The embedded classifier table equals the data file and its hash is pinned", "[layer][classifier]") {
    std::ifstream in(std::string(G2M_SOURCE_DIR) + "/data/classifiers/esa_worldcover_v200.json", std::ios::binary);
    REQUIRE(in);
    const std::string file((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    CHECK(file == embedded_text());
    const auto& table = builtin_esa_worldcover_classifier();
    CHECK(to_hex(table.table_hash()) == "cc47aa8343c0c795f3f2e5a5f889725876b890effb2ffd1212102ffc7e757e16");
    CHECK(table.table_hash() == Sha256::of(builtin_esa_worldcover_classifier_bytes()));
    CHECK(table.name() == "g2m.esa_worldcover_v200");
    CHECK(table.scheme() == "esa_worldcover_v200");
}

TEST_CASE("The ESA WorldCover table maps the 11 classes", "[layer][classifier]") {
    const auto& table = builtin_esa_worldcover_classifier();
    REQUIRE(table.entries().size() == 11);
    struct Row { std::uint8_t code; LandClass land; SurfaceKind surface; };
    const Row expected[] = {
        {10, LandClass::TreeCover, SurfaceKind::Dirt}, {20, LandClass::Shrub, SurfaceKind::Grass},
        {30, LandClass::Grassland, SurfaceKind::Grass}, {40, LandClass::Cropland, SurfaceKind::Dirt},
        {50, LandClass::BuiltUp, SurfaceKind::Asphalt}, {60, LandClass::Bare, SurfaceKind::Compacted},
        {70, LandClass::SnowIce, SurfaceKind::Unknown}, {80, LandClass::Water, SurfaceKind::Water},
        {90, LandClass::Wetland, SurfaceKind::Grass}, {95, LandClass::Wetland, SurfaceKind::Dirt},
        {100, LandClass::Bare, SurfaceKind::Grass}};
    for (const auto& row : expected) {
        INFO(static_cast<int>(row.code));
        CHECK(table.land_class(facts("esa_worldcover_v200", row.code)) == row.land);
        CHECK(table.surface(facts("esa_worldcover_v200", row.code)) == row.surface);
    }
    // No-data (0), a code outside the legend and a foreign scheme all classify as Unknown.
    for (const std::uint8_t code : {std::uint8_t{0}, std::uint8_t{1}, std::uint8_t{11}, std::uint8_t{255}}) {
        CHECK(table.land_class(facts("esa_worldcover_v200", code)) == LandClass::Unknown);
        CHECK(table.surface(facts("esa_worldcover_v200", code)) == SurfaceKind::Unknown);
    }
    CHECK(table.land_class(facts("other_scheme", 10)) == LandClass::Unknown);
    CHECK(table.surface(facts("other_scheme", 10)) == SurfaceKind::Unknown);
    CHECK(table.entries()[0].code == 10); CHECK(table.entries()[10].code == 100); // ascending
}

TEST_CASE("The classifier loader is strict", "[layer][classifier]") {
    const auto good = load_text(embedded_text()); REQUIRE(good);
    CHECK(good->entries().size() == 11);
    CHECK(good->table_hash() == builtin_esa_worldcover_classifier().table_hash());
    CHECK_FALSE(load_text("not json"));
    CHECK_FALSE(load_text(""));
    CHECK_FALSE(load_text(mutated("g2m.classifier_table/1", "g2m.classifier_table/2")));
    CHECK_FALSE(load_text(mutated("\"scheme\": \"esa_worldcover_v200\",", "\"scheme\": \"esa_worldcover_v200\", \"bogus\": 1,")));
    CHECK_FALSE(load_text(mutated("\"scheme\": \"esa_worldcover_v200\"", "\"scheme\": \"\"")));
    CHECK_FALSE(load_text(mutated("\"95\":", "\"96\":")));                                  // unknown code + missing 95
    CHECK_FALSE(load_text(mutated("\"100\": {", "\"101\": {")));
    CHECK_FALSE(load_text(mutated("\"tree_cover\"", "\"forest\"")));                         // unknown land class name
    CHECK_FALSE(load_text(mutated("\"surface\": \"water\"", "\"surface\": \"ocean\"")));     // unknown surface name
    CHECK_FALSE(load_text(mutated("\"surface\": \"water\"", "\"surface\": 11")));
    CHECK_FALSE(load_text(mutated("{ \"land_class\": \"shrub\",      \"surface\": \"grass\" }", "{ \"land_class\": \"shrub\" }")));
    CHECK_FALSE(load_text(mutated("\"sources\": {", "\"sourcesx\": {")));
    // A one-name edit moves the table hash (it is the classifier's identity in the terrain.class config hash).
    const auto edited = load_text(mutated("\"shrub\",      \"surface\": \"grass\"", "\"shrub\",      \"surface\": \"dirt\"")); REQUIRE(edited);
    CHECK(edited->table_hash() != good->table_hash());
    CHECK(edited->surface(facts("esa_worldcover_v200", 20)) == SurfaceKind::Dirt);
}
