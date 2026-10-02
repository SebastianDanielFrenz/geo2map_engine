#include "g2m/layer/road_profile.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
using namespace g2m;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};
RoadGraph graph(bool bridge = false, bool tunnel = false) {
    RoadGraph out; out.nodes = {{1, {10000, 10000}}, {2, {110000, 10000}}};
    std::vector<std::pair<std::string, std::string>> tags;
    if (bridge) tags.emplace_back("bridge", "yes");
    tags.emplace_back("highway", "service");
    if (tunnel) tags.emplace_back("tunnel", "yes");
    out.ways = {{10, 1, {1, 2}, tags}}; out.edges = {{10, 0, 1, 2}}; return out;
}
HeightTile tile() { HeightTile out; out.key = {zone, 0, 0, 0}; out.h.fill(25600); return out; }
}
TEST_CASE("Road profiles fix shared DEM anchors and evaluate a complete flat stretch", "[layer][road_profile]") {
    const auto g = graph(); const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto t = tile(); const HeightTile* tiles[] = {&t}; const auto dem = RoadDemSampler::make(tiles); REQUIRE(dem);
    const auto profile = build_road_profile(*builder, *dem, zone, 10, {0, 1}); REQUIRE(profile);
    REQUIRE(profile->vertical.segments.size() == 20);
    for (int i = 0; i <= 100; ++i) {
        const auto pose = profile->at(i); REQUIRE(pose); CHECK(pose->height_m == 100); CHECK(pose->grade == 0);
    }
    CHECK_FALSE(profile->at(-1)); CHECK_FALSE(profile->at(101));
    RoadProfileLimits limits; limits.max_samples = 3;
    CHECK_FALSE(build_road_profile(*builder, *dem, zone, 10, {0, 1}, limits));
    CHECK_FALSE(build_road_profile(*builder, *dem, zone, 10, {0, 2}));
    CHECK_FALSE(build_road_profile(*builder, *dem, zone, 999, {0, 1}));
}
TEST_CASE("Bridge profiles ignore interior bare-earth NoData but require both abutments", "[layer][road_profile]") {
    auto t = tile();
    for (int y = 0; y < 256; ++y) for (int x = 40; x < 80; ++x) t.h[y * 256 + x] = kHeightNoData;
    for (int y = 0; y < 256; ++y) for (int x = 100; x < 256; ++x) t.h[y * 256 + x] += 256;
    const HeightTile* tiles[] = {&t}; const auto dem = RoadDemSampler::make(tiles); REQUIRE(dem);
    const auto bridge = graph(true); const auto builder = RoadReferenceBuilder::make(bridge); REQUIRE(builder);
    const auto profile = build_road_profile(*builder, *dem, zone, 10, {0, 1}); REQUIRE(profile);
    CHECK(profile->attributes.bridge); REQUIRE(profile->vertical.segments.size() == 1);
    const auto mid = profile->at(50); REQUIRE(mid); CHECK(mid->height_m == 100.5); CHECK(mid->grade == 0.015);
    CHECK(profile->at(0)->grade == 0); CHECK(profile->at(profile->reference.length_m)->grade == 0);
    const auto ground = graph(); const auto ground_builder = RoadReferenceBuilder::make(ground); REQUIRE(ground_builder);
    CHECK_FALSE(build_road_profile(*ground_builder, *dem, zone, 10, {0, 1}));
    t.h[10 * 256 + 10] = kHeightNoData;
    CHECK_FALSE(build_road_profile(*builder, *dem, zone, 10, {0, 1}));
}
TEST_CASE("Tunnel profiles remain unavailable rather than sampling ground as a tunnel", "[layer][road_profile]") {
    const auto g = graph(false, true); const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    const auto t = tile(); const HeightTile* tiles[] = {&t}; const auto dem = RoadDemSampler::make(tiles); REQUIRE(dem);
    const auto profile = build_road_profile(*builder, *dem, zone, 10, {0, 1}); CHECK_FALSE(profile);
    CHECK(profile.error().code == ErrorCode::Unsupported);
}
TEST_CASE("Cross-road median rejects a centreline DEM outlier without hiding missing data", "[layer][road_profile]") {
    const auto g = graph(); const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    auto t = tile();
    for (int x = 30; x < 90; ++x) { t.h[9 * 256 + x] += 25600; t.h[10 * 256 + x] += 25600; }
    const HeightTile* tiles[] = {&t}; const auto dem = RoadDemSampler::make(tiles); REQUIRE(dem);
    const auto profile = build_road_profile(*builder, *dem, zone, 10, {0, 1}); REQUIRE(profile);
    for (const auto& s : profile->vertical.segments) {
        CHECK(s.height0_m == 100); CHECK(s.height1_m == 100);
        CHECK(s.grade0 == 0); CHECK(s.grade1 == 0);
    }
    t.h[7 * 256 + 50] = kHeightNoData;
    CHECK_FALSE(build_road_profile(*builder, *dem, zone, 10, {0, 1}));
}
