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
    for (int y = 0; y < 256; ++y) for (int x = 55; x < 145; ++x) t.h[y * 256 + x] = kHeightNoData;
    for (int y = 0; y < 256; ++y) for (int x = 155; x < 256; ++x) t.h[y * 256 + x] += 256;
    const HeightTile* tiles[] = {&t}; const auto dem = RoadDemSampler::make(tiles); REQUIRE(dem);
    auto bridge = graph(true); bridge.nodes = {{1, {50000, 10000}}, {2, {150000, 10000}}, {3, {10000, 10000}}, {4, {200000, 10000}}};
    bridge.ways.push_back({20, 1, {3, 1}, {{"highway", "service"}}}); bridge.ways.push_back({30, 1, {2, 4}, {{"highway", "service"}}});
    bridge.edges.push_back({20, 0, 3, 1}); bridge.edges.push_back({30, 0, 2, 4});
    const auto builder = RoadReferenceBuilder::make(bridge); REQUIRE(builder);
    const auto profile = build_road_profile(*builder, *dem, zone, 10, {0, 1}); REQUIRE(profile);
    CHECK(profile->attributes.bridge); REQUIRE(profile->vertical.segments.size() == 1);
    CHECK(profile->start_anchor_inferred); CHECK(profile->end_anchor_inferred);
    const auto mid = profile->at(50); REQUIRE(mid); CHECK(mid->height_m == 100.5); CHECK(mid->grade == 0.015);
    CHECK(profile->at(0)->grade == 0); CHECK(profile->at(profile->reference.length_m)->grade == 0);
    const auto ground = graph(); const auto ground_builder = RoadReferenceBuilder::make(ground); REQUIRE(ground_builder);
    CHECK_FALSE(build_road_profile(*ground_builder, *dem, zone, 10, {0, 1}));
    const auto approach = build_road_profile(*builder, *dem, zone, 20, {0, 1}); REQUIRE(approach);
    CHECK(approach->at(approach->reference.length_m)->height_m == profile->at(0)->height_m);
    CHECK(approach->at(approach->reference.length_m)->grade == profile->at(0)->grade);
    t.h[10 * 256 + 18] = kHeightNoData;
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
TEST_CASE("Incoming roads share a tilted junction plane instead of flat endpoint grades", "[layer][road_profile]") {
    auto g = graph(); g.nodes.push_back({3, {10000, 110000}});
    g.ways.push_back({20, 1, {1, 3}, {{"highway", "service"}}}); g.edges.push_back({20, 0, 1, 3});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    auto t = tile();
    for (int y = 0; y < 256; ++y) for (int x = 0; x < 256; ++x) t.h[y * 256 + x] += 2 * x + 4 * y;
    const HeightTile* tiles[] = {&t}; const auto dem = RoadDemSampler::make(tiles); REQUIRE(dem);
    const auto east = build_road_profile(*builder, *dem, zone, 10, {0, 1}); REQUIRE(east);
    const auto north = build_road_profile(*builder, *dem, zone, 20, {0, 1}); REQUIRE(north);
    const auto e0 = east->at(0), n0 = north->at(0); REQUIRE(e0); REQUIRE(n0);
    CHECK(e0->height_m == n0->height_m);
    CHECK(std::fabs(e0->grade - 2.0 / 256) < 1e-14); CHECK(std::fabs(n0->grade - 4.0 / 256) < 1e-14);
    for (int i = 0; i <= 100; ++i) {
        const auto e = east->at(i), n = north->at(i); REQUIRE(e); REQUIRE(n);
        CHECK(std::fabs(e->height_m - (e0->height_m + i * 2.0 / 256)) < 1e-10);
        CHECK(std::fabs(n->height_m - (n0->height_m + i * 4.0 / 256)) < 1e-10);
    }
}
TEST_CASE("Short motorway decks share bounded DEM corrections with their approaches", "[layer][road_profile]") {
    auto g = graph(true); g.nodes = {{1, {50000, 10000}}, {2, {68000, 10000}}, {3, {10000, 10000}}, {4, {120000, 10000}}};
    g.ways[0].rank = 9; g.ways[0].tags = {{"bridge", "yes"}, {"highway", "motorway"}};
    g.ways.push_back({20, 9, {3, 1}, {{"highway", "motorway"}}}); g.ways.push_back({30, 9, {2, 4}, {{"highway", "motorway"}}});
    g.edges.push_back({20, 0, 3, 1}); g.edges.push_back({30, 0, 2, 4});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    auto t = tile();
    for (int y = 0; y < 256; ++y) for (int x = 80; x < 256; ++x) t.h[y * 256 + x] += 38;
    const HeightTile* tiles[] = {&t}; const auto dem = RoadDemSampler::make(tiles); REQUIRE(dem);
    const auto deck = build_road_profile(*builder, *dem, zone, 10, {0, 1}); REQUIRE(deck);
    CHECK(deck->start_anchor_adjusted); CHECK(deck->end_anchor_adjusted);
    CHECK(std::fabs(deck->at(0)->height_m - 100) <= 0.25);
    CHECK(std::fabs(deck->at(18)->height_m - (100 + 38.0 / 256)) <= 0.25);
    for (int i = 0; i <= 100; ++i) {
        const auto pose = deck->at(18.0 * i / 100); REQUIRE(pose);
        CHECK(std::fabs(pose->grade) <= 0.06 + 1e-10);
        CHECK(std::fabs(pose->second_derivative) <= 1.0 / 1500 + 1e-10);
    }
    const auto approach = build_road_profile(*builder, *dem, zone, 20, {0, 1}); REQUIRE(approach);
    CHECK(approach->at(approach->reference.length_m)->height_m == deck->at(0)->height_m);
    CHECK(approach->at(approach->reference.length_m)->grade == deck->at(0)->grade);
    for (int y = 0; y < 256; ++y) for (int x = 80; x < 256; ++x) t.h[y * 256 + x] += 2560;
    const auto excessive = build_road_profile(*builder, *dem, zone, 10, {0, 1}); REQUIRE_FALSE(excessive);
    CHECK(excessive.error().message == "road profile: bridge anchor correction exceeds 0.25 m");
    const auto incomplete = graph(true); const auto incomplete_builder = RoadReferenceBuilder::make(incomplete); REQUIRE(incomplete_builder);
    CHECK_FALSE(build_road_profile(*incomplete_builder, *dem, zone, 10, {0, 1}));
}

TEST_CASE("Bridge merges share bounded approach estimates without sampling underpass DEM", "[layer][road_profile]") {
    auto g = graph(true);
    g.nodes = {{1, {50000, 10000}}, {2, {150000, 10000}}, {3, {10000, 10000}},
        {4, {200000, 10000}}, {5, {50000, 60000}}};
    g.ways.push_back({20, 1, {3, 1}, {{"highway", "service"}}});
    g.ways.push_back({30, 1, {2, 4}, {{"highway", "service"}}});
    g.ways.push_back({40, 1, {5, 1}, {{"highway", "service"}}});
    g.edges.push_back({20, 0, 3, 1}); g.edges.push_back({30, 0, 2, 4}); g.edges.push_back({40, 0, 5, 1});
    const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
    auto t = tile();
    for (int y = 5; y <= 15; ++y) for (int x = 25; x <= 45; ++x) t.h[y * 256 + x] = kHeightNoData;
    const HeightTile* tiles[] = {&t}; const auto dem = RoadDemSampler::make(tiles); REQUIRE(dem);
    const auto deck = build_road_profile(*builder, *dem, zone, 10, {0, 1}); REQUIRE(deck);
    const auto ground = build_road_profile(*builder, *dem, zone, 20, {0, 1}); REQUIRE(ground);
    const auto ramp = build_road_profile(*builder, *dem, zone, 40, {0, 1}); REQUIRE(ramp);
    CHECK(deck->at(0)->height_m == ground->at(ground->reference.length_m)->height_m);
    CHECK(deck->at(0)->height_m == ramp->at(ramp->reference.length_m)->height_m);
    CHECK(deck->start_anchor_inferred);
    for (int y = 37; y <= 47; ++y) for (int x = 45; x <= 55; ++x) t.h[y * 256 + x] += 256;
    const auto disagreement = build_road_profile(*builder, *dem, zone, 10, {0, 1}); REQUIRE_FALSE(disagreement);
    CHECK(disagreement.error().message == "road profile: junction approach estimates disagree by more than 0.5 m");
}
