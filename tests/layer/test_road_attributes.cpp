#include "g2m/layer/road_attributes.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <limits>
using namespace g2m;
namespace {
RoadGraphWay way(std::vector<std::pair<std::string, std::string>> tags) {
    std::sort(tags.begin(), tags.end()); return {1, 0, {1, 2}, std::move(tags)};
}
}
TEST_CASE("Road attributes resolve physical tags without locale or floating parsing", "[layer][road_attributes]") {
    const auto a = resolve_road_attributes(way({{"highway", "residential"}, {"lanes", "2"},
        {"width", " 7.2505 m "}, {"surface", "gravel"}, {"bridge", "viaduct"}, {"incline", "-3.25%"}}));
    REQUIRE(a); CHECK(a->width_mm == 7251); CHECK(a->lanes == 2); CHECK_FALSE(a->width_inferred);
    CHECK(a->surface == SurfaceKind::Gravel); CHECK(a->land_class == LandClass::UnpavedRoad);
    CHECK_FALSE(a->surface_inferred); CHECK(a->bridge); CHECK_FALSE(a->tunnel);
    CHECK(a->incline_per_mille == -33); CHECK(a->max_grade_per_mille == 150);
    CHECK(a->crown_per_mille == 25); CHECK(a->shoulder_mm == 1500); CHECK(a->blend_mm == 5000);
}
TEST_CASE("Road attributes reject malformed optional values and use bounded class defaults", "[layer][road_attributes]") {
    for (const auto& width : {"nan", "inf", "1e9", "200", "0", "0.2", "5 feet", "999999999999999", "7.1234567"}) {
        const auto a = resolve_road_attributes(way({{"highway", "primary"}, {"width", width}, {"lanes", "2"}}));
        REQUIRE(a); CHECK(a->width_mm == 6500); CHECK(a->width_inferred);
    }
    for (const auto& lanes : {"", "0", "-2", "1.5", "33", "999999999999999"}) {
        const auto a = resolve_road_attributes(way({{"highway", "service"}, {"lanes", lanes}, {"surface", "unknown"}}));
        REQUIRE(a); CHECK(a->width_mm == 4000); CHECK(a->lanes == 0); CHECK(a->surface_inferred);
    }
    const auto tunnel = resolve_road_attributes(way({{"highway", "motorway"}, {"tunnel", "building_passage"},
        {"bridge", "no"}, {"incline", "up"}}));
    REQUIRE(tunnel); CHECK(tunnel->tunnel); CHECK_FALSE(tunnel->bridge); CHECK_FALSE(tunnel->incline_per_mille);
    CHECK(tunnel->max_grade_per_mille == 60); CHECK(tunnel->blend_mm == 15000);
    CHECK_FALSE(resolve_road_attributes(way({{"highway", "footway"}})));
    CHECK_FALSE(resolve_road_attributes(way({{"highway", "service"}, {"area", "yes"}})));
    auto bad = way({{"highway", "service"}}); bad.tags.push_back({"highway", "primary"});
    CHECK_FALSE(resolve_road_attributes(bad));
}
