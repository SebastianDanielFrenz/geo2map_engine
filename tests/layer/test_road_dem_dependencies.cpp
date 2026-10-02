#include "g2m/layer/road_dem_dependencies.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
using namespace g2m;
namespace {
RoadGraph road(geom::PointMm a, geom::PointMm b) {
    return {{{1, a}, {2, b}}, {{10, 1, {1, 2}, {{"highway", "service"}}}}, {{10, 0, 1, 2}}, 0, 0};
}
const geo::UtmZone zone{32, geo::Hemisphere::North};
}
TEST_CASE("Road DEM dependencies cover complete geometry beyond a tile halo", "[layer][road_dem]") {
    const auto refs = road_dem_dependencies(road({128000, 128000}, {1664000, 128000}), zone, 0);
    REQUIRE(refs); REQUIRE(refs->size() == 7);
    for (std::size_t i = 0; i < refs->size(); ++i) {
        CHECK((*refs)[i].layer == "g2m.elev.base"); CHECK((*refs)[i].key.level == 0);
        CHECK((*refs)[i].key.x == static_cast<std::int32_t>(i)); CHECK((*refs)[i].key.y == 0);
    }
    const auto negative = road_dem_dependencies(road({-1000, -1000}, {1000, 1000}), zone, 0);
    REQUIRE(negative); CHECK(negative->size() == 4);
    const auto expanded = road_dem_dependencies(road({1000, 1000}, {2000, 2000}), zone, 65000);
    REQUIRE(expanded); CHECK(expanded->size() == 4);
}
TEST_CASE("Road DEM dependencies are exact, sorted and bounded", "[layer][road_dem]") {
    auto diagonal = road({128000, 128000}, {1152000, 1152000});
    const auto refs = road_dem_dependencies(diagonal, zone, 0); REQUIRE(refs);
    CHECK(refs->size() < 25); CHECK(std::is_sorted(refs->begin(), refs->end()));
    CHECK_FALSE(road_dem_dependencies(diagonal, zone, 0, 1));
    CHECK_FALSE(road_dem_dependencies(diagonal, zone, -1));
    CHECK_FALSE(road_dem_dependencies(diagonal, zone, 0, 0));
    CHECK_FALSE(road_dem_dependencies(road({0, 0}, {std::int64_t{1} << 50, 0}), zone));
    CHECK_FALSE(road_dem_dependencies(road({0, 0}, {1000000000, 1000000000}), zone));
    diagonal.ways[0].node_ids[1] = 99; CHECK_FALSE(road_dem_dependencies(diagonal, zone));
    const auto empty = road_dem_dependencies({}, zone); REQUIRE(empty); CHECK(empty->empty());
}
