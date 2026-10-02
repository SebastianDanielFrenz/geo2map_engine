#include "g2m/layer/road_dem_sampler.h"
#include <catch2/catch_test_macros.hpp>
#include <memory>
using namespace g2m;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};
auto tile(int tx, int ty) {
    auto out = std::make_unique<HeightTile>(); out->key = {zone, 0, tx, ty};
    for (int y = 0; y < 256; ++y) for (int x = 0; x < 256; ++x)
        out->h[y * 256 + x] = (tx * 256 + x) * 2 + (ty * 256 + y) * 4;
    return out;
}
}
TEST_CASE("Road DEM sampling interpolates cell-centred heights across tile boundaries", "[layer][road_dem]") {
    auto a = tile(0, 0), b = tile(1, 0), c = tile(0, 1), d = tile(1, 1);
    const HeightTile* tiles[] = {d.get(), a.get(), c.get(), b.get()};
    const auto sampler = RoadDemSampler::make(tiles); REQUIRE(sampler);
    for (const auto& p : {geom::PointMm{500, 500}, {1000, 1000}, {256000, 256000}, {256500, 256500}}) {
        const auto value = sampler->sample(zone, p); REQUIRE(value);
        CHECK(*value == ((p.x - 500) * 2 + (p.y - 500) * 4) / 1000);
    }
    const auto height = sampler->junction_height(zone, {256000, 256000}); REQUIRE(height); CHECK(*height == 1533);
    const auto missing = sampler->sample(zone, {-1, -1}); CHECK_FALSE(missing);
    CHECK_FALSE(sampler->junction_height(zone, {1000, 1000}));
    CHECK_FALSE(sampler->junction_height(zone, {256000, 256000}, 16));
    b->h[0] = kHeightNoData; CHECK_FALSE(sampler->sample(zone, {256000, 500}));
}
TEST_CASE("Road DEM sampler rejects invalid dependencies and rejects NoData before averaging", "[layer][road_dem]") {
    auto a = tile(0, 0); const HeightTile* tiles[] = {a.get()};
    const auto sampler = RoadDemSampler::make(tiles); REQUIRE(sampler);
    a->h[0] = 1; a->h[1] = 2;
    CHECK(sampler->sample(zone, {1000, 500}).value() == 2); // half-even
    a->h[0] = -1; a->h[1] = -2;
    CHECK(sampler->sample(zone, {1000, 500}).value() == -2);
    a->h[1] = kHeightNoData;
    CHECK(sampler->sample(zone, {500, 500}).value() == -1); // zero-weight neighbour is irrelevant
    CHECK_FALSE(sampler->sample(zone, {1000, 500}));
    const HeightTile* duplicate[] = {a.get(), a.get()}; CHECK_FALSE(RoadDemSampler::make(duplicate));
    const HeightTile* null[] = {nullptr}; CHECK_FALSE(RoadDemSampler::make(null));
    a->key.level = 1; CHECK_FALSE(RoadDemSampler::make(tiles));
}
TEST_CASE("Junction planes preserve both terrain slopes and resist a local height return", "[layer][road_dem]") {
    auto a = tile(0, 0); const HeightTile* tiles[] = {a.get()};
    const auto sampler = RoadDemSampler::make(tiles); REQUIRE(sampler);
    const auto plane = sampler->junction_plane(zone, {10500, 10500}); REQUIRE(plane);
    CHECK(plane->height == 60); CHECK(plane->grade_x == 2.0 / 256); CHECK(plane->grade_y == 4.0 / 256);
    a->h[10 * 256 + 10] += 100000;
    const auto robust = sampler->junction_plane(zone, {10500, 10500}); REQUIRE(robust);
    CHECK(robust->grade_x == plane->grade_x); CHECK(robust->grade_y == plane->grade_y);
    a->h[10 * 256 + 10] = kHeightNoData;
    CHECK_FALSE(sampler->junction_plane(zone, {10500, 10500}));
    CHECK_FALSE(sampler->junction_plane(zone, {10500, 10500}, 0));
}
