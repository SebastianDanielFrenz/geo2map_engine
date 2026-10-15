// tests/world/test_buildings.cpp - g2m::world::extract_buildings / build_building_set (terrain migration phase 3b)
// against racing_game 5a3baed's own buildings.cpp: building_pins.inc was recorded by refgen/refgen_view.cpp from the
// game's code over building_test_data.h's cases; the ported code must reproduce every hash bit for bit.
#include "building_test_data.h"

#include "g2m/world/buildings.h"

#include <catch2/catch_test_macros.hpp>

#include <map>
#include <string>

namespace {
struct BuildingPin {
    const char* name;
    std::size_t count;
    std::uint64_t hash;
};
constexpr BuildingPin kPins[] = {
#include "building_pins.inc"
};
} // namespace

TEST_CASE("extract_buildings reproduces the game's buildings", "[world][buildings]") {
    std::map<std::string, BuildingPin> pins;
    for (const auto& p : kPins) pins.emplace(p.name, p);
    std::size_t matched = 0;
    for (const auto& c : g2m_building_test::cases()) {
        INFO(c.name);
        const auto b = g2m::world::extract_buildings(c.tile, c.key, c.min_height, c.fallback, c.storey, &g2m_building_test::ground);
        const auto it = pins.find(c.name);
        if (it == pins.end()) {
            CHECK(b.empty()); // a fixture tile the game found no building in
            continue;
        }
        CHECK(b.size() == it->second.count);
        CHECK(g2m_building_test::hash_buildings(b) == it->second.hash);
        ++matched;
    }
    CHECK(matched == pins.size());
}

TEST_CASE("the building scene covers every tag form", "[world][buildings]") {
    const auto scene = g2m_building_test::building_scene();
    const auto b = g2m::world::extract_buildings(scene, g2m_building_test::kOsmKey, 0.0, 8.0, 3.0, &g2m_building_test::ground);
    std::map<std::string, int> sources;
    int relations = 0, with_inner = 0, colour = 0, raised = 0;
    for (const auto& x : b) {
        ++sources[x.height_source];
        relations += x.relation;
        with_inner += !x.inners.empty();
        colour += !x.colour.empty();
        raised += x.min_height > 0;
        CHECK(x.bottom <= x.base + x.min_height);
    }
    CHECK(sources["height"] > 0);
    CHECK(sources["building:levels"] > 0);
    CHECK(sources["fallback"] > 0);
    CHECK(relations == 2);
    CHECK(with_inner == 1);
    CHECK(colour == 1);
    CHECK(raised == 2);
}

TEST_CASE("build_building_set is extract_buildings with the params", "[world][buildings]") {
    const auto scene = g2m_building_test::building_scene();
    g2m::world::BuildingParams params;
    params.min_height_m = 50.0;
    const auto set = g2m::world::build_building_set(scene, g2m_building_test::kOsmKey, params, &g2m_building_test::ground);
    CHECK(set.key == g2m_building_test::kOsmKey);
    const auto direct = g2m::world::extract_buildings(scene, g2m_building_test::kOsmKey, 50.0, 8.0, 3.0, &g2m_building_test::ground);
    CHECK(g2m_building_test::hash_buildings(set.buildings) == g2m_building_test::hash_buildings(direct));
}
