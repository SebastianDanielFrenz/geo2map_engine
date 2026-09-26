// tests/test_smoke.cpp - g2m_ps_bridge scaffolding smoke test (PLAN.md R2.2
// section 1c, G5). Proves the bridge target actually links against BOTH
// physics_sim's ps_core (Jolt included - ps::jolt_backend_self_check()
// touches the Jolt backend for real) and geo2map_engine's own g2m_phys
// (PhysicsTileGrid), from one executable outside either project's own build.
// G2mTerrainSource (g2m_terrain_source.h) is the real deliverable, added G6 -
// this file is deliberately trivial.
#include "g2m/core/geo/session_frame.h"
#include "g2m/phys/physics_grid.h"
#include "g2m/ps_bridge/version.h"
#include "ps/version.h"

#include <catch2/catch_test_macros.hpp>

#include <cstring>

TEST_CASE("g2m_ps_bridge: version string is non-empty", "[bridge][smoke]") {
    const char* v = g2m::ps_bridge::bridge_version_string();
    REQUIRE(v != nullptr);
    REQUIRE(std::strlen(v) > 0);
}

TEST_CASE("g2m_ps_bridge: links against physics_sim's ps_core, Jolt included", "[bridge][smoke]") {
    const char* v = ps::version_string();
    REQUIRE(v != nullptr);
    REQUIRE(std::strlen(v) > 0);
    REQUIRE(ps::jolt_backend_self_check());
}

TEST_CASE("g2m_ps_bridge: links against geo2map_engine's own g2m_phys", "[bridge][smoke]") {
    const g2m::geo::SessionFrame frame(g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North}, 474000, 5560000);
    const g2m::phys::PhysicsTileGrid grid(frame);
    CHECK(grid.samples() == g2m::phys::kPhysSamples);
    CHECK(grid.tile_size_m() == static_cast<double>(g2m::phys::kPhysSamples - 1));
}
