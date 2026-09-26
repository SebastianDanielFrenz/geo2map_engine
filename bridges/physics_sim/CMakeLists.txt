# bridges/physics_sim - g2m_ps_bridge (PLAN.md R2.2 section 1b): the seam
# between geo2map_engine's own DGM1-derived physics-tile data (g2m_phys:
# PhysicsTileGrid, ResidentHeightSet, fill_physics_heights) and physics_sim's
# ps::terrain::ITerrainSource (G2mTerrainSource, include/g2m/ps_bridge/
# g2m_terrain_source.h, G6). No physics_sim type ever appears in g2m_phys's
# own public headers (include/g2m/phys) - this library is the only place the
# two projects meet.
#
# physics_sim.pin records the exact physics_sim commit this bridge is built
# and tested against (read, never enforced by CMake itself - tools/ci.ps1's
# bridge-release leg is what actually checks a local clone out to this pin).
#
# Reached only via the root CMakeLists.txt's `if(TARGET ps_core)
# add_subdirectory(bridges/physics_sim) endif()` guard: physics_sim's own
# ps_core must already have been add_subdirectory()-ed by the parent build
# (G2M_PS_SOURCE_DIR, see the root CMakeLists.txt's own comment) before this
# directory is ever processed.
add_library(g2m_ps_bridge STATIC
  src/version.cpp
  src/g2m_terrain_source.cpp
)
target_include_directories(g2m_ps_bridge PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(g2m_ps_bridge
  PUBLIC  ps_core g2m_phys
  PRIVATE g2m_warnings g2m_options
)

if(G2M_BUILD_TESTS)
  add_subdirectory(tests)
endif()
