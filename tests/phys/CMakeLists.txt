# g2m_phys_tests: Catch2 v3, g2m_phys (PLAN.md R2.2 section 1a). Every test
# carries the [phys] tag plus its own topic tag ([physics_grid],
# [resident_heights], [fill], [loader], [streamer]); tags become CTest labels. alloc_counter.cpp
# overrides the GLOBAL operator new/delete for this one executable only (see
# alloc_counter.h) - kept out of g2m_unit_tests deliberately, so no other
# test binary's allocations are ever counted (same reasoning as
# tests/mesh/CMakeLists.txt's own g2m_mesh_tests).
add_executable(g2m_phys_tests
  test_physics_grid.cpp
  test_resident_heights.cpp
  test_fill_physics_heights.cpp
  test_height_tile_loader.cpp
  test_physics_streamer.cpp
  alloc_counter.cpp
)
target_link_libraries(g2m_phys_tests PRIVATE g2m_phys g2m_test_catch_main g2m_warnings g2m_options)

include(Catch)
catch_discover_tests(g2m_phys_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
