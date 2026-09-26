# g2m_mesh_tests: Catch2 v3, g2m_mesh (PLAN.md G2.3). alloc_counter.cpp
# overrides the GLOBAL operator new/delete for this one executable only (see
# alloc_counter.h) - kept out of g2m_unit_tests deliberately, so no other
# test binary's allocations are ever counted.
add_executable(g2m_mesh_tests
  test_terrain_chunk.cpp
  test_lod_select.cpp
  alloc_counter.cpp
)
target_link_libraries(g2m_mesh_tests PRIVATE g2m_mesh g2m_test_catch_main g2m_warnings g2m_options)

include(Catch)
catch_discover_tests(g2m_mesh_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
