# g2m_phys_tests: Catch2 v3, g2m_phys (PLAN.md R2.2 section 1a). Every test
# carries the [phys] tag plus its own topic tag ([physics_grid],
# [resident_heights], [fill]); tags become CTest labels.
add_executable(g2m_phys_tests
  test_physics_grid.cpp
)
target_link_libraries(g2m_phys_tests PRIVATE g2m_phys g2m_test_catch_main g2m_warnings g2m_options)

include(Catch)
catch_discover_tests(g2m_phys_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
