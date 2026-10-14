# g2m_route_tests: Catch2 v3 over g2m_route (G4, docs/routing_design.md).
# Tags: [route] plus [restriction], [oneway], [real-data], [bench] (hidden,
# timing print only, never asserted). The brute-force Dijkstra the search is
# compared against lives in route_reference.h (test-only).
add_executable(g2m_route_tests
  test_turn_restrictions.cpp
  test_route_search.cpp
  test_route_fixture.cpp
  test_route_slice2.cpp
  test_route_guidance.cpp
)
target_include_directories(g2m_route_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/tests/support)
target_link_libraries(g2m_route_tests PRIVATE g2m_route g2m_layers g2m_decode g2m_test_catch_main g2m_warnings g2m_options)
target_compile_definitions(g2m_route_tests PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(Catch)
catch_discover_tests(g2m_route_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
