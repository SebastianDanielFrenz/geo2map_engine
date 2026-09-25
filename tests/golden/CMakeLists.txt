# tests/golden - G0 golden hashes (PLAN.md 2.4):
#   g2m_golden        the hash computation (golden.h/.cpp), test-only
#   g2m_golden_dump   prints them; tools/ci.ps1 compares its output across legs
#   g2m_golden_tests  Catch2 check against testdata/golden/g0_golden.txt
add_library(g2m_golden STATIC golden.cpp)
target_include_directories(g2m_golden PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g2m_golden PUBLIC g2m_core g2m_headless_env PRIVATE g2m_warnings g2m_options)

add_executable(g2m_golden_dump dump_main.cpp)
target_link_libraries(g2m_golden_dump PRIVATE g2m_golden g2m_headless_env g2m_warnings g2m_options)

add_executable(g2m_golden_tests test_golden.cpp)
target_link_libraries(g2m_golden_tests PRIVATE g2m_golden g2m_test_catch_main g2m_warnings g2m_options)
target_compile_definitions(g2m_golden_tests PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(Catch)
catch_discover_tests(g2m_golden_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
