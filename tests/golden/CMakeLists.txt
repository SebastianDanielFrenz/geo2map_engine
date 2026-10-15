# tests/golden - golden hashes (PLAN.md 2.4):
#   g2m_golden        the hash computation (golden.h, golden.cpp = G0,
#                     golden_g1.cpp = G1, golden_g1c.cpp = G1c,
#                     golden_g2.cpp = G2, PLAN.md G2.1, golden_g2roads.cpp =
#                     G2roads, PLAN.md 6.5, golden_g2paint.cpp = G2paint,
#                     G2.5a-grip commit G-a), test-only
#   g2m_golden_dump   prints one set; tools/ci.ps1 compares its output across legs
#   g2m_golden_tests  Catch2 check against testdata/golden/g0_golden.txt, g1_golden.txt, g1c_golden.txt, g2_golden.txt, g2roads_golden.txt, g2paint_golden.txt, g3xs_golden.txt, g3j_golden.txt, g3d_golden.txt, g3c_golden.txt
add_library(g2m_golden STATIC golden.cpp golden_g1.cpp golden_g1c.cpp golden_g2.cpp golden_g2mesh.cpp golden_g2roads.cpp golden_g2paint.cpp golden_g3xs.cpp golden_g3j.cpp golden_g3d.cpp golden_g3c.cpp)
target_include_directories(g2m_golden PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# golden_g3xs.cpp, golden_g3j.cpp and golden_g3d.cpp share the road fixture helpers of the layer tests (header-only).
target_include_directories(g2m_golden PRIVATE ${PROJECT_SOURCE_DIR}/tests/support)
target_link_libraries(g2m_golden PUBLIC g2m_core g2m_layers g2m_server g2m_import g2m_mesh g2m_headless_env PRIVATE g2m_warnings g2m_options)
# golden_g1c.cpp/golden_g2.cpp read testdata/fixtures/** directly (unlike
# golden.cpp/golden_g1.cpp, which only use synthetic SplitMix64 inputs).
target_compile_definitions(g2m_golden PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_executable(g2m_golden_dump dump_main.cpp)
target_link_libraries(g2m_golden_dump PRIVATE g2m_golden g2m_headless_env g2m_warnings g2m_options)

add_executable(g2m_golden_tests test_golden.cpp)
target_link_libraries(g2m_golden_tests PRIVATE g2m_golden g2m_test_catch_main g2m_warnings g2m_options)
target_compile_definitions(g2m_golden_tests PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(Catch)
catch_discover_tests(g2m_golden_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
