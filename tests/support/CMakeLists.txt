# tests/support - test-only helpers, never linked into g2m_core:
#   g2m_headless_env      Windows headless CRT setup (also used by tools)
#   g2m_test_catch_main   Catch2 main installing it (every Catch2 executable)
#   splitmix64.h          header-only, cross-platform-identical PRNG
#   zip_builder.h/tiff_builder.h   header-only synthetic-fixture writers
#                                  (promoted from tests/unit, G1c I2)
#   fixture_cut/          g2m_fixture_cut: test-only fixture-cutting CLI
#                                  (G1c I2, D20 - keeps libosmium's PBF
#                                  writer out of product code)
#   g2m_fixture_release   fixture_release.{h,cpp}: the import -> store ->
#                                  local release -> Server pipeline, factored
#                                  out of tests/layer/test_derive_terrain.cpp
#                                  (PLAN.md R2.2 section 1c) so the physics
#                                  bridge tests (bridges/physics_sim/tests,
#                                  G7+) can reuse the same PKT DGM1 fixture
#                                  release without duplicating the wiring.

add_library(g2m_headless_env STATIC headless_env.cpp)
target_include_directories(g2m_headless_env PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g2m_headless_env PRIVATE g2m_warnings g2m_options)

add_library(g2m_test_catch_main STATIC catch_main.cpp)
target_link_libraries(g2m_test_catch_main
  PUBLIC  Catch2::Catch2 g2m_headless_env
  PRIVATE g2m_warnings g2m_options
)

add_library(g2m_fixture_release STATIC fixture_release.cpp)
target_include_directories(g2m_fixture_release PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g2m_fixture_release
  PUBLIC  g2m_layers g2m_decode g2m_import g2m_server g2m_client Catch2::Catch2
  PRIVATE g2m_warnings g2m_options
)
# Same fixture-path convention as g2m_layer_tests' own G2M_SOURCE_DIR
# (tests/layer/CMakeLists.txt): testdata/fixtures lives under the repo root.
target_compile_definitions(g2m_fixture_release PUBLIC G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

add_subdirectory(fixture_cut)
