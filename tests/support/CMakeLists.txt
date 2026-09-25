# tests/support - test-only helpers, never linked into g2m_core:
#   g2m_headless_env      Windows headless CRT setup (also used by tools)
#   g2m_test_catch_main   Catch2 main installing it (every Catch2 executable)
#   splitmix64.h          header-only, cross-platform-identical PRNG

add_library(g2m_headless_env STATIC headless_env.cpp)
target_include_directories(g2m_headless_env PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g2m_headless_env PRIVATE g2m_warnings g2m_options)

add_library(g2m_test_catch_main STATIC catch_main.cpp)
target_link_libraries(g2m_test_catch_main
  PUBLIC  Catch2::Catch2 g2m_headless_env
  PRIVATE g2m_warnings g2m_options
)
