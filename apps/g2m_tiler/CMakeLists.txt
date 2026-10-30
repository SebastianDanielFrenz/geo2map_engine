# apps/g2m_tiler (PLAN.md G1c I2): the importer CLI. Gated independently of
# G2M_BUILD_TESTS by G2M_BUILD_APPS (top-level CMakeLists.txt) - must build
# with tests off, so it links only g2m_import plus its own tiny rss.cpp
# (no tests/support g2m_headless_env dependency).
add_executable(g2m_tiler
  main.cpp
  rss.cpp
)
target_link_libraries(g2m_tiler PRIVATE g2m_import g2m_warnings g2m_options)

if(WIN32)
  # rss.cpp's K32GetProcessMemoryInfo resolves directly from kernel32.dll
  # (available since Vista) - no psapi.lib to link.
endif()

# G3 geometry diagnostics over an existing immutable source store.
add_executable(g2m_road_probe ../../tools/road_probe/main.cpp)
target_link_libraries(g2m_road_probe PRIVATE g2m_server g2m_warnings g2m_options)

# Farm field census and picture over a baked g2m.fields.geom store (docs/pipeline/fields.md).
add_executable(g2m_fields_probe ../../tools/fields_probe/main.cpp)
target_link_libraries(g2m_fields_probe PRIVATE g2m_server g2m_warnings g2m_options)
