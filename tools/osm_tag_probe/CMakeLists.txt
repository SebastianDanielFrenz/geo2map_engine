# g2m_osm_tag_probe (street-level fidelity plan, phase P0): per-region OSM tag
# coverage report. A diagnostic tool, not part of any served layer; it lives
# in its own library so the aggregation and scan logic can be unit-tested
# (tests/unit/test_osm_tag_probe.cpp). Added from the top-level CMakeLists.txt
# inside the G2M_BUILD_IMPORT block, after src/decode and src/import.
add_library(g2m_osm_tag_probe_lib STATIC tag_probe.cpp scan.cpp)
target_include_directories(g2m_osm_tag_probe_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
# g2m_decode is PUBLIC: tag_probe.h exposes g2m::decode::osm::Tag.
target_link_libraries(g2m_osm_tag_probe_lib PUBLIC g2m_core g2m_layers g2m_decode g2m_import
                                            PRIVATE g2m_warnings g2m_options)

if(G2M_BUILD_APPS)
  # rss.cpp (peak memory for the stderr summary) is the tiler's own tiny helper.
  add_executable(g2m_osm_tag_probe main.cpp ${PROJECT_SOURCE_DIR}/apps/g2m_tiler/rss.cpp)
  target_include_directories(g2m_osm_tag_probe PRIVATE ${PROJECT_SOURCE_DIR}/apps/g2m_tiler)
  target_link_libraries(g2m_osm_tag_probe PRIVATE g2m_osm_tag_probe_lib g2m_warnings g2m_options)
endif()
