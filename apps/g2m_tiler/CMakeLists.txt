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

# Read-only listing of OSM ways near a UTM point (grade-separation inspection).
add_executable(g2m_osm_near ../../tools/osm_near/main.cpp)
target_link_libraries(g2m_osm_near PRIVATE g2m_server g2m_warnings g2m_options)

# Read-only inspection of the served road geometry (profile vs DEM vs carved terrain) at a spot.
add_executable(g2m_spot_dump ../../tools/spot_dump/main.cpp)
target_link_libraries(g2m_spot_dump PRIVATE g2m_server g2m_warnings g2m_options)

add_executable(g2m_restriction_probe ../../tools/restriction_probe/main.cpp)
target_link_libraries(g2m_restriction_probe PRIVATE g2m_server g2m_warnings g2m_options)

# Read-only censuses over the road graphs of a block of tiles (tools/road_census: turn arrows, reference-line bow).
add_executable(g2m_road_census ../../tools/road_census/main.cpp)
target_link_libraries(g2m_road_census PRIVATE g2m_world g2m_server g2m_warnings g2m_options)

# Railways R1/R2 census over an existing immutable source store (docs/formats/rail_geom.md).
add_executable(g2m_rail_census ../../tools/rail_census/main.cpp)
target_link_libraries(g2m_rail_census PRIVATE g2m_server g2m_warnings g2m_options)

# Terrain-above-ribbon census (vault G2M-025): drapes the tile ribbons both ways and probes them against the terrain mesh.
add_executable(g2m_ribbon_census ../../tools/ribbon_census/main.cpp)
target_link_libraries(g2m_ribbon_census PRIVATE g2m_server g2m_world g2m_warnings g2m_options)
# Cross-section dump of OSM ways and a lane-change census (vault G2M-025).
add_executable(g2m_xs_probe ../../tools/xs_probe/main.cpp)
target_link_libraries(g2m_xs_probe PRIVATE g2m_server g2m_warnings g2m_options)

add_executable(g2m_geographic_dem ../../tools/geographic_dem/main.cpp)
target_link_libraries(g2m_geographic_dem PRIVATE g2m_decode g2m_server g2m_layers g2m_warnings g2m_options)

add_executable(g2m_elevation_prepare ../../tools/elevation_prepare/main.cpp)
target_link_libraries(g2m_elevation_prepare PRIVATE g2m_decode g2m_world g2m_dep_json g2m_warnings g2m_options)
