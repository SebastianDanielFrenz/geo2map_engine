# g2m_layer_tests: Catch2 v3 over g2m_layers (container, raw encodings, OSM
# clip, release ids/manifests, packs, layer registry, regions, terrain
# derivers). Topic tags: [layer], [container], [elev], [landcover], [osm],
# [release], [pack], [region], [terrain]; tags become CTest labels.
#
# test_derive_terrain.cpp (PLAN.md G2.1) additionally needs g2m_decode/
# g2m_import (DGM1 fixture import: dgm1::Catalog, import_dem, resolve_region)
# and g2m_server (Server/TileStore, to exercise the elev.base -> terrain.height
# resolver chain) - no other test in this binary does, but pulling all three
# in here is simpler than a second executable just for one test file (same
# call tests/unit/CMakeLists.txt makes for its own test_import_e2e.cpp).
add_executable(g2m_layer_tests
  test_container.cpp
  test_raster_encodings.cpp
  test_osm.cpp
  test_release.cpp
  test_pack.cpp
  test_region.cpp
  test_derive_terrain.cpp
  test_terrain_pyramid.cpp
)
target_link_libraries(g2m_layer_tests PRIVATE
  g2m_layers g2m_decode g2m_import g2m_server
  g2m_test_catch_main g2m_warnings g2m_options
  libdeflate::libdeflate_static
)
target_compile_definitions(g2m_layer_tests PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(Catch)
catch_discover_tests(g2m_layer_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
