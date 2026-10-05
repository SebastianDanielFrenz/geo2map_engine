# g2m_layer_tests: Catch2 v3 over g2m_layers (container, raw encodings, OSM
# clip, release ids/manifests, packs, layer registry, regions, terrain
# derivers, OSM road extraction/rasterisation). Topic tags: [layer],
# [container], [elev], [landcover], [osm], [release], [pack], [region],
# [terrain]; tags become CTest labels.
#
# test_derive_terrain.cpp (PLAN.md G2.1) additionally needs g2m_decode/
# g2m_import (DGM1 fixture import: dgm1::Catalog, import_dem, resolve_region)
# and g2m_server (Server/TileStore, to exercise the elev.base -> terrain.height
# resolver chain) - no other test in this binary does, but pulling all three
# in here is simpler than a second executable just for one test file (same
# call tests/unit/CMakeLists.txt makes for its own test_import_e2e.cpp).
# test_osm_roads.cpp (PLAN.md 6.5) reuses g2m_decode's read_pbf directly, to
# load testdata/fixtures/osm/frankfurt_hoechst.osm.pbf for its own fixture
# test - already linked here for test_derive_terrain.cpp's sake.
# test_osm_paint.cpp (G2.5a-grip G-a) needs alloc_counter.cpp's own global
# operator new/delete override (its "allocation-free" check) - same
# duplicated-per-executable reasoning as tests/phys/CMakeLists.txt's own
# g2m_phys_tests.
add_executable(g2m_layer_tests
  test_container.cpp
  test_raster_encodings.cpp
  test_osm.cpp
  test_osm_roads.cpp
  test_road_geom.cpp
  test_road_context.cpp
  test_road_rules.cpp
  test_road_profile.cpp
  test_road_reference.cpp
  test_vertical_profile.cpp
  test_road_dem_sampler.cpp
  test_road_dem_dependencies.cpp
  test_road_attributes.cpp
  test_road_graph.cpp
  test_road_graph_tile.cpp
  test_osm_paint.cpp
  test_release.cpp
  test_pack.cpp
  test_region.cpp
  test_derive_terrain.cpp
  test_terrain_pyramid.cpp
  alloc_counter.cpp
)
target_link_libraries(g2m_layer_tests PRIVATE
  g2m_layers g2m_decode g2m_import g2m_server
  g2m_test_catch_main g2m_warnings g2m_options
  g2m_fixture_release
  libdeflate::libdeflate_static
)
target_compile_definitions(g2m_layer_tests PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(Catch)
catch_discover_tests(g2m_layer_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
