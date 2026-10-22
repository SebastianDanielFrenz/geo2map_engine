# g2m_unit_tests: Catch2 v3. Every test carries a topic tag ([math],
# [geo], [tile], [raster], [hash], [decode], [import]); tags become CTest
# labels. test_import_resolve.cpp reads testdata/fixtures/ (committed
# fixture PBF, G1c I2) via G2M_SOURCE_DIR below. test_import_e2e.cpp (G1c
# I6 item E) additionally needs g2m_server/g2m_client (TileStore, Server,
# ClientCache, the in-process transport) - no other test in this binary
# does, but pulling both in here is simpler than a second executable just
# for one test file.
add_executable(g2m_unit_tests
  test_det_math.cpp
  test_tm_reference.cpp
  test_geo_frames.cpp
  test_tile_raster_hash.cpp
  test_decode_zip.cpp
  test_decode_tiff.cpp
  test_geographic_dem.cpp
  test_terrarium_dem.cpp
  test_decode_osm.cpp
  test_decode_real_data.cpp
  test_clothoid.cpp
  test_geom_exact.cpp
  test_ring_assembly.cpp
  test_import_resolve.cpp
  test_ordered_parallel.cpp
  test_decode_nested_zip.cpp
  test_import_dem.cpp
  test_corridor_import.cpp
  test_dem_km_span.cpp
  test_osm_clip_index.cpp
  test_import_osm.cpp
  test_import_landcover.cpp
  test_fixtures.cpp
  test_import_e2e.cpp
  test_tiler_cli_args.cpp
  test_bake.cpp
  test_osm_tag_probe.cpp
)
target_link_libraries(g2m_unit_tests PRIVATE g2m_core g2m_decode g2m_import g2m_server g2m_client
                                              g2m_test_catch_main g2m_warnings g2m_options
                                              libdeflate::libdeflate_static g2m_osm_tag_probe_lib)
target_compile_definitions(g2m_unit_tests PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
# test_tiler_cli_args.cpp includes apps/g2m_tiler/cli_args.h directly (it is
# header-only precisely so this test needs no new library target, G1c I7).
target_include_directories(g2m_unit_tests PRIVATE ${PROJECT_SOURCE_DIR}/apps/g2m_tiler)

include(Catch)
catch_discover_tests(g2m_unit_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
