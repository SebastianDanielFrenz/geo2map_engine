# g2m_unit_tests: Catch2 v3. Every test carries a topic tag ([math],
# [geo], [tile], [raster], [hash], [decode], [import]); tags become CTest
# labels. test_import_resolve.cpp reads testdata/fixtures/ (committed
# fixture PBF, G1c I2) via G2M_SOURCE_DIR below.
add_executable(g2m_unit_tests
  test_det_math.cpp
  test_tm_reference.cpp
  test_geo_frames.cpp
  test_tile_raster_hash.cpp
  test_decode_zip.cpp
  test_decode_tiff.cpp
  test_decode_osm.cpp
  test_decode_real_data.cpp
  test_geom_exact.cpp
  test_ring_assembly.cpp
  test_import_resolve.cpp
  test_ordered_parallel.cpp
  test_decode_nested_zip.cpp
  test_import_dem.cpp
)
target_link_libraries(g2m_unit_tests PRIVATE g2m_core g2m_decode g2m_import g2m_test_catch_main g2m_warnings
                                              g2m_options libdeflate::libdeflate_static)
target_compile_definitions(g2m_unit_tests PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(Catch)
catch_discover_tests(g2m_unit_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
