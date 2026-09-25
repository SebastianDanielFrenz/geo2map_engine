# g2m_layer_tests: Catch2 v3 over g2m_layers (container, raw encodings, OSM
# clip, release ids/manifests, packs, layer registry, regions). Topic tags:
# [layer], [container], [elev], [landcover], [osm], [release], [pack],
# [region]; tags become CTest labels.
add_executable(g2m_layer_tests
  test_container.cpp
  test_raster_encodings.cpp
  test_osm.cpp
  test_release.cpp
  test_pack.cpp
  test_region.cpp
)
target_link_libraries(g2m_layer_tests PRIVATE g2m_layers g2m_test_catch_main g2m_warnings g2m_options)
target_compile_definitions(g2m_layer_tests PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(Catch)
catch_discover_tests(g2m_layer_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
