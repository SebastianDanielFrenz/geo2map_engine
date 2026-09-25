# g2m_server_tests: Catch2 v3 over g2m_server and g2m_client (tile stores,
# protocol objects, the resolver, the client cache). Topic tags: [server],
# [store], [protocol], [offline], [pack], [client]; tags become CTest
# labels. Links the SQLite amalgamation target directly for the tests that
# damage a database row on purpose.
add_executable(g2m_server_tests
  test_tile_store.cpp
  test_protocol.cpp
  test_server.cpp
  test_client_cache.cpp
)
target_link_libraries(g2m_server_tests PRIVATE
  g2m_server g2m_client g2m_dep_sqlite3 g2m_test_catch_main g2m_warnings g2m_options
)
target_compile_definitions(g2m_server_tests PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(Catch)
catch_discover_tests(g2m_server_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
