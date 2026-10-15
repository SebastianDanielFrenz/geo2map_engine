# g2m_http_tests (G6): the HTTP binding of protocol v1. Tags [http] plus
# [wire] (pure mapping, no socket), [roundtrip], [malformed], [concurrency],
# [status] (real sockets on 127.0.0.1, ephemeral ports) and [pack] (the pack
# directory format of the /hashes body). Links cpp-httplib directly for the
# raw-request tests.
add_executable(g2m_http_tests
  test_http_wire.cpp
  test_http_server.cpp
)
target_include_directories(g2m_http_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/tests/server
                                                  ${PROJECT_SOURCE_DIR}/tests/support)
target_link_libraries(g2m_http_tests PRIVATE
  g2m_http g2m_client g2m_dep_httplib g2m_test_catch_main g2m_warnings g2m_options
)
include(Catch)
catch_discover_tests(g2m_http_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
