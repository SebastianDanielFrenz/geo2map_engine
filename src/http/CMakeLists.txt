# g2m_http (G6, PLAN.md 2.3/3.7): protocol v1 over HTTP - the wire mapping
# (http_wire.cpp, pure; also compiled into the fuzz build), the client
# HttpTransport and the HttpServer front end. cpp-httplib is PRIVATE: no
# httplib type appears in include/g2m/http.
add_library(g2m_http STATIC
  http_wire.cpp
  http_transport.cpp
  http_server.cpp
)
target_include_directories(g2m_http PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(g2m_http
  PUBLIC  g2m_server
  PRIVATE g2m_dep_httplib g2m_warnings g2m_options
)
