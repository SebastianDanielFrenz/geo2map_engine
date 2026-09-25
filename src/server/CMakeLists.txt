# g2m_server (PLAN.md 2.3, 3.1): the tile stores (SQLite + content-addressed
# blob files), protocol v1 request/response objects and the in-process
# g2m::Server (router, resolver, upstream chaining). SQLite and
# nlohmann_json are PRIVATE: no third-party type appears in include/g2m/server.
add_library(g2m_server STATIC
  tile_store.cpp
  protocol.cpp
  server.cpp
)
target_include_directories(g2m_server PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(g2m_server
  PUBLIC  g2m_layers
  PRIVATE g2m_dep_sqlite3 g2m_dep_json g2m_warnings g2m_options
)
