# g2m_client (PLAN.md 2.3, 7.2): G1 part only - the tile cache (on
# g2m_server's TileStore) and the in-process transport. The streamer and
# the rest of g2m::Client arrive in G2.
add_library(g2m_client STATIC
  cache.cpp
)
target_include_directories(g2m_client PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(g2m_client
  PUBLIC  g2m_server
  PRIVATE g2m_warnings g2m_options
)
