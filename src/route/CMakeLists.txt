# g2m_route (G4, docs/routing_design.md): in-memory routable car graph built
# from roads.graph data plus OSM turn restrictions, and an edge-based A*
# search. Engine-neutral, exception-free (Result<T>), deterministic integer
# costs. Depends only on g2m_layers (RoadGraph, OsmData, RoadStyle).
set(G2M_ROUTE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/turn_restriction.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/routing_graph.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/route_search.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/region_graph.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/route_guidance.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/route_map.cpp
  CACHE INTERNAL "g2m_route sources"
)
add_library(g2m_route STATIC ${G2M_ROUTE_SOURCES})
target_include_directories(g2m_route PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(g2m_route PUBLIC g2m_layers PRIVATE g2m_warnings g2m_options)
