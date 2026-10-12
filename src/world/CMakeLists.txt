# g2m_world (terrain migration plan, vault G2M-020): engine-neutral terrain
# service pieces moved from racing_game. Phase 1: tile decode/fetch-cache
# helpers, the two level-0 height samplers (bilinear, Jolt-diagonal exact),
# the chunk road-class glue and the SurfaceKind/LandClass id tables. No
# physics_sim, Jolt or Godot type anywhere in include/g2m/world, and the
# target never links physics_sim. Built UNCONDITIONALLY (like g2m_phys), so
# every CI leg exercises it.
set(G2M_WORLD_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/tile_fetch.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/height_sampler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/chunk_road_classes.cpp
  CACHE INTERNAL "g2m_world sources"
)

add_library(g2m_world STATIC ${G2M_WORLD_SOURCES})
target_include_directories(g2m_world PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(g2m_world
  PUBLIC  g2m_client g2m_mesh g2m_phys
  PRIVATE g2m_warnings g2m_options
)
