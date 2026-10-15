# g2m_world (terrain migration plan, vault G2M-020): engine-neutral terrain
# service pieces moved from racing_game. Phase 1: tile decode/fetch-cache
# helpers, the two level-0 height samplers (bilinear, Jolt-diagonal exact),
# the chunk road-class glue and the SurfaceKind/LandClass id tables. Phase 2:
# TerrainService (terrain_service.cpp: stores, release, in-process server,
# memory cache, tickets, warm-up, PhysicsFetch) on its own worker pool
# (job_scheduler.cpp: priority queue, per-job dedup, cancellation). Phase 3a:
# structure decks (road_structures.cpp) and road ribbons (road_ribbons.cpp). Phase 3b:
# buildings (buildings.cpp), the LOD view streamer (view_streamer.cpp) and the
# ring streamer policy (ring_streamer.cpp). No
# physics_sim, Jolt or Godot type anywhere in include/g2m/world, and the
# target never links physics_sim. Built UNCONDITIONALLY (like g2m_phys), so
# every CI leg exercises it.
find_package(Threads REQUIRED)
set(G2M_WORLD_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/tile_fetch.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/height_sampler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/chunk_road_classes.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/job_scheduler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/terrain_service.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_structures.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_ribbons.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/buildings.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/view_streamer.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/ring_streamer.cpp
  CACHE INTERNAL "g2m_world sources"
)

add_library(g2m_world STATIC ${G2M_WORLD_SOURCES})
target_include_directories(g2m_world PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(g2m_world
  PUBLIC  g2m_client g2m_mesh g2m_phys g2m_server
  PRIVATE g2m_warnings g2m_options Threads::Threads
)
