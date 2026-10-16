# g2m_world_tests: Catch2 v3, g2m_world (terrain migration plan phase 1, vault
# G2M-020). Every test carries the [world] tag plus its own topic tag
# ([tile_fetch], [height_sampler], [chunk_road_classes], [surface_id_table], [terrain_service],
# [road_structures], [road_ribbons], [buildings], [view_streamer], [ring_streamer]);
# tags become CTest labels. Pinned values were produced by racing_game's own
# code (see world_test_data.h and the comment above each pin).
add_executable(g2m_world_tests
  test_tile_fetch.cpp
  test_height_sampler.cpp
  test_chunk_road_classes.cpp
  test_chunk_terrain_classes.cpp
  test_surface_id_table.cpp
  test_terrain_service.cpp
  test_road_meshes.cpp
  test_buildings.cpp
  test_view_streamer.cpp
  test_ring_streamer.cpp
)
target_link_libraries(g2m_world_tests PRIVATE g2m_world g2m_layers g2m_server g2m_decode g2m_test_catch_main g2m_warnings g2m_options)
# test_road_meshes.cpp reads the committed OSM fixtures and the shared road fixtures of tests/support.
target_include_directories(g2m_world_tests PRIVATE ${PROJECT_SOURCE_DIR}/tests/support)
target_compile_definitions(g2m_world_tests PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")

include(Catch)
catch_discover_tests(g2m_world_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)

# Reference generator (NOT part of any normal build, needs a racing_game
# checkout): the program that ran racing_game a8a1b13's code over the shared
# inputs of world_test_data.h to produce the values pinned in test_*.cpp.
#   bash tests/world/refgen/extract_game_ref.sh <racing_game> <dir>/game_ref.inc
#   cmake --preset release -DG2M_WORLD_REFGEN_DIR=<dir>
#         -DG2M_WORLD_REFGEN_GAME=<racing_game>
#   cmake --build out/build/release --target g2m_world_refgen
# <dir> holds game_ref.inc. road_classes.cpp is compiled from the checkout.
if(G2M_WORLD_REFGEN_DIR AND G2M_WORLD_REFGEN_GAME)
  add_executable(g2m_world_refgen refgen/refgen.cpp ${G2M_WORLD_REFGEN_GAME}/core/src/road_classes.cpp)
  target_include_directories(g2m_world_refgen PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
    ${G2M_WORLD_REFGEN_GAME}/core/include ${G2M_WORLD_REFGEN_DIR})
  target_link_libraries(g2m_world_refgen PRIVATE g2m_world g2m_headless_env g2m_options)
endif()

# Cold-store throughput of TerrainService vs the plain serial server (phase 2 measurement; NOT a CTest test, run by
# hand: see the header comment of bench_terrain_service.cpp).
add_executable(g2m_terrain_service_bench bench_terrain_service.cpp)
target_link_libraries(g2m_terrain_service_bench PRIVATE g2m_world g2m_layers g2m_server g2m_headless_env g2m_warnings g2m_options)

# Phase-3a mesh reference generator (NOT part of any normal build, needs a racing_game checkout): runs racing_game
# 945455a's own road_structures.cpp / road_visual.cpp over road_mesh_test_data.h's cases and prints road_mesh_pins.inc
# (2>/dev/null: the game code logs RG_STRUCTURE lines to stderr).
#   bash tests/world/refgen/extract_mesh_ref.sh <racing_game> <dir>
#   cmake --preset linux-release -DG2M_WORLD_MESH_REFGEN_DIR=<dir>
#   cmake --build <build> --target g2m_world_mesh_refgen && <build>/tests/world/g2m_world_mesh_refgen 2>/dev/null > tests/world/road_mesh_pins.inc
if(G2M_WORLD_MESH_REFGEN_DIR)
  add_executable(g2m_world_mesh_refgen refgen/refgen_meshes.cpp ${G2M_WORLD_MESH_REFGEN_DIR}/road_structures.cpp
    ${G2M_WORLD_MESH_REFGEN_DIR}/road_visual.cpp ${G2M_WORLD_MESH_REFGEN_DIR}/traffic_rank.cpp)
  target_include_directories(g2m_world_mesh_refgen PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/tests/support
    ${G2M_WORLD_MESH_REFGEN_DIR})
  target_compile_definitions(g2m_world_mesh_refgen PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  target_link_libraries(g2m_world_mesh_refgen PRIVATE g2m_world g2m_layers g2m_decode g2m_headless_env g2m_options)
endif()

# Phase-3b reference generator (NOT part of any normal build, needs a racing_game checkout): racing_game 5a3baed's
# view functions, TerrainViewStreamer, diff-order helpers and extract_buildings over view_test_data.h and
# building_test_data.h. Steps:
#   bash tests/world/refgen/extract_view_ref.sh <racing_game> <dir>
#   cmake --preset linux-release -DG2M_WORLD_VIEW_REFGEN_DIR=<dir>
#   cmake --build <build> --target g2m_world_view_refgen && <build>/tests/world/g2m_world_view_refgen > pins.txt
#   (split pins.txt at "// ---- building_pins.inc" into tests/world/view_pins.inc and tests/world/building_pins.inc)
if(G2M_WORLD_VIEW_REFGEN_DIR)
  add_executable(g2m_world_view_refgen refgen/refgen_view.cpp ${G2M_WORLD_VIEW_REFGEN_DIR}/world_terrain_view.cpp
    ${G2M_WORLD_VIEW_REFGEN_DIR}/terrain_view_streamer.cpp ${G2M_WORLD_VIEW_REFGEN_DIR}/terrain_stream_part.cpp
    ${G2M_WORLD_VIEW_REFGEN_DIR}/lod_settings_part.cpp ${G2M_WORLD_VIEW_REFGEN_DIR}/buildings.cpp)
  target_include_directories(g2m_world_view_refgen PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/tests/support
    ${G2M_WORLD_VIEW_REFGEN_DIR})
  target_compile_definitions(g2m_world_view_refgen PRIVATE G2M_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
  target_link_libraries(g2m_world_view_refgen PRIVATE g2m_world g2m_layers g2m_decode g2m_headless_env g2m_options)
endif()
