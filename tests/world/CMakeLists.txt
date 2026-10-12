# g2m_world_tests: Catch2 v3, g2m_world (terrain migration plan phase 1, vault
# G2M-020). Every test carries the [world] tag plus its own topic tag
# ([tile_fetch], [height_sampler], [chunk_road_classes], [surface_id_table], [terrain_service]);
# tags become CTest labels. Pinned values were produced by racing_game's own
# code (see world_test_data.h and the comment above each pin).
add_executable(g2m_world_tests
  test_tile_fetch.cpp
  test_height_sampler.cpp
  test_chunk_road_classes.cpp
  test_surface_id_table.cpp
  test_terrain_service.cpp
)
target_link_libraries(g2m_world_tests PRIVATE g2m_world g2m_layers g2m_server g2m_test_catch_main g2m_warnings g2m_options)

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
