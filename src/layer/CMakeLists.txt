# g2m_layers (PLAN.md 2.3): layer model, the g2m.tile/1 container, the raw
# source encodings (src.elev, src.landcover, src.osm), the OSM tile clip,
# release ids/manifests and tile packs. zstd and nlohmann_json are PRIVATE:
# no third-party type appears in include/g2m/layer.
set(G2M_LAYERS_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/layer_id.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/zstd_codec.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tile_container.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src_elev.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src_landcover.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/osm_data.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/src_osm.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/osm_clip.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/osm_clip_index.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/release.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/region.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/pack.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/terrain_layers.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/height_tile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/derive_terrain.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/osm_roads.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_geom_tile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/derive_road_geom.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_profile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_reference.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vertical_profile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_dem_sampler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_dem_dependencies.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_attributes.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_graph.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_graph_tile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_rules.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_cross_section.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/derive_road_graph.cpp
  CACHE INTERNAL "g2m_layers sources (also compiled into the fuzz-instrumented copy, tests/fuzz_formats)"
)

# data/road_rules/de.json is embedded as a byte array (street fidelity P1): the
# library has no runtime data directory. The generated include lives in the build
# tree; the fuzz-instrumented copy of these sources (tests/fuzz_formats) adds the
# same directory. Editing the JSON re-runs configure.
set(G2M_ROAD_RULES_DE_JSON ${PROJECT_SOURCE_DIR}/data/road_rules/de.json)
set(G2M_GENERATED_INCLUDE_DIR ${PROJECT_BINARY_DIR}/generated/g2m_layers CACHE INTERNAL "generated includes of g2m_layers")
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${G2M_ROAD_RULES_DE_JSON})
file(READ ${G2M_ROAD_RULES_DE_JSON} _g2m_rules_hex HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _g2m_rules_bytes "${_g2m_rules_hex}")
file(CONFIGURE OUTPUT ${G2M_GENERATED_INCLUDE_DIR}/road_rules_de.inc
  CONTENT "const unsigned char kRoadRulesDe[] = {${_g2m_rules_bytes}};
" @ONLY)

add_library(g2m_layers STATIC ${G2M_LAYERS_SOURCES})
target_include_directories(g2m_layers PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(g2m_layers PRIVATE ${G2M_GENERATED_INCLUDE_DIR})
target_link_libraries(g2m_layers
  PUBLIC  g2m_core
  PRIVATE g2m_dep_zstd g2m_dep_json g2m_warnings g2m_options
)
