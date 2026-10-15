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
  ${CMAKE_CURRENT_SOURCE_DIR}/road_completion.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_reference.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/vertical_profile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_dem_sampler.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_dem_dependencies.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_attributes.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_graph.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_graph_tile.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_rules.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/carve_params.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/landcover_classifier.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_cross_section.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_cross_section_join.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_cross_section_api.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_envelope.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_surface.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_junction.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_furniture.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_underpass.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/road_features.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/terrain_carve.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/terrain_class.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/derive_road_graph.cpp
  CACHE INTERNAL "g2m_layers sources (also compiled into the fuzz-instrumented copy, tests/fuzz_formats)"
)

# data/road_rules/de.json is embedded as a byte array (street fidelity P1): the
# library has no runtime data directory. The generated include lives in the build
# tree; the fuzz-instrumented copy of these sources (tests/fuzz_formats) adds the
# same directory. Editing the JSON re-runs configure.
#
# Street fidelity P2 embeds two more data files the same way: data/carve/carve_default.json
# (carve_default.inc, hashed only into the carved/class layer configs) and
# data/classifiers/esa_worldcover_v200.json (esa_worldcover_v200.inc).
set(G2M_GENERATED_INCLUDE_DIR ${PROJECT_BINARY_DIR}/generated/g2m_layers CACHE INTERNAL "generated includes of g2m_layers")
function(g2m_embed_json json_file inc_name array_name)
  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${json_file})
  file(READ ${json_file} _hex HEX)
  string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," _bytes "${_hex}")
  file(CONFIGURE OUTPUT ${G2M_GENERATED_INCLUDE_DIR}/${inc_name}
    CONTENT "const unsigned char ${array_name}[] = {${_bytes}};
" @ONLY)
endfunction()
g2m_embed_json(${PROJECT_SOURCE_DIR}/data/road_rules/de.json road_rules_de.inc kRoadRulesDe)
g2m_embed_json(${PROJECT_SOURCE_DIR}/data/carve/carve_default.json carve_default.inc kCarveDefault)
g2m_embed_json(${PROJECT_SOURCE_DIR}/data/classifiers/esa_worldcover_v200.json esa_worldcover_v200.inc kEsaWorldcoverV200)

add_library(g2m_layers STATIC ${G2M_LAYERS_SOURCES})
target_include_directories(g2m_layers PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_include_directories(g2m_layers PRIVATE ${G2M_GENERATED_INCLUDE_DIR})
target_link_libraries(g2m_layers
  PUBLIC  g2m_core
  PRIVATE g2m_dep_zstd g2m_dep_json g2m_warnings g2m_options Threads::Threads
)
