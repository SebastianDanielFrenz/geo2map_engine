# g2m_mesh (PLAN.md G2.3): engine-neutral terrain render chunks and LOD
# selection. Links g2m_layers (for HeightTile) and, transitively, g2m_core;
# no third-party type and no rendering-engine type appears in include/g2m/mesh.
set(G2M_MESH_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/terrain_chunk.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/lod_select.cpp
  CACHE INTERNAL "g2m_mesh sources"
)

add_library(g2m_mesh STATIC ${G2M_MESH_SOURCES})
target_include_directories(g2m_mesh PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(g2m_mesh
  PUBLIC  g2m_core g2m_layers
  PRIVATE g2m_warnings g2m_options
)
