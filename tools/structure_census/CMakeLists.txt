# g2m_structure_census (terrain migration phase 7a, docs/structure_classification.md): read-only census of the
# tunnel stretches and the roads under bridges of a source store. The classification rule is the served one
# (src/layer/structure_classify.cpp, phase 7a-2); the tool links it from g2m_layers, unchanged in behaviour.
if(G2M_BUILD_APPS)
  add_executable(g2m_structure_census main.cpp)
  target_link_libraries(g2m_structure_census PRIVATE g2m_layers g2m_server g2m_warnings g2m_options)
endif()
