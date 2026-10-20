# g2m_structure_census (terrain migration phase 7a, docs/structure_classification.md): read-only census of the
# tunnel stretches and the roads under bridges of a source store. The classification rule lives in its own library
# so it is unit-tested (tests/unit/test_structure_classify.cpp) and later reproduced by the served implementation.
add_library(g2m_structure_census_lib STATIC structure_classify.cpp)
target_include_directories(g2m_structure_census_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g2m_structure_census_lib PRIVATE g2m_warnings g2m_options)

if(G2M_BUILD_APPS)
  add_executable(g2m_structure_census main.cpp)
  target_link_libraries(g2m_structure_census PRIVATE g2m_structure_census_lib g2m_server g2m_warnings g2m_options)
endif()
