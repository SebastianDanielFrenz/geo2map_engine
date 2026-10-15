# apps/g2m_server (G6, PLAN.md 3.1 "sidecar"): serves one local release of a
# source store over HTTP on localhost. Offline by construction: the server
# gets no fetcher and no network upstream.
add_executable(g2m_server_app main.cpp)
set_target_properties(g2m_server_app PROPERTIES OUTPUT_NAME g2m_server)
target_link_libraries(g2m_server_app PRIVATE g2m_http g2m_client g2m_warnings g2m_options)
# Shares g2m_tiler's header-only argv parser.
target_include_directories(g2m_server_app PRIVATE ${PROJECT_SOURCE_DIR}/apps/g2m_tiler)
