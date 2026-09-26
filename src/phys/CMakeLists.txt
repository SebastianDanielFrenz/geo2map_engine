# g2m_phys (PLAN.md R2.2 section 1a): ps-free physics-tile grid + resident
# height cache + fill_physics_heights, shared by g2m_ps_bridge (G6+) and any
# other physics_sim peer. No physics_sim/Jolt/Godot type appears anywhere in
# include/g2m/phys - this library never links physics_sim. Built
# UNCONDITIONALLY (not gated by G2M_BUILD_IMPORT), so every CI leg, including
# the sanitizer legs, exercises it.
set(G2M_PHYS_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/physics_grid.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/resident_heights.cpp
  CACHE INTERNAL "g2m_phys sources"
)

add_library(g2m_phys STATIC ${G2M_PHYS_SOURCES})
target_include_directories(g2m_phys PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(g2m_phys
  PUBLIC  g2m_client
  PRIVATE g2m_warnings g2m_options
)
