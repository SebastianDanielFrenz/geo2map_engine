# g2m_import (PLAN.md G1c I2): the importer support library shared by
# apps/g2m_tiler and its tests - region resolution against an OSM PBF
# snapshot (region_resolve.h), the tile-write seam (sink.h) and the
# ordered-parallel driver every importer's own per-tile loop uses
# (ordered_parallel.h - header-only, no library code). g2m_decode is
# PRIVATE: no osmium/protozero type appears in any g2m/import public header.
add_library(g2m_import STATIC
  region_resolve.cpp
  sink.cpp
  report.cpp
  dem.cpp
)
target_include_directories(g2m_import PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(g2m_import
  PUBLIC  g2m_server
  PRIVATE g2m_decode g2m_warnings g2m_options
)
