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
  osm.cpp
  landcover.cpp
  soil.cpp
  bake.cpp
)
target_include_directories(g2m_import PUBLIC ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(g2m_import
  PUBLIC  g2m_server
  PRIVATE g2m_decode g2m_warnings g2m_options
)

# clang-cl ASan stack instrumentation corrupts an optimised function's catch
# parameter (physics_sim vault TOOL-019; see src/decode/CMakeLists.txt for the
# same fix on osm/pbf_reader.cpp and why the flag is two /clang: items).
# soil.cpp's open_set reads e.what() in a catch(const DecodeError&) and crashed
# the Windows asan leg ("Soil import: partial coverage, several sets, tiles
# without coverage and every rejection", access violation at soil.cpp:57).
# The other catch-by-reference sites in src/import pass on that leg today, so
# the fix stays scoped to this one translation unit.
if(MSVC AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_FLAGS MATCHES "-fsanitize=address")
  set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/soil.cpp
    PROPERTIES COMPILE_OPTIONS "/clang:-mllvm;/clang:-asan-stack=0"
  )
endif()
