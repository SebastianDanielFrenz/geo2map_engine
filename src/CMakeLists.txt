# g2m_core (PLAN.md 2.3): deterministic math, geodesy, tile keys, rasters,
# hashing. No third-party dependency; the musl port's objects are compiled
# straight into this archive.
add_library(g2m_core STATIC
  core/det_math.cpp
  core/version.cpp
  core/hash.cpp
  core/result.cpp
  core/tile_key.cpp
  core/raster.cpp
  core/geo/angle.cpp
  core/geo/tm.cpp
  core/geo/utm.cpp
  core/geo/ecef.cpp
  core/geo/session_frame.cpp
  core/geom/exact.cpp
  core/geom/ring_set.cpp
  core/geom/ring_assembly.cpp
  $<TARGET_OBJECTS:g2m_musl_math>
)
target_include_directories(g2m_core
  PUBLIC  ${PROJECT_SOURCE_DIR}/include
  PRIVATE ${PROJECT_SOURCE_DIR}/third_party/musl_math
)
target_link_libraries(g2m_core PRIVATE g2m_warnings g2m_options)
target_compile_definitions(g2m_core PRIVATE
  G2M_VERSION_STRING="${PROJECT_VERSION}"
)
