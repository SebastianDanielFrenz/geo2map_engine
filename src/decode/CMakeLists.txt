# g2m_decode (PLAN.md 2.2-2.4, 13, milestone G1b): pure, deterministic,
# bounds-checked decoders for g2m's source data - ZIP (own code + libdeflate
# DEFLATE), TIFF/BigTIFF/GeoTIFF (own code, COG-capable via IByteSource),
# OSM PBF (libosmium + protozero + zlib-ng, wrapped behind an
# osmium-type-free API). Every third-party dependency declared in
# cmake/deps_decoders.cmake (included by the top-level CMakeLists.txt
# before this subdirectory).
set(G2M_DECODE_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/byte_source.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/zip/zip_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tiff/lzw.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tiff/predictor.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tiff/packbits.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tiff/geotransform.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tiff/tfw.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/tiff/tiff_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/osm/pbf_reader.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/dgm1.cpp
  CACHE INTERNAL "g2m_decode sources (also compiled into the fuzz-instrumented copy, tests/fuzz)"
)
add_library(g2m_decode STATIC ${G2M_DECODE_SOURCES})
target_include_directories(g2m_decode
  PUBLIC  ${PROJECT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(g2m_decode
  PUBLIC  g2m_core
  PRIVATE libdeflate::libdeflate_static g2m_dep_libosmium g2m_warnings g2m_options
)
