# g2m_decode (PLAN.md 2.2-2.4, 13, milestone G1b): pure, deterministic,
# bounds-checked decoders for g2m's source data - ZIP (own code + libdeflate
# DEFLATE), TIFF/BigTIFF/GeoTIFF (own code, COG-capable via IByteSource),
# OSM PBF (libosmium + protozero + zlib-ng, wrapped behind an
# osmium-type-free API). Every third-party dependency declared in
# cmake/deps_decoders.cmake (included by the top-level CMakeLists.txt
# before this subdirectory).
add_library(g2m_decode STATIC
  byte_source.cpp
  zip/zip_reader.cpp
  tiff/lzw.cpp
  tiff/predictor.cpp
  tiff/packbits.cpp
  tiff/geotransform.cpp
  tiff/tfw.cpp
  tiff/tiff_reader.cpp
  osm/pbf_reader.cpp
  dgm1.cpp
)
target_include_directories(g2m_decode
  PUBLIC  ${PROJECT_SOURCE_DIR}/include
  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(g2m_decode
  PUBLIC  g2m_core
  PRIVATE libdeflate::libdeflate_static g2m_dep_libosmium g2m_warnings g2m_options
)
