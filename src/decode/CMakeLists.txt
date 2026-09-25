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

# clang-cl ASan stack instrumentation corrupts an optimised function's catch
# parameter (physics_sim vault TOOL-019, confirmed here on the Windows asan
# leg: "osm: reading a garbage .osm.pbf file throws DecodeError, not a
# crash" / "... reading a nonexistent path ..." crashed instead of throwing).
# osm/pbf_reader.cpp is the only src/ site that catches std::exception by
# reference and then touches it (e.what()); every other catch site in src/
# either doesn't exist or catches purely by type (safe, see TOOL-019). Scope
# the fix to exactly that translation unit, only under clang-cl + ASan - the
# same narrow pattern the top-level CMakeLists.txt already applies to
# Catch2's own translator.
# "SHELL:-mllvm -asan-stack=0" (the syntax the top-level CMakeLists.txt uses
# via target_compile_options() for Catch2) is NOT honoured the same way by
# set_source_files_properties()'s COMPILE_OPTIONS: Ninja passed the whole
# quoted "SHELL:..." string as one literal argument to clang-cl instead of
# splitting/de-quoting it ("error: no such file or directory: 'SHELL:-mllvm
# -asan-stack=0'"). Two separate /clang:-prefixed list items give clang-cl
# the same two-token "-mllvm -asan-stack=0" without needing SHELL: grouping.
if(MSVC AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_FLAGS MATCHES "-fsanitize=address")
  set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/osm/pbf_reader.cpp
    PROPERTIES COMPILE_OPTIONS "/clang:-mllvm;/clang:-asan-stack=0"
  )
endif()
