# tests/support/fixture_cut (PLAN.md G1c I2, D20): test-only fixture-cutting
# CLI. Links g2m_dep_libosmium DIRECTLY (g2m_decode only links it PRIVATE)
# because this is the one place in the whole repo allowed to use libosmium's
# own PBF WRITER (osmium::io::Writer, io/pbf_output.hpp - NOT
# io/any_output.hpp, which registers every osmium output format including
# bzip2-compressed ones and pulls in an undeclared bzlib.h dependency
# cmake/deps_decoders.cmake never declares) - g2m_decode itself only ever
# reads PBF.
add_executable(g2m_fixture_cut main.cpp)
target_link_libraries(g2m_fixture_cut PRIVATE g2m_decode g2m_dep_libosmium g2m_warnings g2m_options
                                               libdeflate::libdeflate_static)
# repack-dgm1 (G1c I3) reuses tests/support/zip_builder.h (header-only).
target_include_directories(g2m_fixture_cut PRIVATE ${PROJECT_SOURCE_DIR}/tests/support)

# clang-cl ASan stack instrumentation corrupts an optimised function's catch
# parameter (physics_sim vault TOOL-019, see src/decode/CMakeLists.txt's own
# copy of this workaround for osm/pbf_reader.cpp): main.cpp catches
# DecodeError/std::exception by reference and touches e.what().
if(MSVC AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_FLAGS MATCHES "-fsanitize=address")
  set_source_files_properties(
    ${CMAKE_CURRENT_SOURCE_DIR}/main.cpp
    PROPERTIES COMPILE_OPTIONS "/clang:-mllvm;/clang:-asan-stack=0"
  )
endif()
