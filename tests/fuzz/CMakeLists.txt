# tests/fuzz - libFuzzer targets for the ZIP/TIFF/PBF decode entry points
# (PLAN.md 13, milestone G1b). Built ONLY when G2M_BUILD_FUZZERS is ON AND
# the compiler is a genuine clang++ front end - clang-cl (Windows) reports
# CMAKE_CXX_COMPILER_ID "Clang" too but does not support -fsanitize=fuzzer,
# so it is explicitly excluded via MSVC (true for clang-cl, which emulates
# the MSVC driver). In practice this means: never built by the Windows
# release/debug presets this milestone's own build is done with, only ever
# by a WSL/Linux clang++ configure. G2M_BUILD_FUZZERS defaults OFF, so this
# file configures cleanly (adding nothing) everywhere else.
option(G2M_BUILD_FUZZERS "Build libFuzzer targets for the decode entry points (clang++, Linux only)" OFF)

if(G2M_BUILD_FUZZERS)
  if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR MSVC)
    message(FATAL_ERROR "G2M_BUILD_FUZZERS requires a genuine clang++ front end (not clang-cl) - configure under WSL/Linux")
  endif()

  add_library(g2m_fuzz_options INTERFACE)
  target_compile_options(g2m_fuzz_options INTERFACE -fsanitize=fuzzer,address -g)
  target_link_options(g2m_fuzz_options INTERFACE -fsanitize=fuzzer,address)

  add_executable(fuzz_zip fuzz_zip.cpp)
  target_link_libraries(fuzz_zip PRIVATE g2m_decode g2m_fuzz_options)

  add_executable(fuzz_tiff fuzz_tiff.cpp)
  target_link_libraries(fuzz_tiff PRIVATE g2m_decode g2m_fuzz_options)

  add_executable(fuzz_pbf fuzz_pbf.cpp)
  target_link_libraries(fuzz_pbf PRIVATE g2m_decode g2m_fuzz_options)
endif()
