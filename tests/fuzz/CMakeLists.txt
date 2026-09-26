# tests/fuzz - libFuzzer targets for the g2m_decode entry points (ZIP, TIFF,
# OSM PBF; PLAN.md 13, milestone G1b). Same scheme as tests/fuzz_formats
# (G1a): built only with G2M_BUILD_FUZZERS (top-level option, set by the WSL
# clang presets), the decoders compiled again as a coverage-instrumented
# OBJECT copy (-fsanitize=fuzzer-no-link) so libFuzzer is guided by the
# decoder code itself, not just the harness; each target runs for
# G2M_FUZZ_SECONDS with a fixed seed as a CTest test labelled `fuzz`.
# ASan comes from the preset's own flags (linux-asan-ubsan), never from here.
if(NOT G2M_BUILD_FUZZERS)
  return()
endif()
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR MSVC)
  message(FATAL_ERROR "G2M_BUILD_FUZZERS needs clang (libFuzzer); it is set only by the WSL clang presets")
endif()

add_library(g2m_decode_fuzz_instrumented OBJECT ${G2M_DECODE_SOURCES})
target_include_directories(g2m_decode_fuzz_instrumented
  PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src/decode)
target_link_libraries(g2m_decode_fuzz_instrumented
  PRIVATE g2m_core libdeflate::libdeflate_static g2m_dep_libosmium g2m_warnings g2m_options)
target_compile_options(g2m_decode_fuzz_instrumented PRIVATE -fsanitize=fuzzer-no-link)

set(_g2m_decode_fuzz_dir ${CMAKE_CURRENT_BINARY_DIR}/corpus)
# Clear the work corpus before every run so -seed=1 reproduces.
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/reset_corpus.cmake
  "file(REMOVE_RECURSE \"${_g2m_decode_fuzz_dir}\")
"
  "file(MAKE_DIRECTORY \"${_g2m_decode_fuzz_dir}/zip\" \"${_g2m_decode_fuzz_dir}/tiff\" \"${_g2m_decode_fuzz_dir}/pbf\")
")
add_test(NAME g2m_decode_fuzz_corpus_reset
  COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/reset_corpus.cmake)
set_tests_properties(g2m_decode_fuzz_corpus_reset PROPERTIES FIXTURES_SETUP g2m_decode_fuzz_corpus LABELS "fuzz")
foreach(_t zip tiff pbf)
  add_executable(fuzz_${_t} fuzz_${_t}.cpp $<TARGET_OBJECTS:g2m_decode_fuzz_instrumented>)
  target_compile_options(fuzz_${_t} PRIVATE -fsanitize=fuzzer-no-link)
  target_link_options(fuzz_${_t} PRIVATE -fsanitize=fuzzer)
  target_link_libraries(fuzz_${_t} PRIVATE
    g2m_core libdeflate::libdeflate_static g2m_dep_libosmium g2m_warnings g2m_options)
  # Real-data-shaped seed corpus (PLAN.md G1c I6 item B, testdata/fuzz_seeds/
  # <t>/) as a SECOND, read-only libFuzzer corpus dir - libFuzzer reads seeds
  # from every corpus dir given but only ever WRITES new finds into the
  # first, so this never mutates the committed seeds. -max_len bounds a
  # single fuzz iteration's input size regardless of corpus content.
  add_test(NAME fuzz_${_t}
    COMMAND fuzz_${_t} -max_total_time=${G2M_FUZZ_SECONDS} -seed=1 -rss_limit_mb=2048 -timeout=20 -max_len=1048576
            -artifact_prefix=${_g2m_decode_fuzz_dir}/fuzz_${_t}- -print_final_stats=1
            ${_g2m_decode_fuzz_dir}/${_t} ${PROJECT_SOURCE_DIR}/testdata/fuzz_seeds/${_t})
  set_tests_properties(fuzz_${_t} PROPERTIES FIXTURES_REQUIRED g2m_decode_fuzz_corpus LABELS "fuzz" TIMEOUT 120)
endforeach()
