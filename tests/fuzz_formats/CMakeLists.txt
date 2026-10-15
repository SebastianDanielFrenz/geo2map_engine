# tests/fuzz_formats - fuzzing of every decoder of file- or server-supplied
# bytes (PLAN.md 2.4): the g2m.tile/1 container, src.elev, src.landcover,
# src.osm, tile packs, release/server manifest JSON, protocol paths and the
# HTTP binding (pack directories, request lines, responses; G6).
#   g2m_fuzz_run          target bodies (fuzz_run.cpp) over the normal libraries
#   g2m_fuzz_seeds_lib    synthetic seed corpus per target (fuzz_seeds.cpp)
#   g2m_fuzz_smoke_tests  Catch2, every leg: seeds + SplitMix64 mutations
#   g2m_fuzz_seeds        writes the corpus directories (CTest fixture)
#   fuzz_<name>           libFuzzer executables, only with G2M_BUILD_FUZZERS
#                         (clang, WSL legs): a fuzzer-instrumented copy of
#                         g2m_layers' sources + protocol.cpp; each runs for
#                         G2M_FUZZ_SECONDS as a CTest test labelled `fuzz`.
# (tests/fuzz is G1b's decoder fuzzing; this directory is G1a's formats.)

# http_wire.cpp (G6) is compiled in directly: it needs only g2m_layers and
# protocol.cpp, and this way the targets exist without G2M_BUILD_HTTP.
add_library(g2m_fuzz_run STATIC fuzz_run.cpp ${PROJECT_SOURCE_DIR}/src/http/http_wire.cpp)
target_include_directories(g2m_fuzz_run PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g2m_fuzz_run PUBLIC g2m_server PRIVATE g2m_warnings g2m_options)

add_library(g2m_fuzz_seeds_lib STATIC fuzz_seeds.cpp)
target_include_directories(g2m_fuzz_seeds_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g2m_fuzz_seeds_lib PUBLIC g2m_server g2m_headless_env PRIVATE g2m_warnings g2m_options)

add_executable(g2m_fuzz_smoke_tests test_fuzz_smoke.cpp)
target_link_libraries(g2m_fuzz_smoke_tests PRIVATE
  g2m_fuzz_run g2m_fuzz_seeds_lib g2m_test_catch_main g2m_warnings g2m_options
)
include(Catch)
catch_discover_tests(g2m_fuzz_smoke_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)

if(NOT G2M_BUILD_FUZZERS)
  return()
endif()
if(NOT CMAKE_CXX_COMPILER_ID STREQUAL "Clang" OR MSVC)
  message(FATAL_ERROR "G2M_BUILD_FUZZERS needs clang (libFuzzer); it is set only by the WSL clang presets")
endif()

set(G2M_FUZZ_SECONDS 10 CACHE STRING "Seconds each libFuzzer CTest run fuzzes for")

add_executable(g2m_fuzz_seeds fuzz_seeds_main.cpp)
target_link_libraries(g2m_fuzz_seeds PRIVATE g2m_fuzz_seeds_lib g2m_headless_env g2m_warnings g2m_options)

# The decoders under test, compiled again with coverage instrumentation.
add_library(g2m_fuzz_instrumented OBJECT
  ${G2M_LAYERS_SOURCES}
  ${PROJECT_SOURCE_DIR}/src/server/protocol.cpp
  ${PROJECT_SOURCE_DIR}/src/http/http_wire.cpp
  fuzz_run.cpp
)
target_include_directories(g2m_fuzz_instrumented PRIVATE ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src/layer ${G2M_GENERATED_INCLUDE_DIR})
target_link_libraries(g2m_fuzz_instrumented PRIVATE g2m_core g2m_dep_zstd g2m_dep_json g2m_warnings g2m_options)
target_compile_options(g2m_fuzz_instrumented PRIVATE -fsanitize=fuzzer-no-link)

set(_g2m_fuzz_dir ${CMAKE_CURRENT_BINARY_DIR}/corpus)
add_test(NAME g2m_fuzz_seeds COMMAND g2m_fuzz_seeds ${_g2m_fuzz_dir})
set_tests_properties(g2m_fuzz_seeds PROPERTIES FIXTURES_SETUP g2m_fuzz_corpus LABELS "fuzz")

foreach(_t container elev landcover osm pack manifest path region roadgraph roadgeom roadCrossSectionEval
           packDirectory httpRequest httpResponse)
  # Enumerator name: container -> Container.
  string(SUBSTRING ${_t} 0 1 _first)
  string(TOUPPER ${_first} _first)
  string(SUBSTRING ${_t} 1 -1 _rest)
  add_executable(fuzz_${_t} fuzz_main.cpp $<TARGET_OBJECTS:g2m_fuzz_instrumented>)
  target_compile_definitions(fuzz_${_t} PRIVATE G2M_FUZZ_TARGET=${_first}${_rest})
  target_compile_options(fuzz_${_t} PRIVATE -fsanitize=fuzzer-no-link)
  target_link_options(fuzz_${_t} PRIVATE -fsanitize=fuzzer)
  target_include_directories(fuzz_${_t} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(fuzz_${_t} PRIVATE g2m_core g2m_dep_zstd g2m_warnings g2m_options Threads::Threads)
  # -seed fixed so a failure reproduces; the work corpus is cleared by the
  # fixture; crash artifacts land next to it.
  add_test(NAME fuzz_${_t}
    COMMAND fuzz_${_t} -max_total_time=${G2M_FUZZ_SECONDS} -seed=1 -rss_limit_mb=2048 -timeout=20
            -artifact_prefix=${_g2m_fuzz_dir}/fuzz_${_t}- -print_final_stats=1
            ${_g2m_fuzz_dir}/work/${_t} ${_g2m_fuzz_dir}/seeds/${_t})
  set_tests_properties(fuzz_${_t} PROPERTIES FIXTURES_REQUIRED g2m_fuzz_corpus LABELS "fuzz" TIMEOUT 120)
endforeach()
