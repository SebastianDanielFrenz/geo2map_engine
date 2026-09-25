cmake_minimum_required(VERSION 3.25)

# Static CRT for every target (PLAN.md 2.3: same CRT as physics_sim, so one
# game DLL can link both). Must be set BEFORE project(): CMake's internal
# "does the compiler work" try-compile already reads it, and a debug dynamic
# CRT picked there breaks the clang-cl ASan link. The asan preset also sets
# CMAKE_TRY_COMPILE_CONFIGURATION, because try_compile defaults to Debug.
if(POLICY CMP0091)
  cmake_policy(SET CMP0091 NEW)
endif()
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")

# clang-cl + AddressSanitizer (Windows "asan" preset only): CMake's Ninja
# generator links with lld-link/link directly, which ignores
# -fsanitize=address, so the ASan runtime must be named explicitly (static
# /MT CRT variant, the same libraries clang-cl itself would pass). Set before
# project() because the compiler-check link step needs it too.
if(CMAKE_CXX_COMPILER MATCHES "clang-cl" AND CMAKE_CXX_FLAGS MATCHES "-fsanitize=address")
  execute_process(
    COMMAND "${CMAKE_CXX_COMPILER}" -print-resource-dir
    OUTPUT_VARIABLE G2M_CLANG_RESOURCE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  set(G2M_CLANG_ASAN_LIB_DIR "${G2M_CLANG_RESOURCE_DIR}/lib/windows")
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /LIBPATH:\"${G2M_CLANG_ASAN_LIB_DIR}\" /WHOLEARCHIVE:clang_rt.asan-x86_64.lib \"${G2M_CLANG_ASAN_LIB_DIR}/clang_rt.asan-x86_64.lib\" /WHOLEARCHIVE:clang_rt.asan_cxx-x86_64.lib \"${G2M_CLANG_ASAN_LIB_DIR}/clang_rt.asan_cxx-x86_64.lib\" /INFERASANLIBS:NO")
endif()

project(geo2map_engine VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

option(G2M_BUILD_TESTS "Build the Catch2 test suites" ON)
option(G2M_BUILD_REFGEN "Build tools/refgen (fetches GeographicLib; reference-value generator, never linked into g2m)" OFF)
option(G2M_SANITIZER_BUILD "This build instruments code with a sanitizer (ASan/UBSan/TSan)" OFF)
option(G2M_BUILD_FUZZERS "Build the libFuzzer targets in tests/fuzz_formats and tests/fuzz (clang only; set by the WSL clang presets)" OFF)

# An optimised build type must carry an optimisation flag. CMake seeds
# CMAKE_CXX_FLAGS_<CONFIG> only on a build dir's first configure; a cache
# that lost them keeps testing an unoptimised "release" forever.
if(CMAKE_BUILD_TYPE MATCHES "^(Release|RelWithDebInfo|MinSizeRel)$")
  string(TOUPPER "${CMAKE_BUILD_TYPE}" _g2m_bt)
  set(_g2m_flags " ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${_g2m_bt}} ")
  if(NOT _g2m_flags MATCHES " [-/]O[123sxz]")
    message(FATAL_ERROR "CMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} but no optimisation flag in CMAKE_CXX_FLAGS / "
                        "CMAKE_CXX_FLAGS_${_g2m_bt}: stale cache. Reconfigure: cmake --preset <name> --fresh")
  endif()
endif()

include(FetchContent)

# --- Catch2 v3 (BSL-1.0), tests only. Same pin as physics_sim.
if(G2M_BUILD_TESTS)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG        v3.16.0
  )
  FetchContent_MakeAvailable(Catch2)
  # Catch2 needs C++ exceptions ON in its own TUs, or its exception
  # translator is an aborting stub. /EHsc lives only in CMake's default
  # CMAKE_CXX_FLAGS, which presets that set CMAKE_CXX_FLAGS replace.
  if(MSVC)
    target_compile_options(Catch2 PRIVATE /EHsc)
    target_compile_options(Catch2WithMain PRIVATE /EHsc)
  endif()
  # clang-cl ASan stack instrumentation corrupts an optimised function's
  # catch parameter (MSVC EH); Catch2's translator catches by reference.
  # Turn only stack instrumentation off, only for Catch2 (physics_sim's
  # vault TOOL-019).
  if(MSVC AND CMAKE_CXX_COMPILER_ID STREQUAL "Clang" AND CMAKE_CXX_FLAGS MATCHES "-fsanitize=address")
    target_compile_options(Catch2 PRIVATE "SHELL:-mllvm -asan-stack=0")
    target_compile_options(Catch2WithMain PRIVATE "SHELL:-mllvm -asan-stack=0")
  endif()
endif()

# ===========================================================================
# Compile-option INTERFACE targets for OUR OWN targets only (PLAN.md 2.3).
# Third-party targets (Catch2, GeographicLib) keep their own flags.
# ===========================================================================

add_library(g2m_warnings INTERFACE)
if(MSVC)
  target_compile_options(g2m_warnings INTERFACE /W4 /WX)
else()
  target_compile_options(g2m_warnings INTERFACE -Wall -Wextra -Wpedantic -Werror)
endif()

# g2m_options: /arch:AVX2, /fp:precise, FMA contraction OFF, no fast-math
# (PLAN.md 2.4). Per compiler:
#  - cl:       /fp:precise does not contract a*b+c into an FMA (VS 2022 needs
#              an explicit /fp:contract for that), so nothing more is needed.
#  - clang-cl: clang defaults to -ffp-contract=on; clang-cl's own option
#              parser does not know the spelling, so it is forwarded with
#              /clang:.
#  - clang:    -ffp-contract=off directly.
add_library(g2m_options INTERFACE)
if(MSVC)
  target_compile_options(g2m_options INTERFACE /arch:AVX2 /fp:precise /EHsc)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(g2m_options INTERFACE /clang:-ffp-contract=off)
  endif()
else()
  target_compile_options(g2m_options INTERFACE -mavx2 -mfma -ffp-contract=off)
endif()

# G1a: zstd / SQLite / nlohmann (cmake/deps_core.cmake) and the layer, server
# and client libraries (src/layer, src/server, src/client).
include(cmake/deps_core.cmake)
add_subdirectory(src/layer)
add_subdirectory(src/server)
add_subdirectory(src/client)

add_subdirectory(third_party/musl_math)
add_subdirectory(src)

include(cmake/deps_decoders.cmake)
add_subdirectory(src/decode)

if(G2M_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests/layer)
  add_subdirectory(tests/server)
  add_subdirectory(tests/fuzz_formats)
  add_subdirectory(tests/support)
  add_subdirectory(tests/unit)
  add_subdirectory(tests/golden)
  add_subdirectory(tests/fuzz)
endif()

if(G2M_BUILD_REFGEN)
  add_subdirectory(tools/refgen)
endif()

# Later targets (PLAN.md 2.3), not built yet - placeholders only, see CLAUDE.md:
#   g2m_layers (G1+), g2m_server / g2m_client (G1+), g2m_c (C API), g2m_http
#   (G6), g2m_ps_bridge (bridges/physics_sim, only if(TARGET ps_core)),
#   g2m_godot (adapters/godot), apps g2m_server / g2m_tiler / g2m_cli.
