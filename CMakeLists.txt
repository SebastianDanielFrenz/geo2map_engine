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
#
# Guarded to the actual top-level CMakeLists.txt (PLAN.md G2.0 Task D):
# PROJECT_IS_TOP_LEVEL does not exist yet at this point (it is set by
# project(), a few lines below) - CMAKE_SOURCE_DIR (the directory of the
# outermost CMakeLists.txt CMake was invoked on) vs. CMAKE_CURRENT_SOURCE_DIR
# (this file's own directory) is the equivalent check that is already valid
# before project() runs. If this file is ever add_subdirectory()-ed from a
# parent build (e.g. a future bridges/physics_sim consumer), the mutation is
# skipped: CMAKE_EXE_LINKER_FLAGS is a directory-scoped variable that this
# APPENDS to, and a parent project using the same asan-setup pattern for its
# own targets must not have this geo2map_engine subtree append the same
# /WHOLEARCHIVE flags to it a second time. A directly-invoked (top-level)
# configure is completely unaffected: CMAKE_SOURCE_DIR ==
# CMAKE_CURRENT_SOURCE_DIR is always true there, so behaviour stays
# byte-identical.
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR AND
   CMAKE_CXX_COMPILER MATCHES "clang-cl" AND CMAKE_CXX_FLAGS MATCHES "-fsanitize=address")
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
# PLAN.md G2.0 Task D: OFF skips the whole decoder/importer stack - the
# libdeflate/zlib-ng/protozero/libosmium FetchContent deps (cmake/deps_decoders.cmake),
# src/decode, src/import and apps/g2m_tiler - for a pure CONSUMER of the
# tile-serving stack (g2m_core/g2m_layers/g2m_server/g2m_client only, e.g. a
# game linking g2m_server to read an already-built store, PLAN.md's
# bridges/physics_sim / racing_game use case). ON reproduces the previous,
# only behaviour exactly (every decoder dep still fetched, src/decode and
# src/import still built).
option(G2M_BUILD_IMPORT "Build src/decode, src/import and apps/g2m_tiler (and fetch their decoder dependencies)" ON)
option(G2M_BUILD_APPS "Build the g2m_tiler CLI (apps/g2m_tiler, PLAN.md G1c I2) - requires G2M_BUILD_IMPORT" ON)

if(G2M_BUILD_APPS AND NOT G2M_BUILD_IMPORT)
  message(FATAL_ERROR "G2M_BUILD_APPS requires G2M_BUILD_IMPORT: apps/g2m_tiler links g2m_import")
endif()
if(G2M_BUILD_TESTS AND NOT G2M_BUILD_IMPORT)
  message(FATAL_ERROR "G2M_BUILD_TESTS requires G2M_BUILD_IMPORT: several test suites link g2m_import "
                      "or exercise g2m_decode directly")
endif()

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

# ---------------------------------------------------------------------------
# physics_sim (bridges/physics_sim, PLAN.md R2.2 section 1a/G5). Not a
# submodule - physics_sim stays entirely in its own sibling repo
# (S:\claude_code\physics_sim); G2M_PS_SOURCE_DIR instead points at a checkout
# of it (bridges/physics_sim/physics_sim.pin records the exact commit this
# bridge is built against; tools/ci.ps1's own bridge-release leg clones/checks
# it out under out/ps_pin/src and wires its FetchContent _deps offline via
# FETCHCONTENT_SOURCE_DIR_JOLTPHYSICS/_NLOHMANN_JSON, never a network fetch).
# Mirrors racing_game's own CMakeLists.txt ordering (its physics_sim
# add_subdirectory sits before its own Catch2 FetchContent_Declare, lines
# ~81-170): embedded BEFORE our own Catch2 declare below, so physics_sim's
# own FetchContent declarations (JoltPhysics, nlohmann_json, and - only if a
# caller leaves PS_BUILD_TESTS ON - its own Catch2) resolve first and nothing
# here can shadow or clash with them, even though in practice the
# bridge-release preset sets PS_BUILD_TESTS=OFF so physics_sim never declares
# its own Catch2 at all. EXCLUDE_FROM_ALL: physics_sim's own tools/tests are
# not part of geo2map_engine's default build - only ps_core (plus whatever
# PS_BUILD_TESTS/PS_BUILD_GODOT_ADAPTER, the caller's own cache variables,
# additionally pull in) is built here. Binary dir "${CMAKE_BINARY_DIR}/ps" so
# its build tree stays out of geo2map_engine's own out-of-source layout.
#
# Empty (the default): the physics_sim bridge is skipped entirely - every
# existing geo2map_engine build (consumer-check, the sanitizer legs, etc.) is
# completely unaffected.
# ---------------------------------------------------------------------------
set(G2M_PS_SOURCE_DIR "" CACHE PATH
    "Path to a physics_sim checkout (bridges/physics_sim/physics_sim.pin's own commit) to embed as a nested project; empty skips bridges/physics_sim entirely")
if(G2M_PS_SOURCE_DIR)
  add_subdirectory("${G2M_PS_SOURCE_DIR}" "${CMAKE_BINARY_DIR}/ps" EXCLUDE_FROM_ALL)
endif()

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

# Always-defined G2M_SANITIZER_BUILD=0/1 compile definition (mirrors
# physics_sim's own PS_SANITIZER_BUILD pattern) for any of our own targets
# that need to know at compile time whether they are built under ASan/UBSan/
# TSan (set ON by the asan/linux-asan-ubsan/linux-tsan presets above) - e.g.
# tests/mesh/alloc_counter.cpp, whose global operator new/delete overrides
# must be compiled out under a sanitizer build (they would otherwise
# duplicate-symbol-clash with the sanitizer runtime's own overrides at link
# time). $<BOOL:...> turns the cached ON/OFF option into a literal 0/1 so
# `#if G2M_SANITIZER_BUILD` works without an extra defined()/IS_ON check.
target_compile_definitions(g2m_options INTERFACE G2M_SANITIZER_BUILD=$<BOOL:${G2M_SANITIZER_BUILD}>)

# G1a: zstd / SQLite / nlohmann (cmake/deps_core.cmake) and the layer, server
# and client libraries (src/layer, src/server, src/client).
include(cmake/deps_core.cmake)
add_subdirectory(src/layer)
add_subdirectory(src/server)
add_subdirectory(src/client)

# G2.3: engine-neutral terrain render chunks and LOD selection (links
# g2m_core and g2m_layers only - no rendering-engine type anywhere in it).
add_subdirectory(src/mesh)

# R2.2 section 1a: g2m_phys (ps-free physics-tile grid, resident height
# cache, fill_physics_heights), built UNCONDITIONALLY (not gated by
# G2M_BUILD_IMPORT) so every CI leg, including the sanitizer legs, exercises
# it - see src/phys/CMakeLists.txt.
add_subdirectory(src/phys)

# Terrain migration plan (vault G2M-020): g2m_world, the engine-neutral terrain
# service pieces moved from racing_game (links g2m_client/g2m_mesh/g2m_phys).
add_subdirectory(src/world)

add_subdirectory(third_party/musl_math)
add_subdirectory(src)

if(G2M_BUILD_IMPORT)
  include(cmake/deps_decoders.cmake)
  add_subdirectory(src/decode)

  # G1c I2: the importer support library (region resolution, tile-write seam)
  # and its CLI, apps/g2m_tiler.
  add_subdirectory(src/import)
  if(G2M_BUILD_APPS)
    add_subdirectory(apps/g2m_tiler)
  endif()
  # Street-level fidelity plan P0: OSM tag-coverage probe (tools/osm_tag_probe).
  add_subdirectory(tools/osm_tag_probe)
endif()

if(G2M_BUILD_TESTS)
  include(CTest)
  enable_testing()
  add_subdirectory(tests/layer)
  add_subdirectory(tests/server)
  add_subdirectory(tests/fuzz_formats)
  add_subdirectory(tests/support)
  add_subdirectory(tests/unit)
  add_subdirectory(tests/mesh)
  add_subdirectory(tests/phys)
  add_subdirectory(tests/world)
  add_subdirectory(tests/golden)
  add_subdirectory(tests/fuzz)
endif()

if(G2M_BUILD_REFGEN)
  add_subdirectory(tools/refgen)
endif()

# Later targets (PLAN.md 2.3), not built yet - placeholders only, see CLAUDE.md:
#   g2m_c (C API), g2m_http (G6), g2m_godot (adapters/godot), apps g2m_server /
#   g2m_cli. (g2m_layers/g2m_server/g2m_client: G1+, already built above.
#   g2m_import and its app g2m_tiler: G1c I2, already built above.
#   g2m_ps_bridge: bridges/physics_sim, G5, built below.)

# bridges/physics_sim (g2m_ps_bridge, PLAN.md R2.2 section 1a/G5): only when
# G2M_PS_SOURCE_DIR (above) actually embedded physics_sim's own ps_core -
# `if(TARGET ps_core)` is the guard. Deliberately OUTSIDE the G2M_BUILD_IMPORT
# block above: the bridge only needs g2m_server/g2m_client/g2m_layers/g2m_phys,
# never the decoder/importer stack.
if(TARGET ps_core)
  add_subdirectory(bridges/physics_sim)
endif()
