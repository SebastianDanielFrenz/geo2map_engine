# cmake/deps_decoders.cmake - FetchContent for g2m_decode's third-party
# dependencies (PLAN.md 13, owner-approved list). Pinned tags, verified to
# exist against the real upstream remotes on 2026-09-25.
#
# | Dependency | Pin     | Licence      | Use                                |
# |------------|---------|--------------|-------------------------------------|
# | libdeflate | v1.26   | MIT          | DEFLATE for ZIP + TIFF/COG          |
# | protozero  | v1.8.2  | BSD-2-Clause | header-only protobuf decode (PBF)   |
# | zlib-ng    | 2.3.3   | zlib         | zlib-API DEFLATE for libosmium's PBF blob decompression |
# | libosmium  | v2.23.1 | BSL-1.0      | OSM PBF parsing (g2m_decode's osm/) |
#
# libosmium and protozero are consumed header-only: their own CMakeLists.txt
# pulls in Boost, GDAL, test frameworks etc. we do not want and do not need
# for PBF-only reading, so they are fetched WITHOUT add_subdirectory()-ing
# them (FetchContent_Populate, not FetchContent_MakeAvailable) and exposed as
# plain INTERFACE targets with SYSTEM include dirs (so their headers' own
# warnings never trip g2m_warnings' /WX on a g2m_decode translation unit that
# includes them).

# libdeflate and zlib-ng are C projects (project(... LANGUAGES C)) - the
# first C sources this all-C++ repo ever builds, so CMAKE_C_COMPILER has
# never been probed before this point. Left to its own defaults, CMake picks
# whatever bare "clang"/"cc" the VS dev shell's PATH happens to expose (the
# GNU-style driver, not clang-cl) even though CMAKE_CXX_COMPILER is
# clang-cl - that C compiler then rejects every /-style MSVC flag our
# clang-cl-oriented CMAKE_C_FLAGS inherit. Force the C compiler to match the
# C++ one (clang-cl handles .c inputs fine) before the first project() call
# that needs C runs. Only meaningful on the clang-cl presets; msvc-* already
# gets a matching C default (cl.exe) either way.
if(CMAKE_CXX_COMPILER MATCHES "clang-cl" AND NOT CMAKE_C_COMPILER)
  set(CMAKE_C_COMPILER "${CMAKE_CXX_COMPILER}" CACHE FILEPATH "C compiler (forced to match clang-cl - see deps_decoders.cmake)" FORCE)
endif()

include(FetchContent)

# --- libdeflate (MIT): built as a static lib, no shared lib/gzip CLI/tests.
set(LIBDEFLATE_BUILD_STATIC_LIB ON CACHE BOOL "" FORCE)
set(LIBDEFLATE_BUILD_SHARED_LIB OFF CACHE BOOL "" FORCE)
set(LIBDEFLATE_BUILD_GZIP OFF CACHE BOOL "" FORCE)
set(LIBDEFLATE_BUILD_TESTS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  libdeflate
  GIT_REPOSITORY https://github.com/ebiggers/libdeflate.git
  GIT_TAG        v1.26
)
FetchContent_MakeAvailable(libdeflate)

# --- zlib-ng (zlib licence): built ZLIB_COMPAT so it drop-in-replaces
# zlib.h/libz for libosmium's PBF blob decompression (osmium/io/detail/
# zlib.hpp just does #include <zlib.h>). No tests/install utils.
set(ZLIB_COMPAT ON CACHE BOOL "" FORCE)
set(ZLIB_ALIASES ON CACHE BOOL "" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
set(WITH_GTEST OFF CACHE BOOL "" FORCE)
set(INSTALL_UTILS OFF CACHE BOOL "" FORCE)
# zlib-ng's own CMakeLists gates its shared-vs-static "zlib" ALIAS target
# purely on whether BUILD_SHARED_LIBS is DEFINED at all
# (`if(NOT DEFINED BUILD_SHARED_LIBS OR BUILD_SHARED_LIBS)` -> shared) - left
# unset (this repo's own top-level CMakeLists never sets it), it silently
# built a shared zlibd1.dll and aliased "zlib" to it, which every g2m_decode
# test executable then failed to start at all (STATUS_DLL_NOT_FOUND,
# 0xC0000135 - the DLL sits under the FetchContent build dir, nowhere near
# any executable's directory or PATH). Force it OFF so "zlib" aliases the
# static library instead - this is the one dependency (unlike
# libdeflate/protozero/libosmium above) that has no separate
# always-static CMake option of its own.
set(BUILD_SHARED_LIBS OFF CACHE BOOL "" FORCE)
FetchContent_Declare(
  zlibng
  GIT_REPOSITORY https://github.com/zlib-ng/zlib-ng.git
  GIT_TAG        2.3.3
)
FetchContent_MakeAvailable(zlibng)
# zlib-ng's own CMakeLists always exports a CMake target named "zlib" (ALIAS
# to zlib-ng or zlib-ng-static depending on BUILD_SHARED_LIBS) when
# ZLIB_COMPAT=ON - g2m_decode links against that, never the OS/vcpkg zlib.

# --- protozero (BSD-2-Clause): header-only. Populate source only, no
# add_subdirectory (its CMakeLists builds fuzzers/tests we do not want).
FetchContent_Declare(
  protozero
  GIT_REPOSITORY https://github.com/mapbox/protozero.git
  GIT_TAG        v1.8.2
)
FetchContent_GetProperties(protozero)
if(NOT protozero_POPULATED)
  FetchContent_Populate(protozero)
endif()
add_library(g2m_dep_protozero INTERFACE)
target_include_directories(g2m_dep_protozero SYSTEM INTERFACE "${protozero_SOURCE_DIR}/include")

# --- libosmium (BSL-1.0): header-only for our PBF-only usage. Populate
# source only (its own CMakeLists needs Boost + FindOsmium.cmake + GDAL for
# its full test/example suite, none of which g2m_decode needs). Only the
# node/way/relation/tag PBF-reading path is used; that path does not touch
# libosmium's two boost-dependent headers (osm/crc.hpp, util/string_matcher.hpp).
FetchContent_Declare(
  libosmium
  GIT_REPOSITORY https://github.com/osmcode/libosmium.git
  GIT_TAG        v2.23.1
)
FetchContent_GetProperties(libosmium)
if(NOT libosmium_POPULATED)
  FetchContent_Populate(libosmium)
endif()
add_library(g2m_dep_libosmium INTERFACE)
target_include_directories(g2m_dep_libosmium SYSTEM INTERFACE "${libosmium_SOURCE_DIR}/include")
target_link_libraries(g2m_dep_libosmium INTERFACE g2m_dep_protozero zlib)
target_compile_definitions(g2m_dep_libosmium INTERFACE
  NOMINMAX
  WIN32_LEAN_AND_MEAN
  _CRT_SECURE_NO_WARNINGS
)
if(WIN32)
  # libosmium's Windows file-mapping path (osmium/util/file.hpp -> Windows.h)
  # needs the Win32 threading libs pulled in transitively by <thread>/<future>
  # on MSVC-ABI clang-cl; nothing extra to link explicitly (ucrt/msvcrt handle
  # it), listed here only as the point where a future gap would be fixed.
endif()
