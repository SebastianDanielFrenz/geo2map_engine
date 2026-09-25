# cmake/deps_core.cmake - FetchContent for g2m_layers / g2m_server /
# g2m_client (G1a). Owner-approved list: PLAN.md 13. Every download is
# pinned to an exact release archive plus its hash; nothing is installed
# system-wide.
#
# | Dependency    | Pin                     | Licence        | Target          | Used by                    |
# |---------------|-------------------------|----------------|-----------------|----------------------------|
# | zstd          | v1.5.7                  | BSD-3-Clause   | g2m_dep_zstd    | g2m_layers (PRIVATE)       |
# | SQLite        | 3.53.4 (amalgamation)   | public domain  | g2m_dep_sqlite3 | g2m_server (PRIVATE)       |
# | nlohmann_json | v3.12.0                 | MIT            | g2m_dep_json    | g2m_layers (PRIVATE)       |
#
# zstd and SQLite are C, compiled here as our own static targets from the
# upstream sources (their own build systems pull in programs, tests, shared
# libs and threading we do not want). Their C flags come from the preset's
# CMAKE_C_FLAGS (sanitizers), not from g2m_warnings/g2m_options.

enable_language(C)
include(FetchContent)

# --- zstd v1.5.7 (tag commit f8745da6ff1ad1e7bab384bd1f9d742439278e99).
# Release tarball, not the git tag: smaller, and hash-verified.
FetchContent_Declare(
  g2m_zstd
  URL      https://github.com/facebook/zstd/releases/download/v1.5.7/zstd-1.5.7.tar.gz
  URL_HASH SHA256=eb33e51f49a15e023950cd7825ca74a4a2b43db8354825ac24fc1b7ee09e6fa3
  SOURCE_SUBDIR g2m_no_cmake   # populate only; never add_subdirectory upstream's build
)
FetchContent_MakeAvailable(g2m_zstd)

# Determinism (PLAN.md 2.4, docs/formats/tile_container.md): no
# multithreading (ZSTD_MULTITHREAD is never defined), no legacy formats, no
# hand-written assembly, and one code path on every CPU (DYNAMIC_BMI2=0: no
# run-time dispatch on BMI2). The compression parameters themselves are
# pinned in src/layer/zstd_codec.cpp.
file(GLOB _g2m_zstd_sources CONFIGURE_DEPENDS
  "${g2m_zstd_SOURCE_DIR}/lib/common/*.c"
  "${g2m_zstd_SOURCE_DIR}/lib/compress/*.c"
  "${g2m_zstd_SOURCE_DIR}/lib/decompress/*.c"
)
list(SORT _g2m_zstd_sources)
add_library(g2m_dep_zstd STATIC ${_g2m_zstd_sources})
target_include_directories(g2m_dep_zstd SYSTEM PUBLIC "${g2m_zstd_SOURCE_DIR}/lib")
target_compile_definitions(g2m_dep_zstd
  PRIVATE ZSTD_LEGACY_SUPPORT=0 ZSTD_DISABLE_ASM=1 DYNAMIC_BMI2=0 ZSTD_TRACE=0
  PUBLIC  XXH_NAMESPACE=ZSTD_
)
set_target_properties(g2m_dep_zstd PROPERTIES FOLDER third_party POSITION_INDEPENDENT_CODE ON)

# --- SQLite 3.53.4 amalgamation (public domain). SHA3-256 as published on
# sqlite.org/download.html.
FetchContent_Declare(
  g2m_sqlite
  URL      https://sqlite.org/2026/sqlite-amalgamation-3530400.zip
  URL_HASH SHA3_256=628a44cfe82c66aed1ccbbe85a562d2e33ebe64b3288981ed76285612227934e
  SOURCE_SUBDIR g2m_no_cmake
)
FetchContent_MakeAvailable(g2m_sqlite)

add_library(g2m_dep_sqlite3 STATIC "${g2m_sqlite_SOURCE_DIR}/sqlite3.c")
target_include_directories(g2m_dep_sqlite3 SYSTEM PUBLIC "${g2m_sqlite_SOURCE_DIR}")
# Recommended-options subset (sqlite.org/compile.html#rcmd). THREADSAFE=1
# (serialized) even though every g2m connection is used under its own mutex:
# the cost is negligible and it keeps a misuse from corrupting memory.
target_compile_definitions(g2m_dep_sqlite3 PRIVATE
  SQLITE_THREADSAFE=1
  SQLITE_DQS=0
  SQLITE_DEFAULT_MEMSTATUS=0
  SQLITE_DEFAULT_WAL_SYNCHRONOUS=1
  SQLITE_LIKE_DOESNT_MATCH_BLOBS
  SQLITE_OMIT_DEPRECATED
  SQLITE_OMIT_LOAD_EXTENSION
  SQLITE_OMIT_SHARED_CACHE
  SQLITE_OMIT_PROGRESS_CALLBACK
  SQLITE_USE_ALLOCA
)
find_package(Threads REQUIRED)
target_link_libraries(g2m_dep_sqlite3 PUBLIC Threads::Threads)
if(UNIX)
  target_link_libraries(g2m_dep_sqlite3 PUBLIC m)
endif()
set_target_properties(g2m_dep_sqlite3 PROPERTIES FOLDER third_party POSITION_INDEPENDENT_CODE ON)

# --- nlohmann_json v3.12.0 (MIT), header-only. The release's json.tar.xz
# (top directory json/). Its include dir is SYSTEM: v3.12.0 trips
# -Wtautological-constant-out-of-range-compare in a Debug build.
FetchContent_Declare(
  g2m_json
  URL      https://github.com/nlohmann/json/releases/download/v3.12.0/json.tar.xz
  URL_HASH SHA256=42f6e95cad6ec532fd372391373363b62a14af6d771056dbfc86160e6dfff7aa
  SOURCE_SUBDIR g2m_no_cmake
)
FetchContent_MakeAvailable(g2m_json)
add_library(g2m_dep_json INTERFACE)
target_include_directories(g2m_dep_json SYSTEM INTERFACE "${g2m_json_SOURCE_DIR}/single_include")
