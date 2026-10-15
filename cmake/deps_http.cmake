# cmake/deps_http.cmake - FetchContent for g2m_http (G6, PLAN.md 3.7: the HTTP
# transport and the sidecar/hosted server front end). Owner-approved list:
# PLAN.md 13 ("cpp-httplib | MIT | one header | HTTP server (sidecar/hosted)").
#
# | Dependency  | Pin                                                | Licence | Target           | Used by             |
# |-------------|----------------------------------------------------|---------|------------------|---------------------|
# | cpp-httplib | v0.58.0 (commit 4f3f9ef19be83ae97a5d9a059432dc00e445b7ab) | MIT     | g2m_dep_httplib  | g2m_http (PRIVATE)  |
#
# Fetched by git at the tag's exact commit (the GitHub archive URL is not
# reachable from every build host; the commit id pins the content just as a
# tarball hash would). Source only: upstream's own CMakeLists (tests,
# OpenSSL/zlib/brotli detection, install rules) is never added. Plain HTTP
# only - no CPPHTTPLIB_OPENSSL_SUPPORT / ZLIB / BROTLI: TLS and compression
# are a reverse proxy's job (PLAN.md 3.7, G7), and tiles are already zstd.
include(FetchContent)
FetchContent_Declare(
  g2m_httplib
  GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
  GIT_TAG        4f3f9ef19be83ae97a5d9a059432dc00e445b7ab # v0.58.0
  SOURCE_SUBDIR  g2m_no_cmake   # populate only; never add_subdirectory upstream's build
)
FetchContent_MakeAvailable(g2m_httplib)

if(NOT EXISTS "${g2m_httplib_SOURCE_DIR}/httplib.h")
  message(FATAL_ERROR "cpp-httplib: httplib.h missing in ${g2m_httplib_SOURCE_DIR}")
endif()

add_library(g2m_dep_httplib INTERFACE)
target_include_directories(g2m_dep_httplib SYSTEM INTERFACE "${g2m_httplib_SOURCE_DIR}")
find_package(Threads REQUIRED)
target_link_libraries(g2m_dep_httplib INTERFACE Threads::Threads)
if(WIN32)
  target_link_libraries(g2m_dep_httplib INTERFACE ws2_32)
endif()
