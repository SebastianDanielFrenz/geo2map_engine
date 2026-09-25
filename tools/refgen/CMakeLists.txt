# tools/refgen - reference-value generator (G2M_BUILD_REFGEN=ON only).
#
# Fetches GeographicLib (MIT) and writes testdata/geodesy/*.txt, the values
# tests/unit/test_tm_reference.cpp checks g2m's own geodesy against.
# GeographicLib is used ONLY here: never linked into g2m_core or any test.
#
# Built with GEOGRAPHICLIB_PRECISION=3 (Math::real = long double), which
# also selects GeographicLib's 8th-order Krueger series, so the reference is
# both more precise and of higher order than g2m's 6th-order double
# implementation. Needs an 80-bit long double: build it on Linux x86-64
# (WSL), never with MSVC/clang-cl, where long double is plain double
# (main.cpp static_asserts this).
#
#   cmake -S . -B ~/build/geo2map_engine/refgen -G Ninja -DCMAKE_BUILD_TYPE=Release \
#         -DCMAKE_CXX_COMPILER=clang++ -DG2M_BUILD_TESTS=OFF -DG2M_BUILD_REFGEN=ON
#   cmake --build ~/build/geo2map_engine/refgen --target g2m_refgen
#   ~/build/geo2map_engine/refgen/tools/refgen/g2m_refgen testdata/geodesy

set(GEOGRAPHICLIB_PRECISION 3 CACHE STRING "GeographicLib: long double" FORCE)
set(BUILD_SHARED_LIBS OFF)
set(BUILD_DOCUMENTATION OFF CACHE BOOL "" FORCE)
set(BUILD_MANPAGES OFF CACHE BOOL "" FORCE)
set(GEOGRAPHICLIB_LIB_TYPE STATIC CACHE STRING "" FORCE)

FetchContent_Declare(
  GeographicLib
  GIT_REPOSITORY https://github.com/geographiclib/geographiclib.git
  GIT_TAG        v2.7
)
FetchContent_MakeAvailable(GeographicLib)

add_executable(g2m_refgen main.cpp)
target_include_directories(g2m_refgen PRIVATE ${PROJECT_SOURCE_DIR}/tests/support)
if(TARGET GeographicLib::GeographicLib)
  target_link_libraries(g2m_refgen PRIVATE GeographicLib::GeographicLib)
else()
  target_link_libraries(g2m_refgen PRIVATE GeographicLib_STATIC)
endif()
