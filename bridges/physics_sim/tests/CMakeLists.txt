# bridges/physics_sim/tests - g2m_bridge_tests (PLAN.md R2.2 section 1c/4b),
# built only as part of the bridge-release preset/leg (tools/ci.ps1's
# `-Only bridge-release`) - g2m_ps_bridge only exists when the parent build
# supplied G2M_PS_SOURCE_DIR, so this whole subdirectory is only ever reached
# via the root CMakeLists.txt's `if(TARGET ps_core)` guard.
#
# Deliberately does NOT link tests/mesh/alloc_counter.cpp or
# tests/phys/alloc_counter.cpp: both override the global operator new/delete
# for their own executable, which would clash with ps_core's own operator
# new/delete override (core/src/jobs/debug_alloc.cpp, linked in transitively
# via g2m_ps_bridge -> ps_core) - see the plan's own 1c note ("Don't link
# tests/mesh/alloc_counter.cpp into the bridge tests: it clashes with
# ps_core's operator new").
add_executable(g2m_bridge_tests
  test_smoke.cpp
  test_g2m_terrain_source.cpp
  test_render_vs_jolt.cpp
  test_residency.cpp
  test_determinism.cpp
)
# g2m_mesh: build_chunk for the render-vs-Jolt comparison. g2m_fixture_release
# (tests/support): the PKT DGM1 fixture release for the fixture/residency
# tests - it links Catch2 itself (REQUIRE inside make_fixture_release).
target_link_libraries(g2m_bridge_tests PRIVATE
  g2m_ps_bridge g2m_mesh g2m_fixture_release g2m_test_catch_main g2m_warnings g2m_options)

include(Catch)
catch_discover_tests(g2m_bridge_tests ADD_TAGS_AS_LABELS DISCOVERY_MODE PRE_TEST)
