# Deterministic libm port (musl v1.2.5, MIT; PLAN.md 2.4). Built as an
# OBJECT library whose objects are compiled into g2m_core (src/CMakeLists.txt),
# so consumers link one archive. Library code reaches it only through
# include/g2m/core/det_math.h; tests may include musl_math.h directly.
add_library(g2m_musl_math OBJECT
  exp.cpp
  exp_data.cpp
  expm1.cpp
  hyperbolic.cpp
  hypot.cpp
  inverse_trig.cpp
  log.cpp
  log1p.cpp
  log_data.cpp
  pow.cpp
  pow_data.cpp
  rem_pio2.cpp
  trig.cpp
  trig_kernel.cpp
)
target_include_directories(g2m_musl_math INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(g2m_musl_math PRIVATE g2m_warnings g2m_options)
