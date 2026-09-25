// g2m/core/det_math.h - deterministic double-precision math (PLAN.md 2.4).
//
// Library code must never call the platform libm's transcendentals
// (std::sin, std::atan2, std::sinh, std::hypot, ...): MSVC's ucrtbase and
// glibc round them differently, so the same input would give different
// bits on Windows and Linux, and a client-derived tile would differ from
// the server's. Use these instead. They are a port of musl libm v1.2.5
// (MIT, third_party/musl_math) and never call the platform libm, so they
// are bit-identical on every compiler/OS this project builds for. A CI grep
// (tools/ci.ps1) enforces this for include/ and src/.
//
// Exempt (IEEE 754 mandates one bit-exact result, so no library rounding
// choice exists): sqrt, fabs, floor, ceil, trunc, fmod, remainder,
// copysign, scalbn. std:: versions of those may be called directly;
// det::sqrt is provided only for symmetry.
#pragma once

#include <cmath>

namespace g2m::det {

double sin(double x);
double cos(double x);
void sincos(double x, double& s, double& c);
double tan(double x);
double asin(double x);
double acos(double x);
double atan(double x);
double atan2(double y, double x);
double exp(double x);
double expm1(double x);
double log(double x);
double log1p(double x);
double pow(double x, double y);
double sinh(double x);
double cosh(double x);
double tanh(double x);
double asinh(double x);
double atanh(double x);
double hypot(double x, double y);

inline double sqrt(double x) { return std::sqrt(x); }

inline constexpr double kPi = 3.141592653589793238462643383279502884;

} // namespace g2m::det
