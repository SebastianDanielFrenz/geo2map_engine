// third_party/musl_math/musl_math.h - internal declarations of g2m's
// deterministic double-precision libm port (PLAN.md section 2.4). Ported
// from musl libm v1.2.5 (MIT; see THIRD_PARTY_NOTICES.md for the licence
// text and exact source commit). Bit-identical across every
// platform/compiler this project builds for, because it never calls the
// platform libm.
//
// Library code includes g2m/core/det_math.h and calls g2m::det::sin/... (a
// thin forwarder to this namespace, src/core/det_math.cpp). This header
// exists so white-box tests can call the ported functions directly.
#pragma once

namespace g2m::musl {

double sin(double x);
double cos(double x);
void sincos(double x, double* s, double* c);
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

} // namespace g2m::musl
