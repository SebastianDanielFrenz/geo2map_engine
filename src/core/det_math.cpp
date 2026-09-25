// src/core/det_math.cpp - thin forwarder from the public g2m::det API to
// the musl port in third_party/musl_math (PLAN.md 2.4).

#include "g2m/core/det_math.h"

#include "musl_math.h"

namespace g2m::det {

double sin(double x) { return musl::sin(x); }
double cos(double x) { return musl::cos(x); }
void sincos(double x, double& s, double& c) { musl::sincos(x, &s, &c); }
double tan(double x) { return musl::tan(x); }
double asin(double x) { return musl::asin(x); }
double acos(double x) { return musl::acos(x); }
double atan(double x) { return musl::atan(x); }
double atan2(double y, double x) { return musl::atan2(y, x); }
double exp(double x) { return musl::exp(x); }
double expm1(double x) { return musl::expm1(x); }
double log(double x) { return musl::log(x); }
double log1p(double x) { return musl::log1p(x); }
double pow(double x, double y) { return musl::pow(x, y); }
double sinh(double x) { return musl::sinh(x); }
double cosh(double x) { return musl::cosh(x); }
double tanh(double x) { return musl::tanh(x); }
double asinh(double x) { return musl::asinh(x); }
double atanh(double x) { return musl::atanh(x); }
double hypot(double x, double y) { return musl::hypot(x, y); }

} // namespace g2m::det
