// g2m/core/geo/angle.h - angles in degrees with exact argument reduction.
// Same semantics as GeographicLib's Math::sincosd/atan2d/AngNormalize/
// AngDiff (Karney), implemented on g2m::det so results are bit-identical on
// every platform. Degrees are the public unit throughout g2m::geo: a degree
// argument can be reduced to [-45, 45] exactly before conversion to radians,
// which a radian argument cannot.
#pragma once

namespace g2m::geo {

// sin and cos of x degrees. The reduction to [-45, 45] is exact; exact
// values at multiples of 30 and 45 degrees. sin(-0) = -0, cos(+-90) = +0.
void sincosd(double x, double& s, double& c);

// atan2(y, x) in degrees, in [-180, 180], with the quadrant handled exactly
// (the underlying det::atan2 only ever sees an angle in [-45, 45]).
double atan2d(double y, double x);

// atan(x) in degrees.
double atand(double x);

// x reduced to [-180, 180] (180 keeps the sign of x). Exact.
double ang_normalize(double x);

// y - x reduced to [-180, 180] via an error-free two-sum (as
// GeographicLib): the result is the correctly rounded difference, and *e
// (if non-null) receives the rounding error, so y - x == result + *e exactly.
double ang_diff(double x, double y, double* e = nullptr);

// Degree/radian conversion factor pi/180 (correctly rounded double).
inline constexpr double kDegree = 3.141592653589793238462643383279502884 / 180.0;

} // namespace g2m::geo
