// src/core/geo/angle.cpp - see g2m/core/geo/angle.h. Mirrors GeographicLib
// v2 Math::sincosd/atan2d/AngNormalize/AngDiff (MIT, Karney), with the
// argument reduction done by std::fmod (exact) instead of std::remquo.

#include "g2m/core/geo/angle.h"

#include "g2m/core/det_math.h"

#include <cmath>
#include <utility>

namespace g2m::geo {
namespace {

constexpr double kQd = 90.0;
constexpr double kHd = 180.0;
constexpr double kTd = 360.0;

// Error-free transformation: s + t == u + v exactly (Knuth two-sum).
double two_sum(double u, double v, double& t) {
    const double s = u + v;
    double up = s - v;
    double vpp = s - up;
    up -= u;
    vpp -= v;
    // s == 0 keeps the sign of s for t (GeographicLib: t = s != 0 ? 0-(up+vpp) : s).
    t = s != 0 ? 0.0 - (up + vpp) : s;
    return s;
}

} // namespace

void sincosd(double x, double& sinx, double& cosx) {
    // r = x mod 360 (exact), then q = nearest multiple of 90, d = r - 90q.
    // d is exact: r and 90q are both multiples of ulp(r) (ulp(r) <= ulp(360)
    // < 1 divides every integer) and |d| <= 45 < |r| whenever q != 0.
    const double r = std::fmod(x, kTd);
    const double qf = std::floor(r / kQd + 0.5);
    const double d = r - kQd * qf;
    const int q = static_cast<int>(qf); // in [-4, 4] for finite x
    const double rad = d * kDegree;
    double s = 0;
    double c = 0;
    det::sincos(rad, s, c);
    if (2 * std::fabs(d) == kQd) {
        c = std::sqrt(0.5);
        s = std::copysign(c, rad);
    } else if (3 * std::fabs(d) == kQd) {
        c = std::sqrt(3.0) / 2;
        s = std::copysign(0.5, rad);
    }
    switch (static_cast<unsigned>(q) & 3U) {
    case 0U: sinx = s; cosx = c; break;
    case 1U: sinx = c; cosx = -s; break;
    case 2U: sinx = -s; cosx = -c; break;
    default: sinx = -c; cosx = s; break;
    }
    cosx += 0.0; // -0 -> +0
    if (sinx == 0) sinx = std::copysign(sinx, x);
}

double atan2d(double y, double x) {
    // Reduce to x >= |y| so det::atan2 sees an angle in [-45, 45], then add
    // the quadrant back exactly in degrees.
    int q = 0;
    if (std::fabs(y) > std::fabs(x)) {
        std::swap(x, y);
        q = 2;
    }
    if (std::signbit(x)) {
        x = -x;
        ++q;
    }
    double ang = det::atan2(y, x) / kDegree;
    switch (q) {
    case 1: ang = std::copysign(kHd, y) - ang; break;
    case 2: ang = kQd - ang; break;
    case 3: ang = -kQd + ang; break;
    default: break;
    }
    return ang;
}

double atand(double x) { return atan2d(x, 1.0); }

double ang_normalize(double x) {
    const double y = std::remainder(x, kTd);
    return std::fabs(y) == kHd ? std::copysign(kHd, x) : y;
}

double ang_diff(double x, double y, double* e) {
    double t = 0;
    double d = two_sum(std::remainder(-x, kTd), std::remainder(y, kTd), t);
    d = two_sum(std::remainder(d, kTd), t, t);
    if (d == 0 || std::fabs(d) == kHd) d = std::copysign(d, t == 0 ? y - x : -t);
    if (e != nullptr) *e = t;
    return d;
}

} // namespace g2m::geo
