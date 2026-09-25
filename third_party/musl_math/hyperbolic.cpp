// third_party/musl_math/hyperbolic.cpp - sinh/cosh/tanh/asinh/atanh.
// Ported from musl libm v1.2.5: src/math/sinh.c, cosh.c, tanh.c, asinh.c,
// atanh.c and src/math/__expo2.c (all musl-original rewrites; musl's
// COPYRIGHT covers them under its MIT licence). See common.h for the
// porting rules and THIRD_PARTY_NOTICES.md for the licence.
//
// Transverse Mercator (PLAN.md section 4) needs sinh, cosh, asinh and atanh;
// tanh is ported alongside because it shares expm1.

#include "musl_math.h"
#include "common.h"

#include <cmath> // std::sqrt only (IEEE-exact, see common.h file comment)

namespace g2m::musl {
namespace {

// __expo2.c: exp(x)/2 for x >= log(DBL_MAX), slightly better than
// 0.5*exp(x/2)*exp(x/2). k is such that k*ln2 has minimal relative error and
// x - kln2 > log(DBL_MIN).
constexpr int k_expo2 = 2043;
constexpr double kln2 = 0x1.62066151add8bp+10;

double expo2(double x, double sign) {
    /* note that k is odd and scale*scale overflows */
    const double scale = insert_words(static_cast<uint32_t>(0x3ff + k_expo2 / 2) << 20, 0);
    /* exp(x - k ln2) * 2**(k-1) */
    /* in directed rounding correct sign before rounding or overflow is important */
    return exp(x - kln2) * (sign * scale) * scale;
}

constexpr uint64_t abs_mask = ~0ull >> 1; // musl's (uint64_t)-1/2

} // namespace

/* sinh(x) = (exp(x) - 1/exp(x))/2
 *         = (exp(x)-1 + (exp(x)-1)/exp(x))/2
 *         = x + x^3/6 + o(x^5)
 */
double sinh(double x) {
    uint64_t ui = asuint64(x);
    double t, h, absx;

    h = 0.5;
    if (ui >> 63) h = -h;
    /* |x| */
    ui &= abs_mask;
    absx = asdouble(ui);
    const uint32_t w = static_cast<uint32_t>(ui >> 32);

    /* |x| < log(DBL_MAX) */
    if (w < 0x40862e42) {
        t = expm1(absx);
        if (w < 0x3ff00000) {
            if (w < 0x3ff00000 - (26 << 20))
                /* note: inexact and underflow are raised by expm1 */
                /* note: this branch avoids spurious underflow */
                return x;
            return h * (2 * t - t * t / (t + 1));
        }
        /* note: |x|>log(0x1p26)+eps could be just h*exp(x) */
        return h * (t + t / (t + 1));
    }

    /* |x| > log(DBL_MAX) or nan */
    /* note: the result is stored to handle overflow */
    t = expo2(absx, 2 * h);
    return t;
}

double cosh(double x) {
    uint64_t ui = asuint64(x);
    double t;

    /* |x| */
    ui &= abs_mask;
    x = asdouble(ui);
    const uint32_t w = static_cast<uint32_t>(ui >> 32);

    /* |x| < log(2) */
    if (w < 0x3fe62e42) {
        if (w < 0x3ff00000 - (26 << 20)) {
            /* musl raises inexact here (FORCE_EVAL); value unaffected */
            return 1;
        }
        t = expm1(x);
        return 1 + t * t / (2 * (1 + t));
    }

    /* |x| < log(DBL_MAX) */
    if (w < 0x40862e42) {
        t = exp(x);
        /* note: if x>log(0x1p26) then the 1/t is not needed */
        return 0.5 * (t + 1 / t);
    }

    /* |x| > log(DBL_MAX) or nan */
    /* note: the result is stored to handle overflow */
    t = expo2(x, 1.0);
    return t;
}

/* tanh(x) = (exp(x) - exp(-x))/(exp(x) + exp(-x))
 *         = (exp(2*x) - 1)/(exp(2*x) - 1 + 2)
 *         = (1 - exp(-2*x))/(exp(-2*x) - 1 + 2)
 */
double tanh(double x) {
    uint64_t ui = asuint64(x);
    double t;

    /* x = |x| */
    const int sign = static_cast<int>(ui >> 63);
    ui &= abs_mask;
    x = asdouble(ui);
    const uint32_t w = static_cast<uint32_t>(ui >> 32);

    if (w > 0x3fe193ea) {
        /* |x| > log(3)/2 ~= 0.5493 or nan */
        if (w > 0x40340000) {
            /* |x| > 20 or nan */
            /* note: this branch avoids raising overflow */
            // musl: t = 1 - 0/x; (1 for finite/inf x, NaN for NaN x)
            t = 1 - fp_barrier(0.0) / x;
        } else {
            t = expm1(2 * x);
            t = 1 - 2 / (t + 2);
        }
    } else if (w > 0x3fd058ae) {
        /* |x| > log(5/3)/2 ~= 0.2554 */
        t = expm1(2 * x);
        t = t / (t + 2);
    } else if (w >= 0x00100000) {
        /* |x| >= 0x1p-1022, up to 2ulp error in [0.1,0.2554] */
        t = expm1(-2 * x);
        t = -t / (t + 2);
    } else {
        /* |x| is subnormal */
        t = x;
    }
    return sign ? -t : t;
}

/* asinh(x) = sign(x)*log(|x|+sqrt(x*x+1)) ~= x - x^3/6 + o(x^5) */
double asinh(double x) {
    uint64_t ui = asuint64(x);
    const unsigned e = static_cast<unsigned>(ui >> 52) & 0x7ff;
    const unsigned s = static_cast<unsigned>(ui >> 63);

    /* |x| */
    ui &= abs_mask;
    x = asdouble(ui);

    if (e >= 0x3ff + 26) {
        /* |x| >= 0x1p26 or inf or nan */
        x = log(x) + 0.693147180559945309417232121458176568;
    } else if (e >= 0x3ff + 1) {
        /* |x| >= 2 */
        x = log(2 * x + 1 / (std::sqrt(x * x + 1) + x));
    } else if (e >= 0x3ff - 26) {
        /* |x| >= 0x1p-26 */
        x = log1p(x + x * x / (std::sqrt(x * x + 1) + 1));
    } else {
        /* |x| < 0x1p-26: asinh(x) == x (musl only raises inexact here) */
    }
    return s ? -x : x;
}

/* atanh(x) = log((1+x)/(1-x))/2 = log1p(2x/(1-x))/2 ~= x + x^3/3 + o(x^5) */
double atanh(double x) {
    uint64_t ui = asuint64(x);
    const unsigned e = static_cast<unsigned>(ui >> 52) & 0x7ff;
    const unsigned s = static_cast<unsigned>(ui >> 63);
    double y;

    /* |x| */
    ui &= abs_mask;
    y = asdouble(ui);

    if (e < 0x3ff - 1) {
        if (e < 0x3ff - 32) {
            /* |x| < 2^-32: atanh(x) == x (musl only raises underflow here) */
        } else {
            /* |x| < 0.5, up to 1.7ulp error */
            y = 0.5 * log1p(2 * y + 2 * y * y / (1 - y));
        }
    } else {
        /* avoid overflow */
        y = 0.5 * log1p(2 * (y / (1 - y)));
    }
    return s ? -y : y;
}

} // namespace g2m::musl
