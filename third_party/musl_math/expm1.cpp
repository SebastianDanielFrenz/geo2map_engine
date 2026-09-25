// third_party/musl_math/expm1.cpp - expm1(x) = exp(x) - 1.
// Ported from musl libm v1.2.5: src/math/expm1.c (origin: FreeBSD
// s_expm1.c). See common.h for the porting rules and THIRD_PARTY_NOTICES.md
// for the licence. Every constant carries its IEEE-754 bit pattern from the
// upstream comment; a static_assert checks the decimal literal against it.
/*
 * ====================================================
 * Copyright (C) 1993 by Sun Microsystems, Inc. All rights reserved.
 *
 * Developed at SunPro, a Sun Microsystems, Inc. business.
 * Permission to use, copy, modify, and distribute this
 * software is freely granted, provided that this notice
 * is preserved.
 * ====================================================
 */

#include "musl_math.h"
#include "common.h"

namespace g2m::musl {
namespace {

constexpr double o_threshold = 7.09782712893383973096e+02; /* 0x40862E42, 0xFEFA39EF */
constexpr double ln2_hi = 6.93147180369123816490e-01;      /* 0x3fe62e42, 0xfee00000 */
constexpr double ln2_lo = 1.90821492927058770002e-10;      /* 0x3dea39ef, 0x35793c76 */
constexpr double invln2 = 1.44269504088896338700e+00;      /* 0x3ff71547, 0x652b82fe */
/* Scaled Q's: Qn_here = 2**n * Qn_above, for R(2*z) where z = hxs = x*x/2: */
constexpr double Q1 = -3.33333333333331316428e-02; /* BFA11111 111110F4 */
constexpr double Q2 = 1.58730158725481460165e-03;  /* 3F5A01A0 19FE5585 */
constexpr double Q3 = -7.93650757867487942473e-05; /* BF14CE19 9EAADBB7 */
constexpr double Q4 = 4.00821782732936239552e-06;  /* 3ED0CFCA 86E65239 */
constexpr double Q5 = -2.01099218183624371326e-07; /* BE8AFDB7 6E09C32D */

static_assert(std::bit_cast<uint64_t>(o_threshold) == 0x40862E42FEFA39EFull);
static_assert(std::bit_cast<uint64_t>(ln2_hi) == 0x3fe62e42fee00000ull);
static_assert(std::bit_cast<uint64_t>(ln2_lo) == 0x3dea39ef35793c76ull);
static_assert(std::bit_cast<uint64_t>(invln2) == 0x3ff71547652b82feull);
static_assert(std::bit_cast<uint64_t>(Q1) == 0xBFA11111111110F4ull);
static_assert(std::bit_cast<uint64_t>(Q2) == 0x3F5A01A019FE5585ull);
static_assert(std::bit_cast<uint64_t>(Q3) == 0xBF14CE199EAADBB7ull);
static_assert(std::bit_cast<uint64_t>(Q4) == 0x3ED0CFCA86E65239ull);
static_assert(std::bit_cast<uint64_t>(Q5) == 0xBE8AFDB76E09C32Dull);

} // namespace

double expm1(double x) {
    double y, hi, lo, c = 0.0, t, e, hxs, hfx, r1, twopk;
    uint64_t ui = asuint64(x);
    const uint32_t hx = static_cast<uint32_t>(ui >> 32) & 0x7fffffff;
    const int sign = static_cast<int>(ui >> 63);
    int k;

    /* filter out huge and non-finite argument */
    if (hx >= 0x4043687A) { /* if |x|>=56*ln2 */
        if (std::isnan(x)) return x;
        if (sign) return -1;
        if (x > o_threshold) {
            x *= 0x1p1023;
            return x;
        }
    }

    /* argument reduction */
    if (hx > 0x3fd62e42) {     /* if  |x| > 0.5 ln2 */
        if (hx < 0x3FF0A2B2) { /* and |x| < 1.5 ln2 */
            if (!sign) {
                hi = x - ln2_hi;
                lo = ln2_lo;
                k = 1;
            } else {
                hi = x + ln2_hi;
                lo = -ln2_lo;
                k = -1;
            }
        } else {
            k = static_cast<int>(invln2 * x + (sign ? -0.5 : 0.5));
            t = k;
            hi = x - t * ln2_hi; /* t*ln2_hi is exact here */
            lo = t * ln2_lo;
        }
        x = hi - lo;
        c = (hi - x) - lo;
    } else if (hx < 0x3c900000) { /* |x| < 2**-54, return x */
        return x;
    } else {
        k = 0;
    }

    /* x is now in primary range */
    hfx = 0.5 * x;
    hxs = x * hfx;
    r1 = 1.0 + hxs * (Q1 + hxs * (Q2 + hxs * (Q3 + hxs * (Q4 + hxs * Q5))));
    t = 3.0 - r1 * hfx;
    e = hxs * ((r1 - t) / (6.0 - x * t));
    if (k == 0) /* c is 0 */
        return x - (x * e - hxs);
    e = x * (e - c) - c;
    e -= hxs;
    /* exp(x) ~ 2^k (x_reduced - e + 1) */
    if (k == -1) return 0.5 * (x - e) - 0.5;
    if (k == 1) {
        if (x < -0.25) return -2.0 * (e - (x + 0.5));
        return 1.0 + 2.0 * (x - e);
    }
    twopk = asdouble(static_cast<uint64_t>(0x3ff + k) << 52); /* 2^k */
    if (k < 0 || k > 56) { /* suffice to return exp(x)-1 */
        y = x - e + 1.0;
        if (k == 1024)
            y = y * 2.0 * 0x1p1023;
        else
            y = y * twopk;
        return y - 1.0;
    }
    const double twomk = asdouble(static_cast<uint64_t>(0x3ff - k) << 52); /* 2^-k */
    if (k < 20)
        y = (x - e + (1 - twomk)) * twopk;
    else
        y = (x - (e + twomk) + 1) * twopk;
    return y;
}

} // namespace g2m::musl
