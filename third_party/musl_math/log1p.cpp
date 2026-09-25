// third_party/musl_math/log1p.cpp - log1p(x) = log(1 + x).
// Ported from musl libm v1.2.5: src/math/log1p.c (origin: FreeBSD
// s_log1p.c). See common.h for the porting rules and THIRD_PARTY_NOTICES.md
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

constexpr double ln2_hi = 6.93147180369123816490e-01; /* 3fe62e42 fee00000 */
constexpr double ln2_lo = 1.90821492927058770002e-10; /* 3dea39ef 35793c76 */
constexpr double Lg1 = 6.666666666666735130e-01;      /* 3FE55555 55555593 */
constexpr double Lg2 = 3.999999999940941908e-01;      /* 3FD99999 9997FA04 */
constexpr double Lg3 = 2.857142874366239149e-01;      /* 3FD24924 94229359 */
constexpr double Lg4 = 2.222219843214978396e-01;      /* 3FCC71C5 1D8E78AF */
constexpr double Lg5 = 1.818357216161805012e-01;      /* 3FC74664 96CB03DE */
constexpr double Lg6 = 1.531383769920937332e-01;      /* 3FC39A09 D078C69F */
constexpr double Lg7 = 1.479819860511658591e-01;      /* 3FC2F112 DF3E5244 */

static_assert(std::bit_cast<uint64_t>(ln2_hi) == 0x3fe62e42fee00000ull);
static_assert(std::bit_cast<uint64_t>(ln2_lo) == 0x3dea39ef35793c76ull);
static_assert(std::bit_cast<uint64_t>(Lg1) == 0x3FE5555555555593ull);
static_assert(std::bit_cast<uint64_t>(Lg2) == 0x3FD999999997FA04ull);
static_assert(std::bit_cast<uint64_t>(Lg3) == 0x3FD2492494229359ull);
static_assert(std::bit_cast<uint64_t>(Lg4) == 0x3FCC71C51D8E78AFull);
static_assert(std::bit_cast<uint64_t>(Lg5) == 0x3FC7466496CB03DEull);
static_assert(std::bit_cast<uint64_t>(Lg6) == 0x3FC39A09D078C69Full);
static_assert(std::bit_cast<uint64_t>(Lg7) == 0x3FC2F112DF3E5244ull);

} // namespace

double log1p(double x) {
    double hfsq, f = 0.0, c = 0.0, s, z, R, w, t1, t2, dk;
    uint64_t ui = asuint64(x);
    uint32_t hx, hu;
    int k;

    hx = static_cast<uint32_t>(ui >> 32);
    k = 1;
    if (hx < 0x3fda827a || hx >> 31) { /* 1+x < sqrt(2)+ */
        if (hx >= 0xbff00000) {        /* x <= -1.0 */
            if (x == -1) return math_divzero(1); /* log1p(-1) = -inf */
            return math_invalid(x);              /* log1p(x<-1) = NaN */
        }
        if (hx << 1 < 0x3ca00000u << 1) { /* |x| < 2**-53 */
            return x;
        }
        if (hx <= 0xbfd2bec4) { /* sqrt(2)/2- <= 1+x < sqrt(2)+ */
            k = 0;
            c = 0;
            f = x;
        }
    } else if (hx >= 0x7ff00000) {
        return x;
    }
    if (k) {
        double uf = 1 + x;
        uint64_t uu = asuint64(uf);
        hu = static_cast<uint32_t>(uu >> 32);
        hu += 0x3ff00000 - 0x3fe6a09e;
        k = static_cast<int>(hu >> 20) - 0x3ff;
        /* correction term ~ log(1+x)-log(u), avoid underflow in c/u */
        if (k < 54) {
            c = k >= 2 ? 1 - (uf - x) : x - (uf - 1);
            c /= uf;
        } else {
            c = 0;
        }
        /* reduce u into [sqrt(2)/2, sqrt(2)] */
        hu = (hu & 0x000fffff) + 0x3fe6a09e;
        uu = static_cast<uint64_t>(hu) << 32 | (uu & 0xffffffff);
        uf = asdouble(uu);
        f = uf - 1;
    }
    hfsq = 0.5 * f * f;
    s = f / (2.0 + f);
    z = s * s;
    w = z * z;
    t1 = w * (Lg2 + w * (Lg4 + w * Lg6));
    t2 = z * (Lg1 + w * (Lg3 + w * (Lg5 + w * Lg7)));
    R = t2 + t1;
    dk = k;
    return s * (hfsq + R) + (dk * ln2_lo + c) - hfsq + f + dk * ln2_hi;
}

} // namespace g2m::musl
