// third_party/musl_math/hypot.cpp - hypot(x, y) = sqrt(x*x + y*y) without
// undue overflow/underflow.
// Ported from musl libm v1.2.5: src/math/hypot.c (a musl-original rewrite;
// musl's COPYRIGHT covers it under its MIT licence). See common.h for the
// porting rules and THIRD_PARTY_NOTICES.md for the licence.
//
// Only the FLT_EVAL_METHOD == 0 branch is ported (SPLIT = 0x1p27 + 1,
// Dekker's exact split): x86-64/SSE2 never evaluates in long double. The
// split-product arithmetic below relies on FMA contraction being OFF (which
// CMakeLists.txt guarantees); a contracted `xh*xh - hi` would change bits.

#include "musl_math.h"
#include "common.h"

#include <cmath> // std::sqrt only (IEEE-exact, see common.h file comment)

namespace g2m::musl {
namespace {

constexpr double SPLIT = 0x1p27 + 1;

void sq(double* hi, double* lo, double x) {
    double xh, xl, xc;

    xc = x * SPLIT;
    xh = x - xc + xc;
    xl = x - xh;
    *hi = x * x;
    *lo = xh * xh - *hi + 2 * xh * xl + xl * xl;
}

} // namespace

double hypot(double x, double y) {
    uint64_t uxi = asuint64(x) & (~0ull >> 1);
    uint64_t uyi = asuint64(y) & (~0ull >> 1);
    int ex, ey;
    double hx, lx, hy, ly, z;

    /* arrange |x| >= |y| */
    if (uxi < uyi) {
        const uint64_t ut = uxi;
        uxi = uyi;
        uyi = ut;
    }

    /* special cases */
    ex = static_cast<int>(uxi >> 52);
    ey = static_cast<int>(uyi >> 52);
    x = asdouble(uxi);
    y = asdouble(uyi);
    /* note: hypot(inf,nan) == inf */
    if (ey == 0x7ff) return y;
    if (ex == 0x7ff || uyi == 0) return x;
    /* note: hypot(x,y) ~= x + y*y/x/2 with inexact for small y/x */
    /* 64 difference is enough for ld80 double_t */
    if (ex - ey > 64) return x + y;

    /* precise sqrt argument in nearest rounding mode without overflow */
    /* xh*xh must not overflow and xl*xl must not underflow in sq */
    z = 1;
    if (ex > 0x3ff + 510) {
        z = 0x1p700;
        x *= 0x1p-700;
        y *= 0x1p-700;
    } else if (ey < 0x3ff - 450) {
        z = 0x1p-700;
        x *= 0x1p700;
        y *= 0x1p700;
    }
    sq(&hx, &lx, x);
    sq(&hy, &ly, y);
    return z * std::sqrt(ly + lx + hy + hx);
}

} // namespace g2m::musl
