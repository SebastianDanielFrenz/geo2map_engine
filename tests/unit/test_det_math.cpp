// tests/unit/test_det_math.cpp - accuracy of g2m::det (the musl port)
// against the platform libm, and special values.
//
// The platform libm is the only reference available without another
// dependency. musl's algorithms are well under 1 ulp from the true value;
// ucrtbase and glibc are each within about 1 ulp too but round differently
// (the reason the port exists), so the port may legitimately sit up to 2
// ulp from one platform's libm while being correct: the bound is 2 ulp.
// Bit-identity across platforms is checked separately, by the golden hashes
// (tests/golden).

#include "g2m/core/det_math.h"

#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace {

using g2m_test::SplitMix64;

// Consecutive doubles map to consecutive integers, so the difference is the
// exact ulp distance, also across zero.
std::int64_t ordered(double d) {
    std::uint64_t u = std::bit_cast<std::uint64_t>(d);
    u = (u & 0x8000000000000000ull) ? ~u : (u | 0x8000000000000000ull);
    return std::bit_cast<std::int64_t>(u);
}

double ulp_distance(double a, double b) {
    if (std::isnan(a) || std::isnan(b)) return (std::isnan(a) && std::isnan(b)) ? 0.0 : 1e300;
    if (a == b) return 0.0; // also +0 vs -0
    const std::int64_t d = ordered(a) - ordered(b);
    return static_cast<double>(d < 0 ? -d : d);
}

constexpr double kUlpBound = 2.0;

// Worst ulp distance of f vs ref over n samples of x in [lo, hi).
template <class F, class R>
double worst_unary(F f, R ref, SplitMix64& rng, double lo, double hi, int n) {
    double worst = 0;
    for (int i = 0; i < n; ++i) {
        const double x = rng.uniform(lo, hi);
        worst = std::max(worst, ulp_distance(f(x), ref(x)));
    }
    return worst;
}

// Same with x = +-10^u, u uniform in [elo, ehi): log-uniform magnitudes.
template <class F, class R>
double worst_unary_log(F f, R ref, SplitMix64& rng, double elo, double ehi, bool both_signs, int n) {
    double worst = 0;
    for (int i = 0; i < n; ++i) {
        double x = std::pow(10.0, rng.uniform(elo, ehi)); // only an input generator
        if (both_signs && (rng.next() & 1)) x = -x;
        worst = std::max(worst, ulp_distance(f(x), ref(x)));
    }
    return worst;
}

void report(const char* name, double worst) {
    std::printf("det::%-6s worst %.0f ulp vs platform libm\n", name, worst);
}

} // namespace

TEST_CASE("det trig vs platform libm", "[math]") {
    SplitMix64 rng(1);
    double ws = 0, wc = 0, wt = 0;
    for (const auto& [lo, hi] : {std::pair{-4.0, 4.0}, {-1e3, 1e3}, {-1e8, 1e8}, {-1e300, 1e300}, {-1e-30, 1e-30}}) {
        ws = std::max(ws, worst_unary([](double x) { return g2m::det::sin(x); }, [](double x) { return std::sin(x); }, rng, lo, hi, 20000));
        wc = std::max(wc, worst_unary([](double x) { return g2m::det::cos(x); }, [](double x) { return std::cos(x); }, rng, lo, hi, 20000));
        wt = std::max(wt, worst_unary([](double x) { return g2m::det::tan(x); }, [](double x) { return std::tan(x); }, rng, lo, hi, 20000));
    }
    report("sin", ws);
    report("cos", wc);
    report("tan", wt);
    CHECK(ws <= kUlpBound);
    CHECK(wc <= kUlpBound);
    CHECK(wt <= kUlpBound);

    // sincos is bit-identical to separate sin/cos (same kernels).
    for (double x : {0.0, 0.7, 1.4, -2.3, 1e6, -1e10, 5e-320}) {
        double s = 0, c = 0;
        g2m::det::sincos(x, s, c);
        CHECK(std::bit_cast<std::uint64_t>(s) == std::bit_cast<std::uint64_t>(g2m::det::sin(x)));
        CHECK(std::bit_cast<std::uint64_t>(c) == std::bit_cast<std::uint64_t>(g2m::det::cos(x)));
    }
}

TEST_CASE("det inverse trig vs platform libm", "[math]") {
    SplitMix64 rng(2);
    const double wasin = worst_unary([](double x) { return g2m::det::asin(x); }, [](double x) { return std::asin(x); }, rng, -1.0, 1.0, 40000);
    const double wacos = worst_unary([](double x) { return g2m::det::acos(x); }, [](double x) { return std::acos(x); }, rng, -1.0, 1.0, 40000);
    const double watan = worst_unary_log([](double x) { return g2m::det::atan(x); }, [](double x) { return std::atan(x); }, rng, -20, 20, true, 40000);
    double watan2 = 0;
    for (int i = 0; i < 40000; ++i) {
        const double y = rng.uniform(-1e6, 1e6);
        const double x = rng.uniform(-1e6, 1e6);
        watan2 = std::max(watan2, ulp_distance(g2m::det::atan2(y, x), std::atan2(y, x)));
    }
    report("asin", wasin);
    report("acos", wacos);
    report("atan", watan);
    report("atan2", watan2);
    CHECK(wasin <= kUlpBound);
    CHECK(wacos <= kUlpBound);
    CHECK(watan <= kUlpBound);
    CHECK(watan2 <= kUlpBound);
}

TEST_CASE("det exp/log/pow family vs platform libm", "[math]") {
    SplitMix64 rng(3);
    const double wexp = worst_unary([](double x) { return g2m::det::exp(x); }, [](double x) { return std::exp(x); }, rng, -700.0, 700.0, 40000);
    double wexpm1 = worst_unary([](double x) { return g2m::det::expm1(x); }, [](double x) { return std::expm1(x); }, rng, -40.0, 700.0, 40000);
    wexpm1 = std::max(wexpm1, worst_unary_log([](double x) { return g2m::det::expm1(x); }, [](double x) { return std::expm1(x); }, rng, -300, 0.5, true, 40000));
    const double wlog = worst_unary_log([](double x) { return g2m::det::log(x); }, [](double x) { return std::log(x); }, rng, -300, 300, false, 40000);
    double wlog1p = worst_unary_log([](double x) { return g2m::det::log1p(x); }, [](double x) { return std::log1p(x); }, rng, -300, 300, false, 40000);
    wlog1p = std::max(wlog1p, worst_unary([](double x) { return g2m::det::log1p(x); }, [](double x) { return std::log1p(x); }, rng, -0.999999, 1.0, 40000));
    double wpow = 0;
    for (int i = 0; i < 40000; ++i) {
        const double x = rng.uniform(0.01, 100.0);
        const double y = rng.uniform(-20.0, 20.0);
        wpow = std::max(wpow, ulp_distance(g2m::det::pow(x, y), std::pow(x, y)));
    }
    report("exp", wexp);
    report("expm1", wexpm1);
    report("log", wlog);
    report("log1p", wlog1p);
    report("pow", wpow);
    CHECK(wexp <= kUlpBound);
    CHECK(wexpm1 <= kUlpBound);
    CHECK(wlog <= kUlpBound);
    CHECK(wlog1p <= kUlpBound);
    CHECK(wpow <= kUlpBound);
}

TEST_CASE("det hyperbolic and hypot vs platform libm", "[math]") {
    SplitMix64 rng(4);
    auto both = [&](auto f, auto ref, double lo, double hi) {
        double w = worst_unary(f, ref, rng, lo, hi, 40000);
        w = std::max(w, worst_unary_log(f, ref, rng, -300, std::log10(hi), true, 40000));
        return w;
    };
    const double wsinh = both([](double x) { return g2m::det::sinh(x); }, [](double x) { return std::sinh(x); }, -710.0, 710.0);
    const double wcosh = both([](double x) { return g2m::det::cosh(x); }, [](double x) { return std::cosh(x); }, -710.0, 710.0);
    const double wtanh = both([](double x) { return g2m::det::tanh(x); }, [](double x) { return std::tanh(x); }, -25.0, 25.0);
    const double wasinh = both([](double x) { return g2m::det::asinh(x); }, [](double x) { return std::asinh(x); }, -1e6, 1e6);
    double watanh = worst_unary([](double x) { return g2m::det::atanh(x); }, [](double x) { return std::atanh(x); }, rng, -1.0, 1.0, 40000);
    watanh = std::max(watanh, worst_unary_log([](double x) { return g2m::det::atanh(x); }, [](double x) { return std::atanh(x); }, rng, -300, -0.0001, true, 40000));
    double whypot = 0;
    for (int i = 0; i < 40000; ++i) {
        const double x = std::pow(10.0, rng.uniform(-300, 300));
        const double y = std::pow(10.0, rng.uniform(-300, 300)) * ((rng.next() & 1) ? -1 : 1);
        whypot = std::max(whypot, ulp_distance(g2m::det::hypot(x, y), std::hypot(x, y)));
        const double u = rng.uniform(-10.0, 10.0);
        const double v = rng.uniform(-10.0, 10.0);
        whypot = std::max(whypot, ulp_distance(g2m::det::hypot(u, v), std::hypot(u, v)));
    }
    report("sinh", wsinh);
    report("cosh", wcosh);
    report("tanh", wtanh);
    report("asinh", wasinh);
    report("atanh", watanh);
    report("hypot", whypot);
    // musl documents up to ~2 ulp from the true value for sinh, tanh and
    // atanh (measured against 80-bit long double: 1.72, 2.07, 1.60 ulp - see
    // "det vs long double truth" below), so up to 3 ulp from a platform libm
    // that is itself ~1 ulp off in the other direction is correct behaviour.
    constexpr double kHyperbolicBound = 3.0;
    CHECK(wsinh <= kHyperbolicBound);
    CHECK(wcosh <= kUlpBound);
    CHECK(wtanh <= kHyperbolicBound);
    CHECK(wasinh <= kUlpBound);
    CHECK(watanh <= kHyperbolicBound);
    // musl's hypot is correctly rounded in practice; glibc's and ucrt's
    // are within 1 ulp.
    CHECK(whypot <= 1.0);
}

TEST_CASE("det special values", "[math]") {
    namespace d = g2m::det;
    constexpr double kPi = g2m::det::kPi;
    const double inf = std::numeric_limits<double>::infinity();
    const double nan = std::numeric_limits<double>::quiet_NaN();
    const double tiny = std::numeric_limits<double>::denorm_min();

    CHECK(d::sin(0.0) == 0.0);
    CHECK(std::signbit(d::sin(-0.0)));
    CHECK(d::cos(0.0) == 1.0);
    CHECK(std::isnan(d::sin(inf)));
    CHECK(std::isnan(d::cos(nan)));
    CHECK(d::sin(tiny) == tiny);
    CHECK(d::atan2(0.0, -1.0) == kPi);
    CHECK(d::atan2(-0.0, -1.0) == -kPi);
    CHECK(d::exp(0.0) == 1.0);
    CHECK(d::exp(-inf) == 0.0);
    CHECK(d::exp(inf) == inf);
    CHECK(d::exp(1000.0) == inf);
    CHECK(d::log(1.0) == 0.0);
    CHECK(d::log(0.0) == -inf);
    CHECK(std::isnan(d::log(-1.0)));
    CHECK(d::expm1(0.0) == 0.0);
    CHECK(std::signbit(d::expm1(-0.0)));
    CHECK(d::expm1(-inf) == -1.0);
    CHECK(d::expm1(tiny) == tiny);
    CHECK(d::log1p(0.0) == 0.0);
    CHECK(std::signbit(d::log1p(-0.0)));
    CHECK(d::log1p(-1.0) == -inf);
    CHECK(std::isnan(d::log1p(-2.0)));
    CHECK(d::log1p(tiny) == tiny);
    CHECK(d::sinh(0.0) == 0.0);
    CHECK(std::signbit(d::sinh(-0.0)));
    CHECK(d::sinh(inf) == inf);
    CHECK(d::sinh(-inf) == -inf);
    CHECK(d::sinh(800.0) == inf);
    CHECK(d::cosh(0.0) == 1.0);
    CHECK(d::cosh(-inf) == inf);
    CHECK(d::tanh(inf) == 1.0);
    CHECK(d::tanh(-inf) == -1.0);
    CHECK(std::isnan(d::tanh(nan)));
    CHECK(std::signbit(d::tanh(-0.0)));
    CHECK(d::asinh(0.0) == 0.0);
    CHECK(std::signbit(d::asinh(-0.0)));
    CHECK(d::asinh(inf) == inf);
    CHECK(d::asinh(-inf) == -inf);
    CHECK(d::atanh(0.0) == 0.0);
    CHECK(d::atanh(1.0) == inf);
    CHECK(d::atanh(-1.0) == -inf);
    CHECK(std::isnan(d::atanh(2.0)));
    CHECK(d::hypot(3.0, 4.0) == 5.0);
    CHECK(d::hypot(inf, nan) == inf);
    CHECK(d::hypot(nan, -inf) == inf);
    CHECK(std::isnan(d::hypot(nan, 1.0)));
    CHECK(d::hypot(-0.0, 0.0) == 0.0);
    CHECK(ulp_distance(d::hypot(1e300, 1e300), std::hypot(1e300, 1e300)) <= 1.0);
    CHECK(ulp_distance(d::hypot(1e-300, 3e-301), std::hypot(1e-300, 3e-301)) <= 1.0);
}

// Where long double has at least 64 mantissa bits (x86-64 Linux: 80-bit
// x87), it gives a reference ~2^11 times more precise than double, so the
// port's error from the TRUE value can be measured directly. MSVC's long
// double is plain double: skipped there (the Linux legs cover it).
TEST_CASE("det vs long double truth", "[math]") {
    if constexpr (std::numeric_limits<long double>::digits < 64) {
        SKIP("long double is not extended precision on this platform");
    } else {
        auto ulp_err = [](double a, long double ref) {
            const double r = static_cast<double>(ref);
            const double u = std::nextafter(std::fabs(r), std::numeric_limits<double>::infinity()) - std::fabs(r);
            return static_cast<double>(std::fabs(static_cast<long double>(a) - ref) / u);
        };
        struct Fn {
            const char* name;
            double (*f)(double);
            long double (*ref)(long double);
            double lo, hi, bound; // bound: musl's documented accuracy, rounded up
        };
        const Fn fns[] = {
            {"sin", g2m::det::sin, [](long double x) { return std::sin(x); }, -10, 10, 1.0},
            {"cos", g2m::det::cos, [](long double x) { return std::cos(x); }, -10, 10, 1.0},
            {"atan", g2m::det::atan, [](long double x) { return std::atan(x); }, -10, 10, 1.0},
            {"exp", g2m::det::exp, [](long double x) { return std::exp(x); }, -10, 10, 1.0},
            {"log", g2m::det::log, [](long double x) { return std::log(x); }, 0.001, 1000, 1.0},
            {"expm1", g2m::det::expm1, [](long double x) { return std::expm1(x); }, -5, 5, 1.0},
            {"log1p", g2m::det::log1p, [](long double x) { return std::log1p(x); }, -0.99, 5, 1.0},
            {"sinh", g2m::det::sinh, [](long double x) { return std::sinh(x); }, -30, 30, 2.0},
            {"cosh", g2m::det::cosh, [](long double x) { return std::cosh(x); }, -30, 30, 1.5},
            {"tanh", g2m::det::tanh, [](long double x) { return std::tanh(x); }, -3, 3, 2.5},
            {"asinh", g2m::det::asinh, [](long double x) { return std::asinh(x); }, -1e3, 1e3, 1.5},
            {"atanh", g2m::det::atanh, [](long double x) { return std::atanh(x); }, -1, 1, 2.0},
        };
        SplitMix64 rng(5);
        for (const Fn& fn : fns) {
            double worst = 0;
            for (int i = 0; i < 200000; ++i) {
                const double x = rng.uniform(fn.lo, fn.hi);
                worst = std::max(worst, ulp_err(fn.f(x), fn.ref(x)));
            }
            std::printf("det::%-6s worst %.3f ulp vs long double truth\n", fn.name, worst);
            CHECK(worst <= fn.bound);
        }
        double wh = 0;
        for (int i = 0; i < 200000; ++i) {
            const double x = rng.uniform(-10, 10);
            const double y = rng.uniform(-10, 10);
            const long double lx = x, ly = y;
            wh = std::max(wh, ulp_err(g2m::det::hypot(x, y), std::sqrt(lx * lx + ly * ly)));
        }
        std::printf("det::hypot  worst %.3f ulp vs long double truth\n", wh);
        CHECK(wh <= 1.1);
    }
}
