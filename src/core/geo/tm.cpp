// src/core/geo/tm.cpp - transverse Mercator, Krueger series to 6th order.
// Mirrors GeographicLib v2 TransverseMercator::Forward/Reverse (MIT, C. F.
// F. Karney) step for step, with three changes:
//  - every transcendental is g2m::det (bit-identical everywhere);
//  - the Clenshaw sums use explicit real arithmetic instead of
//    std::complex, whose operators differ between standard libraries
//    (libstdc++ calls __muldc3 for complex multiplication);
//  - degree reduction via angle.h (std::fmod instead of std::remquo).

#include "g2m/core/geo/tm.h"

#include "g2m/core/det_math.h"
#include "g2m/core/geo/angle.h"

#include <cmath>
#include <limits>

namespace g2m::geo {
namespace {

constexpr double kQd = 90.0;
constexpr double kHd = 180.0;
constexpr double kHalfPi = det::kPi / 2;

double sq(double x) { return x * x; }

// Horner evaluation of p[0] x^n + ... + p[n] (GeographicLib Math::polyval).
double polyval(int n, const double* p, double x) {
    double y = n < 0 ? 0.0 : *p++;
    while (--n >= 0) y = y * x + *p++;
    return y;
}

// e * atanh(e * x) (GeographicLib Math::eatanhe).
double eatanhe(double x, double es) {
    return es > 0 ? es * det::atanh(es * x) : -es * det::atan(es * x);
}

// tan(chi) from tan(phi): conformal latitude (GeographicLib Math::taupf).
double taupf(double tau, double es) {
    if (!std::isfinite(tau)) return tau;
    const double tau1 = det::hypot(1.0, tau);
    const double sig = det::sinh(eatanhe(tau / tau1, es));
    return det::hypot(1.0, sig) * tau - sig * tau1;
}

// Inverse of taupf by Newton's method (GeographicLib Math::tauf).
double tauf(double taup, double es) {
    constexpr int kNumIt = 5;
    const double eps = std::numeric_limits<double>::epsilon();
    const double tol = std::sqrt(eps) / 10;
    const double taumax = 2 / std::sqrt(eps);
    const double e2m = 1 - sq(es);
    double tau = std::fabs(taup) > 70 ? taup * det::exp(eatanhe(1.0, es)) : taup / e2m;
    const double stol = tol * std::fmax(1.0, std::fabs(taup));
    if (!(std::fabs(tau) < taumax)) return tau; // +-inf, nan, or large enough that the guess is exact
    for (int i = 0; i < kNumIt; ++i) {
        const double taupa = taupf(tau, es);
        const double dtau = (taup - taupa) * (1 + e2m * sq(tau)) /
                            (e2m * det::hypot(1.0, tau) * det::hypot(1.0, taupa));
        tau += dtau;
        if (!(std::fabs(dtau) >= stol)) break;
    }
    return tau;
}

double lat_fix(double x) {
    return std::fabs(x) > kQd ? std::numeric_limits<double>::quiet_NaN() : x;
}

// Krueger series coefficients, 6th order (GeographicLib's maxpow_ == 6
// tables). For l = 1..6, alpha_l = n^l * P_l(n) / d_l, with P_l given by its
// integer coefficients (highest power first) followed by the denominator d_l.
constexpr double kAlpCoeff[] = {
    // alp[1]/n^1, polynomial in n of order 5
    31564, -66675, 34440, 47250, -100800, 75600, 151200,
    // alp[2]/n^2, polynomial in n of order 4
    -1983433, 863232, 748608, -1161216, 524160, 1935360,
    // alp[3]/n^3, polynomial in n of order 3
    670412, 406647, -533952, 184464, 725760,
    // alp[4]/n^4, polynomial in n of order 2
    6601661, -7732800, 2230245, 7257600,
    // alp[5]/n^5, polynomial in n of order 1
    -13675556, 3438171, 7983360,
    // alp[6]/n^6, polynomial in n of order 0
    212378941, 319334400,
};
constexpr double kBetCoeff[] = {
    // bet[1]/n^1, polynomial in n of order 5
    384796, -382725, -6720, 932400, -1612800, 1209600, 2419200,
    // bet[2]/n^2, polynomial in n of order 4
    -1118711, 1695744, -1174656, 258048, 80640, 3870720,
    // bet[3]/n^3, polynomial in n of order 3
    22276, -16929, -15984, 12852, 362880,
    // bet[4]/n^4, polynomial in n of order 2
    -830251, -158400, 197865, 7257600,
    // bet[5]/n^5, polynomial in n of order 1
    -435388, 453717, 15966720,
    // bet[6]/n^6, polynomial in n of order 0
    20648693, 638668800,
};
// b1 * (1 + n) * 256 = n^6 + 4 n^4 + 64 n^2 + 256, a polynomial in n^2.
constexpr double kB1Coeff[] = {1, 4, 64, 256, 256};

static_assert(sizeof(kAlpCoeff) / sizeof(double) == 27);
static_assert(sizeof(kBetCoeff) / sizeof(double) == 27);

// A complex number as two reals; the operations below are exactly the
// textbook formulas, in a fixed order.
struct Cx {
    double re;
    double im;
};
Cx mul(Cx a, Cx b) { return {a.re * b.re - a.im * b.im, a.re * b.im + a.im * b.re}; }

// Clenshaw summation shared by forward (sign = +1, coefficients alpha) and
// reverse (sign = -1, coefficients beta), for a = 2 cos(2 zeta). Returns
// GeographicLib's y0 (the sine sum is y0 * sin(2 zeta)) and z0/z1 (for the
// derivative, 1 - z1 + z0 * cos(2 zeta)).
struct ClenshawOut {
    Cx y0;
    Cx z0;
    Cx z1;
};
ClenshawOut clenshaw(Cx a, const double* coef, double sign, int order) {
    int n = order;
    Cx y0{(n & 1) ? sign * coef[n] : 0.0, 0.0};
    Cx y1{0.0, 0.0};
    Cx z0{(n & 1) ? sign * (2 * n * coef[n]) : 0.0, 0.0};
    Cx z1{0.0, 0.0};
    if (n & 1) --n;
    while (n) {
        // y1 = a * y0 - y1 +- coef[n]; z1 = a * z0 - z1 +- 2n coef[n]
        Cx t = mul(a, y0);
        y1 = {t.re - y1.re + sign * coef[n], t.im - y1.im};
        t = mul(a, z0);
        z1 = {t.re - z1.re + sign * (2 * n * coef[n]), t.im - z1.im};
        --n;
        t = mul(a, y1);
        y0 = {t.re - y0.re + sign * coef[n], t.im - y0.im};
        t = mul(a, z1);
        z0 = {t.re - z0.re + sign * (2 * n * coef[n]), t.im - z0.im};
        --n;
    }
    return {y0, z0, z1};
}

} // namespace

TransverseMercator::TransverseMercator(const Ellipsoid& ellipsoid, double k0)
    : ellipsoid_(ellipsoid), k0_(k0) {
    const double f = ellipsoid.f;
    e2_ = f * (2 - f);
    es_ = (f < 0 ? -1 : 1) * std::sqrt(std::fabs(e2_));
    e2m_ = 1 - e2_;
    c_ = std::sqrt(e2m_) * det::exp(eatanhe(1.0, es_));
    n_ = f / (2 - f);
    const int m = kOrder / 2;
    b1_ = polyval(m, kB1Coeff, sq(n_)) / (kB1Coeff[m + 1] * (1 + n_));
    a1_ = b1_ * ellipsoid.a;
    alp_[0] = 0;
    bet_[0] = 0;
    int o = 0;
    double d = n_;
    for (int l = 1; l <= kOrder; ++l) {
        const int mm = kOrder - l;
        alp_[l] = d * polyval(mm, kAlpCoeff + o, n_) / kAlpCoeff[o + mm + 1];
        bet_[l] = d * polyval(mm, kBetCoeff + o, n_) / kBetCoeff[o + mm + 1];
        o += mm + 2;
        d *= n_;
    }
}

TmForward TransverseMercator::forward(double lon0, double lat, double lon) const {
    lat = lat_fix(lat);
    lon = ang_diff(lon0, lon);
    // Explicitly enforce the parity.
    int latsign = std::signbit(lat) ? -1 : 1;
    const int lonsign = std::signbit(lon) ? -1 : 1;
    lon *= lonsign;
    lat *= latsign;
    const bool backside = lon > kQd;
    if (backside) {
        if (lat == 0) latsign = -1;
        lon = kHd - lon;
    }
    double sphi = 0, cphi = 0, slam = 0, clam = 0;
    sincosd(lat, sphi, cphi);
    sincosd(lon, slam, clam);

    // xi', eta': Gauss-Schreiber TM coordinates from the conformal latitude.
    double etap = 0, xip = 0, gamma = 0, k = 0;
    if (lat != kQd) {
        const double tau = sphi / cphi;
        const double taup = taupf(tau, es_);
        xip = det::atan2(taup, clam);
        etap = det::asinh(slam / det::hypot(taup, clam));
        gamma = atan2d(slam * taup, clam * det::hypot(1.0, taup));
        k = std::sqrt(e2m_ + e2_ * sq(cphi)) * det::hypot(1.0, tau) / det::hypot(taup, clam);
    } else {
        xip = kHalfPi;
        etap = 0;
        gamma = lon;
        k = c_;
    }

    // zeta = zeta' + sum alpha_j sin(2 j zeta'), by Clenshaw summation.
    const double c0 = det::cos(2 * xip), ch0 = det::cosh(2 * etap);
    const double s0 = det::sin(2 * xip), sh0 = det::sinh(2 * etap);
    Cx a{2 * c0 * ch0, -2 * s0 * sh0}; // 2 cos(2 zeta')
    const ClenshawOut cl = clenshaw(a, alp_, +1.0, kOrder);
    a = {a.re / 2, a.im / 2};          // cos(2 zeta')
    const Cx az0 = mul(a, cl.z0);
    const Cx z1{1.0 - cl.z1.re + az0.re, -cl.z1.im + az0.im};
    a = {s0 * ch0, c0 * sh0};          // sin(2 zeta')
    const Cx ay0 = mul(a, cl.y0);
    const Cx y1{xip + ay0.re, etap + ay0.im};

    // Convergence and scale: Gauss-Schreiber -> Gauss-Krueger.
    gamma -= atan2d(z1.im, z1.re);
    k *= b1_ * det::hypot(z1.re, z1.im);
    const double xi = y1.re, eta = y1.im;

    TmForward out{};
    out.y = a1_ * k0_ * (backside ? det::kPi - xi : xi) * latsign;
    out.x = a1_ * k0_ * eta * lonsign;
    if (backside) gamma = kHd - gamma;
    gamma *= latsign * lonsign;
    out.gamma = ang_normalize(gamma);
    out.k = k * k0_;
    return out;
}

TmReverse TransverseMercator::reverse(double lon0, double x, double y) const {
    double xi = y / (a1_ * k0_);
    double eta = x / (a1_ * k0_);
    // Explicitly enforce the parity.
    const int xisign = std::signbit(xi) ? -1 : 1;
    const int etasign = std::signbit(eta) ? -1 : 1;
    xi *= xisign;
    eta *= etasign;
    const bool backside = xi > kHalfPi;
    if (backside) xi = det::kPi - xi;

    // zeta' = zeta - sum beta_j sin(2 j zeta), by Clenshaw summation.
    const double c0 = det::cos(2 * xi), ch0 = det::cosh(2 * eta);
    const double s0 = det::sin(2 * xi), sh0 = det::sinh(2 * eta);
    Cx a{2 * c0 * ch0, -2 * s0 * sh0}; // 2 cos(2 zeta)
    const ClenshawOut cl = clenshaw(a, bet_, -1.0, kOrder);
    a = {a.re / 2, a.im / 2};          // cos(2 zeta)
    const Cx az0 = mul(a, cl.z0);
    const Cx z1{1.0 - cl.z1.re + az0.re, -cl.z1.im + az0.im};
    a = {s0 * ch0, c0 * sh0};          // sin(2 zeta)
    const Cx ay0 = mul(a, cl.y0);
    const Cx y1{xi + ay0.re, eta + ay0.im};

    double gamma = atan2d(z1.im, z1.re);
    double k = b1_ / det::hypot(z1.re, z1.im);
    const double xip = y1.re, etap = y1.im;
    const double s = det::sinh(etap);
    const double c = std::fmax(0.0, det::cos(xip)); // cos(pi/2) might be negative
    const double r = det::hypot(s, c);
    double lat = 0, lon = 0;
    if (r != 0) {
        lon = atan2d(s, c);
        const double sxip = det::sin(xip);
        const double tau = tauf(sxip / r, es_);
        gamma += atan2d(sxip * det::tanh(etap), c);
        lat = atand(tau);
        k *= std::sqrt(e2m_ + e2_ / (1 + sq(tau))) * det::hypot(1.0, tau) * r;
    } else {
        lat = kQd;
        lon = 0;
        k *= c_;
    }
    lat *= xisign;
    if (backside) lon = kHd - lon;
    lon *= etasign;
    TmReverse out{};
    out.lat = lat;
    out.lon = ang_normalize(lon + lon0);
    if (backside) gamma = kHd - gamma;
    gamma *= xisign * etasign;
    out.gamma = ang_normalize(gamma);
    out.k = k * k0_;
    return out;
}

} // namespace g2m::geo
