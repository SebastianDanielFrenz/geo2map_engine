#include "g2m/core/geom/clothoid.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <cmath>
// Scalar reduction: Bertolazzi/Frego, https://arxiv.org/abs/1209.0910.
// Quadrature, bounded solver and validation below are our implementation.
namespace g2m::geom {
namespace {
struct Integrals { double x = 0, y = 0, derivative = 0; };
// Composite 8-point Gauss-Legendre quadrature on 16 fixed panels.
// The integrand is evaluated directly, retaining accuracy near lines/arcs.
Integrals integrate(double a, double b, double heading, double end = 1.0) {
    constexpr double nodes[] = {0.18343464249564980494, 0.52553240991632898582,
        0.79666647741362673959, 0.96028985649753623168};
    constexpr double weights[] = {0.36268378337836198297, 0.31370664587788728734,
        0.22238103445337447054, 0.10122853629037625915};
    Integrals out;
    const double half = end / 32.0;
    for (int panel = 0; panel < 16; ++panel) {
        const double mid = (2 * panel + 1) * half;
        for (int i = 0; i < 4; ++i) for (const int sign : {-1, 1}) {
            const double t = mid + sign * half * nodes[i];
            double s, c; det::sincos((a * t + b) * t + heading, s, c);
            const double weight = half * weights[i];
            out.x += c * weight; out.y += s * weight;
            out.derivative += c * (t * t - t) * weight;
        }
    }
    return out;
}
double normalise(double angle) {
    constexpr double tau = 2 * det::kPi;
    angle -= tau * std::floor((angle + det::kPi) / tau); return angle;
}
Error error(const char* message) { return make_error(ErrorCode::InvalidArgument, message); }
}
Result<Clothoid> fit_clothoid(double x0, double y0, double h0, double x1, double y1, double h1) {
    if (!std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(x1) || !std::isfinite(y1) ||
        !std::isfinite(h0) || !std::isfinite(h1) || std::fabs(x0) > 1e9 || std::fabs(y0) > 1e9 ||
        std::fabs(x1) > 1e9 || std::fabs(y1) > 1e9 || std::fabs(h0) > 1e6 || std::fabs(h1) > 1e6)
        return error("clothoid: invalid coordinates/headings");
    const auto dx = x1 - x0, dy = y1 - y0;
    const auto chord = det::sqrt(dx * dx + dy * dy);
    if (chord < 1e-6 || chord > 1e6) return error("clothoid: chord range");
    const auto chord_heading = det::atan2(dy, dx);
    const auto theta = normalise(h0 - chord_heading);
    const auto delta = normalise(h1 - h0);
    double a = 6 * theta + 3 * delta;
    Integrals values; bool converged = false;
    for (int iteration = 0; iteration < 32; ++iteration) {
        values = integrate(a, delta - a, theta);
        if (std::fabs(values.y) <= 1e-12) { converged = true; break; }
        if (std::fabs(values.derivative) < 1e-12) break;
        const auto step = std::clamp(values.y / values.derivative, -8 * det::kPi, 8 * det::kPi);
        a -= step;
        if (std::fabs(a) > 16 * det::kPi) break;
    }
    if (!converged || values.x < 0.125 || values.x > 1.000000000001)
        return error("clothoid: no bounded forward solution");
    const auto length = chord / values.x;
    return Clothoid{x0, y0, normalise(h0), length, (delta - a) / length, 2 * a / (length * length)};
}
Result<CurvePose> Clothoid::at(double station) const {
    if (!std::isfinite(station) || !std::isfinite(length) || length <= 0 || station < 0 || station > length ||
        !std::isfinite(x0) || !std::isfinite(y0) || !std::isfinite(heading0) ||
        !std::isfinite(curvature0) || !std::isfinite(curvature_rate) || length > 8e6 ||
        std::fabs(x0) > 1e9 || std::fabs(y0) > 1e9 || std::fabs(heading0) > 1e6)
        return error("clothoid: invalid station/segment");
    const double a = 0.5 * curvature_rate * length * length, b = curvature0 * length;
    if (!std::isfinite(a) || !std::isfinite(b) || std::fabs(a) > 16 * det::kPi || std::fabs(b) > 20 * det::kPi)
        return error("clothoid: integration phase budget");
    const auto values = integrate(a, b, heading0, station / length);
    return CurvePose{x0 + length * values.x, y0 + length * values.y,
        normalise(heading0 + curvature0 * station + 0.5 * curvature_rate * station * station),
        curvature0 + curvature_rate * station};
}
}
