#include "g2m/layer/vertical_profile.h"
#include <algorithm>
#include <array>
#include <cmath>
// Dykstra's weighted half-space projection; original method:
// https://doi.org/10.1080/01621459.1983.10477029. Sparse road constraints are ours.
namespace g2m {
namespace {
struct Term { std::size_t index; double coefficient; };
struct Constraint {
    std::array<Term, 4> terms{}; std::size_t count = 0;
    double bound = 0, norm = 0, correction = 0;
};
Error invalid(const char* message) { return make_error(ErrorCode::InvalidArgument, message); }
}
Result<VerticalPose> VerticalCubic::at(double station) const {
    if (!std::isfinite(station) || !std::isfinite(station_m) || !std::isfinite(length_m) || length_m <= 0 ||
        station < station_m || station > station_m + length_m || !std::isfinite(height0_m) ||
        !std::isfinite(height1_m) || !std::isfinite(grade0) || !std::isfinite(grade1))
        return invalid("vertical profile: invalid station/cubic");
    const double t = (station - station_m) / length_m;
    const double delta = height1_m - height0_m;
    const double a = length_m * (grade0 + grade1) - 2 * delta;
    const double b = 3 * delta - length_m * (2 * grade0 + grade1);
    const double c = length_m * grade0;
    return VerticalPose{t == 0 ? height0_m : t == 1 ? height1_m : ((a * t + b) * t + c) * t + height0_m,
        t == 0 ? grade0 : t == 1 ? grade1 : ((3 * a * t + 2 * b) * t + c) / length_m,
        (6 * a * t + 2 * b) / (length_m * length_m)};
}
Result<VerticalProfile> fit_vertical_profile(std::span<const VerticalSample> samples, const VerticalFitLimits& limits) {
    const auto n = samples.size();
    if (n < 2 || n > 2048 || !std::isfinite(limits.max_grade) || limits.max_grade <= 0 || limits.max_grade > 1 ||
        !std::isfinite(limits.min_radius_m) || limits.min_radius_m < 1 || limits.min_radius_m > 100000 ||
        !std::isfinite(limits.start_grade) || !std::isfinite(limits.end_grade) ||
        std::fabs(limits.start_grade) > limits.max_grade || std::fabs(limits.end_grade) > limits.max_grade ||
        !std::isfinite(limits.slope_weight_m) || limits.slope_weight_m < 0.1 || limits.slope_weight_m > 1000 ||
        limits.max_iterations < 1 || limits.max_iterations > 16384)
        return invalid("vertical profile: limits/sample count");
    for (std::size_t i = 0; i < n; ++i)
        if (!std::isfinite(samples[i].station_m) || !std::isfinite(samples[i].height_m) ||
            std::fabs(samples[i].height_m) > 1e6 || samples[i].station_m > 100000 ||
            (!i && samples[i].station_m != 0) || (i && samples[i].station_m - samples[i - 1].station_m < 0.5))
            return invalid("vertical profile: invalid station/height");
    if (std::fabs(samples.back().height_m - samples.front().height_m) > limits.max_grade * samples.back().station_m + 1e-10)
        return invalid("vertical profile: endpoint grade infeasible");
    std::vector<double> values(n * 2), inverse_weight(n * 2, 1.0);
    const auto origin = samples.front().height_m;
    for (std::size_t i = 0; i < n; ++i) {
        values[i] = samples[i].height_m - origin;
        const auto lo = i ? i - 1 : i, hi = i + 1 < n ? i + 1 : i;
        values[n + i] = std::clamp((samples[hi].height_m - samples[lo].height_m) /
            (samples[hi].station_m - samples[lo].station_m), -limits.max_grade, limits.max_grade);
        inverse_weight[n + i] = 1 / (limits.slope_weight_m * limits.slope_weight_m);
    }
    values[n] = limits.start_grade; values[2 * n - 1] = limits.end_grade;
    inverse_weight[0] = inverse_weight[n - 1] = inverse_weight[n] = inverse_weight[2 * n - 1] = 0;
    std::vector<Constraint> constraints; constraints.reserve(n * 8);
    const auto add_pair = [&](std::initializer_list<Term> terms, double bound) {
        for (const int sign : {-1, 1}) {
            Constraint c; c.bound = bound;
            for (const auto& term : terms) {
                c.terms[c.count++] = {term.index, sign * term.coefficient};
                c.norm += term.coefficient * term.coefficient * inverse_weight[term.index];
            }
            constraints.push_back(c);
        }
    };
    for (std::size_t i = 0; i < n; ++i) add_pair({{n + i, 1}}, limits.max_grade);
    for (std::size_t i = 0; i + 1 < n; ++i) {
        const double d = samples[i + 1].station_m - samples[i].station_m, d2 = d * d;
        add_pair({{i, -3 / d}, {i + 1, 3 / d}, {n + i, -1}, {n + i + 1, -1}}, limits.max_grade);
        add_pair({{i, -6 / d2}, {i + 1, 6 / d2}, {n + i, -4 / d}, {n + i + 1, -2 / d}}, 1 / limits.min_radius_m);
        add_pair({{i, 6 / d2}, {i + 1, -6 / d2}, {n + i, 2 / d}, {n + i + 1, 4 / d}}, 1 / limits.min_radius_m);
    }
    bool converged = false; int iterations = 0;
    // Fixed-order sparse projections. A scalar Dykstra correction suffices
    // because each half-space correction lies along its weighted normal.
    const int budget = std::min(limits.max_iterations, static_cast<int>(64000000 / constraints.size()));
    for (; iterations < budget; ++iterations) {
        double max_change = 0;
        for (auto& c : constraints) {
            double dot = 0;
            for (std::size_t j = 0; j < c.count; ++j) dot += c.terms[j].coefficient * values[c.terms[j].index];
            if (!c.norm) {
                if (dot > c.bound + 1e-10) return invalid("vertical profile: fixed boundary constraints infeasible");
                continue;
            }
            const double correction = std::max(0.0, (dot + c.correction * c.norm - c.bound) / c.norm);
            const double delta = c.correction - correction; c.correction = correction;
            for (std::size_t j = 0; j < c.count; ++j) {
                const auto& term = c.terms[j]; const auto change = delta * term.coefficient * inverse_weight[term.index];
                values[term.index] += change; max_change = std::max(max_change, std::fabs(change));
            }
        }
        if (max_change > 1e-10) continue;
        double violation = 0;
        for (const auto& c : constraints) {
            double dot = 0;
            for (std::size_t j = 0; j < c.count; ++j) dot += c.terms[j].coefficient * values[c.terms[j].index];
            violation = std::max(violation, dot - c.bound);
        }
        if (violation <= 1e-9) { converged = true; ++iterations; break; }
    }
    if (!converged) return make_error(ErrorCode::LimitExceeded, "vertical profile: bounded solver did not converge");
    VerticalProfile out; out.iterations = iterations; out.segments.reserve(n - 1);
    for (std::size_t i = 0; i + 1 < n; ++i)
        out.segments.push_back({samples[i].station_m, samples[i + 1].station_m - samples[i].station_m,
            i == 0 ? samples.front().height_m : values[i] + origin,
            i + 1 == n - 1 ? samples.back().height_m : values[i + 1] + origin, values[n + i], values[n + i + 1]});
    return out;
}
}
