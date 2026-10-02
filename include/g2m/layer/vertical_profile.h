#pragma once
#include "g2m/core/result.h"
#include <span>
#include <vector>
namespace g2m {
struct VerticalSample { double station_m = 0, height_m = 0; };
struct VerticalFitLimits {
    double max_grade = 0.15;
    double min_radius_m = 100;
    double start_grade = 0, end_grade = 0;
    double slope_weight_m = 5;
    int max_iterations = 4096;
};
struct VerticalPose { double height_m = 0, grade = 0, second_derivative = 0; };
struct VerticalCubic {
    double station_m = 0, length_m = 0;
    double height0_m = 0, height1_m = 0, grade0 = 0, grade1 = 0;
    Result<VerticalPose> at(double station) const;
};
struct VerticalProfile {
    std::vector<VerticalCubic> segments;
    int iterations = 0;
};
// Weighted least-squares projection onto linear cubic-Bezier derivative
// constraints, solved with bounded deterministic Dykstra iterations.
// Endpoint heights/grades are fixed; all joins are C1. Bernstein derivative
// bounds guarantee grade and vertical curvature bounds throughout each cubic,
// rather than checking only sampled points. Missing DEM is a caller error.
// Stations start at zero, step >=0.5 m, at most 2048 samples/100 km total.
Result<VerticalProfile> fit_vertical_profile(std::span<const VerticalSample> samples,
    const VerticalFitLimits& limits = {});
}
