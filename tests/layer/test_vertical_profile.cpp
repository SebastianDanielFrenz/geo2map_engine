#include "g2m/layer/vertical_profile.h"
#include "g2m/core/hash.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
using namespace g2m;

TEST_CASE("Vertical reachability rejects incompatible endpoint curvature before iteration", "[layer][vertical_profile]") {
    VerticalFitLimits limits; limits.min_radius_m = 100; limits.max_grade = 1;
    const VerticalSample impossible[] = {{0, 0}, {5, 0.15}, {10, 0.3}};
    const auto no = fit_vertical_profile(impossible, limits); REQUIRE_FALSE(no);
    CHECK(no.error().message == "vertical profile: endpoint curvature infeasible");
    const VerticalSample downward[] = {{0, 0}, {5, -0.15}, {10, -0.3}};
    CHECK_FALSE(fit_vertical_profile(downward, limits));
    const VerticalSample plane[] = {{0, 0}, {10, 0.5}};
    limits.start_grade = limits.end_grade = 0.05;
    CHECK(fit_vertical_profile(plane, limits));
    limits.start_grade = -0.1; limits.end_grade = 0.1;
    const auto changing = fit_vertical_profile(plane, limits); REQUIRE_FALSE(changing);
    CHECK(changing.error().message == "vertical profile: endpoint curvature infeasible");
}
TEST_CASE("Vertical cubic least squares preserves an exact inclined plane", "[layer][vertical_profile]") {
    const VerticalSample samples[] = {{0, 100}, {25, 100.25}, {50, 100.5}, {75, 100.75}, {100, 101}};
    VerticalFitLimits limits; limits.start_grade = limits.end_grade = 0.01;
    const auto profile = fit_vertical_profile(samples, limits); REQUIRE(profile); CHECK(profile->iterations == 1);
    for (const auto& segment : profile->segments) for (int i = 0; i <= 10; ++i) {
        const auto s = segment.station_m + segment.length_m * i / 10;
        const auto value = segment.at(s); REQUIRE(value);
        CHECK(std::fabs(value->height_m - 100 - s * 0.01) < 1e-10);
        CHECK(std::fabs(value->grade - 0.01) < 1e-10); CHECK(std::fabs(value->second_derivative) < 1e-10);
    }
}
TEST_CASE("Vertical profiles smooth DEM noise while satisfying continuous bounds", "[layer][vertical_profile]") {
    std::vector<VerticalSample> samples;
    for (int i = 0; i <= 10; ++i) samples.push_back({i * 10.0, 100 + i * 0.1 + (i == 5 ? 2.0 : 0.0)});
    VerticalFitLimits limits; limits.max_grade = 0.06; limits.min_radius_m = 500;
    const auto profile = fit_vertical_profile(samples, limits); REQUIRE(profile);
    REQUIRE(profile->segments.size() == 10);
    CHECK(profile->segments.front().height0_m == 100); CHECK(profile->segments.back().height1_m == 101);
    CHECK(profile->segments.front().grade0 == 0); CHECK(profile->segments.back().grade1 == 0);
    for (std::size_t j = 0; j < profile->segments.size(); ++j) {
        const auto& segment = profile->segments[j];
        if (j) {
            CHECK(segment.height0_m == profile->segments[j - 1].height1_m);
            CHECK(segment.grade0 == profile->segments[j - 1].grade1);
        }
        for (int i = 0; i <= 50; ++i) {
            const auto value = segment.at(segment.station_m + segment.length_m * i / 50); REQUIRE(value);
            CHECK(std::fabs(value->grade) <= limits.max_grade + 1e-8);
            CHECK(std::fabs(value->second_derivative) <= 1 / limits.min_radius_m + 1e-8);
        }
    }
    const auto repeat = fit_vertical_profile(samples, limits); REQUIRE(repeat);
    CHECK(repeat->iterations == profile->iterations);
    for (std::size_t i = 0; i < profile->segments.size(); ++i)
        CHECK(repeat->segments[i].height0_m == profile->segments[i].height0_m);
    Fnv1a64 golden;
    golden.i32(profile->iterations);
    for (const auto& s : profile->segments)
        golden.f64(s.station_m).f64(s.length_m).f64(s.height0_m).f64(s.height1_m).f64(s.grade0).f64(s.grade1);
    CHECK(to_hex(golden.value()) == "5ae5a60af3c10f3f");
    limits.max_iterations = 1; CHECK_FALSE(fit_vertical_profile(samples, limits));
}
TEST_CASE("Vertical least squares reaches the analytic symmetric constrained optimum", "[layer][vertical_profile]") {
    const VerticalSample samples[] = {{0, 0}, {10, 5}, {20, 0}};
    VerticalFitLimits limits; limits.max_grade = 0.1; limits.min_radius_m = 1;
    const auto profile = fit_vertical_profile(samples, limits); REQUIRE(profile);
    CHECK(std::fabs(profile->segments[0].height1_m - 1.0 / 3) < 1e-8);
    CHECK(std::fabs(profile->segments[0].grade1) < 1e-8);
}
TEST_CASE("Vertical profiles reject infeasible boundaries and invalid DEM", "[layer][vertical_profile]") {
    const VerticalSample steep[] = {{0, 100}, {10, 110}}; CHECK_FALSE(fit_vertical_profile(steep));
    const VerticalSample short_curve[] = {{0, 100}, {0.5, 100.1}};
    VerticalFitLimits limits; limits.max_grade = 1; limits.min_radius_m = 1000;
    CHECK_FALSE(fit_vertical_profile(short_curve, limits));
    const VerticalSample missing[] = {{0, 100}, {10, std::numeric_limits<double>::quiet_NaN()}};
    CHECK_FALSE(fit_vertical_profile(missing));
    const VerticalSample unordered[] = {{0, 0}, {10, 1}, {5, 1}}; CHECK_FALSE(fit_vertical_profile(unordered));
    CHECK_FALSE(fit_vertical_profile({}));
    const VerticalSample flat[] = {{0, 0}, {10, 0}};
    const auto profile = fit_vertical_profile(flat); REQUIRE(profile);
    CHECK_FALSE(profile->segments[0].at(-1)); CHECK_FALSE(profile->segments[0].at(11));
}
