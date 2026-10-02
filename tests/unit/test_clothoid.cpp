#include "g2m/core/geom/clothoid.h"
#include "g2m/core/det_math.h"
#include "g2m/core/hash.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
#include <limits>
using namespace g2m;
using namespace g2m::geom;
TEST_CASE("Clothoid fitting reproduces a straight line and a circular arc", "[geom][clothoid]") {
    const auto line = fit_clothoid(10, 20, 0, 110, 20, 0); REQUIRE(line);
    CHECK(std::fabs(line->length - 100) < 1e-10); CHECK(line->curvature0 == 0); CHECK(line->curvature_rate == 0);
    const auto p = line->at(37); REQUIRE(p); CHECK(std::fabs(p->x - 47) < 1e-10); CHECK(std::fabs(p->y - 20) < 1e-10);
    const auto arc = fit_clothoid(0, 0, 0, 100, 100, det::kPi / 2); REQUIRE(arc);
    CHECK(std::fabs(arc->length - 50 * det::kPi) < 1e-9);
    CHECK(std::fabs(arc->curvature0 - 0.01) < 1e-12); CHECK(std::fabs(arc->curvature_rate) < 1e-12);
    const auto middle = arc->at(arc->length / 2); REQUIRE(middle);
    CHECK(std::fabs(middle->x - 100 * det::sin(det::kPi / 4)) < 1e-8);
    CHECK(std::fabs(middle->y - 100 * (1 - det::cos(det::kPi / 4))) < 1e-8);
}
TEST_CASE("Clothoids meet independent endpoint and unit-speed checks over tangent pairs", "[geom][clothoid]") {
    Fnv1a64 golden;
    for (int i = -4; i <= 4; ++i) for (int j = -4; j <= 4; ++j) {
        const double h0 = i * 0.3, h1 = j * 0.3;
        const auto curve = fit_clothoid(474000, 5560000, h0, 474100, 5560000, h1); REQUIRE(curve);
        golden.f64(curve->length).f64(curve->curvature0).f64(curve->curvature_rate);
        const auto endpoint = curve->at(curve->length); REQUIRE(endpoint);
        CHECK(std::fabs(endpoint->x - 474100) < 1e-7); CHECK(std::fabs(endpoint->y - 5560000) < 1e-7);
        CHECK(std::fabs(endpoint->heading - h1) < 1e-10);
        const auto a = curve->at(curve->length / 2 - 0.01), b = curve->at(curve->length / 2 + 0.01);
        REQUIRE(a); REQUIRE(b);
        const auto dx = b->x - a->x, dy = b->y - a->y;
        CHECK(std::fabs(det::sqrt(dx * dx + dy * dy) / 0.02 - 1) < 1e-6);
        CHECK(std::fabs(b->curvature - a->curvature - curve->curvature_rate * 0.02) < 1e-12);
        golden.f64(endpoint->x).f64(endpoint->y).f64(endpoint->heading).f64(a->x).f64(a->y).f64(b->curvature);
    }
    CHECK(to_hex(golden.value()) == "3d038842e637f8cc");
}
TEST_CASE("Clothoid fitting rejects invalid and unbounded requests", "[geom][clothoid]") {
    CHECK_FALSE(fit_clothoid(0, 0, 0, 0, 0, 0));
    CHECK_FALSE(fit_clothoid(0, 0, std::numeric_limits<double>::quiet_NaN(), 10, 0, 0));
    CHECK_FALSE(fit_clothoid(0, 0, 0, 1000001, 0, 0));
    const auto curve = fit_clothoid(0, 0, 0, 100, 20, 0); REQUIRE(curve);
    CHECK_FALSE(curve->at(-1)); CHECK_FALSE(curve->at(curve->length + 1));
    CHECK_FALSE(curve->at(std::numeric_limits<double>::infinity()));
}
