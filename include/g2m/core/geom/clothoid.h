#pragma once
#include "g2m/core/result.h"
namespace g2m::geom {
struct CurvePose { double x = 0, y = 0, heading = 0, curvature = 0; };
// Arc-length parametrised G1 segment: curvature(s) = curvature0 + rate*s.
// A zero rate is an arc; zero curvature and rate form a straight line.
struct Clothoid {
    double x0 = 0, y0 = 0, heading0 = 0;
    double length = 0, curvature0 = 0, curvature_rate = 0;
    Result<CurvePose> at(double station) const;
};
// Fits positions and tangents using a bounded scalar Newton solve over the
// normalised clothoid integral. Deterministic math and fixed-order quadrature;
// no external clothoid implementation. Rejects nonconvergence/looping solutions.
Result<Clothoid> fit_clothoid(double x0, double y0, double heading0,
    double x1, double y1, double heading1);
}
