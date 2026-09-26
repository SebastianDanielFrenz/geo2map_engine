// tests/unit/test_dem_km_span.cpp - g2m::import::dgm1_km_span (PLAN.md
// G2.0): the fix for the "at most two DGM1 1 km squares per axis" bug that
// silently dropped the middle square of a three-square-spanning level-2
// tile (a west/south edge at km-remainder 977..999), producing a NoData
// stripe through otherwise-covered terrain. See dem.cpp's own header
// comment and this project's CLAUDE.md for the fix's context.

#include "g2m/import/dem.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdint>

using g2m::import::dgm1_km_span;
using g2m::import::KmSpan;

namespace {
int span_count(const KmSpan& s) { return s.hi - s.lo + 1; }
} // namespace

TEST_CASE("dgm1_km_span: west/south mod 1000 in {0,1,23,976} spans exactly two km squares",
          "[import][dem]") {
    // A level-2 tile is 1024 m; picking bases well away from zero keeps the
    // "mod 1000" framing readable while still exercising floor_div's
    // ordinary (non-negative) branch.
    for (const std::int64_t min_m : {100000LL, 100001LL, 100023LL, 100976LL}) {
        const KmSpan s = dgm1_km_span(min_m, 1024);
        CHECK(span_count(s) == 2);
        CHECK(s.lo == min_m / 1000);
        CHECK(s.hi == s.lo + 1);
    }
}

TEST_CASE("dgm1_km_span: west/south mod 1000 in {977,992,999} spans exactly three km squares",
          "[import][dem]") {
    for (const std::int64_t min_m : {100977LL, 100992LL, 100999LL}) {
        const KmSpan s = dgm1_km_span(min_m, 1024);
        CHECK(span_count(s) == 3);
        CHECK(s.lo == min_m / 1000);
        CHECK(s.hi == s.lo + 2);
    }
}

TEST_CASE("dgm1_km_span: negative coordinates use floor_div, not truncation toward zero",
          "[import][dem]") {
    // -24 = 1000*(-1) + 976 in the floor sense (remainder 976, the "two
    // squares" band) - truncating division would instead give -24/1000 == 0
    // with remainder -24, a different (and wrong) square entirely.
    {
        const KmSpan s = dgm1_km_span(-24, 1024);
        CHECK(s.lo == -1);
        CHECK(s.hi == 0);
        CHECK(span_count(s) == 2);
    }
    // -1 = 1000*(-1) + 999 in the floor sense (remainder 999, the "three
    // squares" band).
    {
        const KmSpan s = dgm1_km_span(-1, 1024);
        CHECK(s.lo == -1);
        CHECK(s.hi == 1);
        CHECK(span_count(s) == 3);
    }
    // An exact multiple of -1000 (remainder 0, the "two squares" band).
    {
        const KmSpan s = dgm1_km_span(-2000, 1024);
        CHECK(s.lo == -2);
        CHECK(s.hi == -1);
        CHECK(span_count(s) == 2);
    }
}

TEST_CASE("dgm1_km_span: a sub-1000m extent that does not straddle a boundary spans one square",
          "[import][dem]") {
    // Not a level-2 tile's own footprint (which is always 1024 m > 1000 m
    // and therefore always straddles at least one boundary), but a useful
    // sanity check on the helper's own general contract.
    const KmSpan s = dgm1_km_span(474000, 500);
    CHECK(s.lo == 474);
    CHECK(s.hi == 474);
    CHECK(span_count(s) == 1);
}
