// tests/unit/test_geom_exact.cpp - g2m::geom::orient/segments_intersect
// (g2m/core/geom/exact.h, PLAN G1c I1): explicit edge cases, plus the same
// "small-scale ground truth, then invariant under an affine transform near
// the documented |coordinate| < 2^62 limit" pattern
// tests/layer/test_osm.cpp's own segment_touches_box test uses.

#include "g2m/core/geom/exact.h"

#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdint>

using namespace g2m::geom;

namespace {

PointMm pt(std::int64_t x, std::int64_t y) { return PointMm{x, y}; }

int sgn(std::int64_t v) { return v > 0 ? 1 : (v < 0 ? -1 : 0); }

// Ground truth for orient() at small coordinates (|v| <= 20, so the cross
// product fits safely in a plain int64 - no U128 needed): a single
// subtraction of two int64 products, independent of exact.cpp's own
// sign_diff_products path.
int reference_orient(PointMm a, PointMm b, PointMm c) {
    return sgn((b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x));
}

struct RefHit {
    bool hit;
    bool proper;
};

// Ground truth for segments_intersect() at small coordinates: the same
// CLRS-style case analysis, but every cross product computed directly in
// plain int64 (not via g2m::geom::orient), so this independently checks
// the arithmetic exact.cpp's segments_intersect performs through the
// exact/U128 path.
RefHit reference_intersect(PointMm a, PointMm b, PointMm c, PointMm d) {
    const int d1 = reference_orient(c, d, a);
    const int d2 = reference_orient(c, d, b);
    const int d3 = reference_orient(a, b, c);
    const int d4 = reference_orient(a, b, d);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        return {true, true};
    }
    const auto on_seg = [](PointMm p, PointMm q, PointMm r) {
        return std::min(p.x, q.x) <= r.x && r.x <= std::max(p.x, q.x) && std::min(p.y, q.y) <= r.y &&
               r.y <= std::max(p.y, q.y);
    };
    bool hit = false;
    if (d1 == 0 && on_seg(c, d, a)) hit = true;
    if (d2 == 0 && on_seg(c, d, b)) hit = true;
    if (d3 == 0 && on_seg(a, b, c)) hit = true;
    if (d4 == 0 && on_seg(a, b, d)) hit = true;
    return {hit, false};
}

} // namespace

TEST_CASE("geom: orient explicit cases", "[math][geom]") {
    CHECK(orient(pt(0, 0), pt(1, 0), pt(0, 1)) > 0);  // left turn (CCW)
    CHECK(orient(pt(0, 0), pt(0, 1), pt(1, 0)) < 0);  // right turn (CW)
    CHECK(orient(pt(0, 0), pt(2, 0), pt(1, 0)) == 0); // collinear, between
    CHECK(orient(pt(0, 0), pt(1, 0), pt(2, 0)) == 0); // collinear, beyond b
    CHECK(orient(pt(0, 0), pt(1, 0), pt(-1, 0)) == 0); // collinear, before a
    CHECK(orient(pt(5, 5), pt(5, 5), pt(1, 1)) == 0);  // degenerate a == b
}

TEST_CASE("geom: orient matches reference at small scale, and is invariant at scale near 2^62", "[math][geom]") {
    g2m_test::SplitMix64 r(101);
    const auto v = [&] { return static_cast<std::int64_t>(r.next() % 41) - 20; };
    for (int iter = 0; iter < 200000; ++iter) {
        const PointMm a = pt(v(), v());
        const PointMm b = pt(v(), v());
        const PointMm c = pt(v(), v());
        const int ref = reference_orient(a, b, c);
        REQUIRE(orient(a, b, c) == ref);
        // Scaled by 2^40 and shifted near the documented |v| < 2^62 limit
        // (mirrors test_osm.cpp's segment_touches_box test): orientation
        // sign is invariant under a positive uniform scale plus a
        // per-axis translation, and every product here needs 128 bits.
        constexpr std::int64_t s = std::int64_t{1} << 40;
        constexpr std::int64_t off = std::int64_t{1} << 61;
        const auto scale = [&](PointMm p) { return pt(p.x * s + off, p.y * s - off); };
        REQUIRE(orient(scale(a), scale(b), scale(c)) == ref);
    }
}

TEST_CASE("geom: segments_intersect explicit cases", "[math][geom]") {
    bool proper = false;

    // Proper crossing.
    CHECK(segments_intersect(pt(0, 0), pt(2, 2), pt(0, 2), pt(2, 0), &proper));
    CHECK(proper);

    // Shared endpoint only.
    CHECK(segments_intersect(pt(0, 0), pt(1, 1), pt(1, 1), pt(2, 0), &proper));
    CHECK_FALSE(proper);

    // T-touch: one segment's endpoint lies in the interior of the other.
    CHECK(segments_intersect(pt(0, 0), pt(2, 0), pt(1, 0), pt(1, 1), &proper));
    CHECK_FALSE(proper);

    // Collinear, overlapping (a run, not a point).
    CHECK(segments_intersect(pt(0, 0), pt(4, 0), pt(2, 0), pt(6, 0), &proper));
    CHECK_FALSE(proper);

    // Collinear, touching at exactly one point.
    CHECK(segments_intersect(pt(0, 0), pt(2, 0), pt(2, 0), pt(4, 0), &proper));
    CHECK_FALSE(proper);

    // Collinear, disjoint.
    CHECK_FALSE(segments_intersect(pt(0, 0), pt(2, 0), pt(3, 0), pt(5, 0), &proper));
    CHECK_FALSE(proper);

    // Parallel, disjoint (not collinear).
    CHECK_FALSE(segments_intersect(pt(0, 0), pt(2, 0), pt(0, 1), pt(2, 1), &proper));
    CHECK_FALSE(proper);

    // Disjoint bounding boxes.
    CHECK_FALSE(segments_intersect(pt(0, 0), pt(1, 1), pt(10, 10), pt(11, 11), &proper));

    // proper defaults to null and is optional.
    CHECK(segments_intersect(pt(0, 0), pt(2, 2), pt(0, 2), pt(2, 0)));
    CHECK_FALSE(segments_intersect(pt(0, 0), pt(1, 1), pt(10, 10), pt(11, 11)));
}

TEST_CASE("geom: segments_intersect matches reference at small scale, and is invariant at scale near 2^62",
         "[math][geom]") {
    g2m_test::SplitMix64 r(202);
    const auto v = [&] { return static_cast<std::int64_t>(r.next() % 41) - 20; };
    int hits = 0, propers = 0;
    for (int iter = 0; iter < 200000; ++iter) {
        const PointMm a = pt(v(), v());
        const PointMm b = pt(v(), v());
        const PointMm c = pt(v(), v());
        const PointMm d = pt(v(), v());
        const RefHit ref = reference_intersect(a, b, c, d);
        hits += ref.hit ? 1 : 0;
        propers += ref.proper ? 1 : 0;

        bool proper = !ref.proper; // start disagreeing, so a no-op bug can't pass silently
        const bool hit = segments_intersect(a, b, c, d, &proper);
        REQUIRE(hit == ref.hit);
        REQUIRE(proper == ref.proper);

        // Scaled by 2^40 and shifted near the documented |v| < 2^62 limit:
        // the answer is invariant, and every product needs 128 bits.
        constexpr std::int64_t s = std::int64_t{1} << 40;
        constexpr std::int64_t off = std::int64_t{1} << 61;
        const auto scale = [&](PointMm p) { return pt(p.x * s + off, p.y * s - off); };
        bool proper_scaled = !ref.proper;
        const bool hit_scaled = segments_intersect(scale(a), scale(b), scale(c), scale(d), &proper_scaled);
        REQUIRE(hit_scaled == ref.hit);
        REQUIRE(proper_scaled == ref.proper);
    }
    CHECK(hits > 1000);
    CHECK(hits < 199000);
    CHECK(propers > 100);
}
