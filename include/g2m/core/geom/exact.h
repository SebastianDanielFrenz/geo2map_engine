// g2m/core/geom/exact.h - exact integer geometry predicates (G1c plan I1):
// the building blocks for polygon assembly, point-in-polygon and
// tile/segment tests, shared by osm_clip.cpp and the new ring_set /
// ring_assembly code. Every predicate is EXACT (128-bit intermediate
// products, no floating point) for coordinates with |value| < 2^62
// millimetres - about 4,611 km, far beyond any UTM zone's extent.
#pragma once

#include <cstdint>

namespace g2m::geom {

// A point in integer millimetres (a projected OSM node, a ring vertex, a
// tile corner, ...).
struct PointMm {
    std::int64_t x = 0, y = 0;

    friend bool operator==(const PointMm&, const PointMm&) = default;
};

// An axis-aligned box in integer millimetres, CLOSED (both edges included).
struct BoxMm {
    std::int64_t w = 0, s = 0, e = 0, n = 0;

    bool valid() const { return w <= e && s <= n; }

    friend bool operator==(const BoxMm&, const BoxMm&) = default;
};

// sign(a*b - c*d), exact for any int64 a, b, c, d (a 128-bit intermediate
// product on each side, no overflow).
int sign_diff_products(std::int64_t a, std::int64_t b, std::int64_t c, std::int64_t d);

// Orientation of the turn a -> b -> c: > 0 left (CCW), < 0 right (CW), 0
// collinear. Exact for |coordinate| < 2^62.
int orient(PointMm a, PointMm b, PointMm c);

// Does segment (ax,ay)-(bx,by) touch the closed box [x0,x1] x [y0,y1]?
// Exact for all int64 inputs with |value| < 2^62. (Moved out of
// osm_clip.cpp verbatim, PLAN G1c I1 - clip_osm's own way-selection test.)
bool segment_touches_box(std::int64_t ax, std::int64_t ay, std::int64_t bx, std::int64_t by, std::int64_t x0,
                          std::int64_t y0, std::int64_t x1, std::int64_t y1);

// Do segments (a,b) and (c,d) share any point? Exact. If `proper` is not
// null, it is always written: true iff the segments cross at a single
// point interior to both (not at an endpoint, and not a collinear
// overlap), false otherwise (including every "touching" case: a shared
// endpoint, a T-touch, or a collinear overlap).
bool segments_intersect(PointMm a, PointMm b, PointMm c, PointMm d, bool* proper = nullptr);

} // namespace g2m::geom
