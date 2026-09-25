// src/core/geom/exact.cpp - see g2m/core/geom/exact.h.
//
// sign_diff_products/segment_touches_box (and their U128 helpers) were
// moved here verbatim from src/layer/osm_clip.cpp (PLAN G1c I1) so
// ring_set.cpp/ring_assembly.cpp can share them without a g2m_layers
// dependency; osm_clip.cpp now calls this header instead of defining its
// own copy. Behaviour is byte-identical (same algorithm, same operations).

#include "g2m/core/geom/exact.h"

#include <algorithm>
#include <cstdint>

namespace g2m::geom {
namespace {

struct U128 {
    std::uint64_t hi, lo;
};

U128 umul(std::uint64_t a, std::uint64_t b) {
    const std::uint64_t a0 = a & 0xffffffffu, a1 = a >> 32;
    const std::uint64_t b0 = b & 0xffffffffu, b1 = b >> 32;
    const std::uint64_t p00 = a0 * b0, p01 = a0 * b1, p10 = a1 * b0, p11 = a1 * b1;
    const std::uint64_t mid = (p00 >> 32) + (p01 & 0xffffffffu) + (p10 & 0xffffffffu);
    return U128{p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32), (mid << 32) | (p00 & 0xffffffffu)};
}

int ucmp(U128 x, U128 y) {
    if (x.hi != y.hi) return x.hi < y.hi ? -1 : 1;
    if (x.lo != y.lo) return x.lo < y.lo ? -1 : 1;
    return 0;
}

std::uint64_t mag(std::int64_t v) {
    return v < 0 ? (0 - static_cast<std::uint64_t>(v)) : static_cast<std::uint64_t>(v);
}

} // namespace

int sign_diff_products(std::int64_t a, std::int64_t b, std::int64_t c, std::int64_t d) {
    const U128 p = umul(mag(a), mag(b));
    const U128 q = umul(mag(c), mag(d));
    const bool pz = p.hi == 0 && p.lo == 0;
    const bool qz = q.hi == 0 && q.lo == 0;
    const int ps = pz ? 0 : ((a < 0) != (b < 0) ? -1 : 1);
    const int qs = qz ? 0 : ((c < 0) != (d < 0) ? -1 : 1);
    if (ps != qs) return ps > qs ? 1 : -1;
    if (ps == 0) return 0;
    const int m = ucmp(p, q);
    return ps > 0 ? m : -m;
}

int orient(PointMm a, PointMm b, PointMm c) {
    return sign_diff_products(b.x - a.x, c.y - a.y, b.y - a.y, c.x - a.x);
}

bool segment_touches_box(std::int64_t ax, std::int64_t ay, std::int64_t bx, std::int64_t by, std::int64_t x0,
                          std::int64_t y0, std::int64_t x1, std::int64_t y1) {
    // Separating axes: x, y, and the segment's own normal.
    if (std::max(ax, bx) < x0 || std::min(ax, bx) > x1 || std::max(ay, by) < y0 || std::min(ay, by) > y1) {
        return false;
    }
    const std::int64_t dx = bx - ax, dy = by - ay;
    int pos = 0, neg = 0;
    const std::int64_t cx[4] = {x0, x1, x1, x0};
    const std::int64_t cy[4] = {y0, y0, y1, y1};
    for (int i = 0; i < 4; ++i) {
        const int sg = sign_diff_products(dx, cy[i] - ay, dy, cx[i] - ax);
        if (sg == 0) return true; // a box corner lies exactly on the segment's line, and the axes overlap
        (sg > 0 ? pos : neg)++;
    }
    return pos > 0 && neg > 0;
}

bool segments_intersect(PointMm a, PointMm b, PointMm c, PointMm d, bool* proper) {
    if (proper) *proper = false;
    const int d1 = orient(c, d, a);
    const int d2 = orient(c, d, b);
    const int d3 = orient(a, b, c);
    const int d4 = orient(a, b, d);
    if (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) && ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) {
        if (proper) *proper = true;
        return true;
    }
    const auto on_segment = [](PointMm p, PointMm q, PointMm r) {
        return std::min(p.x, q.x) <= r.x && r.x <= std::max(p.x, q.x) && std::min(p.y, q.y) <= r.y &&
               r.y <= std::max(p.y, q.y);
    };
    if (d1 == 0 && on_segment(c, d, a)) return true;
    if (d2 == 0 && on_segment(c, d, b)) return true;
    if (d3 == 0 && on_segment(a, b, c)) return true;
    if (d4 == 0 && on_segment(a, b, d)) return true;
    return false;
}

} // namespace g2m::geom
