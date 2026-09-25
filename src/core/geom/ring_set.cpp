// src/core/geom/ring_set.cpp - see g2m/core/geom/ring_set.h.

#include "g2m/core/geom/ring_set.h"

#include <algorithm>
#include <limits>
#include <map>

namespace g2m::geom {
namespace {

struct Seg {
    PointMm a, b;
};

void add_ring_segments(const Ring& ring, std::vector<Seg>& out) {
    const std::size_t n = ring.size();
    for (std::size_t i = 0; i < n; ++i) out.push_back(Seg{ring[i], ring[(i + 1) % n]});
}

std::vector<Seg> collect_segments(const RingSet& rings) {
    std::vector<Seg> segs;
    for (const Ring& r : rings.outers) add_ring_segments(r, segs);
    for (const Ring& r : rings.inners) add_ring_segments(r, segs);
    return segs;
}

bool on_segment_inclusive(PointMm p, PointMm q, PointMm r) {
    return std::min(p.x, q.x) <= r.x && r.x <= std::max(p.x, q.x) && std::min(p.y, q.y) <= r.y &&
           r.y <= std::max(p.y, q.y);
}

// Ray-casts `p` against one ring, folding into `crossing` (XOR toggle).
// Returns true immediately (an "inside" short-circuit) iff `p` lies
// exactly on one of the ring's own edges.
bool ring_contains_edge_toggle(const Ring& ring, PointMm p, bool& crossing) {
    const std::size_t n = ring.size();
    for (std::size_t i = 0; i < n; ++i) {
        const PointMm a = ring[i], b = ring[(i + 1) % n];
        const int side = orient(a, b, p);
        if (side == 0 && on_segment_inclusive(a, b, p)) return true;
        if ((a.y > p.y) != (b.y > p.y)) {
            // The edge crosses the horizontal line y = p.y strictly (so
            // `side` above is guaranteed nonzero here - see ring_set.h's
            // design note / the derivation in the G1c I1 report): toggle
            // when the crossing point is to the right of p (equivalent to
            // Franklin's PNPOLY test, restated as an exact sign compare).
            if ((side > 0) == (b.y > a.y)) crossing = !crossing;
        }
    }
    return false;
}

double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

double point_box_dist2(PointMm p, BoxMm b) {
    const double cx = clampd(static_cast<double>(p.x), static_cast<double>(b.w), static_cast<double>(b.e));
    const double cy = clampd(static_cast<double>(p.y), static_cast<double>(b.s), static_cast<double>(b.n));
    const double dx = static_cast<double>(p.x) - cx, dy = static_cast<double>(p.y) - cy;
    return dx * dx + dy * dy;
}

double point_segment_dist2(PointMm p, PointMm a, PointMm b) {
    const double abx = static_cast<double>(b.x - a.x), aby = static_cast<double>(b.y - a.y);
    const double apx = static_cast<double>(p.x - a.x), apy = static_cast<double>(p.y - a.y);
    const double ab2 = abx * abx + aby * aby;
    double t = ab2 > 0.0 ? (apx * abx + apy * aby) / ab2 : 0.0;
    t = clampd(t, 0.0, 1.0);
    const double cx = static_cast<double>(a.x) + t * abx, cy = static_cast<double>(a.y) + t * aby;
    const double dx = static_cast<double>(p.x) - cx, dy = static_cast<double>(p.y) - cy;
    return dx * dx + dy * dy;
}

// Minimum squared distance between box `box` and segment (a,b): the
// general convex-convex minimum is achieved at a vertex of one against the
// other (box corner vs. segment, or segment endpoint vs. box).
double box_segment_dist2(BoxMm box, PointMm a, PointMm b) {
    double d = point_box_dist2(a, box);
    d = std::min(d, point_box_dist2(b, box));
    const PointMm corners[4] = {{box.w, box.s}, {box.e, box.s}, {box.e, box.n}, {box.w, box.n}};
    for (const PointMm& c : corners) d = std::min(d, point_segment_dist2(c, a, b));
    return d;
}

TileRelation classify_with_segments(const RingSet& rings, BoxMm sq, double halo_m, const std::vector<Seg>& segs) {
    for (const Seg& s : segs) {
        if (segment_touches_box(s.a.x, s.a.y, s.b.x, s.b.y, sq.w, sq.s, sq.e, sq.n)) return TileRelation::Boundary;
    }
    if (contains(rings, PointMm{sq.w, sq.s})) return TileRelation::Interior;
    if (halo_m > 0.0) {
        const double h_mm = halo_m * 1000.0;
        const double h2 = h_mm * h_mm;
        for (const Seg& s : segs) {
            if (box_segment_dist2(sq, s.a, s.b) <= h2) return TileRelation::Boundary;
        }
    }
    return TileRelation::Outside;
}

} // namespace

BoxMm bounding_box(const RingSet& rings) {
    BoxMm b{std::numeric_limits<std::int64_t>::max(), std::numeric_limits<std::int64_t>::max(),
            std::numeric_limits<std::int64_t>::min(), std::numeric_limits<std::int64_t>::min()};
    const auto add_ring = [&](const Ring& r) {
        for (const PointMm& p : r) {
            b.w = std::min(b.w, p.x);
            b.e = std::max(b.e, p.x);
            b.s = std::min(b.s, p.y);
            b.n = std::max(b.n, p.y);
        }
    };
    for (const Ring& r : rings.outers) add_ring(r);
    for (const Ring& r : rings.inners) add_ring(r);
    return b;
}

bool contains(const RingSet& rings, PointMm p) {
    bool crossing = false;
    for (const Ring& r : rings.outers) {
        if (ring_contains_edge_toggle(r, p, crossing)) return true;
    }
    for (const Ring& r : rings.inners) {
        if (ring_contains_edge_toggle(r, p, crossing)) return true;
    }
    return crossing;
}

TileRelation classify_square(const RingSet& rings, BoxMm sq, double halo_m) {
    return classify_with_segments(rings, sq, halo_m, collect_segments(rings));
}

std::vector<std::pair<TileKey, TileRelation>> tiles_near(const RingSet& rings, geo::UtmZone zone, std::uint8_t level,
                                                          std::uint32_t halo_m) {
    std::vector<std::pair<TileKey, TileRelation>> out;
    if (!rings.bbox.valid()) return out;

    const std::vector<Seg> segs = collect_segments(rings);
    if (segs.empty()) return out;

    const std::int64_t size_m = TileKey::kLevel0SizeM << level;
    const std::int64_t halo_mm = static_cast<std::int64_t>(halo_m) * 1000;

    const auto cell_of = [&](std::int64_t mm) { return floor_div(floor_div(mm, 1000), size_m); };

    const std::int64_t x0 = cell_of(rings.bbox.w - halo_mm);
    const std::int64_t x1 = cell_of(rings.bbox.e + halo_mm);
    const std::int64_t y0 = cell_of(rings.bbox.s - halo_mm);
    const std::int64_t y1 = cell_of(rings.bbox.n + halo_mm);

    // Uniform grid bucket (cell = tile size): each segment is inserted into
    // every cell its own bbox, dilated by halo_mm on every side, overlaps -
    // a superset of "could this segment matter for tile T's touch or
    // within-halo test" (Euclidean distance <= halo implies both the x and
    // y gaps are <= halo, so this Chebyshev dilation never misses a real
    // candidate; see the G1c I1 report for the short proof).
    std::map<std::pair<std::int64_t, std::int64_t>, std::vector<std::uint32_t>> buckets;
    for (std::uint32_t i = 0; i < segs.size(); ++i) {
        const Seg& s = segs[i];
        const std::int64_t bw = std::min(s.a.x, s.b.x) - halo_mm;
        const std::int64_t be = std::max(s.a.x, s.b.x) + halo_mm;
        const std::int64_t bs = std::min(s.a.y, s.b.y) - halo_mm;
        const std::int64_t bn = std::max(s.a.y, s.b.y) + halo_mm;
        const std::int64_t cx0 = cell_of(bw), cx1 = cell_of(be), cy0 = cell_of(bs), cy1 = cell_of(bn);
        for (std::int64_t cy = cy0; cy <= cy1; ++cy) {
            for (std::int64_t cx = cx0; cx <= cx1; ++cx) buckets[{cx, cy}].push_back(i);
        }
    }

    const double halo_m_d = static_cast<double>(halo_m);
    for (std::int64_t y = y0; y <= y1; ++y) {
        for (std::int64_t x = x0; x <= x1; ++x) {
            if (x < TileKey::kIndexMin || x > TileKey::kIndexMax || y < TileKey::kIndexMin ||
                y > TileKey::kIndexMax) {
                continue;
            }
            const TileKey key{zone, level, static_cast<std::int32_t>(x), static_cast<std::int32_t>(y)};
            if (!key.valid()) continue;
            const std::int64_t sw_e = key.min_easting() * 1000, sw_n = key.min_northing() * 1000;
            const BoxMm sq{sw_e, sw_n, sw_e + size_m * 1000, sw_n + size_m * 1000};
            std::vector<Seg> local;
            const auto it = buckets.find({x, y});
            if (it != buckets.end()) {
                local.reserve(it->second.size());
                for (std::uint32_t si : it->second) local.push_back(segs[si]);
            }
            const TileRelation rel = classify_with_segments(rings, sq, halo_m_d, local);
            if (rel != TileRelation::Outside) out.push_back({key, rel});
        }
    }
    std::sort(out.begin(), out.end(),
              [](const auto& a, const auto& b) { return a.first.packed() < b.first.packed(); });
    return out;
}

} // namespace g2m::geom
