#include "g2m/layer/road_junction.h"
#include "g2m/core/det_math.h"
#include "g2m/layer/road_attributes.h"
#include "g2m/layer/road_cross_section.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <string_view>
#include <tuple>

namespace g2m {
namespace {
using geom::PointMm;
constexpr int kLayers = 3;
constexpr int kRight = 0, kLeft = 1;      // sides in the OUTWARD frame of a dart (sigma away from its node)
constexpr double kTrimClampM = 0.5;       // trim <= L - 0.5 (design 3.4); also the polyline end margin of 3.3
constexpr std::size_t kNone = std::numeric_limits<std::size_t>::max();
constexpr double kNegInf = -std::numeric_limits<double>::infinity();

struct Tagged { PointMm p; std::uint16_t tag = 0; };

struct Key {
    std::int64_t way = 0; std::uint32_t start = 0; bool at_end = false;
    friend bool operator<(const Key& a, const Key& b) { return std::tie(a.way, a.start, a.at_end) < std::tie(b.way, b.start, b.at_end); }
};

// One end of a used stretch at a junction node: a STUB (an approach that keeps a core beyond its trim) or, in a
// cluster, an INTERNAL dart (one end of an absorbed stretch; its twin is the other end). Edge polylines per design
// 3.3: stubs are sampled out to min(L - 0.5, reach), internal darts over their whole length.
struct Dart {
    std::size_t entry = 0;
    bool at_end = false;
    Key key;
    std::size_t node = 0;                                         // index into the junction's node list
    bool internal = false;
    std::size_t twin = kNone;                                     // internal: the dart of the other end
    const RoadReferenceLine* ref = nullptr;
    const RoadCrossSection* xs = nullptr;
    bool bridge = false;
    double length = 0;
    std::vector<double> sigma;                                    // sample stations, ascending, sigma.front() == 0
    std::array<std::array<std::vector<PointMm>, 2>, kLayers> edge; // [layer][side][sample]
    PointMm origin;                                               // reference point at sigma 0 (the cap's inner end)
    PointMm direction;                                            // first reference segment
};

// sigma_a / sigma_b: the station on the left-edge dart (a) and on the right-edge dart (b). A station in [-1, 0)
// marks a corner on that dart's CAP (the segment from its reference point at sigma 0 out to the edge at sigma 0):
// -1 at the reference point, 0 at the edge.
struct Corner {
    bool found = false;
    PointMm point;
    double sigma_a = 0, sigma_b = 0;
    // Corner fillet (kerb return): the ring leaves A's left edge at station fillet_a, follows `arc` (its first point
    // on A's edge, its last on B's right edge) and joins B's right edge at fillet_b. The sharp corner point is unused.
    bool fillet = false;
    double fillet_a = 0, fillet_b = 0;
    std::vector<PointMm> arc;
    double from_a() const { return fillet ? fillet_a : sigma_a; }
    double from_b() const { return fillet ? fillet_b : sigma_b; }
};

std::int64_t floor_div(std::int64_t n, std::int64_t d) {   // d > 0
    std::int64_t q = n / d;
    if ((n % d != 0) && (n < 0)) --q;
    return q;
}
// round-half-up of n / d for d > 0, integer only.
std::int64_t round_div(std::int64_t n, std::int64_t d) { return floor_div(2 * n + d, 2 * d); }
std::int64_t cross(PointMm a, PointMm b) { return a.x * b.y - a.y * b.x; }
PointMm minus(PointMm a, PointMm b) { return {a.x - b.x, a.y - b.y}; }
PointMm to_mm(const LateralPoint& p) { return {std::llround(p.x * 1000.0), std::llround(p.y * 1000.0)}; }

double edge_t(const CarveFootprint& f, int layer, int side, bool at_end) {
    const bool reference_left = (side == kLeft) != at_end;
    switch (layer) {
    case 0: return reference_left ? f.roadbed_left : f.roadbed_right;
    case 1: return reference_left ? f.paved_left : f.paved_right;
    default: return reference_left ? f.outer_left : f.outer_right;
    }
}

bool place(const Dart& g, double sigma, int layer, int side, PointMm& out) {
    const double s = std::clamp(g.at_end ? g.length - sigma : sigma, 0.0, g.length);
    const CarveFootprint f = carve_footprint(*g.xs, s);
    const auto p = lateral_point(*g.ref, s, edge_t(f, layer, side, g.at_end));
    if (!p) return false;
    out = to_mm(*p);
    return true;
}

// The edge polylines, the cap origin and the first direction of one dart. False if a point cannot be placed.
bool sample_dart(Dart& g, const RoadRules& rules) {
    const double spacing = static_cast<double>(rules.junction.edge_station_spacing_mm) / 1000.0;
    const double reach = static_cast<double>(rules.junction.reach_max_m);
    const double sigma_max = g.internal ? g.length : std::max(0.0, std::min(g.length - kTrimClampM, reach));
    g.sigma.clear();
    for (std::int64_t k = 0;; ++k) {
        const double sigma = static_cast<double>(k) * spacing;
        if (sigma >= sigma_max) break;
        g.sigma.push_back(sigma);
    }
    g.sigma.push_back(sigma_max);
    // sigma_max == 0 leaves one sample at 0 (a stretch shorter than the clamp): no segments.
    if (g.sigma.size() >= 2 && g.sigma[0] == g.sigma[1]) g.sigma.pop_back();
    for (auto& layer : g.edge) for (auto& side : layer) { side.clear(); side.reserve(g.sigma.size()); }
    std::array<PointMm, 2> reference{};
    for (std::size_t k = 0; k < g.sigma.size(); ++k) {
        for (int layer = 0; layer < kLayers; ++layer) for (int side = 0; side < 2; ++side) {
            PointMm p;
            if (!place(g, g.sigma[k], layer, side, p)) return false;
            g.edge[layer][side].push_back(p);
        }
        if (k < 2) {
            const double s = std::clamp(g.at_end ? g.length - g.sigma[k] : g.sigma[k], 0.0, g.length);
            const auto p = lateral_point(*g.ref, s, 0.0);
            if (!p) return false;
            reference[k] = to_mm(*p);
        }
    }
    if (g.sigma.size() < 2) return false;
    g.origin = reference[0];
    g.direction = minus(reference[1], reference[0]);
    return g.direction.x != 0 || g.direction.y != 0;
}

// First intersection of the outward-left boundary of A with the outward-right boundary of B on one layer (design
// 3.3): in increasing max(segment index of A, segment index of B), then increasing index of A. Each boundary is the
// dart's CAP (segment 0: reference point at sigma 0 to the edge at sigma 0) followed by its edge polyline, so a pair
// whose edge lines meet behind the node (an obtuse pair with unequal offsets) closes on the cap instead of falling
// back to the open chord (slice 4, decision D17). Cap against cap (both at the node) never counts. Parallel and
// collinear segment pairs never produce a corner.
Corner find_corner(const Dart& A, const Dart& B, int layer, bool use_caps) {
    const auto& pa = A.edge[layer][kLeft];
    const auto& pb = B.edge[layer][kRight];
    Corner none;
    if (pa.size() < 2 || pb.size() < 2) return none;
    // Vertex i of a boundary: i == 0 is the cap origin, i >= 1 is edge sample i - 1.
    const auto va = [&](std::size_t i) { return i == 0 ? A.origin : pa[i - 1]; };
    const auto vb = [&](std::size_t i) { return i == 0 ? B.origin : pb[i - 1]; };
    const auto sa = [&](std::size_t i) { return i == 0 ? -1.0 : A.sigma[i - 1]; };
    const auto sb = [&](std::size_t i) { return i == 0 ? -1.0 : B.sigma[i - 1]; };
    const bool cap_a = use_caps && A.origin != pa[0], cap_b = use_caps && B.origin != pb[0];
    const std::size_t na = pa.size(), nb = pb.size();   // segments including the cap
    const auto attempt = [&](std::size_t ia, std::size_t ib, Corner& out) {
        if ((ia == 0 && !cap_a) || (ib == 0 && !cap_b) || (ia == 0 && ib == 0)) return false;
        const PointMm a = va(ia), b = va(ia + 1), c = vb(ib), d = vb(ib + 1);
        if (std::max(a.x, b.x) < std::min(c.x, d.x) || std::max(c.x, d.x) < std::min(a.x, b.x) ||
            std::max(a.y, b.y) < std::min(c.y, d.y) || std::max(c.y, d.y) < std::min(a.y, b.y)) return false;
        if (!geom::segments_intersect(a, b, c, d)) return false;
        const PointMm r = minus(b, a), s = minus(d, c), ca = minus(c, a);
        std::int64_t den = cross(r, s), num_u = cross(ca, s), num_v = cross(ca, r);
        if (den == 0) return false;
        if (den < 0) { den = -den; num_u = -num_u; num_v = -num_v; }
        out.found = true;
        out.point = {a.x + round_div(r.x * num_u, den), a.y + round_div(r.y * num_u, den)};
        out.sigma_a = sa(ia) + static_cast<double>(num_u) / static_cast<double>(den) * (sa(ia + 1) - sa(ia));
        out.sigma_b = sb(ib) + static_cast<double>(num_v) / static_cast<double>(den) * (sb(ib + 1) - sb(ib));
        return true;
    };
    const std::size_t m_max = std::max(na, nb);
    Corner hit;
    for (std::size_t m = 0; m < m_max; ++m) {
        for (std::size_t ia = 0; ia <= m && ia < na; ++ia) {
            if (ia < m) { if (m < nb && attempt(ia, m, hit)) return hit; }
            else for (std::size_t ib = 0; ib <= m && ib < nb; ++ib) if (attempt(m, ib, hit)) return hit;
        }
    }
    return none;
}

// ---- rings ----

void clean_ring(std::vector<Tagged>& v) {
    bool changed = true;
    while (changed && v.size() >= 2) {
        changed = false;
        for (std::size_t i = 0; i < v.size() && v.size() >= 2;) {   // consecutive duplicates, cyclic
            const std::size_t next = (i + 1) % v.size();
            if (v[i].p == v[next].p) { v.erase(v.begin() + static_cast<std::ptrdiff_t>(i)); changed = true; }
            else ++i;
        }
        for (std::size_t i = 0; i < v.size() && v.size() >= 3;) {   // straight interior points (not spikes)
            const std::size_t prev = (i + v.size() - 1) % v.size(), next = (i + 1) % v.size();
            const PointMm a = v[prev].p, b = v[i].p, c = v[next].p;
            if (geom::orient(a, b, c) == 0 && (b.x - a.x) * (c.x - b.x) + (b.y - a.y) * (c.y - b.y) > 0) {
                v.erase(v.begin() + static_cast<std::ptrdiff_t>(i)); changed = true;
            } else ++i;
        }
    }
}

std::int64_t twice_area(const std::vector<Tagged>& v) {
    std::int64_t sum = 0;
    const PointMm o = v.empty() ? PointMm{} : v[0].p;
    for (std::size_t i = 0; i < v.size(); ++i) sum += cross(minus(v[i].p, o), minus(v[(i + 1) % v.size()].p, o));
    return sum;
}

// Exact simplicity. Crossing segment pairs are appended to `crossings` (as segment indices) when given.
bool ring_simple(const std::vector<Tagged>& v, std::vector<std::pair<std::size_t, std::size_t>>* crossings) {
    const std::size_t n = v.size();
    if (n < 3) return false;
    bool ok = twice_area(v) > 0;
    for (std::size_t i = 0; i < n; ++i) {
        const PointMm a = v[i].p, b = v[(i + 1) % n].p;
        for (std::size_t j = i + 1; j < n; ++j) {
            const PointMm c = v[j].p, d = v[(j + 1) % n].p;
            const bool adjacent = (j == i + 1) || (i == 0 && j == n - 1);
            if (adjacent) {
                // The shared vertex; the other ends must not fold back along the same line.
                const PointMm s = (j == i + 1) ? b : a;
                const PointMm p = (j == i + 1) ? a : c, q = (j == i + 1) ? d : b;   // p - s - q
                if (geom::orient(p, s, q) == 0 && (s.x - p.x) * (q.x - s.x) + (s.y - p.y) * (q.y - s.y) < 0) {
                    ok = false; if (crossings) crossings->push_back({i, j});
                }
                continue;
            }
            if (std::max(a.x, b.x) < std::min(c.x, d.x) || std::max(c.x, d.x) < std::min(a.x, b.x) ||
                std::max(a.y, b.y) < std::min(c.y, d.y) || std::max(c.y, d.y) < std::min(a.y, b.y)) continue;
            if (geom::segments_intersect(a, b, c, d)) { ok = false; if (crossings) crossings->push_back({i, j}); }
        }
    }
    return ok;
}

// Exact: p inside or on the boundary of the ring (crossing number, exact orientations).
bool inside_or_on(const std::vector<PointMm>& v, PointMm p) {
    bool in = false;
    for (std::size_t i = 0, j = v.size() - 1; i < v.size(); j = i++) {
        const PointMm a = v[j], b = v[i];
        const int o = geom::orient(a, b, p);
        if (o == 0 && std::min(a.x, b.x) <= p.x && p.x <= std::max(a.x, b.x) && std::min(a.y, b.y) <= p.y && p.y <= std::max(a.y, b.y)) return true;
        if ((a.y > p.y) != (b.y > p.y) && (b.y > a.y ? o > 0 : o < 0)) in = !in;
    }
    return in;
}
// p within `allowance_mm` of the ring's boundary (+ - * / on coordinate differences, which are exact doubles here).
bool near_boundary(const std::vector<PointMm>& v, PointMm p, double allowance_mm) {
    const double limit = allowance_mm * allowance_mm;
    for (std::size_t i = 0, j = v.size() - 1; i < v.size(); j = i++) {
        const double dx = static_cast<double>(v[i].x - v[j].x), dy = static_cast<double>(v[i].y - v[j].y);
        const double wx = static_cast<double>(p.x - v[j].x), wy = static_cast<double>(p.y - v[j].y);
        const double len2 = dx * dx + dy * dy;
        const double t = len2 > 0 ? std::clamp((wx * dx + wy * dy) / len2, 0.0, 1.0) : 0.0;
        const double ex = wx - t * dx, ey = wy - t * dy;
        if (ex * ex + ey * ey <= limit) return true;
    }
    return false;
}
// Road bed in paved in outer: every vertex inside, on, or within the allowance (design 9.2, D21). Outer rings of >= 3 vertices.
bool rings_nested(const std::array<const std::vector<PointMm>*, kLayers>& rings) {
    for (std::size_t inner = 0; inner + 1 < static_cast<std::size_t>(kLayers); ++inner) {
        const auto& outer = *rings[inner + 1];
        if (outer.size() < 3) return false;
        for (const auto& p : *rings[inner])
            if (!inside_or_on(outer, p) && !near_boundary(outer, p, kJunctionNestingAllowanceMm)) return false;
    }
    return true;
}
bool tagged_rings_nested(const std::array<std::vector<Tagged>, kLayers>& rings) {
    std::array<std::vector<PointMm>, kLayers> plain;
    std::array<const std::vector<PointMm>*, kLayers> views{};
    for (std::size_t layer = 0; layer < plain.size(); ++layer) {
        for (const auto& t : rings[layer]) plain[layer].push_back(t.p);
        views[layer] = &plain[layer];
    }
    return rings_nested(views);
}

// Convex hull (Andrew's monotone chain, exact), counter-clockwise, strictly convex. May return < 3 points.
std::vector<Tagged> convex_hull(std::vector<PointMm> pts) {
    std::sort(pts.begin(), pts.end(), [](PointMm a, PointMm b) { return a.x != b.x ? a.x < b.x : a.y < b.y; });
    pts.erase(std::unique(pts.begin(), pts.end()), pts.end());
    std::vector<Tagged> out;
    if (pts.size() < 3) { for (const auto& p : pts) out.push_back({p, 0}); return out; }
    std::vector<PointMm> h(2 * pts.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < pts.size(); ++i) {
        while (k >= 2 && geom::orient(h[k - 2], h[k - 1], pts[i]) <= 0) --k;
        h[k++] = pts[i];
    }
    for (std::size_t i = pts.size() - 1, t = k + 1; i-- > 0;) {
        while (k >= t && geom::orient(h[k - 2], h[k - 1], pts[i]) <= 0) --k;
        h[k++] = pts[i];
    }
    h.resize(k - 1);
    for (const auto& p : h) out.push_back({p, 0});
    return out;
}

// The boundary walk of a junction: walk[k] is the dart leaving its node at step k. Pair k is (arrival(walk[k]),
// walk[k + 1]) at one node: the left boundary of the first against the right boundary of the second.
struct Frame {
    const std::vector<Dart>* darts = nullptr;
    std::vector<std::size_t> walk;
    std::vector<double> trims;                          // per dart; stubs only
    std::array<std::vector<Corner>, kLayers> corners;   // [layer][pair k]
    std::size_t arrival(std::size_t k) const {
        const Dart& d = (*darts)[walk[k]];
        return d.internal ? d.twin : walk[k];
    }
    double bound(std::size_t dart) const { return (*darts)[dart].internal ? (*darts)[dart].length : trims[dart]; }
    bool corner_valid(int layer, std::size_t k) const {
        const Corner& c = corners[layer][k];
        return c.found && c.from_a() <= bound(arrival(k)) && c.from_b() <= bound(walk[(k + 1) % walk.size()]);
    }
};

// The ring of one layer (design 3.5, generalised to the boundary walk of a cluster): per step the trim points of a
// stub, then the left boundary of the arriving dart back to the pair's corner, the corner, and the right boundary
// of the next stub out to its trim. An internal dart contributes its outer edge between the corners at its two
// ends. An open pair joins the two edges at sigma 0 by a chord. False if a trim point cannot be placed.
bool build_ring(const Frame& fr, int layer, std::vector<Tagged>& out) {
    const auto& darts = *fr.darts;
    const std::size_t m = fr.walk.size();
    out.clear();
    for (std::size_t k = 0; k < m; ++k) {
        const std::size_t D = fr.walk[k], A = fr.arrival(k), N = fr.walk[(k + 1) % m];
        const Dart& dd = darts[D];
        if (!dd.internal) {
            PointMm right, left;
            if (!place(dd, fr.trims[D], layer, kRight, right) || !place(dd, fr.trims[D], layer, kLeft, left)) return false;
            out.push_back({right, static_cast<std::uint16_t>(D)}); out.push_back({left, static_cast<std::uint16_t>(D)});
        }
        const bool cv = fr.corner_valid(layer, k);
        const Corner& c = fr.corners[layer][k];
        const double lo = cv ? c.from_a() : kNegInf;
        const Dart& a = darts[A];
        if (!a.internal) {
            for (std::size_t s = a.sigma.size(); s-- > 0;)
                if (a.sigma[s] < fr.trims[A] && a.sigma[s] > lo) out.push_back({a.edge[layer][kLeft][s], static_cast<std::uint16_t>(A)});
        } else {
            // A runs back along the internal stretch; its far end is the start of D, where pair k - 1 has its corner.
            const std::size_t km1 = (k + m - 1) % m;
            const bool prev = fr.corner_valid(layer, km1);
            const double ub = prev ? a.length - fr.corners[layer][km1].from_b() : a.length;
            for (std::size_t s = a.sigma.size(); s-- > 0;)
                if (a.sigma[s] > lo && (prev ? a.sigma[s] < ub : a.sigma[s] <= ub))
                    out.push_back({a.edge[layer][kLeft][s], static_cast<std::uint16_t>(A)});
        }
        if (cv && c.fillet) for (const auto& p : c.arc) out.push_back({p, static_cast<std::uint16_t>(A)});
        else if (cv) out.push_back({c.point, static_cast<std::uint16_t>(A)});
        const Dart& n = darts[N];
        if (!n.internal) {
            const double from = cv ? c.from_b() : kNegInf;
            for (std::size_t s = 0; s < n.sigma.size(); ++s)
                if (n.sigma[s] < fr.trims[N] && n.sigma[s] > from) out.push_back({n.edge[layer][kRight][s], static_cast<std::uint16_t>(N)});
        }
    }
    clean_ring(out);
    return true;
}


// Star-union ring (design D45): the boundary of the union of every approach strip (its road from the node to its trim
// on this layer) and the node core (convex hull of the approaches' cross-section ends at the node). Every strip and
// the core contain the node, so the union is star-shaped about it and its boundary is the upper envelope of the
// pieces' boundary segments in polar angle: exact between the sorted angles of every vertex and every crossing of two
// pieces' segments. Simple by construction (one point per angle, radius > 0) and, because a wider layer's strips and
// core contain the narrower layer's, nested by construction; it replaces the convex hull of a single-node junction
// whose offset ring could not be repaired, so a broken corner never inflates the junction over the open wedges.
std::vector<Tagged> star_union_ring(const std::vector<Dart>& darts, const std::vector<double>& trims, int layer, PointMm centre) {
    struct Seg { double ax, ay, bx, by; };
    std::vector<std::vector<Seg>> pieces;
    const double ox = static_cast<double>(centre.x), oy = static_cast<double>(centre.y);
    const auto add_piece = [&](const std::vector<PointMm>& poly) {
        if (poly.size() < 3) return;
        std::vector<Seg> segs;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const PointMm a = poly[i], b = poly[(i + 1) % poly.size()];
            if (a == b) continue;
            segs.push_back({static_cast<double>(a.x) - ox, static_cast<double>(a.y) - oy, static_cast<double>(b.x) - ox, static_cast<double>(b.y) - oy});
        }
        if (segs.size() >= 3) pieces.push_back(std::move(segs));
    };
    std::vector<PointMm> core;
    double piece_area = 0;   // sum of the pieces' areas, an upper bound of the union's (2 x mm^2)
    const auto area2 = [](const std::vector<PointMm>& poly) {
        double a = 0;
        for (std::size_t i = 0; i < poly.size(); ++i) {
            const PointMm p = poly[i], q = poly[(i + 1) % poly.size()];
            a += static_cast<double>(p.x - poly[0].x) * static_cast<double>(q.y - poly[0].y) - static_cast<double>(q.x - poly[0].x) * static_cast<double>(p.y - poly[0].y);
        }
        return std::abs(a);
    };
    for (std::size_t i = 0; i < darts.size(); ++i) {
        const Dart& g = darts[i];
        if (g.internal) return {};
        const double trim = trims[i];
        std::vector<PointMm> strip;
        for (std::size_t k = 0; k < g.sigma.size() && g.sigma[k] < trim; ++k) strip.push_back(g.edge[layer][kRight][k]);
        PointMm r, l;
        if (!place(g, trim, layer, kRight, r) || !place(g, trim, layer, kLeft, l)) return {};
        strip.push_back(r); strip.push_back(l);
        for (std::size_t k = g.sigma.size(); k-- > 0;) if (g.sigma[k] < trim) strip.push_back(g.edge[layer][kLeft][k]);
        piece_area += area2(strip);
        add_piece(strip);
        core.push_back(g.edge[layer][kRight][0]); core.push_back(g.edge[layer][kLeft][0]);
    }
    core.push_back(centre);
    std::vector<PointMm> core_ring;
    for (const auto& t : convex_hull(core)) core_ring.push_back(t.p);
    piece_area += area2(core_ring);
    add_piece(core_ring);
    if (pieces.empty()) return {};
    // Candidate angles: every vertex, every crossing of segments of two different pieces.
    std::vector<double> angles;
    for (const auto& piece : pieces) for (const auto& sg : piece) angles.push_back(det::atan2(sg.ay, sg.ax));
    for (std::size_t p = 0; p < pieces.size(); ++p)
        for (std::size_t q = p + 1; q < pieces.size(); ++q)
            for (const auto& u : pieces[p])
                for (const auto& v : pieces[q]) {
                    const double rx = u.bx - u.ax, ry = u.by - u.ay, sx = v.bx - v.ax, sy = v.by - v.ay;
                    const double den = rx * sy - ry * sx;
                    if (den == 0) continue;
                    const double cx = v.ax - u.ax, cy = v.ay - u.ay;
                    const double t = (cx * sy - cy * sx) / den, w = (cx * ry - cy * rx) / den;
                    if (t < 0 || t > 1 || w < 0 || w > 1) continue;
                    angles.push_back(det::atan2(u.ay + t * ry, u.ax + t * rx));
                }
    std::sort(angles.begin(), angles.end());
    std::vector<double> unique_angles;
    for (const double a : angles) if (unique_angles.empty() || a - unique_angles.back() > 1e-9) unique_angles.push_back(a);
    if (unique_angles.size() >= 2 && unique_angles.front() + 2 * det::kPi - unique_angles.back() <= 1e-9) unique_angles.pop_back();
    if (unique_angles.size() < 3) return {};
    // The farthest hit of the ray at `angle` on any segment: its segment, or none.
    const auto farthest = [&](double angle) -> const Seg* {
        const double dx = det::cos(angle), dy = det::sin(angle);
        const Seg* best = nullptr; double best_t = 0;
        for (const auto& piece : pieces)
            for (const auto& sg : piece) {
                const double ex = sg.bx - sg.ax, ey = sg.by - sg.ay;
                const double den = dx * ey - dy * ex;
                if (den == 0) continue;
                const double w = (sg.ax * dy - sg.ay * dx) / den;          // position on the segment
                if (w < 0 || w > 1) continue;
                const double t = (sg.ax * ey - sg.ay * ex) / den;          // distance along the ray
                if (t > best_t) { best_t = t; best = &sg; }
            }
        return best;
    };
    const auto on_line = [&](const Seg& sg, double angle, std::vector<Tagged>& out) {
        const double dx = det::cos(angle), dy = det::sin(angle), ex = sg.bx - sg.ax, ey = sg.by - sg.ay;
        const double den = dx * ey - dy * ex;
        if (den == 0) return;
        const double t = (sg.ax * ey - sg.ay * ex) / den;
        if (!(t > 0)) return;
        out.push_back({{static_cast<std::int64_t>(std::llround(ox + t * dx)), static_cast<std::int64_t>(std::llround(oy + t * dy))}, 0});
    };
    std::vector<Tagged> out;
    const std::size_t n = unique_angles.size();
    for (std::size_t k = 0; k < n; ++k) {
        const double a0 = unique_angles[k], a1 = k + 1 < n ? unique_angles[k + 1] : unique_angles[0] + 2 * det::kPi;
        const Seg* sg = farthest((a0 + a1) / 2);
        if (!sg) { out.push_back({centre, 0}); continue; }
        on_line(*sg, a0, out); on_line(*sg, a1, out);
    }
    clean_ring(out);
    // A strip that curves around the node (a roundabout arc) is not star-shaped about it: the envelope then fills its
    // shadow. The union's area is at most the sum of the pieces' areas; more than that (plus rounding) means a filled
    // shadow, and the caller keeps the hull.
    if (out.size() < 3 || static_cast<double>(twice_area(out)) > piece_area * 1.02 + 1e6) return {};
    return out;
}

bool is_roundabout(const RoadGraphWay& way) {
    for (const auto& [k, v] : way.tags) if (k == "junction") return v == "roundabout" || v == "circular";
    return false;
}

const RoadGraphWay* find_way(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.ways.begin(), g.ways.end(), id, [](const RoadGraphWay& w, std::int64_t v) { return w.osm_id < v; });
    return it != g.ways.end() && it->osm_id == id ? &*it : nullptr;
}
const RoadGraphNode* find_node(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.nodes.begin(), g.nodes.end(), id, [](const RoadGraphNode& n, std::int64_t v) { return n.osm_id < v; });
    return it != g.nodes.end() && it->osm_id == id ? &*it : nullptr;
}

// ---- corner fillets (kerb returns) ----

std::string_view way_tag(const RoadGraphWay& way, std::string_view key) {
    for (const auto& [k, v] : way.tags) if (k == key) return v;
    return {};
}
bool way_is_oneway(const RoadGraphWay& way) {
    const auto o = way_tag(way, "oneway");
    if (o == "yes" || o == "1" || o == "true" || o == "-1") return true;
    if (o == "no") return false;
    const auto j = way_tag(way, "junction");
    return j == "roundabout" || j == "circular" || way_tag(way, "highway") == "motorway";
}

// A point on an edge polyline at a fractional station, in mm (double).
struct EdgePoint { double x = 0, y = 0, sigma = 0; };

// The segment of a dart's edge polyline that holds station `sigma` (>= 0): index i with sigma[i] <= sigma < sigma[i+1].
std::size_t edge_segment(const Dart& g, double sigma) {
    const std::size_t n = g.sigma.size();
    std::size_t i = static_cast<std::size_t>(std::upper_bound(g.sigma.begin(), g.sigma.end(), sigma) - g.sigma.begin());
    i = i == 0 ? 0 : i - 1;
    return std::min(i, n - 2);
}
// Walks outward (increasing sigma) along edge (layer, side) of `g` from `start` (at station sigma0) by arc length
// `length` mm, never past station `sigma_limit`. Returns the point reached and the arc length actually walked.
EdgePoint walk_edge(const Dart& g, int layer, int side, EdgePoint start, double length, double sigma_limit, double& walked) {
    const auto& pts = g.edge[layer][side];
    std::size_t i = edge_segment(g, start.sigma);
    EdgePoint at = start;
    walked = 0;
    while (i + 1 < pts.size()) {
        double nx = static_cast<double>(pts[i + 1].x), ny = static_cast<double>(pts[i + 1].y), ns = g.sigma[i + 1];
        bool last = false;
        if (ns > sigma_limit) {   // stop on the limit station inside this segment
            const double span = g.sigma[i + 1] - g.sigma[i];
            const double f = span > 0 ? (sigma_limit - g.sigma[i]) / span : 0.0;
            nx = static_cast<double>(pts[i].x) + f * static_cast<double>(pts[i + 1].x - pts[i].x);
            ny = static_cast<double>(pts[i].y) + f * static_cast<double>(pts[i + 1].y - pts[i].y);
            ns = sigma_limit; last = true;
        }
        const double dx = nx - at.x, dy = ny - at.y, d = det::sqrt(dx * dx + dy * dy);
        if (ns > at.sigma && d >= length - walked) {
            const double f = d > 0 ? (length - walked) / d : 0.0;
            walked = length;
            return {at.x + f * dx, at.y + f * dy, at.sigma + f * (ns - at.sigma)};
        }
        if (ns > at.sigma) { walked += d; at = {nx, ny, ns}; }
        if (last) break;
        ++i;
    }
    return at;
}
// Unit direction of edge (layer, side) of `g` going outward at station sigma.
bool edge_direction(const Dart& g, int layer, int side, double sigma, double& ux, double& uy) {
    const auto& pts = g.edge[layer][side];
    const std::size_t i = edge_segment(g, sigma);
    const double dx = static_cast<double>(pts[i + 1].x - pts[i].x), dy = static_cast<double>(pts[i + 1].y - pts[i].y);
    const double d = det::sqrt(dx * dx + dy * dy);
    if (!(d > 0)) return false;
    ux = dx / d; uy = dy / d;
    return true;
}
// |t(layer) - t(layer - 1)| of one side of `g` at station sigma: how far the layer's edge lies outside the inner one.
double layer_offset_m(const Dart& g, int layer, int side, double sigma) {
    const double s = std::clamp(g.at_end ? g.length - sigma : sigma, 0.0, g.length);
    const CarveFootprint f = carve_footprint(*g.xs, s);
    return std::abs(edge_t(f, layer, side, g.at_end) - edge_t(f, layer - 1, side, g.at_end));
}

// Rounds the block corner of pair k (A's left edge against B's right edge, both stubs) on every layer with a circular
// kerb return. Road bed radius R0 = the lower class radius of the two ways (rules); each further layer is concentric:
// R_L = max(R_{L-1} - min(offset on A, offset on B), 0), which keeps road bed in paved in outer for straight edges
// (the opening of a smaller wedge by a radius in [R_{L-1} - d, R_{L-1}] lies inside the opening of the larger one).
// One scale factor shrinks every layer's radius together when the tangent would leave the available edge (the trim
// clamp) or exceed the tangent cap of an acute wedge. Each arc is the rational quadratic Bezier through the tangent
// points with the corner as control point (an exact circle for straight edges; for curved edges it stays inside the
// triangle of the tangent points and the corner, so it never crosses either edge). Returns true if a fillet was set.
bool fillet_corner(const RoadGeomTile& tile, const RoadRules& rules, const std::vector<Dart>& darts, std::size_t A, std::size_t B,
    const std::vector<double>& cap, double margin, double radius_factor, std::array<Corner*, kLayers> corners) {
    const Dart& a = darts[A];
    const Dart& b = darts[B];
    if (a.internal || b.internal) return false;
    const Corner& c0 = *corners[0];
    if (!c0.found || c0.sigma_a < 0 || c0.sigma_b < 0) return false;
    const RoadGraphWay* wa = find_way(tile.source.graph, tile.entries[a.entry].way_id);
    const RoadGraphWay* wb = find_way(tile.source.graph, tile.entries[b.entry].way_id);
    if (!wa || !wb) return false;
    if (is_roundabout(*wa) && is_roundabout(*wb)) return false;   // the island side of a roundabout node
    double r0 = static_cast<double>(std::min(rules.junction.corner_radius_for(way_tag(*wa, "highway")),
                                             rules.junction.corner_radius_for(way_tag(*wb, "highway"))));   // mm
    r0 *= radius_factor;
    if (!(r0 > 0)) return false;
    // Wedge angle on the road bed: the angle between the two edges leaving the corner.
    double uax = 0, uay = 0, ubx = 0, uby = 0;
    if (!edge_direction(a, 0, kLeft, c0.sigma_a, uax, uay) || !edge_direction(b, 0, kRight, c0.sigma_b, ubx, uby)) return false;
    const double dot0 = std::clamp(uax * ubx + uay * uby, -1.0, 1.0);
    const double wedge_deg = det::acos(dot0) * 180.0 / det::kPi;
    if (wedge_deg < static_cast<double>(rules.junction.corner_min_wedge_deg)) return false;
    if (way_is_oneway(*wa) && way_is_oneway(*wb) && wedge_deg < static_cast<double>(rules.junction.corner_oneway_pair_max_wedge_deg))
        r0 = std::min(r0, static_cast<double>(rules.junction.corner_oneway_pair_radius_mm));
    constexpr double kMinRadiusMm = 250.0;
    if (r0 < kMinRadiusMm) return false;

    struct LayerPlan { bool use = false; double r = 0, tan_half = 0, dot = 0; EdgePoint ca, cb; };
    std::array<LayerPlan, kLayers> plan{};
    double scale = 1.0;
    const double tangent_cap = static_cast<double>(rules.junction.corner_tangent_max_percent) / 100.0;
    double r_prev = r0;
    for (int layer = 0; layer < kLayers; ++layer) {
        const Corner& c = *corners[layer];
        double r = r0;
        if (layer > 0) {
            // A layer whose inner neighbour is sharp stays sharp (radius 0 nests in anything).
            if (r_prev <= 0) break;
            if (!c.found || c.sigma_a < 0 || c.sigma_b < 0) return false;   // cannot follow the inner arc: no fillet at all
            const double d = std::min(layer_offset_m(a, layer, kLeft, c.sigma_a), layer_offset_m(b, layer, kRight, c.sigma_b)) * 1000.0;
            r = std::max(r_prev - d, 0.0);
            // Any radius in [r_prev - d, r_prev] nests; a tiny positive one is raised to the minimum instead of being
            // dropped, because a sharp corner nests only when r_prev - d <= 0.
            if (r > 0 && r < kMinRadiusMm) r = kMinRadiusMm;
        }
        r_prev = r;
        if (r <= 0) break;
        double ax = 0, ay = 0, bx = 0, by = 0;
        if (!edge_direction(a, layer, kLeft, c.sigma_a, ax, ay) || !edge_direction(b, layer, kRight, c.sigma_b, bx, by)) return false;
        const double dot = std::clamp(ax * bx + ay * by, -1.0, 1.0);
        const double crs = std::abs(ax * by - ay * bx);
        if (!(crs > 1e-9) || dot <= -1.0 + 1e-12) return false;
        LayerPlan& p = plan[layer];
        p.use = true; p.r = r; p.dot = dot;
        p.tan_half = crs / (1.0 + dot);                       // tan(wedge / 2)
        p.ca = {static_cast<double>(c.point.x), static_cast<double>(c.point.y), c.sigma_a};
        p.cb = {static_cast<double>(c.point.x), static_cast<double>(c.point.y), c.sigma_b};
        const double t = r / p.tan_half;                      // tangent length, mm
        if (layer == 0 && t > tangent_cap * r) scale = std::min(scale, tangent_cap * r / t);
        // The tangent points must stay before the trim clamp minus the margin on both approaches.
        double walked_a = 0, walked_b = 0;
        walk_edge(a, layer, kLeft, p.ca, t, cap[A] - margin, walked_a);
        walk_edge(b, layer, kRight, p.cb, t, cap[B] - margin, walked_b);
        scale = std::min({scale, walked_a / t, walked_b / t});
    }
    if (!plan[0].use || scale * r0 < kMinRadiusMm) return false;
    bool any = false;
    for (int layer = 0; layer < kLayers; ++layer) {
        const LayerPlan& p = plan[layer];
        if (!p.use) break;
        const double t = p.r * scale / p.tan_half;
        double walked = 0;
        const EdgePoint ta = walk_edge(a, layer, kLeft, p.ca, t, cap[A] - margin, walked);
        const EdgePoint tb = walk_edge(b, layer, kRight, p.cb, t, cap[B] - margin, walked);
        Corner& c = *corners[layer];
        // Rational quadratic Bezier, weight sin(wedge / 2) = sqrt((1 - cos wedge) / 2): a circular arc of central
        // angle pi - wedge for straight edges. One segment per <= 7.5 degrees of turn.
        const double w = det::sqrt((1.0 - p.dot) / 2.0);
        const double turn = det::kPi - det::acos(p.dot);
        const int n = std::max(2, static_cast<int>(std::ceil(turn / (7.5 * det::kPi / 180.0))));
        const double px = static_cast<double>(c.point.x), py = static_cast<double>(c.point.y);
        c.arc.clear();
        for (int i = 0; i <= n; ++i) {
            const double u = static_cast<double>(i) / static_cast<double>(n);
            const double b0 = (1 - u) * (1 - u), b1 = 2 * w * u * (1 - u), b2 = u * u, den = b0 + b1 + b2;
            const double x = (b0 * ta.x + b1 * px + b2 * tb.x) / den, y = (b0 * ta.y + b1 * py + b2 * tb.y) / den;
            const PointMm q{std::llround(x), std::llround(y)};
            if (c.arc.empty() || c.arc.back() != q) c.arc.push_back(q);
        }
        c.fillet = true; c.fillet_a = ta.sigma; c.fillet_b = tb.sigma;
        any = true;
    }
    return any;
}

struct RawApproach { std::size_t entry; bool at_end; };
struct NodeInput { std::int64_t id = 0; PointMm position; std::vector<RawApproach> raw; };

void add_stats(JunctionBuildStats& to, const JunctionBuildStats& from) {
    to.built += from.built;
    for (std::size_t i = 0; i < kJunctionDeclineCount; ++i) to.declines[i] += from.declines[i];
    to.partial += from.partial; to.on_structure += from.on_structure; to.hull_fallbacks += from.hull_fallbacks;
    to.bowtie_repaired += from.bowtie_repaired; to.bowtie_rounds += from.bowtie_rounds;
    to.excluded_approaches += from.excluded_approaches; to.approaches_used += from.approaches_used;
    to.max_ring_vertices = std::max(to.max_ring_vertices, from.max_ring_vertices);
    to.clusters += from.clusters; to.cluster_nodes += from.cluster_nodes; to.absorbed += from.absorbed;
    to.face_fallbacks += from.face_fallbacks;
    to.raised_tables += from.raised_tables; to.calming_ignored += from.calming_ignored;
    to.nesting_hulls += from.nesting_hulls; to.nesting_declined += from.nesting_declined; to.ring_cap_hulls += from.ring_cap_hulls;
    to.fillets += from.fillets; to.filleted_junctions += from.filleted_junctions; to.fillet_fallbacks += from.fillet_fallbacks;
    to.star_fallbacks += from.star_fallbacks;
}
}

// ---------------------------------------------------------------------------------------------------------------

namespace {
JunctionApproachStatus approach_status(const RoadGeomTile& tile, const RawApproach& r) {
    const RoadGeomEntry& e = tile.entries[r.entry];
    if (!e.profile) {
        if (const RoadGraphWay* way = find_way(tile.source.graph, e.way_id)) {
            const auto attrs = resolve_road_attributes(*way);
            if (attrs && attrs->tunnel) return JunctionApproachStatus::Tunnel;
        }
        return JunctionApproachStatus::NoProfile;
    }
    if (!e.cross_section) return JunctionApproachStatus::NoCrossSection;
    if (e.profile->attributes.tunnel) return JunctionApproachStatus::Tunnel;
    return JunctionApproachStatus::Used;
}

// Design 4.4: a table or hump tag on a junction node raises the whole junction by its height (rules, the P1 owner
// decision: every vertical kind is carved); a cluster takes the greatest (ties: the smallest node id, the nodes
// come ascending). Every other vertical kind raises nothing and is counted.
void apply_junction_calming(const RoadGeomTile& tile, const std::vector<NodeInput>& nodes, const RoadRules& rules,
    RoadJunction& j, JunctionBuildStats& stats) {
    struct Name { std::string_view tag; ProfileFeatureKind kind; bool raises; };
    static constexpr Name kNames[] = {{"table", ProfileFeatureKind::Table, true}, {"hump", ProfileFeatureKind::Hump, true},
        {"bump", ProfileFeatureKind::Bump, false}, {"mini_bumps", ProfileFeatureKind::MiniBumps, false},
        {"rumble_strip", ProfileFeatureKind::RumbleStrip, false}, {"cushion", ProfileFeatureKind::Cushion, false},
        {"dip", ProfileFeatureKind::Dip, false}};
    std::int32_t best_mm = 0;
    for (const auto& n : nodes) {
        const RoadGraphNode* node = find_node(tile.source.graph, n.id);
        if (!node) continue;
        for (const auto& [key, value] : node->tags) {
            if (key != "traffic_calming") continue;
            for (const auto& name : kNames) {
                if (name.tag != value) continue;
                if (!name.raises) { ++stats.calming_ignored; break; }
                const std::int32_t height = rules.calming(name.kind).height_mm;
                if (height > best_mm) { best_mm = height; j.raised_node = n.id; }
                break;
            }
        }
    }
    if (best_mm <= 0) return;
    j.flags |= static_cast<std::uint16_t>(JunctionFlags::RaisedTable);
    j.raised_height_m = static_cast<double>(best_mm) / 1000.0;
    ++stats.raised_tables;
}

// The junction of `nodes` (one node, or a cluster) with the `absorbed` entries inside it (design 3.2-3.7). Every
// used approach whose entry is absorbed becomes an internal dart; the others are stubs. The ring is the boundary
// walk of the darts' rotation system (decision D16), which for one node is the counter-clockwise order of 3.3.
RoadJunction build_from(const RoadGeomTile& tile, const std::vector<NodeInput>& nodes, const std::vector<std::size_t>& absorbed,
    const RoadRules& rules, const JunctionBuildOptions& options, JunctionBuildStats& stats) {
    RoadJunction j;
    for (const auto& n : nodes) j.nodes.push_back(n.id);
    if (nodes.size() > 1) j.flags |= static_cast<std::uint16_t>(JunctionFlags::Cluster);
    // ---- approaches and their status (3.1) ----
    struct Item { RawApproach r; Key key; JunctionApproachStatus status; std::size_t node; };
    std::vector<Item> items;
    for (std::size_t ni = 0; ni < nodes.size(); ++ni)
        for (const auto& r : nodes[ni].raw) {
            const RoadGeomEntry& e = tile.entries[r.entry];
            items.push_back({r, {e.way_id, e.stretch.start_ref, r.at_end}, approach_status(tile, r), ni});
        }
    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.key < b.key; });
    const auto is_absorbed = [&](std::size_t entry) { return std::binary_search(absorbed.begin(), absorbed.end(), entry); };
    std::vector<Item> stubs, internal, excluded;
    for (const auto& it : items) {
        if (it.status != JunctionApproachStatus::Used) excluded.push_back(it);
        else if (is_absorbed(it.r.entry)) internal.push_back(it);
        else stubs.push_back(it);
    }
    for (const auto e : absorbed) j.absorbed.push_back(e);
    const bool all_tunnels = !items.empty() && std::all_of(items.begin(), items.end(), [](const Item& i) { return i.status == JunctionApproachStatus::Tunnel; });
    if (!excluded.empty()) j.flags |= static_cast<std::uint16_t>(JunctionFlags::Partial);

    const auto record = [&](RoadJunction& built) {
        ++stats.built;
        ++stats.declines[static_cast<std::size_t>(built.decline)];
        stats.excluded_approaches += excluded.size();
        stats.approaches_used += stubs.size();
        if (built.nodes.size() > 1) { ++stats.clusters; stats.cluster_nodes += built.nodes.size(); stats.absorbed += built.absorbed.size(); }
        if (built.flags & static_cast<std::uint16_t>(JunctionFlags::Partial)) ++stats.partial;
        if (built.flags & static_cast<std::uint16_t>(JunctionFlags::OnStructure)) ++stats.on_structure;
        if (built.flags & static_cast<std::uint16_t>(JunctionFlags::HullFallback)) ++stats.hull_fallbacks;
        stats.max_ring_vertices = std::max(stats.max_ring_vertices, built.rings[2].vertices.size());
        return built;
    };
    const auto finish = [&](JunctionDecline decline) {
        j.decline = decline;
        j.approaches.clear();
        for (auto& ring : j.rings) ring.vertices.clear();
        for (const auto& u : stubs) j.approaches.push_back({u.r.entry, u.r.at_end, u.status, 0.0});
        for (const auto& x : excluded) j.approaches.push_back({x.r.entry, x.r.at_end, x.status, 0.0});
        return record(j);
    };
    if (all_tunnels) return finish(JunctionDecline::Underground);
    if (stubs.size() < 2) return finish(JunctionDecline::SingleApproach);

    // ---- darts: stubs first (index = position in `stubs`, ascending key), then the internal darts ----
    std::vector<Dart> darts(stubs.size() + internal.size());
    const auto init = [&](Dart& g, const Item& it, bool is_internal) {
        const RoadGeomEntry& e = tile.entries[it.r.entry];
        g.entry = it.r.entry; g.at_end = it.r.at_end; g.key = it.key; g.node = it.node; g.internal = is_internal;
        g.ref = &e.profile->reference; g.xs = &*e.cross_section; g.bridge = e.profile->attributes.bridge;
        g.length = e.profile->reference.length_m;
        return sample_dart(g, rules);
    };
    for (std::size_t i = 0; i < stubs.size(); ++i) if (!init(darts[i], stubs[i], false)) return finish(JunctionDecline::Degenerate);
    for (std::size_t i = 0; i < internal.size(); ++i) if (!init(darts[stubs.size() + i], internal[i], true)) return finish(JunctionDecline::Degenerate);
    for (std::size_t i = stubs.size(); i < darts.size(); ++i)
        for (std::size_t t = stubs.size(); t < darts.size(); ++t)
            if (t != i && darts[t].entry == darts[i].entry) darts[i].twin = t;
    const std::size_t k_stubs = stubs.size();

    // ---- rotation system: darts at each node in counter-clockwise order (half-plane, exact cross, key) ----
    const auto half = [](PointMm d) { return (d.y > 0 || (d.y == 0 && d.x > 0)) ? 0 : 1; };
    std::vector<std::vector<std::size_t>> rotation(nodes.size());
    std::vector<std::size_t> slot(darts.size());
    for (std::size_t i = 0; i < darts.size(); ++i) rotation[darts[i].node].push_back(i);
    for (auto& rot : rotation) {
        std::sort(rot.begin(), rot.end(), [&](std::size_t a, std::size_t b) {
            const PointMm da = darts[a].direction, db = darts[b].direction;
            const int ha = half(da), hb = half(db);
            if (ha != hb) return ha < hb;
            const std::int64_t c = cross(da, db);
            if (c != 0) return c > 0;
            return darts[a].key < darts[b].key;
        });
        for (std::size_t i = 0; i < rot.size(); ++i) slot[rot[i]] = i;
    }
    // ---- boundary walk from the stub with the smallest key (stubs[0]): next = the dart after the arrival ----
    Frame fr; fr.darts = &darts;
    {
        std::size_t d = 0, guard = 0;
        do {
            fr.walk.push_back(d);
            const std::size_t a = darts[d].internal ? darts[d].twin : d;
            const auto& rot = rotation[darts[a].node];
            d = rot[(slot[a] + 1) % rot.size()];
        } while (d != 0 && ++guard <= darts.size());
    }
    std::size_t stubs_on_walk = 0;
    for (const auto d : fr.walk) if (!darts[d].internal) ++stubs_on_walk;
    const bool face_ok = stubs_on_walk == k_stubs;

    const double trim_min = static_cast<double>(rules.junction.trim_min_mm) / 1000.0;
    const double margin = static_cast<double>(rules.junction.trim_margin_mm) / 1000.0;
    const double step = static_cast<double>(rules.junction.bowtie_step_mm) / 1000.0;
    std::vector<double> base(darts.size(), 0.0), cap(darts.size(), 0.0), extra(darts.size(), 0.0);
    for (std::size_t i = 0; i < k_stubs; ++i) { base[i] = trim_min + margin; cap[i] = std::max(darts[i].length - kTrimClampM, 0.0); }
    const auto set_trims = [&] {
        fr.trims.assign(darts.size(), 0.0);
        for (std::size_t i = 0; i < k_stubs; ++i) fr.trims[i] = std::min(base[i] + extra[i], cap[i]);
    };
    std::array<std::vector<Tagged>, kLayers> rings;
    std::array<bool, kLayers> hull{};
    int rounds_used = 0;
    if (face_ok) {
        // ---- corners per layer, trims of the stubs from the outer layer (3.3, 3.4) ----
        const std::size_t m = fr.walk.size();
        std::array<std::vector<Corner>, kLayers> sharp;
        for (int layer = 0; layer < kLayers; ++layer) {
            sharp[layer].resize(m);
            for (std::size_t k = 0; k < m; ++k)
                sharp[layer][k] = find_corner(darts[fr.arrival(k)], darts[fr.walk[(k + 1) % m]], layer, options.corner_caps);
        }
        // One attempt: with corner fillets (kerb returns) or with the sharp offset corners. Returns the number of
        // fillets applied; the rings, trims and rounds are left in `rings`, `fr`, `rounds_used`.
        // allow[k]: 0 sharp corner, 1 full kerb radius, 2 half radius.
        static constexpr double kFilletFactors[] = {0.0, 1.0, 0.5};
        const auto attempt = [&](const std::vector<char>& allow) -> std::size_t {
            fr.corners = sharp;
            std::fill(extra.begin(), extra.end(), 0.0);
            for (std::size_t i = 0; i < k_stubs; ++i) base[i] = trim_min + margin;
            rounds_used = 0;
            bool beyond = false;
            for (std::size_t k = 0; k < m; ++k) {
                const Corner& c = fr.corners[2][k];
                if (!c.found) continue;
                const std::size_t a = fr.arrival(k), b = fr.walk[(k + 1) % m];
                if (!darts[a].internal) { base[a] = std::max(base[a], std::max(c.sigma_a, trim_min) + margin); if (c.sigma_a > cap[a]) beyond = true; }
                if (!darts[b].internal) { base[b] = std::max(base[b], std::max(c.sigma_b, trim_min) + margin); if (c.sigma_b > cap[b]) beyond = true; }
            }
            if (beyond) j.flags |= static_cast<std::uint16_t>(JunctionFlags::Partial);
            std::size_t filleted = 0;
            {
                for (std::size_t k = 0; k < m; ++k) {
                    if (!allow[k]) continue;
                    const std::size_t a = fr.arrival(k), b = fr.walk[(k + 1) % m];
                    if (!fillet_corner(tile, rules, darts, a, b, cap, margin, kFilletFactors[static_cast<std::size_t>(allow[k])], {&fr.corners[0][k], &fr.corners[1][k], &fr.corners[2][k]})) continue;
                    ++filleted;
                    // The trims reach past every tangent point, so the whole kerb return lies inside the ring.
                    for (int layer = 0; layer < kLayers; ++layer) {
                        const Corner& c = fr.corners[layer][k];
                        if (!c.fillet) continue;
                        base[a] = std::max(base[a], c.fillet_a + margin);
                        base[b] = std::max(base[b], c.fillet_b + margin);
                    }
                }
            }
            set_trims();
            // ---- rings, bow-tie repair (3.5) ----
            for (int round = 0;; ++round) {
                bool all_simple = true;
                std::array<std::vector<std::pair<std::size_t, std::size_t>>, kLayers> per_layer;
                for (int layer = 0; layer < kLayers; ++layer) {
                    if (!build_ring(fr, layer, rings[layer])) return kNone;
                    if (!ring_simple(rings[layer], &per_layer[layer])) all_simple = false;
                }
                if (all_simple) break;
                // Which stubs take part in a crossing: the tags of the four end vertices of every crossing pair.
                std::vector<bool> involved(darts.size(), false);
                bool any = false;
                for (int layer = 0; layer < kLayers; ++layer) {
                    const auto& v = rings[layer];
                    for (const auto& [a, b] : per_layer[layer])
                        for (const std::size_t s : {a, a + 1, b, b + 1}) {
                            const std::size_t tag = v[s % v.size()].tag;
                            if (tag < k_stubs) { involved[tag] = true; any = true; }
                        }
                }
                if (round >= rules.junction.bowtie_rounds || !any) break;
                const std::vector<double> before = fr.trims;
                for (std::size_t i = 0; i < k_stubs; ++i) if (involved[i]) extra[i] += step;
                set_trims();
                if (fr.trims == before) break;           // every involved trim sits on its clamp: nothing left to extend
                ++rounds_used;
            }
            return filleted;
        };
        // No fillets where an approach is excluded (a tunnel or a declined stretch): its mouth lies in one of the
        // wedges, so that wedge is not a block corner, and the ring already closes across the mouth.
        std::vector<char> allow(m, options.corner_fillets && excluded.empty() ? 1 : 0);
        std::size_t filleted = attempt(allow);
        if (filleted == kNone) return finish(JunctionDecline::Degenerate);
        // The fillets must not cost the junction its offset rings: rings that are not simple, do not nest or exceed
        // the vertex budget lose fillets, never the rings. Corners are then re-admitted one at a time in walk order,
        // each at the largest radius (full, then half) with which the rings stay good, else sharp (deterministic, at
        // most 2 m + 1 extra builds).
        const auto good = [&] {
            if (!tagged_rings_nested(rings)) return false;
            for (int layer = 0; layer < kLayers; ++layer)
                if (!ring_simple(rings[layer], nullptr) || rings[layer].size() > kJunctionMaxRingVertices) return false;
            return true;
        };
        if (filleted > 0 && !good()) {
            const std::size_t wanted = filleted;
            std::vector<char> keep(m, 0);
            // Rings that are bad without any fillet go to the hull below anyway; no corner is to blame there.
            const std::size_t sharp_result = attempt(keep);
            if (sharp_result == kNone) return finish(JunctionDecline::Degenerate);
            const bool sharp_good = good();
            for (std::size_t k = 0; k < m && sharp_good; ++k) {
                if (!allow[k]) continue;
                for (char level = 1; level <= 2 && !keep[k]; ++level) {
                    std::vector<char> trial = keep; trial[k] = level;
                    const std::size_t n = attempt(trial);
                    const std::size_t before = std::count_if(keep.begin(), keep.end(), [](char c) { return c != 0; });
                    if (n != kNone && n > before && good()) keep[k] = level;
                }
            }
            filleted = attempt(keep);
            if (filleted == kNone) return finish(JunctionDecline::Degenerate);
            if (sharp_good) stats.fillet_fallbacks += wanted - std::min(wanted, filleted);
        }
        if (filleted > 0) { stats.fillets += filleted; ++stats.filleted_junctions; }
    } else {
        // A stub off the outer boundary walk (inside a cycle of absorbed stretches, D16): no single ring order
        // reaches every stub. Every stub keeps trim_min + margin and every layer is the hull.
        ++stats.face_fallbacks;
        set_trims();
    }
    // ---- hull fallback (3.5 step 4): trim points of the stubs and the edges of the absorbed stretches. When one
    // layer needs it every layer takes it (D17): a hull next to an offset ring is not nested, three hulls of points
    // on the same trim lines are. ----
    bool any_hull = !face_ok;
    for (int layer = 0; layer < kLayers && !any_hull; ++layer) any_hull = !ring_simple(rings[layer], nullptr);
    // Simple offset rings that still do not nest (D21's residual on short, strongly curved approaches) take the
    // all-layer hull as well (D34): encoding 7's validation requires road bed in paved in outer.
    if (!any_hull && options.hull_when_not_nested && !tagged_rings_nested(rings)) { any_hull = true; ++stats.nesting_hulls; }
    // A ring above the encoding's vertex budget takes the hull too (never on the fixtures).
    for (int layer = 0; layer < kLayers && !any_hull; ++layer) if (rings[layer].size() > kJunctionMaxRingVertices) { any_hull = true; ++stats.ring_cap_hulls; }
    // A single node whose offset rings failed takes the star union of its strips (D45) when it is simple and nested
    // on every layer, else the convex hull below.
    if (any_hull && options.star_union_fallback && nodes.size() == 1 && face_ok) {
        std::array<std::vector<Tagged>, kLayers> star;
        bool ok = true;
        for (int layer = 0; layer < kLayers && ok; ++layer) {
            star[layer] = star_union_ring(darts, fr.trims, layer, nodes.front().position);
            ok = star[layer].size() >= 3 && star[layer].size() <= kJunctionMaxRingVertices && ring_simple(star[layer], nullptr);
        }
        if (ok && tagged_rings_nested(star)) {
            rings = std::move(star);
            hull.fill(true);
            any_hull = false;
            ++stats.star_fallbacks;
            j.flags |= static_cast<std::uint16_t>(JunctionFlags::HullFallback);
        }
    }
    for (int layer = 0; layer < kLayers; ++layer) {
        if (!any_hull || (!options.hull_all_layers && face_ok && ring_simple(rings[layer], nullptr))) continue;
        std::vector<PointMm> pts;
        for (std::size_t i = 0; i < darts.size(); ++i) {
            if (!darts[i].internal) {
                PointMm a, b;
                if (!place(darts[i], fr.trims[i], layer, kRight, a) || !place(darts[i], fr.trims[i], layer, kLeft, b)) return finish(JunctionDecline::Degenerate);
                pts.push_back(a); pts.push_back(b);
            } else {
                for (int side = 0; side < 2; ++side) for (const auto& p : darts[i].edge[layer][side]) pts.push_back(p);
            }
        }
        rings[layer] = convex_hull(std::move(pts));
        hull[layer] = true;
        if (rings[layer].size() < 3 || twice_area(rings[layer]) <= 0) return finish(JunctionDecline::Degenerate);
        j.flags |= static_cast<std::uint16_t>(JunctionFlags::HullFallback);
    }
    // Hulls of points on the same trim lines nest up to the mm rounding; a record that still does not is declined
    // rather than stored with rings the validation rejects (D34).
    if (any_hull && options.hull_when_not_nested && options.hull_all_layers && !tagged_rings_nested(rings)) {
        ++stats.nesting_declined; return finish(JunctionDecline::Degenerate);
    }

    // ---- reach (3.5 step 6): every vertex within reach of the nearest cluster node ----
    const std::int64_t reach_mm = static_cast<std::int64_t>(rules.junction.reach_max_m) * 1000;
    for (int layer = 0; layer < kLayers; ++layer)
        for (const auto& v : rings[layer]) {
            bool near = false;
            for (const auto& n : nodes) { const PointMm d = minus(v.p, n.position); if (d.x * d.x + d.y * d.y <= reach_mm * reach_mm) { near = true; break; } }
            if (!near) return finish(JunctionDecline::ReachExceeded);
        }

    // ---- the record: stubs in walk order (starting at the smallest key), then the excluded approaches ----
    j.approaches.clear();
    if (face_ok) { for (const auto d : fr.walk) if (!darts[d].internal) j.approaches.push_back({darts[d].entry, darts[d].at_end, JunctionApproachStatus::Used, fr.trims[d]}); }
    else for (std::size_t i = 0; i < k_stubs; ++i) j.approaches.push_back({darts[i].entry, darts[i].at_end, JunctionApproachStatus::Used, fr.trims[i]});
    for (const auto& x : excluded) j.approaches.push_back({x.r.entry, x.r.at_end, x.status, 0.0});
    for (int layer = 0; layer < kLayers; ++layer) {
        j.rings[layer].vertices.reserve(rings[layer].size());
        for (const auto& v : rings[layer]) j.rings[layer].vertices.push_back(v.p);
    }
    if (std::all_of(darts.begin(), darts.end(), [](const Dart& g) { return g.bridge; })) j.flags |= static_cast<std::uint16_t>(JunctionFlags::OnStructure);
    stats.bowtie_rounds += static_cast<std::size_t>(rounds_used);
    if (rounds_used > 0 && !std::any_of(hull.begin(), hull.end(), [](bool h) { return h; })) ++stats.bowtie_repaired;
    if (options.raised_tables) apply_junction_calming(tile, nodes, rules, j, stats);
    return record(j);
}

// (node, entry, at_end) for every stretch end of the tile, sorted.
struct Hit { std::int64_t node; std::size_t entry; bool at_end; };
std::vector<Hit> stretch_ends(const RoadGeomTile& tile) {
    const auto& graph = tile.source.graph;
    std::vector<Hit> hits;
    for (std::size_t i = 0; i < tile.entries.size(); ++i) {
        const RoadGeomEntry& e = tile.entries[i];
        const RoadGraphWay* way = find_way(graph, e.way_id);
        if (!way || e.stretch.start_ref >= way->node_ids.size() || e.stretch.end_ref >= way->node_ids.size()) continue;
        hits.push_back({way->node_ids[e.stretch.start_ref], i, false});
        hits.push_back({way->node_ids[e.stretch.end_ref], i, true});
    }
    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return std::tie(a.node, a.entry, a.at_end) < std::tie(b.node, b.entry, b.at_end); });
    return hits;
}
std::vector<RawApproach> approaches_at(const std::vector<Hit>& hits, std::int64_t node_id) {
    std::vector<RawApproach> raw;
    const auto lo = std::lower_bound(hits.begin(), hits.end(), node_id, [](const Hit& h, std::int64_t id) { return h.node < id; });
    for (auto it = lo; it != hits.end() && it->node == node_id; ++it) raw.push_back({it->entry, it->at_end});
    return raw;
}
} // namespace

bool junction_ring_valid(const JunctionRing& ring) {
    std::vector<Tagged> v;
    v.reserve(ring.vertices.size());
    for (const auto& p : ring.vertices) v.push_back({p, 0});
    if (v.size() < 3) return false;
    for (std::size_t i = 0; i < v.size(); ++i)
        for (std::size_t j = i + 1; j < v.size(); ++j) if (v[i].p == v[j].p) return false;   // repeated vertex
    return ring_simple(v, nullptr);
}

bool junction_rings_nested(const RoadJunction& junction) {
    return rings_nested({&junction.rings[0].vertices, &junction.rings[1].vertices, &junction.rings[2].vertices});
}

Result<RoadJunction> build_node_junction(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    std::int64_t node_id, const RoadRules& rules, JunctionBuildStats* stats, const JunctionBuildOptions& options) {
    const RoadGraphNode* node = find_node(tile.source.graph, node_id);
    if (!node) return make_error(ErrorCode::InvalidArgument, "road junction: node not in the tile graph");
    const std::uint32_t degree = context.degree(node_id);
    if (degree < 3) return make_error(ErrorCode::InvalidArgument, "road junction: node degree below 3");
    const auto raw = approaches_at(stretch_ends(tile), node_id);
    if (raw.size() != degree) return make_error(ErrorCode::InvalidArgument, "road junction: the tile lacks some approach of the node");
    JunctionBuildStats local;
    auto j = build_from(tile, {{node_id, node->position, raw}}, {}, rules, options, local);
    if (stats) add_stats(*stats, local);
    return j;
}

Result<std::vector<RoadJunction>> build_node_junctions(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    const RoadRules& rules, JunctionBuildStats* stats, const JunctionBuildOptions& options) {
    const auto hits = stretch_ends(tile);
    std::vector<RoadJunction> out;
    for (const auto& node : tile.source.graph.nodes) {   // ascending OSM id
        const std::uint32_t degree = context.degree(node.osm_id);
        if (degree < 3) continue;
        if (stats) ++stats->junction_nodes;
        auto raw = approaches_at(hits, node.osm_id);
        if (raw.size() != degree) { if (stats) ++stats->skipped_incomplete; continue; }
        JunctionBuildStats local;
        out.push_back(build_from(tile, {{node.osm_id, node.position, std::move(raw)}}, {}, rules, options, local));
        if (stats) add_stats(*stats, local);
    }
    return out;
}

Result<std::vector<RoadJunction>> build_road_junctions(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    const RoadRules& rules, JunctionBuildStats* stats, const JunctionBuildOptions& options) {
    const auto& graph = tile.source.graph;
    const auto hits = stretch_ends(tile);
    const double link_cap = static_cast<double>(rules.junction.link_trim_cap_m);
    const std::int64_t extent_mm = static_cast<std::int64_t>(rules.junction.cluster_max_extent_m) * 1000;
    const std::size_t max_nodes = static_cast<std::size_t>(std::max(rules.junction.cluster_max_nodes, 1));

    // ---- 1. every junction node of the tile graph, its approaches and, when complete, its single-node junction ----
    struct Info {
        PointMm position;
        std::vector<RawApproach> raw;
        bool complete = false;
        RoadJunction single;
        JunctionBuildStats single_stats;
        std::vector<std::pair<std::int64_t, std::size_t>> links;   // (neighbour node, linking entry), ascending
        bool unknown = false;                                       // a possible link to an incomplete node
    };
    std::map<std::int64_t, Info> info;   // ascending node id
    std::size_t junction_nodes = 0;
    for (const auto& node : graph.nodes) {
        const std::uint32_t degree = context.degree(node.osm_id);
        if (degree < 3) continue;
        ++junction_nodes;
        Info in; in.position = node.position; in.raw = approaches_at(hits, node.osm_id);
        in.complete = in.raw.size() == degree;
        if (in.complete) in.single = build_from(tile, {{node.osm_id, node.position, in.raw}}, {}, rules, options, in.single_stats);
        info.emplace(node.osm_id, std::move(in));
    }
    const auto single_trim = [](const RoadJunction& j, std::size_t entry, bool at_end) {
        for (const auto& a : j.approaches) if (a.entry_index == entry && a.at_end == at_end) return a.trim_m;
        return 0.0;
    };
    // The node at the other end of an entry, and whether the entry may link at all (design 3.2 rule 2).
    const auto other_end = [&](std::size_t entry, bool at_end) -> std::int64_t {
        const RoadGeomEntry& e = tile.entries[entry];
        const RoadGraphWay* way = find_way(graph, e.way_id);
        return way->node_ids[at_end ? e.stretch.start_ref : e.stretch.end_ref];
    };
    const auto may_link = [&](std::size_t entry) {
        const RoadGeomEntry& e = tile.entries[entry];
        if (!e.profile || !e.cross_section || e.profile->attributes.tunnel) return false;
        const RoadGraphWay* way = find_way(graph, e.way_id);
        return way && (!options.roundabouts_never_link || !is_roundabout(*way));
    };
    // ---- 2. links: the two single-node trims (each capped) cover the stretch ----
    for (auto& [id, in] : info) {
        if (!in.complete) continue;
        for (const auto& a : in.single.approaches) {
            if (a.status != JunctionApproachStatus::Used || !may_link(a.entry_index)) continue;
            const std::int64_t b = other_end(a.entry_index, a.at_end);
            if (b == id) continue;                                  // a loop links nothing
            const auto bi = info.find(b);
            if (bi == info.end()) continue;                         // degree < 3: a join or a dead end
            const double length = tile.entries[a.entry_index].profile->reference.length_m;
            const double ta = std::min(a.trim_m, link_cap);
            if (!bi->second.complete) {
                if (ta + link_cap >= length) in.unknown = !options.unknown_links_are_absent;
                continue;
            }
            const double tb = std::min(single_trim(bi->second.single, a.entry_index, !a.at_end), link_cap);
            if (ta + tb >= length) in.links.push_back({b, a.entry_index});
        }
        std::sort(in.links.begin(), in.links.end());
    }
    // ---- 3. bounded discovery from every node (3.2 rule 4) ----
    enum class Found { Component, TooLarge, Unresolved };
    const auto discover = [&](std::int64_t start, std::vector<std::int64_t>& visited) {
        visited.assign(1, start);
        const PointMm p0 = info.at(start).position;
        std::int64_t x0 = p0.x, x1 = p0.x, y0 = p0.y, y1 = p0.y;
        for (std::size_t q = 0; q < visited.size(); ++q) {
            const Info& in = info.at(visited[q]);
            if (in.unknown) return Found::Unresolved;
            for (const auto& [b, entry] : in.links) {
                (void)entry;
                if (std::find(visited.begin(), visited.end(), b) != visited.end()) continue;
                visited.push_back(b);
                const PointMm p = info.at(b).position;
                x0 = std::min(x0, p.x); x1 = std::max(x1, p.x); y0 = std::min(y0, p.y); y1 = std::max(y1, p.y);
                if (options.cluster_caps && (visited.size() > max_nodes || x1 - x0 > extent_mm || y1 - y0 > extent_mm)) return Found::TooLarge;
            }
        }
        return Found::Component;
    };
    // The linking entries among a node set, plus loops at its nodes whose two trims overlap (3.2 rule 3).
    const auto absorbed_of = [&](const std::vector<std::int64_t>& set) {
        std::vector<std::size_t> out;
        for (const auto id : set) {
            const Info& in = info.at(id);
            for (const auto& [b, entry] : in.links) if (std::binary_search(set.begin(), set.end(), b)) out.push_back(entry);
            if (set.size() < 2) continue;
            for (const auto& a : in.single.approaches) {
                if (a.status != JunctionApproachStatus::Used || a.at_end || !may_link(a.entry_index) || other_end(a.entry_index, false) != id) continue;
                const double length = tile.entries[a.entry_index].profile->reference.length_m;
                const double t0 = std::min(a.trim_m, link_cap), t1 = std::min(single_trim(in.single, a.entry_index, true), link_cap);
                if (t0 + t1 >= length) out.push_back(a.entry_index);
            }
        }
        std::sort(out.begin(), out.end());
        out.erase(std::unique(out.begin(), out.end()), out.end());
        return out;
    };
    const auto inputs_of = [&](const std::vector<std::int64_t>& set) {
        std::vector<NodeInput> in;
        for (const auto id : set) { const Info& i = info.at(id); in.push_back({id, i.position, i.raw}); }
        return in;
    };

    // ---- 4. records, ascending by owner node ----
    std::vector<RoadJunction> out;
    JunctionBuildStats total;
    total.junction_nodes = junction_nodes;
    std::vector<std::int64_t> visited;
    for (auto& [id, in] : info) {
        if (!in.complete) { ++total.skipped_incomplete; continue; }
        const Found found = discover(id, visited);
        if (found == Found::Unresolved) { ++total.skipped_unresolved; continue; }
        std::sort(visited.begin(), visited.end());
        if (found == Found::Component) {
            if (visited.size() == 1) { out.push_back(in.single); add_stats(total, in.single_stats); continue; }
            if (visited.front() != id) continue;                    // the cluster is emitted by its owner node
            total.max_cluster_nodes = std::max(total.max_cluster_nodes, visited.size());
            JunctionBuildStats local;
            out.push_back(build_from(tile, inputs_of(visited), absorbed_of(visited), rules, options, local));
            add_stats(total, local);
            continue;
        }
        // Too large: the node keeps its single-node junction; the node that is the smallest of its own discovery
        // set also stores the declined record (nodes = that set, every approach untrimmed).
        out.push_back(in.single); add_stats(total, in.single_stats);
        if (visited.front() != id) continue;
        RoadJunction declined;
        declined.nodes = visited;
        declined.flags = static_cast<std::uint16_t>(JunctionFlags::Cluster);
        declined.decline = JunctionDecline::ClusterTooLarge;
        declined.absorbed = absorbed_of(visited);
        struct Item { Key key; JunctionApproach a; };
        std::vector<Item> items;
        for (const auto n : visited)
            for (const auto& r : info.at(n).raw) {
                if (std::binary_search(declined.absorbed.begin(), declined.absorbed.end(), r.entry)) continue;
                const RoadGeomEntry& e = tile.entries[r.entry];
                items.push_back({{e.way_id, e.stretch.start_ref, r.at_end}, {r.entry, r.at_end, approach_status(tile, r), 0.0}});
            }
        std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.key < b.key; });
        for (const auto& it : items) {
            declined.approaches.push_back(it.a);
            if (it.a.status != JunctionApproachStatus::Used) declined.flags |= static_cast<std::uint16_t>(JunctionFlags::Partial);
        }
        ++total.built; ++total.declines[static_cast<std::size_t>(JunctionDecline::ClusterTooLarge)];
        out.push_back(std::move(declined));
    }
    if (stats) {
        stats->junction_nodes += total.junction_nodes;
        stats->skipped_incomplete += total.skipped_incomplete;
        stats->skipped_unresolved += total.skipped_unresolved;
        stats->max_cluster_nodes = std::max(stats->max_cluster_nodes, total.max_cluster_nodes);
        add_stats(*stats, total);
    }
    return out;
}


// ---------------------------------------------------------------------------------------------------------------
// Slice 5: junction heights (design 4.2) and calming at junctions (4.4).

namespace {
// The ramp record of one used approach of a RaisedTable junction (design 4.4, decisions D22 and D24): a Table-kind
// feature of the table's height whose plateau runs [0, trim] from the junction-side end of the entry and whose
// single ramp beyond the trim line is max(10 x height, 1 m) long (a Table's own ratio). The record is carried by the
// table node. length_mm = trim + 2 ramp, rounded to mm; the centre is placed so the plateau ends exactly at trim.
ProfileFeature junction_ramp_record(const RoadGeomEntry& e, const JunctionApproach& a, const RoadJunction& j) {
    ProfileFeature f;
    f.entry_index = a.entry_index; f.node_id = j.raised_node; f.kind = ProfileFeatureKind::Table; f.carve = true;
    f.height_mm = static_cast<std::int32_t>(std::llround(j.raised_height_m * 1000.0)); f.width_mm = 0;
    f.provenance = Provenance::Derived;
    const double ramp0 = std::max(j.raised_height_m * static_cast<double>(kFeatureTableRampRatio), static_cast<double>(kFeatureMinRampMm) / 1000.0);
    f.length_mm = static_cast<std::int32_t>(std::llround((a.trim_m + 2.0 * ramp0) * 1000.0));
    const double plateau = feature_effective_length_m(f) / 2.0 - feature_ramp_m(f);
    const double sigma_c = a.trim_m - plateau;
    f.station_m = a.at_end ? e.profile->reference.length_m - sigma_c : sigma_c;
    return f;
}
} // namespace

namespace {
// One approach read at a point: the full approach surface z_i and the base b_i (lateral offset 0) of D42, from one
// projection onto the reference polyline. `full` false skips z_i (it is then left at b_i).
struct ApproachValue { double z = 0, paved = 0, base = 0; };
ApproachValue approach_value(const JunctionSurface& surface, const RoadGeomTile& tile, std::size_t approach,
    double x, double y, const RoadSurfaceParams& params, bool full) noexcept {
    const JunctionSurfaceApproach& a = surface.approaches[approach];
    // Nearest segment of the reference polyline; the first segment is a ray backwards past station 0 (the tangent
    // extension), every other parameter is clamped to its segment. Ties keep the lowest segment.
    const std::size_t segments = a.sigma.size() - 1;
    double best = std::numeric_limits<double>::infinity(), best_u = 0.0, best_cross = 0.0, best_len = 1.0;
    std::size_t best_k = 0;
    for (std::size_t k = 0; k < segments; ++k) {
        const double ax = a.ref_x[k], ay = a.ref_y[k], dx = a.ref_x[k + 1] - ax, dy = a.ref_y[k + 1] - ay;
        const double len2 = dx * dx + dy * dy;
        const double vx = x - ax, vy = y - ay;
        double u = len2 > 0.0 ? (vx * dx + vy * dy) / len2 : 0.0;
        if (u < 0.0 && k > 0) u = 0.0;
        if (u > 1.0) u = 1.0;
        const double qx = ax + u * dx - x, qy = ay + u * dy - y;
        const double d2 = qx * qx + qy * qy;
        if (d2 < best) { best = d2; best_k = k; best_u = u; best_cross = dx * vy - dy * vx; best_len = len2 > 0.0 ? det::sqrt(len2) : 1.0; }
    }
    const double sigma = a.sigma[best_k] + best_u * (a.sigma[best_k + 1] - a.sigma[best_k]);
    const double t_out = best_cross / best_len;                 // positive left of the outward direction
    const double t = a.at_end ? -t_out : t_out;                 // the reference direction runs the other way at an end
    const double s_free = a.at_end ? a.length_m - sigma : sigma;
    const double s = std::clamp(s_free, 0.0, a.length_m);
    double extension = 0.0;
    if (s_free != s) {
        const auto pose = tile.entries[a.entry_index].profile->at(s);
        if (pose) extension = pose->grade * (s_free - s);
    }
    ApproachValue out;
    // The raise of a table junction acts over the carriageway only (the rule of road_surface_height_m for features).
    const bool ramp = a.has_ramp;
    const double ramp_z = ramp ? feature_height_contribution_m(a.ramp, s) : 0.0;
    const CarveFootprint f = carve_footprint(*tile.entries[a.entry_index].cross_section, s);
    const auto height = [&](double tt) {
        double z = road_surface_height_m(tile, a.entry_index, s, tt, params) + extension;
        if (ramp && tt >= f.carriageway_right && tt <= f.carriageway_left) z += ramp_z;
        return z;
    };
    out.base = out.paved = out.z = height(0.0);
    if (full) {
        const double tp = std::clamp(t, f.paved_right, f.paved_left);
        out.paved = tp == 0.0 ? out.base : height(tp);
        out.z = tp == t ? out.paved : height(t);
    }
    return out;
}

// The relief of approach `a` at its trim station: max |z(t) - z(0)| over the paved band (D42), at the band edges,
// the carriageway edges and the crown pivot (the crown is linear or a rounded roof, so its extremes lie there).
double trim_relief_m(const RoadGeomTile& tile, const JunctionSurfaceApproach& a, const RoadSurfaceParams& params) {
    const double s = std::clamp(a.at_end ? a.length_m - a.trim_m : a.trim_m, 0.0, a.length_m);
    const CarveFootprint f = carve_footprint(*tile.entries[a.entry_index].cross_section, s);
    const double raise = a.has_ramp ? feature_height_contribution_m(a.ramp, s) : 0.0;
    const auto z_at = [&](double t) {
        double z = road_surface_height_m(tile, a.entry_index, s, t, params);
        if (a.has_ramp && t >= f.carriageway_right && t <= f.carriageway_left) z += raise;
        return z;
    };
    const double z0 = z_at(0.0);
    double relief = 0.0;
    for (const double t : {f.paved_right, f.carriageway_right, f.crown_pivot, f.carriageway_left, f.paved_left}) {
        if (t < f.paved_right || t > f.paved_left) continue;
        const double d = z_at(t) - z0;
        relief = std::max(relief, d < 0.0 ? -d : d);
    }
    return relief;
}
} // namespace

std::optional<JunctionSurface> make_junction_surface(const RoadGeomTile& tile, const RoadJunction& junction, const RoadRules& rules,
    const RoadSurfaceParams& params, const JunctionFadeParams& fade) {
    if (junction.decline != JunctionDecline::None) return std::nullopt;
    const double spacing = static_cast<double>(rules.junction.edge_station_spacing_mm) / 1000.0;
    JunctionSurface surface;
    for (const auto& a : junction.approaches) {
        if (a.status != JunctionApproachStatus::Used) continue;
        if (a.entry_index >= tile.entries.size()) return std::nullopt;
        const RoadGeomEntry& e = tile.entries[a.entry_index];
        if (!e.profile || !e.cross_section) return std::nullopt;
        Dart g;
        g.entry = a.entry_index; g.at_end = a.at_end; g.ref = &e.profile->reference; g.xs = &*e.cross_section;
        g.length = e.profile->reference.length_m;
        JunctionSurfaceApproach out;
        out.entry_index = a.entry_index; out.at_end = a.at_end; out.length_m = g.length; out.trim_m = a.trim_m;
        PointMm right, left;
        if (!place(g, a.trim_m, 2, kRight, right) || !place(g, a.trim_m, 2, kLeft, left)) return std::nullopt;
        out.trim_ax = static_cast<double>(right.x) / 1000.0; out.trim_ay = static_cast<double>(right.y) / 1000.0;
        out.trim_bx = static_cast<double>(left.x) / 1000.0; out.trim_by = static_cast<double>(left.y) / 1000.0;
        const double sigma_max = std::min(a.trim_m + 1.0, g.length);
        for (std::int64_t k = 0;; ++k) {
            const double sigma = static_cast<double>(k) * spacing;
            if (sigma >= sigma_max) break;
            out.sigma.push_back(sigma);
        }
        out.sigma.push_back(sigma_max);
        if (out.sigma.size() >= 2 && out.sigma[out.sigma.size() - 2] == sigma_max) out.sigma.pop_back();
        if (out.sigma.size() < 2) return std::nullopt;
        for (const double sigma : out.sigma) {
            const double s = std::clamp(a.at_end ? g.length - sigma : sigma, 0.0, g.length);
            const auto p = lateral_point(*g.ref, s, 0.0);
            if (!p) return std::nullopt;
            out.ref_x.push_back(p->x); out.ref_y.push_back(p->y);
        }
        if (junction.flags & static_cast<std::uint16_t>(JunctionFlags::RaisedTable)) { out.has_ramp = true; out.ramp = junction_ramp_record(e, a, junction); }
        out.relief_m = trim_relief_m(tile, out, params);
        out.fade_m = std::clamp(fade.slope > 0.0 ? 2.0 * out.relief_m / fade.slope : fade.max_m, fade.min_m, fade.max_m);
        out.verge_fade_m = std::min(fade.verge_m, out.fade_m);   // psi <= phi: v_i stays a convex combination
        surface.approaches.push_back(std::move(out));
    }
    if (surface.approaches.size() < 2) return std::nullopt;
    return surface;
}

namespace {
double segment_distance2(double px, double py, double ax, double ay, double bx, double by) noexcept {
    const double dx = bx - ax, dy = by - ay, len2 = dx * dx + dy * dy;
    double u = len2 > 0.0 ? ((px - ax) * dx + (py - ay) * dy) / len2 : 0.0;
    if (u < 0.0) u = 0.0;
    if (u > 1.0) u = 1.0;
    const double qx = ax + u * dx - px, qy = ay + u * dy - py;
    return qx * qx + qy * qy;
}
} // namespace

double junction_approach_height_m(const JunctionSurface& surface, const RoadGeomTile& tile, std::size_t approach,
    double x, double y, const RoadSurfaceParams& params) noexcept {
    return approach_value(surface, tile, approach, x, y, params, true).z;
}

JunctionApproachHeights junction_approach_heights(const JunctionSurface& surface, const RoadGeomTile& tile, std::size_t approach,
    double x, double y, const RoadSurfaceParams& params) noexcept {
    const ApproachValue v = approach_value(surface, tile, approach, x, y, params, true);
    return {v.base, v.paved, v.z};
}

namespace {
double fade_weight(double d2, double& d, bool& have_d, double length) noexcept {
    if (!(length < std::numeric_limits<double>::infinity())) return 1.0;
    if (d2 >= length * length) return 0.0;
    if (!have_d) { d = det::sqrt(d2); have_d = true; }
    return road_smooth_step(1.0 - d / length);
}
} // namespace

void junction_relief_weights(const JunctionSurface& surface, std::size_t approach, double x, double y, double& phi, double& psi) noexcept {
    const JunctionSurfaceApproach& a = surface.approaches[approach];
    const double d2 = segment_distance2(x, y, a.trim_ax, a.trim_ay, a.trim_bx, a.trim_by);
    double d = 0.0; bool have_d = false;
    phi = fade_weight(d2, d, have_d, a.fade_m);
    psi = fade_weight(d2, d, have_d, a.verge_fade_m);
}

void junction_weights(const JunctionSurface& surface, double x, double y, std::span<double> weights) noexcept {
    const std::size_t n = surface.approaches.size();
    double sum = 0.0;
    for (std::size_t m = 0; m < n; ++m) {
        double product = 1.0;
        for (std::size_t j = 0; j < n; ++j) {
            if (j == m) continue;
            const JunctionSurfaceApproach& a = surface.approaches[j];
            product *= segment_distance2(x, y, a.trim_ax, a.trim_ay, a.trim_bx, a.trim_by);
        }
        weights[m] = product;
        sum += product;
    }
    if (sum > 0.0) {
        for (std::size_t m = 0; m < n; ++m) weights[m] /= sum;
        return;
    }
    // On two trim segments at once (or an underflow): the average over the approaches whose segment is touched,
    // else over all of them.
    std::size_t touched = 0;
    for (std::size_t m = 0; m < n; ++m) {
        const JunctionSurfaceApproach& a = surface.approaches[m];
        weights[m] = segment_distance2(x, y, a.trim_ax, a.trim_ay, a.trim_bx, a.trim_by) == 0.0 ? 1.0 : 0.0;
        touched += weights[m] > 0.0 ? 1 : 0;
    }
    if (touched == 0) { for (std::size_t m = 0; m < n; ++m) weights[m] = 1.0; touched = n; }
    for (std::size_t m = 0; m < n; ++m) weights[m] /= static_cast<double>(touched);
}

double junction_height_m(const JunctionSurface& surface, const RoadGeomTile& tile, double x, double y,
    const RoadSurfaceParams& params) noexcept {
    constexpr std::size_t kInline = 32;
    const std::size_t n = surface.approaches.size();
    double inline_weights[kInline];
    std::vector<double> heap;
    std::span<double> w(inline_weights, std::min(n, kInline));
    if (n > kInline) { heap.assign(n, 0.0); w = std::span<double>(heap); }   // beyond 32 approaches only; never on the fixtures
    junction_weights(surface, x, y, w);
    double z = 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        if (w[i] == 0.0) continue;
        double phi = 0.0, psi = 0.0;
        junction_relief_weights(surface, i, x, y, phi, psi);
        const ApproachValue v = approach_value(surface, tile, i, x, y, params, phi > 0.0 || psi > 0.0);
        z += w[i] * (v.base + phi * (v.paved - v.base) + psi * (v.z - v.paved));
    }
    return z;
}

std::vector<ProfileFeature> JunctionFeatureAdjustment::core_features() const {
    std::vector<ProfileFeature> all = features;
    all.insert(all.end(), ramps.begin(), ramps.end());
    std::sort(all.begin(), all.end(), [](const ProfileFeature& a, const ProfileFeature& b) {
        return std::tie(a.entry_index, a.station_m, a.node_id) < std::tie(b.entry_index, b.station_m, b.node_id);
    });
    return all;
}

JunctionFeatureAdjustment adjust_features_for_junctions(const RoadGeomTile& tile, std::span<const RoadJunction> junctions) {
    JunctionFeatureAdjustment out;
    constexpr double kNoTrim = -1.0;
    std::vector<double> trim_start(tile.entries.size(), kNoTrim), trim_end(tile.entries.size(), kNoTrim);
    for (const auto& j : junctions) {
        if (j.decline != JunctionDecline::None) continue;
        for (const auto& a : j.approaches) {
            if (a.status != JunctionApproachStatus::Used || a.entry_index >= tile.entries.size()) continue;
            (a.at_end ? trim_end : trim_start)[a.entry_index] = a.trim_m;
        }
    }
    for (const auto& f : tile.features) {
        bool inside = false;
        if (f.entry_index < tile.entries.size() && tile.entries[f.entry_index].profile) {
            const double length = tile.entries[f.entry_index].profile->reference.length_m;
            const double half = feature_effective_length_m(f) / 2.0;
            if (trim_start[f.entry_index] >= 0.0 && f.station_m - half < trim_start[f.entry_index]) inside = true;
            if (trim_end[f.entry_index] >= 0.0 && (length - f.station_m) - half < trim_end[f.entry_index]) inside = true;
        }
        if (inside) ++out.suppressed; else out.features.push_back(f);
    }
    for (const auto& j : junctions) {
        if (j.decline != JunctionDecline::None || !(j.flags & static_cast<std::uint16_t>(JunctionFlags::RaisedTable))) continue;
        for (const auto& a : j.approaches) {
            if (a.status != JunctionApproachStatus::Used || a.entry_index >= tile.entries.size() || !tile.entries[a.entry_index].profile) continue;
            out.ramps.push_back(junction_ramp_record(tile.entries[a.entry_index], a, j));
        }
    }
    std::sort(out.ramps.begin(), out.ramps.end(), [](const ProfileFeature& a, const ProfileFeature& b) {
        return std::tie(a.entry_index, a.station_m, a.node_id) < std::tie(b.entry_index, b.station_m, b.node_id);
    });
    return out;
}

} // namespace g2m
