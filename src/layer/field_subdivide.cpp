#include "g2m/layer/field_subdivide.h"
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <tuple>

namespace g2m {
namespace {
using geom::PointMm;
using geom::Ring;
constexpr std::uint8_t kIn = 1, kLine = 2, kArea = 4;

std::int64_t fdiv(std::int64_t a, std::int64_t b) { // b > 0
    const std::int64_t q = a / b;
    return (a % b != 0 && a < 0) ? q - 1 : q;
}
std::int64_t cdiv(std::int64_t a, std::int64_t b) { // b > 0
    const std::int64_t q = a / b;
    return (a % b != 0 && a > 0) ? q + 1 : q;
}
std::int64_t crossing_ceil_x(PointMm a, PointMm b, std::int64_t y) {
    std::int64_t num = (y - a.y) * (b.x - a.x), den = b.y - a.y;
    if (den < 0) { num = -num; den = -den; }
    return a.x + cdiv(num, den);
}
std::int64_t crossing_floor_x(PointMm a, PointMm b, std::int64_t y) {
    std::int64_t num = (y - a.y) * (b.x - a.x), den = b.y - a.y;
    if (den < 0) { num = -num; den = -den; }
    return a.x + fdiv(num, den);
}

struct Grid {
    std::int64_t ox = 0, oy = 0, c = 0, half = 0, nx = 0, ny = 0;
};

// Sets `flag` on every cell whose centre is inside the even-odd union of `rings`.
void fill_rings(const Grid& g, std::span<const Ring* const> rings, std::vector<std::uint8_t>& mask, std::uint8_t flag) {
    std::vector<std::vector<std::int64_t>> xs(static_cast<std::size_t>(g.ny));
    for (const Ring* ring : rings) {
        const std::size_t n = ring->size();
        for (std::size_t k = 0; k < n; ++k) {
            const PointMm a = (*ring)[k], b = (*ring)[(k + 1) % n];
            if (a.y == b.y) continue;
            const std::int64_t lo = std::min(a.y, b.y), hi = std::max(a.y, b.y);
            const std::int64_t j0 = std::max<std::int64_t>(cdiv(lo - g.oy - g.half, g.c), 0);
            const std::int64_t j1 = std::min<std::int64_t>(cdiv(hi - g.oy - g.half, g.c) - 1, g.ny - 1);
            for (std::int64_t j = j0; j <= j1; ++j)
                xs[static_cast<std::size_t>(j)].push_back(crossing_ceil_x(a, b, g.oy + j * g.c + g.half));
        }
    }
    for (std::int64_t j = 0; j < g.ny; ++j) {
        auto& row = xs[static_cast<std::size_t>(j)];
        std::sort(row.begin(), row.end());
        for (std::size_t p = 0; p + 1 < row.size(); p += 2) {
            const std::int64_t i0 = std::max<std::int64_t>(cdiv(row[p] - g.ox - g.half, g.c), 0);
            const std::int64_t i1 = std::min<std::int64_t>(fdiv(row[p + 1] - 1 - g.ox - g.half, g.c), g.nx - 1);
            for (std::int64_t i = i0; i <= i1; ++i) mask[static_cast<std::size_t>(j * g.nx + i)] |= flag;
        }
    }
}

// Is p within `w` of segment (a, b)? Exact.
bool within_capsule(PointMm p, PointMm a, PointMm b, std::int64_t w) {
    const std::int64_t w2 = w * w;
    const auto d2 = [](PointMm u, PointMm v) { const std::int64_t dx = u.x - v.x, dy = u.y - v.y; return dx * dx + dy * dy; };
    const std::int64_t dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
    if (len2 == 0) return d2(p, a) <= w2;
    const std::int64_t t = (p.x - a.x) * dx + (p.y - a.y) * dy;
    if (t <= 0) return d2(p, a) <= w2;
    if (t >= len2) return d2(p, b) <= w2;
    const std::int64_t cross = dx * (p.y - a.y) - dy * (p.x - a.x);
    return geom::sign_diff_products(cross, cross, w2, len2) <= 0;
}

void mark_capsule(const Grid& g, const FieldSplitSegment& s, std::vector<std::uint8_t>& mask, std::uint8_t flag) {
    const std::int64_t w = s.half_width_mm;
    const PointMm a = s.a, b = s.b;
    const std::int64_t ylo = std::min(a.y, b.y), yhi = std::max(a.y, b.y);
    const std::int64_t j0 = std::max<std::int64_t>(cdiv(ylo - w - g.oy - g.half, g.c), 0);
    const std::int64_t j1 = std::min<std::int64_t>(fdiv(yhi + w - g.oy - g.half, g.c), g.ny - 1);
    for (std::int64_t j = j0; j <= j1; ++j) {
        const std::int64_t yc = g.oy + j * g.c + g.half;
        const std::int64_t y1 = std::max(ylo, yc - w), y2 = std::min(yhi, yc + w);
        if (y1 > y2) continue;
        std::int64_t xlo, xhi;
        if (a.y == b.y) { xlo = std::min(a.x, b.x); xhi = std::max(a.x, b.x); }
        else {
            const std::int64_t f1 = crossing_floor_x(a, b, y1), f2 = crossing_floor_x(a, b, y2);
            const std::int64_t c1 = crossing_ceil_x(a, b, y1), c2 = crossing_ceil_x(a, b, y2);
            xlo = std::min(f1, f2); xhi = std::max(c1, c2);
        }
        const std::int64_t i0 = std::max<std::int64_t>(fdiv(xlo - w - g.ox - g.half, g.c) - 1, 0);
        const std::int64_t i1 = std::min<std::int64_t>(cdiv(xhi + w - g.ox - g.half, g.c) + 1, g.nx - 1);
        for (std::int64_t i = i0; i <= i1; ++i) {
            const PointMm p{g.ox + i * g.c + g.half, yc};
            if (within_capsule(p, a, b, w)) mask[static_cast<std::size_t>(j * g.nx + i)] |= flag;
        }
    }
}

struct Edge {
    std::int32_t label, sx, sy, ex, ey;
    std::uint8_t dir; // 0 east, 1 north, 2 west, 3 south
};
bool edge_less(const Edge& a, const Edge& b) { return std::tie(a.label, a.sx, a.sy, a.dir) < std::tie(b.label, b.sx, b.sy, b.dir); }

struct GridLoop {
    std::int32_t label;
    std::vector<std::pair<std::int32_t, std::int32_t>> corners;
    std::int64_t twice_area; // grid units, positive = counter-clockwise
};
std::int64_t loop_twice_area(const std::vector<std::pair<std::int32_t, std::int32_t>>& v) {
    std::int64_t sum = 0;
    for (std::size_t i = 0; i < v.size(); ++i) {
        const auto& a = v[i]; const auto& b = v[(i + 1) % v.size()];
        sum += static_cast<std::int64_t>(a.first) * b.second - static_cast<std::int64_t>(b.first) * a.second;
    }
    return sum;
}

double ring_length(const Ring& ring) {
    double total = 0;
    for (std::size_t i = 0; i < ring.size(); ++i) {
        const auto& a = ring[i]; const auto& b = ring[(i + 1) % ring.size()];
        const double dx = static_cast<double>(b.x - a.x), dy = static_cast<double>(b.y - a.y);
        total += std::sqrt(dx * dx + dy * dy);
    }
    return total;
}

// Simplifies every ring with `tol`, normalises them; false when the result is degenerate or crosses itself.
bool build_polygon(const Ring& outer, const std::vector<Ring>& holes, std::int64_t tol, FieldPolygon& out) {
    out = {};
    out.outer = simplify_field_ring(outer, tol);
    if (out.outer.size() < 3) return false;
    normalise_field_ring(out.outer, true);
    if (ring_twice_area_mm2(out.outer) <= 0) return false;
    for (const auto& h : holes) {
        Ring r = simplify_field_ring(h, tol);
        if (r.size() < 3) continue; // a hole that simplifies away is closed up
        normalise_field_ring(r, false);
        if (ring_twice_area_mm2(r) >= 0) continue;
        out.holes.push_back(std::move(r));
    }
    std::sort(out.holes.begin(), out.holes.end(), [](const Ring& a, const Ring& b) { return a[0].x != b[0].x ? a[0].x < b[0].x : a[0].y < b[0].y; });
    return field_polygon_simple(out);
}
} // namespace

Ring simplify_field_ring(const Ring& ring, std::int64_t tol) {
    const std::size_t n = ring.size();
    if (tol <= 0 || n <= 4) return ring;
    std::size_t far = 0; std::int64_t far_d2 = -1;
    for (std::size_t i = 1; i < n; ++i) {
        const std::int64_t dx = ring[i].x - ring[0].x, dy = ring[i].y - ring[0].y, d2 = dx * dx + dy * dy;
        if (d2 > far_d2) { far_d2 = d2; far = i; }
    }
    std::vector<bool> keep(n, false);
    keep[0] = true; keep[far] = true;
    const std::int64_t tol2 = tol * tol;
    std::vector<std::pair<std::size_t, std::size_t>> stack{{0, far}, {far, n}};
    while (!stack.empty()) {
        const auto [lo, hi] = stack.back(); stack.pop_back();
        if (hi - lo < 2) continue;
        const PointMm a = ring[lo % n], b = ring[hi % n];
        const std::int64_t dx = b.x - a.x, dy = b.y - a.y, len2 = dx * dx + dy * dy;
        std::size_t split = 0; double split_d = -1; bool over = false;
        for (std::size_t k = lo + 1; k < hi; ++k) {
            const PointMm p = ring[k % n];
            bool exceeds; double d;
            if (len2 == 0) {
                const std::int64_t ex = p.x - a.x, ey = p.y - a.y, e2 = ex * ex + ey * ey;
                exceeds = e2 > tol2; d = static_cast<double>(e2);
            } else {
                const std::int64_t cross = dx * (p.y - a.y) - dy * (p.x - a.x);
                exceeds = geom::sign_diff_products(cross, cross, tol2, len2) > 0;
                d = static_cast<double>(cross) * static_cast<double>(cross) / static_cast<double>(len2);
            }
            if (exceeds) over = true;
            if (d > split_d) { split_d = d; split = k; }
        }
        if (!over) continue;
        keep[split % n] = true;
        stack.push_back({lo, split}); stack.push_back({split, hi});
    }
    Ring out;
    for (std::size_t i = 0; i < n; ++i) if (keep[i]) out.push_back(ring[i]);
    return out;
}

bool field_polygon_simple(const FieldPolygon& polygon) {
    struct Seg { PointMm a, b; std::size_t ring, index, count; };
    std::vector<Seg> segs;
    const auto add = [&](const Ring& r, std::size_t id) {
        for (std::size_t i = 0; i < r.size(); ++i) segs.push_back({r[i], r[(i + 1) % r.size()], id, i, r.size()});
    };
    add(polygon.outer, 0);
    for (std::size_t h = 0; h < polygon.holes.size(); ++h) add(polygon.holes[h], h + 1);
    for (std::size_t i = 0; i < segs.size(); ++i) {
        const auto& s = segs[i];
        const std::int64_t sx0 = std::min(s.a.x, s.b.x), sx1 = std::max(s.a.x, s.b.x), sy0 = std::min(s.a.y, s.b.y), sy1 = std::max(s.a.y, s.b.y);
        for (std::size_t j = i + 1; j < segs.size(); ++j) {
            const auto& t = segs[j];
            if (s.ring == t.ring) { // adjacent segments of one ring share a vertex by construction
                const std::size_t d = t.index - s.index;
                if (d == 1 || d + 1 == s.count) continue;
            }
            if (std::max(t.a.x, t.b.x) < sx0 || std::min(t.a.x, t.b.x) > sx1 || std::max(t.a.y, t.b.y) < sy0 || std::min(t.a.y, t.b.y) > sy1) continue;
            bool proper = false;
            geom::segments_intersect(s.a, s.b, t.a, t.b, &proper);
            if (proper) return false;
        }
    }
    return true;
}

Result<SubdivideResult> subdivide_polygon(const FieldRules& rules, const FieldPolygon& polygon,
    std::span<const FieldSplitSegment> lines, std::span<const FieldObstacle* const> obstacles) {
    SubdivideResult res;
    if (lines.empty() && obstacles.empty()) return res;
    const auto bb = ring_bbox(polygon.outer);
    Grid g;
    g.c = rules.cell_mm; g.half = g.c / 2;
    g.ox = fdiv(bb.w, g.c) * g.c - g.c; g.oy = fdiv(bb.s, g.c) * g.c - g.c;
    g.nx = (bb.e - g.ox) / g.c + 2; g.ny = (bb.n - g.oy) / g.c + 2;
    if (g.nx * g.ny > rules.max_cells) { res.declined = FieldDeclineReason::TooManyCells; return res; }
    std::vector<std::uint8_t> mask(static_cast<std::size_t>(g.nx * g.ny), 0);
    {
        std::vector<const Ring*> rings{&polygon.outer};
        for (const auto& h : polygon.holes) rings.push_back(&h);
        fill_rings(g, rings, mask, kIn);
    }
    std::int64_t in_cells = 0;
    for (auto m : mask) in_cells += (m & kIn) ? 1 : 0;
    if (in_cells == 0) return res; // thinner than a cell: nothing to cut
    for (const auto& s : lines) mark_capsule(g, s, mask, kLine);
    for (const FieldObstacle* o : obstacles) {
        std::vector<const Ring*> rings;
        for (const auto& r : o->rings) rings.push_back(&r);
        // Only the polygon's own cells matter, but filling the obstacle with its own flag first keeps the even-odd rule per obstacle.
        fill_rings(g, rings, mask, kArea);
    }
    std::int64_t area_cells = 0;
    for (auto m : mask) area_cells += ((m & kIn) && (m & kArea)) ? 1 : 0;
    if (area_cells * 100 > static_cast<std::int64_t>(rules.obstacle_share_cap_percent) * in_cells) {
        for (auto& m : mask) m &= static_cast<std::uint8_t>(~kArea);
        res.obstacles_ignored = true; area_cells = 0;
    }
    bool any_blocked = false;
    for (auto m : mask) if ((m & kIn) && (m & (kLine | kArea))) { any_blocked = true; break; }
    if (!any_blocked) return res;

    // ---- connected components (4-connectivity), labels in row-major order from the south-west -----------------
    const auto idx_free = [&](std::int64_t i) { return (mask[static_cast<std::size_t>(i)] & (kIn | kLine | kArea)) == kIn; };
    std::vector<std::int32_t> label(mask.size(), 0);
    std::vector<std::int64_t> cells_of(1, 0), stack;
    std::int32_t next = 0;
    for (std::int64_t start = 0; start < g.nx * g.ny; ++start) {
        if (!idx_free(start) || label[static_cast<std::size_t>(start)] != 0) continue;
        ++next; cells_of.push_back(0);
        stack.push_back(start); label[static_cast<std::size_t>(start)] = next;
        while (!stack.empty()) {
            const std::int64_t cur = stack.back(); stack.pop_back();
            ++cells_of[static_cast<std::size_t>(next)];
            const std::int64_t i = cur % g.nx, j = cur / g.nx;
            const std::int64_t nb[4] = {i + 1 < g.nx ? cur + 1 : -1, j + 1 < g.ny ? cur + g.nx : -1, i > 0 ? cur - 1 : -1, j > 0 ? cur - g.nx : -1};
            for (const std::int64_t q : nb)
                if (q >= 0 && idx_free(q) && label[static_cast<std::size_t>(q)] == 0) { label[static_cast<std::size_t>(q)] = next; stack.push_back(q); }
        }
    }
    const std::int64_t cell_area = g.c * g.c;
    const std::int64_t min_area_mm2 = static_cast<std::int64_t>(rules.sliver_min_area_m2) * 1000000;
    std::vector<char> trace(static_cast<std::size_t>(next) + 1, 0);
    for (std::int32_t l = 1; l <= next; ++l) {
        // A component below half the sliver area cannot reach it after simplification: dropped without tracing.
        if (cells_of[static_cast<std::size_t>(l)] * cell_area * 2 >= min_area_mm2) trace[static_cast<std::size_t>(l)] = 1;
        else ++res.slivers_dropped;
    }

    // ---- boundary edges, region on the left ------------------------------------------------------------------
    const auto lab = [&](std::int64_t i, std::int64_t j) -> std::int32_t {
        return (i < 0 || j < 0 || i >= g.nx || j >= g.ny) ? 0 : label[static_cast<std::size_t>(j * g.nx + i)];
    };
    std::vector<Edge> edges;
    for (std::int64_t j = 0; j < g.ny; ++j)
        for (std::int64_t i = 0; i < g.nx; ++i) {
            const std::int32_t l = label[static_cast<std::size_t>(j * g.nx + i)];
            if (l == 0 || !trace[static_cast<std::size_t>(l)]) continue;
            const auto x = static_cast<std::int32_t>(i), y = static_cast<std::int32_t>(j);
            if (lab(i, j - 1) != l) edges.push_back({l, x, y, x + 1, y, 0});
            if (lab(i + 1, j) != l) edges.push_back({l, x + 1, y, x + 1, y + 1, 1});
            if (lab(i, j + 1) != l) edges.push_back({l, x + 1, y + 1, x, y + 1, 2});
            if (lab(i - 1, j) != l) edges.push_back({l, x, y + 1, x, y, 3});
        }
    std::sort(edges.begin(), edges.end(), edge_less);
    std::vector<char> used(edges.size(), 0);
    std::vector<GridLoop> loops;
    for (std::size_t e0 = 0; e0 < edges.size(); ++e0) {
        if (used[e0]) continue;
        GridLoop loop; loop.label = edges[e0].label;
        std::vector<std::uint8_t> dirs;
        std::size_t cur = e0;
        for (;;) {
            used[cur] = 1;
            loop.corners.push_back({edges[cur].sx, edges[cur].sy}); dirs.push_back(edges[cur].dir);
            // Outgoing edges at the end vertex; prefer the left turn so cells that only touch diagonally stay separate.
            const Edge probe{edges[cur].label, edges[cur].ex, edges[cur].ey, 0, 0, 0};
            auto it = std::lower_bound(edges.begin(), edges.end(), probe, edge_less);
            std::size_t best = edges.size(); int best_rank = 99;
            for (; it != edges.end() && it->label == probe.label && it->sx == probe.sx && it->sy == probe.sy; ++it) {
                const int turn = (it->dir - edges[cur].dir + 4) % 4; // 1 left, 0 straight, 3 right, 2 back
                const int rank = turn == 1 ? 0 : turn == 0 ? 1 : turn == 3 ? 2 : 3;
                if (rank < best_rank) { best_rank = rank; best = static_cast<std::size_t>(it - edges.begin()); }
            }
            if (best == edges.size()) return make_error(ErrorCode::Malformed, "fields.subdivide: open boundary");
            if (best == e0) break;
            if (used[best]) return make_error(ErrorCode::Malformed, "fields.subdivide: boundary pairing");
            cur = best;
        }
        // Merge collinear unit edges: keep a corner only where the direction changes.
        std::vector<std::pair<std::int32_t, std::int32_t>> corners;
        for (std::size_t k = 0; k < loop.corners.size(); ++k)
            if (dirs[k] != dirs[(k + loop.corners.size() - 1) % loop.corners.size()]) corners.push_back(loop.corners[k]);
        loop.corners = std::move(corners);
        loop.twice_area = loop_twice_area(loop.corners);
        loops.push_back(std::move(loop));
    }

    // ---- pieces ----------------------------------------------------------------------------------------------
    const auto to_ring = [&](const GridLoop& l) {
        Ring r; r.reserve(l.corners.size());
        for (const auto& [x, y] : l.corners) r.push_back({g.ox + x * g.c, g.oy + y * g.c});
        return r;
    };
    struct Candidate { std::int32_t label; FieldPolygon polygon; };
    std::vector<Candidate> candidates;
    for (std::int32_t l = 1; l <= next; ++l) {
        if (!trace[static_cast<std::size_t>(l)]) continue;
        std::vector<const GridLoop*> outers, inners;
        for (const auto& loop : loops) if (loop.label == l) (loop.twice_area > 0 ? outers : inners).push_back(&loop);
        if (outers.empty()) continue;
        for (const GridLoop* o : outers) {
            Ring outer = to_ring(*o); std::vector<Ring> holes;
            for (const GridLoop* in : inners) {
                const Ring hole = to_ring(*in);
                // With several outers a hole belongs to the smallest one containing its first vertex.
                const GridLoop* owner = nullptr;
                for (const GridLoop* cand : outers)
                    if (field_ring_contains(to_ring(*cand), hole[0]) && (!owner || cand->twice_area < owner->twice_area)) owner = cand;
                if (owner == o) holes.push_back(hole);
            }
            FieldPolygon piece; bool ok = false;
            for (std::int64_t tol = rules.simplify_tolerance_mm; ; tol /= 2) {
                if (build_polygon(outer, holes, tol, piece)) { ok = true; break; }
                if (tol == rules.simplify_tolerance_mm && tol > 0) res.simplify_fallback = true;
                if (tol == 0) break;
            }
            if (!ok) { // the unsimplified trace is simple by construction
                piece = {}; piece.outer = outer; normalise_field_ring(piece.outer, true);
                for (auto h : holes) { normalise_field_ring(h, false); piece.holes.push_back(std::move(h)); }
                std::sort(piece.holes.begin(), piece.holes.end(), [](const Ring& a, const Ring& b) { return a[0].x != b[0].x ? a[0].x < b[0].x : a[0].y < b[0].y; });
            }
            candidates.push_back({l, std::move(piece)});
        }
    }
    // ---- sliver rule and the keep-whole decisions --------------------------------------------------------------
    const std::uint64_t original = field_polygon_area_mm2(polygon);
    std::vector<FieldPolygon> kept;
    for (auto& c : candidates) {
        const std::uint64_t area = field_polygon_area_mm2(c.polygon);
        double perimeter = ring_length(c.polygon.outer);
        for (const auto& h : c.polygon.holes) perimeter += ring_length(h);
        const double width = perimeter > 0 ? 2.0 * static_cast<double>(area) / perimeter : 0.0;
        if (static_cast<std::int64_t>(area) < min_area_mm2 || width < static_cast<double>(rules.sliver_min_width_mm)) { ++res.slivers_dropped; continue; }
        kept.push_back(std::move(c.polygon));
    }
    if (kept.empty()) { res.sliver_block = true; return res; }
    if (kept.size() == 1 && field_polygon_area_mm2(kept[0]) * 100 >= original * static_cast<std::uint64_t>(rules.single_piece_keep_percent)) return res;
    res.verbatim = false;
    res.pieces = std::move(kept);
    return res;
}
} // namespace g2m
