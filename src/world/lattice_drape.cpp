// g2m/world/lattice_drape.cpp - see lattice_drape.h.
#include "lattice_drape.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace g2m::world::detail {

namespace {

constexpr double kMinArea = 1e-10; // m2; a piece below this is a sliver from a cut through a vertex
constexpr double kQuant = 1e4;     // vertex sharing grid: 0.1 mm

using Poly = std::vector<DrapeVertex>;

DrapeVertex lerp(const DrapeVertex& p, const DrapeVertex& q, double t) {
    DrapeVertex r;
    r.x = p.x + (q.x - p.x) * t;
    r.y = p.y + (q.y - p.y) * t;
    for (std::size_t i = 0; i < r.a.size(); ++i) r.a[i] = p.a[i] + (q.a[i] - p.a[i]) * t;
    return r;
}

double area(const Poly& p) {
    double s = 0;
    for (std::size_t i = 0; i < p.size(); ++i) {
        const auto& a = p[i];
        const auto& b = p[(i + 1) % p.size()];
        s += a.x * b.y - b.x * a.y;
    }
    return std::abs(s) / 2;
}

// Splits a convex polygon by the line f == 0 (f linear): `low` gets the part with f <= 0, `high` the part with f >= 0.
// `axis` 0 / 1: f is x - v / y - v and the cut point's coordinate is set exactly to v (no rounding off the lattice line).
template <class F>
void split(const Poly& poly, F f, int axis, double v, Poly& low, Poly& high) {
    low.clear();
    high.clear();
    const std::size_t n = poly.size();
    for (std::size_t i = 0; i < n; ++i) {
        const DrapeVertex& p = poly[i];
        const DrapeVertex& q = poly[(i + 1) % n];
        const double fp = f(p), fq = f(q);
        if (fp <= 0) low.push_back(p);
        if (fp >= 0) high.push_back(p);
        if ((fp < 0 && fq > 0) || (fp > 0 && fq < 0)) {
            DrapeVertex m = lerp(p, q, fp / (fp - fq));
            if (axis == 0) m.x = v;
            else if (axis == 1) m.y = v;
            low.push_back(m);
            high.push_back(m);
        }
    }
}

bool usable(const Poly& p) { return p.size() >= 3 && area(p) > kMinArea; }

} // namespace

LatticeDrape::LatticeDrape(const RoadRibbonGround& ground, double ox, double oy, double tolerance_m)
    : ground_(ground), ox_(ox), oy_(oy), tol_(tolerance_m) {}

std::optional<double> LatticeDrape::node(std::int64_t c, std::int64_t r) {
    const std::int64_t wc = c + kWindowMargin, wr = r + kWindowMargin;
    if (wc >= 0 && wc < kWindowSize && wr >= 0 && wr < kWindowSize) {
        if (window_.empty()) window_.assign(static_cast<std::size_t>(kWindowSize * kWindowSize), std::nan(""));
        double& slot = window_[static_cast<std::size_t>(wr * kWindowSize + wc)];
        if (std::isnan(slot)) {
            ++stats_.node_reads;
            const auto h = ground_(ox_ + static_cast<double>(c) + 0.5, oy_ + static_cast<double>(r) + 0.5);
            slot = h ? *h : std::numeric_limits<double>::infinity();
        }
        if (std::isinf(slot)) return std::nullopt;
        return slot;
    }
    const std::uint64_t key = (static_cast<std::uint64_t>(static_cast<std::uint32_t>(c)) << 32) | static_cast<std::uint32_t>(r);
    const auto it = far_nodes_.find(key);
    if (it != far_nodes_.end()) return it->second;
    ++stats_.node_reads;
    const auto h = ground_(ox_ + static_cast<double>(c) + 0.5, oy_ + static_cast<double>(r) + 0.5);
    far_nodes_.emplace(key, h);
    return h;
}

std::optional<double> LatticeDrape::height(double x, double y) {
    const double u = x - 0.5, v = y - 0.5;
    if (!std::isfinite(u) || !std::isfinite(v)) return std::nullopt;
    const double fu = std::floor(u), fv = std::floor(v);
    const auto c = static_cast<std::int64_t>(fu), r = static_cast<std::int64_t>(fv);
    const double tx = u - fu, ty = v - fv;
    const auto h10 = node(c + 1, r), h01 = node(c, r + 1);
    if (!h10 || !h01) return std::nullopt;
    if (tx + ty <= 1.0) {
        const auto h00 = node(c, r);
        if (!h00) return std::nullopt;
        return *h00 + tx * (*h10 - *h00) + ty * (*h01 - *h00);
    }
    const auto h11 = node(c + 1, r + 1);
    if (!h11) return std::nullopt;
    return *h11 + (1.0 - tx) * (*h01 - *h11) + (1.0 - ty) * (*h10 - *h11);
}

std::int32_t LatticeDrape::vertex_for(RoadRibbonTile& mesh, const DrapeVertex& v, double z, double raise) {
    const SharedKey key{static_cast<std::int64_t>(std::llround(v.x * kQuant)), static_cast<std::int64_t>(std::llround(v.y * kQuant)),
                        static_cast<std::int32_t>(std::lround(v.a[4])), static_cast<std::int32_t>(std::lround(v.a[5])),
                        static_cast<std::int32_t>(std::lround(v.a[6])), static_cast<std::int32_t>(std::lround(v.a[7])),
                        static_cast<std::int32_t>(std::lround(raise * kQuant))};
    const auto it = shared_.find(key);
    if (it != shared_.end()) return it->second;
    const auto index = static_cast<std::int32_t>(mesh.vertices.size());
    mesh.vertices.push_back({static_cast<float>(v.x), static_cast<float>(v.y), static_cast<float>(z), static_cast<float>(v.a[0]),
                             static_cast<float>(v.a[1]), static_cast<float>(v.a[2]), static_cast<float>(v.a[3]), static_cast<float>(v.a[4]),
                             static_cast<float>(v.a[5]), static_cast<float>(v.a[6]), static_cast<float>(v.a[7])});
    shared_.emplace(key, index);
    return index;
}

// Straight enough: the terrain's height at every place the terrain surface bends along p-q (a lattice line or a diagonal
// crossing) is within tol_ of the straight line between the end heights. The test depends on the segment alone (its two
// ends, in a canonical order), so the two triangles that share an edge decide alike and never leave a crack.
bool LatticeDrape::straight_edge(double px, double py, double pz, double qx, double qy, double qz, double tol) {
    if (px > qx || (px == qx && py > qy)) {
        std::swap(px, qx);
        std::swap(py, qy);
        std::swap(pz, qz);
    }
    scratch_t_.clear();
    const auto family = [&](double fp, double fq, double offset) {
        if (fp == fq) return;
        const double lo = std::min(fp, fq), hi = std::max(fp, fq);
        for (auto k = static_cast<std::int64_t>(std::ceil(lo - offset)); static_cast<double>(k) + offset <= hi; ++k) {
            const double t = (static_cast<double>(k) + offset - fp) / (fq - fp);
            if (t > 1e-9 && t < 1 - 1e-9) scratch_t_.push_back(t);
        }
    };
    family(px, qx, 0.5);
    family(py, qy, 0.5);
    family(px + py, qx + qy, 0.0);
    for (const double t : scratch_t_) {
        const auto h = height(px + (qx - px) * t, py + (qy - py) * t);
        if (!h || std::abs(*h - (pz + (qz - pz) * t)) > tol) return false;
    }
    return true;
}

// Cuts the triangle a, b, c on the terrain triangles and appends the pieces. `zfn(p, terrain_z)` gives the height of a piece vertex
// from the terrain plane of its piece (plain drape: the straight-edge override; ramp: the max rule).
template <class ZFn>
void LatticeDrape::cut_emit(RoadRibbonTile& mesh, const DrapeVertex& a, const DrapeVertex& b, const DrapeVertex& c, double raise, ZFn&& zfn) {
    const auto col0 = static_cast<std::int64_t>(std::floor(std::min({a.x, b.x, c.x}) - 0.5)), col1 = static_cast<std::int64_t>(std::floor(std::max({a.x, b.x, c.x}) - 0.5));
    // Cut. Pieces by the vertical lines x = k + 0.5, then by the horizontal ones, then by the diagonal of each cell.
    Poly tri{a, b, c};
    struct Cell {
        Poly poly;
        std::int64_t c, r;
    };
    std::vector<Cell> slabs;
    {
        Poly rest = std::move(tri), low, high;
        for (std::int64_t k = col0; k < col1; ++k) {
            const double v = static_cast<double>(k) + 1.5;
            split(rest, [&](const DrapeVertex& p) { return p.x - v; }, 0, v, low, high);
            if (usable(low)) slabs.push_back({low, k, 0});
            rest = high;
        }
        if (usable(rest)) slabs.push_back({std::move(rest), col1, 0});
    }
    std::vector<Cell> cells;
    for (auto& slab : slabs) {
        double lo = 1e300, hi = -1e300;
        for (const auto& p : slab.poly) {
            lo = std::min(lo, p.y);
            hi = std::max(hi, p.y);
        }
        const auto r0 = static_cast<std::int64_t>(std::floor(lo - 0.5)), r1 = static_cast<std::int64_t>(std::floor(hi - 0.5));
        Poly rest = std::move(slab.poly), low, high;
        for (std::int64_t k = r0; k < r1; ++k) {
            const double v = static_cast<double>(k) + 1.5;
            split(rest, [&](const DrapeVertex& p) { return p.y - v; }, 1, v, low, high);
            if (usable(low)) cells.push_back({low, slab.c, k});
            rest = high;
        }
        if (usable(rest)) cells.push_back({std::move(rest), slab.c, r1});
    }
    // Each piece lies in one terrain triangle: its vertices' heights are that triangle's plane. Terrain height at a vertex
    // on a straight edge is the edge's own line instead - what the neighbour across that edge draws (no crack, however it
    // cuts its side).
    for (auto& cell : cells) {
        const double diag = static_cast<double>(cell.c + cell.r) + 2.0;
        Poly sw, ne;
        split(cell.poly, [&](const DrapeVertex& p) { return p.x + p.y - diag; }, -1, 0, sw, ne);
        for (int side = 0; side < 2; ++side) {
            const Poly& poly = side == 0 ? sw : ne;
            if (!usable(poly)) continue;
            const std::int64_t cc = cell.c, rr = cell.r;
            const auto h10 = node(cc + 1, rr), h01 = node(cc, rr + 1);
            std::optional<double> h00, h11;
            if (side == 0) h00 = node(cc, rr);
            else h11 = node(cc + 1, rr + 1);
            if (!h10 || !h01 || (side == 0 ? !h00 : !h11)) {
                ++stats_.dropped;
                continue;
            }
            std::vector<std::int32_t> ids;
            ids.reserve(poly.size());
            for (const auto& p : poly) {
                const double tx = p.x - (static_cast<double>(cc) + 0.5), ty = p.y - (static_cast<double>(rr) + 0.5);
                double z = side == 0 ? *h00 + tx * (*h10 - *h00) + ty * (*h01 - *h00)
                                     : *h11 + (1.0 - tx) * (*h01 - *h11) + (1.0 - ty) * (*h10 - *h11);
                z = zfn(p, z);
                const auto id = vertex_for(mesh, p, z + raise, raise);
                if (ids.empty() || ids.back() != id) ids.push_back(id);
            }
            while (ids.size() > 1 && ids.front() == ids.back()) ids.pop_back();
            for (std::size_t i = 1; i + 1 < ids.size(); ++i) {
                if (ids[0] == ids[i] || ids[i] == ids[i + 1] || ids[0] == ids[i + 1]) continue;
                mesh.indices.insert(mesh.indices.end(), {ids[0], ids[i], ids[i + 1]});
                ++stats_.pieces;
            }
        }
    }
}

void LatticeDrape::add(RoadRibbonTile& mesh, const DrapeVertex& a, const DrapeVertex& b, const DrapeVertex& c, double raise, double tolerance_m) {
    ++stats_.triangles_in;
    const double tol = tolerance_m >= 0 ? tolerance_m : tol_;
    {
        const double det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
        if (!(std::abs(det) / 2 > kMinArea)) return;
    }
    double minx = a.x, maxx = a.x, miny = a.y, maxy = a.y;
    for (const DrapeVertex* p : {&b, &c}) {
        minx = std::min(minx, p->x);
        maxx = std::max(maxx, p->x);
        miny = std::min(miny, p->y);
        maxy = std::max(maxy, p->y);
    }
    const auto col0 = static_cast<std::int64_t>(std::floor(minx - 0.5)), col1 = static_cast<std::int64_t>(std::floor(maxx - 0.5));
    const auto row0 = static_cast<std::int64_t>(std::floor(miny - 0.5)), row1 = static_cast<std::int64_t>(std::floor(maxy - 0.5));
    const DrapeVertex* corner[3] = {&a, &b, &c};
    const auto emit_whole = [&](double za, double zb, double zc) {
        const auto ia = vertex_for(mesh, a, za + raise, raise), ib = vertex_for(mesh, b, zb + raise, raise), ic = vertex_for(mesh, c, zc + raise, raise);
        if (ia != ib && ib != ic && ia != ic) {
            mesh.indices.insert(mesh.indices.end(), {ia, ib, ic});
            ++stats_.pieces;
            ++stats_.uncut;
        }
    };
    std::optional<double> zc[3];
    bool grounded = true;
    for (int k = 0; k < 3 && grounded; ++k) {
        zc[k] = height(corner[k]->x, corner[k]->y);
        if (!zc[k]) grounded = false;
    }
    // Inside one lattice cell and on one side of its diagonal: the triangle already lies in one terrain plane.
    if (grounded && col0 == col1 && row0 == row1) {
        const double diag = static_cast<double>(col0 + row0) + 2.0;
        const double fa = a.x + a.y - diag, fb = b.x + b.y - diag, fc = c.x + c.y - diag;
        if ((fa <= 0 && fb <= 0 && fc <= 0) || (fa >= 0 && fb >= 0 && fc >= 0)) {
            emit_whole(*zc[0], *zc[1], *zc[2]);
            return;
        }
    }
    // Cheap case: the three edges are straight (within tol_) and no lattice node inside the triangle stands off its plane by
    // more than tol_ - the terrain only bends along lattice lines and diagonals, which meet at nodes, so the plane through
    // the three corner heights is then the terrain to within tol_. Emit it uncut.
    bool straight[3] = {false, false, false};
    if (grounded && tol > 0) {
        for (int k = 0; k < 3; ++k)
            straight[k] = straight_edge(corner[k]->x, corner[k]->y, *zc[k], corner[(k + 1) % 3]->x, corner[(k + 1) % 3]->y, *zc[(k + 1) % 3], tol);
        if (straight[0] && straight[1] && straight[2]) {
            const double det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
            bool flat = true;
            for (std::int64_t cc = col0; cc <= col1 + 1 && flat; ++cc)
                for (std::int64_t rr = row0; rr <= row1 + 1 && flat; ++rr) {
                    const double nx = static_cast<double>(cc) + 0.5, ny = static_cast<double>(rr) + 0.5;
                    const double l1 = ((nx - a.x) * (c.y - a.y) - (c.x - a.x) * (ny - a.y)) / det;
                    const double l2 = ((b.x - a.x) * (ny - a.y) - (nx - a.x) * (b.y - a.y)) / det;
                    if (l1 <= 1e-9 || l2 <= 1e-9 || l1 + l2 >= 1 - 1e-9) continue; // not strictly inside
                    const auto h = node(cc, rr);
                    if (!h || std::abs(*h - (*zc[0] + l1 * (*zc[1] - *zc[0]) + l2 * (*zc[2] - *zc[0]))) > tol) flat = false;
                }
            if (flat) {
                emit_whole(*zc[0], *zc[1], *zc[2]);
                return;
            }
        }
    }

    cut_emit(mesh, a, b, c, raise, [&](const DrapeVertex& p, double z) {
                for (int k = 0; k < 3; ++k) {
                    if (!straight[k]) continue;
                    const DrapeVertex& e0 = *corner[k];
                    const DrapeVertex& e1 = *corner[(k + 1) % 3];
                    const double dx = e1.x - e0.x, dy = e1.y - e0.y, len2 = dx * dx + dy * dy;
                    if (len2 <= 0) continue;
                    const double t = ((p.x - e0.x) * dx + (p.y - e0.y) * dy) / len2;
                    if (t < 0 || t > 1) continue;
                    if (std::abs((p.x - e0.x) * dy - (p.y - e0.y) * dx) / std::sqrt(len2) > 1e-6) continue;
                    return *zc[k] + (*zc[(k + 1) % 3] - *zc[k]) * t;
                }
                return z;
    });
}

void LatticeDrape::add_ramp(RoadRibbonTile& mesh, const DrapeVertex& a, const DrapeVertex& b, const DrapeVertex& c, double raise) {
    ++stats_.triangles_in;
    const DrapeVertex* corner[3] = {&a, &b, &c};
    double zc[3];
    bool pinned[3];
    for (int k = 0; k < 3; ++k) {
        pinned[k] = !std::isnan(corner[k]->pin);
        if (pinned[k]) zc[k] = corner[k]->pin;
        else {
            const auto h = height(corner[k]->x, corner[k]->y);
            if (!h) { ++stats_.dropped; return; }
            zc[k] = *h;
        }
    }
    const double det = (b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y);
    if (!(std::abs(det) / 2 > kMinArea)) return;
    const auto plane = [&](double x, double y) {
        const double l1 = ((x - a.x) * (c.y - a.y) - (c.x - a.x) * (y - a.y)) / det;
        const double l2 = ((b.x - a.x) * (y - a.y) - (x - a.x) * (b.y - a.y)) / det;
        return zc[0] + l1 * (zc[1] - zc[0]) + l2 * (zc[2] - zc[0]);
    };
    cut_emit(mesh, a, b, c, raise, [&](const DrapeVertex& p, double terrain) {
        for (int k = 0; k < 3; ++k)
            if (std::abs(p.x - corner[k]->x) < 1e-9 && std::abs(p.y - corner[k]->y) < 1e-9) return zc[k];
        const double zp = plane(p.x, p.y);
        for (int k = 0; k < 3; ++k) {
            const DrapeVertex& e0 = *corner[k];
            const DrapeVertex& e1 = *corner[(k + 1) % 3];
            const double dx = e1.x - e0.x, dy = e1.y - e0.y, len2 = dx * dx + dy * dy;
            if (len2 <= 0) continue;
            const double t = ((p.x - e0.x) * dx + (p.y - e0.y) * dy) / len2;
            if (t < 0 || t > 1) continue;
            if (std::abs((p.x - e0.x) * dy - (p.y - e0.y) * dx) / std::sqrt(len2) > 1e-6) continue;
            const bool p0 = pinned[k], p1 = pinned[(k + 1) % 3];
            if (p0 && p1) return zp;      // a deck edge keeps the deck's own line (the deck quads beside it use it)
            if (!p0 && !p1) return terrain; // a ground edge is the terrain (the ground quads beside it lie on it)
            break;
        }
        return std::max(zp, terrain);
    });
}

} // namespace g2m::world::detail
