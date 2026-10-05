#include "g2m/layer/road_junction.h"
#include "g2m/layer/road_attributes.h"
#include "g2m/layer/road_cross_section.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <tuple>

namespace g2m {
namespace {
using geom::PointMm;
constexpr int kLayers = 3;
constexpr int kRight = 0, kLeft = 1;      // sides in the OUTWARD frame of an approach (sigma away from the junction)
constexpr double kTrimClampM = 0.5;       // trim <= L - 0.5 (design 3.4); also the polyline end margin of 3.3

struct Tagged { PointMm p; std::uint16_t tag = 0; };

// One used approach with its sampled edge polylines (design 3.3).
struct Geo {
    std::size_t entry = 0;
    bool at_end = false;
    std::int64_t way_id = 0;
    std::uint32_t start_ref = 0;
    const RoadReferenceLine* ref = nullptr;
    const RoadCrossSection* xs = nullptr;
    bool bridge = false;
    double length = 0;
    std::vector<double> sigma;                                    // sample stations, ascending, sigma.front() == 0
    std::array<std::array<std::vector<PointMm>, 2>, kLayers> edge; // [layer][side][sample]
    PointMm direction;                                            // first reference segment
};

struct Corner {
    bool found = false;
    PointMm point;
    double sigma_a = 0, sigma_b = 0;   // station on the left-edge approach (a) and the right-edge approach (b)
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

double edge_t(const CarveFootprint& f, int layer, int side, bool at_end) {
    const bool reference_left = (side == kLeft) != at_end;
    switch (layer) {
    case 0: return reference_left ? f.roadbed_left : f.roadbed_right;
    case 1: return reference_left ? f.paved_left : f.paved_right;
    default: return reference_left ? f.outer_left : f.outer_right;
    }
}

bool place(const Geo& g, double sigma, int layer, int side, PointMm& out) {
    const double s = std::clamp(g.at_end ? g.length - sigma : sigma, 0.0, g.length);
    const CarveFootprint f = carve_footprint(*g.xs, s);
    const auto p = lateral_point(*g.ref, s, edge_t(f, layer, side, g.at_end));
    if (!p) return false;
    out = {std::llround(p->x * 1000.0), std::llround(p->y * 1000.0)};
    return true;
}

// The edge polylines and the first direction of one approach. False if a point cannot be placed.
bool sample_approach(Geo& g, const RoadRules& rules) {
    const double spacing = static_cast<double>(rules.junction.edge_station_spacing_mm) / 1000.0;
    const double reach = static_cast<double>(rules.junction.reach_max_m);
    const double sigma_max = std::max(0.0, std::min(g.length - kTrimClampM, reach));
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
            reference[k] = {std::llround(p->x * 1000.0), std::llround(p->y * 1000.0)};
        }
    }
    if (g.sigma.size() < 2) return false;
    g.direction = minus(reference[1], reference[0]);
    return g.direction.x != 0 || g.direction.y != 0;
}

// First intersection of the outward-left edge of A with the outward-right edge of B on one layer (design 3.3):
// in increasing max(segment index of A, segment index of B), then increasing index of A. Parallel and collinear
// segment pairs never produce a corner.
Corner find_corner(const Geo& A, const Geo& B, int layer) {
    const auto& pa = A.edge[layer][kLeft];
    const auto& pb = B.edge[layer][kRight];
    Corner none;
    if (pa.size() < 2 || pb.size() < 2) return none;
    const std::size_t na = pa.size() - 1, nb = pb.size() - 1;
    const auto attempt = [&](std::size_t ia, std::size_t ib, Corner& out) {
        const PointMm a = pa[ia], b = pa[ia + 1], c = pb[ib], d = pb[ib + 1];
        if (std::max(a.x, b.x) < std::min(c.x, d.x) || std::max(c.x, d.x) < std::min(a.x, b.x) ||
            std::max(a.y, b.y) < std::min(c.y, d.y) || std::max(c.y, d.y) < std::min(a.y, b.y)) return false;
        if (!geom::segments_intersect(a, b, c, d)) return false;
        const PointMm r = minus(b, a), s = minus(d, c), ca = minus(c, a);
        std::int64_t den = cross(r, s), num_u = cross(ca, s), num_v = cross(ca, r);
        if (den == 0) return false;
        if (den < 0) { den = -den; num_u = -num_u; num_v = -num_v; }
        out.found = true;
        out.point = {a.x + round_div(r.x * num_u, den), a.y + round_div(r.y * num_u, den)};
        out.sigma_a = A.sigma[ia] + static_cast<double>(num_u) / static_cast<double>(den) * (A.sigma[ia + 1] - A.sigma[ia]);
        out.sigma_b = B.sigma[ib] + static_cast<double>(num_v) / static_cast<double>(den) * (B.sigma[ib + 1] - B.sigma[ib]);
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

bool same(PointMm a, PointMm b) { return a == b; }

void clean_ring(std::vector<Tagged>& v) {
    bool changed = true;
    while (changed && v.size() >= 2) {
        changed = false;
        for (std::size_t i = 0; i < v.size() && v.size() >= 2;) {   // consecutive duplicates, cyclic
            const std::size_t next = (i + 1) % v.size();
            if (same(v[i].p, v[next].p)) { v.erase(v.begin() + static_cast<std::ptrdiff_t>(i)); changed = true; }
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

struct Frame {
    const std::vector<Geo>* geos = nullptr;
    std::vector<double> trims;
    // corners[layer][i]: the corner between approach i (left edge) and approach i + 1 (right edge).
    std::array<std::vector<Corner>, kLayers> corners;
};

bool corner_in_trims(const Frame& fr, int layer, std::size_t i) {
    const std::size_t k = fr.geos->size(), j = (i + 1) % k;
    const Corner& c = fr.corners[layer][i];
    return c.found && c.sigma_a <= fr.trims[i] && c.sigma_b <= fr.trims[j];
}

bool trim_point(const Geo& g, double trim, int layer, int side, PointMm& out) { return place(g, trim, layer, side, out); }

// The ring of one layer, walked counter-clockwise (design 3.5). False if a trim point cannot be placed.
bool build_ring(const Frame& fr, int layer, std::vector<Tagged>& out) {
    const auto& geos = *fr.geos;
    const std::size_t k = geos.size();
    out.clear();
    for (std::size_t i = 0; i < k; ++i) {
        const std::size_t j = (i + 1) % k;
        const Geo& gi = geos[i];
        const Geo& gj = geos[j];
        const auto tag_i = static_cast<std::uint16_t>(i), tag_j = static_cast<std::uint16_t>(j);
        PointMm right, left;
        if (!trim_point(gi, fr.trims[i], layer, kRight, right) || !trim_point(gi, fr.trims[i], layer, kLeft, left)) return false;
        out.push_back({right, tag_i}); out.push_back({left, tag_i});
        const bool corner = corner_in_trims(fr, layer, i);
        const double from_i = corner ? fr.corners[layer][i].sigma_a : -1.0;   // exclusive lower bound on i's left edge
        const double from_j = corner ? fr.corners[layer][i].sigma_b : -1.0;
        for (std::size_t s = gi.sigma.size(); s-- > 0;)
            if (gi.sigma[s] < fr.trims[i] && gi.sigma[s] > from_i) out.push_back({gi.edge[layer][kLeft][s], tag_i});
        if (corner) out.push_back({fr.corners[layer][i].point, tag_i});
        for (std::size_t s = 0; s < gj.sigma.size(); ++s)
            if (gj.sigma[s] < fr.trims[j] && gj.sigma[s] > from_j) out.push_back({gj.edge[layer][kRight][s], tag_j});
    }
    clean_ring(out);
    return true;
}


struct Key {
    std::int64_t way = 0; std::uint32_t start = 0; bool at_end = false;
    friend bool operator<(const Key& a, const Key& b) { return std::tie(a.way, a.start, a.at_end) < std::tie(b.way, b.start, b.at_end); }
};

struct RawApproach { std::size_t entry; bool at_end; };

Result<RoadJunction> build_from(const RoadGeomTile& tile, std::int64_t node_id, PointMm node_pos,
    const std::vector<RawApproach>& raw, const RoadRules& rules, JunctionBuildStats* stats) {
    const auto& graph = tile.source.graph;
    RoadJunction j;
    j.nodes = {node_id};
    // ---- approaches and their status (3.1) ----
    struct Item { RawApproach r; Key key; JunctionApproachStatus status; };
    std::vector<Item> items;
    for (const auto& r : raw) {
        const RoadGeomEntry& e = tile.entries[r.entry];
        Item it{r, {e.way_id, e.stretch.start_ref, r.at_end}, JunctionApproachStatus::Used};
        if (!e.profile) {
            it.status = JunctionApproachStatus::NoProfile;
            const auto way = std::lower_bound(graph.ways.begin(), graph.ways.end(), e.way_id,
                [](const RoadGraphWay& w, std::int64_t id) { return w.osm_id < id; });
            if (way != graph.ways.end() && way->osm_id == e.way_id) {
                const auto attrs = resolve_road_attributes(*way);
                if (attrs && attrs->tunnel) it.status = JunctionApproachStatus::Tunnel;
            }
        } else if (!e.cross_section) it.status = JunctionApproachStatus::NoCrossSection;
        else if (e.profile->attributes.tunnel) it.status = JunctionApproachStatus::Tunnel;
        items.push_back(it);
    }
    std::sort(items.begin(), items.end(), [](const Item& a, const Item& b) { return a.key < b.key; });
    std::vector<Item> used, excluded;
    for (const auto& it : items) (it.status == JunctionApproachStatus::Used ? used : excluded).push_back(it);
    const bool all_tunnels = !items.empty() && std::all_of(items.begin(), items.end(), [](const Item& i) { return i.status == JunctionApproachStatus::Tunnel; });
    if (!excluded.empty()) j.flags |= static_cast<std::uint16_t>(JunctionFlags::Partial);

    const auto finish = [&](JunctionDecline decline) -> Result<RoadJunction> {
        j.decline = decline;
        j.approaches.clear();
        for (const auto& u : used) j.approaches.push_back({u.r.entry, u.r.at_end, u.status, 0.0});
        for (const auto& x : excluded) j.approaches.push_back({x.r.entry, x.r.at_end, x.status, 0.0});
        return j;
    };
    const auto record = [&](const RoadJunction& built) {
        if (!stats) return;
        ++stats->built;
        ++stats->declines[static_cast<std::size_t>(built.decline)];
        stats->excluded_approaches += excluded.size();
        stats->approaches_used += used.size();
        if (built.flags & static_cast<std::uint16_t>(JunctionFlags::Partial)) ++stats->partial;
        if (built.flags & static_cast<std::uint16_t>(JunctionFlags::OnStructure)) ++stats->on_structure;
        if (built.flags & static_cast<std::uint16_t>(JunctionFlags::HullFallback)) ++stats->hull_fallbacks;
        stats->max_ring_vertices = std::max(stats->max_ring_vertices, built.rings[2].vertices.size());
    };
    if (all_tunnels) { auto r = finish(JunctionDecline::Underground); record(*r); return r; }
    if (used.size() < 2) { auto r = finish(JunctionDecline::SingleApproach); record(*r); return r; }

    // ---- geometry of the used approaches ----
    std::vector<Geo> geos(used.size());
    for (std::size_t i = 0; i < used.size(); ++i) {
        const RoadGeomEntry& e = tile.entries[used[i].r.entry];
        Geo& g = geos[i];
        g.entry = used[i].r.entry; g.at_end = used[i].r.at_end; g.way_id = e.way_id; g.start_ref = e.stretch.start_ref;
        g.ref = &e.profile->reference; g.xs = &*e.cross_section; g.bridge = e.profile->attributes.bridge;
        g.length = e.profile->reference.length_m;
        if (!sample_approach(g, rules)) { auto r = finish(JunctionDecline::Degenerate); record(*r); return r; }
    }
    // ---- angular order: half-plane, then the exact cross product, ties by the key (3.3) ----
    const auto half = [](PointMm d) { return (d.y > 0 || (d.y == 0 && d.x > 0)) ? 0 : 1; };
    std::vector<std::size_t> order(geos.size());
    for (std::size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::sort(order.begin(), order.end(), [&](std::size_t a, std::size_t b) {
        const PointMm da = geos[a].direction, db = geos[b].direction;
        const int ha = half(da), hb = half(db);
        if (ha != hb) return ha < hb;
        const std::int64_t c = cross(da, db);
        if (c != 0) return c > 0;
        return used[a].key < used[b].key;
    });
    // `used` is ascending by key, so index 0 is the smallest key; rotate the order to start at it.
    const auto first = std::find(order.begin(), order.end(), std::size_t{0});
    std::rotate(order.begin(), first, order.end());
    {
        std::vector<Geo> g2; std::vector<Item> u2;
        for (const auto i : order) { g2.push_back(std::move(geos[i])); u2.push_back(used[i]); }
        geos = std::move(g2); used = std::move(u2);
    }
    const std::size_t k = geos.size();

    // ---- corners per layer, trims from the outer layer (3.3, 3.4) ----
    Frame fr; fr.geos = &geos;
    for (int layer = 0; layer < kLayers; ++layer) {
        fr.corners[layer].resize(k);
        for (std::size_t i = 0; i < k; ++i) fr.corners[layer][i] = find_corner(geos[i], geos[(i + 1) % k], layer);
    }
    const double trim_min = static_cast<double>(rules.junction.trim_min_mm) / 1000.0;
    const double margin = static_cast<double>(rules.junction.trim_margin_mm) / 1000.0;
    const double step = static_cast<double>(rules.junction.bowtie_step_mm) / 1000.0;
    std::vector<double> base(k), cap(k), extra(k, 0.0);
    bool beyond = false;
    for (std::size_t i = 0; i < k; ++i) {
        const Corner& next = fr.corners[2][i];
        const Corner& prev = fr.corners[2][(i + k - 1) % k];
        double station = trim_min;
        if (next.found) station = std::max(station, next.sigma_a);
        if (prev.found) station = std::max(station, prev.sigma_b);
        base[i] = station + margin;
        cap[i] = std::max(geos[i].length - kTrimClampM, 0.0);
        if ((next.found && next.sigma_a > cap[i]) || (prev.found && prev.sigma_b > cap[i])) beyond = true;
    }
    if (beyond) j.flags |= static_cast<std::uint16_t>(JunctionFlags::Partial);
    const auto set_trims = [&] {
        fr.trims.assign(k, 0.0);
        for (std::size_t i = 0; i < k; ++i) fr.trims[i] = std::min(base[i] + extra[i], cap[i]);
    };

    // ---- rings, bow-tie repair, hull fallback (3.5) ----
    std::array<std::vector<Tagged>, kLayers> rings;
    std::array<bool, kLayers> hull{};
    int rounds_used = 0;
    set_trims();
    for (int round = 0;; ++round) {
        bool all_simple = true;
        std::vector<std::pair<std::size_t, std::size_t>> crossings;
        std::array<std::vector<std::pair<std::size_t, std::size_t>>, kLayers> per_layer;
        for (int layer = 0; layer < kLayers; ++layer) {
            if (!build_ring(fr, layer, rings[layer])) { auto r = finish(JunctionDecline::Degenerate); record(*r); return r; }
            if (!ring_simple(rings[layer], &per_layer[layer])) all_simple = false;
        }
        if (all_simple) break;
        // Which approaches take part in a crossing: the tags of the four end vertices of every crossing pair.
        std::vector<bool> involved(k, false);
        bool any = false;
        for (int layer = 0; layer < kLayers; ++layer) {
            const auto& v = rings[layer];
            for (const auto& [a, b] : per_layer[layer]) {
                for (const std::size_t s : {a, a + 1, b, b + 1}) { involved[v[s % v.size()].tag] = true; any = true; }
            }
        }
        if (round >= rules.junction.bowtie_rounds || !any) break;
        const std::vector<double> before = fr.trims;
        for (std::size_t i = 0; i < k; ++i) if (involved[i]) extra[i] += step;
        set_trims();
        if (fr.trims == before) break;           // every involved trim sits on its clamp: nothing left to extend
        ++rounds_used;
    }
    for (int layer = 0; layer < kLayers; ++layer) {
        if (ring_simple(rings[layer], nullptr)) continue;
        std::vector<PointMm> pts;
        for (std::size_t i = 0; i < k; ++i) {
            PointMm a, b;
            if (!trim_point(geos[i], fr.trims[i], layer, kRight, a) || !trim_point(geos[i], fr.trims[i], layer, kLeft, b)) {
                auto r = finish(JunctionDecline::Degenerate); record(*r); return r;
            }
            pts.push_back(a); pts.push_back(b);
        }
        rings[layer] = convex_hull(std::move(pts));
        hull[layer] = true;
        if (rings[layer].size() < 3 || twice_area(rings[layer]) <= 0) { auto r = finish(JunctionDecline::Degenerate); record(*r); return r; }
        j.flags |= static_cast<std::uint16_t>(JunctionFlags::HullFallback);
    }

    // ---- reach (3.5 step 6) ----
    const std::int64_t reach_mm = static_cast<std::int64_t>(rules.junction.reach_max_m) * 1000;
    for (int layer = 0; layer < kLayers; ++layer)
        for (const auto& v : rings[layer]) {
            const PointMm d = minus(v.p, node_pos);
            if (d.x * d.x + d.y * d.y > reach_mm * reach_mm) { auto r = finish(JunctionDecline::ReachExceeded); record(*r); return r; }
        }

    // ---- the record ----
    for (std::size_t i = 0; i < used.size(); ++i) j.approaches.push_back({used[i].r.entry, used[i].r.at_end, used[i].status, fr.trims[i]});
    for (const auto& x : excluded) j.approaches.push_back({x.r.entry, x.r.at_end, x.status, 0.0});
    for (int layer = 0; layer < kLayers; ++layer) {
        j.rings[layer].vertices.reserve(rings[layer].size());
        for (const auto& v : rings[layer]) j.rings[layer].vertices.push_back(v.p);
    }
    if (std::all_of(geos.begin(), geos.end(), [](const Geo& g) { return g.bridge; })) j.flags |= static_cast<std::uint16_t>(JunctionFlags::OnStructure);
    if (stats) {
        stats->bowtie_rounds += static_cast<std::size_t>(rounds_used);
        if (rounds_used > 0 && !std::any_of(hull.begin(), hull.end(), [](bool h) { return h; })) ++stats->bowtie_repaired;
    }
    record(j);
    return j;
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

namespace {
const RoadGraphNode* find_node(const RoadGraph& g, std::int64_t id) {
    const auto it = std::lower_bound(g.nodes.begin(), g.nodes.end(), id, [](const RoadGraphNode& n, std::int64_t v) { return n.osm_id < v; });
    return it != g.nodes.end() && it->osm_id == id ? &*it : nullptr;
}
std::vector<RawApproach> approaches_at(const RoadGeomTile& tile, std::int64_t node_id) {
    std::vector<RawApproach> out;
    const auto& ways = tile.source.graph.ways;
    for (std::size_t i = 0; i < tile.entries.size(); ++i) {
        const RoadGeomEntry& e = tile.entries[i];
        const auto way = std::lower_bound(ways.begin(), ways.end(), e.way_id, [](const RoadGraphWay& w, std::int64_t id) { return w.osm_id < id; });
        if (way == ways.end() || way->osm_id != e.way_id || e.stretch.start_ref >= way->node_ids.size() || e.stretch.end_ref >= way->node_ids.size()) continue;
        if (way->node_ids[e.stretch.start_ref] == node_id) out.push_back({i, false});
        if (way->node_ids[e.stretch.end_ref] == node_id) out.push_back({i, true});
    }
    return out;
}
} // namespace

Result<RoadJunction> build_node_junction(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    std::int64_t node_id, const RoadRules& rules, JunctionBuildStats* stats) {
    const RoadGraphNode* node = find_node(tile.source.graph, node_id);
    if (!node) return make_error(ErrorCode::InvalidArgument, "road junction: node not in the tile graph");
    const std::uint32_t degree = context.degree(node_id);
    if (degree < 3) return make_error(ErrorCode::InvalidArgument, "road junction: node degree below 3");
    const auto raw = approaches_at(tile, node_id);
    if (raw.size() != degree) return make_error(ErrorCode::InvalidArgument, "road junction: the tile lacks some approach of the node");
    return build_from(tile, node_id, node->position, raw, rules, stats);
}

Result<std::vector<RoadJunction>> build_node_junctions(const RoadGeomTile& tile, const RoadReferenceBuilder& context,
    const RoadRules& rules, JunctionBuildStats* stats) {
    // One pass over the entries: (node, entry, at_end), grouped by node.
    const auto& ways = tile.source.graph.ways;
    struct Hit { std::int64_t node; std::size_t entry; bool at_end; };
    std::vector<Hit> hits;
    for (std::size_t i = 0; i < tile.entries.size(); ++i) {
        const RoadGeomEntry& e = tile.entries[i];
        const auto way = std::lower_bound(ways.begin(), ways.end(), e.way_id, [](const RoadGraphWay& w, std::int64_t id) { return w.osm_id < id; });
        if (way == ways.end() || way->osm_id != e.way_id || e.stretch.start_ref >= way->node_ids.size() || e.stretch.end_ref >= way->node_ids.size()) continue;
        hits.push_back({way->node_ids[e.stretch.start_ref], i, false});
        hits.push_back({way->node_ids[e.stretch.end_ref], i, true});
    }
    std::sort(hits.begin(), hits.end(), [](const Hit& a, const Hit& b) { return std::tie(a.node, a.entry, a.at_end) < std::tie(b.node, b.entry, b.at_end); });
    std::vector<RoadJunction> out;
    for (const auto& node : tile.source.graph.nodes) {   // ascending OSM id
        const std::uint32_t degree = context.degree(node.osm_id);
        if (degree < 3) continue;
        if (stats) ++stats->junction_nodes;
        const auto lo = std::lower_bound(hits.begin(), hits.end(), node.osm_id, [](const Hit& h, std::int64_t id) { return h.node < id; });
        std::vector<RawApproach> raw;
        for (auto it = lo; it != hits.end() && it->node == node.osm_id; ++it) raw.push_back({it->entry, it->at_end});
        if (raw.size() != degree) { if (stats) ++stats->skipped_incomplete; continue; }
        auto built = build_from(tile, node.osm_id, node.position, raw, rules, stats);
        if (!built) return built.error();
        out.push_back(std::move(*built));
    }
    return out;
}

} // namespace g2m
