#include "g2m/layer/road_junction.h"
#include "g2m/layer/road_attributes.h"
#include "g2m/layer/road_cross_section.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
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
        return c.found && c.sigma_a <= bound(arrival(k)) && c.sigma_b <= bound(walk[(k + 1) % walk.size()]);
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
        const double lo = cv ? c.sigma_a : kNegInf;
        const Dart& a = darts[A];
        if (!a.internal) {
            for (std::size_t s = a.sigma.size(); s-- > 0;)
                if (a.sigma[s] < fr.trims[A] && a.sigma[s] > lo) out.push_back({a.edge[layer][kLeft][s], static_cast<std::uint16_t>(A)});
        } else {
            // A runs back along the internal stretch; its far end is the start of D, where pair k - 1 has its corner.
            const std::size_t km1 = (k + m - 1) % m;
            const bool prev = fr.corner_valid(layer, km1);
            const double ub = prev ? a.length - fr.corners[layer][km1].sigma_b : a.length;
            for (std::size_t s = a.sigma.size(); s-- > 0;)
                if (a.sigma[s] > lo && (prev ? a.sigma[s] < ub : a.sigma[s] <= ub))
                    out.push_back({a.edge[layer][kLeft][s], static_cast<std::uint16_t>(A)});
        }
        if (cv) out.push_back({c.point, static_cast<std::uint16_t>(A)});
        const Dart& n = darts[N];
        if (!n.internal) {
            const double from = cv ? c.sigma_b : kNegInf;
            for (std::size_t s = 0; s < n.sigma.size(); ++s)
                if (n.sigma[s] < fr.trims[N] && n.sigma[s] > from) out.push_back({n.edge[layer][kRight][s], static_cast<std::uint16_t>(N)});
        }
    }
    clean_ring(out);
    return true;
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
        for (int layer = 0; layer < kLayers; ++layer) {
            fr.corners[layer].resize(m);
            for (std::size_t k = 0; k < m; ++k)
                fr.corners[layer][k] = find_corner(darts[fr.arrival(k)], darts[fr.walk[(k + 1) % m]], layer, options.corner_caps);
        }
        bool beyond = false;
        for (std::size_t k = 0; k < m; ++k) {
            const Corner& c = fr.corners[2][k];
            if (!c.found) continue;
            const std::size_t a = fr.arrival(k), b = fr.walk[(k + 1) % m];
            if (!darts[a].internal) { base[a] = std::max(base[a], std::max(c.sigma_a, trim_min) + margin); if (c.sigma_a > cap[a]) beyond = true; }
            if (!darts[b].internal) { base[b] = std::max(base[b], std::max(c.sigma_b, trim_min) + margin); if (c.sigma_b > cap[b]) beyond = true; }
        }
        if (beyond) j.flags |= static_cast<std::uint16_t>(JunctionFlags::Partial);
        set_trims();
        // ---- rings, bow-tie repair (3.5) ----
        for (int round = 0;; ++round) {
            bool all_simple = true;
            std::array<std::vector<std::pair<std::size_t, std::size_t>>, kLayers> per_layer;
            for (int layer = 0; layer < kLayers; ++layer) {
                if (!build_ring(fr, layer, rings[layer])) return finish(JunctionDecline::Degenerate);
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

} // namespace g2m
