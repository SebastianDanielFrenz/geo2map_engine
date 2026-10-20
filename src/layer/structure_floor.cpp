// src/layer/structure_floor.cpp - see g2m/layer/structure_floor.h.
#include "g2m/layer/structure_floor.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <utility>

namespace g2m::structure {
namespace {
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

struct Adjacent { int node; double length; };

// Shortest graph distance from `source` to every node. With `stop_at_portals` a portal node other than the source gets its
// distance but is not expanded (`through` is then kInf there): a route may end at another portal, never pass it.
void dijkstra(const std::vector<std::vector<Adjacent>>& adjacent, const std::vector<int>& portal_at, int source, bool stop_at_portals,
    std::vector<double>& reach, std::vector<double>& through) {
    const std::size_t n = adjacent.size();
    reach.assign(n, kInf); through.assign(n, kInf);
    std::set<std::pair<double, int>> queue;
    reach[static_cast<std::size_t>(source)] = 0; queue.insert({0.0, source});
    while (!queue.empty()) {
        const auto [d, u] = *queue.begin(); queue.erase(queue.begin());
        const bool terminal = stop_at_portals && u != source && portal_at[static_cast<std::size_t>(u)] >= 0;
        if (!terminal) through[static_cast<std::size_t>(u)] = d;
        if (terminal) continue;
        for (const auto& a : adjacent[static_cast<std::size_t>(u)]) {
            const double nd = d + a.length;
            auto& r = reach[static_cast<std::size_t>(a.node)];
            if (nd < r) { if (r != kInf) queue.erase({r, a.node}); r = nd; queue.insert({nd, a.node}); }
        }
    }
}

double snap(double v, double quantum) {
    if (!(quantum > 0)) return v;
    return std::floor(v / quantum + 0.5) * quantum;
}
} // namespace

std::size_t station_count(double length_m) { return static_cast<std::size_t>(std::ceil(length_m)) + 1; }
double station_m(double length_m, std::size_t k) { return std::min(static_cast<double>(k), length_m); }

NetworkFloor network_design_floor(const FloorNetwork& net, const NetworkFloorParams& prm) {
    NetworkFloor out;
    const std::size_t n_nodes = static_cast<std::size_t>(std::max(0, net.node_count)), n_edges = net.stretches.size(), n_portals = net.portals.size();
    out.floor.resize(n_edges);
    out.node_floor.assign(n_nodes, kNaN);
    out.portal_z_m.resize(n_portals); out.portal_shift_m.assign(n_portals, 0.0);
    for (std::size_t p = 0; p < n_portals; ++p) out.portal_z_m[p] = net.portals[p].z_m;
    if (n_edges == 0) { out.ok = true; out.converged = true; return out; }
    const double g = prm.max_grade;

    std::vector<std::vector<Adjacent>> adjacent(n_nodes);
    for (const auto& e : net.stretches) {
        adjacent[static_cast<std::size_t>(e.from)].push_back({e.to, e.length_m});
        if (e.to != e.from) adjacent[static_cast<std::size_t>(e.to)].push_back({e.from, e.length_m});
    }
    std::vector<int> portal_at(n_nodes, -1);
    for (std::size_t p = 0; p < n_portals; ++p) {
        auto& slot = portal_at[static_cast<std::size_t>(net.portals[p].node)];
        if (slot < 0) slot = static_cast<int>(p);
    }
    // Per portal: the distance to every node over any route (cones, the depth reach) and over routes that pass no other
    // portal (the chord pairs).
    std::vector<std::vector<double>> full(n_portals), reach_adj(n_portals), through_adj(n_portals);
    {
        std::vector<double> scratch;
        for (std::size_t p = 0; p < n_portals; ++p) {
            dijkstra(adjacent, portal_at, net.portals[p].node, false, full[p], scratch);
            dijkstra(adjacent, portal_at, net.portals[p].node, true, reach_adj[p], through_adj[p]);
        }
    }
    const auto node_of = [&](std::size_t p) { return static_cast<std::size_t>(net.portals[p].node); };

    // The reach handler (phase 7a-1's portal_shift on the pairs of adjacent portals).
    std::vector<std::pair<std::size_t, std::size_t>> pair_list;   // ascending (a, b)
    std::vector<double> pair_distance;
    for (std::size_t a = 0; a < n_portals; ++a)
        for (std::size_t b = a + 1; b < n_portals; ++b) {
            const double d = reach_adj[a][node_of(b)];
            if (d == kInf || !(d > 0)) continue;
            pair_list.emplace_back(a, b); pair_distance.push_back(d);
        }
    out.pairs = pair_list.size();
    for (std::size_t k = 0; k < pair_list.size(); ++k) {
        const auto [a, b] = pair_list[k];
        const double d = pair_distance[k], diff = net.portals[b].z_m - net.portals[a].z_m, mag = std::fabs(diff);
        if (mag > g * d) ++out.pairs_beyond_grade;
        const double reach = prm.reach_factor * g * d;
        if (mag <= reach) continue;
        ++out.pairs_shifted;
        const double excess = mag - reach, up = diff < 0 ? -1.0 : 1.0;   // a moves by +up * share, b by -up * share
        double share_a = excess / 2.0, share_b = excess / 2.0;
        if (net.portals[a].free_end && !net.portals[b].free_end) { share_a = excess; share_b = 0; }
        else if (net.portals[b].free_end && !net.portals[a].free_end) { share_a = 0; share_b = excess; }
        const double move_a = snap(up * share_a, prm.quantum_m), move_b = snap(-up * share_b, prm.quantum_m);
        if (std::fabs(move_a) > std::fabs(out.portal_shift_m[a])) out.portal_shift_m[a] = move_a;
        if (std::fabs(move_b) > std::fabs(out.portal_shift_m[b])) out.portal_shift_m[b] = move_b;
    }
    for (std::size_t p = 0; p < n_portals; ++p) out.portal_z_m[p] = net.portals[p].z_m + out.portal_shift_m[p];
    for (std::size_t k = 0; k < pair_list.size(); ++k) {
        const auto [a, b] = pair_list[k];
        if (std::fabs(out.portal_z_m[b] - out.portal_z_m[a]) > g * pair_distance[k] + 1e-9) ++out.pairs_still_beyond_grade;
    }

    // The upper bound u at every station.
    const double depth_reach = g > 0 ? prm.needed / g : 0.0;
    std::vector<double> to_full(n_portals), to_adj(n_portals);
    for (std::size_t e = 0; e < n_edges; ++e) {
        const auto& s = net.stretches[e];
        const std::size_t count = station_count(s.length_m), from = static_cast<std::size_t>(s.from), to = static_cast<std::size_t>(s.to);
        auto& f = out.floor[e]; f.assign(count, kInf);
        for (std::size_t k = 0; k < count; ++k) {
            const double st = station_m(s.length_m, k);
            double nearest = kInf;
            for (std::size_t p = 0; p < n_portals; ++p) {
                to_full[p] = std::min(full[p][from] + st, full[p][to] + (s.length_m - st));
                to_adj[p] = std::min(through_adj[p][from] + st, through_adj[p][to] + (s.length_m - st));
                nearest = std::min(nearest, to_full[p]);
            }
            double chord = kInf;
            for (std::size_t q = 0; q < pair_list.size(); ++q) {
                const auto [a, b] = pair_list[q];
                const double d = pair_distance[q], da = to_adj[a], db = to_adj[b];
                if (da == kInf || db == kInf || da + db > d + 1e-9 * (1.0 + d)) continue;
                const double za = out.portal_z_m[a], zb = out.portal_z_m[b];
                chord = std::min(chord, za + (zb - za) * da / d);
            }
            const double ground = k < s.ground.size() ? s.ground[k] : kNaN;
            double u = chord;
            if (ground == ground && !(nearest < depth_reach)) u = std::min(u, ground - prm.needed);
            for (std::size_t p = 0; p < n_portals; ++p) if (to_full[p] == 0) u = std::min(u, out.portal_z_m[p]);   // the floor meets an anchor exactly
            f[k] = u;
        }
    }

    // The greatest g-Lipschitz function below u in the graph metric: relax every stretch forward and backward, join the
    // end stations of a node, and repeat to the fixed point.
    std::vector<std::vector<std::size_t>> ends_at(n_nodes);   // (stretch << 1 | end) per node
    for (std::size_t e = 0; e < n_edges; ++e) {
        ends_at[static_cast<std::size_t>(net.stretches[e].from)].push_back(e << 1);
        ends_at[static_cast<std::size_t>(net.stretches[e].to)].push_back((e << 1) | 1);
    }
    const auto end_ref = [&](std::size_t code) -> double& { auto& f = out.floor[code >> 1]; return (code & 1) ? f.back() : f.front(); };
    const std::size_t max_passes = 2 * n_edges + 8;
    bool changed = true;
    while (changed && out.passes < max_passes) {
        changed = false; ++out.passes;
        for (std::size_t e = 0; e < n_edges; ++e) {
            auto& f = out.floor[e]; const double len = net.stretches[e].length_m;
            for (std::size_t k = 1; k < f.size(); ++k) {
                const double v = f[k - 1] + g * (station_m(len, k) - station_m(len, k - 1));
                if (v < f[k]) { f[k] = v; changed = true; }
            }
            for (std::size_t k = f.size() - 1; k-- > 0;) {
                const double v = f[k + 1] + g * (station_m(len, k + 1) - station_m(len, k));
                if (v < f[k]) { f[k] = v; changed = true; }
            }
        }
        for (std::size_t node = 0; node < n_nodes; ++node) {
            if (ends_at[node].size() < 2) continue;
            double m = kInf;
            for (const auto code : ends_at[node]) m = std::min(m, end_ref(code));
            for (const auto code : ends_at[node]) { auto& v = end_ref(code); if (m < v) { v = m; changed = true; } }
        }
    }
    out.converged = !changed;

    // Raised to the portal cones (a max of g-Lipschitz functions stays g-Lipschitz).
    for (std::size_t e = 0; e < n_edges; ++e) {
        const auto& s = net.stretches[e]; auto& f = out.floor[e];
        const std::size_t from = static_cast<std::size_t>(s.from), to = static_cast<std::size_t>(s.to);
        for (std::size_t k = 0; k < f.size(); ++k) {
            const double st = station_m(s.length_m, k);
            for (std::size_t p = 0; p < n_portals; ++p) {
                const double d = std::min(full[p][from] + st, full[p][to] + (s.length_m - st));
                f[k] = std::max(f[k], out.portal_z_m[p] - g * d);
            }
        }
    }
    out.ok = true;
    for (const auto& f : out.floor) for (const double v : f) if (!(v < kInf) || v != v) out.ok = false;
    if (!out.ok) return out;
    for (std::size_t e = 0; e < n_edges; ++e) {
        out.node_floor[static_cast<std::size_t>(net.stretches[e].from)] = out.floor[e].front();
        out.node_floor[static_cast<std::size_t>(net.stretches[e].to)] = out.floor[e].back();
    }
    for (std::size_t p = 0; p < n_portals; ++p)
        if (std::fabs(out.node_floor[node_of(p)] - out.portal_z_m[p]) > 1e-6) ++out.anchor_misses;
    return out;
}

} // namespace g2m::structure
