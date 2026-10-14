// tests/route/route_scenes.h - synthetic road graphs shared by the route tests.
#pragma once

#include "route_reference.h"
#include "splitmix64.h"

#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace g2m_test {

using g2m::RoadGraph;
using g2m::route::TurnRestriction;

// A cross at C(0,0): S south, N north (y = north_y), W west, E east, 100 m arms.
// Way 50 links W to N through (-100, north_y) unless `no_link`.
inline RoadGraph cross(bool no_link = false, std::int64_t north_y = 100, std::vector<std::pair<std::string, std::string>> north_tags = {}) {
    GraphBuilder b;
    b.node(1, 0, 0); b.node(2, 0, -100); b.node(3, 0, north_y); b.node(4, -100, 0); b.node(5, 100, 0); b.node(6, -100, north_y);
    b.way(10, {2, 1});
    b.way(20, {1, 4});
    b.way(30, {1, 3}, std::move(north_tags));
    b.way(40, {1, 5});
    if (!no_link) b.way(50, {4, 6, 3});
    return b.build();
}

// A jittered grid of streets with random oneways, speeds, gaps and restrictions.
struct RandomScene {
    RoadGraph graph;
    std::vector<TurnRestriction> rules;
    std::vector<std::int64_t> node_ids;
};
inline RandomScene random_scene(std::uint64_t seed, int n, double restriction_share) {
    g2m_test::SplitMix64 rng(seed);
    GraphBuilder b;
    RandomScene scene;
    const auto id = [&](int i, int j) { return std::int64_t{1000} + i * n + j; };
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const auto jx = static_cast<std::int64_t>(rng.next() % 41);
            const auto jy = static_cast<std::int64_t>(rng.next() % 41);
            b.node(id(i, j), 200 * j + jx - 20, 200 * i + jy - 20);
        }
    std::int64_t way_id = 1;
    std::vector<std::vector<std::pair<std::int64_t, std::int64_t>>> incident(static_cast<std::size_t>(n * n));
    const char* classes[] = {"residential", "tertiary", "secondary", "primary", "service"};
    for (int dir = 0; dir < 2; ++dir) {
        for (int line = 0; line < n; ++line) {
            int k = 0;
            while (k < n - 1) {
                const int run = 1 + static_cast<int>(rng.next() % 4);
                const int end = std::min(n - 1, k + run);
                const bool gap = rng.next() % 9 == 0;
                if (!gap) {
                    std::vector<std::int64_t> refs;
                    for (int m = k; m <= end; ++m) refs.push_back(dir == 0 ? id(line, m) : id(m, line));
                    std::vector<std::pair<std::string, std::string>> tags{{"highway", classes[rng.next() % 5]}};
                    const auto r = rng.next() % 10;
                    if (r == 0) tags.emplace_back("oneway", "yes");
                    if (r == 1) tags.emplace_back("oneway", "-1");
                    const auto s = rng.next() % 6;
                    if (s == 0) tags.emplace_back("maxspeed", "30");
                    if (s == 1) tags.emplace_back("maxspeed", "80");
                    for (const auto ref : refs) incident[static_cast<std::size_t>(ref - 1000)].push_back({way_id, ref});
                    b.way(way_id, refs, tags);
                }
                ++way_id;
                k = end;
            }
        }
    }
    scene.graph = b.build();
    const char* values[] = {"no_left_turn", "no_right_turn", "no_straight_on", "no_u_turn", "only_straight_on", "only_left_turn"};
    for (std::size_t v = 0; v < incident.size(); ++v) {
        if (incident[v].size() < 2 || rng.unit() >= restriction_share) continue;
        const auto& from = incident[v][rng.next() % incident[v].size()];
        const auto& to = incident[v][rng.next() % incident[v].size()];
        TurnRestriction r = via_node_rule(from.first, from.second, to.first, values[rng.next() % 6]);
        r.relation_id = static_cast<std::int64_t>(v);
        scene.rules.push_back(r);
    }
    for (const auto& node : scene.graph.nodes) scene.node_ids.push_back(node.osm_id);
    return scene;
}

} // namespace g2m_test
