// tests/support/underpass_scene.h - underpass decks (decisions D53-D58): a synthetic grade-separated crossing without
// a bridge tag. Upper way 100 (secondary) runs east-west through the centre of kUnderpassTile, lower way 200
// (residential) north-south under it in a DEM cutting, no shared node. Shared by the layer and world tests. Header-only.
#pragma once

#include "g2m/core/tile_key.h"
#include "g2m/layer/road_graph.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace g2m_test {

inline const g2m::TileKey kUnderpassTile{g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North}, 2, 455, 5421};
inline constexpr double kUnderpassE0 = 455.0 * 1024.0 + 512.0, kUnderpassN0 = 5421.0 * 1024.0 + 512.0;

using UnderpassTags = std::vector<std::pair<std::string, std::string>>;
struct UnderpassSceneOptions {
    UnderpassTags lower_tags = {{"layer", "-1"}};
    UnderpassTags upper_tags = {};
    double lower_x = 0.5;                 // the lower road's easting relative to kUnderpassE0
    double upper_from = -400, upper_to = 400;
    std::vector<double> upper_extra = {}; // extra upper nodes (x), e.g. a junction node near the crossing
    double side_road_x = std::nan("");    // a side road north from the upper node at this x (splits the upper stretch)
};
// Upper nodes 1.. every 100 m from upper_from to upper_to (plus upper_extra), lower nodes 1000.. at lower_x from y =
// -350 every 100 m, side road nodes 2000...
inline g2m::RoadGraph underpass_scene(const UnderpassSceneOptions& o = {}) {
    using namespace g2m;
    RoadGraph g;
    const auto node = [&](std::int64_t id, double x, double y) {
        g.nodes.push_back({id, {std::llround((kUnderpassE0 + x) * 1000.0), std::llround((kUnderpassN0 + y) * 1000.0)}, {}});
    };
    std::vector<double> xs;
    for (double x = o.upper_from; x <= o.upper_to + 1e-9; x += 100) xs.push_back(x);
    xs.insert(xs.end(), o.upper_extra.begin(), o.upper_extra.end());
    if (!std::isnan(o.side_road_x)) xs.push_back(o.side_road_x);
    std::sort(xs.begin(), xs.end()); xs.erase(std::unique(xs.begin(), xs.end()), xs.end());
    std::vector<std::int64_t> un, ln, sn;
    for (std::size_t i = 0; i < xs.size(); ++i) { node(1 + static_cast<std::int64_t>(i), xs[i], 0.0); un.push_back(1 + static_cast<std::int64_t>(i)); }
    for (int i = 0; i <= 8; ++i) { node(1000 + i, o.lower_x, -350.0 + 100.0 * i); ln.push_back(1000 + i); }
    auto upper = UnderpassTags{{"highway", "secondary"}};
    upper.insert(upper.end(), o.upper_tags.begin(), o.upper_tags.end());
    std::sort(upper.begin(), upper.end());
    auto lower = UnderpassTags{{"highway", "residential"}};
    lower.insert(lower.end(), o.lower_tags.begin(), o.lower_tags.end());
    std::sort(lower.begin(), lower.end());
    g.ways.push_back({100, 6, un, upper});
    g.ways.push_back({200, 3, ln, lower});
    if (!std::isnan(o.side_road_x)) {
        const auto at = std::find(xs.begin(), xs.end(), o.side_road_x);
        sn.push_back(1 + (at - xs.begin()));
        for (int i = 1; i <= 3; ++i) { node(2000 + i, o.side_road_x, 100.0 * i); sn.push_back(2000 + i); }
        g.ways.push_back({300, 3, sn, UnderpassTags{{"highway", "residential"}}});
    }
    std::sort(g.nodes.begin(), g.nodes.end(), [](const RoadGraphNode& a, const RoadGraphNode& b) { return a.osm_id < b.osm_id; });
    for (const auto& w : g.ways)
        for (std::size_t i = 1; i < w.node_ids.size(); ++i)
            g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    std::sort(g.edges.begin(), g.edges.end(), [](const RoadGraphEdge& a, const RoadGraphEdge& b) {
        return std::tie(a.way_id, a.segment_index) < std::tie(b.way_id, b.segment_index);
    });
    return g;
}
// Bare earth of an underpass: `rim` m, with a cutting `depth` m deep and 16 m wide (smooth 4 m shoulders) along the
// lower road at easting kUnderpassE0 + lower_x.
inline double underpass_cutting(double e, double lower_x = 0.5, double depth = 6.0, double rim = 100.0) {
    const double d = std::fabs(e - (kUnderpassE0 + lower_x));
    if (d <= 8) return rim - depth;
    if (d >= 12) return rim;
    const double t = (d - 8) / 4;
    return rim - depth + depth * t * t * (3 - 2 * t);
}

} // namespace g2m_test
