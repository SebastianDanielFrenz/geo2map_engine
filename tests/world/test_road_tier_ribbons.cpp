// Road ribbon vertices carry the road clearing tier (RoadRibbonVertex::road_tier, road_tier.h): the ribbon of a stretch the tier
// of its way, a junction patch the most important (lowest) tier of the roads meeting there, 255 when the caller gave no tier map.
//
// Recorded sabotages (2026-11-02; each makes the named tests fail, then reverted):
//   S5 road_ribbons.cpp lane_layout: l.tier = 0 for every way                               -> RTR1, RTR2 fail
//   S6 road_ribbons.cpp: TierScope of the main stretch loop removed                         -> RTR1, RTR2 fail
//   S7 road_ribbons.cpp add_junction_surfaces: patch tier = the LAST arm's tier             -> RTR2 fails (needs both arm orders)
#include "road_fixture.h"

#include "g2m/layer/road_tier.h"
#include "g2m/world/road_ribbons.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <set>
#include <string>
#include <vector>

namespace {
using namespace g2m;
using Tags = std::vector<std::pair<std::string, std::string>>;

const TileKey kKey{geo::UtmZone{32, geo::Hemisphere::North}, 2, 455, 5422};
constexpr double kE0 = 455.0 * 1024.0 + 512.0, kN0 = 5422.0 * 1024.0 + 512.0;   // the tile's middle

RoadGraphWay way(std::int64_t id, std::vector<std::int64_t> nodes, Tags tags) {
    std::sort(tags.begin(), tags.end());
    return RoadGraphWay{id, 3, std::move(nodes), std::move(tags)};
}
void finish(RoadGraph& g) {
    for (const auto& w : g.ways) for (std::size_t i = 1; i < w.node_ids.size(); ++i)
        g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
}
void add_node(RoadGraph& g, std::int64_t id, double x, double y) {
    g.nodes.push_back({id, {std::llround((kE0 + x) * 1000.0), std::llround((kN0 + y) * 1000.0)}, {}});
}

// Two roads far apart: a trunk with motorroad=yes (tier 0) at the tile's middle and a residential road 80 m north (tier 5).
RoadGraph parallel_roads() {
    RoadGraph g;
    add_node(g, 1, -150, 0); add_node(g, 2, 150, 0); add_node(g, 3, -150, 80); add_node(g, 4, 150, 80);
    g.ways.push_back(way(10, {1, 2}, {{"highway", "trunk"}, {"motorroad", "yes"}, {"lanes", "2"}}));
    g.ways.push_back(way(11, {3, 4}, {{"highway", "residential"}}));
    finish(g);
    return g;
}
// A primary road (tier 2) along x with a residential road (tier 5) joining it from the south at the node in the middle.
// The primary road is two ways (ids 10 and 20) so the junction has three arms; `side_id` numbers the residential way, which
// puts its arm first (5), in the middle (15) or last (25) in the junction's arm list.
RoadGraph t_junction(std::int64_t side_id, double side_y = -150) {
    RoadGraph g;
    add_node(g, 1, -150, 0); add_node(g, 2, 0, 0); add_node(g, 3, 150, 0); add_node(g, 4, 0, side_y);
    g.ways.push_back(way(10, {1, 2}, {{"highway", "primary"}}));
    g.ways.push_back(way(20, {2, 3}, {{"highway", "primary"}}));
    g.ways.push_back(way(side_id, {4, 2}, {{"highway", "residential"}}));
    std::sort(g.ways.begin(), g.ways.end(), [](const RoadGraphWay& x, const RoadGraphWay& y) { return x.osm_id < y.osm_id; });
    finish(g);
    return g;
}

world::RoadRibbonTile ribbons(const RoadGraph& graph, const RoadTierMap* tiers) {
    g2m_test::GeomOptions o;
    o.height_m = [](double, double) { return 100.0; };
    auto tile = g2m_test::geometry_in_tile_with(graph, kKey, o);
    if (!tile) throw std::runtime_error(tile.error().message);
    world::RoadRibbonOptions options;
    options.drape = world::RibbonDrape::PointSampled;
    options.road_tiers = tiers;
    return world::build_road_ribbons(*tile, kKey, [](double, double) -> std::optional<double> { return 100.0; }, 0.018, {}, options);
}

RoadTierMap tier_map() {
    RoadTierMap m;
    RoadTierRule motorroad; motorroad.tier = 0; motorroad.require_motorroad_yes = true;
    RoadTierRule trunk; trunk.tier = 1; trunk.highways = {"trunk"}; trunk.exclude_motorroad_yes = true;
    RoadTierRule primary; primary.tier = 2; primary.highways = {"primary"};
    m.rules = {motorroad, trunk, primary};
    m.default_tier = 5;
    return m;
}
double mid_y() { return kN0 - 5422.0 * 1024.0; }
double mid_x() { return kE0 - 455.0 * 1024.0; }
} // namespace

TEST_CASE("RTR1 every ribbon vertex carries the tier of its road", "[world][road_ribbons][road_tier]") {
    const RoadGraph graph = parallel_roads();
    const RoadTierMap map = tier_map();
    // The tier a segment of each way gets: the same rule table through the same tag reader the ribbons use.
    const int trunk_tier = road_tier_of(map, graph.ways[0], LandClass::PavedRoad);
    const int residential_tier = road_tier_of(map, graph.ways[1], LandClass::PavedRoad);
    REQUIRE(trunk_tier == 0);
    REQUIRE(residential_tier == 5);

    const auto mesh = ribbons(graph, &map);
    REQUIRE_FALSE(mesh.vertices.empty());
    std::set<float> tiers;
    std::size_t on_trunk = 0, on_residential = 0;
    for (const auto& v : mesh.vertices) {
        tiers.insert(v.road_tier);
        if (std::abs(v.y - mid_y()) < 15.0) { CHECK(v.road_tier == static_cast<float>(trunk_tier)); ++on_trunk; }
        else if (std::abs(v.y - (mid_y() + 80.0)) < 15.0) { CHECK(v.road_tier == static_cast<float>(residential_tier)); ++on_residential; }
        else FAIL("a vertex on neither road: y " << v.y);
    }
    CHECK(on_trunk > 0);
    CHECK(on_residential > 0);
    CHECK(tiers == std::set<float>{0.0f, 5.0f});

    // No map: the same geometry, every vertex 255 (no tier), the flag bits untouched.
    const auto plain = ribbons(graph, nullptr);
    REQUIRE(plain.vertices.size() == mesh.vertices.size());
    REQUIRE(plain.indices == mesh.indices);
    for (std::size_t i = 0; i < plain.vertices.size(); ++i) {
        CHECK(plain.vertices[i].road_tier == 255.0f);
        CHECK(plain.vertices[i].flags == mesh.vertices[i].flags);
        CHECK(plain.vertices[i].x == mesh.vertices[i].x);
        CHECK(plain.vertices[i].z == mesh.vertices[i].z);
    }
    // The motorroad trunk is drawn as a fast road (bit 8), the residential road is not: the tier does not replace the flag.
    bool fast_on_trunk = false, fast_on_residential = false;
    for (const auto& v : mesh.vertices) {
        const bool fast = (static_cast<int>(v.flags) & 8) != 0;
        if (v.road_tier == 0.0f) fast_on_trunk = fast_on_trunk || fast;
        else fast_on_residential = fast_on_residential || fast;
    }
    CHECK(fast_on_trunk);
    CHECK_FALSE(fast_on_residential);
}

TEST_CASE("RTR2 a junction patch carries the most important tier of the roads meeting there", "[world][road_ribbons][road_tier]") {
  for (const auto [side_id, side_y] : {std::pair<std::int64_t, double>{5, -150}, {25, -150}, {5, 150}, {25, 150}}) {
    INFO("residential way id " << side_id << ", from y " << side_y);
    const RoadGraph graph = t_junction(side_id, side_y);
    const RoadTierMap map = tier_map();
    for (const auto& w : graph.ways) REQUIRE(road_tier_of(map, w, LandClass::PavedRoad) == (w.osm_id == side_id ? 5 : 2));
    const auto mesh = ribbons(graph, &map);
    REQUIRE_FALSE(mesh.vertices.empty());
    std::size_t side_only = 0, near_node = 0, patch = 0;
    for (const auto& v : mesh.vertices) if (v.width == 1000.0f && v.fade == 0.0f && v.lanes == 0.0f) { ++patch; CHECK(v.road_tier == 2.0f); }
    CHECK(patch > 0);   // the junction patch exists, and it is the primary road's tier whichever arm comes last
    for (const auto& v : mesh.vertices) {
        CHECK(v.road_tier != 255.0f);                                     // nothing is left without a tier
        const double dx = v.x - mid_x(), dy = v.y - mid_y();
        if (std::abs(dx) < 4.0 && std::abs(dy) > 40.0) { CHECK(v.road_tier == 5.0f); ++side_only; }     // the residential road alone
        if (std::hypot(dx, dy) < 3.0) { CHECK(v.road_tier == 2.0f); ++near_node; }              // the patch: the primary road's tier wins
        if (std::abs(dy) < 3.0 && std::abs(dx) > 40.0) CHECK(v.road_tier == 2.0f);              // the primary road alone
    }
    CHECK(side_only > 0);
    CHECK(near_node > 0);
  }
}
