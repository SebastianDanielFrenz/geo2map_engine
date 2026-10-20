// Edge markings along a merge lane or an exit (vault G2M-028): where a link lands on the added lane of a carriageway, the line between
// them is one broken line - the through carriageway's edge on the link's side is drawn broken (flag 128, or 64 on the left), the
// link's own facing edge line is dropped (flag 512, or 1024) - along the link's landing length from the node, and nowhere else.
//
// Recorded sabotages (2026-10-20; each makes the named tests fail, then reverted):
//   S1 road_ribbons.cpp: the through carriageway's broken bit on the wrong side (left for a ramp on the right)           -> MK1, MK2 fail
//   S2 road_ribbons.cpp: the through zone covers the whole stretch instead of the landing length                         -> MK2 fails (8919 assertions)
//   S3 road_ribbons.cpp: the through carriageway is not marked (skip the LaneChange branch)                              -> MK1, MK2 fail
//   S4 road_ribbons.cpp: the ramp's facing edge is not dropped (skip the Landing branch)                                 -> MK1 fails
//   S5 road_junction.cpp build_from: no MergeLane decline at a landed ramp (2026-10-20, with MK4)                          -> MK4 fails
#include "road_fixture.h"

#include "g2m/layer/road_rules.h"
#include "g2m/world/road_ribbons.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

namespace {
using namespace g2m;
using Tags = std::vector<std::pair<std::string, std::string>>;

const TileKey kKey{geo::UtmZone{32, geo::Hemisphere::North}, 2, 455, 5422};
constexpr double kE0 = 455.0 * 1024.0 + 512.0, kN0 = 5422.0 * 1024.0 + 512.0;   // the tile's middle: node 2 sits here
constexpr int kOverlay = 32, kRightBroken = 128, kLeftBroken = 64, kLeftHidden = 512, kRightHidden = 1024;

RoadGraphWay way(std::int64_t id, std::vector<std::int64_t> nodes, Tags tags) {
    std::sort(tags.begin(), tags.end());
    return RoadGraphWay{id, 3, std::move(nodes), std::move(tags)};
}
// The site-4 shape on +x (cf. test_road_merge_lane.cpp): A (2 lanes) and the link R meet node 2 and B (3 lanes) leaves; `ramp_y` is the
// link's start, south of the axis (right of the travel direction) for a negative value.
RoadGraph scene(const std::string& ramp_highway, int ramp_y) {
    RoadGraph g;
    const auto node = [&](std::int64_t id, double x, double y) { g.nodes.push_back({id, {std::llround((kE0 + x) * 1000.0), std::llround((kN0 + y) * 1000.0)}, {}}); };
    node(1, -400, 0); node(2, 0, 0); node(3, 200, 0); node(4, -150, ramp_y);
    g.ways.push_back(way(10, {1, 2}, {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}}));
    g.ways.push_back(way(11, {2, 3}, {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "3"}, {"placement", "right_of:1"}, {"turn:lanes", "none|none|merge_to_left"}}));
    g.ways.push_back(way(12, {4, 2}, {{"highway", ramp_highway}, {"oneway", "yes"}, {"lanes", "1"}}));
    for (const auto& w : g.ways) for (std::size_t i = 1; i < w.node_ids.size(); ++i)
        g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    return g;
}
struct Marked { std::vector<world::RoadRibbonVertex> right_broken, left_hidden, others; double ox = 0, oy = 0; };
Marked marked(const RoadGraph& graph, const RoadRules& rules) {
    g2m_test::GeomOptions o;
    o.height_m = [](double, double) { return 100.0; };
    o.completion = served_road_completion_options();
    o.rules = &rules;
    auto tile = g2m_test::geometry_in_tile_with(graph, kKey, o);
    if (!tile) throw std::runtime_error(tile.error().message);
    world::RoadRibbonOptions options;
    options.drape = world::RibbonDrape::PointSampled;
    const auto ribbon = world::build_road_ribbons(*tile, kKey, [](double, double) -> std::optional<double> { return 100.0; }, 0.018, {}, options);
    Marked out;
    out.ox = static_cast<double>(kKey.min_easting()); out.oy = static_cast<double>(kKey.min_northing());
    for (const auto& v : ribbon.vertices) {
        const int f = static_cast<int>(v.flags);
        if (f & kOverlay) continue;   // junction overlays carry their own broken/hidden bits
        if (f & kRightBroken) out.right_broken.push_back(v);
        if (f & kLeftHidden) out.left_hidden.push_back(v);
        if (f & (kLeftBroken | kRightHidden)) out.others.push_back(v);
    }
    return out;
}
double node_x() { return kE0 - 455.0 * 1024.0; }   // node 2's easting inside the tile
double node_y() { return kN0 - 5422.0 * 1024.0; }
} // namespace

TEST_CASE("MK1 a merge lane: the through edge on the ramp's side is broken and the ramp's facing edge is dropped", "[world][road_ribbons][merge_markings]") {
    const auto m = marked(scene("trunk_link", -12), builtin_road_rules_de());
    REQUIRE_FALSE(m.right_broken.empty());
    REQUIRE_FALSE(m.left_hidden.empty());
    // The through carriageway A runs on the axis (y = node_y) west of the node; the ramp lies south of it.
    for (const auto& v : m.right_broken) CHECK(std::abs(v.y - node_y()) < 5.0);
    for (const auto& v : m.left_hidden) CHECK(v.y < node_y() + 1.0);
    // Nothing is marked on the wrong side: the ramp is on the right, so neither a broken left edge nor a hidden right edge.
    CHECK(m.others.empty());
}

TEST_CASE("MK2 the zone is the landing length from the node, and only there", "[world][road_ribbons][merge_markings]") {
    const auto& rules = builtin_road_rules_de();
    const auto m = marked(scene("trunk_link", -12), rules);
    REQUIRE_FALSE(m.right_broken.empty());
    // The landing offset is about 5.5 m: the length is 12 m per metre = 65 m, between the 30 m and 80 m bounds.
    const double lo = node_x() - rules.merge_lane.landing_max_m - 2.0, hi = node_x() + 0.5;
    double first = 1e9, last = -1e9;
    for (const auto& v : m.right_broken) { CHECK(v.x >= lo); CHECK(v.x <= hi); first = std::min<double>(first, v.x); last = std::max<double>(last, v.x); }
    // It starts before the node by about the landing length, and reaches the node.
    CHECK(first < node_x() - 55.0); CHECK(first > node_x() - 80.0);
    CHECK(last > node_x() - 5.0);
    // Every 10 m of the zone carries a broken quad.
    for (double x = node_x() - 60.0; x < node_x() - 10.0; x += 10.0) {
        bool any = false;
        for (const auto& v : m.right_broken) any = any || std::abs(v.x - x) < 2.0;
        INFO("x " << x - node_x());
        CHECK(any);
    }
}

TEST_CASE("MK3 no landing, no marks: a link that is no merge lane and the switched-off rules", "[world][road_ribbons][merge_markings]") {
    const auto none = [](const Marked& m) { CHECK(m.right_broken.empty()); CHECK(m.left_hidden.empty()); CHECK(m.others.empty()); };
    none(marked(scene("tertiary", -12), builtin_road_rules_de()));
    auto off = builtin_road_rules_de();
    off.merge_lane.enabled = false;
    none(marked(scene("trunk_link", -12), off));
}

TEST_CASE("MK4 a node where a merge ramp lands has no junction patch (decline MergeLane); without the rules it has one", "[world][road_ribbons][merge_markings]") {
    const auto declines = [](const RoadRules& rules) {
        g2m_test::GeomOptions o;
        o.height_m = [](double, double) { return 100.0; };
        o.completion = served_road_completion_options();
        o.rules = &rules;
        auto tile = g2m_test::geometry_in_tile_with(scene("trunk_link", -12), kKey, o);
        if (!tile) throw std::runtime_error(tile.error().message);
        std::vector<JunctionDecline> out;
        for (const auto& j : tile->junctions) out.push_back(j.decline);
        return out;
    };
    const auto on = declines(builtin_road_rules_de());
    REQUIRE(on.size() == 1);
    CHECK(on[0] == JunctionDecline::MergeLane);
    auto off_rules = builtin_road_rules_de();
    off_rules.merge_lane.enabled = false;
    const auto off = declines(off_rules);
    REQUIRE(off.size() == 1);
    CHECK(off[0] != JunctionDecline::MergeLane);
}
