// Street fidelity P2 slice 7: junction furniture (design section 7, tests T9-T11): crossings, stop and give-way lines,
// signals; synthetic scenes for every rule, the acceptance rules, and the two committed OSM fixtures with the tile-invariance
// check and its negative control.
#include "road_fixture.h"
#include "g2m/layer/road_furniture.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <functional>
#include <map>
#include <memory>
#include <set>
#include <string>
using namespace g2m;
using Catch::Approx;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};
const TileKey kTile{zone, 2, 455, 5421};
constexpr double kOriginE = 455.0 * 1024.0 + 512.0, kOriginN = 5421.0 * 1024.0 + 512.0;   // tile centre, metres
using Tags = std::vector<std::pair<std::string, std::string>>;
using Fn = std::function<double(double, double)>;
struct N { std::int64_t id; double x, y; Tags tags = {}; };
struct W { std::int64_t id; std::vector<std::int64_t> nodes; Tags tags = {{"highway", "residential"}}; };

int rank_of(const Tags& tags) {   // osm_roads.cpp kDefaultHighways
    for (const auto& [k, v] : tags) if (k == "highway") return v == "residential" ? 3 : v == "tertiary" ? 5 : v == "secondary" ? 6 : v == "primary" ? 7 : v == "motorway" ? 9 : 3;
    return 3;
}
struct Scene {
    RoadGraph graph;
    RoadGeomTile tile;
    std::optional<RoadReferenceBuilder> builder;
    std::vector<RoadJunction> junctions;
};
std::unique_ptr<Scene> make_scene(const std::vector<N>& nodes, std::vector<W> ways, const Fn& dem = {}) {
    auto scene = std::make_unique<Scene>();
    for (const auto& n : nodes) {
        Tags tags = n.tags; std::sort(tags.begin(), tags.end());
        scene->graph.nodes.push_back({n.id, {std::llround((kOriginE + n.x) * 1000.0), std::llround((kOriginN + n.y) * 1000.0)}, tags});
    }
    std::sort(scene->graph.nodes.begin(), scene->graph.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    std::sort(ways.begin(), ways.end(), [](const auto& a, const auto& b) { return a.id < b.id; });
    for (auto& w : ways) {
        Tags tags = w.tags; std::sort(tags.begin(), tags.end());
        scene->graph.ways.push_back({w.id, rank_of(w.tags), w.nodes, tags});
        for (std::size_t i = 1; i < w.nodes.size(); ++i) scene->graph.edges.push_back({w.id, static_cast<std::uint32_t>(i - 1), w.nodes[i - 1], w.nodes[i]});
    }
    auto tile = g2m_test::geometry_in_tile(scene->graph, kTile, true, dem);
    REQUIRE(tile);
    scene->tile = std::move(*tile);
    auto builder = RoadReferenceBuilder::make(scene->graph);
    REQUIRE(builder);
    scene->builder = std::move(*builder);
    auto js = build_road_junctions(scene->tile, *scene->builder);
    REQUIRE(js);
    scene->junctions = std::move(*js);
    return scene;
}
std::vector<RoadFurniture> furniture_of(const Scene& s, FurnitureBuildStats* stats = nullptr) {
    auto f = build_road_furniture(s.tile, *s.builder, s.junctions, builtin_road_rules_de(), stats);
    REQUIRE(f);
    return std::move(*f);
}
std::vector<RoadFurniture> pick(const std::vector<RoadFurniture>& all, FurnitureKind kind, std::int64_t node = 0) {
    std::vector<RoadFurniture> out;
    for (const auto& f : all) if (f.kind == kind && (node == 0 || f.node_id == node)) out.push_back(f);
    return out;
}
std::size_t entry_of(const Scene& s, std::int64_t way, std::uint32_t start = 0) {
    for (std::size_t i = 0; i < s.tile.entries.size(); ++i) if (s.tile.entries[i].way_id == way && s.tile.entries[i].stretch.start_ref == start) return i;
    FAIL("no such entry"); return 0;
}
// A straight east-west road of 400 m with nodes every 100 m: way 10 {1..5}, node k at x = -200 + 100 (k-1).
std::vector<N> line_nodes(const std::vector<std::pair<int, Tags>>& tagged = {}) {
    std::vector<N> nodes;
    for (int k = 1; k <= 5; ++k) nodes.push_back({k, -200.0 + 100.0 * (k - 1), 0});
    for (const auto& [k, tags] : tagged) nodes[static_cast<std::size_t>(k - 1)].tags = tags;
    return nodes;
}
const std::vector<std::int64_t> kLine = {1, 2, 3, 4, 5};

// The lane span of one direction at a station, the acceptance rule of 7.2 recomputed from the entry.
struct Span { double lo = 0, hi = 0; std::size_t count = 0; };
Span lane_span(const Scene& s, std::size_t entry, double station, LaneDirection travel) {
    std::array<LaneQuery, 32> buf{};
    const auto n = driving_lanes_at(*s.tile.entries[entry].cross_section, station, travel, buf);
    Span span; span.count = n;
    span.lo = 1e9; span.hi = -1e9;
    for (std::size_t i = 0; i < n; ++i) { span.lo = std::min(span.lo, buf[i].centre_t - buf[i].width_m / 2); span.hi = std::max(span.hi, buf[i].centre_t + buf[i].width_m / 2); }
    return span;
}
}

// ---- T9: crossings ----

TEST_CASE("T9: crossings take their pattern, length, flags and entry from the node, and the quad lies inside the carriageway", "[layer][roads][furniture][T9]") {
    const auto scene = make_scene(line_nodes({
            {1, {{"highway", "crossing"}, {"crossing", "unmarked"}}},                                                   // dead end
            {2, {{"highway", "crossing"}, {"crossing", "traffic_signals"}, {"kerb", "lowered"}}},
            {3, {{"highway", "crossing"}, {"crossing", "zebra"}}},
            {4, {{"highway", "crossing"}, {"crossing", "marked"}, {"crossing:markings", "ladder"}, {"width", "2.5"}}}}),
        {{10, kLine}});
    FurnitureBuildStats stats;
    const auto all = furniture_of(*scene, &stats);
    const auto crossings = pick(all, FurnitureKind::Crossing);
    REQUIRE(crossings.size() == 4);
    CHECK(stats.crossing_nodes == 4); CHECK(stats.crossing_placed == 4); CHECK(stats.crossing_not_ours == 0);
    const auto entry = entry_of(*scene, 10);
    const double length = scene->tile.entries[entry].profile->reference.length_m;
    const auto at = [&](std::int64_t node) { return pick(all, FurnitureKind::Crossing, node).at(0); };
    const auto dead = at(1), signalised = at(2), zebra = at(3), ladder = at(4);
    CHECK(dead.pattern == CrossingPattern::None); CHECK(dead.length_m == 3.0); CHECK(dead.station_m == 0.0); CHECK(dead.flags == 0);
    CHECK(signalised.pattern == CrossingPattern::Dashes); CHECK(signalised.length_m == 4.0);
    CHECK(signalised.flags == (kFurnitureSignalised | kFurnitureLoweredKerb));
    CHECK(zebra.pattern == CrossingPattern::Zebra); CHECK(zebra.length_m == 4.0); CHECK(zebra.station_m == Approx(length / 2).margin(1e-6));
    CHECK(ladder.pattern == CrossingPattern::Ladder); CHECK(ladder.length_m == 2.5);   // the width tag
    for (const auto& c : crossings) { CHECK(c.entry_index == entry); CHECK(c.decline == FurnitureDecline::None); CHECK(c.t_from < c.t_to); }
    // The quad: on this east-going road y is the lateral offset, and every corner lies inside the carriageway (+ 0.5 m).
    const auto quad = furniture_quad(scene->tile, zebra); REQUIRE(quad);
    const double x0 = kOriginE - 200.0, y0 = kOriginN;
    CHECK((*quad)[0].x == Approx(x0 + zebra.station_m - 2.0).margin(1e-6)); CHECK((*quad)[1].x == Approx(x0 + zebra.station_m + 2.0).margin(1e-6));
    for (const auto& c : crossings) {
        const auto q = furniture_quad(scene->tile, c); REQUIRE(q);
        for (const auto& corner : *q) {
            const double s = std::clamp(c.station_m, 0.0, length);
            const auto edges = carriageway_edges(*scene->tile.entries[entry].cross_section, s);
            CHECK(corner.y - y0 <= edges.left + 0.5); CHECK(corner.y - y0 >= edges.right - 0.5);
        }
    }
    // The dead end quad extends past the road end (flag-free), the end of the road at station 0 is x = -200.
    const auto dead_quad = furniture_quad(scene->tile, dead); REQUIRE(dead_quad);
    CHECK((*dead_quad)[0].x == Approx(kOriginE - 200 - 1.5).margin(1e-6));
    // The T9 acceptance sabotage: one record deleted gives a count mismatch with the nodes on entries.
    auto fewer = crossings; fewer.pop_back();
    CHECK(fewer.size() != stats.crossing_nodes);
}

TEST_CASE("T9: crossings at a way join, a junction, a tunnel and a declined stretch", "[layer][roads][furniture][T9]") {
    {   // A degree-two join: the stretch that starts at the node owns it, the quad extends back over the join.
        const auto scene = make_scene({{1, -200, 0}, {2, -100, 0}, {3, 0, 0, {{"highway", "crossing"}, {"crossing", "zebra"}}}, {4, 100, 0}, {5, 200, 0}},
            {{10, {1, 2, 3}}, {11, {3, 4, 5}}});
        const auto c = pick(furniture_of(*scene), FurnitureKind::Crossing);
        REQUIRE(c.size() == 1);
        CHECK(scene->tile.entries[c[0].entry_index].way_id == 11); CHECK(c[0].station_m == 0.0);
        CHECK(c[0].flags == kFurnitureSpansJoin);
        const auto q = furniture_quad(scene->tile, c[0]); REQUIRE(q);
        CHECK((*q)[0].x == Approx(kOriginE - 2.0).margin(1e-6)); CHECK((*q)[1].x == Approx(kOriginE + 2.0).margin(1e-6));
    }
    {   // Two ways that both END at the node: the smallest way id owns it, at its end station.
        const auto scene = make_scene({{1, -200, 0}, {2, -100, 0}, {3, 0, 0, {{"highway", "crossing"}}}, {4, 100, 0}, {5, 200, 0}},
            {{10, {1, 2, 3}}, {11, {5, 4, 3}}});
        const auto c = pick(furniture_of(*scene), FurnitureKind::Crossing);
        REQUIRE(c.size() == 1);
        CHECK(scene->tile.entries[c[0].entry_index].way_id == 10);
        CHECK(c[0].station_m == Approx(scene->tile.entries[c[0].entry_index].profile->reference.length_m).margin(1e-9));
    }
    {   // A crossing on a junction node is declined (the direction is unknown) and attached to one incident stretch end.
        const auto scene = make_scene({{1, 0, 0, {{"highway", "crossing"}}}, {2, -200, 0}, {3, 200, 0}, {4, 0, -200}}, {{10, {2, 1, 3}}, {11, {1, 4}}});
        FurnitureBuildStats stats;
        const auto c = pick(furniture_of(*scene, &stats), FurnitureKind::Crossing);
        REQUIRE(c.size() == 1);
        CHECK(c[0].decline == FurnitureDecline::JunctionNode); CHECK(c[0].node_id == 1);
        CHECK(scene->tile.entries[c[0].entry_index].way_id == 10); CHECK(scene->tile.entries[c[0].entry_index].stretch.start_ref == 1);   // smallest (way, ref, at_end): the stretch that starts there
        CHECK(stats.crossing_declined[static_cast<std::size_t>(FurnitureDecline::JunctionNode)] == 1);
    }
    {   // A tunnel.
        const auto scene = make_scene(line_nodes({{3, {{"highway", "crossing"}}}}), {{10, kLine, {{"highway", "residential"}, {"tunnel", "yes"}}}});
        const auto c = pick(furniture_of(*scene), FurnitureKind::Crossing);
        REQUIRE(c.size() == 1); CHECK(c[0].decline == FurnitureDecline::Tunnel);
    }
    {   // A stretch the profile fit declined (an 11 % motorway, class bound 6 %).
        std::vector<N> nodes = {{1, -150, 0}, {2, -50, 0, {{"highway", "crossing"}}}, {3, 50, 0}, {4, 150, 0}};
        const auto scene = make_scene(nodes, {{10, {1, 2, 3, 4}, {{"highway", "motorway"}}}}, [](double e, double) { return 100.0 + 0.11 * (e - kOriginE); });
        REQUIRE_FALSE(scene->tile.entries[0].profile);
        const auto c = pick(furniture_of(*scene), FurnitureKind::Crossing);
        REQUIRE(c.size() == 1); CHECK(c[0].decline == FurnitureDecline::StretchDeclined);
    }
    {   // Crossing markings beat the crossing kind; markings=no paints nothing; crossing=no too.
        const auto scene = make_scene(line_nodes({{2, {{"highway", "crossing"}, {"crossing", "uncontrolled"}, {"crossing:markings", "no"}}},
                                                   {3, {{"highway", "crossing"}, {"crossing", "no"}}},
                                                   {4, {{"highway", "crossing"}, {"crossing", "marked"}, {"crossing:markings", "yes"}}}}), {{10, kLine}});
        const auto all = furniture_of(*scene);
        CHECK(pick(all, FurnitureKind::Crossing, 2)[0].pattern == CrossingPattern::None);
        CHECK(pick(all, FurnitureKind::Crossing, 3)[0].pattern == CrossingPattern::None);
        CHECK(pick(all, FurnitureKind::Crossing, 4)[0].pattern == CrossingPattern::Zebra);   // yes: falls through to the kind
    }
}

// ---- T10: stop lines ----

TEST_CASE("T10: a signal inside a two-way stretch gets one stop line per direction, spanning exactly its own lanes", "[layer][roads][furniture][T10]") {
    const auto scene = make_scene(line_nodes({{3, {{"highway", "traffic_signals"}}}}), {{10, kLine}});
    FurnitureBuildStats stats;
    const auto all = furniture_of(*scene, &stats);
    const auto lines = pick(all, FurnitureKind::StopLine);
    REQUIRE(lines.size() == 2);
    const auto entry = entry_of(*scene, 10);
    for (const auto& l : lines) {
        const auto span = lane_span(*scene, entry, l.station_m, l.travel);
        CHECK(l.decline == FurnitureDecline::None); CHECK(l.length_m == 0.5);
        CHECK(l.t_from == Approx(span.lo).margin(0.0006)); CHECK(l.t_to == Approx(span.hi).margin(0.0006));
        // Sabotage: the whole carriageway instead of the direction's lanes covers both lanes of a two-way road.
        const auto edges = carriageway_edges(*scene->tile.entries[entry].cross_section, l.station_m);
        CHECK(std::abs((edges.left - edges.right) - (l.t_to - l.t_from)) > 1.0);
    }
    CHECK(lines[0].travel != lines[1].travel);
    CHECK(stats.signal_nodes == 1); CHECK(stats.line_sets == 2);
}

TEST_CASE("T10: a line covers a turn pocket lane", "[layer][roads][furniture][T10]") {
    // Two forward lanes (a left-turn pocket and a through lane), one backward lane.
    const auto scene = make_scene(line_nodes({{3, {{"highway", "traffic_signals"}}}}),
        {{10, kLine, {{"highway", "residential"}, {"lanes", "3"}, {"lanes:forward", "2"}, {"lanes:backward", "1"}, {"turn:lanes:forward", "left|through"}}}});
    const auto lines = pick(furniture_of(*scene), FurnitureKind::StopLine);
    REQUIRE(lines.size() == 2);
    const auto entry = entry_of(*scene, 10);
    for (const auto& l : lines) {
        const auto span = lane_span(*scene, entry, l.station_m, l.travel);
        CHECK(span.count == (l.travel == LaneDirection::Forward ? 2u : 1u));
        CHECK(l.t_from == Approx(span.lo).margin(0.0006)); CHECK(l.t_to == Approx(span.hi).margin(0.0006));
    }
    const auto& forward = lines[0].travel == LaneDirection::Forward ? lines[0] : lines[1];
    const auto& backward = lines[0].travel == LaneDirection::Forward ? lines[1] : lines[0];
    CHECK((forward.t_to - forward.t_from) > 1.9 * (backward.t_to - backward.t_from));   // the pocket lane is inside the line
}

TEST_CASE("T10: direction of a node: tag, one-way, nearer junction end, default", "[layer][roads][furniture][T10]") {
    const auto count = [](const Scene& s, std::int64_t node, FurnitureKind kind = FurnitureKind::StopLine) { return pick(furniture_of(s), kind, node).size(); };
    const auto travel = [](const Scene& s, std::int64_t node) { return pick(furniture_of(s), FurnitureKind::StopLine, node).at(0).travel; };
    {   // direction=forward / backward (traffic_signals:direction wins over direction)
        const auto s = make_scene(line_nodes({{3, {{"highway", "traffic_signals"}, {"traffic_signals:direction", "forward"}, {"direction", "backward"}}}}), {{10, kLine}});
        CHECK(count(*s, 3) == 1); CHECK(travel(*s, 3) == LaneDirection::Forward);
        const auto b = make_scene(line_nodes({{3, {{"highway", "traffic_signals"}, {"direction", "backward"}}}}), {{10, kLine}});
        CHECK(count(*b, 3) == 1); CHECK(travel(*b, 3) == LaneDirection::Backward);
    }
    {   // a one-way takes the way's direction, oneway=-1 the opposite one
        const auto f = make_scene(line_nodes({{3, {{"highway", "traffic_signals"}}}}), {{10, kLine, {{"highway", "residential"}, {"oneway", "yes"}}}});
        CHECK(count(*f, 3) == 1); CHECK(travel(*f, 3) == LaneDirection::Forward);
        const auto r = make_scene(line_nodes({{3, {{"highway", "traffic_signals"}}}}), {{10, kLine, {{"highway", "residential"}, {"oneway", "-1"}}}});
        CHECK(count(*r, 3) == 1); CHECK(travel(*r, 3) == LaneDirection::Backward);
    }
    {   // near a junction end (within 60 m): towards that junction; far from both: both directions
        const auto s = make_scene({{1, -200, 0}, {2, -30, 0, {{"highway", "traffic_signals"}}}, {3, 0, 0}, {4, 200, 0}, {5, 0, 200}, {6, 100, 0, {{"highway", "traffic_signals"}}}},
            {{10, {1, 2, 3, 6, 4}}, {11, {3, 5}}});
        CHECK(count(*s, 2) == 1); CHECK(travel(*s, 2) == LaneDirection::Forward);      // 30 m before the junction at node 3
        CHECK(count(*s, 6) == 2);                                                      // 100 m beyond it: both directions
    }
    {   // a stop sign without a direction away from every junction cannot be resolved
        const auto s = make_scene(line_nodes({{3, {{"highway", "stop"}}}}), {{10, kLine}});
        FurnitureBuildStats stats;
        const auto all = furniture_of(*s, &stats);
        REQUIRE(all.size() == 1); CHECK(all[0].decline == FurnitureDecline::DirectionUnresolved);
        CHECK(stats.line_declined[static_cast<std::size_t>(FurnitureDecline::DirectionUnresolved)] == 1);
        // ... and with a direction tag it is a StopLine
        const auto t = make_scene(line_nodes({{3, {{"highway", "stop"}, {"direction", "forward"}}}}), {{10, kLine}});
        const auto lines = pick(furniture_of(*t), FurnitureKind::StopLine);
        REQUIRE(lines.size() == 1); CHECK(lines[0].length_m == 0.5);
        // give_way is a GiveWayLine
        const auto g = make_scene(line_nodes({{3, {{"highway", "give_way"}, {"direction", "backward"}}}}), {{10, kLine}});
        const auto gw = pick(furniture_of(*g), FurnitureKind::GiveWayLine);
        REQUIRE(gw.size() == 1); CHECK(gw[0].travel == LaneDirection::Backward);
    }
    {   // a direction with no lane (a signal on a one-way, direction=backward): NoLanes
        const auto s = make_scene(line_nodes({{3, {{"highway", "traffic_signals"}, {"direction", "backward"}}}}), {{10, kLine, {{"highway", "residential"}, {"oneway", "yes"}}}});
        const auto all = furniture_of(*s);
        REQUIRE(all.size() == 1); CHECK(all[0].decline == FurnitureDecline::NoLanes);
    }
}

TEST_CASE("T10: a line stops one metre before a crossing that starts within reach downstream", "[layer][roads][furniture][T10]") {
    // A signalised crossing node is also a signal: its stop lines go 1 m before the 4 m quad, upstream of it.
    const auto scene = make_scene(line_nodes({{3, {{"highway", "crossing"}, {"crossing", "traffic_signals"}}}}), {{10, kLine}});
    const auto all = furniture_of(*scene);
    const auto crossing = pick(all, FurnitureKind::Crossing, 3).at(0);
    const auto lines = pick(all, FurnitureKind::StopLine, 3);
    REQUIRE(lines.size() == 2);
    for (const auto& l : lines) {
        const double expect = l.travel == LaneDirection::Forward ? crossing.station_m - 2.0 - 1.0 : crossing.station_m + 2.0 + 1.0;
        CHECK(l.station_m == Approx(expect).margin(1e-9));
        // Sabotage: without the rule the line would sit at the node, inside the quad.
        CHECK(std::abs(l.station_m - crossing.station_m) > 2.0);
    }
    // A crossing further than 1 m + the line width downstream does not move the line.
    const auto far = make_scene({{1, -200, 0}, {2, -100, 0}, {3, -20, 0, {{"highway", "traffic_signals"}, {"direction", "forward"}}}, {4, 0, 0, {{"highway", "crossing"}}}, {5, 200, 0}}, {{10, {1, 2, 3, 4, 5}}});
    const auto far_all = furniture_of(*far);
    const auto line = pick(far_all, FurnitureKind::StopLine, 3).at(0);
    CHECK(line.station_m == Approx(180.0).margin(1e-6));
}

// ---- junction-derived lines and signals ----

namespace {
// An X with arms of 300 m: way 10 east-west {2,1,3}, way 11 north-south {4,1,5}.
std::unique_ptr<Scene> x_scene(const Tags& centre = {}, const std::vector<N>& extra = {}, std::vector<W> extra_ways = {}, const Tags& main_tags = {{"highway", "residential"}}) {
    std::vector<N> nodes = {{1, 0, 0, centre}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}, {5, 0, 300}};
    nodes.insert(nodes.end(), extra.begin(), extra.end());
    std::vector<W> ways = {{10, {2, 1, 3}, main_tags}, {11, {4, 1, 5}}};
    ways.insert(ways.end(), extra_ways.begin(), extra_ways.end());
    return make_scene(nodes, ways);
}
double trim_of(const Scene& s, std::size_t entry, bool at_end) {
    for (const auto& j : s.junctions) for (const auto& a : j.approaches) if (a.entry_index == entry && a.at_end == at_end) return a.trim_m;
    FAIL("no such approach");
}
}

TEST_CASE("T10/T11: a signalised junction gets a stop line and a signal per approach, one metre behind the trim line", "[layer][roads][furniture][T10][T11]") {
    const auto scene = x_scene({{"highway", "traffic_signals"}});
    REQUIRE(scene->junctions.size() == 1);
    FurnitureBuildStats stats;
    const auto all = furniture_of(*scene, &stats);
    const auto lines = pick(all, FurnitureKind::StopLine), signals = pick(all, FurnitureKind::Signal);
    REQUIRE(lines.size() == 4); REQUIRE(signals.size() == 4);
    CHECK(stats.junction_lines == 4); CHECK(stats.signals == 4); CHECK(stats.left_posts == 0);
    for (const auto& l : lines) {
        CHECK(l.junction_index == 1); CHECK(l.node_id == 1); CHECK(l.decline == FurnitureDecline::None);
        const auto& entry = scene->tile.entries[l.entry_index];
        const double length = entry.profile->reference.length_m;
        const bool at_end = l.travel == LaneDirection::Forward;
        const double sigma = at_end ? length - l.station_m : l.station_m;
        CHECK(sigma == Approx(trim_of(*scene, l.entry_index, at_end) + 1.0).margin(1e-9));
        const auto span = lane_span(*scene, l.entry_index, l.station_m, l.travel);
        CHECK(l.t_from == Approx(span.lo).margin(0.0006)); CHECK(l.t_to == Approx(span.hi).margin(0.0006));
    }
    // T11: post at the travel-right carriageway edge + 0.5 m, facing opposite to the travel direction.
    for (const auto& p : signals) {
        const auto& entry = scene->tile.entries[p.entry_index];
        const auto edges = carriageway_edges(*entry.cross_section, p.station_m);
        const bool forward = p.travel == LaneDirection::Forward;
        CHECK(p.t_from == Approx(forward ? edges.right - 0.5 : edges.left + 0.5).margin(0.0006)); CHECK(p.t_from == p.t_to);
        CHECK((p.flags & kFurnitureLeftPost) == 0);
        const auto post = signal_post(scene->tile, p); REQUIRE(post);
        // Travel direction at the station, from the reference line.
        const auto pose = entry.profile->reference.at(p.station_m); REQUIRE(pose);
        const double tx = (forward ? 1.0 : -1.0) * std::cos(pose->heading), ty = (forward ? 1.0 : -1.0) * std::sin(pose->heading);
        CHECK(post->face_x * tx + post->face_y * ty == Approx(-1.0).margin(1e-9));
        CHECK(post->face_x * post->face_x + post->face_y * post->face_y == Approx(1.0).margin(1e-12));
    }
    // A sabotaged offset (post on the carriageway edge itself) would fail the 0.5 m rule.
    for (const auto& p : signals) {
        const auto edges = carriageway_edges(*scene->tile.entries[p.entry_index].cross_section, p.station_m);
        const double edge = p.travel == LaneDirection::Forward ? edges.right : edges.left;
        CHECK(std::abs(p.t_from - edge) == Approx(0.5).margin(0.0006));
    }
}

TEST_CASE("T11: the left post exists only for three or more lanes of a one-way carriageway", "[layer][roads][furniture][T11]") {
    const auto make = [](const char* lanes) {
        return make_scene(line_nodes({{3, {{"highway", "traffic_signals"}}}}), {{10, kLine, {{"highway", "primary"}, {"oneway", "yes"}, {"lanes", lanes}}}});
    };
    {
        const auto s = make("3");
        const auto signals = pick(furniture_of(*s), FurnitureKind::Signal);
        REQUIRE(signals.size() == 2);
        const auto& right = (signals[0].flags & kFurnitureLeftPost) ? signals[1] : signals[0];
        const auto& left = (signals[0].flags & kFurnitureLeftPost) ? signals[0] : signals[1];
        const auto edges = carriageway_edges(*s->tile.entries[0].cross_section, right.station_m);
        CHECK(right.t_from == Approx(edges.right - 0.5).margin(0.0006)); CHECK(left.t_from == Approx(edges.left + 0.5).margin(0.0006));
        CHECK((left.flags & kFurnitureLeftPost) != 0);
    }
    {
        const auto s = make("2");
        CHECK(pick(furniture_of(*s), FurnitureKind::Signal).size() == 1);
    }
    {   // a two-way road with three lanes each way has opposing lanes: no left post
        const auto s = make_scene(line_nodes({{3, {{"highway", "traffic_signals"}}}}), {{10, kLine, {{"highway", "primary"}, {"lanes", "6"}}}});
        const auto signals = pick(furniture_of(*s), FurnitureKind::Signal);
        CHECK(signals.size() == 2);                                  // one right post per direction
        for (const auto& p : signals) CHECK((p.flags & kFurnitureLeftPost) == 0);
    }
}

TEST_CASE("T10: a signal on an arm 30 m before the junction controls that approach; the junction adds nothing there", "[layer][roads][furniture][T10]") {
    const auto s2 = make_scene({{1, 0, 0, {{"highway", "traffic_signals"}}}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}, {5, 0, 300}, {6, -30, 0, {{"highway", "traffic_signals"}, {"direction", "forward"}}}},
        {{10, {2, 6, 1, 3}}, {11, {4, 1, 5}}});
    FurnitureBuildStats stats;
    const auto all = furniture_of(*s2, &stats);
    CHECK(stats.suppressed_junction_lines == 1);
    const auto west_entry = [&] { for (std::size_t i = 0; i < s2->tile.entries.size(); ++i) if (s2->tile.entries[i].way_id == 10 && s2->tile.entries[i].stretch.start_ref == 0) return i; return std::size_t{0}; }();
    std::size_t node6_lines = 0, junction_lines_west = 0, junction_lines = 0;
    for (const auto& l : pick(all, FurnitureKind::StopLine)) {
        if (l.node_id == 6) { ++node6_lines; CHECK(l.junction_index == 0); CHECK(l.travel == LaneDirection::Forward); }
        else { ++junction_lines; if (l.entry_index == west_entry && l.travel == LaneDirection::Forward) ++junction_lines_west; }
    }
    CHECK(node6_lines == 1); CHECK(junction_lines == 3); CHECK(junction_lines_west == 0);
}

TEST_CASE("T10: stop and give-way signs on a junction node control the lower-ranked approaches; equal ranks are ambiguous", "[layer][roads][furniture][T10]") {
    {   // tertiary main road, residential side road: the side road gets the line, half a metre behind the trim
        const auto s = make_scene({{1, 0, 0, {{"highway", "stop"}}}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}}, {{10, {2, 1, 3}, {{"highway", "tertiary"}}}, {11, {1, 4}}});
        REQUIRE(s->junctions.size() == 1);
        const auto all = furniture_of(*s);
        const auto lines = pick(all, FurnitureKind::StopLine);
        REQUIRE(lines.size() == 1);
        const auto& l = lines[0];
        CHECK(s->tile.entries[l.entry_index].way_id == 11); CHECK(l.junction_index == 1); CHECK(l.node_id == 1);
        CHECK(l.travel == LaneDirection::Backward);   // the side road starts at the junction: traffic arrives running backwards
        CHECK(l.station_m == Approx(trim_of(*s, l.entry_index, false) + 0.5).margin(1e-9));
        CHECK(l.length_m == 0.5);
    }
    {   // give_way
        const auto s = make_scene({{1, 0, 0, {{"highway", "give_way"}}}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}}, {{10, {2, 1, 3}, {{"highway", "tertiary"}}}, {11, {1, 4}}});
        CHECK(pick(furniture_of(*s), FurnitureKind::GiveWayLine).size() == 1);
        CHECK(pick(furniture_of(*s), FurnitureKind::StopLine).empty());
    }
    {   // all approaches of the same rank: one Ambiguous record, no line
        const auto s = make_scene({{1, 0, 0, {{"highway", "stop"}}}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}}, {{10, {2, 1, 3}}, {11, {1, 4}}});
        FurnitureBuildStats stats;
        const auto all = furniture_of(*s, &stats);
        REQUIRE(all.size() == 1); CHECK(all[0].decline == FurnitureDecline::Ambiguous); CHECK(all[0].junction_index == 1);
        CHECK(stats.line_declined[static_cast<std::size_t>(FurnitureDecline::Ambiguous)] == 1);
    }
}

TEST_CASE("T10: a junction control of a declined junction is a JunctionDeclined record", "[layer][roads][furniture][T10]") {
    auto s = x_scene({{"highway", "traffic_signals"}});
    REQUIRE(s->junctions.size() == 1);
    s->junctions[0].decline = JunctionDecline::Degenerate;
    FurnitureBuildStats stats;
    const auto all = furniture_of(*s, &stats);
    REQUIRE(all.size() == 1);
    CHECK(all[0].decline == FurnitureDecline::JunctionDeclined); CHECK(all[0].kind == FurnitureKind::Signal); CHECK(all[0].junction_index == 1);
    // A control node of degree >= 3 that no listed junction contains produces nothing (its owner tile does).
    const auto none = build_road_furniture(s->tile, *s->builder, {}, builtin_road_rules_de(), &stats);
    REQUIRE(none); CHECK(none->empty());
}

TEST_CASE("D31: a ClusterTooLarge record adds no furniture; each member's own junction controls it", "[layer][roads][furniture][T10]") {
    // Five junction nodes 18 m apart on way 10 (72 m, beyond the 48 m cluster extent), each crossed by a 400 m way at 30
    // degrees: five single-node junctions plus one ClusterTooLarge record from node 1 (D19). Node 3 carries signals.
    std::vector<N> nodes = {{900, -300, 0}, {901, 390, 0}};
    std::vector<W> ways; std::vector<std::int64_t> main = {900};
    for (int i = 1; i <= 5; ++i) {
        const double x = 18.0 * (i - 1);
        nodes.push_back({i, x, 0, i == 3 ? Tags{{"highway", "traffic_signals"}} : Tags{}});
        nodes.push_back({100 + i, x - 173.20508075688772, -100}); nodes.push_back({200 + i, x + 173.20508075688772, 100});
        ways.push_back({100 + i, {100 + i, i, 200 + i}});
        main.push_back(i);
    }
    main.push_back(901); ways.push_back({10, main});
    const auto scene = make_scene(nodes, ways);
    std::size_t too_large = scene->junctions.size(), single3 = scene->junctions.size();
    for (std::size_t j = 0; j < scene->junctions.size(); ++j) {
        const auto& jn = scene->junctions[j];
        if (jn.decline == JunctionDecline::ClusterTooLarge) too_large = j;
        else if (jn.nodes == std::vector<std::int64_t>{3}) single3 = j;
    }
    REQUIRE(too_large < scene->junctions.size()); REQUIRE(single3 < scene->junctions.size());
    REQUIRE(std::binary_search(scene->junctions[too_large].nodes.begin(), scene->junctions[too_large].nodes.end(), 3));
    REQUIRE(scene->junctions[single3].decline == JunctionDecline::None);
    FurnitureBuildStats stats;
    const auto all = furniture_of(*scene, &stats);
    std::size_t from_single = 0, from_too_large = 0, declined = 0;
    for (const auto& f : all) {
        if (f.junction_index == single3 + 1) ++from_single;
        if (f.junction_index == too_large + 1) ++from_too_large;
        if (f.decline == FurnitureDecline::JunctionDeclined) ++declined;
    }
    std::printf("D31 cluster-too-large scene: %zu records from node 3's own junction, %zu from the ClusterTooLarge record, %zu JunctionDeclined; signal nodes counted %zu\n",
        from_single, from_too_large, declined, stats.signal_nodes);
    CHECK(from_single > 0);
    CHECK(from_too_large == 0);
    CHECK(declined == 0);
    CHECK(stats.signal_nodes == 1);   // the node is counted once, by the junction that controls it
    CHECK(stats.junction_controls_not_owned == 0);
    // Only the ClusterTooLarge record passed in: nothing, and the node is reported as not owned by any listed junction.
    FurnitureBuildStats only_stats;
    const auto only = build_road_furniture(scene->tile, *scene->builder, std::span<const RoadJunction>(&scene->junctions[too_large], 1), builtin_road_rules_de(), &only_stats);
    REQUIRE(only); CHECK(only->empty());
    CHECK(only_stats.junction_controls_not_owned == 1); CHECK(only_stats.signal_nodes == 0);
}

TEST_CASE("D31: a line is moved before a crossing only onto a station that has lanes of its travel", "[layer][roads][furniture][T10]") {
    // A forward signal 3 m into the way, a crossing 2 m further on: the line moves upstream to the crossing's near edge - 1 m.
    auto scene = make_scene({{1, -200, 0}, {2, -197, 0, {{"highway", "traffic_signals"}, {"direction", "forward"}}}, {3, -195, 0, {{"highway", "crossing"}}}, {4, 0, 0}, {5, 200, 0}},
        {{10, {1, 2, 3, 4, 5}}});
    const auto plain = furniture_of(*scene);
    const auto crossing = pick(plain, FurnitureKind::Crossing, 3).at(0);
    const auto line = pick(plain, FurnitureKind::StopLine, 2).at(0);
    const double moved = crossing.station_m - crossing.length_m / 2 - 1.0, original = 3.0;
    REQUIRE(line.station_m == Approx(moved).margin(1e-9));
    REQUIRE(moved < original);
    // White-box: the forward lane ramps in from zero width at the stretch start, so it is narrower than 0.5 m (no lane)
    // at the moved station and a lane at the original one. The ramp length is searched, the scene stays the same.
    const std::size_t e = line.entry_index;
    auto& xs = *scene->tile.entries[e].cross_section;
    bool found = false;
    for (int r = 2; r <= 200 && !found; ++r) {
        auto bands = xs.bands;
        for (auto& b : xs.bands) if (b.counted && b.kind == LaneKind::Driving && b.direction == LaneDirection::Forward) b.start = WidthRamp{r * 500, 0};
        if (lane_span(*scene, e, moved, LaneDirection::Forward).count == 0 && lane_span(*scene, e, original, LaneDirection::Forward).count > 0) found = true;
        else xs.bands = bands;
    }
    REQUIRE(found);
    FurnitureBuildStats stats;
    const auto all = furniture_of(*scene, &stats);
    const auto lines = pick(all, FurnitureKind::StopLine, 2);
    REQUIRE_FALSE(lines.empty());
    for (const auto& l : lines) {
        const auto span = lane_span(*scene, l.entry_index, l.station_m, l.travel);
        std::printf("D31 lanes at the moved station: line at %.3f m (moved %.3f, original %.3f), %zu lanes there, t %.3f..%.3f vs lane span %.3f..%.3f\n",
            l.station_m, moved, original, span.count, l.t_from, l.t_to, span.lo, span.hi);
        CHECK(span.count > 0);
        CHECK(l.t_from == Approx(span.lo).margin(0.0006)); CHECK(l.t_to == Approx(span.hi).margin(0.0006));
    }
    CHECK(stats.kept_without_lanes == 1); CHECK(stats.moved_before_crossing == 0);
}

// ---- fixtures ----

namespace {
std::string record_key(const RoadGeomTile& tile, const RoadFurniture& f) {
    const auto& e = tile.entries[f.entry_index];
    char buf[256];
    std::snprintf(buf, sizeof buf, "j%u w%lld r%u k%d d%u n%lld s%.17g v%d t%.6f..%.6f l%.4f p%d f%u", f.junction_index, static_cast<long long>(e.way_id), e.stretch.start_ref,
        static_cast<int>(f.kind), static_cast<unsigned>(f.decline), static_cast<long long>(f.node_id), f.station_m, static_cast<int>(f.travel), f.t_from, f.t_to, f.length_m,
        static_cast<int>(f.pattern), static_cast<unsigned>(f.flags));
    return buf;
}
struct FixtureTotals {
    std::size_t crossing_nodes_world = 0;
    std::map<std::int64_t, FurnitureDecline> crossing_by_node;          // union over tiles
    std::map<std::int64_t, int> crossing_pattern;
    std::array<std::size_t, kFurnitureDeclineCount> crossing_declined{};
    std::size_t crossing_placed = 0, repeated = 0, mismatches = 0;
    std::set<std::int64_t> signal_nodes, sign_nodes;
    std::size_t lines_stop = 0, lines_give_way = 0, signals = 0, left_posts = 0, junction_lines = 0, suppressed = 0, moved = 0, line_sets = 0;
    std::array<std::size_t, kFurnitureDeclineCount> line_declined{};
    std::size_t placed_quads = 0, quads_outside = 0, lines_off_lanes = 0;
};
FixtureTotals run_fixture(const std::string& pbf, bool tile_graph_context) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const auto world_builder = RoadReferenceBuilder::make(*world); REQUIRE(world_builder);
    FixtureTotals t;
    for (const auto& n : world->nodes) for (const auto& [k, v] : n.tags) if (k == "highway" && v == "crossing") ++t.crossing_nodes_world;
    std::set<TileKey> keys;
    for (const auto& n : world->nodes) {
        const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0);
        if (k) keys.insert(*k);
    }
    std::map<std::string, std::vector<std::string>> first;     // entry or junction -> sorted record keys
    std::set<std::string> seen_lines;
    for (const auto& key : keys) {
        const auto probe = select_road_graph_tile(*world, key);
        if (!probe || probe->owned_edges.empty()) continue;
        auto g = g2m_test::geometry_in_tile(*world, key); REQUIRE(g);
        std::optional<RoadReferenceBuilder> own;
        if (tile_graph_context) { auto b = RoadReferenceBuilder::make(g->source.graph); REQUIRE(b); own = std::move(*b); }
        const RoadReferenceBuilder& builder = tile_graph_context ? *own : *world_builder;
        auto js = build_road_junctions(*g, builder); REQUIRE(js);
        FurnitureBuildStats stats;
        auto fur = build_road_furniture(*g, builder, *js, builtin_road_rules_de(), &stats); REQUIRE(fur);
        // Group the records of this tile by their owner (an entry, or a junction) for the invariance comparison.
        std::map<std::string, std::vector<std::string>> mine;
        for (std::size_t i = 0; i < g->entries.size(); ++i) mine["e " + std::to_string(g->entries[i].way_id) + " " + std::to_string(g->entries[i].stretch.start_ref)];
        for (const auto& f : *fur) {
            std::string owner;
            if (f.junction_index) owner = "j " + std::to_string((*js)[f.junction_index - 1].nodes.front());
            else owner = "e " + std::to_string(g->entries[f.entry_index].way_id) + " " + std::to_string(g->entries[f.entry_index].stretch.start_ref);
            mine[owner].push_back(record_key(*g, f));
            if (f.kind == FurnitureKind::Crossing) {
                t.crossing_by_node.emplace(f.node_id, f.decline);
                if (f.decline == FurnitureDecline::None) t.crossing_pattern.emplace(f.node_id, static_cast<int>(f.pattern));
            }
        }
        for (auto& [owner, keys_of] : mine) {
            std::sort(keys_of.begin(), keys_of.end());
            const auto [it, inserted] = first.emplace(owner, keys_of);
            if (!inserted) { ++t.repeated; if (it->second != keys_of) ++t.mismatches; }
        }
        // Acceptance checks on this tile's records.
        for (const auto& f : *fur) {
            if (f.decline != FurnitureDecline::None) continue;
            const auto& e = g->entries[f.entry_index];
            const double length = e.profile->reference.length_m;
            if (f.kind == FurnitureKind::Crossing) {
                ++t.placed_quads;
                const auto edges = carriageway_edges(*e.cross_section, std::clamp(f.station_m, 0.0, length));
                if (!(f.t_from >= edges.right - 0.0006 && f.t_to <= edges.left + 0.0006)) ++t.quads_outside;
            } else if (f.kind != FurnitureKind::Signal) {
                std::array<LaneQuery, 32> buf{};
                const auto n = driving_lanes_at(*e.cross_section, std::clamp(f.station_m, 0.0, length), f.travel, buf);
                double lo = 1e9, hi = -1e9;
                for (std::size_t i = 0; i < n; ++i) { lo = std::min(lo, buf[i].centre_t - buf[i].width_m / 2); hi = std::max(hi, buf[i].centre_t + buf[i].width_m / 2); }
                if (n == 0 || f.t_from < lo - 0.0006 || f.t_to > hi + 0.0006) ++t.lines_off_lanes;
            }
        }
        // Totals: each owner counted once (the first tile that produces it), via the first-seen record keys.
        for (const auto& f : *fur) {
            const std::string k = record_key(*g, f);
            if (!seen_lines.insert(k).second) continue;
            if (f.kind == FurnitureKind::Crossing) {
                if (f.decline == FurnitureDecline::None) ++t.crossing_placed; else ++t.crossing_declined[static_cast<std::size_t>(f.decline)];
            } else if (f.decline != FurnitureDecline::None) ++t.line_declined[static_cast<std::size_t>(f.decline)];
            else if (f.kind == FurnitureKind::StopLine) ++t.lines_stop;
            else if (f.kind == FurnitureKind::GiveWayLine) ++t.lines_give_way;
            else { ++t.signals; if (f.flags & kFurnitureLeftPost) ++t.left_posts; }
            if (f.junction_index && f.decline == FurnitureDecline::None && f.kind != FurnitureKind::Signal) ++t.junction_lines;
        }
        t.suppressed += stats.suppressed_junction_lines; t.moved += stats.moved_before_crossing;
    }
    return t;
}
}

TEST_CASE("T9-T11 fixtures: counts, acceptance rules and tile invariance with its negative control", "[layer][roads][furniture][fixtures]") {
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto t = run_fixture(pbf, false);
        std::size_t declined = 0;
        for (std::size_t d = 1; d < kFurnitureDeclineCount; ++d) declined += t.crossing_declined[d];
        std::printf("[furniture] %s: crossing nodes %zu, placed %zu, declined %zu (stretch %zu, tunnel %zu, junction node %zu); lines stop %zu, give-way %zu (junction-derived %zu, suppressed by an arm node %zu, moved before a crossing, summed over tiles %zu), "
                    "signal posts %zu (left %zu); line declines: direction %zu, no lanes %zu, ambiguous %zu, junction declined %zu, stretch %zu, tunnel %zu\n",
            pbf, t.crossing_nodes_world, t.crossing_placed, declined, t.crossing_declined[1], t.crossing_declined[2], t.crossing_declined[3], t.lines_stop, t.lines_give_way, t.junction_lines,
            t.suppressed, t.moved, t.signals, t.left_posts, t.line_declined[4], t.line_declined[5], t.line_declined[6], t.line_declined[7], t.line_declined[1], t.line_declined[2]);
        std::printf("[furniture] %s: %zu records of repeated owners compared across tiles, %zu differ; placed crossing quads %zu (outside the carriageway %zu); lines off their lanes %zu\n",
            pbf, t.repeated, t.mismatches, t.placed_quads, t.quads_outside, t.lines_off_lanes);
        // T9 acceptance: every crossing node of the world has exactly one record in the union of the tiles.
        CHECK(t.crossing_by_node.size() == t.crossing_nodes_world);
        CHECK(t.crossing_placed + declined == t.crossing_nodes_world);
        CHECK(t.quads_outside == 0); CHECK(t.lines_off_lanes == 0);
        CHECK(t.mismatches == 0); CHECK(t.repeated > 0);
        // Negative control: node degrees and incident ways from the tile graph alone.
        const auto control = run_fixture(pbf, true);
        std::printf("[furniture] %s control (tile graph as context): %zu of %zu repeated owners differ, crossing records %zu (world %zu)\n", pbf, control.mismatches, control.repeated,
            control.crossing_by_node.size(), t.crossing_nodes_world);
        CHECK(control.mismatches > 0);
    }
}
