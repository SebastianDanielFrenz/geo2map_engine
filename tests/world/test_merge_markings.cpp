// Edge markings along a merge lane or an exit (vault G2M-028): where a link lands on the added lane of a carriageway, the line between
// them is one broken line - the through carriageway's edge on the link's side is drawn broken (flag 128, or 64 on the left), the
// link's own facing edge line is dropped (flag 512, or 1024) - from the node to the gore nose (the link's facing gore shoulder ramp,
// ROADS stage 2b; without one its landing length; ROADS stage 4a), at every landed node whatever the through arm's join kind, and
// nowhere else.
//
// Recorded sabotages (2026-10-20; each makes the named tests fail, then reverted):
//   S1 road_ribbons.cpp: the through carriageway's broken bit on the wrong side (left for a ramp on the right)           -> MK1, MK2 fail
//   S2 road_ribbons.cpp: the through zone covers the whole stretch instead of the landing length                         -> MK2 fails (8919 assertions)
//   S3 road_ribbons.cpp: the through carriageway is not marked (skip the LaneChange branch)                              -> MK1, MK2 fail
//   S4 road_ribbons.cpp: the ramp's facing edge is not dropped (skip the Landing branch)                                 -> MK1 fails
//   S5 road_junction.cpp build_from: no MergeLane decline at a landed ramp (2026-10-20, with MK4)                          -> MK4 fails
//   S6 road_ribbons.cpp: the zone is the landing length even where a gore ramp exists (pre-4a) (2026-11-17)           -> MK2, MK2b fail
//   S7 road_ribbons.cpp: only a LaneChange through arm is marked (pre-4a) (2026-11-17)                                 -> MK1b fails
//   S10 (ROADS stage 8a-2, 2026-11-18) road_ribbons.cpp span_at: a zero-width counted section falls back to the attribute width -> MK9 fails
//   S8 (ROADS stage 6, 2026-11-18): see MK5. (S9 tested the stage 6 fallback marker, removed in stage 6r with its test.)
#include "road_fixture.h"

#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_rules.h"
#include "g2m/world/road_ribbons.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
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
RoadGraph scene(const std::string& ramp_highway, int ramp_y, bool placement = true) {
    RoadGraph g;
    const auto node = [&](std::int64_t id, double x, double y) { g.nodes.push_back({id, {std::llround((kE0 + x) * 1000.0), std::llround((kN0 + y) * 1000.0)}, {}}); };
    node(1, -400, 0); node(2, 0, 0); node(3, 200, 0); node(4, -150, ramp_y);
    g.ways.push_back(way(10, {1, 2}, {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}}));
    if (placement) g.ways.push_back(way(11, {2, 3}, {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "3"}, {"placement", "right_of:1"}, {"turn:lanes", "none|none|merge_to_left"}}));
    else g.ways.push_back(way(11, {2, 3}, {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "3"}, {"turn:lanes", "none|none|merge_to_left"}}));
    g.ways.push_back(way(12, {4, 2}, {{"highway", ramp_highway}, {"oneway", "yes"}, {"lanes", "1"}}));
    for (const auto& w : g.ways) for (std::size_t i = 1; i < w.node_ids.size(); ++i)
        g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    return g;
}
struct Marked {
    std::vector<world::RoadRibbonVertex> right_broken, left_hidden, others;
    double ox = 0, oy = 0;
    double landing_m = 0, gore_m = 0;   // the link's landing length and its facing gore shoulder ramp at the node (0 = none)
    JoinKind through_kind = JoinKind::None;   // B's join at the node
};
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
    for (const auto& e : tile->entries) {
        if (!e.cross_section) continue;
        if (e.way_id == 11) out.through_kind = e.cross_section->start_join.kind;
        if (e.way_id != 12) continue;
        const auto& xs = *e.cross_section;
        out.landing_m = xs.offset.end_length_mm / 1000.0;
        int first = -1, last = -1;
        for (std::size_t i = 0; i < xs.bands.size(); ++i) if (xs.bands[i].counted) { if (first < 0) first = static_cast<int>(i); last = static_cast<int>(i); }
        const int f = xs.end_join.raw_delta_mm < 0 ? first - 1 : last + 1;
        if (first >= 0 && f >= 0 && f < static_cast<int>(xs.bands.size())) {
            const auto& b = xs.bands[static_cast<std::size_t>(f)];
            if (b.kind == LaneKind::Shoulder && b.end.length_mm > 0 && b.end.from_width_mm == 0) out.gore_m = b.end.length_mm / 1000.0;
        }
    }
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

TEST_CASE("MK1b an inferred alignment through arm is marked too (ROADS stage 4a)", "[world][road_ribbons][merge_markings]") {
    const auto m = marked(scene("trunk_link", -12, false), builtin_road_rules_de());
    INFO("through join kind " << static_cast<int>(m.through_kind));
    REQUIRE(m.through_kind != JoinKind::LaneChange);
    REQUIRE(m.through_kind != JoinKind::None);
    REQUIRE_FALSE(m.right_broken.empty());
    REQUIRE_FALSE(m.left_hidden.empty());
    for (const auto& v : m.right_broken) CHECK(std::abs(v.y - node_y()) < 5.0);
    CHECK(m.others.empty());
}

TEST_CASE("MK2 the zone runs from the node to the gore nose (else the landing length), and only there", "[world][road_ribbons][merge_markings]") {
    const auto& rules = builtin_road_rules_de();
    // A link 40 m off the axis (slope-derived landing) and the 12 m link of MK1 (clearance landing, ROADS stage 2a).
    for (const int y : {-40, -12}) {
        const auto m = marked(scene("trunk_link", y), rules);
        INFO("link start y " << y << ", landing " << m.landing_m << " m, gore " << m.gore_m << " m");
        REQUIRE_FALSE(m.right_broken.empty());
        REQUIRE(m.landing_m > 0);
        const double zone = m.gore_m > 0 ? m.gore_m : m.landing_m;
        double first = 1e9, last = -1e9;
        for (const auto& v : m.right_broken) { CHECK(v.x <= node_x() + 0.5); first = std::min<double>(first, v.x); last = std::max<double>(last, v.x); }
        for (const auto& v : m.left_hidden) CHECK(v.x >= node_x() - zone - 2.5);
        // It starts at the gore nose and reaches the node.
        CHECK(first > node_x() - zone - 2.5); CHECK(first < node_x() - zone + 2.5);
        CHECK(last > node_x() - 2.5);
        // Every 5 m of the zone carries a broken quad.
        for (double x = node_x() - zone + 2.5; x < node_x() - 2.5; x += 5.0) {
            bool any = false;
            for (const auto& v : m.right_broken) any = any || std::abs(v.x - x) < 2.5;
            INFO("x " << x - node_x());
            CHECK(any);
        }
    }
}

TEST_CASE("MK2b a clearance landing with a gore ramp: the gore nose, not the landing, ends the zone", "[world][road_ribbons][merge_markings]") {
    const auto& rules = builtin_road_rules_de();
    const auto m = marked(scene("trunk_link", -12), rules);
    INFO("landing " << m.landing_m << " m, gore " << m.gore_m << " m");
    // The 12 m link gets a clearance landing longer than the slope-derived landing could be (ROADS stage 2a) ...
    CHECK(m.landing_m > rules.merge_lane.landing_max_m);
    CHECK(m.landing_m <= rules.merge_lane.landing_clear_max_m + 0.001);
    // ... and a gore ramp (ROADS stage 2b, at most gore_max_m), which bounds the zone (stage 4a).
    REQUIRE(m.gore_m > 0);
    CHECK(m.gore_m <= rules.merge_lane.gore_max_m + 0.001);
    double first = 1e9;
    for (const auto& v : m.right_broken) first = std::min<double>(first, v.x);
    CHECK(first > node_x() - m.gore_m - 2.5); CHECK(first < node_x() - m.gore_m + 2.5);
}

TEST_CASE("MK3 no landing, no marks: a link that is no merge lane and the switched-off rules", "[world][road_ribbons][merge_markings]") {
    const auto none = [](const Marked& m) { CHECK(m.right_broken.empty()); CHECK(m.left_hidden.empty()); CHECK(m.others.empty()); };
    // A non-link arm whose lanes balance (3 = 2 + 1) is a merge arm since ROADS 7b and gets the zone; with non_link_arm_classes empty it does not.
    CHECK_FALSE(marked(scene("tertiary", -12), builtin_road_rules_de()).right_broken.empty());
    auto no_non_link = builtin_road_rules_de();
    no_non_link.merge_lane.non_link_arm_classes.clear();
    none(marked(scene("tertiary", -12), no_non_link));
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

// ROADS stage 6: a taper node (S == T, no added lane). Over L_t the through carriageway's edge on the ramp's side is broken, the
// link's facing edge dropped; beyond L_t both are solid; the single arm gets no broken edge (no lane line is added on S). Also for a
// way passing through the node (its through end found by direction). Sabotage S8 (2026-11-18): the zone is the gore ramp / landing
// length as before stage 6, not the taper's width ramp -> MK5 fails.
namespace {
RoadGraph taper_scene(bool passing, bool short_link = false, bool link_shoulder = true) {
    RoadGraph g;
    const auto node = [&](std::int64_t id, double x, double y) { g.nodes.push_back({id, {std::llround((kE0 + x) * 1000.0), std::llround((kN0 + y) * 1000.0)}, {}}); };
    node(1, -400, 0); node(2, 0, 0); node(3, 200, 0);
    if (short_link) { node(4, -20, -3); node(5, -20, -60); node(6, -20, 60); } else node(4, -150, -12);
    const Tags t2 = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}};
    if (passing) g.ways.push_back(way(10, {1, 2, 3}, t2));
    else { g.ways.push_back(way(10, {1, 2}, t2)); g.ways.push_back(way(11, {2, 3}, t2)); }
    Tags link = {{"highway", "trunk_link"}, {"oneway", "yes"}, {"lanes", "1"}};
    if (!link_shoulder) link.push_back({"shoulder", "no"});
    g.ways.push_back(way(12, {4, 2}, link));
    if (short_link) g.ways.push_back(way(13, {5, 4, 6}, {{"highway", "residential"}}));
    for (const auto& w : g.ways) for (std::size_t i = 1; i < w.node_ids.size(); ++i)
        g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    return g;
}
// The link's taper length at the node: its counted lane's width ramp from 0.
double taper_m(const RoadGraph& graph, const RoadRules& rules) {
    g2m_test::GeomOptions o;
    o.height_m = [](double, double) { return 100.0; };
    o.completion = served_road_completion_options();
    o.rules = &rules;
    auto tile = g2m_test::geometry_in_tile_with(graph, kKey, o);
    if (!tile) throw std::runtime_error(tile.error().message);
    for (const auto& e : tile->entries) if (e.way_id == 12 && e.cross_section)
        for (const auto& b : e.cross_section->bands) if (b.counted && b.end.length_mm > 0 && b.end.from_width_mm == 0) return b.end.length_mm / 1000.0;
    return 0;
}
} // namespace

TEST_CASE("MK5 a taper node: the broken line and the dropped edge run over L_t, then both edges are solid", "[world][road_ribbons][merge_markings]") {
    const auto& rules = builtin_road_rules_de();
    for (const bool passing : {false, true}) {
        INFO("passing " << passing);
        const auto g = taper_scene(passing);
        const double lt = taper_m(g, rules);
        INFO("L_t " << lt << " m");
        REQUIRE(lt >= rules.merge_lane.taper_length_m - 0.001);
        const auto m = marked(g, rules);
        REQUIRE_FALSE(m.right_broken.empty());
        REQUIRE_FALSE(m.left_hidden.empty());
        CHECK(m.others.empty());
        double first = 1e9, last = -1e9;
        for (const auto& v : m.right_broken) {
            CHECK(std::abs(v.y - node_y()) < 5.0);
            CHECK(v.x <= node_x() + 0.5);   // only the through arm (west of the node); S east of it has none
            first = std::min<double>(first, v.x); last = std::max<double>(last, v.x);
        }
        CHECK(first > node_x() - lt - 2.5); CHECK(first < node_x() - lt + 2.5);
        CHECK(last > node_x() - 2.5);
        for (const auto& v : m.left_hidden) CHECK(v.x >= node_x() - lt - 2.5);
    }
}

TEST_CASE("MK6 a taper node has no junction patch (decline MergeLane), also where the way passes through", "[world][road_ribbons][merge_markings]") {
    for (const bool passing : {false, true}) {
        INFO("passing " << passing);
        const auto& rules = builtin_road_rules_de();
        g2m_test::GeomOptions o;
        o.height_m = [](double, double) { return 100.0; };
        o.completion = served_road_completion_options();
        o.rules = &rules;
        auto tile = g2m_test::geometry_in_tile_with(taper_scene(passing), kKey, o);
        REQUIRE(tile);
        REQUIRE(tile->junctions.size() == 1);
        CHECK(tile->junctions[0].decline == JunctionDecline::MergeLane);
    }
}

// ROADS stage 6r: a taper candidate without room (a 20 m link from a junction) is a junction: the node gets its patch and no marks.
// An accepted taper never has a patch (MK6), a refused one always has.
TEST_CASE("MK7 a short link at an S == T node is no taper: a junction patch, no merge marks", "[world][road_ribbons][merge_markings]") {
    const auto& rules = builtin_road_rules_de();
    const auto g = taper_scene(false, true);
    CHECK(taper_m(g, rules) == 0);
    g2m_test::GeomOptions o;
    o.height_m = [](double, double) { return 100.0; };
    o.completion = served_road_completion_options();
    o.rules = &rules;
    auto tile = g2m_test::geometry_in_tile_with(g, kKey, o);
    REQUIRE(tile);
    bool node2 = false;
    for (const auto& j : tile->junctions) {
        bool at2 = false;
        for (const auto& a : j.approaches) if (a.entry_index < tile->entries.size() && tile->entries[a.entry_index].way_id == 12) at2 = true;
        if (!at2) continue;
        node2 = true;
        CHECK(j.decline != JunctionDecline::MergeLane);
    }
    CHECK(node2);
    const auto m = marked(g, rules);
    CHECK(m.right_broken.empty());
    CHECK(m.left_hidden.empty());
}

// ROADS stage 6r-3: the classifier accepts exactly the tapers the geometry lands, an accepted taper never has a junction patch
// at its node and a refused one always has. Stage 6 accepted six region nodes whose far-end clearance or held offset left the link
// unlanded, so they got a patch (and the short link merged them into a two-node cluster). A link of `link_m` from node 4 on a
// two-way primary (node 4 a junction) to node 2 on the carriageway. Sabotage S10 (2026-11-18): classify_merge_node without the
// room gate (stage 6) -> MK8 fails.
TEST_CASE("MK8 an accepted taper never has a junction patch at its node, a refused one always has", "[world][road_ribbons][merge_markings]") {
    const auto scene = [](int link_m) {
        RoadGraph g;
        const auto node = [&](std::int64_t id, double x, double y) { g.nodes.push_back({id, {std::llround((kE0 + x) * 1000.0), std::llround((kN0 + y) * 1000.0)}, {}}); };
        node(1, -400, 0); node(2, 0, 0); node(3, 200, 0); node(4, -link_m, -3); node(5, -link_m - 300, 27); node(6, -link_m + 300, -33);
        const Tags t2 = {{"highway", "trunk"}, {"oneway", "yes"}, {"lanes", "2"}};
        g.ways.push_back(way(10, {1, 2}, t2)); g.ways.push_back(way(11, {2, 3}, t2));
        g.ways.push_back(way(12, {4, 2}, {{"highway", "trunk_link"}, {"oneway", "yes"}, {"lanes", "1"}}));
        // node 4: the link leaves a wide two-way road at a shallow angle
        g.ways.push_back(way(13, {5, 4, 6}, {{"highway", "primary"}, {"lanes", "4"}}));
        for (const auto& w : g.ways) for (std::size_t i = 1; i < w.node_ids.size(); ++i)
            g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
        return g;
    };
    auto small = builtin_road_rules_de();
    small.merge_lane.taper_min_length_m = 1;
    // No far-end clearance: short links land too.
    auto tight = small;
    tight.merge_lane.landing_width_far_margin_m = 0; tight.merge_lane.landing_width_far_max_m = 0;
    std::size_t landed = 0, refused = 0;
    for (const RoadRules* rules : {&builtin_road_rules_de(), static_cast<const RoadRules*>(&small), static_cast<const RoadRules*>(&tight)})
        for (const int link_m : {6, 8, 10, 12, 16, 20, 25, 30, 40, 60, 120}) {
            INFO("link " << link_m << " m, min " << rules->merge_lane.taper_min_length_m);
            const auto g = scene(link_m);
            const auto builder = RoadReferenceBuilder::make(g); REQUIRE(builder);
            const bool accepted = diagnose_merge_node(*builder, 2, *rules).taper;
            g2m_test::GeomOptions o;
            o.height_m = [](double, double) { return 100.0; };
            o.completion = served_road_completion_options();
            o.rules = rules;
            auto tile = g2m_test::geometry_in_tile_with(g, kKey, o);
            REQUIRE(tile);
            bool lands = false;
            for (const auto& e : tile->entries) if (e.way_id == 12 && e.cross_section && e.cross_section->end_join.kind == JoinKind::Landing) lands = true;
            std::size_t at2 = 0;
            for (const auto& j : tile->junctions) {
                if (std::find(j.nodes.begin(), j.nodes.end(), 2) == j.nodes.end()) continue;
                ++at2;
                if (lands) { CHECK(j.nodes.size() == 1); CHECK(j.decline == JunctionDecline::MergeLane); }
                else CHECK(j.decline == JunctionDecline::None);
            }
            CHECK(at2 == 1);
            CHECK(accepted == lands);   // the classifier accepts exactly the tapers the geometry lands
            ++(lands ? landed : refused);
        }
    std::printf("[MK8] landed %zu, refused %zu\n", landed, refused);
    CHECK(landed > 0);
    CHECK(refused > 0);
}

// ROADS stage 8a-2: the link's tip at a taper node. Its counted lanes ramp to zero width there; the ribbon used to fall back to the
// attribute width for that last section and drew a full-width metre of bed and edge line beside the through carriageway (the notch of
// view 13). Now the section collapses at the node, and no row of the link's bed is narrower than 0.05 m: its outer shoulder runs on to
// the node at the single arm's shoulder width (also with shoulder=no, whose shoulder ramps up from 0 there). Sabotage S10.
TEST_CASE("MK9 a taper link's tip: no full-width end row, no bed row narrower than 0.05 m", "[world][road_ribbons][merge_markings]") {
    const auto& rules = builtin_road_rules_de();
    for (const bool passing : {false, true}) for (const bool shoulder : {true, false}) {
        INFO("passing " << passing << " link shoulder " << shoulder);
        const auto g = taper_scene(passing, false, shoulder);
        REQUIRE(taper_m(g, rules) > 0);
        g2m_test::GeomOptions o;
        o.height_m = [](double, double) { return 100.0; };
        o.completion = served_road_completion_options();
        o.rules = &rules;
        auto built = g2m_test::geometry_in_tile_with(g, kKey, o);
        REQUIRE(built);
        // As served: through the tile encoding (the section's widths and lengths exactly as the game reads them).
        const auto body = encode_road_geom(*built);
        REQUIRE(body);
        const auto tile = decode_road_geom(*body);
        REQUIRE(tile);
        world::RoadRibbonOptions options;
        options.drape = world::RibbonDrape::PointSampled;
        const auto ribbon = world::build_road_ribbons(*tile, kKey, [](double, double) -> std::optional<double> { return 100.0; }, 0.018, {}, options);
        // The link's quads in its landing zone carry the dropped facing edge (512); group its road-bed vertices by station.
        std::vector<std::pair<long long, std::pair<float, float>>> rows;   // station (mm) -> (min u, max u)
        double nearest = 1e9;
        float last = -1e9f;
        for (const auto& v : ribbon.vertices) {
            const int f = static_cast<int>(v.flags);
            if ((f & kOverlay) || !(f & kLeftHidden) || v.verge != 0) continue;
            nearest = std::min(nearest, std::hypot(v.x - node_x(), v.y - node_y()));
            last = std::max(last, v.v);
        }
        for (const auto& v : ribbon.vertices) {
            const int f = static_cast<int>(v.flags);
            if ((f & kOverlay) || !(f & kLeftHidden) || v.verge != 0) continue;
            // the counted width over the link's last metre is (near) zero, not the attribute width
            if (v.v > last - 1.01f) { INFO("station " << v.v << " of " << last); CHECK(v.width < 0.1f); }
            const long long st = std::llround(static_cast<double>(v.v) * 1000.0);
            auto it = std::find_if(rows.begin(), rows.end(), [&](const auto& r) { return r.first == st; });
            if (it == rows.end()) rows.push_back({st, {v.u, v.u}});
            else { it->second.first = std::min(it->second.first, v.u); it->second.second = std::max(it->second.second, v.u); }
        }
        REQUIRE_FALSE(rows.empty());
        for (const auto& r : rows) { INFO("station " << r.first << " mm"); CHECK(r.second.second - r.second.first >= 0.05f - 1e-4f); }
        CHECK(nearest < 15.0);   // the link still reaches the node region
    }
}
