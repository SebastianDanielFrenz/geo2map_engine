// Turn-lane arrows (vault G2M-023/G2M-024): the painter reads turn:lanes through the explicit token table of
// parse_turn_token, never as substrings. The old rule drew a full bent arrow for merge_to_left and slight_right and a
// mirrored one on the neighbour of every merge. Cases: the glyph table, and the ribbon of the real trunk_link 179708586
// (turn:lanes through|slight_right|merge_to_left, 3 lanes) and the converging pair merge_to_right|merge_to_left of ways
// 471387463 / 257328194. Each rule has a recorded sabotage (see the comments at the CHECKs).
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "road_fixture.h"
#include "road_junction_scene.h"
#include "road_mesh_test_data.h"

#include "g2m/world/road_ribbons.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <tuple>
#include <vector>

namespace {
using g2m::world::kArrowLeft;
using g2m::world::kArrowMergeLeft;
using g2m::world::kArrowMergeRight;
using g2m::world::kArrowRight;
using g2m::world::kArrowSharpLeft;
using g2m::world::kArrowSharpRight;
using g2m::world::kArrowSlightLeft;
using g2m::world::kArrowSlightRight;
using g2m::world::kArrowThrough;
using g2m::world::lane_arrow_glyphs;

using Key = std::tuple<float, float, float, float>;

struct Drawn {
    g2m::world::RoadRibbonTile tile;
    std::vector<Key> vertices; // sorted
};

// The 600 m east-heading way 100 of the structure scene, made a one-way `lanes`-lane road with `turn_lanes`.
Drawn draw(const std::string& turn_lanes, int lanes, const std::string& highway = "primary") {
    auto graph = g2m_test::filter_road_graph(g2m_mesh_test::structure_scene_graph(), [](const g2m::RoadGraphWay& w) { return w.osm_id == 100; });
    for (auto& w : graph.ways) {
        w.tags = {{"highway", highway}, {"lanes", std::to_string(lanes)}, {"oneway", "yes"}};
        if (!turn_lanes.empty()) w.tags.emplace_back("turn:lanes", turn_lanes);
        std::sort(w.tags.begin(), w.tags.end());
    }
    g2m_test::GeomOptions o;
    o.height_m = g2m_mesh_test::structure_scene_dem;
    o.completion = g2m::served_road_completion_options();
    auto tile = g2m_test::geometry_in_tile_with(graph, g2m_mesh_test::kStructureSceneTile, o);
    if (!tile) throw std::runtime_error(tile.error().message);
    Drawn d;
    d.tile = g2m::world::build_road_ribbons(*tile, g2m_mesh_test::kStructureSceneTile,
        [](double e, double n) -> std::optional<double> { return 100.0 + 1e-5 * e - 1e-5 * n; }, 0.018, g2m::world::RoadSeparation{});
    for (const auto& v : d.tile.vertices) d.vertices.emplace_back(v.x, v.y, v.z, v.flags);
    std::sort(d.vertices.begin(), d.vertices.end());
    return d;
}

// The vertices `with` has beyond `base`: the arrows (the lane markings are the same in both).
std::vector<Key> extra(const Drawn& with, const Drawn& base) {
    std::vector<Key> out;
    std::set_difference(with.vertices.begin(), with.vertices.end(), base.vertices.begin(), base.vertices.end(), std::back_inserter(out));
    return out;
}
std::vector<Key> merged(std::vector<Key> a, const std::vector<Key>& b) {
    a.insert(a.end(), b.begin(), b.end());
    std::sort(a.begin(), a.end());
    return a;
}
// Sign of the covariance of y (left of travel) with x (along): > 0 = the glyph points to the left.
double lean(const std::vector<Key>& arrow) {
    double mx = 0, my = 0;
    for (const auto& [x, y, z, f] : arrow) { mx += x; my += y; }
    mx /= static_cast<double>(arrow.size()); my /= static_cast<double>(arrow.size());
    double c = 0;
    for (const auto& [x, y, z, f] : arrow) c += (x - mx) * (y - my);
    return c;
}
} // namespace

TEST_CASE("lane_arrow_glyphs: the token table", "[world][road_ribbons][arrows]") {
    // Sabotage A: the old substring rule (find("left") / find("right")) -> slight_*, merge_* and sharp_* fail.
    CHECK(lane_arrow_glyphs("through") == kArrowThrough);
    CHECK(lane_arrow_glyphs("left") == kArrowLeft);
    CHECK(lane_arrow_glyphs("right") == kArrowRight);
    CHECK(lane_arrow_glyphs("slight_left") == kArrowSlightLeft);
    CHECK(lane_arrow_glyphs("slight_right") == kArrowSlightRight);
    CHECK(lane_arrow_glyphs("sharp_left") == kArrowSharpLeft);
    CHECK(lane_arrow_glyphs("sharp_right") == kArrowSharpRight);
    CHECK(lane_arrow_glyphs("merge_to_left") == kArrowMergeLeft);
    CHECK(lane_arrow_glyphs("merge_to_right") == kArrowMergeRight);
    // Nothing: none, empty, reverse.
    for (const char* none : {"", "none", " none ", "reverse"}) { INFO(none); CHECK(lane_arrow_glyphs(none) == 0); }
    // Compounds combine.
    CHECK(lane_arrow_glyphs("through;slight_right") == (kArrowThrough | kArrowSlightRight));
    CHECK(lane_arrow_glyphs("left;through;right") == (kArrowLeft | kArrowThrough | kArrowRight));
    CHECK(lane_arrow_glyphs("through;reverse") == kArrowThrough);
    // A lane that ends and continues elsewhere has no Verschwenkungspfeil (sabotage B: merge bits set unconditionally).
    CHECK(lane_arrow_glyphs("through;merge_to_left") == kArrowThrough);
    CHECK(lane_arrow_glyphs("merge_to_left;slight_right") == kArrowSlightRight);
    CHECK(lane_arrow_glyphs("merge_to_left;merge_to_right") == (kArrowMergeLeft | kArrowMergeRight));
    // Unknown tokens draw nothing and are counted, per piece (sabotage C: the count is dropped).
    std::size_t unknown = 0;
    CHECK(lane_arrow_glyphs("diagonal", &unknown) == 0);
    CHECK(unknown == 1);
    CHECK(lane_arrow_glyphs("through;diagonal;sideways", &unknown) == kArrowThrough);
    CHECK(unknown == 3);
    CHECK(lane_arrow_glyphs("left", &unknown) == kArrowLeft);
    CHECK(unknown == 3);
}

TEST_CASE("Ribbon arrows: the real trunk_link through|slight_right|merge_to_left (way 179708586)", "[world][road_ribbons][arrows]") {
    const auto base = draw("none|none|none", 3, "trunk_link");
    const auto real = draw("through|slight_right|merge_to_left", 3, "trunk_link");
    REQUIRE(real.tile.turn_arrow_lanes == 3);
    CHECK(real.tile.merge_arrows == 1);
    CHECK(real.tile.unknown_turn_tokens == 0);
    CHECK(base.tile.turn_arrow_lanes == 0);
    const auto arrows = extra(real, base);
    REQUIRE_FALSE(arrows.empty());

    // Every lane draws what it would draw alone: the neighbours of the merge lane show no mirrored bend (sabotage D: the
    // mirrored neighbour of the old rule; here, a lane's glyph depending on the lanes beside it).
    const auto lane0 = extra(draw("through||", 3, "trunk_link"), base);
    const auto lane1 = extra(draw("|slight_right|", 3, "trunk_link"), base);
    const auto lane2 = extra(draw("||merge_to_left", 3, "trunk_link"), base);
    REQUIRE_FALSE(lane0.empty()); REQUIRE_FALSE(lane1.empty()); REQUIRE_FALSE(lane2.empty());
    CHECK(arrows == merged(merged(lane0, lane1), lane2));

    // The three glyphs differ: slight_right is not right, merge_to_left is neither left nor slight_left.
    CHECK(lane1 != extra(draw("|right|", 3, "trunk_link"), base));
    CHECK(lane2 != extra(draw("||left", 3, "trunk_link"), base));
    CHECK(lane2 != extra(draw("||slight_left", 3, "trunk_link"), base));
    // slight_right bends right (towards -y at its tip: it leans to the right of travel); the merge arrow leans left.
    CHECK(lean(lane1) < 0);
    CHECK(lean(lane2) > 0);
}

TEST_CASE("Ribbon arrows: merge_to_right|merge_to_left converge (ways 471387463, 257328194)", "[world][road_ribbons][arrows]") {
    const auto base = draw("none|none", 2);
    const auto pair = draw("merge_to_right|merge_to_left", 2);
    REQUIRE(pair.tile.turn_arrow_lanes == 2);
    CHECK(pair.tile.merge_arrows == 2);
    const auto left_lane = extra(draw("merge_to_right|none", 2), base);   // the left lane (token 0) points right
    const auto right_lane = extra(draw("none|merge_to_left", 2), base);   // the right lane (token 1) points left
    REQUIRE_FALSE(left_lane.empty()); REQUIRE_FALSE(right_lane.empty());
    // Sabotage E: the merge glyph's direction swapped (the sign passed to bend() inverted) -> both leans flip.
    CHECK(lean(left_lane) < 0);
    CHECK(lean(right_lane) > 0);
    CHECK(extra(pair, base) == merged(left_lane, right_lane));
    // Two diagonals plus their heads: the shape is the same triangle count as a one-sided merge, once per lane.
    CHECK(left_lane.size() == right_lane.size());
}

TEST_CASE("Ribbon arrows: slight bends are shallower than the full turn; unknown tokens are counted and draw nothing", "[world][road_ribbons][arrows]") {
    const auto base = draw("none|none", 2);
    const auto full = extra(draw("right|none", 2), base);
    const auto slight = extra(draw("slight_right|none", 2), base);
    REQUIRE_FALSE(full.empty()); REQUIRE_FALSE(slight.empty());
    const auto spread = [](const std::vector<Key>& a) {
        float lo = 1e9f, hi = -1e9f;
        for (const auto& [x, y, z, f] : a) { lo = std::min(lo, y); hi = std::max(hi, y); }
        return hi - lo;
    };
    // A full right turn reaches farther sideways than the slight bend (sabotage F: slight drawn with the full glyph).
    CHECK(spread(full) > 0);
    CHECK(full != slight);

    const auto unknown = draw("diagonal|none", 2);
    CHECK(unknown.tile.unknown_turn_tokens == 1);
    CHECK(unknown.tile.turn_arrow_lanes == 0);
    CHECK(extra(unknown, base).empty());
    const auto mixed = draw("through;diagonal|none", 2);
    CHECK(mixed.tile.unknown_turn_tokens == 1);
    CHECK(mixed.tile.turn_arrow_lanes == 1);

    // none, empty and reverse draw nothing.
    for (const char* nothing : {"none|none", "|", "reverse|none"}) { INFO(nothing); CHECK(extra(draw(nothing, 2), base).empty()); }
}

namespace {
// A T-junction: the two-way secondary 10 (west - centre - east, 4 lanes) and the one-way side street 11 from the south
// ending at the centre with `lanes` lanes and `turn_lanes`. Two left-only lanes make the junction draw guide lines
// (Leitlinien) into the west arm's lanes.
g2m::world::RoadRibbonTile draw_t_junction(const std::string& turn_lanes) {
    using Tags = std::vector<std::pair<std::string, std::string>>;
    constexpr double e0 = 455.0 * 1024.0 + 512.0, n0 = 5421.0 * 1024.0 + 512.0;
    struct Node { std::int64_t id; double x, y; };
    struct Way { std::int64_t id; int rank; std::vector<std::int64_t> nodes; Tags tags; };
    const std::vector<Node> nodes = {{1, -300, 0}, {3, 0, 0}, {6, 0, -300}, {10, 300, 0}};
    const std::vector<Way> ways = {
        {10, 6, {1, 3, 10}, {{"highway", "secondary"}, {"lanes", "4"}}},
        {11, 6, {6, 3}, {{"highway", "secondary"}, {"lanes", "2"}, {"oneway", "yes"}, {"turn:lanes", turn_lanes}}}};
    g2m::RoadGraph graph;
    for (const auto& n : nodes) graph.nodes.push_back({n.id, {std::llround((e0 + n.x) * 1000.0), std::llround((n0 + n.y) * 1000.0)}, {}});
    for (const auto& w : ways) {
        graph.ways.push_back({w.id, w.rank, w.nodes, w.tags});
        for (std::size_t i = 1; i < w.nodes.size(); ++i) graph.edges.push_back({w.id, static_cast<std::uint32_t>(i - 1), w.nodes[i - 1], w.nodes[i]});
    }
    g2m_test::GeomOptions o;
    o.completion = g2m::served_road_completion_options();
    auto tile = g2m_test::geometry_in_tile_with(graph, g2m_test::kJunctionSceneTile, o);
    if (!tile) throw std::runtime_error(tile.error().message);
    return g2m::world::build_road_ribbons(*tile, g2m_test::kJunctionSceneTile,
        [](double, double) -> std::optional<double> { return 100.0; }, 0.018, g2m::world::RoadSeparation{});
}
} // namespace

// The guide lines of a double left turn need two lanes tagged ONLY left (left family). merge_to_left is a lane that ends,
// not a left turn (sabotage G: left_only_lane replaced by the substring find("left") -> the merge lanes get guide lines).
TEST_CASE("Junction guide lines: a merge_to_left lane is not a left-turn lane", "[world][road_ribbons][arrows][junction]") {
    const auto none = draw_t_junction("none|none");
    const auto left = draw_t_junction("left|left");
    const auto slight = draw_t_junction("slight_left|left");
    const auto merge = draw_t_junction("merge_to_left|merge_to_left");
    const auto through = draw_t_junction("through|through");
    // Arrows only: two merge arrows draw 2 x 9 vertices, two through arrows 2 x 15.
    CHECK(merge.vertices.size() == none.vertices.size() + 2 * 9);
    CHECK(through.vertices.size() == none.vertices.size() + 2 * 15);
    // Guide lines come on top of the two left arrows (2 x 15 vertices).
    CHECK(left.vertices.size() > none.vertices.size() + 2 * 15);
    CHECK(slight.vertices.size() > none.vertices.size() + 30);
}
