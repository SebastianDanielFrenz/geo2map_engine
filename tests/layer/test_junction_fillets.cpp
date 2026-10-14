// Junction corner fillets (kerb returns): every block corner of a junction ring is rounded with the corner radius of
// the lower of its two approaches' classes (data/road_rules/de.json junction.corner_radius_mm) on the road bed edge;
// the paved and outer edges follow concentrically. Hand-computed X junctions, the class rule, gores that stay sharp,
// the fixtures' census and a randomised robustness sweep (any star junction: rings valid and nested, never a hull
// the sharp build did not need).
#include "road_fixture.h"
#include "splitmix64.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_junction.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <memory>
#include <set>
#include <string>
using namespace g2m;
using Catch::Approx;

namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};
const TileKey kTile{zone, 2, 455, 5421};
constexpr double kOriginE = 455.0 * 1024.0 + 512.0, kOriginN = 5421.0 * 1024.0 + 512.0;   // tile centre, metres
constexpr double kPi = 3.14159265358979323846;
using Tags = std::vector<std::pair<std::string, std::string>>;

struct N { std::int64_t id; double x, y; };
struct W { std::int64_t id; std::vector<std::int64_t> nodes; Tags tags = {{"highway", "residential"}}; };
struct Scene {
    RoadGraph graph;
    RoadGeomTile tile;
    std::optional<RoadReferenceBuilder> builder;
};
std::unique_ptr<Scene> make_scene(const std::vector<N>& nodes, std::vector<W> ways) {
    auto scene = std::make_unique<Scene>();
    for (const auto& n : nodes) scene->graph.nodes.push_back({n.id, {std::llround((kOriginE + n.x) * 1000.0), std::llround((kOriginN + n.y) * 1000.0)}, {}});
    std::sort(scene->graph.nodes.begin(), scene->graph.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    std::sort(ways.begin(), ways.end(), [](const auto& a, const auto& b) { return a.id < b.id; });
    for (auto& w : ways) {
        Tags tags = w.tags; std::sort(tags.begin(), tags.end());
        scene->graph.ways.push_back({w.id, 3, w.nodes, tags});
        for (std::size_t i = 1; i < w.nodes.size(); ++i) scene->graph.edges.push_back({w.id, static_cast<std::uint32_t>(i - 1), w.nodes[i - 1], w.nodes[i]});
    }
    auto tile = g2m_test::geometry_in_tile(scene->graph, kTile);
    REQUIRE(tile);
    scene->tile = std::move(*tile);
    auto builder = RoadReferenceBuilder::make(scene->graph);
    REQUIRE(builder);
    scene->builder = std::move(*builder);
    return scene;
}
// Straight perpendicular crossing: way 10 west-east, way 11 south-north.
std::unique_ptr<Scene> x_scene(const std::string& we = "residential", const std::string& sn = "residential") {
    return make_scene({{1, 0, 0}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}, {5, 0, 300}},
        {{10, {2, 1, 3}, {{"highway", we}}}, {11, {4, 1, 5}, {{"highway", sn}}}});
}
JunctionBuildOptions sharp() { JunctionBuildOptions o; o.corner_fillets = false; return o; }
RoadJunction junction_of(const Scene& s, std::int64_t node, JunctionBuildStats* stats = nullptr, const JunctionBuildOptions& o = {},
    const RoadRules& rules = builtin_road_rules_de()) {
    auto j = build_node_junction(s.tile, *s.builder, node, rules, stats, o);
    REQUIRE(j);
    return std::move(*j);
}
double roadbed_half_width(const Scene& s, std::int64_t way) {
    for (const auto& e : s.tile.entries)
        if (e.way_id == way && e.cross_section) return carve_footprint(*e.cross_section, 50.0).roadbed_left;
    FAIL("no entry of way " << way);
    return 0;
}
geom::PointMm node_pos(const Scene& s, std::int64_t id) {
    return std::find_if(s.graph.nodes.begin(), s.graph.nodes.end(), [&](const auto& n) { return n.osm_id == id; })->position;
}
void check_rings(const RoadJunction& j) {
    for (const auto& ring : j.rings) CHECK(junction_ring_valid(ring));
    CHECK(junction_rings_nested(j));
}
// The road bed ring of a perpendicular X junction (half widths wx of way 10, wy of way 11) with corner radius r:
// every vertex strictly inside a corner quadrant (beyond both edges) lies on the circle of radius r about the
// quadrant's centre (wy + r, wx + r) away from the node, and no vertex lies inside any of the four circles.
void check_x_arcs(const RoadJunction& j, geom::PointMm node, double wx, double wy, double r, int layer = 0) {
    std::size_t on_arcs = 0;
    for (const auto& v : j.rings[layer].vertices) {
        const double x = static_cast<double>(v.x - node.x) / 1000.0, y = static_cast<double>(v.y - node.y) / 1000.0;
        for (const double sx : {-1.0, 1.0}) for (const double sy : {-1.0, 1.0}) {
            const double cx = sx * (wy + r), cy = sy * (wx + r);
            const double d = std::hypot(x - cx, y - cy);
            CHECK(d >= r - 0.003);   // never inside the land disk of a corner
            if (sx * x > wy + 0.01 && sy * y > wx + 0.01) { CHECK(d == Approx(r).margin(0.003)); ++on_arcs; }
        }
    }
    CHECK(on_arcs >= 4 * 8);   // 90 degrees of turn in 7.5 degree steps: 11 inner points per corner
}
}

TEST_CASE("Fillets: a residential X junction gets 6 m kerb returns on all four corners", "[layer][junction][fillet]") {
    const auto scene = x_scene();
    JunctionBuildStats stats;
    const auto j = junction_of(*scene, 1, &stats);
    REQUIRE(j.decline == JunctionDecline::None);
    CHECK(stats.fillets == 4); CHECK(stats.filleted_junctions == 1); CHECK(stats.fillet_fallbacks == 0);
    CHECK_FALSE((j.flags & static_cast<std::uint16_t>(JunctionFlags::HullFallback)));
    check_rings(j);
    const double w = roadbed_half_width(*scene, 10);
    REQUIRE(w == Approx(roadbed_half_width(*scene, 11)));
    check_x_arcs(j, node_pos(*scene, 1), w, w, 6.0);
    // At 90 degrees the tangent points lie r beyond the corner on every layer (the outer layers are concentric), so
    // every trim is the road bed half width + r + the margin.
    for (const auto& a : j.approaches) CHECK(a.trim_m == Approx(w + 6.0 + 0.5).margin(0.002));
    // The sharp build of the same node: the 12-vertex cross with the trims at the outer corner + margin.
    const auto s = junction_of(*scene, 1, nullptr, sharp());
    CHECK(s.rings[2].vertices.size() == 12);
    for (std::size_t i = 0; i < j.approaches.size(); ++i) CHECK(j.approaches[i].trim_m > s.approaches[i].trim_m + 1.0);
}

TEST_CASE("Fillets: the paved and outer rings follow the road bed arc concentrically", "[layer][junction][fillet]") {
    // A road with sidewalks: the paved edge lies the sidewalk width outside the road bed, so its arc has radius
    // r - sidewalk about the same centre; the rings nest.
    const Tags side = {{"highway", "residential"}, {"sidewalk", "both"}};
    const auto scene = make_scene({{1, 0, 0}, {2, -300, 0}, {3, 300, 0}, {4, 0, -300}, {5, 0, 300}}, {{10, {2, 1, 3}, side}, {11, {4, 1, 5}, side}});
    JunctionBuildStats stats;
    const auto j = junction_of(*scene, 1, &stats);
    REQUIRE(j.decline == JunctionDecline::None);
    CHECK(stats.fillets == 4);
    check_rings(j);
    const auto& e = *std::find_if(scene->tile.entries.begin(), scene->tile.entries.end(), [](const auto& x) { return x.way_id == 10; });
    const auto f = carve_footprint(*e.cross_section, 50.0);
    const double sidewalk = f.paved_left - f.roadbed_left;
    REQUIRE(sidewalk > 1.0);
    check_x_arcs(j, node_pos(*scene, 1), f.roadbed_left, f.roadbed_left, 6.0, 0);
    // Paved: the same centres, radius r - sidewalk (the arc centre does not move).
    const double rp = 6.0 - sidewalk;
    check_x_arcs(j, node_pos(*scene, 1), f.paved_left, f.paved_left, rp, 1);
}

TEST_CASE("Fillets: the radius is that of the lower class; motorway corners stay sharp", "[layer][junction][fillet]") {
    {
        const auto scene = x_scene("primary", "residential");
        const auto j = junction_of(*scene, 1);
        check_rings(j);
        check_x_arcs(j, node_pos(*scene, 1), roadbed_half_width(*scene, 10), roadbed_half_width(*scene, 11), 6.0);
    }
    {
        const auto scene = x_scene("primary", "primary");
        const auto j = junction_of(*scene, 1);
        check_rings(j);
        const double w = roadbed_half_width(*scene, 10);
        check_x_arcs(j, node_pos(*scene, 1), w, w, 12.0);
        for (const auto& a : j.approaches) CHECK(a.trim_m == Approx(w + 12.0 + 0.5).margin(0.002));
    }
    {
        // Rules with a zero radius for both classes: identical to the sharp build.
        auto rules = builtin_road_rules_de();
        for (auto& [cls, r] : rules.junction.corner_radius_mm) r = 0;
        rules.junction.corner_radius_default_mm = 0;
        const auto scene = x_scene();
        JunctionBuildStats stats;
        CHECK(junction_of(*scene, 1, &stats, {}, rules) == junction_of(*scene, 1, nullptr, sharp()));
        CHECK(stats.fillets == 0);
    }
}

TEST_CASE("Fillets: a narrow gore stays sharp, the wide side of a Y is rounded", "[layer][junction][fillet]") {
    for (const double degrees : {10.0, 15.0}) {
        INFO("angle " << degrees);
        const double a = degrees * kPi / 180.0;
        const auto scene = make_scene({{1, 0, 0}, {2, -300, 0}, {3, 300, 0}, {4, 300 * std::cos(a), 300 * std::sin(a)}}, {{10, {2, 1, 3}}, {11, {1, 4}}});
        const auto j = junction_of(*scene, 1);
        const auto s = junction_of(*scene, 1, nullptr, sharp());
        check_rings(j);
        // The gore (wedge under corner_min_wedge_deg) keeps its sharp corner: the branch trims are the gore trims.
        double longest = 0, longest_sharp = 0;
        for (const auto& x : j.approaches) longest = std::max(longest, x.trim_m);
        for (const auto& x : s.approaches) longest_sharp = std::max(longest_sharp, x.trim_m);
        CHECK(longest == Approx(longest_sharp).margin(0.001));
    }
}

TEST_CASE("Fillets: two one-way carriageways splitting at a narrow angle get a nose radius, not a kerb return", "[layer][junction][fillet]") {
    // A two-way road (way 10) splits into two one-way carriageways at +-20 degrees (a 40 degree wedge).
    const double a = 20.0 * kPi / 180.0;
    const Tags one = {{"highway", "residential"}, {"oneway", "yes"}};
    const auto scene = make_scene({{1, 0, 0}, {2, -300, 0}, {3, 300 * std::cos(a), 300 * std::sin(a)}, {4, 300 * std::cos(a), -300 * std::sin(a)}},
        {{10, {2, 1}}, {11, {1, 3}, one}, {12, {4, 1}, one}});
    JunctionBuildStats stats;
    const auto j = junction_of(*scene, 1, &stats);
    check_rings(j);
    auto two_way = make_scene({{1, 0, 0}, {2, -300, 0}, {3, 300 * std::cos(a), 300 * std::sin(a)}, {4, 300 * std::cos(a), -300 * std::sin(a)}},
        {{10, {2, 1}}, {11, {1, 3}}, {12, {4, 1}}});
    const auto t = junction_of(*two_way, 1);
    // The one-way nose (1 m) ends far nearer the node than the 6 m kerb return of the two-way version.
    double one_max = 0, two_max = 0;
    for (const auto& x : j.approaches) if (scene->tile.entries[x.entry_index].way_id != 10) one_max = std::max(one_max, x.trim_m);
    for (const auto& x : t.approaches) if (two_way->tile.entries[x.entry_index].way_id != 10) two_max = std::max(two_max, x.trim_m);
    CHECK(one_max + 3.0 < two_max);
}

TEST_CASE("Fillets: random star junctions always give valid nested rings and never a hull the sharp build avoided", "[layer][junction][fillet]") {
    // Robustness against new data: 3 to 6 arms at random angles (any gap, also near-parallel ones), random classes,
    // sidewalks and one-way tags, some arms curving. Every junction must be declined or carry three valid, nested
    // rings; a fillet that would break that is dropped (stats.fillet_fallbacks), never served.
    static const char* kClasses[] = {"residential", "tertiary", "secondary", "primary", "service", "living_street", "unclassified", "track"};
    g2m_test::SplitMix64 rng(0x6a756e6374696f6eULL);
    std::size_t built = 0, filleted = 0, fallbacks = 0, hull_new = 0, hull_sharp = 0;
    for (int scene_i = 0; scene_i < 60; ++scene_i) {
        const int arms = 3 + static_cast<int>(rng.uniform(0.0, 3.999));
        std::vector<double> angles;
        for (int k = 0; k < arms; ++k) { const double v = rng.uniform(0.0, 360.0); angles.push_back(v); }
        std::sort(angles.begin(), angles.end());
        std::vector<N> nodes = {{1, 0, 0}};
        std::vector<W> ways;
        for (int k = 0; k < arms; ++k) {
            const double ang = angles[static_cast<std::size_t>(k)] * kPi / 180.0;
            const double bend = rng.uniform(-1.0, 1.0) < -0.3 ? rng.uniform(-0.01, 0.01) : 0.0;   // curvature 1/m
            const double cls_pick = rng.uniform(0.0, 7.999);
            const double side_pick = rng.uniform(0.0, 1.0);
            const double one_pick = rng.uniform(0.0, 1.0);
            std::vector<std::int64_t> ids = {1};
            double x = 0, y = 0, h = ang;
            for (int p = 1; p <= 10; ++p) {
                x += 25.0 * std::cos(h); y += 25.0 * std::sin(h); h += bend * 25.0;
                const std::int64_t id = 100 * (k + 1) + p;
                nodes.push_back({id, x, y}); ids.push_back(id);
            }
            Tags tags = {{"highway", kClasses[static_cast<int>(cls_pick)]}};
            if (side_pick < 0.4) tags.push_back({"sidewalk", "both"});
            if (one_pick < 0.2) tags.push_back({"oneway", "yes"});
            ways.push_back({1000 + k, ids, tags});
        }
        const auto scene = make_scene(nodes, ways);
        JunctionBuildStats fs, ss;
        const auto rf = build_node_junction(scene->tile, *scene->builder, 1, builtin_road_rules_de(), &fs);
        const auto rs = build_node_junction(scene->tile, *scene->builder, 1, builtin_road_rules_de(), &ss, sharp());
        REQUIRE(rf); REQUIRE(rs);
        INFO("scene " << scene_i << " arms " << arms);
        CHECK(rf->decline == rs->decline);
        if (rf->decline != JunctionDecline::None) continue;
        ++built;
        check_rings(*rf);
        filleted += fs.filleted_junctions; fallbacks += fs.fillet_fallbacks;
        hull_new += fs.hull_fallbacks; hull_sharp += ss.hull_fallbacks;
        CHECK(fs.hull_fallbacks <= ss.hull_fallbacks);
        // Determinism: a second build is identical.
        const auto again = build_node_junction(scene->tile, *scene->builder, 1, builtin_road_rules_de());
        REQUIRE(again); CHECK(*again == *rf);
    }
    std::printf("[fillet sweep] built %zu, with fillets %zu, fillet fallbacks %zu, hulls %zu (sharp build %zu)\n", built, filleted, fallbacks, hull_new, hull_sharp);
    CHECK(built >= 40);
    CHECK(filleted >= built / 2);
}

TEST_CASE("Fillets: fixture census - kerb returns on most junctions, rings valid and nested", "[layer][junction][fillet][roads]") {
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
        const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
        const auto builder = RoadReferenceBuilder::make(*world); REQUIRE(builder);
        std::set<TileKey> keys;
        for (const auto& n : world->nodes) {
            const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0);
            if (k) keys.insert(*k);
        }
        JunctionBuildStats with, without;
        std::size_t rings = 0;
        for (const auto& key : keys) {
            const auto probe = select_road_graph_tile(*world, key);
            if (!probe || probe->owned_edges.empty()) continue;
            auto g = g2m_test::geometry_in_tile(*world, key); REQUIRE(g);
            auto js = build_road_junctions(*g, *builder, builtin_road_rules_de(), &with); REQUIRE(js);
            REQUIRE(build_road_junctions(*g, *builder, builtin_road_rules_de(), &without, sharp()));
            for (const auto& j : *js) {
                if (j.decline != JunctionDecline::None) continue;
                ++rings;
                for (const auto& ring : j.rings) CHECK(junction_ring_valid(ring));
                CHECK(junction_rings_nested(j));
            }
        }
        std::printf("[fillet census] %s: ring records over all tiles' builds %zu, junctions with fillets %zu, corners rounded %zu, corners left sharp %zu, "
            "hull fallbacks %zu (sharp %zu), clusters %zu (sharp %zu)\n", pbf, rings, with.filleted_junctions, with.fillets, with.fillet_fallbacks,
            with.hull_fallbacks, without.hull_fallbacks, with.clusters, without.clusters);
        CHECK(with.filleted_junctions * 10 >= rings * 7);           // most junctions get kerb returns
        CHECK(with.fillet_fallbacks * 100 <= with.fillets);         // and rarely have to leave a corner sharp
        CHECK(with.hull_fallbacks <= without.hull_fallbacks + 2);
    }
}
