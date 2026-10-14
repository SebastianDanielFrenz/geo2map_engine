// G4 slice 2: ALT landmarks, heading-aware snapping, point-to-point routes,
// and the region graph assembled from roads.graph tiles.
#include "route_reference.h"
#include "route_scenes.h"
#include "splitmix64.h"
#include "synthetic.h"

#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/layer/osm_clip.h"
#include "g2m/route/region_graph.h"

#include <catch2/catch_test_macros.hpp>

#include <cstdio>
#include <map>
#include <set>
#include <string>

using namespace g2m;
using namespace g2m::route;
using g2m_test::cross;
using g2m_test::via_node_rule;

namespace {

RoutePoint snap_ok(const RoutingGraph& g, std::int64_t x_m, std::int64_t y_m, double hx = 0, double hy = 0) {
    SnapOptions o;
    o.heading_dx = hx;
    o.heading_dy = hy;
    auto p = g.snap({x_m * 1000, y_m * 1000}, o);
    REQUIRE(p);
    return *p;
}

Route point_route(const RoutingGraph& g, const RouteLandmarks* lm, const RoutePoint& a, const RoutePoint& b) {
    RouteSearcher searcher(g, lm);
    auto r = searcher.find(a, b);
    REQUIRE(r);
    CHECK(g2m_test::validate_point_route(g, *r, a, b).empty());
    const auto ref = g2m_test::dijkstra_point_reference(g, a, b);
    CHECK(ref.reachable == r->reachable);
    CHECK(ref.cost_ms == r->cost_ms);
    return *r;
}

} // namespace

TEST_CASE("Snapping: nearest edge point, heading picks the direction", "[route][snap]") {
    const auto g = build_routing_graph(cross(), {});
    REQUIRE(g);
    const auto arcs = g->arcs();
    {
        const auto p = snap_ok(*g, 30, -50); // beside way 10 (S -> C runs north along x = 0)
        CHECK(p.position == geom::PointMm{0, -50000});
        CHECK(p.distance_mm == 30000);
        CHECK_FALSE(p.heading_matched);
        REQUIRE(p.count == 2);
        for (int i = 0; i < 2; ++i) {
            CHECK(arcs[p.candidates[i].arc].way_id == 10);
            CHECK(p.candidates[i].offset_mm == 50000);
            CHECK(g->point_at(p.candidates[i].arc, p.candidates[i].offset_mm) == p.position);
        }
    }
    {
        const auto north = snap_ok(*g, 30, -70, 0, 1);
        CHECK(north.heading_matched);
        REQUIRE(north.count == 1);
        CHECK(arcs[north.candidates[0].arc].forward); // refs 2 -> 1 run north
        CHECK(north.candidates[0].offset_mm == 30000);
        const auto south = snap_ok(*g, 30, -70, 0.2, -1);
        REQUIRE(south.count == 1);
        CHECK_FALSE(arcs[south.candidates[0].arc].forward);
        CHECK(south.candidates[0].offset_mm == 70000);
        // Heading east is 90 degrees off both directions: nearest edge, both directions.
        const auto east = snap_ok(*g, 30, -70, 1, 0);
        CHECK_FALSE(east.heading_matched);
        CHECK(east.count == 2);
    }
    {
        // A heading may prefer a farther edge: 10 m from way 10 (north-south), 12 m from way 40 (east-west).
        const auto p = snap_ok(*g, 10, 12, 1, 0);
        CHECK(p.heading_matched);
        REQUIRE(p.count == 1);
        CHECK(arcs[p.candidates[0].arc].way_id == 40);
        CHECK(p.position == geom::PointMm{10000, 0});
    }
    {
        // One-way against the heading: nothing qualifies, so the one traversable direction is offered.
        const auto oneway = build_routing_graph(cross(false, 100, {{"oneway", "yes"}}), {});
        REQUIRE(oneway);
        const auto p = snap_ok(*oneway, 5, 60, 0, -1);
        CHECK_FALSE(p.heading_matched);
        REQUIRE(p.count == 1);
        CHECK(oneway->arcs()[p.candidates[0].arc].way_id == 30);
    }
    const auto far = g->snap({0, -200000});
    REQUIRE_FALSE(far);
    CHECK(far.error().code == ErrorCode::NotFound);
}

TEST_CASE("Point routes: partial arcs, same arc ahead and behind, turns", "[route][snap]") {
    const auto g = build_routing_graph(cross(), {});
    REQUIRE(g);
    {
        // 50 m north on way 10, left onto way 20, 50 m west: 6000 + 4000 + 6000 ms.
        const auto r = point_route(*g, nullptr, snap_ok(*g, 0, -50, 0, 1), snap_ok(*g, -50, 0));
        REQUIRE(r.reachable);
        CHECK(r.cost_ms == 16000);
        CHECK(r.length_mm == 100000);
        CHECK(r.arcs.size() == 2);
    }
    {
        const auto r = point_route(*g, nullptr, snap_ok(*g, 0, -80, 0, 1), snap_ok(*g, 0, -20));
        REQUIRE(r.arcs.size() == 1);
        CHECK(r.cost_ms == 7200); // 60 m at 30 km/h
        CHECK(r.length_mm == 60000);
        CHECK(r.start_offset_mm == 20000);
        CHECK(r.end_offset_mm == 80000);
    }
    {
        // The destination is behind the car on the same edge: around the block, back down way 10.
        const auto r = point_route(*g, nullptr, snap_ok(*g, 0, -20, 0, 1), snap_ok(*g, 0, -80));
        REQUIRE(r.reachable);
        CHECK(r.arcs.size() >= 4);
        CHECK(g->arcs()[r.arcs.back()].way_id == 10);
        CHECK_FALSE(g->arcs()[r.arcs.back()].forward);
    }
    {
        // Restrictions still apply from a mid-edge start.
        const auto restricted = build_routing_graph(cross(), std::vector<TurnRestriction>{via_node_rule(10, 1, 20, "no_left_turn")});
        REQUIRE(restricted);
        const auto r = point_route(*restricted, nullptr, snap_ok(*restricted, 0, -50, 0, 1), snap_ok(*restricted, -50, 0));
        REQUIRE(r.reachable);
        CHECK(r.cost_ms > 16000);
    }
}

TEST_CASE("ALT landmarks: deterministic, admissible, A* + ALT equals Dijkstra", "[route][alt]") {
    std::size_t queries = 0, saved = 0;
    for (std::uint64_t seed = 11; seed <= 14; ++seed) {
        const auto scene = g2m_test::random_scene(seed, 16, 0.3);
        const auto g = build_routing_graph(scene.graph, scene.rules);
        REQUIRE(g);
        const auto lm = RouteLandmarks::build(*g, 8);
        REQUIRE(lm);
        const auto again = RouteLandmarks::build(*g, 8);
        REQUIRE(again);
        CHECK(std::vector<std::uint32_t>(lm->landmark_nodes().begin(), lm->landmark_nodes().end()) ==
              std::vector<std::uint32_t>(again->landmark_nodes().begin(), again->landmark_nodes().end()));
        CHECK(std::set<std::uint32_t>(lm->landmark_nodes().begin(), lm->landmark_nodes().end()).size() == 8);
        RouteSearcher plain(*g), alt(*g, &*lm);
        g2m_test::SplitMix64 rng(seed * 31);
        const auto nodes = g->nodes();
        for (int q = 0; q < 100; ++q) {
            const auto from = nodes[rng.next() % nodes.size()].osm_id;
            const auto to = nodes[rng.next() % nodes.size()].osm_id;
            const auto a = alt.find({from, to});
            REQUIRE(a);
            const auto p = plain.find({from, to});
            REQUIRE(p);
            const auto ref = g2m_test::dijkstra_reference(*g, from, to);
            REQUIRE(a->reachable == ref.reachable);
            REQUIRE(a->cost_ms == ref.cost_ms);
            REQUIRE(g2m_test::validate_route(*g, *a, from, to).empty());
            // Admissible: the bound never exceeds the node-graph cost of the route found.
            if (a->reachable && !a->arcs.empty())
                REQUIRE(lm->bound(g->find_node(from), g->find_node(to)) <= a->cost_ms);
            if (alt.stats().settled < plain.stats().settled) ++saved;
            ++queries;
            // Point queries between random edge points, with and without a heading.
            const auto pa = g->snap({static_cast<std::int64_t>(rng.next() % 3000000), static_cast<std::int64_t>(rng.next() % 3000000)},
                                    SnapOptions{200000, rng.next() % 2 ? 1.0 : 0.0, rng.next() % 2 ? 1.0 : 0.0, 60.0});
            const auto pb = g->snap({static_cast<std::int64_t>(rng.next() % 3000000), static_cast<std::int64_t>(rng.next() % 3000000)},
                                    SnapOptions{200000});
            if (!pa || !pb) continue;
            const auto ra = alt.find(*pa, *pb);
            REQUIRE(ra);
            const auto rr = g2m_test::dijkstra_point_reference(*g, *pa, *pb);
            REQUIRE(ra->reachable == rr.reachable);
            REQUIRE(ra->cost_ms == rr.cost_ms);
            REQUIRE(g2m_test::validate_point_route(*g, *ra, *pa, *pb).empty());
        }
    }
    std::printf("ALT random grids: %zu node queries, %zu settled fewer states than plain A*\n", queries, saved);
    CHECK(saved > queries / 2);
    const auto g = build_routing_graph(cross(), {});
    REQUIRE(g);
    CHECK_FALSE(RouteLandmarks::build(*g, 0));
    CHECK_FALSE(RouteLandmarks::build(*g, 65));
    const auto other = build_routing_graph(cross(true), {});
    REQUIRE(other);
    const auto lm = RouteLandmarks::build(*other, 2);
    REQUIRE(lm);
    RouteSearcher wrong(*g, &*lm);
    CHECK_FALSE(wrong.find({2, 4}));
}

namespace {

osm::OsmData fixture_osm(const std::string& name) {
    struct Reader final : decode::osm::PbfHandler {
        osm::OsmBuilder builder;
        void on_node(const decode::osm::OsmNode& n) override {
            osm::OsmBuilder::TagList tags;
            for (const auto& t : n.tags) tags.emplace_back(t.key, t.value);
            builder.add_node(n.id, n.lat_1e7, n.lon_1e7, std::move(tags));
        }
        void on_way(const decode::osm::OsmWay& w) override {
            osm::OsmBuilder::TagList tags;
            for (const auto& t : w.tags) tags.emplace_back(t.key, t.value);
            builder.add_way(w.id, w.node_refs, std::move(tags));
        }
        void on_relation(const decode::osm::OsmRelation& r) override {
            std::vector<osm::OsmBuilder::MemberIn> members;
            for (const auto& m : r.members)
                members.push_back({m.type == decode::osm::OsmMemberType::Node ? osm::MemberType::Node
                                   : m.type == decode::osm::OsmMemberType::Way ? osm::MemberType::Way
                                                                               : osm::MemberType::Relation,
                                   m.ref, m.role});
            osm::OsmBuilder::TagList tags;
            for (const auto& t : r.tags) tags.emplace_back(t.key, t.value);
            builder.add_relation(r.id, std::move(members), std::move(tags));
        }
    } reader;
    decode::osm::read_pbf(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + name + ".osm.pbf", reader, nullptr);
    auto data = reader.builder.build();
    REQUIRE(data);
    return std::move(*data);
}

} // namespace

TEST_CASE("Region graph from roads.graph tiles equals the whole-world graph", "[route][region][real-data]") {
    const auto data = fixture_osm("hoechst_a66");
    const auto world = build_road_graph(data, g2m_test::kZone32N);
    REQUIRE(world);
    // Every level-2 tile holding a road node, served through the real codec.
    std::set<TileKey> keys;
    for (const auto& n : world->nodes) {
        const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0,
                                   static_cast<double>(n.position.y) / 1000.0);
        REQUIRE(k);
        keys.insert(*k);
    }
    std::map<TileKey, TileBody> served;
    std::vector<RestrictionSet> per_tile_restrictions;
    for (const auto& key : keys) {
        const auto tile = select_road_graph_tile(*world, key);
        REQUIRE(tile);
        const auto body = encode_road_graph(*tile);
        REQUIRE(body);
        served.emplace(key, *body);
        // The src.osm tile of the same key carries the restriction relations.
        const auto osm_tile = clip_osm(data, key);
        REQUIRE(osm_tile);
        const auto r = extract_turn_restrictions(osm_tile->data);
        REQUIRE(r);
        per_tile_restrictions.push_back(*r);
    }
    std::size_t fetches = 0;
    const RoadGraphTileFetch fetch = [&](const TileKey& key) -> Result<std::optional<RoadGraphTile>> {
        ++fetches;
        const auto it = served.find(key);
        if (it == served.end()) return std::optional<RoadGraphTile>{};
        auto decoded = decode_road_graph(it->second);
        if (!decoded) return decoded.error();
        return std::optional<RoadGraphTile>{std::move(*decoded)};
    };
    // Keys in reverse order, repeated, plus one key outside the data.
    std::vector<TileKey> request(keys.rbegin(), keys.rend());
    request.push_back(*keys.begin());
    TileKey outside = *keys.begin();
    outside.x += 50;
    request.push_back(outside);
    RegionGraphStats stats;
    const auto region = assemble_region_graph(request, fetch, &stats);
    REQUIRE(region);
    CHECK(fetches == keys.size() + 1);
    CHECK(stats.tiles_present == keys.size());
    CHECK(stats.tiles_absent == 1);
    CHECK(stats.way_copies > stats.ways); // neighbouring tiles repeat ways
    CHECK(region->nodes == world->nodes);
    CHECK(region->ways == world->ways);
    CHECK(region->edges == world->edges);

    const auto restrictions = merge_turn_restrictions(per_tile_restrictions);
    const auto whole = extract_turn_restrictions(data);
    REQUIRE(whole);
    CHECK(restrictions == whole->restrictions);

    const auto built = build_region_routing(*region, restrictions);
    REQUIRE(built);
    REQUIRE(built->landmarks);
    CHECK(built->landmarks->count() == 16);
    const auto direct = build_routing_graph(*world, whole->restrictions);
    REQUIRE(direct);
    REQUIRE(built->graph->arcs().size() == direct->arcs().size());
    for (std::size_t i = 0; i < direct->arcs().size(); ++i) {
        CHECK(built->graph->arcs()[i].way_id == direct->arcs()[i].way_id);
        CHECK(built->graph->arcs()[i].cost_ms == direct->arcs()[i].cost_ms);
    }
    std::printf("region assembly: %zu tiles, %zu way copies -> %zu ways, %zu node copies -> %zu nodes, %zu restrictions\n",
                stats.tiles_present, stats.way_copies, stats.ways, stats.node_copies, stats.nodes, restrictions.size());

    // A conflicting copy of a way is rejected.
    RoadGraph a = *world, b = *world;
    b.ways.front().tags.push_back({"zz", "1"});
    const std::vector<const RoadGraph*> graphs{&a, &b};
    const auto conflict = merge_region_graphs(graphs);
    REQUIRE_FALSE(conflict);
    CHECK(conflict.error().code == ErrorCode::Malformed);
}

TEST_CASE("Point routes with ALT on the real fixtures equal Dijkstra", "[route][alt][snap][real-data]") {
    for (const char* name : {"frankfurt_hoechst", "hoechst_a66"}) {
        const auto data = fixture_osm(name);
        const auto roads = build_road_graph(data, g2m_test::kZone32N);
        REQUIRE(roads);
        const auto restrictions = extract_turn_restrictions(data);
        REQUIRE(restrictions);
        const auto built = build_region_routing(*roads, restrictions->restrictions);
        REQUIRE(built);
        const RoutingGraph& g = *built->graph;
        RouteSearcher searcher(g, built->landmarks.get());
        g2m_test::SplitMix64 rng(0xa17);
        std::size_t done = 0, reachable = 0, headed = 0;
        for (int q = 0; q < 300; ++q) {
            // Near a random graph node (+-40 m), so most points snap within 50 m.
            const auto pick = [&]() {
                const auto& n = g.nodes()[rng.next() % g.nodes().size()];
                const auto dx = static_cast<std::int64_t>(rng.next() % 80001) - 40000;
                const auto dy = static_cast<std::int64_t>(rng.next() % 80001) - 40000;
                return geom::PointMm{n.position.x + dx, n.position.y + dy};
            };
            SnapOptions with_heading;
            with_heading.heading_dx = static_cast<double>(rng.next() % 2001) - 1000.0;
            with_heading.heading_dy = static_cast<double>(rng.next() % 2001) - 1000.0;
            const auto a = g.snap(pick(), with_heading);
            const auto b = g.snap(pick());
            if (!a || !b) continue;
            const auto r = searcher.find(*a, *b);
            REQUIRE(r);
            const auto ref = g2m_test::dijkstra_point_reference(g, *a, *b);
            REQUIRE(r->reachable == ref.reachable);
            REQUIRE(r->cost_ms == ref.cost_ms);
            REQUIRE(g2m_test::validate_point_route(g, *r, *a, *b).empty());
            ++done;
            reachable += r->reachable ? 1 : 0;
            headed += a->heading_matched ? 1 : 0;
        }
        std::printf("%s: %zu point queries (%zu heading-matched starts), %zu reachable\n", name, done, headed, reachable);
        CHECK(done > 100);
    }
}
