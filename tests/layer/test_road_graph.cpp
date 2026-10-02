#include "g2m/layer/road_graph.h"
#include "synthetic.h"
#include "g2m/core/hash.h"
#include "g2m/decode/osm/pbf_reader.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cstdio>

using namespace g2m;
namespace {
void node(osm::OsmBuilder& builder, std::int64_t id, double x, double y) {
    const geo::Utm utm(geo::kGRS80);
    const auto ll = utm.reverse(g2m_test::kZone32N, x, y);
    builder.add_node(id, g2m_test::to_e7(ll.lat), g2m_test::to_e7(ll.lon));
}
osm::OsmData fixture(bool reverse) {
    osm::OsmBuilder builder;
    for (int i = 0; i < 6; ++i) {
        const int id = reverse ? 6 - i : i + 1;
        const double x[] = {358900, 358950, 359000, 358950, 358900, 359000};
        const double y[] = {5581300, 5581300, 5581300, 5581350, 5581250, 5581350};
        node(builder, id, x[id - 1], y[id - 1]);
    }
    const auto add = [&](int which) {
        if (which == 0) builder.add_way(10, {1, 2, 3}, {{"highway", "residential"}, {"lanes", "2"}, {"oneway", "-1"}});
        if (which == 1) builder.add_way(20, {2, 4}, {{"highway", "service"}, {"incline", "8%"}});
        if (which == 2) builder.add_way(30, {5, 6}, {{"highway", "primary"}, {"bridge", "yes"}, {"layer", "1"}, {"width", "7"}});
        if (which == 3) builder.add_way(40, {3, 4}, {{"highway", "secondary"}, {"tunnel", "yes"}, {"surface", "concrete"}});
    };
    for (int i = 0; i < 4; ++i) add(reverse ? 3 - i : i);
    const auto data = builder.build();
    REQUIRE(data);
    return *data;
}
}

TEST_CASE("Road graph preserves junction ids and grade-separated crossings", "[layer][road_graph]") {
    const auto graph = build_road_graph(fixture(false), g2m_test::kZone32N);
    REQUIRE(graph);
    REQUIRE(graph->nodes.size() == 6);
    REQUIRE(graph->ways.size() == 4);
    REQUIRE(graph->edges.size() == 5);
    // Node 2 joins three edge ends; crossing the bridge's geometry does not
    // invent an extra node or connect its refs 5/6 to the ground road.
    CHECK(std::count_if(graph->edges.begin(), graph->edges.end(), [](const auto& e) {
        return e.from_node == 2 || e.to_node == 2;
    }) == 3);
    CHECK(graph->edges[3] == RoadGraphEdge{30, 0, 5, 6});
    CHECK(graph->ways[2].tags == std::vector<std::pair<std::string, std::string>>{
        {"bridge", "yes"}, {"highway", "primary"}, {"layer", "1"}, {"width", "7"}});
    CHECK(graph->ways[3].tags == std::vector<std::pair<std::string, std::string>>{
        {"highway", "secondary"}, {"surface", "concrete"}, {"tunnel", "yes"}});
    const auto shuffled = build_road_graph(fixture(true), g2m_test::kZone32N);
    REQUIRE(shuffled);
    CHECK(*shuffled == *graph);
}

TEST_CASE("Road edge identity survives different neighbouring ways", "[layer][road_graph]") {
    osm::OsmBuilder builder;
    node(builder, 1, 358900, 5581300); node(builder, 2, 358950, 5581300); node(builder, 3, 359000, 5581300);
    builder.add_way(10, {1, 2, 3}, {{"highway", "residential"}, {"lanes", "2"}, {"oneway", "-1"}});
    const auto data = builder.build(); REQUIRE(data);
    const auto local = build_road_graph(*data, g2m_test::kZone32N); REQUIRE(local);
    const auto wider = build_road_graph(fixture(false), g2m_test::kZone32N); REQUIRE(wider);
    REQUIRE(local->edges.size() == 2);
    CHECK(local->edges[0] == wider->edges[0]); CHECK(local->edges[1] == wider->edges[1]);
    CHECK(local->ways[0] == wider->ways[0]);
}

TEST_CASE("Road graph retains closed ways and stable indices past repeated refs", "[layer][road_graph]") {
    osm::OsmBuilder builder;
    node(builder, 1, 358900, 5581300); node(builder, 2, 358950, 5581300); node(builder, 3, 359000, 5581350);
    builder.add_way(10, {1, 2, 2, 3, 1}, {{"highway", "residential"}, {"junction", "roundabout"}});
    builder.add_way(20, {1, 3}, {{"highway", "footway"}});
    builder.add_way(30, {1, 3}, {{"highway", "residential"}, {"area", "yes"}});
    const auto data = builder.build(); REQUIRE(data);
    const auto graph = build_road_graph(*data, g2m_test::kZone32N); REQUIRE(graph);
    CHECK(graph->ways.size() == 1); CHECK(graph->repeated_ref_pairs == 1);
    CHECK(graph->edges == std::vector<RoadGraphEdge>{{10, 0, 1, 2}, {10, 2, 2, 3}, {10, 3, 3, 1}});
}

TEST_CASE("Road graph fails malformed inputs and rejects far ways as a whole", "[layer][road_graph]") {
    auto data = fixture(false);
    data.way_refs[0] = 999;
    CHECK_FALSE(build_road_graph(data, g2m_test::kZone32N));
    CHECK_FALSE(build_road_graph(fixture(false), {0, geo::Hemisphere::North}));
    osm::OsmBuilder builder;
    node(builder, 1, 358900, 5581300);
    builder.add_node(2, 500000000, 1200000000);
    builder.add_way(10, {1, 2}, {{"highway", "primary"}});
    const auto input = builder.build(); REQUIRE(input);
    const auto graph = build_road_graph(*input, g2m_test::kZone32N); REQUIRE(graph);
    CHECK(graph->far_zone_ways == 1); CHECK(graph->nodes.empty()); CHECK(graph->ways.empty()); CHECK(graph->edges.empty());
}

TEST_CASE("Road graph real fixture retains complete source identity", "[layer][road_graph][real-data]") {
    struct Reader final : decode::osm::PbfHandler {
        osm::OsmBuilder builder;
        void on_node(const decode::osm::OsmNode& n) override {
            osm::OsmBuilder::TagList tags;
            for (const auto& tag : n.tags) tags.emplace_back(tag.key, tag.value);
            builder.add_node(n.id, n.lat_1e7, n.lon_1e7, std::move(tags));
        }
        void on_way(const decode::osm::OsmWay& w) override {
            osm::OsmBuilder::TagList tags;
            for (const auto& tag : w.tags) tags.emplace_back(tag.key, tag.value);
            builder.add_way(w.id, w.node_refs, std::move(tags));
        }
    } reader;
    const auto stats = decode::osm::read_pbf(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/frankfurt_hoechst.osm.pbf",
        reader, nullptr, decode::osm::PbfEntities::Nodes | decode::osm::PbfEntities::Ways);
    REQUIRE(stats.way_count == 5073);
    const auto data = reader.builder.build(); REQUIRE(data);
    const auto graph = build_road_graph(*data, g2m_test::kZone32N); REQUIRE(graph);
    CHECK(graph->far_zone_ways == 0);
    REQUIRE(graph->ways.size() > 100);
    Fnv1a64 hash;
    hash.u64(graph->nodes.size());
    for (const auto& n : graph->nodes) hash.i64(n.osm_id).i64(n.position.x).i64(n.position.y);
    hash.u64(graph->ways.size());
    for (const auto& w : graph->ways) {
        hash.i64(w.osm_id).i32(w.rank).u64(w.node_ids.size());
        for (const auto id : w.node_ids) hash.i64(id);
        hash.u64(w.tags.size());
        for (const auto& [key, value] : w.tags) hash.str(key).str(value);
        const auto* original = data->find_way(w.osm_id); REQUIRE(original);
        CHECK(w.node_ids == std::vector<std::int64_t>(data->refs_of(*original).begin(), data->refs_of(*original).end()));
    }
    hash.u64(graph->edges.size());
    for (const auto& e : graph->edges) hash.i64(e.way_id).u32(e.segment_index).i64(e.from_node).i64(e.to_node);
    CHECK(hash.value() == 0x2d63f916719a0c3bull);
    std::printf("G3 fixture: nodes=%zu ways=%zu edges=%zu fingerprint=%s\n", graph->nodes.size(), graph->ways.size(),
        graph->edges.size(), to_hex(hash.value()).c_str());
}
