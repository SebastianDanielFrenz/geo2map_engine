// G4 acceptance on the committed real OSM fixtures: A* equals the reference
// Dijkstra on many random node pairs, with the fixture's own turn
// restrictions applied. Prints graph sizes and timings (never asserted).
#include "route_reference.h"
#include "splitmix64.h"
#include "synthetic.h"

#include "g2m/decode/osm/pbf_reader.h"

#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>

using namespace g2m;
using namespace g2m::route;

namespace {

osm::OsmData read_with_relations(const std::string& path) {
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
            for (const auto& m : r.members) {
                const auto type = m.type == decode::osm::OsmMemberType::Node ? osm::MemberType::Node
                                  : m.type == decode::osm::OsmMemberType::Way ? osm::MemberType::Way
                                                                              : osm::MemberType::Relation;
                members.push_back({type, m.ref, m.role});
            }
            osm::OsmBuilder::TagList tags;
            for (const auto& t : r.tags) tags.emplace_back(t.key, t.value);
            builder.add_relation(r.id, std::move(members), std::move(tags));
        }
    } reader;
    decode::osm::read_pbf(path, reader, nullptr);
    auto data = reader.builder.build();
    REQUIRE(data);
    return std::move(*data);
}

double ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

} // namespace

TEST_CASE("A* equals Dijkstra on the real fixtures, with their turn restrictions", "[route][real-data][restriction]") {
    for (const char* name : {"frankfurt_hoechst", "hoechst_a66"}) {
        const auto data = read_with_relations(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + name + ".osm.pbf");
        const auto roads = build_road_graph(data, g2m_test::kZone32N);
        REQUIRE(roads);
        const auto restrictions = extract_turn_restrictions(data);
        REQUIRE(restrictions);
        auto t0 = std::chrono::steady_clock::now();
        const auto g = build_routing_graph(*roads, restrictions->restrictions);
        REQUIRE(g);
        const double build_ms = ms_since(t0);
        const auto& st = g->stats();
        std::printf("%s: road ways=%zu nodes=%zu | routing nodes=%zu arcs=%zu ways used=%zu not routable=%zu access=%zu "
                    "no-direction=%zu | restriction relations=%zu rules=%zu applied=%zu via-way=%zu unmatched=%zu "
                    "not-for-cars=%zu conditional=%zu malformed=%zu unknown=%zu | build %.1f ms\n",
                    name, roads->ways.size(), roads->nodes.size(), g->nodes().size(), g->arcs().size(), st.ways_used,
                    st.ways_not_routable, st.ways_access_blocked, st.ways_no_direction, restrictions->relations_seen,
                    restrictions->restrictions.size(), st.restrictions_applied, st.restrictions_via_way,
                    st.restrictions_unmatched, restrictions->not_for_cars, restrictions->conditional_only,
                    restrictions->malformed, restrictions->unknown_value, build_ms);
        CHECK(st.restrictions_applied > 0);

        // Negative control: the same graph without restrictions must route some query differently.
        const auto free = build_routing_graph(*roads, {});
        REQUIRE(free);
        RouteSearcher free_searcher(*free);
        std::size_t changed_by_restrictions = 0;
        RouteSearcher searcher(*g);
        g2m_test::SplitMix64 rng(0x6a4c0de);
        std::size_t reachable = 0, settled = 0;
        double astar_ms = 0, astar_max = 0, dijkstra_ms = 0;
        const int queries = 400;
        for (int q = 0; q < queries; ++q) {
            const auto from = g->nodes()[rng.next() % g->nodes().size()].osm_id;
            const auto to = g->nodes()[rng.next() % g->nodes().size()].osm_id;
            t0 = std::chrono::steady_clock::now();
            const auto r = searcher.find({from, to});
            const double ms = ms_since(t0);
            REQUIRE(r);
            astar_ms += ms;
            astar_max = std::max(astar_max, ms);
            settled += searcher.stats().settled;
            t0 = std::chrono::steady_clock::now();
            const auto ref = g2m_test::dijkstra_reference(*g, from, to);
            dijkstra_ms += ms_since(t0);
            REQUIRE(r->reachable == ref.reachable);
            REQUIRE(r->cost_ms == ref.cost_ms);
            REQUIRE(g2m_test::validate_route(*g, *r, from, to).empty());
            if (r->reachable) ++reachable;
            const auto unrestricted = free_searcher.find({from, to});
            REQUIRE(unrestricted);
            if (r->reachable) REQUIRE((unrestricted->reachable && unrestricted->cost_ms <= r->cost_ms));
            if (unrestricted->reachable != r->reachable || unrestricted->cost_ms != r->cost_ms) ++changed_by_restrictions;
        }
        std::printf("%s: %zu of %d queries change when the restrictions are dropped\n", name, changed_by_restrictions, queries);
        CHECK(changed_by_restrictions > 0);
        std::printf("%s: %d queries, %zu reachable; A* mean %.3f ms max %.3f ms settled mean %zu; reference Dijkstra mean %.3f ms\n",
                    name, queries, reachable, astar_ms / queries, astar_max, settled / queries, dijkstra_ms / queries);
        CHECK(reachable > queries / 4);
    }
}
