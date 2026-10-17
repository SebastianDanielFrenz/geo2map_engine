// G4 slice 3: turn restrictions carried in g2m.roads.graph (encoding 6). A routing region
// assembled from served tiles must apply the same restrictions as one built from the canonical
// OSM extract: via-node rules of the fixtures' own relations, and (on the fixtures' real
// geometry) via-way rules, which the fixtures do not contain, added as synthetic relations at
// the graph level. The roads.graph deriver itself is run on the 3 x 3 src.osm neighbourhood of
// every tile of both fixtures.
#include "route_fixture_io.h"
#include "route_reference.h"
#include "splitmix64.h"
#include "synthetic.h"
#include "via_reference.h"

#include "g2m/core/hash.h"
#include "g2m/layer/osm_clip.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/src_osm.h"
#include "g2m/route/region_graph.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdio>
#include <map>
#include <set>

using namespace g2m;
using namespace g2m::route;

namespace {

std::set<TileKey> keys_of(const RoadGraph& world) {
    std::set<TileKey> keys;
    for (const auto& n : world.nodes) {
        const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0,
                                   static_cast<double>(n.position.y) / 1000.0);
        REQUIRE(k);
        keys.insert(*k);
    }
    return keys;
}

// The rules a region built from every tile of `world` holds: via node or a via way is a road object.
std::vector<TurnRestriction> held_by(const RoadGraph& world, const std::vector<TurnRestriction>& all) {
    std::set<std::int64_t> nodes, ways;
    for (const auto& n : world.nodes) nodes.insert(n.osm_id);
    for (const auto& w : world.ways) ways.insert(w.osm_id);
    std::vector<TurnRestriction> out;
    for (const auto& r : all) {
        bool held = r.via_node != 0 && nodes.count(r.via_node);
        for (const auto w : r.via_ways) held = held || ways.count(w);
        if (held) out.push_back(r);
    }
    std::sort(out.begin(), out.end(), restriction_key_less);
    return out;
}

bool same_rules(const RoutingGraph& a, const RoutingGraph& b) {
    if (a.rules().size() != b.rules().size()) return false;
    for (std::size_t i = 0; i < a.rules().size(); ++i) {
        const auto &x = a.rules()[i], &y = b.rules()[i];
        if (x.from_way != y.from_way || x.to_way != y.to_way || x.kind != y.kind) return false;
    }
    return true;
}

// Both graphs route 300 random pairs alike (cost and arcs); returns how many pairs were reachable.
std::size_t compare_routes(const RoutingGraph& a, const RouteLandmarks* alm, const RoutingGraph& b, std::uint64_t seed) {
    RouteSearcher sa(a, alm), sb(b);
    g2m_test::SplitMix64 rng(seed);
    std::size_t reachable = 0;
    for (int q = 0; q < 300; ++q) {
        const auto from = a.nodes()[rng.next() % a.nodes().size()].osm_id;
        const auto to = a.nodes()[rng.next() % a.nodes().size()].osm_id;
        const auto ra = sa.find({from, to}), rb = sb.find({from, to});
        REQUIRE(ra); REQUIRE(rb);
        REQUIRE(ra->reachable == rb->reachable);
        REQUIRE(ra->cost_ms == rb->cost_ms);
        REQUIRE(ra->arcs == rb->arcs);
        reachable += ra->reachable ? 1 : 0;
    }
    return reachable;
}

// Number of random pairs on which two graphs disagree (reachability or cost).
std::size_t differing_pairs(const RoutingGraph& a, const RoutingGraph& b, std::uint64_t seed) {
    RouteSearcher sa(a), sb(b);
    g2m_test::SplitMix64 rng(seed);
    std::size_t differ = 0;
    for (int q = 0; q < 400; ++q) {
        const auto from = a.nodes()[rng.next() % a.nodes().size()].osm_id;
        const auto to = a.nodes()[rng.next() % a.nodes().size()].osm_id;
        const auto ra = sa.find({from, to}), rb = sb.find({from, to});
        REQUIRE(ra); REQUIRE(rb);
        if (ra->reachable != rb->reachable || ra->cost_ms != rb->cost_ms) ++differ;
    }
    return differ;
}

Result<std::optional<RoadGraphTile>> serve(const std::map<TileKey, TileBody>& tiles, const TileKey& key) {
    const auto it = tiles.find(key);
    if (it == tiles.end()) return std::optional<RoadGraphTile>{};
    auto decoded = decode_road_graph(it->second);
    if (!decoded) return decoded.error();
    return std::optional<RoadGraphTile>{std::move(*decoded)};
}

} // namespace

TEST_CASE("A region from encoding-6 tiles routes like the whole OSM extract, via-node and via-way rules", "[route][region][restriction][real-data]") {
    for (const char* name : {"frankfurt_hoechst", "hoechst_a66"}) {
        const auto data = g2m_test::read_route_fixture(name);
        const auto world = build_road_graph(data, g2m_test::kZone32N);
        REQUIRE(world);
        const auto whole = extract_turn_restrictions(data);
        REQUIRE(whole);
        // The fixtures' own relations have no via ways: add some on the real geometry.
        auto all = whole->restrictions;
        g2m_test::add_via_way_rules(*world, all, 0x51ce3, 0.15, 2000000000);
        std::sort(all.begin(), all.end(), restriction_key_less);
        std::size_t via_ways = 0;
        for (const auto& r : all) via_ways += r.via_node == 0 ? 1 : 0;
        REQUIRE(via_ways > 5);

        // Served tiles through the real codec, fetched in a shuffled order with repeats and a miss.
        std::map<TileKey, TileBody> served;
        const auto keys = keys_of(*world);
        std::size_t tile_rules = 0;
        for (const auto& key : keys) {
            const auto tile = select_road_graph_tile(*world, key, all);
            REQUIRE(tile);
            tile_rules += tile->restrictions.size();
            const auto body = encode_road_graph(*tile);
            REQUIRE(body);
            CHECK(body->encoding == kRoadGraphEncoding);
            served.emplace(key, *body);
        }
        std::vector<TileKey> request(keys.rbegin(), keys.rend());
        request.push_back(*keys.begin());
        TileKey outside = *keys.begin();
        outside.x += 60;
        request.push_back(outside);
        RegionGraphStats stats;
        const auto region = assemble_region(request, [&](const TileKey& k) { return serve(served, k); }, &stats);
        REQUIRE(region);
        CHECK(region->graph.nodes == world->nodes);
        CHECK(region->graph.ways == world->ways);
        CHECK(region->restrictions == held_by(*world, all));
        CHECK(stats.restriction_copies == tile_rules);
        CHECK(stats.restrictions == region->restrictions.size());
        CHECK(stats.restriction_copies > stats.restrictions); // neighbouring tiles repeat rules
        std::printf("%s: %zu tiles, %zu rules (%zu via-way) -> %zu rule copies in tiles -> %zu in the region\n", name, keys.size(),
                    all.size(), via_ways, stats.restriction_copies, stats.restrictions);

        const auto built = build_region_routing(region->graph, region->restrictions);
        REQUIRE(built);
        const auto direct = build_routing_graph(*world, all);
        REQUIRE(direct);
        const RoutingGraph& g = *built->graph;
        CHECK(g.arcs().size() == direct->arcs().size());
        CHECK(g.state_count() == direct->state_count());
        CHECK(same_rules(g, *direct));
        CHECK(g.stats().restrictions_applied == direct->stats().restrictions_applied);
        CHECK(g.stats().restrictions_via_way_applied == direct->stats().restrictions_via_way_applied);
        CHECK(g.stats().via_way_instances == direct->stats().via_way_instances);
        CHECK(g.stats().via_way_states == direct->stats().via_way_states);
        CHECK(g.stats().restrictions_applied > 0);
        CHECK(g.stats().restrictions_via_way_applied > 0);
        const std::size_t reachable = compare_routes(g, built->landmarks.get(), *direct, 0xbeef);
        CHECK(reachable > 75);

        // The history reference agrees with the library on the real geometry, via-way rules included.
        const g2m_test::ViaReference ref(*world, all);
        RouteSearcher searcher(g, built->landmarks.get());
        g2m_test::SplitMix64 rng(0x7e57);
        std::size_t checked = 0, broken_without = 0;
        const auto without_via = [&] {
            std::vector<TurnRestriction> only_node;
            for (const auto& r : region->restrictions) if (r.via_node != 0) only_node.push_back(r);
            auto graph = build_routing_graph(region->graph, only_node);
            REQUIRE(graph);
            return std::move(*graph);
        }();
        RouteSearcher lax(without_via);
        for (int q = 0; q < 120; ++q) {
            const auto from = g.nodes()[rng.next() % g.nodes().size()].osm_id;
            const auto to = g.nodes()[rng.next() % g.nodes().size()].osm_id;
            const auto r = searcher.find({from, to});
            REQUIRE(r);
            const Route expect = g2m_test::route_via_reference(g, ref, from, to);
            REQUIRE(r->reachable == expect.reachable);
            REQUIRE(r->cost_ms == expect.cost_ms);
            ++checked;
            if (!r->reachable) continue;
            CHECK_FALSE(g2m_test::route_violates(g, ref, *r));
            const auto l = lax.find({from, to});
            REQUIRE(l);
            if (g2m_test::route_violates(g, ref, *l)) ++broken_without;
        }
        std::printf("%s: %zu reference queries agree; %zu routes of the graph without the via-way rules break one\n", name, checked, broken_without);
        // Negative controls: dropping all restrictions, or only the via-way ones, changes routes.
        const auto free = build_routing_graph(region->graph, {});
        REQUIRE(free);
        const std::size_t no_rules = differing_pairs(g, *free, 0xc0de), no_via = differing_pairs(g, without_via, 0xc0de);
        std::printf("%s: of 400 pairs, %zu route differently without any rule, %zu without the via-way rules\n", name, no_rules, no_via);
        CHECK(no_rules > 0);
        CHECK(no_via > 0);
        CHECK(broken_without > 0);
    }
}

TEST_CASE("The roads.graph deriver on the OSM fixtures carries the restrictions of the OSM relations", "[route][region][restriction][real-data]") {
    for (const char* name : {"frankfurt_hoechst", "hoechst_a66"}) {
        const auto data = g2m_test::read_route_fixture(name);
        const auto world = build_road_graph(data, g2m_test::kZone32N);
        REQUIRE(world);
        const auto whole = extract_turn_restrictions(data);
        REQUIRE(whole);
        RoadGraphDeriver deriver;
        std::map<TileKey, InputTile> sources; // src.osm tiles cut from the whole file, once each
        const auto source = [&](const InputTileRef& ref) -> const InputTile& {
            auto it = sources.find(ref.key);
            if (it != sources.end()) return it->second;
            const auto clipped = clip_osm(data, ref.key);
            REQUIRE(clipped);
            const auto tile_body = encode_src_osm(*clipped);
            REQUIRE(tile_body);
            const auto bytes = encode_body(*tile_body);
            REQUIRE(bytes);
            TileHeader header;
            header.layer = ref.layer; header.layer_version = 1; header.key = ref.key;
            header.body_size = bytes->size(); header.body_hash = content_hash(*bytes);
            return sources.emplace(ref.key, InputTile{ref, header, *bytes}).first->second;
        };
        std::map<TileKey, TileBody> served;
        std::size_t rules_in_tiles = 0, equal_to_selected = 0;
        const auto keys = keys_of(*world);
        for (const auto& key : keys) {
            std::vector<InputTile> inputs;
            for (const auto& ref : deriver.inputs_for(key)) inputs.push_back(source(ref));
            const auto derived = deriver.derive({key, 7, inputs});
            REQUIRE(derived);
            const auto body = decode_body(derived->body);
            REQUIRE(body);
            CHECK(body->encoding == kRoadGraphEncoding);
            const auto tile = decode_road_graph(*body);
            REQUIRE(tile);
            const auto selected = select_road_graph_tile(*world, key, whole->restrictions);
            REQUIRE(selected);
            if (*tile == *selected) ++equal_to_selected;
            CHECK(*tile == *selected);
            rules_in_tiles += tile->restrictions.size();
            served.emplace(key, *body);
        }
        CHECK(equal_to_selected == keys.size());
        CHECK(rules_in_tiles > 0);
        std::vector<TileKey> request(keys.begin(), keys.end());
        RegionGraphStats stats;
        const auto region = assemble_region(request, [&](const TileKey& k) { return serve(served, k); }, &stats);
        REQUIRE(region);
        CHECK(region->restrictions == held_by(*world, whole->restrictions));
        const auto built = build_region_routing(region->graph, region->restrictions);
        REQUIRE(built);
        const auto direct = build_routing_graph(*world, whole->restrictions);
        REQUIRE(direct);
        CHECK(same_rules(*built->graph, *direct));
        CHECK(built->graph->stats().restrictions_applied == direct->stats().restrictions_applied);
        CHECK(built->graph->stats().restrictions_applied > 0);
        const std::size_t reachable = compare_routes(*built->graph, built->landmarks.get(), *direct, 0xd0d0);
        // Negative control: a region assembled from the same tiles with the restrictions dropped routes differently.
        const auto dropped = build_region_routing(region->graph, {});
        REQUIRE(dropped);
        const std::size_t differ = differing_pairs(*built->graph, *dropped->graph, 0xd0d0);
        std::printf("%s: %zu derived tiles hold %zu rules; region %zu rules (%zu applied), %zu of 300 pairs reachable, %zu of 400 differ without them\n",
                    name, keys.size(), rules_in_tiles, region->restrictions.size(), built->graph->stats().restrictions_applied, reachable, differ);
        CHECK(differ > 0);
    }
}
