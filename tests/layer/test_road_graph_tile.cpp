#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/src_osm.h"
#include "g2m/core/bytes.h"
#include "g2m/core/hash.h"
#include "synthetic.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <map>
#include <limits>
#include "g2m/import/bake.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/upstream.h"
#include <chrono>
#include <filesystem>

using namespace g2m;
namespace {
RoadGraph graph(const TileKey& key) {
    const auto x = key.min_easting() * 1000, y = key.min_northing() * 1000;
    const auto size = key.size_m() * 1000;
    RoadGraph result;
    result.nodes = {{1, {x + size - 100000, y + 400000}}, {2, {x + size + 100000, y + 400000}},
        {3, {x + size + 300000, y + 800000}}, {4, {x + 200000, y + 400000}}};
    result.ways = {{10, 3, {1, 2, 3}, {{"bridge", "yes"}, {"highway", "residential"}, {"lanes", "2"}}},
        {20, 1, {4, 1}, {{"highway", "service"}}}};
    result.edges = {{10, 0, 1, 2}, {10, 1, 2, 3}, {20, 0, 4, 1}};
    return result;
}
osm::OsmData raw(const TileKey& key, bool conflict = false, bool relation_conflict = false) {
    const auto source = graph(key);
    osm::OsmBuilder builder;
    const geo::Utm utm(geo::kGRS80);
    for (const auto& n : source.nodes) {
        const auto ll = utm.reverse(key.zone, n.position.x / 1000.0, n.position.y / 1000.0);
        builder.add_node(n.osm_id, g2m_test::to_e7(ll.lat), g2m_test::to_e7(ll.lon));
    }
    for (const auto& w : source.ways) {
        auto tags = w.tags;
        if (conflict && w.osm_id == 10) tags.emplace_back("width", "12");
        builder.add_way(w.osm_id, w.node_ids, std::move(tags));
    }
    using M = osm::OsmBuilder::MemberIn;
    const auto way = osm::MemberType::Way, node = osm::MemberType::Node;
    // 500: via node 1 held by the tile. 501: via way 10 (held). 502: via node 999 is in no road way. 503: not for cars.
    builder.add_relation(500, {M{way, 20, "from"}, M{node, 1, "via"}, M{way, 10, "to"}},
        {{"restriction", relation_conflict ? "no_right_turn" : "no_left_turn"}, {"type", "restriction"}});
    builder.add_relation(501, {M{way, 20, "from"}, M{way, 10, "via"}, M{way, 20, "to"}},
        {{"restriction", "only_straight_on"}, {"type", "restriction"}});
    builder.add_relation(502, {M{way, 20, "from"}, M{node, 999, "via"}, M{way, 10, "to"}},
        {{"restriction", "no_u_turn"}, {"type", "restriction"}});
    builder.add_relation(503, {M{way, 20, "from"}, M{node, 1, "via"}, M{way, 10, "to"}},
        {{"restriction:hgv", "no_left_turn"}, {"type", "restriction"}});
    const auto data = builder.build(); REQUIRE(data); return *data;
}
InputTile input(const InputTileRef& ref, const osm::OsmData& data) {
    const auto tile_body = encode_src_osm({128, data}); REQUIRE(tile_body);
    const auto bytes = encode_body(*tile_body); REQUIRE(bytes);
    TileHeader header; header.layer = ref.layer; header.layer_version = 1; header.key = ref.key;
    header.body_size = bytes->size(); header.body_hash = content_hash(*bytes);
    return {ref, header, *bytes};
}
std::vector<InputTile> inputs(const RoadGraphDeriver& deriver, const TileKey& key) {
    std::vector<InputTile> result;
    const auto data = raw(key);
    for (const auto& ref : deriver.inputs_for(key)) {
        if (ref.key == key || (ref.key.y == key.y && ref.key.x == key.x + 1)) result.push_back(input(ref, data));
        else result.push_back({ref, {}, {}});
    }
    return result;
}
}

TEST_CASE("Road graph tiles preserve complete ways with unique boundary ownership", "[layer][road_graph_tile]") {
    const auto key = g2m_test::home_tile(); auto east = key; ++east.x;
    const auto source = graph(key);
    const auto west_tile = select_road_graph_tile(source, key); REQUIRE(west_tile);
    const auto east_tile = select_road_graph_tile(source, east); REQUIRE(east_tile);
    CHECK(west_tile->graph.ways[0] == source.ways[0]); CHECK(east_tile->graph.ways[0] == source.ways[0]);
    CHECK(west_tile->owned_edges == std::vector<std::uint32_t>{2});
    CHECK(east_tile->owned_edges == std::vector<std::uint32_t>{0, 1});
    for (const auto* tile : {&*west_tile, &*east_tile}) {
        const auto encoded = encode_road_graph(*tile); REQUIRE(encoded);
        const auto bytes = encode_body(*encoded); REQUIRE(bytes);
        const auto body = decode_body(*bytes); REQUIRE(body);
        const auto decoded = decode_road_graph(*body); REQUIRE(decoded);
        CHECK(*decoded == *tile);
    }
    auto distant = key; distant.x += 10;
    const auto empty = select_road_graph_tile(source, distant); REQUIRE(empty);
    CHECK(empty->graph.ways.empty()); CHECK(empty->owned_edges.empty());
    const auto encoded = encode_road_graph(*empty); REQUIRE(encoded); CHECK(decode_road_graph(*encoded));
}

TEST_CASE("Road graph codec rejects truncations and noncanonical topology", "[layer][road_graph_tile]") {
    const auto selected = select_road_graph_tile(graph(g2m_test::home_tile()), g2m_test::home_tile()); REQUIRE(selected);
    const auto encoded = encode_road_graph(*selected); REQUIRE(encoded);
    for (std::size_t n = 0; n < encoded->sections[0].data.size(); ++n) {
        auto short_body = *encoded; short_body.sections[0].data.resize(n);
        CHECK_FALSE(decode_road_graph(short_body));
    }
    auto extra = *encoded; extra.sections[0].data.push_back(0); CHECK_FALSE(decode_road_graph(extra));
    auto wrong = *encoded; ++wrong.encoding_version; CHECK_FALSE(decode_road_graph(wrong));
    auto bad = *selected; bad.graph.nodes[1].osm_id = bad.graph.nodes[0].osm_id; CHECK_FALSE(encode_road_graph(bad));
    bad = *selected; bad.graph.ways[0].node_ids[0] = 999; CHECK_FALSE(encode_road_graph(bad));
    bad = *selected; bad.owned_edges.push_back(999); CHECK_FALSE(encode_road_graph(bad));
    bad = *selected; bad.graph.ways[0].tags[1].first = bad.graph.ways[0].tags[0].first; CHECK_FALSE(encode_road_graph(bad));
    bad = *selected; bad.graph.nodes[0].position.x = std::numeric_limits<std::int64_t>::max();
    CHECK_FALSE(select_road_graph_tile(bad.graph, bad.key));
    // Count bomb is rejected before allocation: meta + a million-million nodes.
    ByteWriter bomb; bomb.u64(selected->key.packed()); bomb.u32(128); bomb.varint(0); bomb.varint(0); bomb.varint(1ull << 40);
    CHECK_FALSE(decode_road_graph({kRoadGraphEncoding, 1, {{1, SectionCodec::None, bomb.take()}}}));
}

TEST_CASE("Road graph derivation deduplicates sources and ignores input order", "[layer][road_graph_tile]") {
    RoadGraphDeriver deriver; const auto key = g2m_test::home_tile();
    auto source = inputs(deriver, key);
    REQUIRE(source.size() == 9);
    const auto a = deriver.derive({key, 7, source}); REQUIRE(a);
    std::reverse(source.begin(), source.end());
    const auto b = deriver.derive({key, 7, source}); REQUIRE(b); CHECK(a->body == b->body);
    const auto body = decode_body(a->body); REQUIRE(body);
    const auto decoded = decode_road_graph(*body); REQUIRE(decoded);
    REQUIRE(decoded->graph.ways.size() == 2); CHECK(decoded->graph.edges.size() == 3);
    CHECK(decoded->graph.ways[0].tags == graph(key).ways[0].tags);
}

TEST_CASE("Road graph server and parallel bake produce identical pinned containers", "[layer][road_graph_tile]") {
    struct Temp {
        std::filesystem::path path = std::filesystem::temp_directory_path() /
            ("g2m_road_bake_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        ~Temp() { std::error_code ec; std::filesystem::remove_all(path, ec); }
    } temp;
    auto sources = TileStore::open({temp.path, "sources"}); REQUIRE(sources);
    auto live = TileStore::open({temp.path, "live"}); REQUIRE(live);
    auto baked = TileStore::open({temp.path, "baked"}); REQUIRE(baked);
    const auto key = g2m_test::home_tile(); auto east = key; ++east.x;
    for (const auto& k : {key, east}) {
        const auto source = input({"g2m.src.osm", k}, raw(key)); REQUIRE(source.header);
        REQUIRE((*sources)->put({"road_test", source.ref.layer, k}, *source.header, source.body, true));
    }
    BuiltinTerrainDerivers derivers;
    const auto manifest = make_local_release(**sources, builtin_local_release_params("road_test", derivers, {}, true));
    REQUIRE(manifest);
    LocalSourceUpstream upstream(**sources);
    ServerConfig config; config.offline = true; config.derived_store = live->get();
    config.derivers = derivers.ordered(true); config.upstreams = {&upstream};
    Server server(config); REQUIRE(server.add_release(*manifest, "road_test"));
    import::BakeOptions options; options.layers = {std::string(kRoadGraphLayer)};
    options.min_level = options.max_level = 2; options.jobs = 4; options.pin = true;
    const auto report = import::bake(**sources, **baked, "road_test", options); REQUIRE(report);
    CHECK(report->release_id_hex == to_hex(manifest->rid)); REQUIRE(report->layers.size() == 1);
    CHECK(report->layers[0].tiles_total == 2);
    for (const auto& k : {key, east}) {
        const auto response = server.tile({manifest->rid, std::string(kRoadGraphLayer), k, {}});
        REQUIRE(response.meta.status == Status::Ok);
        const auto stored = (*baked)->get({report->release_id_hex, std::string(kRoadGraphLayer), k});
        REQUIRE(stored); REQUIRE(*stored); CHECK((*stored)->pinned);
        CHECK((*stored)->container() == response.container);
        const auto body = decode_body((*stored)->body); REQUIRE(body);
        const auto tile = decode_road_graph(*body); REQUIRE(tile); CHECK(tile->graph.ways.size() == 2);
    }
}

TEST_CASE("Road graph derivation fails inconsistent duplicate sources", "[layer][road_graph_tile]") {
    RoadGraphDeriver deriver; const auto key = g2m_test::home_tile();
    auto source = inputs(deriver, key);
    for (auto& value : source) if (value.header && value.ref.key.x == key.x + 1)
        value = input(value.ref, raw(key, true));
    CHECK_FALSE(deriver.derive({key, 0, source}));
    source = inputs(deriver, key); source[0] = source[1]; CHECK_FALSE(deriver.derive({key, 0, source}));
    source = inputs(deriver, key); source.pop_back(); CHECK_FALSE(deriver.derive({key, 0, source}));
    source = inputs(deriver, key);
    for (auto& value : source) if (value.header) { value.header->body_hash[0] ^= 1; break; }
    CHECK_FALSE(deriver.derive({key, 0, source}));
    source = inputs(deriver, key);
    for (auto& value : source) { value.header.reset(); value.body.clear(); }
    const auto empty = deriver.derive({key, 0, source}); REQUIRE(empty);
    const auto body = decode_body(empty->body); REQUIRE(body);
    const auto tile = decode_road_graph(*body); REQUIRE(tile); CHECK(tile->graph.nodes.empty());
}

TEST_CASE("Road graph nodes keep whitelisted tags in encoding 6 and encodings 5 and 4 still decode", "[layer][road_graph_tile][road_node_tags]") {
    const auto key = g2m_test::home_tile();
    auto source = graph(key);
    source.nodes[0].tags = {{"highway", "traffic_signals"}, {"traffic_calming", "table"}, {"traffic_calming:length", "6"}};
    source.nodes[3].tags = {{"barrier", "bollard"}};
    auto tile = select_road_graph_tile(source, key); REQUIRE(tile);
    const auto encoded = encode_road_graph(*tile); REQUIRE(encoded);
    CHECK(encoded->encoding == 6);
    const auto decoded = decode_road_graph(*encoded); REQUIRE(decoded);
    CHECK(*decoded == *tile);
    CHECK(decoded->graph.nodes[0].tags.size() + decoded->graph.nodes[3].tags.size() == 4);
    // Encoding 4: the same graph without node tags; both encodings decode, and the old one has no tags.
    const auto five = encode_road_graph(*tile, kRoadGraphEncodingTags); REQUIRE(five);
    CHECK(five->encoding == 5);
    const auto decoded_five = decode_road_graph(*five); REQUIRE(decoded_five); CHECK(*decoded_five == *tile); // no restrictions to lose
    const auto legacy = encode_road_graph(*tile, kRoadGraphEncodingLegacy); REQUIRE(legacy);
    CHECK(legacy->encoding == 4);
    CHECK(legacy->sections[0].data.size() < encoded->sections[0].data.size());
    const auto old = decode_road_graph(*legacy); REQUIRE(old);
    for (const auto& node : old->graph.nodes) CHECK(node.tags.empty());
    auto stripped = *tile; for (auto& node : stripped.graph.nodes) node.tags.clear();
    CHECK(*old == stripped);
    CHECK_FALSE(encode_road_graph(*tile, 7));
    // Truncation of the tagged section never decodes.
    for (std::size_t n = 0; n < encoded->sections[0].data.size(); ++n) {
        auto cut = *encoded; cut.sections[0].data.resize(n); CHECK_FALSE(decode_road_graph(cut));
    }
    // Noncanonical node tags are rejected by the encoder: unsorted, duplicated, a key outside the whitelist.
    auto bad = *tile; bad.graph.nodes[0].tags = {{"traffic_calming", "table"}, {"highway", "x"}}; CHECK_FALSE(encode_road_graph(bad));
    bad = *tile; bad.graph.nodes[0].tags = {{"highway", "x"}, {"highway", "y"}}; CHECK_FALSE(encode_road_graph(bad));
    bad = *tile; bad.graph.nodes[0].tags = {{"name", "x"}}; CHECK_FALSE(encode_road_graph(bad));
    // A node-tag count bomb is rejected before allocation.
    ByteWriter bomb; bomb.u64(tile->key.packed()); bomb.u32(128); bomb.varint(0); bomb.varint(0); bomb.varint(1);
    bomb.svarint(1); bomb.svarint(0); bomb.svarint(0); bomb.varint(1ull << 40);
    CHECK_FALSE(decode_road_graph({kRoadGraphEncoding, 1, {{1, SectionCodec::None, bomb.take()}}}));
}

TEST_CASE("Road graph derivation copies only whitelisted node tags, sorted", "[layer][road_graph_tile][road_node_tags]") {
    osm::OsmBuilder builder; const auto key = g2m_test::home_tile();
    const geo::Utm utm(geo::kGRS80);
    const auto x = key.min_easting() + 100.0, y = key.min_northing() + 100.0;
    for (int i = 0; i < 3; ++i) {
        const auto ll = utm.reverse(key.zone, x + 50.0 * i, y);
        osm::OsmBuilder::TagList tags;
        if (i == 1) tags = {{"name", "Kreuzung"}, {"traffic_calming:length", "6"}, {"highway", "crossing"}, {"traffic_calming", "hump"}, {"addr:street", "x"}};
        builder.add_node(i + 1, g2m_test::to_e7(ll.lat), g2m_test::to_e7(ll.lon), std::move(tags));
    }
    builder.add_way(10, {1, 2, 3}, {{"highway", "residential"}});
    const auto data = builder.build(); REQUIRE(data);
    const auto world = build_road_graph(*data, key.zone); REQUIRE(world);
    REQUIRE(world->nodes.size() == 3);
    CHECK(world->nodes[0].tags.empty()); CHECK(world->nodes[2].tags.empty());
    using Tag = std::pair<std::string, std::string>;
    CHECK(world->nodes[1].tags == std::vector<Tag>{{"highway", "crossing"}, {"traffic_calming", "hump"}, {"traffic_calming:length", "6"}});
    const auto tile = select_road_graph_tile(*world, key); REQUIRE(tile);
    const auto encoded = encode_road_graph(*tile); REQUIRE(encoded);
    const auto decoded = decode_road_graph(*encoded); REQUIRE(decoded); CHECK(*decoded == *tile);
    // Context merging keeps the tags and rejects two copies of a node whose tags differ.
    auto other = *world; other.nodes[1].tags.pop_back();
    const RoadGraph* both[] = {&*world, &other};
    CHECK_FALSE(merge_road_graphs(both));
    const RoadGraph* same[] = {&*world, &*world};
    const auto merged = merge_road_graphs(same); REQUIRE(merged); CHECK(*merged == *world);
}

TEST_CASE("Road graph derivation stores the restrictions a tile holds, one copy across sources", "[layer][road_graph_tile][restriction]") {
    RoadGraphDeriver deriver; const auto key = g2m_test::home_tile();
    auto source = inputs(deriver, key); // the relations come with two of the nine source tiles
    const auto a = deriver.derive({key, 7, source}); REQUIRE(a);
    std::reverse(source.begin(), source.end());
    const auto b = deriver.derive({key, 7, source}); REQUIRE(b); CHECK(a->body == b->body);
    const auto body = decode_body(a->body); REQUIRE(body);
    CHECK(body->encoding == kRoadGraphEncoding);
    const auto tile = decode_road_graph(*body); REQUIRE(tile);
    REQUIRE(tile->restrictions.size() == 2); // 502 holds no road node, 503 is for another vehicle
    const auto& via_node = tile->restrictions[0]; const auto& via_way = tile->restrictions[1];
    CHECK(via_node.relation_id == 500); CHECK(via_node.kind == RestrictionKind::No); CHECK(via_node.value == "no_left_turn");
    CHECK(via_node.from_way == 20); CHECK(via_node.via_node == 1); CHECK(via_node.to_way == 10); CHECK(via_node.via_ways.empty());
    CHECK(via_way.relation_id == 501); CHECK(via_way.kind == RestrictionKind::Only);
    CHECK(via_way.via_node == 0); CHECK(via_way.via_ways == std::vector<std::int64_t>{10}); CHECK(via_way.to_way == 20);
    // The tile is exactly what select_road_graph_tile makes from the world graph and the world rule list.
    const auto world = build_road_graph(raw(key), key.zone); REQUIRE(world);
    const auto all = extract_turn_restrictions(raw(key)); REQUIRE(all);
    CHECK(all->restrictions.size() == 3);
    const auto selected = select_road_graph_tile(*world, key, all->restrictions); REQUIRE(selected);
    CHECK(*selected == *tile);
    // A tile that holds none of the via objects keeps none; the list argument is optional.
    auto far = key; far.y += 6;
    const auto empty = select_road_graph_tile(*world, far, all->restrictions); REQUIRE(empty); CHECK(empty->restrictions.empty());
    const auto plain = select_road_graph_tile(*world, key); REQUIRE(plain); CHECK(plain->restrictions.empty());
    // Two source tiles that disagree about a relation fail the derivation.
    source = inputs(deriver, key);
    for (auto& value : source) if (value.header && value.ref.key.x == key.x + 1) value = input(value.ref, raw(key, false, true));
    CHECK_FALSE(deriver.derive({key, 0, source}));
}

TEST_CASE("Road graph encoding 6 round trips restrictions; 5 and 4 leave them out; bad ones are rejected", "[layer][road_graph_tile][restriction]") {
    const auto key = g2m_test::home_tile();
    const auto source = graph(key);
    const auto rule = [](std::int64_t relation, const char* value, std::int64_t from, std::int64_t via_node, std::int64_t to,
                         std::vector<std::int64_t> via_ways = {}) {
        TurnRestriction r; r.relation_id = relation; r.value = value; std::size_t code = 0;
        REQUIRE(restriction_value_code(value, code)); r.kind = restriction_value_kind(code);
        r.from_way = from; r.via_node = via_node; r.via_ways = std::move(via_ways); r.to_way = to; return r;
    };
    const std::vector<TurnRestriction> list{rule(5, "no_left_turn", 20, 1, 10), rule(5, "no_left_turn", 20, 2, 99),
        rule(7, "only_straight_on", 20, 4, 10), rule(9, "no_u_turn", 98, 0, 97, {10, 20}), rule(12, "only_u_turn", 20, 3, 20)};
    auto tile = select_road_graph_tile(source, key, list); REQUIRE(tile);
    // The tile keeps the ones it holds: all but the via node 3 ... which is a node of way 10, so all five.
    CHECK(tile->restrictions.size() == 5);
    const auto encoded = encode_road_graph(*tile); REQUIRE(encoded);
    CHECK(encoded->encoding == 6);
    const auto decoded = decode_road_graph(*encoded); REQUIRE(decoded);
    CHECK(*decoded == *tile);
    // Encoding 5 and 4 drop the rules and decode without them.
    for (const auto old : {kRoadGraphEncodingTags, kRoadGraphEncodingLegacy}) {
        const auto older = encode_road_graph(*tile, old); REQUIRE(older);
        const auto back = decode_road_graph(*older); REQUIRE(back); CHECK(back->restrictions.empty());
    }
    // No prefix of the section decodes.
    for (std::size_t n = 0; n < encoded->sections[0].data.size(); ++n) {
        auto cut = *encoded; cut.sections[0].data.resize(n); CHECK_FALSE(decode_road_graph(cut));
    }
    auto trailing = *encoded; trailing.sections[0].data.push_back(0); CHECK_FALSE(decode_road_graph(trailing));
    // Encoder validation.
    const auto rejected = [&](auto&& edit) { auto bad = *tile; edit(bad.restrictions); return !encode_road_graph(bad); };
    CHECK(rejected([](auto& r) { std::swap(r[0], r[2]); }));                       // order
    CHECK(rejected([](auto& r) { r.push_back(r.back()); }));                       // duplicate key
    CHECK(rejected([](auto& r) { r[0].value = "no_such_turn"; }));                 // unknown value
    CHECK(rejected([](auto& r) { r[0].kind = RestrictionKind::Only; }));           // kind contradicts the value
    CHECK(rejected([](auto& r) { r[0].via_ways = {10}; }));                        // both via node and via ways
    CHECK(rejected([](auto& r) { r[0].via_node = 0; }));                           // neither
    CHECK(rejected([](auto& r) { r[0].via_node = 777; }));                         // via node not in the tile graph
    CHECK(rejected([](auto& r) { r[3].via_ways = {555, 556}; }));                  // no via way in the tile graph
    CHECK(rejected([](auto& r) { r[3].via_ways.assign(kRoadGraphMaxViaWays + 1, 10); }));
    CHECK(rejected([](auto& r) { r[0].from_way = 0; }));
    CHECK(rejected([](auto& r) { r[0].relation_id = 0; }));
    // Decoder: an unknown value code and a count bomb.
    auto one = *tile; one.restrictions = {rule(5, "no_left_turn", 20, 1, 10)};
    auto one_body = encode_road_graph(one); REQUIRE(one_body);
    auto& data = one_body->sections[0].data;
    REQUIRE(data.size() > 6);
    auto bad_code = *one_body; bad_code.sections[0].data[data.size() - 5] = static_cast<std::uint8_t>(kRestrictionValueCount);
    CHECK_FALSE(decode_road_graph(bad_code));
    auto no_rules = *tile; no_rules.restrictions.clear();
    auto bomb = encode_road_graph(no_rules); REQUIRE(bomb);
    bomb->sections[0].data.pop_back(); // the zero count
    ByteWriter w; w.varint(1ull << 40);
    for (const auto byte : w.take()) bomb->sections[0].data.push_back(byte);
    CHECK_FALSE(decode_road_graph(*bomb));
}
