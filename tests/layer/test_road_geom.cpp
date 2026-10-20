#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/core/hash.h"
#include "g2m/server/builtin_release.h"
#include <catch2/catch_test_macros.hpp>
#include "g2m/layer/src_osm.h"
#include "g2m/layer/src_elev.h"
#include "g2m/import/bake.h"
#include "g2m/server/upstream.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <limits>
using namespace g2m;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};
RoadGraphTile source() {
    RoadGraph graph;
    graph.nodes = {{1, {10000, 10000}}, {2, {110000, 10000}}, {3, {10000, 100000}}, {4, {110000, 100000}}};
    graph.ways = {{10, 1, {1, 2}, {{"highway", "service"}, {"maxspeed", "30"}}},
        {20, 1, {3, 4}, {{"highway", "service"}, {"tunnel", "yes"}}}};
    graph.edges = {{10, 0, 1, 2}, {20, 0, 3, 4}};
    return *select_road_graph_tile(graph, {zone, 2, 0, 0});
}
InputTile input(std::string layer, TileKey key, const TileBody& body, VerticalDatum datum = VerticalDatum::None) {
    InputTile out; out.ref = {std::move(layer), key}; out.body = *encode_body(body);
    TileHeader h; h.layer = out.ref.layer; h.layer_version = 1; h.key = key; h.datum = datum;
    h.body_size = out.body.size(); h.body_hash = content_hash(out.body); out.header = h; return out;
}
}
TEST_CASE("roads.geom round trips complete profiles and explicit declines", "[layer][road_geom]") {
    const auto graph = source(); HeightTile heights; heights.key = {zone, 0, 0, 0}; heights.h.fill(25600);
    const HeightTile* views[] = {&heights}; const auto dem = RoadDemSampler::make(views); REQUIRE(dem);
    auto geometry = build_road_geom_tile(graph, *dem); REQUIRE(geometry);
    REQUIRE(geometry->entries.size() == 2); REQUIRE(geometry->entries[0].profile); REQUIRE(geometry->entries[1].decline);
    CHECK(geometry->entries[1].decline->code == ErrorCode::Unsupported);
    // The encoding-6 body of this tile is unchanged since P1 (written on request, slice 8).
    auto previous = *geometry; previous.encoding = kRoadGeomEncodingPrevious;
    const auto body6 = encode_road_geom(previous); REQUIRE(body6); CHECK(body6->sections.size() == 4);
    CHECK(to_hex(content_hash(*encode_body(*body6))) == "17d9b7c708b1bda9513ae80f8be1255d1f7c7cc1d6977af8e98a24dfd05ee2f9");
    // Encoding 7 (written on request, no tunnel floor here): unchanged since P2 slice 8.
    auto junctions7 = *geometry; junctions7.encoding = kRoadGeomEncodingJunctions;
    const auto body7 = encode_road_geom(junctions7); REQUIRE(body7);
    CHECK(to_hex(content_hash(*encode_body(*body7))) == "a453aed52159ea3e3bac6ce41c3e98eaaebaeb57f0cc68004cf66e4d6791a618");
    const auto body = encode_road_geom(*geometry); REQUIRE(body);
    // Encoding 8 (D47): sections 1-4 byte for byte the encoding-6 ones, then empty junction and furniture sections and
    // the provenance of the two entries (count 2, two Strict records); only the encoding id differs from encoding 7.
    for (std::size_t section = 0; section < 7; ++section) CHECK(body->sections[section].data == body7->sections[section].data);
    CHECK(to_hex(content_hash(*encode_body(*body))) == "3b9bd9c548e078e03c0d919165d99b197c22c0630b7272f50745ecbf4e397720");
    const auto decoded = decode_road_geom(*body); REQUIRE(decoded);
    CHECK(decoded->source == graph);
    CHECK(decoded->entries[0].profile->at(50)->height_m == 100);
    CHECK(decoded->entries[1].decline->message == geometry->entries[1].decline->message);
    CHECK(*encode_body(*encode_road_geom(*decoded)) == *encode_body(*body));
    CHECK(body->encoding == 8); REQUIRE(body->sections.size() == 7);
    for (std::size_t section = 0; section < 4; ++section) CHECK(body->sections[section].data == body6->sections[section].data);
    CHECK(body->sections[4].data.size() == 1); CHECK(body->sections[5].data.size() == 1);
    CHECK(body->sections[6].data == std::vector<std::uint8_t>{2, 0, 0, 0, 0, 0, 0});
    for (std::size_t section = 1; section < 7; ++section) {
        for (std::size_t i = 0; i < body->sections[section].data.size(); ++i) {
            auto truncated = *body; truncated.sections[section].data.resize(i); CHECK_FALSE(decode_road_geom(truncated));
        }
        auto trailing = *body; trailing.sections[section].data.push_back(0); CHECK_FALSE(decode_road_geom(trailing));
    }
    geometry->entries[0].profile->vertical.segments[0].grade0 = 1.0; CHECK_FALSE(encode_road_geom(*geometry));
    geometry = build_road_geom_tile(graph, *dem); geometry->entries.pop_back(); CHECK_FALSE(encode_road_geom(*geometry));
    geometry = build_road_geom_tile(graph, *dem);
    geometry->entries[0].profile->reference.anchors.push_back({1, 25}); CHECK_FALSE(encode_road_geom(*geometry));
    geometry = build_road_geom_tile(graph, *dem);
    geometry->entries[0].profile->reference.segments.front().curve.x0 += 1; CHECK_FALSE(encode_road_geom(*geometry));
    auto forged = *body; forged.sections[1].data[0] = 255; CHECK_FALSE(decode_road_geom(forged));
}
TEST_CASE("roads.geom discovers complete DEM dependencies and applies offsets once", "[layer][road_geom]") {
    const auto graph = source(); RoadGeomDeriver deriver;
    std::vector<InputTile> inputs{input(std::string(kRoadGraphLayer), graph.key, *encode_road_graph(graph))};
    const auto deps = deriver.additional_inputs_for({graph.key, 0, inputs}); REQUIRE(deps); REQUIRE_FALSE(deps->empty());
    std::size_t graph_deps = 0;
    for (const auto& dep : *deps) {
        if (dep.layer == kRoadGraphLayer) {
            // D63: the roads.graph tiles of the 61 m squares around the ways' end nodes (here the three neighbours of the
            // tile corner the nodes sit 10 m from); empty, as no other way is there.
            ++graph_deps;
            inputs.push_back(input(dep.layer, dep.key, *encode_road_graph(*select_road_graph_tile(RoadGraph{}, dep.key))));
            continue;
        }
        HeightTile t; t.key = dep.key; t.h.fill(25600);
        auto entry = input(dep.layer, dep.key, *encode_height_tile(t), VerticalDatum::DHHN2016);
        entry.header->height_offset = 256; inputs.push_back(std::move(entry));
    }
    CHECK(graph_deps == 3);
    std::sort(inputs.begin(), inputs.end(), [](const auto& a, const auto& b) { return a.ref < b.ref; });
    const auto result = deriver.derive({graph.key, 0, inputs}); REQUIRE(result);
    CHECK(result->height_offset == 0); CHECK(result->datum == VerticalDatum::DHHN2016);
    const auto body = decode_body(result->body); REQUIRE(body); const auto decoded = decode_road_geom(*body); REQUIRE(decoded);
    REQUIRE(decoded->entries[0].profile); CHECK(decoded->entries[0].profile->at(50)->height_m == 101);
    std::reverse(inputs.begin(), inputs.end()); const auto reversed = deriver.derive({graph.key, 0, inputs}); REQUIRE(reversed); CHECK(reversed->body == result->body);
    auto absent = inputs; absent.front().header.reset(); CHECK_FALSE(deriver.derive({graph.key, 0, absent}));
    auto conflict = inputs;
    for (auto& i : conflict) if (i.ref.layer == kElevBaseLayer) { i.header->datum = VerticalDatum::EGM2008; break; }
    CHECK_FALSE(deriver.derive({graph.key, 0, conflict}));
    CHECK(BuiltinTerrainDerivers{}.ordered().size() == 2);
    CHECK(BuiltinTerrainDerivers{}.ordered(false, true).size() == 4);
}

TEST_CASE("roads.geom server and four-worker bake agree and pin complete dependencies", "[layer][road_geom]") {
    struct Temp {
        std::filesystem::path path = std::filesystem::temp_directory_path() /
            ("g2m_geom_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        ~Temp() { std::error_code ec; std::filesystem::remove_all(path, ec); }
    } temp;
    auto sources = TileStore::open({temp.path, "sources"}); REQUIRE(sources);
    auto live = TileStore::open({temp.path, "live"}); REQUIRE(live);
    auto baked = TileStore::open({temp.path, "baked"}); REQUIRE(baked);
    const TileKey key{zone, 2, 0, 0}; const geo::Utm utm(geo::kGRS80);
    osm::OsmBuilder builder;
    for (const auto [id, east] : {std::pair<std::int64_t, double>{1, 10}, {2, 110}}) {
        const auto point = utm.reverse(zone, east, 10);
        builder.add_node(id, static_cast<std::int32_t>(std::llround(point.lat * 1e7)), static_cast<std::int32_t>(std::llround(point.lon * 1e7)));
    }
    builder.add_way(10, {1, 2}, {{"highway", "service"}, {"maxspeed", "30"}});
    const auto osm = builder.build(); REQUIRE(osm);
    auto raw = input(std::string(kSrcOsmLayer), key, *encode_src_osm(OsmTile{64, *osm}));
    REQUIRE((*sources)->put({"geom_test", raw.ref.layer, key}, *raw.header, raw.body, true));
    for (int y = -1; y <= 1; ++y) for (int x = -1; x <= 1; ++x) {
        const TileKey k{zone, 2, x, y}; ElevTile elev; elev.georef = *tile_georef(k, 1024, SampleRegistration::Cell);
        elev.georef.row_order = RowOrder::NorthToSouth; elev.heights = HeightRaster(1024, 1024); std::fill(elev.heights.data().begin(), elev.heights.data().end(), 25600);
        auto data = input(std::string(kSrcElevDgm1Layer), k, *encode_src_elev(elev), VerticalDatum::DHHN2016);
        REQUIRE((*sources)->put({"geom_test", data.ref.layer, k}, *data.header, data.body, true));
    }
    BuiltinTerrainDerivers derivers;
    const auto manifest = make_local_release(**sources, builtin_local_release_params("geom_test", derivers, {}, true, true)); REQUIRE(manifest);
    LocalSourceUpstream upstream(**sources); ServerConfig config; config.offline = true; config.derived_store = live->get();
    config.derivers = derivers.ordered(true, true); config.upstreams = {&upstream}; Server server(config); REQUIRE(server.add_release(*manifest, "geom_test"));
    import::BakeOptions options; options.layers = {std::string(kRoadGeomLayer)}; options.min_level = options.max_level = 2; options.jobs = 4; options.pin = true;
    const auto report = import::bake(**sources, **baked, "geom_test", options);
    INFO((report ? "ok" : report.error().message));
    REQUIRE(report);
    CHECK(report->release_id_hex == to_hex(manifest->rid)); REQUIRE(report->layers.size() == 1); CHECK(report->layers[0].tiles_total == 1);
    const auto response = server.tile({manifest->rid, std::string(kRoadGeomLayer), key, {}}); REQUIRE(response.meta.status == Status::Ok);
    const auto stored = (*baked)->get({report->release_id_hex, std::string(kRoadGeomLayer), key}); REQUIRE(stored); REQUIRE(*stored);
    CHECK((*stored)->pinned); CHECK((*stored)->container() == response.container); CHECK((*stored)->header.inputs.size() > 1);
    const auto body = decode_body((*stored)->body); REQUIRE(body); const auto tile = decode_road_geom(*body); REQUIRE(tile);
    REQUIRE(tile->entries.size() == 1); REQUIRE(tile->entries[0].profile); CHECK(tile->entries[0].profile->at(50)->height_m == 100);
}

TEST_CASE("roads.geom discovers the far end of an incident bridge in a second stage (D35)", "[layer][road_geom]") {
    // Way 10 ends at node 4 (tile x = 1, beyond tile 0's 128 m halo) where a 1000 m bridge (way 20) starts; the ground
    // road way 12 leaves its far end in tile x = 2. Tile 0 holds way 10 only; neither the bridge nor its far end is in
    // its graph or its stage-1 dependencies.
    RoadGraph world;
    for (const auto& [id, x] : std::vector<std::pair<std::int64_t, std::int64_t>>{{1, 100000}, {2, 500000}, {3, 900000},
             {4, 1200000}, {5, 1700000}, {6, 2200000}, {7, 2500000}})
        world.nodes.push_back({id, {x, 128000}});
    world.ways = {{10, 3, {1, 2, 3, 4}, {{"highway", "tertiary"}}}, {12, 3, {6, 7}, {{"highway", "tertiary"}}},
        {20, 3, {4, 5, 6}, {{"bridge", "yes"}, {"highway", "tertiary"}}}};
    for (const auto& w : world.ways) for (std::size_t i = 1; i < w.node_ids.size(); ++i)
        world.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    const RoadGeomDeriver deriver; CHECK(deriver.discovery_stages() == 24);
    const TileKey west{zone, 2, 0, 0}, east{zone, 2, 1, 0};
    const InputTileRef far_graph{std::string(kRoadGraphLayer), {zone, 2, 2, 0}};
    struct Run { std::vector<InputTileRef> stage1, stage2; Result<DeriveResult> result; };
    const auto run = [&](const TileKey& target, bool second_stage) {
        const auto graph = select_road_graph_tile(world, target); REQUIRE(graph);
        std::vector<InputTile> inputs{input(std::string(kRoadGraphLayer), target, *encode_road_graph(*graph))};
        const auto resolve = [&](const std::vector<InputTileRef>& refs) {
            for (const auto& ref : refs) {
                if (std::any_of(inputs.begin(), inputs.end(), [&](const auto& in) { return in.ref == ref; })) continue;
                if (ref.layer == kRoadGraphLayer) inputs.push_back(input(ref.layer, ref.key, *encode_road_graph(*select_road_graph_tile(world, ref.key))));
                else { HeightTile t; t.key = ref.key; t.h.fill(25600); inputs.push_back(input(ref.layer, ref.key, *encode_height_tile(t), VerticalDatum::DHHN2016)); }
            }
            std::sort(inputs.begin(), inputs.end(), [](const auto& a, const auto& b) { return a.ref < b.ref; });
        };
        Run out{{}, {}, make_error(ErrorCode::NotFound, "not run")};
        const auto stage1 = deriver.additional_inputs_for({target, 0, inputs}); REQUIRE(stage1); out.stage1 = *stage1; resolve(*stage1);
        const auto stage2 = deriver.additional_inputs_for({target, 0, inputs}); REQUIRE(stage2); out.stage2 = *stage2;
        if (second_stage) resolve(*stage2);
        out.result = deriver.derive({target, 0, inputs});
        return out;
    };
    const auto a = run(west, true);
    // Stage 1 names tile 1 (node 4) but not the far end; stage 2 adds its graph tile and the DEM along the span.
    CHECK(std::binary_search(a.stage1.begin(), a.stage1.end(), InputTileRef{std::string(kRoadGraphLayer), east}));
    CHECK_FALSE(std::binary_search(a.stage1.begin(), a.stage1.end(), far_graph));
    CHECK(std::binary_search(a.stage2.begin(), a.stage2.end(), far_graph));
    CHECK(a.stage2.size() > a.stage1.size());
    CHECK(std::includes(a.stage2.begin(), a.stage2.end(), a.stage1.begin(), a.stage1.end()));
    REQUIRE(a.result);
    // The east tile holds the bridge itself: its stage 1 already names the far end; stage 2 adds only the D48 walk
    // squares (DEM and graph tiles within 205 m of the bridge's nodes).
    const auto b = run(east, true); CHECK(std::includes(b.stage2.begin(), b.stage2.end(), b.stage1.begin(), b.stage1.end())); REQUIRE(b.result);
    const auto entry_of = [](const DeriveResult& r) {
        const auto geometry = decode_road_geom(*decode_body(r.body)); REQUIRE(geometry);
        for (const auto& e : geometry->entries) if (e.way_id == 10) return e;
        FAIL("way 10 missing"); return geometry->entries.front();
    };
    const auto ea = entry_of(*a.result), eb = entry_of(*b.result);
    REQUIRE(ea.profile); REQUIRE(eb.profile);
    for (double s : {0.0, 100.0, 500.0, 1000.0}) {
        const auto ha = ea.profile->at(s), hb = eb.profile->at(s); REQUIRE(ha); REQUIRE(hb); CHECK(ha->height_m == hb->height_m);
    }
    // Sabotage: stage 1 alone is an incomplete dependency set; the deriver refuses instead of deriving the old tile.
    const auto stage1_only = run(west, false);
    REQUIRE_FALSE(stage1_only.result);
    CHECK(stage1_only.result.error().message.find("incomplete dependency set") != std::string::npos);
}
