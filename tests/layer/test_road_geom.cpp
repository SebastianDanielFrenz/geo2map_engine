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
    const auto body = encode_road_geom(*geometry); REQUIRE(body);
    CHECK(to_hex(content_hash(*encode_body(*body))) == "17d9b7c708b1bda9513ae80f8be1255d1f7c7cc1d6977af8e98a24dfd05ee2f9");
    const auto decoded = decode_road_geom(*body); REQUIRE(decoded);
    CHECK(decoded->source == graph);
    CHECK(decoded->entries[0].profile->at(50)->height_m == 100);
    CHECK(decoded->entries[1].decline->message == geometry->entries[1].decline->message);
    CHECK(*encode_body(*encode_road_geom(*decoded)) == *encode_body(*body));
    CHECK(body->encoding == 6); REQUIRE(body->sections.size() == 4);
    for (std::size_t section = 1; section < 4; ++section) {
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
    for (const auto& dep : *deps) {
        HeightTile t; t.key = dep.key; t.h.fill(25600);
        auto entry = input(dep.layer, dep.key, *encode_height_tile(t), VerticalDatum::DHHN2016);
        entry.header->height_offset = 256; inputs.push_back(std::move(entry));
    }
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
