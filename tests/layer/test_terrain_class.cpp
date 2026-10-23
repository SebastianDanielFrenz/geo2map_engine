// tests/layer/test_terrain_class.cpp - street fidelity P2 slice 10 (terrain migration phase 6): g2m.terrain.class
// (design 6.5, 9.4; tests T16 and the class half of T17 of section 11, decision D49).
//
// Fixture runs use the committed OSM fixtures with served roads.geom (carve_fixture.h) on the tilted rough DEM, and
// synthetic WorldCover-like land cover (make_landcover_tile). Every assertion has a sabotage or negative control.
#include "carve_fixture.h"
#include "synthetic.h"

#include "g2m/core/hash.h"
#include "g2m/import/bake.h"
#include "g2m/layer/landcover_classifier.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/src_landcover.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/terrain_class.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/upstream.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <vector>

using namespace g2m;

namespace {
constexpr const char* kA66 = "hoechst_a66.osm.pbf";
using DemFn = std::function<double(double, double)>;

TileKey l0(std::int32_t x, std::int32_t y) { return TileKey{g2m_test::kZone32N, 0, x, y}; }

const RoadGraph& world(const std::string& file) {
    static std::map<std::string, std::unique_ptr<RoadGraph>> worlds;
    auto it = worlds.find(file);
    if (it == worlds.end()) it = worlds.emplace(file, std::make_unique<RoadGraph>(g2m_test::fixture_world(file))).first;
    return *it->second;
}
g2m_test::CarveFixture& fixture(const std::string& file) {
    static std::map<std::string, std::unique_ptr<g2m_test::CarveFixture>> fixtures;
    auto it = fixtures.find(file);
    if (it == fixtures.end())
        it = fixtures.emplace(file, std::make_unique<g2m_test::CarveFixture>(world(file), DemFn(g2m_test::carve_tilted_dem))).first;
    return *it->second;
}
// Synthetic land cover per level-2 key (seeded by the key).
const LandcoverTile& landcover(const TileKey& l2) {
    static std::map<TileKey, std::unique_ptr<LandcoverTile>> tiles;
    auto it = tiles.find(l2);
    if (it == tiles.end()) it = tiles.emplace(l2, std::make_unique<LandcoverTile>(g2m_test::make_landcover_tile(l2.packed(), l2))).first;
    return *it->second;
}
// The class tile of a level-0 fixture key; `ancestor_only` reads only the land cover ancestor (the D49 control).
std::unique_ptr<TerrainClassTile> classify(const std::string& file, const TileKey& key, bool ancestor_only = false) {
    auto& fx = fixture(file);
    std::vector<LandcoverTile> lc;
    for (const auto& k : terrain_class_landcover_keys(key)) {
        if (ancestor_only && k != key.parent().parent()) continue;
        lc.push_back(landcover(k));
    }
    const LandcoverBase base(lc, builtin_esa_worldcover_classifier());
    const auto w = fx.window(key);
    const auto r = fx.roads(key);
    auto out = std::make_unique<TerrainClassTile>();
    const auto ok = classify_terrain_tile(*out, key, w, r, base);
    if (!ok) throw std::runtime_error("classify: " + ok.error().message);
    return out;
}
std::array<std::size_t, 8> role_counts(const TerrainClassTile& t) {
    std::array<std::size_t, 8> n{};
    for (const auto r : t.role) ++n[r];
    return n;
}
InputTile input_tile(std::string layer, TileKey key, const TileBody& body, VerticalDatum datum) {
    InputTile out; out.ref = {std::move(layer), key}; out.body = *encode_body(body);
    TileHeader h; h.layer = out.ref.layer; h.layer_version = 1; h.key = key; h.datum = datum;
    h.body_size = out.body.size(); h.body_hash = content_hash(out.body); out.header = h; return out;
}
} // namespace

TEST_CASE("terrain.class: encoding round trip and every rejection", "[layer][terrain_class]") {
    TerrainClassTile t; t.reset(TileKey{g2m_test::kZone32N, 3, 227, 2710});
    for (std::size_t i = 0; i < kTerrainClassNodes; ++i) {
        t.land_class[i] = static_cast<std::uint8_t>(i % 14); t.surface[i] = static_cast<std::uint8_t>(i % 12);
        t.role[i] = static_cast<std::uint8_t>(i % 8); t.rank[i] = static_cast<std::uint8_t>(i % 256);
    }
    const auto body = encode_terrain_class(t); REQUIRE(body);
    CHECK(body->encoding == static_cast<std::uint16_t>(LayerEncoding::TerrainClass));
    CHECK(body->sections.size() == 5);
    const auto bytes = encode_body(*body); REQUIRE(bytes);
    TerrainClassTile back; REQUIRE(decode_terrain_class(*decode_body(*bytes), t.key, back));
    CHECK(back == t);
    const auto g = terrain_class_georef(t.key);
    CHECK(g.width == 257); CHECK(g.registration == SampleRegistration::Node); CHECK(g.spacing_mm == 8000);
    // Value limits on encode.
    for (auto plane : {&TerrainClassTile::land_class, &TerrainClassTile::surface, &TerrainClassTile::role}) {
        auto bad = t; (bad.*plane)[5] = 200; CHECK_FALSE(encode_terrain_class(bad));
    }
    auto short_plane = t; short_plane.rank.pop_back(); CHECK_FALSE(encode_terrain_class(short_plane));
    // Decode rejections, one malformed body each.
    const auto rejects = [&](auto mutate) { TileBody b = *body; mutate(b); TerrainClassTile o; return !decode_terrain_class(b, t.key, o); };
    CHECK(rejects([](TileBody& b) { b.sections.pop_back(); }));
    CHECK(rejects([](TileBody& b) { b.sections.push_back(Section{6, SectionCodec::Zstd, {1}}); }));
    CHECK(rejects([](TileBody& b) { b.sections[1].type = 9; }));
    CHECK(rejects([](TileBody& b) { b.sections[1].data[0] = 14; }));    // land_class > 13
    CHECK(rejects([](TileBody& b) { b.sections[2].data[0] = 12; }));    // surface > 11
    CHECK(rejects([](TileBody& b) { b.sections[3].data[0] = 8; }));     // role > 7
    CHECK(rejects([](TileBody& b) { b.sections[4].data.pop_back(); })); // plane size
    CHECK(rejects([](TileBody& b) { b.sections[0].data.push_back(0); }));   // trailing meta byte
    CHECK(rejects([](TileBody& b) { b.sections[0].data.back() = 'x'; }));   // another scheme
    CHECK(rejects([](TileBody& b) { b.encoding = 1; }));
    CHECK(rejects([](TileBody& b) { b.encoding_version = 2; }));
    TerrainClassTile o;
    CHECK_FALSE(decode_terrain_class(*body, TileKey{g2m_test::kZone32N, 3, 228, 2710}, o));   // georef of another key
    CHECK_FALSE(decode_terrain_class(*body, TileKey{g2m_test::kZone32N, 2, 454, 5420}, o));   // another level
}

TEST_CASE("terrain.class: layer description, inputs and config text", "[layer][terrain_class]") {
    const LayerDesc d = terrain_class_desc();
    CHECK(d.id.name == "g2m.terrain.class");
    CHECK(d.encoding == static_cast<std::uint16_t>(LayerEncoding::TerrainClass));
    CHECK(d.inputs == std::vector<std::string>{"g2m.elev.base", "g2m.roads.geom", "g2m.src.landcover"});
    const TerrainClassDeriver deriver;
    CHECK(deriver.coverage_inputs().size() == 1);
    CHECK(deriver.rejected_input_is_absent({std::string(kRoadGeomLayer), TileKey{g2m_test::kZone32N, 2, 0, 0}}));
    CHECK_FALSE(deriver.rejected_input_is_absent({std::string(kElevBaseLayer), l0(0, 0)}));
    // L0: 9 elev.base + carve roads.geom keys + land cover (D49: the ancestor and its east / north neighbours where the tile
    // touches them).
    CHECK(terrain_class_landcover_keys(l0(1821, 21685)).size() == 1);
    CHECK(terrain_class_landcover_keys(l0(1823, 21685)).size() == 2);   // east edge of its level-2 tile
    CHECK(terrain_class_landcover_keys(l0(1823, 21687)).size() == 4);   // north-east corner
    CHECK(terrain_class_landcover_keys(l0(1820, 21684)).size() == 1);   // the south-west corner reads no neighbour
    CHECK(deriver.inputs_for(l0(1823, 21687)).size() == 9 + carve_road_geom_keys(l0(1823, 21687)).size() + 4);
    // L1-L6: the 4 x 4 window of level L-1 around the children.
    const auto refs = deriver.inputs_for(TileKey{g2m_test::kZone32N, 2, 455, 5421});
    REQUIRE(refs.size() == 16);
    CHECK(refs.front().key == TileKey{g2m_test::kZone32N, 1, 909, 10841});
    CHECK(refs.back().key == TileKey{g2m_test::kZone32N, 1, 912, 10844});
    CHECK(deriver.inputs_for(TileKey{g2m_test::kZone32N, 7, 0, 0}).empty());
    CHECK(terrain_class_config_text().find("g2m.terrain.class/1 nodes=257") == 0);
    CHECK(TerrainClassDeriver().config_hash() == deriver.config_hash());
}

// T16: the integer-metre nodes classify road bed / sidewalk / verge exactly by the carve footprint bands of the core that
// owns them; the negative control samples at the cell centres (half metres) and misclassifies the road edges.
TEST_CASE("terrain.class T16: nodes classify by the footprint bands of their owning core", "[layer][terrain_class][T16]") {
    for (const auto& [x, y] : {std::pair{1820, 21684}, std::pair{1822, 21685}}) {
        const TileKey key = l0(x, y);
        auto& fx = fixture(kA66);
        const auto tile = classify(kA66, key);
        const auto m = fx.model(key);
        const auto counts = role_counts(*tile);
        std::size_t checked = 0, wrong = 0, base_wrong = 0, edge = 0, edge_shifted = 0;
        for (int j = 0; j < kTerrainClassSamples; ++j) for (int i = 0; i < kTerrainClassSamples; ++i) {
            const double px = static_cast<double>(key.min_easting() + i), py = static_cast<double>(key.min_northing() + j);
            const std::size_t idx = TerrainClassTile::index(i, j);
            const CarveSample c = m.model->evaluate(px, py);
            const auto role = static_cast<TerrainRole>(tile->role[idx]);
            if (c.owner == CarveOwner::ApproachCore) {
                ++checked;
                double s = 0, t = 0;
                TerrainRole want = TerrainRole::Verge;
                if (m.model->station_of(c.element, px, py, s, t)) {
                    const CarveFootprint f = carve_footprint(*m.model->core_tile().entries[c.element].cross_section, s);
                    if (t >= f.roadbed_right && t <= f.roadbed_left) want = TerrainRole::RoadBed;
                    else if (t >= f.paved_right && t <= f.paved_left) want = TerrainRole::Sidewalk;
                }
                if (role != want) ++wrong;
                if (role == TerrainRole::RoadBed) {
                    const auto& attrs = m.model->core_tile().entries[c.element].profile->attributes;
                    if (tile->surface[idx] != static_cast<std::uint8_t>(attrs.surface) ||
                        tile->land_class[idx] != static_cast<std::uint8_t>(attrs.land_class) || tile->rank[idx] == 0) ++base_wrong;
                }
            } else if (c.owner == CarveOwner::JunctionCore) {
                CHECK((role == TerrainRole::JunctionRoadBed || role == TerrainRole::JunctionSidewalk || role == TerrainRole::Verge));
            } else if (c.owner == CarveOwner::ApproachBlend || c.owner == CarveOwner::JunctionBlend) {
                CHECK((role == TerrainRole::Blend || role == TerrainRole::BridgeDeck));
            }
            if (role == TerrainRole::Terrain || role == TerrainRole::Blend || role == TerrainRole::Verge || role == TerrainRole::BridgeDeck)
                if (tile->rank[idx] != 0) ++base_wrong;
            // Road edge nodes: a road-bed node next to a non-road-bed node. The cell-centre control classifies them
            // half a metre off.
            if (role == TerrainRole::RoadBed && i + 1 < kTerrainClassSamples && tile->role[TerrainClassTile::index(i + 1, j)] != tile->role[idx]) {
                ++edge;
                const CarveSample h = m.model->evaluate(px + 0.5, py + 0.5);
                if (m.model->classify(px + 0.5, py + 0.5, h).role != role) ++edge_shifted;
            }
        }
        std::printf("T16 tile %d/%d: approach-core nodes %zu, role counts terrain %zu road bed %zu sidewalk %zu verge %zu "
                    "junction %zu/%zu blend %zu deck %zu; mismatches %zu; road edges %zu, cell-centre control differs at %zu\n",
            x, y, checked, counts[0], counts[1], counts[2], counts[3], counts[4], counts[5], counts[6], counts[7], wrong, edge, edge_shifted);
        CHECK(checked > 1000);
        CHECK(counts[1] > 500);
        CHECK(counts[6] > 500);
        CHECK(wrong == 0);
        CHECK(base_wrong == 0);
        CHECK(edge > 50);
        CHECK(edge_shifted * 4 > edge);   // negative control: the half-metre lattice moves at least a quarter of the edges
    }
}

// Edge nodes are shared: neighbouring tiles agree on them, across a level-0 seam and across a level-2 seam (where the
// land cover of the neighbour level-2 tile decides, D49). Control: reading only the land cover ancestor differs there.
TEST_CASE("terrain.class: neighbouring tiles agree on their shared edge nodes (D49 control)", "[layer][terrain_class]") {
    const auto same_column = [](const TerrainClassTile& west, const TerrainClassTile& east) {
        std::size_t diff = 0;
        for (int j = 0; j < kTerrainClassSamples; ++j) {
            const auto a = TerrainClassTile::index(kTerrainClassSamples - 1, j), b = TerrainClassTile::index(0, j);
            if (west.land_class[a] != east.land_class[b] || west.surface[a] != east.surface[b] || west.role[a] != east.role[b] ||
                west.rank[a] != east.rank[b]) ++diff;
        }
        return diff;
    };
    const auto same_row = [](const TerrainClassTile& south, const TerrainClassTile& north) {
        std::size_t diff = 0;
        for (int i = 0; i < kTerrainClassSamples; ++i) {
            const auto a = TerrainClassTile::index(i, kTerrainClassSamples - 1), b = TerrainClassTile::index(i, 0);
            if (south.land_class[a] != north.land_class[b] || south.surface[a] != north.surface[b] || south.role[a] != north.role[b] ||
                south.rank[a] != north.rank[b]) ++diff;
        }
        return diff;
    };
    CHECK(same_column(*classify(kA66, l0(1820, 21684)), *classify(kA66, l0(1821, 21684))) == 0);   // level-0 seam
    CHECK(same_row(*classify(kA66, l0(1820, 21684)), *classify(kA66, l0(1820, 21685))) == 0);
    const auto west = classify(kA66, l0(1819, 21684)), east = classify(kA66, l0(1820, 21684));   // a level-2 seam
    CHECK(same_column(*west, *east) == 0);
    const auto west_ancestor = classify(kA66, l0(1819, 21684), true);
    const std::size_t control = same_column(*west_ancestor, *east);
    std::printf("terrain.class level-2 seam: ancestor-only land cover differs at %zu of 257 edge nodes\n", control);
    CHECK(control > 0);
}

// T17 (class half): L1 = the rank-max of the 3 x 3 L0 nodes around each coincident node, ties to the centre then row-major,
// recomputed here over a global node map; neighbouring L1 tiles agree on their edge; rank-max keeps more road nodes than
// subsampling (the rejected design).
TEST_CASE("terrain.class T17: the rank-max pyramid", "[layer][terrain_class][terrain][long]") {
    const std::int32_t px = 910, py = 10842;   // L1 tiles (910, 10842) and (911, 10842): L0 x 1819..1824, y 21683..21686
    std::map<std::pair<std::int32_t, std::int32_t>, std::unique_ptr<TerrainClassTile>> l0s;
    for (std::int32_t y = 2 * py - 1; y <= 2 * py + 2; ++y)
        for (std::int32_t x = 2 * px - 1; x <= 2 * px + 4; ++x) l0s[{x, y}] = classify(kA66, l0(x, y));
    const auto parent_of = [&](std::int32_t ppx) {
        const TileKey parent{g2m_test::kZone32N, 1, ppx, py};
        std::array<const TerrainClassTile*, 16> window{};
        for (int dy = -1; dy <= 2; ++dy) for (int dx = -1; dx <= 2; ++dx)
            window[static_cast<std::size_t>((dy + 1) * 4 + (dx + 1))] = l0s.at({2 * ppx + dx, 2 * py + dy}).get();
        auto out = std::make_unique<TerrainClassTile>();
        downsample_terrain_class(parent, window, *out);
        return out;
    };
    const auto l1 = parent_of(px), l1e = parent_of(px + 1);
    // Global level-0 nodes (gi, gj) -> (tile, index): every tile writes its own nodes; shared nodes are equal (checked above).
    const auto at = [&](std::int64_t gi, std::int64_t gj, std::size_t& idx) -> const TerrainClassTile* {
        const std::int64_t cx = floor_div(gi, 256), cy = floor_div(gj, 256);
        auto it = l0s.find({static_cast<std::int32_t>(cx), static_cast<std::int32_t>(cy)});
        if (it == l0s.end()) return nullptr;
        idx = TerrainClassTile::index(static_cast<int>(gi - 256 * cx), static_cast<int>(gj - 256 * cy));
        return it->second.get();
    };
    std::size_t differ = 0, roads_max = 0, roads_sub = 0;
    for (int j = 0; j < kTerrainClassSamples; ++j) for (int i = 0; i < kTerrainClassSamples; ++i) {
        const std::int64_t gi = 512 * static_cast<std::int64_t>(px) + 2 * i, gj = 512 * static_cast<std::int64_t>(py) + 2 * j;
        std::size_t bi = 0; const TerrainClassTile* best = at(gi, gj, bi);
        if (best && best->rank[bi] > 0) ++roads_sub;
        for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
            std::size_t k = 0; const TerrainClassTile* t = at(gi + dx, gj + dy, k);
            if (t && (!best || t->rank[k] > best->rank[bi])) { best = t; bi = k; }
        }
        const std::size_t o = TerrainClassTile::index(i, j);
        if (l1->rank[o] > 0) ++roads_max;
        if (!best || l1->land_class[o] != best->land_class[bi] || l1->surface[o] != best->surface[bi] || l1->role[o] != best->role[bi] ||
            l1->rank[o] != best->rank[bi]) ++differ;
    }
    std::size_t edge_diff = 0;
    for (int j = 0; j < kTerrainClassSamples; ++j) {
        const auto a = TerrainClassTile::index(kTerrainClassSamples - 1, j), b = TerrainClassTile::index(0, j);
        if (l1->role[a] != l1e->role[b] || l1->rank[a] != l1e->rank[b] || l1->land_class[a] != l1e->land_class[b]) ++edge_diff;
    }
    std::printf("T17 class L1 %d/%d: road nodes rank-max %zu vs subsampled %zu, differ from the reference %zu, edge diff %zu\n",
        px, py, roads_max, roads_sub, differ, edge_diff);
    CHECK(differ == 0);
    CHECK(edge_diff == 0);
    CHECK(roads_max > roads_sub + roads_sub / 4);
    // The encoded L1 tile decodes through the deriver path too (all 16 inputs present).
    const TerrainClassDeriver deriver;
    const TileKey parent{g2m_test::kZone32N, 1, px, py};
    std::vector<InputTile> inputs;
    for (const auto& ref : deriver.inputs_for(parent)) {
        const auto& t = *l0s.at({ref.key.x, ref.key.y});
        inputs.push_back(input_tile(ref.layer, ref.key, *encode_terrain_class(t), VerticalDatum::None));
    }
    const auto result = deriver.derive({parent, 0, inputs}); REQUIRE(result);
    TerrainClassTile got; REQUIRE(decode_terrain_class(*decode_body(result->body), parent, got));
    CHECK(got == *l1);
    CHECK(result->datum == VerticalDatum::None);
}

// The served path: a release with the class layer (which implies carved and both road layers), server and four-worker bake
// agreeing, land cover read through the classifier, L0 and L1 served.
TEST_CASE("terrain.class: server and bake serve L0 and L1, land cover base classes", "[layer][terrain_class][terrain]") {
    struct Temp {
        std::filesystem::path path = std::filesystem::temp_directory_path() /
            ("g2m_class_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
        ~Temp() { std::error_code ec; std::filesystem::remove_all(path, ec); }
    } temp;
    const geo::UtmZone zone{32, geo::Hemisphere::North};
    auto sources = TileStore::open({temp.path, "sources"}); REQUIRE(sources);
    auto live = TileStore::open({temp.path, "live"}); REQUIRE(live);
    auto baked = TileStore::open({temp.path, "baked"}); REQUIRE(baked);
    const TileKey key2{zone, 2, 0, 0}; const geo::Utm utm(geo::kGRS80);
    osm::OsmBuilder builder;
    for (const auto [id, east] : {std::pair<std::int64_t, double>{1, 10}, {2, 110}}) {
        const auto point = utm.reverse(zone, east, 10.5);
        builder.add_node(id, static_cast<std::int32_t>(std::llround(point.lat * 1e7)), static_cast<std::int32_t>(std::llround(point.lon * 1e7)));
    }
    builder.add_way(10, {1, 2}, {{"highway", "residential"}});
    const auto osm = builder.build(); REQUIRE(osm);
    auto raw = input_tile(std::string(kSrcOsmLayer), key2, *encode_src_osm(OsmTile{64, *osm}), VerticalDatum::None);
    REQUIRE((*sources)->put({"class_test", raw.ref.layer, key2}, *raw.header, raw.body, true));
    for (int y = -1; y <= 1; ++y) for (int x = -1; x <= 1; ++x) {
        const TileKey k{zone, 2, x, y}; ElevTile elev; elev.georef = *tile_georef(k, 1024, SampleRegistration::Cell);
        elev.georef.row_order = RowOrder::NorthToSouth; elev.heights = HeightRaster(1024, 1024);
        std::fill(elev.heights.data().begin(), elev.heights.data().end(), 25600);
        auto data = input_tile(std::string(kSrcElevDgm1Layer), k, *encode_src_elev(elev), VerticalDatum::DHHN2016);
        REQUIRE((*sources)->put({"class_test", data.ref.layer, k}, *data.header, data.body, true));
    }
    // Land cover: tree cover (ESA 10) everywhere in the centre tile.
    LandcoverTile lc; lc.georef = *tile_georef(key2, 256, SampleRegistration::Cell); lc.classes = ClassRaster(256, 256);
    lc.class_scheme = "esa_worldcover_v200";
    std::fill(lc.classes.data().begin(), lc.classes.data().end(), std::uint8_t{10});
    auto lct = input_tile(std::string(kSrcLandcoverLayer), key2, *encode_src_landcover(lc), VerticalDatum::None);
    REQUIRE((*sources)->put({"class_test", lct.ref.layer, key2}, *lct.header, lct.body, true));

    BuiltinTerrainDerivers derivers;
    CHECK(derivers.ordered(false, false, false, true).size() == 6);
    const auto manifest = make_local_release(**sources, builtin_local_release_params("class_test", derivers, {}, false, false, false, true));
    REQUIRE(manifest);
    REQUIRE(manifest->find_layer(std::string(kTerrainClassLayer)));
    REQUIRE(manifest->find_layer(std::string(kTerrainCarvedLayer)));
    const auto carved_only = make_local_release(**sources, builtin_local_release_params("class_test", derivers, {}, false, false, true));
    REQUIRE(carved_only); CHECK(carved_only->rid != manifest->rid);
    CHECK_FALSE(carved_only->find_layer(std::string(kTerrainClassLayer)));
    LocalSourceUpstream upstream(**sources); ServerConfig config; config.offline = true; config.derived_store = live->get();
    config.derivers = derivers.ordered(false, false, false, true); config.upstreams = {&upstream};
    Server server(config); REQUIRE(server.add_release(*manifest, "class_test"));
    const TileKey key0{zone, 0, 0, 0};
    const auto response = server.tile({manifest->rid, std::string(kTerrainClassLayer), key0, {}});
    INFO(response.meta.message); REQUIRE(response.meta.status == Status::Ok);
    const auto view = parse_container(response.container); REQUIRE(view);
    CHECK(view->header.datum == VerticalDatum::None);
    CHECK(view->header.inputs.size() == 11);   // 9 elev.base + roads.geom + land cover
    TerrainClassTile tile; REQUIRE(decode_terrain_class(*decode_body(view->body), key0, tile));
    // The residential road (centre line y = 10.5, east 10..110): road bed nodes on it, tree cover far from it.
    const auto& tree = *builtin_esa_worldcover_classifier().find(10);
    CHECK(tile.role[TerrainClassTile::index(60, 10)] == static_cast<std::uint8_t>(TerrainRole::RoadBed));
    CHECK(tile.rank[TerrainClassTile::index(60, 10)] > 0);
    CHECK(tile.land_class[TerrainClassTile::index(60, 10)] == static_cast<std::uint8_t>(LandClass::PavedRoad));
    CHECK(tile.role[TerrainClassTile::index(200, 200)] == static_cast<std::uint8_t>(TerrainRole::Terrain));
    CHECK(tile.land_class[TerrainClassTile::index(200, 200)] == static_cast<std::uint8_t>(tree.land_class));
    CHECK(tile.surface[TerrainClassTile::index(200, 200)] == static_cast<std::uint8_t>(tree.surface));
    import::BakeOptions options; options.layers = {std::string(kTerrainClassLayer)}; options.min_level = 0; options.max_level = 1; options.jobs = 4;
    const auto report = import::bake(**sources, **baked, "class_test", options);
    INFO((report ? "ok" : report.error().message)); REQUIRE(report);
    CHECK(report->release_id_hex == to_hex(manifest->rid));
    const auto stored = (*baked)->get({report->release_id_hex, std::string(kTerrainClassLayer), key0}); REQUIRE(stored); REQUIRE(*stored);
    CHECK((*stored)->container() == response.container);
    const auto parent = server.tile({manifest->rid, std::string(kTerrainClassLayer), TileKey{zone, 1, 0, 0}, {}});
    REQUIRE(parent.meta.status == Status::Ok);
    const auto stored1 = (*baked)->get({report->release_id_hex, std::string(kTerrainClassLayer), TileKey{zone, 1, 0, 0}}); REQUIRE(stored1); REQUIRE(*stored1);
    CHECK((*stored1)->container() == parent.container);
}
