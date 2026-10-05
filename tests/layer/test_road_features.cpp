// Street fidelity P1 slice 7: traffic-calming features of roads.geom section 4 (design 8.3, owner decision Q2).
#include "road_fixture.h"
#include "g2m/layer/road_geom_tile.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <set>
#include <string>
using namespace g2m;
using Catch::Approx;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};
using Tags = std::vector<std::pair<std::string, std::string>>;

struct Pt { std::int64_t id; double x_m, y_m; Tags tags; };

// One way (id 10, residential) through the points, in a tile of its own, DEM flat.
RoadGeomTile tile_of(const std::vector<Pt>& pts, const Tags& way_tags = {{"highway", "residential"}}) {
    RoadGraph graph;
    for (const auto& p : pts) {
        Tags tags = p.tags; std::sort(tags.begin(), tags.end());
        graph.nodes.push_back({p.id, {std::llround(p.x_m * 1000.0), std::llround(p.y_m * 1000.0)}, std::move(tags)});
    }
    std::sort(graph.nodes.begin(), graph.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    RoadGraphWay way{10, 3, {}, way_tags};
    std::sort(way.tags.begin(), way.tags.end());
    for (const auto& p : pts) way.node_ids.push_back(p.id);
    graph.ways = {way};
    for (std::size_t i = 1; i < way.node_ids.size(); ++i) graph.edges.push_back({10, static_cast<std::uint32_t>(i - 1), way.node_ids[i - 1], way.node_ids[i]});
    const auto source = select_road_graph_tile(graph, {zone, 2, 0, 0}); REQUIRE(source);
    HeightTile heights; heights.key = {zone, 0, 0, 0}; heights.h.fill(25600);
    const HeightTile* views[] = {&heights}; const auto dem = RoadDemSampler::make(views); REQUIRE(dem);
    auto tile = build_road_geom_tile(*source, *dem); REQUIRE(tile);
    return std::move(*tile);
}
Tags calming(const std::string& v) { return {{"traffic_calming", v}}; }

// A 200 m straight way along y = 10 m from x = 10 m, with a feature node every 20 m or so.
std::vector<Pt> row() {
    return {{1, 10, 10, calming("table")},                                   // end node: skipped
        {2, 30, 10, calming("bump")}, {3, 60, 10, {{"traffic_calming", "table"}, {"traffic_calming:length", "8"}}},
        {4, 100, 10, calming("hump")}, {5, 130, 10, calming("chicane")}, {6, 150, 10, calming("dip")},
        {7, 170, 10, calming("cushion")}, {8, 180, 10, calming("yes")}, {9, 185, 10, {{"traffic_calming", "island"}, {"traffic_calming:length", "abc"}}},
        {10, 207, 10, {{"traffic_calming", "table"}, {"traffic_calming:length", "20"}}},  // 3 m from the end: shortened
        {11, 209.5, 10, calming("hump")},                                         // under 1 m to the end: dropped
        {12, 210, 10, calming("hump")}};                                        // end node: skipped
}
} // namespace

TEST_CASE("Features come from interior node tags with rules defaults, tagged lengths and end clipping", "[layer][road_features]") {
    const auto tile = tile_of(row());
    REQUIRE(tile.entries.size() == 1); REQUIRE(tile.entries[0].cross_section);
    CHECK(tile.encoding == 6);
    // 2 (bump) 3 (table) 4 (hump) 5 (chicane) 6 (dip) 7 (cushion) 9 (island) 10 (table, shortened); 1, 8, 11, 12 absent.
    REQUIRE(tile.features.size() == 8);
    const auto& f = tile.features;
    const std::int64_t nodes[] = {2, 3, 4, 5, 6, 7, 9, 10};
    for (std::size_t i = 0; i < 8; ++i) { CHECK(f[i].node_id == nodes[i]); CHECK(f[i].entry_index == 0); }
    CHECK(f[0].station_m == Approx(20.0).margin(1e-6)); CHECK(f[1].station_m == Approx(50.0).margin(1e-6));
    CHECK(f[0].kind == ProfileFeatureKind::Bump); CHECK(f[0].carve); CHECK(f[0].length_mm == 900); CHECK(f[0].height_mm == 70);
    CHECK(f[0].provenance == Provenance::Inferred); CHECK(f[0].width_mm == 0);
    CHECK(f[1].kind == ProfileFeatureKind::Table); CHECK(f[1].length_mm == 8000); CHECK(f[1].height_mm == 80); CHECK(f[1].provenance == Provenance::Tagged);
    CHECK(f[2].kind == ProfileFeatureKind::Hump); CHECK(f[2].length_mm == 3700);
    CHECK(f[3].kind == ProfileFeatureKind::Chicane); CHECK_FALSE(f[3].carve); CHECK(f[3].height_mm == 0);
    CHECK(f[4].kind == ProfileFeatureKind::Dip); CHECK(f[4].carve);
    CHECK(f[5].kind == ProfileFeatureKind::Cushion); CHECK(f[5].carve); CHECK(f[5].width_mm == 0);
    CHECK(f[6].kind == ProfileFeatureKind::Island); CHECK_FALSE(f[6].carve); CHECK(f[6].provenance == Provenance::Inferred); // "abc" is not a length
    CHECK(f[7].kind == ProfileFeatureKind::Table); CHECK(f[7].provenance == Provenance::Derived);
    CHECK(f[7].station_m == Approx(197.0).margin(1e-6)); CHECK(f[7].length_mm == 6000); // 3 m to the end: 2 x 3 m
    // The shortened footprint ends exactly at the stretch end.
    CHECK(f[7].station_m + feature_effective_length_m(f[7]) / 2 <= tile.entries[0].profile->reference.length_m + 1e-9);
    CHECK(validate_road_geom_tile(tile));
    // Sorted by (entry, station, node) and stable through the container.
    const auto body = encode_road_geom(tile); REQUIRE(body);
    const auto back = decode_road_geom(*body); REQUIRE(back);
    CHECK(back->features == tile.features);
    CHECK(feature_height_m(tile, 1, 50.0) == 0.0); // another entry index
}

TEST_CASE("Feature shapes: plateau and C1 smooth-step ramps with a 1 m minimum ramp per side", "[layer][road_features]") {
    const auto tile = tile_of(row());
    const auto& f = tile.features;
    // Table, 8 m tagged, 80 mm: 1:10 would be 0.8 m, the minimum ramp is 1.0 m, plateau 6 m.
    CHECK(feature_ramp_m(f[1]) == Approx(1.0)); CHECK(feature_effective_length_m(f[1]) == Approx(8.0));
    CHECK(feature_height_m(tile, 0, 50.0) == Approx(0.080)); CHECK(feature_height_m(tile, 0, 53.0) == Approx(0.080));
    CHECK(feature_height_m(tile, 0, 53.5) == Approx(0.040)); CHECK(feature_height_m(tile, 0, 54.0) == Approx(0.0).margin(1e-12));
    CHECK(feature_height_m(tile, 0, 46.5) == Approx(0.040)); CHECK(feature_height_m(tile, 0, 46.0) == Approx(0.0).margin(1e-12));
    CHECK(feature_height_m(tile, 0, 60.0) == 0.0);
    // Hump 3.7 m x 70 mm: no plateau, ramp 1.85 m, a smooth peak.
    CHECK(feature_ramp_m(f[2]) == Approx(1.85)); CHECK(feature_height_m(tile, 0, 90.0) == Approx(0.070));
    CHECK(feature_height_m(tile, 0, 90.925) == Approx(0.035)); CHECK(feature_height_m(tile, 0, 91.85) == Approx(0.0).margin(1e-12));
    // Bump 0.9 m is carved 2 m long (ramp 1 m per side), peak 70 mm.
    CHECK(feature_ramp_m(f[0]) == Approx(1.0)); CHECK(feature_effective_length_m(f[0]) == Approx(2.0));
    CHECK(feature_height_m(tile, 0, 20.0) == Approx(0.070)); CHECK(feature_height_m(tile, 0, 20.5) == Approx(0.035));
    CHECK(feature_height_m(tile, 0, 21.0) == Approx(0.0).margin(1e-12));
    // Dip 60 mm: a depression. Cushion 70 mm: carved across the carriageway like a hump (4 m? 1.8 m long -> 2 m).
    CHECK(feature_height_m(tile, 0, 140.0) == Approx(-0.060));
    CHECK(feature_height_m(tile, 0, 160.0) == Approx(0.070)); CHECK(feature_effective_length_m(f[5]) == Approx(2.0));
    // Render-only kinds carve nothing.
    CHECK(feature_height_m(tile, 0, 120.0) == 0.0); CHECK(feature_height_m(tile, 0, 175.0) == 0.0);
    CHECK(feature_height_contribution_m(f[3], 130.0) == 0.0);
    // Table with a tall 200 mm: 1:10 ramps of 2 m, plateau where the length allows.
    ProfileFeature t = f[1]; t.height_mm = 200; t.length_mm = 12000;
    CHECK(feature_ramp_m(t) == Approx(2.0)); CHECK(feature_effective_length_m(t) == Approx(12.0));
    // A 1 m table is carved 2 m long.
    t.length_mm = 1000; CHECK(feature_effective_length_m(t) == Approx(2.0));
    // Two overlapping features add.
    auto two = tile;
    ProfileFeature extra = f[2]; extra.node_id = 77; extra.station_m = 91.0;
    two.features.insert(std::upper_bound(two.features.begin(), two.features.end(), extra,
        [](const ProfileFeature& a, const ProfileFeature& b) { return std::pair(a.station_m, a.node_id) < std::pair(b.station_m, b.node_id); }), extra);
    CHECK(feature_height_m(two, 0, 90.5) == Approx(feature_height_contribution_m(f[2], 90.5) + feature_height_contribution_m(extra, 90.5)));
    CHECK(feature_height_m(two, 0, 90.5) > feature_height_m(tile, 0, 90.5));
}

TEST_CASE("Feature height is C1: bounded slope and bounded curvature with no steps", "[layer][road_features]") {
    const auto tile = tile_of(row());
    for (const auto& f : tile.features) {
        if (!f.carve) continue;
        const double h = f.height_mm / 1000.0, r = feature_ramp_m(f);
        const double half = feature_effective_length_m(f) / 2.0, d = 1e-3;
        // S has |S'| <= 2 and |S''| <= 4: slope bound 2h/r, curvature bound 4h/r^2.
        for (double s = f.station_m - half - 0.5; s < f.station_m + half + 0.5; s += d) {
            const double a = feature_height_contribution_m(f, s - d), b = feature_height_contribution_m(f, s), c = feature_height_contribution_m(f, s + d);
            INFO("kind " << static_cast<int>(f.kind) << " s " << s);
            REQUIRE(std::abs(c - b) <= 2.0 * h / r * d + 1e-12);
            REQUIRE(std::abs(c - 2 * b + a) <= 4.0 * h / (r * r) * d * d + 1e-12);
        }
        // Zero slope at the top and at the foot.
        const double second_order = 4.0 * h / (r * r) * 1e-8 + 1e-12; // |S''| d^2 / 2 at d = 1e-4
        CHECK(std::abs(feature_height_contribution_m(f, f.station_m + 1e-4) - feature_height_contribution_m(f, f.station_m)) < second_order);
        CHECK(std::abs(feature_height_contribution_m(f, f.station_m + half - 1e-4)) < second_order);
    }
}

TEST_CASE("A feature at a rounded corner takes the station of the corner arc, not of the way's ends", "[layer][road_features]") {
    const auto tile = tile_of({{1, 10, 10, {}}, {2, 110, 10, calming("table")}, {3, 210, 40, {}}});
    REQUIRE(tile.entries.size() == 1);
    INFO("profile " << tile.entries[0].profile.has_value() << " xs " << tile.entries[0].cross_section.has_value() << " nfeat " << tile.features.size() << " stretch " << tile.entries[0].stretch.start_ref << ".." << tile.entries[0].stretch.end_ref);
    // A 17 degree bend between two 100 m legs: either a clothoid knot or a rounded arc, always near the node.
    REQUIRE(tile.features.size() == 1);
    const auto& ref = tile.entries[0].profile->reference;
    const auto pose = ref.at(tile.features[0].station_m); REQUIRE(pose);
    const double dx = pose->x - 110.0, dy = pose->y - 10.0;
    CHECK(std::sqrt(dx * dx + dy * dy) < 3.1); // within the 3 m deviation corridor of a residential road
    CHECK(tile.features[0].station_m > 90.0); CHECK(tile.features[0].station_m < 110.0);
}

TEST_CASE("Features on a way split into stretches belong to the stretch holding the node", "[layer][road_features]") {
    // A second way crosses at node 3: way 10 has two stretches, the table on node 2 sits in the first and the
    // hump on node 4 in the second; node 3 itself (a junction) carries a tag that no stretch owns.
    RoadGraph graph;
    const auto add = [&](std::int64_t id, int x, int y, Tags tags = {}) { std::sort(tags.begin(), tags.end()); graph.nodes.push_back({id, {x * 1000LL, y * 1000LL}, tags}); };
    add(1, 10, 10); add(2, 50, 10, calming("table")); add(3, 110, 10, calming("bump")); add(4, 170, 10, calming("hump")); add(5, 230, 10);
    add(6, 110, 60); add(7, 110, -40 + 100);
    std::sort(graph.nodes.begin(), graph.nodes.end(), [](const auto& a, const auto& b) { return a.osm_id < b.osm_id; });
    graph.ways = {{10, 3, {1, 2, 3, 4, 5}, {{"highway", "residential"}}}, {20, 3, {6, 3}, {{"highway", "residential"}}}};
    for (const auto& w : graph.ways) for (std::size_t i = 1; i < w.node_ids.size(); ++i)
        graph.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    // node 7 unused: remove it again so the graph validates.
    graph.nodes.erase(std::remove_if(graph.nodes.begin(), graph.nodes.end(), [](const auto& n) { return n.osm_id == 7; }), graph.nodes.end());
    const auto source = select_road_graph_tile(graph, {zone, 2, 0, 0}); REQUIRE(source);
    HeightTile heights; heights.key = {zone, 0, 0, 0}; heights.h.fill(25600);
    const HeightTile* views[] = {&heights}; const auto dem = RoadDemSampler::make(views); REQUIRE(dem);
    const auto tile = build_road_geom_tile(*source, *dem); REQUIRE(tile);
    std::vector<std::pair<std::size_t, std::int64_t>> placed;
    for (const auto& f : tile->features) placed.emplace_back(f.entry_index, f.node_id);
    REQUIRE(placed.size() == 2);
    CHECK(tile->entries[placed[0].first].stretch.end_ref == 2); CHECK(placed[0].second == 2);
    CHECK(tile->entries[placed[1].first].stretch.start_ref == 2); CHECK(placed[1].second == 4);
    CHECK(validate_road_geom_tile(*tile));
}

TEST_CASE("Committed OSM fixtures: every feature found lies inside its stretch and the height function is bounded", "[layer][road_features][roads]") {
    std::size_t total = 0, carved = 0, derived = 0, tagged_nodes = 0, tiles_built = 0;
    for (const char* name : {"hoechst_a66.osm.pbf", "frankfurt_hoechst.osm.pbf"}) {
        const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + name);
        const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
        std::set<std::pair<int, int>> keys;
        for (const auto& n : world->nodes) {
            for (const auto& [k, v] : n.tags) tagged_nodes += k == "traffic_calming";
            const auto key = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0);
            if (key) keys.insert({key->x, key->y});
        }
        for (const auto& [kx, ky] : keys) {
            const TileKey key{g2m_test::kZone32N, 2, kx, ky};
            const auto probe = select_road_graph_tile(*world, key);
            if (!probe || probe->owned_edges.empty()) continue;
            const auto geometry = g2m_test::geometry_in_tile(*world, key); REQUIRE(geometry);
            ++tiles_built;
            REQUIRE(validate_road_geom_tile(*geometry));
            for (const auto& f : geometry->features) {
                const double length = geometry->entries[f.entry_index].profile->reference.length_m;
                REQUIRE(f.station_m >= 0.0); REQUIRE(f.station_m <= length);
                REQUIRE(f.carve == (f.height_mm > 0));
                REQUIRE(f.station_m - feature_effective_length_m(f) / 2 >= -1e-6);
                REQUIRE(f.station_m + feature_effective_length_m(f) / 2 <= length + 1e-6);
                ++total; carved += f.carve; derived += f.provenance == Provenance::Derived;
            }
            for (std::size_t e = 0; e < geometry->entries.size(); ++e) {
                if (!geometry->entries[e].cross_section) continue;
                const double length = geometry->entries[e].profile->reference.length_m;
                for (double s = 0; s <= length; s += 0.5) { const double z = feature_height_m(*geometry, e, s); REQUIRE(std::isfinite(z)); REQUIRE(std::abs(z) <= 0.3 + 1e-9); }
            }
            const auto body = encode_road_geom(*geometry); REQUIRE(body);
            const auto back = decode_road_geom(*body); REQUIRE(back); CHECK(back->features == geometry->features);
        }
    }
    // The two small fixtures carry only two traffic-calming nodes (measured; one is a render-only kind, one an end
    // node), so the invariants above run on an almost empty feature set; the synthetic cases carry the behaviour.
    std::printf("[road_features] fixtures: %zu tiles, %zu traffic_calming nodes, %zu features (carved %zu, shortened %zu)\n", tiles_built, tagged_nodes, total, carved, derived);
    CHECK(tiles_built > 0);
    CHECK(total <= tagged_nodes);
}

TEST_CASE("diag: effective carved shapes of the default features on the 1 m grid", "[.][diag][layer][road_features]") {
    const auto& rules = builtin_road_rules_de();
    std::printf("kind          nominal(L x h)   ramp  carved L  plateau  grid peak/h over 16 phases (min..max)  grid samples nonzero (min..max)\n");
    const char* names[] = {"bump", "mini_bumps", "rumble_strip", "cushion", "hump", "table", "chicane", "choker", "island", "dip"};
    for (std::size_t k = 0; k < kProfileFeatureKindCount; ++k) {
        const auto kind = static_cast<ProfileFeatureKind>(k);
        const auto& d = rules.calming(kind);
        ProfileFeature f; f.kind = kind; f.carve = d.height_mm > 0; f.length_mm = d.length_mm; f.height_mm = d.height_mm; f.station_m = 50.0;
        if (!f.carve) { std::printf("%-13s %5d mm x %3d mm  render only\n", names[k], d.length_mm, d.height_mm); continue; }
        double lo = 1e9, hi = -1e9; int nlo = 1000, nhi = 0;
        for (int p = 0; p < 16; ++p) {
            double peak = 0; int nonzero = 0;
            for (int i = -12; i <= 12; ++i) {
                const double s = 50.0 + i + p / 16.0;
                const double z = std::abs(feature_height_contribution_m(f, s)); peak = std::max(peak, z); nonzero += z > 1e-9;
            }
            lo = std::min(lo, peak / (d.height_mm / 1000.0)); hi = std::max(hi, peak / (d.height_mm / 1000.0)); nlo = std::min(nlo, nonzero); nhi = std::max(nhi, nonzero);
        }
        std::printf("%-13s %5d mm x %3d mm  %.2f  %5.2f m  %5.2f m   %.2f..%.2f   %d..%d\n", names[k], d.length_mm, d.height_mm, feature_ramp_m(f),
            feature_effective_length_m(f), feature_effective_length_m(f) - 2 * feature_ramp_m(f), lo, hi, nlo, nhi);
    }
}
