// tests/layer/test_road_underpass.cpp - street fidelity P2 slice 12 / request G-6 (terrain migration phase 6): the
// census of ground stretches over lower untagged crossings (road_underpass.h). A synthetic underpass in a DEM cutting
// shows the dip the served carve inherits from the bare-earth DEM; flat and same-layer controls; the census of both
// OSM fixtures on the flat and tilted synthetic DEMs is printed (their DEMs have no cuttings, so the dips there bound
// what the profile fit itself adds). The fixture tiles are the served ones (completion tier on).
#include "carve_fixture.h"
#include "road_fixture.h"

#include "g2m/layer/road_underpass.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <functional>
#include <set>
#include <string>
#include <vector>

using namespace g2m;

namespace {
const TileKey kTile{geo::UtmZone{32, geo::Hemisphere::North}, 2, 455, 5421};
constexpr double kE0 = 455.0 * 1024.0 + 512.0, kN0 = 5421.0 * 1024.0 + 512.0;

// Upper way 100 (secondary, east-west through the centre), lower way 200 (residential, north-south) crossing it at the
// centre without a shared node. `lower_tags` and `upper_tags` are added to the ways' highway tag.
RoadGraph scene(const std::vector<std::pair<std::string, std::string>>& lower_tags,
                const std::vector<std::pair<std::string, std::string>>& upper_tags = {}) {
    RoadGraph g;
    const auto node = [&](std::int64_t id, double x, double y) {
        g.nodes.push_back({id, {std::llround((kE0 + x) * 1000.0), std::llround((kN0 + y) * 1000.0)}, {}});
    };
    for (int i = 0; i <= 8; ++i) node(1 + i, -400.0 + 100.0 * i, 0.0);
    for (int i = 0; i <= 8; ++i) node(20 + i, 0.5, -400.0 + 100.0 * i + 50.0);   // never on the upper way's nodes
    auto upper = std::vector<std::pair<std::string, std::string>>{{"highway", "secondary"}};
    upper.insert(upper.end(), upper_tags.begin(), upper_tags.end());
    std::sort(upper.begin(), upper.end());
    auto lower = std::vector<std::pair<std::string, std::string>>{{"highway", "residential"}};
    lower.insert(lower.end(), lower_tags.begin(), lower_tags.end());
    std::sort(lower.begin(), lower.end());
    std::vector<std::int64_t> un, ln;
    for (int i = 0; i <= 8; ++i) un.push_back(1 + i);
    for (int i = 0; i <= 8; ++i) ln.push_back(20 + i);
    g.ways.push_back({100, 6, un, upper});
    g.ways.push_back({200, 3, ln, lower});
    for (const auto& w : g.ways)
        for (std::size_t i = 1; i < w.node_ids.size(); ++i)
            g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    return g;
}
// Bare-earth DEM of an underpass: 100 m, with a 6 m deep cutting 16 m wide (smooth 4 m shoulders) along the lower road.
double cutting(double e, double /*n*/) {
    const double d = std::fabs(e - (kE0 + 0.5));
    if (d <= 8) return 94.0;
    if (d >= 12) return 100.0;
    const double t = (d - 8) / 4;
    return 94.0 + 6.0 * t * t * (3 - 2 * t);
}
UnderpassCensus census_of(const RoadGraph& world, const std::function<double(double, double)>& dem) {
    const auto tile = g2m_test::served_geometry_in_tile(world, kTile, dem);
    REQUIRE(tile);
    return road_underpass_census(*tile);
}
} // namespace

TEST_CASE("underpass census (G-6): a ground road over a cutting dips; flat and same-layer controls", "[layer][census][underpass]") {
    // layer=-1 cutting, not a tunnel: measured, and the profile sags into the DEM cutting.
    const auto dipped = census_of(scene({{"layer", "-1"}}), cutting);
    std::printf("G-6 synthetic: ground entries %zu, crossings %zu, short %zu, same %zu\n", dipped.ground_entries,
                dipped.crossings.size(), dipped.short_side, dipped.same_layer);
    CHECK(dipped.ground_entries >= 1);
    REQUIRE(dipped.crossings.size() == 1);
    const auto& x = dipped.crossings.front();
    CHECK(x.upper_way == 100);
    CHECK(x.lower_way == 200);
    CHECK(x.lower_layer == -1);
    CHECK_FALSE(x.lower_tunnel);
    CHECK(std::fabs(x.station_m - 400.5) < 0.6);
    CHECK(x.dip_m > 0.3);
    CHECK(dipped.over_threshold == 1);
    std::printf("G-6 synthetic cutting (6 m deep, 16 m wide): dip %.3f m (profile %.3f, chord %.3f)\n", x.dip_m, x.profile_m, x.chord_m);

    // Control: the same crossing over a flat DEM does not dip.
    const auto flat = census_of(scene({{"layer", "-1"}}), {});
    REQUIRE(flat.crossings.size() == 1);
    CHECK(std::fabs(flat.crossings.front().dip_m) < 0.01);
    CHECK(flat.over_threshold == 0);

    // A tunnel is listed as such.
    const auto tunnel = census_of(scene({{"tunnel", "yes"}, {"layer", "-1"}}), cutting);
    REQUIRE(tunnel.crossings.size() == 1);
    CHECK(tunnel.crossings.front().lower_tunnel);

    // No layer tag: a crossing without a node on the same layer is counted apart, not measured.
    const auto same = census_of(scene({}), cutting);
    CHECK(same.crossings.empty());
    CHECK(same.same_layer == 2);   // seen from each of the two ways

    // The upper road on a bridge is not a ground stretch: nothing to report.
    const auto bridged = census_of(scene({{"layer", "-1"}}, {{"bridge", "yes"}, {"layer", "1"}}), cutting);
    CHECK(bridged.crossings.empty());
}

TEST_CASE("underpass census (G-6): both fixtures, flat and tilted DEM (diag counts)", "[layer][census][underpass]") {
    std::size_t measured_all = 0;
    for (const char* file : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const RoadGraph world = g2m_test::fixture_world(file);
        std::set<TileKey> keys;
        for (const auto& n : world.nodes) {
            keys.insert(TileKey{geo::UtmZone{32, geo::Hemisphere::North}, 2, static_cast<std::int32_t>(floor_div(n.position.x, 1024000)),
                                static_cast<std::int32_t>(floor_div(n.position.y, 1024000))});
        }
        for (const bool tilted : {false, true}) {
            std::size_t ground = 0, measured = 0, tunnels = 0, same = 0, short_side = 0, over = 0;
            double max_dip = 0;
            std::set<std::pair<std::int64_t, std::int64_t>> pairs;
            for (const auto& key : keys) {
                const auto tile = g2m_test::served_geometry_in_tile(world, key,
                    tilted ? std::function<double(double, double)>(g2m_test::carve_tilted_dem) : std::function<double(double, double)>{});
                REQUIRE(tile);
                const auto c = road_underpass_census(*tile);
                ground += c.ground_entries; same += c.same_layer; short_side += c.short_side;
                for (const auto& x : c.crossings) {
                    if (!pairs.insert({x.upper_way, x.lower_way}).second) continue;   // a pair seen from two tiles
                    ++measured;
                    tunnels += x.lower_tunnel ? 1 : 0;
                    over += x.dip_m > 0.3 ? 1 : 0;
                    max_dip = std::max(max_dip, x.dip_m);
                }
            }
            std::printf("G-6 census %s %s: L2 tiles %zu, ground entries %zu, lower crossings %zu (tunnel %zu), same-layer %zu, "
                        "short side %zu, dip > 0.3 m %zu, max dip %.3f m\n",
                        file, tilted ? "tilted" : "flat", keys.size(), ground, measured, tunnels, same, short_side, over, max_dip);
            measured_all += measured;
            CHECK(over == 0);   // synthetic DEMs have no cuttings: any dip here would be the fit's own
        }
    }
    CHECK(measured_all > 0);   // the A 66 fixture's two tunnels under ground roads
}
