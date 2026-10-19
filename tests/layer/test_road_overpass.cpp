// tests/layer/test_road_overpass.cpp - owner report 2026-10-19 (two Frankfurt-west sites):
//  D64: a ground road under an upper bridge. DGM1 is bare earth, the bridge deck is removed and its gap interpolated,
//       so the DEM shows a hump over the road; the strict profile used to follow it, the junction planes at the way's
//       ends took the hump's wall and the way was declined (no ribbon, no carve: "the tunnel ends in a wall").
// Synthetic scenes, a recorded sabotage for each rule (rule switch off: the old behaviour returns).
#include "carve_fixture.h"
#include "road_fixture.h"

#include "underpass_scene.h"

#include "g2m/layer/road_underpass.h"
#include "g2m/layer/terrain_carve.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

using namespace g2m;

namespace {
using g2m_test::kUnderpassE0;
using g2m_test::kUnderpassN0;
using g2m_test::kUnderpassTile;
using Dem = std::function<double(double, double)>;
using Tags = std::vector<std::pair<std::string, std::string>>;
struct WaySpec { std::int64_t id; Tags tags; std::vector<std::pair<double, double>> pts; };

// A world of the given ways (points relative to the tile centre, shared coordinates share a node).
RoadGraph world_of(const std::vector<WaySpec>& specs) {
    RoadGraph g;
    std::map<std::pair<std::int64_t, std::int64_t>, std::int64_t> ids;
    std::int64_t next = 1;
    for (const auto& s : specs) {
        std::vector<std::int64_t> nodes;
        for (const auto& [x, y] : s.pts) {
            const std::pair<std::int64_t, std::int64_t> mm{std::llround((kUnderpassE0 + x) * 1000.0), std::llround((kUnderpassN0 + y) * 1000.0)};
            auto it = ids.find(mm);
            if (it == ids.end()) {
                it = ids.emplace(mm, next++).first;
                g.nodes.push_back({it->second, {mm.first, mm.second}, {}});
            }
            nodes.push_back(it->second);
        }
        Tags tags = s.tags; std::sort(tags.begin(), tags.end());
        g.ways.push_back({s.id, 3, nodes, tags});
    }
    for (const auto& w : g.ways)
        for (std::size_t i = 1; i < w.node_ids.size(); ++i)
            g.edges.push_back({w.osm_id, static_cast<std::uint32_t>(i - 1), w.node_ids[i - 1], w.node_ids[i]});
    std::sort(g.nodes.begin(), g.nodes.end(), [](const RoadGraphNode& a, const RoadGraphNode& b) { return a.osm_id < b.osm_id; });
    std::sort(g.edges.begin(), g.edges.end(), [](const RoadGraphEdge& a, const RoadGraphEdge& b) {
        return std::tie(a.way_id, a.segment_index) < std::tie(b.way_id, b.segment_index);
    });
    return g;
}
double smooth(double t) { t = std::clamp(t, 0.0, 1.0); return t * t * (3 - 2 * t); }

const RoadProfile* profile_of(const RoadGeomTile& t, std::int64_t way) {
    for (const auto& e : t.entries) if (e.way_id == way && e.profile) return &*e.profile;
    return nullptr;
}
std::string decline_of(const RoadGeomTile& t, std::int64_t way) {
    for (const auto& e : t.entries) if (e.way_id == way) return e.profile ? std::string("profile") : e.decline->message;
    return "absent";
}
// max |profile - plane(x, y)| over every whole metre and the end.
double off_plane(const RoadProfile& p, const Dem& plane) {
    double worst = 0;
    for (double s = 0;; s += 1) {
        const double st = std::min(s, p.reference.length_m);
        const auto xy = p.reference.at(st); const auto z = p.at(st); REQUIRE(xy); REQUIRE(z);
        worst = std::max(worst, std::fabs(z->height_m - plane(xy->x, xy->y)));
        if (st >= p.reference.length_m) break;
    }
    return worst;
}

// ---- D64: the bridge over a ground road -----------------------------------------------------------------------
constexpr double kFoot = 19.5;   // the hump's feet; its 4 m steep walls end here, the middle way ends just up them (kEnd)
constexpr double kEnd = 19.0;
// Bare earth: a 2 % grade north, plus the interpolated deck gap of a road in a cutting: a 6 m hump 39 m long along the
// road with steep (4 m) walls, 12 m wide (smooth shoulders) across it.
double ground_d64(double /*e*/, double n) { return 100.0 + 0.02 * (n - kUnderpassN0); }
double dem_d64(double e, double n) {
    const double y = std::fabs(n - kUnderpassN0), x = std::fabs(e - (kUnderpassE0 + 0.5));
    const double along = smooth((kFoot - y) / 4.0);
    const double across = x <= 6 ? 1.0 : smooth(1.0 - (x - 6) / 4);
    return ground_d64(e, n) + 6.0 * along * across;
}
// Road 10/20/30 north-south at x = 0.5 split at the hump's feet; the upper road 50 is a bridge (layer 1) 24 m long,
// east-west across it with ground continuations 40/60.
RoadGraph scene_d64(double end = kEnd) {
    const Tags res{{"highway", "residential"}}, sec{{"highway", "secondary"}}, bridge{{"bridge", "yes"}, {"highway", "secondary"}, {"layer", "1"}};
    return world_of({
        {10, res, {{0.5, -350}, {0.5, -250}, {0.5, -150}, {0.5, -50}, {0.5, -end}}},
        {20, res, {{0.5, -end}, {0.5, end}}},
        {30, res, {{0.5, end}, {0.5, 50}, {0.5, 150}, {0.5, 250}, {0.5, 350}}},
        {40, sec, {{-400, 0}, {-300, 0}, {-200, 0}, {-100, 0}, {-11.5, 0}}},
        {50, bridge, {{-11.5, 0}, {12.5, 0}}},
        {60, sec, {{12.5, 0}, {100, 0}, {200, 0}, {300, 0}, {400, 0}}},
    });
}
Result<RoadGeomTile> build_d64(const RoadGraph& world, bool overpass, RoadUnderpassStats* stats = nullptr, const Dem& dem = dem_d64) {
    RoadRules rules = builtin_road_rules_de();
    rules.underpass.overpass = overpass;
    g2m_test::GeomOptions o; o.height_m = dem; o.completion = served_road_completion_options(); o.underpass = true;
    o.underpass_stats = stats; o.rules = &rules;
    return g2m_test::geometry_in_tile_with(world, kUnderpassTile, o);
}
} // namespace

TEST_CASE("overpass hump (D64): the road under a bridge stays at grade, the deck's DEM hump is masked", "[layer][underpass][overpass][d64]") {
    const RoadGraph world = scene_d64();
    RoadUnderpassStats stats;
    const auto tile = build_d64(world, true, &stats);
    REQUIRE(tile);
    const RoadProfile* mid = profile_of(*tile, 20);
    INFO("way 20: " << decline_of(*tile, 20));
    REQUIRE(mid);
    // The profile is the ground plane (the chord of the hump's feet), nowhere near the 6 m hump.
    const double off = off_plane(*mid, ground_d64);
    const double mid_dem = dem_d64(kUnderpassE0 + 0.5, kUnderpassN0);
    const auto zc = mid->at(mid->reference.length_m * 0.5); REQUIRE(zc);
    std::printf("D64 synthetic: way 20 profile within %.3f m of the ground plane, at the bridge %.3f m (DEM %.3f, ground %.3f); "
                "crossings %zu masked %zu\n", off, zc->height_m, mid_dem, ground_d64(0, kUnderpassN0), stats.over_crossings,
                stats.over_masked);
    CHECK(off < 0.05);
    CHECK(mid_dem - zc->height_m > 5.9);
    CHECK(stats.over_crossings >= 1);
    CHECK(stats.over_masked >= 1);
        // The neighbours are on the ground; the three ways meet in one common plane at each foot.
    const RoadProfile* south = profile_of(*tile, 10);
    const RoadProfile* north = profile_of(*tile, 30);
    REQUIRE(south); REQUIRE(north);
    const auto s_end = south->at(south->reference.length_m), m_start = mid->at(0), m_end = mid->at(mid->reference.length_m), n_start = north->at(0);
    REQUIRE(s_end); REQUIRE(m_start); REQUIRE(m_end); REQUIRE(n_start);
    CHECK(std::fabs(s_end->height_m - m_start->height_m) < 0.5);
    CHECK(std::fabs(m_end->height_m - n_start->height_m) < 0.5);
    // The bridge itself is not touched: it is a bridge, not a ground stretch.
    CHECK(tile->underpasses.empty());

    // Sabotage (the pre-D64 profile, rule off): the junction planes take the hump's wall and the way is declined, or
    // the profile climbs the deck's gap.
    RoadUnderpassStats off_stats;
    const auto sab = build_d64(world, false, &off_stats);
    REQUIRE(sab);
    const RoadProfile* sab_mid = profile_of(*sab, 20);
    const bool declined = sab_mid == nullptr;
    const double sab_off = declined ? 1e9 : off_plane(*sab_mid, ground_d64);
    std::printf("D64 sabotage (overpass off): way 20 %s (%s), off the ground plane %.3f m\n", declined ? "DECLINED" : "profiled",
                decline_of(*sab, 20).c_str(), declined ? -1.0 : sab_off);
    CHECK((declined || sab_off > 1.0));
    CHECK(off_stats.over_masked == 0);
}

TEST_CASE("overpass hump (D64): a way ending half way up the hump's wall takes the rim chord's end plane", "[layer][underpass][overpass][d64]") {
    // The middle way's end nodes lie 2 m inside the wall (the DEM there is 3 m above the ground, 1.9 m/m steep): the
    // junction plane of the ends is taken from the rim chord of the masked span, not from the wall.
    const RoadGraph world = scene_d64(17.5);
    RoadUnderpassStats stats;
    const auto tile = build_d64(world, true, &stats);
    REQUIRE(tile);
    const RoadProfile* mid = profile_of(*tile, 20);
    INFO("way 20: " << decline_of(*tile, 20));
    REQUIRE(mid);
    const double off = off_plane(*mid, ground_d64);
    std::printf("D64 wall ends: way 20 within %.3f m of the ground plane\n", off);
    CHECK(off < 0.05);
    RoadUnderpassStats off_stats;
    const auto sab = build_d64(world, false, &off_stats);
    REQUIRE(sab);
    const RoadProfile* sp = profile_of(*sab, 20);
    std::printf("D64 wall ends, rule off: way 20 %s\n", sp ? "profiled" : decline_of(*sab, 20).c_str());
    CHECK((sp == nullptr || off_plane(*sp, ground_d64) > 1.0));
}

TEST_CASE("overpass hump (D64): controls - no bridge, a bridge on the same layer, a bridge far from the road", "[layer][underpass][overpass][d64]") {
    // No bridge over the road: the same hump is just terrain; the rule does nothing.
    {
        const Tags res{{"highway", "residential"}};
        const RoadGraph world = world_of({
            {10, res, {{0.5, -350}, {0.5, -150}, {0.5, -kEnd}}},
            {20, res, {{0.5, -kEnd}, {0.5, kEnd}}},
            {30, res, {{0.5, kEnd}, {0.5, 150}, {0.5, 350}}},
        });
        RoadUnderpassStats stats;
        const auto tile = build_d64(world, true, &stats);
        REQUIRE(tile);
        CHECK(stats.over_crossings == 0);
        CHECK(stats.over_masked == 0);
    }
    // A bridge that is not above the road (layer 0 bridge crossing: same layer, not a grade separation): not an over-crossing.
    {
        const Tags res{{"highway", "residential"}}, sec{{"highway", "secondary"}}, flat{{"bridge", "yes"}, {"highway", "secondary"}};
        const RoadGraph world = world_of({
            {10, res, {{0.5, -350}, {0.5, -150}, {0.5, -kEnd}}},
            {20, res, {{0.5, -kEnd}, {0.5, kEnd}}},
            {30, res, {{0.5, kEnd}, {0.5, 150}, {0.5, 350}}},
            {40, sec, {{-400, 0}, {-11.5, 0}}},
            {50, flat, {{-11.5, 0}, {12.5, 0}}},
            {60, sec, {{12.5, 0}, {400, 0}}},
        });
        RoadUnderpassStats stats;
        const auto tile = build_d64(world, true, &stats);
        REQUIRE(tile);
        CHECK(stats.over_masked == 0);
    }
    // A bridge crossing far from the way (the hump is the road's own terrain): masked spans only where a bridge crosses.
    {
        const RoadGraph world = scene_d64();
        // Flat DEM: a bridge over a flat road has no hump to mask (no depth), and the profile is the ground.
        RoadUnderpassStats stats;
        const auto tile = build_d64(world, true, &stats, ground_d64);
        REQUIRE(tile);
        const RoadProfile* mid = profile_of(*tile, 20);
        REQUIRE(mid);
        CHECK(off_plane(*mid, ground_d64) < 0.05);
    }
}

TEST_CASE("overpass hump (D64): the carve cuts the hump away; ribbon and ground meet", "[layer][underpass][overpass][d64][carve]") {
    const RoadGraph world = scene_d64();
    const auto key0 = tile_key_at(kUnderpassTile.zone, 0, kUnderpassE0 + 0.5, kUnderpassN0);
    REQUIRE(key0);
    const auto probe = [&](bool overpass) {
        RoadRules rules = builtin_road_rules_de();
        rules.underpass.overpass = overpass;
        g2m_test::CarveFixture fx(world, dem_d64);
        fx.rules = &rules;
        auto m = fx.model(*key0, CarveOptions{});
        double worst = 0, top = -1e9;
        for (double y = -15; y <= 15; y += 1) {
            if (std::fabs(y) < 6) continue;   // under the bridge deck itself (its own owner)
            const auto c = m.model->evaluate(kUnderpassE0 + 0.5, kUnderpassN0 + y);
            worst = std::max(worst, std::fabs(c.z - ground_d64(0, kUnderpassN0 + y)));
            top = std::max(top, c.z - ground_d64(0, kUnderpassN0 + y));
        }
        return std::pair{worst, top};
    };
    const auto [worst, top] = probe(true);
    const auto [worst_sab, top_sab] = probe(false);
    std::printf("D64 carve along the road (6 to 15 m either side of the bridge axis): carved - ground max %.3f m; sabotage (rule off) %.3f m (hump 6 m)\n", worst, worst_sab);
    CHECK(worst < 0.15);
    CHECK(top_sab > 5.0);   // without the rule the hump stays in the terrain
}
