// tests/layer/test_road_overpass.cpp - owner report 2026-10-19 (two Frankfurt-west sites):
//  D64: a ground road under an upper bridge. DGM1 is bare earth, the bridge deck is removed and its gap interpolated,
//       so the DEM shows a hump over the road; the strict profile used to follow it, the junction planes at the way's
//       ends took the hump's wall and the way was declined (no ribbon, no carve: "the tunnel ends in a wall").
//  D65: a short straight tunnel under an embankment (a railway on a bank, tagged tunnel in OSM) takes the portal chord
//       as its floor and the terrain carve cuts the bank, instead of a DEM-following floor that climbs the ridge.
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
double max_above(const RoadProfile& p, const Dem& plane) {
    double worst = -1e9;
    for (double s = 0; s <= p.reference.length_m; s += 1) {
        const auto xy = p.reference.at(s); const auto z = p.at(s); REQUIRE(xy); REQUIRE(z);
        worst = std::max(worst, z->height_m - plane(xy->x, xy->y));
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
    g2m_test::GeomOptions o; o.height_m = dem; o.completion = g2m_test::served_7a1(); o.underpass = true;
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

// ---- D65: the short tunnel under a bank ------------------------------------------------------------------------
namespace {
constexpr double kBank = 4.5;
double ground_d65(double /*e*/, double n) { return 99.5 - 0.01 * (n - kUnderpassN0); }
// A bank across the road (ridge east-west), `half` m wide at the base, `height` m high, flat-topped (smooth shoulders).
Dem dem_d65(double half, double height) {
    return [=](double e, double n) {
        const double y = std::fabs(n - kUnderpassN0);
        return ground_d65(e, n) + height * smooth((half - y) / (half - 8.0));
    };
}
RoadGraph scene_d65(double tunnel_half, bool crossing = false) {
    const Tags res{{"highway", "residential"}}, tun{{"highway", "residential"}, {"layer", "-1"}, {"tunnel", "yes"}};
    std::vector<WaySpec> extra;
    // A ground road over the tunnel at the bank crest (no shared node): the A66 carriageways over a service tunnel.
    if (crossing) extra.push_back({40, res, {{-300, 0}, {-150, 0}, {150, 0}, {300, 0}}});
    std::vector<WaySpec> main = {
        {10, res, {{0.5, -350}, {0.5, -250}, {0.5, -150}, {0.5, -tunnel_half}}},
        {20, tun, {{0.5, -tunnel_half}, {0.5, tunnel_half}}},
        {30, res, {{0.5, tunnel_half}, {0.5, 150}, {0.5, 250}, {0.5, 350}}},
    };
    main.insert(main.end(), extra.begin(), extra.end());
    return world_of(main);
}
struct D65Run { const RoadProfile* p = nullptr; RoadGeomTile tile; RoadCompletionStats stats; };
// roads.geom of the scene with the served completion options, `opt` applied on top.
Result<RoadGeomTile> build_d65(const RoadGraph& world, const Dem& dem, RoadCompletionStats* stats, const std::function<void(RoadCompletionOptions&)>& opt = {}) {
    g2m_test::GeomOptions o; o.height_m = dem; o.completion = g2m_test::served_7a1(); o.underpass = true; o.stats = stats;
    if (opt) opt(o.completion);
    return g2m_test::geometry_in_tile_with(world, kUnderpassTile, o);
}
// Straight? max |h(s) - chord| over the whole metres.
double off_chord(const RoadProfile& p) {
    const auto a = p.at(0), b = p.at(p.reference.length_m); REQUIRE(a); REQUIRE(b);
    double worst = 0;
    for (double s = 0; s <= p.reference.length_m; s += 1) {
        const auto z = p.at(s); REQUIRE(z);
        worst = std::max(worst, std::fabs(z->height_m - (a->height_m + (b->height_m - a->height_m) * s / p.reference.length_m)));
    }
    return worst;
}
} // namespace

TEST_CASE("tunnel cut (D65): a 46 m tunnel under a 4.5 m bank takes the portal chord as its floor", "[layer][carve][tunnel][d65]") {
    const double half = 23.15;
    const RoadGraph world = scene_d65(half);
    const Dem dem = dem_d65(half, kBank);
    RoadCompletionStats stats;
    const auto tile = build_d65(world, dem, &stats);
    REQUIRE(tile);
    const RoadProfile* p = profile_of(*tile, 20);
    INFO("tunnel: " << decline_of(*tile, 20));
    REQUIRE(p);
    const auto a = p->at(0), b = p->at(p->reference.length_m); REQUIRE(a); REQUIRE(b);
    const double peak = max_above(*p, ground_d65);
    std::printf("D65 synthetic: tunnel %.1f m, floor %.3f -> %.3f m, off the chord %.3f m, highest above ground %.3f m; cut %zu chord %zu\n",
                p->reference.length_m, a->height_m, b->height_m, off_chord(*p), peak, stats.tunnel_cut, stats.tunnel_chord);
    CHECK(stats.tunnel_cut == 1);
    CHECK(off_chord(*p) < 0.01);
    CHECK(peak < 0.1);   // the floor never leaves the portal level; the bank is above it

    // Sabotage: the cut rule off (cut_max_length 0): the floor follows the DEM over the bank.
    RoadCompletionStats off_stats;
    const auto sab = build_d65(world, dem, &off_stats, [](RoadCompletionOptions& o) { o.tunnel_cut_max_length_m = 0; });
    REQUIRE(sab);
    const RoadProfile* sp = profile_of(*sab, 20);
    REQUIRE(sp);
    const double sab_peak = max_above(*sp, ground_d65);
    std::printf("D65 sabotage (cut off): floor climbs %.3f m above the portals\n", sab_peak);
    CHECK(off_stats.tunnel_cut == 0);
    CHECK(sab_peak > 2.0);
}

TEST_CASE("tunnel cut (D65): controls - a long tunnel, a bank over the hump bound, a tunnel on flat ground", "[layer][carve][tunnel][d65]") {
    // 90 m tunnel: over cut_max_length (80 m, de.json): DEM-following as before.
    {
        const double half = 45.0;
        RoadCompletionStats stats;
        const auto tile = build_d65(scene_d65(half), dem_d65(half + 0.0, 3.0), &stats);
        REQUIRE(tile);
        CHECK(stats.tunnel_cut == 0);
    }
    // Hump above the bound: not a bank to cut (a hill) - bound lowered to 3 m for a 4.5 m bank.
    {
        const double half = 23.15;
        RoadCompletionStats stats;
        const auto tile = build_d65(scene_d65(half), dem_d65(half, kBank), &stats, [](RoadCompletionOptions& o) { o.tunnel_cut_hump_max_m = 3.0; });
        REQUIRE(tile);
        CHECK(stats.tunnel_cut == 0);
    }
    // Hump below the lower bound: the DEM follows the chord anyway, nothing to cut.
    {
        const double half = 23.15;
        RoadCompletionStats stats;
        const auto tile = build_d65(scene_d65(half), dem_d65(half, 0.4), &stats);
        REQUIRE(tile);
        CHECK(stats.tunnel_cut == 0);
    }
}

TEST_CASE("tunnel cut (D65): the terrain carve cuts the bank along the tunnel; a long tunnel is still never carved", "[layer][carve][tunnel][d65]") {
    const auto key0 = tile_key_at(kUnderpassTile.zone, 0, kUnderpassE0 + 0.5, kUnderpassN0);
    REQUIRE(key0);
    const auto run = [&](double half, double bank, CarveOptions options) {
        const RoadGraph world = scene_d65(half);
        g2m_test::CarveFixture fx(world, dem_d65(half, bank));
        fx.completion = g2m_test::served_7a1();   // D65 carve: the floors of the 7a-1 release (the carve of the design floor is phase 7b)
        auto m = fx.model(*key0, options);
        const RoadGeomTile& geom = fx.geom(tile_key_at(kUnderpassTile.zone, 2, kUnderpassE0 + 0.5, kUnderpassN0).value());
        const RoadProfile* p = profile_of(geom, 20);
        REQUIRE(p);
        double worst_dev = 0, centre_above = -1e9, centre_dem_above = -1e9;
        for (double s = 2; s <= p->reference.length_m - 2; s += 1) {
            const auto xy = p->reference.at(s); const auto z = p->at(s); REQUIRE(xy); REQUIRE(z);
            const auto c = m.model->evaluate(xy->x, xy->y);
            worst_dev = std::max(worst_dev, std::fabs(c.z - z->height_m));
            centre_above = std::max(centre_above, c.z - ground_d65(xy->x, xy->y));
            centre_dem_above = std::max(centre_dem_above, dem_d65(half, bank)(xy->x, xy->y) - ground_d65(xy->x, xy->y));
        }
        return std::tuple{worst_dev, centre_above, centre_dem_above, m.stats.tunnel_entries};
    };
    const auto [dev, above, dem_above, tunnels] = run(23.15, kBank, CarveOptions{});
    std::printf("D65 carve: carved - tunnel floor at most %.3f m along the tunnel; carved centre above the portal level at most %.3f m (DEM bank %.2f m)\n",
                dev, above, dem_above);
    CHECK(dev < 0.05);
    CHECK(above < 0.1);
    CHECK(dem_above > 4.0);
    CHECK(tunnels == 0);

    // Sabotage (tunnel_cut off): never carved, the terrain along the tunnel stays the 4.5 m bank.
    CarveOptions sabotage; sabotage.tunnel_cut = false;
    const auto [dev_s, above_s, dem_above_s, tunnels_s] = run(23.15, kBank, sabotage);
    std::printf("D65 carve sabotage: carved centre above the portal level up to %.3f m, tunnel entries skipped %zu\n", above_s, tunnels_s);
    CHECK(above_s > 4.0);
    CHECK(tunnels_s == 1);
    (void)dev_s; (void)dem_above_s;

    // Control: a long (90 m) tunnel is not cut: the bank stays.
    const auto [dev_l, above_l, dem_above_l, tunnels_l] = run(45.0, 3.0, CarveOptions{});
    std::printf("D65 control, 90 m tunnel: carved centre above the portal level up to %.3f m, skipped %zu\n", above_l, tunnels_l);
    CHECK(above_l > 2.5);
    CHECK(tunnels_l == 1);
    (void)dev_l; (void)dem_above_l;
}

TEST_CASE("tunnel cut (D65): a ground road over the tunnel keeps its terrain - the cut is not carved under it", "[layer][carve][tunnel][d65]") {
    const double half = 23.15;
    const auto key0 = tile_key_at(kUnderpassTile.zone, 0, kUnderpassE0 + 0.5, kUnderpassN0);
    REQUIRE(key0);
    const auto run = [&](CarveOptions options) {
        const RoadGraph world = scene_d65(half, true);
        g2m_test::CarveFixture fx(world, dem_d65(half, kBank));
        fx.completion = g2m_test::served_7a1();   // D65 carve: the floors of the 7a-1 release (the carve of the design floor is phase 7b)
        auto m = fx.model(*key0, options);
        const RoadGeomTile& geom = fx.geom(tile_key_at(kUnderpassTile.zone, 2, kUnderpassE0 + 0.5, kUnderpassN0).value());
        const RoadProfile* p = profile_of(geom, 40);
        REQUIRE(p);
        double worst = 0, over = -1e9;
        for (double s = 0; s <= p->reference.length_m; s += 1) {
            const auto xy = p->reference.at(s); const auto z = p->at(s); REQUIRE(xy); REQUIRE(z);
            if (std::fabs(xy->x - (kUnderpassE0 + 0.5)) > 12.0 || std::fabs(xy->y - kUnderpassN0) > 6.0) continue;
            const double c = m.model->evaluate(xy->x, xy->y).z;
            worst = std::max(worst, std::fabs(c - z->height_m));
            over = std::max(over, z->height_m - ground_d65(xy->x, xy->y));
        }
        return std::tuple{worst, over, m.stats.tunnel_entries};
    };
    const auto [dev, over, skipped] = run(CarveOptions{});
    std::printf("D65 crossing: ground road %.3f m above the portal level at the crest; carved - profile at most %.3f m; tunnel entries skipped %zu\n", over, dev, skipped);
    CHECK(over > 3.0);
    CHECK(dev < 0.3);
    CHECK(skipped == 1);

    // Sabotage (crossing check off): the cut core slots the road above the tunnel.
    CarveOptions sabotage; sabotage.tunnel_cut_crossing_check = false;
    const auto [dev_s, over_s, skipped_s] = run(sabotage);
    std::printf("D65 crossing sabotage: carved is %.3f m off the road profile (slot), skipped %zu\n", dev_s, skipped_s);
    CHECK(dev_s > 2.0);
    CHECK(skipped_s == 0);
    (void)over_s;
}

// ---- 7a-2: the design floor must not turn every short tunnel into an open cut ---------------------------------
// The design floor of a short tunnel is its chord (the ground term is skipped within needed/G of a portal), and the carve
// recognises an open cut by a floor straight within 1 cm. D65's hump rule therefore stays the only way to the exact chord;
// any other short floor holds the portal grade over the end ramps.
TEST_CASE("tunnel cut (7a-2): the design floor keeps D65's hump rule; a tunnel under a hill is not a cut", "[layer][carve][tunnel][d65][structure]") {
    const auto design = [](RoadCompletionOptions& o) { o = g2m::served_road_completion_options(); };
    const double half = 23.15;
    for (const double height : {4.5, 15.0, 0.4}) {
        RoadCompletionStats stats;
        const auto tile = build_d65(scene_d65(half), dem_d65(half, height), &stats, design);
        REQUIRE(tile);
        const RoadProfile* p = profile_of(*tile, 20);
        REQUIRE(p);
        std::printf("[cut 7a-2] bank %.1f m: cut %zu, off the chord %.4f m, floor entries %zu\n", height, stats.tunnel_cut, off_chord(*p), stats.floor_entries);
        if (height == 4.5) { CHECK(stats.tunnel_cut == 1); CHECK(off_chord(*p) < 0.01); }
        else CHECK(stats.tunnel_cut == 0);
    }
}

TEST_CASE("tunnel cut (7a-2): the carve opens a design-floor tunnel only under a hump of ground; the hump check off is the sabotage", "[layer][carve][tunnel][d65][structure]") {
    const double half = 23.15;
    const auto key0 = tile_key_at(kUnderpassTile.zone, 0, kUnderpassE0 + 0.5, kUnderpassN0);
    REQUIRE(key0);
    const auto run = [&](double bank, CarveOptions options) {
        const RoadGraph world = scene_d65(half);
        g2m_test::CarveFixture fx(world, dem_d65(half, bank));
        fx.completion = g2m::served_road_completion_options();   // the design floor
        auto m = fx.model(*key0, options);
        const RoadGeomTile& geom = fx.geom(tile_key_at(kUnderpassTile.zone, 2, kUnderpassE0 + 0.5, kUnderpassN0).value());
        const RoadProfile* p = profile_of(geom, 20);
        REQUIRE(p);
        double above = -1e9;
        for (double s = 2; s <= p->reference.length_m - 2; s += 1) {
            const auto xy = p->reference.at(s); REQUIRE(xy);
            above = std::max(above, m.model->evaluate(xy->x, xy->y).z - ground_d65(xy->x, xy->y));
        }
        return std::pair{above, m.stats.tunnel_entries};
    };
    // A 4.5 m bank is D65's case: cut (the floor is the portal chord).
    const auto [bank_above, bank_skipped] = run(kBank, CarveOptions{});
    // A 15 m hill is above the hump bound: the tunnel is a tunnel, the hill stays whatever the floor looks like.
    const auto [hill_above, hill_skipped] = run(15.0, CarveOptions{});
    CarveOptions sabotage; sabotage.tunnel_cut_hump_check = false;
    const auto [sab_above, sab_skipped] = run(15.0, sabotage);
    std::printf("[cut 7a-2 carve] bank: carved centre %.3f m above the portal level, skipped %zu; hill: %.3f m, skipped %zu; hill with the hump check off: %.3f m, skipped %zu\n",
                bank_above, bank_skipped, hill_above, hill_skipped, sab_above, sab_skipped);
    CHECK(bank_skipped == 0); CHECK(bank_above < 0.1);
    CHECK(hill_skipped == 1); CHECK(hill_above > 10.0);
    CHECK(sab_skipped == 0);   // the sabotage opens a straight design floor under a 15 m hill (a trench through it)
    CHECK(sab_above < 0.5);
}
