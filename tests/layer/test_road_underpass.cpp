// tests/layer/test_road_underpass.cpp - street fidelity P2 slice 12 / request G-6 (terrain migration phase 6): the
// census of ground stretches over lower untagged crossings (road_underpass.h). A synthetic underpass in a DEM cutting
// shows the dip the served carve inherits from the bare-earth DEM; flat and same-layer controls; the census of both
// OSM fixtures on the flat and tilted synthetic DEMs is printed (their DEMs have no cuttings, so the dips there bound
// what the profile fit itself adds). The fixture tiles are the served ones (completion tier on).
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS // getenv/fopen below; -Werror makes the MSVC deprecation a build break
#endif
#include "carve_fixture.h"
#include "road_fixture.h"

#include "underpass_scene.h"

#include "g2m/layer/road_underpass.h"
#include "g2m/layer/terrain_carve.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <climits>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <optional>
#include <cstdio>
#include <functional>
#include <set>
#include <tuple>
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
// The census of the tile without the underpass pass (the profile as it was before D53), or of the served tile.
UnderpassCensus census_of(const RoadGraph& world, const std::function<double(double, double)>& dem, bool underpass = false) {
    g2m_test::GeomOptions o; o.height_m = dem; o.completion = served_road_completion_options(); o.underpass = underpass;
    const auto tile = g2m_test::geometry_in_tile_with(world, kTile, o);
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
    // Served (underpass decks on, D53): the same crossing no longer dips.
    const auto served = census_of(scene({{"layer", "-1"}}), cutting, true);
    REQUIRE(served.crossings.size() == 1);
    CHECK(served.crossings.front().dip_m < 0.1);
    CHECK(served.over_threshold == 0);

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
                for (const auto& u : tile->underpasses) {
                    const auto& e = tile->entries[u.entry_index];
                    std::printf("  served deck %s %s: tile %d/%d way %lld [%.3f, %.3f] of %.1f m, depth %d mm, lower way %lld\n", file,
                                tilted ? "tilted" : "flat", key.x, key.y, static_cast<long long>(e.way_id), u.start_m, u.end_m,
                                e.profile->reference.length_m, static_cast<int>(u.depth_mm), static_cast<long long>(u.lower_ways.front()));
                    CHECK(u.end_m - u.start_m <= 80.0);
                }
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

// ---- Underpass decks (owner decision 2026-10-15, racing_game PR #5; decisions D53-D58) ----

namespace {
using g2m_test::kUnderpassE0;
using g2m_test::kUnderpassTile;
using g2m_test::underpass_cutting;
using g2m_test::underpass_scene;
using g2m_test::UnderpassSceneOptions;
using Dem = std::function<double(double, double)>;

// The cutting of the scene on a 2 % grade east (so "on the ground" is not trivially flat).
Dem tilted_cutting(double lower_x = 0.5, double depth = 6.0) {
    return [=](double e, double) { return underpass_cutting(e, lower_x, depth) + 0.02 * (e - kUnderpassE0); };
}
Result<RoadGeomTile> build(const RoadGraph& world, const TileKey& key, const Dem& dem, bool underpass = true,
                           RoadUnderpassStats* stats = nullptr, bool crossing_context = true) {
    g2m_test::GeomOptions o; o.height_m = dem; o.completion = served_road_completion_options();
    o.underpass = underpass; o.underpass_stats = stats; o.crossing_context = crossing_context;
    return g2m_test::geometry_in_tile_with(world, key, o);
}
// The entry of `way` whose reference passes nearest the easting kUnderpassE0 + x (the station where it does).
std::size_t entry_at(const RoadGeomTile& t, std::int64_t way, double x, double* station = nullptr) {
    for (std::size_t i = 0; i < t.entries.size(); ++i) {
        const auto& e = t.entries[i];
        if (e.way_id != way || !e.profile) continue;
        const auto& ref = e.profile->reference;
        const auto a = ref.at(0), b = ref.at(ref.length_m);
        if (!a || !b) continue;
        const double ex = kUnderpassE0 + x;
        if ((a->x - ex) * (b->x - ex) > 0) continue;
        double lo = 0, hi = ref.length_m;   // the scene's upper reference runs east, x increasing with the station
        for (int k = 0; k < 60; ++k) { const double mid = 0.5 * (lo + hi); (ref.at(mid)->x < ex ? lo : hi) = mid; }
        if (station) *station = 0.5 * (lo + hi);
        return i;
    }
    std::string why;
    for (const auto& e : t.entries) if (e.way_id == way) why += " [" + std::to_string(e.stretch.start_ref) + "-" + std::to_string(e.stretch.end_ref) + ": " + (e.profile ? "profile" : e.decline->message) + "]";
    FAIL("no entry of way " << way << " at x " << x << why);
    return 0;
}
// |profile - DEM| over every whole-metre station outside [from, to] (DEM on the reference line).
double off_ground_outside(const RoadProfile& p, const Dem& dem, double from, double to) {
    double worst = 0;
    for (double s = 0; s <= p.reference.length_m; s += 1) {
        if (s >= from && s <= to) continue;
        const auto xy = p.reference.at(s); const auto z = p.at(s);
        REQUIRE(xy); REQUIRE(z);
        worst = std::max(worst, std::fabs(z->height_m - dem(xy->x, xy->y)));
    }
    return worst;
}
// Bitwise equality of two vertical profiles (VerticalCubic has no operator==).
bool same_vertical(const VerticalProfile& a, const VerticalProfile& b) {
    if (a.segments.size() != b.segments.size()) return false;
    for (std::size_t i = 0; i < a.segments.size(); ++i) {
        const auto& x = a.segments[i]; const auto& y = b.segments[i];
        if (std::tie(x.station_m, x.length_m, x.height0_m, x.height1_m, x.grade0, x.grade1) !=
            std::tie(y.station_m, y.length_m, y.height0_m, y.height1_m, y.grade0, y.grade1)) return false;
    }
    return true;
}
} // namespace

TEST_CASE("underpass deck (D53-D58): only the span over the cutting is a deck; the road stays on the ground outside it",
          "[layer][underpass][deck]") {
    const RoadGraph world = underpass_scene();
    const Dem dem = tilted_cutting();
    RoadUnderpassStats stats;
    const auto tile = build(world, kUnderpassTile, dem, true, &stats);
    REQUIRE(tile);
    CHECK(tile->encoding == kRoadGeomEncoding);
    REQUIRE(tile->underpasses.size() == 1);
    const RoadUnderpass& u = tile->underpasses.front();
    double s_c = 0;
    const std::size_t upper = entry_at(*tile, 100, 0.5, &s_c);
    CHECK(u.entry_index == upper);
    CHECK(u.lower_ways == std::vector<std::int64_t>{200});
    CHECK(u.depth_mm >= 5900); CHECK(u.depth_mm <= 6100);
    const RoadProfile& p = *tile->entries[upper].profile;
    const double L = p.reference.length_m;
    const auto lower_attrs = resolve_road_attributes(world.ways[1]); REQUIRE(lower_attrs);
    const double half_lower = lower_attrs->width_mm / 2000.0;
    std::printf("D53 synthetic: stretch %.1f m, crossing at %.2f, deck [%.3f, %.3f] (%.1f m), depth %d mm, lower half width %.2f m\n",
                L, s_c, u.start_m, u.end_m, u.end_m - u.start_m, u.depth_mm, half_lower);
    // The deck covers the lower road plus the 1 m margin ...
    CHECK(u.start_m <= s_c - half_lower - 1.0);
    CHECK(u.end_m >= s_c + half_lower + 1.0);
    // ... and never extends beyond the crossing span: the cutting's rims are 12 m either side (the first draft decked
    // the whole stretch and floated the road 0.55 m over the ground).
    CHECK(u.start_m >= s_c - 12.0 - 1e-3);
    CHECK(u.end_m <= s_c + 12.0 + 1e-3);
    CHECK(u.end_m - u.start_m < 0.05 * L);
    // Across the deck the profile stays at the rim chord (the 2 % grade through the rims), not in the cutting.
    double worst_sag = 0;
    for (double s = u.start_m; s <= u.end_m; s += 0.25) {
        const auto xy = p.reference.at(s); const auto z = p.at(s); REQUIRE(xy); REQUIRE(z);
        worst_sag = std::max(worst_sag, (100.0 + 0.02 * (xy->x - kUnderpassE0)) - z->height_m);
    }
    CHECK(worst_sag <= 0.1);
    // Outside it the road is on the ground.
    const double off = off_ground_outside(p, dem, u.start_m, u.end_m);
    std::printf("D53 synthetic: worst sag over the deck %.3f m, worst |profile - DEM| outside it %.3f m\n", worst_sag, off);
    CHECK(off <= 0.1);
    CHECK(stats.crossings == 1); CHECK(stats.served == 1); CHECK(stats.decks == 1);

    // Before D53 (no mask, no deck): the profile sags into the cutting and nothing is served.
    const auto before = build(world, kUnderpassTile, dem, false);
    REQUIRE(before);
    CHECK(before->encoding == kRoadGeomEncodingTunnels);
    CHECK(before->underpasses.empty());
    const auto z_before = before->entries[entry_at(*before, 100, 0.5)].profile->at(s_c); REQUIRE(z_before);
    const auto z_after = p.at(s_c); REQUIRE(z_after);
    std::printf("D53 synthetic: profile at the crossing %.3f m before, %.3f m after (rim chord %.3f m)\n", z_before->height_m, z_after->height_m,
                100.0 + 0.02 * 0.5);
    CHECK(z_before->height_m < 100.0 - 0.3);
    CHECK(z_after->height_m > 100.0 - 0.1);

    // The rules switch: enabled = false serves no deck and leaves the profile on the DEM, as before.
    RoadRules off_rules = builtin_road_rules_de(); off_rules.underpass.enabled = false;
    g2m_test::GeomOptions o; o.height_m = dem; o.completion = served_road_completion_options(); o.underpass = true; o.rules = &off_rules;
    const auto switched = g2m_test::geometry_in_tile_with(world, kUnderpassTile, o);
    REQUIRE(switched);
    CHECK(switched->underpasses.empty());
    CHECK(same_vertical(switched->entries[entry_at(*switched, 100, 0.5)].profile->vertical, before->entries[entry_at(*before, 100, 0.5)].profile->vertical));
}

TEST_CASE("underpass deck (D53-D58): what is not a deck", "[layer][underpass][deck]") {
    // Flat: no depression. Shallow: 0.8 m under the rims (below min_depth). Same layer: not a lower road. Lower bridge:
    // above, not below. A V valley with 20 % sides: the hull reaches the window edge (no rim, a valley, not a cutting).
    struct Case { const char* name; UnderpassSceneOptions scene; Dem dem; std::size_t RoadUnderpassStats::*counter; };
    UnderpassSceneOptions same; same.lower_tags = {};
    UnderpassSceneOptions bridge; bridge.lower_tags = {{"bridge", "yes"}, {"layer", "1"}};
    const Case cases[] = {
        {"flat", {}, [](double, double) { return 100.0; }, &RoadUnderpassStats::shallow},
        {"shallow", {}, tilted_cutting(0.5, 0.8), &RoadUnderpassStats::shallow},
        {"valley", {}, [](double e, double) { return 90.0 + 0.2 * std::fabs(e - kUnderpassE0 - 0.5); }, &RoadUnderpassStats::no_rim},
        {"same layer", same, tilted_cutting(), nullptr},
        {"lower bridge", bridge, tilted_cutting(), nullptr},
    };
    for (const auto& c : cases) {
        CAPTURE(c.name);
        const RoadGraph world = underpass_scene(c.scene);
        // The finder directly (the valley's 20 % grades leave the stretch without a profile, so no deck is recorded).
        const auto in = g2m_test::road_geom_inputs(world, kUnderpassTile); REQUIRE(in);
        std::vector<HeightTile> heights(in->dem_keys.size());
        std::vector<const HeightTile*> views;
        for (std::size_t i = 0; i < heights.size(); ++i) { heights[i].key = in->dem_keys[i]; g2m_test::fill_dem(heights[i], c.dem); views.push_back(&heights[i]); }
        const auto dem = RoadDemSampler::make(views); REQUIRE(dem);
        const auto builder = RoadReferenceBuilder::make(in->context); REQUIRE(builder);
        const auto stretches = builder->stretches(100); REQUIRE(stretches); REQUIRE(stretches->size() == 1);
        ReferenceFitLimits fit; fit.allow_curvature_exceptions = true;
        const auto ref = builder->fit_stretch(100, stretches->front(), fit); REQUIRE(ref);
        const RoadUnderpassFinder finder(in->crossings, *dem, kUnderpassTile.zone, builtin_road_rules_de().underpass);
        RoadUnderpassStats stats;
        const auto spans = finder.spans(100, stretches->front(), *ref, &stats);
        CHECK(spans.empty());
        if (c.counter) { CHECK(stats.crossings == 1); CHECK(stats.*c.counter == 1); }
        else CHECK(stats.crossings == 0);
    }
}

TEST_CASE("underpass deck (D55): a crossing near a stretch end (the census's 20 m gap) is decked when the deck fits",
          "[layer][underpass][deck]") {
    // The Kalbach case: a side road splits the upper way 18 m before the crossing; the window reads on along the
    // tangent of the stretch end. With a 5 m end clearance (test rules) the same deck, 6 m into the stretch, still fits;
    // 14.5 m before the crossing its deck would start 2.5 m into the stretch: declined (reaches_end), never stretched.
    struct Case { double side_x; std::int32_t clearance_mm; bool served; };
    for (const Case c : {Case{-17.5, 1000, true}, Case{-17.5, 5000, true}, Case{-14.0, 5000, false}}) {
        CAPTURE(c.side_x, c.clearance_mm);
        UnderpassSceneOptions o; o.side_road_x = c.side_x;
        const RoadGraph world = underpass_scene(o);
        RoadRules rules = builtin_road_rules_de(); rules.underpass.end_clearance_mm = c.clearance_mm;
        RoadUnderpassStats stats;
        g2m_test::GeomOptions go; go.height_m = tilted_cutting(); go.completion = served_road_completion_options(); go.underpass = true;
        go.underpass_stats = &stats; go.rules = &rules;
        const auto tile = g2m_test::geometry_in_tile_with(world, kUnderpassTile, go);
        REQUIRE(tile);
        double s_c = 0;
        const std::size_t upper = entry_at(*tile, 100, 0.5, &s_c);
        CHECK(std::fabs(s_c - (0.5 - c.side_x)) < 0.6);
        const auto census = road_underpass_census(*tile);
        CHECK(census.short_side == 1);   // the census still cannot measure it
        const auto z = tile->entries[upper].profile->at(s_c); REQUIRE(z);
        if (c.served) {
            REQUIRE(tile->underpasses.size() == 1);
            const auto& u = tile->underpasses.front();
            CHECK(u.entry_index == upper);
            CHECK(u.start_m >= c.clearance_mm / 1000.0);
            CHECK(u.start_m >= s_c - 12.0 - 1e-3); CHECK(u.end_m <= s_c + 12.0 + 1e-3);
            CHECK(z->height_m > 100.0 - 0.1);
            std::printf("D55 near end: crossing %.2f m from the stretch start, deck [%.3f, %.3f], profile there %.3f m\n", s_c, u.start_m, u.end_m, z->height_m);
        } else {
            CHECK(tile->underpasses.empty());
            CHECK(stats.reaches_end == 1);
            CHECK(z->height_m < 100.0 - 0.1);   // no deck, no mask: the profile sags toward the DEM as before
        }
    }
}

TEST_CASE("underpass deck (D57): tile invariance needs the crossing context", "[layer][underpass][deck][invariance]") {
    // The upper way runs 2 km east; the lower road crosses 1200 m east of the centre tile's middle, outside its graph
    // (tile + 128 m halo). Both level-2 tiles holding the upper stretch must serve the same deck and profile.
    UnderpassSceneOptions o; o.upper_to = 1600; o.lower_x = 1200.5;
    const RoadGraph world = underpass_scene(o);
    const Dem dem = tilted_cutting(1200.5);
    const TileKey east{kUnderpassTile.zone, 2, kUnderpassTile.x + 1, kUnderpassTile.y};
    const auto a = build(world, kUnderpassTile, dem);
    const auto b = build(world, east, dem);
    REQUIRE(a); REQUIRE(b);
    const std::size_t ea = entry_at(*a, 100, 1200.5), eb = entry_at(*b, 100, 1200.5);
    REQUIRE(a->underpasses.size() == 1); REQUIRE(b->underpasses.size() == 1);
    const auto same = [](const RoadUnderpass& x, const RoadUnderpass& y) {
        return x.start_m == y.start_m && x.end_m == y.end_m && x.depth_mm == y.depth_mm && x.lower_ways == y.lower_ways && x.flags == y.flags;
    };
    CHECK(a->underpasses.front().entry_index == ea); CHECK(b->underpasses.front().entry_index == eb);
    CHECK(same(a->underpasses.front(), b->underpasses.front()));
    CHECK(same_vertical(a->entries[ea].profile->vertical, b->entries[eb].profile->vertical));
    // Negative control: the tile graph alone does not hold the lower way in the centre tile, so it serves no deck and
    // its profile of the same stretch differs from the east tile's.
    const auto alone = build(world, kUnderpassTile, dem, true, nullptr, false);
    REQUIRE(alone);
    CHECK(alone->underpasses.empty());
    CHECK_FALSE(same_vertical(alone->entries[entry_at(*alone, 100, 1200.5)].profile->vertical, b->entries[eb].profile->vertical));
    // Determinism: the same inputs give the same bytes.
    const auto again = build(world, kUnderpassTile, dem);
    REQUIRE(again);
    const auto x = encode_road_geom(*a), y = encode_road_geom(*again); REQUIRE(x); REQUIRE(y);
    CHECK(*encode_body(*x) == *encode_body(*y));
}

TEST_CASE("underpass deck (D56): encoding 9 round trip and validation", "[layer][underpass][deck][format]") {
    const RoadGraph world = underpass_scene();
    const auto tile = build(world, kUnderpassTile, tilted_cutting());
    REQUIRE(tile); REQUIRE(tile->underpasses.size() == 1);
    const auto body = encode_road_geom(*tile); REQUIRE(body);
    CHECK(body->encoding == 9); REQUIRE(body->sections.size() == 8); CHECK(body->sections[7].type == 8);
    const auto back = decode_road_geom(*body); REQUIRE(back);
    CHECK(back->underpasses == tile->underpasses);
    CHECK(back->encoding == 9);
    // Encoding 8 cannot carry a deck; an encoding-8 body has 7 sections and decodes without decks.
    auto eight = *tile; eight.encoding = 8; CHECK_FALSE(encode_road_geom(eight));
    eight.underpasses.clear(); const auto body8 = encode_road_geom(eight); REQUIRE(body8);
    CHECK(body8->sections.size() == 7);
    // Section count must match the encoding.
    auto short9 = *body; short9.sections.pop_back(); CHECK_FALSE(decode_road_geom(short9));
    auto long8 = *body; long8.encoding = 8; CHECK_FALSE(decode_road_geom(long8));
    const auto rejects = [&](auto mutate) { auto t = *tile; mutate(t); return !validate_road_geom_tile(t); };
    CHECK(rejects([](RoadGeomTile& t) { t.underpasses[0].end_m = t.underpasses[0].start_m; }));
    CHECK(rejects([](RoadGeomTile& t) { t.underpasses[0].start_m = 0; }));
    CHECK(rejects([](RoadGeomTile& t) { t.underpasses[0].end_m = t.entries[t.underpasses[0].entry_index].profile->reference.length_m + 1; }));
    CHECK(rejects([](RoadGeomTile& t) { t.underpasses[0].start_m += 0.0004; }));   // integer mm
    CHECK(rejects([](RoadGeomTile& t) { t.underpasses[0].flags = 1; }));
    CHECK(rejects([](RoadGeomTile& t) { t.underpasses[0].depth_mm = 0; }));
    CHECK(rejects([](RoadGeomTile& t) { t.underpasses[0].lower_ways.clear(); }));
    CHECK(rejects([](RoadGeomTile& t) { t.underpasses[0].lower_ways = {200, 200}; }));
    CHECK(rejects([](RoadGeomTile& t) { t.underpasses[0].entry_index = t.entries.size(); }));
    CHECK(rejects([](RoadGeomTile& t) { t.underpasses.push_back(t.underpasses[0]); }));   // overlapping
    CHECK(rejects([](RoadGeomTile& t) {   // on a non-ground entry
        for (std::size_t i = 0; i < t.entries.size(); ++i) if (t.entries[i].way_id == 200 && t.entries[i].profile) { t.underpasses[0].entry_index = i; break; }
        t.entries[t.underpasses[0].entry_index].profile->attributes.tunnel = true;
    }));
    // Truncated or trailing section bytes.
    auto cut = *body; cut.sections[7].data.pop_back(); CHECK_FALSE(decode_road_geom(cut));
    auto trail = *body; trail.sections[7].data.push_back(0); CHECK_FALSE(decode_road_geom(trail));
}

TEST_CASE("underpass deck (D58): the carve keeps the cutting under the deck; no slot in the upper road", "[layer][underpass][deck][carve]") {
    const RoadGraph world = underpass_scene();
    const Dem dem = tilted_cutting();
    g2m_test::CarveFixture fx(world, dem);
    const auto tile = build(world, kUnderpassTile, dem);
    REQUIRE(tile); REQUIRE(tile->underpasses.size() == 1);
    const auto& u = tile->underpasses.front();
    double s_c = 0;
    const std::size_t upper = entry_at(*tile, 100, 0.5, &s_c);
    const RoadProfile& p = *tile->entries[upper].profile;
    const auto key0 = tile_key_at(kUnderpassTile.zone, 0, kUnderpassE0 + 0.5, g2m_test::kUnderpassN0);
    REQUIRE(key0);
    const auto run = [&](bool decks, const char* label) {
        CarveOptions options; options.underpass_decks = decks;
        auto m = fx.model(*key0, options);
        const CarveModel& model = *m.model;
        std::size_t upper_merged = 0, lower_merged = 0;
        for (std::size_t i = 0; i < model.kept_tile().entries.size(); ++i) {
            const auto& e = model.kept_tile().entries[i];
            if (e.way_id == 100 && e.stretch.start_ref == tile->entries[upper].stretch.start_ref) upper_merged = i;
            if (e.way_id == 200) lower_merged = i;
        }
        std::size_t upper_core_in_deck = 0, deck_class = 0;
        double max_core_jump = 0, last_core = std::nan("");
        for (double s = u.start_m + 0.5; s <= u.end_m - 0.5; s += 0.5) {
            const auto xy = p.reference.at(s); REQUIRE(xy);
            const auto c = model.evaluate(xy->x, xy->y);
            if (c.owner == CarveOwner::ApproachCore && c.element == upper_merged) ++upper_core_in_deck;
            if (model.classify(xy->x, xy->y, c).role == TerrainRole::BridgeDeck) ++deck_class;
            // Cores along the upper centreline: a jump between neighbouring core cells is a slot.
            if (c.owner == CarveOwner::ApproachCore) {
                if (last_core == last_core) max_core_jump = std::max(max_core_jump, std::fabs(c.z - last_core));
                last_core = c.z;
            }
        }
        const auto cross = model.evaluate(kUnderpassE0 + 0.5, g2m_test::kUnderpassN0);
        std::printf("D58 carve %s: upper core cells in the deck %zu, deck-class cells %zu, max core step %.2f m, at the crossing z %.2f m (owner %d, entry %zu)\n",
                    label, upper_core_in_deck, deck_class, max_core_jump, cross.z, static_cast<int>(cross.owner), cross.element);
        // The lower road is unchanged: it owns the crossing point at its own (cutting) level in both variants.
        CHECK(cross.owner == CarveOwner::ApproachCore);
        CHECK(cross.element == lower_merged);
        CHECK(std::fabs(cross.z - dem(kUnderpassE0 + 0.5, g2m_test::kUnderpassN0)) < 0.3);
        // Outside the deck the upper road is carved at its profile.
        for (const double ds : {-25.0, 25.0}) {
            const auto xy = p.reference.at(s_c + ds); REQUIRE(xy);
            const auto c = model.evaluate(xy->x, xy->y);
            CHECK(c.owner == CarveOwner::ApproachCore); CHECK(c.element == upper_merged);
            CHECK(std::fabs(c.z - p.at(s_c + ds)->height_m) < 0.05);
        }
        return std::tuple{upper_core_in_deck, deck_class, max_core_jump, m.stats.underpass_decks};
    };
    const auto [core_with, deck_with, jump_with, decks_with] = run(true, "with decks");
    CHECK(core_with == 0);           // the upper road is not carved over the cutting ...
    CHECK(deck_with > 0);            // ... it is a deck there
    CHECK(jump_with < 1.0);
    CHECK(decks_with == 1);
    // Sabotage (the pre-D58 carve of a masked profile): the upper road is carved across the cutting at rim height and
    // the lower road cuts a 5-6 m slot into it.
    const auto [core_without, deck_without, jump_without, decks_without] = run(false, "sabotage, no decks");
    CHECK(core_without > 0);
    CHECK(jump_without > 4.0);
    CHECK(decks_without == 0);
    (void)deck_without;
}

TEST_CASE("underpass deck (D59): steep cutting walls under the deck, no abutment slope from the deck ends",
          "[layer][underpass][deck][carve][walls]") {
    // Owner 2026-10-15 ("steep walls"): under a deck the carved terrain is the (smoothed) DEM's cutting, walls included;
    // the upper road's blend fades out within 1 m past the deck's end planes. Measured along the upper centreline and
    // 3 m either side of it, every 0.25 m over the deck span less that metre at each end: the samples the upper road owns, the fill above the smoothed terrain T, and
    // the width of the cutting at mid depth (97 m) in the carve against T's (a sloped wall narrows it).
    const RoadGraph world = underpass_scene();
    const Dem dem = tilted_cutting();
    g2m_test::CarveFixture fx(world, dem);
    const auto tile = build(world, kUnderpassTile, dem);
    REQUIRE(tile); REQUIRE(tile->underpasses.size() == 1);
    const auto& u = tile->underpasses.front();
    const std::size_t upper = entry_at(*tile, 100, 0.5);
    const RoadProfile& p = *tile->entries[upper].profile;
    const auto key0 = tile_key_at(kUnderpassTile.zone, 0, kUnderpassE0 + 0.5, g2m_test::kUnderpassN0);
    REQUIRE(key0);
    struct Walls { std::size_t upper_cells = 0, samples = 0; double max_fill = 0, mean_fill = 0, width = 0, terrain_width = 0; };
    const auto measure = [&](bool steep) {
        CarveOptions options; options.underpass_steep_walls = steep;
        auto m = fx.model(*key0, options);
        const CarveModel& model = *m.model;
        std::size_t upper_merged = 0;
        for (std::size_t i = 0; i < model.kept_tile().entries.size(); ++i) {
            const auto& e = model.kept_tile().entries[i];
            if (e.way_id == 100 && e.stretch.start_ref == tile->entries[upper].stretch.start_ref) upper_merged = i;
        }
        Walls w;
        for (const double t : {-3.0, 0.0, 3.0}) {
            // The first metre past each deck end is the D59 fade (the ground blend reaches the terrain there).
            for (double s = u.start_m + 1.125; s < u.end_m - 1.0; s += 0.25) {
                const auto pose = p.reference.at(s); REQUIRE(pose);
                const double x = pose->x - t * std::sin(pose->heading), y = pose->y + t * std::cos(pose->heading);
                const auto c = model.evaluate(x, y);
                if ((c.owner == CarveOwner::ApproachCore || c.owner == CarveOwner::ApproachBlend) && c.element == upper_merged) ++w.upper_cells;
                ++w.samples;
                w.max_fill = std::max(w.max_fill, c.z - c.terrain);
                w.mean_fill += c.z - c.terrain;
                if (c.z < 97.0) w.width += 0.25 / 3;
                if (c.terrain < 97.0) w.terrain_width += 0.25 / 3;
            }
        }
        w.mean_fill /= static_cast<double>(w.samples);
        std::printf("D59 walls %s: deck %.1f m, upper-owned samples %zu of %zu, fill above T max %.3f mean %.3f m, "
                    "cutting width at 97 m %.2f m (terrain %.2f m)\n", steep ? "steep" : "sabotage (abutment slope)",
                    u.end_m - u.start_m, w.upper_cells, w.samples, w.max_fill, w.mean_fill, w.width, w.terrain_width);
        return w;
    };
    const Walls steep = measure(true);
    CHECK(steep.upper_cells == 0);                                      // nothing of the upper road under its deck
    CHECK(steep.max_fill < 0.05);                                       // nothing raised above the cutting (the lower road's own profile: mm)
    CHECK(std::fabs(steep.width - steep.terrain_width) < 0.26);         // the walls stand where the DEM's do
    CHECK(steep.terrain_width > 10.0);
    // Sabotage (the D58 carve): the ground ends' blend caps reach into the deck, fill the cutting and narrow it.
    const Walls slope = measure(false);
    CHECK(slope.upper_cells > 0);
    CHECK(slope.max_fill > 2.0);
    CHECK(slope.width < slope.terrain_width - 3.0);
}

// The side profile of the PR's example 4 (hidden diag): the A 66 fixture's way 24504840 over the service way 52405506
// with its tunnel tag removed (an untagged underpass), on a flat 100 m DEM with a 5.3 m cutting 16 m wide (4 m smooth
// shoulders) along the lower way. Writes `s_rel,dem_cut,profile_today,carve_today,profile_served,carve_served,deck` to
// the path in G2M_UNDERPASS_CSV (default underpass_example4.csv): before D53 (no mask, no deck) and served.
TEST_CASE("underpass deck: example 4 side profile (diag CSV)", "[.][diag][underpass]") {
    RoadGraph world = g2m_test::fixture_world("hoechst_a66.osm.pbf");
    std::vector<std::pair<double, double>> lower;
    for (auto& w : world.ways) {
        if (w.osm_id != 52405506) continue;
        std::erase_if(w.tags, [](const auto& t) { return t.first == "tunnel"; });
        for (const auto id : w.node_ids)
            for (const auto& n : world.nodes)
                if (n.osm_id == id) lower.emplace_back(n.position.x / 1000.0, n.position.y / 1000.0);
    }
    REQUIRE(lower.size() >= 2);
    const auto dist = [lower](double e, double n) {
        double d = 1e18;
        for (std::size_t i = 1; i < lower.size(); ++i) {
            const auto [ax, ay] = lower[i - 1]; const auto [bx, by] = lower[i];
            const double vx = bx - ax, vy = by - ay, l2 = vx * vx + vy * vy;
            const double t = l2 > 0 ? std::clamp(((e - ax) * vx + (n - ay) * vy) / l2, 0.0, 1.0) : 0.0;
            d = std::min(d, std::hypot(e - (ax + t * vx), n - (ay + t * vy)));
        }
        return d;
    };
    const Dem dem = [dist](double e, double n) {
        const double d = dist(e, n);
        if (d <= 8) return 94.7;
        if (d >= 12) return 100.0;
        const double t = (d - 8) / 4;
        return 94.7 + 5.3 * t * t * (3 - 2 * t);
    };
    // The L2 tile with the crossing: the one holding the lower way's middle node.
    const auto mid = lower[lower.size() / 2];
    const auto key2 = tile_key_at(geo::UtmZone{32, geo::Hemisphere::North}, 2, mid.first, mid.second);
    REQUIRE(key2);
    g2m_test::CarveFixture today(world, dem), served(world, dem);
    today.underpass = false;
    const RoadGeomTile& t0 = today.geom(*key2);
    const RoadGeomTile& t1 = served.geom(*key2);
    const RoadUnderpass* deck = nullptr;
    for (const auto& u : t1.underpasses)
        if (t1.entries[u.entry_index].way_id == 24504840 &&
            std::find(u.lower_ways.begin(), u.lower_ways.end(), 52405506) != u.lower_ways.end()) deck = &u;
    REQUIRE(deck);
    const auto& e1 = t1.entries[deck->entry_index];
    const RoadGeomEntry* e0 = nullptr;
    for (const auto& e : t0.entries)
        if (e.way_id == e1.way_id && e.stretch.start_ref == e1.stretch.start_ref && e.stretch.end_ref == e1.stretch.end_ref) e0 = &e;
    REQUIRE(e0); REQUIRE(e0->profile); REQUIRE(e1.profile);
    const auto& ref = e1.profile->reference;
    // The crossing station: the reference sample nearest the lower way.
    double s_c = 0, best = 1e18;
    for (double s = deck->start_m; s <= deck->end_m; s += 0.05) {
        const auto xy = ref.at(s); REQUIRE(xy);
        const double z = dist(xy->x, xy->y);
        if (z < best) { best = z; s_c = s; }
    }
    std::map<TileKey, HeightTile> carved0, carved1;
    const auto carved = [&](g2m_test::CarveFixture& fx, std::map<TileKey, HeightTile>& cache, double x, double y) {
        const auto k = tile_key_at(key2->zone, 0, x, y); REQUIRE(k);
        auto it = cache.find(*k);
        if (it == cache.end()) { auto c = fx.carve(*k); REQUIRE(c); it = cache.emplace(*k, std::move(*c)).first; }
        // Bilinear between the cell centres (x0 + i + 0.5) of this tile (clamped at its edge: a plot, not a measurement).
        const double x0 = static_cast<double>(k->x) * 256.0, y0 = static_cast<double>(k->y) * 256.0;
        const double gx = std::clamp(x - x0 - 0.5, 0.0, 255.0), gy = std::clamp(y - y0 - 0.5, 0.0, 255.0);
        const int ix = std::min(static_cast<int>(gx), 254), iy = std::min(static_cast<int>(gy), 254);
        const double tx = gx - ix, ty = gy - iy;
        const auto at = [&](int i, int j) { return it->second.h[static_cast<std::size_t>(j * 256 + i)] / 256.0; };
        return (1 - ty) * ((1 - tx) * at(ix, iy) + tx * at(ix + 1, iy)) + ty * ((1 - tx) * at(ix, iy + 1) + tx * at(ix + 1, iy + 1));
    };
    const char* path = std::getenv("G2M_UNDERPASS_CSV");
    std::FILE* f = std::fopen(path ? path : "underpass_example4.csv", "w");
    REQUIRE(f);
    std::fprintf(f, "s_rel,dem_cut,profile_today,carve_today,profile_served,carve_served,deck\n");
    for (double d = -60; d <= 60.001; d += 0.25) {
        const double s = s_c + d;
        if (s < 0 || s > ref.length_m) continue;
        const auto xy = ref.at(s); REQUIRE(xy);
        const auto xy0 = e0->profile->reference.at(s); REQUIRE(xy0);
        std::fprintf(f, "%.2f,%.4f,%.4f,%.4f,%.4f,%.4f,%d\n", d, dem(xy->x, xy->y), e0->profile->at(s)->height_m,
                     carved(today, carved0, xy0->x, xy0->y), e1.profile->at(s)->height_m, carved(served, carved1, xy->x, xy->y),
                     s >= deck->start_m && s <= deck->end_m ? 1 : 0);
    }
    std::fclose(f);
    std::printf("example 4: deck [%.2f, %.2f] m (%.1f m) around the crossing at %.2f m, depth %d mm\n", deck->start_m, deck->end_m,
                deck->end_m - deck->start_m, s_c, static_cast<int>(deck->depth_mm));
}

// ---- D61: tag rules (owner answers on geo2map PR #13) ----

TEST_CASE("underpass deck (D61): a tunnel under the road is covered - masked, no deck", "[layer][underpass][deck][d61]") {
    const Dem dem = tilted_cutting();
    UnderpassSceneOptions tunnel; tunnel.lower_tags = {{"layer", "-1"}, {"tunnel", "yes"}};
    const RoadGraph world = underpass_scene(tunnel);
    RoadUnderpassStats stats;
    const auto tile = build(world, kUnderpassTile, dem, true, &stats);
    REQUIRE(tile);
    CHECK(tile->underpasses.empty());
    CHECK(stats.crossings == 1); CHECK(stats.served == 1); CHECK(stats.covered == 1); CHECK(stats.decks == 0);
    // The road stays on the ground over the tunnel: the profile is still masked at the rim chord, not in the cutting.
    double s_c = 0;
    const auto& p = *tile->entries[entry_at(*tile, 100, 0.5, &s_c)].profile;
    const auto z = p.at(s_c); REQUIRE(z);
    CHECK(z->height_m > 100.0 - 0.1);
    // Negative control: the same crossing without the tunnel tag is a deck.
    RoadUnderpassStats open_stats;
    const auto open = build(underpass_scene(), kUnderpassTile, dem, true, &open_stats);
    REQUIRE(open); CHECK(open->underpasses.size() == 1); CHECK(open_stats.covered == 0);
}

TEST_CASE("underpass deck (D61): parking-structure storeys are neither upper nor lower roads", "[layer][underpass][deck][d61]") {
    const Dem dem = tilted_cutting();
    // A lower storey way is counted twice: as a lower crossing of way 100 and as an upper stretch of its own.
    struct Case { const char* name; UnderpassSceneOptions scene; std::size_t in_structure; };
    UnderpassSceneOptions lower_aisle; lower_aisle.lower_tags = {{"layer", "-1"}, {"service", "parking_aisle"}};
    UnderpassSceneOptions lower_level; lower_level.lower_tags = {{"layer", "-1"}, {"level", "-1"}};
    UnderpassSceneOptions upper_aisle; upper_aisle.upper_tags = {{"service", "parking_aisle"}};
    UnderpassSceneOptions upper_level; upper_level.upper_tags = {{"level", "1"}};
    for (const Case& c : {Case{"lower parking aisle", lower_aisle, 2}, Case{"lower level", lower_level, 2},
                          Case{"upper parking aisle", upper_aisle, 1}, Case{"upper level", upper_level, 1}}) {
        CAPTURE(c.name);
        RoadUnderpassStats stats;
        const auto tile = build(underpass_scene(c.scene), kUnderpassTile, dem, true, &stats);
        REQUIRE(tile);
        CHECK(tile->underpasses.empty());
        CHECK(stats.crossings == 0); CHECK(stats.in_structure == c.in_structure);
        // No mask either: the profile follows the DEM as before D53.
        double s_c = 0;
        const auto z = tile->entries[entry_at(*tile, 100, 0.5, &s_c)].profile->at(s_c); REQUIRE(z);
        CHECK(z->height_m < 100.0 - 0.3);
    }
}

namespace {
// The owner's crossings in the fixtures (PR #13): the intersection of the two ways' node polylines, metres.
bool crossing_point(const RoadGraph& world, std::int64_t upper, std::int64_t lower, double& x, double& y) {
    const auto node = [&](std::int64_t id) -> const RoadGraphNode* {
        const auto it = std::lower_bound(world.nodes.begin(), world.nodes.end(), id, [](const auto& n, auto v) { return n.osm_id < v; });
        return it != world.nodes.end() && it->osm_id == id ? &*it : nullptr;
    };
    const auto way = [&](std::int64_t id) -> const RoadGraphWay* {
        for (const auto& w : world.ways) if (w.osm_id == id) return &w;
        return nullptr;
    };
    const auto *u = way(upper), *l = way(lower);
    if (!u || !l) return false;
    for (std::size_t i = 0; i + 1 < u->node_ids.size(); ++i)
        for (std::size_t k = 0; k + 1 < l->node_ids.size(); ++k) {
            const auto *a = node(u->node_ids[i]), *b = node(u->node_ids[i + 1]), *c = node(l->node_ids[k]), *d = node(l->node_ids[k + 1]);
            if (!a || !b || !c || !d || !geom::segments_intersect(a->position, b->position, c->position, d->position)) continue;
            const double ax = a->position.x / 1000.0, ay = a->position.y / 1000.0, bx = b->position.x / 1000.0, by = b->position.y / 1000.0;
            const double cx = c->position.x / 1000.0, cy = c->position.y / 1000.0, dx = d->position.x / 1000.0, dy = d->position.y / 1000.0;
            const double den = (bx - ax) * (dy - cy) - (by - ay) * (dx - cx);
            const double t = den != 0 ? ((cx - ax) * (dy - cy) - (cy - ay) * (dx - cx)) / den : 0.0;
            x = ax + t * (bx - ax); y = ay + t * (by - ay);
            return true;
        }
    return false;
}
// A 6 m cutting 16 m wide (4 m smooth sides) centred on the crossing point, running along the lower way's direction
// (approximated as a disc of the same radius: the fixture lower ways are short).
Dem cutting_at(double cx, double cy) {
    return [=](double e, double n) {
        const double d = std::hypot(e - cx, n - cy);
        if (d <= 8) return 94.0;
        if (d >= 12) return 100.0;
        const double t = (d - 8) / 4;
        return 94.0 + 6.0 * t * t * (3 - 2 * t);
    };
}
} // namespace

TEST_CASE("underpass deck (D61): the owner's crossings in the fixtures", "[layer][underpass][deck][d61]") {
    // Kalbach: two parking aisles of a multi-storey car park under a parking aisle (no deck, not a crossing). A 66: the
    // motorway over the continuous service tunnel 52405506 (covered: masked, no deck). Main-Taunus-Zentrum: the level-1
    // aisle of the two-level car park over the tunnel-tagged ground-level aisle (no deck, not a crossing). Each crossing
    // gets a synthetic 6 m DEM cutting so the D53-D58 rules alone would act there; the negative control strips the
    // D61 tags (service, level, tunnel) from both ways and shows the crossing is then examined.
    struct Case { const char* file; std::int64_t upper, lower; bool covered; };
    const Case cases[] = {{"frankfurt_hoechst.osm.pbf", 402193257, 402193252, false}, {"frankfurt_hoechst.osm.pbf", 402193257, 1160921224, false},
                          {"hoechst_a66.osm.pbf", 24504840, 52405506, true}, {"hoechst_a66.osm.pbf", 313879329, 52405506, true},
                          {"hoechst_a66.osm.pbf", 199241226, 1529362865, false}};
    for (const auto& c : cases) {
        CAPTURE(c.file, c.upper, c.lower);
        RoadGraph world = g2m_test::fixture_world(c.file);
        double x = 0, y = 0;
        REQUIRE(crossing_point(world, c.upper, c.lower, x, y));
        const Dem dem = cutting_at(x, y);
        // The tile holding the upper stretch over the crossing (an L2 neighbour of the crossing's tile near a border).
        const auto holds_upper = [&](const RoadGeomTile& t) {
            for (const auto& e : t.entries) {
                if (e.way_id != c.upper || !e.profile) continue;
                for (double s = 0; s <= e.profile->reference.length_m; s += 1)
                    if (const auto xy = e.profile->reference.at(s); xy && std::hypot(xy->x - x, xy->y - y) < 5) return true;
            }
            return false;
        };
        std::optional<TileKey> found;
        for (int dy = -1; dy <= 1 && !found; ++dy) for (int dx = -1; dx <= 1 && !found; ++dx) {
            const TileKey k{geo::UtmZone{32, geo::Hemisphere::North}, 2, static_cast<std::int32_t>(std::floor(x / 1024.0)) + dx,
                            static_cast<std::int32_t>(std::floor(y / 1024.0)) + dy};
            const auto t = build(world, k, dem, false);
            if (t && holds_upper(*t)) found = k;
        }
        REQUIRE(found);
        const TileKey key = *found;
        const auto decked = [&](const RoadGeomTile& t) {
            for (const auto& u : t.underpasses)
                if (t.entries[u.entry_index].way_id == c.upper && std::binary_search(u.lower_ways.begin(), u.lower_ways.end(), c.lower)) return true;
            return false;
        };
        // The finder on every stretch of the upper way (whether or not the profile fit succeeds on the synthetic DEM).
        const auto finder_crossings = [&](const RoadGraph& w) {
            const auto in = g2m_test::road_geom_inputs(w, key); REQUIRE(in);
            std::vector<HeightTile> heights(in->dem_keys.size());
            std::vector<const HeightTile*> views;
            for (std::size_t i = 0; i < heights.size(); ++i) { heights[i].key = in->dem_keys[i]; g2m_test::fill_dem(heights[i], dem); views.push_back(&heights[i]); }
            const auto sampler = RoadDemSampler::make(views); REQUIRE(sampler);
            const auto builder = RoadReferenceBuilder::make(in->context); REQUIRE(builder);
            const auto stretches = builder->stretches(c.upper); REQUIRE(stretches);
            const RoadUnderpassFinder finder(in->crossings, *sampler, key.zone, builtin_road_rules_de().underpass);
            RoadUnderpassStats st;
            ReferenceFitLimits fit; fit.allow_curvature_exceptions = true;
            for (const auto& stretch : *stretches)
                if (const auto ref = builder->fit_stretch(c.upper, stretch, fit)) (void)finder.spans(c.upper, stretch, *ref, &st);
            return st.crossings;
        };
        RoadUnderpassStats stats;
        const auto tile = build(world, key, dem, true, &stats);
        REQUIRE(tile);
        CHECK_FALSE(decked(*tile));
        const std::size_t tagged_crossings = finder_crossings(world);
        if (c.covered) CHECK(stats.covered >= 1); else CHECK(stats.in_structure >= 1);
        // Negative control: without the D61 tags the crossing is examined (and served, or declined only by geometry).
        for (auto& w : world.ways)
            if (w.osm_id == c.upper || w.osm_id == c.lower)
                std::erase_if(w.tags, [](const auto& t) { return t.first == "service" || t.first == "level" || t.first == "tunnel"; });
        RoadUnderpassStats control;
        const auto plain = build(world, key, dem, true, &control);
        REQUIRE(plain);
        const std::size_t plain_crossings = finder_crossings(world);
        std::printf("D61 %s %lld over %lld: upper-way crossings %zu -> %zu without the tags, deck %d -> %d, covered %zu, in_structure %zu\n", c.file,
                    static_cast<long long>(c.upper), static_cast<long long>(c.lower), tagged_crossings, plain_crossings,
                    static_cast<int>(decked(*tile)), static_cast<int>(decked(*plain)), stats.covered, stats.in_structure);
        if (c.covered) CHECK(decked(*plain));
        else CHECK(plain_crossings > tagged_crossings);
    }
}

// Where the fixtures' grade-separated crossings are on a map (hidden diag): every pair of ways whose node polylines
// cross without a shared node, one of them on a lower OSM layer or tagged tunnel (never a bridge), with the
// intersection in WGS84 (GRS80 UTM 32N reverse; the two ellipsoids differ by far less than the printed 1e-6 deg).
TEST_CASE("underpass crossings: WGS84 coordinates in both fixtures (diag)", "[.][diag][underpass]") {
    const geo::Utm utm;
    for (const char* file : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const RoadGraph world = g2m_test::fixture_world(file);
        const auto node = [&](std::int64_t id) -> const RoadGraphNode* {
            const auto it = std::lower_bound(world.nodes.begin(), world.nodes.end(), id, [](const auto& n, auto v) { return n.osm_id < v; });
            return it != world.nodes.end() && it->osm_id == id ? &*it : nullptr;
        };
        const auto tag = [](const RoadGraphWay& w, std::string_view k) -> std::string {
            for (const auto& [key, value] : w.tags) if (key == k) return value;
            return {};
        };
        const auto layer = [&](const RoadGraphWay& w) { const auto v = tag(w, "layer"); return v.empty() ? 0 : std::atoi(v.c_str()); };
        for (const auto& up : world.ways) {
            if (tag(up, "bridge") == "yes" || tag(up, "tunnel") == "yes") continue;
            for (const auto& lo : world.ways) {
                if (lo.osm_id == up.osm_id || tag(lo, "bridge") == "yes") continue;
                if (!(tag(lo, "tunnel") == "yes" || layer(lo) < layer(up))) continue;
                for (std::size_t i = 0; i + 1 < up.node_ids.size(); ++i)
                    for (std::size_t k = 0; k + 1 < lo.node_ids.size(); ++k) {
                        const auto *a = node(up.node_ids[i]), *b = node(up.node_ids[i + 1]), *c = node(lo.node_ids[k]), *d = node(lo.node_ids[k + 1]);
                        if (!a || !b || !c || !d) continue;
                        if (a->osm_id == c->osm_id || a->osm_id == d->osm_id || b->osm_id == c->osm_id || b->osm_id == d->osm_id) continue;
                        if (!geom::segments_intersect(a->position, b->position, c->position, d->position)) continue;
                        const double ax = a->position.x / 1000.0, ay = a->position.y / 1000.0, bx = b->position.x / 1000.0, by = b->position.y / 1000.0;
                        const double cx = c->position.x / 1000.0, cy = c->position.y / 1000.0, dx = d->position.x / 1000.0, dy = d->position.y / 1000.0;
                        const double den = (bx - ax) * (dy - cy) - (by - ay) * (dx - cx);
                        const double t = den != 0 ? ((cx - ax) * (dy - cy) - (cy - ay) * (dx - cx)) / den : 0.0;
                        const double x = ax + t * (bx - ax), y = ay + t * (by - ay);
                        const auto g = utm.reverse(geo::UtmZone{32, geo::Hemisphere::North}, x, y);
                        std::printf("crossing %s: upper way %lld (%s, layer %d) over way %lld (%s, layer %d%s) at %.6f N, %.6f E\n", file,
                                    static_cast<long long>(up.osm_id), tag(up, "highway").c_str(), layer(up), static_cast<long long>(lo.osm_id),
                                    tag(lo, "highway").c_str(), layer(lo), tag(lo, "tunnel") == "yes" ? ", tunnel" : "", g.lat, g.lon);
                    }
            }
        }
    }
}

// The owner's crossings (PR #13): every tag of the upper and lower ways, and every other OSM way (area or not) with a
// parking/building/covered/tunnel tag whose node bbox holds a crossing point (hidden diag).
TEST_CASE("underpass crossings: OSM tags of the owner's crossings (diag)", "[.][diag][underpass][tags]") {
    struct Case { const char* file; std::int64_t upper, lower; };
    const Case cases[] = {{"frankfurt_hoechst.osm.pbf", 402193257, 402193252}, {"frankfurt_hoechst.osm.pbf", 402193257, 1160921224},
                          {"hoechst_a66.osm.pbf", 24504840, 52405506}, {"hoechst_a66.osm.pbf", 313879329, 52405506},
                          {"hoechst_a66.osm.pbf", 199241226, 1529362865}, {"frankfurt_hoechst.osm.pbf", 199241226, 1529362865}};
    for (const auto& c : cases) {
        const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + c.file);
        std::printf("== %s upper %lld lower %lld\n", c.file, static_cast<long long>(c.upper), static_cast<long long>(c.lower));
        std::int32_t lat0 = 0, lon0 = 0;
        for (const std::int64_t id : {c.upper, c.lower}) {
            const auto* w = data.find_way(id);
            if (!w) { std::printf("  way %lld: absent\n", static_cast<long long>(id)); continue; }
            std::printf("  way %lld (%u nodes):", static_cast<long long>(id), w->refs_count);
            for (const auto& t : data.tags_of(*w)) std::printf(" %.*s=%.*s", static_cast<int>(data.str(t.key).size()), data.str(t.key).data(),
                                                              static_cast<int>(data.str(t.value).size()), data.str(t.value).data());
            std::printf("\n");
            if (id == c.lower) {
                const auto refs = data.refs_of(*w);
                if (const auto* n = data.find_node(refs[refs.size() / 2])) { lat0 = n->lat; lon0 = n->lon; }
            }
        }
        if (lat0 == 0) continue;
        for (const auto& w : data.ways) {
            bool interesting = false;
            for (const auto& t : data.tags_of(w)) {
                const auto k = data.str(t.key), v = data.str(t.value);
                if (k == "amenity" && v == "parking") interesting = true;
                if (k == "building" && (v == "parking" || v == "garage" || v == "garages")) interesting = true;
                if (k == "parking" || k == "covered" || k == "building:levels" || (k == "tunnel" && w.id != c.lower)) interesting = true;
            }
            if (!interesting) continue;
            std::int32_t a = INT32_MAX, b = INT32_MIN, e = INT32_MAX, f = INT32_MIN;
            for (const auto r : data.refs_of(w)) if (const auto* n = data.find_node(r)) { a = std::min(a, n->lat); b = std::max(b, n->lat); e = std::min(e, n->lon); f = std::max(f, n->lon); }
            if (lat0 < a - 3000 || lat0 > b + 3000 || lon0 < e - 5000 || lon0 > f + 5000) continue;   // ~30 m margin
            std::printf("    near way %lld:", static_cast<long long>(w.id));
            for (const auto& t : data.tags_of(w)) std::printf(" %.*s=%.*s", static_cast<int>(data.str(t.key).size()), data.str(t.key).data(),
                                                             static_cast<int>(data.str(t.value).size()), data.str(t.value).data());
            std::printf("\n");
        }
    }
}

// D61 on the golden DEM (hidden diag): every served deck and every covered span of both fixtures over
// g2m_test::carve_tilted_dem (the g3c/g3d golden DEM), and the D61 skip counters per tile.
TEST_CASE("underpass deck (D61): decks and covered spans of both fixtures on the golden DEM (diag)", "[.][diag][underpass][d61]") {
    for (const char* file : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) {
        const RoadGraph world = g2m_test::fixture_world(file);
        std::set<TileKey> keys;
        for (const auto& n : world.nodes)
            keys.insert(TileKey{geo::UtmZone{32, geo::Hemisphere::North}, 2, static_cast<std::int32_t>(floor_div(n.position.x, 1024000)),
                                static_cast<std::int32_t>(floor_div(n.position.y, 1024000))});
        for (const auto& key : keys) {
            RoadUnderpassStats stats;
            const auto tile = build(world, key, g2m_test::carve_tilted_dem, true, &stats);
            REQUIRE(tile);
            for (const auto& u : tile->underpasses)
                std::printf("  %s tile %d/%d deck: way %lld [%.2f, %.2f] m, depth %d mm, lower %lld\n", file, key.x, key.y,
                            static_cast<long long>(tile->entries[u.entry_index].way_id), u.start_m, u.end_m, static_cast<int>(u.depth_mm),
                            static_cast<long long>(u.lower_ways.front()));
            if (stats.crossings || stats.in_structure)
                std::printf("D61 %s tile %d/%d: crossings %zu, served %zu, decks %zu, covered %zu, in_structure %zu\n", file, key.x, key.y,
                            stats.crossings, stats.served, stats.decks, stats.covered, stats.in_structure);
            // Before D61 a covered span was a deck: the same tile with the lower ways' tunnel tags stripped.
            if (!stats.covered) continue;
            RoadGraph open = world;
            for (auto& w : open.ways) std::erase_if(w.tags, [](const auto& t) { return t.first == "tunnel"; });
            const auto before = build(open, key, g2m_test::carve_tilted_dem, true);
            REQUIRE(before);
            for (const auto& u : before->underpasses)
                std::printf("  %s tile %d/%d deck without tunnel tags: way %lld [%.2f, %.2f] m, depth %d mm, lower %lld\n", file, key.x, key.y,
                            static_cast<long long>(before->entries[u.entry_index].way_id), u.start_m, u.end_m, static_cast<int>(u.depth_mm),
                            static_cast<long long>(u.lower_ways.front()));
        }
    }
}
