// Street fidelity P2 slice 2: the one carved road height z(s, t) = p(s) + C(s, t) - C(s, 0) + V(s, t) + F(s, t)
// (design section 4.1, test T1) and its numbers from data/carve/carve_default.json.
#include "alloc_counter.h"
#include "road_fixture.h"
#include "g2m/layer/carve_params.h"
#include "g2m/layer/road_surface.h"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
using namespace g2m;
using Catch::Approx;
namespace {
const geo::UtmZone zone{32, geo::Hemisphere::North};

CrossSectionBand band(LaneKind kind, LaneDirection dir, bool counted, int width_mm) {
    CrossSectionBand b; b.kind = kind; b.direction = dir; b.counted = counted; b.width_mm = width_mm;
    b.paved = kind != LaneKind::Verge; return b;
}
// Sidewalk 2.0 | Shoulder 0.5 | Backward 3.0 | Forward 3.25 | Shoulder 0.5 | Verge 1.0, reference on the lane boundary:
// roof ridge at t = 0, carriageway [-3.25, 3.0], paved [-3.75, 5.5], outer [-4.75, 5.5].
RoadCrossSection two_way() {
    RoadCrossSection xs;
    xs.bands = {band(LaneKind::Sidewalk, LaneDirection::None, false, 2000), band(LaneKind::Shoulder, LaneDirection::None, false, 500),
        band(LaneKind::Driving, LaneDirection::Backward, true, 3000), band(LaneKind::Driving, LaneDirection::Forward, true, 3250),
        band(LaneKind::Shoulder, LaneDirection::None, false, 500), band(LaneKind::Verge, LaneDirection::None, false, 1000)};
    xs.anchor = {2, AnchorEdge::Right, PlacementSource::Explicit};
    return xs;
}
// Shoulder 0.5 | Driving 3.5 | Driving 3.5 | Shoulder 2.5 | Verge 1.5; the reference is the middle of the carriageway.
RoadCrossSection one_way(bool reversed) {
    RoadCrossSection xs;
    const auto dir = reversed ? LaneDirection::Backward : LaneDirection::Forward;
    xs.bands = {band(LaneKind::Shoulder, LaneDirection::None, false, 500), band(LaneKind::Driving, dir, true, 3500),
        band(LaneKind::Driving, dir, true, 3500), band(LaneKind::Shoulder, LaneDirection::None, false, 2500), band(LaneKind::Verge, LaneDirection::None, false, 1500)};
    xs.anchor = {2, AnchorEdge::Left, PlacementSource::Absent};
    xs.flags = static_cast<std::uint16_t>(kXsOneway | (reversed ? kXsReversedOneway : 0));
    return xs;
}
constexpr double kBaseHeight = 500.0, kGrade = 0.05;

// A straight 100 m stretch (way 10) built by the real deriver, then given a hand-made section and a sloped profile.
RoadGeomTile straight_tile(const RoadCrossSection& xs) {
    RoadGraph graph;
    graph.nodes = {{1, {10000, 10000}}, {2, {110000, 10000}}};
    graph.ways = {{10, 4, {1, 2}, {{"highway", "residential"}, {"maxspeed", "30"}}}};
    graph.edges = {{10, 0, 1, 2}};
    const auto source = *select_road_graph_tile(graph, {zone, 2, 0, 0});
    HeightTile heights; heights.key = {zone, 0, 0, 0}; heights.h.fill(25600);
    const HeightTile* views[] = {&heights}; const auto dem = RoadDemSampler::make(views); REQUIRE(dem);
    auto tile = build_road_geom_tile(source, *dem); REQUIRE(tile);
    REQUIRE(tile->entries.size() == 1); REQUIRE(tile->entries[0].profile); REQUIRE(tile->entries[0].cross_section);
    auto& profile = *tile->entries[0].profile;
    const double length = profile.reference.length_m;
    VerticalCubic cubic; cubic.station_m = 0; cubic.length_m = length;
    cubic.height0_m = kBaseHeight; cubic.height1_m = kBaseHeight + kGrade * length; cubic.grade0 = cubic.grade1 = kGrade;
    profile.vertical.segments = {cubic};
    auto section = xs; section.length_m = length;
    tile->entries[0].cross_section = section;
    tile->features.clear();
    return std::move(*tile);
}
double profile_height(double s) { return kBaseHeight + kGrade * s; }
constexpr double kC = 0.025;   // crown_per_mille 25 of a residential road
constexpr double kStations[] = {0.0, 7.5, 33.3, 50.0, 99.9};
}

TEST_CASE("T1: the surface equals the profile on the reference line", "[layer][road_surface]") {
    for (int variant = 0; variant < 3; ++variant) {
        const auto tile = straight_tile(variant == 0 ? two_way() : one_way(variant == 2));
        INFO("variant " << variant);
        for (const double s : kStations) CHECK(road_surface_height_m(tile, 0, s, 0.0) == Approx(profile_height(s)).margin(1e-12));
        // Without a crown the surface is the profile at every t of the carriageway: a sanity anchor for the grade term.
        CHECK(road_surface_height_m(tile, 0, 40.0, 0.0) - road_surface_height_m(tile, 0, 20.0, 0.0) == Approx(kGrade * 20.0).margin(1e-12));
    }
}

TEST_CASE("T1: the roof crown is C1 at the ridge and falls by the crossfall outside r0", "[layer][road_surface]") {
    const auto tile = straight_tile(two_way());
    const double r0 = RoadSurfaceParams{}.crown_round_m;
    const auto rel = [&](double t) { return road_surface_height_m(tile, 0, 50.0, t) - profile_height(50.0); };
    // At the ridge the surface IS the profile; at |t| = r0 the rounded ridge has dropped c*r0/2.
    CHECK(rel(0.0) == Approx(0.0).margin(1e-15));
    CHECK(rel(r0) == Approx(-kC * r0 / 2.0).margin(1e-12));
    CHECK(rel(-r0) == Approx(-kC * r0 / 2.0).margin(1e-12));
    // Outside r0: exactly c per metre on both sides.
    CHECK(rel(1.5) - rel(0.75) == Approx(-kC * 0.75).margin(1e-12));
    CHECK(rel(-2.5) - rel(-1.0) == Approx(-kC * 1.5).margin(1e-12));
    // C1: the one-sided slopes on either side of +-r0 agree to the finite-difference error.
    constexpr double h = 1e-6;
    for (const double edge : {r0, -r0}) {
        const double left = (rel(edge) - rel(edge - h)) / h, right = (rel(edge + h) - rel(edge)) / h;
        CHECK(left == Approx(right).margin(1e-4));
        CHECK(std::abs(std::abs(left) - kC) < 1e-4);
    }
    CHECK(std::abs((rel(h) - rel(0.0)) / h) < 1e-4);   // flat ridge
    // The pure crown function against the formula.
    CHECK(road_crown_m(CrownKind::Roof, kC, 0.3, 0.5) == Approx(-kC * (0.09 / 1.0 + 0.25)));
    CHECK(road_crown_m(CrownKind::Roof, kC, -2.0, 0.5) == Approx(-kC * 2.0));
}

TEST_CASE("T1: the verge falls exactly verge_drop over the verge band", "[layer][road_surface]") {
    const auto tile = straight_tile(two_way());
    const RoadSurfaceParams base;
    RoadSurfaceParams flat = base; flat.verge_drop_m = 0;
    const auto f = carve_footprint(*tile.entries[0].cross_section, 50.0);
    REQUIRE(f.paved_right == Approx(-3.75)); REQUIRE(f.outer_right == Approx(-4.75));
    const auto drop = [&](double t) { return road_surface_height_m(tile, 0, 50.0, t, base) - road_surface_height_m(tile, 0, 50.0, t, flat); };
    CHECK(drop(f.paved_right) == 0.0);                       // flush at the paved edge
    CHECK(drop(f.paved_right + 0.5) == 0.0);                 // inside the paved band
    CHECK(drop(f.paved_right - 0.5) == Approx(-0.03).margin(1e-12));  // half way: smooth step 1/2
    CHECK(drop(f.outer_right) == Approx(-0.06).margin(1e-12));        // exactly verge_drop at the outer edge
    CHECK(drop(f.outer_right - 3.0) == Approx(-0.06).margin(1e-12));  // t clamped to the outer edge
    // The left side has no verge here (the sidewalk is paved and outermost): outer == paved, no drop.
    CHECK(drop(f.outer_left) == 0.0);
    // Monotone and C1 across the band.
    double previous = 0;
    for (int i = 0; i <= 100; ++i) {
        const double d = drop(f.paved_right - 0.01 * i);
        CHECK(d <= previous + 1e-15); previous = d;
    }
    // A different parameter moves the result exactly (the numbers are data).
    RoadSurfaceParams deep = base; deep.verge_drop_m = 0.2;
    CHECK(road_surface_height_m(tile, 0, 50.0, f.outer_right, deep) - road_surface_height_m(tile, 0, 50.0, f.outer_right, flat) == Approx(-0.2).margin(1e-12));
}

TEST_CASE("T1: a one-way carriageway falls to the right of travel with the pivot edge c*w/2 above the profile", "[layer][road_surface]") {
    const double w = 7.0;
    for (const bool reversed : {false, true}) {
        const auto tile = straight_tile(one_way(reversed));
        const auto f = carve_footprint(*tile.entries[0].cross_section, 40.0);
        INFO("reversed " << reversed);
        REQUIRE(f.carriageway_left == Approx(3.5)); REQUIRE(f.carriageway_right == Approx(-3.5));
        const auto rel = [&](double t) { return road_surface_height_m(tile, 0, 40.0, t) - profile_height(40.0); };
        const double high_edge = reversed ? f.carriageway_right : f.carriageway_left;   // the crown pivot
        const double low_edge = reversed ? f.carriageway_left : f.carriageway_right;
        CHECK(rel(high_edge) == Approx(kC * w / 2.0).margin(1e-12));
        CHECK(rel(low_edge) == Approx(-kC * w / 2.0).margin(1e-12));
        CHECK(rel(0.0) == Approx(0.0).margin(1e-12));
        // Falls to the right of the travel direction: for oneway=yes (reference direction = travel) that is -t.
        CHECK((rel(1.0) < rel(-1.0)) == reversed);
    }
}

TEST_CASE("T1: the carved features act over their lateral extent only", "[layer][road_surface]") {
    auto tile = straight_tile(two_way());
    ProfileFeature bump; bump.entry_index = 0; bump.node_id = 9; bump.station_m = 50.0; bump.kind = ProfileFeatureKind::Bump;
    bump.carve = true; bump.length_mm = 6000; bump.height_mm = 100; bump.width_mm = 0;
    ProfileFeature cushion = bump; cushion.node_id = 10; cushion.station_m = 80.0; cushion.kind = ProfileFeatureKind::Cushion; cushion.width_mm = 1600;
    ProfileFeature render_only = bump; render_only.node_id = 11; render_only.station_m = 20.0; render_only.kind = ProfileFeatureKind::Chicane; render_only.carve = false;
    tile.features = {render_only, bump, cushion};
    const auto f = carve_footprint(*tile.entries[0].cross_section, 50.0);
    const auto without = [&](double s, double t) { auto bare = tile; bare.features.clear(); return road_surface_height_m(bare, 0, s, t); };
    // Full carriageway: the plateau height is added on the carriageway, nothing outside it.
    CHECK(road_surface_height_m(tile, 0, 50.0, 1.0) - without(50.0, 1.0) == Approx(feature_height_contribution_m(bump, 50.0)).margin(1e-12));
    CHECK(feature_height_contribution_m(bump, 50.0) > 0.09);
    CHECK(road_surface_height_m(tile, 0, 50.0, f.carriageway_left + 0.2) == Approx(without(50.0, f.carriageway_left + 0.2)).margin(1e-15));
    CHECK(road_surface_height_m(tile, 0, 50.0, f.carriageway_right - 0.2) == Approx(without(50.0, f.carriageway_right - 0.2)).margin(1e-15));
    // A cushion is carved across the whole carriageway whatever its stored width (it cannot be narrower on the 1 m grid).
    CHECK(road_surface_height_m(tile, 0, 80.0, f.carriageway_left - 0.1) - without(80.0, f.carriageway_left - 0.1) == Approx(feature_height_contribution_m(cushion, 80.0)).margin(1e-12));
    // A partial width is centred on the carriageway.
    tile.features = {bump}; tile.features[0].width_mm = 2000;
    const double mid = (f.carriageway_left + f.carriageway_right) / 2.0;
    CHECK(road_surface_height_m(tile, 0, 50.0, mid + 0.9) - without(50.0, mid + 0.9) > 0.09);
    CHECK(road_surface_height_m(tile, 0, 50.0, mid + 1.1) == Approx(without(50.0, mid + 1.1)).margin(1e-15));
    // Render-only kinds never change the surface.
    tile.features = {render_only};
    CHECK(road_surface_height_m(tile, 0, 20.0, 0.5) == Approx(without(20.0, 0.5)).margin(1e-15));
    // A dip is negative.
    ProfileFeature dip = bump; dip.kind = ProfileFeatureKind::Dip; tile.features = {dip};
    CHECK(road_surface_height_m(tile, 0, 50.0, 1.0) < without(50.0, 1.0) - 0.05);
}

TEST_CASE("T1: clamping, undefined entries and the carve file's numbers", "[layer][road_surface]") {
    auto tile = straight_tile(two_way());
    const auto f = carve_footprint(*tile.entries[0].cross_section, 50.0);
    // t beyond the outer edges and s beyond the stretch clamp instead of extrapolating.
    CHECK(road_surface_height_m(tile, 0, 50.0, 100.0) == road_surface_height_m(tile, 0, 50.0, f.outer_left));
    CHECK(road_surface_height_m(tile, 0, 50.0, -100.0) == road_surface_height_m(tile, 0, 50.0, f.outer_right));
    const double length = tile.entries[0].profile->reference.length_m;
    CHECK(road_surface_height_m(tile, 0, -5.0, 0.0) == road_surface_height_m(tile, 0, 0.0, 0.0));
    CHECK(road_surface_height_m(tile, 0, length + 5.0, 0.0) == road_surface_height_m(tile, 0, length, 0.0));
    // Undefined: index out of range, no profile, no cross-section.
    CHECK(std::isnan(road_surface_height_m(tile, 1, 0.0, 0.0)));
    auto no_profile = tile; no_profile.entries[0].profile.reset();
    CHECK(std::isnan(road_surface_height_m(no_profile, 0, 0.0, 0.0)));
    auto no_xs = tile; no_xs.entries[0].cross_section.reset();
    CHECK(std::isnan(road_surface_height_m(no_xs, 0, 0.0, 0.0)));
    // The defaults of RoadSurfaceParams are the numbers of data/carve/carve_default.json.
    const auto params = builtin_carve_default().surface_params();
    CHECK(params == RoadSurfaceParams{});
    CHECK(params.crown_round_m == 0.5); CHECK(params.verge_drop_m == 0.06);
}

TEST_CASE("T1: road_surface_height_m allocates nothing", "[layer][road_surface][alloc]") {
    auto tile = straight_tile(two_way());
    ProfileFeature bump; bump.entry_index = 0; bump.node_id = 9; bump.station_m = 50.0; bump.kind = ProfileFeatureKind::Bump;
    bump.carve = true; bump.length_mm = 6000; bump.height_mm = 100; tile.features = {bump};
    if (!g2m_test::kAllocCounterAvailable) SKIP("allocation counting unavailable under a sanitizer build (G2M_SANITIZER_BUILD)");
    double sink = 0;
    g2m_test::ScopedAllocCounter counter;
    for (int i = 0; i < 2000; ++i) sink += road_surface_height_m(tile, 0, 0.05 * i, -4.0 + 0.004 * i);
    CHECK(counter.count() == 0);
    CHECK(std::isfinite(sink));
}

TEST_CASE("T1 fixtures: z on the reference equals the profile along real stretches", "[layer][road_surface][roads]") {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/hoechst_a66.osm.pbf");
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    const TileKey key{g2m_test::kZone32N, 2, 455, 5421};
    auto geometry = g2m_test::geometry_in_tile(*world, key); REQUIRE(geometry);
    std::size_t checked = 0, samples = 0;
    for (std::size_t ei = 0; ei < geometry->entries.size(); ++ei) {
        const auto& entry = geometry->entries[ei];
        if (!entry.profile || !entry.cross_section) continue;
        // Entries with a carved feature are skipped: F is non-zero on the reference there.
        if (std::any_of(geometry->features.begin(), geometry->features.end(), [&](const auto& x) { return x.entry_index == ei; })) continue;
        // A merge ramp lands off its reference line (G2M-028): t = 0 is then off the carriageway centre and the crown shows.
        if (entry.cross_section->start_join.kind == JoinKind::Landing || entry.cross_section->end_join.kind == JoinKind::Landing) continue;
        ++checked;
        const double length = entry.profile->reference.length_m;
        for (double s = 0; s <= length; s += 2.5) {
            const auto pose = entry.profile->at(s); REQUIRE(pose);
            CHECK(road_surface_height_m(*geometry, ei, s, 0.0) == Approx(pose->height_m).margin(1e-9));
            ++samples;
        }
    }
    CHECK(checked > 20); CHECK(samples > 200);
    INFO("entries checked " << checked << ", samples " << samples);
}
