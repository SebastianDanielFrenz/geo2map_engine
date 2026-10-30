// tests/unit/test_structure_classify.cpp - the structure classification rule of terrain migration phase 7a
// (tools/structure_census/structure_classify.h, docs/structure_classification.md): the design floor and the run / class /
// shape rule on hand-built profiles, plus the measured profile of the Unterliederbach underpass (way 4812754, the
// Königsteiner Straße under the rail embankment, DGM1 via g2m_structure_census --dump).
//
// Recorded sabotages (each made the named cases fail, then was reverted):
//   S1 station_kind without the bridge check (a mapped bridge above becomes a tube where the DEM gap is high):
//      "a mapped bridge above is a deck whatever the DEM says" fails (3 assertions).
//   S2 design_floor without the portal cones (the floor no longer meets the portal anchors):
//      "the design floor meets every portal anchor exactly ..." fails (the valley profile leaves both portals at 71.8 m
//      instead of 100 m) and "an end inside the tunnel network ..." fails (the portal end 3e-13 m off its anchor).
//   S3 the bored rule on the design cover instead of the natural cover:
//      "bored needs length AND natural cover" fails (300 m tubes with design cover 20 m and natural cover 14.9 / 10 m
//      turn bored).
//   S4 the gap rule's `>=` against min_gap_m as `>`: "a cut gap inside a tube ..." fails at a span of exactly 10 m.
#include "structure_classify.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <limits>
#include <vector>

using namespace g2m::structure;

namespace {
std::vector<Above> nothing(std::size_t n) { return std::vector<Above>(n, Above::Nothing); }
} // namespace

TEST_CASE("a deep tube under a hill is real cover with two portals", "[structure]") {
    const std::vector<double> cover(201, 20.0), natural(201, 20.0);
    const auto r = classify(cover, natural, nothing(201), Thresholds{});
    REQUIRE(r.cls == ChainClass::RealCover);
    REQUIRE(r.runs.size() == 1);
    CHECK(r.runs[0].kind == RunKind::Tube);
    CHECK(r.runs[0].shape == TubeShape::Box);   // 200 m < 300 m
    CHECK(r.portals == 2);
}

TEST_CASE("a mapped bridge above is a deck whatever the DEM says", "[structure]") {
    // DGM1 removed the bridge and interpolated: the 'cover' there is an artefact; stations 40..59 lie under a rail bridge.
    std::vector<double> cover(101, 1.0);
    std::vector<Above> above = nothing(101);
    for (std::size_t i = 40; i < 60; ++i) { cover[i] = 9.0; above[i] = Above::RailBridge; }
    const auto r = classify(cover, cover, above, Thresholds{});
    REQUIRE(r.runs.size() == 3);
    CHECK(r.runs[1].kind == RunKind::Deck);
    CHECK(r.runs[1].first == 40);
    CHECK(r.runs[1].last == 59);
    CHECK(r.cls == ChainClass::NoCover);
    CHECK(r.portals == 0);
}

TEST_CASE("a ground road above with too little cover is an implied deck and a conflict", "[structure]") {
    std::vector<double> cover(41, 0.5);
    std::vector<Above> above = nothing(41);
    for (std::size_t i = 15; i < 25; ++i) { cover[i] = 3.0; above[i] = Above::RoadGround; }
    const auto r = classify(cover, cover, above, Thresholds{});
    REQUIRE(r.runs.size() == 3);
    CHECK(r.runs[0].kind == RunKind::Slab);
    CHECK(r.runs[1].kind == RunKind::ImpliedDeck);
    CHECK(r.ground_road_conflicts == 10);
    // With enough cover the same road above is just terrain over a tube: no conflict.
    for (std::size_t i = 0; i < cover.size(); ++i) cover[i] = 8.0;
    const auto deep = classify(cover, cover, above, Thresholds{});
    CHECK(deep.ground_road_conflicts == 0);
    CHECK(deep.cls == ChainClass::RealCover);
}

TEST_CASE("a cut gap inside a tube shorter than min_gap stays tube with an exposed roof", "[structure]") {
    // Tube 0..49, gap, tube from the gap end to 120. The span between the tube stations either side is the gap length.
    const auto run_with_gap = [](std::size_t gap_first, std::size_t gap_last) {
        std::vector<double> cover(121, 10.0);
        for (std::size_t i = gap_first; i <= gap_last; ++i) cover[i] = 2.0;
        return classify(cover, cover, nothing(cover.size()), Thresholds{});
    };
    const auto short_gap = run_with_gap(50, 58);   // tube stations 49 and 59: span 10 m - not < 10 m
    CHECK(short_gap.cls == ChainClass::Mixed);
    const auto shorter = run_with_gap(50, 57);     // span 9 m
    REQUIRE(shorter.runs.size() == 1);
    CHECK(shorter.runs[0].kind == RunKind::Tube);
    CHECK(shorter.runs[0].exposed_roof);
    CHECK(shorter.cls == ChainClass::RealCover);
    CHECK(shorter.portals == 2);
}

TEST_CASE("a tube run shorter than min_tube is not a tube", "[structure]") {
    std::vector<double> cover(61, 1.0);
    std::vector<Above> above = nothing(61);
    for (std::size_t i = 20; i < 29; ++i) { cover[i] = 7.0; above[i] = Above::RailGround; }   // 8 m of 'tube'
    const auto r = classify(cover, cover, above, Thresholds{});
    CHECK(r.cls == ChainClass::NoCover);
    REQUIRE(r.runs.size() == 3);
    CHECK(r.runs[1].kind == RunKind::ImpliedDeck);   // the rail above now needs a deck
}

TEST_CASE("bored needs length AND natural cover", "[structure]") {
    const Thresholds t;
    const auto shape = [&](std::size_t len, double design, double natural) {
        const std::vector<double> c(len + 1, design), n(len + 1, natural);
        const auto r = classify(c, n, nothing(len + 1), t);
        REQUIRE(r.runs.size() == 1);
        return r.runs[0].shape;
    };
    CHECK(shape(300, 20, 15.0) == TubeShape::Bored);
    CHECK(shape(299, 20, 30.0) == TubeShape::Box);    // too short
    CHECK(shape(300, 20, 14.9) == TubeShape::Box);    // too little terrain above the portal chord
    CHECK(shape(2000, 6.0, -1.0) == TubeShape::Box);  // a flat-city tunnel deepened to its needed cover
    CHECK(shape(300, 20, 10.0) == TubeShape::Box);    // S3: design cover alone does not make it bored
}

TEST_CASE("no DEM is never a tube", "[structure]") {
    std::vector<double> cover(80, 10.0);
    for (std::size_t i = 20; i < 50; ++i) cover[i] = std::numeric_limits<double>::quiet_NaN();
    const auto r = classify(cover, cover, nothing(80), Thresholds{});
    CHECK(r.cls == ChainClass::Mixed);
    bool nan_tube = false;
    for (const auto& run : r.runs) if (run.kind == RunKind::Tube && run.first <= 20 && run.last >= 49) nan_tube = true;
    CHECK_FALSE(nan_tube);
}

TEST_CASE("the design floor meets every portal anchor exactly and never exceeds the grade", "[structure]") {
    // A hump of ground between two portals at different heights; the floor must start and end on the anchors bit for
    // bit (profile continuity at a portal is a hard requirement, docs/structure_classification.md section 5).
    std::vector<double> ground(121);
    for (std::size_t i = 0; i < ground.size(); ++i) { const double s = static_cast<double>(i); ground[i] = 100.0 + 0.002 * s * (120.0 - s); }
    const double z0 = 99.37, z1 = 101.05, g = 0.05, needed = 5.7;
    const auto f = design_floor(ground, 120.0, z0, z1, true, true, g, needed);
    REQUIRE(f.size() == ground.size());
    CHECK(f.front() == z0);
    CHECK(f.back() == z1);
    for (std::size_t i = 1; i < f.size(); ++i) CHECK(std::fabs(f[i] - f[i - 1]) <= g + 1e-12);
    // Never above the chord.
    for (std::size_t i = 0; i < f.size(); ++i) CHECK(f[i] <= z0 + (z1 - z0) * static_cast<double>(i) / 120.0 + 1e-12);
    // A deep depression just past the portal reach (a valley the tunnel line crosses): the floor would like to be far
    // below the portal, yet still leaves the portal on its anchor and at most at the grade.
    std::vector<double> valley(401, 100.0);
    for (std::size_t i = 150; i <= 250; ++i) valley[i] = 70.0;
    const auto v = design_floor(valley, 400.0, 100.0, 100.0, true, true, g, needed);
    CHECK(v.front() == 100.0);
    CHECK(v.back() == 100.0);
    for (std::size_t i = 1; i < v.size(); ++i) CHECK(std::fabs(v[i] - v[i - 1]) <= g + 1e-12);
}

TEST_CASE("hill: the design floor is the portal chord (D52)", "[structure]") {
    std::vector<double> ground(401);
    for (std::size_t i = 0; i < ground.size(); ++i) { const double s = static_cast<double>(i); ground[i] = 100.0 + 0.004 * s * (400.0 - s); }   // 160 m hill
    ground.front() = 100.0; ground.back() = 104.0;
    const auto f = design_floor(ground, 400.0, 100.0, 104.0, true, true, 0.05, 5.7);
    for (std::size_t i = 0; i < f.size(); ++i) {
        const double chord = 100.0 + 4.0 * static_cast<double>(i) / 400.0;
        // The ground falls steeply to both portal faces; the floor still stays on the chord all the way (no sag).
        CHECK(f[i] == Catch::Approx(chord).margin(1e-12));
    }
}

TEST_CASE("flat ground: the design floor drops at the grade to the needed depth and follows the ground", "[structure]") {
    const std::vector<double> ground(1001, 100.0);
    const double g = 0.04, needed = 7.0;
    const auto f = design_floor(ground, 1000.0, 100.0, 100.0, true, true, g, needed);
    CHECK(f[0] == 100.0);
    CHECK(f[100] == Catch::Approx(96.0).margin(1e-9));          // 100 m at 4 %
    CHECK(f[175] == Catch::Approx(93.0).margin(1e-9));          // 175 m: needed depth reached
    CHECK(f[500] == Catch::Approx(93.0).margin(1e-9));
    CHECK(f[1000] == 100.0);
    std::vector<double> cover(ground.size()), natural(ground.size(), 0.0);
    for (std::size_t i = 0; i < ground.size(); ++i) cover[i] = ground[i] - f[i];
    Thresholds t; t.clearance_m = 6.0; t.max_grade = g;
    const auto r = classify(cover, natural, nothing(cover.size()), t);
    CHECK(r.cls == ChainClass::Mixed);   // 175 m portal hoods at each end, a tube between
    REQUIRE(r.runs.size() == 3);
    CHECK(r.runs[1].kind == RunKind::Tube);
    CHECK(r.runs[1].first == 175);
    CHECK(r.runs[1].last == 825);
    CHECK(r.runs[1].shape == TubeShape::Box);
}

TEST_CASE("an end inside the tunnel network is anchored at the needed depth, not at the ground", "[structure]") {
    const std::vector<double> ground(301, 100.0);
    const auto f = design_floor(ground, 300.0, 100.0, 100.0, true, false, 0.05, 5.7);
    CHECK(f.front() == 100.0);
    CHECK(f.back() == Catch::Approx(94.3).margin(1e-12));
    CHECK(f[200] == Catch::Approx(94.3).margin(1e-12));
}

TEST_CASE("Unterliederbach: Koenigsteiner Strasse under the rail embankment (way 4812754)", "[structure]") {
    // Lowest DGM1 height across the 10 m carriageway every 1 m (g2m_structure_census --dump 4812754, store
    // rhein-main-hessen-r1); stations 4..30 and 37..45 lie under ground rail sidings on an embankment (OSM tags no bridge).
    const std::vector<double> ground{98.72, 98.83, 100.09, 100.33, 100.51, 100.69, 100.98, 101.41, 101.84, 102.22, 102.57, 102.88,
                                     103.20, 103.51, 103.78, 103.89, 103.96, 103.91, 103.54, 103.45, 103.39, 103.32, 103.06, 102.76,
                                     102.54, 102.43, 102.49, 102.01, 99.91, 99.00, 98.85, 98.87, 98.86, 98.88, 98.87, 98.92,
                                     98.90, 99.23, 99.70, 99.76, 99.06, 99.01, 99.01, 99.02, 99.00, 99.01, 99.04, 99.03};
    std::vector<Above> above = nothing(ground.size());
    for (std::size_t i = 4; i <= 30; ++i) above[i] = Above::RailGround;
    for (std::size_t i = 37; i <= 45; ++i) above[i] = Above::RailGround;
    const double length = 46.31;
    // The served tunnel floor (D47/D65) anchors at ONE DEM sample at the NW portal node: 99.52 on the axis. The ground
    // road 480160094 outside lies at 98.68 (median DGM1 2..8 m out). That 0.84 m is the step the game measured
    // (coordinator, 2026-10-20: 0.8-0.9 m over 1-2 m, a car launched off the lip).
    const double node_sample = 99.52, approach = 98.68, se_portal = 99.09;
    CHECK(node_sample - approach > 0.8);
    Thresholds t;   // road: clearance 4.7 + roof 1.0, grade 5 %
    const auto f = design_floor(ground, length, approach, se_portal, true, true, t.max_grade, t.cover_needed());
    CHECK(f.front() == approach);       // anchored on the ground road's end: no step
    CHECK(f.back() == se_portal);
    std::vector<double> cover(ground.size()), natural(ground.size());
    for (std::size_t i = 0; i < ground.size(); ++i) {
        const double s = std::fmin(static_cast<double>(i), length);
        cover[i] = ground[i] - f[i];
        natural[i] = ground[i] - (approach + (se_portal - approach) * s / length);
    }
    const auto r = classify(cover, natural, above, t);
    CHECK(r.cls == ChainClass::NoCover);   // the embankment is too thin over a 46 m underpass: no tube
    bool implied = false;
    for (const auto& run : r.runs) implied = implied || (run.kind == RunKind::ImpliedDeck && run.first <= 10 && run.last >= 25);
    CHECK(implied);                        // a box opening whose roof is the (untagged) rail deck
    CHECK(r.portals == 0);
}
