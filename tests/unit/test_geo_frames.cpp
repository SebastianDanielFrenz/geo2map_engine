// tests/unit/test_geo_frames.cpp - angles, UTM wrapper, session frame.

#include "g2m/core/geo/angle.h"
#include "g2m/core/geo/session_frame.h"
#include "g2m/core/geo/utm.h"

#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdio>
#include <stdexcept>

using namespace g2m;
using namespace g2m::geo;

TEST_CASE("sincosd exact values and reduction", "[geo]") {
    double s = 0, c = 0;
    for (double x : {0.0, 90.0, 180.0, 270.0, 360.0, -90.0, 720.0, 1e9 * 360.0}) {
        sincosd(x, s, c);
        CHECK(std::fabs(s) == std::round(std::fabs(s)));
        CHECK(std::fabs(c) == std::round(std::fabs(c)));
    }
    sincosd(30.0, s, c);
    CHECK(s == 0.5);
    sincosd(45.0, s, c);
    CHECK(s == c);
    sincosd(-0.0, s, c);
    CHECK(std::signbit(s));
    CHECK(c == 1.0);
    sincosd(90.0, s, c);
    CHECK(s == 1.0);
    CHECK((c == 0.0 && !std::signbit(c)));
    // x and x + 360 k give identical bits (exact reduction).
    g2m_test::SplitMix64 rng(11);
    for (int i = 0; i < 1000; ++i) {
        const double x = std::round(rng.uniform(-180.0, 180.0) * 1024.0) / 1024.0;
        double s1 = 0, c1 = 0, s2 = 0, c2 = 0;
        sincosd(x, s1, c1);
        sincosd(x + 3600.0, s2, c2);
        CHECK(s1 == s2);
        CHECK(c1 == c2);
    }
}

TEST_CASE("atan2d quadrants and ang_diff", "[geo]") {
    CHECK(atan2d(0.0, 1.0) == 0.0);
    CHECK(atan2d(1.0, 0.0) == 90.0);
    CHECK(atan2d(0.0, -1.0) == 180.0);
    CHECK(atan2d(-0.0, -1.0) == -180.0);
    CHECK(atan2d(-1.0, 0.0) == -90.0);
    CHECK(atan2d(1.0, 1.0) == 45.0);
    CHECK(atand(1.0) == 45.0);
    CHECK(ang_normalize(540.0) == 180.0);
    CHECK(ang_normalize(-540.0) == -180.0);
    CHECK(ang_normalize(190.0) == -170.0);
    CHECK(ang_diff(179.0, -179.0) == 2.0);
    CHECK(ang_diff(-179.0, 179.0) == -2.0);
    double e = 1;
    CHECK(ang_diff(0.1, 0.3, &e) == 0.3 - 0.1);
}

TEST_CASE("UTM zones, bands and scheme names", "[geo]") {
    CHECK(utm_central_meridian(32) == 9.0);
    CHECK(utm_central_meridian(1) == -177.0);
    CHECK(utm_latitude_band(50.1) == 6);  // U
    CHECK(utm_latitude_band(-80.0) == -10);
    CHECK(utm_latitude_band(83.9) == 9);  // X (12 degrees tall)
    const auto frankfurt = utm_standard_zone(50.11, 8.68);
    REQUIRE(frankfurt);
    CHECK(frankfurt->zone == 32);
    CHECK(frankfurt->hemisphere == Hemisphere::North);
    CHECK(utm_scheme_name(*frankfurt) == "utm32n");
    CHECK(utm_scheme_name({33, Hemisphere::South}) == "utm33s");
    CHECK(utm_standard_zone(60.0, 5.0)->zone == 32);  // Norway exception
    CHECK(utm_standard_zone(78.0, 10.0)->zone == 33); // Svalbard exception
    CHECK(utm_standard_zone(-33.9, 18.4)->hemisphere == Hemisphere::South);
    CHECK(utm_standard_zone(0.0, 180.0)->zone == 1);
    CHECK_FALSE(utm_standard_zone(84.0, 0.0));
    CHECK_FALSE(utm_standard_zone(-80.5, 0.0));
    CHECK_FALSE(utm_standard_zone(NAN, 0.0));
}

TEST_CASE("UTM forward adds false offsets and reverse removes them", "[geo]") {
    const Utm utm;
    const UtmForward n = utm.forward({32, Hemisphere::North}, 50.0, 9.0);
    CHECK(n.easting == 500000.0);
    CHECK(n.northing > 5.5e6);
    CHECK(n.northing < 5.6e6);
    CHECK(n.gamma == 0.0);
    const UtmForward s = utm.forward({32, Hemisphere::South}, -50.0, 9.0);
    CHECK(s.easting == 500000.0);
    CHECK(s.northing == 1e7 - n.northing);
    const TmReverse back = utm.reverse({32, Hemisphere::North}, n.easting, n.northing);
    CHECK(std::fabs(back.lat - 50.0) < 1e-13);
    CHECK(std::fabs(back.lon - 9.0) < 1e-13);
}

TEST_CASE("session frame: origin, local coordinates, round trip", "[geo]") {
    const auto frame = SessionFrame::around(50.1109, 8.6821); // Frankfurt
    REQUIRE(frame);
    CHECK(frame->zone() == UtmZone{32, Hemisphere::North});
    CHECK(frame->e0_m() % 1000 == 0);
    CHECK(frame->n0_m() % 1000 == 0);
    const Utm utm;
    const UtmForward g = utm.forward(frame->zone(), 50.1109, 8.6821);
    CHECK(static_cast<double>(frame->e0_m()) <= g.easting);
    CHECK(g.easting < static_cast<double>(frame->e0_m() + 1000));
    const LocalXY p = frame->from_geodetic(50.1109, 8.6821);
    CHECK(p.x >= 0.0);
    CHECK(p.x < 1000.0);
    CHECK(p.y >= 0.0);
    CHECK(p.y < 1000.0);
    // Local = grid - origin (to well below a nanometre at this magnitude).
    const LocalXY q = frame->from_grid(g.easting, g.northing);
    CHECK(std::fabs(q.x - p.x) < 1e-9);
    CHECK(std::fabs(q.y - p.y) < 1e-9);

    // Round trip across a 100 x 100 km session area.
    g2m_test::SplitMix64 rng(21);
    double worst = 0;
    for (int i = 0; i < 20000; ++i) {
        const LocalXY a{rng.uniform(-50000.0, 50000.0), rng.uniform(-50000.0, 50000.0)};
        const TmReverse geo = frame->to_geodetic(a);
        const LocalXY b = frame->from_geodetic(geo.lat, geo.lon);
        worst = std::max(worst, std::hypot(b.x - a.x, b.y - a.y));
    }
    std::printf("session frame local->geo->local round trip over +-50 km: max %.3f nm\n", worst * 1e9);
    CHECK(worst < 5e-9);

    CHECK_THROWS_AS(SessionFrame({32, Hemisphere::North}, 500500, 5500000), std::invalid_argument);
    CHECK_THROWS_AS(SessionFrame({0, Hemisphere::North}, 500000, 5500000), std::invalid_argument);
}

TEST_CASE("session frame tile lookup is exact at tile edges", "[geo][tile]") {
    const SessionFrame frame({32, Hemisphere::North}, 475000, 5550000);
    // 475000 = 1855 * 256 + 120: local x = 136 is the grid easting 475136 = 1856 * 256.
    const auto on_edge = frame.tile_at({136.0, 0.0}, 0);
    REQUIRE(on_edge);
    CHECK(on_edge->x == 1856);
    const auto just_below = frame.tile_at({std::nextafter(136.0, 0.0), 0.0}, 0);
    REQUIRE(just_below);
    CHECK(just_below->x == 1855);
    const auto tiny_negative = frame.tile_at({-1e-300, -1e-300}, 0);
    REQUIRE(tiny_negative);
    CHECK(tiny_negative->x == 1855);  // floor(474999.999...) / 256
    CHECK(tiny_negative->y == 21679); // floor(5549999.999...) / 256
    const auto t = *frame.tile_at({1000.5, 2000.25}, 3);
    const LocalXY o = frame.tile_origin(t);
    CHECK(o.x <= 1000.5);
    CHECK(o.x + 2048.0 > 1000.5);
    CHECK(o.y <= 2000.25);
    CHECK(o.y + 2048.0 > 2000.25);
    CHECK(o.x == std::floor(o.x));
    CHECK_FALSE(frame.tile_at({NAN, 0.0}, 0));
    CHECK_FALSE(frame.tile_at({0.0, 0.0}, 21));
}
