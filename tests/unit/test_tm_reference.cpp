// tests/unit/test_tm_reference.cpp - g2m's transverse Mercator, ECEF and
// UTM zone logic against GeographicLib reference values
// (testdata/geodesy/*.txt, written by tools/refgen with an 80-bit long
// double, 8th-order GeographicLib build). G0 acceptance (racing_game
// PLAN.md section 12): forward/inverse < 1 um at 10,000 points including
// the extended zone +-9 degrees; round trip <= 5 nm (ruling 2026-09-25).
//
// Every reference value is stored as hi + lo (hi = the double nearest the
// long double result), so errors are measured below 1 ulp: err = (v - hi)
// - lo, where v - hi is exact for close values.

#include "g2m/core/geo/angle.h"
#include "g2m/core/geo/ecef.h"
#include "g2m/core/geo/tm.h"
#include "g2m/core/geo/utm.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace {

using namespace g2m::geo;

constexpr double kA = 6378137.0;

struct Record {
    char kind;
    std::vector<double> v;
};

std::vector<Record> load(const std::string& name) {
    const std::string path = std::string(G2M_SOURCE_DIR) + "/testdata/geodesy/" + name;
    std::ifstream in(path);
    REQUIRE(in.good());
    std::vector<Record> out;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') continue;
        std::istringstream ss(line);
        Record r;
        std::string tok;
        ss >> tok;
        r.kind = tok[0];
        while (ss >> tok) r.v.push_back(std::strtod(tok.c_str(), nullptr)); // hex floats: exact
        out.push_back(std::move(r));
    }
    return out;
}

double err(double v, double hi, double lo) { return (v - hi) - lo; }

// Ground distance (m) of a small (dlat, dlon) degree offset at latitude lat.
// Sphere of radius a: accurate to ~0.7 % for error measurement.
double ground_m(double dlat_deg, double dlon_deg, double lat_deg) {
    double s = 0, c = 0;
    sincosd(lat_deg, s, c);
    return std::hypot(dlat_deg * kDegree * kA, dlon_deg * kDegree * kA * c);
}

struct TmStats {
    int forward = 0, reverse = 0;
    double fwd_pos = 0, fwd_gamma = 0, fwd_k = 0;
    double rev_pos = 0, rev_gamma = 0, rev_k = 0;
    double rt_geo = 0;   // forward -> reverse, ground metres
    double rt_grid = 0;  // reverse -> forward, grid metres
    int rt_geo_over_1nm = 0, rt_grid_over_1nm = 0;
};

TmStats run_tm(const std::string& file, const Ellipsoid& ell) {
    const TransverseMercator tm(ell, kUtmK0);
    TmStats s;
    for (const Record& r : load(file)) {
        const auto& v = r.v;
        if (r.kind == 'F') {
            REQUIRE(v.size() == 11);
            const double lon0 = v[0], lat = v[1], lon = v[2];
            const TmForward f = tm.forward(lon0, lat, lon);
            s.fwd_pos = std::max(s.fwd_pos, std::hypot(err(f.x, v[3], v[4]), err(f.y, v[5], v[6])));
            s.fwd_gamma = std::max(s.fwd_gamma, std::fabs(err(f.gamma, v[7], v[8])));
            s.fwd_k = std::max(s.fwd_k, std::fabs(err(f.k, v[9], v[10])));
            // Round trip geodetic -> grid -> geodetic, in g2m alone.
            const TmReverse back = tm.reverse(lon0, f.x, f.y);
            const double e_geo = ground_m(back.lat - lat, ang_diff(lon, back.lon), lat);
            s.rt_geo = std::max(s.rt_geo, e_geo);
            if (e_geo > 1e-9) ++s.rt_geo_over_1nm;
            ++s.forward;
        } else if (r.kind == 'R') {
            REQUIRE(v.size() == 11);
            const double lon0 = v[0], x = v[1], y = v[2];
            const TmReverse b = tm.reverse(lon0, x, y);
            const double dlat = err(b.lat, v[3], v[4]);
            const double dlon = ang_diff(v[5], b.lon) - v[6];
            s.rev_pos = std::max(s.rev_pos, ground_m(dlat, dlon, v[3]));
            s.rev_gamma = std::max(s.rev_gamma, std::fabs(err(b.gamma, v[7], v[8])));
            s.rev_k = std::max(s.rev_k, std::fabs(err(b.k, v[9], v[10])));
            // Round trip grid -> geodetic -> grid.
            const TmForward again = tm.forward(lon0, b.lat, b.lon);
            const double e_grid = std::hypot(again.x - x, again.y - y);
            s.rt_grid = std::max(s.rt_grid, e_grid);
            if (e_grid > 1e-9) ++s.rt_grid_over_1nm;
            ++s.reverse;
        }
    }
    std::printf("%s: %d forward, %d reverse points\n", file.c_str(), s.forward, s.reverse);
    std::printf("  forward  max position error %.3f nm, gamma %.3g deg, k %.3g\n", s.fwd_pos * 1e9, s.fwd_gamma,
                s.fwd_k);
    std::printf("  reverse  max position error %.3f nm, gamma %.3g deg, k %.3g\n", s.rev_pos * 1e9, s.rev_gamma,
                s.rev_k);
    std::printf("  round trip geo->grid->geo max %.3f nm (%d points > 1 nm), grid->geo->grid max %.3f nm (%d > 1 nm)\n",
                s.rt_geo * 1e9, s.rt_geo_over_1nm, s.rt_grid * 1e9, s.rt_grid_over_1nm);
    return s;
}

} // namespace

TEST_CASE("TM vs GeographicLib, GRS80, 10000 points incl. extended zone", "[geo]") {
    const TmStats s = run_tm("tm_grs80.txt", kGRS80);
    CHECK(s.forward >= 10000);
    CHECK(s.reverse >= 10000);
    // Acceptance: < 1 um.
    CHECK(s.fwd_pos < 1e-6);
    CHECK(s.rev_pos < 1e-6);
    // Convergence and scale: far below any use (measured ~1e-13 deg / ~1e-15).
    CHECK(s.fwd_gamma < 1e-11);
    CHECK(s.rev_gamma < 1e-11);
    CHECK(s.fwd_k < 1e-13);
    CHECK(s.rev_k < 1e-13);
}

TEST_CASE("TM vs GeographicLib, WGS84", "[geo]") {
    const TmStats s = run_tm("tm_wgs84.txt", kWGS84);
    CHECK(s.forward >= 1000);
    CHECK(s.fwd_pos < 1e-6);
    CHECK(s.rev_pos < 1e-6);
}

// Round trip, both directions, g2m only. Acceptance (ruling 2026-09-25,
// PLAN.md 4.9): <= 5 nm. Measured 3.16 nm geo->grid->geo. GeographicLib's
// own double build (GEOGRAPHICLIB_PRECISION=2) reaches 3.17 nm on these
// points, so the floor is representation, not the series: near 84 N the
// northing's ulp is 1.86 nm, and xi (~1.47 rad) carries 2.2e-16 rad =
// 1.4 nm of rounding before it is scaled by A k0.
TEST_CASE("TM round trip <= 5 nm", "[geo]") {
    const TmStats s = run_tm("tm_grs80.txt", kGRS80);
    REQUIRE(s.rt_geo <= 5e-9);
    REQUIRE(s.rt_grid <= 5e-9);
}

TEST_CASE("ECEF vs GeographicLib", "[geo]") {
    double worst = 0;
    int n = 0;
    for (const Record& r : load("ecef_grs80.txt")) {
        REQUIRE(r.kind == 'E');
        const auto& v = r.v;
        const Vec3d p = geodetic_to_ecef(kGRS80, v[0], v[1], v[2]);
        const double e = std::hypot(std::hypot(err(p.x, v[3], v[4]), err(p.y, v[5], v[6])), err(p.z, v[7], v[8]));
        worst = std::max(worst, e);
        ++n;
    }
    std::printf("ecef: %d points, max error %.3f nm\n", n, worst * 1e9);
    CHECK(n == 1000);
    CHECK(worst < 1e-8);
}

TEST_CASE("UTM standard zone vs GeographicLib", "[geo]") {
    int n = 0, mismatches = 0;
    for (const Record& r : load("utm_zone.txt")) {
        REQUIRE(r.kind == 'Z');
        const auto& v = r.v;
        const int expected = static_cast<int>(v[2]);
        const auto z = utm_standard_zone(v[0], v[1]);
        const int got = z ? z->zone : 0;
        if (got != expected) {
            ++mismatches;
            std::printf("zone mismatch at lat %.17g lon %.17g: g2m %d, GeographicLib %d\n", v[0], v[1], got, expected);
        }
        if (z) CHECK((z->hemisphere == Hemisphere::South) == (v[0] < 0));
        ++n;
    }
    CHECK(n == 4000);
    CHECK(mismatches == 0);
}
