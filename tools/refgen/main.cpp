// tools/refgen/main.cpp - writes GeographicLib reference values for g2m's
// geodesy tests (see CMakeLists.txt). Usage: g2m_refgen <output dir>
//
// Inputs are drawn with SplitMix64 (fixed, specified sequence) and stored
// as exact hex doubles. Every reference result is stored as hi + lo, where
// hi = (double)v and lo = (double)(v - hi), both hex: the long double value
// survives to ~64 bits, so a test can measure g2m's error below 1 ulp.
//
// Files (one record per line, '#' comments, whitespace-separated hex):
//   tm_grs80.txt / tm_wgs84.txt  (k0 = 0.9996)
//     F lon0 lat lon  x_hi x_lo  y_hi y_lo  gamma_hi gamma_lo  k_hi k_lo
//     R lon0 x y      lat_hi lat_lo  lon_hi lon_lo  gamma_hi gamma_lo  k_hi k_lo
//   ecef_grs80.txt
//     E lat lon h  X_hi X_lo  Y_hi Y_lo  Z_hi Z_lo
//   utm_zone.txt
//     Z lat lon zone     (GeographicLib UTMUPS::StandardZone; 0 = UPS)

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/TransverseMercator.hpp>
#include <GeographicLib/UTMUPS.hpp>

#include "splitmix64.h"

#include <cstdio>
#include <limits>
#include <string>
#include <vector>

using GeographicLib::Math;
using real = Math::real;

static_assert(std::numeric_limits<real>::digits >= 64,
              "refgen needs GEOGRAPHICLIB_PRECISION=3 and an 80-bit long double (build on Linux x86-64)");

namespace {

constexpr int kTmPointsGrs80 = 10000;
constexpr int kTmPointsWgs84 = 1000;
constexpr int kEcefPoints = 1000;
constexpr int kZonePoints = 4000;

void put(std::FILE* f, double d) { std::fprintf(f, " %a", d); }
void put_hilo(std::FILE* f, real v) {
    const double hi = static_cast<double>(v);
    const double lo = static_cast<double>(v - static_cast<real>(hi));
    std::fprintf(f, " %a %a", hi, lo);
}

struct Point {
    double lon0, lat, lon;
};

// Edge cases first (central meridian, strip edges at +-3, the extended zone
// at +-9, equator, near-pole, exact multiples of 30/45 degrees), then
// uniform random points: lat in [-80, 84], lon - lon0 in [-9, 9], random zone.
std::vector<Point> tm_points(int count, std::uint64_t seed) {
    std::vector<Point> pts;
    const double lats[] = {-80, -60, -45, -30, -1e-9, 0, 1e-9, 30, 45, 47.5, 50.1, 60, 75, 84, 89.5};
    const double dlons[] = {-9, -6, -3, -1e-9, 0, 1e-9, 0.5, 3, 6, 9};
    for (double lat : lats) {
        for (double dl : dlons) pts.push_back({9.0, lat, 9.0 + dl});
    }
    g2m_test::SplitMix64 rng(seed);
    while (static_cast<int>(pts.size()) < count) {
        const int zone = 1 + static_cast<int>(rng.next() % 60);
        const double lon0 = 6.0 * zone - 183.0;
        const double lat = rng.uniform(-80.0, 84.0);
        const double dlon = rng.uniform(-9.0, 9.0);
        pts.push_back({lon0, lat, lon0 + dlon});
    }
    return pts;
}

bool write_tm(const std::string& path, real a, real f, int count, std::uint64_t seed, const char* name) {
    std::FILE* out = std::fopen(path.c_str(), "w");
    if (!out) return false;
    const GeographicLib::TransverseMercator tm(a, f, real(0.9996));
    std::fprintf(out, "# GeographicLib %s TransverseMercator, %s, k0 = 0.9996, long double, series order %d\n",
                 GEOGRAPHICLIB_VERSION_STRING, name, GEOGRAPHICLIB_TRANSVERSEMERCATOR_ORDER);
    std::fprintf(out, "# F lon0 lat lon  x_hi x_lo y_hi y_lo gamma_hi gamma_lo k_hi k_lo\n");
    std::fprintf(out, "# R lon0 x y  lat_hi lat_lo lon_hi lon_lo gamma_hi gamma_lo k_hi k_lo\n");
    for (const Point& p : tm_points(count, seed)) {
        real x, y, gamma, k;
        tm.Forward(real(p.lon0), real(p.lat), real(p.lon), x, y, gamma, k);
        std::fprintf(out, "F");
        put(out, p.lon0);
        put(out, p.lat);
        put(out, p.lon);
        put_hilo(out, x);
        put_hilo(out, y);
        put_hilo(out, gamma);
        put_hilo(out, k);
        std::fprintf(out, "\n");
        // Reverse from the double-rounded forward result.
        const double xd = static_cast<double>(x);
        const double yd = static_cast<double>(y);
        real lat, lon;
        tm.Reverse(real(p.lon0), real(xd), real(yd), lat, lon, gamma, k);
        std::fprintf(out, "R");
        put(out, p.lon0);
        put(out, xd);
        put(out, yd);
        put_hilo(out, lat);
        put_hilo(out, lon);
        put_hilo(out, gamma);
        put_hilo(out, k);
        std::fprintf(out, "\n");
    }
    return std::fclose(out) == 0;
}

bool write_ecef(const std::string& path) {
    std::FILE* out = std::fopen(path.c_str(), "w");
    if (!out) return false;
    const GeographicLib::Geocentric geo(real(6378137), real(1.0 / 298.257222101));
    std::fprintf(out, "# GeographicLib %s Geocentric::Forward, GRS80, long double\n", GEOGRAPHICLIB_VERSION_STRING);
    std::fprintf(out, "# E lat lon h  X_hi X_lo Y_hi Y_lo Z_hi Z_lo\n");
    g2m_test::SplitMix64 rng(0xec3f);
    for (int i = 0; i < kEcefPoints; ++i) {
        const double lat = i == 0 ? 90.0 : i == 1 ? -90.0 : rng.uniform(-90.0, 90.0);
        const double lon = rng.uniform(-180.0, 180.0);
        const double h = rng.uniform(-500.0, 9000.0);
        real X, Y, Z;
        geo.Forward(real(lat), real(lon), real(h), X, Y, Z);
        std::fprintf(out, "E");
        put(out, lat);
        put(out, lon);
        put(out, h);
        put_hilo(out, X);
        put_hilo(out, Y);
        put_hilo(out, Z);
        std::fprintf(out, "\n");
    }
    return std::fclose(out) == 0;
}

bool write_zones(const std::string& path) {
    std::FILE* out = std::fopen(path.c_str(), "w");
    if (!out) return false;
    std::fprintf(out, "# GeographicLib %s UTMUPS::StandardZone (0 = UPS)\n", GEOGRAPHICLIB_VERSION_STRING);
    std::fprintf(out, "# Z lat lon zone\n");
    g2m_test::SplitMix64 rng(0x2043e);
    for (int i = 0; i < kZonePoints; ++i) {
        double lat, lon;
        if (i % 4 == 0) { // concentrate on the Norway/Svalbard exception area
            lat = rng.uniform(55.0, 84.5);
            lon = rng.uniform(-3.0, 45.0);
        } else {
            lat = rng.uniform(-90.0, 90.0);
            lon = rng.uniform(-180.0, 180.0);
        }
        // Integer-degree and band-edge points exercise the floor() edges.
        if (i % 16 == 1) {
            lat = static_cast<double>(static_cast<int>(lat));
            lon = static_cast<double>(static_cast<int>(lon));
        }
        const int zone = GeographicLib::UTMUPS::StandardZone(real(lat), real(lon));
        std::fprintf(out, "Z");
        put(out, lat);
        put(out, lon);
        std::fprintf(out, " %d\n", zone);
    }
    return std::fclose(out) == 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: g2m_refgen <output dir>\n");
        return 2;
    }
    const std::string dir = argv[1];
    const real a = 6378137;
    bool ok = write_tm(dir + "/tm_grs80.txt", a, real(1.0 / 298.257222101), kTmPointsGrs80, 0x9e0d, "GRS80");
    ok = ok && write_tm(dir + "/tm_wgs84.txt", a, real(1.0 / 298.257223563), kTmPointsWgs84, 0x3a84, "WGS84");
    ok = ok && write_ecef(dir + "/ecef_grs80.txt");
    ok = ok && write_zones(dir + "/utm_zone.txt");
    if (!ok) {
        std::fprintf(stderr, "g2m_refgen: failed writing to %s\n", dir.c_str());
        return 1;
    }
    std::printf("g2m_refgen: wrote %s/{tm_grs80,tm_wgs84,ecef_grs80,utm_zone}.txt\n", dir.c_str());
    return 0;
}
