// tests/golden/golden.cpp - see golden.h.

#include "golden.h"

#include "g2m/core/det_math.h"
#include "g2m/core/geo/ecef.h"
#include "g2m/core/geo/session_frame.h"
#include "g2m/core/geo/tm.h"
#include "g2m/core/geo/utm.h"
#include "g2m/core/hash.h"
#include "g2m/core/raster.h"
#include "g2m/core/tile_key.h"

#include "splitmix64.h"

#include <cmath>
#include <cstdio>
#include <functional>
#include <iterator>
#include <string_view>
#include <limits>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;
using g2m_test::SplitMix64;
namespace det = g2m::det;
namespace geo = g2m::geo;

// Accumulates one entry. NaNs are hashed as one canonical marker: whether a
// NaN result keeps the input's payload is not part of the contract.
class Acc {
public:
    void f(double v) {
        if (std::isnan(v)) {
            h_.u64(0x7ff8dead0000beefull);
        } else {
            h_.f64(v);
        }
        ++n_;
    }
    void u(std::uint64_t v) {
        h_.u64(v);
        ++n_;
    }
    void s(const std::string& v) {
        h_.str(v);
        ++n_;
    }
    Entry done(std::string name) const { return {std::move(name), n_, h_.value()}; }

private:
    Fnv1a64 h_;
    std::uint64_t n_ = 0;
};

// Wide-range magnitude: (1 + u) * 2^e, e uniform in [emin, emax]; exact.
double wide(SplitMix64& r, int emin, int emax) {
    const int e = emin + static_cast<int>(r.next() % static_cast<std::uint64_t>(emax - emin + 1));
    return std::ldexp(1.0 + r.unit(), e);
}
double signed_wide(SplitMix64& r, int emin, int emax) {
    const double m = wide(r, emin, emax);
    return (r.next() & 1) ? -m : m;
}

constexpr double kInf = std::numeric_limits<double>::infinity();
const double kSpecials[] = {0.0, -0.0, kInf, -kInf, std::numeric_limits<double>::quiet_NaN(),
                            std::numeric_limits<double>::denorm_min(), -std::numeric_limits<double>::denorm_min(),
                            std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
                            -std::numeric_limits<double>::max(), 1.0, -1.0, 0.5, 2.0, 0x1p-30, 710.0, -745.2};

constexpr int kMathN = 20000;

Entry unary(const char* name, double (*fn)(double), std::uint64_t seed, const std::function<double(SplitMix64&)>& gen) {
    Acc a;
    for (double x : kSpecials) a.f(fn(x));
    SplitMix64 r(seed);
    for (int i = 0; i < kMathN; ++i) a.f(fn(gen(r)));
    return a.done(name);
}

Entry binary(const char* name, double (*fn)(double, double), std::uint64_t seed,
             const std::function<double(SplitMix64&)>& gx, const std::function<double(SplitMix64&)>& gy) {
    Acc a;
    for (double x : kSpecials) {
        for (double y : kSpecials) a.f(fn(x, y));
    }
    SplitMix64 r(seed);
    for (int i = 0; i < kMathN; ++i) {
        const double x = gx(r);
        a.f(fn(x, gy(r)));
    }
    return a.done(name);
}

void det_math(std::vector<Entry>& out) {
    const auto trig = [](SplitMix64& r) {
        const std::uint64_t k = r.next() % 4;
        return k == 0 ? r.uniform(-4.0, 4.0) : k == 1 ? r.uniform(-1e4, 1e4) : signed_wide(r, -40, 60);
    };
    const auto unit_range = [](SplitMix64& r) { return r.uniform(-1.0, 1.0); };
    const auto any = [](SplitMix64& r) { return signed_wide(r, -60, 60); };
    const auto expo = [](SplitMix64& r) { return (r.next() & 1) ? r.uniform(-750.0, 710.0) : r.uniform(-2.0, 2.0); };
    const auto positive = [](SplitMix64& r) { return wide(r, -1074, 1023); };
    const auto hyper = [](SplitMix64& r) { return (r.next() & 1) ? r.uniform(-25.0, 25.0) : r.uniform(-0.5, 0.5); };

    out.push_back(unary("det.sin", det::sin, 1, trig));
    out.push_back(unary("det.cos", det::cos, 2, trig));
    {
        Acc a;
        const auto both = [&a](double x) {
            double s = 0, c = 0;
            det::sincos(x, s, c);
            a.f(s);
            a.f(c);
        };
        for (double x : kSpecials) both(x);
        SplitMix64 r(3);
        for (int i = 0; i < kMathN; ++i) both(trig(r));
        out.push_back(a.done("det.sincos"));
    }
    out.push_back(unary("det.tan", det::tan, 4, trig));
    out.push_back(unary("det.asin", det::asin, 5, unit_range));
    out.push_back(unary("det.acos", det::acos, 6, unit_range));
    out.push_back(unary("det.atan", det::atan, 7, any));
    out.push_back(binary("det.atan2", det::atan2, 8, any, any));
    out.push_back(unary("det.exp", det::exp, 9, expo));
    out.push_back(unary("det.expm1", det::expm1, 10, expo));
    out.push_back(unary("det.log", det::log, 11, positive));
    out.push_back(unary("det.log1p", det::log1p, 12, [](SplitMix64& r) { return r.uniform(-1.0, 1e3); }));
    out.push_back(binary("det.pow", det::pow, 13, [](SplitMix64& r) { return wide(r, -20, 20); },
                         [](SplitMix64& r) { return r.uniform(-40.0, 40.0); }));
    out.push_back(unary("det.sinh", det::sinh, 14, hyper));
    out.push_back(unary("det.cosh", det::cosh, 15, hyper));
    out.push_back(unary("det.tanh", det::tanh, 16, hyper));
    out.push_back(unary("det.asinh", det::asinh, 17, any));
    out.push_back(unary("det.atanh", det::atanh, 18, unit_range));
    out.push_back(binary("det.hypot", det::hypot, 19, any, any));
    out.push_back(unary("det.sqrt", det::sqrt, 20, positive));
}

void geodesy(std::vector<Entry>& out) {
    const geo::TransverseMercator tm(geo::kGRS80, geo::kUtmK0);
    {
        Acc fwd, rev;
        SplitMix64 r(101);
        for (int i = 0; i < 5000; ++i) {
            const double lon0 = geo::utm_central_meridian(1 + static_cast<int>(r.next() % 60));
            const double lat = r.uniform(-80.0, 84.0);
            const double lon = lon0 + r.uniform(-9.0, 9.0);
            const geo::TmForward f = tm.forward(lon0, lat, lon);
            fwd.f(f.x);
            fwd.f(f.y);
            fwd.f(f.gamma);
            fwd.f(f.k);
            const geo::TmReverse b = tm.reverse(lon0, f.x, f.y);
            rev.f(b.lat);
            rev.f(b.lon);
            rev.f(b.gamma);
            rev.f(b.k);
        }
        out.push_back(fwd.done("tm.forward"));
        out.push_back(rev.done("tm.reverse"));
    }
    {
        Acc a;
        SplitMix64 r(102);
        for (int i = 0; i < 2000; ++i) {
            const double lat = r.uniform(-90.0, 90.0);
            const double lon = r.uniform(-180.0, 180.0);
            const geo::Vec3d p = geo::geodetic_to_ecef(geo::kGRS80, lat, lon, r.uniform(-500.0, 9000.0));
            a.f(p.x);
            a.f(p.y);
            a.f(p.z);
            const geo::EnuFrame e = geo::enu_frame(lat, lon);
            for (const geo::Vec3d* v : {&e.east, &e.north, &e.up}) {
                a.f(v->x);
                a.f(v->y);
                a.f(v->z);
            }
        }
        out.push_back(a.done("geo.ecef_enu"));
    }
    {
        Acc a;
        SplitMix64 r(103);
        for (int i = 0; i < 5000; ++i) {
            const double lat = (i % 4 == 0) ? r.uniform(55.0, 84.5) : r.uniform(-90.0, 90.0);
            const double lon = (i % 4 == 0) ? r.uniform(-3.0, 45.0) : r.uniform(-180.0, 180.0);
            const auto z = geo::utm_standard_zone(lat, lon);
            a.u(z ? static_cast<std::uint64_t>(z->zone * 2 + static_cast<int>(z->hemisphere)) : 0xffffu);
            a.u(static_cast<std::uint64_t>(static_cast<std::int64_t>(geo::utm_latitude_band(lat))));
        }
        out.push_back(a.done("utm.zone_band"));
    }
}

void tiles_and_frames(std::vector<Entry>& out) {
    {
        Acc a;
        SplitMix64 r(201);
        for (int i = 0; i < 20000; ++i) {
            const geo::UtmZone zone{1 + static_cast<int>(r.next() % 60),
                                    (r.next() & 1) ? geo::Hemisphere::South : geo::Hemisphere::North};
            const int level = static_cast<int>(r.next() % 23) - 1; // includes -1 and 21 (invalid)
            const double e = r.uniform(-2e6, 3e6);
            const double n = (i % 8 == 0) ? std::floor(r.uniform(0.0, 1e7) / 256.0) * 256.0 : r.uniform(-1e5, 1e7);
            const auto k = g2m::tile_key_at(zone, level, e, n);
            a.u(k ? k->packed() : 0);
            if (k && i % 50 == 0) a.s(g2m::to_string(*k));
            if (k && k->level < g2m::TileKey::kMaxLevel) a.u(k->parent().packed());
        }
        out.push_back(a.done("tile.keys"));
    }
    {
        Acc a;
        const double spawns[][2] = {{50.1109, 8.6821}, {-33.8688, 151.2093}, {64.1466, -21.9426}, {0.0, 9.0},
                                    {-79.9, -177.0}, {83.9, 20.0}};
        SplitMix64 r(202);
        for (const auto& s : spawns) {
            const auto frame = geo::SessionFrame::around(s[0], s[1]);
            a.u(frame ? static_cast<std::uint64_t>(frame->e0_m()) : 0);
            a.u(frame ? static_cast<std::uint64_t>(frame->n0_m()) : 0);
            if (!frame) continue;
            for (int i = 0; i < 1000; ++i) {
                const geo::LocalXY p{r.uniform(-60000.0, 60000.0), r.uniform(-60000.0, 60000.0)};
                const geo::TmReverse g = frame->to_geodetic(p);
                a.f(g.lat);
                a.f(g.lon);
                a.f(g.gamma);
                a.f(g.k);
                const geo::LocalXY q = frame->from_geodetic(g.lat, g.lon);
                a.f(q.x);
                a.f(q.y);
                const auto t = frame->tile_at(p, static_cast<int>(r.next() % 6));
                a.u(t ? t->packed() : 0);
                if (t) {
                    const geo::LocalXY o = frame->tile_origin(*t);
                    a.f(o.x);
                    a.f(o.y);
                }
            }
        }
        out.push_back(a.done("session.frame"));
    }
}

void rasters(std::vector<Entry>& out) {
    {
        Acc a;
        SplitMix64 r(301);
        for (int i = 0; i < 20000; ++i) {
            const double h = (i % 3 == 0) ? r.uniform(-500.0, 9000.0) : signed_wide(r, -12, 25);
            const auto q = g2m::quantise_height(h);
            a.u(q ? static_cast<std::uint64_t>(static_cast<std::int64_t>(*q)) : 0x8000000000000000ull);
            if (q) a.f(g2m::dequantise_height(*q));
        }
        out.push_back(a.done("raster.quantise"));
    }
    {
        Acc a;
        SplitMix64 r(302);
        const int cells[] = {1, 2, 4, 5, 8, 16, 32, 64, 128, 256, 512, 1024, 1280, 3};
        for (int i = 0; i < 3000; ++i) {
            // One draw per statement: function-argument evaluation order is
            // unspecified (MSVC ABI right-to-left, Itanium left-to-right).
            const int level = static_cast<int>(r.next() % 5);
            const double e = r.uniform(3e5, 7e5);
            const double n = r.uniform(5e6, 6e6);
            const auto k = g2m::tile_key_at({32, geo::Hemisphere::North}, level, e, n);
            const int c = cells[r.next() % std::size(cells)];
            const auto reg = (r.next() & 1) ? g2m::SampleRegistration::Cell : g2m::SampleRegistration::Node;
            const auto order = (r.next() & 1) ? g2m::RowOrder::NorthToSouth : g2m::RowOrder::SouthToNorth;
            const auto g = g2m::tile_georef(*k, c, reg, order);
            a.u(g ? 1 : 0);
            if (!g) continue;
            a.u(static_cast<std::uint64_t>(g->width));
            for (int s : {0, 1, g->width / 2, g->width - 1}) {
                a.u(static_cast<std::uint64_t>(g->sample_easting_half_mm(s)));
                a.u(static_cast<std::uint64_t>(g->sample_northing_half_mm(s)));
                a.f(g->sample_easting_m(s));
                a.f(g->sample_northing_m(s));
            }
            // Against a DGM1-like source lattice (1 m cells, half-metre centres, N->S).
            g2m::RasterGeoref dgm;
            dgm.zone = {32, geo::Hemisphere::North};
            dgm.west_mm = (g->west_mm / 1000000) * 1000000;
            dgm.south_mm = (g->south_mm / 1000000) * 1000000;
            dgm.registration = g2m::SampleRegistration::Cell;
            dgm.row_order = g2m::RowOrder::NorthToSouth;
            dgm.width = 1000;
            dgm.height = 1000;
            const auto off = g2m::lattice_offset(dgm, *g);
            a.u(off ? 1 : 0);
            if (off) {
                a.u(static_cast<std::uint64_t>(off->dcol));
                a.u(static_cast<std::uint64_t>(off->drow));
                a.u(off->row_flip ? 1 : 0);
            }
        }
        out.push_back(a.done("raster.georef"));
    }
    {
        // SHA-256 of 1 MiB of SplitMix bytes, plus of each prefix length 0..200.
        Acc a;
        SplitMix64 r(303);
        std::string buf(1u << 20, '\0');
        for (std::size_t i = 0; i < buf.size(); i += 8) {
            std::uint64_t v = r.next();
            for (std::size_t b = 0; b < 8; ++b) buf[i + b] = static_cast<char>((v >> (8 * b)) & 0xff);
        }
        for (std::size_t len = 0; len <= 200; ++len) a.s(g2m::to_hex(g2m::Sha256::of(std::string_view(buf).substr(0, len))));
        a.s(g2m::to_hex(g2m::Sha256::of(buf)));
        out.push_back(a.done("hash.sha256"));
    }
}

} // namespace

std::vector<Entry> compute_g0() {
    std::vector<Entry> out;
    det_math(out);
    geodesy(out);
    tiles_and_frames(out);
    rasters(out);
    // All of the above in one value.
    Fnv1a64 all;
    std::uint64_t n = 0;
    for (const Entry& e : out) {
        all.str(e.name).u64(e.count).u64(e.hash);
        n += e.count;
    }
    out.push_back({"all", n, all.value()});
    return out;
}

std::string format(const std::vector<Entry>& entries) {
    std::string s;
    for (const Entry& e : entries) {
        s += e.name;
        s += ' ';
        s += std::to_string(e.count);
        s += ' ';
        s += g2m::to_hex(e.hash);
        s += '\n';
    }
    return s;
}

} // namespace g2m_golden
