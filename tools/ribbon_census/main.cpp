// tools/ribbon_census/main.cpp - g2m_ribbon_census: how far the terrain mesh stands above the road ribbons (vault G2M-025).
//
//   g2m_ribbon_census SOURCE_DIR SCOPE DERIVED_DIR ZONE (--site LAT LON RADIUS_M | --box E0 N0 E1 N1) [--drape point|lattice|both]
//                     [--lift M] [--step M] [--tiles-csv FILE] [--carved 0|1]
//
// SOURCE_DIR is opened read-only (never written); DERIVED_DIR receives the derived tiles (a private scratch directory).
// For every level-2 tile in the area the road ribbons are built with the same ground function the game passes (the
// bilinear sampler on g2m.terrain.carved) and each ribbon triangle is probed on a STEP lattice (default 0.25 m): the probe's
// deviation is  terrain - ribbon  where terrain is sample_height_exact (the surface the terrain mesh and Jolt have) and
// ribbon is the interpolated vertex height minus the lift. Positive = the terrain stands above the road. Triangles are
// classified by their vertex attributes: bed (the asphalt strip, verge 0), verge (gravel strips, verge 1), patch (junction
// asphalt), paint (marking triangles, which sit a few mm above the road on purpose). With --site only probes within
// RADIUS_M of the point count. Reports probe counts above 1 cm / 10 cm, the maximum, vertices, triangles and build time.
#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/world/height_sampler.h"
#include "g2m/world/road_ribbons.h"
#include "g2m/world/terrain_service.h"
#include "g2m/world/ticket.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <vector>

using namespace g2m;
using namespace g2m::world;
using Clock = std::chrono::steady_clock;

namespace {

struct Heights {
    TerrainService& svc;
    std::mutex m;
    std::map<std::uint64_t, std::shared_ptr<const HeightTile>> tiles;
    const HeightTile* get(const TileKey& key) {
        std::lock_guard<std::mutex> lock(m);
        auto it = tiles.find(key.packed());
        if (it == tiles.end()) {
            auto t = svc.height(key, Priority::Render);
            t.wait_for(std::chrono::minutes(5));
            it = tiles.emplace(key.packed(), t.value()).first;
        }
        return it->second.get();
    }
};

struct Tally {
    std::size_t probes = 0, above1 = 0, above10 = 0, below1 = 0, nodata = 0;
    double max_above = 0, max_below = 0;
    void add(double d) {
        ++probes;
        if (d > 0.01) ++above1;
        if (d > 0.10) ++above10;
        if (d < -0.01) ++below1;
        max_above = std::max(max_above, d);
        max_below = std::min(max_below, d);
    }
};

// Triangles of the stretches that are NOT on the ground (decks over a bridge, a tunnel, an underpass), found by
// building the ribbons with a ground that has no height anywhere: the ground strips drop out, the deck strips remain.
struct DeckMask {
    std::map<std::pair<long, long>, std::vector<std::array<double, 6>>> cells;
    static constexpr double kCell = 8.0;
    void add(const RoadRibbonTile& t, double ox, double oy) {
        for (std::size_t i = 0; i + 2 < t.indices.size(); i += 3) {
            std::array<double, 6> tri{};
            double lo_x = 1e30, hi_x = -1e30, lo_y = 1e30, hi_y = -1e30;
            for (int k = 0; k < 3; ++k) {
                const auto& v = t.vertices[static_cast<std::size_t>(t.indices[i + static_cast<std::size_t>(k)])];
                tri[static_cast<std::size_t>(2 * k)] = ox + v.x;
                tri[static_cast<std::size_t>(2 * k + 1)] = oy + v.y;
                lo_x = std::min<double>(lo_x, ox + v.x); hi_x = std::max<double>(hi_x, ox + v.x);
                lo_y = std::min<double>(lo_y, oy + v.y); hi_y = std::max<double>(hi_y, oy + v.y);
            }
            for (long cx = static_cast<long>(std::floor(lo_x / kCell)); cx <= static_cast<long>(std::floor(hi_x / kCell)); ++cx)
                for (long cy = static_cast<long>(std::floor(lo_y / kCell)); cy <= static_cast<long>(std::floor(hi_y / kCell)); ++cy) cells[{cx, cy}].push_back(tri);
        }
    }
    bool covers(double x, double y) const {
        const auto it = cells.find({static_cast<long>(std::floor(x / kCell)), static_cast<long>(std::floor(y / kCell))});
        if (it == cells.end()) return false;
        for (const auto& t : it->second) {
            const auto side = [&](int a, int b) { return (t[2 * b] - t[2 * a]) * (y - t[2 * a + 1]) - (t[2 * b + 1] - t[2 * a + 1]) * (x - t[2 * a]); };
            const double s0 = side(0, 1), s1 = side(1, 2), s2 = side(2, 0);
            constexpr double eps = 1e-3; // m2: a probe on a triangle edge counts as inside
            if ((s0 >= -eps && s1 >= -eps && s2 >= -eps) || (s0 <= eps && s1 <= eps && s2 <= eps)) return true;
        }
        return false;
    }
};

const char* kind_of(const RoadRibbonVertex& v) {
    const int flags = static_cast<int>(v.flags);
    if (flags & (16 | 32)) return "paint";
    if (v.lanes == 0 && v.verge > 0.5f) return "jverge";
    if (v.lanes == 0) return "patch";
    return v.verge > 0.5f ? "verge" : "bed";
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 7) {
        std::cerr << "usage: see the header comment of tools/ribbon_census/main.cpp\n";
        return 2;
    }
    const std::string source = argv[1], scope = argv[2], derived = argv[3];
    const geo::UtmZone zone{std::atoi(argv[4]), geo::Hemisphere::North};
    double site_e = 0, site_n = 0, radius = 0, box[4] = {0, 0, 0, 0};
    bool have_site = false, have_box = false, carved = true;
    std::string drape_mode = "both", csv_path;
    double lift = 0.0, step = 0.25;
    std::size_t worst_n = 0;
    double tol = 0.005;
    double dump_e = 0, dump_n = 0, dump_r = -1;
    for (int i = 5; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--site" && i + 3 < argc) {
            const double lat = std::atof(argv[++i]), lon = std::atof(argv[++i]);
            radius = std::atof(argv[++i]);
            const auto f = geo::Utm().forward(zone, lat, lon);
            site_e = f.easting;
            site_n = f.northing;
            have_site = true;
        } else if (a == "--box" && i + 4 < argc) {
            for (double& b : box) b = std::atof(argv[++i]);
            have_box = true;
        } else if (a == "--drape" && i + 1 < argc) drape_mode = argv[++i];
        else if (a == "--lift" && i + 1 < argc) lift = std::atof(argv[++i]);
        else if (a == "--step" && i + 1 < argc) step = std::atof(argv[++i]);
        else if (a == "--worst" && i + 1 < argc) worst_n = static_cast<std::size_t>(std::atoi(argv[++i]));
        else if (a == "--dump" && i + 3 < argc) { dump_e = std::atof(argv[++i]); dump_n = std::atof(argv[++i]); dump_r = std::atof(argv[++i]); }
        else if (a == "--tol" && i + 1 < argc) tol = std::atof(argv[++i]);
        else if (a == "--carved" && i + 1 < argc) carved = std::atoi(argv[++i]) != 0;
        else if (a == "--tiles-csv" && i + 1 < argc) csv_path = argv[++i];
        else { std::cerr << "unknown option " << a << '\n'; return 2; }
    }
    if (!have_site && !have_box) { std::cerr << "need --site or --box\n"; return 2; }
    if (have_site) { box[0] = site_e - radius - 1024; box[1] = site_n - radius - 1024; box[2] = site_e + radius + 1024; box[3] = site_n + radius + 1024; }
    TerrainServiceConfig cfg;
    cfg.stores.source_dir = source;
    cfg.stores.source_scope = scope;
    cfg.stores.derived_dir = derived;
    cfg.layers.carved = carved;
    cfg.zone = zone;
    auto opened = TerrainService::open(cfg);
    if (!opened) { std::cerr << opened.error().message << '\n'; return 1; }
    TerrainService& svc = **opened;
    Heights heights{svc, {}, {}};
    const L0TileLookupFn lookup = [&](const TileKey& k) { return heights.get(k); };
    const RoadRibbonGround bilinear = [&](double e, double n) { return sample_height_bilinear(lookup, zone, 0, 0, e, n); };
    const auto k0 = tile_key_at(zone, 2, box[0], box[1]), k1 = tile_key_at(zone, 2, box[2], box[3]);
    if (!k0 || !k1) return 1;

    if (dump_r >= 0) {
        const double cx = std::floor(dump_e - 0.5) + 0.5, cy = std::floor(dump_n - 0.5) + 0.5;
        for (double dy : {0.0, 1.0})
            for (double dx : {0.0, 1.0}) {
                const auto b = bilinear(cx + dx, cy + dy);
                const auto e = sample_height_exact(lookup, zone, 0, 0, cx + dx, cy + dy);
                std::printf("   node E %.1f N %.1f bilinear %.4f exact %.4f\n", cx + dx, cy + dy, b ? *b : -1.0, e ? *e : -1.0);
            }
    }
    std::ofstream csv;
    if (!csv_path.empty()) { csv.open(csv_path); csv << "mode,tile,vertices,triangles,build_ms\n"; }
    for (const std::string_view mode : {std::string_view("point"), std::string_view("lattice")}) {
        if (drape_mode != "both" && drape_mode != mode) continue;
        std::map<std::string, Tally> tallies;
        struct Worst { double d, e, n, z; const char* kind; float lanes, flags, v; };
        std::vector<Worst> worst;
        std::size_t vertices = 0, triangles = 0, tiles = 0, d_in = 0, d_out = 0, d_uncut = 0, d_dropped = 0, d_nodes = 0;
        double build_ms = 0;
        DeckMask decks;
        RoadRibbonOptions options;
        options.drape_tolerance_m = tol;
        options.drape = mode == "point" ? RibbonDrape::PointSampled : RibbonDrape::Lattice;
        for (std::int32_t ty = k0->y; ty <= k1->y; ++ty) {
            { std::lock_guard<std::mutex> lock(heights.m); heights.tiles.clear(); } // bound the memory of a region run
            std::vector<Ticket<RoadGeomTile>> row;
            for (std::int32_t tx = k0->x; tx <= k1->x; ++tx) {
                TileKey key = *k0;
                key.x = tx;
                key.y = ty;
                row.push_back(svc.road_geom(key, Priority::Render)); // the whole row derives in parallel
            }
            for (std::int32_t tx = k0->x; tx <= k1->x; ++tx) {
                TileKey key = *k0;
                key.x = tx;
                key.y = ty;
                auto& geo = row[static_cast<std::size_t>(tx - k0->x)];
                geo.wait_for(std::chrono::minutes(30));
                const auto geometry = geo.value();
                if (!geometry) continue;
                // warm the height tiles the ribbons read (outside the timed build)
                for (double e = key.min_easting() - 4; e < key.min_easting() + 1030; e += 128)
                    for (double n = key.min_northing() - 4; n < key.min_northing() + 1030; n += 128) (void)bilinear(e, n);
                const auto t0 = Clock::now();
                const RoadRibbonTile mesh = build_road_ribbons(*geometry, key, bilinear, lift, {}, options);
                const double ms = std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
                ++tiles;
                build_ms += ms;
                vertices += mesh.vertices.size();
                d_in += mesh.drape_in; d_out += mesh.drape_out; d_uncut += mesh.drape_uncut; d_dropped += mesh.drape_dropped; d_nodes += mesh.drape_node_reads;
                triangles += mesh.indices.size() / 3;
                if (csv) csv << mode << ',' << to_string(key) << ',' << mesh.vertices.size() << ',' << mesh.indices.size() / 3 << ',' << ms << '\n';
                const double ox = key.min_easting(), oy = key.min_northing();
                {
                    const RoadRibbonGround none = [](double, double) -> std::optional<double> { return std::nullopt; };
                    RoadRibbonOptions point_only;
                    point_only.drape = RibbonDrape::PointSampled;
                    decks.add(build_road_ribbons(*geometry, key, none, lift, {}, point_only), ox, oy);
                }
                for (std::size_t t = 0; t + 2 < mesh.indices.size(); t += 3) {
                    const RoadRibbonVertex* v[3] = {&mesh.vertices[static_cast<std::size_t>(mesh.indices[t])],
                                                    &mesh.vertices[static_cast<std::size_t>(mesh.indices[t + 1])],
                                                    &mesh.vertices[static_cast<std::size_t>(mesh.indices[t + 2])]};
                    double longest = 0;
                    for (int a = 0; a < 3; ++a) longest = std::max(longest, std::hypot(double(v[a]->x) - v[(a + 1) % 3]->x, double(v[a]->y) - v[(a + 1) % 3]->y));
                    if (dump_r >= 0 && std::hypot(ox + v[0]->x - dump_e, oy + v[0]->y - dump_n) < dump_r) {
                        std::printf("   tri");
                        for (int a = 0; a < 3; ++a) {
                            const auto th = sample_height_exact(lookup, zone, 0, 0, ox + v[a]->x, oy + v[a]->y);
                            std::printf(" | E %.2f N %.2f z %.3f terr %.3f v %.2f fl %g vg %g", ox + v[a]->x, oy + v[a]->y, double(v[a]->z), th ? *th : -1.0, double(v[a]->v), double(v[a]->flags), double(v[a]->verge));
                        }
                        std::printf(" deck=%d\n", decks.covers(ox + (v[0]->x + v[1]->x + v[2]->x) / 3, oy + (v[0]->y + v[1]->y + v[2]->y) / 3) ? 1 : 0);
                    }
                    const int n = std::max(1, static_cast<int>(std::ceil(longest / step)));
                    Tally& tally = tallies[kind_of(*v[0])];
                    for (int i = 0; i <= n; ++i)
                        for (int j = 0; i + j <= n; ++j) {
                            const double b1 = double(i) / n, b2 = double(j) / n, b0 = 1 - b1 - b2;
                            const double x = b0 * v[0]->x + b1 * v[1]->x + b2 * v[2]->x, y = b0 * v[0]->y + b1 * v[1]->y + b2 * v[2]->y;
                            const double z = b0 * v[0]->z + b1 * v[1]->z + b2 * v[2]->z;
                            if (decks.covers(ox + x, oy + y)) { ++tally.nodata; continue; }
                            if (have_site && std::hypot(ox + x - site_e, oy + y - site_n) > radius) continue;
                            const auto terrain = sample_height_exact(lookup, zone, 0, 0, ox + x, oy + y);
                            if (!terrain) { ++tally.nodata; continue; }
                            tally.add(*terrain - (z - lift));
                            if (worst_n && *terrain - (z - lift) > 0.01 && *terrain - (z - lift) < 1.0 && kind_of(*v[0])[0] == 'b') worst.push_back({*terrain - (z - lift), ox + x, oy + y, z, kind_of(*v[0]), v[0]->lanes, v[0]->flags, v[0]->v});
                        }
                }
            }
        }
        std::printf("== drape %s: tiles %zu vertices %zu triangles %zu build %.0f ms\n", std::string(mode).c_str(), tiles, vertices, triangles, build_ms);
        std::printf("   drape counters: in %zu out %zu uncut %zu dropped %zu node reads %zu\n", d_in, d_out, d_uncut, d_dropped, d_nodes);
        for (const auto& [k, t] : tallies)
            std::printf("   %-7s probes %9zu  terrain>1cm %7zu  >10cm %6zu  ribbon>1cm above terrain %7zu  max terrain-above %.3f m  max ribbon-above %.3f m  nodata %zu\n",
                        k.c_str(), t.probes, t.above1, t.above10, t.below1, t.max_above, -t.max_below, t.nodata);
        std::sort(worst.begin(), worst.end(), [](const Worst& a, const Worst& b) { return a.d > b.d; });
        for (std::size_t i = 0; i < std::min(worst_n, worst.size()); ++i)
            std::printf("   worst %zu: %.3f m at E %.2f N %.2f z %.3f %s lanes %g flags %g v %g\n", i, worst[i].d, worst[i].e, worst[i].n, worst[i].z,
                        worst[i].kind, double(worst[i].lanes), double(worst[i].flags), double(worst[i].v));
    }
    return 0;
}
