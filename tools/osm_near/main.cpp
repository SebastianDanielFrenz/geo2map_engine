// tools/osm_near/main.cpp - g2m_osm_near: read-only listing of the OSM ways (with tags) near a UTM point
// of an immutable source store (g2m.src.osm tiles). Used to inspect grade separations by hand.
//
//   g2m_osm_near STORE SCOPE ZONE EASTING NORTHING RADIUS_M [--nodes]
//   g2m_osm_near STORE SCOPE ZONE EASTING NORTHING HALF_M --rail-census [CSV]
//
// --rail-census: every level-2 src.osm tile within HALF_M (a box) is read; a railway way and a road (highway
// whitelist) whose segments cross in plan without sharing a node are one crossing, counted by the structure tags on
// each side (bridge / tunnel / ground) and listed in the CSV (easting, northing, ways, structures, layers, classes).
// Rail ways are not part of roads.graph, so this reads the OSM source tiles directly.
//
// Prints every way with at least one node segment within RADIUS_M of the point: id, tags, number of
// nodes, closest distance, node ids at the closest segment, and the UTM coordinates of the segment
// when --nodes is given. ZONE is the UTM zone number (northern hemisphere), e.g. 32.
#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/osm_data.h"
#include "g2m/layer/src_osm.h"
#include "g2m/server/tile_store.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>

using namespace g2m;

namespace {
struct CensusWay { std::int64_t id = 0; std::string highway, railway, bridge, tunnel, layer; std::vector<std::int64_t> refs; };
const char* structure_of(const CensusWay& w) {
    if (!w.tunnel.empty() && w.tunnel != "no") return "tunnel";
    if (!w.bridge.empty() && w.bridge != "no") return "bridge";
    return "ground";
}
int layer_of(const CensusWay& w) { return w.layer.empty() ? 0 : std::atoi(w.layer.c_str()); }
bool segments_cross(std::pair<double, double> a, std::pair<double, double> b, std::pair<double, double> c, std::pair<double, double> d) {
    const auto cross = [](double ax, double ay, double bx, double by) { return ax * by - ay * bx; };
    const double d1 = cross(b.first - a.first, b.second - a.second, c.first - a.first, c.second - a.second);
    const double d2 = cross(b.first - a.first, b.second - a.second, d.first - a.first, d.second - a.second);
    const double d3 = cross(d.first - c.first, d.second - c.second, a.first - c.first, a.second - c.second);
    const double d4 = cross(d.first - c.first, d.second - c.second, b.first - c.first, b.second - c.second);
    return d1 * d2 < 0 && d3 * d4 < 0;
}
int rail_census(const char* store_dir, const char* scope, int zone_number, double e0, double n0, double half, const char* csv_path) {
    const geo::UtmZone zone{zone_number, geo::Hemisphere::North};
    TileStoreConfig config; config.dir = store_dir; config.read_only = true;
    auto store = TileStore::open(config);
    if (!store) { std::cerr << store.error().message << '\n'; return 1; }
    const geo::Utm utm;
    const auto k0 = tile_key_at(zone, 2, e0 - half, n0 - half), k1 = tile_key_at(zone, 2, e0 + half, n0 + half);
    if (!k0 || !k1) return 1;
    std::map<std::int64_t, std::pair<double, double>> xy;
    std::map<std::int64_t, CensusWay> ways;
    std::size_t tiles = 0;
    for (std::int32_t ty = k0->y; ty <= k1->y; ++ty) for (std::int32_t tx = k0->x; tx <= k1->x; ++tx) {
        auto key = *k0; key.x = tx; key.y = ty;
        auto tile = (*store)->get({scope, std::string(kSrcOsmLayer), key});
        if (!tile || !*tile) continue;
        auto body = decode_body((*tile)->body); if (!body) continue;
        auto osm_tile = decode_src_osm(*body); if (!osm_tile) continue;
        ++tiles;
        const auto& d = osm_tile->data;
        for (const auto& n : d.nodes) if (!xy.count(n.id)) { const auto f = utm.forward(zone, n.lat * 1e-7, n.lon * 1e-7); xy[n.id] = {f.easting, f.northing}; }
        for (const auto& w : d.ways) {
            if (ways.count(w.id)) continue;
            CensusWay c; c.id = w.id;
            for (const auto& t : d.tags_of(w)) {
                const auto k = d.str(t.key); const std::string v(d.str(t.value));
                if (k == "highway") c.highway = v; else if (k == "railway") c.railway = v; else if (k == "bridge") c.bridge = v;
                else if (k == "tunnel") c.tunnel = v; else if (k == "layer") c.layer = v;
            }
            static const std::set<std::string> rails{"rail", "light_rail", "subway", "tram", "narrow_gauge", "preserved", "monorail", "funicular"};
            static const std::set<std::string> roads{"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary", "secondary_link",
                "tertiary", "tertiary_link", "unclassified", "residential", "living_street", "service", "track"};
            if (!rails.count(c.railway) && !roads.count(c.highway)) continue;
            const auto refs = d.refs_of(w);
            c.refs.assign(refs.begin(), refs.end());
            ways.emplace(w.id, std::move(c));
        }
    }
    // Bucket the road segments (64 m cells), then test every rail segment against the roads of its cells.
    std::map<std::pair<long, long>, std::vector<std::pair<const CensusWay*, std::size_t>>> grid;
    const auto cell = [](double v) { return static_cast<long>(std::floor(v / 64.0)); };
    for (const auto& [id, w] : ways) {
        if (w.highway.empty()) continue;
        for (std::size_t i = 0; i + 1 < w.refs.size(); ++i) {
            if (!xy.count(w.refs[i]) || !xy.count(w.refs[i + 1])) continue;
            const auto a = xy[w.refs[i]], b = xy[w.refs[i + 1]];
            for (long cx = cell(std::min(a.first, b.first)); cx <= cell(std::max(a.first, b.first)); ++cx)
                for (long cy = cell(std::min(a.second, b.second)); cy <= cell(std::max(a.second, b.second)); ++cy) grid[{cx, cy}].push_back({&w, i});
        }
    }
    std::ofstream csv; if (csv_path && *csv_path) csv.open(csv_path);
    if (csv) csv << "east,north,rail_way,road_way,rail,road,rail_layer,road_layer,highway,railway\n";
    std::map<std::string, std::size_t> counts;
    std::set<std::pair<std::int64_t, std::int64_t>> seen;
    for (const auto& [id, rw] : ways) {
        if (rw.railway.empty()) continue;
        for (std::size_t i = 0; i + 1 < rw.refs.size(); ++i) {
            if (!xy.count(rw.refs[i]) || !xy.count(rw.refs[i + 1])) continue;
            const auto a = xy[rw.refs[i]], b = xy[rw.refs[i + 1]];
            for (long cx = cell(std::min(a.first, b.first)); cx <= cell(std::max(a.first, b.first)); ++cx)
                for (long cy = cell(std::min(a.second, b.second)); cy <= cell(std::max(a.second, b.second)); ++cy) {
                    const auto it = grid.find({cx, cy}); if (it == grid.end()) continue;
                    for (const auto& [hw, j] : it->second) {
                        if (seen.count({rw.id, hw->id})) continue;
                        const auto c = xy[hw->refs[j]], dd = xy[hw->refs[j + 1]];
                        if (!segments_cross(a, b, c, dd)) continue;
                        if (std::find(rw.refs.begin(), rw.refs.end(), hw->refs[j]) != rw.refs.end() ||
                            std::find(rw.refs.begin(), rw.refs.end(), hw->refs[j + 1]) != rw.refs.end()) continue;
                        seen.insert({rw.id, hw->id});
                        ++counts[std::string("rail ") + structure_of(rw) + " x road " + structure_of(*hw)];
                        const double rx = b.first - a.first, ry = b.second - a.second, sx = dd.first - c.first, sy = dd.second - c.second;
                        const double t = ((c.first - a.first) * sy - (c.second - a.second) * sx) / (rx * sy - ry * sx);
                        if (csv) csv << a.first + t * rx << ',' << a.second + t * ry << ',' << rw.id << ',' << hw->id << ',' << structure_of(rw) << ',' << structure_of(*hw)
                                     << ',' << layer_of(rw) << ',' << layer_of(*hw) << ',' << hw->highway << ',' << rw.railway << '\n';
                    }
                }
        }
    }
    std::cout << "rail census: tiles " << tiles << " ways " << ways.size() << " crossings " << seen.size() << '\n';
    for (const auto& [k, v] : counts) std::cout << "  " << v << "  " << k << '\n';
    return 0;
}
} // namespace

int main(int argc, char** argv) {
    if (argc >= 8 && std::string_view(argv[7]) == "--rail-census")
        return rail_census(argv[1], argv[2], std::atoi(argv[3]), std::atof(argv[4]), std::atof(argv[5]), std::atof(argv[6]), argc > 8 ? argv[8] : "");
    if (argc < 7) {
        std::cerr << "usage: g2m_osm_near STORE SCOPE ZONE EASTING NORTHING RADIUS_M [--nodes]\n";
        return 1;
    }
    const bool show_nodes = argc > 7 && std::string_view(argv[7]) == "--nodes";
    const int zone_number = std::atoi(argv[3]);
    const double e0 = std::atof(argv[4]), n0 = std::atof(argv[5]), radius = std::atof(argv[6]);
    const geo::UtmZone zone{zone_number, geo::Hemisphere::North};
    TileStoreConfig config; config.dir = argv[1]; config.read_only = true;
    auto store = TileStore::open(config);
    if (!store) { std::cerr << store.error().message << '\n'; return 1; }
    const geo::Utm utm;
    const auto centre = tile_key_at(zone, 2, e0, n0);
    if (!centre) { std::cerr << "bad point\n"; return 1; }
    struct Hit { double dist; std::string text; };
    std::map<std::int64_t, Hit> hits;
    for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
        auto key = *centre; key.x += dx; key.y += dy;
        auto tile = (*store)->get({argv[2], std::string(kSrcOsmLayer), key});
        if (!tile) { std::cerr << tile.error().message << '\n'; return 1; }
        if (!*tile) continue;
        auto body = decode_body((*tile)->body); if (!body) { std::cerr << body.error().message << '\n'; return 1; }
        auto osm_tile = decode_src_osm(*body); if (!osm_tile) { std::cerr << osm_tile.error().message << '\n'; return 1; }
        const auto& d = osm_tile->data;
        std::map<std::int64_t, std::pair<double, double>> xy;
        for (const auto& n : d.nodes) {
            const auto f = utm.forward(zone, n.lat * 1e-7, n.lon * 1e-7);
            xy[n.id] = {f.easting, f.northing};
        }
        for (const auto& w : d.ways) {
            const auto refs = d.refs_of(w);
            double best = 1e18; std::size_t best_i = 0;
            for (std::size_t i = 0; i + 1 < refs.size(); ++i) {
                const auto a = xy[refs[i]], b = xy[refs[i + 1]];
                const double vx = b.first - a.first, vy = b.second - a.second;
                const double l2 = vx * vx + vy * vy;
                double t = l2 > 0 ? ((e0 - a.first) * vx + (n0 - a.second) * vy) / l2 : 0;
                t = std::clamp(t, 0.0, 1.0);
                const double px = a.first + t * vx - e0, py = a.second + t * vy - n0;
                const double dist = std::sqrt(px * px + py * py);
                if (dist < best) { best = dist; best_i = i; }
            }
            if (best > radius || hits.count(w.id)) continue;
            std::string text = "way " + std::to_string(w.id) + "  dist=" + std::to_string(best) + " m  nodes=" +
                               std::to_string(refs.size()) + "\n   tags:";
            for (const auto& t : d.tags_of(w)) text += " " + std::string(d.str(t.key)) + "=" + std::string(d.str(t.value)) + ";";
            text += "\n   closest segment nodes " + std::to_string(refs[best_i]) + " -> " + std::to_string(refs[best_i + 1]) + "\n";
            if (show_nodes) {
                for (std::size_t i = 0; i < refs.size(); ++i) {
                    const auto p = xy[refs[i]];
                    if (std::hypot(p.first - e0, p.second - n0) > radius * 3) continue;
                    text += "   node " + std::to_string(refs[i]) + " E " + std::to_string(p.first) + " N " + std::to_string(p.second);
                    if (const auto* nd = d.find_node(refs[i]); nd && nd->tags_count) {
                        text += "  tags:";
                        for (const auto& t : d.tags_of(*nd)) text += " " + std::string(d.str(t.key)) + "=" + std::string(d.str(t.value)) + ";";
                    }
                    text += "\n";
                }
            }
            hits[w.id] = {best, text};
        }
    }
    std::vector<const Hit*> sorted;
    for (const auto& [id, h] : hits) sorted.push_back(&h);
    std::sort(sorted.begin(), sorted.end(), [](auto* a, auto* b) { return a->dist < b->dist; });
    for (const auto* h : sorted) std::cout << h->text;
    std::cout << sorted.size() << " ways within " << radius << " m\n";
}
