// tools/rail_census/main.cpp - g2m_rail_census: read-only census of the railways in an immutable source store (g2m.src.osm
// tiles), the numbers behind the g2m.rail.geom layer decisions (docs/formats/rail_geom.md).
//
//   g2m_rail_census STORE SCOPE ZONE EMIN EMAX NMIN NMAX [CSV] [--spot E N] [--baked DERIVED_DIR RELEASE_ID_HEX]
//
// Metres in the UTM zone (northern hemisphere), e.g. 32 455000 490000 5540000 5585000. Every level-2 src.osm tile
// that intersects the box is read once (complete ways, the same way in two tiles is one way). A way counts when its
// first vertex lies inside the box. Prints, per railway class (tram is counted although the layer does not serve it in
// v1): ways, kilometres, bridges, tunnels, embankments, cuttings, construction ways; the number of bridge STRUCTURES
// (bridge ways connected through shared end nodes are one structure); the pieces the layer would cut; and the plan
// crossings of railway segments with road segments that do not share a node, by structure of both sides. "Bridge over
// road" = a railway bridge (not a tunnel) crossing a road way that is neither a bridge nor a tunnel, or a lower layer.
// CSV: one row per rail-bridge-over-road crossing. --spot: the three nearest of them to a point. --baked: also reads the
// served g2m.rail.geom tiles of a bake (derived store, scope = release id) and prints their totals.
#include "g2m/core/geom/project.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/rail_geom_tile.h"
#include "g2m/layer/src_osm.h"
#include <climits>
#include "g2m/server/tile_store.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <set>
#include <string>
#include <string_view>
#include <vector>

using namespace g2m;

namespace {
struct Road {
    std::int64_t id = 0;
    std::string highway;
    int layer = 0;
    bool bridge = false, tunnel = false;
    std::vector<std::int64_t> refs;
    std::vector<std::pair<double, double>> pts;
};
bool segments_cross(std::pair<double, double> a, std::pair<double, double> b, std::pair<double, double> c, std::pair<double, double> d) {
    const auto cross = [](double ax, double ay, double bx, double by) { return ax * by - ay * bx; };
    const double d1 = cross(b.first - a.first, b.second - a.second, c.first - a.first, c.second - a.second);
    const double d2 = cross(b.first - a.first, b.second - a.second, d.first - a.first, d.second - a.second);
    const double d3 = cross(d.first - c.first, d.second - c.second, a.first - c.first, a.second - c.second);
    const double d4 = cross(d.first - c.first, d.second - c.second, b.first - c.first, b.second - c.second);
    return d1 * d2 < 0 && d3 * d4 < 0;
}
const char* structure(bool bridge, bool tunnel) { return tunnel ? "tunnel" : (bridge ? "bridge" : "ground"); }
}

int main(int argc, char** argv) {
    if (argc < 8) {
        std::cerr << "usage: g2m_rail_census STORE SCOPE ZONE EMIN EMAX NMIN NMAX [CSV] [--spot E N]\n";
        return 1;
    }
    const geo::UtmZone zone{std::atoi(argv[3]), geo::Hemisphere::North};
    const double emin = std::atof(argv[4]), emax = std::atof(argv[5]), nmin = std::atof(argv[6]), nmax = std::atof(argv[7]);
    std::string csv_path, baked_dir, baked_release; bool have_spot = false; double spot_e = 0, spot_n = 0;
    for (int i = 8; i < argc; ++i) {
        if (std::string_view(argv[i]) == "--baked" && i + 2 < argc) { baked_dir = argv[i + 1]; baked_release = argv[i + 2]; i += 2; }
        else if (std::string_view(argv[i]) == "--spot" && i + 2 < argc) { have_spot = true; spot_e = std::atof(argv[i + 1]); spot_n = std::atof(argv[i + 2]); i += 2; }
        else csv_path = argv[i];
    }
    TileStoreConfig config; config.dir = argv[1]; config.read_only = true;
    auto store = TileStore::open(config);
    if (!store) { std::cerr << store.error().message << '\n'; return 1; }
    const auto k0 = tile_key_at(zone, 2, emin, nmin), k1 = tile_key_at(zone, 2, emax, nmax);
    if (!k0 || !k1) { std::cerr << "bad box\n"; return 1; }

    if (!baked_dir.empty()) {
        // The served tiles of a bake (scope = the release id hex): pieces, declined, structure pieces, height range.
        TileStoreConfig dconfig; dconfig.dir = baked_dir; dconfig.read_only = true;
        auto derived = TileStore::open(dconfig);
        if (!derived) { std::cerr << derived.error().message << '\n'; return 1; }
        std::size_t served_tiles = 0, empty_tiles = 0, served_pieces = 0, declined = 0, bridges = 0, tunnels = 0, bad = 0;
        std::int32_t zmin = INT32_MAX, zmax = INT32_MIN; std::size_t bytes = 0, max_bytes = 0;
        for (std::int32_t ty = k0->y; ty <= k1->y; ++ty) for (std::int32_t tx = k0->x; tx <= k1->x; ++tx) {
            auto key = *k0; key.x = tx; key.y = ty;
            auto tile = (*derived)->get({baked_release, std::string(kRailGeomLayer), key});
            if (!tile || !*tile) continue;
            ++served_tiles; bytes += (*tile)->body.size(); max_bytes = std::max(max_bytes, (*tile)->body.size());
            auto body = decode_body((*tile)->body); if (!body) { ++bad; continue; }
            auto rail = decode_rail_geom(*body, key); if (!rail) { ++bad; continue; }
            if (rail->pieces.empty()) ++empty_tiles;
            for (const auto& p : rail->pieces) {
                ++served_pieces;
                if (p.flags & kRailDeclined) ++declined;
                if (p.flags & kRailBridge) ++bridges;
                if (p.flags & kRailTunnel) ++tunnels;
                if (!(p.flags & kRailDeclined)) for (const auto z : p.z) { zmin = std::min(zmin, z); zmax = std::max(zmax, z); }
            }
        }
        std::cout << "baked tiles in the box: " << served_tiles << " (empty " << empty_tiles << ", undecodable " << bad << "), body bytes " << bytes << " (max " << max_bytes
                  << "), pieces " << served_pieces << " (declined " << declined << ", bridge " << bridges << ", tunnel " << tunnels << "), z range " << zmin / 256.0 << ".." << zmax / 256.0 << " m\n";
    }
    // Census rules: every class on (tram included), construction on; everything else as shipped.
    RailRules census = builtin_rail_rules_de();
    for (auto& c : census.classes) c.enabled = true;
    census.construction_enabled = true;
    const geo::Utm utm(geo::kGRS80);

    std::map<std::int64_t, RailWay> rails;
    std::map<std::int64_t, Road> roads;
    std::size_t tiles = 0;
    static const std::set<std::string> kRoads{"motorway", "motorway_link", "trunk", "trunk_link", "primary", "primary_link", "secondary", "secondary_link",
        "tertiary", "tertiary_link", "unclassified", "residential", "living_street", "service", "track"};
    for (std::int32_t ty = k0->y; ty <= k1->y; ++ty) for (std::int32_t tx = k0->x; tx <= k1->x; ++tx) {
        auto key = *k0; key.x = tx; key.y = ty;
        auto tile = (*store)->get({argv[2], std::string(kSrcOsmLayer), key});
        if (!tile || !*tile) continue;
        auto body = decode_body((*tile)->body); if (!body) continue;
        auto osm_tile = decode_src_osm(*body); if (!osm_tile) continue;
        ++tiles;
        const osm::OsmData& d = osm_tile->data;
        for (const auto& w : d.ways) {
            const auto attrs = resolve_rail_attrs(d, w, census);
            if (attrs && !rails.count(w.id)) {
                RailWay out; out.id = w.id; out.attrs = *attrs;
                bool ok = true;
                for (const auto ref : d.refs_of(w)) {
                    if (!out.refs.empty() && out.refs.back() == ref) continue;
                    const auto* node = d.find_node(ref);
                    const auto p = node ? geom::project_e7_to_mm(utm, zone, node->lat, node->lon) : std::nullopt;
                    if (!p) { ok = false; break; }
                    if (!out.pts.empty() && out.pts.back() == *p) continue;
                    out.refs.push_back(ref); out.pts.push_back(*p);
                }
                if (ok && out.refs.size() >= 2) rails.emplace(out.id, std::move(out));
                continue;
            }
            if (roads.count(w.id)) continue;
            const auto tags = d.tags_of(w);
            const auto highway = d.tag(tags, "highway");
            if (!highway || !kRoads.count(std::string(*highway))) continue;
            Road r; r.id = w.id; r.highway = std::string(*highway);
            const auto bridge = d.tag(tags, "bridge"), tunnel = d.tag(tags, "tunnel"), layer = d.tag(tags, "layer");
            r.bridge = bridge && *bridge != "no"; r.tunnel = tunnel && *tunnel != "no" && *tunnel != "culvert";
            r.layer = layer ? std::atoi(std::string(*layer).c_str()) : 0;
            bool ok = true;
            for (const auto ref : d.refs_of(w)) {
                const auto* node = d.find_node(ref);
                const auto p = node ? geom::project_e7_to_mm(utm, zone, node->lat, node->lon) : std::nullopt;
                if (!p) { ok = false; break; }
                r.refs.push_back(ref); r.pts.push_back({static_cast<double>(p->x) / 1000.0, static_cast<double>(p->y) / 1000.0});
            }
            if (ok && r.refs.size() >= 2) roads.emplace(r.id, std::move(r));
        }
    }
    // Ways whose first vertex is inside the box.
    const auto inside = [&](const RailWay& w) {
        const double e = static_cast<double>(w.pts.front().x) / 1000.0, n = static_cast<double>(w.pts.front().y) / 1000.0;
        return e >= emin && e < emax && n >= nmin && n < nmax;
    };
    std::cout << "rail census: zone " << zone.zone << " E " << emin << ".." << emax << " N " << nmin << ".." << nmax << ", src.osm tiles read " << tiles
              << ", railway ways " << rails.size() << ", road ways " << roads.size() << '\n';

    struct Row { std::size_t ways = 0, bridges = 0, tunnels = 0, embankments = 0, cuttings = 0, construction = 0, electrified = 0; double km = 0, bridge_km = 0, tunnel_km = 0; };
    std::map<std::string, Row> by_class;
    Row all;
    std::size_t pieces = 0, pieces_served = 0;
    std::vector<const RailWay*> bridge_ways;
    const auto length_m = [](const RailWay& w) {
        double s = 0;
        for (std::size_t i = 1; i < w.pts.size(); ++i) s += std::hypot(static_cast<double>(w.pts[i].x - w.pts[i - 1].x), static_cast<double>(w.pts[i].y - w.pts[i - 1].y)) / 1000.0;
        return s;
    };
    for (const auto& [id, w] : rails) {
        if (!inside(w)) continue;
        const auto* cls = census.find_class_id(w.attrs.cls);
        for (Row* row : {&by_class[cls->railway], &all}) {
            const double len = length_m(w) / 1000.0;
            ++row->ways; row->km += len;
            if (w.attrs.flags & kRailBridge) { ++row->bridges; row->bridge_km += len; }
            if (w.attrs.flags & kRailTunnel) { ++row->tunnels; row->tunnel_km += len; }
            if (w.attrs.flags & kRailEmbankment) ++row->embankments;
            if (w.attrs.flags & kRailCutting) ++row->cuttings;
            if (w.attrs.flags & kRailElectrified) ++row->electrified;
            if (w.attrs.condition == RailCondition::Construction) ++row->construction;
        }
        const auto plan = plan_way_pieces(w, builtin_rail_rules_de());
        pieces += plan.size();
        if (builtin_rail_rules_de().find_class_id(w.attrs.cls)->enabled) pieces_served += plan.size();
        if (w.attrs.flags & kRailBridge) bridge_ways.push_back(&w);
    }
    std::cout << "class         ways      km  bridges  bridge_km  tunnels  tunnel_km  embank  cutting  constr  electrified\n";
    const auto print = [](const std::string& name, const Row& r) {
        char line[200];
        std::snprintf(line, sizeof line, "%-12s %6zu %7.1f %8zu %10.2f %8zu %10.2f %7zu %8zu %7zu %12zu\n", name.c_str(), r.ways, r.km, r.bridges, r.bridge_km, r.tunnels, r.tunnel_km, r.embankments, r.cuttings, r.construction, r.electrified);
        std::cout << line;
    };
    for (const auto& [name, row] : by_class) print(name, row);
    print("all", all);
    std::cout << "pieces cut (max " << builtin_rail_rules_de().piece_max_length_m << " m): all classes " << pieces << ", served classes " << pieces_served << '\n';

    // Bridge structures: bridge ways connected through shared end nodes of any bridge way.
    {
        std::map<std::int64_t, std::size_t> parent_of_way; std::vector<std::size_t> parent;
        std::map<std::int64_t, std::size_t> first_at_node;
        for (std::size_t i = 0; i < bridge_ways.size(); ++i) parent.push_back(i);
        const auto find = [&](std::size_t x) { while (parent[x] != x) x = parent[x] = parent[parent[x]]; return x; };
        for (std::size_t i = 0; i < bridge_ways.size(); ++i) for (const auto n : {bridge_ways[i]->refs.front(), bridge_ways[i]->refs.back()}) {
            const auto it = first_at_node.find(n);
            if (it == first_at_node.end()) first_at_node[n] = i; else parent[find(i)] = find(it->second);
        }
        std::set<std::size_t> roots; for (std::size_t i = 0; i < bridge_ways.size(); ++i) roots.insert(find(i));
        std::cout << "rail bridge ways " << bridge_ways.size() << ", bridge structures (connected through end nodes) " << roots.size() << '\n';
    }

    // Plan crossings rail x road without a shared node.
    std::map<std::pair<long, long>, std::vector<std::pair<const Road*, std::size_t>>> grid;
    const auto cell = [](double v) { return static_cast<long>(std::floor(v / 64.0)); };
    for (const auto& [id, r] : roads)
        for (std::size_t i = 0; i + 1 < r.pts.size(); ++i)
            for (long cx = cell(std::min(r.pts[i].first, r.pts[i + 1].first)); cx <= cell(std::max(r.pts[i].first, r.pts[i + 1].first)); ++cx)
                for (long cy = cell(std::min(r.pts[i].second, r.pts[i + 1].second)); cy <= cell(std::max(r.pts[i].second, r.pts[i + 1].second)); ++cy) grid[{cx, cy}].push_back({&r, i});
    struct Crossing { double e, n; std::int64_t rail, road; std::string rail_class, highway; int rail_layer, road_layer; bool over; };
    std::vector<Crossing> crossings;
    std::map<std::string, std::size_t> counts;
    std::set<std::pair<std::int64_t, std::int64_t>> seen;
    std::set<std::int64_t> bridges_over, bridge_ways_over;
    for (const auto& [id, w] : rails) {
        if (!inside(w)) continue;
        const bool served = builtin_rail_rules_de().find_class_id(w.attrs.cls)->enabled;
        for (std::size_t i = 0; i + 1 < w.pts.size(); ++i) {
            const std::pair<double, double> a{static_cast<double>(w.pts[i].x) / 1000.0, static_cast<double>(w.pts[i].y) / 1000.0},
                b{static_cast<double>(w.pts[i + 1].x) / 1000.0, static_cast<double>(w.pts[i + 1].y) / 1000.0};
            for (long cx = cell(std::min(a.first, b.first)); cx <= cell(std::max(a.first, b.first)); ++cx)
                for (long cy = cell(std::min(a.second, b.second)); cy <= cell(std::max(a.second, b.second)); ++cy) {
                    const auto it = grid.find({cx, cy}); if (it == grid.end()) continue;
                    for (const auto& [road, j] : it->second) {
                        if (seen.count({w.id, road->id})) continue;
                        if (!segments_cross(a, b, road->pts[j], road->pts[j + 1])) continue;
                        if (std::find(w.refs.begin(), w.refs.end(), road->refs[j]) != w.refs.end() || std::find(w.refs.begin(), w.refs.end(), road->refs[j + 1]) != w.refs.end()) continue;
                        seen.insert({w.id, road->id});
                        const bool rb = (w.attrs.flags & kRailBridge) != 0, rt = (w.attrs.flags & kRailTunnel) != 0;
                        counts[std::string("rail ") + structure(rb, rt) + " x road " + structure(road->bridge, road->tunnel)] += 1;
                        const bool over = rb && !road->bridge && w.attrs.layer > road->layer;
                        const bool over_same = rb && !road->bridge && !road->tunnel && w.attrs.layer <= road->layer;   // tagged bridge, layer not above the road
                        if (over) counts["  rail bridge, layer above the road (bridge over road)"] += 1;
                        if (over_same) counts["  rail bridge, road not a structure, layer not above (layer tag missing or equal)"] += 1;
                        if ((over || over_same) && served) { bridges_over.insert(w.id); }
                        if (over || over_same) bridge_ways_over.insert(w.id);
                        const double rx = b.first - a.first, ry = b.second - a.second, sx = road->pts[j + 1].first - road->pts[j].first, sy = road->pts[j + 1].second - road->pts[j].second;
                        const double t = ((road->pts[j].first - a.first) * sy - (road->pts[j].second - a.second) * sx) / (rx * sy - ry * sx);
                        if (over || over_same) crossings.push_back({a.first + t * rx, a.second + t * ry, w.id, road->id, census.find_class_id(w.attrs.cls)->railway + (w.attrs.condition == RailCondition::Construction ? " under construction" : ""), road->highway, w.attrs.layer, road->layer, over});
                    }
                }
        }
    }
    std::cout << "plan crossings rail x road (no shared node): " << seen.size() << '\n';
    for (const auto& [k, v] : counts) std::cout << "  " << v << "  " << k << '\n';
    std::cout << "rail bridge ways over a ground road: " << bridge_ways_over.size() << " (served classes: " << bridges_over.size() << "), crossings listed: " << crossings.size() << '\n';
    if (!csv_path.empty()) {
        std::ofstream csv(csv_path);
        csv << "east,north,rail_way,road_way,rail_class,highway,rail_layer,road_layer,layer_above\n";
        for (const auto& c : crossings) csv << c.e << ',' << c.n << ',' << c.rail << ',' << c.road << ',' << c.rail_class << ',' << c.highway << ',' << c.rail_layer << ',' << c.road_layer << ',' << (c.over ? 1 : 0) << '\n';
    }
    if (have_spot) {
        std::sort(crossings.begin(), crossings.end(), [&](const Crossing& x, const Crossing& y) { return std::hypot(x.e - spot_e, x.n - spot_n) < std::hypot(y.e - spot_e, y.n - spot_n); });
        for (std::size_t i = 0; i < std::min<std::size_t>(3, crossings.size()); ++i) {
            const auto& c = crossings[i];
            std::cout << "nearest " << i + 1 << ": " << std::hypot(c.e - spot_e, c.n - spot_n) << " m, rail way " << c.rail << " (" << c.rail_class << ", layer " << c.rail_layer << ") over road way " << c.road << " (" << c.highway << ", layer " << c.road_layer << ") at E " << c.e << " N " << c.n << '\n';
        }
    }
    return 0;
}
