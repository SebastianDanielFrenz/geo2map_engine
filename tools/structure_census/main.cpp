// tools/structure_census/main.cpp - g2m_structure_census: read-only census of the tunnel stretches and the roads
// under bridges of an immutable source store, the numbers behind the structure classification of terrain migration
// phase 7a (docs/structure_classification.md).
//
//   g2m_structure_census STORE SCOPE ZONE EMIN EMAX NMIN NMAX [--csv CHAINS.csv] [--runs RUNS.csv]
//                        [--under UNDER.csv] [--dump WAY_ID FILE.csv] [--spot E N]
//
// Reads every level-2 g2m.src.osm tile intersecting the box once (complete ways) and the g2m.src.elev.dgm1 tiles the
// samples need (raw DGM1, bare earth, 1 m). Never writes the store.
//
// Tunnel chains: OSM tunnel ways (tunnel other than no/culvert) of the 15 road classes of roads.graph, and of the
// railway classes (all four, construction included), joined end to end like g2m.rail.geom's chain walk (a node where
// exactly one other way of the same kind touches, it is a tunnel and it ends there). A chain counts when its first
// vertex lies in the box. Per chain, every 1 m station: the floor is the straight chord between the DEM at the two
// chain ends (the portal anchors); the cover is the DEM minus that floor, the minimum of the centreline and both tube
// edges (half width from the width/lanes tags or the class default); "above" is the strongest of rail bridge, road
// bridge, building, ground road, ground rail, higher tunnel whose footprint covers the station (ways that share a node
// with the chain never count). The chain is then classified (structure_classify.h) at the nominal thresholds and at
// 0.5x / 1.5x of the cover threshold and of the minimum lengths.
//
// Roads under bridges: every plan crossing of a ground road (no bridge, no tunnel) with a bridge way (rail or road) on a
// higher layer that shares no node with it. At the crossing, the DEM along the lower road is compared with the chord of
// the DEM 30 m before and after: the hump is the largest excess within the bridge footprint (DGM1 interpolates over the
// removed deck; D64 masks it for road bridges only).
#include "g2m/layer/structure_classify.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/geom/project.h"
#include "g2m/core/raster.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/rail_geom_tile.h"
#include "g2m/layer/rail_rules.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/src_osm.h"
#include "g2m/server/tile_store.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <string_view>
#include <vector>

using namespace g2m;
namespace st = g2m::structure;

namespace {
using P = std::pair<double, double>;

enum class Kind : std::uint8_t { Road, Rail, Building };

struct Way {
    std::int64_t id = 0;
    Kind kind = Kind::Road;
    std::string cls;            // highway or railway class
    std::string tunnel_value;   // the tunnel tag value ("" = none)
    std::string name;           // name tag (tunnel:name preferred), for the named-site list only
    bool bridge = false, tunnel = false, embankment = false, cutting = false, construction = false;
    int layer = 0;
    double half_width_m = 3.0;
    std::vector<std::int64_t> refs;
    std::vector<P> pts;
};

struct RoadClass { const char* highway; double width_m; };
constexpr RoadClass kRoadClasses[] = {
    {"track", 3.0}, {"service", 4.0}, {"living_street", 5.0}, {"residential", 5.5}, {"unclassified", 5.5},
    {"tertiary_link", 5.0}, {"tertiary", 6.0}, {"secondary_link", 5.0}, {"secondary", 6.5}, {"primary_link", 5.5},
    {"primary", 7.0}, {"trunk_link", 6.0}, {"trunk", 7.5}, {"motorway_link", 6.0}, {"motorway", 12.0}};

const RoadClass* road_class(std::string_view h) {
    for (const auto& c : kRoadClasses) if (h == c.highway) return &c;
    return nullptr;
}

double clean_number(std::string_view v) {
    const std::string s(v);
    char* end = nullptr;
    const double d = std::strtod(s.c_str(), &end);
    return (end == s.c_str() || *end != '\0') ? -1.0 : d;
}

// ---- DGM1 sampler (raw source tiles, bilinear over the sample centres, LRU of 256 tiles) ----------------------
struct Dem {
    TileStore& store;
    std::string scope;
    geo::UtmZone zone;
    struct Entry { std::unique_ptr<ElevTile> tile; std::uint64_t used = 0; };
    std::map<std::pair<std::int32_t, std::int32_t>, Entry> cache;
    std::uint64_t clock = 0;
    std::size_t loads = 0;

    const ElevTile* tile_at(const TileKey& key) {
        const std::pair<std::int32_t, std::int32_t> k{key.x, key.y};
        auto it = cache.find(k);
        if (it == cache.end()) {
            if (cache.size() >= 256) {
                auto oldest = cache.begin();
                for (auto j = cache.begin(); j != cache.end(); ++j) if (j->second.used < oldest->second.used) oldest = j;
                cache.erase(oldest);
            }
            Entry e;
            auto t = store.get({scope, std::string(kSrcElevDgm1Layer), key});
            if (t && *t) {
                auto body = decode_body((*t)->body);
                if (body) { auto el = decode_src_elev(*body); if (el) e.tile = std::make_unique<ElevTile>(std::move(*el)); }
            }
            ++loads;
            it = cache.emplace(k, std::move(e)).first;
        }
        it->second.used = ++clock;
        return it->second.tile.get();
    }
    double raw(const ElevTile& t, int c, int r) const {
        c = std::clamp(c, 0, t.heights.width() - 1);
        r = std::clamp(r, 0, t.heights.height() - 1);
        const std::int32_t v = t.heights.at(c, r);
        return v == kHeightNoData ? NAN : static_cast<double>(v) / 256.0;
    }
    double at(double e, double n) {
        const auto key = tile_key_at(zone, kRawTileLevel, e, n);
        if (!key) return NAN;
        const ElevTile* t = tile_at(*key);
        if (!t) return NAN;
        const auto& g = t->georef;
        const double sp = static_cast<double>(g.spacing_mm) / 1000.0;
        const double half = g.registration == SampleRegistration::Cell ? 0.5 : 0.0;
        const double fx = (e - static_cast<double>(g.west_mm) / 1000.0) / sp - half;
        double fy = (n - static_cast<double>(g.south_mm) / 1000.0) / sp - half;   // rows counted from the south
        const int i = static_cast<int>(std::floor(fx)), j = static_cast<int>(std::floor(fy));
        const double tx = fx - i, ty = fy - j;
        const auto h = [&](int a, int b) {
            const int row = g.row_order == RowOrder::SouthToNorth ? b : (g.height - 1 - b);
            return raw(*t, a, row);
        };
        return (1 - ty) * ((1 - tx) * h(i, j) + tx * h(i + 1, j)) + ty * ((1 - tx) * h(i, j + 1) + tx * h(i + 1, j + 1));
    }
};

// ---- polyline helpers --------------------------------------------------------------------------------------------
struct Line {
    std::vector<P> pts;
    std::vector<double> cum;
    void build() { cum.assign(pts.size(), 0); for (std::size_t i = 1; i < pts.size(); ++i) cum[i] = cum[i - 1] + std::hypot(pts[i].first - pts[i - 1].first, pts[i].second - pts[i - 1].second); }
    double length() const { return cum.empty() ? 0 : cum.back(); }
    // Position and unit tangent at arc length s (clamped).
    void at(double s, P& p, P& d) const {
        s = std::clamp(s, 0.0, length());
        std::size_t k = static_cast<std::size_t>(std::upper_bound(cum.begin(), cum.end(), s) - cum.begin());
        k = std::clamp<std::size_t>(k, 1, pts.size() - 1);
        const double seg = cum[k] - cum[k - 1];
        const double t = seg > 0 ? (s - cum[k - 1]) / seg : 0;
        p = {pts[k - 1].first + (pts[k].first - pts[k - 1].first) * t, pts[k - 1].second + (pts[k].second - pts[k - 1].second) * t};
        d = seg > 0 ? P{(pts[k].first - pts[k - 1].first) / seg, (pts[k].second - pts[k - 1].second) / seg} : P{1, 0};
    }
};

double seg_dist(P p, P a, P b) {
    const double dx = b.first - a.first, dy = b.second - a.second, l2 = dx * dx + dy * dy;
    double t = l2 > 0 ? ((p.first - a.first) * dx + (p.second - a.second) * dy) / l2 : 0;
    t = std::clamp(t, 0.0, 1.0);
    return std::hypot(p.first - (a.first + dx * t), p.second - (a.second + dy * t));
}
bool seg_cross(P a, P b, P c, P d, double* ta) {
    const auto cr = [](double ax, double ay, double bx, double by) { return ax * by - ay * bx; };
    const double r_x = b.first - a.first, r_y = b.second - a.second, s_x = d.first - c.first, s_y = d.second - c.second;
    const double den = cr(r_x, r_y, s_x, s_y);
    if (den == 0) return false;
    const double t = cr(c.first - a.first, c.second - a.second, s_x, s_y) / den;
    const double u = cr(c.first - a.first, c.second - a.second, r_x, r_y) / den;
    if (t <= 0 || t >= 1 || u <= 0 || u >= 1) return false;
    if (ta) *ta = t;
    return true;
}
bool inside_polygon(P p, const std::vector<P>& poly) {
    bool in = false;
    for (std::size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++) {
        const auto& a = poly[i]; const auto& b = poly[j];
        if ((a.second > p.second) != (b.second > p.second) &&
            p.first < (b.first - a.first) * (p.second - a.second) / (b.second - a.second) + a.first) in = !in;
    }
    return in;
}

// ---- spatial grid over segments / building boxes (ordered containers only) ---------------------------------------
constexpr double kCell = 64.0;
struct Grid {
    std::map<std::pair<int, int>, std::vector<std::pair<std::size_t, std::size_t>>> cells;   // (way index, segment index)
    static int c(double v) { return static_cast<int>(std::floor(v / kCell)); }
    void add(std::size_t w, std::size_t s, P a, P b, double pad) {
        for (int y = c(std::min(a.second, b.second) - pad); y <= c(std::max(a.second, b.second) + pad); ++y)
            for (int x = c(std::min(a.first, b.first) - pad); x <= c(std::max(a.first, b.first) + pad); ++x) cells[{x, y}].push_back({w, s});
    }
    const std::vector<std::pair<std::size_t, std::size_t>>* at(P p) const {
        auto it = cells.find({c(p.first), c(p.second)});
        return it == cells.end() ? nullptr : &it->second;
    }
};

double median(std::vector<double> v) { if (v.empty()) return NAN; std::sort(v.begin(), v.end()); return v[v.size() / 2]; }

struct ChainResult {
    std::vector<std::int64_t> ways;
    Kind kind = Kind::Road;
    std::string cls, tunnel_value, name;
    int layer = 0;
    double length = 0, half_width = 0, z0 = 0, z1 = 0;
    P mid{0, 0}, start{0, 0};
    std::vector<double> ground, ground_centre;   // lowest DEM across the tube width / DEM on the axis, per 1 m station
    std::vector<double> cover, natural_cover;    // at the nominal thresholds: ground - design floor, ground - portal chord
    bool portal0 = true, portal1 = true;         // false: that chain end continues into another tunnel way (no portal)
    // Portal join (coordinator acceptance case 2026-10-20): the served tunnel floor takes its end height from one DEM
    // sample AT the portal node; the ground way outside meets it at its own level. Per end: the DEM at the node minus
    // the median DEM 2..8 m out along the joining ground way (NaN: no ground way of the same kind joins there).
    std::array<double, 2> portal_step{NAN, NAN};
    std::array<std::int64_t, 2> portal_node{0, 0}, portal_neighbour{0, 0};
    std::vector<st::Above> above;
    std::array<std::size_t, st::kAboveKinds> above_count{};
    std::map<std::int64_t, st::Above> above_ways;   // way id -> its kind, the ways above this chain
    bool nodata = false;
};
} // namespace

int main(int argc, char** argv) {
    if (argc < 8) {
        std::cerr << "usage: g2m_structure_census STORE SCOPE ZONE EMIN EMAX NMIN NMAX [--csv F] [--runs F] [--under F] [--dump WAY F] [--spot E N]\n";
        return 1;
    }
    const geo::UtmZone zone{std::atoi(argv[3]), geo::Hemisphere::North};
    const double emin = std::atof(argv[4]), emax = std::atof(argv[5]), nmin = std::atof(argv[6]), nmax = std::atof(argv[7]);
    std::string csv_path, runs_path, under_path, dump_path, portals_path;
    std::int64_t dump_way = 0;
    std::vector<P> spots;
    for (int i = 8; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--csv" && i + 1 < argc) csv_path = argv[++i];
        else if (a == "--runs" && i + 1 < argc) runs_path = argv[++i];
        else if (a == "--under" && i + 1 < argc) under_path = argv[++i];
        else if (a == "--dump" && i + 2 < argc) { dump_way = std::atoll(argv[i + 1]); dump_path = argv[i + 2]; i += 2; }
        else if (a == "--portals" && i + 1 < argc) { portals_path = argv[++i]; }
        else if (a == "--spot" && i + 2 < argc) { spots.push_back({std::atof(argv[i + 1]), std::atof(argv[i + 2])}); i += 2; }
        else { std::cerr << "unknown option " << a << '\n'; return 1; }
    }
    TileStoreConfig config; config.dir = argv[1]; config.read_only = true;
    auto store = TileStore::open(config);
    if (!store) { std::cerr << store.error().message << '\n'; return 1; }
    const auto k0 = tile_key_at(zone, kRawTileLevel, emin, nmin), k1 = tile_key_at(zone, kRawTileLevel, emax, nmax);
    if (!k0 || !k1) { std::cerr << "bad box\n"; return 1; }

    RailRules rail_rules = builtin_rail_rules_de();
    for (auto& c : rail_rules.classes) c.enabled = true;
    rail_rules.construction_enabled = true;
    const geo::Utm utm(geo::kGRS80);

    // ---- read OSM ----
    std::map<std::int64_t, Way> by_id;   // roads, rails and buildings (ids are per OSM element type: ways only here)
    std::size_t tiles = 0;
    for (std::int32_t ty = k0->y; ty <= k1->y; ++ty) for (std::int32_t tx = k0->x; tx <= k1->x; ++tx) {
        auto key = *k0; key.x = tx; key.y = ty;
        auto tile = (*store)->get({argv[2], std::string(kSrcOsmLayer), key});
        if (!tile || !*tile) continue;
        auto body = decode_body((*tile)->body); if (!body) continue;
        auto osm_tile = decode_src_osm(*body); if (!osm_tile) continue;
        ++tiles;
        const osm::OsmData& d = osm_tile->data;
        for (const auto& w : d.ways) {
            if (by_id.count(w.id)) continue;
            const auto tags = d.tags_of(w);
            Way out; out.id = w.id;
            if (const auto nm = d.tag(tags, "tunnel:name")) out.name = std::string(*nm);
            else if (const auto nm2 = d.tag(tags, "name")) out.name = std::string(*nm2);
            const auto rail = resolve_rail_attrs(d, w, rail_rules);
            const auto highway = d.tag(tags, "highway");
            const auto building = d.tag(tags, "building");
            const RoadClass* rc = highway ? road_class(*highway) : nullptr;
            if (rail) {
                out.kind = Kind::Rail;
                out.cls = rail_rules.find_class_id(rail->cls)->railway;
                out.bridge = rail->flags & kRailBridge; out.tunnel = rail->flags & kRailTunnel;
                out.embankment = rail->flags & kRailEmbankment; out.cutting = rail->flags & kRailCutting;
                out.construction = rail->condition == RailCondition::Construction;
                out.layer = rail->layer;
                out.half_width_m = 2.5 + 2.25 * (std::max<int>(1, rail->tracks) - 1);
            } else if (rc) {
                out.kind = Kind::Road; out.cls = rc->highway;
                const auto bridge = d.tag(tags, "bridge"), tunnel = d.tag(tags, "tunnel"), layer = d.tag(tags, "layer");
                const auto cutting = d.tag(tags, "cutting"), embankment = d.tag(tags, "embankment");
                out.bridge = bridge && *bridge != "no";
                out.tunnel = tunnel && *tunnel != "no" && *tunnel != "culvert";
                if (out.tunnel) { out.tunnel_value = std::string(*tunnel); out.bridge = false; }
                out.cutting = cutting && *cutting != "no"; out.embankment = embankment && *embankment != "no";
                out.layer = layer ? std::atoi(std::string(*layer).c_str()) : 0;
                double width = rc->width_m;
                if (const auto wt = d.tag(tags, "width")) { const double v = clean_number(*wt); if (v > 1 && v < 60) width = v; }
                else if (const auto ln = d.tag(tags, "lanes")) { const double v = clean_number(*ln); if (v >= 1 && v <= 12) width = std::max(width, v * 3.25); }
                out.half_width_m = width / 2;
            } else if (building && *building != "no") {
                out.kind = Kind::Building; out.cls = std::string(*building);
            } else continue;
            bool ok = true;
            for (const auto ref : d.refs_of(w)) {
                const auto* node = d.find_node(ref);
                const auto p = node ? geom::project_e7_to_mm(utm, zone, node->lat, node->lon) : std::nullopt;
                if (!p) { ok = false; break; }
                if (!out.refs.empty() && out.refs.back() == ref) continue;
                out.refs.push_back(ref); out.pts.push_back({static_cast<double>(p->x) / 1000.0, static_cast<double>(p->y) / 1000.0});
            }
            if (!ok || out.refs.size() < 2) continue;
            if (out.kind == Kind::Building && (out.refs.size() < 4 || out.refs.front() != out.refs.back())) continue;
            by_id.emplace(out.id, std::move(out));
        }
    }
    std::vector<const Way*> ways;
    for (const auto& [id, w] : by_id) ways.push_back(&w);
    Grid lines, buildings;
    std::map<std::int64_t, std::vector<std::size_t>> node_ways;   // node -> way indices (roads and rails)
    std::size_t n_roads = 0, n_rails = 0, n_buildings = 0, road_bridges = 0, road_cuttings = 0, road_tunnels = 0, rail_tunnels = 0;
    for (std::size_t i = 0; i < ways.size(); ++i) {
        const Way& w = *ways[i];
        if (w.kind == Kind::Building) {
            ++n_buildings;
            for (std::size_t s = 0; s + 1 < w.pts.size(); ++s) buildings.add(i, s, w.pts[s], w.pts[s + 1], 0);
            continue;
        }
        (w.kind == Kind::Road ? n_roads : n_rails)++;
        if (w.kind == Kind::Road && w.bridge) ++road_bridges;
        if (w.kind == Kind::Road && w.cutting) ++road_cuttings;
        if (w.tunnel) ++(w.kind == Kind::Road ? road_tunnels : rail_tunnels);
        for (std::size_t s = 0; s + 1 < w.pts.size(); ++s) lines.add(i, s, w.pts[s], w.pts[s + 1], w.half_width_m);
        for (const auto r : w.refs) { auto& v = node_ways[r]; if (v.empty() || v.back() != i) v.push_back(i); }
    }
    std::cout << "structure census: zone " << zone.zone << " E " << emin << ".." << emax << " N " << nmin << ".." << nmax << ", src.osm tiles " << tiles
              << ", road ways " << n_roads << " (bridge " << road_bridges << ", cutting " << road_cuttings << ", tunnel " << road_tunnels << "), railway ways " << n_rails
              << " (tunnel " << rail_tunnels << "), buildings " << n_buildings << '\n';

    Dem dem{**store, argv[2], zone, {}, 0, 0};

    // Nominal thresholds per chain kind (the proposed de.json "structure" block, docs/structure_classification.md).
    const auto nominal_of = [](const ChainResult& c) {
        st::Thresholds t;
        if (c.kind == Kind::Rail) {
            t.clearance_m = (c.cls == "rail") ? 6.0 : 4.5;
            t.max_grade = (c.cls == "rail") ? 0.04 : 0.05;
        }
        return t;
    };
    // Cover and natural cover of chain c at thresholds t (the design floor depends on clearance + roof and the grade).
    const auto profile_at = [](const ChainResult& c, const st::Thresholds& t, std::vector<double>& cover, std::vector<double>& natural) {
        const auto fl = st::design_floor(c.ground, c.length, c.z0, c.z1, c.portal0, c.portal1, t.max_grade, t.cover_needed());
        cover.clear(); natural.clear();
        for (std::size_t s = 0; s < c.ground.size(); ++s) {
            const double ss = std::min(static_cast<double>(s), c.length);
            cover.push_back(c.ground[s] - fl[s]);
            natural.push_back(c.ground[s] - (c.z0 + (c.z1 - c.z0) * ss / c.length));
        }
    };

    // ---- tunnel chains ----
    std::vector<ChainResult> chains;
    std::set<std::size_t> visited;
    const auto same_kind_at = [&](std::int64_t node, Kind k, std::size_t except) {
        std::vector<std::size_t> out;
        auto it = node_ways.find(node);
        if (it != node_ways.end()) for (const auto j : it->second) if (j != except && ways[j]->kind == k) out.push_back(j);
        return out;
    };
    for (std::size_t i = 0; i < ways.size(); ++i) {
        const Way& seed = *ways[i];
        if (!seed.tunnel || seed.kind == Kind::Building || visited.count(i)) continue;
        // Walk the chain both ways. Oriented sequence of (way index, forward?).
        std::vector<std::pair<std::size_t, bool>> seq{{i, true}};
        visited.insert(i);
        for (int side = 0; side < 2; ++side) {
            for (int guard = 0; guard < 64; ++guard) {
                const auto& [wi, fwd] = side == 0 ? seq.back() : seq.front();
                const Way& cw = *ways[wi];
                const std::int64_t end = side == 0 ? (fwd ? cw.refs.back() : cw.refs.front()) : (fwd ? cw.refs.front() : cw.refs.back());
                const auto others = same_kind_at(end, cw.kind, wi);
                if (others.size() != 1) break;
                const std::size_t j = others[0];
                const Way& ow = *ways[j];
                if (!ow.tunnel || visited.count(j) || (ow.refs.front() != end && ow.refs.back() != end)) break;
                visited.insert(j);
                if (side == 0) seq.push_back({j, ow.refs.front() == end});
                else seq.insert(seq.begin(), {j, ow.refs.back() == end});
            }
        }
        ChainResult c;
        c.kind = seed.kind; c.cls = seed.cls; c.tunnel_value = seed.tunnel_value; c.layer = seed.layer;
        {
            // A chain end is a portal unless every other way of its kind at that node is a tunnel (a switch or fork inside
            // the tunnel network: the chain walk stops there, but the floor continues underground).
            const auto is_portal = [&](std::int64_t node, std::size_t own) {
                const auto others = same_kind_at(node, seed.kind, own);
                if (others.empty()) return true;
                for (const auto j : others) if (!ways[j]->tunnel) return true;
                return false;
            };
            const auto& [w0, f0] = seq.front(); const auto& [w1, f1] = seq.back();
            c.portal0 = is_portal(f0 ? ways[w0]->refs.front() : ways[w0]->refs.back(), w0);
            c.portal1 = is_portal(f1 ? ways[w1]->refs.back() : ways[w1]->refs.front(), w1);
            const std::int64_t end_nodes[2] = {f0 ? ways[w0]->refs.front() : ways[w0]->refs.back(), f1 ? ways[w1]->refs.back() : ways[w1]->refs.front()};
            const std::size_t end_ways[2] = {w0, w1};
            for (int e = 0; e < 2; ++e) {
                const std::int64_t node = end_nodes[e];
                c.portal_node[static_cast<std::size_t>(e)] = node;
                double worst = NAN;
                for (const auto j : same_kind_at(node, seed.kind, end_ways[e])) {
                    const Way& g = *ways[j];
                    if (g.tunnel || g.bridge) continue;
                    if (g.refs.front() != node && g.refs.back() != node) continue;   // joins mid-way: skip (rare)
                    Line gl; gl.pts = g.pts; if (g.refs.front() != node) std::reverse(gl.pts.begin(), gl.pts.end()); gl.build();
                    const double z_node = dem.at(gl.pts.front().first, gl.pts.front().second);
                    std::vector<double> zs;
                    for (double s = 2; s <= 8.0 && s <= gl.length(); s += 1) { P p, dd; gl.at(s, p, dd); const double z = dem.at(p.first, p.second); if (!std::isnan(z)) zs.push_back(z); }
                    if (zs.empty() || std::isnan(z_node)) continue;
                    std::sort(zs.begin(), zs.end());
                    const double step = z_node - zs[zs.size() / 2];
                    if (std::isnan(worst) || std::fabs(step) > std::fabs(worst)) { worst = step; c.portal_neighbour[static_cast<std::size_t>(e)] = g.id; }
                }
                c.portal_step[static_cast<std::size_t>(e)] = worst;
            }
        }
        Line line;
        std::set<std::int64_t> chain_nodes;
        for (const auto& [wi, fwd] : seq) {
            const Way& w = *ways[wi];
            c.ways.push_back(w.id);
            if (c.name.empty() && !w.name.empty()) c.name = w.name;
            c.half_width = std::max(c.half_width, w.half_width_m);
            for (const auto r : w.refs) chain_nodes.insert(r);
            const std::size_t n = w.pts.size();
            for (std::size_t k = 0; k < n; ++k) {
                const P p = fwd ? w.pts[k] : w.pts[n - 1 - k];
                if (!line.pts.empty() && line.pts.back() == p) continue;
                line.pts.push_back(p);
            }
        }
        if (line.pts.size() < 2) continue;
        line.build();
        c.length = line.length();
        c.start = line.pts.front();
        if (!(c.start.first >= emin && c.start.first < emax && c.start.second >= nmin && c.start.second < nmax)) continue;
        if (c.length < 1.0) continue;
        P p, d;
        line.at(c.length / 2, p, d); c.mid = p;
        c.z0 = dem.at(line.pts.front().first, line.pts.front().second);
        c.z1 = dem.at(line.pts.back().first, line.pts.back().second);
        if (std::isnan(c.z0) || std::isnan(c.z1)) { c.nodata = true; chains.push_back(std::move(c)); continue; }
        const std::size_t stations = static_cast<std::size_t>(std::ceil(c.length)) + 1;
        for (std::size_t s = 0; s < stations; ++s) {
            const double ss = std::min(static_cast<double>(s), c.length);
            line.at(ss, p, d);
            const P nrm{-d.second, d.first};
            double gmin = 1e18;
            for (const double off : {-c.half_width, 0.0, c.half_width}) {
                const double z = dem.at(p.first + nrm.first * off, p.second + nrm.second * off);
                if (!std::isnan(z)) gmin = std::min(gmin, z);
            }
            if (gmin > 1e17) { c.nodata = true; gmin = NAN; }
            c.ground.push_back(gmin);
            c.ground_centre.push_back(dem.at(p.first, p.second));
            // What lies above this station.
            st::Above best = st::Above::Nothing;
            if (const auto* cand = lines.at(p)) {
                for (const auto& [wj, sj] : *cand) {
                    const Way& o = *ways[wj];
                    bool shares = false;
                    for (const auto r : o.refs) if (chain_nodes.count(r)) { shares = true; break; }
                    if (shares) continue;
                    if (seg_dist(p, o.pts[sj], o.pts[sj + 1]) > o.half_width_m) continue;
                    st::Above a = st::Above::Nothing;
                    if (o.tunnel) { if (o.layer > c.layer) a = st::Above::TunnelAbove; else continue; }
                    else if (o.kind == Kind::Rail) a = o.bridge ? st::Above::RailBridge : st::Above::RailGround;
                    else a = o.bridge ? st::Above::RoadBridge : st::Above::RoadGround;
                    if (best == st::Above::Nothing || static_cast<int>(a) < static_cast<int>(best)) best = a;
                    auto& slot = c.above_ways[o.id];
                    if (slot == st::Above::Nothing || static_cast<int>(a) < static_cast<int>(slot)) slot = a;
                }
            }
            if (const auto* cand = buildings.at(p)) {
                std::set<std::size_t> seen;
                for (const auto& [wj, sj] : *cand) {
                    if (!seen.insert(wj).second) continue;
                    if (inside_polygon(p, ways[wj]->pts)) {
                        if (best == st::Above::Nothing || static_cast<int>(st::Above::Building) < static_cast<int>(best)) best = st::Above::Building;
                        auto& slot = c.above_ways[ways[wj]->id];
                        if (slot == st::Above::Nothing) slot = st::Above::Building;
                    }
                }
            }
            c.above.push_back(best);
            ++c.above_count[static_cast<std::size_t>(best)];
        }
        if (!c.nodata) {
            profile_at(c, nominal_of(c), c.cover, c.natural_cover);
            if (dump_way && std::find(c.ways.begin(), c.ways.end(), dump_way) != c.ways.end() && !dump_path.empty()) {
                const auto t = nominal_of(c);
                const auto fl = st::design_floor(c.ground, c.length, c.z0, c.z1, c.portal0, c.portal1, t.max_grade, t.cover_needed());
                std::ofstream f(dump_path);
                f << "s_m,east,north,chord,design_floor,dem_centre,ground_min,cover,natural_cover,above\n";
                for (std::size_t s = 0; s < c.ground.size(); ++s) {
                    const double ss = std::min(static_cast<double>(s), c.length);
                    line.at(ss, p, d);
                    char buf[256];
                    std::snprintf(buf, sizeof buf, "%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s\n", ss, p.first, p.second, c.z0 + (c.z1 - c.z0) * ss / c.length,
                                  fl[s], c.ground_centre[s], c.ground[s], c.cover[s], c.natural_cover[s], st::to_string(c.above[s]));
                    f << buf;
                }
            }
        }
        chains.push_back(std::move(c));
    }

    // ---- classification ----
    const auto& nominal = nominal_of;
    std::ofstream csv; if (!csv_path.empty()) csv.open(csv_path);
    std::ofstream runs_csv; if (!runs_path.empty()) runs_csv.open(runs_path);
    if (csv) csv << "first_way,ways,kind,class,tunnel,layer,length_m,half_width_m,start_e,start_n,mid_e,mid_n,z0,z1,cover_min,cover_median,cover_max,"
                    "above_rail_bridge_m,above_road_bridge_m,above_building_m,above_road_ground_m,above_rail_ground_m,above_tunnel_m,"
                    "natural_cover_median,portal0,portal1,class_nominal,tube_runs,deck_runs,implied_deck_runs,slab_runs,portals,bored_runs,road_conflicts,class_cover_x0.5,class_cover_x1.5,class_len_x0.5,class_len_x1.5,class_grade_x0.5,class_grade_x1.5,name,above_ways\n";
    if (runs_csv) runs_csv << "first_way,kind,class,run,run_kind,start_m,end_m,length_m,min_cover,median_cover,max_cover,median_natural_cover,shape,exposed_roof\n";

    struct Tally { std::size_t n = 0; double km = 0; };
    std::map<std::string, Tally> table;           // "<kind> <variant> <class>"
    std::map<std::string, Tally> above_table;     // "<kind> <dominant above>"
    std::map<std::string, std::size_t> run_table; // "<kind> <run kind>[ shape]" -> runs
    std::map<std::string, double> run_km;
    std::size_t nodata = 0, conflicts_chains = 0, interior_end_chains = 0;
    std::vector<std::pair<double, double>> tube_len_cover;   // (length, median cover) of nominal tube runs, roads and rails
    for (const auto& c : chains) {
        const std::string kind = c.kind == Kind::Road ? "road" : "rail";
        if (c.nodata) { ++nodata; continue; }
        const st::Thresholds t = nominal(c);
        const auto run_variant = [&](double fc, double fl, double fg) {
            st::Thresholds v = t;
            v.clearance_m *= fc; v.roof_m *= fc; v.min_tube_m *= fl; v.min_gap_m *= fl; v.max_grade *= fg;
            std::vector<double> cv, nc; profile_at(c, v, cv, nc);
            return st::classify(cv, nc, c.above, v);
        };
        const auto res = st::classify(c.cover, c.natural_cover, c.above, t);
        const auto c05 = run_variant(0.5, 1, 1), c15 = run_variant(1.5, 1, 1), l05 = run_variant(1, 0.5, 1), l15 = run_variant(1, 1.5, 1);
        const auto g05 = run_variant(1, 1, 0.5), g15 = run_variant(1, 1, 1.5);
        const auto add = [&](const std::string& variant, const st::Classification& r) {
            auto& e = table[kind + " " + variant + " " + st::to_string(r.cls)]; ++e.n; e.km += c.length / 1000;
        };
        add("nominal", res); add("cover_x0.5", c05); add("cover_x1.5", c15); add("len_x0.5", l05); add("len_x1.5", l15); add("grade_x0.5", g05); add("grade_x1.5", g15);
        if (!c.portal0 || !c.portal1) ++interior_end_chains;
        // Dominant "above" (by station count, Nothing excluded unless alone).
        std::size_t dom = 0;
        for (std::size_t a = 1; a < c.above_count.size(); ++a) if (c.above_count[a] > (dom == 0 ? 0 : c.above_count[dom])) dom = a;
        { auto& e = above_table[kind + " " + st::to_string(static_cast<st::Above>(dom)) + " -> " + st::to_string(res.cls)]; ++e.n; e.km += c.length / 1000; }
        std::array<std::size_t, st::kRunKinds> nk{}; std::size_t bored = 0;
        for (std::size_t r = 0; r < res.runs.size(); ++r) {
            const auto& run = res.runs[r];
            const double len = static_cast<double>(run.last - run.first);
            std::string key = kind + " " + st::to_string(run.kind);
            ++nk[static_cast<std::size_t>(run.kind)];
            if (run.kind == st::RunKind::Tube) { key += std::string(" ") + st::to_string(run.shape); if (run.shape == st::TubeShape::Bored) ++bored; tube_len_cover.push_back({len, run.median_natural_cover_m}); }
            ++run_table[key]; run_km[key] += len / 1000;
            if (runs_csv) {
                char buf[320];
                std::snprintf(buf, sizeof buf, "%lld,%s,%s,%zu,%s,%zu,%zu,%.1f,%.2f,%.2f,%.2f,%.2f,%s,%d\n", static_cast<long long>(c.ways.front()), kind.c_str(), c.cls.c_str(), r,
                              st::to_string(run.kind), run.first, run.last, len, run.min_cover_m, run.median_cover_m, run.max_cover_m, run.median_natural_cover_m,
                              run.kind == st::RunKind::Tube ? st::to_string(run.shape) : "-", run.exposed_roof ? 1 : 0);
                runs_csv << buf;
            }
        }
        if (res.ground_road_conflicts > 0) ++conflicts_chains;
        if (csv) {
            std::vector<double> cv(c.cover.begin(), c.cover.end());
            std::vector<double> nv(c.natural_cover.begin(), c.natural_cover.end());
            std::string ways_s, above_s;
            for (std::size_t k = 0; k < c.ways.size(); ++k) ways_s += (k ? ";" : "") + std::to_string(c.ways[k]);
            for (const auto& [id, a] : c.above_ways) above_s += (above_s.empty() ? "" : ";") + std::to_string(id) + ":" + st::to_string(a);
            char buf[1024];
            std::snprintf(buf, sizeof buf, "%lld,%s,%s,%s,%s,%d,%.1f,%.2f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%zu,%zu,%zu,%zu,%zu,%zu,%.2f,%d,%d,%s,%zu,%zu,%zu,%zu,%zu,%zu,%zu,%s,%s,%s,%s,%s,%s,",
                          static_cast<long long>(c.ways.front()), ways_s.c_str(), kind.c_str(), c.cls.c_str(), c.tunnel_value.c_str(), c.layer, c.length, c.half_width,
                          c.start.first, c.start.second, c.mid.first, c.mid.second, c.z0, c.z1, *std::min_element(cv.begin(), cv.end()), median(cv),
                          *std::max_element(cv.begin(), cv.end()), c.above_count[1], c.above_count[2], c.above_count[3], c.above_count[4], c.above_count[5], c.above_count[6],
                          median(nv), c.portal0 ? 1 : 0, c.portal1 ? 1 : 0, st::to_string(res.cls), nk[0], nk[1], nk[2], nk[3], res.portals, bored, res.ground_road_conflicts, st::to_string(c05.cls), st::to_string(c15.cls),
                          st::to_string(l05.cls), st::to_string(l15.cls), st::to_string(g05.cls), st::to_string(g15.cls));
            std::string nm = c.name; for (auto& ch : nm) if (ch == ',' || ch == '"') ch = ' ';
            csv << buf << nm << ',' << above_s << '\n';
        }
    }
    std::cout << "tunnel chains with the first vertex in the box: " << chains.size() << " (no DEM under a sample: " << nodata << ")\n";
    std::cout << "chains by class (kind variant class: chains, km)\n";
    for (const auto& [k, v] : table) { char b[200]; std::snprintf(b, sizeof b, "  %-36s %6zu %8.2f\n", k.c_str(), v.n, v.km); std::cout << b; }
    std::cout << "chains by dominant structure above -> nominal class\n";
    for (const auto& [k, v] : above_table) { char b[200]; std::snprintf(b, sizeof b, "  %-40s %6zu %8.2f\n", k.c_str(), v.n, v.km); std::cout << b; }
    std::cout << "nominal runs (kind run [shape]: runs, km)\n";
    for (const auto& [k, v] : run_table) { char b[200]; std::snprintf(b, sizeof b, "  %-24s %6zu %8.2f\n", k.c_str(), v, run_km[k]); std::cout << b; }
    std::cout << "chains with a ground road above at less than clearance + roof: " << conflicts_chains << '\n';
    std::cout << "chains with an end inside the tunnel network (not a portal): " << interior_end_chains << '\n';
    // Tube-run length x median-cover histogram (the box / bored split).
    {
        const double lb[] = {0, 50, 100, 200, 300, 500, 1000, 1e9};
        const double cb[] = {0, 8, 10, 15, 20, 30, 1e9};
        std::cout << "nominal tube runs by length (rows, m) and median natural cover = ground - portal chord (columns, m)\n          ";
        for (int j = 0; j + 1 < 7; ++j) { char b[32]; std::snprintf(b, sizeof b, " %5.0f+", cb[j]); std::cout << b; }
        std::cout << '\n';
        for (int i = 0; i + 1 < 8; ++i) {
            char b[32]; std::snprintf(b, sizeof b, "  %6.0f+ ", lb[i]); std::cout << b;
            for (int j = 0; j + 1 < 7; ++j) {
                std::size_t n = 0;
                for (const auto& [l, cv] : tube_len_cover) if (l >= lb[i] && l < lb[i + 1] && cv >= cb[j] && cv < cb[j + 1]) ++n;
                std::snprintf(b, sizeof b, " %6zu", n); std::cout << b;
            }
            std::cout << '\n';
        }
    }

    // ---- portal joins: the tunnel floor's end height (a DEM sample at the node) against the ground way outside ----
    {
        std::ofstream pc; if (!portals_path.empty()) pc.open(portals_path);
        if (pc) pc << "first_way,kind,class,end,node,ground_way,length_m,step_m\n";
        const double bins[] = {0, 0.1, 0.3, 0.5, 1.0, 2.0, 1e9};
        std::map<std::string, std::array<std::size_t, 6>> hist;
        for (const auto& ch : chains) {
            if (ch.nodata) continue;
            for (int e = 0; e < 2; ++e) {
                const double st_m = ch.portal_step[static_cast<std::size_t>(e)];
                if (!(e == 0 ? ch.portal0 : ch.portal1) || std::isnan(st_m)) continue;
                auto& h = hist[ch.kind == Kind::Road ? "road" : "rail"];
                for (int b = 0; b < 6; ++b) if (std::fabs(st_m) >= bins[b] && std::fabs(st_m) < bins[b + 1]) ++h[static_cast<std::size_t>(b)];
                if (pc) {
                    char buf[200];
                    std::snprintf(buf, sizeof buf, "%lld,%s,%s,%d,%lld,%lld,%.1f,%.3f\n", static_cast<long long>(ch.ways.front()), ch.kind == Kind::Road ? "road" : "rail",
                                  ch.cls.c_str(), e, static_cast<long long>(ch.portal_node[static_cast<std::size_t>(e)]), static_cast<long long>(ch.portal_neighbour[static_cast<std::size_t>(e)]), ch.length, st_m);
                    pc << buf;
                }
            }
        }
        std::cout << "portal joins: |DEM at the portal node - median DEM 2..8 m out along the ground way| (portals: <0.1 / 0.1-0.3 / 0.3-0.5 / 0.5-1 / 1-2 / >=2 m)\n";
        for (const auto& [k, h] : hist) { char b[200]; std::snprintf(b, sizeof b, "  %-6s %6zu %6zu %6zu %6zu %6zu %6zu\n", k.c_str(), h[0], h[1], h[2], h[3], h[4], h[5]); std::cout << b; }
    }

    // ---- roads under bridges ----
    {
        std::ofstream under; if (!under_path.empty()) under.open(under_path);
        if (under) under << "lower_way,lower_class,upper_way,upper_kind,upper_class,upper_layer,east,north,hump_m,dem_at_crossing,chord_at_crossing\n";
        std::map<std::string, std::array<std::size_t, 4>> tally;   // upper kind -> {crossings, hump>=1, hump>=3, hump>=5}
        std::set<std::pair<std::int64_t, std::int64_t>> done;
        for (std::size_t i = 0; i < ways.size(); ++i) {
            const Way& lo = *ways[i];
            if (lo.kind != Kind::Road || lo.bridge || lo.tunnel) continue;
            const P f = lo.pts.front();
            if (!(f.first >= emin && f.first < emax && f.second >= nmin && f.second < nmax)) continue;
            Line line; line.pts = lo.pts; line.build();
            const std::set<std::int64_t> lo_nodes(lo.refs.begin(), lo.refs.end());
            for (std::size_t s = 0; s + 1 < lo.pts.size(); ++s) {
                const P mid{(lo.pts[s].first + lo.pts[s + 1].first) / 2, (lo.pts[s].second + lo.pts[s + 1].second) / 2};
                std::set<std::pair<std::size_t, std::size_t>> cands;
                for (const P q : {lo.pts[s], mid, lo.pts[s + 1]}) if (const auto* v = lines.at(q)) cands.insert(v->begin(), v->end());
                for (const auto& [wj, sj] : cands) {
                    const Way& up = *ways[wj];
                    if (!up.bridge || wj == i) continue;
                    if (up.layer < lo.layer) continue;
                    if (done.count({lo.id, up.id})) continue;
                    bool shares = false;
                    for (const auto r : up.refs) if (lo_nodes.count(r)) { shares = true; break; }
                    if (shares) continue;
                    double t = 0;
                    if (!seg_cross(lo.pts[s], lo.pts[s + 1], up.pts[sj], up.pts[sj + 1], &t)) continue;
                    done.insert({lo.id, up.id});
                    const double sc = line.cum[s] + t * (line.cum[s + 1] - line.cum[s]);
                    // Footprint of the upper bridge along the lower road: its half width over the crossing angle, capped.
                    P pc, dc; line.at(sc, pc, dc);
                    const double ux = up.pts[sj + 1].first - up.pts[sj].first, uy = up.pts[sj + 1].second - up.pts[sj].second;
                    const double ul = std::hypot(ux, uy);
                    const double sine = ul > 0 ? std::fabs(dc.first * uy - dc.second * ux) / ul : 1;
                    const double reach = std::min(30.0, up.half_width_m / std::max(sine, 0.2) + 2.0);
                    // Chord ends up to 30 m either side, clipped to the lower way (a short way ending at the portal of an
                    // underpass still counts); skipped only when the chord would not clear the bridge footprint.
                    const double sa = std::max(0.0, sc - 30.0), sb = std::min(line.length(), sc + 30.0);
                    if (sc - sa < reach + 2.0 || sb - sc < reach + 2.0) continue;
                    P pa, pb, d2; line.at(sa, pa, d2); line.at(sb, pb, d2);
                    const double za = dem.at(pa.first, pa.second), zb = dem.at(pb.first, pb.second);
                    if (std::isnan(za) || std::isnan(zb)) continue;
                    double hump = -1e9;
                    for (double x = sc - reach; x <= sc + reach; x += 1.0) {
                        P px, dx; line.at(x, px, dx);
                        const double z = dem.at(px.first, px.second);
                        if (!std::isnan(z)) hump = std::max(hump, z - (za + (zb - za) * (x - sa) / (sb - sa)));
                    }
                    const std::string uk = std::string(up.kind == Kind::Rail ? "rail" : "road") + " bridge" + (up.construction ? " (construction)" : "");
                    auto& tl = tally[uk];
                    ++tl[0]; if (hump >= 1) ++tl[1]; if (hump >= 3) ++tl[2]; if (hump >= 5) ++tl[3];
                    if (under) {
                        char buf[300];
                        const double zc = dem.at(pc.first, pc.second);
                        std::snprintf(buf, sizeof buf, "%lld,%s,%lld,%s,%s,%d,%.1f,%.1f,%.2f,%.2f,%.2f\n", static_cast<long long>(lo.id), lo.cls.c_str(),
                                      static_cast<long long>(up.id), up.kind == Kind::Rail ? "rail" : "road", up.cls.c_str(), up.layer, pc.first, pc.second, hump, zc,
                                      za + (zb - za) * 0.5);
                        under << buf;
                    }
                }
            }
        }
        std::cout << "ground roads crossing under a bridge (upper: crossings, DEM hump >= 1 / 3 / 5 m)\n";
        for (const auto& [k, v] : tally) { char b[200]; std::snprintf(b, sizeof b, "  %-28s %6zu %6zu %6zu %6zu\n", k.c_str(), v[0], v[1], v[2], v[3]); std::cout << b; }
    }

    // ---- spots: the nearest chains ----
    for (const auto& sp : spots) {
        std::vector<std::pair<double, std::size_t>> near;
        for (std::size_t k = 0; k < chains.size(); ++k) near.push_back({std::hypot(chains[k].mid.first - sp.first, chains[k].mid.second - sp.second), k});
        std::sort(near.begin(), near.end());
        std::cout << "spot " << sp.first << ' ' << sp.second << ":\n";
        for (std::size_t k = 0; k < std::min<std::size_t>(3, near.size()); ++k) {
            const auto& c = chains[near[k].second];
            if (c.nodata) continue;
            const auto res = st::classify(c.cover, c.natural_cover, c.above, nominal(c));
            std::cout << "  " << near[k].first << " m: chain " << c.ways.front() << " (" << c.ways.size() << " ways) " << (c.kind == Kind::Road ? "road " : "rail ") << c.cls
                      << " len " << c.length << " class " << st::to_string(res.cls) << " runs";
            for (const auto& r : res.runs) std::cout << ' ' << st::to_string(r.kind) << '[' << r.first << ',' << r.last << "]med" << r.median_cover_m << "/nat" << r.median_natural_cover_m;
            std::cout << " above";
            for (const auto& [id, a] : c.above_ways) std::cout << ' ' << id << ':' << st::to_string(a);
            std::cout << '\n';
        }
    }
    std::cerr << "dem tiles loaded " << dem.loads << '\n';
    return 0;
}
