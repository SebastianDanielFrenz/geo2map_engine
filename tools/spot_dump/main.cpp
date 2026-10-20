// tools/spot_dump/main.cpp - g2m_spot_dump: read-only inspection of the SERVED road geometry at a spot.
//
//   g2m_spot_dump STORE SCOPE SCRATCH_DIR ZONE EASTING NORTHING RADIUS_M [--way ID] [--step M]
//   g2m_spot_dump STORE SCOPE SCRATCH_DIR ZONE EASTING NORTHING RADIUS_M --census HALF_M [--csv FILE] [--baseline]
// --portals FILE (with --census): phase 7a-1 portal census, one CSV row per node a tunnel entry ends at: the tunnel floor end
// height and grade, the profiled non-tunnel entries ending there (height, grade) and the declined ground stretches ending
// there; a summary line counts the portals, the steps above 0.01 m / 0.5 m and the declined ground stretches at portals.
// --baseline: run with D64 (hump mask under upper bridges) and D65 (short-tunnel cut) switched off, for before/after.
//
// --census: every level-2 roads.geom tile within HALF_M of the point is derived and tallied: entries by
// provenance, declined entries by message and by kind (ground / bridge / tunnel), and the declined entries that
// end at a node a profiled entry also ends at ("a road that ends in a wall"); with --csv one line per declined
// entry (way, stretch, first node east/north, highway, bridge, tunnel, message, touches_profiled).
//
// Derives g2m.roads.geom (level 2) and g2m.terrain.carved / g2m.terrain.height (level 0) for the tiles
// around the point through the same local release the game builds (builtin_local_release_params), with
// the derived tiles written to SCRATCH_DIR (a private scratch directory; STORE is opened read-only and is
// never written). Lists every roads.geom entry whose reference line passes within RADIUS_M of the point
// (way, stretch, provenance, bridge/tunnel, underpass decks) and, with --way ID, samples that way's
// stretch(es) every --step metres (default 2): station, UTM position, served profile height, bare-earth
// DEM height (terrain.height L0), carved terrain height (terrain.carved L0), carved - profile.
#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/release.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/road_profile.h"
#include "g2m/layer/terrain_carve.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <map>
#include <set>
#include <memory>
#include <string>
#include <tuple>
#include <vector>
#include <string_view>

using namespace g2m;

namespace {
struct Sampler {
    Server& server;
    ReleaseId rid;
    std::string layer;
    geo::UtmZone zone;
    std::map<std::pair<int, int>, std::unique_ptr<HeightTile>> tiles;
    std::map<std::pair<int, int>, double> offsets;
    // Heights in metres at (e, n): bilinear over the cell centres of the level-0 tile.
    double at(double e, double n) {
        const auto key = tile_key_at(zone, 0, e, n);
        if (!key) return NAN;
        auto it = tiles.find({key->x, key->y});
        if (it == tiles.end()) {
            TileRequest req;
            req.rid = rid;
            req.layer = layer;
            req.key = *key;
            const auto resp = server.tile(req);
            std::unique_ptr<HeightTile> t;
            if (resp.meta.status == Status::Ok) {
                const auto view = parse_container(resp.container);
                if (view) {
                    const auto body = decode_body(view->body);
                    t = std::make_unique<HeightTile>();
                    if (!body || !decode_height_tile(*body, *key, *t)) t.reset();
                    else offsets[{key->x, key->y}] = view->header.height_offset;
                }
            }
            it = tiles.emplace(std::make_pair(key->x, key->y), std::move(t)).first;
        }
        if (!it->second) return NAN;
        const double fx = (e - static_cast<double>(key->min_easting())) - 0.5, fy = (n - static_cast<double>(key->min_northing())) - 0.5;
        const int i = static_cast<int>(std::floor(fx)), j = static_cast<int>(std::floor(fy));
        const double tx = fx - i, ty = fy - j;
        const double off = offsets[{key->x, key->y}];
        const auto h = [&](int a, int b) {
            a = std::clamp(a, 0, 255);
            b = std::clamp(b, 0, 255);
            const auto v = it->second->h[static_cast<std::size_t>(b) * 256 + static_cast<std::size_t>(a)];
            return v == kHeightNoData ? static_cast<double>(NAN) : (static_cast<double>(v) + off) / 256.0;
        };
        return (1 - ty) * ((1 - tx) * h(i, j) + tx * h(i + 1, j)) + ty * ((1 - tx) * h(i, j + 1) + tx * h(i + 1, j + 1));
    }
};
} // namespace

int main(int argc, char** argv) {
    if (argc < 8) {
        std::cerr << "usage: g2m_spot_dump STORE SCOPE SCRATCH_DIR ZONE E N RADIUS_M [--way ID] [--step M]\n";
        return 1;
    }
    std::int64_t way_filter = 0;
    double step = 2, census_half = 0, cross_s = -1;
    std::string csv_path;
    bool baseline = false;
    std::string portals_path;
    for (int i = 8; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--way" && i + 1 < argc) way_filter = std::atoll(argv[++i]);
        else if (a == "--step" && i + 1 < argc) step = std::atof(argv[++i]);
        else if (a == "--cross" && i + 1 < argc) cross_s = std::atof(argv[++i]);
        else if (a == "--census" && i + 1 < argc) census_half = std::atof(argv[++i]);
        else if (a == "--csv" && i + 1 < argc) csv_path = argv[++i];
        else if (a == "--baseline") baseline = true;
        else if (a == "--portals" && i + 1 < argc) portals_path = argv[++i];
        else { std::cerr << "unknown option\n"; return 1; }
    }
    if (baseline) {   // diagnostic only: the rules as they were before D64/D65 (separate SCRATCH_DIR: other derived keys)
        auto& rules = const_cast<RoadRules&>(builtin_road_rules_de());
        rules.underpass.overpass = false;
        rules.tunnel.cut_max_length_m = 0;
    }
    const std::string scope = argv[2];
    const geo::UtmZone zone{std::atoi(argv[4]), geo::Hemisphere::North};
    const double e0 = std::atof(argv[5]), n0 = std::atof(argv[6]), radius = std::atof(argv[7]);
    TileStoreConfig sc;
    sc.dir = argv[1];
    sc.read_only = true;
    auto sources = TileStore::open(sc);
    if (!sources) { std::cerr << sources.error().message << '\n'; return 1; }
    TileStoreConfig dc;
    dc.dir = argv[3];
    auto derived = TileStore::open(dc);
    if (!derived) { std::cerr << derived.error().message << '\n'; return 1; }
    BuiltinTerrainDerivers derivers;
    auto params = builtin_local_release_params(scope, derivers, {}, true, true, true, false);
    auto manifest = make_local_release(**sources, params);
    if (!manifest) { std::cerr << manifest.error().message << '\n'; return 1; }
    LocalSourceUpstream upstream(**sources);
    ServerConfig config;
    config.server_id = "g2m_spot_dump";
    config.offline = true;
    config.derived_store = &**derived;
    config.derivers = derivers.ordered(true, true, true, false);
    config.upstreams = {&upstream};
    Server server(config);
    if (auto added = server.add_release(*manifest, scope); !added) { std::cerr << added.error().message << '\n'; return 1; }
    if (census_half > 0) {
        std::map<std::string, std::size_t> by_message, by_prov;
        std::size_t entries = 0, declined = 0, orphan = 0, tiles = 0, tunnel_floors = 0, tunnel_straight = 0, tunnel_straight_short = 0, tunnel_carved = 0, tunnel_crossed = 0;
        std::map<std::string, std::size_t> by_kind, orphan_by_kind;
        std::ofstream csv; if (!csv_path.empty()) csv.open(csv_path);
        if (csv) csv << "way,start_ref,end_ref,east,north,highway,bridge,tunnel,kind,touches_profiled,message\n";
        const auto k0 = tile_key_at(zone, 2, e0 - census_half, n0 - census_half), k1 = tile_key_at(zone, 2, e0 + census_half, n0 + census_half);
        if (!k0 || !k1) return 1;
        static const char* kProvN[] = {"strict", "retry", "bridge-completed", "ground-completed", "tunnel-floor"};
        // 7a-1: what ends at each node, across the census tiles (a node is listed once per entry end).
        struct EndAt { std::int64_t way; int prov; bool tunnel, bridge; double z, grade, length, max_grade; };
        struct NodeEnds { double east = 0, north = 0; std::vector<EndAt> ends; int declined_ground = 0; std::vector<std::string> declined_msgs; };
        std::map<std::int64_t, NodeEnds> node_ends;
        // 7a-2: the carve opens a short straight floor only under a hump of ground (D65); the base DEM is read as the carve reads it.
        Sampler hump_dem{server, manifest->rid, std::string(kTerrainHeightLayer), zone, {}, {}};
        const CutGroundAt hump_ground = [&](double x, double y) { return hump_dem.at(x, y); };
        std::set<std::tuple<std::int64_t, std::uint32_t, int>> seen_ends, seen_declined;
        for (std::int32_t ty = k0->y; ty <= k1->y; ++ty) for (std::int32_t tx = k0->x; tx <= k1->x; ++tx) {
            TileKey key = *k0; key.x = tx; key.y = ty;
            TileRequest req; req.rid = manifest->rid; req.layer = std::string(kRoadGeomLayer); req.key = key;
            const auto resp = server.tile(req);
            if (resp.meta.status != Status::Ok) continue;
            const auto view = parse_container(resp.container); if (!view) continue;
            const auto body = decode_body(view->body); if (!body) continue;
            const auto tile = decode_road_geom(*body); if (!tile) continue;
            ++tiles;
            const auto& graph = tile->source.graph;
            std::set<std::int64_t> profiled_ends;
            for (const auto& e : tile->entries) {
                if (!e.profile) continue;
                const auto way = std::lower_bound(graph.ways.begin(), graph.ways.end(), e.way_id, [](const auto& w, auto id) { return w.osm_id < id; });
                if (way == graph.ways.end() || way->osm_id != e.way_id || e.stretch.end_ref >= way->node_ids.size()) continue;
                profiled_ends.insert(way->node_ids[e.stretch.start_ref]); profiled_ends.insert(way->node_ids[e.stretch.end_ref]);
            }
            if (!portals_path.empty()) {
                const auto position_of = [&](std::int64_t id) {
                    const auto n = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), id, [](const auto& a, auto v) { return a.osm_id < v; });
                    return n->position;
                };
                for (std::size_t ei = 0; ei < tile->entries.size(); ++ei) {
                    const auto& e = tile->entries[ei];
                    const auto way = std::lower_bound(graph.ways.begin(), graph.ways.end(), e.way_id, [](const auto& w, auto id) { return w.osm_id < id; });
                    if (way == graph.ways.end() || way->osm_id != e.way_id || e.stretch.end_ref >= way->node_ids.size()) continue;
                    const std::int64_t ends[2] = {way->node_ids[e.stretch.start_ref], way->node_ids[e.stretch.end_ref]};
                    if (e.profile) {
                        const auto& p = *e.profile;
                        const auto prov = ei < tile->origins.size() ? static_cast<int>(tile->origins[ei].provenance) : 0;
                        for (int side = 0; side < 2; ++side) {
                            if (!seen_ends.insert({e.way_id, e.stretch.start_ref, side}).second) continue;
                            const double st = side ? p.reference.length_m : 0;
                            const auto z = p.at(st); const auto xy = p.reference.at(st); if (!z || !xy) continue;
                            // The grade is reported rising away from the node (the end grade at side 1, the negated start grade at side 0).
                            auto& n = node_ends[ends[side]]; const auto pos = position_of(ends[side]);
                            n.east = pos.x / 1000.0; n.north = pos.y / 1000.0;
                            n.ends.push_back({e.way_id, prov, p.attributes.tunnel, p.attributes.bridge, z->height_m, side ? -z->grade : z->grade, p.reference.length_m, p.attributes.max_grade_per_mille / 1000.0});
                        }
                    } else {
                        bool tunnel = false, bridge = false;
                        for (const auto& [k, v] : way->tags) { if (k == "tunnel" && v != "no") tunnel = true; if (k == "bridge" && v != "no") bridge = true; }
                        if (tunnel || bridge) continue;
                        for (int side = 0; side < 2; ++side) if (seen_declined.insert({e.way_id, e.stretch.start_ref, side}).second) {
                            auto& nd = node_ends[ends[side]]; ++nd.declined_ground;
                            const auto pos = position_of(ends[side]); nd.east = pos.x / 1000.0; nd.north = pos.y / 1000.0;
                            nd.declined_msgs.push_back("way " + std::to_string(e.way_id) + ": " + (e.decline ? e.decline->message : std::string("?")));
                        }
                    }
                }
            }
            for (std::size_t ei = 0; ei < tile->entries.size(); ++ei) {
                const auto& e = tile->entries[ei];
                ++entries;
                const auto prov = ei < tile->origins.size() ? static_cast<int>(tile->origins[ei].provenance) : 0;
                if (e.profile) {
                    ++by_prov[kProvN[prov]];
                    if (prov == 4) {   // a tunnel floor: length and deviation from the straight chord of its portal heights
                        const auto& p = *e.profile;
                        const auto a = p.at(0), b = p.at(p.reference.length_m);
                        double dev = 0;
                        if (a && b) for (double s = 0; s <= p.reference.length_m; s += 1) {
                            const auto z = p.at(s);
                            if (z) dev = std::max(dev, std::fabs(z->height_m - (a->height_m + (b->height_m - a->height_m) * s / p.reference.length_m)));
                        }
                        const auto way = std::lower_bound(graph.ways.begin(), graph.ways.end(), e.way_id, [](const auto& w, auto id) { return w.osm_id < id; });
                        const auto xy = p.reference.at(0.5 * p.reference.length_m);
                        const bool short_straight = dev <= 0.01 && p.reference.length_m <= 80;
                        const bool carved_flag = csv && short_straight && tunnel_entry_is_carved(*tile, ei, CarveOptions{}, builtin_road_rules_de().tunnel, &hump_ground);
                        const bool carved_nohump = csv && short_straight && tunnel_entry_is_carved(*tile, ei, CarveOptions{}, builtin_road_rules_de().tunnel, nullptr);
                        if (csv && xy && way != graph.ways.end() && way->osm_id == e.way_id)
                            csv << e.way_id << ',' << e.stretch.start_ref << ',' << e.stretch.end_ref << ',' << xy->x << ',' << xy->y << ",,," << "yes" << ",tunnel-floor,0,\"len="
                                << p.reference.length_m << " off_chord=" << dev << " carved=" << carved_flag << " carved_without_hump_check=" << carved_nohump << "\"\n";
                        ++tunnel_floors; if (dev <= 0.01) ++tunnel_straight;
                        if (dev <= 0.01 && p.reference.length_m <= 80) {
                            ++tunnel_straight_short;
                            if (tunnel_entry_is_carved(*tile, ei, CarveOptions{}, builtin_road_rules_de().tunnel, &hump_ground)) ++tunnel_carved;
                            else if (tunnel_entry_is_carved(*tile, ei, [] { CarveOptions o; o.tunnel_cut_crossing_check = false; return o; }(), builtin_road_rules_de().tunnel, &hump_ground)) {
                                ++tunnel_crossed;
                                std::cout << "  crossed by a ground road, left uncarved: way " << e.way_id << " len " << p.reference.length_m << " at " << (xy ? xy->x : 0) << ' ' << (xy ? xy->y : 0) << std::endl;
                            }
                        }
                    }
                    continue;
                }
                ++declined;
                const auto way = std::lower_bound(graph.ways.begin(), graph.ways.end(), e.way_id, [](const auto& w, auto id) { return w.osm_id < id; });
                if (way == graph.ways.end() || way->osm_id != e.way_id || e.stretch.end_ref >= way->node_ids.size()) continue;
                std::string highway, bridge, tunnel;
                for (const auto& [k, v] : way->tags) { if (k == "highway") highway = v; else if (k == "bridge") bridge = v; else if (k == "tunnel") tunnel = v; }
                const char* kind = !tunnel.empty() && tunnel != "no" ? "tunnel" : !bridge.empty() && bridge != "no" ? "bridge" : "ground";
                const std::string msg = e.decline ? e.decline->message : "?";
                ++by_message[std::string(kind) + ": " + msg]; ++by_kind[kind];
                const bool touches = profiled_ends.count(way->node_ids[e.stretch.start_ref]) || profiled_ends.count(way->node_ids[e.stretch.end_ref]);
                if (touches) { ++orphan; ++orphan_by_kind[kind]; }
                const auto n = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), way->node_ids[e.stretch.start_ref], [](const auto& a, auto id) { return a.osm_id < id; });
                if (csv && n != graph.nodes.end())
                    csv << e.way_id << ',' << e.stretch.start_ref << ',' << e.stretch.end_ref << ',' << n->position.x / 1000.0 << ',' << n->position.y / 1000.0 << ','
                        << highway << ',' << bridge << ',' << tunnel << ',' << kind << ',' << (touches ? 1 : 0) << ",\"" << msg << "\"\n";
            }
        }
        std::cout << "census tiles " << tiles << " entries " << entries << " declined " << declined << " declined_touching_profiled " << orphan << '\n';
        std::cout << "  tunnel floors " << tunnel_floors << " straight " << tunnel_straight << " straight and <= 80 m " << tunnel_straight_short
                  << " carved as a cut " << tunnel_carved << " left uncarved, crossed by a ground road " << tunnel_crossed << '\n';
        for (const auto& [k, v] : by_prov) std::cout << "  profiled " << k << ' ' << v << '\n';
        for (const auto& [k, v] : by_kind) std::cout << "  declined " << k << ' ' << v << " (touching profiled " << orphan_by_kind[k] << ")\n";
        for (const auto& [k, v] : by_message) std::cout << "  " << v << "  " << k << '\n';
        if (!portals_path.empty()) {
            std::ofstream out(portals_path);
            out << "node,east,north,tunnel_ways,floor_z,floor_grade,ground_ways,ground_z,ground_grade,dz,dgrade,declined_ground,ground_prov,tunnel_len,ground_len,decline\n";
            std::size_t unreachable = 0, unreachable_strict = 0;
            std::size_t portals = 0, with_ground = 0, step_001 = 0, step_05 = 0, declined_at_portal = 0, tunnel_tunnel = 0, tt_bad = 0, grade_bad = 0;
            double worst = 0, worst_grade = 0;
            for (const auto& [id, n] : node_ends) {
                std::vector<const EndAt*> tun, gro;
                for (const auto& e : n.ends) { if (e.tunnel) tun.push_back(&e); else if (!e.bridge) gro.push_back(&e); }
                if (tun.empty()) continue;
                ++portals; declined_at_portal += static_cast<std::size_t>(n.declined_ground);
                double dz = 0, dg = 0;
                for (const auto* t : tun) {
                    for (const auto* g : gro) { dz = std::max(dz, std::fabs(t->z - g->z)); dg = std::max(dg, std::fabs(t->grade + g->grade)); }
                    for (const auto* u : tun) if (u != t) { dz = std::max(dz, std::fabs(t->z - u->z)); dg = std::max(dg, std::fabs(t->grade + u->grade)); }
                }
                if (!gro.empty()) ++with_ground;
                if (tun.size() > 1) ++tunnel_tunnel;
                if (!gro.empty() || tun.size() > 1) {
                    if (dz > 0.01) ++step_001;
                    if (dz > 0.5) ++step_05;
                    if (dg > 0.02) ++grade_bad;
                    worst = std::max(worst, dz); worst_grade = std::max(worst_grade, dg);
                }
                if (tun.size() > 1 && dz > 0.01) ++tt_bad;
                out << id << ',' << n.east << ',' << n.north << ',' << tun.size() << ',' << tun[0]->z << ',' << tun[0]->grade << ',' << gro.size() << ','
                    << (gro.empty() ? 0.0 : gro[0]->z) << ',' << (gro.empty() ? 0.0 : gro[0]->grade) << ',' << dz << ',' << dg << ',' << n.declined_ground << ',' << (gro.empty() ? "-" : kProvN[gro[0]->prov]) << ',' << tun[0]->length << ',' << (gro.empty() ? 0.0 : gro[0]->length) << ',';
                for (std::size_t k = 0; k < n.declined_msgs.size(); ++k) out << (k ? " | " : "") << n.declined_msgs[k];
                out << '\n';
                if (!gro.empty() && dz > 0.01 && tun.size() == 1) {
                    ++unreachable; if (gro[0]->prov < 2) ++unreachable_strict;
                    std::cout << "  unreachable node " << id << " at " << n.east << ' ' << n.north << ": dz " << dz << " m, tunnel stub " << tun[0]->length << " m (floor bound 0.30), ground " << kProvN[gro[0]->prov]
                              << ' ' << gro[0]->length << " m (class bound " << std::max(0.12, gro[0]->max_grade) << ", end grade " << gro[0]->grade << "), floor z " << tun[0]->z << ", ground z " << gro[0]->z << '\n';
                }
            }
            std::cout << "portals " << portals << " with_ground_profile " << with_ground << " tunnel_tunnel " << tunnel_tunnel << " (step > 0.01 m: " << tt_bad
                      << ") step>0.01m " << step_001 << " step>0.5m " << step_05 << " grade>0.02 " << grade_bad << " worst_dz " << worst << " worst_dgrade " << worst_grade
                      << " declined_ground_at_portals " << declined_at_portal << " unreachable(dz>0.01, one tunnel) " << unreachable << " of which strict/retry ground " << unreachable_strict << '\n';
        }
        return 0;
    }
    const auto l2 = tile_key_at(zone, 2, e0, n0);
    if (!l2) return 1;
    TileRequest req;
    req.rid = manifest->rid;
    req.layer = std::string(kRoadGeomLayer);
    req.key = *l2;
    const auto resp = server.tile(req);
    if (resp.meta.status != Status::Ok) { std::cerr << "roads.geom: " << resp.meta.message << '\n'; return 1; }
    const auto view = parse_container(resp.container);
    if (!view) { std::cerr << view.error().message << '\n'; return 1; }
    const auto body = decode_body(view->body);
    if (!body) { std::cerr << body.error().message << '\n'; return 1; }
    const auto tile = decode_road_geom(*body);
    if (!tile) { std::cerr << tile.error().message << '\n'; return 1; }
    std::cout << "roads.geom " << to_string(*l2) << " encoding " << tile->encoding << " entries " << tile->entries.size()
              << " underpasses " << tile->underpasses.size() << " height_offset " << view->header.height_offset << '\n';
    static const char* kProv[] = {"strict", "retry", "bridge-completed", "ground-completed", "tunnel-floor"};
    Sampler dem{server, manifest->rid, std::string(kTerrainHeightLayer), zone, {}, {}};
    Sampler carved{server, manifest->rid, std::string(kTerrainCarvedLayer), zone, {}, {}};
    for (std::size_t ei = 0; ei < tile->entries.size(); ++ei) {
        const auto& e = tile->entries[ei];
        if (way_filter && e.way_id != way_filter) continue;
        if (!e.profile) {
            if (!way_filter) continue;
            std::cout << "entry " << ei << " way " << e.way_id << " stretch " << e.stretch.start_ref << ".." << e.stretch.end_ref
                      << " DECLINED: " << (e.decline ? e.decline->message : "?") << '\n';
            // The way's OSM node polyline instead of a reference line: DEM and carved height every --step metres.
            const auto& graph = tile->source.graph;
            const auto way = std::lower_bound(graph.ways.begin(), graph.ways.end(), e.way_id, [](const auto& w, auto id) { return w.osm_id < id; });
            if (way == graph.ways.end() || way->osm_id != e.way_id) continue;
            std::vector<std::pair<double, double>> pts;
            for (std::size_t k = e.stretch.start_ref; k <= e.stretch.end_ref && k < way->node_ids.size(); ++k) {
                const auto n = std::lower_bound(graph.nodes.begin(), graph.nodes.end(), way->node_ids[k], [](const auto& a, auto id) { return a.osm_id < id; });
                if (n != graph.nodes.end()) pts.emplace_back(static_cast<double>(n->position.x) / 1000.0, static_cast<double>(n->position.y) / 1000.0);
            }
            double run = 0;
            std::printf("    s_m      east       north      dem   carved\n");
            for (std::size_t k = 0; k + 1 < pts.size(); ++k) {
                const double len = std::hypot(pts[k + 1].first - pts[k].first, pts[k + 1].second - pts[k].second);
                for (double t = 0; t < len; t += step) {
                    const double x = pts[k].first + (pts[k + 1].first - pts[k].first) * t / len;
                    const double y = pts[k].second + (pts[k + 1].second - pts[k].second) * t / len;
                    std::printf("%7.1f %10.1f %10.1f %8.2f %8.2f\n", run + t, x, y, dem.at(x, y), carved.at(x, y));
                }
                run += len;
            }
            continue;
        }
        const auto& p = *e.profile;
        double best = 1e18, best_s = 0;
        for (double s = 0; s <= p.reference.length_m; s += 1.0) {
            const auto pose = p.reference.at(s);
            if (!pose) continue;
            const double d = std::hypot(pose->x - e0, pose->y - n0);
            if (d < best) { best = d; best_s = s; }
        }
        if (best > radius) continue;
        const auto prov = ei < tile->origins.size() ? static_cast<int>(tile->origins[ei].provenance) : 0;
        std::cout << "entry " << ei << " way " << e.way_id << " stretch " << e.stretch.start_ref << ".." << e.stretch.end_ref
                  << " len " << p.reference.length_m << " m  closest " << best << " m at s=" << best_s << "  bridge=" << p.attributes.bridge
                  << " tunnel=" << p.attributes.tunnel << " prov=" << kProv[prov] << " lanes=" << p.attributes.lanes << '\n';
        for (const auto& u : tile->underpasses)
            if (u.entry_index == ei) std::cout << "   underpass deck " << u.start_m << " .. " << u.end_m << " depth_mm " << u.depth_mm << '\n';
        if (!way_filter) continue;
        if (cross_s >= 0) {   // lateral profile (left positive) of the DEM and the carved terrain at one station
            const double ss = std::min(cross_s, p.reference.length_m);
            const auto pose = p.reference.at(ss); const auto v = p.at(ss);
            if (!pose || !v) continue;
            std::printf("    lateral at s=%.1f (profile %.2f)\n   off     dem  carved\n", ss, v->height_m);
            const double nx = -std::sin(pose->heading), ny = std::cos(pose->heading);
            for (double o = -30; o <= 30.001; o += 1.5)
                std::printf("%6.1f %7.2f %7.2f\n", o, dem.at(pose->x + o * nx, pose->y + o * ny), carved.at(pose->x + o * nx, pose->y + o * ny));
            continue;
        }
        std::printf("    s_m      east       north   profile      dem   carved  carved-prof  dem-prof\n");
        for (double s = 0; s <= p.reference.length_m + 1e-9; s += step) {
            const double ss = std::min(s, p.reference.length_m);
            const auto pose = p.reference.at(ss);
            const auto v = p.at(ss);
            if (!pose || !v) continue;
            const double d = dem.at(pose->x, pose->y), c = carved.at(pose->x, pose->y);
            std::printf("%7.1f %10.1f %10.1f %8.2f %8.2f %8.2f %9.2f %9.2f\n", ss, pose->x, pose->y, v->height_m, d, c, c - v->height_m,
                        d - v->height_m);
        }
    }
}
