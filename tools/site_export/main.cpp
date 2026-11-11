// g2m_site_export - the map features a farming game places buyers and supply stores on (docs/pipeline/site_export.md).
//
//   g2m_site_export --store DIR --scope SCOPE --rules data/site_export/de.json --out sites.json [--region-min E,N --region-max E,N]
//                   [--pbf A.osm.pbf,B.osm.pbf,... --margin-km 60]
//
// --pbf/--margin-km: the farm's fallback sites need the REAL plants up to the largest spacing (60 km) outside the region (else it places a fake one
// next to a real plant just over the border). The scope's store holds only the region, so the margin is read straight from the given PBF files:
// every feature also carries g2m:landuse (farmyard | industrial | commercial | retail | railway | none): the closed landuse way it lies in, so the farm can tell
// a merchant's silo from a farmyard's. Multipolygon relations are not read.
// every tagged node or way (the same rules) whose position lies within margin of the region and that the store did not already give. No industrial
// candidates from the margin (a procedural site is only placed inside the region). Ids and positions as for the store's features.
//
// Reads every g2m.src.osm tile of the scope (read only) and writes "farm.site_input/1": one record per OSM node or way carrying a listed tag or a
// listed word in its name (id, ALL tags, name, position in integer UTM millimetres of the bake's zone; a way takes the mean of its nodes), plus
// industrial land (landuse=industrial ways: mean position and the distance to the nearest highway vertex). Ids: a node keeps its OSM id, a way's is
// its id with bit 60 set (a node and a way never collide; the top bit stays free for procedural sites). Tile halos repeat features: they are
// kept once. The region is the bounding box of the scope's tiles (metres in the flags) unless given. Output is sorted by id and deterministic.
#include "g2m/core/geo/utm.h"
#include "g2m/decode/osm/pbf_reader.h"
#include "g2m/core/geom/project.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/tile_store.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <memory>
#include <string>
#include <vector>

using namespace g2m;
using json = nlohmann::json;

namespace {

constexpr std::uint64_t kWayBit = 1ull << 60;

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

struct Feature {
    std::uint64_t id = 0;
    std::map<std::string, std::string> tags;
    std::string name;
    std::int64_t e = 0, n = 0;
};
struct Industrial {
    std::int64_t e = 0, n = 0, road = 0;
};
// A closed landuse way of a kind that tells a merchant's silo from a farm's: the exporter writes the containing kind as the feature tag g2m:landuse.
struct Poly {
    std::string kind;
    std::vector<std::pair<std::int64_t, std::int64_t>> pts;
    std::int64_t x0 = INT64_MAX, y0 = INT64_MAX, x1 = INT64_MIN, y1 = INT64_MIN;
};
bool polygon_kind(const std::string& k) { return k == "farmyard" || k == "industrial" || k == "commercial" || k == "retail" || k == "railway"; }
void close_poly(Poly& g) {
    for (const auto& [x, y] : g.pts) {
        g.x0 = std::min(g.x0, x);
        g.y0 = std::min(g.y0, y);
        g.x1 = std::max(g.x1, x);
        g.y1 = std::max(g.y1, y);
    }
}
bool inside(const Poly& g, std::int64_t px, std::int64_t py) {
    if (px < g.x0 || px > g.x1 || py < g.y0 || py > g.y1) return false;
    bool in = false;
    for (std::size_t i = 0, j = g.pts.size() - 1; i < g.pts.size(); j = i++) {
        const auto& a = g.pts[i];
        const auto& b = g.pts[j];
        if ((a.second > py) != (b.second > py) && static_cast<long double>(px - a.first) < static_cast<long double>(b.first - a.first) * (py - a.second) / (b.second - a.second)) in = !in;
    }
    return in;
}

bool get_pair(const std::string& s, std::int64_t& a, std::int64_t& b) {
    const auto c = s.find(',');
    if (c == std::string::npos) return false;
    try {
        a = std::stoll(s.substr(0, c)) * 1000;
        b = std::stoll(s.substr(c + 1)) * 1000;
    } catch (...) {
        return false;
    }
    return true;
}

} // namespace

int main(int argc, char** argv) {
    std::map<std::string, std::string> args;
    for (int i = 1; i + 1 < argc; i += 2) args[argv[i]] = argv[i + 1];
    if (!args.count("--store") || !args.count("--scope") || !args.count("--rules") || !args.count("--out")) {
        std::cerr << "usage: g2m_site_export --store DIR --scope SCOPE --rules FILE --out FILE [--region-min E,N --region-max E,N] [--pbf A,B --margin-km N]\n";
        return 1;
    }
    std::ifstream rin(args["--rules"]);
    if (!rin) { std::cerr << "cannot open rules\n"; return 1; }
    const json rules = json::parse(rin);
    if (rules.at("format").get<std::string>() != "g2m.site_export_rules/1") { std::cerr << "rules: wrong format\n"; return 1; }
    std::vector<std::pair<std::string, std::string>> tags, land;
    auto split = [](const std::string& s) { const auto e = s.find('='); return std::make_pair(s.substr(0, e), s.substr(e + 1)); };
    for (const json& t : rules.at("tags")) tags.push_back(split(t.get<std::string>()));
    for (const json& t : rules.at("industrial_land")) land.push_back(split(t.get<std::string>()));
    std::vector<std::string> words;
    for (const json& w : rules.at("name_words")) words.push_back(w.get<std::string>());
    const std::string road_key = rules.at("road_key").get<std::string>();

    TileStoreConfig config;
    config.dir = args["--store"];
    config.read_only = true;
    auto store = TileStore::open(config);
    if (!store) { std::cerr << "store: " << store.error().message << "\n"; return 1; }
    auto index = (*store)->index(args["--scope"]);
    if (!index) { std::cerr << "index: " << index.error().message << "\n"; return 1; }

    const geo::Utm utm;
    std::map<std::uint64_t, Feature> features;
    std::vector<Poly> polys;
    std::map<std::uint64_t, Industrial> industrial;   // keyed by the way id: the halo repeats ways
    std::int64_t min_e = INT64_MAX, min_n = INT64_MAX, max_e = INT64_MIN, max_n = INT64_MIN;
    std::size_t tiles = 0;
    geo::UtmZone out_zone{32, geo::Hemisphere::North};
    for (const auto& entry : *index) {
        if (entry.layer != kSrcOsmLayer) continue;
        auto stored = (*store)->get({args["--scope"], entry.layer, entry.key});
        if (!stored || !*stored) continue;
        auto body = decode_body((*stored)->body);
        if (!body) { std::cerr << "body: " << body.error().message << "\n"; return 1; }
        auto tile = decode_src_osm(*body);
        if (!tile) { std::cerr << "tile: " << tile.error().message << "\n"; return 1; }
        ++tiles;
        out_zone = entry.key.zone;
        const std::int64_t sz = entry.key.size_m() * 1000;
        min_e = std::min<std::int64_t>(min_e, entry.key.x * sz);
        min_n = std::min<std::int64_t>(min_n, entry.key.y * sz);
        max_e = std::max<std::int64_t>(max_e, (entry.key.x + 1) * sz);
        max_n = std::max<std::int64_t>(max_n, (entry.key.y + 1) * sz);
        const osm::OsmData& d = tile->data;
        const geo::UtmZone zone = entry.key.zone;
        std::map<std::int64_t, geom::PointMm> pos;
        for (const osm::Node& nd : d.nodes) {
            const auto p = geom::project_e7_to_mm(utm, zone, nd.lat, nd.lon);
            if (p) pos[nd.id] = *p;
        }
        auto tag_map = [&](std::span<const osm::Tag> ts, std::string& name) {
            std::map<std::string, std::string> m;
            for (const osm::Tag& t : ts) {
                m[std::string(d.str(t.key))] = std::string(d.str(t.value));
                if (d.str(t.key) == "name") name = std::string(d.str(t.value));
            }
            return m;
        };
        auto relevant = [&](const std::map<std::string, std::string>& m, const std::string& name) {
            for (const auto& [k, v] : tags) {
                const auto it = m.find(k);
                if (it != m.end() && it->second == v) return true;
            }
            const std::string ln = lower(name);
            for (const std::string& w : words)
                if (!w.empty() && ln.find(w) != std::string::npos) return true;
            return false;
        };
        for (const osm::Node& nd : d.nodes) {
            if (nd.tags_count == 0) continue;
            std::string name;
            auto m = tag_map(d.tags_of(nd), name);
            if (!relevant(m, name)) continue;
            const auto p = pos.find(nd.id);
            if (p == pos.end()) continue;
            Feature f;
            f.id = static_cast<std::uint64_t>(nd.id);
            f.tags = std::move(m);
            f.name = name;
            f.e = p->second.x;
            f.n = p->second.y;
            features.emplace(f.id, std::move(f));
        }
        std::vector<geom::PointMm> road;   // vertices of highway ways: the road distance of an industrial candidate
        for (const osm::Way& w : d.ways) {
            std::string name;
            const auto m = tag_map(d.tags_of(w), name);
            if (m.count(road_key))
                for (std::uint32_t i = 0; i < w.refs_count; ++i) {
                    const auto p = pos.find(d.way_refs[w.refs_begin + i]);
                    if (p != pos.end()) road.push_back(p->second);
                }
        }
        for (const osm::Way& w : d.ways) {
            if (w.tags_count == 0) continue;
            std::string name;
            auto m = tag_map(d.tags_of(w), name);
            if (const auto lu = m.find("landuse"); lu != m.end() && polygon_kind(lu->second) && w.refs_count >= 4 && d.way_refs[w.refs_begin] == d.way_refs[w.refs_begin + w.refs_count - 1]) {
                Poly pg;
                pg.kind = lu->second;
                for (std::uint32_t i = 0; i < w.refs_count; ++i)
                    if (const auto q = pos.find(d.way_refs[w.refs_begin + i]); q != pos.end()) pg.pts.emplace_back(q->second.x, q->second.y);
                if (pg.pts.size() >= 4) {
                    close_poly(pg);
                    polys.push_back(std::move(pg));
                }
            }
            const bool is_feature = relevant(m, name);
            bool is_land = false;
            for (const auto& [k, v] : land) {
                const auto it = m.find(k);
                if (it != m.end() && it->second == v) is_land = true;
            }
            if (!is_feature && !is_land) continue;
            long double se = 0, sn = 0;
            std::size_t n = 0;
            for (std::uint32_t i = 0; i < w.refs_count; ++i) {
                const auto p = pos.find(d.way_refs[w.refs_begin + i]);
                if (p == pos.end()) continue;
                se += p->second.x;
                sn += p->second.y;
                ++n;
            }
            if (n == 0) continue;
            const std::int64_t ce = static_cast<std::int64_t>(se / n), cn = static_cast<std::int64_t>(sn / n);
            const std::uint64_t id = static_cast<std::uint64_t>(w.id) | kWayBit;
            if (is_feature) {
                Feature f;
                f.id = id;
                f.tags = m;
                f.name = name;
                f.e = ce;
                f.n = cn;
                features.emplace(id, std::move(f));
            }
            if (is_land && !industrial.count(id)) {
                std::int64_t best = INT64_MAX;
                for (const geom::PointMm& r : road) {
                    const long double dx = static_cast<long double>(r.x - ce), dy = static_cast<long double>(r.y - cn);
                    best = std::min<std::int64_t>(best, static_cast<std::int64_t>(std::sqrt(dx * dx + dy * dy)));
                }
                industrial[id] = Industrial{ce, cn, best == INT64_MAX ? 1000000000 : best};
            }
        }
    }
    if (tiles == 0) { std::cerr << "no g2m.src.osm tiles in the scope\n"; return 1; }
    std::int64_t rmin_e = min_e, rmin_n = min_n, rmax_e = max_e, rmax_n = max_n;
    if (args.count("--region-min") && !get_pair(args["--region-min"], rmin_e, rmin_n)) { std::cerr << "bad --region-min\n"; return 1; }
    if (args.count("--region-max") && !get_pair(args["--region-max"], rmax_e, rmax_n)) { std::cerr << "bad --region-max\n"; return 1; }

    std::size_t margin_features = 0;
    if (args.count("--pbf")) {
        const std::int64_t margin_mm = (args.count("--margin-km") ? std::stoll(args["--margin-km"]) : 60) * 1000000;
        const std::int64_t lo_e = rmin_e - margin_mm, lo_n = rmin_n - margin_mm, hi_e = rmax_e + margin_mm, hi_n = rmax_n + margin_mm;
        std::vector<std::string> pbfs;
        {
            const std::string rest = args["--pbf"];
            for (std::size_t from = 0; from <= rest.size();) {
                const std::size_t c = rest.find(',', from);
                const std::string one = rest.substr(from, c == std::string::npos ? std::string::npos : c - from);
                if (!one.empty()) pbfs.push_back(one);
                if (c == std::string::npos) break;
                from = c + 1;
            }
        }
        const auto tags_of_pbf = [](const std::vector<decode::osm::Tag>& ts, std::string& name) {
            std::map<std::string, std::string> m;
            for (const auto& tg : ts) {
                m[tg.key] = tg.value;
                if (tg.key == "name") name = tg.value;
            }
            return m;
        };
        const auto relevant_map = [&](const std::map<std::string, std::string>& m, const std::string& name) {
            for (const auto& [k, v] : tags) {
                const auto it = m.find(k);
                if (it != m.end() && it->second == v) return true;
            }
            const std::string ln = lower(name);
            for (const std::string& w : words)
                if (!w.empty() && ln.find(w) != std::string::npos) return true;
            return false;
        };
        struct MarginWay {
            std::uint64_t id;
            std::map<std::string, std::string> tags;
            std::string name;
            std::vector<std::int64_t> refs;
        };
        struct LandWay {
            std::string kind;
            std::vector<std::int64_t> refs;
        };
        struct WayPass : decode::osm::PbfHandler {
            const decltype(tags_of_pbf)& tg;
            const decltype(relevant_map)& rel;
            std::vector<MarginWay>& out;
            std::set<std::int64_t>& need;
            std::vector<LandWay>& land;
            WayPass(const decltype(tags_of_pbf)& a, const decltype(relevant_map)& b, std::vector<MarginWay>& c, std::set<std::int64_t>& d, std::vector<LandWay>& e)
                : tg(a), rel(b), out(c), need(d), land(e) {}
            void on_way(const decode::osm::OsmWay& w) override {
                if (w.tags.empty()) return;
                std::string name;
                auto m = tg(w.tags, name);
                if (const auto lu = m.find("landuse"); lu != m.end() && polygon_kind(lu->second) && w.node_refs.size() >= 4 && w.node_refs.front() == w.node_refs.back()) {
                    for (std::int64_t r : w.node_refs) need.insert(r);
                    land.push_back(LandWay{lu->second, w.node_refs});
                }
                if (!rel(m, name)) return;
                for (std::int64_t r : w.node_refs) need.insert(r);
                out.push_back(MarginWay{static_cast<std::uint64_t>(w.id) | kWayBit, std::move(m), name, w.node_refs});
            }
        };
        struct NodePass : decode::osm::PbfHandler {
            const geo::Utm& utm;
            geo::UtmZone zone;
            const std::set<std::int64_t>& need;
            std::map<std::int64_t, geom::PointMm>& pos;
            const decltype(tags_of_pbf)& tg;
            const decltype(relevant_map)& rel;
            std::map<std::uint64_t, Feature>& feats;
            std::int64_t lo_e, lo_n, hi_e, hi_n;
            NodePass(const geo::Utm& u, geo::UtmZone z, const std::set<std::int64_t>& n, std::map<std::int64_t, geom::PointMm>& p, const decltype(tags_of_pbf)& a,
                     const decltype(relevant_map)& b, std::map<std::uint64_t, Feature>& f, std::int64_t e0, std::int64_t n0, std::int64_t e1, std::int64_t n1)
                : utm(u), zone(z), need(n), pos(p), tg(a), rel(b), feats(f), lo_e(e0), lo_n(n0), hi_e(e1), hi_n(n1) {}
            void on_node(const decode::osm::OsmNode& nd) override {
                const bool wanted = need.count(nd.id) != 0;
                const bool tagged = !nd.tags.empty();
                if (!wanted && !tagged) return;
                const auto p = geom::project_e7_to_mm(utm, zone, nd.lat_1e7, nd.lon_1e7);
                if (!p) return;
                if (wanted) pos[nd.id] = *p;
                if (!tagged || p->x < lo_e || p->x >= hi_e || p->y < lo_n || p->y >= hi_n) return;
                std::string name;
                auto m = tg(nd.tags, name);
                if (!rel(m, name)) return;
                Feature f;
                f.id = static_cast<std::uint64_t>(nd.id);
                f.tags = std::move(m);
                f.name = name;
                f.e = p->x;
                f.n = p->y;
                feats.emplace(f.id, std::move(f));
            }
        };
        const std::size_t before = features.size();
        for (const std::string& pbf : pbfs) {
            std::vector<MarginWay> ways;
            std::set<std::int64_t> need;
            std::vector<LandWay> lands;
            WayPass wp(tags_of_pbf, relevant_map, ways, need, lands);
            decode::osm::read_pbf(pbf, wp, nullptr, decode::osm::PbfEntities::Ways);
            std::map<std::int64_t, geom::PointMm> pos;
            NodePass np(utm, out_zone, need, pos, tags_of_pbf, relevant_map, features, lo_e, lo_n, hi_e, hi_n);
            decode::osm::read_pbf(pbf, np, nullptr, decode::osm::PbfEntities::Nodes);
            for (const LandWay& lw : lands) {
                Poly pg;
                pg.kind = lw.kind;
                for (std::int64_t r : lw.refs)
                    if (const auto q = pos.find(r); q != pos.end()) pg.pts.emplace_back(q->second.x, q->second.y);
                if (pg.pts.size() >= 4) {
                    close_poly(pg);
                    polys.push_back(std::move(pg));
                }
            }
            for (const MarginWay& w : ways) {
                long double se = 0, sn = 0;
                std::size_t n = 0;
                for (std::int64_t r : w.refs) {
                    const auto p = pos.find(r);
                    if (p == pos.end()) continue;
                    se += p->second.x;
                    sn += p->second.y;
                    ++n;
                }
                if (n == 0) continue;
                const std::int64_t ce = static_cast<std::int64_t>(se / n), cn = static_cast<std::int64_t>(sn / n);
                if (ce < lo_e || ce >= hi_e || cn < lo_n || cn >= hi_n) continue;
                Feature f;
                f.id = w.id;
                f.tags = w.tags;
                f.name = w.name;
                f.e = ce;
                f.n = cn;
                features.emplace(f.id, std::move(f));
            }
            std::cerr << "g2m_site_export: margin pass over " << pbf << ": " << features.size() - before << " features so far\n";
        }
        margin_features = features.size() - before;
    }

    // g2m:landuse of every feature: farmyard when inside a farmyard polygon, else the first industrial/commercial/retail/railway polygon it lies in, else none.
    // A relation (multipolygon) landuse is not read; a feature near a PBF border may miss a polygon the neighbour file holds.
    for (auto& [id, f] : features) {
        std::string kind = "none";
        for (const Poly& g : polys) {
            if (!inside(g, f.e, f.n)) continue;
            if (g.kind == "farmyard") {
                kind = "farmyard";
                break;
            }
            if (kind == "none") kind = g.kind;
        }
        f.tags["g2m:landuse"] = kind;
    }

    json fa = json::array(), ia = json::array();
    for (const auto& [id, f] : features) {
        json t = json::object();
        for (const auto& [k, v] : f.tags) t[k] = v;
        fa.push_back(json{{"id", id}, {"tags", t}, {"name", f.name}, {"e_mm", f.e}, {"n_mm", f.n}});
    }
    for (const auto& [id, c] : industrial) ia.push_back(json{{"e_mm", c.e}, {"n_mm", c.n}, {"road_mm", c.road}});
    const json out{{"format", "farm.site_input/1"},
                   {"region", json{{"min", json::array({rmin_e, rmin_n})}, {"max", json::array({rmax_e, rmax_n})}}},
                   {"zone", out_zone.zone},
                   {"north", out_zone.hemisphere == geo::Hemisphere::North},
                   {"features", fa},
                   {"industrial", ia}};
    std::ofstream o(args["--out"], std::ios::binary | std::ios::trunc);
    o << out.dump();
    std::cerr << "g2m_site_export: " << tiles << " tiles, " << features.size() << " features, " << industrial.size() << " industrial candidates, " << margin_features << " from the PBF margin\n";
    return 0;
}
