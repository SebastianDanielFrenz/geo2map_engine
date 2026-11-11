// g2m_site_export - the map features a farming game places buyers and supply stores on (docs/pipeline/site_export.md).
//
//   g2m_site_export --store DIR --scope SCOPE --rules data/site_export/de.json --out sites.json [--region-min E,N --region-max E,N]
//
// Reads every g2m.src.osm tile of the scope (read only) and writes "farm.site_input/1": one record per OSM node or way carrying a listed tag or a
// listed word in its name (id, ALL tags, name, position in integer UTM millimetres of the bake's zone; a way takes the mean of its nodes), plus
// industrial land (landuse=industrial ways: mean position and the distance to the nearest highway vertex). Ids: a node keeps its OSM id, a way's is
// its id with bit 60 set (a node and a way never collide; the top bit stays free for procedural sites). Tile halos repeat features: they are
// kept once. The region is the bounding box of the scope's tiles (metres in the flags) unless given. Output is sorted by id and deterministic.
#include "g2m/core/geo/utm.h"
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
        std::cerr << "usage: g2m_site_export --store DIR --scope SCOPE --rules FILE --out FILE [--region-min E,N --region-max E,N]\n";
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
    std::map<std::uint64_t, Industrial> industrial;   // keyed by the way id: the halo repeats ways
    std::int64_t min_e = INT64_MAX, min_n = INT64_MAX, max_e = INT64_MIN, max_n = INT64_MIN;
    std::size_t tiles = 0;
    for (const auto& entry : *index) {
        if (entry.layer != kSrcOsmLayer) continue;
        auto stored = (*store)->get({args["--scope"], entry.layer, entry.key});
        if (!stored || !*stored) continue;
        auto body = decode_body((*stored)->body);
        if (!body) { std::cerr << "body: " << body.error().message << "\n"; return 1; }
        auto tile = decode_src_osm(*body);
        if (!tile) { std::cerr << "tile: " << tile.error().message << "\n"; return 1; }
        ++tiles;
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

    json fa = json::array(), ia = json::array();
    for (const auto& [id, f] : features) {
        json t = json::object();
        for (const auto& [k, v] : f.tags) t[k] = v;
        fa.push_back(json{{"id", id}, {"tags", t}, {"name", f.name}, {"e_mm", f.e}, {"n_mm", f.n}});
    }
    for (const auto& [id, c] : industrial) ia.push_back(json{{"e_mm", c.e}, {"n_mm", c.n}, {"road_mm", c.road}});
    const json out{{"format", "farm.site_input/1"},
                   {"region", json{{"min", json::array({rmin_e, rmin_n})}, {"max", json::array({rmax_e, rmax_n})}}},
                   {"features", fa},
                   {"industrial", ia}};
    std::ofstream o(args["--out"], std::ios::binary | std::ios::trunc);
    o << out.dump();
    std::cerr << "g2m_site_export: " << tiles << " tiles, " << features.size() << " features, " << industrial.size() << " industrial candidates\n";
    return 0;
}
