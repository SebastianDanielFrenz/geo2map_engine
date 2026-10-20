// g2m_restriction_probe STORE SCOPE [--derive]
//
// Read-only census of turn restrictions over the g2m.src.osm tiles of an immutable source store:
// per relation the number of via ways, the relations above kRoadGraphMaxViaWays (deduplicated across
// the tiles that carry a copy) and, with --derive, the roads.graph derivation outcome of every
// src.osm tile (so the tile that fails with "restriction via ways" is named). Writes nothing.
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/src_osm.h"
#include "g2m/server/tile_store.h"
#include <algorithm>
#include <iostream>
#include <map>
#include <set>
#include <string_view>
using namespace g2m;
int main(int argc, char** argv) {
    if (argc < 3 || argc > 4 || (argc == 4 && std::string_view(argv[3]) != "--derive")) {
        std::cerr << "usage: g2m_restriction_probe STORE SCOPE [--derive]\n"; return 1;
    }
    const bool derive = argc == 4;
    TileStoreConfig config; config.dir = argv[1]; config.read_only = true;
    auto store = TileStore::open(config); if (!store) { std::cerr << store.error().message << '\n'; return 1; }
    const std::string scope = argv[2];
    auto index = (*store)->index(scope); if (!index) { std::cerr << index.error().message << '\n'; return 1; }
    std::set<TileKey> osm_keys;
    for (const auto& e : *index) if (e.layer == kSrcOsmLayer) osm_keys.insert(e.key);
    std::cout << "src.osm tiles: " << osm_keys.size() << '\n';
    // Census: relation id -> (via ways, tiles carrying it)
    std::map<std::int64_t, std::pair<std::size_t, std::set<TileKey>>> over;
    std::map<std::size_t, std::size_t> histogram; // via ways -> distinct relations
    std::set<std::int64_t> seen_relation;
    std::size_t rules_total = 0, via_way_relations = 0, relations_seen = 0;
    for (const auto& key : osm_keys) {
        auto tile = (*store)->get({scope, std::string(kSrcOsmLayer), key});
        if (!tile || !*tile) { std::cerr << "cannot read " << to_string(key) << '\n'; return 1; }
        auto body = decode_body((*tile)->body); if (!body) { std::cerr << body.error().message << '\n'; return 1; }
        auto osm_tile = decode_src_osm(*body); if (!osm_tile) { std::cerr << osm_tile.error().message << '\n'; return 1; }
        auto found = extract_turn_restrictions(osm_tile->data); if (!found) { std::cerr << found.error().message << '\n'; return 1; }
        relations_seen += found->relations_seen;
        for (const auto& r : found->restrictions) {
            if (r.via_ways.empty()) continue;
            if (seen_relation.insert(r.relation_id).second) {
                ++via_way_relations; ++histogram[r.via_ways.size()];
            }
            ++rules_total;
            if (r.via_ways.size() > kRoadGraphMaxViaWays) {
                auto& entry = over[r.relation_id]; entry.first = r.via_ways.size(); entry.second.insert(key);
                if (entry.second.size() == 1) { // describe it once, from the first tile copy
                    const auto& d = osm_tile->data;
                    if (const auto* rel = d.find_relation(r.relation_id)) {
                        std::cout << "relation " << rel->id << " tags:";
                        for (const auto& t : d.tags_of(*rel)) std::cout << ' ' << d.str(t.key) << '=' << d.str(t.value);
                        std::cout << "\n";
                        for (const auto& m : d.members_of(*rel)) {
                            std::cout << "  " << (m.type == osm::MemberType::Way ? "way " : m.type == osm::MemberType::Node ? "node " : "rel ")
                                      << m.ref << " role=" << d.str(m.role);
                            if (const auto* w = d.find_way(m.ref); w && m.type == osm::MemberType::Way) {
                                for (const auto& t : d.tags_of(*w)) {
                                    const auto k = d.str(t.key);
                                    if (k == "highway" || k == "name" || k == "tunnel" || k == "ref" || k == "oneway") std::cout << ' ' << k << '=' << d.str(t.value);
                                }
                            } else if (m.type == osm::MemberType::Way) std::cout << " (way not in this tile)";
                            std::cout << "\n";
                        }
                    }
                }
            }
        }
    }
    std::cout << "relation copies seen: " << relations_seen << ", via-way rule copies: " << rules_total << ", via-way relations (distinct): " << via_way_relations << "\n";
    for (const auto& [n, c] : histogram) std::cout << "  via_ways=" << n << ": " << c << " relations\n";
    std::cout << "relations over kRoadGraphMaxViaWays (" << kRoadGraphMaxViaWays << "): " << over.size() << '\n';
    for (const auto& [id, v] : over) {
        std::cout << "  relation " << id << " via_ways=" << v.first << " in src.osm tiles:";
        for (const auto& k : v.second) std::cout << ' ' << to_string(k);
        std::cout << '\n';
    }
    if (!derive) return 0;
    RoadGraphDeriver deriver; std::size_t ok = 0, failed = 0;
    std::map<std::string, std::size_t> reasons;
    std::map<std::string, std::uint64_t> diagnostics;
    for (const auto& key : osm_keys) {
        std::vector<InputTile> inputs;
        for (const auto& ref : deriver.inputs_for(key)) {
            auto tile = (*store)->get({scope, ref.layer, ref.key});
            if (!tile) { std::cerr << tile.error().message << '\n'; return 1; }
            InputTile input; input.ref = ref;
            if (*tile) { input.header = std::move((*tile)->header); input.body = std::move((*tile)->body); }
            inputs.push_back(std::move(input));
        }
        const auto derived = deriver.derive({key, 0, inputs});
        if (derived) {
            ++ok;
            for (const auto& [name, count] : derived->diagnostics) {
                std::cout << "derived " << to_string(key) << ": " << name << '=' << count << "\n";
                diagnostics[name] += count;
            }
            continue;
        }
        ++failed; ++reasons[derived.error().message];
        std::cout << "DERIVE FAIL " << to_string(key) << ": " << derived.error().message << '\n';
    }
    std::cout << "derive ok=" << ok << " failed=" << failed << '\n';
    for (const auto& [name, count] : diagnostics) std::cout << "  diagnostic " << name << " (summed over tiles) = " << count << "\n";
    for (const auto& [m, c] : reasons) std::cout << "  " << c << " x " << m << '\n';
    return failed ? 2 : 0;
}
