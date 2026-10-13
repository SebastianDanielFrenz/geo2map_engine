// Diagnostic dump of the served junction rings and approach edges of the committed OSM fixtures, for visual review
// (tools/junction_view/render.py draws them). Hidden: run `g2m_layer_tests "[.][diag][junction_dump]"` with
// G2M_JUNCTION_DUMP set to an output directory; one JSON file per fixture.
#include "road_fixture.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_junction.h"
#include <catch2/catch_test_macros.hpp>
#include <cstdio>
#include <cstdlib>
#include <set>
#include <string>
using namespace g2m;

namespace {
void dump_fixture(const std::string& pbf, const std::string& out_path) {
    const auto data = g2m_test::read_osm_fixture(std::string(G2M_SOURCE_DIR) + "/testdata/fixtures/osm/" + pbf);
    const auto world = build_road_graph(data, g2m_test::kZone32N); REQUIRE(world);
    std::set<TileKey> keys;
    for (const auto& n : world->nodes) {
        const auto k = tile_key_at(g2m_test::kZone32N, 2, static_cast<double>(n.position.x) / 1000.0, static_cast<double>(n.position.y) / 1000.0);
        if (k) keys.insert(*k);
    }
    std::FILE* f = std::fopen(out_path.c_str(), "w"); REQUIRE(f);
    std::fprintf(f, "{\"junctions\":[\n");
    bool first_j = true;
    std::set<std::pair<std::int64_t, std::uint32_t>> seen_entries;
    std::string entries_json;
    for (const auto& key : keys) {
        const auto probe = select_road_graph_tile(*world, key);
        if (!probe || probe->owned_edges.empty()) continue;
        auto g = g2m_test::served_geometry_in_tile(*world, key); REQUIRE(g);
        for (const auto& j : g->junctions) {
            if (j.decline != JunctionDecline::None) continue;
            std::fprintf(f, "%s{\"nodes\":[", first_j ? "" : ",\n"); first_j = false;
            for (std::size_t i = 0; i < j.nodes.size(); ++i) std::fprintf(f, "%s%lld", i ? "," : "", static_cast<long long>(j.nodes[i]));
            std::fprintf(f, "],\"flags\":%u,\"approaches\":[", j.flags);
            bool fa = true;
            for (const auto& a : j.approaches) {
                if (a.status != JunctionApproachStatus::Used) continue;
                const auto& e = g->entries[a.entry_index];
                std::fprintf(f, "%s{\"way\":%lld,\"trim\":%.3f}", fa ? "" : ",", static_cast<long long>(e.way_id), a.trim_m); fa = false;
            }
            std::fprintf(f, "],\"rings\":[");
            for (std::size_t r = 0; r < 3; ++r) {
                std::fprintf(f, "%s[", r ? "," : "");
                const auto& v = j.rings[r].vertices;
                for (std::size_t i = 0; i < v.size(); ++i) std::fprintf(f, "%s[%lld,%lld]", i ? "," : "", static_cast<long long>(v[i].x), static_cast<long long>(v[i].y));
                std::fprintf(f, "]");
            }
            std::fprintf(f, "]}");
        }
        for (const auto& e : g->entries) {
            if (!e.profile || !e.cross_section) continue;
            if (!seen_entries.insert({e.way_id, e.stretch.start_ref}).second) continue;
            const double L = e.profile->reference.length_m;
            std::string hw;
            for (const auto& w : g->source.graph.ways) if (w.osm_id == e.way_id) hw = g2m_test::way_tag(w, "highway");
            char buf[128];
            std::snprintf(buf, sizeof buf, "%s{\"way\":%lld,\"highway\":\"%s\",\"edges\":[", entries_json.empty() ? "" : ",\n",
                static_cast<long long>(e.way_id), hw.c_str());
            entries_json += buf;
            const int n = std::max(2, static_cast<int>(L / 1.0) + 1);
            for (int side = 0; side < 4; ++side) {   // roadbed left/right, paved left/right
                entries_json += side ? ",[" : "[";
                for (int k = 0; k < n; ++k) {
                    const double s = L * k / (n - 1);
                    const auto fp = carve_footprint(*e.cross_section, s);
                    const double t = side == 0 ? fp.roadbed_left : side == 1 ? fp.roadbed_right : side == 2 ? fp.paved_left : fp.paved_right;
                    const auto p = lateral_point(e.profile->reference, s, t);
                    if (!p) continue;
                    std::snprintf(buf, sizeof buf, "%s[%.3f,%.3f]", k ? "," : "", p->x, p->y);
                    entries_json += buf;
                }
                entries_json += "]";
            }
            entries_json += "]}";
        }
    }
    std::fprintf(f, "\n],\"entries\":[\n%s\n]}\n", entries_json.c_str());
    std::fclose(f);
}
}

TEST_CASE("Diag: dump the fixtures' served junction rings and approach edges", "[.][diag][junction_dump]") {
    const char* dir = std::getenv("G2M_JUNCTION_DUMP");
    if (!dir) SKIP("G2M_JUNCTION_DUMP not set");
    for (const char* pbf : {"frankfurt_hoechst.osm.pbf", "hoechst_a66.osm.pbf"}) dump_fixture(pbf, std::string(dir) + "/" + pbf + ".json");
}
