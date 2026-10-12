// Cold-store throughput of g2m::world::TerrainService against the plain serial server (terrain migration
// plan phase 2, vault G2M-020). NOT registered with CTest: a measurement tool, run by hand.
//
//   g2m_terrain_service_bench --source DIR [--source-name tiles] --scope SCOPE --derived DIR
//       --zone 32 --e0 464000 --n0 5559000 --tiles x,y[;x,y...] | --center x,y (3 x 3 level-2 tiles)
//       --mode serial|pool|warm [--workers N] [--layer g2m.roads.geom] [--heights]
//
//   serial  one thread calls g2m::Server::tile() tile after tile (what the game's geometry_mutex_ does)
//   pool    TerrainService with N workers, every tile requested at once as a Gate-priority ticket
//   warm    TerrainService::warm() over the same tile set (window 2 x workers)
// --derived must be an EMPTY directory for a cold run. The source store is opened read-only and never written.
#include "g2m/client/in_process_transport.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"
#include "g2m/world/terrain_service.h"

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

using namespace g2m;
using namespace g2m::world;

namespace {

struct Args {
    std::string source, source_name = "tiles", scope, derived, mode = "pool", layer = std::string(kRoadGeomLayer);
    int zone = 32, workers = 0;
    long long e0 = 0, n0 = 0;
    std::vector<std::pair<int, int>> tiles;
    bool heights = false;
};

bool parse_xy(const char* s, std::pair<int, int>& out) {
    char* end = nullptr;
    out.first = static_cast<int>(std::strtol(s, &end, 10));
    if (end == s || *end != ',') return false;
    const char* t = end + 1;
    out.second = static_cast<int>(std::strtol(t, &end, 10));
    return end != t && *end == 0;
}

bool parse(int argc, char** argv, Args& a) {
    for (int i = 1; i < argc; ++i) {
        const std::string k = argv[i];
        auto next = [&]() -> const char* { return i + 1 < argc ? argv[++i] : nullptr; };
        const char* v = nullptr;
        if (k == "--heights") {
            a.heights = true;
        } else if (!(v = next())) {
            return false;
        } else if (k == "--source") a.source = v;
        else if (k == "--source-name") a.source_name = v;
        else if (k == "--scope") a.scope = v;
        else if (k == "--derived") a.derived = v;
        else if (k == "--mode") a.mode = v;
        else if (k == "--layer") a.layer = v;
        else if (k == "--zone") a.zone = std::atoi(v);
        else if (k == "--workers") a.workers = std::atoi(v);
        else if (k == "--e0") a.e0 = std::atoll(v);
        else if (k == "--n0") a.n0 = std::atoll(v);
        else if (k == "--center") {
            std::pair<int, int> c;
            if (!parse_xy(v, c)) return false;
            for (int dy = -1; dy <= 1; ++dy)
                for (int dx = -1; dx <= 1; ++dx) a.tiles.emplace_back(c.first + dx, c.second + dy);
        } else if (k == "--tiles") {
            std::string s = v;
            std::size_t p = 0;
            while (p < s.size()) {
                const std::size_t q = s.find(';', p);
                std::pair<int, int> t;
                if (!parse_xy(s.substr(p, q == std::string::npos ? std::string::npos : q - p).c_str(), t)) return false;
                a.tiles.push_back(t);
                if (q == std::string::npos) break;
                p = q + 1;
            }
        } else return false;
    }
    return !a.source.empty() && !a.scope.empty() && !a.derived.empty() && !a.tiles.empty();
}

double seconds_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
}

} // namespace

int main(int argc, char** argv) {
    Args a;
    if (!parse(argc, argv, a)) {
        std::fprintf(stderr, "usage: see the header comment of tests/world/bench_terrain_service.cpp\n");
        return 2;
    }
    const geo::UtmZone zone{a.zone, geo::Hemisphere::North};
    std::vector<WarmItem> items;
    for (const auto& [x, y] : a.tiles) items.push_back({a.layer, TileKey{zone, kRawTileLevel, x, y}});
    if (a.heights)
        for (const auto& [x, y] : a.tiles)
            for (int dy = 0; dy < 4; ++dy)
                for (int dx = 0; dx < 4; ++dx) items.push_back({std::string(kTerrainHeightLayer), TileKey{zone, 0, x * 4 + dx, y * 4 + dy}});

    int ok = 0, absent = 0, failed = 0;
    double wall = 0;
    std::string rid_hex;
    int workers = a.workers;

    if (a.mode == "serial") {
        TileStoreConfig sc;
        sc.dir = a.source;
        sc.name = a.source_name;
        sc.read_only = true;
        auto sources = TileStore::open(sc);
        if (!sources) { std::fprintf(stderr, "source: %s\n", sources.error().message.c_str()); return 1; }
        auto derived = TileStore::open({a.derived, "tiles"});
        if (!derived) { std::fprintf(stderr, "derived: %s\n", derived.error().message.c_str()); return 1; }
        BuiltinTerrainDerivers derivers;
        const auto manifest = make_local_release(**sources, builtin_local_release_params(a.scope, derivers, {}, true, true));
        if (!manifest) { std::fprintf(stderr, "release: %s\n", manifest.error().message.c_str()); return 1; }
        LocalSourceUpstream upstream(**sources);
        ServerConfig cfg;
        cfg.offline = true;
        cfg.derived_store = derived->get();
        cfg.derivers = derivers.ordered(true, true);
        cfg.upstreams = {&upstream};
        Server server(cfg);
        if (!server.add_release(*manifest, a.scope)) return 1;
        rid_hex = to_hex(manifest->rid);
        workers = 1;
        const auto t0 = std::chrono::steady_clock::now();
        for (const WarmItem& it : items) {
            const TileResponse r = server.tile({manifest->rid, it.layer, it.key, std::nullopt});
            if (r.meta.status == Status::Ok) ++ok;
            else if (r.meta.status == Status::NotFound) ++absent;
            else ++failed;
        }
        wall = seconds_since(t0);
    } else {
        TerrainServiceConfig cfg;
        cfg.stores.source_dir = a.source;
        cfg.stores.source_name = a.source_name;
        cfg.stores.source_scope = a.scope;
        cfg.stores.source_read_only = true;
        cfg.stores.derived_dir = a.derived;
        cfg.layers.roads = true;
        cfg.layers.geometry = true;
        cfg.zone = zone;
        cfg.e0_m = a.e0;
        cfg.n0_m = a.n0;
        cfg.derive_workers = a.workers;
        auto svc = TerrainService::open(cfg);
        if (!svc) { std::fprintf(stderr, "open: %s\n", svc.error().message.c_str()); return 1; }
        rid_hex = (*svc)->release_id_hex();
        workers = (*svc)->worker_count();
        const auto t0 = std::chrono::steady_clock::now();
        if (a.mode == "warm") {
            WarmRequest req;
            req.items = items;
            req.priority = Priority::Gate;
            WarmJob job = (*svc)->warm(req);
            job.wait_for(std::chrono::hours(2));
            const WarmProgress p = job.progress();
            ok = static_cast<int>(p.ready);
            absent = static_cast<int>(p.absent);
            failed = static_cast<int>(p.failed + p.cancelled);
        } else {
            std::vector<Ticket<RawTile>> tickets;
            for (const WarmItem& it : items) tickets.push_back((*svc)->raw_tile(it.layer, it.key, Priority::Gate));
            for (auto& t : tickets) {
                t.wait_for(std::chrono::hours(2));
                switch (t.state()) {
                    case TileState::Ready: ++ok; break;
                    case TileState::Absent: ++absent; break;
                    default: ++failed; break;
                }
            }
        }
        wall = seconds_since(t0);
        const ServiceStats st = (*svc)->stats();
        std::printf("service: jobs_run=%llu joined=%llu inline=%llu peak_running=%d\n",
                    static_cast<unsigned long long>(st.jobs_run), static_cast<unsigned long long>(st.joined),
                    static_cast<unsigned long long>(st.inline_runs), st.peak_running);
    }
    std::printf("mode=%s workers=%d items=%zu ready=%d absent=%d failed=%d wall_s=%.3f per_item_s=%.3f rid=%s\n",
                a.mode.c_str(), workers, items.size(), ok, absent, failed, wall, wall / static_cast<double>(items.size()),
                rid_hex.c_str());
    return failed == 0 ? 0 : 1;
}
