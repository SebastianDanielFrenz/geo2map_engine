// apps/g2m_server/main.cpp - the sidecar map server (G6, PLAN.md 3.1): one
// local release over a source store, served as protocol v1 over HTTP.
//
//   g2m_server --source DIR --scope SCOPE [--source-name NAME]
//              [--derived DIR] [--derived-name NAME]
//              [--layers terrain|roads|geometry|carved] [--fields] [--soil] [--world-seed N]
//              [--bind ADDR] [--port N] [--base-path /P] [--threads N]
//              [--server-id ID] [--exit-on-stdin-eof]
//
// The release is built exactly like the game's integrated server builds it
// (builtin_local_release_params, the same opt-ins as
// g2m::world::TerrainService's LayerOptIn; --layers geometry = its default
// roads + geometry), so the same store, scope, opt-ins and world seed give
// the same release id. The source store is opened read-only; derived tiles
// are cached in --derived when given (else derived again per request).
//
// Offline by construction: ServerConfig::offline, no fetcher, and the only
// upstream is the local source store. Binds 127.0.0.1 by default; --port 0
// (the default) picks a free port. Once serving, prints ONE line to stdout:
//   G2M_SERVER_READY url=http://127.0.0.1:<port><base>/g2m/v1 port=<port> rid=<64 hex>
// and runs until SIGINT/SIGTERM, or until stdin reaches EOF with
// --exit-on-stdin-eof (a parent process that dies takes the sidecar along).
// Exit codes: 0 after a clean stop, 1 on a start-up error, 2 on bad usage.

#include "g2m/client/in_process_transport.h"
#include "g2m/http/http_server.h"
#include "g2m/http/http_wire.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/server.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"

#include "cli_args.h"

#include <algorithm>
#include <atomic>
#include <charconv>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <iostream>
#include <optional>
#include <string>
#include <thread>

namespace {

volatile std::sig_atomic_t g_signal = 0;
std::atomic<bool> g_stdin_eof{false};

void on_signal(int sig) { g_signal = sig; }

template <class T>
std::optional<T> parse_int(const std::string& s) {
    T v{};
    const auto [end, ec] = std::from_chars(s.data(), s.data() + s.size(), v);
    if (s.empty() || ec != std::errc{} || end != s.data() + s.size()) return std::nullopt;
    return v;
}

int usage(const std::string& why) {
    std::cerr << "g2m_server: " << why << "\n"
              << "usage: g2m_server --source DIR --scope SCOPE [--source-name NAME] [--derived DIR] [--derived-name NAME]\n"
                 "                  [--layers terrain|roads|geometry|carved] [--fields] [--soil] [--world-seed N] [--bind ADDR] [--port N]\n"
                 "                  [--base-path /P] [--threads N] [--server-id ID] [--exit-on-stdin-eof]\n";
    return 2;
}

} // namespace

int main(int argc, char** argv) {
    const std::vector<std::string> bool_flags{"exit-on-stdin-eof", "help", "fields", "soil"};
    const auto args = g2m_tiler::parse_args(argc, argv, 1, bool_flags);
    if (args.flag("help")) return usage("help");
    static const char* const kKnown[] = {"source", "scope", "source-name", "derived", "derived-name", "layers", "world-seed",
                                         "bind",   "port",  "base-path",   "threads", "server-id",    "exit-on-stdin-eof", "fields", "soil"};
    for (const auto& [k, v] : args.flags) {
        if (std::find(std::begin(kKnown), std::end(kKnown), k) == std::end(kKnown)) return usage("unknown option --" + k);
    }
    if (!args.positional.empty()) return usage("unexpected argument '" + args.positional.front() + "'");
    const auto source = args.get("source");
    const auto scope = args.get("scope");
    if (!source || source->empty() || !scope || scope->empty()) return usage("--source and --scope are required");

    const std::string layers = args.get("layers").value_or("geometry");
    bool roads = false, geometry = false, carved = false;
    if (layers == "terrain") {
    } else if (layers == "roads") {
        roads = true;
    } else if (layers == "geometry") {
        roads = geometry = true;
    } else if (layers == "carved") {
        roads = geometry = carved = true;
    } else {
        return usage("--layers must be terrain, roads, geometry or carved");
    }
    const auto world_seed = parse_int<std::uint64_t>(args.get("world-seed").value_or("0"));
    if (!world_seed) return usage("--world-seed must be an unsigned integer");

    g2m::http::HttpServerConfig hc;
    hc.bind_address = args.get("bind").value_or("127.0.0.1");
    const auto port = parse_int<int>(args.get("port").value_or("0"));
    if (!port || *port < 0 || *port > 65535) return usage("--port must be 0..65535");
    hc.port = *port;
    hc.base_path = args.get("base-path").value_or("");
    if (!g2m::http::is_valid_base_path(hc.base_path)) return usage("--base-path must be \"\" or /seg[/seg...]");
    const auto threads = parse_int<std::size_t>(args.get("threads").value_or("8"));
    if (!threads || *threads == 0 || *threads > 256) return usage("--threads must be 1..256");
    hc.threads = *threads;

    // Stores: the source read-only (never written), the derived cache read-write.
    g2m::TileStoreConfig sc;
    sc.dir = *source;
    sc.name = args.get("source-name").value_or("tiles");
    sc.read_only = true;
    auto sources = g2m::TileStore::open(sc);
    if (!sources) {
        std::cerr << "g2m_server: --source " << *source << ": " << sources.error().message << "\n";
        return 1;
    }
    std::unique_ptr<g2m::TileStore> derived;
    if (const auto d = args.get("derived")) {
        g2m::TileStoreConfig dc;
        dc.dir = *d;
        dc.name = args.get("derived-name").value_or("tiles");
        auto r = g2m::TileStore::open(dc);
        if (!r) {
            std::cerr << "g2m_server: --derived " << *d << ": " << r.error().message << "\n";
            return 1;
        }
        derived = std::move(*r);
    }

    g2m::BuiltinTerrainDerivers derivers;
    g2m::LocalReleaseParams params = g2m::builtin_local_release_params(*scope, derivers, {}, roads, geometry, carved, /*include_classes=*/false, /*include_rail=*/false, args.flag("fields"), args.flag("soil"));
    params.world_seed = *world_seed;
    auto manifest = g2m::make_local_release(**sources, params);
    if (!manifest) {
        std::cerr << "g2m_server: release: " << manifest.error().message << "\n";
        return 1;
    }

    g2m::LocalSourceUpstream upstream(**sources);
    g2m::ServerConfig sv;
    sv.server_id = args.get("server-id").value_or("g2m.sidecar");
    sv.offline = true;        // structural: the server keeps no fetcher
    sv.fetcher = nullptr;
    sv.derived_store = derived.get();
    sv.derivers = params.derivers;
    sv.upstreams = {&upstream};
    g2m::Server server(sv);
    const std::string rid_hex = g2m::to_hex(manifest->rid);
    if (auto ok = server.add_release(std::move(*manifest), *scope); !ok) {
        std::cerr << "g2m_server: add_release: " << ok.error().message << "\n";
        return 1;
    }
    g2m::InProcessTransport backend(server);

    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);
    auto http = g2m::http::HttpServer::start(backend, hc);
    if (!http) {
        std::cerr << "g2m_server: " << http.error().message << "\n";
        return 1;
    }
    std::cout << "G2M_SERVER_READY url=" << (*http)->base_url() << g2m::http::kApiPath << " port=" << (*http)->port()
              << " rid=" << rid_hex << std::endl;
    if (hc.bind_address != "127.0.0.1" && hc.bind_address != "::1" && hc.bind_address != "localhost") {
        std::cerr << "g2m_server: warning: bound to " << hc.bind_address
                  << " - no TLS, authentication or rate limiting; put a reverse proxy in front (PLAN.md 3.7)\n";
    }

    if (args.flag("exit-on-stdin-eof")) {
        // Detached: a blocking read cannot be interrupted portably; the process exits around it.
        std::thread([] {
            char buf[256];
            while (std::fread(buf, 1, sizeof buf, stdin) > 0) {
            }
            g_stdin_eof.store(true);
        }).detach();
    }
    while (g_signal == 0 && !g_stdin_eof.load()) std::this_thread::sleep_for(std::chrono::milliseconds(50));
    (*http)->stop();
    const auto st = (*http)->stats();
    std::cerr << "g2m_server: stopped (" << st.requests << " requests, " << st.rejected << " rejected)\n";
    return 0;
}
