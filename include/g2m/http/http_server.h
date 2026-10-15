// g2m/http/http_server.h - HttpServer: the HTTP front end of a g2m server
// (G6; the sidecar apps/g2m_server, and later the hosted server behind a
// reverse proxy, PLAN.md 3.1/3.7). It answers protocol v1 GET/HEAD requests
// under {base_path}/g2m/v1 by handing them to an ITransport - normally an
// InProcessTransport over a g2m::Server - and serialising the answer
// (http_wire.h, docs/protocol/README.md). Nothing else is served.
//
// Binds 127.0.0.1 by default: a sidecar is local. Binding another address is
// allowed (a hosted server behind a proxy) but the front end itself has no
// TLS, authentication or rate limiting (PLAN.md 3.7: the proxy's job, G7).
//
// Thread safety: requests are served on a fixed pool of `threads` threads;
// the backend must allow concurrent send() (ITransport's contract). stop()
// may be called from any thread except a request handler; the destructor
// stops.
#pragma once

#include "g2m/core/result.h"
#include "g2m/server/transport.h"

#include <cstdint>
#include <memory>
#include <string>

namespace g2m::http {

struct HttpServerConfig {
    std::string bind_address = "127.0.0.1";
    int port = 0;                       // 0: any free port (see HttpServer::port())
    std::string base_path;              // "" or "/a/b" (is_valid_base_path)
    std::size_t threads = 8;            // worker threads (one connection each at a time)
    std::size_t max_queued_connections = 256; // beyond this, new connections are closed at once
    std::uint32_t read_timeout_ms = 10000;    // per request read (also the keep-alive idle limit)
    std::uint32_t write_timeout_ms = 60000;
    std::size_t keep_alive_max_requests = 1000;
};

struct HttpServerStats {
    std::uint64_t requests = 0;       // every answered request
    std::uint64_t rejected = 0;       // answered by the front end itself (400/404/405) without the backend
};

class HttpServer {
public:
    // Binds and starts serving on a background thread. Fails (by value) when
    // the config is invalid or the address/port cannot be bound.
    static Result<std::unique_ptr<HttpServer>> start(ITransport& backend, HttpServerConfig config);
    ~HttpServer();
    HttpServer(const HttpServer&) = delete;
    HttpServer& operator=(const HttpServer&) = delete;

    int port() const;                  // the bound port
    std::string base_url() const;      // "http://<bind>:<port><base_path>", the HttpTransport base_url
    HttpServerStats stats() const;
    void stop();                       // idempotent; joins the serving threads

private:
    struct Impl;
    explicit HttpServer(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

} // namespace g2m::http
