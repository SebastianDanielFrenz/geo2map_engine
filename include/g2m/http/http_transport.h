// g2m/http/http_transport.h - HttpTransport: protocol v1 over plain HTTP
// (G6, PLAN.md 3.1 "sidecar" / "custom endpoint"; wire mapping in
// http_wire.h and docs/protocol/README.md). It is an ITransport, so a client
// talks to a sidecar or another g2m server exactly as it talks to an
// integrated one through InProcessTransport.
//
// Thread safety: send() may be called concurrently from any number of
// threads; each call borrows one connection from a small pool (keep-alive),
// opening a new one when all are busy.
//
// send() never fails by value either: a connection that cannot be made, a
// timeout or a reset answers 503 with Retry-After (failure_retry_after_s),
// and a response that does not decode answers 500 (http_wire.h).
//
// Plain http:// only. TLS for a hosted server comes from a reverse proxy in
// front of the server (PLAN.md 3.7); a TLS-capable client is G7.
#pragma once

#include "g2m/core/result.h"
#include "g2m/http/http_wire.h"
#include "g2m/server/transport.h"

#include <cstdint>
#include <memory>
#include <string>

namespace g2m::http {

struct HttpTransportConfig {
    std::string base_url;                   // "http://host[:port][/base]"; requests go to {base}/g2m/v1/...
    std::uint32_t connect_timeout_ms = 5000;
    std::uint32_t read_timeout_ms = 120000; // an on-demand derivation runs inside the request
    std::uint32_t write_timeout_ms = 5000;
    std::uint64_t max_body_bytes = std::uint64_t{1} << 30; // larger responses fail (503 via transport_failure)
    std::uint32_t failure_retry_after_s = 1;  // Retry-After of a transport failure
    std::uint32_t default_retry_after_s = 5;  // DecodeOptions::default_retry_after_s
    std::size_t max_idle_connections = 8;
    std::string user_agent;                 // empty: "g2m/<version>"
};

class HttpTransport final : public ITransport {
public:
    // Validates the URL; does not connect (the first send() does).
    static Result<std::unique_ptr<HttpTransport>> create(HttpTransportConfig config);
    ~HttpTransport() override;
    HttpTransport(const HttpTransport&) = delete;
    HttpTransport& operator=(const HttpTransport&) = delete;

    Response send(const Request& request) override;

    const HttpUrl& url() const;

private:
    struct Impl;
    explicit HttpTransport(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

} // namespace g2m::http
