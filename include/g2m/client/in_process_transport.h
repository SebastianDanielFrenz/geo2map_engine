// g2m/client/in_process_transport.h - the in-process transport (PLAN.md
// 3.1 "integrated server"): hands protocol v1 request objects straight to
// a g2m::Server in the same process. G6 adds the HTTP transport behind the
// same ITransport interface.
//
// Thread safety: send() may be called concurrently (g2m::Server is
// thread-safe). The server must outlive the transport.
#pragma once

#include "g2m/server/server.h"
#include "g2m/server/transport.h"

namespace g2m {

class InProcessTransport final : public ITransport {
public:
    explicit InProcessTransport(Server& server) : server_(server) {}
    Response send(const Request& request) override { return server_.handle(request); }

private:
    Server& server_;
};

} // namespace g2m
