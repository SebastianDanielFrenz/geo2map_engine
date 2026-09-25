// g2m/server/transport.h - ITransport: how a client reaches a g2m::Server
// (PLAN.md 3.1). The in-process transport (g2m/client/in_process_transport.h)
// calls the server directly; G6 adds HTTP.
#pragma once

#include "g2m/server/protocol.h"

namespace g2m {

// Thread safety: send() may be called concurrently from any number of
// threads (the client's g2m workers, PLAN.md 7.3).
class ITransport {
public:
    virtual ~ITransport() = default;
    virtual Response send(const Request& request) = 0;
};

} // namespace g2m
