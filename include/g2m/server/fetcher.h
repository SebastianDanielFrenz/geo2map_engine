// g2m/server/fetcher.h - IFetcher: the ONLY way g2m code reaches the network
// (PLAN.md 3.1). Network upstreams (IUpstream::needs_network() == true)
// receive one through UpstreamContext; everything else has none.
//
// Offline mode is structural: a Server configured offline never stores the
// fetcher it was given and never calls a network upstream, so no code path
// can make a network call (tests/server's FailingFetcher checks it).
// The HTTP implementation arrives with g2m_http (G6).
#pragma once

#include "g2m/core/result.h"

#include <cstdint>
#include <string>
#include <vector>

namespace g2m {

struct FetchRequest {
    std::string url;
};

struct FetchResponse {
    int http_status = 0;
    std::uint32_t retry_after_s = 0;
    std::vector<std::uint8_t> body;
};

// Thread safety: fetch() may be called concurrently.
class IFetcher {
public:
    virtual ~IFetcher() = default;
    // Transport failures are errors; any HTTP status is a response.
    virtual Result<FetchResponse> fetch(const FetchRequest& request) = 0;
};

} // namespace g2m
