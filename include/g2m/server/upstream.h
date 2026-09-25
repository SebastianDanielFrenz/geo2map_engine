// g2m/server/upstream.h - IUpstream: where a server gets tiles it neither
// has cached nor can derive (PLAN.md 3.1 "upstream chaining"). A server
// tries its upstreams in order. Concrete upstreams in G1: the local source
// store (LocalSourceUpstream). Network upstreams (a public bucket, another
// g2m server) come with g2m_http (G6).
#pragma once

#include "g2m/layer/release.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/fetcher.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {

class TileStore;

struct UpstreamContext {
    // Non-null only for network upstreams of an ONLINE server.
    IFetcher* fetcher = nullptr;
    const ReleaseManifest* release = nullptr;
    // The release's source snapshot scope in the source store.
    std::string_view source_scope;
};

struct UpstreamReply {
    enum class Kind : std::uint8_t {
        Found,       // header + body: a tile of exactly the requested layer and key
        NotFound,    // this upstream does not have it; try the next
        Unavailable, // transient failure; retry_after_s says when to retry
    };
    Kind kind = Kind::NotFound;
    TileHeader header;              // Found: header.rid may be zero (rewritten by the server)
    std::vector<std::uint8_t> body; // Found: body.size() == header.body_size, hash checked by the server
    std::uint32_t retry_after_s = 0;
    std::string message;
};

// Thread safety: fetch() may be called concurrently.
class IUpstream {
public:
    virtual ~IUpstream() = default;
    virtual std::string_view name() const = 0;
    // True if fetch() needs ctx.fetcher. An offline server never calls it.
    virtual bool needs_network() const = 0;
    virtual UpstreamReply fetch(const UpstreamContext& ctx, std::string_view layer, const TileKey& key) = 0;
};

// Raw source tiles from a source store, under the release's source scope.
class LocalSourceUpstream final : public IUpstream {
public:
    explicit LocalSourceUpstream(TileStore& sources) : sources_(sources) {}
    std::string_view name() const override { return "local-source-store"; }
    bool needs_network() const override { return false; }
    UpstreamReply fetch(const UpstreamContext& ctx, std::string_view layer, const TileKey& key) override;

private:
    TileStore& sources_;
};

} // namespace g2m
