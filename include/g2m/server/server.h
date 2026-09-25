// g2m/server/server.h - g2m::Server, the one server implementation behind
// every deployment (PLAN.md 3.1). G1: in-process only.
//
// Router: handle() dispatches a protocol v1 Request (protocol.h).
// Resolver, for one tile of a live release:
//   1. definitive checks -> 404: unknown layer, a level the layer does not
//      have, outside the layer's coverage (410 for a retired release);
//   2. the layer's policy chain (PLAN.md 3.3), default Cache, Derive,
//      Upstream:
//        Cache    - the derived store, under the release id;
//        Derive   - an IDeriver for the layer: its inputs are resolved
//                   recursively (same rules), then derive() runs inline;
//        Upstream - each IUpstream in order. A network upstream is skipped
//                   when the server is offline (and then gets no fetcher).
//      What Derive produces and what network upstreams return is written to
//      the derived store; tiles from local upstreams are not copied.
//   3. nothing found: 503 + Retry-After if a step failed transiently, 503
//      degraded if a network upstream was skipped because the server is
//      offline, else 404.
//
// Thread safety: every public method may be called concurrently. The
// stores are thread-safe (tile_store.h); derivers and upstreams must be
// (their headers say so). Two threads resolving the same missing tile may
// both derive it; both write identical bytes.
#pragma once

#include "g2m/core/result.h"
#include "g2m/layer/classifier.h"
#include "g2m/layer/deriver.h"
#include "g2m/layer/layer_registry.h"
#include "g2m/layer/release.h"
#include "g2m/server/fetcher.h"
#include "g2m/server/protocol.h"
#include "g2m/server/tile_store.h"
#include "g2m/server/upstream.h"

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace g2m {

enum class ResolveStep : std::uint8_t { Cache, Derive, Upstream };

struct LayerPolicy {
    std::string layer;
    std::vector<ResolveStep> steps;
};

struct ServerConfig {
    std::string server_id = "g2m.integrated";
    std::string contact;
    std::string tile_base_url;
    // Offline: network upstreams are never called and `fetcher` is never
    // stored (it may be set; it is ignored).
    bool offline = false;
    IFetcher* fetcher = nullptr;
    // Cache of derived and network-fetched tiles, scoped by release id (hex).
    // Optional; must outlive the server.
    TileStore* derived_store = nullptr;
    std::vector<IUpstream*> upstreams;        // in order; must outlive the server
    std::vector<const IDeriver*> derivers;    // must outlive the server
    std::vector<LayerPolicy> policies;        // layers not listed: Cache, Derive, Upstream
    std::uint32_t retry_after_s = 5;          // for transient failures without a hint
};

class Server {
public:
    explicit Server(ServerConfig config);
    ~Server();
    Server(const Server&) = delete;
    Server& operator=(const Server&) = delete;

    bool offline() const;

    // Makes a release live; source_scope names its raw tiles in the source
    // store (LocalSourceUpstream). The first release added is the default.
    Result<void> add_release(ReleaseManifest manifest, std::string source_scope);
    Result<void> set_default_release(const ReleaseId& rid);
    // Requests for `rid` then answer 410 naming `successor`.
    Result<void> retire_release(const ReleaseId& rid, const ReleaseId& successor);

    Response handle(const Request& request);
    ManifestResponse manifest();
    ReleaseManifestResponse release_manifest(const ReleaseManifestRequest& request);
    TileResponse tile(const TileRequest& request);
    PackResponse pack(const PackRequest& request);
    HashesResponse hashes(const HashesRequest& request);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// A local release (PLAN.md 3.5) over what `sources` holds under
// source_scope: every raw tile there (its layer must be a raw layer of
// `registry`) plus every deriver's output layer (OnDemand, coverage = the
// union of its inputs' coverage). The rid is computed exactly like a hosted
// release's (compute_release_id).
struct LocalReleaseParams {
    std::string source_scope;
    std::uint64_t world_seed = 0;
    std::vector<ReleaseSource> sources;           // attribution records
    std::vector<const IDeriver*> derivers;
    std::vector<const IClassifier*> classifiers;
    const LayerRegistry* registry = nullptr;      // null: LayerRegistry::builtin()
};
Result<ReleaseManifest> make_local_release(TileStore& sources, const LocalReleaseParams& params);

} // namespace g2m
