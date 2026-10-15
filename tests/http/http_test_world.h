// tests/http/http_test_world.h - a small offline g2m::Server with every
// protocol status reachable, for the HTTP round-trip tests: raw landcover/osm
// tiles in a source store, a derived layer (test.sum), a failing deriver
// (test.fail -> 500), a degraded raw layer (dgm1: only a network upstream
// has it, the server is offline -> 503 degraded), a transiently unavailable
// raw layer (glo30 -> 503 Retry-After 7) and a retired release (410).
#pragma once

#include "g2m/client/in_process_transport.h"
#include "g2m/layer/pack.h"
#include "g2m/server/server.h"

#include "server_test_util.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <atomic>
#include <memory>
#include <string>
#include <vector>

namespace g2m_http_test {

using namespace g2m;
using g2m_test::home_tile;
using g2m_test::TempDir;

inline const std::string kLc(kSrcLandcoverLayer);
inline const std::string kOsm(kSrcOsmLayer);
inline const std::string kDgm1 = "g2m.src.elev.dgm1";
inline const std::string kGlo30 = "g2m.src.elev.glo30";
inline const std::string kSum = "test.sum";
inline const std::string kFail = "test.fail";

inline std::vector<std::uint8_t> raw_body(const std::string& layer, const TileKey& k) {
    return g2m_test::make_body(content_hash(std::vector<std::uint8_t>(layer.begin(), layer.end()))[0] * 1000003ull + k.packed(),
                               200);
}

// Derived layer over landcover (target + east neighbour) and osm (target).
class SumDeriver final : public IDeriver {
public:
    explicit SumDeriver(std::string name, bool fail = false) : fail_(fail) {
        desc_.id = LayerId{std::move(name), 3};
        desc_.kind = LayerKind::Baked;
        desc_.encoding = 0x8001;
        desc_.levels = level_bit(kRawTileLevel);
        desc_.inputs = {kLc, kOsm};
        desc_.content = "test layer";
        config_[0] = fail ? 0x66 : 0x5a;
    }
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override { return config_; }
    std::vector<InputTileRef> inputs_for(const TileKey& t) const override {
        TileKey east = t;
        east.x += 1;
        return {InputTileRef{kLc, t}, InputTileRef{kLc, east}, InputTileRef{kOsm, t}};
    }
    Result<DeriveResult> derive(const DeriveRequest& req) const override {
        calls++;
        if (fail_) return make_error(ErrorCode::Malformed, "test deriver failure");
        TileBody b;
        b.encoding = desc_.encoding;
        b.encoding_version = 1;
        Section s{1, SectionCodec::Zstd, {}};
        for (const InputTile& in : req.inputs) {
            s.data.push_back(in.header ? 1 : 0);
            if (in.header) s.data.insert(s.data.end(), in.header->body_hash.begin(), in.header->body_hash.end());
        }
        const std::uint64_t k = req.target.packed() ^ req.world_seed;
        for (int i = 0; i < 8; ++i) s.data.push_back(static_cast<std::uint8_t>(k >> (8 * i)));
        b.sections.push_back(std::move(s));
        auto body = encode_body(b);
        if (!body) return body.error();
        DeriveResult r;
        r.body = std::move(*body);
        return r;
    }
    mutable std::atomic<int> calls{0};

private:
    LayerDesc desc_;
    ContentHash config_{};
    bool fail_ = false;
};

// Local: glo30 is transiently unavailable (Retry-After 7); everything else NotFound.
class FlakyUpstream final : public IUpstream {
public:
    std::string_view name() const override { return "flaky"; }
    bool needs_network() const override { return false; }
    UpstreamReply fetch(const UpstreamContext&, std::string_view layer, const TileKey&) override {
        UpstreamReply r;
        if (layer == kGlo30) {
            r.kind = UpstreamReply::Kind::Unavailable;
            r.retry_after_s = 7;
            r.message = "busy (try later)";
        }
        return r;
    }
};

// A network upstream: never called by an offline server (counts if it is).
class NetUpstream final : public IUpstream {
public:
    std::string_view name() const override { return "net"; }
    bool needs_network() const override { return true; }
    UpstreamReply fetch(const UpstreamContext&, std::string_view, const TileKey&) override {
        calls++;
        return {};
    }
    std::atomic<int> calls{0};
};

inline std::unique_ptr<TileStore> open_store(const TempDir& d, const std::string& name) {
    TileStoreConfig c;
    c.dir = d.path();
    c.name = name;
    auto s = TileStore::open(c);
    REQUIRE(s);
    return std::move(*s);
}

struct World {
    TempDir dir{"http"};
    std::unique_ptr<TileStore> sources = open_store(dir, "sources");
    std::unique_ptr<TileStore> derived = open_store(dir, "derived");
    SumDeriver sum{kSum};
    SumDeriver fail{kFail, true};
    FlakyUpstream flaky;
    NetUpstream net;
    std::unique_ptr<LocalSourceUpstream> local;
    ReleaseManifest release;
    ReleaseId retired{};
    std::unique_ptr<Server> server;
    std::unique_ptr<InProcessTransport> in_process;

    World() {
        for (int dx = 0; dx < 3; ++dx) put_raw(kLc, home_tile(dx, 0));
        for (int dx = 0; dx < 2; ++dx) put_raw(kOsm, home_tile(dx, 0));
        LocalReleaseParams p;
        p.source_scope = "snap1";
        p.world_seed = 42;
        p.derivers = {&sum, &fail};
        p.sources = {ReleaseSource{"synthetic", "2026-09-25", "none", "test \"quoted\" attribution", false}};
        auto m = make_local_release(*sources, p);
        REQUIRE(m);
        release = *m;
        for (const std::string& elev : {kDgm1, kGlo30}) {
            ReleaseLayer l;
            l.desc = make_raw_elev_layer(elev.substr(std::string("g2m.src.elev.").size()));
            REQUIRE(!l.desc.id.name.empty());
            l.availability = Availability::OnDemand;
            l.coverage = Coverage::from_level2_tiles({home_tile(0, 0), home_tile(1, 0)});
            release.layers.push_back(l);
        }
        std::sort(release.layers.begin(), release.layers.end(),
                  [](const ReleaseLayer& a, const ReleaseLayer& b) { return a.desc.id.name < b.desc.id.name; });

        local = std::make_unique<LocalSourceUpstream>(*sources);
        ServerConfig c;
        c.server_id = "g2m.http_test";
        c.contact = "nobody@example.invalid";
        c.offline = true;
        c.derived_store = derived.get();
        c.upstreams = {local.get(), &flaky, &net};
        c.derivers = {&sum, &fail};
        server = std::make_unique<Server>(c);
        REQUIRE(server->add_release(release, "snap1"));
        ReleaseManifest old = release;
        old.rid[0] ^= 0xff;
        retired = old.rid;
        REQUIRE(server->add_release(old, "snap1"));
        REQUIRE(server->set_default_release(release.rid));
        REQUIRE(server->retire_release(retired, release.rid));
        in_process = std::make_unique<InProcessTransport>(*server);
    }

    void put_raw(const std::string& layer, const TileKey& k) {
        const auto body = raw_body(layer, k);
        REQUIRE(sources->put(TileAddress{"snap1", layer, k}, g2m_test::make_header(layer, k, body), body));
    }

    // One request per reachable protocol outcome, every endpoint.
    std::vector<Request> requests() const {
        const ReleaseId rid = release.rid;
        ReleaseId unknown = rid;
        unknown[31] ^= 0x01;
        const TileKey pk = pack_of(home_tile());
        TileKey far = pk;
        far.x += 10;
        TileKey l1 = home_tile();
        l1.level = 1;
        TileKey out = home_tile(40, 40);
        std::vector<Request> r;
        r.push_back(ManifestRequest{});
        r.push_back(ReleaseManifestRequest{rid});
        r.push_back(ReleaseManifestRequest{unknown});   // 404
        r.push_back(ReleaseManifestRequest{retired});   // 410
        r.push_back(TileRequest{rid, kLc, home_tile(), std::nullopt});        // 200 raw
        r.push_back(TileRequest{rid, kSum, home_tile(1, 0), std::nullopt});   // 200 derived
        r.push_back(TileRequest{rid, kSum, home_tile(2, 0), std::nullopt});   // 200 derived (missing inputs)
        r.push_back(TileRequest{rid, kLc, out, std::nullopt});                // 404 outside coverage
        r.push_back(TileRequest{rid, "test.none", home_tile(), std::nullopt}); // 404 unknown layer
        r.push_back(TileRequest{rid, kLc, l1, std::nullopt});                 // 404 level
        r.push_back(TileRequest{unknown, kLc, home_tile(), std::nullopt});    // 404 release
        r.push_back(TileRequest{retired, kLc, home_tile(), std::nullopt});    // 410
        r.push_back(TileRequest{rid, kDgm1, home_tile(), std::nullopt});      // 503 degraded
        r.push_back(TileRequest{rid, kGlo30, home_tile(), std::nullopt});     // 503 Retry-After 7
        r.push_back(TileRequest{rid, kFail, home_tile(), std::nullopt});      // 500
        r.push_back(PackRequest{rid, kLc, pk});      // 200
        r.push_back(PackRequest{rid, kSum, pk});     // 200 derived
        r.push_back(PackRequest{rid, kLc, far});     // 404
        r.push_back(PackRequest{rid, kDgm1, pk});    // 503 degraded, no tile present
        r.push_back(PackRequest{rid, kGlo30, pk});   // 503 retry
        r.push_back(PackRequest{retired, kLc, pk});  // 410
        r.push_back(HashesRequest{rid, kLc, pk});    // 200
        r.push_back(HashesRequest{rid, kSum, pk});   // 200
        r.push_back(HashesRequest{rid, kLc, far});   // 404
        r.push_back(HashesRequest{rid, kDgm1, pk});  // 503 degraded
        r.push_back(HashesRequest{rid, kGlo30, pk}); // 503
        r.push_back(HashesRequest{unknown, kLc, pk}); // 404
        return r;
    }

    // The 200 tile's body hash, for 304 requests.
    ContentHash body_hash(const std::string& layer, const TileKey& k) {
        const TileResponse t = server->tile(TileRequest{release.rid, layer, k, std::nullopt});
        REQUIRE(t.meta.status == Status::Ok);
        REQUIRE(t.meta.etag);
        return *t.meta.etag;
    }
};

// Exact equality of two responses: every ResponseMeta field and the payload
// bytes (manifests compared by value and by their canonical JSON).
inline bool same_meta(const ResponseMeta& a, const ResponseMeta& b) {
    return a.status == b.status && a.degraded == b.degraded && a.retry_after_s == b.retry_after_s && a.etag == b.etag &&
           a.newest_rid == b.newest_rid && a.message == b.message;
}

inline bool same_response(const Response& a, const Response& b) {
    if (a.index() != b.index()) return false;
    if (!same_meta(meta_of(a), meta_of(b))) return false;
    if (const auto* x = std::get_if<ManifestResponse>(&a)) {
        const auto& y = std::get<ManifestResponse>(b);
        return x->manifest == y.manifest && to_json(x->manifest) == to_json(y.manifest);
    }
    if (const auto* x = std::get_if<ReleaseManifestResponse>(&a)) {
        const auto& y = std::get<ReleaseManifestResponse>(b);
        return x->manifest == y.manifest && to_json(x->manifest) == to_json(y.manifest);
    }
    if (const auto* x = std::get_if<TileResponse>(&a)) return x->container == std::get<TileResponse>(b).container;
    if (const auto* x = std::get_if<PackResponse>(&a)) return x->pack == std::get<PackResponse>(b).pack;
    if (const auto* x = std::get_if<HashesResponse>(&a)) return x->directory == std::get<HashesResponse>(b).directory;
    return false;
}

inline std::string describe(const Request& r) { return to_path(r); }

} // namespace g2m_http_test
