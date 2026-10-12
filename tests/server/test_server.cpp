// g2m::Server: router, resolver policy chain (cache -> derive -> upstream),
// 404/410/503 semantics, offline mode (structurally no network call),
// packs and hashes, the local release, and the in-process transport.

#include "g2m/client/in_process_transport.h"
#include "g2m/server/server.h"

#include "server_test_util.h"

#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

using namespace g2m;
using g2m_test::TempDir;
using g2m_test::home_tile;

namespace {

const std::string kLc(kSrcLandcoverLayer);
const std::string kOsm(kSrcOsmLayer);
const std::string kElev(kSrcElevDgm1Layer);
const std::string kSum = "test.sum";

std::unique_ptr<TileStore> open_store(const TempDir& d, const std::string& name) {
    TileStoreConfig c;
    c.dir = d.path();
    c.name = name;
    auto s = TileStore::open(c);
    REQUIRE(s);
    return std::move(*s);
}

std::vector<std::uint8_t> raw_body(const std::string& layer, const TileKey& k) {
    return g2m_test::make_body(content_hash(std::vector<std::uint8_t>(layer.begin(), layer.end()))[0] * 1000003ull +
                                   k.packed(),
                               200);
}

void put_raw(TileStore& s, const std::string& scope, const std::string& layer, const TileKey& k) {
    const auto body = raw_body(layer, k);
    REQUIRE(s.put(TileAddress{scope, layer, k}, g2m_test::make_header(layer, k, body), body));
}

// test.sum = f(landcover at the target and its eastern neighbour, osm at
// the target). Pure, counts its calls.
class SumDeriver final : public IDeriver {
public:
    SumDeriver() {
        desc_.id = LayerId{kSum, 3};
        desc_.kind = LayerKind::Baked;
        desc_.encoding = 0x8001;
        desc_.levels = level_bit(kRawTileLevel);
        desc_.inputs = {kLc, kOsm};
        desc_.content = "test layer";
        config_[0] = 0x5a;
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
        if (delay_ms > 0) std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
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
        r.height_offset = 7;
        return r;
    }
    mutable std::atomic<int> calls{0};
    int delay_ms = 0; // simulates a slow derivation (dedup tests)

private:
    LayerDesc desc_;
    ContentHash config_{};
};

// A fetcher double: records every call. Offline tests assert it is never called.
class RecordingFetcher final : public IFetcher {
public:
    Result<FetchResponse> fetch(const FetchRequest& req) override {
        calls++;
        std::lock_guard lock(mu);
        urls.push_back(req.url);
        FetchResponse r;
        r.http_status = status;
        r.retry_after_s = retry_after;
        r.body = body;
        return r;
    }
    std::atomic<int> calls{0};
    std::mutex mu;
    std::vector<std::string> urls;
    int status = 200;
    std::uint32_t retry_after = 0;
    std::vector<std::uint8_t> body;
};

// A network upstream: a tile container per URL through ctx.fetcher.
class NetUpstream final : public IUpstream {
public:
    std::string_view name() const override { return "test-net"; }
    bool needs_network() const override { return true; }
    UpstreamReply fetch(const UpstreamContext& ctx, std::string_view layer, const TileKey& key) override {
        calls++;
        UpstreamReply reply;
        if (!ctx.fetcher) {
            without_fetcher++;
            reply.kind = UpstreamReply::Kind::Unavailable;
            return reply;
        }
        auto r = ctx.fetcher->fetch(FetchRequest{"https://example.invalid/" + std::string(layer) + "/" + to_string(key)});
        if (!r || r->http_status == 503) {
            reply.kind = UpstreamReply::Kind::Unavailable;
            reply.retry_after_s = r ? r->retry_after_s : 0;
            return reply;
        }
        if (r->http_status == 404) return reply;
        auto v = parse_container(r->body);
        if (!v) {
            reply.kind = UpstreamReply::Kind::Unavailable;
            return reply;
        }
        reply.kind = UpstreamReply::Kind::Found;
        reply.header = v->header;
        reply.body.assign(v->body.begin(), v->body.end());
        return reply;
    }
    std::atomic<int> calls{0};
    std::atomic<int> without_fetcher{0};
};

// Source store with landcover at x = 350..352 and osm at 350..351 (y = 5450).
struct World {
    TempDir dir{"server"};
    std::unique_ptr<TileStore> sources = open_store(dir, "sources");
    std::unique_ptr<TileStore> derived = open_store(dir, "derived");
    SumDeriver sum;
    ReleaseManifest release;

    World() {
        for (int dx = 0; dx < 3; ++dx) put_raw(*sources, "snap1", kLc, home_tile(dx, 0));
        for (int dx = 0; dx < 2; ++dx) put_raw(*sources, "snap1", kOsm, home_tile(dx, 0));
        LocalReleaseParams p;
        p.source_scope = "snap1";
        p.world_seed = 42;
        p.derivers = {&sum};
        p.sources = {ReleaseSource{"synthetic", "2026-09-25", "none", "test", false}};
        auto m = make_local_release(*sources, p);
        REQUIRE(m);
        release = *m;
    }

    // Adds an elevation layer (served only by the network upstream) to the release.
    void add_network_layer() {
        ReleaseLayer l;
        l.desc = make_raw_elev_layer("dgm1");
        l.availability = Availability::OnDemand;
        l.coverage = Coverage::from_level2_tiles({home_tile(0, 0), home_tile(1, 0)});
        release.layers.push_back(l);
        std::sort(release.layers.begin(), release.layers.end(),
                  [](const ReleaseLayer& a, const ReleaseLayer& b) { return a.desc.id.name < b.desc.id.name; });
    }
};

TileResponse get_tile(Server& s, const ReleaseId& rid, const std::string& layer, const TileKey& k) {
    return s.tile(TileRequest{rid, layer, k, std::nullopt});
}

std::vector<std::uint8_t> elev_container(const TileKey& k) {
    const auto body = raw_body(kElev, k);
    return assemble_container(g2m_test::make_header(kElev, k, body), body).value();
}

} // namespace

TEST_CASE("server: discovered dependencies are resolved, hashed and cached", "[server][discovery]") {
    World w;
    class Discovered final : public IDeriver {
    public:
        SumDeriver sum;
        const LayerDesc& output() const override { return sum.output(); }
        ContentHash config_hash() const override { return sum.config_hash(); }
        std::vector<InputTileRef> inputs_for(const TileKey& t) const override { return {{kOsm, t}}; }
        Result<std::vector<InputTileRef>> additional_inputs_for(const DeriveRequest& req) const override {
            REQUIRE(req.inputs.size() == 1); REQUIRE(req.inputs[0].header);
            CHECK(req.inputs[0].body == raw_body(kOsm, req.target));
            ++discoveries;
            auto east = req.target; ++east.x;
            return std::vector<InputTileRef>{{kLc, east}, {kOsm, req.target}}; // overlap deduplicates
        }
        Result<DeriveResult> derive(const DeriveRequest& req) const override {
            REQUIRE(req.inputs.size() == 2); CHECK(req.inputs[0].ref.layer == kLc);
            CHECK(req.inputs[1].ref.layer == kOsm); REQUIRE(req.inputs[0].header);
            return sum.derive(req);
        }
        mutable int discoveries = 0;
    } deriver;
    LocalSourceUpstream upstream(*w.sources);
    ServerConfig config; config.derived_store = w.derived.get(); config.derivers = {&deriver}; config.upstreams = {&upstream};
    Server server(config); REQUIRE(server.add_release(w.release, "snap1"));
    const auto response = get_tile(server, w.release.rid, kSum, home_tile()); REQUIRE(response.meta.status == Status::Ok);
    const auto parsed = parse_container(response.container); REQUIRE(parsed); CHECK(parsed->header.inputs.size() == 2);
    for (const auto& hash : {content_hash(raw_body(kLc, home_tile(1, 0))), content_hash(raw_body(kOsm, home_tile()))})
        CHECK(std::find(parsed->header.inputs.begin(), parsed->header.inputs.end(), hash) != parsed->header.inputs.end());
    const auto cached = get_tile(server, w.release.rid, kSum, home_tile()); REQUIRE(cached.meta.status == Status::Ok);
    CHECK(cached.container == response.container); CHECK(deriver.discoveries == 1); CHECK(deriver.sum.calls == 1);
}

TEST_CASE("server: later discovery stages see the resolved inputs and stop at the first stage that adds nothing", "[server][discovery]") {
    // Stage k names landcover tiles dx = 1..k east of the target, each one only once the previous one is resolved and
    // present; dx = 3 does not exist in the source store (absent), so its successor is never named. With 4 stages the
    // chain stops at the fixed point (stage 4 adds nothing); with 2 stages at dx = 2; with 1 stage (the default) at dx = 1.
    for (const std::size_t stages : {std::size_t{1}, std::size_t{2}, std::size_t{4}}) {
        World w;
        class Staged final : public IDeriver {
        public:
            SumDeriver sum; std::size_t stages = 1;
            const LayerDesc& output() const override { return sum.output(); }
            ContentHash config_hash() const override { return sum.config_hash(); }
            std::size_t discovery_stages() const override { return stages; }
            std::vector<InputTileRef> inputs_for(const TileKey& t) const override { return {{kOsm, t}}; }
            Result<std::vector<InputTileRef>> additional_inputs_for(const DeriveRequest& req) const override {
                sizes.push_back(req.inputs.size());
                std::vector<InputTileRef> out;
                auto key = req.target;
                for (int dx = 1; dx <= 8; ++dx) {
                    key.x = req.target.x + dx; out.push_back({kLc, key});
                    const auto it = std::find_if(req.inputs.begin(), req.inputs.end(), [&](const auto& in) { return in.ref == out.back(); });
                    if (it == req.inputs.end() || !it->header) break;   // the next one only once this one is resolved and present
                }
                std::sort(out.begin(), out.end());
                return out;
            }
            Result<DeriveResult> derive(const DeriveRequest& req) const override { derived = req.inputs.size(); return sum.derive(req); }
            mutable std::vector<std::size_t> sizes; mutable std::size_t derived = 0;
        } deriver;
        deriver.stages = stages;
        LocalSourceUpstream upstream(*w.sources);
        ServerConfig config; config.derived_store = w.derived.get(); config.derivers = {&deriver}; config.upstreams = {&upstream};
        Server server(config); REQUIRE(server.add_release(w.release, "snap1"));
        const auto response = get_tile(server, w.release.rid, kSum, home_tile()); REQUIRE(response.meta.status == Status::Ok);
        // Inputs: the osm target plus landcover dx = 1 .. min(stages, 3) (dx = 3 is named but absent).
        CHECK(deriver.derived == 1 + std::min<std::size_t>(stages, 3));
        const std::vector<std::size_t> expected_sizes = stages == 1 ? std::vector<std::size_t>{1}
            : stages == 2 ? std::vector<std::size_t>{1, 2} : std::vector<std::size_t>{1, 2, 3, 4};
        CHECK(deriver.sizes == expected_sizes);
        const auto parsed = parse_container(response.container); REQUIRE(parsed);
        CHECK(parsed->header.inputs.size() == std::min<std::size_t>(stages, 2) + 1);   // absent inputs carry no hash
    }
}

TEST_CASE("server: invalid or failed discovered dependencies never cache output", "[server][discovery]") {
    for (int mode = 0; mode < 4; ++mode) {
        World w;
        class Invalid final : public IDeriver {
        public:
            int mode = 0;
            SumDeriver sum;
            const LayerDesc& output() const override { return sum.output(); }
            ContentHash config_hash() const override { return sum.config_hash(); }
            std::vector<InputTileRef> inputs_for(const TileKey& t) const override { return {{kOsm, t}}; }
            Result<std::vector<InputTileRef>> additional_inputs_for(const DeriveRequest& req) const override {
                if (mode == 0) return make_error(ErrorCode::Malformed, "bad discovered geometry");
                if (mode == 1) return std::vector<InputTileRef>{{"undeclared.layer", req.target}};
                if (mode == 2) return std::vector<InputTileRef>{{kLc, req.target}, {kLc, req.target}};
                auto key = req.target; key.level = -1; return std::vector<InputTileRef>{{kLc, key}};
            }
            Result<DeriveResult> derive(const DeriveRequest& req) const override { return sum.derive(req); }
        } deriver;
        deriver.mode = mode;
        LocalSourceUpstream upstream(*w.sources);
        ServerConfig config; config.derived_store = w.derived.get(); config.derivers = {&deriver}; config.upstreams = {&upstream};
        Server server(config); REQUIRE(server.add_release(w.release, "snap1"));
        const auto response = get_tile(server, w.release.rid, kSum, home_tile());
        CHECK(response.meta.status == Status::Internal); CHECK(deriver.sum.calls == 0);
        const auto stored = w.derived->get({to_hex(w.release.rid), kSum, home_tile()}); REQUIRE(stored); CHECK_FALSE(*stored);
    }
}

TEST_CASE("server: discovered absent and offline dependencies retain resolver semantics", "[server][discovery]") {
    for (const bool offline_network : {false, true}) {
        World w; w.add_network_layer();
        class Discovery final : public IDeriver {
        public:
            bool network = false; SumDeriver sum; LayerDesc desc = sum.output();
            Discovery() { desc.inputs.push_back(kElev); }
            const LayerDesc& output() const override { return desc; }
            ContentHash config_hash() const override { return sum.config_hash(); }
            std::vector<InputTileRef> inputs_for(const TileKey& t) const override { return {{kOsm, t}}; }
            Result<std::vector<InputTileRef>> additional_inputs_for(const DeriveRequest& req) const override {
                auto key = req.target; if (!network) key.x += 20;
                return std::vector<InputTileRef>{{network ? kElev : kLc, key}};
            }
            Result<DeriveResult> derive(const DeriveRequest& req) const override {
                REQUIRE(req.inputs.size() == 2);
                CHECK(std::count_if(req.inputs.begin(), req.inputs.end(), [](const auto& in) { return !in.header; }) == 1);
                return sum.derive(req);
            }
        } deriver;
        deriver.network = offline_network;
        LocalSourceUpstream local(*w.sources); NetUpstream net;
        ServerConfig config; config.offline = true; config.derived_store = w.derived.get();
        config.derivers = {&deriver}; config.upstreams = {&local, &net};
        Server server(config); REQUIRE(server.add_release(w.release, "snap1"));
        const auto response = get_tile(server, w.release.rid, kSum, home_tile());
        CHECK(net.calls == 0);
        if (offline_network) {
            CHECK(response.meta.status == Status::Unavailable); CHECK(response.meta.degraded); CHECK(deriver.sum.calls == 0);
            const auto stored = w.derived->get({to_hex(w.release.rid), kSum, home_tile()}); REQUIRE(stored); CHECK_FALSE(*stored);
        } else { CHECK(response.meta.status == Status::Ok); CHECK(deriver.sum.calls == 1); }
    }
}

TEST_CASE("server: local release manifest", "[server]") {
    World w;
    const ReleaseManifest& m = w.release;
    REQUIRE(m.layers.size() == 3);
    CHECK(m.layers[0].desc.id.name == kLc);
    CHECK(m.layers[1].desc.id.name == kOsm);
    CHECK(m.layers[2].desc.id.name == kSum);
    CHECK(m.layers[0].availability == Availability::Prebaked);
    CHECK(m.layers[2].availability == Availability::OnDemand);
    CHECK(m.layers[2].config_hash == w.sum.config_hash());
    CHECK(m.layers[0].coverage.covers(home_tile(2, 0)));
    CHECK_FALSE(m.layers[1].coverage.covers(home_tile(2, 0)));
    CHECK(m.layers[2].coverage.covers(home_tile(2, 0))); // union of its inputs
    CHECK_FALSE(m.layers[2].coverage.covers(home_tile(3, 0)));
    CHECK(m.world_seed == 42);
    CHECK(m.rid != ReleaseId{});

    // Same inputs -> same rid; any change -> another rid.
    LocalReleaseParams p;
    p.source_scope = "snap1";
    p.world_seed = 42;
    p.derivers = {&w.sum};
    p.sources = m.sources;
    CHECK(make_local_release(*w.sources, p)->rid == m.rid);
    p.world_seed = 43;
    CHECK(make_local_release(*w.sources, p)->rid != m.rid);
    p.world_seed = 42;
    put_raw(*w.sources, "snap1", kOsm, home_tile(2, 0));
    CHECK(make_local_release(*w.sources, p)->rid != m.rid);
    // A non-raw layer in the source store is refused.
    const auto body = g2m_test::make_body(1, 100);
    REQUIRE(w.sources->put(TileAddress{"snap1", kSum, home_tile()}, g2m_test::make_header(kSum, home_tile(), body), body));
    CHECK_FALSE(make_local_release(*w.sources, p));
}

TEST_CASE("server: manifests", "[server]") {
    World w;
    ServerConfig c;
    c.server_id = "test";
    c.contact = "nobody@example.invalid";
    Server s(c);
    CHECK(s.manifest().manifest.live_releases.empty());
    REQUIRE(s.add_release(w.release, "snap1"));
    CHECK_FALSE(s.add_release(w.release, "snap1"));
    const ManifestResponse m = s.manifest();
    CHECK(m.meta.status == Status::Ok);
    CHECK(m.manifest.server_id == "test");
    CHECK(m.manifest.default_rid == w.release.rid);
    CHECK(m.manifest.live_releases == std::vector<ReleaseId>{w.release.rid});
    CHECK(m.manifest.protocol_versions == std::vector<std::uint32_t>{1});

    const ReleaseManifestResponse r = s.release_manifest({w.release.rid});
    CHECK(r.meta.status == Status::Ok);
    CHECK(r.manifest == w.release);
    ReleaseId other{};
    other[5] = 1;
    CHECK(s.release_manifest({other}).meta.status == Status::NotFound);
}

TEST_CASE("server: raw tiles from the local source upstream", "[server]") {
    World w;
    LocalSourceUpstream local(*w.sources);
    ServerConfig c;
    c.derived_store = w.derived.get();
    c.upstreams = {&local};
    Server s(c);
    REQUIRE(s.add_release(w.release, "snap1"));
    const ReleaseId rid = w.release.rid;

    const TileResponse t = get_tile(s, rid, kLc, home_tile(1, 0));
    REQUIRE(t.meta.status == Status::Ok);
    auto v = parse_container(t.container);
    REQUIRE(v);
    CHECK(v->header.rid == rid); // rewritten to the serving release
    CHECK(v->header.key == home_tile(1, 0));
    CHECK(std::vector<std::uint8_t>(v->body.begin(), v->body.end()) == raw_body(kLc, home_tile(1, 0)));
    REQUIRE(t.meta.etag);
    CHECK(*t.meta.etag == v->header.body_hash);
    // Local raw tiles are not copied into the derived store.
    CHECK(w.derived->stats()->tiles == 0);

    SECTION("304 on a matching if_none_match") {
        const TileResponse n = s.tile(TileRequest{rid, kLc, home_tile(1, 0), v->header.body_hash});
        CHECK(n.meta.status == Status::NotModified);
        CHECK(n.container.empty());
        ContentHash wrong = v->header.body_hash;
        wrong[0] ^= 1;
        CHECK(s.tile(TileRequest{rid, kLc, home_tile(1, 0), wrong}).meta.status == Status::Ok);
    }
    SECTION("404 is definitive: coverage, layer, level, release") {
        CHECK(get_tile(s, rid, kOsm, home_tile(2, 0)).meta.status == Status::NotFound); // outside osm coverage
        CHECK(get_tile(s, rid, kLc, home_tile(9, 9)).meta.status == Status::NotFound);
        CHECK(get_tile(s, rid, "g2m.src.elev.other", home_tile()).meta.status == Status::NotFound);
        CHECK(get_tile(s, rid, kLc, home_tile().child(0, 0)).meta.status == Status::NotFound); // level 1
        ReleaseId unknown{};
        unknown[0] = 9;
        CHECK(get_tile(s, unknown, kLc, home_tile()).meta.status == Status::NotFound);
        // In coverage but gone from every store: also 404.
        REQUIRE(w.sources->remove(TileAddress{"snap1", kLc, home_tile(2, 0)}));
        CHECK(get_tile(s, rid, kLc, home_tile(2, 0)).meta.status == Status::NotFound);
    }
    SECTION("400 on malformed requests") {
        CHECK(get_tile(s, rid, "NotALayer", home_tile()).meta.status == Status::BadRequest);
        TileKey bad = home_tile();
        bad.level = 99;
        CHECK(get_tile(s, rid, kLc, bad).meta.status == Status::BadRequest);
    }
    SECTION("the in-process transport routes to the same server") {
        InProcessTransport tr(s);
        const Response r = tr.send(Request{TileRequest{rid, kLc, home_tile(1, 0), std::nullopt}});
        REQUIRE(std::holds_alternative<TileResponse>(r));
        CHECK(std::get<TileResponse>(r).container == t.container);
        CHECK(meta_of(tr.send(Request{ManifestRequest{}})).status == Status::Ok);
    }
}

TEST_CASE("server: derive from inputs, then serve from the cache", "[server]") {
    World w;
    LocalSourceUpstream local(*w.sources);
    ServerConfig c;
    c.derived_store = w.derived.get();
    c.upstreams = {&local};
    c.derivers = {&w.sum};
    Server s(c);
    REQUIRE(s.add_release(w.release, "snap1"));
    const ReleaseId rid = w.release.rid;

    const TileResponse t = get_tile(s, rid, kSum, home_tile(0, 0));
    REQUIRE(t.meta.status == Status::Ok);
    CHECK(w.sum.calls == 1);
    auto v = parse_container(t.container);
    REQUIRE(v);
    CHECK(v->header.layer_version == 3);
    CHECK(v->header.height_offset == 7);
    CHECK(v->header.rid == rid);
    // inputs: sorted unique body hashes of the three present inputs
    std::vector<ContentHash> expect{content_hash(raw_body(kLc, home_tile(0, 0))),
                                    content_hash(raw_body(kLc, home_tile(1, 0))),
                                    content_hash(raw_body(kOsm, home_tile(0, 0)))};
    std::sort(expect.begin(), expect.end());
    CHECK(v->header.inputs == expect);

    // Second request: from the derived store, no new derivation.
    const TileResponse again = get_tile(s, rid, kSum, home_tile(0, 0));
    CHECK(again.container == t.container);
    CHECK(w.sum.calls == 1);
    auto cached = w.derived->get(TileAddress{to_hex(rid), kSum, home_tile(0, 0)});
    REQUIRE(cached);
    REQUIRE(*cached);

    // x = 352: the eastern landcover neighbour and osm are outside coverage
    // -> passed as absent inputs, the tile still derives.
    const TileResponse edge = get_tile(s, rid, kSum, home_tile(2, 0));
    REQUIRE(edge.meta.status == Status::Ok);
    CHECK(parse_container(edge.container)->header.inputs.size() == 1);
    CHECK(get_tile(s, rid, kSum, home_tile(3, 0)).meta.status == Status::NotFound);

    // A second server with the same stores answers from the cache alone.
    ServerConfig c2;
    c2.derived_store = w.derived.get();
    c2.policies = {LayerPolicy{kSum, {ResolveStep::Cache}}};
    Server s2(c2);
    REQUIRE(s2.add_release(w.release, "snap1"));
    CHECK(get_tile(s2, rid, kSum, home_tile(0, 0)).container == t.container);
    CHECK(get_tile(s2, rid, kSum, home_tile(1, 0)).meta.status == Status::NotFound);
}

TEST_CASE("server: an unavailable input makes the derived tile unavailable", "[server]") {
    World w;
    NetUpstream net;
    RecordingFetcher fetcher;
    fetcher.status = 503;
    fetcher.retry_after = 11;
    ServerConfig c;
    c.fetcher = &fetcher;
    c.upstreams = {&net}; // raw inputs only reachable through the (failing) network
    c.derivers = {&w.sum};
    Server s(c);
    REQUIRE(s.add_release(w.release, "snap1"));
    const TileResponse t = get_tile(s, w.release.rid, kSum, home_tile(0, 0));
    CHECK(t.meta.status == Status::Unavailable);
    CHECK(t.meta.retry_after_s == 11);
    CHECK_FALSE(t.meta.degraded);
    CHECK(w.sum.calls == 0);
}

TEST_CASE("server: offline never calls the network", "[server][offline]") {
    World w;
    w.add_network_layer();
    LocalSourceUpstream local(*w.sources);
    NetUpstream net;
    RecordingFetcher fetcher; // set, but must be ignored
    fetcher.body = elev_container(home_tile(0, 0));
    ServerConfig c;
    c.offline = true;
    c.fetcher = &fetcher;
    c.derived_store = w.derived.get();
    c.upstreams = {&net, &local};
    c.derivers = {&w.sum};
    Server s(c);
    CHECK(s.offline());
    REQUIRE(s.add_release(w.release, "snap1"));
    const ReleaseId rid = w.release.rid;

    // Local data is still served.
    CHECK(get_tile(s, rid, kLc, home_tile(0, 0)).meta.status == Status::Ok);
    CHECK(get_tile(s, rid, kSum, home_tile(0, 0)).meta.status == Status::Ok);
    // Not cached, only on the network: 503 degraded, no retry hint.
    const TileResponse t = get_tile(s, rid, kElev, home_tile(0, 0));
    CHECK(t.meta.status == Status::Unavailable);
    CHECK(t.meta.degraded);
    CHECK(t.meta.retry_after_s == 0);
    // Cached: served.
    const auto cached = elev_container(home_tile(1, 0));
    REQUIRE(w.derived->put_container(TileAddress{to_hex(rid), kElev, home_tile(1, 0)}, cached));
    const TileResponse hit = get_tile(s, rid, kElev, home_tile(1, 0));
    REQUIRE(hit.meta.status == Status::Ok);
    CHECK(parse_container(hit.container)->body.size() == parse_container(cached)->body.size());
    // A pack of the network layer: the uncached tile is Degraded.
    const PackResponse p = s.pack(PackRequest{rid, kElev, pack_of(home_tile())});
    CHECK(p.meta.status == Status::Unavailable);
    CHECK(p.meta.degraded);
    auto pv = decode_pack(p.pack);
    REQUIRE(pv);
    CHECK(pv->dir.entries[static_cast<std::size_t>(pack_entry_index(home_tile(0, 0)))].state == PackTileState::Degraded);
    CHECK(pv->dir.entries[static_cast<std::size_t>(pack_entry_index(home_tile(1, 0)))].state == PackTileState::Present);

    // The structural guarantee: the network upstream was never even asked,
    // and the fetcher never called.
    CHECK(net.calls == 0);
    CHECK(net.without_fetcher == 0);
    CHECK(fetcher.calls == 0);
}

TEST_CASE("server: online network upstream, cached after the first fetch", "[server]") {
    World w;
    w.add_network_layer();
    NetUpstream net;
    RecordingFetcher fetcher;
    fetcher.body = elev_container(home_tile(0, 0));
    ServerConfig c;
    c.fetcher = &fetcher;
    c.derived_store = w.derived.get();
    c.upstreams = {&net};
    Server s(c);
    REQUIRE(s.add_release(w.release, "snap1"));
    const ReleaseId rid = w.release.rid;

    const TileResponse t = get_tile(s, rid, kElev, home_tile(0, 0));
    REQUIRE(t.meta.status == Status::Ok);
    CHECK(fetcher.calls == 1);
    CHECK(get_tile(s, rid, kElev, home_tile(0, 0)).container == t.container);
    CHECK(fetcher.calls == 1); // from the derived store now
    CHECK(net.without_fetcher == 0);

    // An upstream reply for the wrong tile is refused (503, not served).
    const TileResponse wrong = get_tile(s, rid, kElev, home_tile(1, 0));
    CHECK(wrong.meta.status == Status::Unavailable);
    CHECK(wrong.meta.retry_after_s == 5);

    // Transient failure: 503 + Retry-After from the upstream.
    fetcher.status = 503;
    fetcher.retry_after = 30;
    TileKey other = home_tile(1, 0);
    const TileResponse u = get_tile(s, rid, kElev, other);
    CHECK(u.meta.status == Status::Unavailable);
    CHECK(u.meta.retry_after_s == 30);
    CHECK_FALSE(u.meta.degraded);
    // Definitive 404 from the upstream.
    fetcher.status = 404;
    CHECK(get_tile(s, rid, kElev, other).meta.status == Status::NotFound);
}

TEST_CASE("server: retired releases answer 410", "[server]") {
    World w;
    LocalSourceUpstream local(*w.sources);
    ServerConfig c;
    c.upstreams = {&local};
    Server s(c);
    REQUIRE(s.add_release(w.release, "snap1"));
    ReleaseManifest next = w.release;
    next.rid[0] ^= 0xff;
    REQUIRE(s.add_release(next, "snap1"));
    CHECK_FALSE(s.retire_release(w.release.rid, w.release.rid)); // successor must stay live
    REQUIRE(s.retire_release(w.release.rid, next.rid));
    const TileResponse t = get_tile(s, w.release.rid, kLc, home_tile());
    CHECK(t.meta.status == Status::Gone);
    REQUIRE(t.meta.newest_rid);
    CHECK(*t.meta.newest_rid == next.rid);
    CHECK(s.release_manifest({w.release.rid}).meta.status == Status::Gone);
    CHECK(s.manifest().manifest.default_rid == next.rid);
    CHECK(s.manifest().manifest.live_releases == std::vector<ReleaseId>{next.rid});
    CHECK(get_tile(s, next.rid, kLc, home_tile()).meta.status == Status::Ok);
}

TEST_CASE("server: packs and hashes", "[server][pack]") {
    World w;
    LocalSourceUpstream local(*w.sources);
    ServerConfig c;
    c.derived_store = w.derived.get();
    c.upstreams = {&local};
    c.derivers = {&w.sum};
    Server s(c);
    REQUIRE(s.add_release(w.release, "snap1"));
    const ReleaseId rid = w.release.rid;
    const TileKey pk = pack_of(home_tile()); // x 336..351: holds 350 and 351

    const PackResponse p = s.pack(PackRequest{rid, kLc, pk});
    REQUIRE(p.meta.status == Status::Ok);
    auto v = decode_pack(p.pack);
    REQUIRE(v);
    int present = 0;
    for (int i = 0; i < kPackTiles; ++i) {
        const PackEntry& e = v->dir.entries[static_cast<std::size_t>(i)];
        if (e.state != PackTileState::Present) {
            CHECK(e.state == PackTileState::NotFound);
            continue;
        }
        ++present;
        const TileResponse t = get_tile(s, rid, kLc, pack_tile(pk, i));
        CHECK(std::vector<std::uint8_t>(v->containers[static_cast<std::size_t>(i)].begin(),
                                        v->containers[static_cast<std::size_t>(i)].end()) == t.container);
    }
    CHECK(present == 2);

    const HashesResponse h = s.hashes(HashesRequest{rid, kLc, pk});
    REQUIRE(h.meta.status == Status::Ok);
    CHECK(h.directory == v->dir);

    // The derived layer in a pack: derived on demand.
    const PackResponse d = s.pack(PackRequest{rid, kSum, pk});
    CHECK(d.meta.status == Status::Ok);
    CHECK(w.sum.calls == 2);

    // No tile of the pack exists: 404.
    TileKey far = pk;
    far.x += 10;
    CHECK(s.pack(PackRequest{rid, kLc, far}).meta.status == Status::NotFound);
    CHECK(s.hashes(HashesRequest{rid, kLc, far}).meta.status == Status::NotFound);
    CHECK(s.pack(PackRequest{rid, "Bad", pk}).meta.status == Status::BadRequest);
    TileKey l1 = pk;
    l1.level = 1;
    CHECK(s.pack(PackRequest{rid, kLc, l1}).meta.status == Status::NotFound);
}

TEST_CASE("server: concurrent requests", "[server]") {
    World w;
    LocalSourceUpstream local(*w.sources);
    ServerConfig c;
    c.derived_store = w.derived.get();
    c.upstreams = {&local};
    c.derivers = {&w.sum};
    Server s(c);
    REQUIRE(s.add_release(w.release, "snap1"));
    std::vector<std::vector<std::uint8_t>> expect;
    for (int dx = 0; dx < 3; ++dx) expect.push_back(get_tile(s, w.release.rid, kSum, home_tile(dx, 0)).container);
    std::atomic<int> bad{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < 4; ++t) {
        threads.emplace_back([&, t] {
            for (int i = 0; i < 60; ++i) {
                const int dx = (i + t) % 3;
                const TileResponse r = get_tile(s, w.release.rid, kSum, home_tile(dx, 0));
                if (r.meta.status != Status::Ok || r.container != expect[static_cast<std::size_t>(dx)]) bad++;
                if (s.pack(PackRequest{w.release.rid, kLc, pack_of(home_tile())}).meta.status != Status::Ok) bad++;
            }
        });
    }
    for (auto& th : threads) th.join();
    CHECK(bad == 0);
    CHECK(w.sum.calls == 3);
}

// A tile is derived at most once at a time: threads asking for the same cold tile at the same moment
// wait for the one resolution in flight instead of deriving it again (resolver per-tile dedup).
TEST_CASE("server: concurrent cold requests derive each tile exactly once", "[server][dedup]") {
    World w;
    w.sum.delay_ms = 40;
    LocalSourceUpstream local(*w.sources);
    ServerConfig c;
    c.derived_store = w.derived.get();
    c.upstreams = {&local};
    c.derivers = {&w.sum};
    Server s(c);
    REQUIRE(s.add_release(w.release, "snap1"));

    constexpr int kThreads = 8;
    std::atomic<int> ready{0};
    std::atomic<bool> go{false};
    std::atomic<int> not_ok{0};
    std::vector<std::vector<std::uint8_t>> got(kThreads);
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&, t] {
            ready++;
            while (!go) std::this_thread::yield();
            // Two distinct tiles, every thread asks for both.
            const TileResponse a = get_tile(s, w.release.rid, kSum, home_tile(0, 0));
            const TileResponse b = get_tile(s, w.release.rid, kSum, home_tile(1, 0));
            if (a.meta.status != Status::Ok || b.meta.status != Status::Ok) not_ok++;
            got[static_cast<std::size_t>(t)] = a.container;
            got[static_cast<std::size_t>(t)].insert(got[static_cast<std::size_t>(t)].end(), b.container.begin(), b.container.end());
        });
    }
    while (ready < kThreads) std::this_thread::yield();
    go = true;
    for (auto& th : threads) th.join();
    CHECK(not_ok == 0);
    CHECK(w.sum.calls == 2);
    for (int t = 1; t < kThreads; ++t) CHECK(got[static_cast<std::size_t>(t)] == got[0]);
}
