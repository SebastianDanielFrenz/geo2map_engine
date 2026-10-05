// src/server/server.cpp - see g2m/server/server.h.

#include "g2m/server/server.h"

#include <algorithm>
#include <map>
#include <mutex>
#include <set>
#include <shared_mutex>

namespace g2m {

// ---- LocalSourceUpstream ---------------------------------------------------

UpstreamReply LocalSourceUpstream::fetch(const UpstreamContext& ctx, std::string_view layer, const TileKey& key) {
    UpstreamReply reply;
    if (ctx.source_scope.empty()) return reply; // NotFound
    auto t = sources_.get(TileAddress{std::string(ctx.source_scope), std::string(layer), key});
    if (!t) {
        reply.kind = UpstreamReply::Kind::Unavailable;
        reply.retry_after_s = 1;
        reply.message = t.error().message;
        return reply;
    }
    if (!*t) return reply;
    reply.kind = UpstreamReply::Kind::Found;
    reply.header = std::move((*t)->header);
    reply.body = std::move((*t)->body);
    return reply;
}

// ---- Server ----------------------------------------------------------------

namespace {

constexpr int kMaxDeriveDepth = 16;

struct LiveRelease {
    ReleaseManifest manifest;
    std::string rid_hex;
    std::string source_scope;
};

struct Resolved {
    enum class Kind : std::uint8_t { Found, NotFound, Unavailable, Degraded, Error };
    Kind kind = Kind::NotFound;
    TileHeader header; // Found: rid set to the release
    std::vector<std::uint8_t> body;
    std::uint32_t retry_after_s = 0;
    std::string message;
};

ResponseMeta meta(Status s, std::string message = {}) {
    ResponseMeta m;
    m.status = s;
    m.message = std::move(message);
    return m;
}

// Status for a non-Found resolution.
ResponseMeta meta_for(const Resolved& r) {
    switch (r.kind) {
    case Resolved::Kind::Found: return meta(Status::Ok);
    case Resolved::Kind::NotFound: return meta(Status::NotFound, r.message);
    case Resolved::Kind::Unavailable: {
        ResponseMeta m = meta(Status::Unavailable, r.message);
        m.retry_after_s = std::max<std::uint32_t>(1, r.retry_after_s);
        return m;
    }
    case Resolved::Kind::Degraded: {
        ResponseMeta m = meta(Status::Unavailable, r.message);
        m.degraded = true;
        return m;
    }
    case Resolved::Kind::Error: return meta(Status::Internal, r.message);
    }
    return meta(Status::Internal);
}

} // namespace

struct Server::Impl {
    ServerConfig cfg;
    IFetcher* fetcher = nullptr; // null whenever cfg.offline
    std::map<std::string, std::vector<ResolveStep>> policies;

    mutable std::shared_mutex mu; // guards the release maps
    std::map<ReleaseId, std::shared_ptr<const LiveRelease>> live;
    std::map<ReleaseId, ReleaseId> retired; // rid -> successor
    std::optional<ReleaseId> default_rid;

    const std::vector<ResolveStep>& steps_for(const std::string& layer) const {
        static const std::vector<ResolveStep> kDefault{ResolveStep::Cache, ResolveStep::Derive, ResolveStep::Upstream};
        const auto it = policies.find(layer);
        return it == policies.end() ? kDefault : it->second;
    }

    const IDeriver* deriver_for(const LayerDesc& desc) const {
        for (const IDeriver* d : cfg.derivers) {
            if (d && d->output().id == desc.id) return d;
        }
        return nullptr;
    }

    // Looks up a release; fills `out` with 404/410 when it is not live.
    std::shared_ptr<const LiveRelease> find_release(const ReleaseId& rid, ResponseMeta& out) const {
        std::shared_lock lock(mu);
        const auto it = live.find(rid);
        if (it != live.end()) return it->second;
        const auto r = retired.find(rid);
        if (r != retired.end()) {
            out = meta(Status::Gone, "release retired");
            out.newest_rid = r->second;
        } else {
            out = meta(Status::NotFound, "unknown release");
        }
        return nullptr;
    }

    // Definitive per-tile checks; nullopt if the tile may exist.
    static std::optional<ResponseMeta> check_tile(const ReleaseLayer* layer, const TileKey& key) {
        if (!layer || layer->availability == Availability::Unavailable) return meta(Status::NotFound, "unknown layer");
        if (!has_level(layer->desc.levels, key.level)) return meta(Status::NotFound, "level not in layer");
        if (!layer->coverage.covers(key)) return meta(Status::NotFound, "outside coverage");
        return std::nullopt;
    }

    void cache_put(const LiveRelease& rel, const Resolved& r) {
        if (!cfg.derived_store) return;
        // A failed cache write only costs a later re-derive/re-fetch.
        (void)cfg.derived_store->put(TileAddress{rel.rid_hex, r.header.layer, r.header.key}, r.header, r.body);
    }

    Resolved resolve(const LiveRelease& rel, const ReleaseLayer& layer, const TileKey& key, int depth) {
        Resolved out;
        if (depth > kMaxDeriveDepth) {
            out.kind = Resolved::Kind::Error;
            out.message = "derivation too deep (cyclic layer inputs?)";
            return out;
        }
        const std::string& name = layer.desc.id.name;
        bool unavailable = false, skipped_network = false;
        std::uint32_t retry_after = 0;
        std::string message;
        const auto note_unavailable = [&](std::uint32_t after, const std::string& msg) {
            unavailable = true;
            retry_after = std::max(retry_after, after > 0 ? after : cfg.retry_after_s);
            if (message.empty()) message = msg;
        };

        for (ResolveStep step : steps_for(name)) {
            if (step == ResolveStep::Cache) {
                if (!cfg.derived_store) continue;
                auto t = cfg.derived_store->get(TileAddress{rel.rid_hex, name, key});
                if (!t) {
                    note_unavailable(0, t.error().message);
                    continue;
                }
                if (*t) {
                    out.kind = Resolved::Kind::Found;
                    out.header = std::move((*t)->header);
                    out.header.rid = rel.manifest.rid; // stored under it; the body hash does not cover the header
                    out.body = std::move((*t)->body);
                    return out;
                }
            } else if (step == ResolveStep::Derive) {
                const IDeriver* d = deriver_for(layer.desc);
                if (!d) continue;
                std::vector<InputTile> inputs;
                for (const InputTileRef& ref : d->inputs_for(key)) {
                    InputTile in;
                    in.ref = ref;
                    const ReleaseLayer* il = rel.manifest.find_layer(ref.layer);
                    if (il && !check_tile(il, ref.key)) {
                        Resolved r = resolve(rel, *il, ref.key, depth + 1);
                        if (r.kind == Resolved::Kind::Found) {
                            in.header = std::move(r.header);
                            in.body = std::move(r.body);
                        } else if (r.kind != Resolved::Kind::NotFound) {
                            return r; // an input is unavailable/degraded/failed: so is this tile
                        }
                    }
                    inputs.push_back(std::move(in));
                }
                DeriveRequest req{key, rel.manifest.world_seed, inputs};
                // Discovery stages (IDeriver::discovery_stages): each stage sees every input resolved so far, and the
                // loop ends at the first stage that names nothing new.
                const std::size_t stages = std::clamp<std::size_t>(d->discovery_stages(), 1, IDeriver::kMaxDiscoveryStages);
                for (std::size_t stage = 0; stage < stages; ++stage) {
                    auto extra = d->additional_inputs_for(req);
                    if (!extra) {
                        out.kind = Resolved::Kind::Error;
                        out.message = "deriver " + name + " dependency discovery: " + extra.error().message;
                        return out;
                    }
                    std::size_t added = 0;
                    if (!extra->empty()) {
                        const auto invalid = [&]() {
                            if (extra->size() > kMaxInputHashes || inputs.size() > kMaxInputHashes ||
                                !std::is_sorted(extra->begin(), extra->end()) ||
                                std::adjacent_find(extra->begin(), extra->end()) != extra->end()) return true;
                            for (const auto& ref : *extra)
                                if (!ref.key.valid() || std::find(d->output().inputs.begin(), d->output().inputs.end(),
                                    ref.layer) == d->output().inputs.end()) return true;
                            return false;
                        };
                        if (invalid()) {
                            out.kind = Resolved::Kind::Error; out.message = "deriver " + name + ": invalid discovered dependencies";
                            return out;
                        }
                        for (const auto& ref : *extra) {
                            if (std::find_if(inputs.begin(), inputs.end(), [&](const auto& in) { return in.ref == ref; }) != inputs.end()) continue;
                            if (inputs.size() == kMaxInputHashes) {
                                out.kind = Resolved::Kind::Error; out.message = "deriver " + name + ": dependency count limit";
                                return out;
                            }
                            InputTile in; in.ref = ref;
                            const auto* il = rel.manifest.find_layer(ref.layer);
                            if (il && !check_tile(il, ref.key)) {
                                auto r = resolve(rel, *il, ref.key, depth + 1);
                                if (r.kind == Resolved::Kind::Found) {
                                    in.header = std::move(r.header); in.body = std::move(r.body);
                                } else if (r.kind != Resolved::Kind::NotFound) return r;
                            }
                            inputs.push_back(std::move(in)); ++added;
                        }
                        std::sort(inputs.begin(), inputs.end(), [](const auto& a, const auto& b) { return a.ref < b.ref; });
                        req.inputs = inputs; // vector growth invalidated the first-stage span
                    }
                    if (!added) break;
                }
                auto res = d->derive(req);
                if (!res) {
                    out.kind = Resolved::Kind::Error;
                    out.message = "deriver " + name + ": " + res.error().message;
                    return out;
                }
                out.kind = Resolved::Kind::Found;
                out.header.layer = name;
                out.header.layer_version = layer.desc.id.version;
                out.header.rid = rel.manifest.rid;
                out.header.key = key;
                out.header.datum = res->datum;
                out.header.height_offset = res->height_offset;
                for (const InputTile& in : inputs) {
                    if (in.header) out.header.inputs.push_back(in.header->body_hash);
                }
                std::sort(out.header.inputs.begin(), out.header.inputs.end());
                out.header.inputs.erase(std::unique(out.header.inputs.begin(), out.header.inputs.end()),
                                        out.header.inputs.end());
                out.body = std::move(res->body);
                out.header.body_hash = content_hash(out.body);
                out.header.body_size = out.body.size();
                cache_put(rel, out);
                return out;
            } else { // Upstream
                for (IUpstream* up : cfg.upstreams) {
                    if (!up) continue;
                    const bool net = up->needs_network();
                    if (net && !fetcher) {
                        skipped_network = true;
                        continue;
                    }
                    UpstreamContext ctx;
                    ctx.fetcher = net ? fetcher : nullptr;
                    ctx.release = &rel.manifest;
                    ctx.source_scope = rel.source_scope;
                    UpstreamReply reply = up->fetch(ctx, name, key);
                    if (reply.kind == UpstreamReply::Kind::NotFound) continue;
                    if (reply.kind == UpstreamReply::Kind::Unavailable) {
                        note_unavailable(reply.retry_after_s, std::string(up->name()) + ": " + reply.message);
                        continue;
                    }
                    TileHeader& h = reply.header;
                    if (h.layer != name || h.key != key || h.layer_version != layer.desc.id.version ||
                        h.body_size != reply.body.size() || content_hash(reply.body) != h.body_hash) {
                        note_unavailable(0, std::string(up->name()) + ": reply does not match the request");
                        continue;
                    }
                    out.kind = Resolved::Kind::Found;
                    out.header = std::move(h);
                    out.header.rid = rel.manifest.rid;
                    out.body = std::move(reply.body);
                    if (net) cache_put(rel, out);
                    return out;
                }
            }
        }
        if (unavailable) {
            out.kind = Resolved::Kind::Unavailable;
            out.retry_after_s = retry_after;
            out.message = message;
        } else if (skipped_network) {
            out.kind = Resolved::Kind::Degraded;
            out.message = "offline: not cached";
        } else {
            out.kind = Resolved::Kind::NotFound;
            out.message = "no store, deriver or upstream has this tile";
        }
        return out;
    }

    // Shared by pack() and hashes(): resolves all 256 tiles of a pack.
    struct PackResult {
        ResponseMeta meta;
        PackDirectory dir;
        std::array<std::vector<std::uint8_t>, kPackTiles> containers;
        bool has_data = false;
    };
    PackResult resolve_pack(const ReleaseId& rid, const std::string& layer_name, const TileKey& pack, bool want_data) {
        PackResult out;
        if (!is_valid_layer_name(layer_name) || !pack.valid() || !pack_tile(pack, 0).valid() ||
            !pack_tile(pack, kPackTiles - 1).valid()) {
            out.meta = meta(Status::BadRequest, "bad layer or pack key");
            return out;
        }
        auto rel = find_release(rid, out.meta);
        if (!rel) return out;
        const ReleaseLayer* layer = rel->manifest.find_layer(layer_name);
        if (!layer || layer->availability == Availability::Unavailable) {
            out.meta = meta(Status::NotFound, "unknown layer");
            return out;
        }
        if (!has_level(layer->desc.levels, pack.level)) {
            out.meta = meta(Status::NotFound, "level not in layer");
            return out;
        }
        out.dir.layer = layer_name;
        out.dir.rid = rid;
        out.dir.pack = pack;
        bool any_present = false, any_missing_now = false, degraded = false;
        std::uint32_t retry_after = 0;
        for (int i = 0; i < kPackTiles; ++i) {
            const TileKey t = pack_tile(pack, i);
            PackEntry& e = out.dir.entries[static_cast<std::size_t>(i)];
            if (!layer->coverage.covers(t)) continue; // NotFound
            Resolved r = resolve(*rel, *layer, t, 0);
            switch (r.kind) {
            case Resolved::Kind::Found: {
                e.state = PackTileState::Present;
                e.body_hash = r.header.body_hash;
                any_present = true;
                if (want_data) {
                    auto hb = encode_header(r.header);
                    if (!hb) {
                        e = PackEntry{PackTileState::Unavailable, {}};
                        any_missing_now = true;
                        break;
                    }
                    auto& c = out.containers[static_cast<std::size_t>(i)];
                    c = std::move(*hb);
                    c.insert(c.end(), r.body.begin(), r.body.end());
                }
                break;
            }
            case Resolved::Kind::NotFound: break;
            case Resolved::Kind::Degraded:
                e.state = PackTileState::Degraded;
                any_missing_now = true;
                degraded = true;
                break;
            case Resolved::Kind::Unavailable:
            case Resolved::Kind::Error:
                e.state = PackTileState::Unavailable;
                any_missing_now = true;
                retry_after = std::max(retry_after, std::max<std::uint32_t>(1, r.retry_after_s > 0 ? r.retry_after_s
                                                                                                   : cfg.retry_after_s));
                break;
            }
        }
        if (any_missing_now) {
            out.meta = meta(Status::Unavailable, "some tiles are not available now");
            out.meta.degraded = degraded;
            out.meta.retry_after_s = retry_after;
        } else if (!any_present) {
            out.meta = meta(Status::NotFound, "no tile of this pack exists");
            return out;
        } else {
            out.meta = meta(Status::Ok);
        }
        out.has_data = true;
        return out;
    }
};

Server::Server(ServerConfig config) : impl_(std::make_unique<Impl>()) {
    impl_->cfg = std::move(config);
    // Offline is structural: the fetcher is not even kept.
    impl_->fetcher = impl_->cfg.offline ? nullptr : impl_->cfg.fetcher;
    impl_->cfg.fetcher = nullptr;
    for (const LayerPolicy& p : impl_->cfg.policies) impl_->policies[p.layer] = p.steps;
}

Server::~Server() = default;

bool Server::offline() const { return impl_->cfg.offline; }

Result<void> Server::add_release(ReleaseManifest manifest, std::string source_scope) {
    auto rel = std::make_shared<LiveRelease>();
    rel->rid_hex = to_hex(manifest.rid);
    rel->manifest = std::move(manifest);
    rel->source_scope = std::move(source_scope);
    std::unique_lock lock(impl_->mu);
    const ReleaseId rid = rel->manifest.rid;
    if (impl_->live.contains(rid)) return make_error(ErrorCode::InvalidArgument, "release already live");
    impl_->retired.erase(rid);
    impl_->live.emplace(rid, std::move(rel));
    if (!impl_->default_rid) impl_->default_rid = rid;
    return {};
}

Result<void> Server::set_default_release(const ReleaseId& rid) {
    std::unique_lock lock(impl_->mu);
    if (!impl_->live.contains(rid)) return make_error(ErrorCode::NotFound, "release not live");
    impl_->default_rid = rid;
    return {};
}

Result<void> Server::retire_release(const ReleaseId& rid, const ReleaseId& successor) {
    std::unique_lock lock(impl_->mu);
    if (!impl_->live.contains(rid)) return make_error(ErrorCode::NotFound, "release not live");
    if (successor == rid || !impl_->live.contains(successor)) {
        return make_error(ErrorCode::InvalidArgument, "successor must be another live release");
    }
    impl_->live.erase(rid);
    impl_->retired[rid] = successor;
    if (impl_->default_rid == rid) impl_->default_rid = successor;
    return {};
}

ManifestResponse Server::manifest() {
    ManifestResponse r;
    r.manifest.server_id = impl_->cfg.server_id;
    r.manifest.contact = impl_->cfg.contact;
    r.manifest.tile_base_url = impl_->cfg.tile_base_url;
    std::shared_lock lock(impl_->mu);
    if (impl_->default_rid) r.manifest.default_rid = *impl_->default_rid;
    for (const auto& [rid, rel] : impl_->live) r.manifest.live_releases.push_back(rid); // map: sorted
    return r;
}

ReleaseManifestResponse Server::release_manifest(const ReleaseManifestRequest& request) {
    ReleaseManifestResponse r;
    auto rel = impl_->find_release(request.rid, r.meta);
    if (rel) r.manifest = rel->manifest;
    return r;
}

TileResponse Server::tile(const TileRequest& request) {
    TileResponse r;
    if (!is_valid_layer_name(request.layer) || !request.key.valid()) {
        r.meta = meta(Status::BadRequest, "bad layer or key");
        return r;
    }
    auto rel = impl_->find_release(request.rid, r.meta);
    if (!rel) return r;
    const ReleaseLayer* layer = rel->manifest.find_layer(request.layer);
    if (auto m = Impl::check_tile(layer, request.key)) {
        r.meta = std::move(*m);
        return r;
    }
    Resolved res = impl_->resolve(*rel, *layer, request.key, 0);
    if (res.kind != Resolved::Kind::Found) {
        r.meta = meta_for(res);
        return r;
    }
    r.meta.etag = res.header.body_hash;
    if (request.if_none_match && *request.if_none_match == res.header.body_hash) {
        r.meta.status = Status::NotModified;
        return r;
    }
    auto hb = encode_header(res.header);
    if (!hb) {
        r.meta = meta(Status::Internal, hb.error().message);
        return r;
    }
    r.container = std::move(*hb);
    r.container.insert(r.container.end(), res.body.begin(), res.body.end());
    return r;
}

PackResponse Server::pack(const PackRequest& request) {
    PackResponse r;
    auto p = impl_->resolve_pack(request.rid, request.layer, request.pack, true);
    r.meta = std::move(p.meta);
    if (!p.has_data) return r;
    auto bytes = encode_pack(p.dir, p.containers);
    if (!bytes) {
        r.meta = meta(Status::Internal, bytes.error().message);
        return r;
    }
    r.pack = std::move(*bytes);
    return r;
}

HashesResponse Server::hashes(const HashesRequest& request) {
    HashesResponse r;
    auto p = impl_->resolve_pack(request.rid, request.layer, request.pack, false);
    r.meta = std::move(p.meta);
    if (p.has_data) r.directory = std::move(p.dir);
    return r;
}

Response Server::handle(const Request& request) {
    struct V {
        Server& s;
        Response operator()(const ManifestRequest&) const { return s.manifest(); }
        Response operator()(const ReleaseManifestRequest& q) const { return s.release_manifest(q); }
        Response operator()(const TileRequest& q) const { return s.tile(q); }
        Response operator()(const PackRequest& q) const { return s.pack(q); }
        Response operator()(const HashesRequest& q) const { return s.hashes(q); }
    };
    return std::visit(V{*this}, request);
}

// ---- local release ----------------------------------------------------------

Result<ReleaseManifest> make_local_release(TileStore& sources, const LocalReleaseParams& params) {
    const LayerRegistry& registry = params.registry ? *params.registry : LayerRegistry::builtin();
    auto index = sources.index(params.source_scope);
    if (!index) return index.error();

    ReleaseInputs in;
    in.world_seed = params.world_seed;
    std::map<std::string, std::set<TileKey>> level2; // layer -> covered level-2 tiles
    std::map<std::string, const LayerDesc*> raw_layers;
    for (const TileIndexEntry& e : *index) {
        const LayerDesc* d = registry.find(e.layer);
        if (!d || d->kind != LayerKind::Raw) {
            return make_error(ErrorCode::InvalidArgument, "source store holds tiles of non-raw layer '" + e.layer + "'");
        }
        raw_layers[e.layer] = d;
        in.sources.push_back(SourceTileRef{e.layer, e.key.packed(), e.body_hash});
        if (e.key.level == kRawTileLevel) level2[e.layer].insert(e.key);
    }

    ReleaseManifest m;
    m.world_seed = params.world_seed;
    m.sources = params.sources;
    std::sort(m.sources.begin(), m.sources.end(),
              [](const ReleaseSource& a, const ReleaseSource& b) { return a.id < b.id; });
    for (const auto& [name, desc] : raw_layers) {
        ReleaseLayer l;
        l.desc = *desc;
        l.availability = Availability::Prebaked;
        const auto& keys = level2[name];
        l.coverage = Coverage::from_level2_tiles(std::vector<TileKey>(keys.begin(), keys.end()));
        m.layers.push_back(std::move(l));
        in.layers.push_back(LayerVersionRef{name, desc->id.version, {}});
    }
    for (const IDeriver* d : params.derivers) {
        if (!d) continue;
        const LayerDesc& desc = d->output();
        std::set<TileKey> keys;
        for (const std::string& input : d->coverage_inputs()) {
            if (std::find(desc.inputs.begin(), desc.inputs.end(), input) == desc.inputs.end())
                return make_error(ErrorCode::InvalidArgument, "local release: undeclared coverage input");
            const auto it = level2.find(input);
            if (it != level2.end()) keys.insert(it->second.begin(), it->second.end());
        }
        level2[desc.id.name] = keys;
        ReleaseLayer l;
        l.desc = desc;
        l.config_hash = d->config_hash();
        l.availability = Availability::OnDemand;
        l.coverage = Coverage::from_level2_tiles(std::vector<TileKey>(keys.begin(), keys.end()));
        in.layers.push_back(LayerVersionRef{desc.id.name, desc.id.version, l.config_hash});
        m.layers.push_back(std::move(l));
    }
    std::sort(m.layers.begin(), m.layers.end(),
              [](const ReleaseLayer& a, const ReleaseLayer& b) { return a.desc.id.name < b.desc.id.name; });
    for (const IClassifier* c : params.classifiers) {
        if (!c) continue;
        ClassifierRef ref{std::string(c->name()), c->table_hash()};
        m.classifiers.push_back(ref);
        in.classifiers.push_back(std::move(ref));
    }
    std::sort(m.classifiers.begin(), m.classifiers.end(),
              [](const ClassifierRef& a, const ClassifierRef& b) { return a.name < b.name; });
    auto rid = compute_release_id(in);
    if (!rid) return rid.error();
    m.rid = *rid;
    return m;
}

} // namespace g2m
