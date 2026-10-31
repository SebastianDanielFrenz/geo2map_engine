// src/world/terrain_service.cpp - see include/g2m/world/terrain_service.h.
//
// Layout: Impl owns the stores, the in-process server, the memory cache and the job scheduler.
// Every builtin product is one `Job` {key, type, fn}: the async request path submits it to the
// pool, ProductContext joins it. A job's fn does the transport call and the decode and returns an
// ErasedOutcome; the wrapper (Impl::wrap) puts Ready/Absent outcomes into the memory cache BEFORE
// the job is published as finished, so a request arriving right after it ends finds the cache.
#include "g2m/world/terrain_service.h"

#include "g2m/client/in_process_transport.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/soil_tile.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/terrain_carve.h"
#include "g2m/layer/terrain_class.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/builtin_release.h"
#include "g2m/server/tile_store.h"
#include "g2m/world/height_sampler.h"
#include "g2m/world/tile_fetch.h"

#include "job_scheduler.h"
#include "tile_cache.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <mutex>
#include <set>
#include <thread>
#include <utility>
#include <variant>

namespace g2m::world {

using detail::ErasedOutcome;
using detail::JobCore;
using detail::JobKey;
using detail::JobScheduler;

namespace {

// Product kinds. Registered (caller-defined) kinds start at kFirstUserKind.
constexpr ProductKind kKindHeight = 1;
constexpr ProductKind kKindRoadGeom = 2;
constexpr ProductKind kKindRoadSegments = 3;
constexpr ProductKind kKindSourceOsm = 4;
constexpr ProductKind kKindRaw = 5;
constexpr ProductKind kKindStructures = 6;
constexpr ProductKind kKindRibbons = 7;
constexpr ProductKind kKindBuildings = 8;
constexpr ProductKind kKindWarm = 9;
constexpr ProductKind kKindTerrainClass = 10;
constexpr ProductKind kKindRailGeom = 11;
constexpr ProductKind kKindFields = 12;
constexpr ProductKind kKindSoil = 13;
constexpr ProductKind kFirstUserKind = 100;

// Type identity used by the scheduler and the cache. hash_code() is equal for equal types in one
// program, and unlike the address of the type_info it cannot differ between translation units.
const void* type_id(const std::type_info& t) { return reinterpret_cast<const void*>(t.hash_code()); }

std::uint64_t fnv1a(std::string_view s) {
    std::uint64_t h = 1469598103934665603ULL;
    for (const char c : s) {
        h ^= static_cast<std::uint8_t>(c);
        h *= 1099511628211ULL;
    }
    return h;
}

struct WarmDone {};

ErasedOutcome failed(std::string why) {
    ErasedOutcome o;
    o.state = TileState::Failed;
    o.message = std::move(why);
    return o;
}

template <class T>
Outcome<T> to_outcome(const ErasedOutcome& e) {
    Outcome<T> o;
    o.state = e.state;
    o.value = std::static_pointer_cast<const T>(e.value);
    o.message = e.message;
    o.bytes = e.bytes;
    return o;
}

// One transport round trip.
struct Fetched {
    Status status = Status::Internal;
    std::string message;
    std::vector<std::uint8_t> container;
};

// The ancestor at the raw tile level (2); a key already at or above it is returned unchanged.
TileKey to_level2(TileKey key) {
    while (key.level < kRawTileLevel) key = key.parent();
    return key;
}

} // namespace

// ---------------------------------------------------------------------------------------------
// Impl
// ---------------------------------------------------------------------------------------------

struct TerrainService::Impl {
    struct Job {
        JobKey key;
        const std::type_info* type = nullptr;
        std::function<ErasedOutcome()> fn;
    };

    explicit Impl(const TerrainServiceConfig& c) : cfg(c), derivers(c.elevation_source), cache(c.cache_bytes) {}

    TerrainServiceConfig cfg;
    std::unique_ptr<TileStore> sources;
    std::unique_ptr<TileStore> derived;
    BuiltinTerrainDerivers derivers;
    std::optional<LocalSourceUpstream> upstream;
    std::unique_ptr<Server> server;
    std::shared_ptr<ITransport> transport;
    ReleaseManifest manifest;
    std::string rid_hex;
    std::unique_ptr<geo::SessionFrame> frame;
    std::string height_layer;
    int workers = 1;
    detail::TileCache cache;

    mutable std::mutex producer_mu;
    Producer<StructureSet> structures_producer;
    Producer<RoadRibbonTile> ribbons_producer;
    Producer<BuildingSet> buildings_producer;

    std::shared_ptr<JobScheduler> sched; // last: destroyed (and its threads joined) first

    // ---- plumbing -----------------------------------------------------------------------------

    std::function<ErasedOutcome()> wrap(const JobKey& jk, const void* type, std::function<ErasedOutcome()> fn,
                                        bool cache_result) {
        return [this, jk, type, fn = std::move(fn), cache_result]() -> ErasedOutcome {
            ErasedOutcome o = fn();
            if (cache_result) cache.put(jk, type, o);
            return o;
        };
    }
    JobScheduler::Probe probe(const JobKey& jk, const void* type) {
        return [this, jk, type]() -> std::optional<ErasedOutcome> { return cache.get(jk, type); };
    }

    JobScheduler::Submitted request(const Job& job, Priority priority, bool cache_result = true) {
        const void* t = type_id(*job.type);
        return sched->submit(job.key, t, priority, wrap(job.key, t, job.fn, cache_result),
                             cache_result ? probe(job.key, t) : JobScheduler::Probe{});
    }
    template <class T>
    Ticket<T> ticket(const Job& job, Priority priority) {
        auto s = request(job, priority);
        return Ticket<T>(std::move(s.job), std::move(s.interest));
    }
    template <class T>
    Outcome<T> join(const Job& job) {
        const void* t = type_id(*job.type);
        return to_outcome<T>(sched->join_or_run(job.key, t, wrap(job.key, t, job.fn, true), probe(job.key, t)));
    }
    template <class T>
    Ticket<T> finished_ticket(const JobKey& jk, ErasedOutcome o) {
        return Ticket<T>(JobScheduler::make_finished(jk, type_id(typeid(T)), std::move(o)), nullptr);
    }

    Fetched fetch_container(std::string_view layer, const TileKey& key) {
        Response r = transport->send(Request{TileRequest{manifest.rid, std::string(layer), key, std::nullopt}});
        auto* t = std::get_if<TileResponse>(&r);
        if (t == nullptr) return Fetched{Status::Internal, "non-tile response", {}};
        return Fetched{t->meta.status, std::move(t->meta.message), std::move(t->container)};
    }

    static ErasedOutcome status_outcome(const Fetched& f, std::string_view layer, const TileKey& key) {
        ErasedOutcome o;
        if (f.status == Status::NotFound) {
            o.state = TileState::Absent;
            o.message = f.message.empty() ? "not found" : f.message;
            return o;
        }
        o.state = TileState::Failed;
        o.message = std::string(layer) + " " + to_string(key) + ": status " +
                    std::to_string(static_cast<unsigned>(f.status)) + (f.message.empty() ? "" : " (" + f.message + ")");
        return o;
    }

    // Parse + verify the container and check it is the tile that was asked for.
    static Result<TileBody> open_body(const Fetched& f, std::string_view layer, const TileKey& key) {
        const Result<ContainerView> view = parse_container(f.container);
        if (!view.ok()) return view.error();
        if (view.value().header.layer != layer || !(view.value().header.key == key))
            return make_error(ErrorCode::Malformed, "container header does not match the request");
        return decode_body(view.value().body);
    }

    // ---- builtin products ---------------------------------------------------------------------

    Job job_height(const TileKey& key) {
        return Job{{kKindHeight, key.packed(), 0}, &typeid(HeightTile), [this, key]() -> ErasedOutcome {
                       const Fetched f = fetch_container(height_layer, key);
                       if (f.status != Status::Ok) return status_outcome(f, height_layer, key);
                       HeightTileFetchResult d = decode_height_tile_container(f.container, height_layer, key);
                       if (d.status != Status::Ok || !d.tile)
                           return failed(height_layer + " " + to_string(key) + ": decode failed");
                       return detail::erase<HeightTile>(Outcome<HeightTile>::ready(std::move(d.tile), sizeof(HeightTile)));
                   }};
    }

    Job job_terrain_class(const TileKey& key) {
        return Job{{kKindTerrainClass, key.packed(), 0}, &typeid(TerrainClassTile), [this, key]() -> ErasedOutcome {
                       const std::string_view layer = kTerrainClassLayer;
                       const Fetched f = fetch_container(layer, key);
                       if (f.status != Status::Ok) return status_outcome(f, layer, key);
                       const Result<TileBody> body = open_body(f, layer, key);
                       if (!body.ok()) return failed(std::string(layer) + " " + to_string(key) + ": " + body.error().message);
                       auto tile = std::make_shared<TerrainClassTile>();
                       const Result<void> decoded = decode_terrain_class(body.value(), key, *tile);
                       if (!decoded.ok()) return failed(std::string(layer) + " " + to_string(key) + ": " + decoded.error().message);
                       const std::size_t bytes = 4 * kTerrainClassNodes + 256;
                       return detail::erase<TerrainClassTile>(Outcome<TerrainClassTile>::ready(std::move(tile), bytes));
                   }};
    }

    Job job_road_geom(const TileKey& l2) {
        return Job{{kKindRoadGeom, l2.packed(), 0}, &typeid(RoadGeomTile), [this, l2]() -> ErasedOutcome {
                       const std::string_view layer = kRoadGeomLayer;
                       const Fetched f = fetch_container(layer, l2);
                       if (f.status != Status::Ok) return status_outcome(f, layer, l2);
                       const Result<TileBody> body = open_body(f, layer, l2);
                       if (!body.ok()) return failed(std::string(layer) + " " + to_string(l2) + ": " + body.error().message);
                       Result<RoadGeomTile> tile = decode_road_geom(body.value());
                       if (!tile.ok()) return failed(std::string(layer) + " " + to_string(l2) + ": " + tile.error().message);
                       auto p = std::make_shared<const RoadGeomTile>(std::move(tile.value()));
                       return detail::erase<RoadGeomTile>(Outcome<RoadGeomTile>::ready(std::move(p), f.container.size() * 4 + 4096));
                   }};
    }

    Job job_rail_geom(const TileKey& l2) {
        return Job{{kKindRailGeom, l2.packed(), 0}, &typeid(RailGeomTile), [this, l2]() -> ErasedOutcome {
                       const std::string_view layer = kRailGeomLayer;
                       const Fetched f = fetch_container(layer, l2);
                       if (f.status != Status::Ok) return status_outcome(f, layer, l2);
                       const Result<TileBody> body = open_body(f, layer, l2);
                       if (!body.ok()) return failed(std::string(layer) + " " + to_string(l2) + ": " + body.error().message);
                       Result<RailGeomTile> tile = decode_rail_geom(body.value(), l2);
                       if (!tile.ok()) return failed(std::string(layer) + " " + to_string(l2) + ": " + tile.error().message);
                       auto p = std::make_shared<const RailGeomTile>(std::move(tile.value()));
                       return detail::erase<RailGeomTile>(Outcome<RailGeomTile>::ready(std::move(p), f.container.size() * 4 + 4096));
                   }};
    }

    Job job_fields(const TileKey& l2) {
        return Job{{kKindFields, l2.packed(), 0}, &typeid(FieldTile), [this, l2]() -> ErasedOutcome {
                       const std::string_view layer = kFieldsGeomLayer;
                       const Fetched f = fetch_container(layer, l2);
                       if (f.status != Status::Ok) return status_outcome(f, layer, l2);
                       const Result<TileBody> body = open_body(f, layer, l2);
                       if (!body.ok()) return failed(std::string(layer) + " " + to_string(l2) + ": " + body.error().message);
                       Result<FieldTile> tile = decode_fields_geom(body.value());
                       if (!tile.ok()) return failed(std::string(layer) + " " + to_string(l2) + ": " + tile.error().message);
                       auto p = std::make_shared<const FieldTile>(std::move(tile.value()));
                       return detail::erase<FieldTile>(Outcome<FieldTile>::ready(std::move(p), f.container.size() * 4 + 4096));
                   }};
    }

    Job job_soil(const TileKey& l2) {
        return Job{{kKindSoil, l2.packed(), 0}, &typeid(SoilGridTile), [this, l2]() -> ErasedOutcome {
                       const std::string_view layer = kSoilGridLayer;
                       const Fetched f = fetch_container(layer, l2);
                       if (f.status != Status::Ok) return status_outcome(f, layer, l2);
                       const Result<TileBody> body = open_body(f, layer, l2);
                       if (!body.ok()) return failed(std::string(layer) + " " + to_string(l2) + ": " + body.error().message);
                       Result<SoilGridTile> tile = decode_soil_grid(body.value());
                       if (!tile.ok()) return failed(std::string(layer) + " " + to_string(l2) + ": " + tile.error().message);
                       auto p = std::make_shared<const SoilGridTile>(std::move(tile.value()));
                       return detail::erase<SoilGridTile>(Outcome<SoilGridTile>::ready(std::move(p), f.container.size() * 4 + 4096));
                   }};
    }

    Job job_source_osm(const TileKey& l2) {
        return Job{{kKindSourceOsm, l2.packed(), 0}, &typeid(OsmTile), [this, l2]() -> ErasedOutcome {
                       const std::string_view layer = kSrcOsmLayer;
                       const Fetched f = fetch_container(layer, l2);
                       if (f.status != Status::Ok) return status_outcome(f, layer, l2);
                       const Result<TileBody> body = open_body(f, layer, l2);
                       if (!body.ok()) return failed(std::string(layer) + " " + to_string(l2) + ": " + body.error().message);
                       Result<OsmTile> tile = decode_src_osm(body.value());
                       if (!tile.ok()) return failed(std::string(layer) + " " + to_string(l2) + ": " + tile.error().message);
                       auto p = std::make_shared<const OsmTile>(std::move(tile.value()));
                       return detail::erase<OsmTile>(Outcome<OsmTile>::ready(std::move(p), f.container.size() * 4 + 4096));
                   }};
    }

    Job job_road_segments(const TileKey& l2) {
        return Job{{kKindRoadSegments, l2.packed(), 0}, &typeid(RoadSegmentsTile), [this, l2]() -> ErasedOutcome {
                       // The decoded g2m.src.osm tile is its own product: joined if it is running, derived here
                       // otherwise, and cached for the next consumer (ProductContext::source_osm).
                       const Outcome<OsmTile> osm = join<OsmTile>(job_source_osm(l2));
                       if (osm.state == TileState::Absent) return detail::erase<RoadSegmentsTile>(Outcome<RoadSegmentsTile>::absent(osm.message));
                       if (osm.state != TileState::Ready) return failed(osm.message);
                       auto tile = std::make_shared<RoadSegmentsTile>();
                       tile->key = l2;
                       tile->segments = extract_road_segments(osm.value->data, frame->zone(), RoadStyle::default_style());
                       const std::size_t bytes = tile->segments.size() * sizeof(RoadSegment) + 256;
                       return detail::erase<RoadSegmentsTile>(Outcome<RoadSegmentsTile>::ready(std::move(tile), bytes));
                   }};
    }

    Job job_raw(const std::string& layer, const TileKey& key) {
        return Job{{kKindRaw, key.packed(), fnv1a(layer)}, &typeid(RawTile), [this, layer, key]() -> ErasedOutcome {
                       Fetched f = fetch_container(layer, key);
                       if (f.status != Status::Ok) return status_outcome(f, layer, key);
                       const Result<ContainerView> view = parse_container(f.container);
                       if (!view.ok()) return failed(layer + " " + to_string(key) + ": " + view.error().message);
                       auto tile = std::make_shared<RawTile>();
                       tile->key = key;
                       tile->layer = layer;
                       tile->header = view.value().header;
                       const std::size_t bytes = f.container.size() + 256;
                       tile->container = std::move(f.container);
                       return detail::erase<RawTile>(Outcome<RawTile>::ready(std::move(tile), bytes));
                   }};
    }

    template <class T>
    Job job_producer(ProductKind kind, const TileKey& l2, std::uint64_t options, Producer<T> producer,
                     TerrainService& owner) {
        return Job{{kind, l2.packed(), options}, &typeid(T), [&owner, producer = std::move(producer), l2]() -> ErasedOutcome {
                       ProductContext ctx(owner);
                       return detail::erase<T>(producer(ctx, l2));
                   }};
    }

    Job job_warm(const WarmItem& item) {
        return Job{{kKindWarm, item.key.packed(), fnv1a(item.layer)}, &typeid(WarmDone), [this, item]() -> ErasedOutcome {
                       const Fetched f = fetch_container(item.layer, item.key);
                       if (f.status != Status::Ok) return status_outcome(f, item.layer, item.key);
                       static const auto kDone = std::make_shared<const WarmDone>();
                       return detail::erase<WarmDone>(Outcome<WarmDone>::ready(kDone, 0));
                   }};
    }
};

// ---------------------------------------------------------------------------------------------
// ProductContext
// ---------------------------------------------------------------------------------------------

Outcome<HeightTile> ProductContext::height(const TileKey& key) { return svc_.impl_->join<HeightTile>(svc_.impl_->job_height(key)); }
Outcome<RailGeomTile> ProductContext::rail_geom(const TileKey& l2) {
    return svc_.impl_->join<RailGeomTile>(svc_.impl_->job_rail_geom(to_level2(l2)));
}
Outcome<RoadGeomTile> ProductContext::road_geom(const TileKey& l2) {
    return svc_.impl_->join<RoadGeomTile>(svc_.impl_->job_road_geom(to_level2(l2)));
}
Outcome<RoadSegmentsTile> ProductContext::road_segments(const TileKey& l2) {
    return svc_.impl_->join<RoadSegmentsTile>(svc_.impl_->job_road_segments(to_level2(l2)));
}
Outcome<OsmTile> ProductContext::source_osm(const TileKey& l2) {
    return svc_.impl_->join<OsmTile>(svc_.impl_->job_source_osm(to_level2(l2)));
}
Outcome<RawTile> ProductContext::raw_tile(const std::string& layer, const TileKey& key) {
    return svc_.impl_->join<RawTile>(svc_.impl_->job_raw(layer, key));
}
const geo::SessionFrame& ProductContext::frame() const { return *svc_.impl_->frame; }

// ---------------------------------------------------------------------------------------------
// Warm-up
// ---------------------------------------------------------------------------------------------

namespace detail {

struct WarmState {
    std::mutex mu;
    std::condition_variable cv;
    std::vector<WarmItem> items;
    Priority priority = Priority::Warm;
    int window = 1;
    std::size_t next = 0;
    int inflight = 0;
    WarmProgress progress;
    bool cancelled = false;
    bool pumping = false, again = false;
    std::map<std::size_t, Ticket<WarmDone>> tickets;
    std::function<TerrainService::Submitted(const WarmItem&, Priority)> submit;
    std::function<void(const std::shared_ptr<JobCore>&, std::function<void()>)> hook;
};

namespace {

void warm_check_finish_locked(WarmState& st) {
    if (!st.progress.finished && st.next >= st.items.size() && st.inflight == 0) {
        st.progress.finished = true;
        st.cv.notify_all();
    }
}

void warm_pump(const std::shared_ptr<WarmState>& st);

void warm_done(const std::shared_ptr<WarmState>& st, std::size_t idx) {
    {
        std::lock_guard lock(st->mu);
        const auto it = st->tickets.find(idx);
        if (it == st->tickets.end()) return; // already accounted (cancel())
        switch (it->second.state()) {
            case TileState::Ready: ++st->progress.ready; break;
            case TileState::Absent: ++st->progress.absent; break;
            case TileState::Failed: ++st->progress.failed; break;
            default: ++st->progress.cancelled; break;
        }
        st->tickets.erase(it);
        --st->inflight;
    }
    warm_pump(st);
}

// Keeps up to `window` items in flight. Re-entrant calls (a hook that runs at once because its job was
// already finished) only set `again`, so the stack does not grow with the item count.
void warm_pump(const std::shared_ptr<WarmState>& st) {
    {
        std::lock_guard lock(st->mu);
        if (st->pumping) {
            st->again = true;
            return;
        }
        st->pumping = true;
    }
    for (;;) {
        std::vector<std::pair<std::size_t, WarmItem>> batch;
        {
            std::lock_guard lock(st->mu);
            while (!st->cancelled && st->next < st->items.size() && st->inflight < st->window) {
                batch.emplace_back(st->next, st->items[st->next]);
                ++st->next;
                ++st->inflight;
            }
            if (batch.empty()) {
                if (st->again) {
                    st->again = false;
                    continue;
                }
                st->pumping = false;
                warm_check_finish_locked(*st);
                return;
            }
        }
        for (auto& [idx, item] : batch) {
            TerrainService::Submitted s = st->submit(item, st->priority);
            Ticket<WarmDone> t(s.job, s.interest);
            bool abandon = false;
            {
                std::lock_guard lock(st->mu);
                if (st->cancelled) {
                    ++st->progress.cancelled;
                    --st->inflight;
                    warm_check_finish_locked(*st);
                    abandon = true;
                } else {
                    st->tickets.emplace(idx, t);
                }
            }
            if (abandon) {
                t.cancel();
                continue;
            }
            st->hook(s.job, [st, idx = idx] { warm_done(st, idx); });
        }
    }
}

} // namespace
} // namespace detail

WarmProgress WarmJob::progress() const {
    if (!state_) return WarmProgress{0, 0, 0, 0, 0, true};
    std::lock_guard lock(state_->mu);
    return state_->progress;
}

void WarmJob::cancel() {
    if (!state_) return;
    std::vector<Ticket<WarmDone>> to_cancel;
    {
        std::lock_guard lock(state_->mu);
        if (state_->cancelled) return;
        state_->cancelled = true;
        state_->progress.cancelled += state_->items.size() - state_->next;
        state_->next = state_->items.size();
        for (auto& [idx, t] : state_->tickets) {
            ++state_->progress.cancelled;
            --state_->inflight;
            to_cancel.push_back(std::move(t));
        }
        state_->tickets.clear();
        detail::warm_check_finish_locked(*state_);
    }
    for (auto& t : to_cancel) t.cancel();
}

bool WarmJob::wait_for(std::chrono::milliseconds d) const {
    if (!state_) return true;
    std::unique_lock lock(state_->mu);
    return state_->cv.wait_for(lock, d, [&] { return state_->progress.finished; });
}

// ---------------------------------------------------------------------------------------------
// PhysicsFetch
// ---------------------------------------------------------------------------------------------

PhysicsFetch::PhysicsFetch(TerrainService& service, bool with_road_segments, Priority priority)
    : svc_(service), roads_(with_road_segments), priority_(static_cast<std::uint8_t>(priority)) {}

phys::FetchResult PhysicsFetch::fetch(const TileKey& key) {
    const Priority p = priority();
    Ticket<HeightTile> height = svc_.height(key, p);
    Ticket<RoadSegmentsTile> roads;
    if (roads_) roads = svc_.road_segments(to_level2(key), p);

    const auto wait = [this](auto& t) {
        while (!t.wait_for(std::chrono::milliseconds(50))) {
            if (svc_.stopping()) return false;
        }
        return true;
    };
    phys::FetchResult out;
    if (!wait(height) || (roads_ && !wait(roads))) return out; // Internal: the loader retries or gives up
    switch (height.state()) {
        case TileState::Ready: break;
        case TileState::Absent: out.status = Status::NotFound; return out;
        default: return out; // Failed / Cancelled
    }
    if (roads_) {
        switch (roads.state()) {
            case TileState::Ready: {
                const auto seg = roads.value();
                out.roads = std::shared_ptr<const std::vector<RoadSegment>>(seg, &seg->segments); // aliases the tile
                break;
            }
            case TileState::Absent: // outside coverage: a confirmed "no drivable ways"
                out.roads = std::make_shared<const std::vector<RoadSegment>>();
                break;
            default: return out; // Failed / Cancelled: never silently missing
        }
    }
    out.status = Status::Ok;
    out.tile = height.value();
    return out;
}

// ---------------------------------------------------------------------------------------------
// TerrainService
// ---------------------------------------------------------------------------------------------

TerrainService::TerrainService(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

TerrainService::~TerrainService() {
    if (impl_ && impl_->sched) impl_->sched->shutdown();
}

Result<std::unique_ptr<TerrainService>> TerrainService::open(const TerrainServiceConfig& config) {
    const auto bad = [](ErrorCode code, std::string msg) -> Result<std::unique_ptr<TerrainService>> {
        return make_error(code, "TerrainService::open: " + std::move(msg));
    };
    if (!config.zone.valid()) return bad(ErrorCode::InvalidArgument, "zone " + std::to_string(config.zone.zone) + " is out of range");
    if (config.e0_m % 1000 != 0 || config.n0_m % 1000 != 0)
        return bad(ErrorCode::InvalidArgument, "e0_m/n0_m must be multiples of 1000 m");
    if (config.layers.geometry && !config.layers.roads) return bad(ErrorCode::InvalidArgument, "layers.geometry needs layers.roads");
    if (bool(config.remote_transport) != bool(config.remote_manifest))
        return bad(ErrorCode::InvalidArgument, "remote transport and manifest must be supplied together");
    if (!config.remote_transport && config.stores.source_scope.empty()) return bad(ErrorCode::InvalidArgument, "stores.source_scope is empty");

    if (!config.elevation_source.starts_with(kSrcElevPrefix) ||
        !is_valid_layer_name(config.elevation_source) ||
        config.elevation_source.substr(kSrcElevPrefix.size()).find('.') != std::string::npos)
        return make_error(ErrorCode::InvalidArgument, "invalid elevation source layer");
    auto impl = std::make_unique<Impl>(config);
    Impl& m = *impl;

    const bool carved = config.layers.carved;
    const bool classes = config.layers.classes;
    const bool fields = config.layers.fields;
    const bool soil = config.layers.soil;
    if (config.remote_transport) {
        if (!config.remote_manifest) return bad(ErrorCode::InvalidArgument, "remote transport needs a pinned manifest");
        if (!config.extra_derivers.empty()) return bad(ErrorCode::InvalidArgument, "remote mode cannot use local derivers");
        m.manifest = *config.remote_manifest;
        m.rid_hex = to_hex(m.manifest.rid);
        const auto needs = [&](bool enabled, const char* layer) {
            return !enabled || m.manifest.find_layer(layer) != nullptr;
        };
        if (!needs(true, carved ? "g2m.terrain.carved" : "g2m.terrain.height") ||
            !needs(config.layers.roads, "g2m.roads.graph") || !needs(config.layers.geometry, "g2m.roads.geom") ||
            !needs(classes, "g2m.terrain.class") || !needs(config.layers.rail, "g2m.rail.geom") ||
            !needs(fields, "g2m.fields.geom") || !needs(soil, "g2m.soil.grid"))
            return bad(ErrorCode::InvalidArgument, "remote release lacks required terrain layers");
        m.transport = config.remote_transport;
    } else {
        TileStoreConfig sc;
        sc.dir = config.stores.source_dir;
        sc.name = config.stores.source_name;
        sc.read_only = config.stores.source_read_only;
        auto sr = TileStore::open(sc);
        if (!sr.ok()) return bad(sr.error().code, "source store \"" + sc.dir.string() + "\": " + sr.error().message);
        m.sources = std::move(sr.value());

        TileStoreConfig dc;
        dc.dir = config.stores.derived_dir;
        dc.name = config.stores.derived_name;
        auto dr = TileStore::open(dc);
        if (!dr.ok()) return bad(dr.error().code, "derived store \"" + dc.dir.string() + "\": " + dr.error().message);
        m.derived = std::move(dr.value());

        m.upstream.emplace(*m.sources);

        const bool roads = config.layers.roads || carved || classes;
        const bool geometry = config.layers.geometry || carved || classes;
        LocalReleaseParams params =
            builtin_local_release_params(config.stores.source_scope, m.derivers, config.attribution, roads, geometry, carved, classes, config.layers.rail, fields, soil);
        LayerRegistry source_registry = LayerRegistry::builtin();
        if (config.elevation_source != kSrcElevDgm1Layer) {
            auto registered = source_registry.add(make_raw_elev_layer(config.elevation_source.substr(kSrcElevPrefix.size())));
            if (!registered) return bad(registered.error().code, registered.error().message);
        }
        // A scope can contain several imported elevation products. Register
        // their raw descriptors even when this release selects another one.
        auto source_index = m.sources->index(config.stores.source_scope);
        if (!source_index) return bad(source_index.error().code, source_index.error().message);
        for (const auto& entry : *source_index) {
            if (entry.layer.starts_with(kSrcElevPrefix) && !source_registry.find(entry.layer)) {
                auto registered = source_registry.add(make_raw_elev_layer(entry.layer.substr(kSrcElevPrefix.size())));
                if (!registered) return bad(registered.error().code, registered.error().message);
            }
        }
        params.registry = &source_registry;
        params.world_seed = config.world_seed;
        for (const IDeriver* d : config.extra_derivers) params.derivers.push_back(d);
        auto manifest = make_local_release(*m.sources, params);
        if (!manifest.ok()) return bad(manifest.error().code, "make_local_release: " + manifest.error().message);
        m.manifest = std::move(manifest.value());
        m.rid_hex = to_hex(m.manifest.rid);

        ServerConfig sv;
        sv.server_id = "g2m.terrain_service";
        sv.offline = true; // the only upstream is the local source store
        sv.derived_store = m.derived.get();
        sv.derivers = params.derivers;
        sv.upstreams = {&*m.upstream};
        m.server = std::make_unique<Server>(sv);
        const Result<void> added = m.server->add_release(m.manifest, config.stores.source_scope);
        if (!added.ok()) return bad(added.error().code, "add_release: " + added.error().message);
        m.transport = std::make_shared<InProcessTransport>(*m.server);

    }

    m.frame = std::make_unique<geo::SessionFrame>(config.zone, config.e0_m, config.n0_m);
    m.height_layer = std::string(carved ? kTerrainCarvedLayer : kTerrainHeightLayer);
    m.workers = config.derive_workers > 0
                    ? config.derive_workers
                    : std::max(1, static_cast<int>(std::thread::hardware_concurrency()) - 2);
    m.sched = JobScheduler::create(m.workers);
    return std::unique_ptr<TerrainService>(new TerrainService(std::move(impl)));
}

const ReleaseManifest& TerrainService::manifest() const { return impl_->manifest; }
const ReleaseId& TerrainService::release_id() const { return impl_->manifest.rid; }
std::string TerrainService::release_id_hex() const { return impl_->rid_hex; }
const geo::SessionFrame& TerrainService::frame() const { return *impl_->frame; }
bool TerrainService::has_layer(std::string_view layer) const { return impl_->manifest.find_layer(layer) != nullptr; }
std::string_view TerrainService::height_layer() const { return impl_->height_layer; }
int TerrainService::worker_count() const { return impl_->workers; }
bool TerrainService::stopping() const noexcept { return impl_->sched->stopped(); }

std::vector<TileKey> TerrainService::covered_tiles(std::string_view layer, int level) const {
    std::vector<TileKey> out;
    const ReleaseLayer* rl = impl_->manifest.find_layer(layer);
    if (rl == nullptr || !has_level(rl->desc.levels, level) || rl->coverage.global) return out;
    for (const ZoneCoverage& zc : rl->coverage.zones) {
        for (const CoverageRun& run : zc.runs) {
            for (std::int32_t x = run.x0; x <= run.x1; ++x) {
                const TileKey l2{zc.zone, kRawTileLevel, x, run.y};
                if (level == kRawTileLevel) {
                    out.push_back(l2);
                } else if (level > kRawTileLevel) {
                    TileKey k = l2;
                    while (k.level < level) k = k.parent();
                    out.push_back(k);
                } else {
                    std::vector<TileKey> cur{l2};
                    for (int lv = kRawTileLevel; lv > level; --lv) {
                        std::vector<TileKey> nxt;
                        nxt.reserve(cur.size() * 4);
                        for (const TileKey& k : cur)
                            for (int dy = 0; dy < 2; ++dy)
                                for (int dx = 0; dx < 2; ++dx) nxt.push_back(k.child(dx, dy));
                        cur = std::move(nxt);
                    }
                    out.insert(out.end(), cur.begin(), cur.end());
                }
            }
        }
    }
    std::sort(out.begin(), out.end());
    out.erase(std::unique(out.begin(), out.end()), out.end());
    return out;
}

ServiceStats TerrainService::stats() const {
    const detail::SchedulerStats s = impl_->sched->stats();
    const detail::TileCache::Stats c = impl_->cache.stats();
    ServiceStats o;
    o.workers = impl_->workers;
    o.jobs_run = s.jobs_run;
    o.inline_runs = s.inline_runs;
    o.joined = s.joined;
    o.boosted = s.boosted;
    o.cancelled_before_start = s.cancelled_before_start;
    o.cache_hits = c.hits;
    o.cache_misses = c.misses;
    o.cache_entries = c.entries;
    o.cache_bytes = c.bytes;
    o.peak_running = s.peak_running;
    return o;
}

// ---- async requests ------------------------------------------------------------------------

Ticket<HeightTile> TerrainService::height(const TileKey& key, Priority priority) {
    return impl_->ticket<HeightTile>(impl_->job_height(key), priority);
}
Ticket<TerrainClassTile> TerrainService::terrain_class(const TileKey& key, Priority priority) {
    if (!has_layer(kTerrainClassLayer))
        return impl_->finished_ticket<TerrainClassTile>({kKindTerrainClass, key.packed(), 0},
                                                        failed("g2m.terrain.class is not in the release (LayerOptIn::classes)"));
    return impl_->ticket<TerrainClassTile>(impl_->job_terrain_class(key), priority);
}
Ticket<RailGeomTile> TerrainService::rail_geom(const TileKey& l2, Priority priority) {
    if (!has_layer(kRailGeomLayer))
        return impl_->finished_ticket<RailGeomTile>({kKindRailGeom, to_level2(l2).packed(), 0}, failed("g2m.rail.geom is not in the release (LayerOptIn::rail)"));
    return impl_->ticket<RailGeomTile>(impl_->job_rail_geom(to_level2(l2)), priority);
}
Ticket<RoadGeomTile> TerrainService::road_geom(const TileKey& l2, Priority priority) {
    return impl_->ticket<RoadGeomTile>(impl_->job_road_geom(to_level2(l2)), priority);
}
Ticket<RoadSegmentsTile> TerrainService::road_segments(const TileKey& l2, Priority priority) {
    return impl_->ticket<RoadSegmentsTile>(impl_->job_road_segments(to_level2(l2)), priority);
}
Ticket<FieldTile> TerrainService::fields(const TileKey& l2, Priority priority) {
    if (!has_layer(kFieldsGeomLayer))
        return impl_->finished_ticket<FieldTile>({kKindFields, to_level2(l2).packed(), 0},
                                                 failed("g2m.fields.geom is not in the release (LayerOptIn::fields)"));
    return impl_->ticket<FieldTile>(impl_->job_fields(to_level2(l2)), priority);
}
Ticket<SoilGridTile> TerrainService::soil(const TileKey& l2, Priority priority) {
    if (!has_layer(kSoilGridLayer))
        return impl_->finished_ticket<SoilGridTile>({kKindSoil, to_level2(l2).packed(), 0},
                                                    failed("g2m.soil.grid is not in the release (LayerOptIn::soil)"));
    return impl_->ticket<SoilGridTile>(impl_->job_soil(to_level2(l2)), priority);
}
Ticket<OsmTile> TerrainService::source_osm(const TileKey& l2, Priority priority) {
    return impl_->ticket<OsmTile>(impl_->job_source_osm(to_level2(l2)), priority);
}
Ticket<RawTile> TerrainService::raw_tile(const std::string& layer, const TileKey& key, Priority priority) {
    return impl_->ticket<RawTile>(impl_->job_raw(layer, key), priority);
}

Ticket<StructureSet> TerrainService::structures(const TileKey& l2, Priority priority) {
    Producer<StructureSet> p;
    {
        std::lock_guard lock(impl_->producer_mu);
        p = impl_->structures_producer;
    }
    const TileKey k = to_level2(l2);
    if (!p) return impl_->finished_ticket<StructureSet>({kKindStructures, k.packed(), 0}, failed("no structures producer installed"));
    return impl_->ticket<StructureSet>(impl_->job_producer<StructureSet>(kKindStructures, k, 0, std::move(p), *this), priority);
}
Ticket<RoadRibbonTile> TerrainService::road_ribbons(const TileKey& l2, Priority priority, const RibbonOptions& options) {
    Producer<RoadRibbonTile> p;
    {
        std::lock_guard lock(impl_->producer_mu);
        p = impl_->ribbons_producer;
    }
    const TileKey k = to_level2(l2);
    if (!p) return impl_->finished_ticket<RoadRibbonTile>({kKindRibbons, k.packed(), options.hash}, failed("no road_ribbons producer installed"));
    return impl_->ticket<RoadRibbonTile>(impl_->job_producer<RoadRibbonTile>(kKindRibbons, k, options.hash, std::move(p), *this), priority);
}
Ticket<BuildingSet> TerrainService::buildings(const TileKey& l2, Priority priority, const BuildingOptions& options) {
    Producer<BuildingSet> p;
    {
        std::lock_guard lock(impl_->producer_mu);
        p = impl_->buildings_producer;
    }
    const TileKey k = to_level2(l2);
    if (!p) return impl_->finished_ticket<BuildingSet>({kKindBuildings, k.packed(), options.hash}, failed("no buildings producer installed"));
    return impl_->ticket<BuildingSet>(impl_->job_producer<BuildingSet>(kKindBuildings, k, options.hash, std::move(p), *this), priority);
}
void TerrainService::set_structures_producer(Producer<StructureSet> producer) {
    std::lock_guard lock(impl_->producer_mu);
    impl_->structures_producer = std::move(producer);
}
void TerrainService::set_road_ribbons_producer(Producer<RoadRibbonTile> producer) {
    std::lock_guard lock(impl_->producer_mu);
    impl_->ribbons_producer = std::move(producer);
}
void TerrainService::set_buildings_producer(Producer<BuildingSet> producer) {
    std::lock_guard lock(impl_->producer_mu);
    impl_->buildings_producer = std::move(producer);
}

// ---- peeks -----------------------------------------------------------------------------------

std::shared_ptr<const HeightTile> TerrainService::peek_height(const TileKey& key) const noexcept {
    return std::static_pointer_cast<const HeightTile>(
        impl_->cache.peek({kKindHeight, key.packed(), 0}, type_id(typeid(HeightTile))));
}
std::shared_ptr<const TerrainClassTile> TerrainService::peek_terrain_class(const TileKey& key) const noexcept {
    return std::static_pointer_cast<const TerrainClassTile>(
        impl_->cache.peek({kKindTerrainClass, key.packed(), 0}, type_id(typeid(TerrainClassTile))));
}
std::shared_ptr<const RailGeomTile> TerrainService::peek_rail_geom(const TileKey& l2) const noexcept {
    return std::static_pointer_cast<const RailGeomTile>(
        impl_->cache.peek({kKindRailGeom, to_level2(l2).packed(), 0}, type_id(typeid(RailGeomTile))));
}
std::shared_ptr<const RoadGeomTile> TerrainService::peek_road_geom(const TileKey& l2) const noexcept {
    return std::static_pointer_cast<const RoadGeomTile>(
        impl_->cache.peek({kKindRoadGeom, to_level2(l2).packed(), 0}, type_id(typeid(RoadGeomTile))));
}
std::shared_ptr<const RoadSegmentsTile> TerrainService::peek_road_segments(const TileKey& l2) const noexcept {
    return std::static_pointer_cast<const RoadSegmentsTile>(
        impl_->cache.peek({kKindRoadSegments, to_level2(l2).packed(), 0}, type_id(typeid(RoadSegmentsTile))));
}
std::shared_ptr<const FieldTile> TerrainService::peek_fields(const TileKey& l2) const noexcept {
    return std::static_pointer_cast<const FieldTile>(
        impl_->cache.peek({kKindFields, to_level2(l2).packed(), 0}, type_id(typeid(FieldTile))));
}
std::optional<TerrainService::FieldHit> TerrainService::field_at(double x, double y) const {
    return field_at_grid(x + static_cast<double>(frame().e0_m()), y + static_cast<double>(frame().n0_m()));
}
std::optional<TerrainService::FieldHit> TerrainService::field_at_grid(double easting, double northing) const {
    const auto key = tile_key_at(frame().zone(), 2, easting, northing);
    if (!key) return std::nullopt;
    auto tile = peek_fields(*key);
    if (!tile) return std::nullopt;
    const auto index = g2m::field_at(*tile, geom::PointMm{static_cast<std::int64_t>(std::floor(easting * 1000.0)),
                                                         static_cast<std::int64_t>(std::floor(northing * 1000.0))});
    if (!index) return std::nullopt;
    return FieldHit{std::move(tile), *index};
}
std::shared_ptr<const SoilGridTile> TerrainService::peek_soil(const TileKey& l2) const noexcept {
    return std::static_pointer_cast<const SoilGridTile>(
        impl_->cache.peek({kKindSoil, to_level2(l2).packed(), 0}, type_id(typeid(SoilGridTile))));
}
std::optional<SoilSample> TerrainService::soil_at(double x, double y) const {
    return soil_at_grid(x + static_cast<double>(frame().e0_m()), y + static_cast<double>(frame().n0_m()));
}
std::optional<SoilSample> TerrainService::soil_at_grid(double easting, double northing) const {
    const auto key = tile_key_at(frame().zone(), 2, easting, northing);
    if (!key) return std::nullopt;
    const auto tile = peek_soil(*key);
    if (!tile) return std::nullopt;
    return g2m::soil_at(*tile, static_cast<std::int64_t>(std::floor(easting * 1000.0)), static_cast<std::int64_t>(std::floor(northing * 1000.0)));
}
SoilFieldSummary TerrainService::field_soil(const FieldRecord& field) const {
    const SoilLookup lookup = [this](std::int64_t e_mm, std::int64_t n_mm) -> std::optional<SoilSample> {
        const auto key = tile_key_at(frame().zone(), 2, static_cast<double>(e_mm) / 1000.0, static_cast<double>(n_mm) / 1000.0);
        if (!key) return std::nullopt;
        const auto tile = peek_soil(*key);
        if (!tile) return std::nullopt;
        return g2m::soil_at(*tile, e_mm, n_mm);
    };
    return summarise_field_soil(field, builtin_soil_rules(), lookup);
}
std::shared_ptr<const OsmTile> TerrainService::peek_source_osm(const TileKey& l2) const noexcept {
    return std::static_pointer_cast<const OsmTile>(
        impl_->cache.peek({kKindSourceOsm, to_level2(l2).packed(), 0}, type_id(typeid(OsmTile))));
}
std::shared_ptr<const StructureSet> TerrainService::peek_structures(const TileKey& l2) const noexcept {
    return std::static_pointer_cast<const StructureSet>(
        impl_->cache.peek({kKindStructures, to_level2(l2).packed(), 0}, type_id(typeid(StructureSet))));
}

// ---- point heights ---------------------------------------------------------------------------

std::optional<double> TerrainService::surface_height_exact(double x, double y) const {
    std::vector<std::shared_ptr<const HeightTile>> keep; // the sampler gets raw pointers: hold the tiles meanwhile
    const L0TileLookupFn lookup = [&](const TileKey& k) -> const HeightTile* {
        auto p = peek_height(k);
        if (!p) return nullptr;
        keep.push_back(p);
        return keep.back().get();
    };
    const geo::SessionFrame& f = *impl_->frame;
    return sample_height_exact(lookup, f.zone(), f.e0_m(), f.n0_m(), x, y);
}
std::optional<double> TerrainService::surface_height_bilinear(double x, double y) const {
    std::vector<std::shared_ptr<const HeightTile>> keep;
    const L0TileLookupFn lookup = [&](const TileKey& k) -> const HeightTile* {
        auto p = peek_height(k);
        if (!p) return nullptr;
        keep.push_back(p);
        return keep.back().get();
    };
    const geo::SessionFrame& f = *impl_->frame;
    return sample_height_bilinear(lookup, f.zone(), f.e0_m(), f.n0_m(), x, y);
}

// ---- caller-defined products -----------------------------------------------------------------

ProductKind TerrainService::register_product_kind(std::string_view name) {
    static std::mutex mu;
    static std::map<std::string, ProductKind, std::less<>> kinds;
    std::lock_guard lock(mu);
    const auto it = kinds.find(name);
    if (it != kinds.end()) return it->second;
    const ProductKind k = kFirstUserKind + static_cast<ProductKind>(kinds.size());
    kinds.emplace(std::string(name), k);
    return k;
}

TerrainService::Submitted TerrainService::request_erased(const detail::JobKey& key, const std::type_info& type,
                                                         Priority priority, std::function<ErasedOutcome()> fn) {
    Impl::Job job{key, &type, std::move(fn)};
    auto s = impl_->request(job, priority);
    return Submitted{std::move(s.job), std::move(s.interest)};
}

ErasedOutcome TerrainService::join_erased(const detail::JobKey& key, const std::type_info& type,
                                          std::function<ErasedOutcome()> fn) {
    const void* t = type_id(type);
    return impl_->sched->join_or_run(key, t, impl_->wrap(key, t, std::move(fn), true), impl_->probe(key, t));
}

std::shared_ptr<const void> TerrainService::peek_erased(const detail::JobKey& key, const std::type_info& type) const noexcept {
    return impl_->cache.peek(key, type_id(type));
}

// ---- warm / physics -------------------------------------------------------------------------

WarmJob TerrainService::warm(const WarmRequest& request) {
    auto st = std::make_shared<detail::WarmState>();
    st->items = request.items;
    st->priority = request.priority;
    st->window = request.window > 0 ? request.window : std::max(1, 2 * impl_->workers);
    st->progress.total = st->items.size();
    Impl* impl = impl_.get();
    st->submit = [impl](const WarmItem& item, Priority p) {
        auto s = impl->request(impl->job_warm(item), p, /*cache_result=*/false);
        return Submitted{std::move(s.job), std::move(s.interest)};
    };
    st->hook = [impl](const std::shared_ptr<JobCore>& job, std::function<void()> fn) {
        impl->sched->add_hook(job, std::move(fn));
    };
    {
        std::lock_guard lock(st->mu);
        detail::warm_check_finish_locked(*st);
    }
    detail::warm_pump(st);
    return WarmJob(std::move(st));
}

std::shared_ptr<PhysicsFetch> TerrainService::physics_fetch(bool with_road_segments, Priority priority) {
    return std::make_shared<PhysicsFetch>(*this, with_road_segments, priority);
}

} // namespace g2m::world
