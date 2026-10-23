// g2m/world/terrain_service.h - g2m::world::TerrainService (terrain migration plan
// phase 2, vault G2M-020): the one object a game or tool opens to get terrain.
//
// It owns the stores, the layer opt-ins (hence the release id), an in-process
// server and ONE worker pool. Every request is asynchronous and returns a
// Ticket<T> (ticket.h); callers only poll tickets, so no render or simulation
// thread ever blocks on a derive.
//
// Threading and scheduling:
//   - Any thread may call any request or peek method.
//   - Jobs are ordered by Priority (Gate, Physics, Prefetch, Render, Warm), then by
//     request order. A request for a tile that is already queued at a lower priority
//     raises it. Different tiles derive in parallel on the pool.
//   - Each (product, tile, options) is at most ONE job at a time: concurrent requests
//     join the running (or queued) job and every holder gets the same value. The
//     server's resolver deduplicates the dependency tiles inside a derivation the same
//     way (server.h), so two jobs that need the same neighbour tile derive it once.
//   - A job that needs another product (a producer calling ProductContext::height()
//     and so on) joins it if it is running, or runs it itself if it is only queued; it
//     never waits for queued work, so a pool of any size cannot deadlock.
//   - peek_*() read the in-memory cache and never start work.
//   - Derived BYTES are those of the plain server: the pool changes only when tiles
//     are derived, never what they contain (release id unchanged).
//
// No physics_sim, Jolt or Godot type appears here.
#pragma once

#include "g2m/core/geo/session_frame.h"
#include "g2m/core/geo/utm.h"
#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/deriver.h"
#include "g2m/layer/release.h"
#include "g2m/phys/height_tile_loader.h"
#include "g2m/world/products.h"
#include "g2m/world/ticket.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <typeinfo>
#include <vector>

namespace g2m {
class ITransport;
struct TerrainClassTile; // g2m/layer/terrain_class.h (include it to use terrain_class())
}

namespace g2m::world {

struct StoreConfig {
    // Source store (raw layers, read-only by default: the service never writes into it).
    std::filesystem::path source_dir;
    std::string source_name = "tiles";
    std::string source_scope; // names the raw tiles in the source store
    bool source_read_only = true;
    // Derived store (read-write: derived tiles land here and survive restarts).
    std::filesystem::path derived_dir;
    std::string derived_name = "tiles";
};

// Which built-in layers the release contains. The opt-ins are part of the release id.
struct LayerOptIn {
    bool roads = true;      // g2m.roads.graph
    bool geometry = true;   // g2m.roads.geom (needs roads)
    bool carved = false;    // g2m.terrain.carved; when set, height() serves it instead of g2m.terrain.height
    bool classes = false;   // g2m.terrain.class (phase 6); implies the carved layer in the release (height() still follows `carved`)
    bool rail = false;      // g2m.rail.geom (railways R1/R2); independent of the road layers, needs only src.osm and elev.base
};

struct TerrainServiceConfig {
    // Optional remote mode: pinned manifest and a shared, thread-safe transport.
    // Skips all local stores/derivers. The existing scheduler and decoded cache remain.
    std::shared_ptr<ITransport> remote_transport;
    std::optional<ReleaseManifest> remote_manifest;
    StoreConfig stores;
    // Explicit imported source; defaults preserve established DGM1 releases.
    // Caller imports/reprojects this layer before opening the service.
    std::string elevation_source = "g2m.src.elev.dgm1";
    LayerOptIn layers;
    // The session frame (local metres): zone and the origin in whole kilometres.
    geo::UtmZone zone{32, geo::Hemisphere::North};
    std::int64_t e0_m = 0, n0_m = 0;
    std::vector<ReleaseSource> attribution; // manifest records only; not part of the release id
    std::uint64_t world_seed = 0;
    int derive_workers = 0;                 // 0 = max(1, hardware_concurrency - 2)
    std::size_t cache_bytes = std::size_t{1} << 30; // decoded-tile memory cache (LRU)
    // Additional derivers (tests, tools), appended after the built-ins in this order; they join the
    // release (and so its id) and the server. Must outlive the service.
    std::vector<const IDeriver*> extra_derivers;
};

class TerrainService;

// What a producer (a function that builds a product from other products) sees. Valid only
// inside the producer call, which always runs on a pool thread: the calls below may block
// (join a running job or run a queued one inline), so they are for producers, never for a
// render or simulation thread. Each returns the product's Outcome.
class ProductContext {
public:
    Outcome<HeightTile> height(const TileKey& key);
    Outcome<RoadGeomTile> road_geom(const TileKey& l2);
    Outcome<RailGeomTile> rail_geom(const TileKey& l2);
    Outcome<RoadSegmentsTile> road_segments(const TileKey& l2);
    Outcome<OsmTile> source_osm(const TileKey& l2);
    Outcome<RawTile> raw_tile(const std::string& layer, const TileKey& key);
    const geo::SessionFrame& frame() const;
    TerrainService& service() { return svc_; }

private:
    friend class TerrainService;
    explicit ProductContext(TerrainService& s) : svc_(s) {}
    TerrainService& svc_;
};

using ProductKind = std::uint32_t;
template <class T>
using Producer = std::function<Outcome<T>(ProductContext&, const TileKey&)>;

// ---- warm-up ----------------------------------------------------------------------------
struct WarmItem {
    std::string layer;
    TileKey key;
};
struct WarmRequest {
    std::vector<WarmItem> items;
    Priority priority = Priority::Warm;
    int window = 0; // tiles in flight at once; 0 = twice the worker count

    static WarmRequest of(std::string_view layer, const std::vector<TileKey>& keys) {
        WarmRequest r;
        for (const auto& k : keys) r.items.push_back({std::string(layer), k});
        return r;
    }
};
struct WarmProgress {
    std::size_t total = 0;
    std::size_t ready = 0;     // derived (or already stored) and served
    std::size_t absent = 0;    // the release has no such tile
    std::size_t failed = 0;
    std::size_t cancelled = 0; // never started because of cancel() or shutdown
    bool finished = false;
};

namespace detail {
struct WarmState;
}

// Handle of a warm() run. Copies share the run. Dropping every copy does NOT cancel it;
// cancel() does.
class WarmJob {
public:
    WarmJob() = default;
    WarmProgress progress() const;
    bool finished() const { return progress().finished; }
    void cancel();
    // Tools and tests only. Returns true if the run finished within `d`.
    bool wait_for(std::chrono::milliseconds d) const;

private:
    friend class TerrainService;
    explicit WarmJob(std::shared_ptr<detail::WarmState> s) : state_(std::move(s)) {}
    std::shared_ptr<detail::WarmState> state_;
};

struct ServiceStats {
    int workers = 0;
    std::uint64_t jobs_run = 0;        // jobs that ran (pool + inline)
    std::uint64_t inline_runs = 0;     // of those: run by a producer that needed the result
    std::uint64_t joined = 0;          // requests that attached to an existing job instead of starting one
    std::uint64_t boosted = 0;         // joins that raised a queued job's priority
    std::uint64_t cancelled_before_start = 0;
    std::uint64_t cache_hits = 0;
    std::uint64_t cache_misses = 0;
    std::size_t cache_entries = 0;
    std::size_t cache_bytes = 0;
    int peak_running = 0;              // most jobs running at once so far
};

// IHeightTileFetch over a service (replaces racing_game's HeightTileSharedFetch): fetch() asks for the
// L0 height tile (and, with roads, the owning L2 tile's road segments) at the CURRENT priority and
// BLOCKS until they are ready. HeightTileLoader calls it on its own worker threads, which is the only
// place it may be called. The service must outlive it.
class PhysicsFetch final : public phys::IHeightTileFetch {
public:
    PhysicsFetch(TerrainService& service, bool with_road_segments, Priority priority);
    phys::FetchResult fetch(const TileKey& key) override;
    bool provides_roads() const override { return roads_; }
    // Any thread; takes effect on the next fetch (e.g. Gate until the start-up gate opens, then Physics).
    void set_priority(Priority p) noexcept { priority_.store(static_cast<std::uint8_t>(p), std::memory_order_relaxed); }
    Priority priority() const noexcept { return static_cast<Priority>(priority_.load(std::memory_order_relaxed)); }

private:
    TerrainService& svc_;
    bool roads_;
    std::atomic<std::uint8_t> priority_;
};

class TerrainService {
public:
    // Opens the stores, builds the release and starts the pool. Errors (missing store, bad frame,
    // an opt-in that is not available) come back as a Result, nothing is thrown.
    static Result<std::unique_ptr<TerrainService>> open(const TerrainServiceConfig& config);
    // Cancels queued jobs, lets running ones finish, joins the pool. Tickets stay valid.
    ~TerrainService();
    TerrainService(const TerrainService&) = delete;
    TerrainService& operator=(const TerrainService&) = delete;

    const ReleaseManifest& manifest() const;
    const ReleaseId& release_id() const;
    std::string release_id_hex() const;
    const geo::SessionFrame& frame() const;
    bool has_layer(std::string_view layer) const;
    // The layer height() serves: g2m.terrain.carved when opted in, else g2m.terrain.height.
    std::string_view height_layer() const;
    // Every tile of `layer`'s coverage at `level` (sorted): level-2 coverage expanded to finer levels or
    // folded to coarser ones. Empty for an unknown layer or a level the layer does not have.
    std::vector<TileKey> covered_tiles(std::string_view layer, int level) const;
    ServiceStats stats() const;
    int worker_count() const;
    bool stopping() const noexcept;

    // ---- async requests (any thread, never derive on the calling thread) ----------------
    Ticket<HeightTile> height(const TileKey& key, Priority priority);
    // The decoded g2m.terrain.class tile (any level; D62, the render ClassWindow's source). Failed at once when the
    // release has no class layer (LayerOptIn::classes off).
    Ticket<TerrainClassTile> terrain_class(const TileKey& key, Priority priority);
    // road_geom / road_segments / source_osm: `key` is the level-2 tile (a finer key is mapped to its level-2
    // ancestor, so L0 and L2 requests for the same area share one job).
    Ticket<RoadGeomTile> road_geom(const TileKey& l2, Priority priority);
    // The decoded g2m.rail.geom tile (level 2; railway pieces owned by the tile). Failed at once when the release has no
    // rail layer (LayerOptIn::rail off).
    Ticket<RailGeomTile> rail_geom(const TileKey& l2, Priority priority);
    Ticket<RoadSegmentsTile> road_segments(const TileKey& l2, Priority priority);
    Ticket<OsmTile> source_osm(const TileKey& l2, Priority priority);
    // Any layer's container, uncached typed decode skipped (for layers without a typed ticket, and tools).
    Ticket<RawTile> raw_tile(const std::string& layer, const TileKey& key, Priority priority);
    // Phase-3 products. Failed until the matching producer is installed.
    Ticket<StructureSet> structures(const TileKey& l2, Priority priority);
    Ticket<RoadRibbonTile> road_ribbons(const TileKey& l2, Priority priority, const RibbonOptions& options);
    Ticket<BuildingSet> buildings(const TileKey& l2, Priority priority, const BuildingOptions& options);
    void set_structures_producer(Producer<StructureSet> producer);
    void set_road_ribbons_producer(Producer<RoadRibbonTile> producer);
    void set_buildings_producer(Producer<BuildingSet> producer);

    // ---- non-blocking peeks: the cached value or null, never start work ----------------
    std::shared_ptr<const HeightTile> peek_height(const TileKey& key) const noexcept;
    std::shared_ptr<const TerrainClassTile> peek_terrain_class(const TileKey& key) const noexcept;
    std::shared_ptr<const RoadGeomTile> peek_road_geom(const TileKey& l2) const noexcept;
    std::shared_ptr<const RailGeomTile> peek_rail_geom(const TileKey& l2) const noexcept;
    std::shared_ptr<const RoadSegmentsTile> peek_road_segments(const TileKey& l2) const noexcept;
    std::shared_ptr<const OsmTile> peek_source_osm(const TileKey& l2) const noexcept;
    std::shared_ptr<const StructureSet> peek_structures(const TileKey& l2) const noexcept;

    // ---- point heights on resident level-0 tiles (session-local metres; nullopt when a needed tile is not
    // in the memory cache or has no data there; never starts work) ----------------------------------
    std::optional<double> surface_height_exact(double x, double y) const;   // Jolt triangulation
    std::optional<double> surface_height_bilinear(double x, double y) const; // render only

    // ---- caller-defined products (the game's current producers run through the pool this way) ----
    // A name maps to one ProductKind for the life of the process (idempotent).
    static ProductKind register_product_kind(std::string_view name);
    // `options_hash` distinguishes products of the same kind and tile (part of the job and cache key).
    template <class T>
    Ticket<T> product(ProductKind kind, const TileKey& key, Priority priority, std::uint64_t options_hash,
                      Producer<T> producer);
    template <class T>
    std::shared_ptr<const T> peek_product(ProductKind kind, const TileKey& key, std::uint64_t options_hash = 0) const noexcept;

    // ---- warm-up after a release-id change -----------------------------------------------
    // Pre-derives every listed tile through the server (derived store filled, the memory cache untouched),
    // `window` tiles at a time at request.priority. Progress is polled on the returned handle.
    WarmJob warm(const WarmRequest& request);

    // ---- physics ----------------------------------------------------------------------
    std::shared_ptr<PhysicsFetch> physics_fetch(bool with_road_segments, Priority priority = Priority::Physics);

    // ---- internal: type-erased entry points the templates above and ProductContext use ----
    struct Submitted {
        std::shared_ptr<detail::JobCore> job;
        std::shared_ptr<detail::Interest> interest;
    };
    Submitted request_erased(const detail::JobKey& key, const std::type_info& type, Priority priority,
                             std::function<detail::ErasedOutcome()> fn);
    detail::ErasedOutcome join_erased(const detail::JobKey& key, const std::type_info& type,
                                      std::function<detail::ErasedOutcome()> fn);
    std::shared_ptr<const void> peek_erased(const detail::JobKey& key, const std::type_info& type) const noexcept;

    struct Impl; // defined in terrain_service.cpp

private:
    friend class ProductContext;
    explicit TerrainService(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

// ---- templates -----------------------------------------------------------------------------

namespace detail {
template <class T>
ErasedOutcome erase(Outcome<T> o) {
    ErasedOutcome e;
    e.state = o.state == TileState::Ready || o.state == TileState::Absent ? o.state : TileState::Failed;
    e.value = std::move(o.value);
    e.message = std::move(o.message);
    e.bytes = o.bytes;
    return e;
}
} // namespace detail

template <class T>
Ticket<T> TerrainService::product(ProductKind kind, const TileKey& key, Priority priority, std::uint64_t options_hash,
                                  Producer<T> producer) {
    const detail::JobKey jk{kind, key.packed(), options_hash};
    auto fn = [this, producer = std::move(producer), key]() -> detail::ErasedOutcome {
        ProductContext ctx(*this);
        return detail::erase<T>(producer(ctx, key));
    };
    auto s = request_erased(jk, typeid(T), priority, std::move(fn));
    return Ticket<T>(std::move(s.job), std::move(s.interest));
}

template <class T>
std::shared_ptr<const T> TerrainService::peek_product(ProductKind kind, const TileKey& key,
                                                      std::uint64_t options_hash) const noexcept {
    return std::static_pointer_cast<const T>(peek_erased({kind, key.packed(), options_hash}, typeid(T)));
}

} // namespace g2m::world
