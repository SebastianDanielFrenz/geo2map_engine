// g2m/world/tile_fetch.h - engine-neutral tile decode and fetch-cache helpers
// (terrain migration plan phase 1, vault G2M-020; ported from racing_game's
// WorldTerrain: decode_height_tile_container, fetch_height_tile_cached,
// decode_road_segments_response, fetch_road_segments_cached). No physics_sim,
// Godot or game type.
//
// Everything here is pure decode or a small cache seam: no thread, no store,
// no derive. The caller owns the cache map and mutex, supplies the fetch
// function (a transport call, a test lambda) and decides what to log.
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/layer/release.h"
#include "g2m/server/protocol.h"
#include "g2m/server/transport.h"

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <span>
#include <string_view>
#include <vector>

namespace g2m::world {

// Result of a height-tile fetch or decode. `tile` is non-null iff
// `status == Status::Ok`. Heap only: a HeightTile is 256 KiB (vault TOOL-039).
struct HeightTileFetchResult {
    Status status = Status::Internal;
    std::shared_ptr<const HeightTile> tile;
};

// Decodes one height-tile container (HEADER || BODY, as a TileResponse
// carries it): parse_container (body size + SHA-256), the header's layer and
// key must equal `expect_layer`/`expect_key`, decode_body, decode_height_tile
// into a HEAP tile, then the header's height_offset (1/256 m) is added to
// every sample that is not kHeightNoData, in int64; a result <= kHeightNoData
// or above INT32_MAX fails the whole tile. NoData samples and has_nodata are
// left as decoded. Any failure returns Status::Internal with a null tile.
// Same arithmetic as g2m::phys::TransportHeightTileFetch.
HeightTileFetchResult decode_height_tile_container(std::span<const std::uint8_t> container,
                                                   std::string_view expect_layer, const TileKey& expect_key);

// One uncached fetch + decode through a transport: TileRequest{rid, layer,
// key} -> a non-Ok response status passes straight through (null tile), a
// non-tile response is Internal, otherwise decode_height_tile_container.
HeightTileFetchResult fetch_height_tile(ITransport& transport, const ReleaseId& rid, std::string_view layer,
                                        const TileKey& key);

// The uncached fetch function fetch_height_tile_cached calls OUTSIDE any lock.
using HeightTileFetchFn = std::function<HeightTileFetchResult(const TileKey&)>;

// Double-checked-insert cache. `cache_mutex` guards ONLY the `cache` map (the
// find and the eventual emplace), never `fetch_fn`; two threads racing on the
// same missing key both call `fetch_fn` (redundant but safe: the underlying
// work is idempotent), the first emplace wins, and every caller returns the
// SAME shared_ptr read back from the map. A failed fetch (status != Ok) is
// never inserted, so a transient failure is retried by the next call.
// `was_cache_hit`, if non-null, is set to whether the initial lookup found the
// key (a fetch that then fails is not a hit).
HeightTileFetchResult fetch_height_tile_cached(std::mutex& cache_mutex,
                                               std::map<TileKey, std::shared_ptr<const HeightTile>>& cache,
                                               const TileKey& key, const HeightTileFetchFn& fetch_fn,
                                               bool* was_cache_hit = nullptr);

// Tri-state result of a g2m.src.osm tile fetch: a tile outside the release's
// coverage is not the same thing as a failure, nor as a covered tile that
// has no drivable ways.
enum class RoadFetchStatus : std::uint8_t {
    Ok,     // decoded, possibly empty; cached
    Absent, // server NotFound "outside coverage" (a definitive answer); cached, `segments` is an empty list
    Failed, // 5xx/transport/parse/decode/header mismatch; NOT cached, `segments` is null
};

struct RoadSegmentsResult {
    RoadFetchStatus status = RoadFetchStatus::Failed;
    std::shared_ptr<const std::vector<RoadSegment>> segments; // non-null iff status != Failed
};

// Decodes one already-received g2m.src.osm Response ("pure decode, no I/O, no
// cache"): a non-TileResponse variant, any status other than Ok or the exact
// NotFound/"outside coverage" pair, and every parse/header-mismatch/decode
// failure collapse to {Failed, nullptr}. NotFound with meta.message ==
// "outside coverage" is {Absent, empty list}. On success it runs
// extract_road_segments(osm, zone, RoadStyle::default_style()).
RoadSegmentsResult decode_road_segments_response(const Response& response, const TileKey& expect_key,
                                                 geo::UtmZone zone);

// One uncached fetch + decode of a g2m.src.osm tile through a transport.
RoadSegmentsResult fetch_road_segments(ITransport& transport, const ReleaseId& rid, const TileKey& key,
                                       geo::UtmZone zone);

using RoadSegmentsFetchFn = std::function<RoadSegmentsResult(const TileKey&)>;

// Same locking scheme and first-emplace-wins policy as fetch_height_tile_cached.
// Failed is not inserted (retried on the next call); Ok and Absent are.
RoadSegmentsResult fetch_road_segments_cached(std::mutex& cache_mutex, std::map<TileKey, RoadSegmentsResult>& cache,
                                              const TileKey& key, const RoadSegmentsFetchFn& fetch_fn,
                                              bool* was_cache_hit = nullptr);

} // namespace g2m::world
