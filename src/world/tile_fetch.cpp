// g2m/world/tile_fetch.cpp - see include/g2m/world/tile_fetch.h.
//
// decode_height_tile_container, fetch_height_tile_cached,
// decode_road_segments_response and fetch_road_segments_cached are ported
// from racing_game core/src/world_terrain.cpp (a8a1b13) statement for
// statement; tests/world/test_tile_fetch.cpp pins them against values the
// game's code produced.
#include "g2m/world/tile_fetch.h"

#include "g2m/layer/layer_id.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/tile_container.h"

#include <limits>
#include <optional>
#include <utility>
#include <variant>

namespace g2m::world {

namespace {

const std::shared_ptr<const std::vector<RoadSegment>>& empty_road_segments() {
    static const auto kEmpty = std::make_shared<const std::vector<RoadSegment>>();
    return kEmpty;
}

} // namespace

HeightTileFetchResult decode_height_tile_container(std::span<const std::uint8_t> container,
                                                   std::string_view expect_layer, const TileKey& expect_key) {
    const Result<ContainerView> view = parse_container(container);
    if (!view.ok()) {
        return HeightTileFetchResult{Status::Internal, nullptr};
    }
    if (view.value().header.layer != expect_layer || !(view.value().header.key == expect_key)) {
        return HeightTileFetchResult{Status::Internal, nullptr};
    }
    const Result<TileBody> body = decode_body(view.value().body);
    if (!body.ok()) {
        return HeightTileFetchResult{Status::Internal, nullptr};
    }

    // Heap, never the stack (vault TOOL-039: HeightTile is 256 KiB).
    auto tile = std::make_shared<HeightTile>();
    if (!decode_height_tile(body.value(), expect_key, *tile).ok()) {
        return HeightTileFetchResult{Status::Internal, nullptr};
    }

    // NoData stays NoData, everything else gets the offset in int64, and a sum
    // that would collide with NoData or overflow int32 rejects the tile.
    const std::int32_t offset = view.value().header.height_offset;
    if (offset != 0) {
        for (std::int32_t& h : tile->h) {
            if (h == kHeightNoData) {
                continue;
            }
            const std::int64_t v = static_cast<std::int64_t>(h) + offset;
            if (v <= static_cast<std::int64_t>(kHeightNoData) ||
                v > static_cast<std::int64_t>(std::numeric_limits<std::int32_t>::max())) {
                return HeightTileFetchResult{Status::Internal, nullptr};
            }
            h = static_cast<std::int32_t>(v);
        }
    }
    return HeightTileFetchResult{Status::Ok, std::move(tile)};
}

HeightTileFetchResult fetch_height_tile(ITransport& transport, const ReleaseId& rid, std::string_view layer,
                                        const TileKey& key) {
    const Response response = transport.send(Request{TileRequest{rid, std::string(layer), key, std::nullopt}});
    const auto* tile_response = std::get_if<TileResponse>(&response);
    if (tile_response == nullptr) {
        return HeightTileFetchResult{Status::Internal, nullptr};
    }
    if (tile_response->meta.status != Status::Ok) {
        return HeightTileFetchResult{tile_response->meta.status, nullptr};
    }
    return decode_height_tile_container(tile_response->container, layer, key);
}

HeightTileFetchResult fetch_height_tile_cached(std::mutex& cache_mutex,
                                               std::map<TileKey, std::shared_ptr<const HeightTile>>& cache,
                                               const TileKey& key, const HeightTileFetchFn& fetch_fn,
                                               bool* was_cache_hit) {
    if (was_cache_hit != nullptr) {
        *was_cache_hit = false;
    }
    {
        std::lock_guard<std::mutex> lk(cache_mutex);
        auto it = cache.find(key);
        if (it != cache.end()) {
            if (was_cache_hit != nullptr) {
                *was_cache_hit = true;
            }
            return HeightTileFetchResult{Status::Ok, it->second};
        }
    }

    // Deliberately outside the lock: two threads racing to fetch the same
    // missing tile just both call fetch_fn.
    HeightTileFetchResult fetched = fetch_fn(key);
    if (fetched.status != Status::Ok || !fetched.tile) {
        return fetched; // not cached: let a transient failure be retried later
    }

    std::lock_guard<std::mutex> lk(cache_mutex);
    auto [it, inserted] = cache.emplace(key, std::move(fetched.tile));
    (void)inserted; // first inserted value wins; a later racer's own fetch is discarded here
    return HeightTileFetchResult{Status::Ok, it->second};
}

RoadSegmentsResult decode_road_segments_response(const Response& response, const TileKey& expect_key,
                                                 geo::UtmZone zone) {
    const auto* tile_response = std::get_if<TileResponse>(&response);
    if (tile_response == nullptr) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }
    if (tile_response->meta.status == Status::NotFound && tile_response->meta.message == "outside coverage") {
        // Absent: a definitive, cacheable "not in this release" answer.
        // server.cpp's check_tile() gives this exact message only for
        // "outside coverage", never for "unknown layer"/"level not in layer".
        return RoadSegmentsResult{RoadFetchStatus::Absent, empty_road_segments()};
    }
    if (tile_response->meta.status != Status::Ok) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }

    const Result<ContainerView> view = parse_container(tile_response->container);
    if (!view.ok()) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }
    if (view.value().header.layer != kSrcOsmLayer || !(view.value().header.key == expect_key)) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }
    const Result<TileBody> body = decode_body(view.value().body);
    if (!body.ok()) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }
    const Result<OsmTile> osm_tile = decode_src_osm(body.value());
    if (!osm_tile.ok()) {
        return RoadSegmentsResult{RoadFetchStatus::Failed, nullptr};
    }

    std::vector<RoadSegment> segments = extract_road_segments(osm_tile.value().data, zone, RoadStyle::default_style());
    return RoadSegmentsResult{RoadFetchStatus::Ok, std::make_shared<const std::vector<RoadSegment>>(std::move(segments))};
}

RoadSegmentsResult fetch_road_segments(ITransport& transport, const ReleaseId& rid, const TileKey& key,
                                       geo::UtmZone zone) {
    const Response response = transport.send(Request{TileRequest{rid, std::string(kSrcOsmLayer), key, std::nullopt}});
    return decode_road_segments_response(response, key, zone);
}

RoadSegmentsResult fetch_road_segments_cached(std::mutex& cache_mutex, std::map<TileKey, RoadSegmentsResult>& cache,
                                              const TileKey& key, const RoadSegmentsFetchFn& fetch_fn,
                                              bool* was_cache_hit) {
    if (was_cache_hit != nullptr) {
        *was_cache_hit = false;
    }
    {
        std::lock_guard<std::mutex> lk(cache_mutex);
        auto it = cache.find(key);
        if (it != cache.end()) {
            if (was_cache_hit != nullptr) {
                *was_cache_hit = true;
            }
            return it->second;
        }
    }

    // Deliberately outside the lock - same rationale as fetch_height_tile_cached.
    RoadSegmentsResult fetched = fetch_fn(key);
    if (fetched.status == RoadFetchStatus::Failed) {
        return fetched; // not cached: let a transient failure be retried later
    }

    std::lock_guard<std::mutex> lk(cache_mutex);
    auto [it, inserted] = cache.emplace(key, std::move(fetched));
    (void)inserted; // first inserted value wins
    return it->second;
}

} // namespace g2m::world
