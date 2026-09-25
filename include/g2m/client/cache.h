// g2m/client/cache.h - the g2m::Client tile cache (PLAN.md 7.2): tile
// containers of any release, keyed by (release id, layer, key), on a
// TileStore (tile_store.h: dedup by body hash, blob files over 256 KiB,
// LRU under a size cap, pinning, integrity check on every read, WAL crash
// safety). G1 has only the cache; the streamer that fills it arrives in G2.
//
// On disk: <dir>/cache.sqlite3 and <dir>/cache_blobs/ (docs/formats/cache_db.md).
//
// CONCURRENCY CONTRACT: that of TileStore. One ClientCache may be used by
// any number of threads (PLAN.md 7.3: the client's g2m workers); several
// ClientCache objects, in one process or several, may open the same
// directory.
#pragma once

#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/tile_store.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

namespace g2m {

struct ClientCacheConfig {
    std::filesystem::path dir;
    std::uint64_t size_cap_bytes = std::uint64_t{5} << 30; // 5 GiB; 0 = no cap
    std::size_t inline_limit = 256 * 1024;
    int busy_timeout_ms = 5000;
};

class ClientCache {
public:
    static Result<std::unique_ptr<ClientCache>> open(const ClientCacheConfig& config);

    // Stores a complete container under its own header's (rid, layer, key).
    // Fails if it does not parse or verify, or its rid is all zero.
    Result<void> put(std::span<const std::uint8_t> container, bool pin = false);
    // The complete container, or empty on a miss (including an entry just
    // dropped for failing its integrity check).
    Result<std::optional<std::vector<std::uint8_t>>> get(const ReleaseId& rid, std::string_view layer,
                                                         const TileKey& key);
    // Return whether the entry existed.
    Result<bool> set_pinned(const ReleaseId& rid, std::string_view layer, const TileKey& key, bool pinned);
    // Every entry of (rid, layer) in the inclusive key rectangle [lo, hi].
    Result<std::uint64_t> set_pinned_range(const ReleaseId& rid, std::string_view layer, const TileKey& lo,
                                           const TileKey& hi, bool pinned);
    Result<bool> remove(const ReleaseId& rid, std::string_view layer, const TileKey& key);

    Result<TileStoreStats> stats();
    Result<void> check_consistency();
    // The underlying store (tests, diagnostics).
    TileStore& store() { return *store_; }

private:
    explicit ClientCache(std::unique_ptr<TileStore> store) : store_(std::move(store)) {}
    std::unique_ptr<TileStore> store_;
};

} // namespace g2m
