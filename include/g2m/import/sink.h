// g2m/import/sink.h - where an importer (dem/osm/landcover, I3-I5) writes
// its encoded tile containers (G1c I2). Abstracted behind ITileSink so
// import code (and, more importantly, its tests) never has to open a real
// TileStore/SQLite file just to check what an importer produced.
#pragma once

#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/tile_store.h"

#include <cstdint>
#include <map>
#include <mutex>
#include <span>
#include <string>
#include <utility>
#include <vector>

namespace g2m::import {

// A destination for encoded tile containers. `put()` must be safe to call
// concurrently from several importer worker threads at once (I2's
// ordered_parallel.h's `consume` callback itself runs single-threaded, but
// a future importer may still fan a single logical tile out to more than
// one sink call, or share one sink across more than one ordered_for run).
class ITileSink {
public:
    virtual ~ITileSink() = default;

    // Stores one tile container under (layer, key). `header.layer`/
    // `header.key` must already match `layer`/`key` (checked); body_hash/
    // body_size are filled from `body`. Replaces whatever was there for
    // this (layer, key) before, same as TileStore::put.
    virtual Result<void> put(const std::string& layer, const TileKey& key, TileHeader header,
                              std::span<const std::uint8_t> body) = 0;
};

// Writes into a real TileStore, all addresses under one fixed scope (e.g.
// the source-store snapshot name, PLAN.md 3.1). TileStore is documented
// thread-safe for concurrent use by any number of callers, so this sink
// adds no locking of its own.
class StoreSink final : public ITileSink {
public:
    StoreSink(TileStore& store, std::string scope) : store_(store), scope_(std::move(scope)) {}

    Result<void> put(const std::string& layer, const TileKey& key, TileHeader header,
                      std::span<const std::uint8_t> body) override;

private:
    TileStore& store_;
    std::string scope_;
};

// Collects every put() in memory, keyed by (layer, key) - last write wins
// (TileStore::put's own "replaces what was there"). Thread-safe (a mutex
// guards the map): a test-only sink (see test_import_resolve.cpp's own
// MemorySink smoke test) and also handy for g2m_tiler's own dry runs.
class MemorySink final : public ITileSink {
public:
    struct Entry {
        TileHeader header;
        std::vector<std::uint8_t> body;
    };

    Result<void> put(const std::string& layer, const TileKey& key, TileHeader header,
                      std::span<const std::uint8_t> body) override;

    // Snapshot, sorted by (layer, key) (std::map's own iteration order) -
    // stable across runs for test assertions.
    std::vector<std::pair<std::pair<std::string, TileKey>, Entry>> entries() const;
    std::size_t size() const;

private:
    mutable std::mutex mutex_;
    std::map<std::pair<std::string, TileKey>, Entry> entries_;
};

} // namespace g2m::import
