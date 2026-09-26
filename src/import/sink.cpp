// src/import/sink.cpp - see g2m/import/sink.h.

#include "g2m/import/sink.h"

#include <algorithm>

namespace g2m::import {

Result<void> StoreSink::put(const std::string& layer, const TileKey& key, TileHeader header,
                             std::span<const std::uint8_t> body) {
    header.layer = layer;
    header.key = key;
    const TileAddress addr{scope_, layer, key};
    return store_.put(addr, std::move(header), body, pin_);
}

Result<void> MemorySink::put(const std::string& layer, const TileKey& key, TileHeader header,
                              std::span<const std::uint8_t> body) {
    header.layer = layer;
    header.key = key;
    // ITileSink::put's own contract ("body_hash/body_size are filled from
    // body") - StoreSink gets this for free from TileStore::put
    // (tile_store.cpp); MemorySink has no such backing store, so it must
    // do it itself (same content_hash(body)/body.size() pattern).
    header.body_hash = content_hash(body);
    header.body_size = body.size();
    Entry entry;
    entry.header = std::move(header);
    entry.body.assign(body.begin(), body.end());
    std::lock_guard<std::mutex> lock(mutex_);
    entries_[{layer, key}] = std::move(entry);
    return Result<void>();
}

std::vector<std::pair<std::pair<std::string, TileKey>, MemorySink::Entry>> MemorySink::entries() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<std::pair<std::pair<std::string, TileKey>, Entry>> out(entries_.begin(), entries_.end());
    return out;
}

std::size_t MemorySink::size() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return entries_.size();
}

} // namespace g2m::import
