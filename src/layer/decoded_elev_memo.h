// src/layer/decoded_elev_memo.h - a small LRU of decoded g2m.src.elev
// bodies (PLAN.md G2.1), private to derive_terrain.cpp: a level-2 DGM1 tile
// is the shared input of every one of its (up to 16) level-0 children, so
// without this a 256x256-tile bake would decode/inflate the same 1024x1024
// tile's zstd+predictor-coded body up to 16 times over. Keyed by the
// input's own content (body) hash - a pure function of the bytes - so
// output never depends on whether a lookup is a cache hit or miss, and
// concurrent derive() calls for sibling tiles are safe (thread-safe,
// mutex-guarded; ElevBaseDeriver::derive is documented stateless/
// thread-safe per IDeriver's contract, deriver.h).
#pragma once

#include "g2m/core/hash.h"
#include "g2m/core/result.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/tile_container.h"

#include <algorithm>
#include <list>
#include <memory>
#include <mutex>
#include <unordered_map>

namespace g2m::layer_detail {

struct ContentHashHash {
    std::size_t operator()(const ContentHash& c) const {
        return static_cast<std::size_t>(Fnv1a64{}.bytes(c.data(), c.size()).value());
    }
};

// Bounded LRU, capacity fixed at construction (8 entries by default: 8 x
// ~4 MiB decoded level-2 tiles). get() is the only entry point: decode a
// body once per distinct content hash, share the result thereafter.
class DecodedElevMemo {
public:
    explicit DecodedElevMemo(std::size_t capacity = 8) : capacity_(std::max<std::size_t>(1, capacity)) {}

    Result<std::shared_ptr<const ElevTile>> get(const ContentHash& body_hash, const TileBody& body) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto it = map_.find(body_hash);
            if (it != map_.end()) {
                order_.splice(order_.begin(), order_, it->second.lru_it);
                return it->second.tile;
            }
        }
        // Decoded outside the lock: decode_src_elev is a pure function of
        // `body`, so two threads racing on the same missing key simply
        // redo the same work once each - never a correctness issue, only a
        // (rare, bounded) duplicated decode.
        Result<ElevTile> decoded = decode_src_elev(body);
        if (!decoded) return decoded.error();
        auto tile = std::make_shared<const ElevTile>(std::move(decoded).value());

        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = map_.find(body_hash);
        if (it != map_.end()) {
            // Another thread already inserted this key while we decoded;
            // keep the existing (bit-identical) entry.
            order_.splice(order_.begin(), order_, it->second.lru_it);
            return it->second.tile;
        }
        order_.push_front(body_hash);
        map_.emplace(body_hash, Entry{tile, order_.begin()});
        while (order_.size() > capacity_) {
            const ContentHash victim = order_.back();
            order_.pop_back();
            map_.erase(victim);
        }
        return tile;
    }

private:
    struct Entry {
        std::shared_ptr<const ElevTile> tile;
        std::list<ContentHash>::iterator lru_it;
    };

    std::size_t capacity_;
    std::mutex mutex_;
    std::list<ContentHash> order_; // most-recently-used at front
    std::unordered_map<ContentHash, Entry, ContentHashHash> map_;
};

} // namespace g2m::layer_detail
