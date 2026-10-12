// src/world/tile_cache.h - INTERNAL. The in-memory decoded-tile cache of
// g2m::world::TerrainService: a byte-weighted LRU over (JobKey -> ErasedOutcome).
//
// Only Ready and Absent outcomes are stored (Failed is never cached: a later request
// retries). Thread-safe; every method takes the one mutex briefly and never calls out.
#pragma once

#include "g2m/world/ticket.h"

#include <cstddef>
#include <cstdint>
#include <list>
#include <map>
#include <memory>
#include <mutex>
#include <optional>

namespace g2m::world::detail {

class TileCache {
public:
    explicit TileCache(std::size_t capacity_bytes) : capacity_(capacity_bytes) {}

    // The stored outcome (marking it most recently used), or nothing. An entry stored under another
    // product type is a miss. Counts a hit or a miss.
    std::optional<ErasedOutcome> get(const JobKey& key, const void* type) {
        std::lock_guard lock(mu_);
        const auto it = index_.find(key);
        if (it == index_.end() || it->second->type != type) {
            ++misses_;
            return std::nullopt;
        }
        entries_.splice(entries_.begin(), entries_, it->second);
        ++hits_;
        return it->second->outcome;
    }

    // The value of a Ready entry or null. Marks it used; counts nothing (a peek is not a request).
    std::shared_ptr<const void> peek(const JobKey& key, const void* type) {
        std::lock_guard lock(mu_);
        const auto it = index_.find(key);
        if (it == index_.end() || it->second->type != type || it->second->outcome.state != TileState::Ready)
            return nullptr;
        entries_.splice(entries_.begin(), entries_, it->second);
        return it->second->outcome.value;
    }

    void put(const JobKey& key, const void* type, const ErasedOutcome& outcome) {
        if (outcome.state != TileState::Ready && outcome.state != TileState::Absent) return;
        std::lock_guard lock(mu_);
        remove_locked(key);
        const std::size_t weight = outcome.state == TileState::Ready ? outcome.bytes + kEntryOverhead : kEntryOverhead;
        entries_.push_front(Entry{key, type, outcome, weight});
        index_[key] = entries_.begin();
        bytes_ += weight;
        // Evict from the cold end, but never the entry just inserted.
        while (bytes_ > capacity_ && entries_.size() > 1) {
            const Entry& victim = entries_.back();
            bytes_ -= victim.weight;
            index_.erase(victim.key);
            entries_.pop_back();
        }
    }

    struct Stats {
        std::uint64_t hits = 0, misses = 0;
        std::size_t entries = 0, bytes = 0;
    };
    Stats stats() const {
        std::lock_guard lock(mu_);
        return Stats{hits_, misses_, entries_.size(), bytes_};
    }

private:
    static constexpr std::size_t kEntryOverhead = 256;
    struct Entry {
        JobKey key;
        const void* type;
        ErasedOutcome outcome;
        std::size_t weight;
    };
    void remove_locked(const JobKey& key) {
        const auto it = index_.find(key);
        if (it == index_.end()) return;
        bytes_ -= it->second->weight;
        entries_.erase(it->second);
        index_.erase(it);
    }

    mutable std::mutex mu_;
    std::size_t capacity_;
    std::list<Entry> entries_; // front = most recently used
    std::map<JobKey, std::list<Entry>::iterator> index_;
    std::size_t bytes_ = 0;
    std::uint64_t hits_ = 0, misses_ = 0;
};

} // namespace g2m::world::detail
