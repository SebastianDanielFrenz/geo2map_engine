// g2m/server/tile_store.h - a content-addressed tile store on SQLite plus a
// blob file store (PLAN.md 3.1 source/derived stores, 7.2 client cache;
// on-disk layout: docs/formats/cache_db.md).
//
// A store holds tile containers (g2m.tile/1, split into HEADER and BODY)
// addressed by (scope, layer, key). The scope is a string the owner picks:
// the release id in hex for the derived store and the client cache, a
// snapshot name for the source store.
//  - Dedup: bodies are stored once per SHA-256 (the body hash), whatever
//    number of addresses refer to them.
//  - Blobs: bodies larger than inline_limit live in files
//    <dir>/<name>_blobs/ab/cd/<64 hex digits>, smaller ones inside the
//    database.
//  - Integrity: every get() re-hashes the body and checks the header
//    (checksum, body hash, layer, key). A mismatch drops the entry and
//    reports a miss (integrity_drops counts it); a damaged BODY drops
//    every entry that shares it, since it is damaged for all of them.
//  - LRU: every get() and put() marks the entry used. With a size cap,
//    put() evicts the least recently used unpinned entries until the total
//    body size fits the cap again (the entry being put is never evicted by
//    its own put, and pinned entries never are: the total may stay above
//    the cap if pinned entries alone exceed it).
//  - Crash safety: SQLite WAL, synchronous=NORMAL; every mutation is one
//    transaction (BEGIN IMMEDIATE). A blob file is written (temp file,
//    then rename) before the row that refers to it commits, and deleted
//    while the transaction that drops its last reference is open. A crash
//    can therefore leave an orphan blob file, never a row without its
//    file - and if one is ever missing or damaged anyway, get() drops the
//    entry as above.
//
// CONCURRENCY CONTRACT
//  - One TileStore object may be used from any number of threads at once
//    (PLAN.md 7.3: the client's g2m workers). Every method is thread-safe.
//    Database access is serialised on the object's one connection; body
//    hashing and blob-file reads in get() run outside that lock, so readers
//    of large blobs do not block each other or writers.
//  - Several TileStore objects - in one process or several - may open the
//    same directory and name. Writers are serialised by SQLite's write lock
//    (busy_timeout_ms, then ErrorCode::Busy); every blob-file creation and
//    deletion happens under that lock, so no object can delete a file
//    another has just (re)written. A reader that finds a blob file gone
//    re-checks under the write lock: if the entry was removed concurrently
//    it reports a plain miss, not an integrity failure.
//  - Results are values: nothing returned refers to store-internal memory.
#pragma once

#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/tile_container.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

namespace g2m {

struct TileStoreConfig {
    std::filesystem::path dir;                  // created if missing (never for read_only, see below)
    std::string name = "tiles";                 // <dir>/<name>.sqlite3, <dir>/<name>_blobs/; [a-z0-9_]+
    std::uint64_t size_cap_bytes = 0;           // 0 = no cap; counts distinct body bytes
    std::size_t inline_limit = 256 * 1024;      // bodies larger than this go to blob files
    int busy_timeout_ms = 5000;
    // Opens the sqlite database with SQLITE_OPEN_READONLY via the URI
    // "file:<path>?mode=ro&immutable=1" instead of the usual
    // READWRITE|CREATE: no directory is created, no schema-creation
    // transaction runs, and no PRAGMA that could touch the file (journal
    // mode, synchronous) is issued - `immutable=1` tells SQLite the file
    // will not change out from under this connection for as long as it is
    // open, which lets it skip locking and change-detection entirely (and
    // is why this is only for a FINISHED, PINNED store: a concurrent writer
    // elsewhere, or an eviction removing an entry this reader has open,
    // is exactly what `immutable` promises will not happen). Impl::touch()
    // (the LRU-order update every get() would otherwise issue) becomes a
    // no-op, and put()/put_container()/set_pinned()/set_pinned_range()/
    // remove()/evict_to() all return ErrorCode::ReadOnly immediately
    // without touching the database. Not meant for a store still being
    // imported into or evicted from - use the default (false) for that.
    bool read_only = false;
};

struct TileAddress {
    std::string scope; // 1..128 bytes
    std::string layer; // valid layer name
    TileKey key;
    friend bool operator==(const TileAddress&, const TileAddress&) = default;
};

struct StoredTile {
    TileHeader header;
    std::vector<std::uint8_t> header_bytes; // the encoded header (g2m.tile/1)
    std::vector<std::uint8_t> body;
    bool pinned = false;

    // HEADER || BODY: the complete container.
    std::vector<std::uint8_t> container() const;
};

struct TileStoreStats {
    std::uint64_t tiles = 0;          // addresses
    std::uint64_t bodies = 0;         // distinct bodies
    std::uint64_t blob_files = 0;     // bodies stored as files
    std::uint64_t body_bytes = 0;     // sum of distinct body sizes (what the cap limits)
    std::uint64_t pinned_tiles = 0;
    // Since this object was opened:
    std::uint64_t integrity_drops = 0;  // entries dropped by the integrity check
    std::uint64_t evicted_tiles = 0;
};

struct TileIndexEntry {
    std::string layer;
    TileKey key;
    ContentHash body_hash{};
};

class TileStore {
public:
    static Result<std::unique_ptr<TileStore>> open(const TileStoreConfig& config);
    ~TileStore();
    TileStore(const TileStore&) = delete;
    TileStore& operator=(const TileStore&) = delete;

    const TileStoreConfig& config() const;

    // Stores `body` under `addr` with `header` (header.layer/key must match
    // addr; header.body_hash/body_size are set here). Replaces what was
    // there. pin = true pins the entry; false keeps its current pin state.
    Result<void> put(const TileAddress& addr, TileHeader header, std::span<const std::uint8_t> body,
                     bool pin = false);
    // A complete container (parsed and verified first).
    Result<void> put_container(const TileAddress& addr, std::span<const std::uint8_t> container, bool pin = false);

    // Empty on a miss, including an entry just dropped for failing its
    // integrity check. Errors are I/O or database failures only.
    Result<std::optional<StoredTile>> get(const TileAddress& addr);
    // The body hash stored for addr, without reading or verifying the body
    // and without touching the LRU order.
    Result<std::optional<ContentHash>> peek_hash(const TileAddress& addr);

    // Return whether the entry existed.
    Result<bool> set_pinned(const TileAddress& addr, bool pinned);
    Result<bool> remove(const TileAddress& addr);
    // Pins/unpins every entry of (scope, layer) whose key lies in the
    // inclusive rectangle [lo, hi] (same zone and level); returns the count.
    Result<std::uint64_t> set_pinned_range(const std::string& scope, const std::string& layer, const TileKey& lo,
                                           const TileKey& hi, bool pinned);

    // Evicts least recently used unpinned entries until body_bytes <=
    // target_bytes (or nothing unpinned is left); returns entries evicted.
    Result<std::uint64_t> evict_to(std::uint64_t target_bytes);

    Result<TileStoreStats> stats();
    // Every entry of a scope, sorted by (layer, key). For release ids and
    // coverage (release.h).
    Result<std::vector<TileIndexEntry>> index(const std::string& scope);
    // Full consistency check: reference counts, the size total, every blob
    // file present with the right size. Does not re-hash bodies. For tests
    // and diagnostics.
    Result<void> check_consistency();

private:
    struct Impl;
    explicit TileStore(std::unique_ptr<Impl> impl);
    std::unique_ptr<Impl> impl_;
};

} // namespace g2m
