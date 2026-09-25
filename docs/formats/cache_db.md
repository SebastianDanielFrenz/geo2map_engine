# Tile store on disk (schema version 1)

`g2m::TileStore` backs the integrated server's source and derived stores
and the client cache (`g2m::ClientCache`, store name `cache`). A store named
`<name>` in directory `<dir>` consists of:

```
<dir>/<name>.sqlite3            SQLite database (WAL mode: plus -wal / -shm files)
<dir>/<name>_blobs/ab/cd/<hash> body files: 64 lower-case hex digits of the body's SHA-256,
                                ab = its first two digits, cd = the next two
```

Store names are `[a-z0-9_]{1,64}`. Bodies up to `inline_limit` (default
256 KiB) are kept in the database; larger bodies are kept in blob files.
`PRAGMA user_version = 1`; any other version is refused.

## Schema

```sql
CREATE TABLE meta(name TEXT PRIMARY KEY, value INTEGER NOT NULL) WITHOUT ROWID;
  -- 'body_bytes': total size of all distinct bodies (the LRU size cap counts this)
CREATE TABLE bodies(
  id INTEGER PRIMARY KEY,
  hash BLOB NOT NULL UNIQUE,   -- SHA-256 of the body, 32 raw bytes
  size INTEGER NOT NULL,       -- body size in bytes
  refs INTEGER NOT NULL,       -- number of tiles rows with this hash
  data BLOB);                  -- the body, or NULL when it is a blob file
CREATE TABLE tiles(
  scope TEXT NOT NULL,         -- release id (hex) for derived store / cache; snapshot name for the source store
  layer TEXT NOT NULL,
  key INTEGER NOT NULL,        -- TileKey::packed()
  hash BLOB NOT NULL,          -- body hash (bodies.hash)
  header BLOB NOT NULL,        -- the encoded g2m.tile/1 HEADER
  last_used INTEGER NOT NULL,  -- LRU clock: MAX(last_used) + 1 on every get/put
  pinned INTEGER NOT NULL,     -- 0/1
  PRIMARY KEY(scope, layer, key)) WITHOUT ROWID;
CREATE INDEX tiles_lru ON tiles(pinned, last_used);
CREATE INDEX tiles_used ON tiles(last_used);
CREATE INDEX tiles_hash ON tiles(hash);
```

`TileKey::packed()` layout:

| Bits | Field |
|---|---|
| 54..59 | zone |
| 53 | hemisphere (1 = south) |
| 48..52 | level |
| 24..47 | y + 2^23 |
| 0..23 | x + 2^23 |

Numeric order equals the key order `(zone, hemisphere, level, y, x)`, so a
range of x within one row is a range of `key`.

## Rules

- **Dedup:** one `bodies` row (and at most one file) per body hash.
  `refs` counts the addresses that use it; the body is removed with its
  last reference.
- **LRU:** the clock is a counter inside the database (no wall clock), so
  eviction order is deterministic and survives restarts. With a size cap,
  `put` evicts unpinned entries in `last_used` order until `body_bytes`
  fits the cap. The entry being put and pinned entries are never evicted.
- **Integrity:** every read re-hashes the body and checks the header
  (checksum, body hash, layer and key against the address). If the body
  is damaged, every entry sharing it is dropped. If only the header is
  damaged, only that entry is dropped. Either way the read reports a miss.
- **Crash safety:** WAL, `synchronous = NORMAL`, and one `BEGIN IMMEDIATE`
  transaction per mutation. A blob file is written (to a temp file
  `<hash>.tmp`, then renamed) before the row that refers to it
  commits. A crash can leave an orphan file, never a row without its file.
  An orphan is harmless: a later put of the same body rewrites or accepts
  the file after checking its hash.
- **Windows:** a file open for reading can be neither renamed over nor
  deleted. A put that finds an identical file already in place (same hash)
  keeps it, and otherwise retries briefly. A failed delete leaves an
  orphan, as above.
