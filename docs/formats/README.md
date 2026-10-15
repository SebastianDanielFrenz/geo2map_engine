# geo2map_engine formats

Every byte format g2m writes or reads from a file or a server. Each page is
normative: an encoder must produce exactly these bytes, and a decoder must
reject anything that breaks a rule stated here.

| Page | What | Code |
|---|---|---|
| [tile_container.md](tile_container.md) | `g2m.tile/1`: HEADER ‖ BODY, typed zstd sections | `g2m/layer/tile_container.h` |
| [src_elev.md](src_elev.md) | body encoding 1: `g2m.src.elev.<source>` heights | `g2m/layer/src_elev.h` |
| [src_landcover.md](src_landcover.md) | body encoding 2: `g2m.src.landcover` classes | `g2m/layer/src_landcover.h` |
| [src_osm.md](src_osm.md) | body encoding 3: `g2m.src.osm` columnar OSM | `g2m/layer/src_osm.h`, `osm_clip.h`, `osm_data.h` |
| [road_graph.md](road_graph.md) | `g2m.roads.graph` encodings 4/5: complete road ways, node tags, edge ownership | `g2m/layer/road_graph_tile.h` |
| [road_geom.md](road_geom.md) | `g2m.roads.geom` encoding 5: reference line, vertical profile, declines | `g2m/layer/road_geom_tile.h` |
| [terrain_carved.md](terrain_carved.md) | `g2m.terrain.carved`: carved terrain pyramid on body encoding 1 (inputs, values, header) | `g2m/layer/terrain_carve.h` |
| [pack.md](pack.md) | a 16×16 pack of containers with a directory of body hashes | `g2m/layer/pack.h` |
| [release_manifest.md](release_manifest.md) | release id (rid) and the `g2m.release/1` manifest JSON | `g2m/layer/release.h` |
| [regions.md](regions.md) | `g2m.regions/1` region definitions, ring assembly rules, `g2m.resolved_region/1` | `g2m/layer/region.h`, `g2m/core/geom/*` |
| [protocol.md](protocol.md) | protocol v1 paths, statuses, the `g2m.server/1` manifest JSON | `g2m/server/protocol.h` |
| [cache_db.md](cache_db.md) | tile store on disk: SQLite schema and blob-file layout | `g2m/server/tile_store.h` |

## Conventions (all binary formats)

- **Byte order:** little-endian everywhere. `u8/u16/u32/u64` are unsigned,
  `i32/i64` two's complement.
- **varint:** unsigned LEB128, 7 bits per byte, low bits first, at most 10
  bytes. It must be canonical: a multi-byte varint whose last byte is 0 is
  rejected, and so is a value that overflows 64 bits. So every value has
  exactly one encoding, and decoding and then re-encoding gives the input
  back (the fuzz targets check this).
- **svarint:** zigzag (`(v << 1) ^ (v >> 63)`), then varint.
- **Hashes:** `ContentHash` / `ReleaseId` are SHA-256 digests, 32 raw bytes
  in binary formats and 64 lower-case hex digits in JSON and paths.
- **Strings:** raw bytes. Layer names are `[a-z0-9_]` components separated
  by dots, each starting with a letter, at least two components, at most
  64 bytes; `g2m.*` is reserved for built-ins.
- **Tile keys:** scheme `utm{zone}{n|s}` (only scheme 1 = UTM exists), level
  0..20, x/y are tile indices. Level 2 tiles are 1024 m squares. In
  SQLite, a key is stored as `TileKey::packed()` (see
  [cache_db.md](cache_db.md)).
- **Determinism:** every encoder is a pure function of its input. zstd is
  pinned (v1.5.7, level 12, single-threaded, every parameter whose default
  is automatic set explicitly; see [tile_container.md](tile_container.md)).
  The encoded bytes are identical on every CI leg, which
  `testdata/golden/g1_golden.txt` checks.
- **Decoders** of untrusted bytes are bounds-checked and fail by value
  (`Result`), never by reading past a buffer or allocating an attacker-sized
  buffer before checking it against the input size. Each one has a libFuzzer
  target in `tests/fuzz_formats`.
