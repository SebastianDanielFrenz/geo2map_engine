# g2m.tile/1 — tile container

One tile of one layer of one release: `HEADER ‖ BODY`. The header says what
the tile is and names the body by its SHA-256. The body is a list of typed
sections, and the layer's encoding gives them their meaning. Stores keep the
two parts separately: the body is deduplicated by its hash, the header is
stored per address ([cache_db.md](cache_db.md)).

## HEADER

| Field | Type | Rule |
|---|---|---|
| magic | 4 bytes | `G2MT` |
| major | u8 | 1 |
| minor | u8 | 0 (any other version: Unsupported) |
| flags | u16 | 0 |
| layer_len | u8 | |
| layer | layer_len bytes | valid layer name |
| layer_version | u32 | |
| rid | 32 bytes | release id; all zero = not assigned to a release (source-store rows) |
| scheme | u8 | 1 = UTM |
| zone | u8 | 1..60 |
| hemisphere | u8 | 0 north, 1 south |
| level | u8 | 0..20 |
| x | i32 | tile index; the key must be valid |
| y | i32 | |
| datum | u16 | 0 none (not a height datum), 1 DHHN2016 (German normal heights, DGM1), 2 EGM2008, 3 unspecified height reference |
| height_offset | i32 | 1/256 m, added to every stored height (0 for raw layers) |
| n_inputs | u16 | ≤ 4096 |
| inputs | n_inputs × 32 bytes | body hashes of the input tiles, strictly ascending (sorted, unique) |
| body_hash | 32 bytes | SHA-256 of BODY |
| body_size | u64 | size of BODY in bytes |
| checksum | u64 | FNV-1a 64 over every header byte before this field |

Datum value 0 (`None`) marks a tile that does not carry heights. Value 3 (`Unspecified`) marks height data whose vertical reference is unknown. Height-consuming road, rail, and carve derivation accepts `Unspecified` and carries it to the output; `None` remains invalid for elevation inputs. When sources provide known references, DHHN2016 and EGM2008 may not be mixed, even if an `Unspecified` source occurs between them.

`parse_container` checks the header checksum, `body_size` against the bytes
that follow, and the body hash. It does not decode the body.

## BODY

| Field | Type | Rule |
|---|---|---|
| encoding | u16 | 1 src.elev, 2 src.landcover, 3 src.osm, 4/5 roads.graph, 5 roads.geom (ids are per layer); ≥ 0x8000 vendor encodings |
| encoding_version | u16 | per encoding |
| n_sections | u16 | ≤ 64 |
| reserved | u16 | 0 |
| section table | n_sections × 12 bytes | see below |
| payloads | | section payloads in table order, back to back |

Section table entry:

| Field | Type | Rule |
|---|---|---|
| type | u16 | strictly ascending over the table |
| codec | u8 | 0 none, 1 zstd |
| reserved | u8 | 0 |
| raw_size | u32 | uncompressed size, ≤ 64 MiB; the sum over sections ≤ 256 MiB |
| stored_size | u32 | payload size; equals raw_size for codec 0 |

The sum of `stored_size` must equal the bytes after the table exactly. A zstd
payload must be exactly one complete frame, declaring a content size equal
to `raw_size`, with nothing after it. The decoder limits the window to
2^27 bytes.

### Pinned zstd parameters

Library v1.5.7 (the build fails on any other version); one `ZSTD_compress2`
call per section; level 12; `nbWorkers` 0 (never multithreaded); checksum
off (SHA-256 covers the body); content size on; dictionary id off; long
distance matching off; row match finder off (its automatic setting depends
on SIMD support); block splitter level 1 (no pre-splitting); split after
sequences off; literal compression mode auto (a pure function of the
level); format zstd1; forceMaxWindow 0; targetCBlockSize 0.

## Hashes

- **Body hash** = SHA-256(BODY). It is the tile's identity, and so the
  store's deduplication key, the protocol's ETag, and a pack's per-tile
  hash.
- **inputs** make a derived tile's provenance part of its header: a tile
  derived from different inputs has a different header even when its body
  is the same.
