# Tile pack (version 1)

16 × 16 tiles of one layer of one release in one blob: a directory of the
per-tile state and body hash, then the containers. It is the unit of the
`/packs` endpoint. The `/hashes` endpoint returns the directory alone, so a
client can see which of its cached tiles are still current without
downloading them.

Pack coordinates: `pack.x = floor(tile.x / 16)` and the same for y, at the
tile's own zone and level. Entry `i = 16·dy + dx` is tile
`(16·pack.x + dx, 16·pack.y + dy)`.

## Layout

| Field | Type | Rule |
|---|---|---|
| magic | 4 bytes | `G2MP` |
| version | u8 | 1 |
| reserved | u8, u16 | 0, 0 |
| layer_len, layer | u8, bytes | valid layer name |
| rid | 32 bytes | |
| scheme, zone, hemisphere, level | 4 × u8 | scheme 1 = UTM; the key must be valid and 16 × x/y must stay in the tile index range |
| x, y | i32, i32 | pack coordinates |
| directory | 256 × 41 bytes | per entry: `state` u8, `body_hash` 32 bytes, `size` u64 |
| data | | the containers of the present entries, in entry order, back to back |

Entry states:

| State | Meaning |
|---|---|
| 0 NotFound | outside coverage or absent for this release (definitive) |
| 1 Present | complete g2m.tile/1 container included |
| 2 Unavailable | could not be produced now (the protocol's 503: retry later) |
| 3 Degraded | the server is offline and does not have the tile cached |

Rules:

- A non-present entry has `size` 0 and an all-zero hash.
- A present entry has `size` > 0, and its container must parse (header
  checksum and body hash). Its header must name this pack's layer, rid and
  the entry's tile key, and its body hash must equal the entry's hash.
- The sizes must add up to the data exactly.

## Pack directory (version 1, G6)

The body of the `/hashes` endpoint over HTTP (`encode_pack_directory` /
`decode_pack_directory`): the pack's head and its 256 states and hashes,
without sizes or data.

| Field | Type | Rule |
|---|---|---|
| magic | 4 bytes | `G2MH` |
| version | u8 | 1 |
| reserved | u8, u16 | 0, 0 |
| layer_len, layer, rid, scheme, zone, hemisphere, level, x, y | | exactly as in the pack above |
| directory | 256 × 33 bytes | per entry: `state` u8 (0..3, as above), `body_hash` 32 bytes |

Rules: a non-present entry has an all-zero hash; nothing follows the last
entry. The encoding is canonical: `encode(decode(b)) == b` for every
accepted `b` (fuzz target `packDirectory`).
