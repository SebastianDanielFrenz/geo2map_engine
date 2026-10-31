# `g2m.fields.geom` (farm field polygons)

Layer `g2m.fields.geom`, layer version 1, container encoding id 10 (`LayerEncoding::FieldsGeom`), encoding version 1,
level 2 (1024 m tiles). Code: `include/g2m/layer/field_tile.h`, `src/layer/field_tile.cpp` (codec, queries),
`field_subdivide.h` (kernel), `field_rules.h` (rules), `src/layer/derive_fields.cpp` (deriver). How the fields come to
be, ids, limits and the parcel seam: [../pipeline/fields.md](../pipeline/fields.md).

The layer is **facts only**: a polygon, where it came from, its land use class, the `crop=*` tag, area, a row
direction and flags. What a field means (soil, crop state, owner) is the game's.

## Tile contents

Every field whose outline touches the tile square (or contains its south-west corner) is stored in **each** such
tile, so a point query needs only the tile that holds the point. `FieldFlag::Owned` marks the one tile that holds the
field's anchor point (a point inside the field, half-open box test `[w, e) x [s, n)`); counting Owned records over a
set of tiles counts each field once. The header datum is None.

Coordinates are integer UTM millimetres of the tile's zone (same convention as `roads.graph`).

## Body

One section (type 1, zstd). All integers little endian; `varint` = LEB128, `svarint` = zigzag LEB128.

| Field | Type |
|---|---|
| tile key | u64 (`TileKey::packed`), a valid level-2 key |
| counts: blocks, subdivided, slivers_dropped, obstacles_ignored, sliver_blocks | 5 x varint (blocks the tile OWNS: the block's smallest outline vertex lies in the tile) |
| field count | varint (<= 200000) |
| per field | id u64, landuse u8, flags varint, crop (varint length + UTF-8, <= 255), area_mm2 varint, row_dir_cdeg u16, row_edge_mm varint, anchor x svarint + y svarint, hole count varint, outer ring, holes |
| ring | varint vertex count (>= 3), first vertex svarint x, y, then svarint deltas to the previous vertex |
| decline count | varint |
| per decline | source u8, osm_id svarint, reason u8 |

Limits: 8 000 000 vertices per tile, coordinates within +-2^40 mm. Counts are checked against the bytes left before any
allocation.

## Records

- `id`: bits 63..62 source (0 OSM way, 1 OSM relation, 2 Parcel, reserved), bits 61..16 OSM id (46 bits), bits 15..0
  piece index. Fields are ascending by id, ids unique.
- `landuse`: `data/field_rules/de.json` codes, append-only: 0 farmland, 1 meadow, 2 orchard, 3 vineyard, 4 farmyard.
- `flags`: bit 0 Owned, 1 Subdivided (outline from the raster subdivision, else the exact OSM outline), 2
  ObstaclesIgnored (obstacle areas covered more of the block than the cap allows), 3 SliverBlock (every piece was a
  sliver, block kept whole), 4 SimplifyFallback (a finer simplification tolerance was needed), 5 SmallArea, 6
  LargeArea, 7 Thin (plausibility, report only: never a reason to drop). Higher bits are rejected.
- `area_mm2`: outer area minus holes, exact.
- `row_dir_cdeg` / `row_edge_mm`: direction of the outer ring's longest edge (first of equal edges in ring order),
  hundredths of a degree in `[0, 18000)` counter-clockwise from grid east (a line has no heading, so 180 degrees
  fold onto 0), and its length. This is the plough/row direction hint.
- `anchor`: a deterministic point inside the field: the midpoint of the longest interior run over 9 fixed horizontal
  scan lines.
- Outer ring counter-clockwise, holes clockwise, each starting at its lexicographically smallest vertex, holes
  ascending by first vertex.
- declines: blocks that touch the tile but gave no field. Reasons: 0 InvalidGeometry (rings do not assemble into
  simple polygons), 1 RelationIncomplete (a member way is missing after the discovery stages, usually a block that
  reaches past the imported region), 2 TooLarge (bounding box above `block.max_extent_m`), 3 TooManyCells (raster
  budget), 4 TooManyTiles (needs more than `block.max_cover_tiles` tiles).

## Validation (decode and encode)

Canonical order, ids matching their parts, ring orientation and start, area / row direction / row edge / anchor
recomputed from the rings, Owned flag equal to "anchor inside the tile", every polygon touches the tile, flags within
the defined bits, decline order `(source, osm_id)` and known reasons, no trailing bytes. A decoded tile always
satisfies these; the encoder refuses anything else.

## Queries

`field_at(tile, point)`: index of the field containing the point (boundary counts as inside, lowest id on overlap).
`TerrainService::field_at(x, y)` / `field_at_grid(e, n)` answer it from the resident tile. `match_field(saved,
candidates, rules)` finds a saved field again after a re-bake (see the pipeline page).

## Outline export

`g2m_fields_probe outlines DERIVED_DIR RELEASE_HEX out.bin` writes the outline of every field (the owned copy of each, so one per
field) into one small file. A consumer that needs the shapes of all fields of a bake at once (the farming game, 26,432 fields in
Rhein-Main-Hessen) reads this file instead of decoding the tile store. `outlines-check` reads a file back and compares every field with
the layer (ids, anchors, rings, checksum); exit 1 on any difference. It is not a layer and not part of the bake: it is derived from the
layer and carries nothing the layer does not.

Layout, little endian: `"G2MFOUT1"` (8 bytes), zone `u8`, hemisphere `u8` (0 north, 1 south), field count `u32`; per field in ascending
id: id `u64`, anchor x and y (`svarint`), hole count (`varint`), the outer ring, then each hole; finally FNV-1a 64 over everything before it
(`u64`). A ring is its vertex count (`varint`), the first vertex (`svarint` x, y) and then `svarint` deltas to the previous vertex, in
integer UTM millimetres of the file's zone, the layer's own orientation (outer counter-clockwise, holes clockwise). `varint` is LEB128,
`svarint` is zigzag of it. Ids ascend and are unique.

Size and time for Rhein-Main-Hessen (release 2026-10-31): 26,432 fields, 1,020,436 vertices, 5.04 MB, written in about 0.4 s; the game
reads and joins it in about 70 ms. The id is the `id` column of `fields.csv`.
