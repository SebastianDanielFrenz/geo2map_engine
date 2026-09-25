# src.osm — body encoding 3 (version 1)

OSM nodes, ways and relations of `g2m.src.osm`, clipped per level-2 tile
(1024 m) with a halo. The encoding keeps ids, tags, way node lists and
relation membership, so topology survives: G3 builds the road graph from
these tiles, and a way that crosses tiles has the same id and the same full
node list in every tile it touches. OSM metadata (version, timestamp,
changeset, user) is not kept.

## Canonical data (`osm::OsmData`)

A decoded tile always satisfies these rules, and the encoder refuses data
that breaks them:

- **strings:** sorted strictly ascending by bytes, each ≤ 65535 bytes. Every
  tag key, tag value and member role is an index into this list.
- **nodes, ways, relations:** each list sorted strictly ascending by id.
- **tags:** each element's tags have strictly ascending keys, so each key
  has at most one value.
- **way refs:** every way has ≥ 1 ref, and every ref names a node in this
  tile. A way is always complete.
- **relation members:** order kept exactly. Members may name elements that
  are not in the tile: the membership list is complete, but the members are
  not all present.
- **coordinates:** OSM fixed point, int32 1e-7 degrees; latitude within
  ±90°, longitude within ±180°.

## Selection contract (`clip_osm`)

`B` is the tile's square grown by `halo_m` (default 128 m) on every side, as
a closed box in integer millimetres.

1. A node is IN if its position lies in `B`.
2. A way is included if any of its segments touches `B`. A one-node way is
   included if its node is IN. An included way keeps its full ref list.
3. The tile's nodes are the IN nodes plus every node an included way refers
   to.
4. A relation is included if any member is an included node, way or
   relation. This closes transitively: the parent of an included relation
   is included too. The relation's member list is kept complete, but
   members that are not otherwise included are not added.
5. Strings are re-interned to those the tile uses.

Positions come from g2m's own deterministic transverse Mercator (GRS80, the
tile's zone), floored to whole millimetres. Every test after that is exact
integer arithmetic. A node more than 30° of longitude from the central
meridian has no position: it is never IN, and a segment ending in it touches
`B` only if its other end is IN. The output depends only on
`(set, key, halo_m)`.

## Sections

All 12 sections are present, with types 1..12 in order. All but meta are
zstd-compressed.

| Type | Name | Content |
|---|---|---|
| 1 | meta (none) | `halo_m` u32, `n_strings` u32, `n_nodes` u32, `n_ways` u32, `n_relations` u32 |
| 2 | strings | per string: varint length, bytes |
| 3 | node ids | id column |
| 4 | node lat | per node: svarint(lat − previous lat), starting from 0 |
| 5 | node lon | per node: svarint(lon − previous lon), starting from 0 |
| 6 | node tags | per node: tag list |
| 7 | way ids | id column |
| 8 | way refs | per way: varint count (≥ 1), then per ref svarint(ref − previous ref); the chain runs across ways, from 0 |
| 9 | way tags | per way: tag list |
| 10 | relation ids | id column |
| 11 | relation members | per relation: varint count, then per member: varint type (0 node, 1 way, 2 relation), svarint(ref − previous ref) with the chain running across relations from 0, varint role (string index) |
| 12 | relation tags | per relation: tag list |

**Id column:** the first id is an svarint. Every later id is a varint delta
of at least 1 (strictly ascending).

**Tag list:** varint count, then per tag a varint key and a varint value,
both string indices, with keys strictly ascending.

Delta chains use wrap-around uint64 arithmetic, so they are a bijection on
int64. Range rules are checked on the decoded values.

The decoder checks every column to the end. Trailing bytes are an error,
and so are count fields larger than their column could hold (each element
costs at least one byte), before anything is reserved. Every canonical rule
above is enforced, including "every way ref names a node in the tile".
