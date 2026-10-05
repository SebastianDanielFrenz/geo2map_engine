# g2m.roads.geom encoding 7 / version 1

Level-2 baked geometry, explicitly selected. Complete original topology and
midpoint ownership are retained. Every canonical graph stretch has exactly
one accepted parametric profile or explicit declined outcome. A geometry tile
does not promise every road is driveable; consumers must inspect declines.

Encoding 7 (street fidelity P2 slice 8, the deriver writes it) adds junction
polygons, junction furniture and the profile provenance of the completion tier
(sections 5-7) to encoding 6. Encoding 6 (P1 slice 5: lane-aware cross-sections
and profile features) still decodes, with no junctions, furniture or origins
(`RoadGeomTile::encoding == 6`), and is written only when a tile asks for it
(`RoadGeomTile::encoding = 6`, no junctions or furniture, every origin Strict);
the compatibility tests reproduce the P1 bodies that way. Encoding 5 bodies
(sections 1 and 2 only) still decode, with no cross-sections, and are never
written. The decoder requires exactly sections 1 to 7 in that order for encoding
7, 1 to 4 for encoding 6 and 1 and 2 for encoding 5.

| Section | Codec | Content |
|---|---|---|
| 1 | as roads.graph | the roads.graph payload: encoding 5 (nodes with whitelisted tags) in a 6/7 body, encoding 4 (no tags) in a 5 body, byte-identical to the standalone tile's section |
| 2 | Zstd | entries: way id, stretch, outcome, profile (unchanged since encoding 5, byte for byte) |
| 3 | Zstd | one cross-section record per section-2 entry, same order |
| 4 | Zstd | profile features |
| 5 | Zstd | junctions owned by this tile (encoding 7) |
| 6 | Zstd | junction furniture: crossings, stop and give-way lines, signals (encoding 7) |
| 7 | Zstd | profile provenance, one record per entry (encoding 7) |

Sections 1-4 of an encoding-7 body have the encoding-6 format byte for byte;
their content differs where the completion tier (design 6.6) gives a declined
stretch a profile (section 2) and therefore a cross-section (section 3).

Section 2 stores a canonical varint entry count, then entries in ascending
way id / stretch order. Integers use little endian, ids use signed LEB128,
counts use unsigned LEB128, and floating values use IEEE-754 binary64 bits.
All sections are Zstd encoded by the writer; standard container bounds apply.

Each entry contains signed way id, u32 start/end original ref indices and u16
outcome. Outcome zero means success; other values are ErrorCode plus varint
byte count and error text (at most 512 bytes).

Successful entries contain:

- u8 flags: inferred start/end (bits 0/1), adjusted start/end (2/3),
  horizontal curvature exception (4); remaining bits are rejected.
- binary64 reference length, requested/fitted minimum radii; coincident count.
- reference-piece count, then each original u32 pair index, u8 piece index,
  station, x0/y0/heading0/length/curvature0/curvature-rate binary64 values.
- anchor count, then signed node id and binary64 station per anchor.
- u32 solver iterations, cubic count, then binary64 station/length/heights/
  endpoint grades per cubic.

## Section 3: cross-section records (encoding 6)

`varint record_count` (equals the entry count), then per entry:

```
u8 record_kind     0 Absent (the entry has no profile), 1 CrossSection, 2 Declined
kind 2: u16 decline_code (1 TooManyBands, 2 TotalWidthLimit, 3 NoCountedLane), svarint detail
kind 1:
  u16 flags              bits outside kXsDefinedMask are 0; bit 15 is reserved for the P1b licence bit
  u32 field_declines     bits outside kDeclineDefinedMask are 0
  u8  section_provenance bits 0-1 count, 2-3 placement, 4-7 zero
  varint band_count      1..32
  per band (left to right in the way direction):
    u8 kind (0..7), u8 direction (0..3), u8 change (0..4), u8 band_flags (bit0 counted, bit1 paved, bit2 marked_right)
    varint width_mm (0..20000), u16 turn (TurnBit mask), u16 provenance (2 bits each: kind, direction, width, turn, change)
    varint start_length_mm, start_from_width_mm, end_length_mm, end_from_width_mm
  u8 anchor_band (a counted band), u8 anchor_edge (0..2), u8 anchor_source (0..2)
  u8 offset_mode (0 None, 1 Ramps, 2 Hold, 3 Transition)
  svarint offset_start_mm, varint offset_start_length_mm, svarint offset_end_mm, varint offset_end_length_mm
  start join, end join: u8 join_kind (0..4), svarint neighbour_way_id, svarint raw_delta_mm
```

The cross-section's own `length_m` is not stored; the decoder copies the
profile's reference length. Boundaries are computed on demand
(`RoadCrossSection::boundaries_at`).

## Section 4: profile features (encoding 6)

`varint feature_count` (at most 65536), then per feature, sorted by entry
index, station bits (as u64) and node id: `varint entry_index` (an entry with a
cross-section), `svarint node_id`, binary64 station, `u8 kind` (0..9: Bump,
MiniBumps, RumbleStrip, Cushion, Hump, Table, Chicane, Choker, Island, Dip),
`u8 carve` (0 render only, 1 carved), `varint length_mm` (at most 50000),
`varint height_mm` (at most 300), `varint width_mm` (0 = full carriageway),
`u8 provenance` (0..3: 0 Tagged, 1 Derived, 2 Inferred, 3 Official as in
`Provenance`). Street fidelity P1 slice 7 fills it from the way-node tag
`traffic_calming` of every interior shape node of an entry's stretch (end nodes
belong to the junction and are skipped; unknown values and `yes` are ignored;
`painted_island` counts as Island). The nominal length is the clean
`traffic_calming:length` in metres (0 < L <= 50, Tagged) or the rules default
(`data/road_rules/de.json`, Inferred); a feature that would leave the stretch is
shortened to fit (Derived) or dropped when less than 1 m of room remains. `station`
is the reference point nearest the node (a clothoid knot, or the middle of the
rounded corner arc before it). `carve` is 1 exactly when `height_mm` > 0; the
lateral kinds (Chicane, Choker, Island) have height 0 and are render only.

Height function (`feature_height_m`, sum over the entry's features, only + - * /):
a plateau with two C1 smooth-step ramps (`road_smooth_step`). The ramp is half the
stored length, but at least 1 m (`kFeatureMinRampMm`, the grid's Nyquist limit), and
for a Table at most `10 * height` (1:10, `kFeatureTableRampRatio`), again not under
1 m. The carved length is `max(length_mm, 2 * ramp)`, so the narrowest carved feature
is 2 m; the nominal length stays in the file and the carved length is computed.
Dip is a depression (negative height). `width_mm` 0 means the whole carriageway;
cushions are carved across the lane group they sit in like any other feature
(owner decision 2026-10-05). Several features add.

Section 4 stores the features as built from the tags. It is **not** adjusted for
junctions: a consumer that carves runs `adjust_features_for_junctions` (it drops
features whose footprint reaches into a junction ring and adds the approach ramps
of a raised junction, design 4.4, D22/D23) on the decoded tile itself.

## Section 5: junctions (encoding 7)

```
varint junction_count                         <= 65536
per junction (ascending by owner node id; for one owner a single-node record precedes its ClusterTooLarge record):
  svarint owner_node_id                       == nodes[0]
  u16 flags                                   1 Cluster, 2 OnStructure, 4 HullFallback, 8 Partial, 16 RaisedTable; other bits 0
  u16 decline                                 0 None, 1 ClusterTooLarge, 2 ReachExceeded, 3 SingleApproach, 4 Degenerate, 5 Underground
  varint node_count                           1..4096; svarint node_id each, strictly ascending
  varint approach_count                       1..4096
  per approach: varint entry_index, u8 at_end (0/1), u8 status (0 Used, 1 NoProfile, 2 NoCrossSection, 3 Tunnel), binary64 trim_m
  varint absorbed_count; varint entry_index each, strictly ascending
  3 rings (road bed, paved, outer):
    varint vertex_count                       0 iff declined, else 3..1024
    svarint x_mm, y_mm of the first vertex (absolute UTM mm), then svarint dx_mm, dy_mm per further vertex
  binary64 raised_height_m                    RaisedTable: (0, 0.3]; else +0
  svarint raised_node                         RaisedTable: the node whose table/hump tag sets the height; else 0
```

Ownership (design 5.1): a tile stores exactly the junctions whose owner node
(`nodes[0]`, the smallest id) lies in the tile, by half-open floor membership on
integer millimetres (`road_tile_contains`). A junction is therefore stored once
in the world; its entry indices refer to this tile's entries, which hold every
approach and absorbed stretch (the halo). Approaches are the used ones in ring
order, then the excluded ones; `trim_m` is the distance from the junction-side
stretch end to the trim line. A ClusterTooLarge record carries the component's
nodes, untrimmed approaches and linking entries and no rings.

## Section 6: furniture (encoding 7)

```
varint count                                  <= 65536
per item:
  u8 kind (0 Crossing, 1 StopLine, 2 GiveWayLine, 3 Signal), u16 decline (0..7, FurnitureDecline)
  svarint node_id, varint entry_index, varint junction_index (0 = an entry item, else 1 + index into section 5)
  binary64 station_m, u8 travel (0 Forward, 1 Backward)
  svarint t_from_mm, svarint t_to_mm (|t| <= 200 m), varint length_mm (<= 100 m)
  u8 pattern (0..5, CrossingPattern), u8 flags (1 signalised, 2 lowered kerb, 4 spans join, 16 left post; bit 3 reserved, rejected)
```

Sorted strictly by (junction_index, entry_index, kind, station, node_id,
travel, t_from, flags). Entry items (crossings, lines and signals on a node
inside a stretch) are stored with their entry in every tile that holds the entry,
like the features; junction items (lines and signals derived from a junction
node) are stored once, with their junction. A declined item keeps its identity
(kind, node, entry, travel) and has no extent; the entry index of a
`JunctionDeclined` item is 0. Bit 3 (`kFurnitureInsideJunction`) stays reserved
(decision D28/D33): a consumer derives "inside a junction ring" from the owner
junction's approach trims.

## Section 7: provenance (encoding 7)

`varint count` (equals the entry count), then per entry `u8 provenance`
(0 Strict, 1 Retry, 2 BridgeCompleted, 3 GroundCompleted) and `u16
strict_decline_code` (0 for Strict; for a completed profile the ErrorCode of the
strict pass's decline, 1..ReadOnly).

## Junction, furniture and provenance validation (encoding 7)

Hard errors (design 9.2), each with its own rejection test
(`tests/layer/test_road_geom_enc7.cpp`):

- **Ownership and nodes.** Nodes 1..4096, strictly ascending, all in the graph;
  the owner lies in the tile; records ascending by owner (the ClusterTooLarge pair
  excepted); Cluster flag iff more than one node; a ClusterTooLarge record has at
  least two nodes; flags and decline in range.
- **Approaches.** 1..4096; entry index < entries; status consistent with the entry
  (Used: profile and cross-section; NoProfile/Tunnel: no profile; NoCrossSection:
  profile without cross-section); the junction-side end node is one of the
  junction's nodes; no (entry, end) twice; trim finite in [0, max(0, L - 0.5)] (0
  without a profile); absorbed entries ascending, both ends held, not an approach.
- **Rings.** Empty iff declined; otherwise 3..1024 vertices, within 1 km of the
  owner, counter-clockwise and simple (exact predicates), and nested: every road
  bed vertex inside, on, or within 2 mm of the paved ring, every paved vertex
  likewise for the outer ring (decision D34; each layer is mm-rounded on its own).
  At most 262144 ring vertices per tile.
- **Raised table.** With the flag the height is in (0, 0.3] and `raised_node` one of
  the junction's nodes; without it both are zero.
- **Furniture.** Enums and flags in range, bit 3 clear; indices in range; t and
  length mm-exact and bounded; strict order; the node on the junction (junction
  item) or in the graph; a declined item has no extent, a JunctionDeclined or
  Ambiguous item belongs to a junction and is no crossing, a JunctionDeclined
  item's junction is declined. A placed item sits on an entry with a profile and
  a cross-section at a station in [-length, L + length]; a junction item's junction
  is built, its node is the owner and its entry a Used approach. A crossing runs
  Forward with t_from < t_to and length > 0; a line has t_from < t_to, length > 0
  and **spans exactly one lane group of its travel** at its station, recomputed
  from the cross-section (`furniture_lane_groups`, mm-rounded: the acceptance rule
  of design 7.2, enforced at decode); a signal has t_from == t_to and length 0.
- **Provenance.** One record per entry; a Strict record names no decline, a
  completed one a decline code and its entry has a profile.

## Cross-section validation

Hard errors (design section 7.2): record count differs from the entry count;
Absent versus profile mismatch; any enum or reserved bit out of range; band
count outside 1..32; anchor band not a counted band; any ramp or offset length
above ceil(reference length mm) + 1, or the longest start ramp plus the longest
end ramp above that bound (offset ramps included); Hold with start/end lengths or
an end offset, Transition with lengths; a width above 20000 mm or nominal
widths summing above 100000 mm; join kind None with a neighbour or delta;
features unsorted, a station outside [0, L], a node not on the entry's stretch,
length above 50000 mm or height above 300 mm. Trailing bytes, over-size
sections and over-long varints fail like in every other section. A legacy
(encoding 5) tile must carry no cross-section or feature.


Physical attributes and all OSM tags, including directional maxspeed, resolve
from the retained source graph under the versioned class policy. Source ids
and ref indices survive exceptions and rejected profiles. Each stretch is a
single G1 chain; break-before flags are prohibited within a stored stretch.

Validation enforces complete canonical outcomes, finite values, bounded
counts (100,000 entries, 1,000,000 pooled pieces/anchors/cubics), 64 MiB section
bounds, reference extents and joins, node/pair membership, vertical C1 joins,
Bernstein grade bounds and endpoint second-derivative bounds. Truncated,
trailing, forged-count and invalid-profile inputs fail by value.

Derivation first resolves the target roads.graph, then discovers every L0
elev.base dependency for complete ways plus 65 m support margin. Missing
dependencies, mixed datums, hash/header mismatches and offset overflow fail
the whole derivation before a cache write. Native 1/256 m height offsets are
applied once; output offset is zero and datum is retained. Every dependency
body hash enters the standard server header. Only roads.graph drives output
coverage; DEM provides supporting context.

`BuiltinTerrainDerivers::ordered(false, true)` registers graph and geometry.
`g2m_tiler bake --layers g2m.roads.geom --levels 2` selects this path. Default
terrain-only releases and graph-only opt-in releases retain their identities.

The canonical flat-profile + tunnel-decline fixture has SHA-256
`a453aed52159ea3e3bac6ce41c3e98eaaebaeb57f0cc68004cf66e4d6791a618` as an encoding-7 body
(sections 1-4 as below, empty sections 5 and 6, two Strict provenance records),
`17d9b7c708b1bda9513ae80f8be1255d1f7c7cc1d6977af8e98a24dfd05ee2f9` as an encoding-6 body
(`test_road_geom.cpp` writes both) and
`4dcfcbca39838a8922c2749a9513eae7eecdbb398dec9a81b724d2970ff445f8` as the former
encoding-5 body, which `test_road_geom_xs.cpp` still reproduces from sections 1 and 2.

The release id changes with the encoding: the deriver config hash
(`road_geom_config_text()`) carries `xs=1 enc=7 rules=de:<sha256 of de.json>`
and, since slice 8, `junction=1 furniture=1 completion=1
completion-retry=20/16384 completion-abutment=32/60 completion-ground=40/32
strict-anchor=1 ring-nesting=2mm` - every value of
`served_road_completion_options()` the deriver builds with (retry spacing and
iterations, abutment ground path and walk budget, ground spacing and ramp), the
strict-anchor rule (D30) and the nesting allowance (D34). Terrain-only release
ids are unchanged. Goldens: `testdata/golden/g3xs_golden.txt` (the served
bodies), `testdata/golden/g3j_golden.txt` (sections 5-7).
