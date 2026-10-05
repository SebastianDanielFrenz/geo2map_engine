# g2m.roads.geom encoding 6 / version 1

Level-2 baked geometry, explicitly selected. Complete original topology and
midpoint ownership are retained. Every canonical graph stretch has exactly
one accepted parametric profile or explicit declined outcome. A geometry tile
does not promise every road is driveable; consumers must inspect declines.

Encoding 6 (street fidelity P1 slice 5) adds lane-aware cross-sections and
profile features; encoding 5 bodies (sections 1 and 2 only) still decode, with
no cross-sections (`RoadGeomTile::encoding == 5`), and are never written.
The decoder requires exactly sections 1 to 4, in that order, for encoding 6
(1 and 2 for encoding 5).

| Section | Codec | Content |
|---|---|---|
| 1 | as roads.graph | the roads.graph payload: encoding 5 (nodes with whitelisted tags) in a 6 body, encoding 4 (no tags) in a 5 body, byte-identical to the standalone tile's section |
| 2 | Zstd | entries: way id, stretch, outcome, profile (unchanged since encoding 5, byte for byte) |
| 3 | Zstd | one cross-section record per section-2 entry, same order |
| 4 | Zstd | profile features |

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
`u8 provenance` (0..3). The section is empty until street fidelity P1 slice 7
fills it from node tags.

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
`17d9b7c708b1bda9513ae80f8be1255d1f7c7cc1d6977af8e98a24dfd05ee2f9` as an encoding-6 body
(`4dcfcbca39838a8922c2749a9513eae7eecdbb398dec9a81b724d2970ff445f8` as the former
encoding-5 body, which `test_road_geom_xs.cpp` still reproduces from sections 1 and 2).

The release id changes with the encoding: the deriver config hash carries
`xs=1 enc=6 rules=de:<sha256 of de.json>`; terrain-only release ids are
unchanged. Goldens: `testdata/golden/g3xs_golden.txt`.
