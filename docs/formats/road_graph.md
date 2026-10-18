# g2m.roads.graph / encoding 6, version 1 (encodings 5 and 4 still readable)

Level-2 tiles cover 1024 m squares with a fixed 128 m halo. Selection keeps
each complete way that intersects the closed expanded square, including all
of its original nodes and tags beyond the halo. Shared OSM node ids establish
connectivity; geometric crossings do not.

One Zstd section (type 1) contains these fields in order. Fixed integers use
little-endian byte order; signed varints use the core byte codec's ZigZag form.

| Field | Representation |
|---|---|
| Tile key, halo metres | u64 packed key, u32 |
| Far-zone ways, repeated consecutive refs | two unsigned varints |
| Nodes | count varint; signed id, x-mm, y-mm varints per node; **encoding 5 only:** tag count varint and key/value strings per node |
| Ways | count varint; signed id, u8 rank, refs count and signed ref ids; tags count and strings |
| Strings | unsigned byte-length varint followed by UTF-8/source bytes |
| Owned edges | count varint followed by unsigned edge-index varints |
| Turn restrictions | **encoding 6 only:** count varint, then per record: signed relation id, u8 value code, signed from way, signed to way, signed via node (0 = none), via-way count varint (0..16), signed via way ids |

Nodes and ways sort strictly by OSM id; tag keys sort strictly.

**Encodings 4, 5 and 6.** Encoding 5 (street fidelity P1, slice 2) adds the node tags. Encoding 4 is the
same payload without that field. Encoding 6 (G4 routing, slice 3) adds the turn-restriction block after
the owned edges. The decoder reads all three (a 4 tile has no node tags, a 4 or 5 tile has no
restrictions); the writer produces 6. `roads.geom` does not carry restrictions: its encoding-5 section 1
still embeds the encoding-4 payload and its encodings 6 and 7 embed encoding 5, and a decoded
roads.geom graph has no restrictions.

**Turn restrictions (encoding 6).** One record per OSM `type=restriction` relation that applies to cars
(`extract_turn_restrictions` rules: `restriction:motorcar` wins, `except` naming motorcar/motor_vehicle/
vehicle drops it, rules for other vehicles only are skipped, `no_entry`/`no_exit` expand to one record
per pair). The value is a u8 code into the ten OSM restriction values (`no_left_turn`, `no_right_turn`,
`no_straight_on`, `no_u_turn`, `no_entry`, `no_exit`, `only_left_turn`, `only_right_turn`,
`only_straight_on`, `only_u_turn`); the kind (No or Only) follows from the value and is not stored.
A rule has exactly one of a via node or one to sixteen via ways. A tile holds a record when its via node
is a node of the tile graph or any via way is a way of the tile graph; the from and to ways need not be
present. So every tile that touches the junction (or the via way) carries the same record, and a region
assembled from tiles dedups the copies by (relation id, from way, to way); copies that disagree are
rejected. Records are strictly sorted by (relation id, from way, to way). Rejected on decode and encode: a
code of 10 or more, a value whose kind does not match, an out-of-order or duplicate record, a zero id, a
record with both or neither of via node and via ways, more than 16 via ways, a record the tile does not
hold, more than 200,000 records, a count larger than the remaining bytes allows.

**Node tags.** Only these keys are kept, because shared nodes also carry addresses, names and
other tags no road consumer reads: `barrier`, `crossing`, `crossing:markings`, `direction`,
`height`, `highway`, `kerb`, `length`, `traffic_calming`, `traffic_calming:length`,
`traffic_signals:direction`, `width`. The list is sorted; a key outside it, a tag list that is
not strictly ascending by key and a duplicated key are rejected. Adding a key is a format
change (a new encoding id). Node tags count toward the pooled tag and string limits below. Edges reconstruct
from consecutive original refs in way order, omitting equal adjacent ids while
preserving original pair indices. No unused nodes, unresolved refs, redundant
edges or trailing bytes are accepted. Each edge belongs to the half-open tile
containing its floor-rounded integer-mm midpoint. The ownership list must
match exactly; neighbouring halo copies retain identical edge identities.

Limits: 1,000,000 nodes, 100,000 ways, 4,000,000 pooled references,
1,000,000 pooled tags, 65,535 bytes per string and 64 MiB raw section size.
Projected coordinates must have absolute value below 2^62. Decoder counts
must also fit the remaining bytes before allocations.

The deriver requires the exact nine neighbouring level-2 src.osm inputs.
Absent inputs are explicit empty records. Present headers, body sizes and hashes
are validated. Identical source objects deduplicate by id; conflicting copies
fail. Node metadata is checked during merging but is not routing metadata in
this geometric layer. Turn restrictions and traffic controls belong to G4.

Server registration is opt-in via BuiltinTerrainDerivers::ordered(true) and
builtin_local_release_params(..., true). The bake command opts in automatically
when --layers includes g2m.roads.graph; use --levels 2-2 (levels a layer lacks are skipped anyway).
Existing terrain-only releases retain their current identities and goldens.
