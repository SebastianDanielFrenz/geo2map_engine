# g2m.roads.graph / encoding 5, version 1 (encoding 4 still readable)

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

Nodes and ways sort strictly by OSM id; tag keys sort strictly.

**Encoding 4 and 5.** Encoding 5 (street fidelity P1, slice 2) adds the node tags. Encoding 4 is the
same payload without that field; the decoder reads both (a 4 tile has no node tags), the
writer produces 5. The `roads.geom` encoding-5 section 1 still embeds the encoding-4 payload; a
roads.geom encoding 6 will embed 5.

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
