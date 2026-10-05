# g2m.roads.geom encoding 5 / version 1

Level-2 baked geometry, explicitly selected. Complete original topology and
midpoint ownership are retained. Every canonical graph stretch has exactly
one accepted parametric profile or explicit declined outcome. A geometry tile
does not promise every road is driveable; consumers must inspect declines.

Section 1 is the unchanged roads.graph encoding-4 section-1 payload (nodes without tags, even
though roads.graph itself has been encoding 5 since street fidelity P1 slice 2: node tags are
dropped when the geometry tile is written and arrive with roads.geom encoding 6).
Section 2 stores a canonical varint entry count, then entries in ascending
way id / stretch order. Integers use little endian, ids use signed LEB128,
counts use unsigned LEB128, and floating values use IEEE-754 binary64 bits.
Both sections are Zstd encoded by the writer; standard container bounds apply.

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
`4dcfcbca39838a8922c2749a9513eae7eecdbb398dec9a81b724d2970ff445f8`.
