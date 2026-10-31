# rail.geom — layer `g2m.rail.geom` v1 (body encoding 1)

The railways of the map as a served layer (railways R1/R2, decision G2M-026): one centreline per piece of an OSM
railway way, with a height profile taken from the rail line itself, never from the bare-earth DEM under a bridge or
tunnel, plus class, structure flags and track attributes. Opt-in: no existing release contains it, so no existing release
id changes. Code: `include/g2m/layer/rail_geom_tile.h`, `src/layer/rail_geom_tile.cpp` (codec, validator, descriptor),
`rail_geom_build.cpp` (attributes, pieces, chains, profile), `derive_rail_geom.cpp` (deriver); rules
`data/rail_rules/de.json` via `rail_rules.cpp`. Tool: `g2m_rail_census`.

## Bytes

A `g2m.tile/1` container ([tile_container.md](tile_container.md)), body encoding 1 in this layer's id space
(`LayerEncoding::RailGeom`), encoding version 1, six sections, all required, in this order, no others. A tile with no
piece has the six sections with empty columns (n = 0).

| Section | Codec | Content |
|---|---|---|
| 1 meta | none | u32 piece count n, u32 total point count |
| 2 ids | zstd | per piece: way id (the first as svarint, then the varint difference to the previous way id, so ids ascend), varint piece index |
| 3 attrs | zstd | 10 bytes per piece: u8 class, u8 condition, u16 flags, i8 layer, u8 tracks, u8 service, u8 usage, u16 gauge in mm |
| 4 counts | zstd | per piece: varint point count (2 to 4096) |
| 5 xy | zstd | per point: svarint dx, svarint dy in integer millimetres of the key's UTM zone, each against the previous point of the piece (the first against 0) |
| 6 z | zstd | per point: svarint dz in 1/256 m against the previous point of the piece (the first against 0) |

Pieces are strictly ascending by (way id, piece index). A decoder rejects: another encoding or version, a section that
is missing, extra or of an implausible size (`n * 10` bytes for attrs; `n` bytes at least for ids and counts), trailing
bytes in any column, a point total that is not the sum of the counts, a delta or value outside the ranges below, and any
violation of the validator (`validate_rail_geom_tile`, also run before every encode):

- class 0..3 (0 rail, 1 light_rail, 2 subway, 3 tram; append-only), condition 0 in service / 1 under construction;
- flags (u16): 1 bridge, 2 tunnel, 4 embankment, 8 cutting, 16 declined, 32 electrified; other bits rejected; bridge and
  tunnel never together (a way tagged both is a tunnel);
- layer -5..5, tracks 1..16, service 0 main / 1 siding / 2 yard / 3 spur / 4 crossover, usage 0 none / 1 main / 2 branch /
  3 industrial / 4 military / 5 tourism / 6 test, gauge 0..3000 mm;
- 2 to 4096 points per piece, |x|, |y| < 2^40 mm, |z| < 2^27 (1/256 m); a declined piece has all z = 0;
- **ownership**: the first point of a piece lies in the tile (floor division on the integer millimetres by 1 024 000, the
  `roads.geom` junction owner rule).

Limits: 200 000 pieces and 4 000 000 points per tile.

| Field | Value |
|---|---|
| Layer id | `g2m.rail.geom`, version 1, kind Baked |
| Levels | 2 only |
| Header datum | the datum of the `g2m.elev.base` inputs (z is in that datum); None for a tile without pieces |
| Coverage | `g2m.src.osm` |
| Config hash | SHA-256 of `g2m.rail.geom/1 enc=1 owner=first-vertex-mm piece=cut-at-first-vertex-ge-max-length profile=ground:smoothed-triangular+grade-limit-pinned-ends structure:chain-linear-dem-anchors window=src.osm-3x3 portal-anchor=7a1 rules=de:<sha256 of de.json>` (`rail_geom_config_text()`) |

## Inputs

The tile is a function of the 3 x 3 `g2m.src.osm` window around it and of `g2m.elev.base`, nothing else (stage 1 names the
nine OSM tiles; stage 2, `additional_inputs_for`, names the DEM tiles). OSM tiles carry complete ways with a 128 m halo, so
the same way appears in neighbouring tiles; equal copies merge, conflicting ones are an error. A piece reaches at most
`piece.max_length_m` plus one segment past the tile that owns it, a chain walk at most `chain_walk_budget_m` further; both
stay inside the 3 x 3 window for every way shorter than a tile (a longer bridge is walked as far as the window holds).

DEM tiles (`g2m.elev.base`, level 0, 256 m) listed in stage 2: every tile within `dependency_margin_m` (4 m) of a segment
of an owned piece, and of both chain anchors of every owned bridge or tunnel piece. More than 1024 tiles is
`LimitExceeded`. A DEM tile that is absent (outside the elevation coverage) or has no data under a needed sample does not
fail the tile: the piece is **declined** (flag 16, all z = 0) and the consumer decides what to draw.

## Pieces

A way is first cleaned (consecutive duplicate refs and coincident points dropped; fewer than two points = not served),
then cut deterministically from its complete node list: a piece starts at a vertex and ends at the first vertex whose
arc distance from the piece start is at least `piece.max_length_m` (512 m), or at the way's last vertex. The next piece
starts at that vertex (consecutive pieces share their cut vertex). `piece` counts from 0 per way. The piece belongs to
the tile that contains its **first** vertex. Pieces therefore never duplicate across tiles, and a consumer that draws
them needs the ring of tiles that covers one piece length around its view.

## Height profile

z comes from the line itself.

- **Bridge and tunnel pieces** (flag 1 or 2): the *chain* of the way is walked from both ends through connected ways: a
  node where exactly one other served way touches, that way has the same structure flag, and it ends there (a way that
  merely passes through is a junction, the walk stops), up to `structure.chain_walk_budget_m` (3000 m) and
  `chain_max_ways` (16) ways. The chain's two end nodes are the **ground anchors**; their heights are the DEM there. z is
  linear in arc length between the anchors (rounded half up to 1/256 m), so a bridge split into several OSM ways has one
  profile, and the deck does not follow the DEM into a cutting or a valley under it. A tunnel is treated the same way
  (the line between the portals); it never follows the hill above.
- **Ground pieces**: resampled to at most `piece.ground_spacing_m` (20 m, widened by doubling if the piece would exceed
  `piece.max_points`), the raw DEM at every vertex, then triangular smoothing of half width `piece.smoothing_m` (40 m, faded
  to the raw value towards both ends), then a forward and a backward grade limit pass at the class's `max_grade_permille`
  (rail 40, light_rail and subway 60, tram 80). The first and last values stay the raw DEM, so consecutive pieces and
  the ground next to a bridge abutment meet exactly. If the two ends are further apart than the limit allows, the limit
  gives way (the ends win). Arithmetic is `+ - * / sqrt floor` on doubles only.

Embankment and cutting are flags (OSM `embankment=yes`, `cutting=yes`), not geometry: v1 does not model the earthwork.

## Attributes

From the OSM tags of the way (`resolve_rail_attrs`): `railway` in {rail, light_rail, subway, tram} (tram is off in the
shipped rules); `railway=construction` with `construction=<class>` is served as condition 1 when
`construction.enabled`; `area=yes` is dropped; `bridge` other than `structure.bridge_not` is a bridge, `tunnel` other than
`structure.tunnel_not` (`no`, `culvert`) a tunnel; `layer` is clamped to -5..5; `tracks` 1..8 and `gauge` 300..3000 are used
when parsable (a leading integer; `1435;1000` reads as 1435) else the class defaults; `service` and `usage` map the
known values (others read as main / none); `electrified` other than `no` sets flag 32.

## Decisions (G2M-026)

- **A new layer, not a new roads.geom encoding.** `roads.geom` encoding 9 is shared by `terrain.carved` and
  `terrain.class`, which hash it into their config text; a rail section there would have moved every carved and class
  tile for nothing. A separate opt-in layer leaves every existing release id and cache entry as it is, and a consumer
  that does not draw railways pays nothing. Cost: rails are not part of the road graph, so a rail-road crossing is found by
  the consumer from two tiles (rail pieces plus `roads.geom`), and the carve does not see rail bridges.
- **Level 2 (1024 m).** The inputs are level-2 `src.osm` tiles; `roads.geom` is level 2, and the consumer already
  streams road tiles at that size, so the two share a ring. Level 0 (256 m) would cut every rail piece (512 m) across
  two to three tiles; level 1 has no source layer. Measured on the Rhein-Main region (2173 tiles with OSM coverage, the
  whole bake 29 s on 8 jobs, 321 MiB peak): 1.9 MB of containers in total, 875 of 1220 tiles of the census box empty,
  the largest tile 18.5 KB.
- **Pieces and ownership by first vertex.** The same rule as `roads.geom` junctions; it needs no cross-tile coordination and
  makes a tile independent of the other tiles' DEM. A piece length of 512 m bounds how far a tile reaches past its edge.
- **Encoding versioning.** Encoding 1 in this layer's own id space (ids are per layer, `layer_id.h`); additions (more
  columns, tunnel portal records, embankment geometry) get encoding 2 and a decoder rule "1 still decodes", as the road
  layers do. The rules file's SHA-256 is in the config hash, so editing `de.json` changes the release id.
- **Tram is off in v1** (`classes[3].enabled = false`; class id 3 is reserved and the census still counts it). Tram tracks
  lie in the street surface (on the road ribbon, no ballast, no separate profile), a tram stop shares the carriageway, and
  the road layers do not know which lanes are rails: drawing a bed beside or under the road would add z-fighting and a
  wrong crossing count for 575 Rhein-Main ways (154 km) that are mostly inside streets. A switch in the rules enables
  them when the road side can place them.
- **Subway and light rail are in.** Their bridges and tunnels are real structures with their own profile
  (subway: 198 ways, 157 of them tunnels).

## Tests, tools

`tests/layer/test_rail_geom.cpp` (tag `[rail]`): attribute matrix, rules loader, codec/validator/decoder corruption,
bridge profile vs the DEM cutting, chain across split ways, ground profile (pins, grade limit), ownership and tile
invariance, DEM dependencies with the anchor tiles, determinism; every property has a recorded sabotage printed by the test.
`tests/world/test_terrain_service.cpp` (`service: g2m.rail.geom ...`): opt-in, release id, end-to-end through the service,
1 vs 4 workers. Fuzz target `railGeom` (`tests/fuzz_formats`), golden `testdata/golden/g3rail_golden.txt` (the fixture
tiles, compared Windows = Linux).

`g2m_rail_census STORE SCOPE ZONE EMIN EMAX NMIN NMAX [CSV] [--spot E N] [--baked DERIVED RID]`: per class ways, km,
bridges, tunnels, embankments, cuttings, construction ways; bridge structures; pieces; plan crossings with roads by
structure; the rail bridges over roads (CSV); the served tiles of a bake. Read-only.
Home-region numbers (E 455-490 km, N 5540-5585 km, 1220 `src.osm` tiles): [../pipeline/rail.md](../pipeline/rail.md).

## Not in v1

Embankment and cutting geometry, carving of the terrain under or beside a rail bridge, level crossings as records,
rail topology (`rail.graph`: switches, signals, stations), overhead line masts and wires, trains, tram, rail-specific
collision. Bridges over roads are recognised by the consumer from the flags and the layer tag.
