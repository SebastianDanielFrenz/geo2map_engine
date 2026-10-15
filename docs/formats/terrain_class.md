# terrain.class — layer `g2m.terrain.class` v1 (body encoding 7)

The surface classification pyramid that goes with `g2m.terrain.carved` (street fidelity P2 slice 10, terrain migration
phase 6, design [../street_fidelity/p2_junction_design.md](../street_fidelity/p2_junction_design.md) sections 6.5 and 9.4,
decision D49). It answers "which land class and surface is under this point" for physics grip and render colour, under the
same road ownership the carve uses, so grip and the carved shape cannot disagree. Opt-in; asking for it adds the carved
layer and both road layers. Code: `include/g2m/layer/terrain_class.h`, `src/layer/terrain_class.cpp`.

## Bytes

A `g2m.tile/1` container ([tile_container.md](tile_container.md)), body encoding 7 (`LayerEncoding::TerrainClass`),
encoding version 1, five sections, all required, no others:

| Section | Codec | Content |
|---|---|---|
| 1 meta | none | georef block (as [src_elev.md](src_elev.md)): zone, SW corner in mm, spacing 2^L · 1000 mm, 257 × 257, Node registration, SouthToNorth; then u8 length + the class scheme string `g2m.landclass.v1` |
| 2 land_class | zstd | 257 × 257 u8, `LandClass` (classifier.h), 0..13 |
| 3 surface | zstd | 257 × 257 u8, `SurfaceKind`, 0..11 |
| 4 role | zstd | 257 × 257 u8, `TerrainRole`: 0 Terrain, 1 RoadBed, 2 Sidewalk, 3 Verge, 4 JunctionRoadBed, 5 JunctionSidewalk, 6 Blend, 7 BridgeDeck |
| 5 rank | zstd | 257 × 257 u8: 0 = no road, else the owning way's rank clamped to 1..255 |

Planes are row-major, row 0 = south. Node (i, j) lies at (min_easting + i · 2^L, min_northing + j · 2^L) zone metres: both
tile edges are included, so a tile shares its edge nodes with its neighbours, and the level-0 nodes are exactly the
physics cell centres (integer metres, D11). The decoder rejects a missing or extra section, a georef other than the one
of the expected key, another scheme, a plane of the wrong size and a value above its maximum.

| Field | Value |
|---|---|
| Layer id | `g2m.terrain.class`, version 1, kind Baked |
| Levels | 0 to 6 |
| Header datum | None, height offset 0 |
| Coverage | `g2m.elev.base` |
| Config hash | SHA-256 of `g2m.terrain.class/1 nodes=257 L0 carve-ownership roads.geom enc=9 radius=192 junction=D42 underpass-deck=1 underpass-walls=steep/1 carve=<sha256 of carve_default.json> classifier=<sha256 of the ESA classifier table> scheme=g2m.landclass.v1 landcover=floor sidewalk=paving_stones; L1-L6 rank-max 3x3 centre-then-row-major window=4x4` (`terrain_class_config_text()`) |

## Inputs

- **Level 0**, tile Q: the inputs of `terrain.carved` level 0 (the 3 × 3 `g2m.elev.base` window and the `g2m.roads.geom`
  tiles within 192 m), then the `g2m.src.landcover` level-2 tiles holding any node of Q's closed square
  (`terrain_class_landcover_keys`: the ancestor, plus its east / north / north-east neighbours when Q touches the
  ancestor's east or north edge; D49). Absent land cover = Unknown. A `roads.geom` input that fails to resolve (for
  example a tile rejected by its own validation) counts as absent: no roads from it, and the output header carries the
  marker `rejected_input_marker` for it (G-1, [terrain_carved.md](terrain_carved.md)).
- **Levels 1-6**, tile P: the 4 × 4 same-layer tiles of level L-1 at (2x + dx, 2y + dy), dx, dy ∈ {-1, 0, 1, 2}. The 3 × 3
  window around an edge node reaches one node into the neighbouring children (D49). An absent tile's nodes are not
  candidates.

## Values

Level 0, per node at integer metres (x, y): `CarveModel::evaluate` decides the owner exactly as for the carved height;
`CarveModel::classify` turns it into a class:

| Owner | Role | land_class / surface |
|---|---|---|
| junction core, inside the road bed ring | JunctionRoadBed | of the used approach with the highest rank (ties: smallest way id) |
| junction core, inside the paved ring | JunctionSidewalk | PavedRoad / PavingStones |
| junction core, outside both | Verge | land cover |
| approach core, within the road bed band | RoadBed | the entry's profile attributes |
| approach core, within the paved band | Sidewalk | PavedRoad / PavingStones |
| approach core, outside the paved band | Verge | land cover |
| approach or junction blend | Blend | land cover |
| terrain or blend within a bridge's paved band ± deck margin | BridgeDeck | land cover (the ground under the deck) |
| terrain | Terrain | land cover |

Land cover is the `g2m.src.landcover` cell holding the node (floor on both axes) through the embedded ESA WorldCover
classifier table (`data/classifiers/esa_worldcover_v200.json`). Rank is 0 for nodes no road owns.

Levels 1-6: per node, the level L-1 sample of highest rank among the 3 × 3 nodes around the coincident one, ties to the
coincident centre, then the first in row-major order (south row first, west first). Roads therefore stay visible at
coarse levels (rank-max keeps 10465 road nodes on the hoechst_a66 test tile where subsampling keeps 8038).

## Determinism

Every value is a function of the world position only, so neighbouring tiles agree on their shared edge nodes (tested on
level-0 and level-2 seams and at level 1). The model is the carve's (double arithmetic in a fixed order, no libm, no FMA);
golden `testdata/golden/g3c_golden.txt` (four level-0 tiles and one level-1 tile).
