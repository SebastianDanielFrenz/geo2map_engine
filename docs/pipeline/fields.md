# Farm field polygons: `g2m.fields.geom`

Detail page of the [pipeline overview](README.md), section 2.7. Byte format: [../formats/fields_geom.md](../formats/fields_geom.md).
Rules as data: `data/field_rules/de.json` (embedded at build time, SHA-256 in the layer's config hash).

## What it is

Per level-2 tile (1024 m): the agricultural polygons of OSM, cut into plausible fields, each with an id, area,
land use, `crop=*` tag and a row direction. The library exposes facts; the farming game decides what they mean.
Opt-in: `BuiltinTerrainDerivers::ordered(..., include_fields)`, `g2m_tiler bake --layers g2m.fields.geom`,
`g2m_server --fields`, `LayerOptIn::fields` of `TerrainService`. Reads `g2m.src.osm` only; independent of the road layers.

## Where the polygons come from (a block)

A **block** is one OSM element:

- a closed way tagged `landuse=farmland | meadow | orchard | vineyard | farmyard` (farmyard is imported for later use;
  the game decides what to do with it), or
- a `type=multipolygon` relation carrying one of those tags (outer and inner member ways, assembled with the shared
  `geom::assemble_rings`; a way that is a member of such a relation is not a second block).

The `crop=*` value (cut to 255 bytes) travels with the block. Nothing is interpreted: `crop=yes`, `crop=grape` and
`crop=field_cropland;Kartoffeln` arrive as written.

## Subdivision

OSM blocks are often whole fields-plus-tracks. A block is cut along **linear features** and around **area features**
(the lists are the `splitters` and `obstacles` tables of `de.json`):

| Cuts along (half width left free on each side) | |
|---|---|
| roads: major 7 m, minor 4 m, service 2.5 m | `highway=*` |
| field tracks 2 m, paths/footways/cycleways 1 m | `highway=track`, `path`, ... |
| railways 3.5 m | `railway=rail`, `light_rail`, `tram`, ... |
| rivers/canals 5 m, streams 1.5 m, ditches/drains 1.2 m | `waterway=*` |
| hedges 1.5 m, walls 0.5 m, tree rows 2 m | `barrier=hedge`, `wall`, `natural=tree_row` |

A line with `bridge=*`, `tunnel=*` or `layer=-1..-5` never cuts. Cut out as **areas**: water, wood, scrub, wetland,
sand, riverbank, forest, residential, industrial, commercial, retail, cemetery, quarry, buildings, parks, pitches,
parking, schools and similar (`obstacles`). Other farmland is never an obstacle.

Method (`src/layer/field_subdivide.cpp`): a raster of `raster.cell_mm` (1 m) cells on the absolute millimetre grid. A
cell belongs to the block when its centre is inside (exact scanline, even-odd over outer ring and holes). A line takes
out every cell whose centre is within its half width of the segment (exact squared-distance test), an area every cell
whose centre is inside it. The remaining cells are labelled by 4-connectivity (row-major from the south-west), each
label's boundary is traced as unit edges, merged into straight runs and simplified with Douglas-Peucker
(`simplify.tolerance_mm`, exact integer comparisons). Only integer arithmetic, no libm, no 128-bit types: the result is
the same on every platform (golden `g3f_golden.txt`, Windows = Linux).

Decisions, all thresholds in `de.json`:

- **Verbatim.** A block nothing cuts keeps its exact OSM outline (`Subdivided` flag clear). So does a block whose cuts
  leave one piece of at least `single_piece_keep_percent` (90 %) of its area (a farm pond in a field does not punch a hole).
- **Slivers.** A piece smaller than `sliver.min_area_m2` (300) or narrower than `sliver.min_width_mm` (4 m,
  measured as 2 * area / perimeter) is dropped and counted (`slivers_dropped`). When every piece is a sliver the block
  is kept whole (`SliverBlock` flag).
- **Obstacle cap.** Obstacles that remove more than `obstacle_share_cap_percent` (60 %) of a block are treated as a
  mapping overlap (forest drawn over a field, say) and ignored (`ObstaclesIgnored` flag).
- **Budgets.** A block with a bounding box above 3000 m, one needing more than 16 level-2 tiles, or one whose raster
  would exceed 16 million cells is declined and recorded in the tile, never dropped silently.
- **Plausibility flags** (never a reason to drop): `SmallArea` < 200 m2, `LargeArea` > 100 ha, `Thin` < 3 m mean width.

Accuracy: the outline of a **subdivided** piece follows the raster, so it is within about 1 to 1.5 m of the exact cut.
An uncut block is exact. The flag tells which one you have.

Piece order (the piece index of an id): per outer ring of the block (ordered by smallest vertex), the pieces in label
order: lowest raster row first, then the lowest column of the piece's first cell.

## Tile invariance

A field must be the same record in every tile it touches, whichever tile derives it. The deriver takes the block's
bounding box grown by the widest splitter plus two cells, covers it with level-2 tiles and builds the block only from
those tiles (never from "whatever the target's inputs happen to hold"). `inputs_for` names the target's own `g2m.src.osm`
tile; `additional_inputs_for` (four discovery stages) names the cover tiles of every block of the target and, for a
relation, the tile of each open chain end, so a relation outline that crosses tiles is walked tile by tile. Limits: a
relation needing more than about four discovery rounds, or an obstacle relation not complete inside the block's cover,
is incomplete (`RelationIncomplete` decline or the obstacle is skipped); a block that reaches past the imported region
is `RelationIncomplete` too. A field that covers a whole tile with no outline segment near it is not seen from that
tile (the tile only holds fields whose outline or south-west corner lies in it).

## Ids and what breaks them

`id` = source (2 bits), OSM id (46 bits), piece (16 bits). The same OSM element with the same cuts gives the same id
after any re-bake: that is what saves refer to. A save stores the id **and** its own polygon copy and anchor
(`FieldRef`), so an id that stops matching costs a lookup, never the field.

What changes an id (be honest about it):

- OSM way id changes: a mapper redraws, merges or splits the way, converts a way into a multipolygon relation (or
  back), or the way is deleted and re-created. The id is gone; the **anchor** still finds the new field (`Moved`).
- The piece index shifts: a new track or ditch is mapped through the block (pieces renumber), a cut disappears, or a
  piece falls under the sliver rule or comes back above it. The block id stays, the piece index may not (`SameBlock`).
- The rules or raster parameters change (`de.json`, cell size, tolerance, state of the algorithm): pieces and their
  order change. `config_hash` follows the rules SHA, so this is a new release id anyway.
- Nodes of the outline move: the id stays, the polygon changes slightly; the anchor may fall outside the new polygon
  (then `SameBlock` by nearest anchor within `match.max_anchor_distance_m`, 150 m).
- A block turns from declined to produced (or back) when the region or the discovery limits change.

`match_field(saved, candidates, rules)`: `SameId` (id equal and the saved anchor still inside), `SameBlock` (same OSM
element, found by anchor or the nearest anchor within 150 m), `Moved` (another element of the same land use now covers
the saved anchor), `None`. Deterministic, ties to the lowest id. The caller passes the fields of the tiles around the
saved anchor.

## Publishing

Layer `g2m.fields.geom`, served and baked like every other layer ([../formats/fields_geom.md](../formats/fields_geom.md)).
`TerrainService`: `fields(l2, priority)` (typed ticket, a finer key maps to its level-2 ancestor), `peek_fields`,
`field_at(x, y)` (session-local metres) and `field_at_grid(e, n)` answered from the resident tile without starting
work. A point in a cut gap (on a track, in a ditch) is in no field; fields of different blocks may overlap where OSM draws them over each other (the lowest id wins in `field_at`).

## Parcel data seam (not used yet)

Source kind 2 (`Parcel`) is reserved in the id and in the tile; tiles that carry it decode. The intended design: a raw
layer `g2m.src.parcels` (polygons with the authority's id, e.g. FLIK, and attributes) imported per tile like
`g2m.src.osm`; the deriver then takes parcels where they exist and OSM blocks elsewhere, with the same subdivision
kernel only when a parcel is itself a block (reference parcels are, "Referenzparzellen"). Nothing reads or requires it.

Findings on open parcel data (2026-10-31, web search and a read of third-party conversions; nothing was downloaded,
original metadata pages were not all reachable, so licence texts must be confirmed at the source before use):

- **Hessen:** the reference parcels ("Referenzparzellen" of the InVeKoS/LPIS system, id FLIK) are published by Land
  Hessen at `geoportal.hessen.de/spatial-objects/886` (WFS). A conversion on source.coop (`ftw/harmonized-field-data`,
  collection `de_he`) lists them as GeoParquet for 2023, 2024 and 2025 (about 618 000 features in 2025, EPSG:25832),
  licence **CC BY 4.0**, attribution "(c) Land Hessen, CC BY 4.0". These are blocks (reference parcels may hold several
  declared parcels), not per-crop fields.
- Other states per the same listing: Schleswig-Holstein Feldblöcke DL-DE-Zero 2.0 (the state download service CC BY
  4.0), Nordrhein-Westfalen DL-DE-BY 2.0, Niedersachsen/Bremen/Hamburg DL-DE-BY 2.0 (blocks and crop fields), Bayern
  CC BY 4.0 (blocks and crop fields), Brandenburg/Berlin, Sachsen, Sachsen-Anhalt, Thüringen DL-DE-BY 2.0, Saarland
  CC BY 4.0, Baden-Württemberg crop fields DL-DE-BY 2.0, Mecklenburg-Vorpommern "other" (not specified), Austria AMA
  INVEKOS CC BY-AT 4.0.
- All of these need an attribution line in the credits, like OSM and WorldCover (the R5 credits screen is not done).
  No account or purchase was needed for any of them as far as the listings show.
- Why it is only a seam: reference parcels follow the subsidy system, not what a farming game wants as a field
  (several fields per parcel, parcels over tracks), the data is per state, and an OSM-based path works everywhere.

## Real data (Rhein-Main-Hessen, scope `rhein-main-hessen-2026-10r1`)

Measured 2026-10-31 with `g2m_tiler bake --layers g2m.fields.geom --levels 2 --jobs 6` (68 s, 307 MiB peak RSS, 5.1 MB
of tiles for 2173 tiles) and `g2m_fields_probe stats` (`tools/fields_probe`, counts owned records only):

| | |
|---|---|
| tiles / with fields | 2173 / 1852 |
| blocks / fields | 17 451 / 26 432 (farmland 12 478, meadow 9 922, orchard 2 284, vineyard 322, farmyard 1 426) |
| blocks cut / kept whole | 3 662 / 13 789; 12 659 fields (48 %) come from a cut |
| area | 564 km2 in total; median 9 280 m2, p25 3 356, p75 24 896, p95 82 305, p99 154 241, max 213 ha |
| implausible (small, large or thin) | 140 = 0.53 % (132 below 200 m2, 3 above 100 ha, 43 thin; overlaps counted once) |
| slivers dropped | 77 313 pieces; 163 blocks kept whole because every piece was a sliver; 325 blocks had obstacles ignored by the cap |
| declined blocks | 384: relation incomplete 356 (blocks reaching past the imported region), invalid geometry 15, too large 13 |
| pieces per block | 1: 14 753; 2: 1 368; 3-5: 869; 6-10: 270; 11-50: 194; > 50: 6 |
| outline vertices | mean 38.6, max 1 435; 1 879 fields have holes |
| `crop=*` | 384 fields (1.5 %): grape 309, grass 40, field_cropland 12, market_gardening 7 |

Picture: `out/fields_real/overview.png` (3 x 3 tiles, 2 m per pixel) and `out/fields_real/detail.png` (one tile, 0.7 m per
pixel) in the worktree of this branch: fields filled by land use, the OSM lines the cut followed drawn on top
(`g2m_fields_probe png`).

## Tests

`tests/layer/test_fields_geom.cpp` (tags `[fields]`, `[fields_kernel]`, `[fields_derive]`, `[fields_match]`,
`[fields_bake]`, `[fields_fixture_check]`): rules loader and rejections, id packing, geometry helpers, codec round trip
and every validation rule, subdivision kernel cases, deriver on synthetic OSM worlds (every splitter and obstacle kind,
bridges, relations with holes, tile borders, relations needing discovery, regions edge declines, input order),
`match_field` after changes, server and parallel bake byte-identical, the committed OSM fixtures (every tile validates,
a field is the same record in every tile it touches and is owned once); `tests/world/test_terrain_service.cpp` (ticket,
peek, `field_at`); golden `g3f_golden.txt`; fuzz target `fieldsGeom`.

## Not done

- No consumer was changed (`racing_game`, the physics bridge).
- No parcel source (seam only), no crop interpretation, no per-state rules file other than `de.json`.
- Obstacle relations are cut out only when complete inside the block's cover; very large relation outlines are declined.
- The 77 313 dropped slivers are counted, not published.
