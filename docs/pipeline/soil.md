# Soil: `g2m.src.soil` and `g2m.soil.grid`

Detail page of the [pipeline overview](README.md), section 2.8. Byte formats: [../formats/src_soil.md](../formats/src_soil.md)
(raw) and [../formats/soil_grid.md](../formats/soil_grid.md) (derived). Rules as data: `data/soil_rules/default.json`
(embedded at build time, SHA-256 in the layer's config hash). Source fetch script: `tools/soil_fetch/fetch_soilgrids.py`.

## What it is

Per level-2 tile (1024 m), a 32 x 32 grid of 32 m cells. Each cell carries soil **facts** for two horizons (topsoil 0-30
cm, subsoil 30-100 cm): sand, silt and clay (permille, sum 1000), the USDA texture class (12 classes), soil organic
carbon, pH, coarse fragments and bulk density, or the reason there is no soil (no source value, built-up, water). The
library does **not** say what the soil is good for. Field capacity, wilting point, drainage, root zone depth and crop
suitability are derived by the game from the texture (pedotransfer functions are the game's choice).
Opt-in, like the fields: `BuiltinTerrainDerivers::ordered(..., include_soil)`, `g2m_tiler bake --layers g2m.soil.grid`,
`g2m_server --soil`, `LayerOptIn::soil` of `TerrainService`. Reads `g2m.src.soil` (required) and `g2m.src.landcover`
(optional); independent of the road, terrain and field layers.

## The source decision (measured, 2026-10-31)

Question: which source for a global farming game that starts in Rhein-Main-Hessen, with room for a better regional
source later?

| Candidate | Result |
|---|---|
| **ISRIC SoilGrids 2.0, 250 m** (chosen as the global raster source) | Global, anonymous download through the ISRIC WCS (no account), CC BY 4.0. Mean predictions of sand, silt, clay, soil organic carbon, pH, coarse fragments and bulk density at six depths down to 2 m. Native grid Interrupted Goode Homolosine 250 m; the WCS resamples (nearest neighbour) to a geographic grid. |
| HLNUG BFD50 (Hessen soil survey 1:50 000) | Anonymous ArcGIS REST service (`geodienste-umwelt.hessen.de/arcgis/rest/services/boden/bfd50`), polygons with **qualitative** attributes (soil main group, group, soil unit, substrate text such as "aus 6 bis 15 dm Loess ueber ..."), no numeric texture. Much better *where* things change, but it needs a substrate-to-texture table (a classification choice) and exists for Hessen only. Used here as the **reference to judge SoilGrids**, not imported. Licence not confirmed at the source in this pass (Hessian open data, believed CC BY; confirm before use). |
| BGR BUEK200 / BUEK1000 (soil map 1:200 000 / 1:1 000 000) | Not downloaded and not measured in this pass. The soil-map products come per sheet or as a national set with their own terms; the licence was not confirmed at the source. Treat as an unverified candidate for a Germany-wide regional source. |

**Measurement.** 400 random cells with soil of the fetched SoilGrids grid were compared with the BFD50 group at the
cell centre (304 of the 400 lie inside Hessen). Mean topsoil (0-5 cm) texture in g/kg by BFD50 group:

| BFD50 group | cells | sand | silt | clay |
|---|---|---|---|---|
| 5.3 soils from loess | 50 | 186 | 546 | 268 |
| 6.3 / 6.4 solifluction layers with loess (rich, medium) | 69 / 38 | 219 / 206 | 513 / 527 | 269 / 267 |
| 2.1 alluvial sediments | 31 | 257 | 431 | 312 |
| 2.2 flood-plain sediments | 30 | 300 | 384 | 316 |
| 5.1 soils from aeolian sand (Flugsand) | 21 | 271 | 445 | 283 |

So SoilGrids **does** separate loess (silt about 550, sand under 200) from valley-floor sediments (silt 380-430, clay
above 310). It does **not** separate aeolian sand well: the "Flugsand" cells average only 27 % sand, 85 g/kg more than
loess, and no cell of the whole region becomes `sand`, `loamy sand` or `sandy loam` (below). The model compresses the
contrast: a 250 m machine-learning estimate pulls everything toward the regional mean. Resolution in practice: the
fetched lattice is 1/480 degree, 232 m north-south by 149 m east-west at 50 degrees; at 2 m per pixel the lattice
blocks are plainly visible ([the pictures](#real-data-results)). Variation inside a field exists only where the field
crosses a block border (81 % of the fields are one topsoil class).

**Decision.** SoilGrids is the global layer (the only global, openly downloadable numeric source). The
raw layer is a lattice with a *scheme* name (`soilgrids_v2_mean`) and the deriver refuses a source of another scheme or
lattice, so a regional source is added as a second raw scheme with its own rules file (for BFD50: polygons rasterised
to a 42-plane lattice, or to a classification the rules map to sand/silt/clay), merged by the importer's "first data
set that covers a cell wins" rule, without touching the deriver's callers. That seam is designed and tested (order,
partial coverage, scheme mismatch), not used.

**Root depth.** "Usable root depth" is not in SoilGrids 2.0 as a map. The depth-to-bedrock layer (BDRICM) was fetched
for the area and is the constant 175-200 cm everywhere (the model's upper limit, no information), so it is **not**
exposed. A usable depth would come from BFD50 text ("aus 3 bis 6 dm Fliesserde ueber Fliessschutt") or BUEK, i.e. from
the regional source above. Stoniness is `cfvo` (coarse fragments by volume).

## Licence and attribution

SoilGrids 2.0 is published by ISRIC - World Soil Information under **CC BY 4.0** (ISRIC's SoilGrids README, which also
says "publicly available under the CC-BY 4.0 License"; the WCS capabilities state `Fees: None`, `AccessConstraints:
None`). Attribution required. Suggested text:

> Soil data: SoilGrids 2.0, ISRIC - World Soil Information, CC BY 4.0 (Poggio et al. 2021, SOIL 7, 217-240,
> doi:10.5194/soil-7-217-2021).

The credit goes into the game's credits with the other sources (OSM, WorldCover, DGM1). The texture classes follow the
USDA soil texture triangle (Soil Survey Manual); the clause table is in `data/soil_rules/default.json`.

## The raw layer

`tools/soil_fetch/fetch_soilgrids.py --bbox LON0 LAT0 LON1 LAT1 --out DIR` (Python standard library only, anonymous)
fetches the 42 files `<prop>_<depth>_mean.tif` from the ISRIC WCS for a bounding box on the 1/480 degree lattice and
writes `manifest.json` (URLs, SHA-256, size, date). Rhein-Main-Hessen was fetched to
`S:\geo_data\soil\soilgrids_rmh` (bbox 8.2 49.45 8.925 50.5, 348 x 504 cells per file, 5.1 MB in total, fetched
2026-10-30). `g2m_tiler import-soil --resolved R --store S --scope X --data DIR [--data DIR2 ...] --jobs N --pin`
writes `g2m.src.soil` for every level-2 tile of the resolved region whose window a set covers
([src_soil.md](../formats/src_soil.md), importer rules there). `import` (the one-call import) does not include soil: the
layer is opt-in and its sets are separate files.

## The derived layer

For each cell centre (32 m cells, 32 x 32 per tile) the deriver
1. reads the **land cover** class at the centre (ESA WorldCover, 4 m cells): class 80 -> `Water`, 50 -> `BuiltUp`
   (classes in the rules file; no land cover tile = no masking);
2. converts the centre to longitude and latitude with the deterministic UTM reverse projection and takes the source
   lattice cell `floor(lon * 480), floor(lat * 480)` (nearest, no interpolation);
3. a missing value (65535) in any plane the horizons use, an all-zero surface texture, or a position outside the stored
   window -> `NoSource`;
4. each horizon is the thickness-weighted integer mean of the depth intervals it overlaps (topsoil 0-30 cm = 5, 10, 15
   cm weights; subsoil 30-100 cm = 30, 40), round half up;
5. sand, silt and clay are renormalised to sum exactly 1000 (sand and silt rounded down, clay the remainder) and
   classified by the ordered clauses of the rules (first match wins; they partition the triangle: checked for every
   permille triple).

No libm and no float in the decisions after the projection and one floor; the cell is a function of its position
alone, so the result is the same whichever tile derives it and does not depend on the raw tile's window (tested with
windows of several widths). The config hash text is `g2m.soil.grid/1 enc=12 impl=1 rules=<SHA-256 of default.json>
cell=centre-nearest lattice=floor(lon*N),floor(lat*N) horizon=thickness-weighted-mean-half-up
texture=renormalise-floor-clay-remainder nodata=any-65535|surface-texture-zero landcover=centre-cell-builtup-water-override
view=same-key`; editing the rules file changes it and so the release id. Coverage follows `g2m.src.soil`.

## Per-field aggregates

`summarise_field_soil(field, rules, lookup)` (`TerrainService::field_soil(field)`): samples the polygon on a fixed
**absolute lattice** (16 m spacing, points at odd multiples of 8 m of the UTM millimetre grid, boundary inclusive),
reads the soil cell at each point through the lookup and returns a `SoilFieldSummary`: `samples`, `valid` (cells with
soil), `missing` (lookup could not answer: a tile not resident; the aggregate is incomplete if > 0), `state_count[4]`,
`anchor_fallback` (a field smaller than the lattice samples its anchor), and per horizon `dominant_texture`,
`dominant_share_permille`, `mean_texture` (class of the mean triple), `texture_mask` (bit per class present),
`sand`/`silt`/`clay`/`soc`/`ph10`/`cfvo`/`bdod` (means over the valid samples, round half up) and the spread
`sand_min`/`sand_max`/`clay_min`/`clay_max`. The lattice is independent of any tile, so a field has the same summary
whichever tiles serve the lookup (tested against an independent enumeration and across tile seams).

## How a consumer reads soil

```cpp
TerrainServiceConfig config = ...;
config.layers.fields = true; config.layers.soil = true;           // opt in; the release needs g2m.src.soil
auto service = *TerrainService::open(config);
auto tile = service->soil(key, Priority::Render);                    // Ticket<SoilGridTile>; Absent where the source has no tile
...                                                                  // wait_for / state() as for every ticket
auto here = service->soil_at_grid(easting_m, northing_m);           // std::optional<SoilSample>, from the resident tile
if (here && here->state == SoilState::Ok) {
    const SoilHorizon& top = here->horizon[0];                      // texture (USDA 1..12), sand/silt/clay permille, soc, ph10, cfvo, bdod
}
SoilFieldSummary s = service->field_soil(field);                    // field = a FieldRecord of fields(); request the soil tiles first
```

Baked store: the tiles are ordinary `g2m.soil.grid` containers (level 2); read them with `decode_body` +
`decode_soil_grid` and `soil_at`, or run a `g2m_server --soil`. Store the derived numbers you need in the game save
(the source of truth is the bake: a new source or rules file is a new release).

## Limits

- 250 m source resolution, resampled to 1/480 degree blocks (about 232 x 149 m): soil changes at block borders, not at
  field borders. Two fields next to each other usually get the same record.
- SoilGrids compresses contrast. No cell of the Rhein-Main-Hessen region is sand, loamy sand or sandy loam; the Hessian
  Ried (Rhine plain) comes out as clay loam / loam with more sand (334 g/kg) than the loess areas (143-172), which is
  the right direction but far from the BFD50 aeolian sands. A regional source is the fix.
- Cells under built-up land cover or water are masked by the land cover, not by the soil model (the soil model has its
  own mask, `NoSource`, which is wider than WorldCover's built-up class at the town fringes).
- Fields at the edge of the baked region can touch tiles that were not baked (`missing` > 0, 340 of 26 432 fields in
  the Rhein-Main-Hessen bake).
- Only the mean prediction is used; SoilGrids' 90 % prediction intervals are not carried.
- No root depth (see above); no organic soils, drainage class or groundwater.

## Real-data results

Source store `out/soil_real/store` (scope `rhein-main-hessen-2026-10r1`, soil + land cover, 4346 tiles), baked store
`out/soil_real/derived` (release `c78dd4a86ffb37fda2e8e69dc573136737fe3e26efea89db998263072db6f1ec`, 2173 tiles, 3.4
MB; import 1.9 s, bake 0.77 s). Read with `g2m_soil_probe` (below). Cells: 2 225 152 (2279 km2).

| State | Cells | Share |
|---|---|---|
| Ok | 1 684 160 | 75.69 % |
| NoSource | 208 631 | 9.38 % |
| BuiltUp | 316 987 | 14.25 % |
| Water | 15 374 | 0.69 % |

Texture class share of the Ok cells (topsoil / subsoil): loam 7.8 / 6.5 %, silt loam 21.6 / 8.7 %, sandy clay loam 0.2 /
0.4 %, clay loam 45.2 / 52.3 %, silty clay loam 25.2 / 32.2 %, clay 0 / 0.01 %. Mean topsoil: sand 230, silt 486, clay
283 g/kg, SOC 27.4 g/kg, pH 6.17, coarse fragments 126 permille, bulk density 1.32 g/cm3; subsoil: 234 / 471 / 295,
SOC 6.7 g/kg, pH 6.34, 169 permille, 1.54 g/cm3.

Fields (26 432 owned fields of the fields bake, 2.2 million lattice samples, mean 83.5 per field): 24 635 have soil, 1797
(6.8 %) have no valid sample (built-up or masked cells), 155 are small enough to use the anchor fallback. One topsoil
class only: 81.2 % of the fields; the topsoil and subsoil dominant class differ in 26.5 %. Classes per field: p50 1, p90 2,
max 3. Sand range inside a field (permille): p50 5, p90 22, p99 51, max 114; clay range: p50 6, p90 26, p99 51, max 88.
Dominant topsoil class over the fields: silty clay loam 10 010, clay loam 8299, silt loam 5370, loam 889, sandy clay loam 67.

Plausibility (topsoil means over field anchors): loess landscapes are silty - western Wetterau edge 2987 fields, sand
165 / silt 546 / clay 290 (82 % silty clay loam); Main-Taunus foreland 3231 fields, 143 / 577 / 280 (silty clay loam 73 %,
silt loam 27 %); north of Frankfurt 1835 fields, 172 / 542 / 286. The Rhine plain is sandier and less silty: Hessian Ried
2247 fields, 334 / 377 / 289 (clay loam 81 %, loam 15 %, sandy clay loam 3 %); Gross-Gerau to Darmstadt 903 fields, 291 /
390 / 319. Direction right (silt 546-577 on loess, 377-390 on the plain), contrast weaker than the survey maps (see the
Limits).

Pictures (rendered by `g2m_soil_probe png`, topsoil class strongly inside the farm fields, faded outside, field
outlines in dark; legend printed by the tool: sand yellow, loamy sand orange, sandy loam dark orange, loam brown, silt loam
light blue, silt cyan, sandy clay loam orange-red, clay loam dark red, silty clay loam blue, sandy clay pink, silty clay
violet, clay dark violet, water light blue, built-up grey, no source light grey):
`out/soil_real/overview.png` (40 x 40 tiles, 40 m per pixel: loess blue north-west, the Ried in clay loam red to the
south) and `out/soil_real/detail.png` (3 x 3 tiles, 2 m per pixel: the lattice blocks and the fields). They are in the
ignored `out/` folder, not in git.

```powershell
g2m_soil_probe stats SOIL_DIR SOIL_RELEASE FIELDS_DIR FIELDS_RELEASE [fields_soil.csv]
g2m_soil_probe png SOIL_DIR SOIL_RELEASE FIELDS_DIR FIELDS_RELEASE out.png [--tile X,Y] [--tiles N] [--mpp M] [--horizon 0|1] [--lonlat LON,LAT]
```

## Tests

`tests/layer/test_soil.cpp` ([soil]: rules and rejections, partition of the triangle, raw and derived codecs, window,
the deriver against a direct evaluation of the synthetic world in `tests/support/soil_world.h` for 3 x 3 tiles, window
independence, no-data and missing inputs, field aggregates against an independent enumeration, the importer on
synthetic GeoTIFFs with 1 vs 4 jobs, server vs parallel bake), `tests/world/test_terrain_service.cpp` (soil case),
golden `g3s_golden.txt`, fuzz targets `srcSoil` and `soilGrid`. Recorded sabotages: [../../CLAUDE.md](../../CLAUDE.md).
