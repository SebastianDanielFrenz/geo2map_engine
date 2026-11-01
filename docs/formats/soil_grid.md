# `g2m.soil.grid` (soil facts per 32 m cell)

Layer `g2m.soil.grid`, layer version 1, container encoding id 12 (`LayerEncoding::SoilGrid`), encoding version 1,
level 2 (1024 m tiles), header datum None. Code: `include/g2m/layer/soil_tile.h`, `src/layer/soil_tile.cpp` (codec,
queries, field aggregates), `src/layer/derive_soil.cpp` (deriver), `soil_rules.h` (rules). How the facts come to be,
limits and what the layer does not say: [../pipeline/soil.md](../pipeline/soil.md). Raw input:
[src_soil.md](src_soil.md).

The layer is **facts only**: per cell and per horizon the texture (sand, silt, clay, class), organic carbon, pH, coarse
fragments and bulk density, or the reason there is no soil. What the soil means for a crop or the water balance (field
capacity, wilting point, drainage) is the game's: derive it from the texture there.

## Tile contents

32 x 32 cells of 32 m (`cells_per_side`, data in `data/soil_rules/default.json`, must divide 1024). Cell `(c, r)`
covers UTM `[min_e + c * 32, min_e + (c+1) * 32) x [min_n + r * 32, ...)` of the tile's zone; row 0 is the south row. A
cell holds the value of the source lattice cell that contains its **centre** (nearest, no interpolation), so it is a
function of its position alone.

## Body

One section (type 1, zstd). All integers little endian.

| Field | Type |
|---|---|
| tile key | u64 (`TileKey::packed`), a valid level-2 key |
| cells_per_side n | varint, 1..256, divides 1024 |
| state | n * n x u8 |
| per horizon (2, topsoil first) | texture n*n x u8, sand n*n x u16, silt, clay, soc, ph10, cfvo, bdod (each n*n x u16) |

The section is exactly `8 + varint + n*n*(1 + 2*15)` bytes; the decoder checks it before allocating.

## Records

- `state`: 0 Ok (soil facts present), 1 NoSource (the source has no value there, or the position lies outside the
  stored source window), 2 BuiltUp, 3 Water (land cover says so, `landcover.builtup` / `landcover.water` of the rules,
  ESA WorldCover classes 50 and 80, read at the cell centre). A cell that is not Ok has every horizon field 0.
- Horizons: topsoil `0-30 cm`, subsoil `30-100 cm` (rules `horizons`; a horizon is the thickness-weighted mean of the
  source depth intervals it overlaps, integer, round half up).
- `sand`, `silt`, `clay`: permille of the fine earth, **sum exactly 1000** (the means are renormalised: sand and silt
  rounded down, clay takes the remainder, at most 2 permille).
- `texture`: USDA class code, append-only: 1 sand, 2 loamy sand, 3 sandy loam, 4 loam, 5 silt loam, 6 silt, 7 sandy
  clay loam, 8 clay loam, 9 silty clay loam, 10 sandy clay, 11 silty clay, 12 clay. 0 means "none" (cells that are not
  Ok); 255 is reserved. The class comes from ordered integer clauses over `(sand, silt, clay)` in the rules file; the
  twelve classes partition the triangle exactly (checked for all 501 501 permille triples).
- `soc`: soil organic carbon, dg/kg (0.1 g/kg), at most 10 000. `ph10`: pH in water times 10, at most 140. `cfvo`:
  coarse fragments (> 2 mm), volume permille, at most 1000. `bdod`: bulk density of the fine earth, cg/cm3, at most 300.

## Validation (decode and encode)

Valid level-2 key, `cells_per_side` divides 1024 and is at most 256, state at most 3, a non-Ok cell is all zero, an Ok
cell has texture codes 1..254 and `sand + silt + clay == 1000` in both horizons and every value within the limits
above, no trailing bytes. A decoded tile always satisfies these; the encoder refuses anything else.

## Queries

`soil_at(tile, e_mm, n_mm)`: the cell containing the UTM millimetre position (half-open cells, `nullopt` outside the
tile). `TerrainService::soil_at(x, y)` / `soil_at_grid(e, n)` answer it from the resident tile.
`summarise_field_soil(field, rules, lookup)` aggregates a field polygon over a fixed absolute lattice (spacing 16 m,
points at odd multiples of 8 m, boundary inclusive; `TerrainService::field_soil(field)` wires the lookup to the
resident tiles); see the pipeline page for the fields of `SoilFieldSummary`.
