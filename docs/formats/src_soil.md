# `g2m.src.soil` (raw soil source)

Layer `g2m.src.soil`, layer version 1, container encoding id 11 (`LayerEncoding::SrcSoil`), encoding version 1, level 2
(1024 m tiles), header datum None. Code: `include/g2m/layer/src_soil.h`, `src/layer/src_soil.cpp` (codec, window),
`include/g2m/import/soil.h`, `src/import/soil.cpp` (importer). Where the numbers come from and what the layer is for:
[../pipeline/soil.md](../pipeline/soil.md).

Nothing is interpreted here. The tile holds the source's own integers for one window of a global lattice; the derived
layer [soil_grid.md](soil_grid.md) turns them into soil facts.

## The lattice

The source is a global lattice of `1/N` degree cells (`N = cells_per_degree`, 480 for SoilGrids as fetched by
`tools/soil_fetch`). Cell `(col, row)` covers longitude `[col/N, (col+1)/N)` and latitude `[row/N, (row+1)/N)`; the
lattice cell of a position is `floor(lon * N), floor(lat * N)`. A tile holds the **window** of cells that covers its
1024 m square plus one cell on every side: take the 5 x 5 points at `0, 1/4, 1/2, 3/4, 1` of the tile edges, convert
each with the deterministic UTM reverse projection (`g2m::geo::Utm::reverse`), floor, and take the min / max column
and row, minus one and plus one (`soil_source_window`). At 50 degrees north that is about 10 x 8 cells (a cell is
about 150 m east-west and 232 m north-south). The window is a pure function of the key and `N`, so two tiles' windows
overlap consistently and a derived cell never depends on which window it was read from.

## Body

Two sections. All integers little endian; `varint` = LEB128, `svarint` = zigzag LEB128.

Section 1 (meta, uncompressed):

| Field | Type |
|---|---|
| tile key | u64 (`TileKey::packed`), a valid level-2 key |
| scheme | u8 length (1..64) + bytes `[a-z0-9_.]`, e.g. `soilgrids_v2_mean` |
| cells_per_degree | varint, 1..100000 |
| col0, row0 | svarint each: lattice index of the window's west column and south row (|col0| <= 36 000 008, |row0| <= 18 000 008) |
| width, height | varint each, 1..4096 |

Section 2 (values, zstd): `42 * width * height` u16, **plane-major**: `values[plane * width * height + r * width + c]`,
row 0 is the south row, column 0 the west column. `plane = prop * 6 + depth`.

| prop | Name | Unit of the stored integer |
|---|---|---|
| 0 | sand | g/kg of the fine earth (permille) |
| 1 | silt | g/kg |
| 2 | clay | g/kg |
| 3 | soc (soil organic carbon) | dg/kg (0.1 g/kg) |
| 4 | phh2o (pH in water) | pH * 10 |
| 5 | cfvo (coarse fragments > 2 mm) | cm3/dm3 (volume permille) |
| 6 | bdod (bulk density of the fine earth) | cg/cm3 (0.01 g/cm3) |

Depth intervals (cm): 0 `0-5`, 1 `5-15`, 2 `15-30`, 3 `30-60`, 4 `60-100`, 5 `100-200`.

## Values and no data

A value is `<= 32767` (the source's own integer) or `65535` = no data (the source has no value in that cell). The
importer maps every negative source value, the file's `GDAL_NODATA` value and every value above 32767 to 65535. SoilGrids
masks cells it has no estimate for (water, built-up, ice) as 0 in the three texture planes of the surface depth; the
deriver treats a cell with sand = silt = clay = 0 at 0-5 cm as no data (`nodata.surface_texture_sum_zero` in the rules).

## Validation (decode and encode)

Valid level-2 key, scheme charset and length, `1 <= N <= 100000`, window within the size and origin limits, exactly
two sections of the expected types, no trailing bytes in the meta section, the value section is exactly
`42 * width * height * 2` bytes, no value between 32768 and 65534. A window larger than the section size limit is
refused on encode.

## Importer

`g2m_tiler import-soil` (`import::import_soil`) reads **data sets**: directories with the 42 files
`<prop>_<depth>_mean.tif` (the SoilGrids names). Every file must be a single-band 16-bit GeoTIFF (int16 or uint16),
geographic EPSG:4326, `PixelIsArea`, square pixels of exactly `1/N` degree, corner on the `1/N` lattice, and all 42
files of a set must share size, corner and sample type; anything else fails the whole call (`Unsupported`) before a
tile is written. A missing file is an `Io` error. Per tile the window is filled from the first set (in the order given)
that covers a cell; a cell no set covers stays 65535; a tile no set touches is not written. Values are copied as they
are: no resampling, no datum shift. The result does not depend on `--jobs` (`ordered_for`).
