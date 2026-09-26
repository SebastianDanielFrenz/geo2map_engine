# src.elev — body encoding 1 (version 1)

Heights of the raw elevation layers `g2m.src.elev.<source>` (`dgm1`, later
`glo30`, ...). Level-2 tiles (1024 m). The source grid is kept as it is (its
georef travels in the body). DGM1 is cell-registered at 1 m, which makes
1024 × 1024 samples per tile.

Values are `int32` heights in 1/256 m. NoData is `INT32_MIN`, and it is
preserved exactly: the round trip is bit-exact for every value, NoData
included.

### Quantisation (DGM1)

A source height in metres (double) is scaled by `kHeightUnitsPerMetre`
(256, an exact power of two, so the scaling itself introduces no rounding
error) and rounded to the nearest integer using **round-half-to-even**
("banker's rounding": `g2m::quantise_height_half_even`,
`src/core/raster.cpp`) rather than round-half-away-from-zero - a source
value that lands exactly on a `1/512 m` half-unit boundary rounds to
whichever neighbouring `1/256 m` unit is even, not always up (or always
away from zero). This matters for DGM1 specifically because its published
values are themselves already rounded to a fixed number of decimal places,
so exact-half ties are common, not a rare double-rounding edge case; using
a biased rule instead would systematically shift every tied value the same
direction. `INT32_MIN`/`INT32_MAX` (with a 0.5-unit margin, since
`INT32_MIN` is reserved for NoData) still bound the result exactly as they
do for the plain `quantise_height` used by other sources.

### Duplicate DGM1 tile copies

A single 1 km × 1 km DGM1 tile key can appear more than once across a
DGM1 delivery's outer/inner ZIP structure (`g2m::decode::dgm1::Catalog`,
one committed real example: `testdata/fixtures/dgm1/PKT_fixture.zip`'s tile
474/5560, delivered by two different municipalities' zips). `import_dem`
decodes **every** copy the catalog reports for that key
(`Catalog::copies(e_km, n_km)`) and compares them sample-for-sample -
never just decodes the first one found and assumes the rest agree. Copies
are otherwise expected to be bit-exact duplicates (same real-world terrain
surveyed once, redistributed in multiple deliveries); if any pair of
decoded copies differs by even one sample, that key is reported in
`DemImportReport::differing`, but import does not fail - it proceeds using
one designated copy. The designated copy is `copies()[0]`: deterministically
the earliest by `(outer basename, inner index, entry index)` (NOT
"whichever the filesystem/zip iteration happens to visit first", and NOT
influenced by thread count or cache size - `km_cache_tiles`'s only effect on
`DemImportReport` is how much of the dup bookkeeping gets re-derived from a
cache hit vs. freshly decoded, not which copy wins or whether a difference
is detected).

## Sections

| Type | Codec | Content |
|---|---|---|
| 1 meta | none | georef block, then `predictor` u8, `has_nodata` u8 (0/1) |
| 2 samples | zstd | 4 × n bytes: byte-plane-shuffled zigzag residuals |
| 3 nodata | zstd | only if has_nodata = 1: bitmap, ⌈n/8⌉ bytes |

`n = width × height` ≤ 2^24. The body has exactly sections {1, 2} or
{1, 2, 3}.

### Georef block (shared with src.landcover)

| Field | Type | Rule |
|---|---|---|
| zone | u8 | |
| hemisphere | u8 | 0/1 |
| west_mm | i64 | west edge of the raster, UTM easting, mm; \|v\| < 2^40 |
| south_mm | i64 | south edge, UTM northing, mm; \|v\| < 2^40 |
| spacing_mm | i64 | > 0; spacing × max(width, height) < 2^40 |
| registration | u8 | 0 node ("pixel is point"), 1 cell ("pixel is area") |
| row_order | u8 | 0 south-to-north, 1 north-to-south |
| width | u32 | 1..2^24 |
| height | u32 | 1..2^24, width × height ≤ 2^24 |

## Samples

Row-major, row 0 first. For each sample `v` (as uint32), its prediction `p`
is:

- **Left (1):** the west neighbour. In column 0 it is the first sample of
  the previous row, and for the first sample of the tile it is 0.
- **Gradient (2):** `west + north - northwest` (the previous row being
  "north" in storage order), all uint32 wrap-around. The first row and
  column 0 fall back to Left. This is the default.

The residual `d = v - p` (uint32 wrap-around) is zigzag-mapped as an int32,
`z = (d << 1) ^ (d >> 31)`. Plane k holds byte k of every `z`, in sample
order: first all low bytes, then all second bytes, and so on. The mostly
zero high planes become long runs for zstd.

A NoData sample stores `v := p`, so `z = 0`, and its bit is set in the
bitmap (bit `i & 7` of byte `i >> 3`, LSB first). The prediction chain
continues through it with that value. The decoder rejects:

- a NoData sample with `z ≠ 0`;
- a decoded `INT32_MIN` outside the bitmap;
- a bitmap with no bit set;
- padding bits set;
- has_nodata that disagrees with whether section 3 is present.
