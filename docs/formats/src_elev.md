# src.elev — body encoding 1 (version 1)

Heights of the raw elevation layers `g2m.src.elev.<source>` (`dgm1`, later
`glo30`, ...). Level-2 tiles (1024 m). The source grid is kept as it is (its
georef travels in the body). DGM1 is cell-registered at 1 m, which makes
1024 × 1024 samples per tile.

Values are `int32` heights in 1/256 m. NoData is `INT32_MIN`, and it is
preserved exactly: the round trip is bit-exact for every value, NoData
included.

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
