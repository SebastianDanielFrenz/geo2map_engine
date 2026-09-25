# src.landcover — body encoding 2 (version 1)

Land cover classes of `g2m.src.landcover`: a uint8 class raster on the UTM
grid (nearest neighbour), in level-2 tiles (1024 m). The class numbering
belongs to a named scheme, and classifiers (G2) map it to g2m's own
classes.

## Sections

| Type | Codec | Content |
|---|---|---|
| 1 meta | none | georef block ([src_elev.md](src_elev.md#georef-block-shared-with-srclandcover)), `nodata_class` u8, `scheme_len` u8, `class_scheme` scheme_len bytes |
| 2 classes | zstd | width × height bytes, row-major in the georef's row order |

Rules:

- `class_scheme` is 1..64 bytes of `[a-z0-9_.]` (for example `esa_worldcover_v200`).
- `nodata_class` is the scheme's own "no data" value. It is stored as is
  and has no special encoding.
- The meta section has no trailing bytes, and the class section is exactly
  `width × height` bytes.
- The body has exactly sections {1, 2}.
