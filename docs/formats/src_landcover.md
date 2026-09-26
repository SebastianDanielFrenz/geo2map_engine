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

## Import: source pixel -> destination sample mapping (PLAN.md G1c I5)

`esa_worldcover_v200`'s own importer (`g2m::import::import_landcover`,
`core=src/import/landcover.cpp`, per-tile at 4 m cells, 256x256 samples)
resamples a native ~9.26 m WorldCover pixel onto this format's 4 m grid by
NEAREST NEIGHBOUR, not by area-weighting or interpolation - several
adjacent 4 m destination samples legitimately read the same source pixel.
Each destination sample's cell CENTRE (exact: `E = 1024x + 4c + 2`,
`N = 1024y + 4r + 2`) is converted to lon/lat via the project's existing
deterministic `g2m::geo::Utm::reverse`, then to a source pixel index via
`col = floor(lon*N) - C0`, `row = C1 - 1 - floor(lat*N)` (`N`/`C0`/`C1`
from the source file's own validated pixel scale and tiepoint corner).

**Edge rule**: this is intentionally ONE rounding step (`floor`), not a
round-to-nearest - a destination sample whose centre lands EXACTLY on a
source pixel boundary resolves to the pixel to its east (for longitude)
or south (for latitude), never the pixel to the west/north. This is an
accepted, documented convention (D17), not a bug: WorldCover's own pixels
are ~9.26 m and this project's destination cells are 4 m, so an exact
boundary coincidence is measure-zero in practice and every destination
sample near a source pixel edge still gets SOME single, deterministic
source pixel, consistently, independent of `--jobs` or worker scheduling.

**Datum**: ESA WorldCover's own coordinates are WGS84; this project's UTM
math (and every other layer) uses GRS80/ETRS89. The two differ by up to
~0.5 m at Hessen's latitude (a frame-realisation drift accumulated since
1989, not an ellipsoid shape difference - GRS80 and WGS84 agree to a
fraction of a millimetre in flattening). At WorldCover's ~10 m native
resolution this is sub-pixel and is deliberately ignored: no reprojection
or datum-shift step exists anywhere in this import path. See
`docs/formats/regions.md` and PLAN.md section 4.1 for the same note in the
region-resolution context.

**Multi-file coverage**: if more than one source file is given (a region
spanning a WorldCover tile boundary), files are tried in the CALLER's own
given order and the first file covering a given destination sample wins;
a sample not covered by any file resolves to class 0 - the same value the
source itself would report for a genuine "no data" pixel, so downstream
consumers never need to distinguish "not covered" from "covered but 0".
A tile where every one of its 65536 samples resolves to class 0 is
skipped entirely (not written), matching `g2m.src.elev`'s DEM importer's
own no-coverage convention.
