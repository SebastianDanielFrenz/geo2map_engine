# testdata/fixtures

Committed test fixtures for G1c I2's `g2m_import`/`g2m_tiler` tests
(`tests/unit/test_import_resolve.cpp`). Everything under `osm/` is derived
data, not hand-authored.

## osm/frankfurt_hoechst.osm.pbf

- **Source**: `hessen-latest.osm.pbf` (Geofabrik Hessen extract), as staged
  read-only at `S:\claude_code\geo2map_data\downloads\hessen-latest.osm.pbf`
  for this project. © OpenStreetMap contributors, licensed
  [ODbL 1.0](https://opendatacommons.org/licenses/odbl/1-0/). Redistribution
  of this small cut extract is permitted under ODbL's share-alike terms;
  this notice constitutes the required attribution.
- **Cut with**: `g2m_fixture_cut cut-pbf` (`tests/support/fixture_cut`,
  test-only tool, G1c I2 D20; bbox-extraction mode added G1c I4 pre-item 2 -
  see `tests/support/fixture_cut/main.cpp`'s header comment for the full
  algorithm):
  ```
  g2m_fixture_cut cut-pbf --in hessen-latest.osm.pbf --relation 4209939 \
    --bbox 8.635,50.1775,8.665,50.1925 --out frankfurt_hoechst.osm.pbf
  ```
  The task brief's bbox (`8.62,50.17,8.68,50.20`) was trimmed to this
  quarter-area box (same centre, 8.65/50.185) because the full-size cut
  came to 1,815,780 bytes, over the 1.8 MB fixture budget - it is dominated
  by a handful of large route relations whose member ways merely brush the
  bbox edge, not by node/way count in the bbox itself, so a smaller area
  only marginally reduces it (1610 -> 1357 relations for a 4x smaller box).
- **Content** (I4 pre-item 2 bbox mode: every node in the bbox; every way
  with >= 1 node in the bbox, completed with all its nodes; every relation
  with any included member, closed upward to a fixpoint; plus `--relation
  4209939`'s own downward closure; `add_metadata=false`, generator
  `"g2m_fixture_cut"`, no timestamp, entities sorted by type then id): 26315
  nodes, 5073 ways, 1357 relations. Relation 4209939 ("Höchst", a Frankfurt
  am Main Stadtteil, `admin_level=10`, `boundary=administrative`) is
  included via the explicit `--relation` closure; the other 1356 relations
  are pulled in by the upward sweep - mostly neighbouring
  Stadtteil/Stadtbezirk administrative boundaries and bus/hiking/cycling
  route relations whose member ways cross this residential area. See
  `test_import_resolve.cpp`'s `find_admin` test for the resulting set of
  `boundary=administrative` relations (7 at `admin_level=10`, including
  4209939; 3 at `admin_level=9`; none at `admin_level=6` - no kreisfreie
  Stadt/Landkreis boundary way happens to reach this bbox).
- **Size**: 948,364 bytes (~926 KiB).
- **SHA-256**: `dab190344af86d2feca297e68637155c99ade32df8189b0fd5e4def3c3361cae`

Superseded fixture (G1c I2, relation-closure-only extraction, kept only for
history - not committed, see git log): 615 nodes, 5 ways, 1 relation, 4163
bytes, SHA-256 `b9e460b72d896c4ea95e4bbba8c87489d9f01b30a99761cc6ea7a2ae194b3e48`.

## osm/hoechst_a66.osm.pbf

- **Source**: `hessen-latest.osm.pbf` (Geofabrik Hessen extract), as staged
  read-only at `S:\claude_code\geo2map_data\downloads\hessen-latest.osm.pbf`.
  (c) OpenStreetMap contributors, licensed
  [ODbL 1.0](https://opendatacommons.org/licenses/odbl/1-0/); the cut is a
  Derivative Database redistributed under the same terms, this notice is the
  attribution (see `THIRD_PARTY_NOTICES.md`).
- **Purpose** (street fidelity P1 slice 4): the home tile (utm32n/2/455/5421)
  plus about 300 m, around the B 8 / A 66 interchange at Hoechst. The
  lane-aware cross-section tests (`tests/layer/test_road_cross_section.cpp`,
  `test_road_joins.cpp`) use its turn-lane, `placement`, bridge and
  `transition` ways (H1-H7 in `docs/street_fidelity/p1_cross_section_design.md`
  section 4.6). `frankfurt_hoechst.osm.pbf` above is really Kalbach/Bonames
  (A 661) and stays for the F1-F10 cases.
- **Cut with** (test-only tool, same algorithm as above):
  ```
  g2m_fixture_cut cut-pbf --in hessen-latest.osm.pbf --relation 4209939 \
    --bbox 8.5190,50.1085,8.5418,50.1232 --out hoechst_a66.osm.pbf
  ```
  The full requested bbox fits the 1.8 MB budget, so it was not shrunk.
- **Content**: 13668 nodes, 2697 ways, 1266 relations.
- **Size**: 799,511 bytes (~781 KiB).
- **SHA-256**: `8556244549478372cccdead6d3fd59638b5e313fc9358ccb9a1a4b7ac2947dd8`

## osm/unsorted_nodes.osm.pbf

- **Source**: hand-built, not cut from real data - `g2m_fixture_cut
  write-unsorted-nodes` (`tests/support/fixture_cut/main.cpp`, G1c I4):
  a real, valid PBF file (osmium's own writer) with exactly 3 nodes whose
  ids are deliberately NOT strictly increasing (20, then 10, then 30 - a
  single out-of-order pair is enough), a case no real-world PBF writer ever
  produces so it cannot be cut from real data. Coordinates (8.65/50.18E/N
  area, +-0.001 deg per node) are arbitrary but inside Hessen's utm32n
  zone, matching `regions.json`'s "fixture" region, so
  `project_e7_to_mm` never returns "far" for them. `add_metadata=false`,
  generator `"g2m_fixture_cut"`.
- **Purpose**: exercises `g2m::import::extract_region_set`'s N1-pass
  strict-ordering check (`test_import_osm.cpp`'s "reports Malformed on an
  unsorted PBF" test) - `ErrorCode::Malformed`, message contains "PBF not
  sorted".
- **Cut with**:
  ```
  g2m_fixture_cut write-unsorted-nodes --out osm/unsorted_nodes.osm.pbf
  ```
- **Size**: 142 bytes.
- **SHA-256**: `3431f2e25aefc19947999dd657949947441e47e432a8da664befe4a372a53f22`

## dgm1/PKT_fixture.zip

- **Source**: Hessen DGM1 (1 m digital terrain model), staged read-only at
  `S:\claude_code\geo2map_data\downloads\dgm1\` for this project (© HVBG -
  Hessisches Landesamt für Bodenmanagement und Geoinformation, licensed
  [Datenlizenz Deutschland – Zero – Version 2.0](https://www.govdata.de/dl-de/zero-2-0)).
  This notice constitutes the required attribution.
- **Content**: a real, nested-ZIP outer `PKT_*.zip` structure (PLAN.md G1c
  I3's "nested-zip access" fact) repacked byte-for-byte from two of the
  8 real outer delivery zips, containing two inner municipality zips:
  - `inner0.zip` (from `PKT_Kreisfreie Stadt Frankfurt am Main.zip` !
    `Frankfurt_am_Main-DGM1.zip`): `DGM1_32_474_5560_1_he.tif`+`.tfw` and
    `DGM1_32_475_5560_1_he.tif`+`.tfw` (1,972,580 and 1,943,716 bytes).
  - `inner1.zip` (from `PKT_Hochtaunuskreis.zip` !
    `Bad Homburg v.d.Höhe - DGM1.zip`): `dgm1_32_474_5560_1_he.tif`+`.tfw`
    (1,972,344 bytes, lowercase "dgm1" prefix - deliberately kept as-is,
    exercising `dgm1::Catalog`'s case-insensitive filename parsing).
  Tile 474/5560 therefore has two real copies (from two different
  municipalities' deliveries) whose raw bytes differ but whose decoded
  height samples agree exactly (`test_decode_nested_zip.cpp`) - the same
  real-world duplicate-coverage fact `import_dem`'s `dup_*` counters exist
  to measure.
- **Cut with**: `g2m_fixture_cut repack-dgm1` (`tests/support/fixture_cut`,
  test-only tool, G1c I3), which reuses `dgm1::enumerate_tiles` to locate
  the wanted tiles, decompresses each tif/tfw pair with `ZipReader`, and
  re-packs them (STORED, original entry names) into fresh outer/inner zips
  via `tests/support/zip_builder.h`:
  ```
  g2m_fixture_cut repack-dgm1 --out dgm1/PKT_fixture.zip \
    --inner "PKT_Kreisfreie Stadt Frankfurt am Main.zip#Frankfurt_am_Main-DGM1.zip" \
    --inner "PKT_Hochtaunuskreis.zip#Bad Homburg v.d.Höhe - DGM1.zip" \
    --key 474,5560 --key 475,5560
  ```
- **Size**: 5,889,924 bytes (~5.62 MiB); `testdata/fixtures/` as a whole is
  6,842,992 bytes (~6.53 MiB, including the OSM fixture above and
  `regions.json`), within the fixture budget (raised from 8 to 10 MiB when the second OSM fixture hoechst_a66.osm.pbf was added; the whole directory is now about 8.25 MiB).
- **SHA-256**: `8fddb6a1910e5ba90d295026ac802caab2745b7d8312d3dce5b037329c07f956`

## worldcover/fixture_N50E008_hoechst.tif, worldcover/fixture_N50E008_bbox.tif

- **Source**: ESA WorldCover 10 m 2021 v200, tile N48E006, staged read-only
  at `S:\claude_code\geo2map_data\downloads\ESA_WorldCover_10m_2021_v200_N48E006_Map.tif`
  (94,225,409 bytes) for this project. © ESA WorldCover project 2021 /
  Contains modified Copernicus Sentinel data, licensed
  [CC BY 4.0](https://creativecommons.org/licenses/by/4.0/). This notice
  constitutes the required attribution.
- **Why two files**: `regions.json`'s `fixture` region (below) resolves,
  against the real `hessen-latest.osm.pbf`, to TWO separate level-2 tile
  clusters about 11 km apart - the Höchst admin relation's own irregular
  footprint (16 tiles) and the separate `utm_bbox` part (8 tiles), widened
  to full 1024 m tile boundaries by the region's own 200 m halo. A single
  bounding window covering both would have to span the ~11 km gap between
  them, far larger than either cluster on its own (this is a documented
  deviation from PLAN.md G1c I5's own "~330x260 px, ~90 KB" single-window
  estimate, which assumed one contiguous cluster) - `g2m_fixture_cut
  cut-worldcover` therefore cuts one window per CONNECTED cluster (8-
  connectivity over the tile grid) and this fixture ships both.
- **Cut with** (`tests/support/fixture_cut/main.cpp`'s `cut-worldcover`
  subcommand, G1c I5; `--resolved` is the `fixture` region resolved against
  the real PBF - not committed, a one-off scratch artefact, reproducible via
  `g2m_tiler resolve testdata/fixtures/regions.json#fixture --pbf
  <hessen-latest.osm.pbf> --out <scratch path>`):
  ```
  g2m_fixture_cut cut-worldcover \
    --in ESA_WorldCover_10m_2021_v200_N48E006_Map.tif \
    --resolved <scratch>/fixture.resolved.json --margin-px 2 \
    --out worldcover/fixture_N50E008_hoechst.tif \
    --out worldcover/fixture_N50E008_bbox.tif
  ```
  Cluster order matches `tile_keys()`'s own ascending (northing, easting)
  sort, so `--out` order above is stable across re-cuts. Both fixtures are
  uncompressed, single-strip, single-band uint8 GeoTIFFs: same
  `ModelPixelScaleTag` as the source (1/12000 deg), `ModelTiepointTag`
  moved to the window's own corner, a minimal `GeoKeyDirectory`
  (`GTRasterTypeGeoKey=1` PixelIsArea, `GeographicTypeGeoKey=4326`) and
  `GDAL_NODATA="0"`.
- **fixture_N50E008_hoechst.tif** (Höchst cluster, 16 tiles): window
  col `[30000..30973]` x row `[10500..11110]` = 974x611 px (col_lo/row_lo
  widened from the 2 px margin to the next lower multiple of 375, see the
  tiepoint-alignment note below; col_hi/row_hi unaffected).
  Lon `[8.50895..8.58097]`, lat `[50.0743..50.1206]`.
  Size: 595,404 bytes.
  SHA-256: `a81638668b11726e7a9e700af02b3f46ec6dde893461252fdf70b47f1675ff11`.
- **fixture_N50E008_bbox.tif** (utm_bbox cluster, 8 tiles): window
  col `[31125..32166]` x row `[9375..9778]` = 1042x404 px (same widening).
  Lon `[8.62289..8.68041]`, lat `[50.1853..50.2038]`.
  Size: 421,258 bytes.
  SHA-256: `806c442100e789bb1d55f7342199f21ed6d7060efff4c96d65558dd1aeae37f1`.
- **Geodetic note** (PLAN.md 4.1): ESA WorldCover's coordinates are WGS84;
  this project's own UTM math uses GRS80/ETRS89, which differ from WGS84
  by up to ~0.5 m at Hessen's latitude (frame-realisation drift, not an
  ellipsoid shape difference). Sub-pixel at WorldCover's own ~10 m native
  resolution and deliberately ignored - no reprojection/datum-shift step
  exists anywhere in this import path (see `docs/formats/src_landcover.md`).
- **Real product's own georeference** (recorded here for reference, not
  fixture-specific): `ModelPixelScale` = `8.333333333333333e-05` (exactly
  1/12000 deg), `ModelTiepoint` = `[0,0,0, 6.0, 51.0, 0.0]` (so
  `lon0*N=72000`, `lat0*N=612000`, both exact integers), `GDAL_NODATA="0"`,
  `GTRasterTypeGeoKey=1` (PixelIsArea), `GeographicTypeGeoKey=4326`.
- Used by `test_import_landcover.cpp`'s `[real-data]` fixture-vs-source
  window-equality test, gated on `G2M_DATA_DIR`.

### `tests/support/tiff_builder.h`'s `add_ascii` fix (found while cutting this fixture)

`TiffBuilder::add_ascii` used to always store its value at an external
offset, but classic TIFF stores any value whose byte count (including the
NUL terminator) fits in the 4-byte value/offset field INLINE instead, and
`tiff_reader.cpp`'s `read_ascii` follows that convention exactly (it never
dereferences the field as an offset when `count <= 4`). `GDAL_NODATA="0"`
is exactly this short-value case (2 bytes incl. NUL) - every existing
`add_ascii` call in this codebase before G1c I5 used `"-9999"` (6 bytes,
unaffected), so nothing had exercised the bug until this fixture's own
`GDAL_NODATA="0"` did. Fixed in `tiff_builder.h` (now writes inline for
values <= 4 bytes) - this fixture was re-cut after the fix, so its checked-
in bytes and the SHA-256s above are the CORRECT ones. A second, unrelated
bug in the same fix's first attempt (reading `bytes.size()` in the same
call expression as `std::move(bytes)` - unspecified argument evaluation
order let the callee's by-value parameter move-construct, and thus empty
`bytes`, before the sibling `bytes.size()` argument was evaluated on this
compiler) is also fixed by computing the count into a local variable
before the call.

### `cut-worldcover`'s tiepoint round-trip fix (found by the fixture-vs-source byte-equality test)

The first re-cut (after the `add_ascii` fix above) still failed
`test_import_landcover.cpp`'s fixture-vs-source byte-equality test: the
window's own re-derived corner was off by exactly one pixel from the true
cut position. Root cause: WorldCover's `N=12000` factors as
`2^5 * 3 * 5^3` - NOT a power of two - so `new_c0/N` (a plain double
division of the window's integer corner by N, the way `cut-worldcover`
computed the new `ModelTiepointTag` value) does not generally round-trip
back to `new_c0` through `(new_c0/N)*N`, the exact reconstruction
`landcover.cpp`'s own `open_and_validate` performs to recover a file's
`(C0, C1)`. A ULP-level search for a nearby representable double that DOES
round-trip (`std::nextafter`, checked out to +-2000 ULPs) never finds one
for a `new_c0` not already divisible by 375: the target integer sits
exactly BETWEEN the two representable reconstructed products bracketing
it, one ULP-of-the-product apart, so no correctly-rounded multiplication
can land on it. The real fix is upstream of the value itself: `cut-
worldcover` now widens (never narrows) the cut window's `col_lo`/`row_lo`
down to the nearest multiple of 375 (`kTiepointAlignment` in
`fixture_cut/main.cpp`) before computing the new corner - since the real
source file's own tiepoint (`6.0`/`51.0`, whole degrees) is itself already
a multiple of 375 times N, this reduces the condition to `col_lo % 375 ==
0`/`row_lo % 375 == 0`, at which point `new_c0/N = (new_c0/375)/32` is a
division by a pure power of two, which IEEE-754 always performs exactly.
This is why both fixture windows above are wider than the plan's own
`--margin-px 2` alone would produce, and why both fixtures had to be
re-cut and their sizes/hashes above updated again after this fix (the
`add_ascii` fix alone was not sufficient - it fixed the GDAL_NODATA tag but
not this separate tiepoint-precision defect). `test_import_landcover.cpp`
also gained a direct guard: it checks `floor(corner*N) == corner*N` on the
checked-in fixture's own tiepoint, matching `open_and_validate`'s own
exactness requirement, so a future re-cut that reintroduces this defect
fails loudly instead of only failing the byte-equality comparison.

## regions.json

A `g2m.regions/1` document with one region, `fixture`, with two parts
against `osm/frankfurt_hoechst.osm.pbf` (`halo_m=200`):
- the original `osm_admin` part resolving relation 4209939 ("Höchst");
- a `utm_bbox` part, 474000/5560000 - 476000/5561000 (PLAN.md G1c pre-item
  a), chosen so its level-2 tile footprint overlaps `dgm1/PKT_fixture.zip`'s
  two 1 km squares (474/5560, 475/5560) - `test_import_dem.cpp` resolves
  this same bbox directly (not through this file) to exercise `import_dem`
  against the DGM1 fixture.

Used by `test_import_resolve.cpp` for a fixture-scale `resolve_region`/
`find_admin` round trip; the real, multi-gigabyte-PBF-scale equivalent
(relations 62382/62566/4209939, halo_m=2000, ≈1085 tiles) is the
`[real-data]`-gated test, which needs `G2M_DATA_DIR` and is not run by
default CI.

- **SHA-256**: `586df4ace50c7cb45a795814af1b7e7371ca4f370ccb8aeb29cc62a0b59c40a5`

# testdata/fuzz_seeds

Real-data-shaped seed corpus for `tests/fuzz` (PLAN.md G1c I6 item B),
passed as a second, read-only libFuzzer corpus dir alongside each
`fuzz_{zip,tiff,pbf}` target's own mutable work corpus (see
`tests/fuzz/CMakeLists.txt`). Every file here is generated by this
project's own test tools - never hand-crafted bytes or a third-party
sample - and kept under 50 KB per file.

## zip/nested_20x20.zip

- **Content**: a real (not synthetic-bytes) two-level nested ZIP, matching
  `dgm1::Catalog`'s own outer/inner nesting shape
  (`testdata/fixtures/dgm1/PKT_fixture.zip` above): an outer STORED ZIP
  containing `inner0.zip`, itself a STORED ZIP containing `tile.tif` - a
  minimal uncompressed 20x20 uint8 single-strip TIFF built with
  `tests/support/tiff_builder.h`. Exercises `fuzz_zip.cpp`'s one-level
  `entry_source`/recursion path (depth <= `kMaxDepth` = 2) end to end.
- **Generated with**: `g2m_fixture_cut make-fuzz-seeds`
  (`tests/support/fixture_cut/main.cpp`, new subcommand added for this
  item; builds all three seed files below in one invocation - the `zip`
  output is `--out-zip`):
  ```
  g2m_fixture_cut make-fuzz-seeds \
    --out-zip testdata/fuzz_seeds/zip/nested_20x20.zip \
    --out-tiff-lzw testdata/fuzz_seeds/tiff/dgm1_shape_32x32_lzw_pred2_f32.tif \
    --crop-src testdata/fixtures/worldcover/fixture_N50E008_hoechst.tif \
    --out-tiff-crop testdata/fuzz_seeds/tiff/worldcover_crop_64x64_u8.tif
  ```
- **Verified**: decodes correctly through the real `ZipReader`/`TiffReader`
  (outer -> `entry_source` -> inner -> `read_entry` -> `TiffReader::
  decode_window_u8`, 20x20, `at(0,0)=0`) via a one-off scratch harness
  before being committed.
- **Size**: 778 bytes.
- **SHA-256**: `aa19d9049f97ad54a1c618ae245d6bb4e6e262f06613f093c5929bfb505aabd4`

## tiff/dgm1_shape_32x32_lzw_pred2_f32.tif

- **Content**: a 32x32 single-strip float32 TIFF, LZW-compressed
  (compression tag 259 = 5) with horizontal differencing (predictor tag
  317 = 2) - the exact encoding shape a real DGM1 tile uses (vault fact
  G2M-002; per `tests/unit/test_decode_tiff.cpp`'s own reference encoder,
  the predictor differences the raw uint32 WORD each float32 sample
  occupies, not individual bytes). Sample values are
  `100.0f + 0.5f*(i % 37)` for row-major index `i`, so `at(0,0)=100.0`,
  `at(1,0)=100.5`, `at(31,31)=112.0` (index 1023, `1023 % 37 = 24`,
  `100 + 0.5*24 = 112`) - independently confirmed by decoding the
  committed file before adding it to the corpus.
- **Generated with**: same `make-fuzz-seeds` invocation as above
  (`--out-tiff-lzw`).
- **Size**: 5954 bytes.
- **SHA-256**: `8a927176b41a18511fea7c7e4448e92c64a0beca9c532984c842f714b7a4a009`

## tiff/worldcover_crop_64x64_u8.tif

- **Content**: a 64x64 (or the source's full extent if smaller - not the
  case here) uncompressed single-band uint8 crop of
  `testdata/fixtures/worldcover/fixture_N50E008_hoechst.tif`'s own top-left
  corner, decoded via `TiffReader::decode_window_u8` and re-encoded with
  `tiff_builder.h` - real WorldCover landcover-code byte values (`at(0,0)
  =30`, confirmed at generation time), not synthetic ones, at a small,
  fuzz-friendly size.
- **Generated with**: same `make-fuzz-seeds` invocation as above
  (`--crop-src`/`--out-tiff-crop`).
- **Size**: 4242 bytes.
- **SHA-256**: `6856003e2c9970caeb6d14e4ed1f6908ec2a6edbd6b199348da23a06ad84a79a`

## pbf/tiny_bbox_cut.osm.pbf

- **Content**: a much smaller cut of the same source/relation as
  `osm/frankfurt_hoechst.osm.pbf` above, using a tight bbox purely to stay
  under the 50 KB per-seed-file budget (this file is a fuzz seed, not a
  test fixture - it does not need to reproduce any particular relation/
  admin_level coverage, just be a small, real, well-formed PBF for the
  fuzzer to mutate from): 1039 nodes, 89 ways, 74 relations.
- **Cut with** (`g2m_fixture_cut cut-pbf`, same tool/algorithm as the OSM
  fixture above, run against the same read-only staged source):
  ```
  g2m_fixture_cut cut-pbf --in hessen-latest.osm.pbf --relation 4209939 \
    --bbox 8.6490,50.1830,8.6510,50.1850 --out tiny_bbox_cut.osm.pbf
  ```
- **Size**: 44822 bytes.
- **SHA-256**: `6c991f1dd68539403dd932908a0f6ddbf7f9f31cb79f232a1f03935dd1c0c019`
