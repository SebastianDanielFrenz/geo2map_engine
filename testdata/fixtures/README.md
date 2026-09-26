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
  test-only tool, G1c I2 D20)
  ```
  g2m_fixture_cut cut-pbf --in hessen-latest.osm.pbf \
    --relation 4209939 --bbox 8.62,50.17,8.68,50.20 \
    --out frankfurt_hoechst.osm.pbf
  ```
- **Content**: relation 4209939 ("Höchst", a Frankfurt am Main Stadtteil,
  `admin_level=10`, `boundary=administrative`) plus its outer/inner member
  ways and their nodes (relation-closure extraction only - see
  `tests/support/fixture_cut/main.cpp`'s header comment for the exact,
  deliberately narrower-than-"everything in bbox" scope). 615 nodes, 5 ways,
  1 relation.
- **Size**: 4163 bytes.
- **SHA-256**: `b9e460b72d896c4ea95e4bbba8c87489d9f01b30a99761cc6ea7a2ae194b3e48`

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
  ~5.62 MiB, well under the 8 MB fixture budget.
- **SHA-256**: `8fddb6a1910e5ba90d295026ac802caab2745b7d8312d3dce5b037329c07f956`

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
