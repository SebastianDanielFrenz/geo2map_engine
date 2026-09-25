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

## regions.json

A `g2m.regions/1` document with one region, `fixture`, resolving relation
4209939 against `osm/frankfurt_hoechst.osm.pbf` (`halo_m=200`). Used by
`test_import_resolve.cpp` for a fixture-scale `resolve_region`/`find_admin`
round trip; the real, multi-gigabyte-PBF-scale equivalent
(relations 62382/62566/4209939, halo_m=2000, ≈1085 tiles) is the
`[real-data]`-gated test, which needs `G2M_DATA_DIR` and is not run by
default CI.
