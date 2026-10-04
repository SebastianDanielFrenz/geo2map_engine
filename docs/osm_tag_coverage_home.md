> Snapshot of 2026-10-05, copied from `out/osm_tag_coverage_home.md` (written by `g2m_osm_tag_probe`) with this note prepended, so the file is not byte-identical to the tool output. Regenerate: `out\build\release\tools\osm_tag_probe\g2m_osm_tag_probe.exe --pbf <hessen-latest.osm.pbf> --region tools\osm_tag_probe\home_probe_region.json#home --out-md out\osm_tag_coverage_home.md --out-json out\osm_tag_coverage_home.json` (about 28 s, 0.6 GB RSS). The JSON twin lives in `out/` only (gitignored).

# OSM tag coverage: home

Phase P0 of the street-level fidelity plan (`out/street_fidelity_plan.md` section 6). Produced by `g2m_osm_tag_probe`; counts are deterministic (same input, same bytes).

## Input

- PBF: `hessen-latest.osm.pbf`, sha256 `4645bae9973881979636789eeb2681783ab3cc9193a5ba7d41466024410ad281`
- Region `home`: union of these OSM administrative boundaries

| relation | name | admin_level | geometry hash |
|---:|---|---:|---|
| 62382 | Main-Taunus-Kreis | 6 | `165df866b8ff4ae8` |
| 62566 | Hochtaunuskreis | 6 | `d54deea38e138d90` |
| 62400 | Frankfurt am Main | 6 | `47455cf4da8378c1` |

Scan: 31958253 nodes, 4924969 ways, 64103 relations in the PBF. Tagged nodes of interest inside the region: 41581. Relevant ways: 3351398, of which in the region 353496 (anchor node not in the PBF: 0; in-region ways with a node ref missing from the PBF: 0). Relevant relations (multipolygon building/forest): 4427, in the region 591 (exact ring assembly rejected, lenient ring areas used: 31; counted with area 0: 0; no geometry, skipped: 0).

## Method

- A way or polygon is in the region when its **anchor** lies inside the union: the node at index n/2 of a way's node list (a relation: the first vertex of its first assembled ring). Exact integer point-in-polygon on UTM zone 32 (GRS80) millimetres, the same ring assembly and region cut the tiler uses. A way crossing the border is counted whole, once.
- Relation areas: outer rings minus inner rings from the region resolver's exact ring assembly; where that rejects the geometry (crossing or overlapping rings), from a lenient role-based chaining of the member ways (the scan line above counts those).
- Lengths are summed segment lengths in the UTM grid (not corrected for the 0.9996 scale factor, error under 0.1 percent). Areas are shoelace areas in the UTM grid.
- A key is **present** when it exists with a non-empty value (so `sidewalk=no` counts as mapped).
- Percent cells in the road tables read `% of ways / % of km`.
- Metric groups: `lanes_fwd_bwd` = `lanes:forward` or `lanes:backward`; `turn_lanes` = `turn:lanes` with or without `:forward`, `:backward`, `:both_ways`; `change_lanes` likewise; `placement` = `placement`, `placement:forward`, `placement:backward`; `width_lanes` = `width:lanes[:forward|:backward]`; `sidewalk`/`cycleway` = the bare key or `:left`/`:right`/`:both`; `maxspeed` = bare or `:forward`/`:backward`; `overtaking` likewise; `parking_lane` = any key starting `parking:lane`; `parking_any` = any key starting `parking:`; `junction_roundabout` = `junction=roundabout`.
- Node placement is derived from the node's degree in carriageway ways (the 13 reported classes plus `living_street`), counted over the whole PBF: interior occurrence 2, way end 1.
- Buildings: `building=*` other than `no`, closed ways plus multipolygon relations (old-style relations whose outer way also carries `building` count twice; rare). Forests: closed ways and multipolygon relations tagged `natural=wood` or `landuse=forest`, counted once even when both are set.

## 1. Road classes

| class | ways | km |
|---|---:|---:|
| motorway | 775 | 217.630 |
| motorway_link | 832 | 107.161 |
| trunk | 378 | 79.428 |
| trunk_link | 410 | 40.985 |
| primary | 2448 | 244.177 |
| primary_link | 247 | 11.155 |
| secondary | 4562 | 515.040 |
| secondary_link | 319 | 13.904 |
| tertiary | 3381 | 369.952 |
| tertiary_link | 154 | 6.103 |
| unclassified | 2070 | 256.365 |
| residential | 16266 | 2028.875 |
| service | 26830 | 1903.824 |
| **all listed** | 58672 | 5794.598 |

### Lanes (% of ways / % of km)

| class | lanes | lanes_fwd_bwd | turn_lanes | change_lanes | placement | lane_markings |
|---|---:|---:|---:|---:|---:|---:|
| motorway | 100.0 / 100.0 | 0.0 / 0.0 | 35.5 / 26.7 | 3.0 / 1.2 | 33.5 / 26.4 | 0.0 / 0.0 |
| motorway_link | 99.4 / 99.7 | 0.1 / 0.0 | 23.1 / 16.2 | 5.0 / 2.5 | 8.3 / 5.0 | 0.7 / 0.4 |
| trunk | 100.0 / 100.0 | 0.8 / 0.3 | 36.0 / 21.9 | 2.1 / 1.0 | 22.8 / 12.3 | 0.0 / 0.0 |
| trunk_link | 93.4 / 94.2 | 1.5 / 1.4 | 17.6 / 13.8 | 1.5 / 1.3 | 6.1 / 3.8 | 2.7 / 3.7 |
| primary | 94.7 / 92.1 | 11.6 / 9.3 | 35.7 / 18.0 | 6.5 / 6.7 | 12.2 / 7.3 | 3.1 / 2.1 |
| primary_link | 76.5 / 79.0 | 3.2 / 2.7 | 15.4 / 12.6 | 2.4 / 3.6 | 2.4 / 1.3 | 21.9 / 19.8 |
| secondary | 83.8 / 79.8 | 15.4 / 8.7 | 26.9 / 12.0 | 4.0 / 2.3 | 4.3 / 1.8 | 10.2 / 10.5 |
| secondary_link | 67.4 / 66.5 | 0.3 / 0.2 | 10.7 / 10.6 | 0.3 / 0.3 | 0.6 / 0.5 | 24.5 / 23.7 |
| tertiary | 71.2 / 67.1 | 9.5 / 6.2 | 14.8 / 6.9 | 1.7 / 1.3 | 2.6 / 1.1 | 25.3 / 27.0 |
| tertiary_link | 79.2 / 80.3 | 1.9 / 2.7 | 13.6 / 11.5 | 0.0 / 0.0 | 3.2 / 1.7 | 24.0 / 24.4 |
| unclassified | 39.7 / 29.9 | 3.4 / 2.7 | 6.7 / 2.3 | 1.4 / 0.5 | 1.1 / 0.3 | 37.7 / 41.6 |
| residential | 12.1 / 8.5 | 1.1 / 0.4 | 1.7 / 0.5 | 0.1 / 0.0 | 0.2 / 0.1 | 15.3 / 11.0 |
| service | 3.7 / 5.1 | 0.1 / 0.1 | 0.4 / 0.2 | 0.1 / 0.1 | 0.1 / 0.1 | 0.2 / 0.2 |
| **all listed** | **25.9 / 29.3** | **2.7 / 1.9** | **6.6 / 4.4** | **1.0 / 0.8** | **1.9 / 1.9** | **8.4 / 8.7** |

### Width, edges and parking (% of ways / % of km)

| class | width | est_width | width_lanes | sidewalk | cycleway | parking_lane | parking_any |
|---|---:|---:|---:|---:|---:|---:|---:|
| motorway | 4.4 / 4.9 | 0.0 / 0.0 | 1.3 / 0.9 | 0.4 / 0.3 | 4.9 / 6.4 | 0.0 / 0.0 | 0.0 / 0.0 |
| motorway_link | 2.4 / 2.7 | 0.0 / 0.0 | 0.2 / 0.2 | 1.9 / 0.6 | 3.0 / 1.9 | 0.0 / 0.0 | 0.8 / 0.2 |
| trunk | 7.1 / 7.8 | 0.0 / 0.0 | 0.0 / 0.0 | 10.6 / 10.4 | 10.8 / 10.5 | 0.0 / 0.0 | 8.7 / 6.1 |
| trunk_link | 3.9 / 7.2 | 0.0 / 0.0 | 0.0 / 0.0 | 18.5 / 21.2 | 14.1 / 18.4 | 0.0 / 0.0 | 9.3 / 12.5 |
| primary | 0.5 / 0.5 | 0.0 / 0.0 | 0.9 / 0.4 | 73.7 / 63.3 | 79.9 / 77.2 | 0.5 / 0.2 | 52.9 / 39.6 |
| primary_link | 0.0 / 0.0 | 0.0 / 0.0 | 0.0 / 0.0 | 68.0 / 52.0 | 76.9 / 71.3 | 0.0 / 0.0 | 40.9 / 28.0 |
| secondary | 1.5 / 1.5 | 0.0 / 0.0 | 0.7 / 0.4 | 62.3 / 48.8 | 78.5 / 74.7 | 1.4 / 0.9 | 31.5 / 19.8 |
| secondary_link | 0.3 / 0.3 | 0.0 / 0.0 | 0.0 / 0.0 | 58.3 / 46.6 | 71.8 / 70.2 | 0.3 / 0.2 | 36.7 / 28.4 |
| tertiary | 6.8 / 4.4 | 0.0 / 0.0 | 0.5 / 0.2 | 68.8 / 57.1 | 74.2 / 66.0 | 1.2 / 1.4 | 30.1 / 18.8 |
| tertiary_link | 5.8 / 6.0 | 0.0 / 0.0 | 0.0 / 0.0 | 42.2 / 42.1 | 60.4 / 60.4 | 0.6 / 0.8 | 29.2 / 24.8 |
| unclassified | 6.2 / 6.0 | 0.0 / 0.0 | 0.0 / 0.0 | 46.8 / 45.5 | 54.7 / 52.7 | 1.6 / 1.2 | 25.0 / 21.4 |
| residential | 8.4 / 7.5 | 0.0 / 0.0 | 0.1 / 0.0 | 64.3 / 58.6 | 39.5 / 31.9 | 1.4 / 1.3 | 28.2 / 19.6 |
| service | 0.9 / 1.9 | 0.0 / 0.0 | 0.0 / 0.0 | 2.2 / 2.9 | 1.2 / 1.9 | 0.2 / 0.2 | 1.2 / 1.2 |
| **all listed** | **3.7 / 4.4** | **0.0 / 0.0** | **0.2 / 0.1** | **33.3 / 34.7** | **28.3 / 29.1** | **0.8 / 0.7** | **16.2 / 13.2** |

### Surface and rules (% of ways / % of km)

| class | surface | smoothness | maxspeed | lit | oneway | junction_roundabout | divider | overtaking |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| motorway | 93.3 / 93.9 | 25.5 / 24.7 | 95.7 / 96.7 | 82.5 / 86.6 | 100.0 / 100.0 | 0.0 / 0.0 | 0.0 / 0.0 | 0.1 / 0.1 |
| motorway_link | 89.8 / 94.0 | 15.1 / 16.6 | 64.1 / 67.0 | 78.2 / 81.3 | 100.0 / 100.0 | 0.0 / 0.0 | 0.0 / 0.0 | 0.4 / 0.5 |
| trunk | 70.9 / 67.8 | 43.4 / 38.4 | 98.4 / 95.4 | 58.5 / 61.1 | 98.9 / 99.3 | 0.0 / 0.0 | 0.0 / 0.0 | 2.6 / 2.6 |
| trunk_link | 56.1 / 56.4 | 20.0 / 19.9 | 70.5 / 70.0 | 35.1 / 44.0 | 97.6 / 97.8 | 0.0 / 0.0 | 0.0 / 0.0 | 5.1 / 5.8 |
| primary | 98.9 / 99.5 | 45.3 / 43.2 | 98.4 / 99.4 | 81.3 / 69.5 | 74.7 / 58.9 | 1.1 / 0.2 | 0.2 / 0.1 | 0.8 / 1.2 |
| primary_link | 100.0 / 100.0 | 42.9 / 36.3 | 67.2 / 57.8 | 73.7 / 60.0 | 96.0 / 96.9 | 0.0 / 0.0 | 0.0 / 0.0 | 0.8 / 1.8 |
| secondary | 95.9 / 93.2 | 32.9 / 28.9 | 95.0 / 95.4 | 69.6 / 54.5 | 48.9 / 33.1 | 1.4 / 0.5 | 0.0 / 0.0 | 1.5 / 1.5 |
| secondary_link | 95.9 / 95.6 | 22.9 / 18.0 | 55.8 / 52.5 | 72.4 / 69.7 | 97.2 / 95.1 | 0.0 / 0.0 | 0.0 / 0.0 | 0.0 / 0.0 |
| tertiary | 94.8 / 92.1 | 30.6 / 23.7 | 90.5 / 90.2 | 69.6 / 57.0 | 43.4 / 24.0 | 2.3 / 0.8 | 0.0 / 0.0 | 1.9 / 2.3 |
| tertiary_link | 96.8 / 96.8 | 17.5 / 13.7 | 57.8 / 56.7 | 57.8 / 56.6 | 95.5 / 95.1 | 0.0 / 0.0 | 0.0 / 0.0 | 0.0 / 0.0 |
| unclassified | 86.2 / 83.2 | 24.0 / 25.3 | 71.6 / 76.5 | 49.9 / 46.7 | 39.9 / 25.4 | 2.4 / 0.6 | 0.0 / 0.0 | 0.3 / 0.3 |
| residential | 83.9 / 81.8 | 18.2 / 14.1 | 85.2 / 89.2 | 67.7 / 64.1 | 31.3 / 25.1 | 0.3 / 0.1 | 0.0 / 0.0 | 0.1 / 0.0 |
| service | 43.7 / 46.7 | 5.3 / 6.3 | 5.8 / 8.8 | 11.7 / 11.7 | 12.5 / 12.2 | 0.0 / 0.0 | 0.0 / 0.0 | 0.0 / 0.0 |
| **all listed** | **67.9 / 73.2** | **15.9 / 16.1** | **49.5 / 62.9** | **42.4 / 46.0** | **30.5 / 29.0** | **0.5 / 0.2** | **0.0 / 0.0** | **0.4 / 0.5** |

**Top values of turn:lanes (all turn:lanes keys merged)** (3975 tagged, 298 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `left\|through` | 570 | 14.3 |
| 2 | `left\|through;right` | 310 | 7.8 |
| 3 | `through\|right` | 198 | 5.0 |
| 4 | `left\|right` | 182 | 4.6 |
| 5 | `left\|through\|through` | 180 | 4.5 |
| 6 | `through\|through;right` | 171 | 4.3 |
| 7 | `through\|through\|right` | 133 | 3.3 |
| 8 | `left\|through\|through;right` | 107 | 2.7 |
| 9 | `through\|through` | 106 | 2.7 |
| 10 | `left\|through\|right` | 89 | 2.2 |
| 11 | `left;through\|right` | 87 | 2.2 |
| 12 | `left\|left` | 67 | 1.7 |
| 13 | `through\|slight_right` | 59 | 1.5 |
| 14 | `none\|none\|slight_right` | 57 | 1.4 |
| 15 | `through\|through\|slight_right` | 53 | 1.3 |
| | _other (283 values)_ | 1606 | 40.4 |

## 2. Nodes

**highway=crossing nodes**: 9370

| key present | count | % of count |
|---|---:|---:|
| `crossing` | 7913 | 84.5 |
| `crossing:markings` | 4418 | 47.2 |
| `crossing_ref` | 1445 | 15.4 |

Placement (carriageway ways through the node):

| class | count | % | meaning |
|---|---:|---:|---|
| off_carriageway | 179 | 1.9 | no carriageway way uses the node |
| dead_end | 4 | 0.0 | degree 1 (way end) |
| mid_way | 8985 | 95.9 | degree 2 (mid-way or a way split, not a junction) |
| junction | 202 | 2.2 | degree >= 3 (junction) |

**crossing=* values** (7913 tagged, 12 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `traffic_signals` | 3120 | 39.4 |
| 2 | `uncontrolled` | 2074 | 26.2 |
| 3 | `unmarked` | 1741 | 22.0 |
| 4 | `marked` | 770 | 9.7 |
| 5 | `zebra` | 164 | 2.1 |
| 6 | `informal` | 35 | 0.4 |
| 7 | `continuous` | 4 | 0.1 |
| 8 | `controlled` | 1 | 0.0 |
| 9 | `if` | 1 | 0.0 |
| 10 | `unknown` | 1 | 0.0 |
| 11 | `weg` | 1 | 0.0 |
| 12 | `yes` | 1 | 0.0 |

**Top values of crossing:markings** (4418 tagged, 10 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `no` | 2132 | 48.3 |
| 2 | `zebra` | 1184 | 26.8 |
| 3 | `dashes` | 623 | 14.1 |
| 4 | `yes` | 437 | 9.9 |
| 5 | `dots` | 20 | 0.5 |
| 6 | `surface` | 14 | 0.3 |
| 7 | `dashes;surface` | 3 | 0.1 |
| 8 | `lines` | 2 | 0.0 |
| 9 | `yes;dashes` | 2 | 0.0 |
| 10 | `longitudinal_bars2` | 1 | 0.0 |

**highway=traffic_signals nodes**: 3392

| key present | count | % of count |
|---|---:|---:|
| `traffic_signals:direction` | 1880 | 55.4 |
| `direction` | 30 | 0.9 |

Placement (carriageway ways through the node):

| class | count | % | meaning |
|---|---:|---:|---|
| off_carriageway | 107 | 3.2 | no carriageway way uses the node |
| dead_end | 0 | 0.0 | degree 1 (way end) |
| mid_way | 3102 | 91.5 | degree 2 (mid-way or a way split, not a junction) |
| junction | 183 | 5.4 | degree >= 3 (junction) |

**highway=stop nodes**: 261

| key present | count | % of count |
|---|---:|---:|
| `direction` | 225 | 86.2 |

Placement (carriageway ways through the node):

| class | count | % | meaning |
|---|---:|---:|---|
| off_carriageway | 11 | 4.2 | no carriageway way uses the node |
| dead_end | 2 | 0.8 | degree 1 (way end) |
| mid_way | 246 | 94.3 | degree 2 (mid-way or a way split, not a junction) |
| junction | 2 | 0.8 | degree >= 3 (junction) |

**highway=give_way nodes**: 491

| key present | count | % of count |
|---|---:|---:|
| `direction` | 373 | 76.0 |

Placement (carriageway ways through the node):

| class | count | % | meaning |
|---|---:|---:|---|
| off_carriageway | 19 | 3.9 | no carriageway way uses the node |
| dead_end | 0 | 0.0 | degree 1 (way end) |
| mid_way | 472 | 96.1 | degree 2 (mid-way or a way split, not a junction) |
| junction | 0 | 0.0 | degree >= 3 (junction) |

**traffic_calming=* nodes**: 545


Placement (carriageway ways through the node):

| class | count | % | meaning |
|---|---:|---:|---|
| off_carriageway | 78 | 14.3 | no carriageway way uses the node |
| dead_end | 0 | 0.0 | degree 1 (way end) |
| mid_way | 441 | 80.9 | degree 2 (mid-way or a way split, not a junction) |
| junction | 26 | 4.8 | degree >= 3 (junction) |

**traffic_calming by type (nodes)** (545 tagged, 12 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `bump` | 178 | 32.7 |
| 2 | `table` | 136 | 25.0 |
| 3 | `island` | 99 | 18.2 |
| 4 | `hump` | 60 | 11.0 |
| 5 | `choker` | 18 | 3.3 |
| 6 | `rumble_strip` | 17 | 3.1 |
| 7 | `chicane` | 12 | 2.2 |
| 8 | `cushion` | 12 | 2.2 |
| 9 | `mini_bumps` | 9 | 1.7 |
| 10 | `dip` | 2 | 0.4 |
| 11 | `painted_island` | 1 | 0.2 |
| 12 | `yes` | 1 | 0.2 |

**kerb=* nodes**: 6149


Placement (carriageway ways through the node):

| class | count | % | meaning |
|---|---:|---:|---|
| off_carriageway | 4208 | 68.4 | no carriageway way uses the node |
| dead_end | 3 | 0.0 | degree 1 (way end) |
| mid_way | 1906 | 31.0 | degree 2 (mid-way or a way split, not a junction) |
| junction | 32 | 0.5 | degree >= 3 (junction) |

**kerb=* values (nodes)** (6149 tagged, 14 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `lowered` | 4653 | 75.7 |
| 2 | `flush` | 899 | 14.6 |
| 3 | `raised` | 342 | 5.6 |
| 4 | `no` | 214 | 3.5 |
| 5 | `normal` | 14 | 0.2 |
| 6 | `flush;lowered` | 12 | 0.2 |
| 7 | `rolled` | 8 | 0.1 |
| 8 | `flu` | 1 | 0.0 |
| 9 | `lowered_and_sloped` | 1 | 0.0 |
| 10 | `none` | 1 | 0.0 |
| 11 | `nor` | 1 | 0.0 |
| 12 | `regular` | 1 | 0.0 |
| 13 | `separate` | 1 | 0.0 |
| 14 | `yes` | 1 | 0.0 |

Ways carrying the same families:

**traffic_calming=* ways**: 47, 1.231 km


**traffic_calming by type (ways)** (47 tagged, 4 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `island` | 26 | 55.3 |
| 2 | `table` | 16 | 34.0 |
| 3 | `yes` | 3 | 6.4 |
| 4 | `bump` | 2 | 4.3 |

**kerb=* ways**: 171, 6.477 km


**footway=crossing ways**: 4631, 46.874 km

| key present | count | % of count | % of km |
|---|---:|---:|---:|
| `crossing` | 2200 | 47.5 | 49.4 |
| `crossing:markings` | 1609 | 34.7 | 35.2 |

## 3. Buildings

**Buildings (closed ways and multipolygon relations)**: 286571, 5003.7 ha

| key present | count | % of count | % of area |
|---|---:|---:|---:|
| `height` | 658 | 0.2 | 1.1 |
| `building:levels` | 54905 | 19.2 | 24.0 |
| `roof:shape` | 35926 | 12.5 | 14.3 |
| `roof:levels` | 41729 | 14.6 | 14.5 |
| `roof:colour` | 2758 | 1.0 | 1.2 |
| `roof:material` | 1192 | 0.4 | 0.7 |
| `building:colour` | 2543 | 0.9 | 1.5 |
| `building:material` | 1368 | 0.5 | 0.9 |
| `roof:height` | 193 | 0.1 | 0.0 |
| `min_height` | 27 | 0.0 | 0.0 |

Of these, from relations: 442. Open ways tagged `building` (not counted above): 0.

**Top values of roof:shape** (35926 tagged, 30 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `gabled` | 19741 | 54.9 |
| 2 | `flat` | 10859 | 30.2 |
| 3 | `hipped` | 1965 | 5.5 |
| 4 | `gambrel` | 899 | 2.5 |
| 5 | `mansard` | 592 | 1.6 |
| 6 | `skillion` | 549 | 1.5 |
| 7 | `side_hipped` | 288 | 0.8 |
| 8 | `half-hipped` | 285 | 0.8 |
| 9 | `pyramidal` | 198 | 0.6 |
| 10 | `saltbox` | 188 | 0.5 |
| 11 | `quadruple_saltbox` | 118 | 0.3 |
| 12 | `many` | 88 | 0.2 |
| 13 | `double_saltbox` | 48 | 0.1 |
| 14 | `round` | 30 | 0.1 |
| 15 | `shed` | 28 | 0.1 |
| | _other (15 values)_ | 50 | 0.1 |

**Top values of building:colour** (2543 tagged, 134 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `white` | 1154 | 45.4 |
| 2 | `lightyellow` | 76 | 3.0 |
| 3 | `ivory` | 68 | 2.7 |
| 4 | `khaki` | 66 | 2.6 |
| 5 | `#eee888` | 65 | 2.6 |
| 6 | `beige` | 60 | 2.4 |
| 7 | `lemonchiffon` | 55 | 2.2 |
| 8 | `#222222` | 54 | 2.1 |
| 9 | `brown` | 46 | 1.8 |
| 10 | `#ffffdd` | 44 | 1.7 |
| 11 | `antiquewhite` | 43 | 1.7 |
| 12 | `grey` | 43 | 1.7 |
| 13 | `wheat` | 41 | 1.6 |
| 14 | `#cccc88` | 37 | 1.5 |
| 15 | `orange` | 34 | 1.3 |
| | _other (119 values)_ | 657 | 25.8 |

## 4. Trees and forest

**natural=tree nodes**: 23256

| key present | count | % of count |
|---|---:|---:|
| `leaf_type` | 12348 | 53.1 |
| `leaf_cycle` | 4560 | 19.6 |
| `genus` | 726 | 3.1 |
| `species` | 709 | 3.0 |
| `taxon` | 8 | 0.0 |
| `denotation` | 3600 | 15.5 |
| `height` | 538 | 2.3 |

**tree leaf_type** (12348 tagged, 4 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `broadleaved` | 12007 | 97.2 |
| 2 | `needleleaved` | 337 | 2.7 |
| 3 | `mixed` | 3 | 0.0 |
| 4 | `palm` | 1 | 0.0 |

**tree leaf_cycle** (4560 tagged, 5 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `deciduous` | 4304 | 94.4 |
| 2 | `evergreen` | 239 | 5.2 |
| 3 | `semi_deciduous` | 11 | 0.2 |
| 4 | `semi_evergreen` | 5 | 0.1 |
| 5 | `2020 abgestorben` | 1 | 0.0 |

**tree denotation** (3600 tagged, 13 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `urban` | 1850 | 51.4 |
| 2 | `avenue` | 1036 | 28.8 |
| 3 | `agricultural` | 350 | 9.7 |
| 4 | `park` | 161 | 4.5 |
| 5 | `alley` | 120 | 3.3 |
| 6 | `natural_monument` | 32 | 0.9 |
| 7 | `garden` | 17 | 0.5 |
| 8 | `landmark` | 12 | 0.3 |
| 9 | `single` | 9 | 0.3 |
| 10 | `decoration` | 6 | 0.2 |
| 11 | `street` | 5 | 0.1 |
| 12 | `memorial` | 1 | 0.0 |
| 13 | `natural` | 1 | 0.0 |

**natural=tree_row ways**: 1795, 177.788 km

| key present | count | % of count | % of km |
|---|---:|---:|---:|
| `leaf_type` | 1399 | 77.9 | 71.9 |
| `leaf_cycle` | 165 | 9.2 | 10.5 |
| `genus` | 4 | 0.2 | 0.2 |
| `species` | 12 | 0.7 | 0.5 |

**Forest polygons (natural=wood or landuse=forest)**: 2243, 42943.9 ha

| key present | count | % of count | % of area |
|---|---:|---:|---:|
| `leaf_type` | 717 | 32.0 | 6.0 |
| `leaf_cycle` | 101 | 4.5 | 2.7 |

By tag (a polygon with both appears in both rows): natural=wood 1259 (1311.3 ha), landuse=forest 990 (41684.8 ha). Open ways with these tags (not counted): 0.

**forest leaf_type** (717 tagged, 3 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `broadleaved` | 577 | 80.5 |
| 2 | `mixed` | 123 | 17.2 |
| 3 | `needleleaved` | 17 | 2.4 |

**forest leaf_cycle** (101 tagged, 4 distinct values)

| # | value | count | % of tagged |
|---:|---|---:|---:|
| 1 | `deciduous` | 77 | 76.2 |
| 2 | `mixed` | 21 | 20.8 |
| 3 | `evergreen` | 2 | 2.0 |
| 4 | `semi_deciduous` | 1 | 1.0 |
