# Railways: `g2m.rail.geom`

Detail page for the railway layer (railways R1/R2, decision G2M-026). The byte format and the rules are in
[../formats/rail_geom.md](../formats/rail_geom.md); the stage table is section 2.3a of [README.md](README.md).

## Home-region census

`g2m_rail_census` over the Rhein-Main-Hessen store (`rhein-main-hessen-2026-10r1`, immutable), box E 455-490 km,
N 5540-5585 km, 2026-10-20, 1220 `g2m.src.osm` tiles read. A way counts when its first vertex lies in the box.

| Class | Ways | km | Bridges | Bridge km | Tunnels | Tunnel km | Embankment | Cutting | Construction | Electrified |
|---|---|---|---|---|---|---|---|---|---|---|
| rail | 4542 | 1094.2 | 590 | 26.86 | 251 | 61.86 | 472 | 103 | 28 | 3578 |
| light_rail | 564 | 102.0 | 36 | 1.52 | 7 | 0.47 | 16 | 18 | 18 | 564 |
| subway | 198 | 55.9 | 11 | 1.06 | 157 | 50.37 | 0 | 10 | 0 | 198 |
| tram (not served) | 575 | 154.5 | 22 | 2.61 | 58 | 4.91 | 0 | 0 | 2 | 555 |
| all | 5879 | 1406.7 | 659 | 32.06 | 473 | 117.61 | 488 | 131 | 48 | 4895 |

- Railway ways in the 1220 tiles (including the halo ways outside the box): 5913; road ways 102 850.
- Bridge ways 659, connected through shared end nodes into **603 bridge structures**.
- Pieces the layer cuts (512 m): 6817 for all classes, **6088 for the served classes** (tram excluded). The bake of the box
  gives 6060 pieces in 1220 tiles (the tile boundary differs from the box boundary), of which 638 bridge, 485 tunnel and
  **31 declined** (0.5 %, no DEM under a needed sample); 875 of the 1220 tiles hold no railway.
- Plan crossings of a railway segment with a road segment where the two ways share no node: 2905.

| Railway | Road | Crossings |
|---|---|---|
| bridge | ground | 656 |
| bridge | tunnel | 55 |
| ground | bridge | 539 |
| ground | ground | 34 |
| ground | tunnel | 260 |
| tunnel | bridge | 43 |
| tunnel | ground | 1287 |
| tunnel | tunnel | 31 |

- **Rail bridges over roads:** 710 crossings by 481 rail bridge ways (467 of the served classes): 705 with the rail `layer`
  above the road's, 5 where the bridge is tagged but the layer is missing or not above (the road is not a structure).
  The 34 ground-ground crossings without a shared node are level crossings or mapping gaps (not told apart here).
- The nearest rail bridge over a road to the owner spot (E 467577.92, N 5550232.49) is 7 m away: rail way 237156144
  (`railway=construction`, `construction=rail`, `bridge=yes`, layer 1) over the secondary road way 480160094 (layer 0), crossing at E 467573, N about 5550240. The next
  two are 175 m and 400 m away and are under construction too (6 of the 710 crossings are construction bridges; condition 1 in the layer).

CSV of the 710 crossings and the full output: produced by the command below (not committed, it is derived).

```powershell
g2m_rail_census S:\claude_code\geo2map_cache\rhein-main-hessen-r1 rhein-main-hessen-2026-10r1 32 455000 490000 5540000 5585000 rail_bridges_over_roads.csv --spot 467577.92 5550232.49
```

## Bake cost

`g2m_tiler bake --store ... --layers g2m.rail.geom --levels 2 --jobs 8` over the whole region (never into the source
store's own `baked` folder): 2173 tiles with OSM coverage, 1.90 MB of containers in total, 29.4 s wall, 321 MiB peak.
Release id of that rail-only release: `f9af26b80489d3ac69de5a2b1f6809bc5a5818fcf69f062c3b9cc763933381ba` (it holds only
`elev`, `height` and `rail.geom`; a release that also holds the road layers has another id).
