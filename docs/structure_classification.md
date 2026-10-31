# Structure classification (terrain migration phase 7a)

Status: design, 2026-10-20. Measured on the read-only store `rhein-main-hessen-r1` (scope `rhein-main-hessen-2026-10r1`),
home region UTM 32N E 455-490 km, N 5540-5585 km, with `tools/structure_census` (`g2m_structure_census`). Nothing
here is served yet. Figures: [structure_classification_figures.html](structure_classification_figures.html).

Context: `docs/terrain_migration_plan.md` 6.4 (the two carves treat tunnels in opposite ways) and 6.8 (owner answer to
Q6, coordinator decision). Owner, verbatim: "Every bridge and tunnel situation needs to be able to be properly
processed. A tunnel needs a roof, a bridge or a road doesn't need side walls; depending on how the measured data is
in the height map, processing might look different. Decide yourself." Owner on shape, verbatim: "long tunnels with a
lot of terrain above are usually circular, while short tunnels with little height above or "tunnels" that are
classified as such due to a bridge above are usually box-shaped."

Markers: [M] measured here, [C] cited, [AS] assumed, [unverified] not checked.

## 1. Summary

- Every OSM tunnel stretch (road and rail) is cut into **runs**. Every run has a roof; the kinds differ in where the
  roof comes from and whether the terrain stays:
  - **Tube**: real cover. The hill stays; a closed tube (box or bored profile) runs under it.
  - **Deck**: a mapped rail or road bridge above. DGM1 removed it, so the DEM there is an interpolated gap, never
    cover. Box opening, roof = that bridge's deck.
  - **ImpliedDeck**: a ground road or railway above without the cover a tube needs. It stands on a deck OSM did not
    tag (Unterliederbach, A66 service tunnel). Box opening, roof = an implied deck served for the way above.
  - **Slab**: no real cover and no way above. Open cut, roof = a served roof slab, or the building for a building
    passage.
- A chain is **real cover** (only tube runs), **no cover** (no tube run) or **mixed**. A portal sits at each tube end.
- Cover is measured against a **design floor**, not the portal chord and not the DEM. This is the main measured
  finding: against the portal chord, no tunnel in the region has real cover. Frankfurt's 3.5 km City-Tunnel had a
  median cover of 2.4 m, because the city is flat and a straight line between the portals stays near the surface.
  The real floor dives from each portal at the line's grade to the depth the tube needs. With that floor, the
  City-Tunnel is one real-cover box tube.
- The tube shape is a run attribute: **bored** (circular or horseshoe) when the run is at least 300 m long *and* the
  median terrain above the portal chord is at least 15 m. Otherwise a **box**.
- **Profile continuity at every portal and every tunnel/ground join is a hard requirement** (section 5). Cause of the
  launch lip at the NW mouth of way 4812754: the served tunnel floor takes its end height from one DEM sample at the
  portal node, 99.52 m. That is the face of the rail bank. The ground road outside lies at 98.68 m, and its last 6.5 m
  stretch (480160094, stretch 3..4) is declined because it cannot climb the 0.84 m within its grade bound. The cause
  predates D65: the D65-off baseline has the same 99.52 m portal anchor and the same declined stretch.
- Physics needs **no new physics_sim API**. Holes reach the heightfield through the existing `ITerrainSource` fill
  (`kHeightfieldNoCollision`, already passed through by `G2mTerrainSource::fill_from` for NoData). Tubes, portals and
  patches are static `MeshShape` bodies installed by the phase 9 `g2m_ps_bridge` installer in key order. G2M-001
  holds: geo2map core never links physics_sim.
- Proposed vault ticket G2M-029 (section 12).

## 2. Census (deliverable 1)

### 2.1 Method

`g2m_structure_census STORE SCOPE 32 455000 490000 5540000 5585000 --csv --runs --under --portals --dump --spot`
reads every level-2 `g2m.src.osm` tile and the raw `g2m.src.elev.dgm1` tiles in the box. It is read-only and writes
only `out/census/`; a run takes 12.6 s.

- **Inputs** [M]: 1220 src.osm tiles; 102,850 road ways (1735 bridges, 118 cuttings, 1077 tunnels); 5913 railway ways
  (475 tunnels); 396,776 buildings. The DEM is DGM1 at 1 m, bilinear (452 tiles).
- **Chains**: tunnel ways joined end to end like `g2m.rail.geom`'s chain walk. The walk stops at a node where the
  chain does not continue through exactly one other way of its kind. A chain counts when its first vertex is in the
  box: 1344 chains, 5 with no DEM under a sample.
- **Portal or interior end**: a chain end is a portal unless every other way of its kind at that node is also a
  tunnel (a switch or fork underground). 255 chains have at least one interior end.
- **Ground profile**: the lowest DEM across the tube width (centre and the two edges at the way's half width)
  every 1 m.
- **Design floor** (section 3.2) at the chain kind's thresholds; cover = ground - design floor. Natural cover =
  ground - portal chord, used for the shape.
- **What lies above** each station, in priority order:
  - a railway or road way whose footprint (half width) covers the station without sharing a node with the chain;
    bridge-tagged ways first;
  - a building outline (point in polygon);
  - a ground road or railway;
  - another tunnel at a higher layer.
- **Classification**: `tools/structure_census/structure_classify.{h,cpp}`, the pure rule (section 3) that phase 7a-2
  must reproduce. It runs at the nominal thresholds and at six +-50 % variants: cover (clearance and roof), length
  (minimum tube and gap) and grade.
- **Roads under bridges**: every ground road that crosses a bridge in plan without sharing a node. Hump = the highest
  DEM within the bridge footprint minus the chord of the DEM up to 30 m either side (clipped to the lower way).
- **Portal joins**: for every portal end with a ground way of the same kind, the DEM at the portal node minus the
  median DEM 2-8 m out along that ground way.

Unverified simplifications:
- the chain is not the served stretch (`roads.geom` splits stretches at nodes);
- the DEM is raw DGM1, not `terrain.height` L0;
- widths come from the census's class table (`osm_roads.cpp` values) or the `width`/`lanes` tags;
- an interior end is anchored at the needed depth, not on the neighbouring tunnel's floor (the served rule, 3.2,
  solves the whole network).

### 2.2 Counts per class (nominal thresholds)

| Kind | Class | Chains | km |
|---|---|---|---|
| rail | real cover | 96 | 41.92 |
| rail | mixed | 74 | 62.82 |
| rail | no cover | 140 | 12.87 |
| road | real cover | 18 | 1.34 |
| road | mixed | 63 | 12.53 |
| road | no cover | 948 | 20.34 |

Runs (nominal) [M]:

| Kind | Run | Runs | km |
|---|---|---|---|
| rail | tube box | 210 | 92.81 |
| rail | tube bored | 5 | 4.18 |
| rail | deck | 55 | 0.53 |
| rail | implied deck | 164 | 2.73 |
| rail | slab | 387 | 17.00 |
| road | tube box | 112 | 8.86 |
| road | tube bored | 0 | 0 |
| road | deck | 218 | 1.73 |
| road | implied deck | 220 | 2.05 |
| road | slab | 1396 | 21.16 |

What dominates each chain (most stations, `Nothing` only when alone) [M]:
- **Road: buildings dominate.** 768 of 1029 road chains (19.3 km) lie mostly under a building, almost all building
  passages (647 chains tagged `tunnel=building_passage`), and 737 of them have no cover.
- **Ground road above**: 78 road chains (6.4 km).
- **Road bridge above**: 29 road chains (2.6 km).
- **Rail**: a ground road above dominates 118 chains (87.2 km), the city and S-Bahn tunnels under streets. A building
  above dominates 138 chains (22.7 km).
- **Ground road above with less cover than clearance + roof**: 153 chains have at least one such station. Each such
  station is a conflict: today's D65 exclusion leaves those tunnels uncut, and they need an implied deck.

Sensitivity (chains real / mixed / no cover) [M]:

| Variant | Rail | Road |
|---|---|---|
| nominal | 96 / 74 / 140 | 18 / 63 / 948 |
| cover x0.5 | 96 / 93 / 121 | 18 / 126 / 885 |
| cover x1.5 | 96 / 58 / 156 | 18 / 38 / 973 |
| length x0.5 | 97 / 74 / 139 | 18 / 74 / 937 |
| length x1.5 | 96 / 74 / 140 | 18 / 56 / 955 |
| grade x0.5 | 96 / 61 / 153 | 18 / 57 / 954 |
| grade x1.5 | 96 / 83 / 131 | 18 / 78 / 933 |

Reading the sensitivity:
- **Real-cover chains do not move.** They are deep or wholly between interior ends.
- **Mixed and no-cover trade with cover and grade**, through the portal-hood lengths: a hood is needed / grade long,
  114 m for a road at 5 %, 175 m for main-line rail at 4 %.
- **The length variants barely matter.**
- **None of the variants changes a named site's class** (2.5), except the A66 service tunnel at cover x0.5, which
  becomes mixed.

Tube runs by length and median natural cover (ground - portal chord), the box/bored split [M]:

| Length \ natural cover | 0+ | 8+ | 10+ | 15+ | 20+ | 30+ |
|---|---|---|---|---|---|---|
| 0+ m | 64 | 6 | 5 | 0 | 0 | 0 |
| 50+ m | 25 | 2 | 0 | 0 | 0 | 0 |
| 100+ m | 16 | 3 | 2 | 0 | 0 | 0 |
| 200+ m | 13 | 0 | 0 | 0 | 0 | 0 |
| 300+ m | 15 | 2 | 2 | 0 | 2 | 0 |
| 500+ m | 8 | 0 | 0 | 0 | 0 | 0 |
| 1000+ m | 10 | 1 | 2 | 1 | 1 | 1 |

### 2.3 Roads under bridges and side walls [M]

Ground roads crossing under a bridge (crossings; DEM hump >= 1 / 3 / 5 m):

| Upper | Crossings | >= 1 m | >= 3 m | >= 5 m |
|---|---|---|---|---|
| rail bridge | 522 | 81 | 61 | 30 |
| rail bridge (construction) | 5 | 0 | 0 | 0 |
| road bridge | 1166 | 174 | 80 | 29 |

Side walls in today's geometry:
- **Bridges have none.** A deck is a 0.5 m slab with fascia and no parapet, wall or pillar (`road_structures.cpp:103`).
  Nothing to remove.
- **Ground roads in cuttings** (118 `cutting=*` ways) get the carve's graded blend, not walls. Nothing to remove.
- **D59 steep walls** apply only past an underpass deck's end planes (`terrain_carve.cpp:847`). Those decks come from
  road bridges in `roads.geom`. The 174 road-bridge crossings with a hump of at least 1 m are the candidate set. In the
  new rule these walls are the sides of a Deck run's box opening, so they stay.
- **Rail-bridge humps are not masked at all today.** D64 reads only road bridges, because rail bridges are not in
  `roads.geom`. 81 ground roads keep a hump of at least 1 m under a rail bridge, 30 of them at least 5 m. Phase 7a-3
  extends D64 to rail bridges.
- **The game carve cuts every tunnel** (`road_surface.cpp:46`): all 1029 road chains (34.2 km) are trenches with
  terrain walls today. Under the new rule only the non-tube road runs are cut (24.9 km). The 8.9 km of road tube and,
  once rail carving exists, the 97 km of rail tube keep their hill.
- **D65-like served cuts**: 124 road chains are at most 80 m long with a hump of 1-8 m. 38 of them have a ground road
  above, so the `crossed_by_ground_road` exclusion keeps them uncut. In the new rule they become implied decks.

### 2.4 Portal joins [M]

|DEM at the portal node - median DEM 2-8 m out along the ground way|, portals with a joining ground way of the same kind:

| Kind | < 0.1 m | 0.1-0.3 | 0.3-0.5 | 0.5-1 | 1-2 | >= 2 m |
|---|---|---|---|---|---|---|
| rail | 175 | 25 | 6 | 10 | 14 | 39 |
| road | 1011 | 456 | 86 | 59 | 24 | 44 |

- **190 of 1949 portals** (156 positive, node higher) differ by at least 0.5 m. The served tunnel floor anchors at the
  node sample (verified at 4812754: 99.52 m on both). So at each of these portals either the ground road ramps up to
  the floor end, or, where its last stretch is declined (as at 4812754), the carve leaves a step.
- **Up to 8.2 m** on rail (NBS portals near Breckenheim, way 1313342266), where the node sits on the hill face over a
  trough. **6.7 m** at the Mönchwaldtunnel's north portal (road 32145483).
- The served profile at the Mönchwaldtunnel could not be dumped: `roads.graph` fails there with "restriction via ways"
  [M; a separate defect].

### 2.5 Named sites

| # | Site | Way(s) | UTM 32N E / N | Length | Measured | Class / runs (nominal) |
|---|---|---|---|---|---|---|
| 1 | Unterliederbach, Königsteiner Straße under the rail sidings (embankment, no bridge tag) | 4812754 | 467596 / 5550198 | 46.3 m | median cover 1.4 m, max ground 104.0 over a floor 98.7-99.1 | no cover: slab 0-3, implied deck 4-30, slab 31-36, implied deck 37-45, slab 46-47; NW portal join step +0.84 m |
| 2 | Königsteiner Straße under the construction rail bridge (owner site) | 480160094 under 237156144 | crossing 467572.7 / 5550237.3 | - | hump -0.36 m (no DEM remnant) | ground road: no structure, no walls; the bridge deck comes from rail.geom (construction) |
| 3 | A66 service tunnel under both carriageways | 52405506 under 24504840, 313879329 | 466061 / 5551241 | 43.8 m | median cover 3.2 m, 27 conflict stations | no cover: slab, implied deck 6-19, slab 20-25 (median strip), implied deck 26-38, slab |
| 4 | City-Tunnel Frankfurt (S-Bahn) | 60668150 | 477476 / 5551430 | 3530 m | natural cover 2.4 m, design cover 7.0-11.8 m; both ends interior | real cover: one box tube |
| 5 | Hasselborner Tunnel (rail) | 15964594 | 463483 / 5582998 | 1299 m | natural cover 61 m | mixed: slab 0-2, bored tube 3-1295, slab; portal steps 0.24 / 1.67 m |
| 6 | Wandersmann Nord (NBS, both tubes) | 51763073, 27050427 | 455727 / 5546020 | 1144, 1086 m | natural cover 26.5, 18.8 m | mixed (2 m hoods), bored |
| 7 | Tunnel Breckenheim (NBS) | 157604167 | 454681 / 5547415 | 1153 m | natural cover 9.7 m | mixed, box (near the threshold) |
| 8 | Mönchwaldtunnel (road) | 32145483 | 465256 / 5543302 | 582 m | natural cover 5.4 m | mixed: hood and implied decks 0-113, box tube 114-581; N portal step +6.7 m |
| 9 | Building passage (longest no-cover one) | 554045663 | 474902 / 5551245 | 132 m | cover 0.2 m | no cover: one slab, roof = building 1393593022 |
| 10 | Track under four separate road bridges | 1508068478 under 4784443-6 | 462541 / 5541720 | 83 m | cover about 0 | no cover: four deck runs with 9-11 m slab gaps |
| 11 | C-Strecke (U-Bahn) | 60697815 | 479288 / 5551842 | 2344 m | natural cover 10.4 m | mixed, box |
| 12 | Eppsteiner Tunnel (rail, both tubes) | 207289303, 218440014 | 455959 / 5554508 | 335, 332 m | natural cover 28-30 m | bored |

The tunnel names are the OSM `tunnel:name` / `name` tags. Whether each real structure is mined or cut-and-cover is
[unverified] for every row; the rule is checked against the owner's description, not against construction records.

## 3. The rule and its thresholds

### 3.1 Per station

For station i (1 m spacing along the tunnel axis) the inputs are the cover (ground minus design floor) and what lies
above.

1. **A mapped bridge above** (rail or road, footprint over the station) gives `Deck`, whatever the DEM says. DGM1
   removed that bridge and interpolated the gap.
2. **Else, cover >= clearance + roof** (1e-6 m tolerance) gives `Tube`. NaN (no DEM) is never a tube.
3. **Else, a ground road or railway above** gives `ImpliedDeck`. For a road above, this is also counted as a
   conflict.
4. **Else** `Slab`.

Then, over the runs:
- **(a) Gaps inside a tube**: an interior Slab or ImpliedDeck run between two tube runs, whose span from the last tube
  station before it to the first tube station after it is shorter than `min_gap_m`, becomes tube with an *exposed
  roof* (the way above stands on the tube roof).
- **(b) Short tubes**: tube runs shorter than `min_tube_m` take their stations' open kind back.
- **Deck runs never change.**
- **Class**: no tube run = no cover; tube runs only = real cover; else mixed.
- **Shape**: Bored when the run is at least `bored_min_length_m` long and the median natural cover over the run is at
  least `bored_min_cover_m`; else Box.

### 3.2 The design floor

The floor is the highest profile that meets all of:
- (a) it never rises above the portal chord;
- (b) at every station at least needed / max_grade from each portal, it stays at least needed = clearance + roof
  below the ground;
- (c) its grade never exceeds `max_grade`;
- it meets each portal anchor exactly.

How it is computed:
- the upper bound u = min(chord, ground - needed), or just the chord nearer a portal;
- a two-pass 1-D distance transform gives the greatest `max_grade`-Lipschitz profile below u;
- that profile is raised to the portal cones z_portal - max_grade * distance.

`design_floor` in `structure_classify.cpp` does this with only + - min max: deterministic and libm-free.

Why each part:
- **The chord bound** keeps hill tunnels on their straight line (D52). In a hill, u is the chord everywhere and the
  floor is the chord.
- **The needed-depth bound** is what makes city tunnels tubes. On flat ground the floor drops from each portal at
  `max_grade` to the needed depth and follows the ground: Frankfurt's City-Tunnel, the U-Bahn and the airport
  tunnels.
- **The portal reach** (no depth requirement within needed / grade of a portal) keeps hill tunnels free of a sag. The
  ground falls to a portal face faster than any floor may follow; without the reach, every hill tunnel would dip by
  up to 5.7 m near each portal. The unit test "hill: the design floor is the portal chord" is the guard.
- **Portal cones** keep a valley just past the reach from pulling the portal end down. Without them, the "valley"
  check of the continuity test leaves both portals 28 m low: sabotage S2.
- **Interior ends** (inside a tunnel network) are no portals. The census anchors them at the needed depth below the
  ground. The served rule (phase 7a-2) instead solves the floor over each connected tunnel component: the same
  envelope on the graph (Dijkstra-like relaxation over the component's stations, processed in ascending
  (way id, stretch, station) order). One floor value at every shared node, with only the real portals as anchors.

What changes in the served floor:
- D47's DEM-following floor below 200 m is replaced; that floor made cover meaningless, because cover against it is
  about 0 everywhere.
- D52's chord stays as the hill case.
- D65's "short straight cut under a bank" is no longer a special case: such a stretch is a run of kind ImpliedDeck or
  Slab on the chord floor, cut by the carve because it is not a tube, not because it is shorter than 80 m.
- The D47/D52/D65 grade bound of 30 % is replaced by `max_grade` (5 % road, 4 % rail, 5 % other rail classes).

### 3.3 Thresholds (proposed `data/road_rules/de.json` block)

```json
"structure": {
  "clearance_mm": { "road": 4700, "rail": 6000, "subway": 4500, "light_rail": 4500, "tram": 4500, "narrow_gauge": 4500 },
  "roof_mm": 1000,
  "max_grade_per_mille": { "road": 50, "rail": 40, "subway": 50, "light_rail": 50, "tram": 50, "narrow_gauge": 50 },
  "min_tube_m": 10,
  "min_gap_m": 10,
  "bored_min_length_m": 300,
  "bored_min_cover_m": 15,
  "box_wall_mm": 800,
  "bored_lining_mm": 500,
  "box_side_margin_mm": 1000,
  "bored_segments": 32
}
```

The reasons (each value is a starting point, [AS] unless marked):

| Value | Reason |
|---|---|
| clearance road 4.70 m | German standard free height 4.50 m [C: RAA/RAL clearance, from memory, unverified] + 0.20 m reserve. OSM `maxheight` is not used: it is a sign value (3.8 m at site 1) and would let the census call a 3.8 m underpass a tube where the structure is thicker. |
| clearance rail 6.00 m | Electrified main-line clearance (contact wire about 5.5 m plus the structure gauge) [AS, unverified]. Subway and light rail 4.50 m [AS]. |
| roof 1.0 m | A plausible cut-and-cover roof slab; the box opening's deck is 0.5 m (`road_structures.cpp`), the tube's structural roof is thicker [AS]. |
| max grade road 5 % | Tunnel longitudinal grade limit [C: RABT recommendation of at most 5 %, unverified] |
| max grade rail 4 % | The NBS Köln-Rhein/Main and S-Bahn ramps [C: from memory, unverified] |
| min tube 10 m | Below this a tube between two cuts is one slab span (the census length variants move 1-18 chains) [M]. |
| min gap 10 m | A cut gap shorter than about one tube width is a bridged-over slot, not an open cut [AS]. |
| bored >= 300 m and >= 15 m natural cover | From the histogram (2.2): of 41 tube runs of at least 300 m, 33 have less than 8 m of terrain above the portal chord. These are city and airport tunnels: box, cut-and-cover [unverified per structure]. 5 have at least 15 m: Hasselborner, Wandersmann Nord x2, Eppsteiner x2, all mined tunnels by name and setting [unverified]. The 4 runs at 10-15 m (Flughafenschleife x2, C-Strecke x2) and the 4 at 8-10 m (Tunnel Breckenheim, Kelsterbacher Spange x2) stay boxes. 15 m is also about 1.5 bore diameters, where mined construction starts to beat cut-and-cover [AS]. At 10 m, 4 more runs would turn bored. |
| box wall 0.8 m, lining 0.5 m, side margin 1.0 m, 32 segments | Geometry only, see 4 [AS]. |

Release-id cost of the block: the de.json sha is in the `roads.geom` config text, so editing de.json changes every
release with roads.geom (and with carved and class through it). The block is therefore **not** committed in this
step. It lands with phase 7a-2, batched into bump #3. de.json must be committed with LF line endings; its pinned sha
is the LF one.

## 4. Geometry per run and shape

All dimensions are in the run's local frame: s along the axis, y across (left positive), z up from the floor (road
surface). W = carriageway width (2 x half width), H = clearance.

### 4.1 Box (cut-and-cover tube, underpass)

```
          terrain (stays over a Tube run; cut and replaced by the deck or slab over the other kinds)
   ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
        +--------------------------------+   roof: 1.0 m (Tube), deck 0.5 m (Deck/ImpliedDeck), slab 0.5 m (Slab)
        |#|                            |#|
        |#|          H = 4.70 m        |#|   walls 0.8 m
        |#|                            |#|
        |#|____________________________|#|   floor = road surface (the design floor)
          <-- 1.0 -->|<-- W -->|<-- 1.0 -->
```

- **Inner width** W + 2 x side margin, **inner height** H.
- **Collision**: floor (the existing tunnel-floor RoadDeck surface), both walls and the roof underside.
- **Render**: the inner faces, plus the outer roof top where it is exposed.

### 4.2 Bored (mined: circular / horseshoe)

```
                 .-~~~~~~-.
              .'            '.       circle through the four corners of the clearance rectangle
             /                \      centre at H/2 above the floor, R = sqrt((W/2 + m)^2 + (H/2)^2)
            |    clearance     |     road 2 lanes: W 7.5 m, m 1 m, H 4.7 m -> R 5.3 m
            |    rectangle     |     rail 2 tracks: W 9.5 m, H 6.0 m -> R 5.6 m
             \________________/      the floor chord closes the circle (horseshoe: flat invert fill)
```

- **Inner surface**: a circle through the four corners of the clearance rectangle, centre H/2 above the floor,
  radius R = sqrt((W/2 + side margin)^2 + (H/2)^2). Below the road surface the circle is filled (invert): the floor
  chord closes it.
- **Tessellation**: `bored_segments` = 32 over the full circle, so the chord deviation is R(1 - cos(5.625 deg)),
  2.6 cm at R 5.3 m. The vertices use one fixed table of 32 (cos, sin) pairs, written as literals like
  `kHeightfieldYupToZup` (no runtime trig, D7).
- **Collision**: floor plus every arc segment above it. The walls and the roof are one curved surface; a car rolled
  onto its side or roof touches it.
- **Size check**: a 2-lane road tunnel bore of about 10.6 m and a double-track NBS tube of about 11-12 m match common
  practice [unverified].

### 4.3 Box opening under a deck (Deck, ImpliedDeck)

```
   rail/road on its deck (mapped bridge, or the implied deck served for a ground way above)
   ====================================================  deck top = the upper way's profile
   |----------------- deck slab 0.5 m ----------------|  the deck IS the roof: no tube roof
        |#|                                  |#|
        |#|    cut down to the floor         |#|   box sides = abutments, steep (D59) inside the
        |#|                                  |#|   deck footprint only
   _____|#|__________________________________|#|_____ floor
  graded cut slope beyond the deck footprint (no walls), as the carve does today
```

- **The top of the opening is the upper way's deck.** For a Deck run it is the bridge's own deck (`roads.geom`
  bridge or `rail.geom` piece). For an ImpliedDeck run, geo2map serves an inferred deck for the ground way above. It
  generalises today's inferred roofs over lower tunnels (`road_structures.h`, span max(20, (w/2 + 4) / sin) m,
  currently upper road rank >= 5 only) to every rank and to railways.
- **The opening's sides** are the box walls within the deck's footprint. Outside it the cut keeps graded slopes (D59
  stays as it is).
- **The upper way's ground profile** is masked over the opening like D54/D64, so it does not ride the DEM bank.

### 4.4 Slab (open cut with a served roof)

- **The cut is carved to the floor; a 0.5 m roof slab spans the box opening** at floor + H.
- **Building passages** (`Above::Building`): no served slab. The roof is the building from `BuildingSet` (phase 14).
  The building mesh needs an opening where the passage runs; that is out of this step's scope (open question 11.4).
- **At a chain end** (portal hood) the slab runs from the OSM tunnel end to the first tube station, so the box
  continues until the hill takes over.

### 4.5 Portals

A portal sits at every Tube run end that faces an open run or a chain end. Interior chain ends inside a tunnel
network are no portals: the census counts 2 per tube run, which over-counts the 255 chains with interior ends.

| Tube shape | Portal |
|---|---|
| Box | **Headwall**: the box end face extended up to the terrain line, 0.8 m thick. Two **wing walls** along the cut slopes at 45 deg in plan, each running down to the slope line. |
| Bored | **Headwall with a circular opening**: the face carries the bored circle. Above it the headwall extends to the terrain line, and the wing walls are as for a box. Where a Slab hood precedes the bored tube, the box hood meets the circle inside the headwall: the face closes the gap between the hood's rectangle and the circle. |
| Box opening (Deck, ImpliedDeck) | No headwall: the deck's fascia is the face. The box sides end at the deck's end planes (D59 steep walls). |

At a shape change between two tube runs (box to bored, rare) there is a **transition ring** instead of a portal.

### 4.6 Exposed roof and roads above a tube

Where (a) of 3.1 bridged a short gap, the tube's roof top stands at or above the terrain. Its top surface is served
as a walkable patch, so the way above crosses on the roof, and the terrain is holed where it intrudes into the tube
envelope (5.3).

A road above a real-cover tube needs nothing: it rides the hill. The A66 case (site 3) is not a tube: the carriageways
become implied decks over the cut. This replaces the D65 `crossed_by_ground_road` exclusion, which left that tunnel
uncut under the road slot.

## 5. Continuity (hard requirement)

**Requirement.** At every node shared by a tunnel floor and a ground profile, and at every node shared by two tunnel
floors:
- the heights are equal within 0.01 m;
- the grades are equal within 0.02;
- no ground stretch ending at a portal is declined;
- the served carved terrain along the road axis across the join steps by at most 0.10 m per metre more than the
  profile itself.

**Cause at 4812754** [M]: `road_completion.cpp` step 2a builds tunnel floors from `dem.sample` at the stretch ends.
At the NW portal node 30906085 (467585.07 / 5550217.80) that sample is the bank face, 99.52 m. Values along the line:

| Point | z (m) | Source |
|---|---|---|
| ground road 480160094 at s = 31, 6.5 m out (467581.0 / 5550224.1) | 98.69 | served strict profile, stretch 0..3 |
| DEM over the last 6.5 m | 98.64-98.81 | |
| DEM 0.6 m from the node | 98.81 | |
| DEM at the node | 99.52 | D65 cut chord, 99.52 -> 99.10 |

- **The last ground stretch 3..4** (from node 1686121536, 6.5 m) is DECLINED, "estimated endpoint slope exceeds class
  bound". It has no profile, so the carve under it follows the tunnel cut's core: 98.66 -> 99.10 -> 99.46 -> 99.50
  over 3 m.
- **This matches the game** to within its sampling: 98.51-98.71 at s = -6 to -2, 98.92-99.03 at s = -1,
  99.34-99.46 at s = 0.
- **The baseline with D64 and D65 off** has the same 99.52 m portal anchor and the same declined stretch. Only the
  floor inside differs: it follows the DEM up the bank. The lip predates D65.
- **The census portal-join table (2.4)** shows the same mechanism at 190 portals (at least 0.5 m).

**Fix (phase 7a-1):** the ground side owns the join.
- **Order**: ground profiles are fitted first. A tunnel floor's portal anchor is the adjacent ground profile's end
  height and grade at the shared node (the D47 bridge rule, `end_plane`, extended to tunnels).
- **No ground profile at the node** (declined, or none of the same kind): the anchor is the approach height, the
  median DEM 2-8 m out along the joining ground way, with that way's measured grade. The declined ground stretch is
  then refitted to end on the tunnel's portal plane (step 3 `plane_at` gains the accepted tunnel plane, as it has the
  bridge plane).
- **Tunnel/tunnel nodes** take one floor value from the component solve (3.2).
- **Rail** (`rail.geom` v1, chains linear between ground anchors) gets the same anchor rule.

**Test** (`test_portal_continuity`, tags `[roads][structure]`, geo2map):
- on the corridor fixture tiles plus the 4812754 fixture tile (`utm32n/2/456/5420`), for every TunnelFloor entry and
  every node it shares, |dz| <= 0.01 m and |dgrade| <= 0.02;
- no declined ground stretch at a portal node;
- carved L0 along the axis +-8 m around each portal: max |dz per 1 m| - the profile's own <= 0.10 m;
- **sabotage A** (portal anchor = `dem.sample` at the node, today's code): fails at 4812754 NW by 0.83 m;
- **sabotage B** (ground stretch not refitted to the tunnel plane): fails on the declined-stretch check at the same
  node.

The design-level guard exists already: `test_structure_classify.cpp` "the design floor meets every portal anchor
exactly and never exceeds the grade" (bit-equal ends; sabotage S2 recorded). The Unterliederbach case checks that
anchoring on the approach (98.68 m) leaves no step where the node sample (99.52 m) leaves 0.84 m.

## 6. What geo2map serves per class

| Item | Format | Change | Release id |
|---|---|---|---|
| Design floor, runs, shape, roof source | `g2m.roads.geom` | encoding 10: each tunnel entry gains a structure section, one record per run: kind u8, s_start/s_end (mm), shape u8, inner width/height or radius (mm), exposed roof flag, portal flags (start/end), roof source way id (Deck/ImpliedDeck) | bump (batched with #3) |
| Implied decks | `g2m.roads.geom` (road above) and `g2m.rail.geom` (rail above) | inferred RoadDeck span records generalised to every rank and to rail pieces | same bump |
| Rail runs and portal records | `g2m.rail.geom` | encoding 2: the same structure section per tunnel chain; rail decks unchanged | opt-in layer; changes only releases that opt in |
| Thresholds | `data/road_rules/de.json` `structure` block | new block (3.3) | sha in the roads.geom config: same bump |
| Carve | `g2m.terrain.carved` | carves non-tube runs to the floor, never tube runs; D65 tunnel-cut and the crossing exclusion replaced; D64 reads rail bridges; config text `structure=7a/1` replaces `tunnel-cut=D65/80` | same bump |
| Classes | `g2m.terrain.class` | new role: portal apron (render material at portal faces); hole samples are not a class (they come from geometry, 7) | same bump |

All served changes land together in bump #3 (phase 7 is already that bump). There is no bump of its own for 7a.
Tube, portal, slab and patch **meshes** are not served: g2m_world derives them client-side from the served runs, like
RoadDeck meshes today. Mesh changes therefore cost no release id.

## 7. Physics

### 7.1 Holes (via `g2m_ps_bridge`, phase 9 seam)

- **Where they come from**: `G2mTerrainSource::fill_from` already calls `fill_physics_heights(..., ps::kHeightfieldNoCollision)`
  and writes NoData as a heightfield hole. physics_sim's `TileManager` keeps source holes through the per-tile datum
  (`tile_manager.cpp:679/693` skip them) and its own suppression (PHYS-021) stays off (`make_terrain_config`). **No
  physics_sim API change.**
- **Hole set**: a lattice vertex of the 255 m node-registered physics grid (L0 cell centres, anti-diagonal split shared
  with render, G2M-025) is a hole when one of its incident triangles intersects a structure **envelope**: the tube's
  outer prism (inner profile + wall/lining + 0.3 m margin) of a Tube run, the box opening of an open run above the
  floor, or a portal's headwall slab.
- **Deep tubes need no holes.** The hill surface lies above the envelope and keeps colliding; the car below never
  reaches it. Holes appear only at portals, exposed roofs and open runs whose carve is not yet down to the floor.
- **Jolt and back faces** [unverified, to be measured in 7a-5]: a body inside a tube far below the heightfield is
  assumed to get no contact from it. Jolt ignores back faces in the collide queries this backend uses
  (`CLAUDE.md`, "Jolt backend"); for simulation contacts against heightfield triangles a body that does not
  intersect the surface gets none either way.
- **Patch mesh**: a hole vertex disables every incident triangle (Jolt `cNoCollisionValue`), which removes more
  terrain than the envelope covers. Each dropped triangle is re-emitted as patch geometry with its own vertex heights,
  bit-equal to the heightfield, clipped to the outside of the envelope (Sutherland-Hodgman on the envelope's planes).
  Each tile has one static `MeshShape` per structure.
- **Determinism**: the hole set and patch are a pure function of (tile key, structure runs). Holes are written in
  ascending sample index, which keeps `TileManager` prefetch inputs Immutable/Snapshot-valid.

### 7.2 Tube, portal and slab bodies

- **Collision follows the profile**:
  - box: floor, two walls, roof underside;
  - bored: floor plus the arc segments;
  - portals: headwall and wing walls;
  - roof slab: its underside and top;
  - exposed roof: its top.
- **Installation**: through the phase 9 `DeckInstaller`, generalised to a `StructureInstaller`.
  `World::create_shape` builds the shapes off the sim thread (R2 prebuilt shapes, any thread);
  `World::create_body(desc, handle)` installs them on the sim thread between steps.
- **Order**: by `StructureKey{way_id, start_ref, end_ref, run_index, part}`, part = floor/walls/roof/portal/patch,
  ascending, in a `std::map` or a sorted vector. Install order moves BodyIds and the state hash.
- **Mesh size**: a tube MeshShape is split every 64 m along s, which keeps it well under Jolt's per-mesh limits and
  the local-material list of 32 [AS].

### 7.3 Determinism and ordering

- **PHYS-026**: no unordered container is iterated for anything that decides holes, patches, bodies or install order.
  The census and the rule library use `std::map`/`std::set`/sorted vectors only.
- **The component floor solve** (3.2) visits nodes in ascending (way id, stretch, station) order. The relaxation of a
  Lipschitz envelope converges to the same values in any order, but the order is fixed anyway so the float rounding
  is reproducible.
- **Arithmetic**: + - * / and comparisons; the bored circle uses the literal table (4.2).
- **The physics_sim state hash** changes only through installed bodies (BodyIds, shapes) and heightfield samples. Both
  are pure functions of served data and install order. The 7a-5 tests pin the hash at 1 and 10 workers.

## 8. Render

- **Terrain holes** use the same lattice and the same hole rule as physics, so render and physics agree
  (`test_render_vs_jolt` gains a holed tile).
- **The tube interior** is rendered from the same meshes (inner faces, normals inward), with the portal faces and wing
  walls. The terrain patch is rendered from the patch mesh.
- **Exposed roof tops** and **roof slabs** are rendered top and bottom.
- **Building passages** keep the building mesh. Its opening is phase 14's problem (11.4).
- **The game** (racing_game, phase 7a-6) drops `road_surface.cpp:46`'s tunnel admission with the phase 7 carve switch,
  so tube runs are no longer trenches.

## 9. Interaction with existing decisions

- **D47** (tunnel floors from the DEM): replaced by the design floor (3.2). Its portal anchor is the defect of section
  5.
- **D52** (chord at 200 m or more): kept as the hill case of the design floor; the 200 m threshold is gone, because
  the cover decides.
- **D53-D58** (underpass decks for a road over a lower road in a DEM cutting): kept. They are the ground-over-cut case;
  ImpliedDeck is the opposite (ground over a bank). Both give the lower road a box opening under a deck.
- **D59** (steep walls past an underpass deck's end planes): kept, inside a deck footprint only. They become the box
  sides of Deck and ImpliedDeck runs.
- **D64** (hump mask under overpasses): extended to rail bridges, read from `rail.geom` pieces (81 masked crossings
  of at least 1 m, 2.3).
- **D65** (short straight cut, 80 m bound, crossing exclusion): subsumed. Cut where not a tube; the crossing becomes an
  implied deck.
- **"Lowest core wins"** in the served carve: unchanged for ground and open runs. A Tube run contributes no core, so
  it never lowers the terrain. An open run's core is its floor within the box width and the graded slope beyond the
  roof footprint.
- **Junctions (D42)**: a junction at a portal node (a tunnel mouth with an on-ramp) takes the ground profile's height
  by the continuity rule (5). The junction surface stays outside the tube envelope; a junction inside a tube (an
  underground fork) is a tunnel/tunnel node and takes the component floor.
- **G2M-025** (shared lattice): the holes use it, 7.1.
- **G2M-001**: holes and installation live in `g2m_ps_bridge`; g2m core and g2m_world never include physics_sim.

## 10. Phases

| Phase | What | Size | Release id | Repos | Depends on |
|---|---|---|---|---|---|
| 7a-0 | census + rule library + this design | S | none | geo2map | - |
| 7a-1 | portal and join continuity | M | roads.geom bump (batched #3) | geo2map | - |
| 7a-2 | design floor, component solve, runs served (roads.geom enc 10, rail.geom enc 2, de.json `structure`) | L | bump (batched #3) | geo2map | 7a-1 |
| 7a-3 | carve: non-tube runs cut, D65 replaced, D64 on rail bridges, implied decks served | M | bump (batched #3) | geo2map | 7a-2 |
| 7a-4 | structure meshes in g2m_world: box, bored, portals, slabs, patches, exposed roofs | L | none | geo2map | 7a-2 |
| 7a-5 | physics holes + StructureInstaller in g2m_ps_bridge | M | none | geo2map (bridge), racing_game (call site) | 9, 7a-4 |
| 7a-6 | game: render holes and tubes, drop the tunnel admission in the game carve (with phase 7) | M | (phase 7's bump) | racing_game | 7a-3..5, phase 7 |

### 7a-0 (this step)

- **Delivered**: `tools/structure_census`, `tests/unit/test_structure_classify.cpp` (12 cases, `[structure]`), this
  document and the figures.
- **Recorded sabotages S1-S4** (test file header):
  - S1 (no bridge check): the bridge case fails, 3 assertions;
  - S2 (no portal cones): the continuity case fails, with portals at 71.8 m instead of 100 m, and so does the
    interior-end case;
  - S3 (bored rule on the design cover): the shape case fails;
  - S4 (gap `>=` as `>`): the gap case fails at a 10 m span.

### 7a-1 portal and join continuity

- **What**: section 5.
- **Tests**: `test_portal_continuity` with sabotages A and B. Roadcmp census before and after: the number of declined
  ground stretches at portal nodes must fall to 0. Today it is at least the 4812754 one; to be counted with
  `g2m_spot_dump --census` in the phase.
- **Risks**:
  - it moves every tunnel floor end (1949 portals with a ground join; 190 move at least 0.5 m);
  - the approach estimate can land in a trough bottom where the real portal is higher (rail portals at +6-8 m). The
    rule takes the ground side's level by design: the portal is where the road is.

### 7a-2 design floor and runs

- **What**:
  - `structure_classify` moves from the tool into `src/layer` as the served rule; the tool links it from there;
  - the design floor replaces D47/D52/D65 in step 2a;
  - the component solve;
  - the encodings and the de.json block.
- **Tests**:
  - parity: for the fixture tiles, the served runs equal `classify(design_floor(...))` station for station;
  - the census rerun on the home store gives the counts of 2.2 within the served-vs-raw-DEM noise (tolerance to be
    measured);
  - the component floor is equal at every shared node (sabotage: solve chains separately; fails at the City-Tunnel
    switches);
  - golden bytes for one tunnel tile;
  - tile invariance (a tunnel crossing a tile edge serves the same runs in both tiles).
- **Risks**:
  - the serving cost of the component solve over long networks (the City-Tunnel component spans several tiles; the
    solve must be a function of the component, not the tile, so a dependency walk like `road_dem_dependencies` is
    needed);
  - the encoding change touches every roads.geom reader.

### 7a-3 carve

- **What**: the served carve cuts open runs; ImpliedDeck masks the upper way; D64 on rail.geom pieces.
- **Tests**:
  - carved L0 at sites 1, 3 and 10: floor within 0.02 m under the box, deck top equal to the upper way's profile;
  - at site 4 (City-Tunnel), the carve leaves the surface unchanged (Tube; sabotage: carving tube runs changes
    48,000+ samples);
  - D64 rail: the 30 crossings with a hump of at least 5 m are flat within 0.1 m under the rail deck (sabotage: road
    bridges only).
- **Risks**: lowest-core-wins at a portal where a ground core and a tube floor meet (continuity, 7a-1, must hold
  first).

### 7a-4 structure meshes

- **What**: `StructureSet` product in g2m_world (box, bored, portals, slab, exposed roof, patch).
- **Tests**:
  - golden mesh digests for sites 1, 4, 5 and 9;
  - a bored mesh's vertex table equals the literal table (no trig);
  - the patch plus the remaining heightfield triangles cover the original surface outside the envelope exactly
    (area and vertex equality);
  - watertight tube (every interior edge shared by two triangles).
  - Sabotages: off-by-one segment; a patch not clipped.
- **Risks**: the mesh count over the 97 km of rail tube (memory per tile) [unverified].

### 7a-5 physics

- **What**: holes in `G2mTerrainSource`; `StructureInstaller` (phase 9's DeckInstaller generalised).
- **Tests**:
  - the hole set is a pure function: 1 vs 10 workers, cold vs prefetch, equal sample indices;
  - `World::state_hash` equal at 1 and 10 workers over a scripted 30 s drive through site 1 and site 5's south portal;
  - drive-through probe: chassis vertical acceleration at the portal within 2 x the same drive on a flat reference;
    no contact with heightfield bodies inside a tube;
  - `test_render_vs_jolt` with a holed tile;
  - sabotages: holes not written (the car hits the hill face at site 5); install order reversed (hash differs).
- **Risks**:
  - the Jolt one-sidedness assumption (7.1);
  - linux-tsan coverage for the installer threads (as phase 9).

### 7a-6 game

- **What**: render holes and tube meshes; delete `road_surface.cpp:46`'s tunnel admission as part of phase 7.
- **Tests**: the in-game checks below. The fall detector stays silent on the autopilot route, now through site 4's
  portals.

### In-game acceptance checks (after 7a-6)

| Site | Check |
|---|---|
| 1 Unterliederbach NW mouth | Drive NW to SE at 50 km/h: no launch. Collision top across the portal within 0.10 m per metre (the coordinator's probe: was 0.8-0.9 m over 1-2 m). The rail sidings stand on a visible deck over a box opening; no terrain wall along the road. |
| 2 Königsteiner Straße under the construction rail bridge | Flat road, no hump, no walls; the bridge deck above (with rail opt-in). |
| 3 A66 service tunnel | Two implied decks carry the A66 carriageways (no slot in the A66 surface any more); the service road below drives through two box openings; the median strip has a roof slab. |
| 4 City-Tunnel Frankfurt | Streets above untouched (no trench); with rail: the tube is a box, the portals at both ramp ends. |
| 5 Hasselborner Tunnel | Hill intact; circular tube; headwall with wing walls at both portals; no contact spike entering. |
| 6 Wandersmann Nord | As 5, both tubes. |
| 7 Tunnel Breckenheim | Box tube under the hill (natural cover 9.7 m). |
| 8 Mönchwaldtunnel | Portal hood (box, slab roof) on the first 114 m from the north portal, then the box tube. No step at the north portal (census +6.7 m). |
| 9 Building passage 554045663 | Road at grade under the building; no trench (cover 0.2 m: nothing to cut). |
| 10 Track under four road bridges | Four decks, slab roofs over the 9-11 m gaps (or open slots, 11.2). |

## 11. Open questions

1. **Floor depth in flat cities**: the design floor puts city tubes at exactly the needed cover (rail 7 m, subway
   5.5 m). Real ones are often deeper (stations, other lines below). Accept the minimum, or add a per-class extra
   depth? Proposed: accept; served heights are not survey data.
2. **Gaps between separate bridge decks** (site 10): one continuous box with slab roofs (the current rule: every run
   has a roof) or open light slots? Proposed: roof (owner: "a tunnel needs a roof").
3. **Interior-end anchoring in the census**: the census anchors interior ends at the needed depth. The served
   component solve (3.2) is the real rule; the census numbers for the 255 such chains are approximate.
4. **Building passages**: 618 no-cover passages run through building meshes; the mesh needs an opening (phase 14).
5. **Thresholds marked [unverified]** in 3.3: the clearance and grade citations come from memory, not from a source
   read in this session.
6. **Mönchwaldtunnel tile**: `roads.graph` fails with "restriction via ways" at its north portal; a separate defect,
   not investigated.

## 12. Proposed vault text (G2M-029)

```
G2M-029 - Structure classification of tunnel stretches (terrain migration phase 7a)
projects: geo2map, racing_game

Decision:
- Every OSM tunnel stretch (road and rail) is split into runs, and every run has a roof:
  - Tube: real cover; the hill stays, closed box or bored tube.
  - Deck: a mapped bridge above; box opening under that deck.
  - ImpliedDeck: a ground way above without the needed cover; box opening under an implied deck served for that way.
  - Slab: open cut with a served roof slab, or the building for a building passage.
- Cover is measured against a DESIGN FLOOR: the highest grade-limited profile below the portal chord that keeps
  clearance + roof below the ground, applied from needed/grade away from each portal, meeting the portal anchors
  exactly, solved over each connected tunnel network.
- Shape: bored when the run is >= 300 m long and its median natural cover (ground minus portal chord) is >= 15 m; else
  box.
- Profile continuity at every portal and tunnel/ground join is a hard requirement: the ground profile owns the join
  height and grade; tunnel floors anchor on it.
- Thresholds are data in data/road_rules/de.json "structure".

Measurements (home region E455-490 N5540-5585, 1344 chains, DGM1):
- Against the portal chord no tunnel has real cover. The flat-city City-Tunnel Frankfurt had a median cover of 2.4 m.
- With the design floor: rail 96 real / 74 mixed / 140 no cover chains; road 18 / 63 / 948.
- Of 41 tube runs of at least 300 m, 33 have under 8 m natural cover and 5 at least 15 m (Hasselborner, Wandersmann
  Nord x2, Eppsteiner x2).

Alternatives:
- Cover against the portal chord: simple, matches D52. Rejected: every city tunnel becomes an open trench.
- Cover against the served DEM-following floor (D47): rejected, cover is about 0 by construction.
- Classify by length (D65's 80 m): rejected by the owner ruling; length does not say whether a roof exists.
- Bored by length only: rejected; long flat-city tunnels are cut-and-cover boxes.
- Deepening the floor right at the portals: rejected; it puts a sag of up to 5.7 m into every hill tunnel.

Bug cause recorded (coordinator acceptance case 2026-10-20, Unterliederbach way 4812754 NW mouth, launch lip
0.8-0.9 m):
- road_completion step 2a anchors the tunnel floor on dem.sample at the portal node (99.52 m, the bank face).
- The ground road 480160094 outside lies at 98.68 m; its last 6.5 m stretch is declined ("estimated endpoint slope
  exceeds class bound"), so the carve rises 0.84 m over about 2 m.
- Predates D65: the baseline has the same anchor.
- 190 of 1949 portals differ by >= 0.5 m between the node sample and the approach.

Lessons:
- A single DEM sample at a structure end is the structure's face, not the road level.
- Measure cover against the floor the tunnel would really have, not the line the data happens to give.
```

## Appendix: files

- `tools/structure_census/structure_classify.{h,cpp}`: the rule (design floor, runs, class, shape).
- `tools/structure_census/main.cpp`: the census CLI.
- `tests/unit/test_structure_classify.cpp`: 12 cases, `[structure]`, sabotages S1-S4.
- `out/census/` (not committed): `home.txt`, `home_chains.csv`, `home_runs.csv`, `home_under.csv`,
  `home_portals.csv`, `dump_4812754.csv`, `mouth_4812754.txt`, `mouth_480160094.txt` (spot_dump).
