# Street-level fidelity: sources, architecture mapping and phased plan

- **Owner:** Sebastian.
- **Written:** 2026-10-05 by a research/design agent (Opus 5.5). This is a design document, not code.
- **How it was made (read only):**
  - geo2map_engine `PLAN.md` sections 0, 2.4, 3.2, 3.3, 3.9, 5, 6, 7, 13; `CLAUDE.md`; `docs/g3_road_pipeline.md`, `docs/g3_profile_foundation.md`, `docs/g3_reference_lines.md`, `docs/formats/src_osm.md`, `docs/formats/road_geom.md`; `include/g2m/layer/road_attributes.h`.
  - racing_game `PLAN.md` sections 11.5, 12, 13, 15; `CLAUDE.md` (terrain, visible roads, road reconstruction); `docs/buildings.md`; `core/src/road_visual.cpp`.
  - `S:\claude_code\geo2map_data\SOURCES.md` (what has already been downloaded).
  - Official web pages and product sheets (listed under References).
  - Tag-coverage numbers come from Geofabrik's regional taginfo for Hessen (`taginfo.geofabrik.de/europe:germany:hessen`, small JSON API calls, queried 2026-10-05). They cover all of Hessen, not the home region alone. Phase P0 measures the home region.
- **Nothing was downloaded, installed, committed or created except this file.** No account was created and no terms were accepted.
- **Markers:**
  - **[unverified]**: I could not confirm it against a primary source.
  - **[estimate]**: my own arithmetic, to be replaced by a measurement in the phase named.
  - **[project default]**: a value we choose. It is not taken from a standard.
- **Licence readings are my reading, not legal advice.** This matches geo2map PLAN.md 3.9.

---

## 1. Summary

**Roads first, and almost all of it comes from data we already have.**
- The Geofabrik Hessen PBF in `geo2map_data` carries rich lane data on the classes that matter for driving. Shares of Hessen ways tagged:

  | Class | `lanes` | `turn:lanes` | `placement` |
  |---|---:|---:|---:|
  | motorway / trunk | 99.9 % | ~30 % | 23–29 % |
  | primary | 92.5 % | 21 % (+ `:forward`) | 4 % |
  | secondary | 69 % | | |
  | tertiary | 54 % | | |

- Hessen also has 36,925 `highway=crossing` nodes and 13,941 `highway=traffic_signals` nodes.
- `g2m.src.osm` already keeps every node and way tag, and `RoadGraph` keeps every way tag. **No new download is needed for the road phases.**
- German marking rules (RMS) are deterministic data, so lane markings, transitions, crossings, stop lines and arrows can all be **rule-based**. No ML is needed for roads.

**The one physics-relevant change is the lane-aware cross-section.**
- This means `placement`, asymmetric `lanes:forward/backward`, lane-count transitions, traffic-calming bumps and (optionally) sidewalks.
- The cross-section decides the carriageway edges. G3-D junction polygons and terrain carving are built from those edges.
- It must therefore land **before or together with G3-D**, inside `roads.geom`, as the single geometry authority for physics and render.
- Markings, arrows, crossings and signals are **render-only**. A g2m library function builds them on the client, as PLAN.md 6.3 step 9 already says.

**Buildings and trees in Hessen should come from official open data, not from street imagery.**

| Feature | Source | Licence |
|---|---|---|
| Roof shapes, footprints, heights | HVBG LoD2 CityGML | dl-de/by-2.0 (attribution required; corrected 2026-10-05 after the primary metadata record, see geo2map_data/SOURCES.md) |
| Roof colour | sampled deterministically from DOP20 orthophotos | dl-de/zero |
| Tree positions and heights | nDOM = DOM1 − DGM1, local-maximum crown segmentation | dl-de/zero |
| Tree species | Frankfurt Baumkataster | dl-de/by |
| | OSM `natural=tree` | ODbL |
| | Thünen national species map | CC BY 4.0 |
| Foliage colour and seasons | rule-based from DWD phenology dates | CC BY 4.0 |
| Facades | procedural from building type, levels and age, plus the rare OSM `building:colour`/`material` tags | |

- No source gives facade appearance at scale under usable terms. That is the one feature only street-level imagery could really improve.

**Street-level imagery is not recommended as a direct input.**
- Mapillary imagery is CC BY-SA 4.0. Its terms explicitly permit only deriving data *for OpenStreetMap/Overture*, and its API needs an account and token.
- KartaView is CC BY-SA. Panoramax images are CC BY-SA, plus an ODbL permission on the OSM-France instance; German coverage is [unverified].
- Google Street View is excluded by its terms: no derived datasets and no ML use.
- The clean route is indirect: imagery → OSM tags (by mappers, including the owner) → our ODbL pipeline.

**Recommended first phase: P0 + P1.**
- P0 (S): a zero-download tag-coverage probe of the home region.
- P1 (M/L, Opus design + Sonnet implementation): the lane-aware cross-section in `roads.geom`, with lane sections, placement, transitions and sidewalks. It extends G3-C step 4 and is the input G3-D needs.
- P2 (G3-D junction surfaces plus crossing/stop-line/signal furniture) and P3 (RMS markings, replacing the game-side marking shader logic) follow.
- Buildings (P4) and trees/foliage (P5, P6) need three owner-approved manual downloads (LoD2, DOM1, DOP20; together about 11–21 GB [estimate]) and extend G5.

---

## 2. Baseline: what exists today

| Area | State (2026-10-05) |
|---|---|
| OSM raw | `g2m.src.osm` (level-2 1024 m tiles + 128 m halo) keeps **all** node, way and relation tags and topology (`docs/formats/src_osm.md`). |
| Road graph | G3-A/B done: `g2m.roads.graph` with stable ids and all way tags. |
| Road geometry | G3-C: reference lines and vertical profiles exist; `g2m.roads.geom` encoding 5 is served opt-in. `RoadAttributes` has `width_mm`, `lanes` (total only, "never inferred from width"), crown, shoulder and blend. There is no per-direction lane model, no placement, no lane sections. Profile acceptance is still open (395/478 home stretches accepted). |
| Junction surfaces, carving | G3-D not started. |
| Ribbons, markings | G3-E not started in g2m. **racing_game already has a game-side version** in `core/src/road_visual.cpp`: 1 m-station ribbons, per-vertex `lanes`/`forward`/`flags`, a shader that draws lane and edge lines, turn arrows only where `turn:lanes` exists, and an 8 m fade at junction ends instead of junction surfaces. `rg/road_surface.h` and `road_structures.h` do game-side profile surfacing, bridge slabs and tunnel floors. |
| Buildings | racing_game `docs/buildings.md`: OSM footprints extruded, `building:colour` used, **flat roofs only**, no collision. G5 (`g2m.features`) is not started. |
| Trees, vegetation | No source data in the pipeline except WorldCover classes. PLAN.md 6.5 plans a blue-noise scatter from `terrain.class` plus `natural=tree`. D6: no vegetation collision in v1. |
| Downloads already approved and present | Hessen PBF (2026-09-25, 330 MB), DGM1 for 8 districts (8.5 GB, ≈ 2.8 MB/km² delivered), GLO-30, WorldCover. PLAN.md 13 also lists "Hessen LoD2 for the region (optional, G5+), size unknown, manual" among the dev datasets the owner approved on 2026-09-25. I still ask for re-confirmation in section 9, because the size is now estimated. |

---

## 3. Source inventory

### 3.1 OSM tags (ODbL 1.0; source: Geofabrik Hessen extract, already downloaded)

Counts are for all of Hessen (Geofabrik taginfo, 2026-10-05). Shares are "fraction of ways of that `highway` class carrying the key". Hessen has 2.77 M building ways and 1.20 M highway ways, of which 366 k are tracks and 213 k are service roads.

| Feature | Tags | Hessen coverage | Notes |
|---|---|---|---|
| Lane count | `lanes`, `lanes:forward/backward`, `lanes:both_ways`, `lanes:psv` | lanes on motorway 99.9 %, trunk 99.8 %, primary 92.5 %, secondary 69.3 %, tertiary 53.6 %, unclassified 24.4 %, residential 12.8 %. `lanes:forward` 13.9 k ways. `lanes:both_ways` 354. | The core input. Below tertiary, use rules: residential without centre line, etc. |
| Turn lanes | `turn:lanes`, `turn:lanes:forward/backward/both_ways` | 9.4 k + 6.3 k + 6.0 k ways. Share: motorway 28.6 %, trunk 31.7 %, primary 10.1 % + 10.9 % (`:forward`) | Drives arrows and lane connectivity (traffic later). |
| Lane-change restrictions | `change:lanes` (+ `:forward/:backward`) | 1.7 k ways (motorway 2.1 %, motorway_link 4.1 %) | Solid vs broken line per lane boundary. Values `yes|no|not_left|not_right|only_left|only_right` per lane. |
| Lateral position | `placement`, `placement:forward` | 4.5 k + 0.6 k ways. Values: `right_of:1` 2035, `middle_of:2` 726, `right_of:2` 543, `transition` 466, … | **Physics-relevant**: shifts the carriageway relative to the OSM line. Without it, merges and splits jump sideways. |
| Width | `width`, `width:lanes`, `width:carriageway` | 81.7 k ways, but only 2–8 % of road classes. `width:lanes` 365. | Mostly defaults. Optional ATKIS width conflation (P1b). |
| Lane markings present | `lane_markings=yes/no` | 38.3 k ways (tertiary 21.6 %, unclassified 28.4 %, residential 11.7 %) | Already read by `road_visual.cpp`. |
| Overtaking | `overtaking=no` etc. | [not queried] | Solid centre line. |
| Sidewalks, parking | `sidewalk`, `parking:both/right`, `bus_bay`, `kerb` | sidewalk 93 k ways (residential 31.7 %, primary 34.3 %); parking:both 14.3 k; kerb 24.6 k (mostly nodes) | Urban "road definition" depends heavily on sidewalks and kerbs. |
| Crossings | `highway=crossing` (node), `footway=crossing` (way), `crossing=traffic_signals/uncontrolled/…`, `crossing:markings=zebra/dashes/no/lines/…` | crossing nodes 36.9 k; footway=crossing 13.7 k; crossing=traffic_signals 14.4 k; crossing:markings: `no` 13.8 k, `dashes` 5.1 k, `zebra` 4.6 k, `yes` 1.9 k | `crossing:markings` is the newer key; older data uses `crossing=zebra` / `crossing_ref` [not queried]. |
| Signals, stop, give way | `highway=traffic_signals`, `traffic_signals:direction`, `highway=stop`, `highway=give_way`, `direction` | signals 13.9 k nodes (10.2 k with `traffic_signals:direction`); stop 1.7 k; give_way 11.4 k | Stop lines are **not mapped in OSM** (`highway=stop_line` = 0); they are derived. |
| Traffic calming | `traffic_calming=island/bump/table/chicane/hump/choker/cushion/…` | 3.9 k (island 1146, bump 727, table 529, chicane 387, hump 359) | **Physics-relevant** if carved (bumps and tables are drivable geometry). |
| Street lamps | `highway=street_lamp` | 44 k nodes | Render props. |
| Building height | `building:levels`, `height`, `roof:levels`, `roof:height` | levels 201 k (7.3 % of buildings), roof:levels 136 k, height 29.6 k | LoD2 covers everything in Hessen. |
| Roof shape | `roof:shape`, `roof:direction`, `roof:orientation` | 173 k (6.2 %): gabled 113 k, flat 36 k, hipped 10.9 k, skillion 2.6 k, half-hipped 2.2 k, … | Simple 3D Buildings (S3DB) schema. |
| Roof and facade appearance | `roof:colour`, `roof:material`, `building:colour`, `building:material` | 22.8 k / 10.5 k / 10.9 k / 9.0 k (≤ 0.8 %) | Overrides only. |
| Trees | `natural=tree` (node), `natural=tree_row` (way), `leaf_type`, `leaf_cycle`, `genus`, `species`, `denotation`, `height` | 202 k tree nodes; 17.6 k tree rows; leaf_type 106 k (broadleaved 99 k, needleleaved 3.3 k); genus 14.3 k; species 11.2 k; denotation 43 k | Good for street and park trees; forests are polygons only. |
| Forest leaf type | `landuse=forest` / `natural=wood` + `leaf_type` | 26 k + 20 k polygons; leaf_type on 18.7 k ways | Partial. |

Tag semantics: OSM wiki pages for each key (References). Overture transportation has topology, width and surface, but per geo2map PLAN.md 5.2 no lanes. It adds nothing for Hessen roads.

### 3.2 Hessen official geodata (HVBG)

HVBG's general statement: the data and their automated retrieval are free under **Datenlizenz Deutschland – Zero – Version 2.0** (§ 24 HVGG). geo2map PLAN.md 5.1 records "open since 1 Feb 2022". dl-de/zero has no conditions: no attribution, no share-alike, commercial use allowed. **Exception, corrected 2026-10-05:** the primary metadata record of the LoD2 building models states dl-de/**by**-2.0 (attribution required, not share-alike), not zero; `S:\claude_code\geo2map_data\SOURCES.md` records the attribution text. DOM1, DOP20, DGM1 and Basis-DLM stay dl-de/zero-2.0.

| Product | Content and resolution | Licence | Access | Update cycle | Volume for ~1,000 km² | Use here |
|---|---|---|---|---|---|---|
| **LoD2 3D building models** | CityGML 1.0 (AdV profile), standardised roof forms, ~1 m height accuracy, ETRS89/UTM32 + DHHN2016; LoD1 is fee-based | dl-de/by-2.0 (primary metadata record, corrected 2026-10-05; first read as zero from the HVBG general statement; opendata.hessen.de record "INSPIRE HE Gebäude – 3D LoD2"; HVBG 3D page) | free self-service in the Download-Center at gds.hessen.de; INSPIRE ATOM feed per municipality; INSPIRE WFS | annual | 0.3–1.5 GB zipped [estimate: ~200–300 k buildings × 5–15 KB of XML] | roof shapes and footprints (G5) |
| **DOP20 (ATKIS DOP / TrueDOP)** | 20 cm, captured RGBI. Free download is RGB JPEG; CIR and RGBI are WMS only (or fee-based files). TrueDOP is rectified on the bDOM from flight 2020 onward. Positional accuracy better than ±1 m. | dl-de/zero-2.0 (GDI-DE record) | Download-Center (RGB, JPEG); Shop (small areas, TIFF/JPEG); WMS (RGB and CIR) | 2-year cycle since 2008 | 8–15 GB JPEG [estimate: 25 Mpx/km² at ~0.3–0.6 B/px] | roof colour sampling, QA; optional tree and crown checks |
| **DOM1** (laser surface model) | 1 m float32 GeoTIFF, LZW, NoData −9999, ±0.3 m on hard surfaces | "HVBG terms (Zero)" | free self-service in the Download-Center; free arbitrary areas in the Shop | ALS campaigns 2015–2021, flown in **winter months** (leaf-off); target 6-year cycle | 2.5–4 GB [estimate, using DGM1's measured 2.8 MB/km²] | nDOM = DOM1 − DGM1 → tree heights, crowns, canopy |
| **bDOM20** (image-matched surface) | 20 cm LAS/LAZ 1.2 rasterised point grid with RGB+NIR attributes, unclassified, ±0.6 m height | (as above) | **free only for areas up to 5 km² per order** in the Shop; larger areas are fee-based (time-based fee + 19 % VAT) | 2-year cycle since 2022 (50 %/year) | large (≈ 25 M points/km²) | not recommended (cost, volume); possible leaf-on spot checks |
| **3D laser points** (ALS point cloud) | ≥ 8 pts/m², ±15 cm Z | [unverified]: no free bulk offer found | product sheet lists DGM1/DOM1 and "special products on request" | — | — | not needed |
| **ATKIS Basis-DLM** | vector landscape model: road axes with attributes, forest with vegetation feature (deciduous/coniferous/mixed), tree rows, single trees [attribute fill rate unverified] | dl-de/zero (general HVBG statement) | **Hessen-wide free download** (NAS XML, Shape); WFS | ⅓ of the state per year; main roads updated quarterly | 0.5–2 GB [estimate] | optional: carriageway width / lane count conflation (P1b), forest leaf type |
| **ALKIS** (cadastre) | building outlines, building function, sometimes storeys [unverified for Hessen] | open since 2022 per HVBG [licence wording unverified per product] | Download-Center / WFS | continuous | [unverified] | not needed if LoD2 is used (LoD2 is derived from ALKIS outlines) |

**Hessen gaps:** no textured or facade models, no open oblique imagery (none found [unverified]), and no free bulk point cloud.

### 3.3 Municipal and national open data

| Source | Content | Licence | Access | Update | Volume | Notes |
|---|---|---|---|---|---|---|
| **Frankfurt Baumkataster** (Grünflächenamt) | >168,000 public trees: species, genus, tree number, planting year, crown diameter, ETRS89 coordinates | **dl-de/by-2.0** (attribution; not share-alike) | CSV on offenedaten.frankfurt.de (latest version Aug 2023 per search snippet [page not fetchable: certificate error]) | irregular | ~20–40 MB [estimate] | covers only the Höchst part of the home region (~5 km²) plus the margin. Other MTK/HTK towns: no tree cadastre found [unverified]. |
| **Thünen "Dominant Tree Species for Germany (2017/2018)"** (Blickensdörfer et al.) | 11 species groups from Sentinel-1/2 + National Forest Inventory; 87 % accuracy for pure stands, 76 % including mixed | **CC BY 4.0** | OpenAgrar DOI 10.3220/DATA20221214084846 | static | [unverified]; national GeoTIFF, likely 0.3–2 GB | forest species mix for P5/P6 |
| **DWD phenology** (annual and immediate reporters) | leaf unfolding, autumn colouring and leaf fall dates per station and species (beech, oak, birch, horse chestnut, lime, …) | **CC BY 4.0** | opendata.dwd.de CSV | annual | a few MB | seasonal foliage timing (P6) |

### 3.4 Pan-European and global rasters and vectors

| Source | Content | Licence | Access | Notes |
|---|---|---|---|---|
| ESA WorldCover 2021 v200 | 10 m, 11 classes | CC BY 4.0 | already downloaded | existing land-class fallback |
| Copernicus HRL Tree Cover Density / Dominant Leaf Type | 10 m; TCD 0–100 %, DLT broadleaved/coniferous; reference years 2012, 2015, 2018–2021 | Copernicus free, full and open (attribution) | CLMS portal (EU Login account likely required [unverified]) | worldwide-later fallback for leaf type; in Hessen the Thünen map + nDOM are better |
| Copernicus Urban Atlas **Street Tree Layer** 2018 | tree patches/rows ≥ 500 m², ≥ 10 m wide, inside the urban mask per FUA (Frankfurt FUA) | Copernicus free and open | CLMS download per FUA (SQLite) | too coarse for single trees; skip |
| Sentinel-2 L2A | 10 m, 5-day revisit; observed seasonal colour (e.g. rapeseed in May) | Copernicus Sentinel licence: free, full and open, commercial OK, notice "Contains modified Copernicus Sentinel data [year]" | AWS COGs, range reads, no account | optional later (P6b) for observed field and forest colour |
| Meta/WRI global canopy height v2 | 1 m CHM, ML from imagery | CC BY 4.0 | AWS `dataforgood-fb-data` | worldwide fallback for tree height; Hessen nDOM is real lidar and better |
| ETH global canopy height 2020 | 10 m, GEDI + Sentinel-2 | CC BY 4.0 | ETH research collection | coarse fallback |
| Overture buildings | OSM + Esri Community + Microsoft/Google ML footprints; `roof_shape`, `roof_height`, `facade_material` (from OSM) | **ODbL** | GeoParquet, monthly | no gain over OSM + LoD2 in Hessen; worldwide footprint gap-filler later (G8) |
| EUBUCCO v0.1 | 202 M EU buildings, height 73 %, age 24 %, type 46 % | **ODbL** (Prague CC BY-SA, Abruzzo CC BY-NC as separate files) | Zenodo / JRC | possible worldwide height/age fallback; in Germany its heights largely come from LoD data already |

### 3.5 Street-level imagery

| Source | Imagery licence | Derived data | Access | Verdict |
|---|---|---|---|---|
| **Mapillary** (Meta) | CC BY-SA 4.0. Terms (effective 2024-02-15) s. 3(b): user content under CC BY-SA unless indicated otherwise. | Map features (point objects incl. traffic signs, discrete markings such as crosswalks and arrows [class list unverified]) and per-image segmentations. Terms s. 13: use permitted to contribute to **OpenStreetMap or Overture**. s. 11: no mere redistribution, no app that substantially replicates Mapillary services. The licence of API-derived features is not stated separately, so I read it as CC BY-SA. | client **and** user OAuth token required (= account). Tiles 50,000/day; map feature bbox ≤ 0.01°². Attribution "© Mapillary". | **No direct ingestion.** Share-alike on derived layers plus terms friction. Use only indirectly via OSM, optionally as a private QA signal (owner decision). Continuous lane lines exist only as image segmentations, not as georeferenced lines. |
| **Panoramax** | per instance: OSM-France instance CC BY-SA 4.0 with an added permission to derive non-photographic data under ODbL; IGN instance etalab-2.0 | follows the instance | federated API (api.panoramax.xyz meta-catalogue) | German coverage [unverified, likely thin]. ODbL-permitted derived data would equal OSM's terms, but it still needs ML extraction. Later candidate. |
| **KartaView** (Grab) | CC BY-SA 4.0 | — | API | coverage [unverified]; same share-alike issue; skip |
| **Google Street View** | proprietary | Google Maps Platform Terms s. 3.2.4(b) "No Creating Content From Google Maps Content": no tracing, digitising or creating datasets, and no use to train/test/validate ML models (per secondary summaries; the primary page did not render fully) | API key | **Excluded** |

### 3.6 Rule sources (deterministic "data, not code")

| Source | What it gives | Access | Notes |
|---|---|---|---|
| **RMS** (Richtlinien für die Markierung von Straßen, FGSV; Part A Autobahnen 2019, RMS-1 for other roads) | line types, widths, dash patterns, arrows, stop and wait lines, crossings, restricted areas | paid FGSV publication; public summaries on Wikipedia | Values below are from secondary sources, so this repo does **not** reproduce the RMS. Put values in a versioned data table marked with their source. |
| Values found | Narrow line 0.15 m (motorway) / 0.12 m (other); wide line 0.30 / 0.25 m. Lane line motorway 6 m dash / 12 m gap; urban 3 m / 6 m (1:2); 1:1 and 1.0–1.5 m dashes in junction areas; stop line 0.50 m; wait line 0.50 m (broken); exit line wide 6 m / 6 m. **Rural lane line dash/gap [unverified]: German 4 m / 8 m from memory; one source gives 6 m / 9 m, which looks Austrian.** | Wikipedia "Fahrbahnmarkierung" | Owner may want to buy RMS for exactness. |
| Lane widths | RAA 2008 (motorways ~3.50–3.75 m), RAL 2012 (rural ~3.25–3.50 m), RASt 06 (urban 2.75–3.25 m) [all unverified, from memory] | FGSV, paid | Keep the existing 3.25 m default plus a per-class table. P1b can calibrate from ATKIS widths or DOP20. |
| Simple 3D Buildings (OSM wiki) | roof-shape geometry definitions for `roof:shape` | free | for the non-LoD2 fallback |

### 3.7 Licence flags and what they mean for served derived layers

| Licence | Sources | Commercial | Share-alike / NC | Consequence for our served layers |
|---|---|---|---|---|
| dl-de/zero-2.0 | HVBG DOP20, DOM1, DGM1, Basis-DLM | yes | none | No obligation. A layer built **only** from these stays free of ODbL. |
| dl-de/by-2.0 | HVBG LoD2 (3D-Gebäudemodelle), Frankfurt Baumkataster | yes | attribution + licence reference (+ mark changes) | Manifest attribution only. |
| CC BY 4.0 | Thünen species, DWD, WorldCover, Meta/ETH CHM, Google Open Buildings | yes | attribution + indicate changes | Manifest attribution. |
| Copernicus | Sentinel-2, HRL, Urban Atlas | yes | source notice | Manifest attribution. |
| **ODbL 1.0 (share-alike)** | OSM, Overture, EUBUCCO, Microsoft footprints | yes | **SA on Derivative Databases that are publicly used** | Anything that **merges** OSM data with other data in one database (e.g. OSM building + LoD2 roof matched into one record) is ODbL as a whole. Keeping sources as separate, independent layers or sections forms a Collective Database, and the non-OSM part keeps its own licence (ODbL 4.5(b) reading). Owner ruling D3 says the server is private now. D7 already plans ODbL publication of OSM-derived packs if that ever changes. |
| **CC BY-SA 4.0 (share-alike)** | Mapillary, KartaView, Panoramax OSM-FR images | yes (with Mapillary terms limits) | **SA on adaptations** | Facade colours or markings extracted from these images would arguably be adaptations → the derived layer is CC BY-SA. Mapillary terms also restrict use outside OSM/Overture. **Avoid as direct input.** |
| NC | EUBUCCO's Abruzzo part; FABDEM (PLAN 5.1) | **no** | — | Not used. |
| proprietary, no derivation | Google Street View | no | — | Excluded. |

---

## 4. Feature-by-feature mapping

Columns:
- **Where:** S = derived server-side (baked layer, identical bytes for all players). C = derived client-side by a g2m library function (deterministic, never served). G = game adapter (thin).
- **Phys:** whether physics sees it.

| Feature | Primary source(s) | Fallback | Method | Layer | Where | Phys | Effort |
|---|---|---|---|---|---|---|---|
| Lane count per direction, lane widths | OSM `lanes*`, `width*` | class defaults table; ATKIS width (P1b) | rule-based tag resolution | `roads.geom` encoding 6, new **cross-section/lane-section** section | S | **yes** (carriageway edges → carving, junction polygons) | M |
| Lateral position (`placement`) | OSM `placement*` | centred (current) | offset function of s | same | S | **yes** | S (within P1) |
| Lane transitions (merges, splits, lane drops, tapers) | lane-count change at degree-2 nodes; `turn:lanes` `merge_to_*`; `placement=transition`; `change:lanes` | taper on the outer side [project default] | piecewise-linear lane-boundary offsets over a taper length from a table (e.g. max(40 m, 0.6 × v_kph) [project default]) | same | S | **yes** (edge envelope) | M |
| Turn lanes / per-lane directions | `turn:lanes*` | none (no arrows) | parsed per lane section | same (lane attribute) | S | no | S |
| Lane-change restrictions | `change:lanes*`, `overtaking` | class rules (e.g. solid near junctions) | per-boundary line type | same | S | no | S |
| Sidewalks, kerbs, parking lanes | `sidewalk*`, `kerb`, `parking:*` | class rules (urban residential: both sides) [project default] | cross-section bands | same | S | **flush in physics**; kerb step render-only v1 (decision D-SF7) | M |
| Junction layout (surfaces) | lane-aware edges from P1 | — | G3-D step 5: offset, intersect, trim (A/B Street / osm2streets algorithm), height plane, earcut | `roads.geom` junction section (G3-D) | S | **yes** | L (is G3-D) |
| Pedestrian crossings | `highway=crossing` nodes on carriageways, `footway=crossing` ways, `crossing:markings`, `crossing=*` | unmarked if unknown | polygon across the carriageway at the node; width from the footway or 4 m [project default]; pattern from markings | `roads.geom` **junction-furniture** section | S | no (same surface id) | M |
| Stop lines, give-way lines | `highway=stop/give_way/traffic_signals` (+ `direction`, `traffic_signals:direction`), crossings | inferred at signalled approaches | line across the approach lanes of one direction, set back from the crossing/junction polygon | furniture section | S | no | M |
| Traffic signals (heads/poles) | `highway=traffic_signals` nodes | junction-node signals → all approaches | positions + facing per approach | furniture section | S | no (static collision later, optional) | S |
| Traffic calming | `traffic_calming=bump/hump/table/cushion` | — | longitudinal height bumps added to the profile within the carriageway | `roads.geom` profile features | S | **yes** (owner decision D-SF8) | S/M |
| Lane markings (geometry) | lane sections + furniture + RMS rule table | — | rule-based pattern generation; longitudinal lines as ribbon attributes, symbols as decal quads (D-SF3) | **never served**: `g2m::roads::build_markings()` | C → G uploads | no | M |
| Building footprints | **LoD2** (Hessen); OSM elsewhere | OSM | LoD2 ground surface / OSM rings | `g2m.features` buildings section (G5) | S | **yes** (G5/R7 collision; footprint authority D-SF5) | M |
| Building heights | LoD2 (measured roof geometry) | OSM `height`/levels; nDOM over footprint (outside LoD2) | — | same | S | yes (collision) | S |
| Roof shapes | **LoD2 roof surfaces** | OSM `roof:shape` (6 %); rules: residential ≤ 15 m short side → gabled along the long axis [project default] | LoD2: triangulated roof polygons; S3DB generator otherwise | same | S | no (collision = extrusion/hull v1) | M |
| Roof colour | **DOP20 RGB sampled inside each LoD2 roof polygon** (robust median in an integer colour space, quantised to a ~16-entry palette) | OSM `roof:colour`; regional palette by type | deterministic sampling in the tiler | `g2m.src.bldg_appearance` (tiler-imported raw layer) → `g2m.features` | S (tiler) | no | M |
| Facade colour, material, windows/levels | OSM `building:colour`/`material` (≤ 0.4 %); LoD2 `function` / OSM `building=*`; levels | procedural palette by type/region/age, seeded per building id | rule tables; window grid from levels (PLAN 11.5) | `g2m.features` (type, levels) + client material choice | S (attributes) + C (variant) | no | M |
| Tree positions and heights | **nDOM** (DOM1 − DGM1) local maxima + crown segmentation; OSM `natural=tree`; Frankfurt cadastre | blue-noise scatter from land class (6.5 plan) | deterministic integer CHM algorithm; cadastre/OSM points override detections within crown radius | `g2m.features` trees section (or `g2m.veg.trees`) | S | no (D6) | M/L |
| Tree species / leaf type | Frankfurt cadastre; OSM genus/species/leaf_type; **Thünen species map** (forests) | OSM forest `leaf_type`; HRL DLT; generic broadleaved | lookup per tree | same (species code) | S | no | S |
| Tree coverage (canopy) | nDOM | WorldCover tree class | canopy mask for impostor bands | derived from trees + class | S/C | no | S |
| Foliage colour and seasons | DWD phenology dates (Hessen stations) × genus palette | leaf type palette | game date → phase → tint (shader parameter) | rule table in g2m (`g2m::veg::foliage_tint(genus, doy)`); date is game state | C | no | S/M |
| Observed field/forest colour (optional) | Sentinel-2 seasonal composites | palettes | per-10 m tint raster | `g2m.veg.tint` (optional) | S | no | M |

**ML:**
- No phase above requires ML.
- Imagery ML would only be needed for:
  - facade appearance from street images: XL effort, CC BY-SA, not recommended;
  - lane-line extraction from DOP20, to fix wrong OSM lane positions: L effort. Only open semantic-segmentation models exist; no ready open model for German markings on 20 cm DOP was found [unverified]. Defer to an optional QA phase.
- For tree crowns, the nDOM local-maximum approach is the standard non-ML method and is deterministic. ML detectors (DeepForest, detectree2; both MIT [unverified]) add little where lidar exists.

---

## 5. Architecture

### 5.1 Layer changes

| Layer | Change | Kind | Physics | Notes |
|---|---|---|---|---|
| `g2m.roads.geom` | **encoding 6**: new *cross-section* section per stretch (lane sections along s: per lane {direction, width_mm, turn bits, kind (driving/bus/parking/sidewalk/shoulder)}, per boundary {line type from change:lanes/overtaking}, carriageway left/right edge offset functions incl. placement and tapers), *profile features* (calming bumps), and later the G3-D *junction* and *junction-furniture* sections | baked, level 2 | edges + bumps: yes; lane attributes: no | One geometry authority: carving (`terrain.height`, G3-D step 6), junction polygons (step 5), ribbons and markings all read the same edge functions. |
| `g2m.terrain.height` / `terrain.class` | carving uses the P1 edge envelope instead of symmetric `lanes × 3.25` | baked | yes | unchanged mechanics (G3-D) |
| `g2m.src.lod2` (new raw) | CityGML → compact per-level-2-tile buildings: ground ring, wall/roof polygons quantised to mm in the local tile frame, `function`, `roofType`, measured height, AdV id | raw, tiler import | — | needs a CityGML 1.0 subset reader in `g2m_tiler` (XML: pugixml, MIT) |
| `g2m.src.elev.dom1` (new raw) | same codec as `src.elev.dgm1` | raw | **never** feeds `terrain.height` | "No second height source" holds: DOM1 is used only for vegetation and building-height fallback, never for ground. |
| `g2m.src.bldg_appearance` (new raw) | per LoD2 building id: roof palette index + confidence, from DOP20 | raw, tiler-imported | — | JPEG decoding happens **once in the tiler**; clients never decode DOP. Tiler output bytes are hash-checked Windows = Linux on a fixture (decoder bit-exactness [unverified] → pin libjpeg-turbo, `JDCT_ISLOW`, SIMD off in the tiler if needed). |
| `g2m.src.trees.<source>` (new raw) | Frankfurt cadastre points (+ later other municipalities) | raw | — | dl-de/by attribution in the manifest |
| `g2m.src.veg.species` (new raw) | Thünen species raster, re-tiled nearest-neighbour like `src.landcover` | raw | — | CC BY 4.0 |
| `g2m.features` (G5) | buildings (LoD2-first, OSM elsewhere), roof surfaces, palette indices, type/levels; trees (x, y quantised to 0.25 m, height, crown radius, species code, source); signals and lamps as point features | baked, 1 km | building footprints: yes (G5/R7 collision) | Licence hygiene: keep OSM-derived and LoD2-derived records in separate sections with source tags (Collective Database, 3.7) unless the owner accepts ODbL for the whole layer (D-SF6). |

### 5.2 Server vs client, determinism, budgets

- **Server-side (baked):** everything that is identical for all players and costly. That is the lane model, junction surfaces, furniture placement, LoD2 conversion, DOP colour sampling, tree segmentation, species lookup. All follow PLAN.md 2.4:
  - integer geometry;
  - no unordered iteration;
  - fixed halos read from inputs only. Tree segmentation needs a halo of at least the maximum crown radius, e.g. 16 m;
  - golden hashes on Windows and Linux.
- **Client-side (g2m library, never served):** marking geometry, facade variant choice (seeded hash of world seed + building id), foliage tint for the game date, render meshes. They stay pure functions, so screenshots and replays are reproducible. They are not multiplayer-critical.
- **Physics-relevant (deterministic, shared by physics and render):** only carriageway edges, junction surfaces, carving, calming bumps and building collision footprints. Physics never sees markings, trees or roofs. D6 is unchanged.
- **Engine-neutral rule:** marking and foliage logic go into g2m (C++); racing_game's `road_visual.cpp` shrinks to an adapter that turns `build_markings()` output into Godot meshes and material parameters. This also serves a UE5 port.
- **Bake vs on-demand (3.3, D2):** the new raw layers (LoD2, DOM1, appearance, trees) exist only where imported (Hessen first). The resolver's "self-fetched from sources" fallback does not apply to manual-download sources, exactly like DGM1. Outside Hessen every feature degrades to its fallback column in section 4.
- **Budgets (7.1, 7.3)** [estimates, to be measured in each phase]:
  - **Markings:** a residential km² has ~15–25 km of road. Explicit dash quads would be ~20 k triangles/km² → ~250 k triangles in the 2 km road-mesh radius. That is why D-SF3 keeps longitudinal lines in the ribbon shader and makes only symbols geometry. Build cost belongs on the 3 g2m workers, never the main thread; upload goes through the existing ≤ 1 ms/frame budget.
  - **Trees:** Taunus forest at ~300–500 detectable trees/ha. Within the 1.5 km vegetation radius that is ~100–150 k instances → MultiMesh with distance LOD and impostors beyond ~300 m [project default]. Served bytes: ~12–20 M trees in the region × ~8 B ≈ 100–160 MB before zstd [estimate]. If that is too much against the 0.5–1.1 GB pinned-region cache, serve a 2 m CHM for forest interiors and keep explicit trees for urban, edge and street trees (D-SF10).
  - **Buildings:** ~200–300 k LoD2 buildings × ~150 B ≈ 30–45 MB for the region [estimate].

---

## 6. Phased plan (ordered by visible payoff per effort; roads first)

### P0: Region tag-coverage probe (S, Sonnet)

- **Scope:** extend `tools/road_probe` (or the tiler report) to count, inside the home region polygon:
  - per highway class: `lanes*`, `turn:lanes*`, `placement*`, `change:lanes`, `width*`, `sidewalk*`;
  - crossings by `crossing:markings`;
  - signals with and without direction;
  - traffic_calming;
  - building `roof:*`, `building:*`;
  - trees with leaf_type/genus.
  - Also list consecutive ways whose lane counts differ without `placement`, as candidates for wrong transitions.
- **Inputs:** existing Hessen PBF / `g2m.src.osm` store. **No downloads.**
- **Outputs:** a probe report (CSV + summary in `out/`).
- **Acceptance:** counts reproducible (same numbers twice); per-class percentages reported.
- **Risks:** none.
- **Dependencies:** none.
- **Why first:** turns the Hessen-wide numbers above into home-region priorities in under a day.

### P1: Lane-aware cross-section and transitions in `roads.geom` (M/L, Opus design → Sonnet implementation)

- **Scope:**
  - `resolve_cross_section(way tags, RoadStyle, CountryRules)` produces lane sections:
    - per-direction lanes, `lanes:both_ways`, psv;
    - `turn:lanes*` tokens, `change:lanes*`, `width:lanes`/`width`;
    - placement (incl. `transition`);
    - sidewalks/parking/shoulders;
    - dual-carriageway one-way pairs.
  - Lane-count transitions at degree-2 joins, with tapers driven by `merge_to_*` tokens, placement continuity and a rule table.
  - Calming bumps as profile features (behind D-SF8).
  - Encode as `roads.geom` encoding 6 (new sections; existing sections unchanged).
  - Country rules as data (`de.json`: lane widths per class, sidewalk defaults, taper table).
- **Inputs:** existing data only.
- **Outputs:** encoding 6; a C API/struct `RoadCrossSection` for consumers; racing_game `road_visual.cpp` reads lanes/forward/edges from it instead of re-parsing tags.
- **Acceptance:**
  1. On the Frankfurt-Höchst fixture and the A66 ramps: wherever consecutive ways' tags are consistent, carriageway edge offsets are continuous across way joins (|Δ| < 1 cm); taper edges have no kinks above the class curvature limit.
  2. Lane totals equal the tags; `placement` positions the named lane boundary exactly on the OSM line (< 1 mm).
  3. Defaults are flagged "inferred" per field (as `width_inferred` today).
  4. Golden hashes Windows = Linux; fuzzed decoder.
  5. Carving footprint width = carriageway + shoulder from the same function.
- **Risks:**
  - inconsistent OSM tagging on adjacent ways (handle with explicit declines plus the inferred-transition fallback);
  - dual-carriageway one-way pairs need their own pairing rule to avoid a median gap or overlap;
  - encoding churn while G3-C acceptance is still open.
- **Dependencies:** G3-C reference lines (done) and profiles (acceptance open; the owner's endpoint-height/class-grade choice from `g3_road_pipeline.md` is still pending). **Feeds G3-D.**
- **Extends/replaces:**
  - **Replaces** G3-C step 4's "Width = lanes × lane width, or width" with the lane model; crown/superelevation stay.
  - **Extends** `RoadAttributes`, whose `lanes` stays the total.
  - Makes the racing_game per-tag parsing in `road_visual.cpp` obsolete.
- **Optional P1b (M, Sonnet):** ATKIS Basis-DLM road-axis width/lane-count conflation for roads without width tags (download ~0.5–2 GB [estimate]; dl-de/zero). The width source then becomes "official" and is flagged.

### P2: Junction surfaces + junction furniture (L, Opus; = G3-D plus additions)

- **Scope:**
  - G3-D step 5 (junction polygons) built from the **P1 edges**, using A/B Street's offset-intersect-trim algorithm. osm2streets (Apache-2.0, Rust) is the reference implementation; port the algorithm, do not link, because determinism and C++ are required.
  - G3-D step 6 carving.
  - New furniture section:
    - crossing polygons and pattern;
    - stop/give-way lines per approach and direction;
    - signal positions and facing;
    - setbacks from a rule table (stop line ≥ 1 m before a crossing [project default]).
- **Inputs:** existing data.
- **Outputs:** `roads.geom` junction + furniture sections; carved `terrain.height`/`terrain.class`.
- **Acceptance:**
  1. The G3-D criteria (junction continuity < 1 cm, cross-slope ±0.2 %, blend steps ≤ 5 mm, golden hashes).
  2. Every `highway=crossing` node on a rendered carriageway yields a crossing polygon or an explicit decline reason (report counts).
  3. Every signalised approach has a stop line covering exactly its approach-direction lanes.
  4. No furniture polygon lies outside the carriageway + 0.5 m.
- **Risks:**
  - bow-tie junctions and dual-carriageway clusters (the PLAN's 25 m clustering);
  - stop-line setback ambiguity where signals are mapped on the junction node.
- **Dependencies:** P1; G3-C accepted profiles; the physics boundary checks of G3-E/F stay as they are.
- **Extends:** G3-D as written. Furniture is new.

### P3: RMS marking generation (M, Sonnet with an Opus review of the rule table)

- **Scope:** `g2m::roads::build_markings(stretch cross-sections, junction furniture, MarkingRules)`. Outputs:
  - per ribbon vertex: boundary descriptors (line type, dash phase, width) for longitudinal lines, consumed by the ribbon shader;
  - decal quads for:
    - turn arrows (from `turn:lanes`, at 2 positions before the stop line [project default]);
    - zebra stripes, crossing dashes;
    - stop and wait lines;
    - restricted areas in merge/diverge gores;
    - exit lines (wide dashed) on motorway diverges;
  - edge lines interrupted at junction mouths;
  - solid instead of dashed near junctions per `change:lanes`/`overtaking`.
  - The rule table `de` is data: widths, dash/gap per class and urban/rural, colours. "Urban" comes from G5 land class or maxspeed ≤ 50.
- **Inputs:** none new. The owner may buy the RMS to replace the [unverified] values.
- **Outputs:** client-side meshes and attributes; racing_game adapter update (thin).
- **Acceptance:**
  1. Arrow count equals `turn:lanes` tokens on each section ≥ 20 m.
  2. Dash patterns are phase-continuous across stretch joins.
  3. No longitudinal line inside a junction polygon except permitted dotted guide lines.
  4. Build cost per 1 km tile measured on a g2m worker; main-thread upload within the existing 0.8 ms budget.
  5. Screenshot set at 10 owner-picked spots (motorway merge, A66 exit, Höchst zebra, signalised junction).
- **Risks:**
  - RMS values unverified;
  - z-fighting of decals on 1 m terrain (the existing `lift` approach in `road_visual.cpp`).
- **Dependencies:** P1, P2.
- **Extends/replaces:**
  - **Is** the "markings" half of G3-E (PLAN 6.3 step 9).
  - **Replaces** the marking logic now in racing_game `road_visual.cpp`; the ribbon part stays G3-E.

### P4: Buildings: LoD2 roofs, footprints, roof colour (M/L; Opus for the footprint/collision decision, Sonnet for the CityGML importer)

- **Scope:**
  - Tiler `import-lod2` → `g2m.src.lod2`.
  - Tiler `import-dop-appearance` → `g2m.src.bldg_appearance`:
    - median colour of DOP20 pixels inside each roof polygon, eroded by 0.5 m to avoid edges;
    - shadows are rejected by a luminance threshold [project default];
    - quantised to a palette.
  - `g2m.features` buildings (G5) LoD2-first, with OSM tags (`building:colour/material`, levels) merged per D-SF6.
  - Fallback outside LoD2: OSM S3DB `roof:shape` + rule inference.
  - Client: roof triangle meshes + palette materials; facades stay procedural (P7).
- **Inputs (owner approval):**
  - **LoD2 CityGML for MTK, HTK and Frankfurt am Main** (manual Download-Center; 0.3–1.5 GB [estimate]);
  - **DOP20 RGB JPEG for the same area** (manual; 8–15 GB [estimate]).
  - New tiler-only dependencies: pugixml (MIT), libjpeg-turbo (BSD-3/IJG/zlib).
- **Outputs:** new raw layers; G5 buildings section with roof surfaces and colour.
- **Acceptance:**
  1. G5's "building bases sit on terrain, max gap < 5 cm".
  2. Roof ridge heights within 0.3 m of LoD2 after quantisation.
  3. Roof palette agreement ≥ 80 % on a hand-labelled sample of 100 roofs [project default].
  4. Tiler output identical on Windows and Linux.
  5. Per-source attribution in the manifest.
- **Risks:**
  - LoD2 and OSM footprints disagree (matching by IoU; D-SF5);
  - DOP 2-year flights vs annual LoD2 (new buildings);
  - JPEG decoder determinism;
  - DOP volume on disk.
- **Dependencies:** G5 not started, so P4 **defines** G5's building part. R7 collision follows D-SF5.

### P5: Trees: positions, heights, species (M/L; Opus for the segmentation algorithm, Sonnet for importers)

- **Scope:**
  - Tiler `import-dem dom1` → `g2m.src.elev.dom1`.
  - Server-side tree derivation per 1 km tile with a 16 m halo:
    - nDOM = DOM1 − DGM1 on the shared 1 m cell-centre grid, in int 1/256 m;
    - fixed integer smoothing;
    - local maxima with height-dependent window;
    - crown radius by marker-controlled region growing;
    - mask out building footprints (P4/OSM, buffered 1 m) and carriageways (P1);
    - merge with Frankfurt cadastre / OSM `natural=tree` points: points win within the crown radius and bring species and crown diameter;
    - species for forest detections from the Thünen raster, else OSM forest `leaf_type`, else broadleaved.
  - `natural=tree_row` without detections → spaced instances [project default 8 m].
- **Inputs (owner approval):**
  - **DOM1 for MTK, HTK and Frankfurt** (manual; 2.5–4 GB [estimate]);
  - Frankfurt Baumkataster CSV (~20–40 MB [estimate]);
  - Thünen species GeoTIFF (size [unverified]).
- **Outputs:** `g2m.features` trees section; client MultiMesh instances.
- **Acceptance:**
  1. In Höchst: ≥ 70 % of cadastre trees have a derived tree within 2 m, and crown diameter error is reported [project default].
  2. Deterministic per seed and worker count (G5 acceptance).
  3. Served bytes/km² measured.
  4. No tree instance on a carriageway or inside a building.
  5. Frame time within the R9 budget in a forest stretch.
- **Risks:**
  - DOM1 is leaf-off winter ALS from 2015–2021: deciduous heights and crowns are underestimated and stale (felled or new trees) [effect size unverified];
  - forest density → bytes (D-SF10).
- **Dependencies:** G5 vegetation; P1 for road masks.
- **Replaces:** the PLAN 6.5 "blue-noise scatter from terrain.class" **inside Hessen**. The scatter stays as the fallback elsewhere.

### P6: Foliage colour and seasons (S/M, Sonnet)

- **Scope:**
  - DWD phenology (annual reporters, Hessen stations) → per-genus day-of-year table for leaf-out, full leaf, colouring and leaf fall: long-term means, plus optional year-specific values.
  - Per genus/leaf type palette per phase, as data.
  - `g2m::veg::foliage_state(genus, day_of_year)` → tint + leaf density.
  - The game date drives a shader parameter.
  - Grass/field tint seasonal palette from land class.
- **Inputs:** DWD CSVs (a few MB; approval).
- **Outputs:** client function + data table.
- **Acceptance:**
  1. Phase dates within ±3 days of the DWD regional mean for beech/oak/birch.
  2. Pure function of (genus, DOY).
  3. Owner screenshots in April, July, October, December.
- **Risks:** artistic palette tuning.
- **Dependencies:** P5 species codes (works with leaf type alone before P5).
- **Optional P6b:** Sentinel-2 seasonal tint raster (observed rapeseed, harvested fields). Range reads, no account.

### P7: Facades (M, Sonnet)

- **Scope:**
  - Facade rule tables: building type (LoD2 `function` / OSM `building=*`) × era (OSM `start_date`, else regional default) × region → material set + palette weights.
  - Window grid from levels (PLAN 11.5).
  - Shop fronts on ground floors where OSM `shop`/`amenity` nodes fall inside footprints.
  - `building:colour/material` overrides.
  - Seeded variant per building id.
- **Inputs:** none new; CC0 facade textures under the D14 pipeline.
- **Outputs:** client materials.
- **Acceptance:** deterministic per seed; owner visual review of 3 streets.
- **Risks:** looks generic. Only street imagery would fix that (D-SF9).
- **Dependencies:** P4.

### P8 (optional, later): Imagery-assisted QA (L, Opus)

- **Scope:** private, never served, never redistributed:
  - compare derived crossings, arrows and lane counts against DOP20 or Mapillary detections;
  - output a list of suspected OSM errors for the owner to fix **in OSM**. The data then returns legally through the ODbL pipeline.
- **Inputs:** a Mapillary token (owner account) or DOP20 only.
- **Acceptance:** precision of the suspect list ≥ 50 % on a sample.
- **Risks:** Mapillary terms s. 11/13; ML effort.
- **Dependencies:** P1–P3.

### Size and model summary

| Phase | Size | Model | Downloads | Physics change |
|---|---|---|---|---|
| P0 probe | S | Sonnet | none | no |
| P1 cross-section | M/L | Opus design, Sonnet implementation | none (P1b: Basis-DLM) | **yes** (edges, bumps) |
| P2 junctions + furniture | L | Opus | none | **yes** (G3-D) |
| P3 markings | M | Sonnet (+ Opus review) | none (RMS purchase optional) | no |
| P4 buildings | M/L | Opus (decision) + Sonnet | LoD2, DOP20 | footprints for R7 |
| P5 trees | M/L | Opus (algorithm) + Sonnet | DOM1, cadastre, Thünen | no |
| P6 seasons | S/M | Sonnet | DWD | no |
| P7 facades | M | Sonnet | textures (D14) | no |
| P8 QA | L | Opus | token (optional) | no |

### Integration with existing milestones

| Existing item | This plan |
|---|---|
| G3-C step 4 (cross-section width) | **replaced** by P1's lane model; crown, superelevation, shoulder and blend kept |
| G3-D step 5 (junction surfaces) | **is** P2; consumes P1 edges |
| G3-D step 6 (carving) | unchanged mechanics, P1 edge envelope; calming bumps added (D-SF8) |
| G3-E ribbons | unchanged, but read P1 cross-sections |
| G3-E markings (6.3 step 9) | **is** P3 (moves from racing_game into g2m) |
| G3-E bridge bodies, G3-F acceptance | unchanged; G3-F's 20-junction route also checks P2 furniture visually |
| G5 buildings (6.4) | **extended** by P4: LoD2 roofs, DOP roof colour, footprint authority |
| G5 vegetation (6.5 scatter) | **extended/replaced in Hessen** by P5; scatter remains the fallback |
| G5 land cover, water | unchanged |
| PLAN 11.5 rendering | P3 decals, P6 tint, P7 facade tables plug in here |
| Later: traffic (lane-level graph) | P1's per-lane `turn:lanes` and transitions are its input; racing_game's NPC traffic can follow lanes |

---

## 7. Decisions for the owner (recommendation first)

**D-SF1. Where the lane model lives: extend `roads.geom` (encoding 6) vs a separate `g2m.roads.lanes` layer.**
- **Recommend: extend `roads.geom`.**
- Extend:
  - pro: one geometry authority; carving, junction polygons, ribbons and physics provably read the same edges; one dependency set.
  - con: encoding bump while G3-C is unaccepted; a bigger tile.
- Separate layer:
  - pro: geom stays stable and lanes can iterate independently.
  - con: carving and junctions then depend on two layers that can drift between releases. That risks exactly the "second geometry source" problem the PLAN forbids for heights.

**D-SF2. Road attribute source: OSM tags + rule tables vs adding ATKIS Basis-DLM conflation (P1b) vs imagery ML.**
- **Recommend: OSM + rules now; P1b only if P0 shows many wrong widths.**
- OSM + rules: free, already present, deterministic; widths are mostly defaults.
- ATKIS: official widths and lane counts (dl-de/zero), but conflation work (M) and a new download.
- ML on DOP20: L/XL effort and nondeterministic tooling; not justified for a game.

**D-SF3. Marking representation: shader on ribbon attributes vs explicit decal geometry vs hybrid.**
- **Recommend: hybrid.** Longitudinal lines go through ribbon per-vertex boundary descriptors, which extends today's `road_visual.cpp` approach and costs almost no triangles. Arrows, zebras, stop/wait lines and gores become decal quads.
- Shader-only: cheapest, but cannot draw gores, arrows or zebras cleanly.
- Geometry-only: simplest logic, but ~250 k extra triangles in the road radius [estimate].

**D-SF4. Junction algorithm: port A/B Street/osm2streets ideas into g2m C++ vs link osm2streets (Rust) vs keep the game-side fade.**
- **Recommend: port**, as PLAN 6.3 already intends.
- Linking: Apache-2.0 is fine, but Rust in the build, a nondeterministic float pipeline and an unstable API ("API isn't stable yet").
- Fade: zero work, but no junctions, which is exactly what the owner criticised.

**D-SF5. Building footprint authority in Hessen: LoD2 vs OSM.**
- **Recommend: LoD2 geometry, OSM tags merged by footprint overlap.**
- LoD2: official, consistent with DGM1/DOP, has real roofs, dl-de/by-2.0 (attribution in the manifest).
- OSM: has ids, tags and shops, but its footprints differ from LoD2 by up to metres.
- Collision (R7) and render must use the same footprint, or players hit invisible walls. This is also physics-relevant, so the owner should confirm.

**D-SF6. Licence hygiene for `g2m.features`: one merged layer under ODbL vs per-source sections (Collective Database).**
- **Recommend: per-source sections now.** Cost: a little extra code.
- Merged: simpler, but the whole layer becomes ODbL, and so do LoD2/DOP-derived roof data.
- Separate: the dl-de/zero parts stay unencumbered if the server is ever opened (D3/D7); LoD2 needs only attribution (dl-de/by-2.0).
- My reading only; a legal check is needed before any public server (as D7 says).

**D-SF7. Kerbs and sidewalks in physics: flush carving + visual kerb vs a real kerb step.**
- **Recommend: flush in the heightfield (v1), visual kerb mesh only.**
- A 12 cm step cannot be represented on a 1 m heightfield without a ramp. A real step needs kerb static colliders: a new physics_sim brief, and boundary-spike risk (PHYS-008 class).

**D-SF8. Traffic-calming bumps in physics: carve them vs render only.**
- **Recommend: carve humps, tables and cushions as smooth profile features** (they are real drivable geometry and few: 3.9 k in Hessen).
- Render-only: zero physics risk, but cars fly over painted bumps, which breaks immersion.
- Owner call: it is a gameplay choice for a hypercar game.

**D-SF9. Facades: procedural rules vs street-level imagery extraction vs hand-made asset library.**
- **Recommend: procedural now** (S/M effort; no licence issue).
- Imagery: XL effort, CC BY-SA share-alike, Mapillary terms.
- Asset library: art cost; good for landmarks later (racing_game `docs/buildings.md` already foresees overrides).

**D-SF10. Forest trees: explicit tree list vs CHM raster + client scatter.**
- **Recommend: explicit list everywhere, measured in P5.** Fall back to a 2 m CHM for forest interiors if served bytes exceed ~150 MB for the region [project default].

**D-SF11. Canopy height source: DOM1 (free, leaf-off 2015–2021) vs bDOM20 (newer, fee-based above 5 km² per order) vs Meta 1 m CHM (global ML).**
- **Recommend: DOM1.**

**D-SF12. Season model: rule-based phenology vs Sentinel-2 observed colour.**
- **Recommend: phenology first, Sentinel-2 optional later (P6b).**

**D-SF13. Street-level imagery at all.**
- **Recommend: no direct ingestion.**
- Optional private QA (P8) only with an owner account, and fixes go into OSM.

---

## 8. Questions only the owner can answer

1. **Commercial and licence posture.** The plan avoids every CC BY-SA and NC source. Do you agree we never ingest CC BY-SA street imagery directly (D-SF13), and keep `g2m.features` split by source (D-SF6)?
2. **Mapillary:** do you want an account/token for the optional P8 QA? It is not needed for P0–P7.
3. **Downloads:** approve or deny each item in section 9.
4. **Physics-visible choices:** footprint authority (D-SF5), kerbs (D-SF7), speed bumps (D-SF8).
5. **Game season:** should the in-game date follow the real date, be a menu choice, or be fixed (e.g. summer)? This decides how much of P6 is needed.
6. **G3-C dependency:** `docs/g3_road_pipeline.md` says the endpoint-height/class-grade priority needs your choice before incompatible geometry is served. P1 builds on accepted profiles, so this is still on the critical path. racing_game PLAN.md's G3 status says the earlier grade-policy question was "withdrawn", so the two documents disagree. Please confirm which is current.
7. **Ownership of road visuals:** may the marking/lane logic move out of racing_game `road_visual.cpp` into g2m (P3)? Another session owns that file.
8. **RMS purchase:** buy the FGSV RMS (Part A + RMS-1) so the marking table is exact rather than from secondary sources?

---

## 9. Approvals needed (written before the downloads; status 2026-10-05: LoD2, DOM1, DOP20, Basis-DLM, Baumkataster, tree datasets, HRL and phenology are downloaded, see `geo2map_data/SOURCES.md` and [section 5](#5-data-sources))

| # | Item | Licence | Size | Access | Phase | Account? |
|---|---|---|---|---|---|---|
| A1 | HVBG **LoD2 CityGML**: MTK, HTK, Frankfurt am Main (+ margin municipalities as for DGM1) | dl-de/by-2.0 (corrected 2026-10-05; attribution required) | 0.3–1.5 GB zipped [estimate; downloaded 2026-10-05: 1.2 GB] | manual, gds.hessen.de Download-Center (or the INSPIRE ATOM feed per municipality, which may be scriptable [unverified]) | P4 | no |
| A2 | HVBG **DOM1**, same districts | dl-de/zero ("HVBG terms Zero") | 2.5–4 GB [estimate; DGM1 was 2.8 MB/km² delivered] | manual, Download-Center | P5 | no |
| A3 | HVBG **DOP20 RGB JPEG**, same districts | dl-de/zero-2.0 | 8–15 GB [estimate] | manual, Download-Center (bulk WMS harvesting is possible but hammers their service; not recommended) | P4 (roof colour) | no |
| A4 | Frankfurt **Baumkataster** CSV (Aug 2023) | dl-de/by-2.0 | 20–40 MB [estimate] | offenedaten.frankfurt.de | P5 | no |
| A5 | **Thünen** dominant tree species 2017/18 | CC BY 4.0 | [unverified] | OpenAgrar DOI 10.3220/DATA20221214084846 | P5 | no |
| A6 | **DWD phenology** annual-reporter CSVs | CC BY 4.0 | a few MB | opendata.dwd.de | P6 | no |
| A7 (optional) | HVBG **ATKIS Basis-DLM** Hessen (Shape) | dl-de/zero | 0.5–2 GB [estimate] | Download-Center, Hessen-wide | P1b | no |
| A8 (optional) | Copernicus **HRL TCD/DLT 2018** tile | Copernicus open | 50–200 MB [estimate] | CLMS portal | fallback only | likely EU Login [unverified] |
| A9 (optional) | Sentinel-2 L2A range reads | Copernicus open | 50–200 MB per scene subset [estimate] | AWS COGs | P6b | no |
| A10 (optional) | Mapillary API | CC BY-SA + terms | — | API | P8 | **yes (account, client + user token)** |
| L1 | New tiler-only libraries: **pugixml** (MIT, CityGML), **libjpeg-turbo** (BSD-3/IJG/zlib, DOP decode) | permissive | ~1 MB / ~3 MB source [unverified] | FetchContent, pinned | P4 | no |

Total for the core set (A1–A6): about **11–21 GB** [estimate], to `S:\claude_code\geo2map_data` like the existing downloads.

---

## 10. Open questions and risks to resolve during the phases

- **Home-region tag coverage** may be lower than the Hessen-wide numbers (P0).
- **Dual carriageways and junction clusters** are the hardest part of P1/P2. Budget Opus time and real fixtures (A66 interchanges, Höchst).
- **DOM1 vintage:** leaf-off 2015–2021. How much it underestimates deciduous crowns is [unverified]. P5 measures it against the Frankfurt cadastre (which has crown diameters, but no heights [unverified]).
- **JPEG decoder bit-exactness across platforms** [unverified] for the tiler's DOP sampling. Mitigation in 5.1.
- **LoD2 attribute content** in Hessen (`function`, `roofType`, `storeysAboveGround`) [unverified]. Check on the first municipality file.
- **INSPIRE ATOM feed URL** for scripted LoD2 download: the opendata.hessen.de and GDI-DE pages could not be fetched (403/reset). Verify before writing a script.
- **Tree cadastres of other MTK/HTK towns** (Bad Homburg, Oberursel, Hofheim, Eschborn, …): none found [unverified]. Worth asking the towns, or checking opendata.hessen.de by hand.
- **Mapillary marking classes** and whether continuous lane lines exist as map features [unverified; documentation suggests points and segmentations only].
- **Rural lane-line dash/gap in Germany** [unverified] (see 3.6).
- **Repo observation:** racing_game `docs/road_surfaces.md`, which CLAUDE.md cites for "settings and reconstruction bounds", currently contains C++ Catch2 test source rather than documentation. It looks like an accidental overwrite. Not touched.
- **Physics boundary:** visual kerbs and decals must not change the heightfield. Calming bumps must stay within the R3 seam/spike bounds when driven at speed. Add a wheel-force check to G3-F's route.

---

## References

**geo2map_engine and racing_game (read only):** listed in the header.

**OSM tagging and coverage**
- Geofabrik taginfo Hessen (API used for every count): https://taginfo.geofabrik.de/europe:germany:hessen/
- https://wiki.openstreetmap.org/wiki/Key:lanes
- https://wiki.openstreetmap.org/wiki/Key:turn
- https://wiki.openstreetmap.org/wiki/Key:change
- https://wiki.openstreetmap.org/wiki/Key:placement
- https://wiki.openstreetmap.org/wiki/Key:width
- https://wiki.openstreetmap.org/wiki/Key:crossing
- https://wiki.openstreetmap.org/wiki/Key:crossing:markings
- https://wiki.openstreetmap.org/wiki/Tag:highway%3Dtraffic_signals
- https://wiki.openstreetmap.org/wiki/Tag:highway%3Dstop
- https://wiki.openstreetmap.org/wiki/Key:traffic_calming
- https://wiki.openstreetmap.org/wiki/Key:sidewalk
- https://wiki.openstreetmap.org/wiki/Simple_3D_Buildings
- https://wiki.openstreetmap.org/wiki/Key:roof:shape
- https://wiki.openstreetmap.org/wiki/Tag:natural%3Dtree
- https://wiki.openstreetmap.org/wiki/Key:leaf_type
- https://wiki.openstreetmap.org/wiki/Key:denotation
- ODbL: https://opendatacommons.org/licenses/odbl/1-0/
- Produced Work guideline: https://osmfoundation.org/wiki/Licence/Community_Guidelines/Produced_Work_-_Guideline

**Hessen (HVBG) and other official sources**
- Open data overview: https://hvbg.hessen.de/geoinformation/open-data
- 3D building models (LoD1/LoD2): https://hvbg.hessen.de/landesvermessung/geotopographie/3d-daten/3d-gebaeudemodelle
- LoD2 open-data record (primary metadata: dl-de/by-2.0; ATOM): https://opendata.hessen.de/dataset/inspire-he-gebaude-3d-lod2
- LoD2 WFS: https://www.geoportal.hessen.de/spatial-objects/831
- DOP/TrueDOP product sheet (09/2022): https://hvbg.hessen.de/sites/hvbg.hessen.de/files/2022-09/produktblatt_true_dop.pdf
- DOP20 metadata (dl-de/zero-2.0): https://gdk.gdi-de.org/geonetwork/srv/api/records/0b30f537-3bd0-44d4-83b0-e3c1542ca265
- DOP20 CIR WMS: https://gdk.gdi-de.org/geonetwork/srv/api/records/f10d8ced-a5a3-ad28-29f8-d902e549bb92
- DOM product sheet (04/2026): https://hvbg.hessen.de/sites/hvbg.hessen.de/files/2026-05/produktblatt_oberflaechenmodelle_dom-bf.pdf
- bDOM product sheet (04/2026): https://hvbg.hessen.de/sites/hvbg.hessen.de/files/2026-05/produktblatt_oberflaechenmodelle_bdom-bf.pdf
- Laserscanning Hessen 2 (campaigns 2015–2021, winter flights, ≥ 8 pts/m²): https://hvbg.hessen.de/sites/hvbg.hessen.de/files/2022-09/landesweites_laserscanning_hessen_2.pdf
- ATKIS Basis-DLM product sheet (01/2024): https://hvbg.hessen.de/sites/hvbg.hessen.de/files/2022-09/produktblatt_atkis.pdf
- dl-de/zero-2.0: https://www.govdata.de/dl-de/zero-2-0
- dl-de/by-2.0: https://www.govdata.de/dl-de/by-2-0
- Frankfurt Baumkataster: https://www.offenedaten.frankfurt.de/dataset/baumkataster-frankfurt-am-main
- ALKIS open data across states (FOSSGIS 2025): https://pretalx.com/fossgis2025/talk/TRJW9F/

**Vegetation, phenology, rasters**
- Thünen dominant tree species: https://atlas.thuenen.de/catalogue/csw_to_extra_format/3810c144-756b-11ed-a6bf-0242ac120008/dominant-tree-species-for-germany-20172018.html ; data: https://doi.org/10.3220/DATA20221214084846
- DWD phenology (CC BY 4.0): https://opendata.dwd.de/climate_environment/CDC/observations_germany/phenology/annual_reporters/wild/historical/DESCRIPTION_obsgermany_phenology_annual_reporters_wild_historical_en.pdf
- HRL Tree Cover Density 2018: https://land.copernicus.eu/api/en/products/high-resolution-layer-tree-cover-density/tree-cover-density-2018
- HRL forest description: https://docs.terrabyte.lrz.de/datasets/hrl_forest/description/
- Urban Atlas Street Tree Layer: https://land.copernicus.eu/local/urban-atlas/street-tree-layer-stl-2018
- Copernicus Sentinel data licence: https://ads.atmosphere.copernicus.eu/licences/ec-sentinel
- Meta/WRI canopy height v2 (CC BY 4.0): https://registry.opendata.aws/dataforgood-fb-forestsv2 ; https://data.source.coop/tge-labs/meta-chm-v2/LICENSE
- ETH global canopy height: https://research-collection.ethz.ch/handle/20.500.11850/636572
- ESA WorldCover: https://esa-worldcover.s3.eu-central-1.amazonaws.com/readme.html

**Buildings (worldwide fallbacks)**
- Overture attribution and licences: https://docs.overturemaps.org/attribution/
- Overture buildings guide: https://docs.overturemaps.org/guides/buildings
- EUBUCCO: https://ual.sg/publication/2023-sd-eubucco/

**Street-level imagery**
- Mapillary terms (effective 2024-02-15; s. 3(b), 11, 12, 13): https://www.mapillary.com/terms
- Mapillary for OSM: https://www.mapillary.com/osm
- Mapillary API documentation: https://www.mapillary.com/developer/api-documentation
- Mapillary CC BY-SA help page: https://help.mapillary.com/hc/en-us/articles/115001770409-CC-BY-SA-license-for-open-data
- Panoramax: https://wiki.openstreetmap.org/wiki/Panoramax ; https://docs.panoramax.fr/federated-catalog/
- KartaView: https://wiki.openstreetmap.org/wiki/KartaView
- Google Maps Platform terms (s. 3.2.4(b)): https://cloud.google.com/maps-platform/terms ; summary: https://conductatlas.com/platform/google-maps/google-maps-platform-terms-of-service/ai-and-ml-training-data-prohibition/

**Rules and algorithms**
- Fahrbahnmarkierung (RMS summary): https://de.wikipedia.org/wiki/Fahrbahnmarkierung
- RMS Part A (2019) index: https://umwelt-online.de/recht/bau/strassen/rms_in.htm
- osm2streets (Apache-2.0): https://github.com/a-b-street/osm2streets
- A/B Street geometry: https://a-b-street.github.io/docs/tech/map/geometry/index.html
