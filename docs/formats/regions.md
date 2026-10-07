# Regions: `g2m.regions/1` and `g2m.resolved_region/1`

A *region* is a named piece of geometry an importer (`g2m_tiler`) clips
against: either a literal UTM bounding box, or an OSM administrative-
boundary relation resolved against a full-planet PBF extract. A
*regions.json* file (format `g2m.regions/1`) is what a human authors; a
*resolved-region* file (format `g2m.resolved_region/1`) is the artefact
`region_resolve.h` produces from it against one specific PBF snapshot, and
what every importer (`import_dem`/`import_osm`/`import_landcover`) actually
consumes.

Code: `g2m/layer/region.h`, `src/layer/region.cpp`. Ring assembly itself
(shared by region resolution) is `g2m/core/geom/ring_assembly.h` +
`ring_set.h` + `exact.h`.

**Datum** (PLAN.md section 4.1): OSM node coordinates (used by an
`osm_admin` part) and every other geodetic source this project imports
(e.g. `docs/formats/src_landcover.md`'s ESA WorldCover) are WGS84; this
project's own UTM grid (`g2m::geo::Utm`/`project_e7_to_mm`) is GRS80/
ETRS89. The two differ by up to ~0.5 m at this project's latitudes (a
frame-realisation drift, not an ellipsoid shape difference), which every
importer deliberately ignores as sub-metre - no reprojection or
datum-shift step exists anywhere in region resolution or import.

## `regions.json` (`g2m.regions/1`)

Canonical on write (object keys sorted, no whitespace - the same rules as
[release_manifest.md](release_manifest.md)), but hand-authored on disk, so
the parser is STRICT: every unknown key anywhere in the document is
rejected (unlike `release_manifest_from_json`'s forward-compatible "ignore
unknown keys" - a typo here should fail loudly, not silently vanish).

```json
{
  "format": "g2m.regions/1",
  "regions": [
    {
      "name": "home",
      "scheme": "utm32n",
      "halo_m": 2000,
      "parts": [
        {"kind": "utm_bbox", "west": 474000, "south": 5560000, "east": 480000, "north": 5566000},
        {"kind": "osm_admin", "relation": 62650, "expect_name": "Frankfurt am Main", "expect_admin_level": 6}
      ]
    }
  ]
}
```

Rules:

- `name`: `[a-z0-9_]{1,64}`, unique within the file.
- `scheme`: a UTM scheme name (`geo::parse_utm_scheme`, e.g. `"utm32n"`).
  Every `utm_bbox` part's metres, and every tile the region resolves to,
  are in this zone.
- `halo_m`: `0..20000`. Widens every part's own tile footprint by this
  many metres before classifying a tile Boundary/Interior/Outside (see
  "Halo and tile classification" below) - the same margin `clip_osm`'s
  own `halo_m` uses for a single tile, applied here per region.
- `parts`: at least one. Each part is one of:
  - `{"kind": "utm_bbox", "west", "south", "east", "north"}` - whole
    metres in `scheme`'s zone. Requires `west < east` and `south < north`.
  - `{"kind": "utm_corridor", "half_width_m", "points": [[e, n], ...]}` - a
    strip around a polyline (a motorway): every point within `half_width_m`
    (1..20000) of the polyline, plus a square cap of the same size beyond
    each end. `points` are whole metres in `scheme`'s zone, at least two,
    consecutive points distinct. Resolved as ONE RECTANGLE PER SEGMENT
    (`g2m::geom` has no polygon union): a segment's rectangle spans the
    segment, `half_width_m` beyond each end and `half_width_m` to each side, so
    consecutive segments overlap at a bend with no notch. The tile footprint is the
    union of the rectangles' own `tiles_near` sets; the persisted
    `vertices` is `4 * segments` and `geometry_hash` covers the rectangles'
    rings (millimetre-rounded corners, `sqrt` is the only non-`+ - * /`
    operation and is correctly rounded under IEEE 754). A corridor part
    carries no `osm` dependency, so it resolves identically against any PBF.
  - `{"kind": "osm_admin", "relation", "expect_name"?, "expect_admin_level"?}`
    - `relation` is the OSM relation id (a positive integer). If given,
    `expect_name`/`expect_admin_level` are checked against the resolved
    relation's own tags at resolution time and a mismatch is an error - a
    drift check for when the upstream OSM data changes under the region
    author's feet, since identity here is the relation id, not a name
    lookup (a name can be renamed or reused).
- A region with more than one part covers their union (e.g. a city plus a
  bounding box pad around it); parts may overlap.

## Ring assembly rules

An `osm_admin` part resolves to a `g2m::geom::RingSet` (`g2m/core/geom/
ring_set.h`) via `g2m::geom::assemble_rings` (`ring_assembly.h`), fed the
relation's `outer`/`inner` member ways (other roles are ignored). All
arithmetic is exact integer millimetres (`g2m::geom::exact.h`) - no
libm transcendental, so the result is bit-identical on every platform
(PLAN.md D7).

1. Member ways are sorted by way id; a duplicate way id is an error.
2. Every node id is resolved to a projected `PointMm`
   (`g2m::geom::project_e7_to_mm`, the same deterministic Transverse
   Mercator projection `clip_osm` uses); an unresolved node id is an
   error.
3. A way whose first and last node id already match is a ring on its own.
4. Every other (open) way's two endpoint node ids must have even degree
   among the open ways, else "unclosed ring at node N (way W)".
5. Open ways are chained end-to-end starting from the lowest unused way
   id; at a junction (several unused ways share a node), the lowest way
   id is taken.
6. A chain splits into a completed ring wherever a node id repeats (a
   figure-eight case), continuing from that node with whatever chain
   remains.
7. Consecutive equal points (including a ring's own repeated closing
   point) are dropped; a ring left with fewer than 3 distinct points is an
   error.
8. Any proper crossing or collinear overlap between two segments -
   whether in the same ring or different rings - is an error; segments
   are allowed to share a single vertex.
9. Each ring is classified outer/inner by even-odd nesting depth (even =
   outer) - not by the member ways' own declared roles, which are only
   compared against this result for an informational mismatch count. A
   ring tagged `"inner"` that is geometrically an outer boundary (or vice
   versa) is still assembled and used correctly; only the mismatch
   counter is affected.
10. Each ring is normalised: outer rings wind CCW, inner rings CW, and
    each ring starts at its own lexicographically smallest point (so two
    equivalent inputs - e.g. the same ways listed in a different order,
    or a ring started from a different vertex - assemble to byte-identical
    rings).

## Halo and tile classification

`g2m::geom::classify_square`/`tiles_near` (`ring_set.h`) relate a UTM tile
square to a `RingSet`:

- **Boundary** - a ring segment touches the square, or comes within
  `halo_m` of it.
- **Interior** - the square is inside the ring set and no segment touches
  or comes within `halo_m`.
- **Outside** - neither.

The halo distance is computed in `double`, using only `+ - * /` (never
`sqrt` or another libm transcendental), so it is exact and identical on
every platform - a squared-distance comparison, not a true Euclidean
distance. `tiles_near` bins ring segments into a uniform grid (cell size =
one tile) dilated by `halo_m` on every side before testing: Euclidean
distance <= `halo_m` implies both the x and y gaps are <= `halo_m`, so this
box dilation never misses a real candidate.

## Resolved-region artefact (`g2m.resolved_region/1`)

The output of resolving a `RegionSpec` against one specific PBF snapshot
(`region_resolve.h`, I2). Canonical JSON, same rules as `regions.json`
(sorted keys, no whitespace, unknown keys rejected - a build artefact
checked into a repo should fail loudly if corrupted or hand-edited, not
silently accept a typo).

```json
{
  "format": "g2m.resolved_region/1",
  "name": "home",
  "scheme": "utm32n",
  "level": 2,
  "halo_m": 2000,
  "parts": [
    {"kind": "utm_bbox", "id": 0, "name": "", "vertices": 4, "geometry_hash": "<16 hex>", "role_mismatches": 0},
    {"kind": "utm_corridor", "id": 0, "name": "", "vertices": 8, "geometry_hash": "<16 hex>", "role_mismatches": 0},
    {"kind": "osm_admin", "id": 62650, "name": "Frankfurt am Main", "admin_level": 6,
     "vertices": 1842, "geometry_hash": "<16 hex>", "role_mismatches": 0}
  ],
  "pbf_name": "hessen-latest.osm.pbf",
  "pbf_sha256": "<64 hex>",
  "tiles": [[y, x0, x1], ...],
  "interior": [[y, x0, x1], ...],
  "tile_count": 1234
}
```

Rules:

- `pbf_name` / `pbf_sha256` with several PBFs (`g2m_tiler resolve|import ... --pbf A --pbf B`,
  `resolve_region(spec, vector<string>)`): the names joined with `+` in the
  given order, and the SHA-256 of the files' own 32-byte digests concatenated
  in that order (for ONE file it is that file's own digest, so every
  single-file artefact is unchanged). Each `osm_admin` part resolves against
  the FIRST file that holds its relation complete (an extract clips a relation
  at its own border, so a file that lacks a member way is skipped); the
  error of the last file is reported when none does. Parts of kind
  `utm_bbox`/`utm_corridor` never read a PBF.
- `level` is always 2 today (the tiling grid every source layer uses),
  carried explicitly rather than assumed.
- Each part's assembled ring geometry (`g2m::geom::RingSet`) is NOT part of
  this JSON - only `geometry_hash` (Fnv1a64, 16 lower-case hex digits, over
  the canonical ring bytes: each outer then each inner ring's point count
  and x/y values, in the ring set's own already-canonical order) and
  `vertices` (the total vertex count over every ring) are persisted. This
  keeps the artefact small and auditable instead of a raw geometry dump;
  an importer that needs the actual rings re-resolves them (deterministic,
  so re-resolution against the same PBF snapshot reproduces the same
  `geometry_hash`).
- `role_mismatches` (`osm_admin` only, always `0` for `utm_bbox` - no OSM
  roles involved): the number of member ways whose declared `outer`/`inner`
  role (rule 9 above) disagreed with the ring's own geometric nesting
  classification. The ring is still assembled and used correctly either
  way (classification is by nesting depth, never by the declared role) -
  this is purely a data-quality signal for the region author, surfacing
  when the source relation's own tagging has drifted from its geometry.
- `tiles` is the union of every part's own `tiles_near` result (Boundary
  or Interior tiles - the region's whole footprint), as level-2 row runs
  `[y, x0, x1]` (inclusive), the same run-length encoding
  [release_manifest.md](release_manifest.md) uses for layer coverage.
  `interior` is the subset that is Interior for at least one part.
  `tile_count` is the number of distinct tiles in `tiles`.
- `pbf_sha256` is the SHA-256 of the PBF file resolution was run against
  (64 lower-case hex digits) - so a resolved-region file states exactly
  which upstream snapshot it was resolved from.

## Selector syntax

A CLI region selector is `path#name`: the path to a `regions.json` file,
a `#`, then a region `name` from that file (e.g.
`--region S:\...\regions.json#home`). `split_selector` splits at the FIRST
`#` (a region name is `[a-z0-9_]{1,64}` and never contains one, so a path
itself containing `#` still splits correctly); either half being empty is
an error.
