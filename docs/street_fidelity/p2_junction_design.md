# P2: junction surfaces, server-side carving and junction furniture — implementation design

Status: design 2026-10-05. Slices 1-7 implemented (78d189d rules, 399b460 surface
function, 68985c6 single-node junctions, 9826c64 clusters, 90de609 junction heights and raised tables, e73312f profile completion
tier, slice 7 furniture). Deviations found while implementing are recorded in section 15 (D16-D28); the owner answers are in section 17.
Audience: the implementing agents. Each slice in section 14 can be built from this
document plus the P1 design ([p1_cross_section_design.md](p1_cross_section_design.md)).
P2 is G3-D (PLAN.md 6.3 steps 4-7) plus the furniture additions of
[plan.md](plan.md) section P2.

Markers used below:
- **[measured]**: counted by this design pass on the two committed OSM fixtures with a
  throwaway probe (`out/p2_probe/`, not committed) against the release build of
  `107bf2b`. Fixture A = `frankfurt_hoechst.osm.pbf` (really Kalbach/Bonames, A 661),
  fixture B = `hoechst_a66.osm.pbf`.
- **[estimate]**: a probe number from a simplified model (straight rays, widths from
  `lanes` only), good for sizing, not for acceptance.
- **[RMS]**, **[RASt]**, **[R-FGÜ]**: value from the named German guideline, read
  from secondary sources. **[unverified]**: the secondary source was ambiguous.
- **[assumed]**: a project default with no citation.
- **[game]**: the value racing_game uses today, kept for parity.

Contents:
1. Scope, baseline and fixture numbers
2. Data model
3. Junction geometry: approaches, clusters, corners, trims, rings
4. Heights: the road surface function and the junction surface
5. Tile invariance
6. Carving: `g2m.terrain.carved` and `g2m.terrain.class`
7. Junction furniture
8. Rules as data
9. Encodings
10. Consumer switch: what racing_game needs and deletes
11. Tests
12. Measurement plan
13. Risks
14. Implementation slices
15. Decisions record
16. What P2 changes in P1's implementation
17. Owner questions

---

## Summary

- A **junction** is a node of degree ≥ 3 in the merged context graph, or a cluster of
  such nodes whose approach trims overlap on the stretch between them. Caps (48 m
  extent, 12 nodes) keep clusters small enough to prove tile invariance.
- Its **polygon** comes from the osm2streets offset-intersect-trim algorithm, ported
  to C++ on integer-millimetre edge polylines and exact predicates. Each approach is
  trimmed at the farthest corner where its edges meet its angular neighbours' edges.
  Three rings are stored: road bed, paved, outer.
- **Heights inside the junction** are an inverse-squared-distance (Shepard) blend of
  each approach's own surface. On every trim line the junction equals the approach
  exactly, so the junction matches every approach to far below 1 cm. The normal slope
  is continuous (C1), and the surface stays inside the approaches' height range.
- One **canonical road surface function** (profile + crown with a 0.5 m rounded ridge
  + verge drop + carved features), without square roots, is the only definition of a
  carved road height. Carving, junctions and tests all call it.
- **roads.geom encoding 7** adds section 5 (junctions), section 6 (furniture) and
  section 7 (profile provenance). Sections 1-4 keep their format.
- A **profile completion tier** moves the game's `complete_road_profiles` and its
  20 m retry into the library, with provenance flags. Without it the game could not
  drop its own reconstruction.
- Two new baked layers:
  - **`g2m.terrain.carved`** (L0 carved, L1-L6 mean-of-4 of the carved L0). The bare
    `g2m.terrain.height` stays as it is.
  - **`g2m.terrain.class`**: node-registered 257 x 257 at integer metres, so a sample
    sits exactly at a physics cell centre. It holds land class, surface, role and road
    rank, from the same per-cell ownership function as the heights.
- **Carving per cell**:
  - the ported game smoothing comes first (integer kernel);
  - then cores and junctions: the lowest road wins, junction over approach;
  - then blends of 3-15 m with a 1:1.5 slope rule (the blend with the largest weight
    wins);
  - then the deck clearance clamp. Bridges and tunnels are never carved.
- **Furniture**:
  - crossings as quads perpendicular to the reference, with a pattern from tags;
  - stop and give-way lines over exactly the approach-direction driving lanes, set
    back 1 m before a crossing;
  - signal positions and facing.
  Every source node gets a placed record or a declined one with a reason code.
- **racing_game deletes** the smoothing, `RoadSurfacePatch` carving, the deck
  clearance cut, `complete_road_profiles` with its retry, and the src.osm road
  raster. It drapes ribbons on the heightfield triangles, so what it shows and what
  it simulates come from one height source and one class source.

---

## 1. Scope, baseline and fixture numbers

### 1.1 In scope

- Junction polygons from P1 carriageway/footprint edges (D-SF4: port, do not link).
- Junction heights matching every approach.
- G3-D carving of `terrain.carved` and `terrain.class` from the same geometry. The
  bare height layer stays the default until a consumer selects the carved layers.
- Junction furniture: crossings, stop and give-way lines, signals, setbacks from a
  rule table.
- The consumer contract and the exact list of what racing_game removes.

### 1.2 Not in scope

- Markings (P3).
- Bridge bodies, ribbons and meshes (G3-E).
- Routing and turn restrictions (G4).
- Footway geometry: footways are not in `roads.graph`, so crossings are
  perpendicular to the road.
- Tunnels (not drivable in v1).
- Kerb colliders (P3+, owner decision P1-Q1).

### 1.3 Baseline (code at `107bf2b`)

- **Junction heights.** Every stretch endpoint at a junction node uses
  `RoadDemSampler::junction_plane`: the median height over a 3 m footprint plus the
  median rational gradients.
  - All profiled approaches at a node share one height and one plane. The profile end
    grade is the plane gradient projected on the heading.
  - A node with exactly one bridge takes an anchor inferred from 32 m ground
    approaches (`road_profile.cpp` `raw_anchor`).
  - A node with two or more bridges declines those bridges (`Unsupported`).
  - Tunnels are declined.
- **Stretch ends.** At a junction end P1 sets `JoinKind::None`, and turn pockets run
  full width to the node. Profile features skip stretch end nodes ("the junction owns
  them in P2", P1 8.3).
- **Node tags.** `roads.graph` encoding 5 already carries every node tag P2 reads:
  `highway`, `crossing`, `crossing:markings`, `direction`,
  `traffic_signals:direction`, `width`, `kerb`, `traffic_calming`,
  `traffic_calming:length`. **No whitelist change and no roads.graph encoding bump.**
- **Carving in racing_game today** (`world_terrain.cpp` `fetch_and_decode`, L0 only;
  `road_surface.cpp`, `terrain_smoothing.cpp`, `road_structures.cpp`) [game]:
  - Gaussian smoothing: radius 2 m, sigma 1, strength 0.35, one pass, libm `exp`.
  - Nearest-segment ownership by normalised distance.
  - Rounded crown `c·(sqrt(d²+0.25)−0.5)` about the centre line.
  - Verge drop 0.06 m via a smooth step over the shoulder.
  - Blend weight `1 − smoothstep` over `blend_mm`.
  - A 0.35 proximity blend between roads of the same layer.
  - Deck clearance: terrain ≤ deck_top − 0.55·S(edge/4).
  - Tunnel-roof decks that suppress carving.
  - `complete_road_profiles`:
    - bridge spans re-fitted from abutments;
    - ground stretches next to bridge anchors re-fitted with relaxed limits (max
      grade ≥ 12 %, end grades clamped ±10 %, 40 m samples);
    - tunnel floors.
  - A 20 m / 16384-iteration retry for "bounded solver" and "endpoint curvature"
    declines.
  - Physics and render share the carved L0 tile. L1-L6 are uncarved.
  - Grip comes from `extract_road_segments` (src.osm) and colours from a `ClassLookup`
    road raster.

### 1.4 Fixture numbers [measured]

Degree = incident edge ends in the fixture's own graph (a way passing through
counts 2).

| | A (A 661) | B (Höchst A 66) |
|---|---|---|
| ways / nodes / edges | 739 / 3136 / 3309 | 472 / 2025 / 2129 |
| junction nodes (degree 3 / 4 / ≥5) | 480 / 56 / 1 | 278 / 32 / 0 |
| stretches | 1152 | 666 |
| junction-junction stretches < 25 m (< 10 m) | 183 (49) | 107 (34) |
| PLAN 25 m clustering: cluster sizes | 1:267 2:60 3:16 4:5 5:7 6:2 9:2 **17:1** | 1:157 2:35 3:8 4:5 5:1 9:1 11:1 **14:1** |
| 25 m clusters > 40 m extent / > 8 nodes / max extent | 8 / 3 / **88.3 m** | 3 / 3 / **106.3 m** |
| trim-overlap clustering (section 3.2) sizes | 1:451 2:27 3:4 4:1 7:1 9:1 | 1:244 2:16 3:4 4:4 6:1 |
| trim-overlap > 40 m / > 8 nodes / max extent | 1 / 1 / 40.2 m | 1 / 0 / 45.3 m |
| **implemented** clustering (slice 4; records counted in the owner node's L2 tile, context degrees, real trims) | 1:482 2:19 3:2 4:1 7:1; 32 absorbed stretches; max extent 27.6 m; 0 `ClusterTooLarge` | 1:271 2:14 3:2 5:1; 24 absorbed; 35.4 m; 0 |
| roundabout ways (junction nodes on them) | 3 (4) | 0 |
| junction nodes with a bridge / tunnel approach | 1 / 1 | 2 / 5 |
| junction nodes with an approach pair < 15° | 12 | 22 |
| corner trim per approach side p50 / p90 / p99 / max [estimate] | 3.7 / 5.6 / 20.5 / 45.4 m | 3.7 / 9.1 / 42.9 / 78.0 m |
| stretches whose two end trims overlap [estimate] | 53 | 42 |
| `highway=crossing` nodes: interior or join / junction / dead end | 121 / 0 / 2 | 50 / 2 / 0 |
| crossing:markings zebra / dashes / yes / no / absent | 12 / 2 / 11 / 48 / 50 | 0 / 1 / 13 / 7 / 31 |
| crossing= traffic_signals / marked / uncontrolled / unmarked / absent | 29 / 13 / 25 / 39 / 17 | 26 / 16 / 1 / 8 / 1 |
| `highway=traffic_signals`: interior / junction / end | 27 / 0 / 0 | 35 / 0 / 1 |
| signal direction tag forward / backward / none | 12 / 3 / 12 | 23 / 6 / 7 |
| `highway=stop` (direction) / `give_way` | 2 (fwd 1, bwd 1) / 1 | 6 (fwd 2, bwd 1, both 2, none 1) / 1 |

Three facts shape the design:

1. **The PLAN's 25 m rule chains dual-carriageway grids into 14-17-node clusters up
   to 106 m across.** Linking nodes only where the approach trims actually overlap
   gives at most 9 nodes and 45 m. Section 3.2 adopts that rule.
2. **Signals are mapped on approach nodes, not on junction nodes**, in both fixtures,
   mostly with a direction tag. The stop line therefore sits at the signal node, and
   the junction-node rule is the minority path (risk in plan.md P2).
3. **Shallow merges** (< 15°, motorway ramps) give trims of 40-80 m. Section 3.6
   gives them their own cap and decline path.

---

## 2. Data model

All new public types go in `include/g2m/layer/road_junction.h` and
`road_furniture.h`. Stations and lateral offsets follow P1: metres, t positive left of
the reference direction. Positions are integer mm in the tile's UTM frame
(`geom::PointMm`).

```cpp
namespace g2m {
// ---- junctions (section 3) ----
enum class JunctionApproachStatus : std::uint8_t {
    Used = 0,
    NoProfile = 1,        // entry declined: excluded from the ring and the heights
    NoCrossSection = 2,   // profile present, cross-section declined
    Tunnel = 3,
};
struct JunctionApproach {
    std::size_t entry_index = 0;      // into RoadGeomTile::entries of the tile holding the junction
    bool at_end = false;              // the junction is at the stretch's end_ref (else start_ref)
    JunctionApproachStatus status = JunctionApproachStatus::Used;
    double trim_m = 0;                // distance from the junction-side stretch end to the trim line
};
enum class JunctionRingLayer : std::uint8_t { RoadBed = 0, Paved = 1, Outer = 2 };
struct JunctionRing { std::vector<geom::PointMm> vertices; };  // counter-clockwise, simple, no repeated vertex
enum class JunctionFlags : std::uint16_t {
    Cluster = 1,          // more than one node
    OnStructure = 2,      // every used approach is a bridge: polygon served, never carved
    HullFallback = 4,     // the offset ring was not simple; the convex hull of the trim segments is used
    Partial = 8,          // at least one approach is excluded
    RaisedTable = 16,     // a vertical traffic_calming feature on a junction node (section 4.4)
};
enum class JunctionDecline : std::uint16_t {
    None = 0,
    ClusterTooLarge = 1,  // extent > 48 m or > 12 nodes: members fall back to single-node junctions
    ReachExceeded = 2,    // a ring vertex > 128 m from the nearest cluster node
    SingleApproach = 3,   // fewer than two used approaches
    Degenerate = 4,       // even the hull fallback has < 3 non-collinear vertices
    Underground = 5,      // every approach is a tunnel
};
struct RoadJunction {
    std::vector<std::int64_t> nodes;            // ascending OSM ids; nodes.front() is the owner node
    std::vector<JunctionApproach> approaches;   // angular order, counter-clockwise, starting at the smallest
                                                // (way id, stretch start_ref, at_end)
    std::vector<std::size_t> absorbed;          // entry indices of the stretches inside a cluster
    std::array<JunctionRing, 3> rings;          // empty when declined
    std::uint16_t flags = 0;
    JunctionDecline decline = JunctionDecline::None;
    double raised_height_m = 0;                 // RaisedTable only
};

// ---- furniture (section 7) ----
enum class FurnitureKind : std::uint8_t { Crossing = 0, StopLine = 1, GiveWayLine = 2, Signal = 3 };
enum class CrossingPattern : std::uint8_t { None = 0, Zebra = 1, Dashes = 2, Ladder = 3, Lines = 4, Surface = 5 };
enum class FurnitureDecline : std::uint16_t {
    None = 0,
    StretchDeclined = 1,       // the node's stretch has no profile or no cross-section
    Tunnel = 2,
    JunctionNode = 3,          // a crossing on a degree >= 3 node: the crossing direction is unknown
    DirectionUnresolved = 4,
    NoLanes = 5,               // driving_lanes_at returns 0 for the travel direction
    Ambiguous = 6,             // a stop/give_way on a junction node whose approaches all have the same rank
    JunctionDeclined = 7,      // a junction-node signal or stop at a declined junction
};
struct RoadFurniture {
    FurnitureKind kind = FurnitureKind::Crossing;
    FurnitureDecline decline = FurnitureDecline::None;
    std::int64_t node_id = 0;          // source OSM node (the junction's owner node for a derived line)
    std::size_t entry_index = 0;       // the stretch it sits on
    double station_m = 0;
    LaneDirection travel = LaneDirection::Forward;  // lines and signals: the direction it controls
    double t_from = 0, t_to = 0;       // lateral extent (crossing: carriageway; line: lanes; signal: t of the post)
    double length_m = 0;               // along the road: crossing width, line width
    CrossingPattern pattern = CrossingPattern::None;
    std::uint8_t flags = 0;            // bit0 signalised crossing, bit1 lowered kerb, bit2 spans a join,
                                       // bit3 inside a junction ring, bit4 left-side signal
    std::uint32_t junction_index = 0;  // 1 + index into the junction list for lines/signals derived from a junction, else 0
};

// ---- profile provenance (section 6.6), one per entry ----
enum class ProfileProvenance : std::uint8_t {
    Strict = 0,              // build_road_profile at the policy limits
    Retry = 1,               // 20 m samples, 16384 iterations, same limits (game parity)
    BridgeCompleted = 2,     // span re-fitted between abutments
    GroundCompleted = 3,     // relaxed ground fit next to a bridge anchor or after a strict decline
};
struct RoadProfileOrigin {
    ProfileProvenance provenance = ProfileProvenance::Strict;
    std::uint16_t strict_decline_code = 0;  // the strict decline that was replaced (0 for Strict)
};
}
```

`RoadGeomTile` gains:
- `std::vector<RoadJunction> junctions` (only those the tile owns, section 5);
- `std::vector<RoadFurniture> furniture`;
- `std::vector<RoadProfileOrigin> origins` (one per entry; empty for encoding ≤ 6).

### 2.1 Library functions (pure, allocation-free where marked)

```cpp
// Section 4.1: the one carved road height. s, t in the entry's frame; t is clamped to the outer edges.
double road_surface_height_m(const RoadGeomTile&, std::size_t entry, double s, double t) noexcept;
// Section 4.2: junction surface at a world point (metres). Requires the point to be inside the outer ring.
double junction_height_m(const RoadGeomTile&, const RoadJunction&, double x, double y) noexcept;
// Section 7: the world quad of a crossing, the segment of a line, the post of a signal.
Result<std::array<LateralPoint, 4>> furniture_quad(const RoadGeomTile&, const RoadFurniture&);
// Section 6.3: the per-cell carve evaluation shared by terrain.carved and terrain.class.
struct CarveSample { std::int32_t height; std::uint8_t land_class, surface, role, rank; };
class CarveModel { /* built from <= 4 roads.geom tiles + the smoothed DEM window; evaluate(x_mm, y_mm) */ };
```

---

## 3. Junction geometry

### 3.1 Junction nodes and approaches

- **Junction node:** degree ≥ 3 in the **merged context graph** (P1 slice 1). Degree 2
  is a P1 way join. Degree 1 is a dead end: no polygon.
- **Approach:** one per (stretch, end) incident to the junction. A way passing through
  a node gives two approaches. A stretch that starts and ends at the same junction
  (a loop) gives two approaches.
- **Approach frame:** σ runs outward from the junction: σ = s if `!at_end`, else σ = L − s.
  Outward-left is +t if `!at_end`, else −t. All approach geometry below is stated in
  (σ, outward-left).
- **Status:**
  - `NoProfile`: the entry has no profile, even after the completion tier.
  - `NoCrossSection`: the cross-section declined.
  - `Tunnel`: the way is a tunnel.
  Excluded approaches take no part in the rings or the heights. They are listed so
  consumers can count them.

### 3.2 Clusters

The probe (1.4) shows the PLAN's "within 25 m" rule makes chains, not junctions.
**Rule:**

1. Compute the single-node corner trims of every junction node (3.3-3.4). The link
   test caps each trim at 30 m, so a shallow merge never links on its own.
2. Two junction nodes `a`, `b` are **linked** when a stretch `e` joins them directly
   (`a ≠ b`, no other node of degree ≠ 2 in between) and
   `min(trim_a(e), 30) + min(trim_b(e), 30) ≥ L(e)`. In words: the two junction
   surfaces would overlap on `e`. Stretches of `junction=roundabout|circular` ways
   never link (3.6).
3. A cluster is a connected component of links. The stretches that link the cluster
   are **absorbed**: no core, inside the polygon. Any other stretch with both ends in
   the cluster is absorbed too, if both of its trims overlap.
4. **Caps:** bounding box ≤ 48 m per axis and ≤ 12 nodes.
   - A component over a cap is declined `ClusterTooLarge`: one record holds the
     reason, and every member node falls back to a single-node junction.
   - Probe estimate: 1 such component per fixture. Implemented (slice 4): 0 in both
     fixtures (1.4).
   - Cluster discovery is a BFS over links that stops as soon as a cap is exceeded.
     It therefore never visits a node more than 48 m + one link (≤ 60 m) away from
     where it started (section 5).

Alternatives in D2 (section 15).

**As implemented (slice 4, `build_road_junctions`):**
- "Directly" means one `roads.geom` entry. A connection that runs over a degree-2 way
  join is two entries and never links (D20); measured 0 such pairs whose trims would
  cover them in either fixture.
- A possible link to a node the tile sees only partially (`ta + 30 ≥ L`) makes the
  discovery **unresolved**: that tile skips the node, the owner tile resolves it (D20).
- Rule 4's "every member falls back" keeps the members' single records, and the
  smallest node of each discovery set that exceeds a cap stores the `ClusterTooLarge`
  record (D19).
- Rule 3's second sentence covers loops at one member node; two members joined by two
  parallel stretches absorb both when both link.

### 3.3 Edge polylines and corners (the ported osm2streets step)

For each used approach, at the junction end:

- **Edge polylines.** Sample σ = 0, Δ, 2Δ, … up to `σ_max = min(L − 0.5, 128)` with
  Δ = 0.5 m, plus σ_max. At each σ, take `carve_footprint(xs, s(σ))` and form three
  edge pairs: road bed, paved, outer. Each point is `lateral_point(reference, s, t)`,
  rounded to integer mm (`llround`).
  - Rounding error ≤ 0.5 mm. The chord sagitta at Δ = 0.5 m and the minimum radius of
    10 m is 3.1 mm.
- **Angular order.**
  - Single node: sort approaches by the direction of their first polyline segment.
  - Cluster: ~~sort by the direction from the cluster centroid~~ replaced by the
    boundary walk of D16: every node sorts its own darts (stubs and the ends of
    absorbed stretches) as for a single node, and the ring follows the outer face.
  - Comparison is exact: half-plane, then the sign of the integer cross product. Ties
    break on (way id, start_ref, at_end).
- **Corner of an adjacent pair (i, i+1)** (outward-left edge of i against
  outward-right edge of i+1, both on the **outer** layer):
  - The first intersection, in increasing `max(segment index of i, segment index of
    i+1)`, then increasing index of i, found with `geom::exact::segments_intersect`.
  - The point is the exact rational intersection rounded to mm. Its station on each
    approach is the segment's start station plus the segment parameter × Δ.
  - If no intersection exists within σ_max, the pair is **open**: a straight-through
    pair at a T, or an angular gap over 180°.
  - **Caps (D17, slice 4).** Each edge polyline starts with a cap segment from the
    approach's reference point at σ = 0 out to its edge point at σ = 0 (station −1 to
    0 on the cap). A pair whose edge lines meet behind the node (an obtuse pair with
    unequal offsets) closes on a cap instead of staying open; cap against cap never
    counts. Trims use max(station, 0).

### 3.4 Trims

- `trim_i = max(corner station of (i−1, i) on i, corner station of (i, i+1) on i,
  trim_min) + trim_margin`.
  - Open pairs contribute 0.
  - `trim_min` = 1 m. ~~For a cluster it is also at least the projection of every
    cluster node onto approach i's direction + 1 m~~ — dropped (D18): with the
    boundary walk every node lies inside the ring by construction.
  - In a cluster only the stubs (approaches that are not absorbed) have trims; a
    corner on an absorbed stretch moves no trim.
  - `trim_margin` = 0.5 m.
  - All values come from the rules (8.1).
- **Clamp:** `trim_i ≤ L_i − 0.5`. When the clamp bites, the junction gets flag
  `Partial` only if the corner itself lies beyond the clamp. That approach's whole
  remaining length is then junction surface.
- Trims come from the **outer** layer, because the carve domain is the outer
  footprint. The road bed and paved rings use the same trims, so their corners lie
  inside them.

### 3.5 Rings

For each layer, walk the approaches counter-clockwise:

```
for i in approaches:
    emit right trim point of i (outward-right edge at σ = trim_i)
    emit left trim point of i
    if (i, i+1) has a corner on this layer:
        emit i's outward-left edge polyline from trim_i back to the corner (exclusive),
        then the corner,
        then i+1's outward-right edge polyline from the corner out to trim_{i+1} (exclusive)
    else (open pair, or no corner on this layer within the trims):
        emit i's outward-left edge back to σ = 0, then i+1's outward-right edge from σ = 0 out
```

Then:

1. Remove consecutive duplicates and collinear interior points. Collinearity is the
   exact integer orientation = 0.
2. Check that the ring is simple with exact predicates. The cost is O(n²) segment
   pairs; n ≤ 2000 in the worst case [estimate] and below 100 typically.
3. If it is not simple (a bow-tie), do up to **3 repair rounds**. Each round adds 1 m
   to both trims of every approach whose edge takes part in a crossing pair, then
   rebuilds.
4. If the ring is still not simple, use the **hull fallback**: the convex hull of
   all trim points (always simple), flag `HullFallback`. In a cluster the hull also
   takes the edge samples of the absorbed stretches. **When one layer needs the hull,
   all three layers take it (D17)**: hulls of points on the same trim lines nest, a
   hull next to an offset ring does not.
5. A hull with fewer than 3 non-collinear points declines `Degenerate`.
6. **Reach:** every vertex must lie within 128 m of the nearest cluster node, else
   decline `ReachExceeded`.

In a cluster (D16) the walk is the outer face of the darts' rotation system: per
step, the trim points of a stub; then the left edge of the dart arriving at the
node, back to the pair's corner; the corner; the right edge of the next stub out to
its trim. An absorbed stretch contributes its outer edge between the corners at its
two ends (the whole edge when a pair is open).

### 3.6 Degenerate and special cases

| Case | Rule | Fixture count |
|---|---|---|
| **T junction** (straight pair open) | open pair: the ring follows the main road's edge back to the node and out again; no bevel across the main road | most of the 758 degree-3 nodes |
| **Bow-tie** (non-adjacent edges cross: acute Y, approaches curving back) | 3 trim-extension rounds, then `HullFallback` | slice 4: 1 / 1 repaired; hull 21 of 504 / 22 of 283 ring records (D21) |
| **Shallow merge / diverge** (pair < 15°) | corner found at the gore nose (trim = w/sin θ, up to 80 m). The reach cap is 128 m (not the link cap), so the long thin ring is legal. Its heights come from the approaches' own surfaces (4.2), not from extrapolation. Beyond 128 m: `ReachExceeded`, and the approaches carve independently with lowest-wins | 12 / 22 nodes with such a pair; 6 trims > 64 m in B |
| **Dual-carriageway crossing** (4 nodes, a cycle of 4 short stretches) | one cluster; the ring comes from the 8 outer approaches only, so the box interior, including the median gap, is filled. That is right: the median gap inside a crossing is paved. A median wide enough that the two carriageways' trims do not overlap gives four single junctions | slice 4: largest clusters 4 and 7 nodes in A, 5 in B (not classified by kind) |
| **Short stretch to a non-junction node** | trim clamped to L − 0.5; the join or dead end beyond is unaffected | rare |
| **Tile borders** | none: geometry comes from the merged context graph and complete ways (section 5) | — |
| **Bridge at a junction** | exactly one bridge (an abutment): a ground junction; the bridge approach takes part in ring and heights with its anchored profile, and its core is not carved. All used approaches bridges (junction on a deck): `OnStructure`, rings and heights served for G3-E, never carved. Two or more bridges at a node: P1 declines them, so they are `NoProfile` and excluded | 1 / 2 |
| **Tunnel at a junction** | tunnel approaches `Tunnel`, excluded, flag `Partial`; all tunnels: `Underground` | 1 / 5 |
| **Roundabout** | ring stretches never link (3.2), so each entry node is its own junction (or a cluster with its own split-entry arms). The ring carriageway is an ordinary one-way core. P1's one-way crown falls to the right of travel, which on a counter-clockwise DE roundabout is outward, as the design guides want. The island is never filled. A mini roundabout node (`highway=mini_roundabout`, degree ≥ 3) is an ordinary junction | 3 ways / 4 nodes in A |
| **Grade exception** (owner: endpoint heights win, steep stretches flagged) | the approach profile is used as served; the junction surface interpolates between approaches, so it never exceeds their steepest grade plus crossfall (4.3); flagged stretches are not special | — |
| **Calming on a junction node** | section 4.4 (raised junction) | to measure |
| **Approach with a declined profile** | excluded (`NoProfile`), flag `Partial`. The ring closes across its mouth along the neighbours' edges. After the switch that area is a hole in the carved road; the census lists these (10.4, Q1) | to measure on home |

### 3.7 Junction decline reasons

These are listed in `JunctionDecline` (section 2). A declined junction:
- keeps its record (nodes, approaches, reason);
- has no rings;
- leaves its approaches untrimmed (trim 0), so they carve independently up to the node
  under the lowest-wins rule.

A `ClusterTooLarge` record is extra: its member nodes also have their own single-node
records. As implemented (D19) it holds the discovery set of its smallest node (sorted),
every approach of those nodes that is not a link between two of them (key order,
trim 0) and the linking entries as `absorbed`; one over-cap component can produce
more than one such record.

---

## 4. Heights

### 4.1 The road surface function

One definition of a carved road height, in the library (`road_surface_height_m`). For
an entry with profile p, cross-section xs and footprint `f = carve_footprint(xs, s)`:

```
z(s, t) = p(s) + C(s, t) − C(s, 0) + V(s, t) + F(s, t)
```

**C, the crown.** Crossfall c = `crown_per_mille`/1000 (P1, 25 ‰). d = t − f.crown_pivot.
- **Roof:** C = −c·ρ(d), where ρ(u) = |u| for |u| ≥ r0, else u²/(2·r0) + r0/2. This is
  C1, with r0 = 0.5 m (`crown_round_m`, [game] parity with the game's
  `sqrt(d²+0.25)−0.5`) and no square root.
- **OneSidedRight:** C = c·d (falls to −t).
- **OneSidedLeft:** C = −c·d.
- The fall continues unchanged across shoulders and flush sidewalks (P1 Q1: physics
  flush).
- Subtracting C(s, 0) makes the **profile exact on the reference line**, where it was
  fitted to the DEM. On a one-way road with the pivot on its left edge, the left edge
  sits c·w/2 above the profile and the right edge c·w/2 below, so cut and fill stay
  balanced (D5).

**V, the verge.** For |t| between the paved edge and the outer edge on that side:
`V = −verge_drop · S((|t − paved| )/(outer − paved))`.
- S is P1's C1 two-parabola smooth step, `road_smooth_step`.
- `verge_drop` = 0.06 m [game].
- Inside the paved band V = 0.

**F, the features.** `feature_height_m` (P1 slice 7), unchanged.

**Other rules:**
- t is clamped to [outer_right, outer_left].
- The function uses only + − * / and existing det helpers.

### 4.2 Junction surface

For a point X inside the junction's outer ring, with used approaches i = 1..k:

- **Approach surface `ẑ_i(X)`.**
  - Project X onto approach i's reference polyline: the 0.5 m outer-edge stations of
    3.3, reference points only, σ ∈ [0, trim_i + 1]. Take the nearest segment and the
    clamped parameter.
  - This gives (σ*, t*), with t* signed by outward-left.
  - If the projection parameter falls before σ = 0, extend with the tangent at σ = 0:
    p is extended linearly with its end grade, C and V are evaluated at σ = 0.
  - `ẑ_i(X) = z_i(s(σ*), t*)` by 4.1.
  - **This is the approach's own surface, not an extrapolation.** In a 90 m gore
    junction each carriageway keeps its real profile.
- **Weights.** `D_j(X)` = squared Euclidean distance from X to approach j's **trim
  segment**, the outer-ring segment between j's two trim points. Then
  `w_i = Π_{j≠i} D_j / Σ_m Π_{j≠m} D_j`, which is the product form of inverse-squared
  distance and has no division by zero on a trim line.
- **`z_J(X) = Σ_i w_i·ẑ_i(X)`.**
- **RaisedTable:** add `raised_height_m` inside the ring (4.4).

**As implemented (D24, D42).** The raise enters through the approach surfaces (D24). The blend
mixes each approach's surface with its cross-section relief faded out away from its trim line
(D42): `z_J = Σ_i w_i·v_i` with `v_i = b_i + φ_i·(p_i − b_i) + ψ_i·(ẑ_i − p_i)`, where `b_i` is
ẑ_i read at t = 0 (profile, tangent extension, the raise over the carriageway), `p_i` is ẑ_i
with t clamped to the paved band, `φ_i = smooth_step(1 − d_i/l_i)` with
`l_i = clamp(2·relief_i/slope, min, max)` (relief_i = max |ẑ_i − b_i| over the paved band at
the trim station) and `ψ_i` the same over the fixed verge length (`junction_fade` in
`carve_default.json`: 10 ‰, 3 m, 15 m, 1.5 m). d_i is the distance to trim segment i, so
φ_i = ψ_i = 1 with zero slope on the trim line and v_i = ẑ_i there.

**Arithmetic.** Double precision, + − * / only, computed in the fixed approach order.
Under `/fp:precise` and `-ffp-contract=off` this is bit-identical on Windows and Linux
(the project's determinism rule). A product of 19 squared distances under 1e4 m²
stays far inside the double range.

### 4.3 Properties (each one is a test in section 11)

1. **Exact match.** On approach i's trim segment, D_i = 0, so every w_m with m ≠ i
   holds the factor D_i = 0 and w_i = 1. Hence z_J = ẑ_i, the same function the core
   evaluates on its side.
   - The only mismatch is quantisation (1/256 m) plus the evaluation of ẑ_i at a point
     a cell away from the trim line: O(d²·Δz) with d ≤ 0.7 m.
   - **Junction continuity: 0 by construction, ≤ 1/256 m measured on the grid.**
2. **C1 across trim lines.** Near trim line i, each w_m with m ≠ i ∝ D_i = O(d²), so
   ∂w/∂n = 0 on the line. The normal slope of z_J equals that of ẑ_i.
3. **No overshoot.** z_J is a convex combination: min_i ẑ_i ≤ z_J ≤ max_i ẑ_i.
   As implemented (D42): ψ_i ≤ φ_i, so each v_i is a convex combination of the approach's
   own heights b_i, p_i, ẑ_i at the projected station, and z_J lies within the range of
   those readings over all approaches (T5 checks exactly that).
4. **Single node, flat crowns.** At the node all ẑ_i equal the node anchor height
   (P1 profiles share the node plane). Away from it the surface warps smoothly between
   the crowned approaches, with amplitude ≤ c·w_max/2 (≈ 9 cm for 7 m carriageways).
5. **Approaches agree at the node.** The G3 criterion "continuity < 1 cm between all
   incoming edges" already holds in P1, since every approach takes the same anchor.
   With the completion tier, a bridge anchor replaces the plane for **all** stretches
   at that node (6.6), so the property survives.
6. **Gradient (D41, D42).** The design's expectation that z_J stays within its
   approaches' slopes does not hold for any blend: where two trim segments are close
   the trim data themselves force a gradient F = max |ẑ_i(P) − ẑ_j(Q)| / |PQ| (a crown
   of 9 cm a few metres from another approach's crown line, a verge edge centimetres
   from another approach's paved edge at a ring corner), and a C1 transition is steeper
   than Δ/L somewhere (the product weights peak at 2Δ/L between parallel trims). What is
   asserted (`test_junction_gradient.cpp`, D42): on the drivable rings (road bed,
   paved) the gradient is ≤ max(B, 2·F_paved) + 0.02 at points ≥ 2 m from every trim
   segment and ≤ max(B, own, 2·F_all) + 0.02 nearer, B = steepest approach grade +
   crossfall; so wherever 2·F_paved ≤ B the bound B + 0.02 itself holds in the
   interior. Junctions whose data force more are listed with their numbers.

**Rejected:** the PLAN's least-squares plane (PLAN.md 6.3 step 5). A plane can match
neither crowned approach edges nor a gore junction within 1 cm: a 2.5 % crown is
8.75 cm over 3.5 m. Also rejected: Coons patches (they need a quad topology that
junctions do not have). See D6.

### 4.4 Calming features at junctions

- **On the junction node** (owner P1-Q2: every vertical kind is carved):
  - A `table` or `hump` raises the whole junction by its height: flag `RaisedTable`,
    `raised_height_m`.
  - Each used approach gets a ramp of the feature's ramp length, for σ ∈
    [trim_i, trim_i + r]. It is evaluated through F, as an extra feature record with
    `node_id` = the junction node and station `trim_i + r/2`, made by the junction
    builder.
  - Any other vertical kind (bump, cushion, …) on a junction node raises nothing. It
    is recorded as a furniture-style decline count `JunctionCalming` in the
    measurement report. It is too small to carve across a junction.
- **Features of an approach inside the ring** (P1 section 4, station < trim + ramp):
  not carved, counted. A feature cut by a trim line would leave a step.

**As implemented (slice 5):** `adjust_features_for_junctions` (suppression and ramp records, D22/D23), `make_junction_surface` /
`junction_height_m` (4.2, with the raise inside the approach surfaces, D24), `JunctionBuildStats::raised_tables` and
`calming_ignored`. The `JunctionCalming` count of the measurement report is `calming_ignored`. Neither fixture has a
table, hump or any other vertical calming tag on a junction node, so the raised-table path runs on synthetic scenes only.

---

## 5. Tile invariance

### 5.1 Ownership

- A junction (including a declined cluster record) is stored **only** in the
  `roads.geom` tile whose L2 square contains its **owner node**: the smallest OSM id
  in the cluster, using half-open floor tile membership on integer mm, as for edge
  midpoints.
- Furniture on an entry (crossings, signals and lines at interior nodes) is stored
  with that entry, in every tile that holds the entry, as P1 features are. Furniture
  derived from a junction (lines and signals of junction-node tags) is stored with
  the junction.

### 5.2 What a roads.geom tile can compute

Fix tile T, an owner node n inside T, and its cluster:

- Cluster discovery from n visits nodes within 48 m + 60 m = **108 m** of n
  (3.2: cap plus one link of at most 2 × 30 m).
- Every such node lies within T + 128 m. `roads.graph` keeps every complete way that
  touches T + 128 m, so every approach way is complete in T's graph and has an entry
  in T's `roads.geom` (all stretches of all tile ways).
- Node degrees come from the merged context graph (P1 slice 1).
- Trims depend only on the approaches' references, cross-sections and rules, which
  are tile-invariant (P1).
- Heights depend only on the entries' profiles, which are tile-invariant, plus the
  completion tier, which is tile-invariant by 6.6.

**So the junction record is byte-identical whichever tile computes it**, and exactly
one tile stores it. Test: build every fixture tile, collect junctions by owner node,
and compare against a build in which T is replaced by a neighbour that sees the same
cluster through its halo (white-box build without the ownership filter). Negative
control: degree from the tile graph instead of the context graph.

As implemented (slice 4): a tile that cannot see a node's whole discovery set does not
guess. A possible link to an incomplete node marks the discovery unresolved and the
tile skips that node (D20); the owner tile always resolves it (census: every junction
node is covered in its own tile). The slice 4 test compares every record built in two
or more fixture tiles (210 / 160 records, 0 mismatches). Its negative control treats a
possible link to an incomplete node as absent: 4 records then differ between tiles.

### 5.3 What the carver needs

Fix an L0 tile Q (256 m) inside L2 tile T.

- **Junctions.** A junction's outer ring lies within 128 m (reach) of a cluster node.
  Its blend adds ≤ 15 m, and its owner node is ≤ 48 m from every other cluster node.
  Every junction that can touch Q therefore has its owner within **191 m** of Q.
- **Cores.** An approach core or blend touching Q belongs to a way within outer
  half-width + 15 m < 128 m of Q. That way is in T's graph.
- **DEM.** The blend widths read the DEM at road-side points within 15 m of Q, and the
  smoothing kernel needs 2 cells. The 3 x 3 `g2m.elev.base` neighbourhood of Q covers
  both.

So the carver's inputs are:

```
g2m.elev.base L0: Q and its 8 neighbours (absent = NoData)
g2m.roads.geom L2: every L2 tile intersecting Q's square grown by 192 m (1 tile typical, at most 4)
```

**Cell rule.**
- Each L0 tile computes only its own cells, from a function of world position that
  every tile evaluates identically. Neighbouring tiles therefore agree at the shared
  edge region without sharing samples (elev.base has no halo).
- Junctions are taken from the input tiles. Approach cores and their trims are matched
  by (way id, stretch, end); trims are read from whichever loaded junction lists that
  approach.
- A core whose junction lies beyond the 192 m radius is trimmed only within 128 m of
  that junction, so the trim can never matter inside Q.

---

## 6. Carving

### 6.1 Layers

| Layer | Levels | Shape | Encoding | Inputs |
|---|---|---|---|---|
| `g2m.terrain.carved` v1 (new) | L0-L6 | as `terrain.height`: 256 x 256 cell-registered, int32 1/256 m | SrcElev 1 (no new codec) | L0: 5.3; L1-L6: its 4 children (mean-of-4 half-even, as `terrain.height`) |
| `g2m.terrain.class` v1 (slot exists, first deriver) | L0-L6 | **node-registered 257 x 257** at integer metres (L0), spacing 2^L m | new TerrainClass, section 9.4 | L0: as `terrain.carved` L0 + `g2m.src.landcover` L2 ancestor; L1-L6: its 4 children (section 6.5) |

- `g2m.terrain.height` is unchanged and stays the default. A consumer opts in by
  requesting the carved layers.
- `g2m_tiler bake --layers` naming either new layer adds both, plus `roads.graph` and
  `roads.geom` (they must agree: one release, one geometry).
- Keeping a separate layer name (D9) means no layer's bytes depend on what else is in
  the release.
- Coarse levels (L1-L6) become carved (via the pyramid), which the game's L0-only
  carve never was.

### 6.2 Per-tile pipeline

1. **Smoothing (ported) [game].**
   - On the 3 x 3 DEM window, separable 5-tap integer kernel [138, 621, 1024, 621, 138]
     (exp(−i²/2) × 1024, rounded; sum 2542). Each axis pass is an int64 sum divided by
     the weight sum with half-even rounding. Then `h + 0.35·(smoothed − h)` in integer
     form: `h + (35·(s − h) + 50)/100`, rounded half-even.
   - NoData cells are skipped and renormalised, as in the game.
   - Parameters come from `carve_default.json` (8.2). Differences against the game's
     double version are ≤ 1/256 m, plus kernel rounding, measured in slice 9.
2. **CarveModel build.**
   - Decode the ≤ 4 `roads.geom` tiles. Take the union of their junctions (unique by
     owner node) and of their entries (unique by way id and stretch). Entries in two
     tiles are identical (P1 invariance), so the copy from the lowest tile key wins.
   - Bucket every core corridor, junction outer ring and blend band into 16 m blocks
     (bounding boxes in integer mm).
3. **Per cell** (centre at E + 0.5 m, N + 0.5 m), in 6.3.
4. **Deck clearance**, in 6.4.
5. **Quantise.** `height = round_half_even(z·256)`, the same quantiser as the DGM1
   importer and the pyramid (D13).

### 6.3 Cell evaluation and ownership

Candidates for a cell X (from its block bucket):

- **Junction core.** X is inside the outer ring of a non-declined, non-`OnStructure`
  junction. z = z_J(X).
- **Approach core.** Ground entry (not bridge, not tunnel) with a profile and a
  cross-section.
  - Project X onto the entry's polyline (1 m stations, refined by the 0.5 m stations
    within 2 m of a vertex; sagitta ≤ 12.5 mm at R = 10 m) to get (s, t).
  - Core if `s ∈ [trim_start, L − trim_end]` and t is inside the outer edges.
  - z = road_surface_height(s, t).
- **Blend.** For each junction or approach whose core does not contain X, let d be the
  distance from X to that core's outer boundary:
  - for an approach: |t| − outer at the projected s, with s inside the trimmed range;
  - for a junction: the exact point-to-segment distance to the outer ring, using
    det::sqrt.
  - X is a blend candidate if d < b, where b is the blend width at the nearest
    boundary point (below).
  - Weight `w = 1 − S(d/b)`.
  - Value `z = z_b + (T − z_b)·S(d/b)`, where z_b is the core height at the nearest
    boundary point and T is the smoothed terrain.

**Blend width.**
- `b = clamp(max(b_class, 3·|T_edge − z_edge|), 3, 15)`:
  - `b_class` comes from the rules per road class;
  - `T_edge` is the smoothed DEM at the outer-edge point;
  - the factor 3 caps the smooth step's peak slope (2|Δ|/b) at the 1:1.5 cut/fill
    limit (PLAN 6.3 step 4).
- Computed per approach at each station and side, then interpolated linearly in s.
- On a junction ring it is interpolated along the boundary between the trim points'
  values, so the seam on the trim line's extension is continuous (4.3.1). Corner
  vertices take the mean of their two trims' values.
- Where `3·|Δ| > 15 m`, the slope exceeds 1:1.5. This is counted, not hidden.

**Ownership (all rules order-independent, D10).**
1. If any **junction core** covers X:
   - take the lowest z among them;
   - junction cores beat approach cores. The ring contains the approach's area for
     σ < trim, and the core range excludes it, so the two meet only on trim lines.
2. Else if any **approach core** covers X, take the **lowest z** (PLAN 6.3 step 6:
   "lowest road wins").
   - Two cores overlap only where two ground roads share pavement without a junction:
     a parallel service road touching a main road, or a declined junction. Taking the
     minimum keeps the result independent of order.
3. Else if any **blend** covers X, take the blend with the **largest weight w**; ties
   go to the lowest z.
4. Else T.

**Not ported, deliberately:**
- the game's 0.35 proximity blend between neighbouring roads. Junction surfaces and
  lowest-wins replace it.
- the game's nearest-segment ownership. Projection plus explicit trims replace it.

### 6.4 Bridges, tunnels, decks

- **Bridge entries** are never carved. Their footprint (paved + 1 m) applies the
  **deck clearance clamp** as the last step:
  - `z = min(z, deck_top − clear·S(e/ramp))` within the footprint, where e is the
    distance inside the footprint edge.
  - `clear` = 0.55 m and `ramp` = 4 m [game], from `carve_default.json`.
  - `deck_top` = road_surface_height of the bridge entry (crown included).
  - The clamp only ever lowers terrain, so the lower road under a bridge keeps its
    carved height unless the deck would cut into it.
  - That case (clearance < 0.55 m) is counted; it is a profile problem, not a carving
    one.
- **Tunnel entries** are never carved and get no clamp. The ground road above a tunnel
  carves normally: it was fitted to the DEM.
- The game's inferred tunnel-roof decks are not ported. They existed only to stop the
  game carving tunnel floors, which the server never does.
- **`OnStructure` junctions** are not carved. They are served for G3-E.

**As implemented (slice 9):** `include/g2m/layer/terrain_carve.h` / `src/layer/terrain_carve.cpp`: `CarveTerrain` (smoothing,
6.2 step 1), `CarveModel` (build and `evaluate`, 6.2 step 2 - 6.4), `carve_terrain_tile` (rows split over threads, identical for
every worker count), `carve_road_geom_keys`, `TerrainCarvedDeriver` (config text `g2m.terrain.carved/1 smooth+carve L0 roads.geom
enc=7 radius=192 carve=<sha256 of carve_default.json>; L1-L6 mean-of-4 half-even`). Deviations and readings: D36-D41. Wiring of
`BuiltinTerrainDerivers` and `bake` is pulled forward from slice 11 (D39). Tests and measured values: section 11, "Slice 9 as
implemented", and the record after D41.

### 6.5 terrain.class derivation

**Sample lattice.**
- L0 samples sit at integer metres: 257 x 257, both tile edges included. These are
  exactly the physics cell centres, since the physics heightfield nodes sit on the
  DGM1 cell centres and its quads are centred on integer metres (D11).
- The duplicated edge samples are equal in neighbouring tiles by construction (same
  world function).

**Planes:**
- `land_class` (LandClass);
- `surface` (SurfaceKind);
- `role` (0 terrain, 1 road bed, 2 sidewalk, 3 verge, 4 junction road bed,
  5 junction sidewalk, 6 blend, 7 bridge deck footprint);
- `rank` (0 = no road, else the owning way's `RoadGraphWay::rank`, saturated at 255).

**Per sample (shared CarveModel ownership):**
- **Junction core:**
  - inside the road bed ring: role 4, surface of the highest-rank used approach (ties:
    smallest way id), land class PavedRoad or UnpavedRoad from its attributes;
  - inside the paved ring but outside the road bed ring: role 5, `PavingStones`;
  - inside the outer ring only: role 3, base class.
- **Approach core:** the band at t:
  - road bed: role 1, the way's surface (as `envelope_road_segments`);
  - sidewalk: role 2, `PavingStones`;
  - verge: role 3, base class.
- **Blend:** role 6, base class. **Terrain:** role 0, base class.
- **Base class.** Nearest `g2m.src.landcover` 4 m sample (the source's own edge rule),
  mapped by a classifier table (`data/classifiers/esa_worldcover_v200.json`,
  table_hash in the release id) to LandClass and a default SurfaceKind.
- **Bridge footprint** (role 7) keeps the base class underneath.
  - Physics never drives on terrain under a deck: the deck is G3-E's collider.
  - Render colours the ground under it as terrain.

**Pyramid L1-L6.**
- A node of level L takes, from the level L−1 nodes within ±spacing_{L−1} (3 x 3,
  coincident centre), the sample with the highest rank. Ties go to the coincident
  centre, then the row-major first.
- This widens roads by about half a spacing per level (≈ the game's own L3 widening
  to 3/4 spacing). Render level policy (min rank per level) stays client-side, reading
  the `rank` plane.
- Rejected:
  - plain subsampling: roads vanish from L2;
  - mode-of-4: ties, no rank priority.

### 6.6 Profile completion tier (moves out of the game)

A port of racing_game's `complete_road_profiles` and its retry into
`build_road_geom_tile`, after the strict pass. It uses det math, no libm, and turns
headings into tangent vectors instead of cos/sin.

**Order:**
1. **Retry.** Every strict decline whose code is "bounded solver" or "endpoint
   curvature" is re-run at 20 m spacing and 16384 iterations with the same limits.
   Provenance `Retry`.
2. **Bridges.** Each declined bridge span is re-fitted between abutments (the game's
   `abutment`, existing bridge anchors authoritative): max grade
   max(12 %, class), min radius min(250, class), with a common secant grade on
   failure. Provenance `BridgeCompleted`. Its end anchors enter the anchor map.
3. **Ground.** Every ground stretch with an anchored end, **including strict
   profiles** at such a node, and every still-declined ground stretch, is re-fitted:
   - 40 m samples;
   - max grade max(12 %, class), min radius min(500, class);
   - end grades clamped ±10 %;
   - 32 m ramps from anchors.
   Provenance `GroundCompleted`. A stretch that already had a strict profile keeps
   `strict_decline_code` 0.

**Rules:**
- Tunnels are not completed (not carved, not drivable).
- **Tile invariance:** the anchor map holds only bridges incident to the stretch's own
  end nodes. Those bridges are complete ways in every tile that holds the stretch, so
  the tier is a function of the context graph and the DEM like the strict pass.
- **Order independence:** steps run in ascending (way id, start_ref) inside each step.
  Step 3 reads only step 2's anchors, never another step-3 result.
- Section 2 of the encoding stores the completed profiles as ordinary profiles.
  Section 7 holds the provenance.
- The grade-policy owner decision (endpoint heights win, flag the exception) is kept.
  A completed profile that exceeds the class grade is a flagged grade exception, as
  in the strict pass.

**As implemented (slice 6):** `road_completion.h` / `road_completion.cpp` (`complete_road_profiles`, `road_bridge_anchor`),
opt-in through `RoadCompletionOptions::enabled` (default off, so no served bytes change: the deriver does not pass it, and
`encode_road_geom` refuses a tile with a non-Strict `RoadGeomTile::origins` entry). Decisions D25 (anchors recomputed from the node,
60 m / 63 m reads) and D26 (strict ground profiles are not refitted at bridge nodes). Step order retry, bridge, ground, each
and the result is independent of the entry order (a reversed entry list gives the same profiles, tested). Tunnels are counted
(`tunnel_skipped`) and left declined. The validator relaxes the vertical bounds per provenance and checks the origins.
D43: a step-3 fit that fails only on its fixed end values (endpoint curvature or boundary constraints) is retried with both
end grades moved toward the stretch chord in eighths (counted in `ground_grades_eased`), only while each end grade lies
within 0.05 of the chord (a local misread, not terrain that departs from the stretch).

---

## 7. Junction furniture

All furniture is computed in `build_road_geom_tile` after junctions, from node tags
of `roads.graph` encoding 5. Every source node produces exactly one record per item it
implies, placed or declined. Rules in 8.1.

### 7.1 Crossings

- **Source:** each `highway=crossing` node on a stretch of an entry.
  - On an interior node, use that entry.
  - On a degree-2 join, use the entry whose `start_ref` is the node (station 0; the
    quad extends back over the join along the P1 bisector tangent, flag `spans join`).
  - On a dead end, use the end station.
- **Declines:** `StretchDeclined`, `Tunnel`, `JunctionNode` (degree ≥ 3: no direction
  without footway geometry).
- **Extent:**
  - Along the road: `length_m` = the node `width` tag if it parses to 1-10 m, else the
    pattern default (zebra 4 m [R-FGÜ, unverified], signalised 4 m [assumed], other 3 m
    [assumed]).
  - Across: [carriageway_right, carriageway_left] at the station (P1
    `carriageway_edges`), so the quad lies inside the carriageway by construction.
  - Quad = `lateral_point` at (s ± length/2, both edges).
  - Bicycle and parking bands that sit on the carriageway are included (they are on
    it).
- **Pattern** (first matching rule):
  1. `crossing:markings` ∈ {zebra, ladder, ladder:skewed→Ladder, lines, dashes,
     dots→Dashes, surface}: that pattern. `no`: None. `yes`: step 2.
  2. `crossing` = traffic_signals: Dashes (DE "Furt" broken lines [RMS, unverified]).
  3. `crossing` ∈ {zebra, marked, uncontrolled}: Zebra. `crossing_ref=zebra` is not in
     the whitelist, so it is not read.
  4. `crossing` ∈ {unmarked, no, informal}: None.
  5. Otherwise: None.
- **Flags:** signalised (`crossing=traffic_signals` or `highway=traffic_signals` on the
  same node), lowered kerb (`kerb=lowered|flush`), inside a junction ring (station <
  trim of an approach).
- A pattern-None crossing still gets a placed record. Traffic AI and P3 need to know
  it is there. Render draws nothing.

Fixture expectation [measured]: A has 125 crossing nodes: 2 at dead ends, 121
interior or join, 0 at junctions. B has 52: 50 interior, 2 `JunctionNode` declines.

### 7.2 Stop and give-way lines

**Signalised approach.** An (entry, travel direction) is signalised toward a junction
when either:
- **(a)** a `highway=traffic_signals` node lies on the entry and applies to that
  direction, within `signal_assoc_max_m` (60 m [assumed]) of the trim line; or
- **(b)** the junction node itself (any cluster node) carries `highway=traffic_signals`,
  and the approach is not a one-way leaving the junction.

**Direction of a signal node.**
1. `traffic_signals:direction`, then `direction` (forward / backward relative to the
   way; `both`: both).
2. If absent and the way is one-way: the way's travel direction.
3. If absent and the stretch has a junction end within `signal_assoc_max_m`: toward
   the nearer such junction.
4. Else both directions.

A signal node far from any junction (a pedestrian light on a crossing) still gets
lines, for each resolved direction.

**Stop line station** (toward the junction or the controlled point):
- **Case (a):** the signal node's station, moved back to
  `crossing_near_edge − stop_line_before_crossing_m` (1 m, project default) if a
  crossing quad on the same entry starts within `stop_line_before_crossing_m +
  line_width` downstream. A signal node that is also the crossing node is the common
  `crossing=traffic_signals` case: the line goes 1 m before the quad.
- **Case (b):** the approach's trim line moved back by `junction_stop_setback_m`
  (1 m [assumed]), then the crossing rule.
- If a node of case (a) exists, case (b) adds nothing for that approach. One stop line
  per approach and direction.

**Lateral extent: the acceptance rule.**
- `driving_lanes_at(xs, s, travel)` gives the counted, non-Parking lanes of exactly that
  direction, at least 0.5 m wide.
- The line spans [min, max] of those lanes' `centre_t ± width/2`. If the lanes are
  not contiguous (a contraflow lane between them), one record per contiguous group.
- If zero lanes: decline `NoLanes`.

**Stop and give-way signs.**
- `highway=stop` → StopLine, `highway=give_way` → GiveWayLine.
- Direction as for signals; `direction=both` gives both.
- Without a direction:
  - toward the nearer junction end within `sign_assoc_max_m` (40 m [assumed]);
  - on a junction node: every approach whose rank is lower than the junction's
    highest-ranked approach. If all approaches have equal rank: `Ambiguous`.
- Station:
  - on an interior node: the node station, then the crossing rule;
  - on a junction: the trim line − 0.5 m [assumed] (DE "Haltlinie/Wartelinie at the
    junction edge" [RMS, unverified]).
- Line width (`length_m`):
  - stop 0.50 m [RMS, unverified];
  - give-way 0.50 m broken (pattern 0.50/0.25 for P3) [RMS, unverified];
  - signal stop line 0.50 m.

### 7.3 Signals

- **One per stop line from signals.** Post position: station = stop line station,
  t = the travel-right carriageway edge + `signal_lateral_offset_m` (0.5 m [assumed]),
  outside the carriageway.
- **Facing:** opposite to the travel direction, as a unit vector in mm × 1000 ints.
- **Second post** on the travel-left side when the direction has ≥
  `signal_left_min_lanes` (3 [assumed]) lanes (flag bit4).
- Junction-node signals at a declined junction: `JunctionDeclined`.
- Height is not stored. The consumer stands the post on the carved surface.

### 7.4 The 0.5 m containment rule

- **Crossing quads** and **lines** lie inside the carriageway by construction. Their
  lateral extents are carriageway edges and lane edges.
- **Signals** stand 0.5 m outside on purpose. They are posts, not polygons, so the
  acceptance rule ("no furniture polygon > 0.5 m outside the carriageway") does not
  apply to them. The test checks their offset equals the rule value instead.
- Inside junction rings, "carriageway" means the union of the approach carriageway
  bands and the junction road bed ring.


**As implemented (slice 7):** `road_furniture.h` / `road_furniture.cpp`; see decisions D27 (ownership) and D28 (rule readings: association
distance to the junction node, the left post only on one-ways, one signal per line set, bit 3 never set). The crossing count of fixture A is
123 in the road graph, not 125. Records are not yet stored (encoding 7).

---

## 8. Rules as data

### 8.1 `data/road_rules/de.json`: new blocks `junction` and `furniture`

Country rules stay in one file with one SHA in the `roads.geom` config hash. The
strict loader rejects unknown keys, as in P1.

```json
"junction": {
  "trim_min_m": 1.0, "trim_margin_m": 0.5, "edge_station_spacing_m": 0.5,
  "link_trim_cap_m": 30, "cluster_max_extent_m": 48, "cluster_max_nodes": 12,
  "reach_max_m": 128, "bowtie_rounds": 3, "bowtie_step_m": 1.0
},
"furniture": {
  "crossing_length_m": { "zebra": 4.0, "signalised": 4.0, "other": 3.0 },
  "stop_line_before_crossing_m": 1.0,
  "junction_stop_setback_m": 1.0, "sign_line_setback_m": 0.5,
  "line_width_m": { "stop": 0.5, "give_way": 0.5, "signal": 0.5 },
  "signal_assoc_max_m": 60, "sign_assoc_max_m": 40,
  "signal_lateral_offset_m": 0.5, "signal_left_min_lanes": 3,
  "crossing_markings": { "zebra": "zebra", "ladder": "ladder", "ladder:skewed": "ladder", "lines": "lines",
                          "dashes": "dashes", "dots": "dashes", "surface": "surface", "no": "none" },
  "crossing_kind": { "traffic_signals": "dashes", "zebra": "zebra", "marked": "zebra", "uncontrolled": "zebra",
                     "unmarked": "none", "no": "none", "informal": "none" }
}
```

The `sources` block gains the provenance of each value ([RMS]/[R-FGÜ] unverified,
[assumed]).

**Validation:**
- every length is in (0, 200];
- cluster caps satisfy 48 + 2·30 ≤ 128 (the proof of 5.2): the loader rejects a file
  that breaks it;
- `reach_max_m` + 15 + `cluster_max_extent_m` ≤ 192 (the radius of 5.3).

### 8.2 `data/carve/carve_default.json` (new): carving parameters

This file is hashed only into `terrain.carved` and `terrain.class`. Re-tuning carving
must not re-derive `roads.geom`, which costs minutes of profile fitting per tile (D12).

```json
{ "format": "g2m.carve/1",
  "smoothing": { "enabled": true, "radius_m": 2, "kernel": [138, 621, 1024, 621, 138], "strength_percent": 35 },
  "crown_round_m": 0.5, "verge_drop_m": 0.06,
  "blend_m": { "motorway": 15, "trunk": 12, "primary": 10, "secondary": 8, "tertiary": 6, "default": 4, "service": 3 },
  "blend_slope_factor": 3, "blend_min_m": 3, "blend_max_m": 15,
  "deck": { "clearance_m": 0.55, "ramp_m": 4, "margin_m": 1 } }
```

As implemented the keys are integers in mm (`crown_round_mm`, `blend_mm`, ...), and D42
adds `"junction_fade": { "slope_per_mille": 10, "min_mm": 3000, "max_mm": 15000,
"verge_mm": 1500 }` (the relief fade of 4.2; verge_mm ≤ min_mm is enforced).

The blend widths per class are [assumed] (the PLAN gives only the 3-15 m range); the
game uses one `blend_mm` per class from `RoadAttributes`, which slice 1 reads and
records as the defaults. The classifier table `data/classifiers/esa_worldcover_v200.json`
maps the 11 ESA classes to LandClass/SurfaceKind.

---

## 9. Encodings

### 9.1 roads.geom encoding 7

`kRoadGeomEncoding = 7`, version 1. Sections 1-4 are byte-identical in format to
encoding 6. Their **content** changes where the completion tier adds profiles
(section 2) and therefore cross-sections and features (sections 3, 4). Required
sections: exactly 1, 2, 3, 4, 5, 6, 7, in order.

**Section 5 (Zstd), junctions:**

```
varint junction_count                         <= 65536, sorted by owner node id
per junction:
  svarint owner_node_id                       == nodes[0]; its L2 tile == this tile
  u16 flags                                   bits outside the 5 defined must be 0
  u16 decline                                 0..5
  varint node_count                           1..12 (ClusterTooLarge: the full component, <= 4096)
  per node: svarint node_id                   strictly ascending
  varint approach_count                       2..64
  per approach: varint entry_index, u8 at_end (0/1), u8 status (0..3), u64 trim_m (binary64 bits)
  varint absorbed_count; per absorbed: varint entry_index
  3 x ring (road bed, paved, outer):          all three empty iff declined
    varint vertex_count                       0 or >= 3
    svarint x_mm, svarint y_mm                first vertex absolute
    per further vertex: svarint dx_mm, svarint dy_mm
  u64 raised_height_m                         binary64; 0 unless RaisedTable
```

**Section 6 (Zstd), furniture:**

```
varint count                                  sorted (entry_index or junction, kind, station bits, node_id, travel)
per item:
  u8 kind (0..3), u16 decline (0..7)
  svarint node_id, varint entry_index, varint junction_index (0 = none)
  u64 station_m (binary64), u8 travel (0 Forward, 1 Backward)
  svarint t_from_mm, svarint t_to_mm, varint length_mm
  u8 pattern (0..5), u8 flags (bits 0-4)
```

**Section 7 (Zstd), provenance:** `varint count == entries.size()`; per entry
`u8 provenance (0..3)`, `u16 strict_decline_code`.

### 9.2 Validation (hard errors, one rejection test each)

- **Ownership.** Every junction's owner node is in the tile, and its id is the
  minimum of `nodes`.
- **Indices.** Every `entry_index` < entries; an approach has status Used only if its
  entry has a profile and a cross-section; `at_end` matches the node at that end.
- **Rings.** Counter-clockwise (exact signed area > 0), simple (exact predicates), no
  duplicate consecutive vertex; the road bed ring lies inside the paved ring, and the
  paved ring inside the outer ring (vertex-in-polygon, exact).
  - **Slice 8 must settle two points first (D21).** (1) Each layer's vertices are
    rounded to mm independently, so a vertex on an edge two layers share can lie up
    to ~1.5 mm outside the next ring; an exact check rejects that. Either allow
    2 mm or build the inner rings by containment. (2) After slice 4, 5 vertices in 2
    junctions of fixture A (worst 352 mm, both on short, strongly curved approaches)
    still stick out. Before validation runs, the builder should send such a junction
    to the all-layer hull, which nests.
- **Trims.** In [0, L − 0.5], finite.
- **Furniture.**
  - Enums in range.
  - A placed crossing/line has `t_from < t_to` and a station in [−length, L + length].
  - A placed line's [t_from, t_to] equals its lanes' span recomputed from the entry.
    This is the acceptance rule, enforced at decode.
- **Provenance.** The count equals the number of entries; a Strict entry with a
  decline has provenance 0 and no profile.

### 9.3 Back-compatibility and versioning

- Encoding 6 and 5 bodies still decode: no junctions, furniture or origins.
- **Config hash text:** append `enc=7 junction=1 furniture=1 completion=1`. The `rules=`
  SHA changes with de.json.
- `terrain.carved`: `g2m.terrain.carved/1 smooth+carve L0 roads.geom enc=7 radius=192
  carve=<sha256 carve_default.json>; L1-L6 mean-of-4 half-even`.
- `terrain.class`: `g2m.terrain.class/1 node-257 carve=<sha> classifier=<table_hash>
  pyramid=rank-max-3x3`.
- The release id changes only for releases that name these layers.

### 9.4 TerrainClass encoding (layer `g2m.terrain.class`, encoding 7 in that layer's id space)

| Type | Codec | Content |
|---|---|---|
| 1 meta | none | georef block (src_elev.md, registration 0 = node), `scheme_len` u8 + `class_scheme` ("g2m.landclass.v1") |
| 2 land_class | zstd | 257 x 257 u8 |
| 3 surface | zstd | 257 x 257 u8 |
| 4 role | zstd | 257 x 257 u8, 0..7 |
| 5 rank | zstd | 257 x 257 u8 |

**Validation:** land_class ≤ 13, surface ≤ 11 and role ≤ 7 are append-only limits
checked against the enums. Planes 2-5 must all be present. A new fuzz target
(`Target::TerrainClass`) and a format doc `docs/formats/terrain_class.md` are added.

---

## 10. Consumer switch: what racing_game needs and deletes

### 10.1 What the game needs from the server

| Need | Served by |
|---|---|
| Carved heights for physics and near render | `g2m.terrain.carved` L0 |
| Carved far terrain | `g2m.terrain.carved` L1-L6 (new: coarse LODs now carved too) |
| Grip per physics cell | `g2m.terrain.class` L0, `land_class` + `surface` at the physics cell centre: `fill_physics_surfaces` gains a class-raster mode that reads the sample directly, so no capsule painting and no ownership rule run in the client |
| Render colour per vertex | `g2m.terrain.class` at the render level; render vertices sit on cell centres (half metres), so the vertex takes the class of the node at `floor` of its position (its south-west corner node). This is documented in the consumer API and differs from physics by at most half a cell at class borders |
| Road ribbons, lane lines, sidewalks, kerb visual | `roads.geom` cross-sections (P1) and junction rings (section 5) |
| Bridge decks | `roads.geom` bridge profiles, including completed ones (6.6) |
| Furniture | `roads.geom` section 6 + `furniture_quad` |
| NPC lanes | `driving_lanes_at` (P1), unchanged |

### 10.2 What the game deletes (no fallback, owner decision 2026-10-05)

1. **`smooth_terrain`** call in `fetch_and_decode`. The server smooths before
   carving. Smoothing again would bend crossfalls the server defined.
2. **`RoadSurfacePatch`** construction and `apply`. With them go
   `road_surface_patch`'s 20 m retry block and the `RG_ROAD_SURFACE` fallback paths.
3. **`complete_road_profiles`** (bridge, ground and tunnel completion) and the
   **retry**. The server tier (6.6) serves those profiles; tunnel-floor completion is
   dropped.
4. **The deck clearance cut** (moved to 6.4).
5. **The tunnel-roof deck inference** in `build_road_decks`, used only to suppress
   carving. Bridge decks stay, built from served profiles.
6. **`extract_road_segments`-based grip** (`paint_road_blocks` in the physics fill)
   and **`rasterize_chunk_road_classes`** colours. Both are replaced by
   `terrain.class`.
7. **The junction fades** in `road_visual.cpp`. Junction rings replace them.

The game requests only the carved layers. A missing carved tile is a missing tile,
handled by the existing Absent/gate logic. It never falls back to `terrain.height`.

### 10.3 Visual = physics

- **Heights.**
  - Physics fills its 255 m node-registered tiles by integer offset copy from carved
    L0 (`fill_physics_heights`, unchanged).
  - Render chunks build from the same carved tiles with the Jolt triangle diagonal
    (`terrain_chunk.h`, bit-identical at chunk borders).
  - So both draw one triangle surface.
- **Draped geometry.** Ribbons, junction meshes, markings and furniture take their
  **vertex heights from the heightfield triangles** (same diagonal) + `surface_lift`
  (0.018 m [game]). The analytic surface (4.1, 4.2) only places them laterally.
  Draping on the analytic surface would differ from the triangles by up to c·cell/2 at
  a crown ridge (≈ 1.3 cm) plus 1/256 m quantisation: floating or z-fighting.
- **Kerbs.** The 12 cm visual kerb (P1 Q1) is drawn at `kerb_t` on top of the flush
  carved surface. Physics stays flush. `terrain.class` gives the sidewalk
  `PavingStones` grip, so a wheel on the sidewalk feels the sidewalk surface.
- **Features.** Carved bumps and tables are in the heights, hence in both physics and
  render.
- **Grip = colour.** Both read the same `terrain.class` sample set (10.1).

### 10.4 Switch gate

The switch is one racing_game commit. It removes completion and carving in the
game, so from then on only server-carved stretches are carved.

**Owner answer to Q1 (2026-10-05):** "There is no prod environment; work in progress
states are acceptable." So the gate is **not** zero gaps:
- The switch may happen as soon as the carved layer works end to end (slices 9-11).
- The census (section 12) lists the stretches that are still uncarved after the
  completion tier. The list is information for later work; it does not block the
  switch.
- Those stretches show as bare terrain between carved roads until fixed.

---

## 11. Tests

Each test is listed with its negative control or sabotage, which must make it fail.

| Test | Asserts | Negative control / sabotage |
|---|---|---|
| T1 road surface function | z on the reference = p(s); roof crown C1 at ±r0; crossfall c per metre outside r0; verge drop exactly −0.06 at the outer edge; one-way left edge +c·w/2 | drop the −C(s,0) term: reference ≠ profile |
| T2 corners and trims (synthetic X, T, Y, 30°, 10°, curved approaches) | trims equal hand-computed offset intersections within 1 mm; T: the straight pair is open, no bevel across the main road | swap left and right edges: trims wrong by the width |
| T3 rings | counter-clockwise, simple, nested road bed ⊂ paved ⊂ outer; acute Y triggers a bow-tie repair; a constructed self-crossing case falls back to the hull with the flag | disable repair: validation rejects the ring |
| T4 clusters | dual-carriageway crossing fixture → 1 cluster of 4, interior filled; a 5-node chain at 20 m spacing beyond 48 m → `ClusterTooLarge` + singles; roundabout ring never absorbed | the PLAN 25 m rule on the fixtures reproduces the 17/14-node chains (diag) |
| T4 as implemented (slice 4) | synthetic dual crossing (median 10.5 m between centre lines) → 1 cluster of 4, 4 absorbed, 8 stubs turning one full turn, median and nodes inside the road bed ring; staggered T pair 5 m apart → 1 cluster, the main road between the nodes inside the road bed ring; a 3-node chain at 18 m (30° diagonal crossings) → 1 cluster, 5 nodes over 72 m → 5 singles + 1 `ClusterTooLarge`; node cap 2 → too large; roundabout middle stretch → 2 singles; fixture census with tile identity; D17 caps and all-layer hull on fixture nodes 507567 / 562041 | options in the tests: caps off → 507567 not nested (and 527505 back to a bow-tie round); one-layer hull → 562041 not nested; roundabout rule off → linked; cluster caps off → one 5-node cluster; unknown links absent → 4 records differ between tiles. Code sabotage: the walk not crossing absorbed stretches → dual crossing and staggered T fail, 34 face-walk fallbacks per fixture. The PLAN 25 m diag was not built: D2 already rests on the probe's measurement |
| T5 junction heights | on 64 points per trim segment |z_J − ẑ_i| ≤ 1e-9 m; finite-difference normal slope across the trim line equal within 1e-4; min ≤ z_J ≤ max everywhere on a 0.25 m lattice | weights from plain distances instead of squared distances: the slope test fails; plain average: the match test fails |
| T6 node agreement | for every fixture junction all used approaches' ẑ at the node agree within 1/256 m (strict and completed) | completion tier refitting only one stretch at an anchored node: fails |
| T5/T6 as implemented (slice 5) | `test_road_junction_height.cpp`: X, T, Y 30 deg and a curved side arm over a tilted bumpy DEM: \|z_J - z_i\| <= 1e-9 on 64 points per trim segment (measured 5.7e-14 on both fixtures, ~25k points each), one-sided inward slope of z_J equals that of z_i within 1e-4 (h = 1 um; fixtures worst 8.7e-5), no overshoot on a 0.25 m lattice (0 violations); weights sum to 1 and average on two touched segments; zero allocation; tangent extension before station 0; every fixture junction (flat and tilted DEM): worst approach-to-node mismatch 6.9e-11 m (limit 1/256 m); a table / hump raises, other vertical kinds count `calming_ignored`; raised junction equals the raised core on all 4 trim lines incl. sidewalks (D24); ramp plateau [0, trim] (D22); feature inside the trim suppressed, beyond it kept (D23) | T5: plain-distance weights (slope mismatch 0.43 against 1e-4) and plain average (match error 0.249 m) fail; T6: one profile lifted 2 cm fails the node-agreement check (spread 0.02); D24: a constant raise after the blend steps 0.080 m on the sidewalk probes; D23: the suppressed bump would leave 0.01+ m on the trim line |
| T7 tile invariance | every junction and furniture record identical whichever fixture tile builds it; owned exactly once | degree from the tile graph instead of the context graph: differs at halo edges (P1 slice 1 negative control reused) |
| T8 encoding 7 | round trip; every rejection of 9.2 with its own malformed body; encoding 6/5 decode | flip one ring vertex: rejected as non-simple; change one t_to: rejected by the lane-span rule |
| T9 crossings acceptance | per fixture, #crossing nodes on entries == #placed + #declined; counts printed; every placed quad within carriageway + 0.5 m (it is 0 by construction) | delete one record: count mismatch |
| T10 stop-line acceptance | every signalised (entry, direction) has exactly one stop line set whose span equals its lanes; on a P1 turn-pocket fixture the line covers the pocket lane | use `carriageway_edges` instead of lanes: fails on two-way roads |
| T11 signal facing and offsets | heading opposite to travel; post at edge + 0.5 m; left post only for ≥ 3 lanes | — (plain rule check) |
| T12 carve continuity (G3-D) | carved tile around fixture junctions on a synthetic tilted, bumpy DEM: trim-line samples match approach cores ≤ 1/256 m; no step > 5 mm between road and blend at any adjacent cell pair across the outer boundary | blend from raw instead of smoothed T: steps at the edge |
| T13 cross-slope (G3-D) | per core station: least-squares slope across the carriageway cells = intended crossfall ± 0.2 % | — single-cell slopes cannot meet it (1/256 m over 1 m = 0.39 %); the test documents why it fits over ≥ 3 m |
| T14 ownership | lowest-wins and max-weight results are independent of input order (shuffle 100 times) | first-come ownership: differs |
| T15 decks | terrain under a synthetic bridge ≤ deck_top − 0.55 inside, untouched beyond footprint + ramp; lower road under the deck keeps its height | — |
| T16 terrain.class | physics cell centre (integer metre) samples classify road bed / sidewalk / verge exactly by footprint bands; edge samples equal in neighbouring tiles | Cell registration (half metres): misclassifies every road edge cell |
| T17 pyramid | carved L1 = mean-of-4 of carved L0; class L1 rank-max | — |
| T18 completion tier | each of the game's three paths reproduces the game's profile within 1 mm on a fixture where the game's own code is run in the test (copied as a frozen reference in tests/support, libm allowed in tests only) | omit the 10 % clamp: differs |
| T18 as implemented (slice 6) | `test_road_completion.cpp`: a bridge with a junction 15 m beyond each end (span, both ground stubs), a motorway at 11 % (clamp), a noisy 1.5 km road (retry) and a noisy 3 km road (ground) each equal the frozen game reference (`tests/support/road_completion_reference.h`) within 1 mm (measured 0 on all four); completed stretches at a bridge node agree within 1/256 m (T6 completion variant: 0 on both fixtures, flat and tilted DEM); a reversed entry list gives the same profiles; two-tile invariance and every fixture entry held by two tiles identical (828 and 853 repeated entries, 0 differ); origins validated | omit the 10 % clamp: grade0 0.109375 not 0.10, parity with the game 5.2 cm off, the 3 km ground road no longer fits; retry-from-tile-graph context: 1 of 2 entries differ on the two-tile world, 2 / 8 / 1 / 31 of ~830 on the fixtures; one stretch 2 cm off at a bridge node fails T6; the game's own tier is the order and tile control (0.035 m shift on entry reversal, differs between tiles) |
| T9 as implemented (slice 7) | `test_road_furniture.cpp`: pattern, length, flags and entry per node on a synthetic road (zebra 4 m, `crossing=traffic_signals` Dashes 4 m + signalised, unmarked None but placed, `width` tag, `kerb=lowered`), dead end, degree-2 join (spans-join, station 0), two ways ending at the node, junction node (`JunctionNode`), tunnel, declined stretch, markings precedence; quad inside the carriageway + 0.5 m; per fixture the union over tiles has exactly one record per crossing node (A 123 = 122 placed + 1 declined, B 52 = 50 + 2 `JunctionNode`) | one record deleted: count mismatch (in-test control) |
| T10 as implemented (slice 7) | a signal inside a two-way stretch gives one line set per direction spanning exactly `driving_lanes_at` of that direction (1 mm), a turn pocket (`lanes:forward=2`, `turn:lanes:forward=left\|through`) is inside the line; direction from tag / one-way / nearer junction end / both, `DirectionUnresolved`, `NoLanes`; crossing rule (line 3 m before the node of a signalised crossing); junction signal on an X gives 4 lines at trim + 1 m, an arm signal 30 m before it suppresses its approach's junction line; stop / give-way on a T lines only the lower rank at trim + 0.5 m, equal ranks `Ambiguous`, declined junction `JunctionDeclined`; fixtures: every placed line lies within its lanes (0 off, both fixtures) | lanes replaced by `carriageway_edges`: 4 test cases fail (two-way line, turn pocket, junction case, fixtures); crossing rule off: the signalised-crossing case fails (4 checks) |
| T11 as implemented (slice 7) | post at the travel-right edge + 0.5 m (left edge - 0.5 m for the left post), facing opposite to the travel direction (dot -1 to 1e-9); left post only on a one-way with >= 3 lanes (3 lanes: 2 records, 2 lanes: 1, two-way 6 lanes: none) | post offset 0: 3 test cases fail (10 checks) |
| Slice 7 tile invariance | every owner (entry or junction) emitted by two or more tiles has identical records (828 / 853 repeated owners on the fixtures, 0 differ) | node degrees and incident ways read from the tile graph instead of the context: 2 / 6 owners differ |
| T19 zero allocation / workers | carve with 1 and 10 workers bit-identical; CarveModel::evaluate allocation-free | — |
| Slice 9 as implemented | `test_terrain_carve.cpp` over the four busiest fixture L0 tiles (A 1856/21718, 1852/21718; B 1822/21685, 1820/21684), flat DEM and the libm-free tilted bumpy rough DEM of `carve_fixture.h`. **T12**: owner switches between adjacent cell centres bisected to 1e-10 m: core↔blend ≤ 0.12 mm, blend↔terrain 0, junction core↔approach core on the trim line ≤ 5.04 mm (bound 1 cm), trim-line samples (16 per used approach, 336 points) ≤ 2.02 mm (bound 1/256 m), every cell = half-even quantisation of the model; reported only (D41): blend↔blend up to 3.44 m, core↔core up to 2.20 m, ring overlap up to 0.296 m. **T13**: 1507 stations, LS crossfall error p50 0.00012, p95 0.00070, p99 0.00108, max 0.00155 (bound 0.002; patch ±2.5 m along the road, ≥ 3 m across, regressors grade + vertical curvature + crown). **T14**: 8851 core/blend cells × 100 seeded candidate orders, 0 differ. **T15**: synthetic bridge over a valley and a lower road: 1696 footprint cells, 542 lowered, 0 above the limit, 0 changed outside the footprint, 2048 lower-road cells unchanged, 0 clearance violations. **T17**: served L0 = `carve_terrain_tile`, header offset applied once, L1 = mean-of-4 of the served L0, L1 ≠ bare pyramid; absent centre = NotFound, roads.geom in another datum rejected. **T19**: 1 = 10 workers, evaluate 0 allocations. **Tile invariance**: 3 seams (an L0 seam, an L0 seam that is also an L2 seam, an L2 seam with roads crossing it), flat and tilted, 2048 cells each: 0 difference. **Smoothing parity** against the game's double smoothing: max 0.80, p99 0.65 height units (1/256 m), 0 NoData mismatches. **Served path**: server and four-worker bake give the same L0 and L1 containers (10 inputs per L0 tile) | T12: blends toward the raw DEM: blend↔terrain 7.03 cm; T13: single-cell slopes 0.102 (cannot meet 0.2 %); T14: first-come ownership: 354 cells differ; T15: clamp off: 542 cells above the limit; T17: no centre, wrong datum, short input list rejected; T19: an allocation in the counted scope is seen; invariance: centre-only DEM differs (0.08-1.05 m), own-L2-tile roads only differs (0.224 m on the seam with crossing roads); smoothing strength 100: 42.9 units |
| Junction gradient as implemented (D42) | `test_junction_gradient.cpp` over the junctions touching the four busy L0 tiles (71 flat, 20 tilted DEM), 0.5 m lattice of core points each junction owns: on the road bed and paved rings gradient ≤ max(B, 2·F_paved) + 0.02 at ≥ 2 m from every trim segment and ≤ max(B, own, 2·F_all) + 0.02 nearer (B steepest approach grade + crossfall, F the forced gradient between two approaches' trim points, own the approaches' slope at their trims): 0 over on both DEMs; where 2·F_paved ≤ B the interior meets B + 0.02 (worst excess over B 0.000 flat, 0.005 tilted); B + 0.02 met everywhere on the drivable rings at 36 of 71 flat junctions (D22-D24: 1), 6 of 20 tilted (5); 30 flat / 8 tilted junctions have F_paved > B + 0.02 (no surface can meet B) and are printed with their numbers. T5's no-overshoot reads the range of each approach's base, paved and full height | `CarveOptions::junction_relief_fade = false` (the D22-D24 surface): 14 flat junctions over their limit |
| Slice 10 as implemented (terrain migration phase 6) | `test_terrain_class.cpp`: **T16** on A66 tiles 1820/21684 and 1822/21685: every approach-core node's role equals the footprint band of its projection (16388 and 2110 nodes, 0 mismatches; road bed 13066, sidewalk 886, verge 2479, junction road bed 196, blend 14445; 2400 bridge-deck nodes on 1822/21685); edge nodes equal on an L0 seam and an L2 seam; **T17 class half** on L1 910/10842: rank-max equals a brute-force reference over a global node map (0 differ), L1 edge equal to the neighbour (0 differ), 10465 road nodes vs 8038 subsampled; served path: server and 4-worker bake give the same containers, 11 inputs at L0, datum None; encoding round trip and every rejection | cell-centre (half-metre) registration: 244 and 99 of 807 / 188 road edge nodes classify differently; land cover from the ancestor only (D49): 220 of 257 L2 seam edge nodes differ |
| G-1 / G-2 / G-5 as implemented (phase 6) | `[G-1]` (D50); `tests/phys/test_class_mode.cpp`: every cell of a 2 x 2 straddle equals `table.resolve` of its centre node, Absent / Missing / class-less tiles take the fallback (counts 1 / 2), zero allocation, roads kept beside the class tile, `require_classes` fails a class-less fetch after 3 attempts, the streamer installs both, `TransportHeightTileFetch` class layer (404 / wrong key = Internal); `bridges/physics_sim/tests/test_carved_prefetch.cpp` on the PKT fixture release with carved + class: 36 gate keys with class tiles, snapshot fill = live fill, class-only replacement invalidates, class-less entry declines, World state hash with prefetch at 1 / 4 workers = synchronous, prefetch installs > 0, fill miss 0 | reading the node one metre west: > half the cells differ; dropping the class identity check from `prefetch_inputs_current`: the bridge test fails |
| Goldens | `g3xs` re-recorded (encoding 7 bodies); new `g3j` (junction + furniture sections of both fixtures), `g3d` (carved L0 over 4 fixture L0 tiles on the synthetic DEM); `g3c` (class planes of the same 4 tiles and L1 910/10842, synthetic land cover; a separate set so g3d keeps its lines); Windows = Linux (g3c: Linux only so far) | — |

---

## 12. Measurement plan

1. **Junction census** (slice 12, `g2m_road_probe --junctions`) on both fixtures and
   the home tile set:
   - junctions per kind (single, cluster, `OnStructure`, partial);
   - cluster size and extent histograms;
   - trim and reach distributions;
   - hull fallbacks;
   - each decline reason;
   - excluded approaches by reason.
   Replace the [estimate] trims of 1.4 with real ones.
2. **Furniture census:**
   - crossing nodes, placed / declined by reason / pattern;
   - signalised approaches and stop lines (must be equal);
   - signs by direction source;
   - junction-node calming.
3. **G3-D metrics** on carved tiles:
   - max |trim-line mismatch|;
   - LS cross-slope error percentiles;
   - max road-to-blend step;
   - number of blends over 1:1.5;
   - max junction gradient against the steepest approach grade + crossfall (as
     measured after D42: per junction against B, F_paved, F_all and the approaches'
     own slope at their trims, interior ≥ 2 m from the trims and near zone separately,
     drivable rings asserted, outer ring reported - 4.3 property 6);
   - deck clearance violations.
4. **Cost and size:**
   - CPU-s/km² of `terrain.carved` and `terrain.class` L0 (release, 1 and 10 workers)
     against bare `terrain.height`;
   - bytes/km² of both new layers, and the added roads.geom bytes (sections 5-7);
   - time of the completion tier per L2 tile.
   G3 asks for these numbers; there is no prior bound, so the first measurement sets
   the budget.
5. **Completion tier vs game:**
   - per home stretch: strict / retry / bridge / ground / still declined;
   - for each, the max |Δz| against the game's own result (from the game's
     `RG_STRUCTURE` logs or a frozen-reference run).
6. **Smoothing parity:** max and p99 |server − game| per L0 tile on 20 home tiles.
7. **Visual/physics agreement:**
   - `bridges/physics_sim/tests/test_render_vs_physics_roads.cpp` extended to carved
     tiles: the triangle height at every class-sampled cell equals the physics height;
   - racing_game 10-junction screenshot set before and after the switch (G3-F's
     20-junction route extends it).
8. **Golden stability:** run the slice goldens on `release`, `linux-release`,
   `linux-asan-ubsan`.

---

## 13. Risks

| Risk | Effect | Mitigation |
|---|---|---|
| Profile declines remain after the completion tier | holes in carved roads after the switch (no game fallback) | the census lists them; per the Q1 answer they do not block the switch (10.4) |
| Multi-node rings (clusters) look wrong: edges attached to different nodes, hull fallback covering verge or buildings | odd paved shapes at dual-carriageway crossings | clusters are a separate Opus slice; hull flagged and counted; the 48 m cap keeps the hull small |
| Gore junctions up to 128 m | long thin rings, Shepard mixing two diverging profiles | both profiles are the approaches' own surfaces (4.2); the mix lies between them; measured max gradient |
| Shepard warp ("egg-crate") in large boxes | small bumps of ≤ c·w/2 | bounded by property 4.3.3; measured; fallback (later): flatten crowns inside clusters above 30 m |
| Carve cost: projections onto many polylines per cell | slow derivation | 16 m buckets; per-entry polylines built once per tile; measured per km² |
| Carver radius 192 m pulls up to 4 roads.geom tiles per L0 tile | cold-cache latency in the game | tiles are cached; prefetch ring requests the carved tile, not roads.geom |
| Smoothing parity is not bit-exact against the game | visible difference ≤ a few mm | measured (12.6); owner sees it once at the switch |
| terrain.class base classifier is new (game used only road vs not road) | colours or grip off-road change | the game reads only PavedRoad/UnpavedRoad/other (`RoadSurfaceMap`), so the base class is information the game ignores for now |
| Signals mapped ambiguously (no direction, far from junctions) | lines in both directions | rule 7.2 with counts; P3 review |
| Determinism of double Shepard arithmetic | Windows ≠ Linux | fixed order, no FMA (`-ffp-contract=off`, `/fp:precise`), golden `g3j`/`g3d` on both |
| Encoding 7 forces re-derivation of every roads.geom tile | store churn | expected; release id changes only for releases with roads |
| Completion tier changes strict profiles at bridge-anchored nodes | strict heights move by up to the anchor correction | game parity: the game does the same today; provenance records it |

---

## 14. Implementation slices

One commit each. CI scope: `tools\ci.ps1 -Only release` while building a slice, and
the leg set below before the commit. `tools/ci.ps1` has no `-Topic` switch in this
repo; the P1 design's "-Topic roads" means "the road tests".

Sizes: **S** ≤ 1 day, **M** 1-2 days, **L** 3+ days of agent work [estimate].

| # | Slice | Size | Model | Legs | Goldens |
|---|---|---|---|---|---|
| 1 | Rules: `junction` and `furniture` blocks in de.json, `carve_default.json`, classifier table, strict loaders, loader rejection tests; read `RoadAttributes::blend_mm` defaults into the carve table | S | Sonnet | release, linux-release | none (rules SHA moves the roads.geom config hash: update the rid/config tests only) |
| 2 | `road_surface_height_m` (4.1) and its tests T1 | S | Sonnet | release | none |
| 3 | Single-node junctions: approaches, edge polylines, corners, trims, rings, simplicity, bow-tie repair, hull fallback, declines (3.1, 3.3-3.7 without clusters); T2, T3 | M | Sonnet | release | none |
| 4 | Clusters: trim-overlap linking, caps, absorbed stretches, multi-node angular order and rings, roundabout rule; T4 | L | **Opus** | release, linux-release | none |
| 5 | Junction heights (4.2), raised junction tables (4.4), feature suppression in rings; T5, T6 | M | Sonnet | release | none |
| 6 | Profile completion tier (6.6) with a frozen game reference in tests/support; T18 | M | Sonnet (Opus review of tile invariance) | release, linux-release | none yet |
| 7 | Furniture (7.1-7.4): crossings, lines, signals, declines, `furniture_quad`; T9-T11 | M | Sonnet | release | none |
| 8 | roads.geom encoding 7 (sections 5-7), validation 9.2, ownership filter, fuzz target extension, config hash; T7, T8; `docs/formats/road_geom.md` | M | Sonnet | release, linux-release, linux-asan-ubsan | **`g3xs` re-recorded** (body SHA `17d9b7c7…ee2f9` changes), **new `g3j`** |
| 9 | `CarveModel` + `g2m.terrain.carved` deriver: inputs (5.3), smoothing port, buckets, ownership, blends, decks, quantise, pyramid; T12-T15, T17, T19 | L | **Opus** | release, linux-release, linux-asan-ubsan, linux-tsan | **new `g3d`** (carved part) |
| 10 | `g2m.terrain.class` deriver + TerrainClass encoding 9.4, format doc, fuzz target; T16 | M | Sonnet | release, linux-release, linux-asan-ubsan | new `g3c` (the class part, separate from `g3d`) **- done (terrain migration phase 6, D49, D50)** |
| 11 | Wiring: bake opt-in (both layers + roads), builtin release, `fill_physics_surfaces` class mode, a render `ClassWindow` from terrain.class, `g2m_ps_bridge` layer option; tests in `tests/phys` and `bridges/physics_sim` | S | Sonnet | full `tools\ci.ps1` + `-Only bridge-release` | none new **- partly done (phase 6: bake/builtin opt-in, `fill_physics_classes`, `ClassSurfaceConfig`, carved+class prefetch test; the render `ClassWindow` from terrain.class is open)** |
| 12 | Measurement: `g2m_road_probe --junctions --carve`, home-region run, numbers into this document, the pipeline docs and CLAUDE.md | S | Sonnet | none (docs + probe) | none **- partly done (phase 6: G-6 `road_underpass_census` + probe `--underpasses`, fixture numbers in D51; the home-region run is open)** |

**Order:** 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11 → 12.

- Slices 2-7 are pure computation with no encoding change, so `roads.geom` bytes stay
  untouched until slice 8 bumps the encoding once (as P1 did in its slice 5).
- The racing_game switch (10.2) is a separate task; by the Q1 answer (10.4) it may
  follow as soon as the carved layer works (slices 9-11), with slice 12's list of
  uncarved stretches as information, not a gate.

---

## 15. Decisions record

**D1. Port osm2streets' offset-intersect-trim on our own P1 edges (D-SF4).**
- Alt A: link osm2streets (Rust). Con: no bit-determinism guarantee, a Rust toolchain,
  its own lane model would disagree with P1.
- Alt B: Clipper2 offsetting of centre lines. Pro: robust unions. Con: symmetric
  offsets ignore P1's asymmetric bands and turn pockets; a new dependency.
- Alt C: our own port on integer-mm polylines with exact predicates. Pro: uses the
  exact P1 footprint, deterministic, no new dependency. Con: we own the edge cases.
- **Chosen: C.** The geometry must be the P1 footprint, or carving, grip and render
  disagree.

**D2. Cluster by overlapping trims, not by a 25 m distance.**
- Alt A: PLAN's 25 m rule. Con: measured 14-17-node chains up to 106 m wide (1.4),
  which breaks the tile-invariance budget and produces huge polygons.
- Alt B: no clusters. Con: dual-carriageway crossings get four overlapping polygons
  and stretches with negative length.
- Alt C: link only when trims overlap. Pro: links exactly where the surfaces would
  collide; measured ≤ 9 nodes, ≤ 45 m. Con: needs single-node trims first.
- **Chosen: C** with caps 48 m / 12 nodes.

**D3. Store each junction once (owner tile), carver reads ≤ 4 roads.geom tiles.**
- Alt A: store each junction in every tile whose halo sees it. Con: the halo (128 m)
  cannot cover reach + blend + extent (191 m), so a tile could still miss junctions;
  duplicated bytes.
- Alt B: enlarge the roads.graph halo to 192 m. Con: a roads.graph encoding change
  and much larger graph tiles for every consumer.
- Alt C: owner tile only; the carver's dependency list covers 192 m. Pro: exactly once,
  provable (5.2, 5.3). Con: a data-dependent multi-tile dependency (roads.geom already
  has one).
- **Chosen: C.**

**D4. Separate layer for furniture? No: sections in roads.geom.**
- Alt A: `g2m.roads.junctions` / `g2m.roads.furniture` layers. Pro: independent
  versioning. Con: two layers that can drift from the entries they index; consumers
  fetch twice; same inputs anyway.
- Alt B: new sections in roads.geom encoding 7. Pro: one fetch, entry indices stay
  valid, one config hash. Con: re-derives roads.geom.
- **Chosen: B.**

**D5. Profile height defined on the reference line, crown relative to it.**
- Alt A: profile at the crown pivot. Con: one-way roads sit c·w/2 above the DEM-fitted
  line on average (fill only).
- Alt B: profile on the reference (C(t) − C(0)). Pro: the fitted line stays exact, cut
  and fill balance. Con: on one-way roads the pivot edge is above the profile.
- **Chosen: B.**

**D6. Junction heights by Shepard blending of the approaches' own surfaces.**
- Alt A: least-squares plane (PLAN). Con: cannot meet crowned edges within 1 cm.
- Alt B: Coons/transfinite patches. Con: need four-sided topology.
- Alt C: linear extrapolation of each approach from its trim line. Con: wrong by
  ½·p''·u² over long gore rings (decimetres at 90 m).
- Alt D: Shepard over own surfaces with product weights. Pro: exact on trims, C1, no
  overshoot, any k. Con: mild warping inside; all + − * /.
- **Chosen: D.**

**D7. Absent crossing direction: perpendicular, no footways.**
- Alt A: add footways to roads.graph. Con: a much larger graph and an encoding bump
  for a cosmetic skew.
- Alt B: perpendicular to the reference. Pro: no new data; matches most DE crossings.
- **Chosen: B.** P3 can revisit with real footway data.

**D8. Rules location: country rules in de.json, carving parameters in their own file.**
- Alt A: everything in de.json. Con: a carve tweak re-derives every roads.geom tile
  (profile fitting).
- Alt B: furniture in a sibling file. Con: a second country file and SHA for what is
  a country rule (setbacks, crossing widths).
- **Chosen:** junction + furniture in de.json, carving in `carve_default.json`.

**D9. Carved heights as a new layer `g2m.terrain.carved`, not a carved `terrain.height`.**
- Alt A: carve inside `terrain.height` when roads are baked. Con: one layer name with
  two meanings depending on release composition; the bare heights vanish for
  consumers that want them.
- Alt B: new layer. Pro: both exist; consumers choose; "default stays until carved
  layers are selected" literally. Con: the plan text says "carved terrain.height".
- **Chosen: B.**

**D10. Order-independent ownership: lowest core wins, then largest blend weight.**
- Alt A: the game's canonical carve order (class, way id) with later writes winning.
  Con: harder to reason about; order-dependent.
- Alt B: min / max-weight. Pro: commutative, so no ordering rule is needed.
- **Chosen: B.** Junction over approach is a containment rule, not an order rule.

**D11. terrain.class node-registered at integer metres.**
- Alt A: cell-registered like the heights (half metres). Con: a physics cell centre is
  the shared corner of four samples, so ownership at road edges is ambiguous.
- Alt B: node-registered 257 x 257. Pro: one sample per physics cell, exact. Con:
  duplicate edge samples (equal by construction), render reads its south-west corner
  node.
- **Chosen: B.**

**D12. Port the game's smoothing to the server (integer kernel), not drop it.**
- Alt A: drop it. Con: changes the look of all terrain at the switch, uncontrolled.
- Alt B: keep it in the game after carving. Con: bends server crossfalls; render and
  class disagree with heights.
- Alt C: port it. Pro: parity, deterministic. Con: not bit-equal to the game's libm
  version.
- **Chosen: C.**

**D13. Quantise carved heights half-even** (the DGM1 importer and pyramid quantiser),
not ties-away. This keeps the carved pyramid consistent with `terrain.height`'s and
does not touch the open Q3 question for older code.

**D14. Completion tier in the server (game's reconstruction moved, flagged).**
- Alt A: wait for G3-C to reach zero declines. Con: the switch waits indefinitely;
  home probe has 139 of 478 stretches without profiles (`road_probe`, 2026-10-02).
- Alt B: keep completion in the game. Con: the server would carve without those
  profiles; game decks and server terrain disagree; violates "no fallback".
- Alt C: move it with provenance. Pro: game parity, single source, honest flags. Con:
  served profiles at relaxed limits.
- **Chosen: C** (owner question Q2 confirms the policy side).

**D15. No roads.graph change.** Every tag P2 needs is whitelisted since encoding 5.
`stop=*`, `crossing_ref` and `traffic_signals=*` details are not added. Their absence
costs only the all-way-stop case, which is rare in DE.

### Decisions made while implementing (slice 4)

**D16. Cluster rings by a boundary walk, not by the angular order around the centroid.**
- Alt A (3.3 as designed): sort the stubs by the direction from the cluster centroid
  and run the single-node ring over the stubs only. Con: at a staggered T pair the
  pair (west arm of node 1, east arm of node 2) gets a chord or a corner that cuts
  across the main road between the nodes, and its other edge is never part of the
  ring. A dual crossing works only by luck of symmetry.
- Alt B: union of single-node polygons and approach strips (polygon clipping). Pro:
  exact coverage. Con: a clipping library with exact integer output, new failure
  modes, no corners of its own.
- Alt C: treat each absorbed stretch as two darts. Every node sorts its darts as for a
  single node (exact half-plane and cross product). The ring is the outer face of
  that rotation system: next = the dart after the arrival dart at its node. Pairs
  are always two darts at one node, so 3.3's corner search is reused unchanged.
  Pro: a single node is exactly slice 3. The absorbed stretches' edges become ring
  pieces, so the road between the nodes is covered. Con: a stub that lies inside a
  cycle of absorbed stretches is not on the outer face. Then every layer falls back
  to the hull (counted as `face_fallbacks`; 0 on both fixtures).
- **Chosen: C.** Sabotage: arrival = departure dart (not its twin). The walk then never
  leaves a node; the dual crossing and the staggered T fail, with 34 face-walk
  fallbacks per fixture.

**D17. Corner caps and all-layer hulls (slice-3 finding (a), fixed now).**
- Finding: rings not nested at open pairs whose angular gap is under 180° but whose
  edge lines meet behind the node (unequal offsets). The chord between the two σ = 0
  points then runs on the wrong side of the node, and inner layers with smaller
  offsets poke out. Slice 3: A 97 vertices in 46 junctions, worst 663 mm; B 85 in
  50, worst 876 mm. A second source: one non-simple layer took the hull while the
  others kept offset rings.
- Alt A: tolerate it and let slice 8 validation allow it. Con: slices 8 and 10 assume
  road bed ⊆ paved ⊆ outer (9.2; 6.5 classifies by "inside the road bed ring" first).
- Alt B: clip inner rings to the outer ring. Con: polygon clipping again (D16 Alt B).
- Alt C: start each edge polyline with a cap segment from the reference point at
  σ = 0, so such a pair closes on the cap. For straight arms, a cap hit happens
  exactly when the plain corner lies at σ < 0; the result is the union of the
  half-strips per layer. When one layer needs the hull, all three take it.
- **Chosen: C.** Measured with clustering: A 5 vertices in 2 junctions, worst 352 mm
  (both on short, strongly curved approaches; D21 hands the rest to slice 8); B 0.
  Side effects: the single-node hull count falls (A 31 → 27, B 25 → 23), and A 66
  node 527505, slice 3's bow-tie example, needs no repair round any more.
- What the later slices assume: heights (4.2) and the carver's junction core (6.3)
  use only the outer ring and the trims, so the residual does not affect them. Slice
  10's class roles and slice 8's validation need the nesting.
- Sabotages: caps off → node 507567 not nested (and 527505 back to one bow-tie
  round); one-layer hull → node 562041 not nested.

**D18. No cluster `trim_min` projection.** 3.4 asked every stub's trim to reach past the
projection of every cluster node, so that no node is left outside the ring. With D16
every node is on the walk's interior side by construction, and the projection would
over-trim stubs on long clusters (a 30 m cluster would push every stub ≥ 30 m out).
Dropped; T4 checks that the nodes lie inside the road bed ring.

**D19. `ClusterTooLarge` records per discovery set.** The full over-cap component is
not computable in one tile: discovery stops at the cap, which is the point of the cap.
- Alt A: one record per component. Con: needs the whole component, which the halo does
  not guarantee.
- Alt B (chosen): each node keeps its single-node record. A node that is the smallest
  of its own discovery set also stores a `ClusterTooLarge` record (nodes = that set,
  every non-link approach untrimmed, the links as `absorbed`, no rings). The set
  depends only on links within 108 m, so the record is tile-invariant.
- Con: an over-cap component can yield more than one such record. Slice 8's encoding
  must allow several. The fixtures have none.

**D20. Links over one entry only; unresolved discovery is skipped, not guessed.**
- A stretch between two junction nodes is one `roads.geom` entry, unless two ways join
  end to end at a degree-2 node between them. Linking across such joins needs the
  entry chain and the trims of both ends of each entry.
  - Measured (`[diag]` "junction pairs joined over two stretches"): 0 such pairs
    whose capped trims would cover the chain, against 32 / 24 direct links.
  - Kept simple: one entry.
- A tile computing links of a complete node may meet a neighbour it sees only
  partially. If the capped trim of the near end plus the 30 m cap could cover the
  stretch, the link is unknown, and any discovery that reaches it is unresolved.
  The tile skips that node (16 / 17 skips over all tiles).
  - Treating unknown as no link is the tested negative control: 4 records then
    differ between tiles.
  - The owner tile always resolves its nodes: 0 uncovered junction nodes on both
    fixtures.

**D21. Judgement of the open slice-3 findings.**
- (a) Ring nesting: **a defect, fixed now** (D17). The residual (A: 2 junctions,
  worst 352 mm) and the mm rounding between layers go to slice 8 (9.2): validation
  needs a ≤ 2 mm allowance or nested construction, and the builder should send a
  junction that still does not nest to the all-layer hull. This is deferred
  because the exact check belongs with the encoding's own validation, and the
  residual does not affect heights or carving (D17).
- (b) Hull fallback rate: **acceptable, deferred.**
  - Rate before / after: A 31/536 (5.8 %) → 27/536 with caps → 21/504 ring records
    with clusters (4.2 %); B 25/305 (8.2 %) → 23/305 → 22/283 (7.8 %; the
    denominator fell by 22 merged records).
  - Of the remaining hulls (A / B):
    - 12 / 13 have a pair under 25°: a shallow merge whose facing edges do not meet
      before one approach ends at a way join;
    - 6 / 6 have an approach shorter than 12 m;
    - 3 / 3 are other.
  - Clustering fixes only short stretches between two junctions. It cannot fix the
    merges: the link test caps trims at 30 m, and the approach that ends early ends
    at a degree-2 join, not at a junction.
  - A hull is a valid simple ring, flagged and counted. It covers the gore with a
    convex shape, slightly over-paving the nose.
  - Later fix, if slice 12's home census shows it matters: continue edge polylines
    across degree-2 way joins up to the reach (the same chain walk D20 avoided).

### Decisions made while implementing (slice 5)

**D22. Junction ramp records are not part of `RoadGeomTile::features`, and a junction ramp follows the Table ratio.**
- 4.4 asked for "an extra feature record" per approach, made by the junction builder, with a ramp "of the feature's ramp length".
  - Alt A: append the records to `RoadGeomTile::features`. Con: that changes the served roads.geom bytes (section 4 is
    encoded) before slice 8 re-records the goldens, and a tile would then hold two feature sets (suppression is a function
    of the junction records, which are not stored yet).
  - Alt B (chosen): `adjust_features_for_junctions(tile, junctions)` returns the kept features, the ramp records and the
    suppressed count; `core_features()` is features + ramps sorted for the carver. Slice 8 stores the same data.
    Pro: no served byte moves; the function is a pure derivation, so a tile can recompute it.
- The ramp record is a `Table`-kind `ProfileFeature` (height = the node's table or hump height) with length = trim + 2 r, so
  its plateau is exactly [0, trim] and its one ramp beyond the trim line is r = max(10 x height, 1 m).
  - Alt: a hump's own ramp (half its 3.7 m length = 1.85 m). A `ProfileFeature` cannot express "plateau plus a ramp of
    arbitrary length" (only the Table kind has a plateau, and its ramp is capped by 10 x height), so a hump junction uses
    the Table ratio too: 1 m ramps at 7 cm and 8 cm. Con: steeper than a stand-alone hump (peak slope 1:7.1 at 7 cm over
    a 1 m smooth step, against 1:13 for the 1.85 m ramp). Accepted: a field per feature would change encoding 6 for a
    detail the junction slice does not need; revisit with slice 8's section 5 if the owner wants hump parity.

**D23. Feature suppression rule: footprint against trim.** 4.4 said "station < trim + ramp". Implemented as `station - effective_length/2 < trim`
(measured from the junction-side end, both ends of an entry): any feature whose carved footprint reaches the ring is dropped.
- Alt: the literal rule drops a feature whose centre is within trim + ramp. Con: it keeps a feature whose footprint
  already reaches into the ring (centre beyond trim + ramp but a long feature), leaving a step on the trim line.
- Measured on the fixtures: 0 features inside a ring (no traffic-calming node sits within a trim of a junction in either file);
  the rule is exercised by the synthetic bump 0.3 m inside the trim (suppressed) and 4 m beyond it (kept).

**D24. The raise of a table or hump junction enters through the approach surfaces, not as a constant over the ring.**
4.2 and 4.4 said: add `raised_height_m` inside the ring.
- Problem: the approach cores carry the raise only over the carriageway (features act over `carriageway_right..left`, no
  lateral blend). A constant over the whole ring raises the sidewalks and verges of the junction too, so on every trim
  line the junction would step by the full height against the unraised sidewalk of the core, which breaks the G3-D match
  (T12: trim-line samples equal the cores) and the property 4.3(1).
- Chosen: each approach carries its own ramp record, and z_i adds the ramp's plateau over that approach's carriageway
  band exactly as `road_surface_height_m` does for a feature. On the trim line of approach i (w_i = 1) the junction then
  equals the raised core everywhere, sidewalks included (measured: 0 difference on the 4 trim lines of a synthetic X
  table, a constant raise would step 0.080 m on the 8 sidewalk/verge probes). Inside, the raise falls off toward the
  corners where no approach carriageway reaches, which is the shape of a real raised table.
- `RoadJunction::raised_height_m` stays as the record's metadata (slice 8 encodes it); `junction_height_m` does not add it.
- Con: a junction with two approaches of different ramp state would blend; not possible, every used approach of a RaisedTable
  junction gets a ramp.
- Sabotage in the test: adding the constant after the blend fails the trim-line match by exactly 0.08 m on the sidewalk probes.

**Slice 5 findings for later slices (not defects of this slice).**
- Hull-fallback and shallow-merge junctions: at the gore of a shallow Y two diverging carriageways have their own cores 1-2 m
  apart whose heights can differ by up to ~0.3 m (profiles of independent carriageways, e.g. node 507397 on fixture B).
  `z_J` follows both, so a 1 m lattice step of 0.30 m (fixture B flat DEM) and 0.17 m (fixture A) appears inside such a
  ring. The blend does not create it; slice 9's outer-boundary blend is where a step this size matters.
- Cluster junctions blend only the stubs; an absorbed stretch has no surface of its own (4.2 lists approaches). Heights
  inside a cluster between its nodes therefore follow the stubs' tangent extensions. On the fixtures the cluster extents are
  <= 35.4 m; slice 12's measurement should compare a cluster interior against its absorbed stretches' own profiles.


### Decisions made while implementing (slice 6)

**D25. Completion anchors are recomputed from the node, not read from other entries; the tier's inputs are bounded to 60 m / 63 m.**
6.6 said: "the anchor map holds only bridges incident to the stretch's own end nodes", filled by step 2 and read by step 3.
- Problem: an anchor map filled from other ENTRIES' profiles depends on which entries a tile holds and on their order. The game's
  own map (`anchors`, filled in entry order) is tile-dependent: a stretch next to a bridge that another tile owns sees no anchor.
  Measured (test negative controls): on the two-tile synthetic world the game's tier completes the stub in the east tile and not
  in the west one; with the entries reversed the game's height at a node shared by two bridge halves moves 0.035 m.
- Chosen: `road_bridge_anchor(graph, dem, zone, node)` is a pure function of the node (the game's `abutment` walk with a total
  path budget `abutment_path_m` = 60 m, bridge edges included, starting AWAY from the first incident bridge edge in
  (way id, segment index) order; the plane is the DEM plane at the 32 m support point carried back to the node). Every step
  recomputes it per node (memoised inside one call only). The result for an entry is a function of its own strict outcome, the
  context graph within 60 m of its end nodes and the DEM within 63 m of them and along its own way. A way incident to a node
  within 60 m of an end node touches the level-2 tile of that node widened by the 128 m halo, so the context (the tiles of
  every node of the tile's ways, `road_context_dependencies`) already holds it; the DEM margin of `road_dem_dependencies`
  (65 m) covers the walk. No new dependency is needed.
- Alternatives: (a) read the neighbour entries' completed profiles - not invariant; (b) widen the context margin - not needed,
  and a 100-step walk as in the game has no bound; (c) refit every stretch of a bridge node jointly - needs a global solve.
- Order differs from the game's in two places, both on purpose: retry runs FIRST (the game completes first, then retries; a
  retry-able stretch that the ground step would otherwise take at 40 m samples keeps the strict-limit 20 m result), and the walk
  takes its outward direction from the bridge edge (the game uses the fitted heading of the stretch, which needs the stretch's
  own fit and differs from the edge direction only inside the corner arc).
- Negative controls in the tests: the tile graph handed to the tier as its context (1 of 2 entries differ on the two-tile world,
  2/8/1/31 of ~830 on the fixtures), the game's tier (differs between tiles, order dependent).

**D26. A strict-accepted ground profile is never refitted at a bridge-anchored node (6.6 step 3 refits it).**
- 6.6: "every ground stretch with an anchored end, including strict profiles at such a node". Implemented: only declined ground
  stretches are completed.
- Reason: a refit replaces a strict, in-limits profile by a relaxed one (12 % grades, 500 m radius, 40 m samples) to chase an
  abutment plane. Strict profiles end at the junction plane of the node, the bridge completed profile at the abutment plane
  carried back to the node; the two can differ by the strict height correction (<= 0.25 m). Measured on both fixtures with the
  flat and the tilted bumpy DEM: 0 nodes where a strict profile meets a completed one (when a bridge declines for lack of
  support the ground stretch next to it declines for the same reason), so the refit would never have fired; the stretches that
  ARE completed agree at their bridge nodes to 0 (<= 1/256 m, T6 completion variant).
- Con: a ground stretch strictly accepted next to a bridge that was completed (not seen on the fixtures) can sit up to 0.25 m
  off at the node. Slice 8's validator does not check this; slice 12 should count such nodes.

**Slice 6 findings for later slices.**
- The strict pass keeps the latent far-end DEM and graph dependence found in P1 (not changed or re-measured here). The
  completion tier adds only the
  60 m / 63 m reads of D25. On the fixtures every entry that two tiles both hold has the same provenance, decline code and
  vertical profile bits (828 and 853 repeated entries, 0 differ, flat and tilted DEM), so the latent case did not occur there.
- Encoding 7's section 7 (provenance) must also carry the completion options in the deriver's config hash
  (`abutment_path_m`, `retry_spacing_m`, ... and the relaxed limits) - the library takes them as `RoadCompletionOptions`;
  the deriver is unchanged in slice 6 and still writes encoding 6 with no completion. `encode_road_geom` refuses a tile with a
  non-Strict origin ("encoding 6 cannot carry a completed profile").
- On the tilted bumpy DEM (2 m amplitude, 23 m period, deliberately hostile) the tier completes about half of what the strict
  pass declines; the rest stays declined with its strict decline.

### Decisions made while implementing (slice 7)

Slice 7 is `road_furniture.h` / `road_furniture.cpp`: `build_road_furniture(tile, context builder, junctions, rules, stats)`, `furniture_quad`,
`signal_post`, `road_node_station`. Library functions: `RoadGeomTile` does not carry the records yet and no served byte changes (encoding 7,
slice 8, stores them; the golden `g3xs` 6681f11d1fbdbcdf is unchanged).

**D27. Entry furniture belongs to ONE entry chosen from the context graph; junction furniture to the junction record.**
- Decision: a record is owned by exactly one entry and emitted by every tile that holds that entry, identical in all of them.
  Choice (reads the context graph only): an interior node of a stretch: that entry; a node at a degree-1 or degree-2 stretch end:
  the incident stretch end that STARTS at the node (smallest way id, then ref), else the one that ends there (end station); a node
  of degree >= 3: the smallest (way id, ref, at_end) incident stretch end (a crossing there is `JunctionNode`-declined; a
  signal or sign there is junction furniture and takes the junction record instead, see below). Lines and signals derived from a
  junction node are emitted only where the junction record is passed in (the owner tile), `junction_index` = 1 + its index in the list.
- Alternatives: (1) every tile emits every record it can see: duplicates across tiles, the owner filter of slice 8 would have to
  dedupe by geometry; (2) choose the owner by the tile that holds the node: the chosen entry may not be in that tile (a way is held
  by every tile it touches), so records would go missing. Con of the decision: a tile without the chosen entry emits nothing for the
  node (counted in `crossing_not_ours` / `controls_not_ours`), the identity per tile is `crossing_nodes == placed + declined + not_ours`.
- Why: it is the same ownership principle as D-5.1, it needs only what every tile has, and the fixtures confirm it
  (828 / 853 repeated entries, 0 differ; with the tile graph as context 2 / 6 owners differ, the negative control).

**D28. Rule readings of 7.2 and 7.3 that the design leaves open.**
- Case (a) association (a signal or sign node on an entry within `signal_assoc_max_m` / `sign_assoc_max_m` of a junction end) measures
  the distance to the junction END NODE, not the trim line: the trim needs the junction, which only its owner tile builds.
  The 60 m / 40 m are therefore 60 m / 40 m to the node. Con: the trim lies a few metres nearer, the reach is that much shorter.
- A `crossing=traffic_signals` node is also a signal source (a signalised crossing is the "common case" of 7.2, the design says the line
  goes before the quad); its crossing flag `signalised` is set either way.
- Sign setback: moved back (away from the junction) by 0.5 m, the same sense as the signal setback of 1 m.
- Left post (7.3): only where the direction has >= 3 lanes AND no opposing driving lane exists (a one-way): on a two-way road the
  left side of a direction is the road's centre, a post there would stand in the carriageway. Deviation from the literal design,
  which does not exclude two-way roads. Result: 3 lanes one-way gives 2 records, 2 lanes 1, a two-way road never a left post.
- One line set per (entry or approach, travel). Signals take precedence over signs at one junction. A case (b) line is suppressed when an
  (a) line exists for the same entry and travel (`suppressed_junction_lines`). The crossing rule (move the line upstream to the crossing
  near edge - 1 m, never downstream) applies to (a) lines and (b) signal lines, not to (b) sign lines. A (b) approach that is a one-way
  leaving the junction is skipped silently. All approaches of equal rank: one `Ambiguous` record per sign node. A declined junction:
  one `JunctionDeclined` record per control node. A sign without a direction away from a junction end: `DirectionUnresolved`.
- One signal record per line set (the first contiguous lane group); the left post carries flag bit 4. Facing is derived by
  `signal_post` (opposite to the travel direction) and not stored. t values and widths are mm-rounded doubles.
- `kFurnitureInsideJunction` (flag bit 3, "inside a junction ring") is reserved and never set: it needs the junction, which only the owner
  tile builds, and setting it from the tile that holds the entry would break tile invariance. Slice 8's junction section can derive it.
- Rank: higher `RoadGraphWay::rank` is more important (osm_roads.cpp kDefaultHighways); the lower ranks are controlled.

**Slice 7 findings.**
- Fixture A (`frankfurt_hoechst`): 123 crossing nodes in the world graph (the design's 125 counted two nodes the road graph drops),
  122 placed, 1 declined (`StretchDeclined`), 0 `JunctionNode`; 61 stop lines + 1 give-way, 61 signal posts (2 left posts), no junction-derived lines.
  Fixture B (`hoechst_a66`): 52 crossing nodes, 50 placed, 2 `JunctionNode`; 73 stop lines + 1 give-way (5 junction-derived), 71 posts (4 left),
  1 `NoLanes`, 1 line decline `StretchDeclined`. Every placed crossing quad lies inside the carriageway (+- 0.6 mm), every line within its lanes.

### Review of slices 5-7 before slice 8 (tile invariance and the handed-forward issues)

**D29. The completion walk budget is bounded by the DEM margin: `abutment_path_m` <= 61.5 m (slice 6 accepted 62 m).**
- Finding: D25's argument counted the walk (60 m) plus the 3 m footprint against the 65 m margin of `road_dem_dependencies`,
  but not the bilinear sample, which reads a cell whose square ends up to 0.5 m beyond the sample point per axis. The default
  60 m reads at most 63.5 m away (safe); the options check accepted up to 62 m, which reads up to 65.5 m away, i.e. a DEM tile
  the tile holding the entry need not load. Not a defect of the served data (the deriver did not run the tier yet), but the
  library accepted options under which the D25 claim is false.
- Fix: `kRoadCompletionMaxAbutmentPathM` = 61.5 (61.5 + 3 + 0.5 = 65, a `static_assert` against `kRoadCompletionDemMarginMm`);
  `complete_road_profiles` refuses anything above it.
- Test (`test_road_completion.cpp`, "the walk budget bound keeps every DEM read inside ..."): a bridge ending 65.3 m west of a
  DEM tile edge, outside the 128 m halo, with a second bridge and a ground road beyond it so that the walk's path is 61.4 m or
  61.9 m. With the tile's own DEM dependencies and with a DEM that has no holes the tier gives the same result for every entry
  at the 61.5 m budget (completed at 61.4 m, declined at 61.9 m in both). Control: at slice 6's 62 m budget the 61.9 m walk
  finds its abutment with the full DEM and not with the tile's own (the hazard). Sabotage: the old bound 62 accepts 61.6 m and
  62 m (2 assertions fail).
- Alternatives: (a) widen `road_dem_dependencies` to 66 m - changes the DEM dependency set (and the release) for a budget nobody
  uses; (b) keep 62 and document - leaves an option value that breaks invariance.

**D30. A completed profile takes the strict pass's anchor at a bridging end node where the strict pass has one (closes D26's 0.25 m gap).**
- Finding (handed forward by D26): a strict ground profile ends at the strict anchor of a bridge boundary node (the approach plane
  with the <= 0.25 m feasibility correction toward the span's cubic rise), a completed bridge stretch at the same node at the
  walk's uncorrected abutment plane. Not seen on the fixtures (0 such nodes), but reachable whenever the strict anchor at the
  node exists and the bridge stretch still declines - e.g. a bridge with a junction at an interior node (both its stretches
  decline at the interior node, the outer nodes keep their strict anchors). The synthetic scene (30 m bridge, a side road at its
  middle, a 0.5 m ground step under it) measured 0.148 m at one node and 0.152 m at the other. Junction continuity < 1 cm is a
  G3-D criterion, so this is fixed rather than flagged.
- Fix: `road_strict_anchor` (road_profile.h) exposes the strict pass's `surface_anchor`; the tier's anchor of a bridging node is
  that anchor when it succeeds and is inferred, else the walk's plane (D25). Both directions close: a completed bridge next to a
  strict ground profile, and a completed ground stretch next to a strict bridge. `RoadCompletionStats::strict_anchors` counts
  the nodes (0 on both fixtures, flat and tilted DEM: their results are unchanged byte for byte).
- Tile invariance: the strict anchor at an end node is exactly what the strict pass of this entry reads at that node, so the
  tier's inputs become "the entry's strict inputs + the D25 walk"; it inherits P1's latent far-end read (`surface_anchor`
  reads the span's far end) and adds nothing beyond it. The fixture invariance check (828 / 853 repeated entries) still reads 0
  differences; the far-end measurement is in D32.
- Test (`test_road_completion.cpp`, "a completed bridge stretch meets the strict profiles ..."): gaps 0 / 0 m (limit 1/256 m),
  the walk plane at the node sat 0.148 m off; a ground stretch forced to decline next to the same node completes to the strict
  anchor height. The fixture test now also checks the mixed-node spread (<= 1/256 m). Sabotage (walk plane kept): 0.1484 m,
  0.1523 m and 0.1484 m against 1/256 m, 3 assertions fail.
- Alternatives: (a) refit the strict ground profile to the walk plane (6.6's literal step 3, rejected by D26: replaces an
  in-limits profile by a relaxed one); (b) flag the node and leave the gap - violates G3-D; (c) correct the walk plane by the
  same feasibility projection inside the tier - duplicates the strict computation and still differs where the strict pass
  averages several approaches.

**D31. Furniture review for slice 8: no record is lost or duplicated at a tile border once the owner filter is applied; two
defects fixed in the junction pass and the crossing rule.**
- Ownership as encoded by slice 8: entry furniture is stored with its entry in every tile that holds the entry (5.1, like P1
  features and the entries themselves), identical in all of them (the fixture check: 828 / 853 repeated owners, 0 differ);
  junction furniture is built from the OWNED junction list only (the junctions section 5 stores) and stored once, with
  `junction_index` = 1 + the index in that stored list. A junction is always resolved by its owner tile (D20) and its approach
  entries are all in that tile (5.2), so nothing derived from a junction can go missing; a tile that is not the owner neither
  builds nor stores it, so nothing is duplicated.
- Defect 1: `junction_controls` read the controls of a ClusterTooLarge record as well. Its members keep their single-node records
  (D19), so a control node inside an over-cap chain got its lines from its own junction AND a `JunctionDeclined` record from the
  ClusterTooLarge one (contradicting, and one more per overlapping discovery set). Not on the fixtures (no ClusterTooLarge record).
  Fix: the pass skips ClusterTooLarge records. Test: five junction nodes 18 m apart (72 m), signals on node 3: 8 records from node
  3's own junction, 0 from the ClusterTooLarge record, 0 `JunctionDeclined`. Sabotage (no skip): 1 record from the
  ClusterTooLarge record, 1 `JunctionDeclined`, 4 further assertions fail. The node counters counted a junction control node twice
  (once as an entry node, once by its junction) and `junction_controls_not_owned` was never incremented; both corrected
  (counted once; not-owned = no listed non-ClusterTooLarge junction holds the node).
- Defect 2: the crossing rule moved a line to a station where its travel had no lane and kept the lane groups of the original
  station: a record whose [t_from, t_to] is not a lane group at its own station, which 9.2's acceptance rule rejects at decode
  (a whole tile would fail to encode). Reachable where a lane ramps in from zero width near a stretch end. Fix: the line moves
  only onto a station with lanes of its travel, else it stays (`kept_without_lanes`). Test: a forward lane ramping in from 0
  (white-box ramp searched so that 2.5 m has no lane and 3.0 m has one), a crossing moving the line from 3.0 to 2.5 m: the line
  stays at 3.0 m on its lane span. Sabotage (slice 7's code): line at 2.5 m with 0 lanes there, 4 assertions fail. Fixture
  counts unchanged (63 / 82 moved lines summed over tiles).

**D32. Verdict on the completion tier's tile invariance; the strict far-end dependence re-measured.**
- The tier (retry, bridge, ground; D25 anchors recomputed from the node, D29 budget, D30 strict anchors) is a function of the
  entry's strict inputs plus the walk, all of which every tile holding the entry loads. Verdict: invariant, with one documented
  exception it inherits from P1 (below).
- The slice 6 test (828 / 853 repeated entries, 0 differ) compares only entries that two tiles both hold and never reads near a
  bound; its control (the tile graph as the tier's context) shows it can see a context dependence, but not a DEM one. Added: every
  entry of every fixture tile against a WORLD reference (the same tile built with the whole world graph as context and a DEM
  without holes), flat and tilted bumpy DEM: 1980 (A) and 1519 (B) entries, 21 / 85 / 12 / 223 completed; the completion outcome
  differs from the reference for 0 entries wherever the strict outcome agrees. Control (tile graph as the tier's context): 2 / 9 /
  1 / 33 differ. The bound itself is covered by D29's fence test.
- **Strict-pass far-end dependence (P1, latent): re-measured.** `surface_anchor` at a bridge boundary node projects both raw
  anchors of the bridge span onto its feasible cubic rise, so it reads the span's FAR end: the graph around it (its ground approach)
  and the DEM 32 m + 3.5 m beyond it. A tile that holds a stretch ending at the near end but not the bridge loads neither. Fixtures:
  0 of 1980 / 1519 entries differ from the world reference in the strict outcome (flat and tilted DEM): no stretch ending at a
  bridge boundary has the bridge outside its tile's DEM margin there. Synthetic (`test_road_completion.cpp`, "strict pass: the
  latent far-end read ..."): a road ending 188 m outside its tile at a 600 m bridge: tile 455 declines it ("road DEM: missing
  elevation dependency"), tile 456 profiles it; with the tier the west copy becomes a ground completion and the east stays strict.
  Fixed by D35 (option 1, a second discovery stage). Options considered: add every bridge way incident
  to a node of the tile's ways to the DEM dependency segments (and its far end's ground approach to the context), or bound
  `surface_anchor`'s far-end read to the tile's margin and decline both copies alike. Slice 12's census should count bridge
  boundaries whose span leaves the margin.

**D33. Furniture flag bit 3 (`kFurnitureInsideJunction`) stays reserved and never set; acceptable for P3 markings.**
- Setting it needs the junction (its trims), which only the owner tile builds; entry furniture is emitted by every tile holding the
  entry, so a tile-dependent bit would make the copies differ (D27). Encoding 7 keeps the bit defined-but-reserved: the decoder
  rejects a record that sets it (9.2 "bits 0-4" is read as "the defined bits, of which bit 3 must be 0" until a later encoding
  gives it a writer).
- A later consumer (P3 markings, the carver) derives "inside a junction ring" itself: the entry furniture's (entry, station) and
  the trim of that entry's end in the owner junction's approach list, which section 5 of the owner tile carries (σ < trim measured
  from the junction-side end). This needs the owner tile, which the carver loads anyway (5.3: every L2 tile within 192 m). So the
  information is not lost, only not denormalised. Alternative rejected: let the owner tile set the bit on its own copies - the
  copies of one record would then differ between tiles.

### Decisions made while implementing (slice 8)

**D34. Encoding 7 as implemented: where it deviates from 9.1-9.3, and why.**
- **Ring nesting (settles D21).** The validator allows 2 mm (`ring-nesting=2mm` in the config text): each layer's vertices are
  rounded to mm on their own, so a shared edge can stand up to ~1.5 mm outside the next ring. The builder checks the same rule
  (`junction_rings_nested`) before a record leaves it: simple offset rings that do not nest take the all-layer hull
  (`JunctionStats::nesting_hulls`, `hull_when_not_nested`); a ring above 1024 vertices (`kJunctionMaxRingVertices`, the
  quadratic ring checks' budget) takes the hull too; a hull that still does not nest declines the junction `Degenerate`.
  Fixture A: hull fallbacks 23 of 504 junctions (21 before), 3 of them by nesting, 0 violations; fixture B 22 of 283; largest
  outer ring 246 vertices. Alternative rejected: build the inner rings by containment - a second ring algorithm for a 1.5 mm
  effect. Alternative rejected: exact nesting - rejects valid records.
- **Counts.** `approach_count` is 1..4096 and `node_count` 1..4096 (9.1: 2..64 and 1..12). A declined single (`SingleApproach`)
  has one approach, and a `ClusterTooLarge` record carries its whole component; the builder's caps decide the real sizes, the
  decoder only bounds memory. Budgets per tile: 65536 junctions, 262144 ring vertices, 65536 furniture records; every ring
  vertex within 1 km of the owner node per axis.
- **raised_node** is encoded after `raised_height_m` (svarint, 0 unless RaisedTable): slice 5 made it part of the record, and the
  carver needs it to place the table plateau without rebuilding the junction.
- **Trim bound** is [0, max(0, L - 0.5)] (9.2 reads [0, L - 0.5], undefined under 0.5 m); a NoProfile/Tunnel approach has trim 0.
- **Furniture sort key** is (junction_index, entry_index, kind, station, node_id, travel, t_from, flags), the builder's order of
  slice 7 (9.1 named fewer keys, which left equal-station lane groups unordered).
- **Bit 3** of the furniture flags is rejected (D33).
- **Config text.** `enc=6` is replaced by `enc=7`, not appended to; the completion options join the text with their values
  (`completion=1 completion-retry=20/16384 completion-abutment=32/60 completion-ground=40/32 strict-anchor=1`), so a change of
  any served option moves the config hash. `road_geom_config_text()` is the one source; the test pins the literal text.
- **Encoding 6 stays writable** when `RoadGeomTile::encoding` is 6 (no junctions, furniture or non-Strict origin; the encoder
  refuses otherwise): the P1 body hash `17d9b7c7...` stays reproducible and the fuzz target re-encodes a seed of each encoding.
- **Features (section 4) are not junction-adjusted.** The tile stores the features as P1 builds them; a carver calls
  `adjust_features_for_junctions` with the owner tile's junctions (D23's suppression needs the trims, which only the owner holds,
  and a junction-adjusted section 4 would make the copies of an entry differ between tiles).
- **Ownership.** `road_tile_contains` (half-open floor on integer mm) decides both the build filter and the validator, so a node on
  a tile edge belongs to exactly one tile.

**Slice 8 as implemented.**
- T7 (`test_road_geom_enc7.cpp`, "T7"): every junction is stored by exactly one tile and equals the record every other tile's build
  of it produces; entry furniture is identical in every tile that holds its entry. Fixture A: 10 tiles, 505 stored, 0 stored twice,
  0 lost, 0 differing, entry furniture repeated 118 times with 0 differing (A has no junction furniture). Fixture B: 12 tiles, 288
  stored, all 0, junction furniture 10, entry furniture repeated 96 with 0 differing. Negative control (tile graph as context):
  A lost 1 / differing 42 / entry furniture differing 24; B differing 37 / entry furniture differing 12. Sabotage: no ownership
  filter - the build's own validation rejects the tile, T7 fails.
- T8 ("T8"): one rejection per rule (37), each also as a malformed body the decoder rejects (except "t not mm-exact", which a body
  cannot carry), round trip and truncation of sections 5-7 on a synthetic scene (3 junctions, 12 furniture, 9 entries); encoding-6
  and encoding-5 bodies decode. Sabotage: lane-span and nesting checks disabled - 6 assertions fail.
- Goldens: `g3xs` `all 1290583 6681f11d1fbdbcdf` -> `all 1521342 0596d2bf1d42c58e` (served tiles now carry completed entries
  and the sections 5-7; the per-fixture `stretches` hashes move because completed entries get cross-sections, the `tiles`
  hashes do not); new `g3j` `all 7108 869f7df74a9dc10f` (A: 505 junctions, 499 furniture, 21 of 1980 entries completed; B: 288,
  496, 12 of 1519).
- Far-end dependence of the strict pass (D32) re-measured on the served build: unchanged - A 1980 entries over 10 tiles (21 / 85
  completed flat / tilted), B 1519 over 12 (12 / 223), 0 strict and 0 completion differences against the world reference.


### Decisions made while implementing (D32 follow-up, before slice 9)

**D35. The strict far-end read is closed by a second dependency-discovery stage (D32 option 1).**
- **Decision.** roads.geom's dependency set gains every bridge way incident to a node of the tile's ways (found in the merged
  stretch context): the elev.base L0 tiles along those bridges plus the 65 m margin (`road_dem_dependencies`) and the roads.graph
  tiles of every node of those bridges (`road_context_dependencies`), so the far end's ground approach is in the context.
  `road_bridge_dependencies(tile_graph, context, zone, exclude)` names them. A tile that holds a stretch ending at the near end of
  a bridge now loads exactly what `surface_anchor` reads at the far end (the 32 m approach walk plus the 3.5 m support footprint,
  inside the 65 m DEM margin and the far-end node's graph tile, whose 128 m halo holds the approach).
- **Why a second stage.** The bridge is not always in the tile's own graph: it is known only once the stage-1 context tiles (the
  tiles of every node of the tile's ways) are resolved. `IDeriver` therefore gains `discovery_stages()` (default 1, at most
  `kMaxDiscoveryStages` = 4): the server calls `additional_inputs_for` again with every input resolved so far until a stage names
  nothing new or the count is reached. roads.geom uses 2; stage 2 is a fixed point (bridges incident to a tile-way node are fully
  known from the stage-1 tiles, and the far end needs one hop, not a walk), so the set stays bounded: at most the bridges' graph
  tiles and DEM along them. `derive()` recomputes both stages from its inputs and rejects an incomplete set ("incomplete
  dependency set"), so a stage-1-only input list cannot silently derive the old tile. Every other deriver keeps one stage and
  identical behaviour.
- **Alternatives.** (a) D32 option 2, bound the far-end read to the tile's margin and decline both copies alike: no server change,
  but it declines profiles that are well defined (every bridge longer than the margin with a stretch ending at it) and changes
  served bytes for the worse. (b) Widen stage 1 to a fixed radius (e.g. all graph tiles within 1 km): a single stage, but an
  arbitrary radius is still a bound a long bridge can cross, and it multiplies every tile's inputs. (c) Read the far end through
  the bridge's own tile output: violates the "no stage reads neighbouring outputs" rule of `IDeriver`. Option 1 with a generic,
  bounded multi-stage discovery is exact and costs only on tiles that touch a bridge.
- **Effect.** Served bytes on both fixtures are unchanged: stage 2 adds no graph tile and no DEM tile on any of the 10 / 12 fixture
  tiles (no stretch ends at a bridge whose span leaves its tile's margin), goldens `g3xs` and `g3j` unchanged. The config text gains
  ` bridge-far-end=1`, so the roads.geom config hash and every release id containing roads.geom move.
- **Tests.** The synthetic far-end case (`test_road_completion.cpp`, "strict pass: the far-end read of a bridge anchor is
  tile-invariant ...") is now an invariance assertion at spans 600 and 1000 m: tiles 455 and 456 give the same strict outcome.
  Sabotage / negative control: the stage-1 set alone (`road_geom_inputs(..., false)`) - tile 455 declines ("road DEM: missing
  elevation dependency" at 600 m, "bridge boundary requires ground approach context" at 1000 m). The world-reference test asserts
  0 strict differences (was counted only). `test_road_geom.cpp` ("... second stage (D35)") checks the deriver's stages (stage 1
  without the far-end graph tile, stage 2 with it, the east tile complete at stage 1) and that `derive()` rejects the stage-1
  inputs; `test_server.cpp` checks the server's stage loop (1, 2 and 4 stages, stop at the first stage that adds nothing).
  D29's fence test now builds on the stage-1 set explicitly (D35 would add DEM around its bridge and hide the control's missing
  cell; the fence is a property of the stage-1 read).

### Decisions made while implementing (slice 9)

**D36. Readings of 6.2-6.4 where the text left a choice.**
- **Smoothing rounding.** 6.2 writes `h + (35·(s − h) + 50)/100, rounded half-even`; the `+ 50` is a half-up rounding, which
  contradicts "half-even". Implemented: `h + round_half_even(strength·(s − h) / 100)` exactly on int64 (`div_half_even`), and
  each axis pass `round_half_even(Σ k·h / Σ k)` over the taps that are not NoData. A tap row whose own centre cell is NoData gives
  no x value and is skipped by the y pass (the game's first pass yields NaN there and its second pass skips the NaN), so the
  NoData pattern matches the game exactly (0 mismatches measured). Alternative: the literal `+ 50` then floor - half-up, the only
  rounding in the layer that would not be half-even (D13); rejected.
- **Junction blend widths along the ring.** 6.3 says corner vertices take the mean of their two trims' values. Implemented: one
  knot per used approach and side, at the ring point nearest the approach's outer edge on its trim line, whose value is the
  approach's own blend width at the trim station (the linear interpolation between the two stations of the approach's station set
  that bracket it, computed from the station set, not from the tile's culled line, so every tile gets the same knot); between
  knots the width is linear in arc length, cyclic. Corners therefore take the arc-weighted value, not the mean. Reason: the knot
  on the extension of a trim line must equal the value the approach blend uses there, or the two blends step on that line; a
  corner mean adds a kink in the width for no gain. Alternative (the mean at corner vertices): continuous too, but a second rule
  for the same quantity; rejected.
- **Deck ramp distance.** 6.4 writes "e = the distance inside the footprint edge". Implemented: `e = min(s, L − s)`, the station
  distance from the nearer bridge end, laterally a hard footprint (paved + 1 m). The ramp lets the clearance meet the ground at
  the abutments, where the deck itself meets the road; a lateral ramp would leave terrain under the deck edges up to the deck top.
  Alternative (lateral distance): rejected for that reason; T15 tests the implemented rule.

**D37. The smoothed DEM outside the covered region, and what the model builds.**
- **Decision.** `CarveTerrain` tabulates the smoothed DEM over the tile grown by 32 cells (`kCarveMarginCells`) and computes any
  other cell on demand with the same function. A junction ring's blend knot reads the DEM at the trim point of each approach, which
  can lie up to ~190 m from the tile (5.3's own radius), outside the 3 x 3 window; there the blend width falls back to `b_class`
  and the knot is counted (`CarveStats::junction_knots_outside_window`), because that value is not tile-invariant. Measured: 0 on
  both fixtures, flat and tilted. Approach stations obey 5.3's bound (≤ 15 m + 2 cells outside the tile) and never fall back.
- **Culling.** Approach stations are built only over reference segments within 64 m of the covered region, and junction surfaces
  only for junctions whose ring plus blend can reach it (`make_junction_surface` is the expensive call). Both only drop work
  that cannot touch a cell; the tile-invariance test covers them.
- **Alternatives.** (a) A 5 x 5 DEM window: knots would always read real DEM, at 25 instead of 9 inputs per tile, for a case
  that does not occur on the fixtures. (b) Store the knot widths in roads.geom: changes the served encoding for a carver detail.
  Both rejected while the counter stays 0; slice 12 reports it for the home region.

**D38. Ends without a junction blend round the end (end caps).** 6.3 allows an approach blend only "with s inside the trimmed
range", which leaves a cliff at a dead end or an untrimmed end. Implemented: beyond a global end the blend continues round the
end as a cap (the distance to the end line, the width interpolated across the road between its two sides). Since D40 this is a
property of the band rectangles, not separate code. Alternative (no cap): a step of up to the cut depth at every dead end;
rejected.

**D39. Wiring pulled forward from slice 11; output datum of L1-L6.**
- `BuiltinTerrainDerivers` gains `carved`; `ordered(include_roads, include_geometry, include_carved)` appends it after the road
  derivers, and `bake` with `--layers g2m.terrain.carved` adds both road layers (6.1: one release, one geometry). T17 and the
  served-path test need a deriver the server can run; slice 11 keeps the physics, render and bridge parts. `terrain.class` does
  not exist yet, so naming the carved layer does not add it (6.1's "naming either adds both" follows in slice 10).
- Release ids: a release changes only when it names the carved layer; terrain-only, roads.graph and roads.geom releases keep
  their ids (the test checks a carved release differs from the same release with roads.geom only).
- L1-L6 are labelled DHHN2016, as `g2m.terrain.height`'s pyramid is (the children carry no datum the pyramid code reads). L0
  takes the DEM's datum and rejects a roads.geom tile in another one.

**D40. Blends are per segment band rectangle and per ring edge; a core needs an unclamped foot.**
- **Problem.** 6.3's approach blend uses the nearest projection: d = |t| − outer at the projected s, width and edge height at
  that station. Inside a bend the nearest segment switches on the medial axis, where the two stations' widths and edge heights
  differ: a step at w > 0 (measured up to several cm between blend and terrain before the fix). Outside a bend the projection
  clamps to a vertex, so the core (radial distance) and the rectangular blend next to it disagreed by up to 1 cm. The trim cut
  of the nearest-projection blend produced a third step.
- **Decision.** Every segment of a line in the cell's bucket offers the blend of its own band rectangle: the segment's part of
  the carved range (between the trims, the global ends where there are none) times [outer_right, outer_left]; d is the distance to
  that rectangle, the width is interpolated along the segment and, round an end, across the road between its two sides; the edge
  height z_b is the core height at the clamped point. Each junction ring edge likewise offers its own blend from its clamped
  nearest point, width `ring_blend_width` at that arc position. The largest weight wins, ties to the lowest z (6.3's own rule):
  the maximum of functions each continuous in the position is continuous. A cell is an approach core only where its foot on the
  nearest segment is not clamped to a vertex; the wedges beyond the vertices on the outer side of a bend belong to the blends,
  whose rectangles meet the core there exactly.
- **Effect.** T12 on all 8 tile/DEM combinations: blend↔terrain 0, core↔blend ≤ 0.12 mm. Cost: every segment of a line in the
  bucket within reach is evaluated (part of the derive cost below).
- **Alternatives.** (a) The nearest projection of 6.3: steps of cm, rejected. (b) Smooth the station switch across the medial
  axis: approximate, and the cliff at the trim cut stays. (c) The distance to the polygonal outer boundary of the whole core
  (offset polygon, union with the caps): exact too, but needs polygon offsetting and boolean union per line; the rectangle
  maximum gives the same continuity with plain arithmetic. (b) and (c) rejected.

**D41. The junction gradient is measured, not bounded; T12's other owner switches are reported.** (Gradient part superseded by D42.)
- 12.3 lists "max junction gradient against the steepest approach grade + crossfall" and 4.3 suggests the Shepard surface stays
  within its approaches' slopes. It does not: the surface is a convex combination of the approach surfaces (no overshoot, 4.3.3
  holds), but its gradient is not bounded by theirs - where two trim segments are close (gore corners, short arms) it turns from
  one approach surface to the other within a fraction of a metre. Measured on the busy tiles (91 junctions): max gradient 1.10
  (A66 tile 1822/21685, flat DEM, a 4-approach junction whose bound is 0.025), every junction above its bound + 0.02, max ratio
  44. The test asserts what carving itself must keep: no discontinuity inside a junction (max gradient ≤ 1.5; a step reads as a
  gradient of tens - the deck clamp under the flat DEM's ground-level bridges gave 12.7 before clamped samples were excluded) and
  fewer than 1 % of blend stations steeper than 1:1.5. A bound would need a different junction surface (P3, with the cluster
  review); recorded as an open item, not hidden.
- T12 asserts the switches the design names (core↔blend, blend↔terrain, the trim line). The others are reported: blend↔blend up
  to 3.44 m (two blends of equal weight meet on their bisector; the max-weight rule is order-independent but not continuous
  there), core↔core up to 2.20 m (lowest-wins between overlapping cores of roads at different heights, 6.3 rule 2), ring overlap
  up to 0.30 m (a junction ring reaching past its own approach's trim line, e.g. merging ramps). They are properties of 6.3's
  ownership rules, not of the implementation; making them continuous is a design change.

**Slice 9 as implemented.**
- Tests: section 11 "Slice 9 as implemented". New golden `g3d` `all 346681 7e1f25b0cea2bd92` (heights and owner counters of
  the four busy tiles on the tilted DEM, 1 = 4 workers); the other goldens and every existing release id are unchanged.
- G3-D measurements (12.3) on the two fixtures (A frankfurt_hoechst, 86 L0 tiles; B hoechst_a66, 84), flat / tilted DEM:
  trim-line mismatch ≤ 2.02 mm; LS crossfall error max 0.00155; road-to-blend step ≤ 0.12 mm; blends steeper than 1:1.5 A 0 of
  172435 / 837 of 133847 stations, B 0 of 137739 / 256 of 96292 (busy tiles 74 of 29963); junction gradient as D41; deck
  clearance violations (core cells the clamp lowered: a ground road closer than 0.55 m under a deck) A 1229 / 15, B 2570 / 28 -
  the flat DEM puts every bridge at ground level, so the flat counts are a property of the synthetic DEM; junction knots outside
  the window 0.
- Derive cost per level-0 tile (`"carve: per-tile census and derive cost of the fixtures (diag)"`, release, Windows, two runs
  on an otherwise idle machine): mean 201-445 ms on 1 worker and 97-172 ms on 10 workers (busiest case: A on the flat DEM),
  worst tile 0.59-1.54 s, of which smoothing and model build 89-126 ms (serial); about 3-7 CPU-s per km². A third run while
  other test suites saturated the CPU measured 0.48-3.9 s mean / 2.5-9.7 s worst - an upper bound, not a figure for the
  layer. `terrain.height` L0 is a copy. No budget exists (12.4); the blend loop in `CarveModel::evaluate` evaluates every
  candidate in the bucket, so a distance pruning bound there is the obvious follow-up if the cost matters.

### Decisions made after slice 9 (junction gradient)

**D42. The junction blend mixes the approach bases; each approach's cross-section relief fades out from its trim line.**
- Diagnosis (busy tiles, D22-D24 surface, gradient on a 0.25 m lattice of the core points each junction owns): 71 flat and 20
  tilted junction rows, all above B + 0.02 (B = steepest approach grade + crossfall at the trims). Two separate causes:
  - **The approach surfaces disagree at the ring, by data.** The forced gradient F (max |ẑ_i(P) − ẑ_j(Q)| / |PQ| over trim
    points of two approaches) exceeds B + 0.02 at 69 of 71 flat junctions when verge points count (up to 2.95), and over
    the paved points alone at 30. At ring corners one approach's outer verge edge sits centimetres from another approach's
    paved edge (F_all 1-2.6 at 3-approach junctions of the A66); trims 3-10 m apart carry crowns of 8-12 cm (F_paved
    0.04-0.08 on flat terrain). The worst case, node 507507 (A66 1822/21685, 1.10), has two trim segments that CROSS
    (approaches 2 and 3 near (466676, 5551487), heights 0.33 m apart): F = 2.95, a ring defect, not a surface one.
  - **The blend carried that relief into the interior.** D22-D24 evaluates ẑ_i at every point of the ring, so far from
    approach i laterally it reads approach i's crown fall and verge drop (a verge slope of 0.24 at a road-bed point of
    node 256327242 whose forced gradient is 0.012). By distance to the nearest trim, flat DEM: < 0.5 m max 1.10, 0.5-1 m
    0.39, 1-2 m 0.24, 2-4 m 0.23, > 4 m 0.20 (7100 of 160930 points over B + 0.02). The worst point of a junction lies a
    median 0.27 m from a trim. The weight shape (∇w times the spread of the ẑ_i) dominates at 36 of 71 flat junctions.
    Tilted DEM: the profiles were fitted along each centre line, so the bases b_i themselves disagree across the ring, by
    up to F_paved 0.70 (node 561158).
- Alternatives considered:
  - Alt A: compactly supported Shepard weights with a minimum transition length. Pro: a direct handle on the weight slope.
    Con: the transition cannot be longer than the distance between two trims (centimetres at a ring corner), so the
    gradient stays Δ/L of the data; it does not touch the relief carried into the interior, which was the larger part;
    a support shorter than a trim gap breaks exactness. Rejected.
  - Alt B: a membrane on a junction-local grid with the trim lines as boundary data (Laplace, or thin plate for C1).
    Pro: Laplace has a maximum principle (no overshoot) and minimises the Dirichlet energy. Con: neither bounds the
    maximum gradient - where the boundary data jump by Δ over r it is Δ/r like any surface; C1 contact with the
    approach slopes needs the biharmonic, which overshoots; an iterative solve per junction (thousands of cells ×
    iterations) with a fixed iteration count for determinism, solved identically by every tile the junction touches.
    Several times the cost for the same data-bound result. Rejected.
  - Alt C: a least-squares plane per junction, blended into each approach over a bounded distance. Pro: a flat interior.
    Con: the crown relief still has to dissolve between the trim and the plane within the available distance (the same
    F); a plane cannot follow approaches of different grades on tilted terrain, so the blend zones carry that mismatch;
    it is PLAN 6.3 step 5, rejected in D6 for exactly that. Rejected.
  - Alt D: the Lipschitz-minimal (McShane-Whitney) extension min_j (ẑ_j + F·d_j). Pro: reaches F exactly, the best any
    surface can. Con: only Lipschitz - creases inside the junction and a slope jump on every trim line (no C1, 4.3
    property 2), which a car feels. Rejected.
  - Alt E (chosen): keep the product weights (exact, C1, convex, + − * / and det::sqrt only, no allocation) and blend
    v_i = b_i + φ_i (p_i − b_i) + ψ_i (ẑ_i − p_i) instead of ẑ_i (4.2): the paved relief fades over
    l_i = clamp(2·relief_i / 0.010, 3 m, 15 m) (the fade itself is never steeper than 1 %), the verge term over 1.5 m.
    Pro: on the trim line v_i = ẑ_i with zero slope of φ and ψ, so exactness, C1 and the trim-line mismatch are
    unchanged; inside, the weights mix the bases, which agree at the node on P1 profiles; the cost is the same; the
    surface stays a convex combination of approach heights (ψ ≤ φ). Con: within ~2 m of the trims little changes (that
    zone is set by F and C1); the product weights' factor 2 over F remains; on tilted terrain the bases' own
    disagreement remains.
- Verge fade: a version without the verge split (one fade for the whole cross-section) left the flat numbers unchanged,
  because the verge drop sits inside the relief and dominates the near interior; the split is what moved them. Fade
  parameter variants (slope 5-20 per mille, max 10-20 m, verge 0.75-1.5 m) changed the binned maxima by little.
- What the test asserts and why not B everywhere: 4.3 property 6.
- Results (busy tiles, 0.25 m lattice; "drivable" = road bed and paved rings, "interior" = drivable ≥ 2 m from every trim):

  | Fixture, DEM | junctions | drivable max D22-D24 → D42 | interior max | interior over B + 0.02 | drivable over B + 0.02 |
  |---|---|---|---|---|---|
  | A frankfurt_hoechst, flat | 38 | 0.222 → 0.180 | 0.185 → 0.068 | 30 → 2 | 38 → 16 |
  | B hoechst_a66, flat | 33 | 1.102 → 1.102 | 0.199 → 0.047 | 21 → 1 | 33 → 25 |
  | A, tilted | 11 | 0.536 → 0.501 | 0.236 → 0.236 | 3 → 2 | 8 → 7 |
  | B, tilted | 9 | 0.855 → 0.855 | 0.166 → 0.166 | 4 → 3 | 8 → 9 |

  By distance to the nearest trim, all core points, flat: 1-2 m 0.242 → 0.185, 2-4 m 0.232 → 0.070, > 4 m 0.202 → 0.038
  (7100 → 0 of 161011 points over B + 0.02). Tilted: 2-4 m 0.299 → 0.250, > 4 m 0.218 → 0.153. With the outer ring every
  junction still exceeds B + 0.02 somewhere (the verge corners); that ring is reported, not asserted. The flat interiors
  still over B + 0.02 have F_paved 0.029-0.064 (633794, an 8-approach ring, 0.068; 1456043917, 0.065; one A66 gore). B's
  tilted drivable count rises by one junction: node 507103 (two approaches, 17-20 m trims), road-bed maximum 0.062 → 0.068
  against B + 0.02 = 0.068, in the near zone where the large crown relief of its two approaches (0.20 and 0.29 m) mixes.
- Junctions where B is impossible for any surface (F_paved > B + 0.02), printed by the test: 30 flat (F_paved 0.045-2.95;
  above 0.1 at 507147 0.157, 561158 0.126, 10590111146 0.722, 507507 2.95) and 8 tilted (F_paved 0.137-0.702: 561158,
  256327875, 60682035, 60682037, 6977164078, 2650976157, 32241802, 32243447). A further 40 flat / 10 tilted are possible
  in principle (F_paved ≤ B + 0.02) but not for a C1 blend with factor 2. Only a different ring (trims further apart, the
  crossing trims of 507507 repaired) or profiles that follow the DEM's cross slope can move those.
- Cost (release, Windows, idle machine, both builds measured back to back): per L0 tile mean 115-175 ms on 1 worker and
  42-64 ms on 10 workers with D42 against 131-178 / 46-64 ms for the D22-D24 build - no measurable change (the slice 9
  figures of 201-445 / 97-172 ms were taken under other load).
- D41's ownership switches, re-measured with D42 (worst owner switches diag): blend↔blend 3.44 m, core↔core 2.20 m, ring
  overlap 0.29 m, trim ≤ 5.04 mm - unchanged; the junction surface is not involved in any of them. What fixing them takes:
  blend↔blend - replace the max-weight owner rule of 6.3 by a weighted mix of every overlapping blend (Σ w_k z_k / Σ w_k
  over the bucket's candidates in a fixed order; the loop already visits them all); core↔core - a soft transition between
  overlapping cores of roads at different heights, which in practice means the merge/diverge gore becomes a junction (the
  cluster review); ring overlap - clip a junction ring at its own approach's trim line in the ring builder (a roads.geom
  change: g3j/g3xs re-recorded). Each is a 6.3 or ring-construction design change of its own.
- Data: `junction_fade` block in `carve_default.json` (SHA-256 `db0a26a3…dc4020`), `JunctionFadeParams`,
  `CarveOptions::junction_relief_fade` (false = D22-D24, the sabotage). The config text gains `junction=D42`, so the
  carved layer's config hash and every release id that includes the carved layer change. g3d re-recorded: the heights of
  all four tiles change, the owner counters do not (`all 346681 396d4efad6d22338`). g3j and g3xs do not change (roads.geom
  carries no junction heights).

**D43. Ground completion eases contradicting end grades toward the chord.**
- Diagnosis (owner report, racing_game rhein_main_hessen, UTM32 E 469992 N 5552460, "no street under the bridge, terrain
  where the road should be"): the A 648 eastbound carriageway under the Westerbachstrasse bridge is a 27.6 m motorway way
  (608415202, tag split at the bridge edges). DGM1 interpolates under the deck and catches up ~0.5 m in the last 8 m at the
  deck's south edge, so the 3 m junction plane at the end node reads -5.36 % against a chord of -2.43 %. With both end
  heights and grades fixed and R 500, the minimum descent the fit can make is 0.81 m, the measured one 0.67 m: strict and
  step 3 both decline "endpoint curvature infeasible", the way gets no profile, no ribbon and no carve, and the game's
  round end cap of the bridge approach (raised to deck height) spreads over the uncarved carriageway.
- Decision: a step-3 fit that fails only on its fixed end values ("endpoint curvature infeasible" or "fixed boundary
  constraints infeasible") is retried with both end grades moved toward the chord, g_i + t (chord - g_i), t = k/8 for
  k = 1..8, the first success wins; only while |chord| <= the relaxed max grade and |g_i - chord| <= 0.05 at both ends
  (`kGroundEaseMismatch`); any other decline stops the ladder. End heights stay the plane heights. Counted in
  `RoadCompletionStats::ground_grades_eased`; provenance stays GroundCompleted.
- The mismatch cap. Uncapped, the ladder eased 36 / 156 ground stretches of the tilted bumpy fixture DEM (frankfurt_hoechst
  / hoechst_a66), 402 of 423 distinct ones with an end plane steeper than the +-0.10 clamp and most at k = 7-8: terrain
  that really departs from the chord, which the road then leaves. G3-D failed: 138 junctions instead of 91, max core
  gradient 3.079 (frankfurt_hoechst 1856/21718 tilted, node 2648065689: an eased flat approach whose trim corner sits 22 cm
  from a strict approach climbing at 10 %, 1.5 m higher), 1:1.5 blend stations 549 of 37661 (1.46 %, bound 1 %). Max
  core gradient over the four tilted busy tiles vs cap: 0.03 / 0.05 0.919, 0.06 / 0.08 1.045, 0.10 3.079. 0.05 is 2.4x
  the deck-edge misread that motivated D43 (0.021) and half the cap where the failure returns.
- Alternatives. (a) Clamp the end grades harder (e.g. +-4 %): loses real end grades on every steep stretch, and still fails
  where the plane contradicts the rise at a lower grade. (b) Free the end grades (fit them): a grade kink at the node for
  every completed stretch, not only for the ones that need it. (c) Widen the junction plane (5-10 m) at deck-edge nodes:
  a strict-pass change that moves every profile at such nodes and served bytes far beyond the declined stretches.
  (d) Fix it in the game only: the server path (G3-D) would keep the decline. The ladder changes only stretches that
  declined and keeps the smallest kink that fits.
- Effect: the strict pass is unchanged, so g3xs and g3j (flat fixture DEM: no stretch reaches the ladder) do not change.
  On the tilted bumpy DEM (tier counters, an entry once per tile that holds it) step 3 fails 24 instead of 49 ground
  candidates (frankfurt_hoechst) and 77 instead of 202 (hoechst_a66); 25 / 125 were eased. G3-D: 96 junctions (was 91),
  max core gradient 1.102 (unchanged, a flat-DEM junction), steep blend stations 74 of 30325 (was 74 of 29963). g3d is
  re-recorded (`all 347863 458d2268bdc0ebf2`, carved cells 346681 -> 347863). The config text gains
  ` ground-ease=8/0.05`, so the roads.geom config hash and every release id containing roads.geom move.
- Tests: `test_road_completion.cpp` "[D43]" (a 27.6 m motorway between two long ones on a DEM with a 12 cm tanh step at the
  end node: strict and the frozen game reference decline, the tier completes it eased, end heights are the plane heights,
  t = k/8 is the first step that fits; fails on the pre-D43 code at the missing profile), and the same scene with a 0.34 m
  catch-up (end plane -0.089 against a chord of -0.031) stays declined with nothing eased (fails without the cap). G3-D
  (`test_terrain_carve.cpp`) fails without the cap.
- racing_game still runs its own `complete_road_profiles` on the served geometry (road_structures.cpp); the fixed
  rhein_main_hessen store needs a re-derive with this build before the game sees the profile.

**D44. Block corners get circular kerb returns; a fillet that breaks the rings is left sharp, corner by corner.**
- Problem: offset-intersect-trim (3.3) meets two approach edges at a sharp point. Real crossings (RASt 06, 6.3.3; every
  aerial image of the fixtures' streets) round the kerb with a radius that grows with the road class; the sharp rings
  read as a cut-out of two overlapping ribbons, and the trims stop at the corner, so the paved area of a turn is missing.
- Rule data (`de.json` `junction`): `corner_radius_mm` per highway class (default 6 m, residential 6, tertiary 8,
  secondary 10, primary/trunk/links 12, service/track/living_street 4, motorway 0 = sharp), `corner_min_wedge_deg` 20,
  `corner_tangent_max_percent` 150, a one-way pair (dual carriageway split) under `corner_oneway_pair_max_wedge_deg` 60
  takes `corner_oneway_pair_radius_mm` 1 m. Values [assumed/unverified] from RASt 06 ranges.
- Geometry (`fillet_corner`, stub-stub corners only; no fillet at cap corners, roundabout-roundabout pairs, wedges
  under the minimum, or junctions with an excluded approach): road bed radius R0 = min of the two classes' radii;
  outer layers concentric, R_L = max(R_{L-1} - min(d_a, d_b), 0) with d the layer offsets (any radius in
  [R_{L-1} - d, R_{L-1}] nests for straight edges), radii in (0, 0.25 m) raised to 0.25 m. One scale over all layers
  keeps the tangent t = R / tan(wedge/2) within 150 % of R and before each approach's trim clamp minus the margin. The
  arc is the rational quadratic Bezier with the corner as control point and weight sin(wedge/2) (an exact circle on
  straight edges, inside the tangent triangle on curved ones), one segment per 7.5 degrees; the trims extend to the
  tangent stations + margin, so the whole return lies in the ring.
- Fallback: rings with fillets that are not simple, not nested within 2 mm or over the vertex budget are rebuilt with
  every corner sharp; if that is good, corners are re-admitted one at a time in walk order at full, then half radius,
  each kept only if the rings stay good (deterministic, at most 2m + 1 builds). Rings that are bad even sharp go to the
  D34 hull as before. `JunctionBuildStats::fillets / filleted_junctions / fillet_fallbacks` (corners left sharp);
  `JunctionBuildOptions::corner_fillets` = false is the pre-D44 build (sabotage and the sharp-geometry tests).
- Measured (fixture census, every tile's build): frankfurt_hoechst 594 of 658 ring records filleted, 1381 corners
  rounded, 5 left sharp, hulls 36 (sharp build 38); hoechst_a66 370 of 476, 867 rounded, 4 sharp, hulls 47 (49). Longer
  trims link more neighbouring nodes into clusters (frankfurt 34 -> 98 cluster records), within the D19 caps. Random
  star sweep (60 scenes, 3-6 arms, curved arms, one-ways, sidewalks): 42 filleted, 0 fallbacks, never a hull the sharp
  build avoided. Tests `tests/layer/test_junction_fillets.cpp` ([fillet]); diag dump `[.][diag][junction_dump]` +
  `tools/junction_view/render.py`. The config text gains ` corner-fillet=1`; with the de.json change the roads.geom
  config hash and every release id containing roads.geom move; g3xs, g3j, g3d re-recorded (Linux; Windows to confirm).

**D45 (withdrawn). Keep the convex hull for a broken single-node ring; do not replace it with the union of the strips.**
- Tried: the boundary of the union of the approach strips and the node core (polar upper envelope about the node), so
  the hull would not pave the wedge between diverging approaches.
- Measured on every single-node hull of both fixtures and on synthetic 3-arm wedges (motorway/primary/residential
  one-ways at 3-20 degrees): the hull paves at most 38.8 m^2 outside the union (fixtures; under 8 m^2 synthetic),
  because the trims of a broken ring are short. The union instead shows steps where strips of different widths meet,
  and the before/after renders looked worse with it. It also moved the G3-D T12 trim-line bound from 10 to 15 mm.
- Decision: the hull stays; revisit only if real data shows a hull paving a large wedge (the measurement: hull area
  outside the union on a 0.5 m grid).

**D46. A road that changes width at a junction tapers 1:5 instead of stepping.**
- Problem: two stubs that continue each other (edges within 30 degrees of opposite) with different road bed widths meet
  on a cap corner (D17). The ring steps straight across the node by the width difference, so the narrower road's edge
  starts behind the wider one's end: a sudden widening and, seen from the narrow side, a notch (the B 8 where the ramp to
  the A 66 splits off, owner report on PR #1).
- Geometry (`taper_corner`, tried after `fillet_corner` on every corner the fillet pass admits, so the D44 fallback and
  re-admission apply unchanged): the wide stub's edge end (station 0) is joined by a straight line to the narrow stub's
  edge at 5 x the step (at least 1 m, the fillet length factor applied, before the narrow trim clamp minus the margin),
  on every layer whose corner lies on a cap. The taper length comes from the road bed step and is shared by the layers.
  The trims extend past the taper end like a kerb return's tangent points. Ratio [assumed]; RASt 06 tapers are longer on
  fast roads, but the trim clamp bounds it anyway.
- `JunctionBuildOptions::width_tapers` = false is the pre-D46 build. Config text gains ` width-taper=5`; roads.geom
  config hash and release ids move; g3xs, g3j, g3d re-recorded (Linux). Test `[taper]` in
  `tests/layer/test_junction_fillets.cpp`.

### Decisions made for terrain migration phase 4 (completion parity)

**D47. Tunnel floors and the game's accepted-bridge anchors are served (roads.geom encoding 8).**
- Problem (terrain migration plan O2, O3): the game still completed what the served tier left declined. Every tunnel
  (the tier skipped them, `tunnel_skipped`) and bridges whose end node only the game's anchor map, filled from the
  tile's accepted bridges in entry order, could support.
- Tunnel floors (step 2a of `complete_road_profiles`, provenance `TunnelFloor` = 4): the game's tunnel step ported as
  is. `fit_stretch` with curvature exceptions over the context, skip under 0.5 m, `max(1, ceil(L / spacing))` intervals
  (decline over 2047), `RoadDemSampler::sample` at every station (decline when one is missing), `fit_vertical_profile`
  at `tunnel_max_grade` (0.30) / `tunnel_min_radius_m` (20 m) / 16384 iterations with the first and last interval
  secants, clamped, as end grades. Byte-identical to racing_game ef4ce25 on the test scenes (no libm in that step).
  A function of the entry's own way and the DEM along it: tile-invariant. Owner question Q1 (deep tunnels) is open: a
  tunnel at least `tunnel_chord_min_length_m` long takes the straight chord between its two portal samples (a constant
  grade along the curved reference; skipped when the chord exceeds the grade bound) instead. The served value is 0
  (never), the game's rule; curved and sloped tunnels work in both modes (test `[tunnel]`).
- A tunnel profile is never carved (`terrain_carve` skips tunnel entries), never paints grip (`envelope_road_segments`
  skips it) and stays a `Tunnel` junction approach. The validator certifies TunnelFloor profiles at the outer option
  bounds (30 %, 20 m), accepts a tunnel profile only as TunnelFloor and a Tunnel approach with one.
- Encoding 8: encoding 7's decoder rejects provenance 4, so the encoding moves (the byte layout is 7's). 7 still decodes
  and is written on request when no tunnel floor is in the tile.
- Accepted-bridge anchors (`bridge_anchor_reuse`): at a bridging end node the end plane of an accepted bridge stretch
  ending there (height, end grade along its end heading: what the game's map holds for a served bridge) takes
  precedence over the base anchor (D30 strict anchor, else the walk). To stay tile-invariant (D25) the accepted
  outcome of each incident bridge stretch is recomputed from the node: strict, else the retry, else the step-2 span
  between the base anchors of its two end nodes; one level (an accepted plane never feeds another), the last in
  (way id, start ref) order wins as in the game's entry-ordered map, the asking stretch excluded. The incident bridges
  are in the context with their DEM and node graph tiles (D35). The gain is mostly the grade: a ground stretch beside
  a span that fell back to the secant grade now continues it instead of kinking by the approach plane's grade.
- Ground stretches (step 3) at a bridging node where no incident bridge stretch is accepted end on the DEM junction
  plane there, as in the game (its map holds accepted bridges only), not on the walk plane of an unprofiled structure.
  Where these planes leave no cubic (solver or endpoint-curvature failure, after the D43 ease), the base anchors of the
  previous release are tried (`ground_base_fallback`), so the tier never declines a ground stretch the previous
  release completed. roadcmp on both OSM fixtures: entries the game's live code profiles that the new tier leaves
  declined = 0 (was 6 / 25 with the planes alone, 3 / 26 with the DEM-plane rule but no fallback).
- Walk over every incident bridge edge: `Walk::anchor` walks away from each incident bridge edge in order until one
  finds support (D25 tried the first only). Two bridge ways meeting at a node, the first a short one: D25 crossed the
  long one and ran out of its 60 m budget, the game supported both ends (its own walks and its map). Still a pure
  function of the node.
- Config text gains ` tunnel-floor=5/0.29999999999999999/20/chord:0 bridge-anchor-reuse=1` and `enc=8`; roads.geom
  release ids move (and with them every terrain.carved id). Tests: `[D47]` in `tests/layer/test_road_completion.cpp`
  against `game_reference::complete_road_profiles_live` (the game's code, verbatim); goldens g3xs, g3j, g3d re-recorded.

**D48. The abutment walk is bounded at 200 m; stage 2 lists the walk's squares (terrain migration phase 5).**
- Problem (terrain migration plan O4, GAP 2b): the game's `abutment` walks through bridges with no total cap; D25/D29
  bounded it at 60 m (61.5 m max) to keep the reads inside the 65 m margin of the entry's own way. Corridor bridges
  needing more than 60 m: 39 of 63. Owner answer Q2: 200 m (48 of 63), no chain fit; the rest keep the relaxed secant
  or stay declined.
- `RoadCompletionOptions::abutment_path_m` served 200, `kRoadCompletionMaxAbutmentPathM` 201.5. The bound is no longer
  the entry's own way margin: every walk starts at a node of a bridge way incident to a node of the tile's ways (an
  entry end node, or the far end of an accepted incident bridge, D47), so `road_bridge_dependencies` (stage 2, D35) adds
  around EVERY node of those bridge ways the elev.base and roads.graph tiles of a square of half-width
  `kRoadBridgeWalkMarginMm` = 205 m (201.5 + 3 m footprint + 0.5 m bilinear reach; a way incident to a node in the square
  lies in that node's roads.graph tile). The target tile itself is never listed. No third discovery stage: the start
  nodes are known after stage 1. Config text ` bridge-walk-margin=205`, `completion-abutment=32/200`; release ids move.
- Tests `[D48]`: the fence (paths 201.4 / 201.9 m at the bound give the same outcome with the tile's inputs and with a
  hole-free DEM; a 203 m budget reads past the squares; the pre-D48 stage 2 misses a 199 m walk's support), walks of
  58 / 190 / 480 m at 60 and 200 m, and the fixture cost (frankfurt_hoechst DEM inputs 303 -> 306, hoechst_a66 458 -> 467,
  context graph tiles 105 -> 115; build time unchanged within noise; no fixture outcome or byte changes). The D47 bridge
  chain now matches the game's node height (99.941 m both; 99.211 m at 60 m).
- GAP 3 (retry way 285188051 on utm32n/2/464/5394, ground ways 25589596 and 320190187): an input diff needs the real
  stores; not reproduced on the fixtures, open until the probe runs.

**D49. terrain.class inputs: the land cover tiles of the closed square, a 4 x 4 window above level 0; classes imply
carved, not the reverse (terrain migration phase 6).**
- A class tile is node-registered with both edges (257 x 257), so its east and north edge nodes lie on the neighbour's
  first column / row. Level 0 therefore reads every `g2m.src.landcover` level-2 tile holding a node of the CLOSED square
  (`terrain_class_landcover_keys`: the ancestor, plus its east, north and north-east neighbours when the tile touches the
  ancestor's east or north edge), not only the ancestor; with the ancestor only, 220 of the 257 edge nodes on a level-2
  seam differ from the neighbour tile (test control).
- Level L >= 1: the 3 x 3 rank-max window around an edge node of the parent reaches one level L-1 node beyond the 2 x 2
  children, so the deriver reads the 4 x 4 same-layer tiles at (2x + dx, 2y + dy), dx, dy in -1..2. With only the
  children the L1 edge nodes would not equal the neighbour's.
- Wiring: naming the class layer adds the carved layer (one ownership); naming the carved layer does NOT add the class
  layer, so a carved-only release keeps its slice-9 identity (6.1's "naming either adds both" is not applied to carved).
  `TerrainService` accepts `LayerOptIn{classes}` (implies carved); its height layer still follows `carved`.

**D50. A rejected roads.geom input counts as absent for the carved and class layers (request G-1).**
- Problem: a roads.geom tile can fail its own validation (utm32n/2/452/5428, "reference endpoints"); the server resolver
  failed every tile depending on it, so a carved / class tile near it would never be served and the game's gate would
  freeze (no fallback after the switch).
- `IDeriver::rejected_input_is_absent(ref)` (default false): when an input resolves to an error and the deriver answers
  true, the server passes it as absent (`InputTile::rejected`) and records `rejected_input_marker(ref)` = SHA-256 of
  `g2m.rejected-input/1 <layer> <key>` in the output header's inputs, so the output says which input was dropped
  (`header_has_rejected_input`). `TerrainCarvedDeriver` and `TerrainClassDeriver` answer true for `g2m.roads.geom`
  only; a failing `elev.base` still fails the tile. No release id change: the config texts are unchanged and only a tile
  that failed before is now served.
- Cost: errors are not cached, so every dependent tile re-derives the failing roads.geom tile once per request.
- Test `[G-1]`: a release whose roads.geom deriver always fails gives a Found carved L0 (flat DEM, 10 inputs, marker in
  the header), L1 and class tile (no roads, rank 0 everywhere); a carved deriver that does not opt in fails (control).

**D51. Physics class mode and the G-6 census (slices 11 and 12 as far as carve readiness goes, terrain migration phase 6).**
- `g2m_phys`: `FetchResult::classes`, `IHeightTileFetch::provides_classes`, `LoaderConfig::require_classes` (a 200
  without the class tile retried, then Failed, like `require_roads`), `ResidentHeightSet` keeps the class tile beside the
  heights and the RoadSegment metadata (request G-2: class mode keeps the speed-limit / road_ahead data the road mode
  carries), `fill_physics_classes` (cell centre = class node, full `SurfaceIdTable`, request G-3), and
  `TransportHeightTileFetch(transport, rid, height_layer, class_layer)`. Bridge: `ClassSurfaceConfig{table, fallback}`
  and the Snapshot prefetch requiring the class tile and comparing its identity (request G-5).
- G-6: `road_underpass_census` (`road_underpass.h`) lists the ground stretches crossing a lower road (lower OSM layer or
  tunnel, not a bridge, no shared node) and measures the dip = chord over +-60 m minus the profile, threshold 0.3 m;
  `g2m_road_probe ... --underpasses` prints it per tile. Fixtures (synthetic DEMs, no cuttings): hoechst_a66 2 tunnel
  crossings, max dip 0.000 m flat / 0.099 m tilted; frankfurt_hoechst none. A synthetic 6 m deep, 16 m wide cutting under
  a secondary road gives a 2.0 m dip on the served profile, so the B8 case is reproduced in kind. The home-store run
  (B8 at s ~ 2852 / 4453 / 5261) and the DEM-masking decision wait for the store.
- Not done here: the render `ClassWindow` from terrain.class (slice 11), the junction / carve census of section 12 on the
  home tiles.

---


## 16. What P2 changes in P1's implementation

1. **roads.geom encoding 6 → 7** (sections 5-7). Encoding 6 and 5 still decode.
   `kRoadGeomEncoding` becomes 7.
2. **Config hash and release id:**
   - The text gains `enc=7 junction=1 furniture=1 completion=1`, and `rules=` changes
     with the de.json blocks.
   - Every release that includes roads.geom gets a new id. `test_road_geom_xs.cpp`'s
     config and rid expectations change.
3. **Goldens:**
   - `g3xs` is re-recorded: encoding-7 bodies, and section-2 content changes wherever
     the completion tier adds profiles.
   - The roadgeom fixture body SHA `17d9b7c7…5ee2f9` changes. The encoding-6 body stays
     reproducible from an encoding-6 encode, as P1 kept the encoding-5 one.
   - New goldens: `g3j` and `g3d`.
4. **de.json:** new `junction` and `furniture` blocks. The rules SHA pin test is
   updated.
5. **Section 2 content:** completed profiles, and some strict profiles at
   bridge-anchored nodes re-fitted (6.6). Sections 3 and 4 follow, because
   cross-sections and features exist only for profiled entries.
6. **Profile features:** stretch end nodes stay skipped in P1's
   `resolve_profile_features`. Junction-node vertical features become raised
   junctions (4.4). Approach features inside a ring stop being carved and are counted
   instead.
7. **`envelope_road_segments`** stays as it is for the transition, but physics grip
   moves to `terrain.class` (10.1). It is not trimmed at junctions; once racing_game
   switches, its only remaining use is as the reference in `test_road_api.cpp`.
8. **No change** to `roads.graph`, `carve_footprint`, `driving_lanes_at`,
   `resolve_cross_section` or the P1 C-API sketch.

---

## 17. Owner questions

Everything technical is decided above. Only these need the owner.

**Q1 — switch gate.** racing_game drops its own carving and profile completion with no
fallback. Should the switch happen only when the home region has **zero** ground
stretches left uncarved after the server's completion tier? Or do you accept a
measured list of remaining uncarved stretches, which then show as bare terrain
between carved roads? **Recommendation: zero on every road the game can reach on the
home map. The census (section 12) lists them, and each is fixed or explicitly
accepted before the switch.**

> **Answer (owner, 2026-10-05):** "There is no prod environment; work in progress
> states are acceptable." The gate is therefore not zero gaps. The remaining uncarved
> stretches are listed and do not block the switch. 10.4, the risk table (13) and
> section 14 are updated.

**Q2 — relaxed profiles as served data.** The game today invents profiles where the
strict fit fails. For bridges it re-fits spans; for ground roads it uses relaxed
limits (grade bound raised to at least 12 %, end grades clamped to ±10 %, 40 m
samples). May the server serve these as ordinary drivable profiles, flagged as
"completed" so tools can list them? **Recommendation: yes. It is what you drive today.
Moving it to the server only makes it shared and visible, and the flag keeps it
honest.**

> **Answer (owner, 2026-10-05):** yes. Serve the relaxed profiles, flagged
> "completed" (D14, 6.6).
