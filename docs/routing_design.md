# G4 routing design

Milestone G4 (racing_game `PLAN.md` section 12): A* plus a hierarchy, turn
restrictions, guidance data and client map geometry. Consumer: racing_game R4
(minimap, full map, turn-by-turn through 10 manoeuvres on a scripted drive),
later NPC traffic. Acceptance: routes on the fixture equal a brute-force
Dijkstra; restrictions honoured in unit cases; route requests under 10 ms
within 30 km [target].

Status: slices 1 and 2 are built (`include/g2m/route/`, `src/route/`,
`tests/route/`). Everything else here is a plan.

## Owner questions

Questions 2 to 6 were settled on 2026-10-14 with the recommended defaults
(coordinator, PR #2): tracks are routable at 15 km/h, start and destination
snap with the car's heading, the region graph is built on a worker at world
load, ALT landmarks are computed in the background at load, and time-dependent
restrictions are ignored. The original wording stays below for the record;
question 1 is still open.

1. **Turn costs.** Defaults: right turn 2 s, left 4 s, sharp right 4 s, sharp
   left 6 s, slight 0.5 s, U-turn 20 s and only at a dead end. Counted only at
   nodes with three or more edge ends. Fine, or should the route avoid lefts harder?
2. **Access.** Blocked: `no`, `private`, `agricultural`, `forestry`,
   `emergency`, `use_sidepath`. Allowed: `destination`, `delivery`,
   `customers`. Service roads are routable at 15 km/h, tracks are not routable.
   OK for a racing game, or should tracks be drivable too? **Settled: tracks
   routable at 15 km/h.**
3. **Start and destination.** Proposal: snap both to the nearest routable edge
   position. The start takes the car's heading, so the first arc is the one the
   car is driving on. A U-turn can then only happen at a dead end. Is that the
   behaviour you want, or should the route turn the car round on the spot?
4. **Routing scope.** Proposal: one routing graph for the whole loaded region,
   built in the background at world load. The estimate is about 76k nodes and
   about 160k arcs for "home", roughly 10-20 MB. No per-tile streaming for
   routing. Routes across region borders are out of scope until G8.
5. **Time-dependent restrictions.** `restriction:conditional` and conditional
   maxspeed values are ignored, because the game has no clock policy for them.
   OK?
6. **Speed-up method.** The 10 ms target needs a speed-up (measured below).
   Proposal: ALT landmarks (exact) computed at world load, about 1 s in the
   background for the region. The alternative is landmarks served per release
   as a layer. Is a background second at load acceptable?

## 1. Graph source

- **Input:** `g2m.roads.graph` tiles (level 2, 1 km, 128 m halo). Ways are
  complete in every tile they touch, and node ids define connectivity.
  `merge_road_graphs` unions the tiles of a region into one canonical
  `RoadGraph`, with world node degrees. The fixture tests build the same graph
  directly from a PBF with `build_road_graph`.
- **Routable graph:** `build_routing_graph(RoadGraph, restrictions, RouteConfig)`
  (slice 1):
  - **Nodes** are the decision points: way ends, nodes at two or more way
    positions, and restriction via nodes. Shape nodes are collapsed.
  - **Edges** keep their identity as (way id, first and last OSM ref-pair
    index), the same `segment_index` as `RoadGraphEdge`. So guidance and map
    code can always go back to the way geometry and tags.
  - **Arcs:** each traversable direction of an edge is one arc. Arc order is
    canonical: (from node, way id, first segment, forward first).
- **Ways that are left out:**
  - classes not in the config table, or marked not routable (every
    roads.graph class is routable by default; tracks at 15 km/h);
  - access closed: the most specific of `motorcar` > `motor_vehicle` >
    `vehicle` > `access` decides;
  - `oneway=reversible|alternating`.
- **Oneway:**
  - `yes`, `true`, `1` = forward only; `-1`, `reverse` = backward only;
    `no`, `false`, `0` = both directions;
  - implied by `junction=roundabout|circular` and by the config list
    `implied_oneway` (default `motorway`).

## 2. Cost model

- **Cost unit:** integer milliseconds. Arc cost = ceil(length_mm * 3.6 /
  speed_kmh).
- **Length:** the sum over segments of ceil(det::sqrt(dx^2 + dy^2)), in integer
  mm.
- **Speed:**
  - `maxspeed:forward` or `maxspeed:backward`, else `maxspeed`, parsed by the
    existing `parse_road_speed_limit` (units km/h, mph, knots);
  - symbolic values go through the `zone_speeds` table (`DE:urban` 50,
    `DE:rural` 100, `DE:zone30` 30, `walk` 7, ...);
  - `none` and unknown values fall back to the class default;
  - the result is clamped to [5, 130] km/h.
- **Class defaults (km/h, config):**

  | Class | km/h | Class | km/h | Class | km/h |
  |---|---:|---|---:|---|---:|
  | motorway | 120 | motorway_link | 60 | trunk | 100 |
  | trunk_link | 50 | primary | 70 | primary_link | 40 |
  | secondary | 60 | secondary_link | 40 | tertiary | 50 |
  | tertiary_link | 30 | unclassified | 40 | residential | 30 |
  | living_street | 7 | service | 15 | track | 15 |

- **Turn cost:** classified by the heading change between the arriving and the
  leaving segment.
  - The angle is det::atan2 of the cross and dot products of integer mm
    vectors; positive means left.
  - Thresholds (config): straight up to 20 deg, slight up to 60, turn up to
    120, sharp beyond.
  - Charged only at nodes with degree >= 3, so a bend where two ways join costs
    nothing.
- **Heuristic:** floor(straight-line mm) * 3.6 / 130 km/h. It is admissible, and
  the search reopens states, so it stays optimal even where rounding makes the
  heuristic inconsistent by 1 ms.
- **Determinism:**
  - integer costs, and canonical arc order;
  - the open list is ordered by (f, g, arc index), and a parent changes only
    on a strictly smaller g;
  - only det::sqrt, det::atan2 and the IEEE-exact std::ceil / std::floor /
    std::fabs / std::llround are used, so the libm grep stays clean.

## 3. Turn restrictions

- **The import keeps them.**
  - `g2m.src.osm` keeps every relation with a kept member: `extract_region_set`
    pass R and `clip_osm` step 4, closed transitively.
  - The fixtures prove it: frankfurt_hoechst has 24 `type=restriction`
    relations, hoechst_a66 has 17.
- **What is missing:** `g2m.roads.graph` drops relations, because `RoadGraph`
  has no relation field. A consumer that only has roads.graph tiles has no
  restrictions today. Slice 1 reads them from canonical OSM:
  `extract_turn_restrictions(osm::OsmData)`.
- **Import change (slice 3):**
  - roads.graph encoding 6 adds a restriction section: `TurnRestriction`
    records (relation id, kind, value, from way, via node or via ways, to
    way).
  - Each record is stored in every graph tile that holds its via node or a via
    way, the same "complete in every touching tile" rule the ways follow.
    `merge_road_graphs` dedups the records by relation id.
  - The deriver already receives the src.osm tile, which holds these
    relations.
  - This is an encoding bump. The roads.graph config hash changes, and so do
    the release ids of roads.graph and of every layer that depends on it.
- **Tag rules (`extract_turn_restrictions`):**
  - `restriction:motorcar` wins over `restriction`;
  - `except` naming motorcar, motor_vehicle or vehicle drops the relation;
  - restrictions for other vehicles only (`restriction:hgv`, ...) are skipped;
  - `restriction:conditional` is counted, not evaluated;
  - `no_entry` and `no_exit` expand to one No rule per (from, to) pair;
  - only_* rules need exactly one from way and one to way;
  - everything else is counted as malformed, never fatal.
- **Search semantics:** the search is edge-based (state = arc), so a via-node
  rule is a forbidden or forced transition at the node.
  - **No rule:** forbids leaving onto the to way after arriving on the from
    way.
  - **Only rule:** allows only arcs of the to way.
  - **from == to with a No rule (`no_u_turn` on one way):** forbids only the
    U-turn. Going straight on through an interior via node stays allowed.
  - **U-turns:** a U-turn back along the same edge is offered only when no
    other transition is allowed (a dead end), and a No rule can still forbid
    it.
- **Via-way rules (slice 3):** parsed and counted (`restrictions_via_way`),
  not applied yet. The plan is the standard state expansion:
  - the search state becomes (arc, progress), where progress is the number of
    matched elements of one via-way rule;
  - entering the from way and then the first via way advances progress; any
    other arc resets it;
  - the forbidden (or forced) final transition is checked at the last via
    node;
  - via-way rules are rare (0 in both fixtures), so the extra states cost
    nothing measurable.

## 4. Hierarchy and speed-up

**Graph size.**
- Region "home" (`docs/osm_tag_coverage_home.md`): 58,672 drivable ways,
  5,795 km, about 950 km^2.
- The fixtures give a ratio of about 1.3 routing nodes and 2.7 arcs per used
  way: frankfurt_hoechst has 627 ways, 814 nodes and 1,715 arcs (tracks
  routable; 575 / 751 / 1,561 without them).
- That predicts about 76k nodes and about 160k arcs for home [estimate].

**Measured on a stand-in.** The stand-in graph is a 301 x 301 street grid with
200 m blocks. It covers 60 x 60 km, with 90,601 nodes and 361,200 arcs, so it
is denser in arcs than the home estimate. Arterials run every 10th line, and
turn costs apply at every junction. The machine is a Linux clang release build
on a shared 4-core box with load 14-17, so the times are noisy.

| Straight-line distance | Plain A*: mean / max ms | A* settled | ALT (16 landmarks): mean / max ms | ALT settled | Dijkstra (std::set) ms |
|---|---|---:|---|---:|---:|
| 5 km | 0.45 / 0.74 | 2,942 | 1.45 / 31 | 303 | 5.8 |
| 15 km | 7.4 / 22.7 | 25,189 | 1.8 / 26 | 1,187 | 19.7 |
| 30 km | 29.7 / 79 | 78,601 | 4.1 / 22.5 | 2,373 | 68 |

- The ALT column is a test-only experiment (`tests/route/test_route_search.cpp`,
  `[bench]`). It allocates a 361k-entry array per query, which dominates its
  time.
- ALT settles 33 times fewer states than plain A* at 30 km. With the slice 1
  workspace (generation stamps) that is expected to be about 1 ms.
- The 16 landmarks took about 1.4 s to prepare with a std::set Dijkstra.
- ALT returned the same cost as plain A* on every query.
- Plain A* misses the 10 ms target at 30 km. This is expected, because the
  time heuristic (distance at 130 km/h) is weak on 30-50 km/h streets.

**Slice 2, library ALT (measured).** `RouteLandmarks` in `g2m_route`, used by
`RouteSearcher` through the generation-stamped workspace, so a query
allocates nothing after warm-up. Same grid and machine (load 17-19), 40
queries per row:

| Straight-line distance | Plain A*: mean / max ms | A* settled | A* + ALT: mean / max ms | ALT settled |
|---|---|---:|---|---:|
| 5 km | 0.74 / 4.7 | 2,942 | 0.07 / 0.14 | 310 |
| 15 km | 7.3 / 22 | 21,655 | 0.93 / 12.4 | 935 |
| 30 km | 22.7 / 58 | 78,464 | 1.23 / 7.1 | 1,695 |

- Preparing the 16 landmarks (32 node-graph Dijkstras with a binary heap)
  takes 0.6 s on 90k nodes and 361k arcs.
- One snap takes 0.003-0.08 ms.
- The maxima come from preemption on the loaded box: the settled counts are
  the stable measure.
- A* + ALT equals the reference Dijkstra on every test query, for both node
  queries and point queries.
- The 10 ms / 30 km target is met on the stand-in. The real home store still
  has to be measured on the owner machine.

**Recommendation: edge-based A* with ALT landmarks** (built in slice 2).
- 16 landmarks, picked by farthest-point selection from node 0, which is
  deterministic.
- Lower bounds come from node-graph Dijkstras over arc costs. Turn costs are
  >= 0 and restrictions only remove options, so the bound is admissible for
  the edge-based search.
- ALT is exact, so the acceptance check "equals a brute-force Dijkstra" holds
  unchanged.
- Memory: 16 x 2 x 76k x 4 B, about 10 MB.

**Rejected alternatives:**
- **A road-class hierarchy (Valhalla-style levels):** it is inexact, which
  breaks the Dijkstra-equality acceptance check, and it is not needed at
  region size.
- **Bidirectional A*:** it is awkward with edge-based restrictions and gives at
  most about a 2x gain.
- **Contraction hierarchies:** only if ALT misses 10 ms on an all-Hessen
  graph (G8 scale).

## 5. Guidance data (slice 4, done)

`build_guidance(graph, roads, route, GuidanceConfig) -> std::vector<Maneuver>`
(`g2m/route/route_guidance.h`), a pure function of the route arcs and the way
tags. `roads` is the RoadGraph the routing graph was built from (the tags are
looked up by way id).

**Maneuver record:**
- kind;
- `step`: the index into `Route::arcs` of the arc driven after it;
- the graph node and its position (mm; Depart and Arrive use the snapped
  start and end points);
- distance from the start and to the next manoeuvre (mm); the distances to
  the next sum to `Route::length_mm`;
- heading change (deg, rounded, positive = left);
- the road it enters: `name`, `ref`, `destination`, `destination_ref` (OSM
  `destination`, `destination:ref`);
- the exit number for roundabouts.

**Kinds:**
- depart and arrive;
- continue (straight on onto a new name or ref);
- slight, turn and sharp left or right (the RouteConfig angle classes the
  cost model uses: 20 / 60 / 120 deg);
- keep left or right: a fork, another allowed exit within
  `GuidanceConfig::fork_max_deg` (45 deg) of a straight or slight move;
- U-turn (only possible at a dead end);
- roundabout enter with the exit number (exits = ring nodes where an allowed
  exit leaves the ring); roundabout exit only when the route starts on the
  ring;
- ramp off (motorway/trunk onto a `*_link`), ramp on (another road onto a
  `*_link`), merge (a `*_link` onto motorway/trunk).

**When a manoeuvre is emitted:** at a node with degree >= 3 when any of these
holds:
- the move is a ramp, merge, U-turn or roundabout entry;
- the turn class is beyond slight;
- a slight move with another allowed exit (a slight turn), or with one
  within the fork angle (keep left/right);
- the name or ref changes (continue).

Staying on a main road past a slip road is silent: a `*_link` exit does not
make a fork unless the route changes name or ref there. Allowed exits come
from `RoutingGraph::successors`, so a restricted or one-way exit never makes
a fork. Lane hints (`turn:lanes`) come later. Instruction text and
localisation stay in the game.

**Tests** (`tests/route/test_route_guidance.cpp`, `[guidance]`): synthetic
scenes for every kind (distances, names, refs, destination, exit numbers 1-3,
silence on a same-name straight and past a slip road, U-turn on a point
route). On the hoechst_a66 fixture, 200 random node pairs (148 reachable,
1,478 manoeuvres) check the invariants: Depart first at 0, Arrive last at the
route length, monotone distances, distances to the next summing to the route
length. The longest of them (3.4 km, 21 manoeuvres, through Unterliederbach
onto the B 8, off at the Main-Taunus-Zentrum ramp) has its kind sequence
pinned. Guidance of all 148 routes takes well under 1 ms each.

## 6. Client map geometry (slice 5, done)

`build_map_tile(key, lod, level-2 roads.graph tiles, MapConfig) -> MapTile`
(`g2m/route/route_map.h`) is a client-side product with no new server layer.
A served `g2m.map.lines` layer comes only if bandwidth needs it.

**One `MapTile` per tile key per LOD** (`default_map_config()`; every number
is config):

| LOD | Tile size | Classes | Simplification | Used by |
|---|---|---|---|---|
| detail | L2, 1 km | every roads.graph class (track and up) | 1 m Douglas-Peucker | minimap at 0.5-2 km radius |
| mid | L4, 4 km | tertiary and above (links share their tier's rank) | 8 m | full map, zoomed in |
| far | L6, 16 km | primary and above | 40 m | full map, zoomed out |

**Polyline records (`MapLine`):**
- points: int32 decimetres relative to the tile's SW corner, in a shared
  per-tile point array, way order, at least two;
- the class rank;
- flags: oneway, oneway against way order, no cars (reversible), bridge,
  tunnel, link, roundabout (directions use `way_directions`, the routing
  graph's own rule);
- name and ref as indices into the tile's sorted string table;
- a label anchor: the midpoint of the longest piece, plus that piece's
  direction for label rotation;
- (way id, first and last segment index), so the route highlight can pick
  the same records. `route_polyline(graph, route)` gives the exact driven
  line in mm, cut at the snapped start and end.

**Ownership:** a map tile draws the edges owned by the level-2 roads.graph
tiles inside it (`owned_edges`: the tile containing the edge midpoint), so no
edge is drawn twice and every LOD is assembled from level-2 tiles alone.
Owned edges of one way that follow each other form one polyline, across
level-2 borders inside the tile. Output sorted by (way id, first segment);
the input tile order does not matter.

**Measured** (`g2m_route_tests "[map]" -s`, release build, loaded machine):

| fixture | LOD | tiles | lines | points in -> out | total | max per tile | build mean / max |
|---|---|---|---|---|---|---|---|
| frankfurt_hoechst | detail | 10 | 808 | 4117 -> 2679 | 67.3 KiB | 21.7 KiB | 0.50 / 4.4 ms |
| frankfurt_hoechst | mid | 3 | 135 | 702 -> 302 | 10.0 KiB | 6.7 KiB | 0.08 / 0.13 ms |
| frankfurt_hoechst | far | 2 | 43 | 288 -> 90 | 3.1 KiB | 2.0 KiB | 0.05 / 0.06 ms |
| hoechst_a66 | detail | 13 | 543 | 2672 -> 1717 | 44.3 KiB | 18.9 KiB | 0.05 / 0.24 ms |
| hoechst_a66 | mid | 3 | 169 | 1024 -> 382 | 12.5 KiB | 10.6 KiB | 0.08 / 0.21 ms |
| hoechst_a66 | far | 1 | 106 | 733 -> 217 | 7.5 KiB | 7.5 KiB | 0.17 / 0.17 ms |

Sizes are in-memory (`MapTile::byte_size`, 56-byte records, 8-byte points).
The fixtures cover only parts of each mid/far tile, so a full far tile of the
home region is larger; it is built once at load from the region's level-2
tiles. The worst point deviation stays at the tolerance plus the decimetre
rounding (1036 mm at detail, 7.9 m at mid, 39.9 m at far).

**The game (R4):**
- builds ribbon meshes per tile off-thread, with width by class, and caches
  them;
- per frame, only moves and rotates the heading-up view inside the round
  `SubViewport` mask, to stay under the 0.3 ms minimap budget;
- the full map draws the mid and far LODs as GPU vectors.

## 7. Public API

**Slice 1, built:**
- `g2m/route/turn_restriction.h`:
  - `TurnRestriction`, `RestrictionKind`, `RestrictionSet`;
  - `extract_turn_restrictions(const osm::OsmData&, max)`.
- `g2m/route/routing_graph.h`:
  - `RouteConfig` and `default_route_config()`;
  - `RoutingGraph` (`nodes()`, `arcs()`, `out_arcs()`, `find_node()`,
    `successors()`, `turn_cost_ms()`, `stats()`);
  - `build_routing_graph(...)`.
- `g2m/route/route_search.h`:
  - `RouteSearcher(graph).find(RouteQuery{from_node, to_node}) ->
    Result<Route>`, where a Route holds reachable, cost_ms, length_mm and
    arcs;
  - `RouteSearchStats`.
  - The searcher is a reusable workspace, one per thread. The graph is
    immutable and shared.
- Errors are `Result<T>` and nothing throws:
  - an unknown node is `InvalidArgument`;
  - more than `max_settled` states is `LimitExceeded`;
  - an unreachable target is not an error (`reachable = false`).

**Slice 2, built:**
- `RouteLandmarks::build(graph, count = 16)` (`route_search.h`):
  - landmarks by farthest-point selection; a node that no landmark reaches
    yet is taken first, so every connected component gets a landmark;
  - tables in [node][landmark] uint32 order;
  - `bound(v, t)`;
  - immutable, so searchers and threads can share it.
- `RouteSearcher(graph, &landmarks)`: the heuristic is the larger of the chord
  bound and the ALT bound to the target node(s).
- `RoutingGraph::snap(point, SnapOptions{max_distance_mm = 50 m, heading,
  tolerance = 60 deg}) -> RoutePoint`:
  - the snap index is a sorted cell grid (`snap_cell_mm`, 256 m);
  - with a heading, only directions within the tolerance qualify. If none
    qualifies, the nearest edge is returned in every traversable direction
    and `heading_matched` is false;
  - ties go by distance, then edge, then segment, then direction;
  - each candidate holds an arc and the mm offset along its direction of
    travel.
  - Helpers: `shape(arc)`, `point_at(arc, offset)`, and
    `partial_cost_ms(arc, a, b) = ceil(cost * (b - a) / length)`.
- `RouteSearcher::find(RoutePoint from, RoutePoint to)`:
  - travel starts on a from candidate at its offset and ends on a to
    candidate;
  - a destination ahead on the start arc is direct. One behind it goes
    around the block and re-enters the arc;
  - `Route::start_offset_mm` and `end_offset_mm` give the partial first and
    last arc.
- `g2m/route/region_graph.h`:
  - `assemble_region_graph(keys, RoadGraphTileFetch)` sorts and dedups the
    keys, treats absent tiles as allowed, and checks that each fetched tile
    has the requested key.
  - `merge_region_graphs` keeps one copy per OSM id and rejects conflicting
    copies. It sorts, so the result does not depend on the tile order, and it
    applies no per-tile limits.
  - `merge_turn_restrictions` merges the per-tile restriction sets.
  - `build_region_routing(graph, restrictions, config, landmarks = 16)`
    returns the heap-held graph and landmarks. The game runs it on a worker
    at world load.

- `g2m/route/route_guidance.h` (slice 4): `build_guidance(graph, roads,
  route, GuidanceConfig)` -> `Maneuver` list, `to_string(ManeuverKind)`.

- `g2m/route/route_map.h` (slice 5): `build_map_tile`, `MapTile`/`MapLine`,
  `default_map_config`, `simplify_polyline`, `route_polyline`.
- `g2m/route/routing_graph.h`: `way_directions` is public (shared by the map
  flags).

## 8. Slices

1. **Done.** Routable graph, edge-based A*, oneway, access, speeds, via-node
   restrictions, the dead-end U-turn rule, and a test-only Dijkstra.
   - Synthetic tests cover costs, every oneway form, no_left_turn,
     only_straight_on, the U-turn and same-way no_u_turn, unreachable targets,
     ties and limits, plus 720 random restricted grid queries (A* ==
     Dijkstra).
   - On the fixtures, 400 random pairs each agree with Dijkstra and every
     route validates. The negative control is that dropping the restrictions
     changes 4 and 36 routes respectively (3 and 35 before tracks became
     routable).
2. **Done: ALT landmarks, snapping and the region graph.**
   - ALT landmarks in the library.
   - Heading-aware snapping and mid-edge start and end.
   - Region graph assembly from roads.graph tiles.
   - Tests:
     - snapping cases: heading, fallback, one-way, a farther edge chosen by
       heading;
     - point routes: partial arcs, same arc ahead and behind;
     - ALT on random restricted grids: 400 node and point queries equal
       Dijkstra;
     - landmark determinism and admissibility;
     - on both fixtures, about 300 heading-snapped point queries each equal
       Dijkstra;
     - the hoechst_a66 fixture rebuilt from its 12 roads.graph tiles through
       the real codec, in a shuffled order with repeated keys and an absent
       tile, equals the whole-world graph. The restrictions merged from the
       per-tile `clip_osm` src.osm tiles equal the whole-file restrictions.
   - Open: timing on the real home store, on the owner machine.
3. **Restrictions in roads.graph and via-way rules.** The restriction section
   in roads.graph (encoding 6, release id change) and via-way rules through
   state expansion.
4. **Done: guidance.** The manoeuvre list (section 5), synthetic tests per
   kind and the pinned 21-manoeuvre hoechst_a66 route. Open: lane hints.
5. **Done: map geometry.** Map tiles at three LODs and the route polyline
   (section 6). Tests: Douglas-Peucker bounds on random walks, a synthetic
   scene pinning ownership across a level-2 border, LOD filters, flags,
   strings, anchors, determinism and errors; on both fixtures every LOD
   draws every edge of its classes exactly once within tolerance.
6. **Later, with traffic.** The lane graph (`turn:lanes`, lane connectivity at
   junctions), signals and stop points from the P2 furniture records.
