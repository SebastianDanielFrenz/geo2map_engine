# G4 routing design

Milestone G4 (racing_game `PLAN.md` section 12): A* plus a hierarchy, turn
restrictions, guidance data and client map geometry. Consumer: racing_game R4
(minimap, full map, turn-by-turn through 10 manoeuvres on a scripted drive),
later NPC traffic. Acceptance: routes on the fixture equal a brute-force
Dijkstra; restrictions honoured in unit cases; route requests under 10 ms
within 30 km [target].

Status: slice 1 is built (`include/g2m/route/`, `src/route/`, `tests/route/`).
Everything else here is a plan.

## Open owner questions

1. **Turn costs.** Defaults: right turn 2 s, left 4 s, sharp right 4 s, sharp
   left 6 s, slight 0.5 s, U-turn 20 s and only at a dead end. Counted only at
   nodes with three or more edge ends. Fine, or should the route avoid lefts harder?
2. **Access.** Blocked: `no`, `private`, `agricultural`, `forestry`,
   `emergency`, `use_sidepath`. Allowed: `destination`, `delivery`,
   `customers`. Service roads are routable at 15 km/h, tracks are not routable.
   OK for a racing game, or should tracks be drivable too?
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
  - classes not in the config table (tracks by default);
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
  | living_street | 7 | service | 15 | | |

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
  way: frankfurt_hoechst has 575 ways, 751 nodes and 1,561 arcs.
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

**Recommendation: edge-based A* with ALT landmarks.**
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

## 5. Guidance data (slice 4)

`build_guidance(graph, roads, route) -> std::vector<Maneuver>`, a pure
function of the route arcs and the way tags.

**Maneuver record:**
- kind;
- the node index and its position (mm);
- the arc index;
- distance from the start and to the next maneuver (mm);
- heading change (deg);
- the road it enters: `name`, `ref`, `destination`, `destination:ref` from
  the way tags;
- the exit number for roundabouts.

**Kinds:**
- depart and arrive;
- continue;
- slight, turn and sharp left or right (the same angle classes as the cost);
- keep left or right: a fork where two allowed exits lie within 45 deg of
  each other;
- U-turn;
- roundabout enter (exit n) and exit;
- ramp on and ramp off (moving onto or off a `*_link`);
- merge.

**When a maneuver is emitted:** at a node with degree >= 3 when any of these
holds:
- the turn class is not straight;
- the name or ref changes;
- another allowed exit lies within 30 deg of the chosen one, so going straight
  is ambiguous.

Consecutive arcs on the same road with no maneuver are merged. Lane hints
(`turn:lanes`) come later. Instruction text and localisation stay in the game.

**Test:** a scripted route on the hoechst_a66 fixture through 10 or more
manoeuvres. Every manoeuvre kind and distance is pinned, and the route
crosses the B 8 / A 66 links.

## 6. Client map geometry (slice 5)

`build_map_tile(roads.graph tile, lod) -> MapTile` is a client-side product
with no new server layer. A served `g2m.map.lines` layer comes only if
bandwidth needs it.

**One `MapTile` per level-2 tile per LOD:**

| LOD | Tile size | Classes | Simplification | Used by |
|---|---|---|---|---|
| detail | L2, 1 km | all routable classes plus service | 1 m Douglas-Peucker | minimap at 0.5-2 km radius |
| mid | L4, 4 km | tertiary and above | 8 m | full map, zoomed in |
| far | L6, 16 km | primary and above | 40 m | full map, zoomed out |

**Polyline records:**
- int32 decimetres relative to the tile's SW corner;
- the class rank;
- flags: oneway, bridge, tunnel, link, roundabout;
- a name/ref string index;
- a label anchor (the midpoint of the longest piece);
- every record also keeps its (way id, segment range), so the route highlight
  draws from the same records.

**Ownership:** by edge midpoint, the roads.graph `owned_edges` rule, so no line
is drawn twice.

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

**Planned:**
- `RouteLandmarks build_landmarks(graph, count)`, handed to `RouteSearcher` as
  an option (slice 2).
- `RoutePoint snap_to_graph(graph, PointMm, optional heading)`, giving an arc
  and an offset in mm.
- `find(RouteRequest{RoutePoint from, RoutePoint to})`, which charges the
  partial first and last arc (slice 2).
- `build_guidance(...)` (slice 4), `build_map_tile(...)` (slice 5).

## 8. Slices

1. **Done.** Routable graph, edge-based A*, oneway, access, speeds, via-node
   restrictions, the dead-end U-turn rule, and a test-only Dijkstra.
   - Synthetic tests cover costs, every oneway form, no_left_turn,
     only_straight_on, the U-turn and same-way no_u_turn, unreachable targets,
     ties and limits, plus 720 random restricted grid queries (A* ==
     Dijkstra).
   - On the fixtures, 400 random pairs each agree with Dijkstra and every
     route validates. The negative control is that dropping the restrictions
     changes 3 and 35 routes respectively.
2. **ALT landmarks, snapping and the region graph.**
   - ALT landmarks in the library.
   - Snapping and mid-edge start/end with the car's heading.
   - Region graph assembly from a store's roads.graph tiles, built in the
     background.
   - Timing on the real home store, on the owner machine: target < 10 ms for
     30 km.
3. **Restrictions in roads.graph and via-way rules.** The restriction section
   in roads.graph (encoding 6, release id change) and via-way rules through
   state expansion.
4. **Guidance.** The maneuver list and the 10-manoeuvre fixture route.
5. **Map geometry.** Map tiles at three LODs and the route polyline.
6. **Later, with traffic.** The lane graph (`turn:lanes`, lane connectivity at
   junctions), signals and stop points from the P2 furniture records.
