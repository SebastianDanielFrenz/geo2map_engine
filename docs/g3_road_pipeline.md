# G3 road pipeline: implementation slices

## A. Identity-preserving graph foundation (implemented)

`g2m/layer/road_graph.h` / `src/layer/road_graph.cpp` build a canonical geometric
graph directly from validated `osm::OsmData`. Nodes retain OSM ids and projected
integer-mm positions. Ways retain every original reference and every way tag.
Edges have stable `(way_id, original_ref_pair_index)` identities. Shape nodes
remain explicit; later degree-two simplification is a view, not an identity
change. Geometric crossings or coincident coordinates never create a junction.

The existing RoadStyle highway table determines eligibility/rank; area=yes
ways are excluded. Bridges and tunnels are retained as tagged ways here. This
API does not change the existing raster, heightfield, rendering or physics.
A way outside the projection's supported zone is rejected as a whole and
counted. Consecutive repeated refs are omitted from edges without renumbering
later segment identities. Closed ways retain their closing edge.

This graph is geometric and undirected. It is not a navigable G4 routing graph:
oneway/access rules, restrictions, signal/stop-node metadata and routing costs
need their own semantic model. The served tile layer and wire format are described below.

Verification covers shared-node junctions, a crossing bridge, tunnel/tag
retention, insertion-order independence, stable identities with different
neighbouring way sets, closed/repeated refs, malformed data and far-zone ways.
The complete Frankfurt-Hoechst fixture yields 3,136 nodes, 739 ways and 3,309
edges. Its canonical fingerprint is `2d63f916719a0c3b`.

## B. Tile encoding and derivation (implemented)

`g2m.roads.graph` is a level-2 baked layer with a bounded encoding 4/version 1.
See [the format](formats/road_graph.md) for exact bytes and ownership rules.
The deriver merges nine raw source tiles, deduplicates identical complete ways
and fails conflicting copies. Whole ways intersecting the 128 m halo retain
all references and tags. Integer midpoint ownership is unique across tiles.
Server registration is opt-in; `g2m_tiler bake --layers g2m.roads.graph
--min-level 2 --max-level 2` selects it automatically.

Five tests cover codec truncation and malformed data, source order/conflicts,
adjacent ownership and real server/bake parity with four workers and pinning.
Windows release CI passes all 19 checks with existing goldens unchanged.
Windows AddressSanitizer and Linux ThreadSanitizer pass the new tests;
the new decoder is included in the deterministic fuzz mutation corpus.
Bandwidth and derivation CPU measurements are still pending.
## C. Reference lines and vertical profiles

Resolve width/lanes/surface/bridge/tunnel/incline from preserved way tags using
a shared data policy. Keep original ids through any fitted geometry. Fit the
reference line and bounded vertical profile against elev.base; bridge spans
use abutment interpolation. Compute junction heights from a deterministic DEM
footprint. Complete profile dependencies must be explicit, including portions
of complete ways outside the tile; never silently fit from a clipped fragment.

Tests: class grade bounds, missing/NoData dependencies, repeatable fitting,
bridge clearance/abutments, endpoint agreement and cross-tile profiles.

## D. Junction surfaces and carving

Build cleaned junction footprints and deterministic surfaces, crown/shoulder
and blend bands, then derive terrain.height and terrain.class from the same
geometry. Ground roads carve; bridge terrain remains intact. Preserve the
existing default base-height behavior until carved layers are selected.

Acceptance from PLAN.md: cross-slope within 0.2%, junction continuity <1 cm,
blend sample steps <=5 mm, class grade limits and Windows/Linux golden hashes.

## E. Client ribbons, markings and bridge bodies

Use roads.geom for both rendered surfaces and static bridge collision meshes.
Prepare meshes outside ticks using physics_sim's prebuilt-shape API, with
bounded deterministic installation. Measure bridge/heightfield contacts before
claiming bridges driveable: the outstanding heightfield/mesh spike is separate
from terrain tile seams. Record approach steps <=2 cm and wheel-force evidence.

Tunnels remain identified in the graph but not driveable in v1, as PLAN.md
specifies. Portal holes and tunnel collision meshes are a later explicit slice.

## F. R3 acceptance

Script a route through 20 junctions, check SurfaceKind under every wheel,
record wheel-force peaks, and traverse each fixture bridge both ways. Report
full vehicle/frame-time behavior separately from terrain-focus replay timing.
R3 remains incomplete until those checks and ribbons/markings/bridge bodies land.
