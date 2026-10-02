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
need their own semantic model. No new served layer or wire format ships yet.

Verification covers shared-node junctions, a crossing bridge, tunnel/tag
retention, insertion-order independence, stable identities with different
neighbouring way sets, closed/repeated refs, malformed data and far-zone ways.
The complete Frankfurt-Hoechst fixture yields 3,136 nodes, 739 ways and 3,309
edges. Its canonical fingerprint is `2d63f916719a0c3b`.

## B. Tile encoding and derivation (next)

Define a bounded, versioned `roads.graph` encoding and register its layer and
deriver. Merge complete source ways by OSM id before graph construction;
identical duplicates deduplicate, conflicting copies fail. Resolve tile
ownership with the existing integer tile grid, retaining halo references and
stable ids. Do not infer connectivity from the raster capsules.

Tests: codec round trips, truncated/invalid/oversized input rejection, shuffled
and duplicate source inputs, adjacent-tile stable ids, clipped/halo crossings,
closed ways and Windows/Linux goldens. Measure bytes/km2 and derivation CPU.
The current source codec already preserves complete ways and tags, so an OSM
source-format migration is unnecessary for this slice.

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
