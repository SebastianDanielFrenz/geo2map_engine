# Junction-aware reference lines

RoadReferenceBuilder validates and borrows an immutable RoadGraph, then computes
node valence once. Shared node identities split ways into stretches at branches;
coordinate crossings do not. Original pair indices survive coincident-coordinate
collapse, smoothing and corner construction. fit_stretches returns independent
results; fit rejects a whole way if any stretch fails. Concatenated references
mark junction breaks explicitly and retain node/station anchors.

The first fit uses G1 clothoids with length-weighted shape tangents. Curvature
and curve-to-chord bounds are checked with an interpolation-error certificate.
A fallback rounds corners with minimum-radius arcs when their trims fit and
vertex deviation stays bounded. Up to 64 bounded smoothing iterations can
adjust shape nodes; junctions and endpoints remain fixed. No source graph is
mutated. Closed chains are supported. Segment, length and fitting-work limits
fail explicitly.

Default vertex deviation is 0.5 m for rank >= 5 and 3 m for other roads. This is
separate from the 32 m curve-to-chord bow allowance. Radius defaults come from
resolve_road_attributes, including the separate ramp defaults. These are project
policies. The owner continuation selects preservation with explicit curvature exceptions. Strict fitting remains the API default. allow_curvature_exceptions retries only corner corridor/overlap failures, halving the radius to a 1 m floor in at most 17 retries. Output exposes requested and certified fitted radii and an exception flag. Invalid data, reversals and resource failures still fail.

Release CI passes all 19 checks. Windows ASan and Linux TSan pass reference,
attribute, DEM and fitting tests. Closed-reference bit-pattern golden is
`a0edfbfd50facd4c` on both platforms. Tests cover stable identities, independent
stretch failure, junction splits, G1 joins, rounded corners and closed chains.
No roads.geom serving or runtime carving is enabled by this API.
