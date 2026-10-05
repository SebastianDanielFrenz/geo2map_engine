# G3 profile foundation

These are pure construction APIs, not a shipped roads.geom layer yet. The
existing game heightfield and road raster are unchanged.

## Physical road policy

`resolve_road_attributes` reads canonical preserved way tags against RoadStyle.
Explicit usable width takes precedence over lanes times 3.25 m, then the class
default. Width is bounded to 0.5–100 m; lanes must be integers from 1 to 30.
Decimal parsing is integer, locale-independent, bounded, and rounds half away
from zero. A trailing ` m` on width is accepted. Surface overrides use the
existing material table. Numeric incline retains its sign in parts per thousand;
non-numeric up/down tags remain unspecified. Bridge/tunnel metadata remains
available; this API does not make either structure driveable.

Project fitting defaults (not regulatory design standards):

| Rank/class | Grade limit | Minimum vertical radius | Minimum horizontal radius | Blend |
|---|---:|---:|---:|---:|
| 0 track | 20% | 50 m | 6 m | 3 m |
| 1 service | 20% | 80 m | 6 m | 3 m |
| 2 living street | 15% | 100 m | 10 m | 5 m |
| 3 residential | 15% | 150 m | 15 m | 5 m |
| 4 unclassified | 15% | 200 m | 25 m | 5 m |
| 5 tertiary | 12% | 300 m | 40 m | 8 m |
| 6 secondary | 12% | 500 m | 60 m | 10 m |
| 7 primary | 12% | 750 m | 100 m | 12 m |
| 8 trunk | 6% | 1000 m | 250 m | 15 m |
| 9 motorway | 6% | 1500 m | 500 m | 15 m |

Shoulder is 1.5 m and undivided crown is 2.5%. Superelevation and junction
surfaces are later construction steps. Width/surface inference flags remain
explicit so downstream geometry can distinguish supplied data from defaults.

## Complete elevation dependencies

`road_dem_dependencies` enumerates sorted, unique level-0 elev.base tiles over
every complete segment, with exact integer segment/expanded-box intersection.
Its default 65 m margin covers half the maximum road width plus the blend.
The default working set is 1024 tiles; both work and grid limits fail explicitly
instead of clipping geometry. Halo ownership never limits the profile extent.

`RoadDemSampler` borrows decoded level-0 tiles. Bilinear interpolation uses
integer-mm weights and half-even rounding at the native 1/256 m height unit.
Zero-weight neighbours are irrelevant; missing/nonzero-weight NoData samples
fail. Junction height is a median over a complete radius-3 m circular footprint
on an integer-metre lattice centred on the original node. This depends on the
node and DEM rather than whichever neighbouring ways a tile happens to contain.

`IDeriver::additional_inputs_for` adds one pure dependency-discovery stage after
the initial inputs resolve. A road deriver can inspect its complete graph before
naming distant DEM inputs. The resolver validates discovered refs, deduplicates
initial overlaps, resolves absent/unavailable inputs with the usual semantics,
and includes all present input hashes in cached headers. At most 4096 combined
dependencies are allowed. The default method returns no additional refs. A deriver may ask for more stages (`discovery_stages()`, at most 4): the resolver calls `additional_inputs_for` again with every input resolved so far until a stage names nothing new; roads.geom uses two (P2 decision D35).

## Horizontal and vertical fitting primitives

`fit_clothoid` implements the scalar G1 fitting reduction described by
[Bertolazzi and Frego](https://arxiv.org/abs/1209.0910). It uses our deterministic
math, bounded Newton solve and fixed composite Gauss-Legendre quadrature.
Lines and arcs are limiting cases of the same representation. A curve exposes
position, tangent and linear curvature at arc-length stations. This primitive
does not yet smooth complete OSM ways or resolve junction topology.

`fit_vertical_profile` projects DEM height and bounded central-difference grade
targets onto sparse cubic constraints using our weighted implementation of
[Dykstra's method](https://doi.org/10.1080/01621459.1983.10477029).
The objective is squared height error plus slope_weight_m squared times squared
grade error. Endpoint heights and supplied endpoint grades are fixed. Adjacent
Hermite cubics share heights and grades. Bounding the three Bernstein grade
coefficients bounds grade throughout the segment; bounding both second-
derivative endpoints bounds curvature throughout it conservatively. These
constraints can reject a curve that would satisfy a looser exact extremum test.

Limits are 2048 samples, at least 0.5 m station spacing, 100 km extent, bounded
iterations and 64 million half-space visits. Nonconvergence/infeasible fixed
boundaries fail by value. Endpoint heights/grades evaluate exactly at endpoints.
Bridge abutment interpolation and layer orchestration are not implemented here.

## Verification

Release CI passes all 19 checks with existing goldens unchanged. The road
policy/DEM tests, discovery cache/error/offline tests, clothoid tests and vertical
tests pass on Windows and Linux, including AddressSanitizer/ThreadSanitizer
checks. Clothoid bit-pattern golden: `7034801945bff7b1`; constrained vertical
profile golden: `5ae5a60af3c10f3f`. Independent checks include a circular arc,
unit-speed and endpoint tests over 81 tangent pairs, and an analytic symmetric
least-squares optimum. Full G3 acceptance remains pending.

Ramp horizontal-radius defaults are 10/15/20/30/60 m for tertiary_link,
secondary_link, primary_link, trunk_link and motorway_link respectively.
These remain project fitting defaults, not regulatory standards.

## Complete-stretch profile construction

road_profile.h joins RoadReferenceBuilder to RoadDemSampler and the constrained
vertical solver. It preserves the full topological stretch and physical tags,
including explicit horizontal-curvature exceptions. Ground interior samples use
a seven-point cross-road median at 5 m station spacing; every contributor must
exist, so NoData is never hidden by the median. Complete endpoint footprints
use a robust shared-node DEM plane: a common median height and world-space
gradient projected onto each incident road heading. Junctions retain slope.

Bridge boundaries of degree two are structure transitions. Abutment heights
and slopes are estimated from ground approaches 32 m away, avoiding underpass
DEM samples. A canonical bridge-span feasibility projection may move these
inferred heights by at most 0.25 m per endpoint; approaches use the same anchors.
Unsupported topology or larger corrections fail explicitly. Interior bridge
bare-earth DEM is never queried. This does not establish clearance, deck thickness or collision
acceptance. Tunnels return Unsupported. Numeric incline metadata supplies a soft interior slope target; shared endpoint
heights/slopes and road-class grade/curvature bounds retain priority (owner decision 2026-10-05: where an endpoint height needs a steeper grade than the class bound, the endpoint height wins and the stretch is flagged as a grade exception, not clamped). Sample, extent and
iteration budgets fail explicitly; long stretches are not clipped to fit.

Profile and reference regressions pass on Windows release/ASan and Linux TSan
(513 assertions in 11 cases), including interior outliers, missing samples,
bridge NoData exclusion, endpoint grades and station validation. Release CI
passes all 19 checks. This remains construction code, not a served geometry or
runtime carving layer.

The vertical solver now checks necessary endpoint reachability analytically by
integrating the grade envelopes implied by both endpoint slopes, the grade cap
and the second-derivative bound. This rejects impossible curvature boundaries
before consuming the projection budget. It is a necessary test, not a promise
that the conservative cubic constraints will converge or be feasible. Updated
vertical/profile/reference selection: 2,276 assertions in 16 cases pass on
Windows ASan and Linux TSan; existing vertical and reference hashes are unchanged.


Bridge/ramp junction extension (2026-10-02): one incident bridge with up to eight
ground approaches uses a canonical mean plane only when the projected approach
heights agree within 0.5 m (0.25 m uncertainty per estimate). Otherwise it fails.
Ground fitting uses the inferred plane within the 32 m approach zone rather
than bare-earth underpass targets. All support footprints must be complete.
The marked A66 bridge 1045544336, ground ways 1045544335/194653320 and bridge
23658583 now fit with exactly shared endpoint heights. This is estimated
construction geometry, not measured clearance or collision acceptance.
The home probe accepts 395/478 profiles, including six bridges. Remaining
failures are explicit; roads.geom serving is implemented; complete profile acceptance and runtime carving remain pending.
