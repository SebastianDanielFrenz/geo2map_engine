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
dependencies are allowed. The default method returns no additional refs.

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
