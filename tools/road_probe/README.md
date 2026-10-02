# Read-only road construction probe

Build g2m_road_probe with G2M_BUILD_APPS. Example from the geo2map_engine root:

```powershell
out/build/release/apps/g2m_tiler/g2m_road_probe.exe S:\claude_code\racing_game\cache\g2m\home-r1 home-2026-09r1 utm32n/2/455/5421 out/g3c_home_reference.csv
```

The probe reads nine source tiles from the immutable store, derives a graph body
without writing to the tile cache, and fits independent junction stretches.
CSV output records failures, class radius, source radius estimates, owned edge
counts, fitted length and per-way timing. A source radius is the median of
noncollinear three-node circumcircles; it is diagnostic, not proof that a fit is
impossible. Missing source tiles are explicit absent inputs.

The home fixture produced 1,512 nodes, 344 ways, 1,589 edges and 971 owned edges.
Compressed graph body: 29,998 bytes, or 28,608 bytes/km² over a 1.048576 km² tile.
Graph derivation took 79.3 ms excluding store loading. Reference processing took
29.6 ms including CSV diagnostics. These are single-run construction timings,
not game frame timings. Bodies exclude container headers and include halo and
complete-way redundancy.

431 of 478 stretches fitted under strict project bounds; 47 declined, affecting
320 owned original segments. Accepted full-context references totalled 35.609 km;
this includes geometry outside the tile. These counts are not road-length
coverage. Failure causes were overlapping corner trims (18) and corner corridor
bounds (29). The owner must choose how served geometry represents exceptions;
this probe does not silently loosen limits or omit failures from its report.

Pass --exceptions to preserve geometry with flagged curvature exceptions. The
home fixture then fits all 478 stretches (344 complete ways), with 47 flagged
exceptions and no declined owned edges. Full-context fitted length is 40.759 km.
Construction took 33.6 ms in a single run; no runtime behavior is enabled.

Use --profiles (optionally with --exceptions) to construct complete DEM profiles.
The probe derives elev.base in memory from the same read-only raw source store;
it never writes derived tiles to that cache. The home fixture needs 84 DEM tiles.
339 profiles succeed, including one bridge profile. Remaining outcomes are 19
unsupported tunnels, 75 analytically incompatible endpoint-curvature boundaries,
one infeasible fixed cubic boundary, one class-grade conflict and 43 bounded
solver convergence failures. Construction plus DEM derivation took 1.63 seconds
in one run. NoData errors remain explicit; these counts are not runtime coverage.
The analytically incompatible boundaries are distinct from solver failures.


With `--profiles`, the probe writes `<OUTPUT_CSV>.profiles.csv` with success,
endpoint height/grade and failure reason per stretch. `<OUTPUT_CSV>.edges.csv`
records original way/pair/node identities and bridge/tunnel flags. These expose
true branching junctions separately from degree-two structure transitions.
