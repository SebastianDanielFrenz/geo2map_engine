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
