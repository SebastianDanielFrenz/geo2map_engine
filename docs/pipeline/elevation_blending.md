# Source-priority elevation blending

`g2m::world::blend_elevation_sources` joins two quantised rasters that already
share a sample grid. The primary source owns every valid sample. The fallback
fills only primary NoData cells; a sample missing in both sources stays
NoData. Negative heights are ordinary values and remain supported.

At locations where both sources have data, the join measures the integer
height difference `primary - fallback`. For a fallback-only output sample,
it takes the nearest overlap sample by Manhattan distance and adds that
measured difference, linearly fading it to zero at `radius_m`. The estimate
is clamped to `max_adjust_m` (20 m by default); callers with a known larger
offset can raise that value explicitly. Equal-distance overlaps are resolved
by row then column, independent of traversal order. Only overlap samples can
seed the correction, so primary-only data cannot pull the fallback surface.

The caller supplies a `RasterRect` core and context containing a complete
`ceil(radius_m / spacing_m)` sample halo around it. The primitive rejects an
incomplete halo, which makes whole-grid and tiled output agree when each tile
receives that full context. Work is bounded by `max_samples` (16 Mi samples by
default); the nearest-seed pass is a finite-radius multi-source priority-queue
search. A fallback sample with no overlap seed inside its physical support
remains unadjusted and is counted in `fallback_without_overlap`. Diagnostics
report the total overlap seeds in the supplied context, preserved primary
samples, fallback fills and adjustments, and remaining NoData. A core covered
entirely by primary data takes a copy-only path and skips nearest-seed search.
The overlap difference is an empirical seam correction, not a vertical datum
conversion. The caller records known source references and empirical adjustments; incomplete
height-reference documentation is allowed by the owner. A combined heightfield
can explicitly declare an unspecified reference. The primitive performs no provider access, resampling,
or implicit datum conversion.