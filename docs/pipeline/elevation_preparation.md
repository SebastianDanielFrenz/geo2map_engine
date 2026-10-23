# Preparing canonical ground from elevation sources

`g2m_elevation_prepare JOB.json` is an explicit offline import stage. It writes a
new pinned `g2m.src.elev.<name>` tile into an output source store. The existing
terrain service can serve it by selecting that elevation layer; rendering,
road profiles and physics then consume the same prepared heights. It is not a
second renderer-only surface or a runtime network fallback.

## Inputs and priority

The order is measured/preferred primary store > highest available prepared
Mapterhorn zoom > GLO-30. NoData is respected at each step. Primary valid samples
are copied byte-for-byte; the fallback only fills missing primary coverage.

```json
{
  "format": "g2m.elevation_job/1",
  "target": "utm32n/2/455/5421",
  "glo30_files": ["sources/Copernicus_DSM_COG_10_N50_00_E008_00_DEM.tif"],
  "mapterhorn_manifest": "mapterhorn/rgba/sources.json",
  "primary": {
    "store": "measured_source_store",
    "scope": "measured-source-release",
    "layer": "g2m.src.elev.dgm1"
  },
  "ground": true,
  "blend_radius_m": 256,
  "max_adjust_m": 20,
  "output": {
    "store": "prepared_source_store",
    "scope": "prepared-region-v1",
    "layer": "g2m.src.elev.composite"
  }
}
```

Input sections are optional individually, but at least one source is required.
Paths are relative to the job file. Use a new output scope when inputs/options
change. The primary source must already use registered 1 m cell samples in
north-to-south level-2 raw elevation tiles with zero stored height offset.

Mapterhorn input comes from `tools/prepare_mapterhorn.py`, described in
[mapterhorn_preparation.md](mapterhorn_preparation.md). The command verifies raw
pixel hashes and keys before sampling; multiple zoom levels are searched from
highest to lowest. The Python preparer needs Pillow only to decode WebP; native
sampling and verification do not. PMTiles containers can first be extracted with the stdlib-only
`tools/extract_mapterhorn_pmtiles.py`; see [mapterhorn_pmtiles.md](mapterhorn_pmtiles.md).
Unknown height-reference metadata is allowed. Do not infer a dataset-wide data
licence from the provider's code licence.

## Ground preparation before upsampling

GLO-30 is explicitly a DSM. With `ground=true`, `prepare_ground_elevation` samples
it on a globally aligned 32 m metric grid, fetches the complete morphology halo,
runs `correct_dsm_ground`, then bilinearly upsamples the corrected ground to the
canonical 1 m tile. One extra coarse sample ring supports interpolation. Neighbor
tiles therefore use identical lattice/support rather than independently padding
their edges. The DSM algorithm parameters remain expressed in metres.

Mapterhorn source metadata may specify `surface: "dsm"`, `"dtm"`, or `"unknown"`.
Only an explicit DSM receives that correction. DTM and unknown inputs are sampled
directly and retain their detail. Setting `ground=false` disables DSM correction.
Missing coarse interpolation support falls back to the original sampler rather
than inventing data. NoData in a morphology dependency neighborhood suppresses
inference there. Negative real ground remains a normal height value.

This morphology cannot recover every bare-earth feature from a broad canopy or
otherwise missing observations. It is an inference, not measured ground.

## Joining sources

`blend_elevation_sources` measures primary-minus-fallback differences where both
sources exist. Only fallback samples receive the capped empirical adjustment,
which decays with distance to overlap. Measured primary cells never move.
The command supplies a full 256 m halo per join stage and defaults to a 256 m
fade with a 20 m adjustment cap. This spreads a substantial source offset over a
gentler transition than a 20 m fade. The primitive's radius remains configurable;
the command supports 0-256 m and records the choice in provenance.

Whole and independently halo-tiled results are tested for equality. Equal-distance
seeds have deterministic row/column priority. No overlap means an unadjusted
fallback plus an explicit diagnostic; missing both sources remains NoData.
These local seam adjustments do not establish a geodetic datum conversion.

## Identity and height references

Canonical provenance records source roles and GLO file order, Mapterhorn manifest
identity, every queried primary tile address/body hash or absence, source datums,
and algorithm options. Its SHA256 joins the raw input hashes in the output header;
the record is saved under `elevation_provenance/<hash>.json` in the output store.
Existing outputs must match the complete container, including provenance and
datum, or the command refuses replacement. Identical reruns reuse the result.
The command takes an exclusive output-store preparation claim. A concurrent
preparer fails without modifying its output. After a crashed preparer, verify
that no preparer is running before removing a stale `.elevation_prepare_lock`
directory. Other source-store writers must not mutate a pinned scope.

Pure GLO-30 preparation retains EGM2008. Jobs combining sources consistently use
`VerticalDatum::Unspecified` (wire value 3); known source references remain in
provenance. This avoids claiming DHHN2016 or making one composite scope alternate
height labels tile by tile. Road, rail and carving accept this explicit height
reference, while `None` remains distinct from a height reference. Conflicting
known DHHN2016/EGM2008 dependencies still fail, even with unspecified dependencies
between them. Old readers that do not support value 3 reject these new containers;
existing DGM1 containers/goldens retain their original bytes.

The owner's 2026-10-22 ruling permits useful terrain without verified height-datum
documentation. Such documentation is not an availability gate.

## Bounds and evidence

The command imports one 1024 m tile at a time, accepts at most nine GLO files and
256 decoded 512x512 RGBA tiles (256 MiB payload). Intermediate source grids are
bounded to 2048x2048 samples plus the much smaller coarse DSM dependency support.
The primitive blend has a separate 16 Mi-sample ceiling. These are work/data limits,
not a total process RSS promise; temporary arrays and queues also consume memory.

Tests cover real WebP encode/decode, pin verification/corruption, tile seams,
antimeridian, pixel-center roundoff next to NoData, negative ground, composed DSM
halos, source-priority preservation, whole-vs-tiled blending, dependency identity,
same-height provenance changes, and immutable scopes. The native command's
integration tests need `G2M_ELEVATION_PREPARE_EXE` pointing to its built executable.

On four measured Hessen tiles, 16,384 grid points per tile gave raw GLO-30 RMS
errors of 3.147, 1.604, 2.067 and 17.547 m; prepared errors were 1.837, 1.339,
1.490 and 17.438 m. The fourth tile retains a large residual error. These local
comparisons include differing source resolution/reference/epoch effects and do
not prove worldwide accuracy. In all four composed outputs, every one of the
1,048,576 measured primary samples was preserved exactly. `evaluate=true` prints
the raw/prepared comparison against supplied primary data without changing it.

A bounded live Mapterhorn XYZ probe returned HTTP 403 on the development machine.
The real codec tests use generated lossless WebP; current provider pixel data has
not yet been validated through that endpoint. Prepared local extracts remain
supported, and this code does not hotlink sources while driving.
