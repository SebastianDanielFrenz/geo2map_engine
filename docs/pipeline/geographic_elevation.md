# Geographic elevation ingestion (terrain port)

`g2m::decode::GeographicDem` consumes pinned EPSG:4326 float32 GeoTIFF/COG
bytes through the existing `IByteSource` and `TiffReader`. It bilinearly samples
a geographic mosaic and reprojects onto a supplied registered UTM grid using
the existing deterministic geodesy and half-even height quantization.

Sources are searched in declared order. North/south interpolation samples each
row using that row's own longitude spacing, supporting the latitude-band width
changes of the AWS GLO-30 COGs. East/south edge neighbors are read when needed;
missing neighbors or weighted NoData produce NoData, never edge clamping or
invented sea-level land. Negative elevations are preserved. Longitude wraps at
the antimeridian. File registration comes from GeoTIFF metadata, not filenames.

The importer-owned sampler is single-threaded and holds an LRU of 8 blocks by
default (1-64 configurable), at most 1024 squared float samples per block.
Source count is limited to 256; output grids to 4,194,304 samples. A TIFF decode
can also require compressed-block scratch memory; this is not a total RSS cap.
Readers validate byte offsets through the existing bounded TIFF/byte-source API.

This is the sampling/import primitive, not yet a new default world source.
It does not overwrite DGM1, label EGM2008 as DHHN2016, filter DSM, acquire network
data, or blend sources. A supplied height adjustment is applied before
quantization and must be retained in the caller's import provenance. The owner's
2026-10-22 revision allows useful data with incomplete vertical-datum evidence;
documentation gaps are not availability blockers. Result checks (seams, slopes,
road and physics agreement) remain the acceptance criterion.

Reference: [AWS source layout and registration](https://copernicus-dem-30m.s3.amazonaws.com/readme.html).
Mapterhorn WebP/PMTiles decoding and acquisition are separate pending adapters;
its encoded images cannot be handed to this TIFF adapter directly.


## Acquisition, import and serving

Acquire a small extract (up to nine 1-degree source files; each capped at 128 MiB):

```sh
python tools/fetch_glo30.py --bbox 8.65 49.95 8.66 49.96 --destination out/glo30_probe/sources
python tools/fetch_glo30.py --destination out/glo30_probe/sources --verify
```

The atomic `sources.json` lock records URLs, SHA256, lengths, ETags, source type
and height reference. Existing locks do not silently accept changed upstream
bytes. A 404 is recorded as unavailable, not ocean/zero. 429 and temporary server
errors retry a bounded number of times. First acquisition pins the bytes returned
by the public endpoint; it does not imply the provider offers immutable URLs.
The download tool is explicit/offline preparation, not client hotlinking during play.

With `G2M_BUILD_APPS=ON`, build `g2m_geographic_dem` and run:

```sh
g2m_geographic_dem utm32n 455 5403 out/glo30_probe/tile.g2m out/glo30_probe/sources/Copernicus_DSM_COG_10_N49_00_E008_00_DEM.tif --store out/glo30_probe/store --scope glo30-probe-v1
```

This writes a standalone container and, when both optional arguments are present,
imports it into the named source scope (`tiles` store). The layer is
`g2m.src.elev.glo30`, datum EGM2008, with full-file source SHA256 values in its
header inputs. No existing DGM1 scope is changed by this example.

`TerrainServiceConfig.elevation_source` selects the raw layer. The service registers
all imported elevation descriptors found in the scope. `ElevBaseDeriver` copies
that registered 1 m UTM lattice and preserves the source datum; height pyramids
preserve matching child datums. Default DGM1 config hashes remain unchanged.
Different selected sources produce different release identities. This is explicit
source selection, not automatic source blending. Mixed child datums require
normalization before pyramid generation rather than incorrect labeling.

In racing_game, `source_store.elevation_layer` is the optional equivalent; its
default is `g2m.src.elev.dgm1`. To use an imported extract, set that field to
`g2m.src.elev.glo30` and point the source dir/scope at the new store. The imported
DSM is not yet filtered to bare ground and an elevation-only extract has no OSM
roads/buildings or land cover. It is not a complete new playable region.

## Evidence

Windows and Linux imported the real 45,132,234-byte N49/E008 AWS COG into
utm32n/2/455/5403 with 1,048,576 valid samples, no NoData, range 92.8008-120.680 m,
and ~0.62-0.66 s sampling/encoding on the development PC (file hashing excluded).
Standalone container SHA256 on both:
`dcc9f030248edf32de7bf65f04c5333ce1797123a8e7ce00895ede4d2e14db4c`.
Source SHA256:
`14deaaca7e44e821111c1947dc65c7cffbe5f67e3986f371d2c967ff999e592b`.
Synthetic tests exercise mixed-width latitude seams, east neighbors, NoData,
negative land, dateline, UTM registration, caching and output limits. Service
integration exercises GLO-30 and DGM1 in one scope, selected values, L0-L2 datum
propagation and release separation. Acquisition tests use a fake network.
