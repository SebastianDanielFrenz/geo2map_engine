# Local Mapterhorn PMTiles extraction

`tools/extract_mapterhorn_pmtiles.py` reads a local PMTiles v3 archive and
extracts one bounded Web Mercator XYZ zoom and bounding box as `z/x/y.webp`.
It does not open URLs, fetch missing tiles, or fill uncovered areas with zero
elevation. Its output layout can be passed to
[`prepare_mapterhorn.py`](mapterhorn_preparation.md) for WebP decoding and RGBA
pinning.

```powershell
python tools/extract_mapterhorn_pmtiles.py --archive out/mapterhorn/planet.pmtiles `
    --destination out/mapterhorn/extracted --bbox 8.60 49.90 8.75 50.05 --zoom 12
python tools/prepare_mapterhorn.py --source out/mapterhorn/extracted `
    --destination out/mapterhorn/rgba --limitzoom 12 --metadata out/mapterhorn/source_metadata.json
```

The bbox is `WEST SOUTH EAST NORTH` in longitude and latitude degrees. It must
be ordered, non-wrapping across the antimeridian, and within Web Mercator
latitude limits. The selection uses half-open east and south edges, matching
the XYZ tile containing each point. A bbox selecting more than 256 tiles is
rejected. Missing tiles are counted and omitted; the caller gets no invented
terrain at those coordinates.

The reader implements the official fixed 127-byte v3 header, Hilbert TileIDs,
delta-varint directories, run lengths, offset encoding, and root-to-leaf lookup.
It supports internal directory/metadata compression `none` or `gzip`, and tile
compression `none` or `gzip`; the archive must declare WebP tile type. Gzip
limits are enforced while inflating. Unknown, Brotli, Zstandard, and non-WebP
archives are rejected with a clear error.

Default bounds are 64 GiB per input archive, 16,257 compressed bytes for the
root directory (the v3 first-16-KiB rule), 8 MiB per decompressed directory,
100,000 entries per directory, 64 MiB total compressed leaf-directory reads,
32 MiB total decompressed leaf data, 250,000 total cached leaf entries, 4 levels of
leaf indirection, 8 MiB per compressed or decoded WebP blob, and 256 output
tiles. The tool streams a full SHA-256 pass over the archive on each run and
records that hash with archive size, selected bbox/zoom, relevant header fields,
archive JSON metadata, and each tile's archive blob location and output hash.
Hashing a very large local archive can take time, but it pins the exact input.

Files are atomically written. `pmtiles_extraction.json` is the extraction pin;
rerunning the same request against the same archive verifies and reuses matching
outputs. A different archive hash, bbox, zoom, tile set, or changed output is
refused. An absent tile stays absent. Archive metadata attribution is preserved
when available, while licensing decisions remain the caller's responsibility.

The layout, offsets, compression enums, tile type, Hilbert IDs and directory
encoding follow the [official PMTiles v3 specification](https://github.com/protomaps/PMTiles/blob/main/spec/v3/spec.md).
The Hilbert conversion and lookup behavior were checked against the
[Protomaps reference implementation](https://github.com/protomaps/PMTiles/blob/main/python/pmtiles/pmtiles/tile.py).

Output directories reject symlink escapes. An exclusive `.extract.lock` claim
prevents concurrent writers; writes never replace existing files. After a crash,
remove a stale claim only after confirming no extractor is running.
