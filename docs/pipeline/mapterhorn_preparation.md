# Mapterhorn WebP preparation

`tools/prepare_mapterhorn.py` is an offline preparation step for locally acquired
Mapterhorn WebP tiles in Web Mercator XYZ layout (`z/x/y.webp`). It makes no
network requests and does not hotlink tiles at runtime. Acquisition, source
selection, and licence review remain caller responsibilities.

Pillow is loaded only while decoding a new extract. It is not required to verify
an existing prepared extract. Install Pillow into the Python environment used
for the one-time preparation; the repository does not install it globally.

```powershell
python tools/prepare_mapterhorn.py --source out/mapterhorn/tiles `
    --destination out/mapterhorn/rgba --limitzoom 12 `
    --metadata out/mapterhorn/source_metadata.json
python tools/prepare_mapterhorn.py --source out/mapterhorn/tiles `
    --destination out/mapterhorn/rgba --verify
```

`--limitzoom Z` selects only source tiles whose zoom is at most `Z`, matching
the extracted-tile limitzoom convention. The first run pins that selection;
later runs use the same pin when the flag is omitted. A different explicit zoom,
source tile set, compressed input hash, metadata object, or output hash requires
a new destination. Each XYZ coordinate must be canonical and within its zoom.
The default limits are 4096 tiles, 8 MiB per compressed tile, 2 GiB total
compressed input, 512 by 512 pixels, and 1 MiB per raw tile (4 GiB total raw
output). WebP dimensions are checked before pixel decoding.

Output is tightly packed, row-major RGBA8: exactly `width * height * 4` bytes
with no file header. `sources.json` uses format `g2m.mapterhorn_rgba/1`; each
tile entry contains `z`, `x`, `y`, `width`, `height`, raw `path`, raw `sha256`
and `size`, plus compressed `input_path`, `input_sha256` and `input_size`.
Consumers can read the manifest dimensions and allocate the raw tile directly.
Raw files and the lock are written atomically. The lock is the commit marker, so
an interrupted preparation can be rerun after removing unpinned partial files.
`--verify` checks the compressed source pins and raw pixel hashes using only the
standard library; it never imports Pillow or decodes images.

Optional `--metadata` points to caller-provided JSON, for example:

```json
{
  "vertical_reference": "unknown",
  "attribution": "Attribution text supplied with this extract",
  "dataset_license": "License terms supplied with this extract"
}
```

The metadata object is preserved as `source_metadata`. An unknown vertical
reference is allowed and should remain unknown until verified from the source
metadata. Attribution and dataset licensing are independent caller fields; the
tool does not assign or imply a licence for Mapterhorn's mixed sources. Without
metadata, preparation reports the missing vertical reference, attribution and
dataset licence and records them as unknown/null. Provide the applicable source
metadata before redistribution.

The Python tests use an injected fake decoder for deterministic contract and
lock checks. If Pillow is installed, an additional test creates and decodes a
real 512 by 512 lossless WebP fixture. A fake test fixture is not evidence that
the local Pillow/WebP codec stack works; record the real WebP test separately
when validating a deployment environment.

Output directories reject symlink escapes. An exclusive `.prepare.lock` claim
prevents concurrent writers; writes never replace existing files. After a crash,
remove a stale claim only after confirming no preparer is running.
