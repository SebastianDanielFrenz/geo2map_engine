# Release id and release manifest

## Release id (rid)

A release is a frozen set of source data plus everything that turns that
data into tiles. The rid is SHA-256 over this byte string (PLAN.md 3.5):

```
str("g2m.rid/1")
u64 n_sources,     per source tile (sorted by layer, key_packed, body_hash):  str(layer) u64(key_packed) body_hash[32]
u64 n_layers,      per layer (sorted by layer, version, config_hash):          str(layer) u32(version) config_hash[32]
u64 n_classifiers, per classifier (sorted by name, table_hash):               str(name) table_hash[32]
u64 world_seed
```

`str(s)` is `u64 length` followed by the bytes, and every integer is
little-endian.

- **Sources:** every raw tile of the snapshot. The source snapshot index is
  its (layer, key, body hash) triples.
- **Layers:** every layer in the release, raw layers included. A raw layer
  has an all-zero `config_hash`. A derived layer's version is its deriver
  version, and its `config_hash` is that of its configuration.
- **Classifiers:** name plus the hash of the mapping table.

Inputs are sorted before hashing, so the order they were collected in never
matters. A duplicate source (same layer and key), layer or classifier name
is an error. A local import (the integrated server's `make_local_release`)
computes its rid the same way from its source store.

## Manifest JSON (`g2m.release/1`)

Canonical: object keys sorted, no whitespace, integers as JSON numbers,
hashes as 64 lower-case hex digits. Identical manifests serialise to
identical bytes on every platform. The parser ignores unknown keys (forward
compatibility) and fails on missing or mistyped required ones.

```json
{
  "classifiers": [{"name": "...", "table_hash": "<hex>"}],
  "container_major": 1,
  "format": "g2m.release/1",
  "layers": [{
    "availability": "prebaked" | "on_demand" | "unavailable",
    "config_hash": "<hex>",
    "content": "one line for humans",
    "coverage": {"global": false, "zones": [{"runs": [[y, x0, x1], ...], "scheme": "utm32n"}]},
    "encoding": 1, "encoding_version": 1,
    "inputs": ["<layer>", ...],
    "kind": "raw" | "baked",
    "levels": [2],
    "name": "g2m.src.elev.dgm1",
    "tiling": "utm",
    "version": 1
  }],
  "rid": "<hex>",
  "sources": [{"attribution": "...", "id": "dgm1_nrw", "licence": "dl-de/zero-2-0",
               "share_alike": false, "snapshot_date": "2026-09-25"}],
  "world_seed": "<16 lower-case hex digits>"
}
```

Rules:

- Sources are sorted by `id`, layers by `name`, classifiers by `name`.
- Coverage is a set of level-2 (1024 m) tiles per UTM zone, as row runs
  `[y, x0, x1]` (inclusive). Runs are sorted by `(y, x0)` and are neither
  overlapping nor adjacent. Zones are sorted. `global: true` means
  everywhere.
- `world_seed` is a hex string, not a number, so no JSON reader rounds it
  through a double.
