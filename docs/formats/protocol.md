# Protocol v1 (PLAN.md 3.4)

In G1 the requests and responses are in-process objects
(`g2m/server/protocol.h`) passed through `ITransport`. G6's HTTP transport
serialises exactly these objects. Paths are relative to `{base}/g2m/v1`.

## Endpoints

| Path | Request | Response body |
|---|---|---|
| `/manifest` | ManifestRequest | `g2m.server/1` JSON (below) |
| `/releases/{rid}/manifest` | ReleaseManifestRequest | `g2m.release/1` JSON ([release_manifest.md](release_manifest.md)) |
| `/releases/{rid}/tiles/{layer}/{scheme}/{level}/{x}/{y}` | TileRequest (+ optional if_none_match) | one g2m.tile/1 container |
| `/releases/{rid}/packs/{layer}/{scheme}/{level}/{px}/{py}` | PackRequest | a pack ([pack.md](pack.md)) |
| `/releases/{rid}/hashes/{layer}/{scheme}/{level}/{px}/{py}` | HashesRequest | the pack directory only |

- `{rid}` is 64 lower-case hex digits and `{scheme}` is `utm{zone}{n|s}`.
- For packs and hashes, `px`/`py` are pack coordinates.
- Integers are canonical decimal: no leading zeros, no `+`, `-` only for
  negative values. So `to_path(parse_path(p)) == p` for every accepted `p`.

A tile response's container carries the release's rid, even when the body
was cached or fetched under another release: the body is the same, and the
header says which release answered.

## Statuses

| Status | Meaning |
|---|---|
| 200 | the data is in the response |
| 304 | tile only: `if_none_match` equals the body hash (ETag) |
| 400 | malformed request (layer name, key, level) |
| 404 | definitive for the release: unknown release or layer, a level the layer lacks, outside coverage, or no store, deriver or upstream has the tile |
| 410 | release retired; `newest_rid` names its successor |
| 503 | not available now. With `degraded`: offline and not cached, `retry_after_s` = 0. Otherwise `retry_after_s` > 0: an upstream or a derivation failed transiently or is busy |
| 500 | server-side failure (store I/O, deriver error) |

- **Packs and hashes:** 200 when every tile is present or definitively
  absent; 404 when all 256 are absent; otherwise 503, and the response
  still carries every tile that was available, with the directory saying
  which ones were not.
- **Offline:** the server never calls a network upstream. It answers from
  its stores, its local upstreams and its derivers, and otherwise returns
  `503 degraded`.

## Server manifest JSON (`g2m.server/1`)

Canonical (keys sorted, no whitespace). Unknown keys are ignored.

```json
{"auth_mode":"none","contact":"","default_rid":"<hex>","format":"g2m.server/1",
 "live_releases":["<hex>",...],"protocol_versions":[1],"rate_limit":"",
 "server_id":"g2m.integrated","tile_base_url":""}
```

`live_releases` is sorted. An empty `tile_base_url` means the manifest's
own base. `protocol_versions` lists u32 values.
