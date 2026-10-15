# Protocol v1 over HTTP (G6)

Protocol v1 itself (endpoints, request objects, statuses, the server
manifest) is [docs/formats/protocol.md](../formats/protocol.md). This page is
the HTTP binding of exactly those objects: `g2m_http`
(`include/g2m/http/`, `src/http/`) and the sidecar `apps/g2m_server`.

- `http_wire.h`: the pure, socket-free mapping (request target, headers,
  status, body) in both directions. Everything below is implemented there;
  the client and the server only move bytes.
- `HttpTransport` (`http_transport.h`): an `ITransport` over HTTP/1.1.
- `HttpServer` (`http_server.h`): an HTTP front end for any `ITransport`
  (normally an `InProcessTransport` over a `g2m::Server`).

The invariant the tests hold it to: for every request,
`HttpTransport(HttpServer(t)).send(r)` equals `t.send(r)` field for field,
with byte-identical containers, packs and directories
(`tests/http/test_http_server.cpp`, 27 requests covering every endpoint and
every status).

## Requests

`GET {base}/g2m/v1{to_path(request)}`, with `{base}` empty or `/seg[/seg...]`
(`is_valid_base_path`). The server accepts a target only when it is exactly
that string:

| Target | Answer |
|---|---|
| not under `{base}/g2m/v1/` | 404 (text) |
| method other than GET / HEAD | 405, `Allow: GET, HEAD` |
| longer than 1024 bytes, a query, a fragment, a percent escape, a trailing slash, any non-canonical path (`parse_path` + `to_path(parse) == path`) | 400 |
| more than one `If-None-Match` header, or one that is not a single strong ETag `"<64 hex>"` | 400 |

HEAD answers like GET without the body. `If-None-Match` sets
`TileRequest::if_none_match`; the other endpoints ignore it. A request body
is never read (limit 4096 bytes).

The client sends `User-Agent: g2m/<version>` (configurable) and, for a tile
with `if_none_match`, `If-None-Match: "<hex>"`. It never follows redirects.

## Responses

The HTTP status is the protocol status (200, 304, 400, 404, 410, 500, 503).
`ResponseMeta` travels in headers:

| Field | Header |
|---|---|
| `etag` | `ETag: "<64 lower-case hex>"` |
| `retry_after_s` (> 0) | `Retry-After: <seconds>` |
| `degraded` | `X-G2M-Degraded: 1` |
| `newest_rid` | `X-G2M-Newest-Rid: <64 hex>` |
| `message` (non-empty) | `X-G2M-Message: <text>`: bytes outside 0x20..0x7e and `%` as `%XX` (upper-case hex, nothing else escaped; the decoder rejects any other escape and then keeps the raw value) |

Body and `Content-Type`:

| Endpoint | Body |
|---|---|
| manifest, release manifest | 200 only: the canonical JSON, `application/json` |
| tile | 200 only: the g2m.tile/1 container, `application/vnd.g2m.tile` |
| pack | whenever the server produced pack bytes (200, and 503 with the available tiles): `application/vnd.g2m.pack` ([pack.md](../formats/pack.md)) |
| hashes | whenever the server produced a directory (200, 503): `application/vnd.g2m.hashes`, the pack directory ([pack.md](../formats/pack.md) "Pack directory") |
| anything else | `text/plain; charset=utf-8`, the message (informational only) |

304 has no body and no `Content-Type`.

`Cache-Control` (for a CDN or proxy in front, G7): manifest 200
`public, max-age=300`; every other 200/304 `public, max-age=31536000,
immutable` (a release's content never changes); 404/410
`public, max-age=300`; everything else `no-store`.

## Client status mapping

| HTTP answer | Protocol response |
|---|---|
| 200, 304, 400, 404, 410, 500, 503 | the same status; the meta from the headers; the payload checked against the request |
| 429, other 5xx | 503 with `retry_after_s` from `Retry-After`, else 5 (`default_retry_after_s`); message `http <code>[: ...]` |
| other 4xx (e.g. a front end's 405) | 400, message `http <code>[: ...]` |
| anything else (1xx, other 2xx, 3xx) | 500 |
| a malformed `ETag`, `X-G2M-Degraded`, `X-G2M-Newest-Rid` header; a payload that does not decode, or names another release, layer, tile or pack; an ETag that differs from the container's body hash | 500 |
| no connection, timeout, a reset, a body over `max_body_bytes` (1 GiB) | 503, `retry_after_s` >= 1 (`failure_retry_after_s`), message `http: ...` |

A 503 without `Retry-After` and without `X-G2M-Degraded` gets
`default_retry_after_s`, so a client never retries in a tight loop.
`Retry-After` in HTTP-date form is ignored.

## Connections and threads

- `HttpTransport` is thread-safe: a small pool of keep-alive connections
  (`max_idle_connections`, 8); a connection that failed is dropped, never
  reused. Timeouts: connect 5 s, read 120 s (a cold derivation can take
  long), write 5 s.
- `HttpServer`: a fixed pool (`threads`, 8; `max_queued_connections`, 256),
  keep-alive up to 1000 requests, read timeout 10 s, write 60 s. One
  listener per port (`SO_REUSEADDR` on POSIX, `SO_EXCLUSIVEADDRUSE` on
  Windows; never `SO_REUSEPORT`). `stop()` is idempotent and returns while
  idle keep-alive connections are still open. `stats()` counts requests and
  rejected (unparsable) ones.
- The front end itself holds no state beyond the backend; ordering,
  deduplication and derivation stay in `g2m::Server`.

## Sidecar: `g2m_server`

```
g2m_server --source DIR --scope SCOPE [--source-name tiles]
           [--derived DIR] [--derived-name tiles]
           [--layers terrain|roads|geometry|carved] [--world-seed N]
           [--bind 127.0.0.1] [--port 0] [--base-path /P] [--threads 8]
           [--server-id g2m.sidecar] [--exit-on-stdin-eof]
```

- One local release, built like the game's integrated server builds it
  (`builtin_local_release_params` with the same opt-ins as
  `TerrainService`'s `LayerOptIn`; `geometry`, the default, = roads +
  geometry), so the same store, scope, opt-ins and world seed give the same
  release id the game computes in process.
- Offline by construction: `ServerConfig::offline`, no `IFetcher`, the only
  upstream is the read-only source store. Derived tiles are cached in
  `--derived` when given, else derived per request.
- Binds 127.0.0.1 by default (a warning on stderr for any other address);
  `--port 0` picks a free port. When it serves it prints one stdout line:
  `G2M_SERVER_READY url=http://127.0.0.1:<port><base>/g2m/v1 port=<port> rid=<hex>`
  and runs until SIGINT/SIGTERM, or with `--exit-on-stdin-eof` until stdin
  closes (a parent that dies takes the sidecar along). Exit 0 after a clean
  stop, 1 on a start-up error, 2 on bad usage.
- A client: `HttpTransport::create({.base_url = "http://127.0.0.1:<port>"})`
  (the base without `/g2m/v1`), then use it wherever an `ITransport` is
  taken (`TransportHeightTileFetch`, the client cache's fetch path, ...).

Not wired into racing_game (it uses the in-process server). `G2M_BUILD_HTTP`
defaults to ON only when geo2map_engine is the top-level project, so a
consumer that `add_subdirectory`s it fetches nothing new.

## Not in G6 (what G7, the hosted server, needs)

- **TLS.** `g2m_http` speaks plain HTTP. Either a TLS-terminating reverse
  proxy in front of `g2m_server` (recommended; also gives HTTP/2, access
  logs, connection limits) or cpp-httplib's OpenSSL build for the client
  (`CPPHTTPLIB_OPENSSL_SUPPORT`, a new pinned dependency and a CA-bundle
  policy).
- **Auth and quotas.** `auth_mode`/`rate_limit` are advertised in the server
  manifest but nothing enforces them: API keys or tokens, per-key rate
  limits answered as 429 (the client already maps 429 to a 503 retry).
- **Upstream chaining.** An HTTP `IFetcher`/`IUpstream` so a local server
  can fall back to a hosted one (today a networked upstream is only an
  interface); offline mode stays the default.
- **Long derivations.** A cold derivation holds the request open (read
  timeout 120 s). A hosted server wants "503 + Retry-After while queued"
  and background derivation instead.
- **CDN.** The `Cache-Control` above is already CDN-safe; `tile_base_url` in
  the manifest can point at static packs, which needs an upload/bake path.
