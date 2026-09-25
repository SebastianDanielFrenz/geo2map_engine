// g2m/server/protocol.h - protocol v1 (PLAN.md 3.4) as in-process request
// and response objects. G6's HTTP transport serialises exactly these; the
// in-process transport passes them as they are.
//
// Endpoints (under {base}/g2m/v1), and their objects:
//   GET /manifest                                          ManifestRequest       -> ManifestResponse
//   GET /releases/{rid}/manifest                           ReleaseManifestRequest-> ReleaseManifestResponse
//   GET /releases/{rid}/tiles/{layer}/{scheme}/{l}/{x}/{y} TileRequest           -> TileResponse
//   GET /releases/{rid}/packs/{layer}/{scheme}/{l}/{x}/{y} PackRequest           -> PackResponse
//   GET /releases/{rid}/hashes/{layer}/{scheme}/{l}/{x}/{y} HashesRequest        -> HashesResponse
// For packs and hashes, x/y are PACK coordinates (tile x / 16, floored).
//
// Status semantics:
//   200 Ok           the data is in the response.
//   304 NotModified  a tile request's if_none_match equals the body hash.
//   400 BadRequest   malformed request (bad layer name, key, level...).
//   404 NotFound     definitive for the release: unknown release or layer,
//                    a level the layer does not have, or outside coverage.
//                    Also a tile inside coverage that no store, deriver or
//                    upstream has.
//   410 Gone         the release was retired; newest_rid names its successor.
//   503 Unavailable  not available NOW. With degraded = true: the server is
//                    offline and nothing is cached (retrying will not help
//                    until it goes online; retry_after_s = 0). Otherwise
//                    retry_after_s > 0: an upstream or derivation is busy
//                    or failed transiently; retry later.
//   500 Internal     a server-side failure (store I/O, a deriver error).
// A pack or hashes response is 200 when every tile of the pack is present
// or definitively absent, 404 when all 256 are absent, and 503 otherwise -
// a 503 pack still carries every tile that was available, and its
// directory says which ones were not (PackTileState).
#pragma once

#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/pack.h"
#include "g2m/layer/release.h"
#include "g2m/layer/tile_container.h"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <variant>
#include <vector>

namespace g2m {

inline constexpr std::uint32_t kProtocolVersion = 1;

enum class Status : std::uint16_t {
    Ok = 200,
    NotModified = 304,
    BadRequest = 400,
    NotFound = 404,
    Gone = 410,
    Internal = 500,
    Unavailable = 503,
};

struct ResponseMeta {
    Status status = Status::Ok;
    bool degraded = false;           // 503 only: offline and not cached
    std::uint32_t retry_after_s = 0; // 503 only, when retrying may help
    std::optional<ContentHash> etag; // tiles: the body hash
    std::optional<ReleaseId> newest_rid; // 410 only
    std::string message;             // for humans / logs
};

// ---- GET /manifest ----
struct ServerManifest {
    std::string server_id;
    std::vector<std::uint32_t> protocol_versions{kProtocolVersion};
    ReleaseId default_rid{};
    std::vector<ReleaseId> live_releases; // sorted
    std::string tile_base_url;            // empty: same base
    std::string auth_mode = "none";
    std::string rate_limit;               // empty: none
    std::string contact;
    friend bool operator==(const ServerManifest&, const ServerManifest&) = default;
};
// Canonical JSON (sorted keys, no whitespace), and a strict parse.
std::string to_json(const ServerManifest& m);
Result<ServerManifest> server_manifest_from_json(std::string_view json);

struct ManifestRequest {
    friend bool operator==(const ManifestRequest&, const ManifestRequest&) = default;
};
struct ManifestResponse {
    ResponseMeta meta;
    ServerManifest manifest;
};

// ---- GET /releases/{rid}/manifest ----
struct ReleaseManifestRequest {
    ReleaseId rid{};
    friend bool operator==(const ReleaseManifestRequest&, const ReleaseManifestRequest&) = default;
};
struct ReleaseManifestResponse {
    ResponseMeta meta;
    ReleaseManifest manifest; // to_json() gives the wire form
};

// ---- GET /releases/{rid}/tiles/... ----
struct TileRequest {
    ReleaseId rid{};
    std::string layer;
    TileKey key;
    std::optional<ContentHash> if_none_match;
    friend bool operator==(const TileRequest&, const TileRequest&) = default;
};
struct TileResponse {
    ResponseMeta meta;
    std::vector<std::uint8_t> container; // 200 only: a complete g2m.tile/1 container
};

// ---- GET /releases/{rid}/packs/... and /hashes/... ----
struct PackRequest {
    ReleaseId rid{};
    std::string layer;
    TileKey pack; // pack coordinates (pack.h)
    friend bool operator==(const PackRequest&, const PackRequest&) = default;
};
struct PackResponse {
    ResponseMeta meta;
    std::vector<std::uint8_t> pack; // encode_pack() bytes; empty on 4xx/5xx without data
};
struct HashesRequest {
    ReleaseId rid{};
    std::string layer;
    TileKey pack;
    friend bool operator==(const HashesRequest&, const HashesRequest&) = default;
};
struct HashesResponse {
    ResponseMeta meta;
    PackDirectory directory;
};

using Request = std::variant<ManifestRequest, ReleaseManifestRequest, TileRequest, PackRequest, HashesRequest>;
using Response = std::variant<ManifestResponse, ReleaseManifestResponse, TileResponse, PackResponse, HashesResponse>;

const ResponseMeta& meta_of(const Response& r);

// The request's path relative to {base}/g2m/v1, e.g.
// "/releases/<64 hex>/tiles/g2m.src.osm/utm32n/2/350/5450". parse_path is
// its exact inverse; it rejects anything else (fuzzed).
std::string to_path(const Request& r);
Result<Request> parse_path(std::string_view path);

} // namespace g2m
