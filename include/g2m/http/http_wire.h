// g2m/http/http_wire.h - the HTTP binding of protocol v1 (G6,
// docs/protocol/README.md) as pure functions: no socket, no third-party
// type. HttpTransport (client) and HttpServer (front end) are thin shells
// around these; tests and the fuzzers call them directly.
//
// A request is "GET {base_path}/g2m/v1{to_path(request)}" (protocol.h), plus
// "If-None-Match: \"<64 hex>\"" for a tile request with if_none_match. The
// response's HTTP status is the protocol Status; ResponseMeta travels in
// headers (ETag, Retry-After, X-G2M-Degraded, X-G2M-Newest-Rid,
// X-G2M-Message) and the payload in the body:
//   manifest          application/json          to_json(ServerManifest)
//   release manifest  application/json          to_json(ReleaseManifest)   (200 only)
//   tile              application/vnd.g2m.tile  the g2m.tile/1 container   (200 only)
//   pack              application/vnd.g2m.pack  encode_pack() bytes        (whenever the server has them: 200, 503)
//   hashes            application/vnd.g2m.hashes encode_pack_directory()   (whenever the server has one: 200, 503)
// A response without payload carries the message as text/plain (304: no body).
//
// Everything here returns by value and never throws.
#pragma once

#include "g2m/core/result.h"
#include "g2m/server/protocol.h"
#include "g2m/server/transport.h"

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace g2m::http {

inline constexpr std::string_view kApiPath = "/g2m/v1";

inline constexpr std::string_view kTypeJson = "application/json";
inline constexpr std::string_view kTypeTile = "application/vnd.g2m.tile";
inline constexpr std::string_view kTypePack = "application/vnd.g2m.pack";
inline constexpr std::string_view kTypeHashes = "application/vnd.g2m.hashes";
inline constexpr std::string_view kTypeText = "text/plain; charset=utf-8";

inline constexpr std::string_view kHeaderMessage = "X-G2M-Message";
inline constexpr std::string_view kHeaderDegraded = "X-G2M-Degraded";
inline constexpr std::string_view kHeaderNewestRid = "X-G2M-Newest-Rid";

// The longest request target the front end accepts (a tile path is ~130).
inline constexpr std::size_t kMaxTargetLength = 1024;

struct Header {
    std::string name;
    std::string value;
    friend bool operator==(const Header&, const Header&) = default;
};

// One HTTP response as data.
struct Message {
    int status = 0;
    std::vector<Header> headers;
    std::vector<std::uint8_t> body;

    // First header of that name (ASCII case-insensitive), if any.
    std::optional<std::string_view> header(std::string_view name) const;
    std::size_t header_count(std::string_view name) const;
};

// ---- small codecs (exposed for tests) ----

// Header-safe text: bytes outside 0x20..0x7e and '%' become %XX (upper-case
// hex). decode_header_text is the exact inverse; it rejects a bad escape.
std::string encode_header_text(std::string_view text);
std::optional<std::string> decode_header_text(std::string_view text);

// ETag / If-None-Match value: a strong entity tag "\"<64 lower-case hex>\"".
std::string etag_value(const ContentHash& h);
std::optional<ContentHash> parse_etag(std::string_view value);

// "" or "/seg(/seg)*", each segment [A-Za-z0-9._~-]+ and not "." or "..".
bool is_valid_base_path(std::string_view base_path);

// An "http://host[:port][/base]" URL (TLS / https is G7). host is a DNS name,
// an IPv4 address or a bracketed IPv6 literal (brackets kept off `host`).
struct HttpUrl {
    std::string host;
    int port = 80;
    std::string base_path; // is_valid_base_path
    friend bool operator==(const HttpUrl&, const HttpUrl&) = default;
};
Result<HttpUrl> parse_http_url(std::string_view url);

// ---- client side ----

std::string request_target(std::string_view base_path, const Request& request);
std::vector<Header> request_headers(const Request& request);

struct DecodeOptions {
    // A 503 (or 429 / other 5xx) without Retry-After and not degraded gets
    // this many seconds, so a client never retries in a tight loop.
    std::uint32_t default_retry_after_s = 5;
};

// The protocol response for `request` from an HTTP response. Never fails:
// a payload that does not decode, or does not answer this request (another
// layer, key or release), becomes 500 Internal with a message; an HTTP
// status outside protocol v1 maps as docs/protocol/README.md says (429 and
// 5xx -> 503, other 4xx -> 400, anything else -> 500).
Response decode_response(const Request& request, const Message& message, const DecodeOptions& options = {});

// The response for a request that never got an HTTP answer (connection
// refused, timeout, reset): 503 with retry_after_s, message `what`.
Response transport_failure(const Request& request, std::string what, std::uint32_t retry_after_s);

// ---- server side ----

// Parses a request line's method and raw request target plus the
// If-None-Match header values. Errors (the front end's status):
//   ErrorCode::NotFound     not under {base_path}/g2m/v1      -> 404
//   ErrorCode::Unsupported  a method other than GET / HEAD    -> 405
//   ErrorCode::Malformed    anything else not canonical       -> 400
// The target must be exactly base_path + kApiPath + to_path(request): no
// query, no fragment, no percent escapes, no trailing slash.
Result<Request> parse_request(std::string_view base_path, std::string_view method, std::string_view target,
                              const std::vector<std::string_view>& if_none_match);

Message encode_response(const Request& request, const Response& response);

// The front end's own error answer (a status the request parser chose).
Message error_response(int status, std::string_view message);

// parse_request, then backend.send(), then encode_response (or
// error_response on a parse failure).
Message handle_request(ITransport& backend, std::string_view base_path, std::string_view method,
                       std::string_view target, const std::vector<std::string_view>& if_none_match);

} // namespace g2m::http
