// src/http/http_wire.cpp - see g2m/http/http_wire.h and docs/protocol/README.md.
// No socket and no third-party type: compiled into g2m_http and, again, into
// the fuzz build (tests/fuzz_formats).

#include "g2m/http/http_wire.h"

#include "g2m/layer/layer_id.h"

#include <algorithm>
#include <charconv>
#include <limits>

namespace g2m::http {
namespace {

char lower(char c) { return (c >= 'A' && c <= 'Z') ? static_cast<char>(c - 'A' + 'a') : c; }

bool iequals(std::string_view a, std::string_view b) {
    if (a.size() != b.size()) return false;
    for (std::size_t i = 0; i < a.size(); ++i) {
        if (lower(a[i]) != lower(b[i])) return false;
    }
    return true;
}

std::string_view trim(std::string_view s) {
    while (!s.empty() && (s.front() == ' ' || s.front() == '\t')) s.remove_prefix(1);
    while (!s.empty() && (s.back() == ' ' || s.back() == '\t')) s.remove_suffix(1);
    return s;
}

// "application/vnd.g2m.pack; x=y" -> "application/vnd.g2m.pack", compared case-insensitively.
bool media_type_is(std::optional<std::string_view> value, std::string_view type) {
    if (!value) return false;
    std::string_view v = *value;
    const std::size_t semi = v.find(';');
    if (semi != std::string_view::npos) v = v.substr(0, semi);
    std::string_view t = type;
    const std::size_t tsemi = t.find(';');
    if (tsemi != std::string_view::npos) t = t.substr(0, tsemi);
    return iequals(trim(v), trim(t));
}

int hex_digit(char c) {
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Canonical decimal u32 (delta-seconds of Retry-After): digits only, no
// leading zero except "0"; values above u32 max clamp.
std::optional<std::uint32_t> parse_seconds(std::string_view s) {
    s = trim(s);
    if (s.empty() || s.size() > 20) return std::nullopt;
    std::uint64_t v = 0;
    for (char c : s) {
        if (c < '0' || c > '9') return std::nullopt;
        v = std::min<std::uint64_t>(v * 10 + static_cast<std::uint64_t>(c - '0'), std::numeric_limits<std::uint32_t>::max());
    }
    return static_cast<std::uint32_t>(v);
}

std::vector<std::uint8_t> bytes_of(std::string_view s) { return std::vector<std::uint8_t>(s.begin(), s.end()); }

std::string_view text_of(const std::vector<std::uint8_t>& b) {
    return std::string_view(reinterpret_cast<const char*>(b.data()), b.size());
}

ResponseMeta& meta_ref(Response& r) {
    return std::visit([](auto& x) -> ResponseMeta& { return x.meta; }, r);
}

// An empty response of the kind that answers `request`.
Response empty_response_for(const Request& request) {
    struct V {
        Response operator()(const ManifestRequest&) const { return ManifestResponse{}; }
        Response operator()(const ReleaseManifestRequest&) const { return ReleaseManifestResponse{}; }
        Response operator()(const TileRequest&) const { return TileResponse{}; }
        Response operator()(const PackRequest&) const { return PackResponse{}; }
        Response operator()(const HashesRequest&) const { return HashesResponse{}; }
    };
    return std::visit(V{}, request);
}

Response internal_error(const Request& request, std::string message) {
    Response r = empty_response_for(request);
    ResponseMeta& m = meta_ref(r);
    m.status = Status::Internal;
    m.message = "http: " + std::move(message);
    return r;
}

std::optional<Status> protocol_status(int code) {
    switch (code) {
    case 200: return Status::Ok;
    case 304: return Status::NotModified;
    case 400: return Status::BadRequest;
    case 404: return Status::NotFound;
    case 410: return Status::Gone;
    case 500: return Status::Internal;
    case 503: return Status::Unavailable;
    default: return std::nullopt;
    }
}

const char* cache_control_for(const Request& request, Status s) {
    if (s == Status::Ok || s == Status::NotModified) {
        if (std::holds_alternative<ManifestRequest>(request)) return "public, max-age=300";
        return "public, max-age=31536000, immutable";
    }
    if (s == Status::NotFound || s == Status::Gone) return "public, max-age=300";
    return "no-store";
}

void add(std::vector<Header>& h, std::string_view name, std::string value) {
    h.push_back(Header{std::string(name), std::move(value)});
}

void add_meta_headers(std::vector<Header>& h, const ResponseMeta& m) {
    if (m.etag) add(h, "ETag", etag_value(*m.etag));
    if (m.retry_after_s > 0) add(h, "Retry-After", std::to_string(m.retry_after_s));
    if (m.degraded) add(h, kHeaderDegraded, "1");
    if (m.newest_rid) add(h, kHeaderNewestRid, to_hex(*m.newest_rid));
    if (!m.message.empty()) add(h, kHeaderMessage, encode_header_text(m.message));
}

bool layer_key_matches(const std::string& layer, const ReleaseId& rid, const TileKey& key, const PackDirectory& d) {
    return d.layer == layer && d.rid == rid && d.pack == key;
}

} // namespace

// ---- Message ----

std::optional<std::string_view> Message::header(std::string_view name) const {
    for (const Header& h : headers) {
        if (iequals(h.name, name)) return std::string_view(h.value);
    }
    return std::nullopt;
}

std::size_t Message::header_count(std::string_view name) const {
    return static_cast<std::size_t>(
        std::count_if(headers.begin(), headers.end(), [&](const Header& h) { return iequals(h.name, name); }));
}

// ---- small codecs ----

std::string encode_header_text(std::string_view text) {
    static constexpr char kHex[] = "0123456789ABCDEF";
    std::string out;
    out.reserve(text.size());
    for (char ch : text) {
        const auto c = static_cast<unsigned char>(ch);
        if (c >= 0x20 && c <= 0x7e && c != '%') {
            out.push_back(ch);
        } else {
            out.push_back('%');
            out.push_back(kHex[c >> 4]);
            out.push_back(kHex[c & 15]);
        }
    }
    return out;
}

std::optional<std::string> decode_header_text(std::string_view text) {
    // Strict: only what encode_header_text writes (upper-case escapes, and
    // only of bytes that need one), so the two are exact inverses.
    std::string out;
    out.reserve(text.size());
    for (std::size_t i = 0; i < text.size(); ++i) {
        const auto c = static_cast<unsigned char>(text[i]);
        if (c != '%') {
            if (c < 0x20 || c > 0x7e) return std::nullopt;
            out.push_back(text[i]);
            continue;
        }
        if (i + 2 >= text.size()) return std::nullopt;
        const char a = text[i + 1], b = text[i + 2];
        if ((a >= 'a' && a <= 'f') || (b >= 'a' && b <= 'f')) return std::nullopt;
        const int hi = hex_digit(a);
        const int lo = hex_digit(b);
        if (hi < 0 || lo < 0) return std::nullopt;
        const int v = hi * 16 + lo;
        if (v >= 0x20 && v <= 0x7e && v != '%') return std::nullopt;
        out.push_back(static_cast<char>(v));
        i += 2;
    }
    return out;
}

std::string etag_value(const ContentHash& h) { return "\"" + to_hex(h) + "\""; }

std::optional<ContentHash> parse_etag(std::string_view value) {
    value = trim(value);
    if (value.size() != 66 || value.front() != '"' || value.back() != '"') return std::nullopt;
    return content_hash_from_hex(value.substr(1, 64));
}

bool is_valid_base_path(std::string_view p) {
    if (p.empty()) return true;
    if (p.size() > 256 || p.front() != '/') return false;
    std::size_t start = 1;
    while (true) {
        const std::size_t slash = p.find('/', start);
        const std::string_view seg = p.substr(start, slash == std::string_view::npos ? p.size() - start : slash - start);
        if (seg.empty() || seg == "." || seg == "..") return false;
        for (char c : seg) {
            const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '.' ||
                            c == '_' || c == '~' || c == '-';
            if (!ok) return false;
        }
        if (slash == std::string_view::npos) return true;
        start = slash + 1;
    }
}

Result<HttpUrl> parse_http_url(std::string_view url) {
    const auto bad = [&](const char* why) {
        return make_error(ErrorCode::InvalidArgument, "http url '" + std::string(url) + "': " + why);
    };
    constexpr std::string_view kScheme = "http://";
    if (url.size() < kScheme.size() || !iequals(url.substr(0, kScheme.size()), kScheme)) {
        return bad("only http:// is supported (TLS comes with a reverse proxy, PLAN.md 3.7)");
    }
    std::string_view rest = url.substr(kScheme.size());
    const std::size_t slash = rest.find('/');
    std::string_view authority = rest.substr(0, slash);
    std::string_view path = slash == std::string_view::npos ? std::string_view{} : rest.substr(slash);
    while (path.size() > 1 && path.back() == '/') path.remove_suffix(1);
    if (path == "/") path = {};
    if (path.find_first_of("?#") != std::string_view::npos) return bad("query or fragment");
    if (!is_valid_base_path(path)) return bad("bad base path");
    if (authority.empty() || authority.find('@') != std::string_view::npos) return bad("bad authority");
    HttpUrl out;
    std::string_view port;
    if (authority.front() == '[') {
        const std::size_t close = authority.find(']');
        if (close == std::string_view::npos || close < 2) return bad("bad IPv6 literal");
        out.host = std::string(authority.substr(1, close - 1));
        for (char c : out.host) {
            if (hex_digit(c) < 0 && c != ':' && c != '.') return bad("bad IPv6 literal");
        }
        const std::string_view after = authority.substr(close + 1);
        if (!after.empty()) {
            if (after.front() != ':') return bad("bad authority");
            port = after.substr(1);
        }
    } else {
        const std::size_t colon = authority.find(':');
        const std::string_view host = authority.substr(0, colon);
        if (colon != std::string_view::npos) port = authority.substr(colon + 1);
        if (host.empty() || host.size() > 253) return bad("bad host");
        for (char c : host) {
            const bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '.' || c == '-';
            if (!ok) return bad("bad host");
        }
        out.host = std::string(host);
    }
    if (!port.empty() || authority.back() == ':') {
        int p = 0;
        const auto [end, ec] = std::from_chars(port.data(), port.data() + port.size(), p);
        if (port.empty() || ec != std::errc{} || end != port.data() + port.size() || p < 1 || p > 65535) return bad("bad port");
        out.port = p;
    }
    out.base_path = std::string(path);
    return out;
}

// ---- client side ----

std::string request_target(std::string_view base_path, const Request& request) {
    return std::string(base_path) + std::string(kApiPath) + to_path(request);
}

std::vector<Header> request_headers(const Request& request) {
    std::vector<Header> h;
    if (const auto* t = std::get_if<TileRequest>(&request); t && t->if_none_match) {
        add(h, "If-None-Match", etag_value(*t->if_none_match));
    }
    return h;
}

Response decode_response(const Request& request, const Message& message, const DecodeOptions& options) {
    Response out = empty_response_for(request);
    ResponseMeta& m = meta_ref(out);

    // Status: protocol v1 statuses map 1:1; everything else as the spec says.
    std::string prefix;
    if (const auto s = protocol_status(message.status)) {
        m.status = *s;
    } else if (message.status == 429 || (message.status >= 500 && message.status <= 599)) {
        m.status = Status::Unavailable;
        prefix = "http " + std::to_string(message.status);
    } else if (message.status >= 400 && message.status <= 499) {
        m.status = Status::BadRequest;
        prefix = "http " + std::to_string(message.status);
    } else {
        return internal_error(request, "unexpected status " + std::to_string(message.status));
    }

    // Meta headers. A malformed g2m header is a protocol violation (500);
    // Retry-After in HTTP-date form (RFC 9110) is ignored.
    if (const auto v = message.header(kHeaderMessage)) {
        auto text = decode_header_text(*v);
        m.message = text ? std::move(*text) : std::string(*v);
    }
    if (!prefix.empty()) m.message = m.message.empty() ? prefix : prefix + ": " + m.message;
    if (const auto v = message.header("Retry-After")) {
        if (const auto s = parse_seconds(*v)) m.retry_after_s = *s;
    }
    if (const auto v = message.header(kHeaderDegraded)) {
        if (trim(*v) != "1") return internal_error(request, "bad " + std::string(kHeaderDegraded) + " header");
        m.degraded = true;
    }
    if (const auto v = message.header(kHeaderNewestRid)) {
        const auto rid = content_hash_from_hex(trim(*v));
        if (!rid) return internal_error(request, "bad " + std::string(kHeaderNewestRid) + " header");
        m.newest_rid = *rid;
    }
    if (const auto v = message.header("ETag")) {
        const auto tag = parse_etag(*v);
        if (!tag) return internal_error(request, "bad ETag header");
        m.etag = *tag;
    }
    if (m.status == Status::Unavailable && !m.degraded && m.retry_after_s == 0) {
        m.retry_after_s = std::max<std::uint32_t>(1, options.default_retry_after_s);
    }

    // Payload.
    const auto content_type = message.header("Content-Type");
    struct V {
        const Message& msg;
        ResponseMeta& meta;
        std::optional<std::string_view> type;
        std::optional<std::string> operator()(ManifestResponse& r) const {
            if (meta.status != Status::Ok) return std::nullopt;
            auto j = server_manifest_from_json(text_of(msg.body));
            if (!j) return "server manifest: " + j.error().message;
            r.manifest = std::move(*j);
            return std::nullopt;
        }
        std::optional<std::string> operator()(ReleaseManifestResponse& r, const ReleaseManifestRequest& q) const {
            if (meta.status != Status::Ok) return std::nullopt;
            auto j = release_manifest_from_json(text_of(msg.body));
            if (!j) return "release manifest: " + j.error().message;
            if (j->rid != q.rid) return std::string("release manifest of another release");
            r.manifest = std::move(*j);
            return std::nullopt;
        }
        std::optional<std::string> operator()(TileResponse& r, const TileRequest& q) const {
            if (meta.status != Status::Ok) return std::nullopt;
            auto v = parse_container(msg.body);
            if (!v) return "tile: " + v.error().message;
            const TileHeader& h = v->header;
            if (h.layer != q.layer || h.key != q.key || h.rid != q.rid) return std::string("tile: container of another tile");
            if (meta.etag && *meta.etag != h.body_hash) return std::string("tile: ETag does not match the body hash");
            meta.etag = h.body_hash;
            r.container = msg.body;
            return std::nullopt;
        }
        std::optional<std::string> operator()(PackResponse& r, const PackRequest& q) const {
            if (!media_type_is(type, kTypePack)) return std::nullopt;
            auto v = decode_pack(msg.body);
            if (!v) return "pack: " + v.error().message;
            if (!layer_key_matches(q.layer, q.rid, q.pack, v->dir)) return std::string("pack: another pack");
            r.pack = msg.body;
            return std::nullopt;
        }
        std::optional<std::string> operator()(HashesResponse& r, const HashesRequest& q) const {
            if (!media_type_is(type, kTypeHashes)) return std::nullopt;
            auto d = decode_pack_directory(msg.body);
            if (!d) return "hashes: " + d.error().message;
            if (!layer_key_matches(q.layer, q.rid, q.pack, *d)) return std::string("hashes: another pack");
            r.directory = std::move(*d);
            return std::nullopt;
        }
    };
    const V v{message, m, content_type};
    std::optional<std::string> err;
    if (auto* r = std::get_if<ManifestResponse>(&out)) err = v(*r);
    else if (auto* r = std::get_if<ReleaseManifestResponse>(&out)) err = v(*r, std::get<ReleaseManifestRequest>(request));
    else if (auto* r = std::get_if<TileResponse>(&out)) err = v(*r, std::get<TileRequest>(request));
    else if (auto* r = std::get_if<PackResponse>(&out)) err = v(*r, std::get<PackRequest>(request));
    else if (auto* r = std::get_if<HashesResponse>(&out)) err = v(*r, std::get<HashesRequest>(request));
    if (err) return internal_error(request, std::move(*err));
    return out;
}

Response transport_failure(const Request& request, std::string what, std::uint32_t retry_after_s) {
    Response r = empty_response_for(request);
    ResponseMeta& m = meta_ref(r);
    m.status = Status::Unavailable;
    m.retry_after_s = std::max<std::uint32_t>(1, retry_after_s);
    m.message = "http: " + std::move(what);
    return r;
}

// ---- server side ----

Result<Request> parse_request(std::string_view base_path, std::string_view method, std::string_view target,
                              const std::vector<std::string_view>& if_none_match) {
    const auto malformed = [](std::string why) { return make_error(ErrorCode::Malformed, std::move(why)); };
    if (target.size() > kMaxTargetLength) return malformed("request target too long");
    const std::string prefix = std::string(base_path) + std::string(kApiPath);
    if (!target.starts_with(prefix) || (target.size() > prefix.size() && target[prefix.size()] != '/')) {
        return make_error(ErrorCode::NotFound, "not a protocol v1 path (expected " + prefix + "/...)");
    }
    if (method != "GET" && method != "HEAD") return make_error(ErrorCode::Unsupported, "only GET and HEAD are served");
    const std::string_view rest = target.substr(prefix.size());
    auto parsed = parse_path(rest);
    if (!parsed) return malformed("malformed protocol v1 path: " + parsed.error().message);
    if (to_path(*parsed) != rest) return malformed("non-canonical protocol v1 path"); // parse_path's own contract; kept explicit
    if (if_none_match.size() > 1) return malformed("more than one If-None-Match header");
    if (!if_none_match.empty()) {
        auto* tile = std::get_if<TileRequest>(&*parsed);
        if (tile) {
            const auto tag = parse_etag(if_none_match.front());
            if (!tag) return malformed("If-None-Match: expected one strong ETag \"<64 hex>\"");
            tile->if_none_match = *tag;
        }
        // Other endpoints ignore it (their bodies are not addressed by one hash).
    }
    return std::move(*parsed);
}

Message error_response(int status, std::string_view message) {
    Message out;
    out.status = status;
    add(out.headers, "Cache-Control", "no-store");
    if (!message.empty()) add(out.headers, kHeaderMessage, encode_header_text(message));
    if (status == 405) add(out.headers, "Allow", "GET, HEAD");
    add(out.headers, "Content-Type", std::string(kTypeText));
    out.body = bytes_of(std::string(message) + "\n");
    return out;
}

Message encode_response(const Request& request, const Response& response) {
    const ResponseMeta& m = meta_of(response);
    Message out;
    out.status = static_cast<int>(m.status);
    add(out.headers, "Cache-Control", cache_control_for(request, m.status));
    add_meta_headers(out.headers, m);

    std::string_view type;
    std::vector<std::uint8_t> body;
    struct V {
        const ResponseMeta& m;
        std::string_view& type;
        std::vector<std::uint8_t>& body;
        void operator()(const ManifestResponse& r) const {
            if (m.status != Status::Ok) return;
            type = kTypeJson;
            body = bytes_of(to_json(r.manifest));
        }
        void operator()(const ReleaseManifestResponse& r) const {
            if (m.status != Status::Ok) return;
            type = kTypeJson;
            body = bytes_of(to_json(r.manifest));
        }
        void operator()(const TileResponse& r) const {
            if (m.status != Status::Ok) return;
            type = kTypeTile;
            body = r.container;
        }
        void operator()(const PackResponse& r) const {
            if (r.pack.empty()) return;
            type = kTypePack;
            body = r.pack;
        }
        void operator()(const HashesResponse& r) const {
            if (r.directory.layer.empty()) return; // the server had no directory (400/404/410)
            auto d = encode_pack_directory(r.directory);
            if (!d) return;
            type = kTypeHashes;
            body = std::move(*d);
        }
    };
    std::visit(V{m, type, body}, response);

    // A hashes directory that does not encode is a server bug: answer 500.
    if (const auto* h = std::get_if<HashesResponse>(&response); h && !h->directory.layer.empty() && type.empty()) {
        return error_response(500, "hashes: the pack directory does not encode");
    }
    if (m.status == Status::NotModified) return out; // no body, no Content-Type
    if (type.empty()) {
        type = kTypeText;
        body = bytes_of(m.message.empty() ? std::to_string(out.status) + "\n" : m.message + "\n");
    }
    add(out.headers, "Content-Type", std::string(type));
    out.body = std::move(body);
    return out;
}

Message handle_request(ITransport& backend, std::string_view base_path, std::string_view method,
                       std::string_view target, const std::vector<std::string_view>& if_none_match) {
    auto req = parse_request(base_path, method, target, if_none_match);
    if (!req) {
        const ErrorCode c = req.error().code;
        const int status = c == ErrorCode::NotFound ? 404 : c == ErrorCode::Unsupported ? 405 : 400;
        return error_response(status, req.error().message);
    }
    const Response resp = backend.send(*req);
    return encode_response(*req, resp);
}

} // namespace g2m::http
