// g2m_http's pure wire mapping (http_wire.h): header text, ETags, base
// paths, URLs, request parsing, and response encode/decode without a socket.

#include "g2m/http/http_wire.h"

#include "http_test_world.h"

#include <catch2/catch_test_macros.hpp>

using namespace g2m;
using namespace g2m::http;
using namespace g2m_http_test;

namespace {

ContentHash hash_of(std::uint8_t seed) {
    ContentHash h{};
    for (std::size_t i = 0; i < h.size(); ++i) h[i] = static_cast<std::uint8_t>(seed * 31 + i);
    return h;
}

// A backend that answers every request with one scripted response.
class Scripted final : public ITransport {
public:
    Response next;
    Response send(const Request&) override { return next; }
};

} // namespace

TEST_CASE("http wire: header text escaping round-trips every byte and is strict", "[http][wire]") {
    std::string all;
    for (int c = 0; c < 256; ++c) all.push_back(static_cast<char>(c));
    const std::string enc = encode_header_text(all);
    for (char c : enc) CHECK((c >= 0x20 && c <= 0x7e));
    const auto dec = decode_header_text(enc);
    REQUIRE(dec);
    CHECK(*dec == all);
    CHECK(encode_header_text("no escapes: [] {} ~") == "no escapes: [] {} ~");
    CHECK(encode_header_text("100%\r\n") == "100%25%0D%0A");
    CHECK_FALSE(decode_header_text("%"));
    CHECK_FALSE(decode_header_text("%4"));
    CHECK_FALSE(decode_header_text("%zz"));
    CHECK_FALSE(decode_header_text("%0d"));       // lower-case hex: not what the encoder writes
    CHECK_FALSE(decode_header_text("%41"));       // 'A' needs no escape
    CHECK_FALSE(decode_header_text("tab\there")); // raw control byte
    CHECK(decode_header_text("%25") == std::optional<std::string>("%"));
}

TEST_CASE("http wire: ETags, base paths and URLs", "[http][wire]") {
    const ContentHash h = hash_of(3);
    CHECK(parse_etag(etag_value(h)) == h);
    CHECK(parse_etag(" " + etag_value(h) + " ") == h);
    CHECK_FALSE(parse_etag(to_hex(h)));               // unquoted
    CHECK_FALSE(parse_etag("W/" + etag_value(h)));    // weak
    CHECK_FALSE(parse_etag("*"));
    std::string upper = etag_value(h);
    for (char& c : upper) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    CHECK_FALSE(parse_etag(upper));

    CHECK(is_valid_base_path(""));
    CHECK(is_valid_base_path("/maps"));
    CHECK(is_valid_base_path("/a/b-c/d.e_f~"));
    CHECK_FALSE(is_valid_base_path("/"));
    CHECK_FALSE(is_valid_base_path("maps"));
    CHECK_FALSE(is_valid_base_path("/a//b"));
    CHECK_FALSE(is_valid_base_path("/a/"));
    CHECK_FALSE(is_valid_base_path("/a/../b"));
    CHECK_FALSE(is_valid_base_path("/a%20b"));
    CHECK_FALSE(is_valid_base_path("/a?b"));

    CHECK(parse_http_url("http://127.0.0.1:8080").value() == HttpUrl{"127.0.0.1", 8080, ""});
    CHECK(parse_http_url("http://localhost").value() == HttpUrl{"localhost", 80, ""});
    CHECK(parse_http_url("HTTP://maps.example.org/base/v/").value() == HttpUrl{"maps.example.org", 80, "/base/v"});
    CHECK(parse_http_url("http://[::1]:9/x").value() == HttpUrl{"::1", 9, "/x"});
    for (const char* bad : {"https://x", "ftp://x", "http://", "http://:80", "http://a:0", "http://a:65536", "http://a:",
                            "http://a:8x", "http://u@a", "http://a/b?c", "http://a/b#c", "http://[::1", "http://a b",
                            "http://a//b", "x"}) {
        INFO(bad);
        CHECK_FALSE(parse_http_url(bad));
    }
}

TEST_CASE("http wire: request targets parse only in canonical form", "[http][wire]") {
    World w;
    for (const Request& r : w.requests()) {
        const std::string t = request_target("", r);
        auto p = parse_request("", "GET", t, {});
        REQUIRE(p);
        CHECK(*p == r);
        CHECK(parse_request("", "HEAD", t, {}).ok());
        // Under a base path.
        auto q = parse_request("/maps/x", "GET", request_target("/maps/x", r), {});
        REQUIRE(q);
        CHECK(*q == r);
        CHECK(parse_request("/maps/x", "GET", t, {}).error().code == ErrorCode::NotFound);
        CHECK(parse_request("", "POST", t, {}).error().code == ErrorCode::Unsupported);
    }
    const ContentHash h = hash_of(9);
    const TileRequest tile{w.release.rid, kLc, home_tile(), std::nullopt};
    const std::string t = request_target("", Request{tile});
    const std::string tag = etag_value(h);
    auto p = parse_request("", "GET", t, {tag});
    REQUIRE(p);
    CHECK(std::get<TileRequest>(*p).if_none_match == h);
    CHECK(request_headers(*p) == std::vector<Header>{{"If-None-Match", tag}});
    CHECK(parse_request("", "GET", t, {"*"}).error().code == ErrorCode::Malformed);
    CHECK(parse_request("", "GET", t, {tag, tag}).error().code == ErrorCode::Malformed);
    // If-None-Match on a non-tile endpoint is ignored.
    CHECK(parse_request("", "GET", "/g2m/v1/manifest", {tag}).ok());

    const std::string rid = to_hex(w.release.rid);
    std::string upper_rid = rid;
    for (char& c : upper_rid) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    struct Case {
        std::string target;
        ErrorCode code;
    };
    const std::vector<Case> cases = {
        {"/", ErrorCode::NotFound},
        {"/index.html", ErrorCode::NotFound},
        {"/g2m/v2/manifest", ErrorCode::NotFound},
        {"/g2m/v1x/manifest", ErrorCode::NotFound},
        {"/g2m/v1", ErrorCode::Malformed},
        {"/g2m/v1/", ErrorCode::Malformed},
        {"/g2m/v1/manifest/", ErrorCode::Malformed},
        {"/g2m/v1/manifest?x=1", ErrorCode::Malformed},
        {"/g2m/v1/manifest#f", ErrorCode::Malformed},
        {"/g2m/v1//manifest", ErrorCode::Malformed},
        {"/g2m/v1/%6danifest", ErrorCode::Malformed},
        {"/g2m/v1/releases/" + upper_rid + "/manifest", ErrorCode::Malformed},
        {"/g2m/v1/releases/" + rid + "/tiles/g2m.src.osm/utm32n/2/0350/5450", ErrorCode::Malformed},
        {"/g2m/v1/releases/" + rid + "/tiles/g2m.src.osm/utm32n/2/+350/5450", ErrorCode::Malformed},
        {"/g2m/v1/releases/" + rid + "/tiles/G2M.src.osm/utm32n/2/350/5450", ErrorCode::Malformed},
        {"/g2m/v1/releases/" + rid + "/tiles/g2m.src.osm/utm32x/2/350/5450", ErrorCode::Malformed},
        {"/g2m/v1/releases/" + rid + "/tiles/g2m.src.osm/utm32n/2/350/5450/", ErrorCode::Malformed},
        {"/g2m/v1/releases/" + rid + "/tiles/g2m.src.osm/utm32n/2/350", ErrorCode::Malformed},
        {"/g2m/v1/releases/" + rid + "/blobs/g2m.src.osm/utm32n/2/350/5450", ErrorCode::Malformed},
        {"/g2m/v1/releases/" + rid + "/tiles/g2m.src.osm/utm32n/2/350/5450?x", ErrorCode::Malformed},
        {"/g2m/v1/releases/../manifest", ErrorCode::Malformed},
        {"http://127.0.0.1/g2m/v1/manifest", ErrorCode::NotFound},
        {"*", ErrorCode::NotFound},
        {"", ErrorCode::NotFound},
        {"/g2m/v1/" + std::string(2000, 'a'), ErrorCode::Malformed},
    };
    for (const Case& c : cases) {
        INFO(c.target);
        auto r = parse_request("", "GET", c.target, {});
        REQUIRE_FALSE(r);
        CHECK(r.error().code == c.code);
    }
}

TEST_CASE("http wire: encode then decode reproduces every server response exactly", "[http][wire]") {
    World w;
    std::vector<Request> reqs = w.requests();
    reqs.push_back(TileRequest{w.release.rid, kLc, home_tile(), w.body_hash(kLc, home_tile())}); // 304
    reqs.push_back(TileRequest{w.release.rid, kLc, home_tile(), hash_of(1)});                   // 200 (stale tag)
    int seen304 = 0;
    for (const Request& r : reqs) {
        INFO(describe(r));
        const Response a = w.in_process->send(r);
        if (meta_of(a).status == Status::NotModified) ++seen304;
        const Message m = encode_response(r, a);
        CHECK(m.status == static_cast<int>(meta_of(a).status));
        const Response b = decode_response(r, m);
        CHECK(same_response(a, b));
    }
    CHECK(seen304 == 1);
}

TEST_CASE("http wire: scripted metadata survives the mapping", "[http][wire]") {
    World w;
    const Request req = TileRequest{w.release.rid, kLc, home_tile(), std::nullopt};
    std::vector<ResponseMeta> metas;
    for (Status s : {Status::BadRequest, Status::NotFound, Status::Gone, Status::Internal, Status::Unavailable}) {
        ResponseMeta m;
        m.status = s;
        m.message = std::string("line 1\r\nline 2: 100% \"q\" \xc3\xa9 \x01");
        if (s == Status::Gone) m.newest_rid = hash_of(7);
        if (s == Status::Unavailable) m.retry_after_s = 4000000000u;
        metas.push_back(m);
    }
    ResponseMeta deg;
    deg.status = Status::Unavailable;
    deg.degraded = true;
    metas.push_back(deg);
    ResponseMeta nm;
    nm.status = Status::NotModified;
    nm.etag = hash_of(5);
    metas.push_back(nm);
    for (const ResponseMeta& m : metas) {
        INFO(static_cast<int>(m.status));
        TileResponse t;
        t.meta = m;
        const Response a = t;
        const Response b = decode_response(req, encode_response(req, a));
        CHECK(same_response(a, b));
    }
    // The front end over a scripted backend gives the same bytes as encode_response.
    Scripted backend;
    TileResponse gone;
    gone.meta = metas[2];
    backend.next = gone;
    const Message direct = encode_response(req, backend.next);
    const Message via = handle_request(backend, "", "GET", request_target("", req), {});
    CHECK(direct.status == via.status);
    CHECK(direct.headers == via.headers);
    CHECK(direct.body == via.body);
}

TEST_CASE("http wire: foreign statuses and broken payloads", "[http][wire]") {
    World w;
    const TileRequest tile{w.release.rid, kLc, home_tile(), std::nullopt};
    const Request req = tile;
    const auto status_of = [&](const Message& m) { return meta_of(decode_response(req, m)); };

    Message m;
    m.status = 429;
    m.headers = {{"Retry-After", "7"}};
    CHECK(status_of(m).status == Status::Unavailable);
    CHECK(status_of(m).retry_after_s == 7);
    CHECK(status_of(m).message == "http 429");
    m.status = 502;
    m.headers = {{"retry-after", "Wed, 21 Oct 2015 07:28:00 GMT"}, {"x-g2m-message", "upstream%0A"}};
    CHECK(status_of(m).status == Status::Unavailable);
    CHECK(status_of(m).retry_after_s == 5); // HTTP-date ignored -> default
    CHECK(status_of(m).message == "http 502: upstream\n");
    m.headers = {};
    m.status = 418;
    CHECK(status_of(m).status == Status::BadRequest);
    m.status = 401;
    CHECK(status_of(m).status == Status::BadRequest);
    m.status = 302;
    CHECK(status_of(m).status == Status::Internal);
    m.status = 201;
    CHECK(status_of(m).status == Status::Internal);
    m.status = 503;
    CHECK(status_of(m).retry_after_s == 5);
    m.headers = {{std::string(kHeaderDegraded), "1"}};
    CHECK(status_of(m).degraded);
    CHECK(status_of(m).retry_after_s == 0);
    m.headers = {{std::string(kHeaderDegraded), "yes"}};
    CHECK(status_of(m).status == Status::Internal);
    m.status = 410;
    m.headers = {{std::string(kHeaderNewestRid), "zz"}};
    CHECK(status_of(m).status == Status::Internal);

    // A 200 tile must be the container of exactly the requested tile.
    const Response good = w.in_process->send(req);
    Message ok = encode_response(req, good);
    REQUIRE(ok.status == 200);
    CHECK(status_of(ok).status == Status::Ok);
    Message garbage = ok;
    garbage.body = {1, 2, 3};
    CHECK(status_of(garbage).status == Status::Internal);
    Message flipped = ok;
    flipped.body.back() ^= 0x01;
    CHECK(status_of(flipped).status == Status::Internal); // body hash
    const Request other = TileRequest{w.release.rid, kLc, home_tile(1, 0), std::nullopt};
    CHECK(meta_of(decode_response(other, ok)).status == Status::Internal); // another tile
    Message wrong_tag = ok;
    for (Header& h : wrong_tag.headers) {
        if (h.name == "ETag") h.value = etag_value(hash_of(2));
    }
    CHECK(status_of(wrong_tag).status == Status::Internal);
    Message no_tag = ok;
    std::erase_if(no_tag.headers, [](const Header& h) { return h.name == "ETag"; });
    CHECK(same_response(decode_response(req, no_tag), good)); // ETag recovered from the container

    // Packs / hashes: decoded only with their media type; a broken one is 500.
    const Request preq = PackRequest{w.release.rid, kLc, pack_of(home_tile())};
    Message pm = encode_response(preq, w.in_process->send(preq));
    REQUIRE(pm.status == 200);
    pm.body.resize(pm.body.size() - 1);
    CHECK(meta_of(decode_response(preq, pm)).status == Status::Internal);
    const Request hreq = HashesRequest{w.release.rid, kLc, pack_of(home_tile())};
    Message hm = encode_response(hreq, w.in_process->send(hreq));
    REQUIRE(hm.status == 200);
    const Request hother = HashesRequest{w.release.rid, kOsm, pack_of(home_tile())};
    CHECK(meta_of(decode_response(hother, hm)).status == Status::Internal);
    // A release manifest of another release is refused.
    const Request rreq = ReleaseManifestRequest{w.release.rid};
    Message rm = encode_response(rreq, w.in_process->send(rreq));
    REQUIRE(rm.status == 200);
    CHECK(meta_of(decode_response(Request{ReleaseManifestRequest{w.retired}}, rm)).status == Status::Internal);
    rm.body = {'{'};
    CHECK(meta_of(decode_response(rreq, rm)).status == Status::Internal);

    // A transport failure is a retryable 503.
    const Response f = transport_failure(req, "connection refused", 0);
    CHECK(meta_of(f).status == Status::Unavailable);
    CHECK(meta_of(f).retry_after_s == 1);
}

TEST_CASE("pack directory: canonical round trip and strict decode", "[http][pack]") {
    World w;
    const HashesResponse h = w.server->hashes(HashesRequest{w.release.rid, kLc, pack_of(home_tile())});
    REQUIRE(h.meta.status == Status::Ok);
    auto bytes = encode_pack_directory(h.directory);
    REQUIRE(bytes);
    CHECK(bytes->size() == 4 + 4 + 1 + h.directory.layer.size() + 32 + 4 + 8 + 256 * 33);
    auto d = decode_pack_directory(*bytes);
    REQUIRE(d);
    CHECK(*d == h.directory);
    CHECK(encode_pack_directory(*d).value() == *bytes);
    // Every truncation and a trailing byte fail.
    for (std::size_t n = 0; n < bytes->size(); n += 37) {
        CHECK_FALSE(decode_pack_directory(std::span<const std::uint8_t>(bytes->data(), n)));
    }
    auto longer = *bytes;
    longer.push_back(0);
    CHECK_FALSE(decode_pack_directory(longer));
    // Not a pack (magic), and a pack is not a directory.
    auto pack = w.server->pack(PackRequest{w.release.rid, kLc, pack_of(home_tile())}).pack;
    CHECK(decode_pack_directory(pack).error().code == ErrorCode::BadMagic);
    // A non-present entry with a hash.
    PackDirectory bad = h.directory;
    for (PackEntry& e : bad.entries) {
        if (e.state == PackTileState::NotFound) {
            e.body_hash[0] = 1;
            break;
        }
    }
    CHECK_FALSE(encode_pack_directory(bad));
    auto tampered = *bytes;
    const std::size_t dir_at = tampered.size() - 256 * 33;
    for (std::size_t i = 0; i < 256; ++i) {
        if (tampered[dir_at + i * 33] == 0) {
            tampered[dir_at + i * 33 + 1] = 1;
            break;
        }
    }
    CHECK_FALSE(decode_pack_directory(tampered));
    tampered = *bytes;
    tampered[dir_at] = 4; // bad state
    CHECK_FALSE(decode_pack_directory(tampered));
}
