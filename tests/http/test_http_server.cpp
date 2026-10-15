// g2m_http over real sockets: HttpServer in front of an offline g2m::Server,
// HttpTransport as the client. Every protocol request answers byte-identically
// to the in-process transport; malformed requests are refused by the front
// end without reaching the server; concurrent clients agree with a reference.

#include "g2m/http/http_server.h"
#include "g2m/http/http_transport.h"

#include "http_test_world.h"

#include <catch2/catch_test_macros.hpp>

#include <httplib.h>

#include <thread>

#if defined(_WIN32)
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

using namespace g2m;
using namespace g2m::http;
using namespace g2m_http_test;

namespace {

struct Stack {
    World world;
    std::unique_ptr<HttpServer> server;
    std::unique_ptr<HttpTransport> client;

    explicit Stack(HttpServerConfig hc = {}) {
        auto s = HttpServer::start(*world.in_process, hc);
        REQUIRE(s);
        server = std::move(*s);
        HttpTransportConfig tc;
        tc.base_url = server->base_url();
        auto c = HttpTransport::create(tc);
        REQUIRE(c);
        client = std::move(*c);
    }
};

// One raw HTTP exchange on a fresh socket (bytes in, bytes until close out):
// for requests no HTTP client library would send.
std::string raw_exchange(int port, const std::string& bytes) {
#if defined(_WIN32)
    WSADATA wsa;
    WSAStartup(MAKEWORD(2, 2), &wsa);
    using sock_t = SOCKET;
    const sock_t bad = INVALID_SOCKET;
#else
    using sock_t = int;
    const sock_t bad = -1;
#endif
    sock_t s = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (s == bad) return {};
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<std::uint16_t>(port));
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    std::string out;
    if (::connect(s, reinterpret_cast<sockaddr*>(&addr), sizeof addr) == 0) {
        ::send(s, bytes.data(), static_cast<int>(bytes.size()), 0);
#if defined(_WIN32)
        ::shutdown(s, SD_SEND);
#else
        ::shutdown(s, SHUT_WR);
#endif
        char buf[4096];
        while (true) {
            const auto n = ::recv(s, buf, sizeof buf, 0);
            if (n <= 0) break;
            out.append(buf, static_cast<std::size_t>(n));
        }
    }
#if defined(_WIN32)
    ::closesocket(s);
#else
    ::close(s);
#endif
    return out;
}

int status_line_code(const std::string& response) {
    // "HTTP/1.1 400 Bad Request\r\n..."
    if (response.size() < 12 || response.compare(0, 5, "HTTP/") != 0) return 0;
    return std::atoi(response.substr(9, 3).c_str());
}

} // namespace

TEST_CASE("http: every protocol request answers byte-identically over HTTP and in process", "[http][roundtrip]") {
    Stack st;
    World& w = st.world;
    std::vector<Request> reqs = w.requests();
    reqs.push_back(TileRequest{w.release.rid, kLc, home_tile(), w.body_hash(kLc, home_tile())});       // 304
    reqs.push_back(TileRequest{w.release.rid, kSum, home_tile(1, 0), w.body_hash(kSum, home_tile(1, 0))}); // 304
    std::map<Status, int> statuses;
    for (int pass = 0; pass < 2; ++pass) { // pass 1: HTTP first (cold for derived tiles is already gone; order varies)
        for (const Request& r : reqs) {
            INFO(describe(r) << " pass " << pass);
            const Response a = pass == 0 ? w.in_process->send(r) : st.client->send(r);
            const Response b = pass == 0 ? st.client->send(r) : w.in_process->send(r);
            CHECK(same_response(a, b));
            statuses[meta_of(a).status]++;
        }
    }
    for (Status s : {Status::Ok, Status::NotModified, Status::NotFound, Status::Gone, Status::Internal, Status::Unavailable}) {
        INFO(static_cast<int>(s));
        CHECK(statuses[s] > 0);
    }
    CHECK(w.net.calls == 0); // offline: the network upstream was never called
    CHECK(st.server->stats().rejected == 0);

    // Under a base path, and through a second front end on the same server.
    HttpServerConfig hc;
    hc.base_path = "/maps/v";
    auto s2 = HttpServer::start(*w.in_process, hc);
    REQUIRE(s2);
    HttpTransportConfig tc;
    tc.base_url = (*s2)->base_url();
    auto c2 = HttpTransport::create(tc);
    REQUIRE(c2);
    for (const Request& r : reqs) {
        INFO(describe(r));
        CHECK(same_response(w.in_process->send(r), (*c2)->send(r)));
    }
}

TEST_CASE("http: a bad request object answers 400 through both transports", "[http][roundtrip]") {
    Stack st;
    World& w = st.world;
    TileKey bad = home_tile();
    bad.level = 31; // no such level: parse_path refuses the path
    const Request r = TileRequest{w.release.rid, kLc, bad, std::nullopt};
    CHECK(meta_of(w.in_process->send(r)).status == Status::BadRequest);
    CHECK(meta_of(st.client->send(r)).status == Status::BadRequest);
    CHECK(st.server->stats().rejected == 1);
}

TEST_CASE("http: malformed requests are refused and the server keeps serving", "[http][malformed]") {
    Stack st;
    World& w = st.world;
    httplib::Client raw("127.0.0.1", st.server->port());
    raw.set_path_encode(false);
    const std::string rid = to_hex(w.release.rid);
    const std::string tile = "/g2m/v1/releases/" + rid + "/tiles/g2m.src.landcover/utm32n/2/350/5450";
    struct Case {
        std::string target;
        int status;
    };
    const std::vector<Case> cases = {
        {"/", 404},
        {"/g2m/v2/manifest", 404},
        {"/g2m/v1/", 400},
        {"/g2m/v1/manifest?x=1", 400},
        {"/g2m/v1/manifest/", 400},
        {"/g2m/v1/%6Danifest", 400},
        {"/g2m/v1/releases/" + rid + "/tiles/g2m.src.landcover/utm32n/2/0350/5450", 400},
        {"/g2m/v1/releases/" + rid + "/tiles/g2m.src.landcover/utm32n/2/350/5450/", 400},
        {"/g2m/v1/" + std::string(1500, 'a'), 400},
        {tile, 200},
    };
    for (const Case& c : cases) {
        INFO(c.target);
        auto res = raw.Get(c.target);
        REQUIRE(res);
        CHECK(res->status == c.status);
        if (c.status != 200) {
            CHECK(res->get_header_value("Cache-Control") == "no-store");
            CHECK(!res->get_header_value(std::string(kHeaderMessage)).empty());
        }
    }
    // Header problems.
    auto two = raw.Get(tile, httplib::Headers{{"If-None-Match", "\"" + std::string(64, 'a') + "\""},
                                               {"If-None-Match", "\"" + std::string(64, 'b') + "\""}});
    REQUIRE(two);
    CHECK(two->status == 400);
    auto weak = raw.Get(tile, httplib::Headers{{"If-None-Match", "W/\"" + std::string(64, 'a') + "\""}});
    REQUIRE(weak);
    CHECK(weak->status == 400);
    // Methods.
    auto post = raw.Post("/g2m/v1/manifest", "x", "text/plain");
    REQUIRE(post);
    CHECK(post->status == 405);
    CHECK(post->get_header_value("Allow") == "GET, HEAD");
    auto del = raw.Delete("/g2m/v1/manifest");
    REQUIRE(del);
    CHECK(del->status == 405);
    auto head = raw.Head(tile);
    REQUIRE(head);
    CHECK(head->status == 200);
    CHECK(head->body.empty());
    CHECK(head->get_header_value("ETag").size() == 66);
    // Garbage on the wire (no client library sends this).
    const int port = st.server->port();
    for (const std::string& junk : {std::string("GARBAGE\r\n\r\n"), std::string("GET\r\n\r\n"),
                                    std::string("GET /g2m/v1/manifest HTTP/1.1\r\nHost: x\r\nBad Header\r\n\r\n"),
                                    std::string("\x16\x03\x01\x02\x00\x01\x00\x01\xfc\x03\x03", 11), // a TLS ClientHello start
                                    std::string(20000, 'A') + "\r\n\r\n"}) {
        const std::string reply = raw_exchange(port, junk);
        const int code = status_line_code(reply);
        INFO(junk.substr(0, 40) << " -> " << reply.substr(0, 40));
        CHECK((code == 0 || (code >= 400 && code < 500)));
    }
    // Still serving, byte-identically.
    for (const Request& r : w.requests()) CHECK(same_response(w.in_process->send(r), st.client->send(r)));
}

TEST_CASE("http: concurrent clients agree with the in-process reference", "[http][concurrency]") {
    HttpServerConfig hc;
    hc.threads = 4;
    Stack st(hc);
    World& w = st.world;
    const std::vector<Request> reqs = w.requests();
    std::vector<Response> expect;
    for (const Request& r : reqs) expect.push_back(w.in_process->send(r));
    constexpr int kThreads = 8;
    constexpr int kRounds = 4;
    std::atomic<int> bad{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < kThreads; ++t) {
        threads.emplace_back([&, t] {
            // Half the threads share the stack's transport, half own one.
            std::unique_ptr<HttpTransport> own;
            ITransport* tr = st.client.get();
            if (t % 2 == 1) {
                HttpTransportConfig tc;
                tc.base_url = st.server->base_url();
                own = std::move(HttpTransport::create(tc).value());
                tr = own.get();
            }
            for (int round = 0; round < kRounds; ++round) {
                for (std::size_t i = 0; i < reqs.size(); ++i) {
                    const std::size_t k = (i + static_cast<std::size_t>(t) * 7) % reqs.size();
                    if (!same_response(expect[k], tr->send(reqs[k]))) bad++;
                }
            }
        });
    }
    for (auto& th : threads) th.join();
    CHECK(bad == 0);
    CHECK(st.server->stats().requests == static_cast<std::uint64_t>(kThreads * kRounds) * reqs.size());
}

TEST_CASE("http: transport failures are retryable 503s; config errors fail by value", "[http][status]") {
    World w;
    int port = 0;
    {
        auto s = HttpServer::start(*w.in_process, {});
        REQUIRE(s);
        port = (*s)->port();
        (*s)->stop();
        (*s)->stop(); // idempotent
    }
    HttpTransportConfig tc;
    tc.base_url = "http://127.0.0.1:" + std::to_string(port);
    tc.connect_timeout_ms = 500;
    tc.failure_retry_after_s = 3;
    auto c = HttpTransport::create(tc);
    REQUIRE(c);
    const Response r = (*c)->send(TileRequest{w.release.rid, kLc, home_tile(), std::nullopt});
    CHECK(meta_of(r).status == Status::Unavailable);
    CHECK(meta_of(r).retry_after_s == 3);
    CHECK_FALSE(meta_of(r).degraded);
    CHECK(meta_of(r).message.rfind("http: ", 0) == 0);

    tc.base_url = "https://127.0.0.1";
    CHECK_FALSE(HttpTransport::create(tc));
    tc.base_url = "http://127.0.0.1";
    tc.user_agent = "bad\nagent";
    CHECK_FALSE(HttpTransport::create(tc));

    HttpServerConfig hc;
    hc.base_path = "/bad/";
    CHECK_FALSE(HttpServer::start(*w.in_process, hc));
    hc = {};
    hc.threads = 0;
    CHECK_FALSE(HttpServer::start(*w.in_process, hc));
    hc = {};
    hc.bind_address = "203.0.113.1"; // TEST-NET-3: not an address of this host
    CHECK_FALSE(HttpServer::start(*w.in_process, hc));
    // A port in use.
    auto first = HttpServer::start(*w.in_process, {});
    REQUIRE(first);
    hc = {};
    hc.port = (*first)->port();
    auto second = HttpServer::start(*w.in_process, hc);
    CHECK_FALSE(second);
}

TEST_CASE("http: a scripted backend's headers reach the client unchanged", "[http][status]") {
    class Scripted final : public ITransport {
    public:
        Response send(const Request& r) override {
            TileResponse t;
            const auto& q = std::get<TileRequest>(r);
            t.meta.status = q.key.x == 1 ? Status::Gone : Status::Unavailable;
            t.meta.message = "x\r\ny: 50% \xe2\x9c\x93";
            if (q.key.x == 1) t.meta.newest_rid = q.rid;
            else t.meta.retry_after_s = 12;
            return t;
        }
    } scripted;
    auto s = HttpServer::start(scripted, {});
    REQUIRE(s);
    HttpTransportConfig tc;
    tc.base_url = (*s)->base_url();
    auto c = HttpTransport::create(tc);
    REQUIRE(c);
    ReleaseId rid{};
    rid[3] = 9;
    for (int x : {1, 2}) {
        const Request r = TileRequest{rid, kLc, TileKey{geo::UtmZone{32, geo::Hemisphere::North}, 2, x, 5}, std::nullopt};
        CHECK(same_response(scripted.send(r), (*c)->send(r)));
    }
}

TEST_CASE("http: stop() does not wait for idle keep-alive connections", "[http][status]") {
    World w;
    auto s = HttpServer::start(*w.in_process, {});
    REQUIRE(s);
    HttpTransportConfig tc;
    tc.base_url = (*s)->base_url();
    auto c = HttpTransport::create(tc);
    REQUIRE(c);
    REQUIRE(meta_of((*c)->send(ManifestRequest{})).status == Status::Ok); // leaves one pooled connection open
    const auto t0 = std::chrono::steady_clock::now();
    (*s)->stop();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0).count();
    CHECK(ms < 2000);
    // The pooled connection is dead now: the next request is a retryable 503, not a hang.
    CHECK(meta_of((*c)->send(ManifestRequest{})).status == Status::Unavailable);
}

TEST_CASE("http: pooled connections the server closed are replaced transparently", "[http][status]") {
    World w;
    HttpServerConfig hc;
    hc.keep_alive_max_requests = 2; // the server closes every connection after two requests
    hc.read_timeout_ms = 200;       // and idle ones after 0.2 s
    auto s = HttpServer::start(*w.in_process, hc);
    REQUIRE(s);
    HttpTransportConfig tc;
    tc.base_url = (*s)->base_url();
    auto c = HttpTransport::create(tc);
    REQUIRE(c);
    const Request r = TileRequest{w.release.rid, kLc, home_tile(), std::nullopt};
    const Response expect = w.in_process->send(r);
    for (int i = 0; i < 7; ++i) CHECK(same_response(expect, (*c)->send(r)));
    std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    CHECK(same_response(expect, (*c)->send(r)));
}
