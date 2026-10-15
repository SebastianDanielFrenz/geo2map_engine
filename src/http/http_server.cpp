// src/http/http_server.cpp - see g2m/http/http_server.h. The only
// translation unit of the front end that includes cpp-httplib.

#include "g2m/http/http_server.h"

#include "g2m/http/http_wire.h"

#include <httplib.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>

namespace g2m::http {

struct HttpServer::Impl {
    ITransport* backend = nullptr;
    HttpServerConfig cfg;
    httplib::Server server;
    std::thread thread;
    int port = 0;
    std::atomic<bool> listen_returned{false};
    std::atomic<std::uint64_t> requests{0};
    std::atomic<std::uint64_t> rejected{0};
    std::mutex stop_mu;
    bool stopped = false;

    httplib::Server::HandlerResponse serve(const httplib::Request& req, httplib::Response& res) {
        std::vector<std::string_view> inm;
        for (auto [it, end] = req.headers.equal_range("If-None-Match"); it != end; ++it) inm.push_back(it->second);
        Message m;
        auto parsed = parse_request(cfg.base_path, req.method, req.target, inm);
        if (!parsed) {
            const ErrorCode c = parsed.error().code;
            m = error_response(c == ErrorCode::NotFound ? 404 : c == ErrorCode::Unsupported ? 405 : 400,
                               parsed.error().message);
            rejected.fetch_add(1, std::memory_order_relaxed);
        } else {
            m = encode_response(*parsed, backend->send(*parsed));
        }
        requests.fetch_add(1, std::memory_order_relaxed);
        res.status = m.status;
        for (const Header& h : m.headers) res.set_header(h.name, h.value);
        res.body.assign(m.body.begin(), m.body.end());
        return httplib::Server::HandlerResponse::Handled;
    }
};

HttpServer::HttpServer(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}

HttpServer::~HttpServer() { stop(); }

Result<std::unique_ptr<HttpServer>> HttpServer::start(ITransport& backend, HttpServerConfig config) {
    const auto bad = [](std::string why) { return make_error(ErrorCode::InvalidArgument, "http server: " + std::move(why)); };
    if (config.bind_address.empty()) return bad("empty bind address");
    if (config.port < 0 || config.port > 65535) return bad("port out of range");
    if (!is_valid_base_path(config.base_path)) return bad("bad base path '" + config.base_path + "'");
    if (config.threads == 0) return bad("threads must be >= 1");

    auto impl = std::make_unique<Impl>();
    Impl& m = *impl;
    m.backend = &backend;
    m.cfg = std::move(config);
    const std::size_t threads = m.cfg.threads;
    const std::size_t queued = m.cfg.max_queued_connections;
    m.server.new_task_queue = [threads, queued] { return new httplib::ThreadPool(threads, threads, queued); };
    const auto secs = [](std::uint32_t ms) { return static_cast<time_t>(ms / 1000); };
    const auto usecs = [](std::uint32_t ms) { return static_cast<time_t>((ms % 1000) * 1000); };
    m.server.set_read_timeout(secs(m.cfg.read_timeout_ms), usecs(m.cfg.read_timeout_ms));
    m.server.set_write_timeout(secs(m.cfg.write_timeout_ms), usecs(m.cfg.write_timeout_ms));
    m.server.set_keep_alive_timeout(std::max<time_t>(1, secs(m.cfg.read_timeout_ms)));
    m.server.set_keep_alive_max_count(std::max<std::size_t>(1, m.cfg.keep_alive_max_requests));
    m.server.set_payload_max_length(4096); // GET only; a request body is never read for a reply
    // httplib's default is SO_REUSEPORT, which lets a second process bind the
    // same port and silently take half the connections. One listener per
    // port: SO_REUSEADDR on POSIX (rebind after TIME_WAIT only), exclusive
    // use on Windows (where SO_REUSEADDR would allow port theft).
    m.server.set_socket_options([](socket_t sock) {
#if defined(_WIN32)
        httplib::set_socket_opt(sock, SOL_SOCKET, SO_EXCLUSIVEADDRUSE, 1);
#else
        httplib::set_socket_opt(sock, SOL_SOCKET, SO_REUSEADDR, 1);
#endif
    });
    m.server.set_pre_routing_handler(
        [&m](const httplib::Request& req, httplib::Response& res) { return m.serve(req, res); });

    if (m.cfg.port == 0) {
        m.port = m.server.bind_to_any_port(m.cfg.bind_address);
        if (m.port <= 0) return make_error(ErrorCode::Io, "http server: cannot bind " + m.cfg.bind_address + " (any port)");
    } else {
        if (!m.server.bind_to_port(m.cfg.bind_address, m.cfg.port)) {
            return make_error(ErrorCode::Io, "http server: cannot bind " + m.cfg.bind_address + ":" + std::to_string(m.cfg.port));
        }
        m.port = m.cfg.port;
    }
    m.thread = std::thread([&m] {
        m.server.listen_after_bind();
        m.listen_returned.store(true);
    });
    // The socket is bound and listening already (connections queue in the
    // backlog); wait until the accept loop runs so stop() is never early.
    while (!m.server.is_running() && !m.listen_returned.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    if (m.listen_returned.load()) {
        m.thread.join();
        return make_error(ErrorCode::Io, "http server: the accept loop did not start");
    }
    return std::unique_ptr<HttpServer>(new HttpServer(std::move(impl)));
}

int HttpServer::port() const { return impl_->port; }

std::string HttpServer::base_url() const {
    const std::string& a = impl_->cfg.bind_address;
    const bool v6 = a.find(':') != std::string::npos;
    return "http://" + (v6 ? "[" + a + "]" : a) + ":" + std::to_string(impl_->port) + impl_->cfg.base_path;
}

HttpServerStats HttpServer::stats() const {
    return HttpServerStats{impl_->requests.load(std::memory_order_relaxed), impl_->rejected.load(std::memory_order_relaxed)};
}

void HttpServer::stop() {
    if (!impl_) return;
    std::lock_guard lock(impl_->stop_mu);
    if (impl_->stopped) return;
    impl_->stopped = true;
    impl_->server.stop();
    if (impl_->thread.joinable()) impl_->thread.join();
}

} // namespace g2m::http
