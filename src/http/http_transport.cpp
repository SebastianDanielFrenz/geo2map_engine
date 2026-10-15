// src/http/http_transport.cpp - see g2m/http/http_transport.h. The only
// translation unit of the client that includes cpp-httplib.

#include "g2m/http/http_transport.h"

#include "g2m/core/version.h"

#include <httplib.h>

#include <mutex>
#include <vector>

namespace g2m::http {

struct HttpTransport::Impl {
    HttpTransportConfig cfg;
    HttpUrl url;
    std::mutex mu;
    std::vector<std::unique_ptr<httplib::Client>> idle; // guarded by mu

    std::unique_ptr<httplib::Client> borrow() {
        {
            std::lock_guard lock(mu);
            if (!idle.empty()) {
                auto c = std::move(idle.back());
                idle.pop_back();
                return c;
            }
        }
        auto c = std::make_unique<httplib::Client>(url.host, url.port);
        const auto set_ms = [](std::uint32_t ms, auto&& setter) {
            setter(static_cast<time_t>(ms / 1000), static_cast<time_t>((ms % 1000) * 1000));
        };
        set_ms(cfg.connect_timeout_ms, [&](time_t s, time_t us) { c->set_connection_timeout(s, us); });
        set_ms(cfg.read_timeout_ms, [&](time_t s, time_t us) { c->set_read_timeout(s, us); });
        set_ms(cfg.write_timeout_ms, [&](time_t s, time_t us) { c->set_write_timeout(s, us); });
        c->set_keep_alive(true);
        // Our targets are canonical protocol paths ([A-Za-z0-9._~/-] only);
        // nothing may rewrite them.
        c->set_path_encode(false);
        c->set_payload_max_length(static_cast<std::size_t>(cfg.max_body_bytes));
        c->set_follow_location(false);
        return c;
    }

    void give_back(std::unique_ptr<httplib::Client> c) {
        std::lock_guard lock(mu);
        if (idle.size() < cfg.max_idle_connections) idle.push_back(std::move(c));
    }
};

HttpTransport::HttpTransport(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
HttpTransport::~HttpTransport() = default;

Result<std::unique_ptr<HttpTransport>> HttpTransport::create(HttpTransportConfig config) {
    auto url = parse_http_url(config.base_url);
    if (!url) return url.error();
    if (config.user_agent.empty()) config.user_agent = "g2m/" + std::string(version_string());
    for (char c : config.user_agent) {
        if (c < 0x20 || c > 0x7e) return make_error(ErrorCode::InvalidArgument, "http: user_agent must be printable ASCII");
    }
    auto impl = std::make_unique<Impl>();
    impl->cfg = std::move(config);
    impl->url = std::move(*url);
    return std::unique_ptr<HttpTransport>(new HttpTransport(std::move(impl)));
}

const HttpUrl& HttpTransport::url() const { return impl_->url; }

Response HttpTransport::send(const Request& request) {
    const std::string target = request_target(impl_->url.base_path, request);
    httplib::Headers headers;
    headers.emplace("User-Agent", impl_->cfg.user_agent);
    for (const Header& h : request_headers(request)) headers.emplace(h.name, h.value);

    auto client = impl_->borrow();
    httplib::Result res = client->Get(target, headers);
    if (!res) {
        // The connection is in an unknown state: drop it, do not pool it.
        return transport_failure(request, "GET " + target + ": " + httplib::to_string(res.error()),
                                 impl_->cfg.failure_retry_after_s);
    }
    Message m;
    m.status = res->status;
    m.headers.reserve(res->headers.size());
    for (const auto& [name, value] : res->headers) m.headers.push_back(Header{name, value});
    m.body.assign(res->body.begin(), res->body.end());
    impl_->give_back(std::move(client));

    DecodeOptions options;
    options.default_retry_after_s = impl_->cfg.default_retry_after_s;
    return decode_response(request, m, options);
}

} // namespace g2m::http
