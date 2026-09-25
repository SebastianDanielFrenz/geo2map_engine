// src/server/protocol.cpp - see g2m/server/protocol.h.

#include "g2m/server/protocol.h"

#include "g2m/layer/layer_id.h"

#include <nlohmann/json.hpp>

#include <algorithm>

namespace g2m {
namespace {

using json = nlohmann::json;

constexpr std::string_view kReleases = "/releases/";

std::string key_path(const std::string& layer, const TileKey& key) { return layer + "/" + to_string(key); }

// "{layer}/{scheme}/{level}/{x}/{y}"
bool parse_layer_key(std::string_view s, std::string& layer, TileKey& key) {
    const std::size_t slash = s.find('/');
    if (slash == std::string_view::npos) return false;
    layer = std::string(s.substr(0, slash));
    if (!is_valid_layer_name(layer)) return false;
    const auto k = parse_tile_path(s.substr(slash + 1));
    if (!k) return false;
    key = *k;
    return true;
}

} // namespace

std::string to_json(const ServerManifest& m) {
    json j = json::object();
    j["format"] = "g2m.server/1";
    j["server_id"] = m.server_id;
    j["protocol_versions"] = m.protocol_versions;
    j["default_rid"] = to_hex(m.default_rid);
    json live = json::array();
    for (const ReleaseId& r : m.live_releases) live.push_back(to_hex(r));
    j["live_releases"] = live;
    j["tile_base_url"] = m.tile_base_url;
    j["auth_mode"] = m.auth_mode;
    j["rate_limit"] = m.rate_limit;
    j["contact"] = m.contact;
    return j.dump(-1, ' ', false, json::error_handler_t::replace);
}

Result<ServerManifest> server_manifest_from_json(std::string_view text) {
    const json j = json::parse(text.begin(), text.end(), nullptr, /*allow_exceptions=*/false);
    const auto bad = [](const std::string& what) { return make_error(ErrorCode::Malformed, "server manifest: " + what); };
    if (!j.is_object()) return bad("not a JSON object");
    const auto str = [&](const char* k, std::string& out) {
        const auto it = j.find(k);
        if (it == j.end() || !it->is_string()) return false;
        out = it->get_ref<const std::string&>();
        return true;
    };
    std::string format;
    if (!str("format", format) || format != "g2m.server/1") return bad("format must be \"g2m.server/1\"");
    ServerManifest m;
    std::string rid;
    if (!str("server_id", m.server_id) || !str("tile_base_url", m.tile_base_url) || !str("auth_mode", m.auth_mode) ||
        !str("rate_limit", m.rate_limit) || !str("contact", m.contact) || !str("default_rid", rid)) {
        return bad("missing or mistyped string field");
    }
    const auto d = content_hash_from_hex(rid);
    if (!d) return bad("default_rid");
    m.default_rid = *d;
    const auto pv = j.find("protocol_versions");
    const auto lr = j.find("live_releases");
    if (pv == j.end() || !pv->is_array() || lr == j.end() || !lr->is_array()) return bad("missing array");
    m.protocol_versions.clear();
    for (const json& v : *pv) {
        if (!v.is_number_unsigned() || v.get<std::uint64_t>() > 0xffffffffu) return bad("protocol_versions");
        m.protocol_versions.push_back(static_cast<std::uint32_t>(v.get<std::uint64_t>()));
    }
    for (const json& v : *lr) {
        if (!v.is_string()) return bad("live_releases");
        const auto h = content_hash_from_hex(v.get_ref<const std::string&>());
        if (!h) return bad("live_releases");
        m.live_releases.push_back(*h);
    }
    return m;
}

const ResponseMeta& meta_of(const Response& r) {
    return std::visit([](const auto& x) -> const ResponseMeta& { return x.meta; }, r);
}

std::string to_path(const Request& r) {
    struct V {
        std::string operator()(const ManifestRequest&) const { return "/manifest"; }
        std::string operator()(const ReleaseManifestRequest& q) const {
            return std::string(kReleases) + to_hex(q.rid) + "/manifest";
        }
        std::string operator()(const TileRequest& q) const {
            return std::string(kReleases) + to_hex(q.rid) + "/tiles/" + key_path(q.layer, q.key);
        }
        std::string operator()(const PackRequest& q) const {
            return std::string(kReleases) + to_hex(q.rid) + "/packs/" + key_path(q.layer, q.pack);
        }
        std::string operator()(const HashesRequest& q) const {
            return std::string(kReleases) + to_hex(q.rid) + "/hashes/" + key_path(q.layer, q.pack);
        }
    };
    return std::visit(V{}, r);
}

Result<Request> parse_path(std::string_view path) {
    const auto bad = [] { return make_error(ErrorCode::Malformed, "not a protocol v1 path"); };
    if (path == "/manifest") return Request{ManifestRequest{}};
    if (!path.starts_with(kReleases)) return bad();
    path.remove_prefix(kReleases.size());
    if (path.size() < 65 || path[64] != '/') return bad();
    const auto rid = content_hash_from_hex(path.substr(0, 64));
    if (!rid) return bad();
    path.remove_prefix(65);
    if (path == "manifest") return Request{ReleaseManifestRequest{*rid}};
    const std::size_t slash = path.find('/');
    if (slash == std::string_view::npos) return bad();
    const std::string_view kind = path.substr(0, slash);
    std::string layer;
    TileKey key;
    if (!parse_layer_key(path.substr(slash + 1), layer, key)) return bad();
    if (kind == "tiles") return Request{TileRequest{*rid, std::move(layer), key, std::nullopt}};
    if (kind == "packs") return Request{PackRequest{*rid, std::move(layer), key}};
    if (kind == "hashes") return Request{HashesRequest{*rid, std::move(layer), key}};
    return bad();
}

} // namespace g2m
