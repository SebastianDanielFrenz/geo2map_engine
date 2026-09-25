// src/layer/release.cpp - see g2m/layer/release.h and
// docs/formats/release_manifest.md.

#include "g2m/layer/release.h"

#include "g2m/core/bytes.h"
#include "g2m/core/hash.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cstdio>
#include <map>

namespace g2m {
namespace {

using json = nlohmann::json;

template <class T, class Eq>
bool has_adjacent(const std::vector<T>& v, Eq eq) {
    for (std::size_t i = 1; i < v.size(); ++i) {
        if (eq(v[i - 1], v[i])) return true;
    }
    return false;
}

void put_str(ByteWriter& w, std::string_view s) {
    w.u64(s.size());
    w.bytes(s.data(), s.size());
}

std::string scheme_name(geo::UtmZone z) { return geo::utm_scheme_name(z); }

std::optional<geo::UtmZone> parse_scheme(std::string_view s) { return geo::parse_utm_scheme(s); }

const char* kind_name(LayerKind k) { return k == LayerKind::Raw ? "raw" : "baked"; }
const char* availability_name(Availability a) {
    switch (a) {
    case Availability::Prebaked: return "prebaked";
    case Availability::OnDemand: return "on_demand";
    case Availability::Unavailable: return "unavailable";
    }
    return "unavailable";
}

// ---- strict JSON readers (no exceptions: every access is type-checked) ----

struct Reader {
    std::string err;
    bool fail(const std::string& what) {
        if (err.empty()) err = what;
        return false;
    }
    const json* field(const json& o, const char* k) {
        if (!o.is_object()) {
            fail(std::string("expected an object around '") + k + "'");
            return nullptr;
        }
        const auto it = o.find(k);
        if (it == o.end()) {
            fail(std::string("missing '") + k + "'");
            return nullptr;
        }
        return &*it;
    }
    bool str(const json& o, const char* k, std::string& out) {
        const json* v = field(o, k);
        if (!v) return false;
        if (!v->is_string()) return fail(std::string("'") + k + "' must be a string");
        out = v->get_ref<const std::string&>();
        return true;
    }
    bool boolean(const json& o, const char* k, bool& out) {
        const json* v = field(o, k);
        if (!v) return false;
        if (!v->is_boolean()) return fail(std::string("'") + k + "' must be a boolean");
        out = v->get<bool>();
        return true;
    }
    bool uint(const json& o, const char* k, std::uint64_t max, std::uint64_t& out) {
        const json* v = field(o, k);
        if (!v) return false;
        if (!v->is_number_unsigned() || v->get<std::uint64_t>() > max) {
            return fail(std::string("'") + k + "' must be an integer in [0, " + std::to_string(max) + "]");
        }
        out = v->get<std::uint64_t>();
        return true;
    }
    bool hash(const json& o, const char* k, ContentHash& out) {
        std::string s;
        if (!str(o, k, s)) return false;
        const auto h = content_hash_from_hex(s);
        if (!h) return fail(std::string("'") + k + "' must be 64 lower-case hex digits");
        out = *h;
        return true;
    }
    const json* array(const json& o, const char* k) {
        const json* v = field(o, k);
        if (!v) return nullptr;
        if (!v->is_array()) {
            fail(std::string("'") + k + "' must be an array");
            return nullptr;
        }
        return v;
    }
};

} // namespace

std::string to_hex(const ContentHash& h) { return to_hex(std::span<const std::uint8_t>(h)); }

std::optional<ContentHash> content_hash_from_hex(std::string_view hex) {
    if (hex.size() != 64) return std::nullopt;
    ContentHash h{};
    for (std::size_t i = 0; i < 64; ++i) {
        const char c = hex[i];
        int v = 0;
        if (c >= '0' && c <= '9') {
            v = c - '0';
        } else if (c >= 'a' && c <= 'f') {
            v = c - 'a' + 10;
        } else {
            return std::nullopt;
        }
        h[i / 2] = static_cast<std::uint8_t>(h[i / 2] | (v << ((i % 2) ? 0 : 4)));
    }
    return h;
}

Result<ReleaseId> compute_release_id(const ReleaseInputs& in) {
    std::vector<SourceTileRef> src = in.sources;
    std::vector<LayerVersionRef> lay = in.layers;
    std::vector<ClassifierRef> cls = in.classifiers;
    std::sort(src.begin(), src.end());
    std::sort(lay.begin(), lay.end());
    std::sort(cls.begin(), cls.end());
    if (has_adjacent(src, [](const auto& a, const auto& b) { return a.layer == b.layer && a.key_packed == b.key_packed; })) {
        return make_error(ErrorCode::InvalidArgument, "release: duplicate source tile");
    }
    if (has_adjacent(lay, [](const auto& a, const auto& b) { return a.layer == b.layer; })) {
        return make_error(ErrorCode::InvalidArgument, "release: duplicate layer");
    }
    if (has_adjacent(cls, [](const auto& a, const auto& b) { return a.name == b.name; })) {
        return make_error(ErrorCode::InvalidArgument, "release: duplicate classifier");
    }
    ByteWriter w;
    put_str(w, "g2m.rid/1");
    w.u64(src.size());
    for (const auto& s : src) {
        put_str(w, s.layer);
        w.u64(s.key_packed);
        w.bytes(s.body_hash);
    }
    w.u64(lay.size());
    for (const auto& l : lay) {
        put_str(w, l.layer);
        w.u32(l.version);
        w.bytes(l.config_hash);
    }
    w.u64(cls.size());
    for (const auto& c : cls) {
        put_str(w, c.name);
        w.bytes(c.table_hash);
    }
    w.u64(in.world_seed);
    return content_hash(w.data());
}

// ---- coverage ----

Coverage Coverage::from_level2_tiles(std::vector<TileKey> keys) {
    std::sort(keys.begin(), keys.end());
    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
    Coverage c;
    for (const TileKey& k : keys) {
        if (k.level != 2 || !k.valid()) continue;
        if (c.zones.empty() || c.zones.back().zone != k.zone) c.zones.push_back(ZoneCoverage{k.zone, {}});
        auto& runs = c.zones.back().runs;
        if (!runs.empty() && runs.back().y == k.y && runs.back().x1 + 1 == k.x) {
            runs.back().x1 = k.x;
        } else {
            runs.push_back(CoverageRun{k.y, k.x, k.x});
        }
    }
    return c;
}

bool Coverage::covers(const TileKey& key) const {
    if (global) return true;
    if (!key.valid()) return false;
    std::int64_t ylo = 0, yhi = 0, xlo = 0, xhi = 0;
    if (key.level <= 2) {
        const int sh = 2 - key.level;
        ylo = yhi = key.y >> sh;
        xlo = xhi = key.x >> sh;
    } else {
        const int sh = key.level - 2;
        ylo = static_cast<std::int64_t>(key.y) << sh;
        yhi = ((static_cast<std::int64_t>(key.y) + 1) << sh) - 1;
        xlo = static_cast<std::int64_t>(key.x) << sh;
        xhi = ((static_cast<std::int64_t>(key.x) + 1) << sh) - 1;
    }
    for (const ZoneCoverage& z : zones) {
        if (z.zone != key.zone) continue;
        for (const CoverageRun& r : z.runs) {
            if (r.y >= ylo && r.y <= yhi && r.x1 >= xlo && r.x0 <= xhi) return true;
        }
    }
    return false;
}

const ReleaseLayer* ReleaseManifest::find_layer(std::string_view name) const {
    for (const ReleaseLayer& l : layers) {
        if (l.desc.id.name == name) return &l;
    }
    return nullptr;
}

// ---- JSON ----

std::string to_json(const ReleaseManifest& m) {
    json j = json::object();
    j["format"] = "g2m.release/1";
    j["rid"] = to_hex(m.rid);
    j["container_major"] = m.container_major;
    char seed[17];
    std::snprintf(seed, sizeof seed, "%016llx", static_cast<unsigned long long>(m.world_seed));
    j["world_seed"] = seed;
    json sources = json::array();
    for (const ReleaseSource& s : m.sources) {
        sources.push_back({{"id", s.id},
                           {"snapshot_date", s.snapshot_date},
                           {"licence", s.licence},
                           {"attribution", s.attribution},
                           {"share_alike", s.share_alike}});
    }
    j["sources"] = std::move(sources);
    json layers = json::array();
    for (const ReleaseLayer& l : m.layers) {
        json levels = json::array();
        for (int lv = 0; lv <= TileKey::kMaxLevel; ++lv) {
            if (has_level(l.desc.levels, lv)) levels.push_back(lv);
        }
        json cov = json::object();
        cov["global"] = l.coverage.global;
        json zones = json::array();
        for (const ZoneCoverage& z : l.coverage.zones) {
            json runs = json::array();
            for (const CoverageRun& r : z.runs) runs.push_back(json::array({r.y, r.x0, r.x1}));
            zones.push_back({{"scheme", scheme_name(z.zone)}, {"runs", std::move(runs)}});
        }
        cov["zones"] = std::move(zones);
        layers.push_back({{"name", l.desc.id.name},
                          {"version", l.desc.id.version},
                          {"kind", kind_name(l.desc.kind)},
                          {"tiling", "utm"},
                          {"levels", std::move(levels)},
                          {"encoding", l.desc.encoding},
                          {"encoding_version", l.desc.encoding_version},
                          {"inputs", l.desc.inputs},
                          {"content", l.desc.content},
                          {"config_hash", to_hex(l.config_hash)},
                          {"availability", availability_name(l.availability)},
                          {"coverage", std::move(cov)}});
    }
    j["layers"] = std::move(layers);
    json cls = json::array();
    for (const ClassifierRef& c : m.classifiers) cls.push_back({{"name", c.name}, {"table_hash", to_hex(c.table_hash)}});
    j["classifiers"] = std::move(cls);
    return j.dump(-1, ' ', false, json::error_handler_t::replace);
}

Result<ReleaseManifest> release_manifest_from_json(std::string_view text) {
    const json j = json::parse(text.begin(), text.end(), nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return make_error(ErrorCode::Malformed, "release manifest: not valid JSON");
    Reader rd;
    ReleaseManifest m;
    const auto fail = [&]() { return make_error(ErrorCode::Malformed, "release manifest: " + rd.err); };

    std::string format;
    if (!rd.str(j, "format", format)) return fail();
    if (format != "g2m.release/1") return make_error(ErrorCode::Unsupported, "release manifest: format '" + format + "'");
    std::uint64_t v = 0;
    std::string seed;
    if (!rd.hash(j, "rid", m.rid) || !rd.uint(j, "container_major", 255, v) || !rd.str(j, "world_seed", seed)) return fail();
    m.container_major = static_cast<std::uint8_t>(v);
    if (seed.size() != 16) return make_error(ErrorCode::Malformed, "release manifest: world_seed must be 16 hex digits");
    for (char c : seed) {
        const bool hex = (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
        if (!hex) return make_error(ErrorCode::Malformed, "release manifest: world_seed must be 16 hex digits");
        m.world_seed = (m.world_seed << 4) | static_cast<std::uint64_t>(c <= '9' ? c - '0' : c - 'a' + 10);
    }

    const json* sources = rd.array(j, "sources");
    if (!sources) return fail();
    for (const json& s : *sources) {
        ReleaseSource rs;
        if (!rd.str(s, "id", rs.id) || !rd.str(s, "snapshot_date", rs.snapshot_date) || !rd.str(s, "licence", rs.licence) ||
            !rd.str(s, "attribution", rs.attribution) || !rd.boolean(s, "share_alike", rs.share_alike)) {
            return fail();
        }
        m.sources.push_back(std::move(rs));
    }

    const json* layers = rd.array(j, "layers");
    if (!layers) return fail();
    for (const json& l : *layers) {
        ReleaseLayer rl;
        std::string kind, tiling, avail;
        std::uint64_t ver = 0, enc = 0, encv = 0;
        if (!rd.str(l, "name", rl.desc.id.name) || !rd.uint(l, "version", 0xffffffffu, ver) || !rd.str(l, "kind", kind) ||
            !rd.str(l, "tiling", tiling) || !rd.uint(l, "encoding", 0xffff, enc) ||
            !rd.uint(l, "encoding_version", 0xffff, encv) || !rd.str(l, "content", rl.desc.content) ||
            !rd.hash(l, "config_hash", rl.config_hash) || !rd.str(l, "availability", avail)) {
            return fail();
        }
        if (!is_valid_layer_name(rl.desc.id.name)) return make_error(ErrorCode::Malformed, "release manifest: layer name");
        rl.desc.id.version = static_cast<std::uint32_t>(ver);
        rl.desc.encoding = static_cast<std::uint16_t>(enc);
        rl.desc.encoding_version = static_cast<std::uint16_t>(encv);
        if (kind == "raw") {
            rl.desc.kind = LayerKind::Raw;
        } else if (kind == "baked") {
            rl.desc.kind = LayerKind::Baked;
        } else {
            return make_error(ErrorCode::Malformed, "release manifest: layer kind '" + kind + "'");
        }
        if (tiling != "utm") return make_error(ErrorCode::Unsupported, "release manifest: tiling '" + tiling + "'");
        if (avail == "prebaked") {
            rl.availability = Availability::Prebaked;
        } else if (avail == "on_demand") {
            rl.availability = Availability::OnDemand;
        } else if (avail == "unavailable") {
            rl.availability = Availability::Unavailable;
        } else {
            return make_error(ErrorCode::Malformed, "release manifest: availability '" + avail + "'");
        }
        const json* levels = rd.array(l, "levels");
        const json* inputs = rd.array(l, "inputs");
        const json* cov = rd.field(l, "coverage");
        if (!levels || !inputs || !cov) return fail();
        for (const json& lv : *levels) {
            if (!lv.is_number_unsigned() || lv.get<std::uint64_t>() > static_cast<std::uint64_t>(TileKey::kMaxLevel)) {
                return make_error(ErrorCode::Malformed, "release manifest: bad level");
            }
            rl.desc.levels |= level_bit(static_cast<int>(lv.get<std::uint64_t>()));
        }
        for (const json& in : *inputs) {
            if (!in.is_string()) return make_error(ErrorCode::Malformed, "release manifest: bad input");
            rl.desc.inputs.push_back(in.get<std::string>());
        }
        if (!rd.boolean(*cov, "global", rl.coverage.global)) return fail();
        const json* zones = rd.array(*cov, "zones");
        if (!zones) return fail();
        for (const json& z : *zones) {
            std::string scheme;
            if (!rd.str(z, "scheme", scheme)) return fail();
            const auto zone = parse_scheme(scheme);
            if (!zone) return make_error(ErrorCode::Malformed, "release manifest: scheme '" + scheme + "'");
            ZoneCoverage zc{*zone, {}};
            const json* runs = rd.array(z, "runs");
            if (!runs) return fail();
            for (const json& r : *runs) {
                if (!r.is_array() || r.size() != 3) return make_error(ErrorCode::Malformed, "release manifest: bad run");
                std::int32_t vals[3] = {};
                for (std::size_t i = 0; i < 3; ++i) {
                    if (!r[i].is_number_integer()) return make_error(ErrorCode::Malformed, "release manifest: bad run");
                    const std::int64_t x = r[i].get<std::int64_t>();
                    if (x < TileKey::kIndexMin || x > TileKey::kIndexMax) {
                        return make_error(ErrorCode::Malformed, "release manifest: run out of range");
                    }
                    vals[i] = static_cast<std::int32_t>(x);
                }
                if (vals[1] > vals[2]) return make_error(ErrorCode::Malformed, "release manifest: bad run");
                zc.runs.push_back(CoverageRun{vals[0], vals[1], vals[2]});
            }
            rl.coverage.zones.push_back(std::move(zc));
        }
        m.layers.push_back(std::move(rl));
    }

    const json* cls = rd.array(j, "classifiers");
    if (!cls) return fail();
    for (const json& c : *cls) {
        ClassifierRef cr;
        if (!rd.str(c, "name", cr.name) || !rd.hash(c, "table_hash", cr.table_hash)) return fail();
        m.classifiers.push_back(std::move(cr));
    }
    return m;
}

} // namespace g2m
