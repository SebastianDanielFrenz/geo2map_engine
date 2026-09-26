// src/layer/region.cpp - see g2m/layer/region.h and docs/formats/regions.md.

#include "g2m/layer/region.h"

#include "g2m/core/hash.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <climits>
#include <cstdint>

namespace g2m {
namespace {

using json = nlohmann::json;

bool valid_region_name(std::string_view name) {
    if (name.empty() || name.size() > 64) return false;
    return std::all_of(name.begin(), name.end(),
                       [](char c) { return (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '_'; });
}

const char* resolved_kind_name(ResolvedPartKind k) { return k == ResolvedPartKind::OsmAdmin ? "osm_admin" : "utm_bbox"; }

// ---- strict JSON reader: UNKNOWN KEYS ARE REJECTED (region.h's own
// contract - the opposite of release.cpp's Reader, which ignores them for
// forward compatibility). Otherwise the same shape/style as release.cpp's
// own Reader. ----

struct Reader {
    std::string err;
    bool fail(const std::string& what) {
        if (err.empty()) err = what;
        return false;
    }
    bool no_unknown_keys(const json& o, std::initializer_list<const char*> allowed) {
        if (!o.is_object()) return fail("expected an object");
        for (const auto& item : o.items()) {
            const std::string& key = item.key();
            const bool known = std::any_of(allowed.begin(), allowed.end(), [&](const char* k) { return key == k; });
            if (!known) return fail("unknown key '" + key + "'");
        }
        return true;
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
    bool opt_str(const json& o, const char* k, std::optional<std::string>& out) {
        if (!o.is_object()) return fail("expected an object");
        const auto it = o.find(k);
        if (it == o.end()) {
            out.reset();
            return true;
        }
        if (!it->is_string()) return fail(std::string("'") + k + "' must be a string");
        out = it->get_ref<const std::string&>();
        return true;
    }
    bool integer(const json& o, const char* k, std::int64_t lo, std::int64_t hi, std::int64_t& out) {
        const json* v = field(o, k);
        if (!v) return false;
        if (!v->is_number_integer() || v->get<std::int64_t>() < lo || v->get<std::int64_t>() > hi) {
            return fail(std::string("'") + k + "' must be an integer in [" + std::to_string(lo) + ", " +
                        std::to_string(hi) + "]");
        }
        out = v->get<std::int64_t>();
        return true;
    }
    bool opt_int(const json& o, const char* k, std::int64_t lo, std::int64_t hi, std::optional<int>& out) {
        if (!o.is_object()) return fail("expected an object");
        const auto it = o.find(k);
        if (it == o.end()) {
            out.reset();
            return true;
        }
        if (!it->is_number_integer() || it->get<std::int64_t>() < lo || it->get<std::int64_t>() > hi) {
            return fail(std::string("'") + k + "' must be an integer in [" + std::to_string(lo) + ", " +
                        std::to_string(hi) + "]");
        }
        out = static_cast<int>(it->get<std::int64_t>());
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
    const json* array(const json& o, const char* k) {
        const json* v = field(o, k);
        if (!v) return nullptr;
        if (!v->is_array()) {
            fail(std::string("'") + k + "' must be an array");
            return nullptr;
        }
        return v;
    }
    bool hash32(const json& o, const char* k, ContentHash& out) {
        std::string s;
        if (!str(o, k, s)) return false;
        const auto h = content_hash_from_hex(s);
        if (!h) return fail(std::string("'") + k + "' must be 64 lower-case hex digits");
        out = *h;
        return true;
    }
    // 16 lower-case hex digits (an Fnv1a64 value).
    bool hash64(const json& o, const char* k, std::uint64_t& out) {
        std::string s;
        if (!str(o, k, s)) return false;
        if (s.size() != 16 || !std::all_of(s.begin(), s.end(), [](char c) {
                return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
            })) {
            return fail(std::string("'") + k + "' must be 16 lower-case hex digits");
        }
        std::uint64_t v = 0;
        for (char c : s) v = (v << 4) | static_cast<std::uint64_t>(c <= '9' ? c - '0' : c - 'a' + 10);
        out = v;
        return true;
    }
};

} // namespace

// ---- regions.json ----

Result<std::vector<RegionSpec>> parse_regions(std::string_view text) {
    const json j = json::parse(text.begin(), text.end(), nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return make_error(ErrorCode::Malformed, "regions: not valid JSON");
    Reader rd;
    const auto fail = [&]() { return make_error(ErrorCode::InvalidArgument, "regions: " + rd.err); };

    if (!rd.no_unknown_keys(j, {"format", "regions"})) return fail();
    std::string format;
    if (!rd.str(j, "format", format)) return fail();
    if (format != "g2m.regions/1") return make_error(ErrorCode::Unsupported, "regions: format '" + format + "'");

    const json* regions = rd.array(j, "regions");
    if (!regions) return fail();

    std::vector<RegionSpec> out;
    for (const json& rj : *regions) {
        if (!rd.no_unknown_keys(rj, {"name", "scheme", "halo_m", "parts"})) return fail();
        RegionSpec spec;
        std::uint64_t halo = 0;
        if (!rd.str(rj, "name", spec.name) || !rd.str(rj, "scheme", spec.scheme) ||
            !rd.uint(rj, "halo_m", 20000, halo)) {
            return fail();
        }
        spec.halo_m = static_cast<std::uint32_t>(halo);
        if (!valid_region_name(spec.name)) {
            return make_error(ErrorCode::InvalidArgument, "regions: bad name '" + spec.name + "'");
        }
        if (std::any_of(out.begin(), out.end(), [&](const RegionSpec& s) { return s.name == spec.name; })) {
            return make_error(ErrorCode::InvalidArgument, "regions: duplicate name '" + spec.name + "'");
        }
        if (!geo::parse_utm_scheme(spec.scheme)) {
            return make_error(ErrorCode::InvalidArgument, "regions: region '" + spec.name + "': scheme '" +
                                                                spec.scheme + "'");
        }

        const json* parts = rd.array(rj, "parts");
        if (!parts) return fail();
        if (parts->empty()) {
            return make_error(ErrorCode::InvalidArgument, "regions: region '" + spec.name + "' has no parts");
        }
        for (const json& pj : *parts) {
            if (!rd.no_unknown_keys(pj, {"kind", "relation", "expect_name", "expect_admin_level", "west", "south",
                                          "east", "north"})) {
                return fail();
            }
            std::string kind;
            if (!rd.str(pj, "kind", kind)) return fail();
            if (kind == "osm_admin") {
                OsmAdminPart p;
                if (!rd.integer(pj, "relation", 1, INT64_MAX, p.relation)) return fail();
                if (!rd.opt_str(pj, "expect_name", p.expect_name)) return fail();
                if (!rd.opt_int(pj, "expect_admin_level", INT32_MIN, INT32_MAX, p.expect_admin_level)) return fail();
                spec.parts.push_back(std::move(p));
            } else if (kind == "utm_bbox") {
                // A generous sanity bound on the metre fields (harmless: any
                // real UTM zone's extent is a few hundred km), chosen so
                // west/south/east/north * 1000 (millimetres, g2m::geom's own
                // unit) stays far inside geom::exact's |value| < 2^62
                // contract with room to spare.
                constexpr std::int64_t kLim = std::int64_t{1} << 33;
                UtmBboxPart p;
                if (!rd.integer(pj, "west", -kLim, kLim, p.west) || !rd.integer(pj, "south", -kLim, kLim, p.south) ||
                    !rd.integer(pj, "east", -kLim, kLim, p.east) || !rd.integer(pj, "north", -kLim, kLim, p.north)) {
                    return fail();
                }
                if (!(p.west < p.east && p.south < p.north)) {
                    return make_error(ErrorCode::InvalidArgument,
                                       "regions: region '" + spec.name + "': utm_bbox needs west < east, south < north");
                }
                spec.parts.push_back(std::move(p));
            } else {
                return make_error(ErrorCode::InvalidArgument,
                                   "regions: region '" + spec.name + "': part kind '" + kind + "'");
            }
        }
        out.push_back(std::move(spec));
    }
    return out;
}

Result<std::pair<std::string, std::string>> split_selector(std::string_view selector) {
    const std::size_t hash = selector.find('#');
    if (hash == std::string_view::npos) {
        return make_error(ErrorCode::InvalidArgument, "region selector: missing '#' (want \"path#name\")");
    }
    const std::string_view path = selector.substr(0, hash);
    const std::string_view name = selector.substr(hash + 1);
    if (path.empty() || name.empty()) {
        return make_error(ErrorCode::InvalidArgument, "region selector: empty path or name");
    }
    return std::pair<std::string, std::string>(std::string(path), std::string(name));
}

// ---- g2m.resolved_region/1 ----

std::string to_json(const ResolvedRegion& r) {
    json j = json::object();
    j["format"] = "g2m.resolved_region/1";
    j["name"] = r.name;
    j["scheme"] = r.scheme;
    j["level"] = r.level;
    j["halo_m"] = r.halo_m;
    json parts = json::array();
    for (const ResolvedPart& p : r.parts) {
        json pj = json::object();
        pj["kind"] = resolved_kind_name(p.kind);
        pj["id"] = p.id;
        pj["name"] = p.name;
        if (p.admin_level) pj["admin_level"] = *p.admin_level;
        pj["vertices"] = p.vertices;
        pj["geometry_hash"] = to_hex(p.geometry_hash);
        pj["role_mismatches"] = p.role_mismatches;
        parts.push_back(std::move(pj));
    }
    j["parts"] = std::move(parts);
    j["pbf_name"] = r.pbf_name;
    j["pbf_sha256"] = to_hex(r.pbf_sha256);
    const auto runs_json = [](const std::vector<CoverageRun>& runs) {
        json a = json::array();
        for (const CoverageRun& run : runs) a.push_back(json::array({run.y, run.x0, run.x1}));
        return a;
    };
    j["tiles"] = runs_json(r.tiles);
    j["interior"] = runs_json(r.interior);
    j["tile_count"] = r.tile_count;
    return j.dump(-1, ' ', false, json::error_handler_t::replace);
}

Result<ResolvedRegion> parse_resolved_region(std::string_view text) {
    const json j = json::parse(text.begin(), text.end(), nullptr, /*allow_exceptions=*/false);
    if (j.is_discarded()) return make_error(ErrorCode::Malformed, "resolved region: not valid JSON");
    Reader rd;
    const auto fail = [&]() { return make_error(ErrorCode::InvalidArgument, "resolved region: " + rd.err); };

    if (!rd.no_unknown_keys(j, {"format", "name", "scheme", "level", "halo_m", "parts", "pbf_name", "pbf_sha256",
                                "tiles", "interior", "tile_count"})) {
        return fail();
    }
    std::string format;
    if (!rd.str(j, "format", format)) return fail();
    if (format != "g2m.resolved_region/1") {
        return make_error(ErrorCode::Unsupported, "resolved region: format '" + format + "'");
    }

    ResolvedRegion r;
    std::uint64_t level = 0, halo = 0;
    if (!rd.str(j, "name", r.name) || !rd.str(j, "scheme", r.scheme) ||
        !rd.uint(j, "level", static_cast<std::uint64_t>(TileKey::kMaxLevel), level) ||
        !rd.uint(j, "halo_m", 0xffffffffu, halo) || !rd.str(j, "pbf_name", r.pbf_name) ||
        !rd.hash32(j, "pbf_sha256", r.pbf_sha256) || !rd.uint(j, "tile_count", UINT64_MAX, r.tile_count)) {
        return fail();
    }
    r.level = static_cast<int>(level);
    r.halo_m = static_cast<std::uint32_t>(halo);
    if (!geo::parse_utm_scheme(r.scheme)) {
        return make_error(ErrorCode::InvalidArgument, "resolved region: scheme '" + r.scheme + "'");
    }

    const json* parts = rd.array(j, "parts");
    if (!parts) return fail();
    for (const json& pj : *parts) {
        if (!rd.no_unknown_keys(pj, {"kind", "id", "name", "admin_level", "vertices", "geometry_hash",
                                      "role_mismatches"})) {
            return fail();
        }
        ResolvedPart p;
        std::string kind;
        std::uint64_t vertices = 0, role_mismatches = 0;
        if (!rd.str(pj, "kind", kind) || !rd.integer(pj, "id", INT64_MIN, INT64_MAX, p.id) ||
            !rd.str(pj, "name", p.name) || !rd.opt_int(pj, "admin_level", INT32_MIN, INT32_MAX, p.admin_level) ||
            !rd.uint(pj, "vertices", UINT64_MAX, vertices) || !rd.hash64(pj, "geometry_hash", p.geometry_hash) ||
            !rd.uint(pj, "role_mismatches", UINT32_MAX, role_mismatches)) {
            return fail();
        }
        p.vertices = vertices;
        p.role_mismatches = static_cast<std::uint32_t>(role_mismatches);
        if (kind == "osm_admin") {
            p.kind = ResolvedPartKind::OsmAdmin;
        } else if (kind == "utm_bbox") {
            p.kind = ResolvedPartKind::UtmBbox;
        } else {
            return make_error(ErrorCode::InvalidArgument, "resolved region: part kind '" + kind + "'");
        }
        r.parts.push_back(std::move(p));
    }

    const auto read_runs = [&](const char* key, std::vector<CoverageRun>& out) {
        const json* runs = rd.array(j, key);
        if (!runs) return false;
        for (const json& rr : *runs) {
            if (!rr.is_array() || rr.size() != 3) {
                rd.fail(std::string("'") + key + "' has a bad run");
                return false;
            }
            std::int32_t vals[3] = {};
            for (std::size_t i = 0; i < 3; ++i) {
                if (!rr[i].is_number_integer()) {
                    rd.fail(std::string("'") + key + "' has a bad run");
                    return false;
                }
                const std::int64_t x = rr[i].get<std::int64_t>();
                if (x < TileKey::kIndexMin || x > TileKey::kIndexMax) {
                    rd.fail(std::string("'") + key + "' run out of range");
                    return false;
                }
                vals[i] = static_cast<std::int32_t>(x);
            }
            if (vals[1] > vals[2]) {
                rd.fail(std::string("'") + key + "' has a bad run");
                return false;
            }
            out.push_back(CoverageRun{vals[0], vals[1], vals[2]});
        }
        return true;
    };
    if (!read_runs("tiles", r.tiles)) return fail();
    if (!read_runs("interior", r.interior)) return fail();

    return r;
}

namespace {
std::vector<TileKey> keys_from_runs(const std::string& scheme, int level, const std::vector<CoverageRun>& runs) {
    std::vector<TileKey> out;
    const std::optional<geo::UtmZone> zone = geo::parse_utm_scheme(scheme);
    if (!zone) return out;
    for (const CoverageRun& run : runs) {
        for (std::int32_t x = run.x0; x <= run.x1; ++x) out.push_back(TileKey{*zone, level, x, run.y});
    }
    return out;
}
} // namespace

std::vector<TileKey> tile_keys(const ResolvedRegion& r) { return keys_from_runs(r.scheme, r.level, r.tiles); }

std::vector<TileKey> interior_tile_keys(const ResolvedRegion& r) {
    return keys_from_runs(r.scheme, r.level, r.interior);
}

} // namespace g2m
