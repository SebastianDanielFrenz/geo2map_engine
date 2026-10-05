// INTERNAL: the small strict JSON reader shared by the data-file loaders (road rules, carve parameters, classifier
// tables): the first error wins, every message names the key path, integers are checked against a range.
#pragma once
#include <cstdint>
#include <initializer_list>
#include <nlohmann/json.hpp>
#include <string>

namespace g2m::strict {
using json = nlohmann::json;
struct Reader {
    std::string err;
    bool fail(std::string m) { if (err.empty()) err = std::move(m); return false; }
    bool object(const json& o, const std::string& path) {
        return o.is_object() ? true : fail(path + " must be an object");
    }
    bool keys(const json& o, const std::string& path, std::initializer_list<const char*> allowed) {
        if (!object(o, path)) return false;
        for (auto it = o.begin(); it != o.end(); ++it) {
            bool ok = false;
            for (const char* a : allowed) ok = ok || it.key() == a;
            if (!ok) return fail("unknown key '" + it.key() + "' in " + path);
        }
        return true;
    }
    const json* field(const json& o, const std::string& path, const char* k) {
        const auto it = o.find(k);
        if (it == o.end()) { fail("missing key '" + std::string(k) + "' in " + path); return nullptr; }
        return &*it;
    }
    bool integer(const json& o, const std::string& path, const char* k, std::int64_t lo, std::int64_t hi, std::int32_t& out,
        bool optional = false) {
        const auto it = o.find(k);
        if (it == o.end()) {
            if (optional) { out = 0; return true; }
            return fail("missing key '" + std::string(k) + "' in " + path);
        }
        if (!it->is_number_integer()) return fail(path + "." + k + " must be an integer");
        const auto v = it->get<std::int64_t>();
        if (v < lo || v > hi)
            return fail(path + "." + k + " out of range [" + std::to_string(lo) + ", " + std::to_string(hi) + "]");
        out = static_cast<std::int32_t>(v); return true;
    }
    bool text(const json& o, const std::string& path, const char* k, std::string& out) {
        const auto* v = field(o, path, k); if (!v) return false;
        if (!v->is_string()) return fail(path + "." + k + " must be a string");
        out = v->get<std::string>(); return true;
    }
    bool boolean(const json& o, const std::string& path, const char* k, bool& out, bool optional = false) {
        const auto it = o.find(k);
        if (it == o.end()) {
            if (optional) { out = false; return true; }
            return fail("missing key '" + std::string(k) + "' in " + path);
        }
        if (!it->is_boolean()) return fail(path + "." + k + " must be a boolean");
        out = it->get<bool>(); return true;
    }
};
} // namespace g2m::strict
