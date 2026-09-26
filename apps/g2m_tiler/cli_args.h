// apps/g2m_tiler/cli_args.h - minimal argv parsing shared by g2m_tiler's
// main() (main.cpp) and its own unit test (tests/unit/test_tiler_cli_args.cpp,
// G1c I7). No CLI-parsing dependency (PLAN.md's "no new dependencies").
// Header-only (inline functions) so the test can include it directly without
// a new library target or a second linkable translation unit.
#pragma once

#include <algorithm>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace g2m_tiler {

// Splits a "DIR:SCOPE" argument (e.g. diff-scopes's --a/--b, PLAN.md G2.0
// Task E) on the LAST ':' - a Windows drive letter's own colon ("S:\...")
// sits earlier in the string, and a scope name is never allowed to contain
// ':', so the last occurrence is always the DIR/SCOPE boundary. Returns
// nullopt if there is no ':' at all, or the ':' is the first character
// (empty dir), or it is the last character (empty scope). Header-only
// alongside Args/parse_args above so it can be unit-tested directly.
inline std::optional<std::pair<std::string, std::string>> split_dir_scope(const std::string& s) {
    const auto pos = s.rfind(':');
    if (pos == std::string::npos || pos == 0 || pos + 1 >= s.size()) return std::nullopt;
    return std::make_pair(s.substr(0, pos), s.substr(pos + 1));
}

struct Args {
    std::vector<std::string> positional;
    std::vector<std::pair<std::string, std::string>> flags; // --key value

    std::optional<std::string> get(const std::string& key) const {
        for (const auto& [k, v] : flags) {
            if (k == key) return v;
        }
        return std::nullopt;
    }

    // Presence-only ("boolean switch") flags, e.g. --pin: true iff --key was
    // given at all. Meaningful only for a key also listed in parse_args's
    // own `bool_flags` (otherwise --key would have consumed the following
    // argv token as its value instead of being present with an empty one).
    bool flag(const std::string& key) const { return get(key).has_value(); }
};

// Parses argv[from..] as "--flag value" pairs interspersed with positional
// arguments. Every key in `bool_flags` is instead a presence-only switch: it
// does NOT consume the next token as a value, so a boolean flag can be
// followed by a positional argument or another flag without confusion.
inline Args parse_args(int argc, char** argv, int from, const std::vector<std::string>& bool_flags = {}) {
    Args out;
    for (int i = from; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg.size() > 2 && arg[0] == '-' && arg[1] == '-') {
            const std::string key = arg.substr(2);
            if (std::find(bool_flags.begin(), bool_flags.end(), key) != bool_flags.end()) {
                out.flags.emplace_back(key, "");
                continue;
            }
            if (i + 1 >= argc) {
                out.flags.emplace_back(key, "");
                continue;
            }
            out.flags.emplace_back(key, argv[++i]);
        } else {
            out.positional.push_back(arg);
        }
    }
    return out;
}

} // namespace g2m_tiler
