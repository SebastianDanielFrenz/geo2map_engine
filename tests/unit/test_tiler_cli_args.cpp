// tests/unit/test_tiler_cli_args.cpp - g2m_tiler's argv parsing
// (apps/g2m_tiler/cli_args.h, G1c I7): the `--pin` boolean-switch flag added
// alongside the value-taking flags, and that its introduction leaves the
// pre-existing "every --flag consumes exactly one following value"
// behaviour unchanged for every other flag.
//
// cli_args.h is header-only (inline functions) precisely so this test can
// include it directly, without a new library target or a second definition
// of main() - see the header's own top comment.

#include "cli_args.h"

#include <catch2/catch_test_macros.hpp>

using g2m_tiler::Args;
using g2m_tiler::parse_args;
using g2m_tiler::split_dir_scope;

namespace {

std::vector<char*> to_argv(std::vector<std::string>& storage) {
    std::vector<char*> argv;
    argv.reserve(storage.size());
    for (auto& s : storage) argv.push_back(s.data());
    return argv;
}

} // namespace

TEST_CASE("cli_args: a bool_flags entry is a presence-only switch", "[import][cli]") {
    std::vector<std::string> storage = {"g2m_tiler", "import", "sel#name", "--store", "DIR", "--pin", "--scope",
                                         "s1"};
    auto argv = to_argv(storage);
    const Args args = parse_args(static_cast<int>(argv.size()), argv.data(), 2, {"pin"});

    CHECK(args.flag("pin"));
    REQUIRE(args.get("store").has_value());
    CHECK(*args.get("store") == "DIR");
    REQUIRE(args.get("scope").has_value());
    CHECK(*args.get("scope") == "s1"); // --pin did not eat "--scope" as its own value
    REQUIRE(args.positional.size() == 1);
    CHECK(args.positional[0] == "sel#name");
}

TEST_CASE("cli_args: --pin absent is false, and works at the end of argv", "[import][cli]") {
    {
        std::vector<std::string> storage = {"g2m_tiler", "import", "sel#name", "--store", "DIR"};
        auto argv = to_argv(storage);
        const Args args = parse_args(static_cast<int>(argv.size()), argv.data(), 2, {"pin"});
        CHECK_FALSE(args.flag("pin"));
    }
    {
        std::vector<std::string> storage = {"g2m_tiler", "import", "--store", "DIR", "--pin"};
        auto argv = to_argv(storage);
        const Args args = parse_args(static_cast<int>(argv.size()), argv.data(), 2, {"pin"});
        CHECK(args.flag("pin"));
        REQUIRE(args.get("store").has_value());
        CHECK(*args.get("store") == "DIR");
    }
}

TEST_CASE("cli_args: without bool_flags, --pin behaves like any value flag (pre-I7 behaviour)", "[import][cli]") {
    std::vector<std::string> storage = {"g2m_tiler", "import", "--pin", "--store", "DIR"};
    auto argv = to_argv(storage);
    const Args args = parse_args(static_cast<int>(argv.size()), argv.data(), 2 /* no bool_flags */);
    // "--pin" swallows "--store" as its own value, and "DIR" becomes
    // positional - exactly the old (pre-boolean-switch) parser's behaviour,
    // confirming bool_flags is opt-in per key rather than a global change.
    REQUIRE(args.get("pin").has_value());
    CHECK(*args.get("pin") == "--store");
    CHECK_FALSE(args.get("store").has_value());
    REQUIRE(args.positional.size() == 1);
    CHECK(args.positional[0] == "DIR");
}

TEST_CASE("cli_args: repeatable value flags (e.g. --pkt) are unaffected by bool_flags", "[import][cli]") {
    std::vector<std::string> storage = {"g2m_tiler", "import", "--pkt", "a.zip", "--pkt", "b.zip", "--pin"};
    auto argv = to_argv(storage);
    const Args args = parse_args(static_cast<int>(argv.size()), argv.data(), 2, {"pin"});
    CHECK(args.flag("pin"));
    int pkt_count = 0;
    for (const auto& [k, v] : args.flags) {
        if (k == "pkt") ++pkt_count;
    }
    CHECK(pkt_count == 2);
}

// split_dir_scope (PLAN.md G2.0 Task E, diff-scopes's --a/--b): splits on the
// LAST ':' so a Windows drive-letter path still parses correctly.
TEST_CASE("cli_args: split_dir_scope splits on the last ':'", "[import][cli]") {
    {
        const auto r = split_dir_scope("S:\\claude_code\\geo2map_cache\\home:home-2026-09");
        REQUIRE(r.has_value());
        CHECK(r->first == "S:\\claude_code\\geo2map_cache\\home");
        CHECK(r->second == "home-2026-09");
    }
    {
        // A scope name is never allowed to contain ':', so this is
        // unambiguous even though the dir itself has an earlier ':'.
        const auto r = split_dir_scope("C:/a/b:scope");
        REQUIRE(r.has_value());
        CHECK(r->first == "C:/a/b");
        CHECK(r->second == "scope");
    }
    {
        // A relative/POSIX-style dir with no drive letter still works: the
        // one ':' present is the DIR/SCOPE boundary.
        const auto r = split_dir_scope("relative/dir:scope");
        REQUIRE(r.has_value());
        CHECK(r->first == "relative/dir");
        CHECK(r->second == "scope");
    }
}

TEST_CASE("cli_args: split_dir_scope rejects missing/empty dir or scope", "[import][cli]") {
    CHECK_FALSE(split_dir_scope("no-colon-at-all").has_value());
    CHECK_FALSE(split_dir_scope(":scope").has_value());   // empty dir
    CHECK_FALSE(split_dir_scope("dir:").has_value());     // empty scope
    CHECK_FALSE(split_dir_scope("").has_value());
}
