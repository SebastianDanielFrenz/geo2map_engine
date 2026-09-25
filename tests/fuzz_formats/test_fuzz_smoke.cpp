// tests/fuzz_formats/test_fuzz_smoke.cpp - every fuzz target body over its
// seed corpus plus deterministic SplitMix64 mutations of it, on every leg
// (the libFuzzer executables themselves exist only on the WSL clang legs).
// A property violation aborts the test executable.

#include "fuzz_targets.h"

#include "splitmix64.h"

#include <catch2/catch_test_macros.hpp>

#include <string>

namespace {

using Bytes = std::vector<std::uint8_t>;

Bytes mutate(const Bytes& in, g2m_test::SplitMix64& r) {
    Bytes b = in;
    const std::uint64_t kind = r.next() % 6;
    const std::uint64_t pick = r.next();
    const std::size_t pos = b.empty() ? 0 : static_cast<std::size_t>(pick % b.size());
    const std::uint64_t value = r.next();
    switch (kind) {
    case 0: // flip one bit
        if (!b.empty()) b[pos] = static_cast<std::uint8_t>(b[pos] ^ (1u << (value % 8)));
        break;
    case 1: // overwrite one byte
        if (!b.empty()) b[pos] = static_cast<std::uint8_t>(value);
        break;
    case 2: // truncate
        b.resize(pos);
        break;
    case 3: // insert one byte
        b.insert(b.begin() + static_cast<std::ptrdiff_t>(pos), static_cast<std::uint8_t>(value));
        break;
    case 4: // delete a run
        if (!b.empty()) {
            const std::size_t n = std::min<std::size_t>(b.size() - pos, 1 + static_cast<std::size_t>(value % 16));
            b.erase(b.begin() + static_cast<std::ptrdiff_t>(pos), b.begin() + static_cast<std::ptrdiff_t>(pos + n));
        }
        break;
    default: // a little-endian length/count field set to an extreme value
        for (std::size_t i = 0; i < 4 && pos + i < b.size(); ++i) b[pos + i] = (value & 1) ? 0xff : 0x00;
        break;
    }
    return b;
}

} // namespace

TEST_CASE("fuzz targets: seeds and deterministic mutations never crash or break a round trip", "[fuzz][layer]") {
    for (const auto& t : g2m_fuzz::kTargets) {
        INFO("target " << std::string(t.name));
        const auto seeds = g2m_fuzz::seeds(t.target);
        REQUIRE(!seeds.empty());
        g2m_test::SplitMix64 r(0xf022 + static_cast<std::uint64_t>(t.target));
        for (const Bytes& s : seeds) {
            g2m_fuzz::run(t.target, s);
            Bytes cur = s;
            for (int i = 0; i < 300; ++i) {
                // Mostly mutate the seed, sometimes stack mutations.
                const bool stack = r.next() % 4 == 0;
                cur = mutate(stack ? cur : s, r);
                g2m_fuzz::run(t.target, cur);
            }
        }
        g2m_fuzz::run(t.target, {});
    }
    SUCCEED();
}
