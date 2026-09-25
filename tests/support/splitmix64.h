// tests/support/splitmix64.h - a tiny counter-free PRNG with a fixed,
// specified output sequence (Steele/Lea/Flood 2014), so every test and
// generator draws the SAME inputs on every compiler/OS. std::
// distributions are implementation-defined and must not be used for
// anything whose values are compared across platforms.
#pragma once

#include <cstdint>

namespace g2m_test {

class SplitMix64 {
public:
    explicit SplitMix64(std::uint64_t seed) : s_(seed) {}

    std::uint64_t next() {
        std::uint64_t z = (s_ += 0x9e3779b97f4a7c15ull);
        z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
        z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
        return z ^ (z >> 31);
    }
    // Uniform in [0, 1): the top 53 bits (exact, no rounding).
    double unit() { return static_cast<double>(next() >> 11) * 0x1.0p-53; }
    // Uniform in [lo, hi): one multiply and one add, fixed order.
    double uniform(double lo, double hi) { return lo + (hi - lo) * unit(); }

private:
    std::uint64_t s_;
};

} // namespace g2m_test
