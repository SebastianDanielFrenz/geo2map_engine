// tests/golden/golden.h - G0 golden hashes (PLAN.md 2.4).
//
// Each entry hashes (FNV-1a 64, g2m::Fnv1a64) the bit patterns of one
// function family's outputs over a fixed SplitMix64 input set. The same
// build must give the same hashes on every leg (clang-cl, cl, Linux clang,
// with or without sanitizers); testdata/golden/g0_golden.txt records them,
// tests/golden/test_golden.cpp checks against it, and tools/ci.ps1 compares
// g2m_golden_dump's output across legs.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace g2m_golden {

struct Entry {
    std::string name;
    std::uint64_t count; // values hashed
    std::uint64_t hash;
};

std::vector<Entry> compute_g0();

// "name count hash\n" per entry, the golden file's line format.
std::string format(const std::vector<Entry>& entries);

} // namespace g2m_golden
