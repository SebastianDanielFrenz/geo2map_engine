// tests/fuzz_formats/fuzz_main.cpp - libFuzzer entry point. Compiled once per
// target with G2M_FUZZ_TARGET set to a g2m_fuzz::Target enumerator.

#include "fuzz_targets.h"

#include <cstddef>
#include <cstdint>

#ifndef G2M_FUZZ_TARGET
#error "G2M_FUZZ_TARGET must name a g2m_fuzz::Target enumerator"
#endif

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size);

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t* data, std::size_t size) {
    g2m_fuzz::run(g2m_fuzz::Target::G2M_FUZZ_TARGET, std::span<const std::uint8_t>(data, size));
    return 0;
}
