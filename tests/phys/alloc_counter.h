// tests/phys/alloc_counter.h - a global operator-new/delete counting hook,
// scoped to this one test executable (g2m_phys_tests), for
// test_fill_physics_heights.cpp's "fill_physics_heights allocates nothing"
// check (PLAN.md R2.2 section 1a/4a). Single-threaded test code only - no
// thread-safety attempted, unlike physics_sim's own debug_alloc (which must
// be job-system safe); see alloc_counter.cpp for the actual operator
// new/delete overrides, GATED OFF entirely on a sanitizer build - see
// kAllocCounterAvailable below. Exact copy of tests/mesh/alloc_counter.h's
// own pattern, deliberately duplicated per executable (see
// tests/mesh/CMakeLists.txt's own comment) so no other test binary's
// allocations are ever counted here.
#pragma once

#include <cstddef>

#ifndef G2M_SANITIZER_BUILD
#error "G2M_SANITIZER_BUILD is not defined - this target must link g2m_options"
#endif

namespace g2m_test {

// False under a sanitizer build (ASan/UBSan/TSan): alloc_counter.cpp does
// NOT override global operator new/delete there (doing so is a duplicate-
// symbol LINK ERROR against the sanitizer runtime's own overrides), so
// AllocCounter::count() always reads 0 regardless of what actually
// allocated. A caller must check this before trusting a zero count - never
// assume a bare `count() == 0` means "nothing allocated" without it.
inline constexpr bool kAllocCounterAvailable = !G2M_SANITIZER_BUILD;

struct AllocCounter {
    static void reset();
    static void arm();
    static void disarm();
    static std::size_t count(); // allocations observed while armed, since the last reset() - meaningful only when kAllocCounterAvailable
};

// RAII: arms on construction, disarms on destruction.
class ScopedAllocCounter {
public:
    ScopedAllocCounter() {
        AllocCounter::reset();
        AllocCounter::arm();
    }
    ~ScopedAllocCounter() { AllocCounter::disarm(); }
    ScopedAllocCounter(const ScopedAllocCounter&) = delete;
    ScopedAllocCounter& operator=(const ScopedAllocCounter&) = delete;
    std::size_t count() const { return AllocCounter::count(); }
};

} // namespace g2m_test
