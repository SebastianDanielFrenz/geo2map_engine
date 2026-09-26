// tests/mesh/alloc_counter.h - a global operator-new/delete counting hook,
// scoped to this one test executable (g2m_mesh_tests), for
// test_terrain_chunk.cpp's "build_chunk's second call allocates nothing"
// check (PLAN.md G2.3). Single-threaded test code only - no thread-safety
// attempted, unlike physics_sim's own debug_alloc (which must be job-system
// safe); see alloc_counter.cpp for the actual operator new/delete overrides.
#pragma once

#include <cstddef>

namespace g2m_test {

struct AllocCounter {
    static void reset();
    static void arm();
    static void disarm();
    static std::size_t count(); // allocations observed while armed, since the last reset()
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
