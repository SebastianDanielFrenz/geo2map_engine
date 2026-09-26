// tests/mesh/alloc_counter.cpp - see alloc_counter.h. Global operator
// new/delete overrides, scoped to this one executable (g2m_mesh_tests):
// route through std::malloc/std::free directly (never recurse back into
// operator new/delete) and count allocations while `g_armed` is set.
#include "alloc_counter.h"

#include <cstdlib>
#include <new>

namespace g2m_test {
namespace {
bool g_armed = false;
std::size_t g_count = 0;
} // namespace

void AllocCounter::reset() { g_count = 0; }
void AllocCounter::arm() { g_armed = true; }
void AllocCounter::disarm() { g_armed = false; }
std::size_t AllocCounter::count() { return g_count; }

} // namespace g2m_test

namespace {

void* counted_alloc(std::size_t size) {
    if (g2m_test::g_armed) ++g2m_test::g_count;
    void* p = std::malloc(size == 0 ? 1 : size);
    if (!p) throw std::bad_alloc();
    return p;
}

void* counted_alloc_nothrow(std::size_t size) noexcept {
    if (g2m_test::g_armed) ++g2m_test::g_count;
    return std::malloc(size == 0 ? 1 : size);
}

void counted_free(void* p) noexcept { std::free(p); }

} // namespace

void* operator new(std::size_t size) { return counted_alloc(size); }
void* operator new[](std::size_t size) { return counted_alloc(size); }
void* operator new(std::size_t size, const std::nothrow_t&) noexcept { return counted_alloc_nothrow(size); }
void* operator new[](std::size_t size, const std::nothrow_t&) noexcept { return counted_alloc_nothrow(size); }

void operator delete(void* p) noexcept { counted_free(p); }
void operator delete[](void* p) noexcept { counted_free(p); }
void operator delete(void* p, std::size_t) noexcept { counted_free(p); }
void operator delete[](void* p, std::size_t) noexcept { counted_free(p); }
void operator delete(void* p, const std::nothrow_t&) noexcept { counted_free(p); }
void operator delete[](void* p, const std::nothrow_t&) noexcept { counted_free(p); }
