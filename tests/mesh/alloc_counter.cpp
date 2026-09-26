// tests/mesh/alloc_counter.cpp - see alloc_counter.h. Global operator
// new/delete overrides, scoped to this one executable (g2m_mesh_tests):
// route through std::malloc/std::free directly (never recurse back into
// operator new/delete) and count allocations while `g_armed` is set.
//
// The override block below is compiled out entirely when G2M_SANITIZER_BUILD
// is set (the asan/linux-asan-ubsan/linux-tsan presets): ASan's/TSan's own
// runtimes (clang_rt.asan_cxx / clang_rt.tsan_cxx) already define these same
// global operator new/delete symbols, so a second definition in this
// translation unit is a duplicate-symbol LINK ERROR (lld-link/ld.lld both
// reject two definitions of e.g. `operator delete(void*)`) - not a silently
// shadowed/ignored override. There is also nothing useful to count under a
// sanitizer build even if it linked: ASan's redzone/quarantine allocator is
// the one actually servicing every `new`, entirely separate from this file's
// bookkeeping. AllocCounter::count() therefore always reads 0 when gated off
// - see alloc_counter.h's kAllocCounterAvailable, which callers must check
// rather than trust a bare zero count.
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

#if !G2M_SANITIZER_BUILD

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

#endif // !G2M_SANITIZER_BUILD
