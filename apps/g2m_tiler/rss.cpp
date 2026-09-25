// apps/g2m_tiler/rss.cpp - see rss.h.

#include "rss.h"

#if defined(_WIN32)
#include <windows.h>

#include <psapi.h>
#else
#include <cstdio>
#include <cstdlib>
#include <cstring>
#endif

namespace g2m_tiler {

#if defined(_WIN32)

std::uint64_t peak_rss_bytes() {
    PROCESS_MEMORY_COUNTERS counters{};
    counters.cb = sizeof(counters);
    // K32* resolves directly from kernel32.dll (available since Vista) - no
    // need to link psapi.lib the way plain GetProcessMemoryInfo would.
    if (!K32GetProcessMemoryInfo(GetCurrentProcess(), &counters, sizeof(counters))) return 0;
    return static_cast<std::uint64_t>(counters.PeakWorkingSetSize);
}

#else

std::uint64_t peak_rss_bytes() {
    // /proc/self/status's "VmHWM:" line, in kB (Linux only - the only
    // non-Windows leg this project builds on, WSL).
    std::FILE* f = std::fopen("/proc/self/status", "r");
    if (f == nullptr) return 0;
    std::uint64_t kb = 0;
    char line[256];
    while (std::fgets(line, sizeof(line), f) != nullptr) {
        if (std::strncmp(line, "VmHWM:", 6) == 0) {
            kb = static_cast<std::uint64_t>(std::strtoull(line + 6, nullptr, 10));
            break;
        }
    }
    std::fclose(f);
    return kb * 1024;
}

#endif

} // namespace g2m_tiler
