// apps/g2m_tiler/rss.h - peak resident-set-size sampling, for a run's final
// summary line (g2m_tiler's own diagnostic, not part of any g2m library:
// I3-I5's dem/osm/landcover importers can process a whole-country PBF or a
// multi-GB DGM1 delivery, so "how much memory did that actually use" is
// worth printing without needing an external profiler).
#pragma once

#include <cstdint>

namespace g2m_tiler {

// Peak resident set size of THIS process so far, in bytes. 0 if it could
// not be determined (an unsupported platform, or a read failure) - callers
// print it best-effort, never treat 0 as a hard error.
std::uint64_t peak_rss_bytes();

} // namespace g2m_tiler
