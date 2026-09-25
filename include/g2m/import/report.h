// g2m/import/report.h - ImportReport: what one g2m_tiler subcommand's run
// produced, for its own human-readable summary line and for a test to
// assert against directly instead of parsing stdout (G1c I2).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace g2m::import {

struct ImportReport {
    std::string layer; // "dem" / "osm" / "landcover" / "" (not layer-specific, e.g. resolve)
    std::uint64_t tiles_written = 0;
    std::uint64_t tiles_skipped = 0; // e.g. already up to date - later increments decide when
    std::uint64_t warnings = 0;
    double wall_seconds = 0.0;
    std::vector<std::string> messages; // free-form diagnostic lines, in emission order

    friend bool operator==(const ImportReport&, const ImportReport&) = default;
};

// One human-readable line, e.g. "dem: 1085 written, 0 skipped, 0 warnings, 12.3s".
std::string summarize(const ImportReport& report);

} // namespace g2m::import
