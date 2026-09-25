// src/import/report.cpp - see g2m/import/report.h.

#include "g2m/import/report.h"

#include <sstream>

namespace g2m::import {

std::string summarize(const ImportReport& report) {
    std::ostringstream out;
    if (!report.layer.empty()) out << report.layer << ": ";
    out << report.tiles_written << " written, " << report.tiles_skipped << " skipped, " << report.warnings
        << " warnings, " << report.wall_seconds << "s";
    return out.str();
}

} // namespace g2m::import
