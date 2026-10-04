// tools/osm_tag_probe/main.cpp - g2m_osm_tag_probe CLI (street-level fidelity
// plan phase P0). Resolves a region against a PBF, scans the PBF for the tag
// families the plan cares about and writes a markdown and a JSON report.
//
//   g2m_osm_tag_probe --pbf FILE --region REGIONS.json#NAME --out-md FILE --out-json FILE
//
// Reads only. Reports carry no timing or host data; timing and peak memory go
// to stderr.
#include "g2m/import/region_resolve.h"
#include "g2m/layer/region.h"
#include "rss.h"
#include "scan.h"
#include "tag_probe.h"

#include <chrono>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <iterator>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>

namespace {

std::optional<std::string> read_text(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return std::nullopt;
    return std::string(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

bool write_text(const std::string& path, const std::string& text) {
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) return false;
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
    return static_cast<bool>(f);
}

int usage() {
    std::cerr << "usage: g2m_osm_tag_probe --pbf FILE --region REGIONS.json#NAME --out-md FILE --out-json FILE\n";
    return 2;
}

double seconds_since(std::chrono::steady_clock::time_point t) {
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t).count();
}

} // namespace

int main(int argc, char** argv) {
    std::string pbf, region_sel, out_md, out_json;
    for (int i = 1; i < argc; ++i) {
        const std::string_view a = argv[i];
        const auto next = [&](std::string& dst) {
            if (i + 1 >= argc) return false;
            dst = argv[++i];
            return true;
        };
        if (a == "--pbf") { if (!next(pbf)) return usage(); }
        else if (a == "--region") { if (!next(region_sel)) return usage(); }
        else if (a == "--out-md") { if (!next(out_md)) return usage(); }
        else if (a == "--out-json") { if (!next(out_json)) return usage(); }
        else return usage();
    }
    if (pbf.empty() || region_sel.empty() || out_md.empty() || out_json.empty()) return usage();

    const auto sel = g2m::split_selector(region_sel);
    if (!sel) { std::cerr << "g2m_osm_tag_probe: " << sel.error().message << "\n"; return 2; }
    const auto regions_text = read_text(sel->first);
    if (!regions_text) { std::cerr << "g2m_osm_tag_probe: cannot read " << sel->first << "\n"; return 1; }
    const auto specs = g2m::parse_regions(*regions_text);
    if (!specs) { std::cerr << "g2m_osm_tag_probe: " << specs.error().message << "\n"; return 1; }
    const g2m::RegionSpec* spec = nullptr;
    for (const auto& s : *specs)
        if (s.name == sel->second) spec = &s;
    if (spec == nullptr) {
        std::cerr << "g2m_osm_tag_probe: region '" << sel->second << "' not found in " << sel->first << "\n";
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const auto resolved = g2m::import::resolve_region(*spec, pbf);
    if (!resolved) { std::cerr << "g2m_osm_tag_probe: resolve: " << resolved.error().message << "\n"; return 1; }
    const double resolve_s = seconds_since(t0);

    const auto t1 = std::chrono::steady_clock::now();
    g2m::tag_probe::Accumulator acc;
    const auto stats = g2m::tag_probe::scan_pbf(pbf, *resolved, acc);
    if (!stats) { std::cerr << "g2m_osm_tag_probe: scan: " << stats.error().message << "\n"; return 1; }
    const double scan_s = seconds_since(t1);

    const g2m::tag_probe::ReportMeta meta = g2m::tag_probe::make_meta(*resolved, *stats);
    if (!write_text(out_md, acc.to_markdown(meta))) { std::cerr << "g2m_osm_tag_probe: cannot write " << out_md << "\n"; return 1; }
    if (!write_text(out_json, acc.to_json(meta))) { std::cerr << "g2m_osm_tag_probe: cannot write " << out_json << "\n"; return 1; }

    std::cerr << "g2m_osm_tag_probe: region '" << resolved->name << "' resolved in " << resolve_s << " s, scanned in "
              << scan_s << " s, peak RSS " << (g2m_tiler::peak_rss_bytes() >> 20) << " MiB\n";
    return 0;
}
