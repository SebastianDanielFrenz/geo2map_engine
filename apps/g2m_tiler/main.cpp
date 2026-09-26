// apps/g2m_tiler/main.cpp - the importer CLI (PLAN.md G1c I2). Manual argv
// parsing (no CLI-parsing dependency - PLAN.md's "no new dependencies").
//
// Subcommands:
//   find-admin --pbf PATH [--name-contains STR] [--admin-level N]
//   resolve PATH#NAME --pbf PATH --out PATH
//   import-dem --resolved PATH --store DIR --scope SCOPE
//              (--pkt PATH [--pkt PATH ...] | --data DIR) [--jobs N] [--km-cache N]
//   import-osm --pbf PATH --resolved PATH --store DIR --scope SCOPE [--jobs N]
//   import-landcover --resolved PATH --store DIR --scope SCOPE
//                     (--tif PATH [--tif PATH ...] | --data DIR) [--jobs N]
//   import PATH#NAME --pbf PATH --store DIR --scope SCOPE
//          (--pkt PATH [--pkt PATH ...] | --dem-data DIR)
//          (--tif PATH [--tif PATH ...] | --wc-data DIR)
//          [--jobs N] [--km-cache N]
//          (I5: resolve -> import-dem -> import-osm -> import-landcover in
//          one call, writing <store>/regions/<name>.resolved.json and
//          <store>/import_report_<scope>.json)
//   stats --store DIR [--scope SCOPE]
//
// Exit codes: 0 success, 1 usage/runtime error, 2 "not implemented yet".
#include "rss.h"

#include "g2m/decode/dgm1.h"
#include "g2m/import/dem.h"
#include "g2m/import/landcover.h"
#include "g2m/import/osm.h"
#include "g2m/import/region_resolve.h"
#include "g2m/layer/region.h"
#include "g2m/server/tile_store.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

struct Args {
    std::vector<std::string> positional;
    std::vector<std::pair<std::string, std::string>> flags; // --key value

    std::optional<std::string> get(const std::string& key) const {
        for (const auto& [k, v] : flags) {
            if (k == key) return v;
        }
        return std::nullopt;
    }
};

// Parses argv[from..] as "--flag value" pairs interspersed with positional
// arguments (a flag always consumes exactly one following value - none of
// this CLI's flags are boolean switches).
Args parse_args(int argc, char** argv, int from) {
    Args out;
    for (int i = from; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg.size() > 2 && arg[0] == '-' && arg[1] == '-') {
            const std::string key = arg.substr(2);
            if (i + 1 >= argc) {
                out.flags.emplace_back(key, "");
                continue;
            }
            out.flags.emplace_back(key, argv[++i]);
        } else {
            out.positional.push_back(arg);
        }
    }
    return out;
}

std::optional<std::string> read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return std::nullopt;
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool write_file(const std::string& path, const std::string& content) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) return false;
    out << content;
    return static_cast<bool>(out);
}

// Scans `dir` for *.tif files (case-insensitive extension) and returns them
// SORTED lexicographically by full path - g2m::import::import_landcover's
// own multi-file search order is meaningful (landcover.h's doc comment), and
// std::filesystem::directory_iterator's own enumeration order is not
// guaranteed stable across platforms/filesystems, so this function - not the
// iterator - is what makes --data reproducible.
std::optional<std::vector<std::filesystem::path>> find_tifs(const std::string& dir) {
    std::vector<std::filesystem::path> out;
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file()) continue;
        std::string ext = entry.path().extension().string();
        std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (ext == ".tif" || ext == ".tiff") out.push_back(entry.path());
    }
    if (ec) return std::nullopt;
    std::sort(out.begin(), out.end());
    return out;
}

// Minimal JSON string escaping for the hand-rolled import_report_*.json
// below (no nlohmann::json dependency in this app - see its own
// CMakeLists.txt comment: g2m_tiler links only g2m_import/g2m_warnings/
// g2m_options). Every value this report ever puts through this is either a
// path/scope/region name (arbitrary but never attacker-controlled - this is
// a local maintainer CLI) or a plain identifier; escaping quotes/backslashes
// and control characters is enough for a valid, if not fully general, JSON
// string.
std::string json_escape(const std::string& s) {
    std::string out;
    out.reserve(s.size() + 2);
    for (unsigned char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            default:
                if (c < 0x20) {
                    char buf[8];
                    std::snprintf(buf, sizeof buf, "\\u%04x", c);
                    out += buf;
                } else {
                    out += static_cast<char>(c);
                }
        }
    }
    return out;
}

int usage() {
    std::cerr << "usage: g2m_tiler <command> [options]\n"
                 "commands:\n"
                 "  find-admin --pbf PATH [--name-contains STR] [--admin-level N]\n"
                 "  resolve PATH#NAME --pbf PATH --out PATH\n"
                 "  import-dem --resolved PATH --store DIR --scope SCOPE\n"
                 "             (--pkt PATH [--pkt PATH ...] | --data DIR)\n"
                 "             [--jobs N] [--km-cache N]\n"
                 "  import-osm --pbf PATH --resolved PATH --store DIR --scope SCOPE [--jobs N]\n"
                 "  import-landcover --resolved PATH --store DIR --scope SCOPE\n"
                 "                   (--tif PATH [--tif ...] | --data DIR) [--jobs N]\n"
                 "  import PATH#NAME --pbf PATH --store DIR --scope SCOPE\n"
                 "         (--pkt PATH [--pkt ...] | --dem-data DIR)\n"
                 "         (--tif PATH [--tif ...] | --wc-data DIR)\n"
                 "         [--jobs N] [--km-cache N]\n"
                 "  stats --store DIR [--scope SCOPE]\n";
    return 1;
}

int cmd_find_admin(const Args& args) {
    const auto pbf = args.get("pbf");
    if (!pbf) {
        std::cerr << "g2m_tiler find-admin: missing --pbf\n";
        return 1;
    }
    std::optional<std::string> name_contains = args.get("name-contains");
    std::optional<int> admin_level;
    if (const auto lvl = args.get("admin-level")) {
        try {
            admin_level = std::stoi(*lvl);
        } catch (const std::exception&) {
            std::cerr << "g2m_tiler find-admin: --admin-level must be an integer\n";
            return 1;
        }
    }
    std::vector<g2m::import::AdminCandidate> candidates;
    try {
        candidates = g2m::import::find_admin(
            *pbf, name_contains ? std::optional<std::string_view>(*name_contains) : std::nullopt, admin_level);
    } catch (const std::exception& e) {
        std::cerr << "g2m_tiler find-admin: " << e.what() << "\n";
        return 1;
    }
    for (const auto& c : candidates) {
        std::cout << c.relation_id << '\t' << (c.admin_level ? std::to_string(*c.admin_level) : "-") << '\t'
                  << c.name << '\n';
    }
    std::cerr << "g2m_tiler find-admin: " << candidates.size() << " candidate(s)\n";
    return 0;
}

int cmd_resolve(const Args& args) {
    if (args.positional.empty()) {
        std::cerr << "g2m_tiler resolve: missing PATH#NAME selector\n";
        return 1;
    }
    const auto selector = g2m::split_selector(args.positional.front());
    if (!selector) {
        std::cerr << "g2m_tiler resolve: " << selector.error().message << "\n";
        return 1;
    }
    const auto pbf = args.get("pbf");
    const auto out_path = args.get("out");
    if (!pbf || !out_path) {
        std::cerr << "g2m_tiler resolve: --pbf and --out are required\n";
        return 1;
    }
    const auto& [regions_path, region_name] = *selector;
    const auto regions_text = read_file(regions_path);
    if (!regions_text) {
        std::cerr << "g2m_tiler resolve: cannot read " << regions_path << "\n";
        return 1;
    }
    const auto specs = g2m::parse_regions(*regions_text);
    if (!specs) {
        std::cerr << "g2m_tiler resolve: " << specs.error().message << "\n";
        return 1;
    }
    const g2m::RegionSpec* spec = nullptr;
    for (const auto& s : *specs) {
        if (s.name == region_name) {
            spec = &s;
            break;
        }
    }
    if (spec == nullptr) {
        std::cerr << "g2m_tiler resolve: region '" << region_name << "' not found in " << regions_path << "\n";
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const auto resolved = g2m::import::resolve_region(*spec, *pbf);
    const auto t1 = std::chrono::steady_clock::now();
    if (!resolved) {
        std::cerr << "g2m_tiler resolve: " << resolved.error().message << "\n";
        return 1;
    }
    if (!write_file(*out_path, g2m::to_json(*resolved))) {
        std::cerr << "g2m_tiler resolve: cannot write " << *out_path << "\n";
        return 1;
    }
    const double wall_s = std::chrono::duration<double>(t1 - t0).count();
    std::uint64_t role_mismatches = 0;
    for (const auto& p : resolved->parts) role_mismatches += p.role_mismatches;
    std::cerr << "g2m_tiler resolve: '" << region_name << "': " << resolved->tile_count << " tiles ("
              << resolved->interior.size() << " interior runs), " << role_mismatches << " role mismatch(es), "
              << wall_s << "s, peak RSS " << (g2m_tiler::peak_rss_bytes() / (1024 * 1024)) << " MiB\n";
    return 0;
}

int cmd_import_dem(const Args& args) {
    const auto resolved_path = args.get("resolved");
    const auto store_dir = args.get("store");
    const auto scope = args.get("scope");
    if (!resolved_path || !store_dir || !scope) {
        std::cerr << "g2m_tiler import-dem: --resolved, --store and --scope are required\n";
        return 1;
    }

    std::vector<std::filesystem::path> pkt_zips;
    for (const auto& [k, v] : args.flags) {
        if (k == "pkt") pkt_zips.emplace_back(v);
    }
    if (pkt_zips.empty()) {
        const auto data_dir = args.get("data");
        if (!data_dir) {
            std::cerr << "g2m_tiler import-dem: need --pkt PATH (repeatable) or --data DIR (containing PKT_*.zip)\n";
            return 1;
        }
        std::error_code ec;
        for (const auto& entry : std::filesystem::directory_iterator(*data_dir, ec)) {
            if (!entry.is_regular_file()) continue;
            std::string lower = entry.path().filename().string();
            std::transform(lower.begin(), lower.end(), lower.begin(),
                            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (lower.size() > 4 && lower.compare(0, 3, "pkt") == 0 && lower.compare(lower.size() - 4, 4, ".zip") == 0) {
                pkt_zips.push_back(entry.path());
            }
        }
        if (ec) {
            std::cerr << "g2m_tiler import-dem: cannot list " << *data_dir << ": " << ec.message() << "\n";
            return 1;
        }
        if (pkt_zips.empty()) {
            std::cerr << "g2m_tiler import-dem: no PKT_*.zip found under " << *data_dir << "\n";
            return 1;
        }
    }

    const auto resolved_text = read_file(*resolved_path);
    if (!resolved_text) {
        std::cerr << "g2m_tiler import-dem: cannot read " << *resolved_path << "\n";
        return 1;
    }
    const auto resolved = g2m::parse_resolved_region(*resolved_text);
    if (!resolved) {
        std::cerr << "g2m_tiler import-dem: " << resolved.error().message << "\n";
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    auto catalog = g2m::decode::dgm1::Catalog::open(pkt_zips);
    if (!catalog) {
        std::cerr << "g2m_tiler import-dem: " << catalog.error().message << "\n";
        return 1;
    }
    const auto t_catalog = std::chrono::steady_clock::now();

    g2m::TileStoreConfig config;
    config.dir = *store_dir;
    auto store = g2m::TileStore::open(config);
    if (!store) {
        std::cerr << "g2m_tiler import-dem: " << store.error().message << "\n";
        return 1;
    }

    g2m::import::StoreSink sink(**store, *scope);
    g2m::import::DemImportOptions options;
    if (const auto jobs = args.get("jobs")) {
        try {
            options.jobs = static_cast<unsigned>(std::stoul(*jobs));
        } catch (const std::exception&) {
            std::cerr << "g2m_tiler import-dem: --jobs must be an integer\n";
            return 1;
        }
    }
    if (const auto km_cache = args.get("km-cache")) {
        try {
            options.km_cache_tiles = static_cast<std::size_t>(std::stoul(*km_cache));
        } catch (const std::exception&) {
            std::cerr << "g2m_tiler import-dem: --km-cache must be an integer\n";
            return 1;
        }
    }

    const auto report = g2m::import::import_dem(*resolved, *catalog, sink, *scope, options);
    if (!report) {
        std::cerr << "g2m_tiler import-dem: " << report.error().message << "\n";
        return 1;
    }
    const auto t1 = std::chrono::steady_clock::now();
    const auto stats = catalog->stats();
    std::cerr << "g2m_tiler import-dem: catalog outer=" << stats.outer << " inner=" << stats.inner
              << " copies=" << stats.copies << " unique_keys=" << stats.unique_keys
              << " duplicated_keys=" << stats.duplicated_keys << " (open "
              << std::chrono::duration<double>(t_catalog - t0).count() << "s)\n";
    std::cerr << "g2m_tiler import-dem: tiles_written=" << report->tiles_written
              << " tiles_partial=" << report->tiles_partial << " tiles_skipped_nocov=" << report->tiles_skipped_nocov
              << " tiles_skipped_allnodata=" << report->tiles_skipped_allnodata << " dup_keys=" << report->dup_keys
              << " dup_copies=" << report->dup_copies << " dup_identical=" << report->dup_identical
              << " dup_differing=" << report->dup_differing << " km_decoded=" << report->km_decoded
              << " km_cache_hits=" << report->km_cache_hits << " wall=" << (report->wall_ms / 1000.0) << "s total="
              << std::chrono::duration<double>(t1 - t0).count() << "s peak RSS "
              << (g2m_tiler::peak_rss_bytes() / (1024 * 1024)) << " MiB\n";
    for (const auto& d : report->differing) std::cerr << "g2m_tiler import-dem: differing copy: " << d << "\n";
    return 0;
}

int cmd_import_osm(const Args& args) {
    const auto pbf = args.get("pbf");
    const auto resolved_path = args.get("resolved");
    const auto store_dir = args.get("store");
    const auto scope = args.get("scope");
    if (!pbf || !resolved_path || !store_dir || !scope) {
        std::cerr << "g2m_tiler import-osm: --pbf, --resolved, --store and --scope are required\n";
        return 1;
    }

    const auto resolved_text = read_file(*resolved_path);
    if (!resolved_text) {
        std::cerr << "g2m_tiler import-osm: cannot read " << *resolved_path << "\n";
        return 1;
    }
    const auto resolved = g2m::parse_resolved_region(*resolved_text);
    if (!resolved) {
        std::cerr << "g2m_tiler import-osm: " << resolved.error().message << "\n";
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    g2m::import::OsmExtractReport extract_report;
    const auto region_set = g2m::import::extract_region_set(*pbf, *resolved, &extract_report);
    if (!region_set) {
        std::cerr << "g2m_tiler import-osm: " << region_set.error().message << "\n";
        return 1;
    }
    const auto t_extract = std::chrono::steady_clock::now();

    g2m::TileStoreConfig config;
    config.dir = *store_dir;
    auto store = g2m::TileStore::open(config);
    if (!store) {
        std::cerr << "g2m_tiler import-osm: " << store.error().message << "\n";
        return 1;
    }

    g2m::import::StoreSink sink(**store, *scope);
    g2m::import::OsmImportOptions options;
    if (const auto jobs = args.get("jobs")) {
        try {
            options.jobs = static_cast<unsigned>(std::stoul(*jobs));
        } catch (const std::exception&) {
            std::cerr << "g2m_tiler import-osm: --jobs must be an integer\n";
            return 1;
        }
    }

    const auto report = g2m::import::import_osm(*resolved, *region_set, sink, *scope, options);
    if (!report) {
        std::cerr << "g2m_tiler import-osm: " << report.error().message << "\n";
        return 1;
    }
    const auto t1 = std::chrono::steady_clock::now();
    const auto store_stats = (*store)->stats();
    std::cerr << "g2m_tiler import-osm: nodes_total=" << extract_report.nodes_total
              << " nodes_in=" << extract_report.nodes_in << " ways_in=" << extract_report.ways_in
              << " ways_incomplete=" << extract_report.ways_incomplete
              << " relations_in=" << extract_report.relations_in << " index_bytes=" << extract_report.index_bytes
              << " (n1=" << extract_report.n1_ms << "ms w=" << extract_report.w_ms << "ms r=" << extract_report.r_ms
              << "ms n2=" << extract_report.n2_ms << "ms, extract "
              << std::chrono::duration<double>(t_extract - t0).count() << "s)\n";
    std::cerr << "g2m_tiler import-osm: tiles_written=" << report->tiles_written
              << " tiles_empty=" << report->tiles_empty << " wall=" << (report->wall_ms / 1000.0)
              << "s total=" << std::chrono::duration<double>(t1 - t0).count() << "s peak RSS "
              << (g2m_tiler::peak_rss_bytes() / (1024 * 1024)) << " MiB\n";
    if (store_stats) {
        std::cerr << "g2m_tiler import-osm: store body_bytes=" << store_stats->body_bytes
                  << " bodies=" << store_stats->bodies << "\n";
    }
    return 0;
}

int cmd_import_landcover(const Args& args) {
    const auto resolved_path = args.get("resolved");
    const auto store_dir = args.get("store");
    const auto scope = args.get("scope");
    if (!resolved_path || !store_dir || !scope) {
        std::cerr << "g2m_tiler import-landcover: --resolved, --store and --scope are required\n";
        return 1;
    }

    std::vector<std::filesystem::path> tifs;
    for (const auto& [k, v] : args.flags) {
        if (k == "tif") tifs.emplace_back(v);
    }
    if (tifs.empty()) {
        const auto data_dir = args.get("data");
        if (!data_dir) {
            std::cerr << "g2m_tiler import-landcover: need --tif PATH (repeatable) or --data DIR (containing "
                          "*.tif)\n";
            return 1;
        }
        const auto found = find_tifs(*data_dir);
        if (!found) {
            std::cerr << "g2m_tiler import-landcover: cannot list " << *data_dir << "\n";
            return 1;
        }
        tifs = *found;
        if (tifs.empty()) {
            std::cerr << "g2m_tiler import-landcover: no *.tif found under " << *data_dir << "\n";
            return 1;
        }
    } else {
        std::sort(tifs.begin(), tifs.end());
    }

    const auto resolved_text = read_file(*resolved_path);
    if (!resolved_text) {
        std::cerr << "g2m_tiler import-landcover: cannot read " << *resolved_path << "\n";
        return 1;
    }
    const auto resolved = g2m::parse_resolved_region(*resolved_text);
    if (!resolved) {
        std::cerr << "g2m_tiler import-landcover: " << resolved.error().message << "\n";
        return 1;
    }

    g2m::TileStoreConfig config;
    config.dir = *store_dir;
    auto store = g2m::TileStore::open(config);
    if (!store) {
        std::cerr << "g2m_tiler import-landcover: " << store.error().message << "\n";
        return 1;
    }

    g2m::import::StoreSink sink(**store, *scope);
    g2m::import::LandcoverImportOptions options;
    if (const auto jobs = args.get("jobs")) {
        try {
            options.jobs = static_cast<unsigned>(std::stoul(*jobs));
        } catch (const std::exception&) {
            std::cerr << "g2m_tiler import-landcover: --jobs must be an integer\n";
            return 1;
        }
    }

    const auto t0 = std::chrono::steady_clock::now();
    const auto report = g2m::import::import_landcover(*resolved, tifs, sink, *scope, options);
    if (!report) {
        std::cerr << "g2m_tiler import-landcover: " << report.error().message << "\n";
        return 1;
    }
    const auto t1 = std::chrono::steady_clock::now();
    std::cerr << "g2m_tiler import-landcover: tiles_written=" << report->tiles_written
              << " tiles_skipped_allzero=" << report->tiles_skipped_allzero << " wall=" << (report->wall_ms / 1000.0)
              << "s total=" << std::chrono::duration<double>(t1 - t0).count() << "s peak RSS "
              << (g2m_tiler::peak_rss_bytes() / (1024 * 1024)) << " MiB\n";
    return 0;
}

// import PATH#NAME --pbf PATH --store DIR --scope SCOPE
//        (--pkt PATH [--pkt ...] | --dem-data DIR)
//        (--tif PATH [--tif ...] | --wc-data DIR)
//        [--jobs N] [--km-cache N]
//
// The all-in-one subcommand (PLAN.md G1c I5): resolve -> import-dem ->
// import-osm -> import-landcover against one already-open TileStore, in that
// order (osm before landcover mirrors the CLI's own subcommand order above;
// nothing about the three importers' own tile layers overlaps, so their
// relative order does not affect the resulting store's contents - only this
// report's own timing breakdown). Writes <store>/regions/<name>.resolved.json
// (the same bytes cmd_resolve's own --out would produce) and
// <store>/import_report_<scope>.json (this command's own report - hand-rolled
// JSON, see json_escape's comment for why no nlohmann::json dependency is
// pulled into this app). Stops at the first importer that fails, same as
// running the three subcommands by hand in sequence would.
int cmd_import(const Args& args) {
    if (args.positional.empty()) {
        std::cerr << "g2m_tiler import: missing PATH#NAME selector\n";
        return 1;
    }
    const auto selector = g2m::split_selector(args.positional.front());
    if (!selector) {
        std::cerr << "g2m_tiler import: " << selector.error().message << "\n";
        return 1;
    }
    const auto pbf = args.get("pbf");
    const auto store_dir = args.get("store");
    const auto scope = args.get("scope");
    if (!pbf || !store_dir || !scope) {
        std::cerr << "g2m_tiler import: --pbf, --store and --scope are required\n";
        return 1;
    }

    std::vector<std::filesystem::path> pkt_zips;
    for (const auto& [k, v] : args.flags) {
        if (k == "pkt") pkt_zips.emplace_back(v);
    }
    if (pkt_zips.empty()) {
        const auto dem_data_dir = args.get("dem-data");
        if (!dem_data_dir) {
            std::cerr << "g2m_tiler import: need --pkt PATH (repeatable) or --dem-data DIR (containing "
                          "PKT_*.zip)\n";
            return 1;
        }
        std::error_code ec;
        for (const auto& entry : std::filesystem::directory_iterator(*dem_data_dir, ec)) {
            if (!entry.is_regular_file()) continue;
            std::string lower = entry.path().filename().string();
            std::transform(lower.begin(), lower.end(), lower.begin(),
                            [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
            if (lower.size() > 4 && lower.compare(0, 3, "pkt") == 0 && lower.compare(lower.size() - 4, 4, ".zip") == 0) {
                pkt_zips.push_back(entry.path());
            }
        }
        if (ec) {
            std::cerr << "g2m_tiler import: cannot list " << *dem_data_dir << ": " << ec.message() << "\n";
            return 1;
        }
        if (pkt_zips.empty()) {
            std::cerr << "g2m_tiler import: no PKT_*.zip found under " << *dem_data_dir << "\n";
            return 1;
        }
    }
    std::sort(pkt_zips.begin(), pkt_zips.end());

    std::vector<std::filesystem::path> tifs;
    for (const auto& [k, v] : args.flags) {
        if (k == "tif") tifs.emplace_back(v);
    }
    if (tifs.empty()) {
        const auto wc_data_dir = args.get("wc-data");
        if (!wc_data_dir) {
            std::cerr << "g2m_tiler import: need --tif PATH (repeatable) or --wc-data DIR (containing *.tif)\n";
            return 1;
        }
        const auto found = find_tifs(*wc_data_dir);
        if (!found) {
            std::cerr << "g2m_tiler import: cannot list " << *wc_data_dir << "\n";
            return 1;
        }
        tifs = *found;
        if (tifs.empty()) {
            std::cerr << "g2m_tiler import: no *.tif found under " << *wc_data_dir << "\n";
            return 1;
        }
    } else {
        std::sort(tifs.begin(), tifs.end());
    }

    const auto& [regions_path, region_name] = *selector;
    const auto regions_text = read_file(regions_path);
    if (!regions_text) {
        std::cerr << "g2m_tiler import: cannot read " << regions_path << "\n";
        return 1;
    }
    const auto specs = g2m::parse_regions(*regions_text);
    if (!specs) {
        std::cerr << "g2m_tiler import: " << specs.error().message << "\n";
        return 1;
    }
    const g2m::RegionSpec* spec = nullptr;
    for (const auto& s : *specs) {
        if (s.name == region_name) {
            spec = &s;
            break;
        }
    }
    if (spec == nullptr) {
        std::cerr << "g2m_tiler import: region '" << region_name << "' not found in " << regions_path << "\n";
        return 1;
    }

    const auto t_start = std::chrono::steady_clock::now();

    const auto t_resolve0 = std::chrono::steady_clock::now();
    const auto resolved = g2m::import::resolve_region(*spec, *pbf);
    const auto t_resolve1 = std::chrono::steady_clock::now();
    if (!resolved) {
        std::cerr << "g2m_tiler import: resolve: " << resolved.error().message << "\n";
        return 1;
    }

    g2m::TileStoreConfig config;
    config.dir = *store_dir;
    auto store = g2m::TileStore::open(config);
    if (!store) {
        std::cerr << "g2m_tiler import: " << store.error().message << "\n";
        return 1;
    }

    std::error_code ec;
    const std::filesystem::path regions_dir = std::filesystem::path(*store_dir) / "regions";
    std::filesystem::create_directories(regions_dir, ec);
    if (ec) {
        std::cerr << "g2m_tiler import: cannot create " << regions_dir.string() << ": " << ec.message() << "\n";
        return 1;
    }
    const std::string resolved_json = g2m::to_json(*resolved);
    const std::filesystem::path resolved_out = regions_dir / (region_name + ".resolved.json");
    if (!write_file(resolved_out.string(), resolved_json)) {
        std::cerr << "g2m_tiler import: cannot write " << resolved_out.string() << "\n";
        return 1;
    }

    g2m::import::StoreSink sink(**store, *scope);

    g2m::import::DemImportOptions dem_options;
    if (const auto jobs = args.get("jobs")) {
        try {
            dem_options.jobs = static_cast<unsigned>(std::stoul(*jobs));
        } catch (const std::exception&) {
            std::cerr << "g2m_tiler import: --jobs must be an integer\n";
            return 1;
        }
    }
    if (const auto km_cache = args.get("km-cache")) {
        try {
            dem_options.km_cache_tiles = static_cast<std::size_t>(std::stoul(*km_cache));
        } catch (const std::exception&) {
            std::cerr << "g2m_tiler import: --km-cache must be an integer\n";
            return 1;
        }
    }

    const auto t_dem0 = std::chrono::steady_clock::now();
    auto catalog = g2m::decode::dgm1::Catalog::open(pkt_zips);
    if (!catalog) {
        std::cerr << "g2m_tiler import: dem: " << catalog.error().message << "\n";
        return 1;
    }
    const auto dem_report = g2m::import::import_dem(*resolved, *catalog, sink, *scope, dem_options);
    const auto t_dem1 = std::chrono::steady_clock::now();
    if (!dem_report) {
        std::cerr << "g2m_tiler import: dem: " << dem_report.error().message << "\n";
        return 1;
    }

    const auto t_osm0 = std::chrono::steady_clock::now();
    g2m::import::OsmExtractReport extract_report;
    const auto region_set = g2m::import::extract_region_set(*pbf, *resolved, &extract_report);
    if (!region_set) {
        std::cerr << "g2m_tiler import: osm: " << region_set.error().message << "\n";
        return 1;
    }
    g2m::import::OsmImportOptions osm_options;
    if (const auto jobs = args.get("jobs")) osm_options.jobs = static_cast<unsigned>(std::stoul(*jobs));
    const auto osm_report = g2m::import::import_osm(*resolved, *region_set, sink, *scope, osm_options);
    const auto t_osm1 = std::chrono::steady_clock::now();
    if (!osm_report) {
        std::cerr << "g2m_tiler import: osm: " << osm_report.error().message << "\n";
        return 1;
    }

    g2m::import::LandcoverImportOptions lc_options;
    if (const auto jobs = args.get("jobs")) lc_options.jobs = static_cast<unsigned>(std::stoul(*jobs));
    const auto t_lc0 = std::chrono::steady_clock::now();
    const auto lc_report = g2m::import::import_landcover(*resolved, tifs, sink, *scope, lc_options);
    const auto t_lc1 = std::chrono::steady_clock::now();
    if (!lc_report) {
        std::cerr << "g2m_tiler import: landcover: " << lc_report.error().message << "\n";
        return 1;
    }

    const auto t_end = std::chrono::steady_clock::now();
    const auto seconds = [](auto a, auto b) { return std::chrono::duration<double>(b - a).count(); };
    const std::uint64_t peak_rss = g2m_tiler::peak_rss_bytes();
    const auto store_stats = (*store)->stats();

    std::ostringstream report;
    report << "{\"format\":\"g2m.import_report/1\","
           << "\"scope\":\"" << json_escape(*scope) << "\","
           << "\"region\":\"" << json_escape(region_name) << "\","
           << "\"resolve\":{\"tile_count\":" << resolved->tile_count
           << ",\"interior_runs\":" << resolved->interior.size() << ",\"wall_s\":" << seconds(t_resolve0, t_resolve1)
           << "},"
           << "\"dem\":{\"tiles_written\":" << dem_report->tiles_written
           << ",\"tiles_partial\":" << dem_report->tiles_partial
           << ",\"tiles_skipped_nocov\":" << dem_report->tiles_skipped_nocov
           << ",\"tiles_skipped_allnodata\":" << dem_report->tiles_skipped_allnodata
           << ",\"wall_s\":" << seconds(t_dem0, t_dem1) << "},"
           << "\"osm\":{\"tiles_written\":" << osm_report->tiles_written
           << ",\"tiles_empty\":" << osm_report->tiles_empty << ",\"wall_s\":" << seconds(t_osm0, t_osm1) << "},"
           << "\"landcover\":{\"tiles_written\":" << lc_report->tiles_written
           << ",\"tiles_skipped_allzero\":" << lc_report->tiles_skipped_allzero
           << ",\"wall_s\":" << seconds(t_lc0, t_lc1) << "},"
           << "\"peak_rss_mib\":" << (peak_rss / (1024 * 1024)) << ",\"wall_s_total\":" << seconds(t_start, t_end);
    if (store_stats) {
        report << ",\"store\":{\"tiles\":" << store_stats->tiles << ",\"bodies\":" << store_stats->bodies
               << ",\"body_bytes\":" << store_stats->body_bytes << "}";
    }
    report << "}";

    const std::filesystem::path report_out =
        std::filesystem::path(*store_dir) / ("import_report_" + *scope + ".json");
    if (!write_file(report_out.string(), report.str())) {
        std::cerr << "g2m_tiler import: cannot write " << report_out.string() << "\n";
        return 1;
    }

    std::cerr << "g2m_tiler import: resolve " << resolved->tile_count << " tiles, dem tiles_written="
              << dem_report->tiles_written << ", osm tiles_written=" << osm_report->tiles_written
              << ", landcover tiles_written=" << lc_report->tiles_written << ", total "
              << seconds(t_start, t_end) << "s, peak RSS " << (peak_rss / (1024 * 1024)) << " MiB\n";
    std::cerr << "g2m_tiler import: wrote " << resolved_out.string() << " and " << report_out.string() << "\n";
    return 0;
}

int cmd_stats(const Args& args) {
    const auto store_dir = args.get("store");
    if (!store_dir) {
        std::cerr << "g2m_tiler stats: missing --store\n";
        return 1;
    }
    g2m::TileStoreConfig config;
    config.dir = *store_dir;
    auto store = g2m::TileStore::open(config);
    if (!store) {
        std::cerr << "g2m_tiler stats: " << store.error().message << "\n";
        return 1;
    }
    const auto stats = (*store)->stats();
    if (!stats) {
        std::cerr << "g2m_tiler stats: " << stats.error().message << "\n";
        return 1;
    }
    std::cout << "tiles=" << stats->tiles << " bodies=" << stats->bodies << " blob_files=" << stats->blob_files
              << " body_bytes=" << stats->body_bytes << " pinned_tiles=" << stats->pinned_tiles << "\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string command = argv[1];
    const Args args = parse_args(argc, argv, 2);

    if (command == "find-admin") return cmd_find_admin(args);
    if (command == "resolve") return cmd_resolve(args);
    if (command == "import-dem") return cmd_import_dem(args);
    if (command == "import-osm") return cmd_import_osm(args);
    if (command == "import-landcover") return cmd_import_landcover(args);
    if (command == "import") return cmd_import(args);
    if (command == "stats") return cmd_stats(args);
    if (command == "--help" || command == "-h" || command == "help") return usage(), 0;
    std::cerr << "g2m_tiler: unknown command '" << command << "'\n";
    return usage();
}
