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
//          [--jobs N] [--km-cache N] [--pin]
//          (I5: resolve -> import-dem -> import-osm -> import-landcover in
//          one call, writing <store>/regions/<name>.resolved.json and
//          <store>/import_report_<scope>.json. I7: --pin forwards to every
//          StoreSink::put this run makes, so nothing it writes is ever
//          evicted - see sink.h's own StoreSink pin parameter, G1c I6 item E)
//   stats --store DIR [--scope SCOPE]
//         (also runs TileStore::check_consistency and prints its result)
//   index-hash --store DIR --scope SCOPE
//          (G1c I7: per-layer FNV-1a64 over the store's (key, body_hash)
//          index in TileStore::index's own (layer, key) order - a
//          --jobs-independent, filesystem-enumeration-independent
//          fingerprint for comparing two stores built from the same import)
//   diff-scopes --a DIR:SCOPE --b DIR:SCOPE
//          (PLAN.md G2.0 Task E: a per-tile complement to index-hash - two
//          index-hash values differing says ONLY that something changed,
//          not what. Splits each DIR:SCOPE argument on the LAST ':' (a
//          Windows drive letter's own colon sits earlier; a scope name never
//          contains ':'), opens both stores' TileStore::index(scope) (each
//          already sorted by (layer, key)) and merge-diffs them, printing
//          "added"/"removed"/"changed" per (layer, key) plus a per-layer and
//          overall added/removed/changed/unchanged summary. Read-only: never
//          calls a TileStore mutator)
//   check-dem-nodata --store DIR --scope SCOPE --resolved PATH [--interior-only]
//          (PLAN.md G2.0 Task B: decodes every requested tile's real
//          g2m.src.elev.dgm1 body and counts actual NoData samples - the
//          check `stats`/`index-hash` never did; see cmd_check_dem_nodata's
//          own comment for why the G1c "0 NoData in interior tiles" claim
//          needed this. --interior-only restricts to the region's interior
//          tiles and fails (exit 1) if any of them carry NoData)
//   bake --store DIR --scope SCOPE --derived DIR --layers L[,L...]
//        [--levels LO-HI] [--jobs N] [--pin]
//        (PLAN.md G2.2: prebakes one or more built-in terrain layers -
//        currently g2m.terrain.height - of `scope`'s local release, level-
//        major, into a separate derived store at --derived. `--store` is
//        opened read-only when the store supports it (see g2m/import/
//        bake.h's own comment); `--levels` defaults to 0-6, a bare "N"
//        means "N-N". Every tile goes through the same Server::tile()
//        resolver an on-demand server would use (PLAN.md decision 10), so
//        the result is deterministic across --jobs.)
//
// Exit codes: 0 success, 1 usage/runtime error, 2 "not implemented yet".
#include "cli_args.h"
#include "rss.h"

#include "g2m/decode/dgm1.h"
#include "g2m/decode/dgm1_bw.h"
#include "g2m/import/bake.h"
#include "g2m/import/dem.h"
#include "g2m/import/landcover.h"
#include "g2m/import/osm.h"
#include "g2m/import/region_resolve.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/region.h"
#include "g2m/layer/src_elev.h"
#include "g2m/core/hash.h"
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

using g2m_tiler::Args;
using g2m_tiler::split_dir_scope;

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

// Every value of a repeatable flag, in argv order.
std::vector<std::string> flag_values(const Args& args, const std::string& key) {
    std::vector<std::string> out;
    for (const auto& [k, v] : args.flags) {
        if (k == key) out.push_back(v);
    }
    return out;
}

// Lists the regular files of `dir` whose lower-cased name starts with
// `prefix` and ends with ".zip", sorted by full path (reproducible).
bool list_zips(const std::string& dir, const std::string& prefix, std::vector<std::filesystem::path>& out) {
    std::error_code ec;
    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (!entry.is_regular_file()) continue;
        std::string lower = entry.path().filename().string();
        std::transform(lower.begin(), lower.end(), lower.begin(),
                        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
        if (lower.size() > 4 && lower.compare(0, prefix.size(), prefix) == 0 &&
            lower.compare(lower.size() - 4, 4, ".zip") == 0) {
            out.push_back(entry.path());
        }
    }
    std::sort(out.begin(), out.end());
    return !ec;
}

// The elevation inputs of import-dem / import: Hessen PKT_*.zip (--pkt PATH
// repeatable, or a directory flag) and Baden-Wuerttemberg dgm1_*_bw.zip
// (--bw PATH repeatable, or --bw-data DIR). At least one set is required.
bool collect_dem_inputs(const Args& args, const char* who, const char* hessen_dir_flag,
                         std::vector<std::filesystem::path>& pkt_zips, std::vector<std::filesystem::path>& bw_zips) {
    for (const std::string& v : flag_values(args, "pkt")) pkt_zips.emplace_back(v);
    if (pkt_zips.empty()) {
        if (const auto dir = args.get(hessen_dir_flag)) {
            if (!list_zips(*dir, "pkt", pkt_zips)) {
                std::cerr << "g2m_tiler " << who << ": cannot list " << *dir << "\n";
                return false;
            }
        }
    }
    std::sort(pkt_zips.begin(), pkt_zips.end());
    for (const std::string& v : flag_values(args, "bw")) bw_zips.emplace_back(v);
    if (bw_zips.empty()) {
        if (const auto dir = args.get("bw-data")) {
            if (!list_zips(*dir, "dgm1_", bw_zips)) {
                std::cerr << "g2m_tiler " << who << ": cannot list " << *dir << "\n";
                return false;
            }
        }
    }
    std::sort(bw_zips.begin(), bw_zips.end());
    if (pkt_zips.empty() && bw_zips.empty()) {
        std::cerr << "g2m_tiler " << who << ": need Hessen DEM (--pkt PATH repeatable or --" << hessen_dir_flag
                  << " DIR containing PKT_*.zip) and/or Baden-Wuerttemberg DEM (--bw PATH repeatable or --bw-data "
                     "DIR containing dgm1_*.zip)\n";
        return false;
    }
    return true;
}

std::string dem_report_line(const g2m::import::DemImportReport& r) {
    std::ostringstream o;
    o << "km_bw_only=" << r.km_bw_only << " km_merged=" << r.km_merged << " border_samples_both="
      << r.border_samples_both << " border_samples_differing=" << r.border_samples_differing
      << " border_max_abs_diff_m=" << (static_cast<double>(r.border_max_abs_diff_units) / g2m::kHeightUnitsPerMetre);
    return o.str();
}

int usage() {
    std::cerr << "usage: g2m_tiler <command> [options]\n"
                 "commands:\n"
                 "  find-admin --pbf PATH [--name-contains STR] [--admin-level N]\n"
                 "  resolve PATH#NAME --pbf PATH [--pbf PATH ...] --out PATH\n"
                 "  import-dem --resolved PATH --store DIR --scope SCOPE\n"
                 "             (--pkt PATH [--pkt PATH ...] | --data DIR) and/or (--bw PATH [--bw ...] | --bw-data DIR)\n"
                 "             [--jobs N] [--km-cache N] [--pin]\n"
                 "  import-osm --pbf PATH [--pbf PATH ...] --resolved PATH --store DIR --scope SCOPE [--jobs N] [--pin]\n"
                 "  import-landcover --resolved PATH --store DIR --scope SCOPE\n"
                 "                   (--tif PATH [--tif ...] | --data DIR) [--jobs N] [--pin]\n"
                 "  import PATH#NAME --pbf PATH [--pbf PATH ...] --store DIR --scope SCOPE\n"
                 "         (--pkt PATH [--pkt ...] | --dem-data DIR) and/or (--bw PATH [--bw ...] | --bw-data DIR)\n"
                 "         (--tif PATH [--tif ...] | --wc-data DIR)\n"
                 "         [--jobs N] [--km-cache N] [--pin]\n"
                 "  stats --store DIR [--scope SCOPE]\n"
                 "  index-hash --store DIR --scope SCOPE\n"
                 "  diff-scopes --a DIR:SCOPE --b DIR:SCOPE\n"
                 "  check-dem-nodata --store DIR --scope SCOPE --resolved PATH [--interior-only]\n"
                 "  bake --store DIR --scope SCOPE --derived DIR --layers L[,L...]\n"
                 "       [--levels LO-HI] [--jobs N] [--pin]\n";
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
    const auto resolved = g2m::import::resolve_region(*spec, flag_values(args, "pbf"));
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

    std::vector<std::filesystem::path> pkt_zips, bw_zips;
    if (!collect_dem_inputs(args, "import-dem", "data", pkt_zips, bw_zips)) return 1;

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
    std::optional<g2m::decode::dgm1::Catalog> catalog;
    std::optional<g2m::decode::dgm1_bw::Catalog> bw_catalog;
    if (!pkt_zips.empty()) {
        auto c = g2m::decode::dgm1::Catalog::open(pkt_zips);
        if (!c) {
            std::cerr << "g2m_tiler import-dem: " << c.error().message << "\n";
            return 1;
        }
        catalog.emplace(std::move(*c));
    }
    if (!bw_zips.empty()) {
        auto c = g2m::decode::dgm1_bw::Catalog::open(bw_zips);
        if (!c) {
            std::cerr << "g2m_tiler import-dem: " << c.error().message << "\n";
            return 1;
        }
        bw_catalog.emplace(std::move(*c));
    }
    g2m::import::DemSources dem_sources;
    if (catalog) dem_sources.hessen = &*catalog;
    if (bw_catalog) dem_sources.bw = &*bw_catalog;
    const auto t_catalog = std::chrono::steady_clock::now();

    g2m::TileStoreConfig config;
    config.dir = *store_dir;
    auto store = g2m::TileStore::open(config);
    if (!store) {
        std::cerr << "g2m_tiler import-dem: " << store.error().message << "\n";
        return 1;
    }

    g2m::import::StoreSink sink(**store, *scope, args.flag("pin"));
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

    const auto report = g2m::import::import_dem(*resolved, dem_sources, sink, *scope, options);
    if (!report) {
        std::cerr << "g2m_tiler import-dem: " << report.error().message << "\n";
        return 1;
    }
    const auto t1 = std::chrono::steady_clock::now();
    if (catalog) {
        const auto stats = catalog->stats();
        std::cerr << "g2m_tiler import-dem: catalog outer=" << stats.outer << " inner=" << stats.inner
                  << " copies=" << stats.copies << " unique_keys=" << stats.unique_keys
                  << " duplicated_keys=" << stats.duplicated_keys << " (open "
                  << std::chrono::duration<double>(t_catalog - t0).count() << "s)\n";
    }
    if (bw_catalog) {
        const auto stats = bw_catalog->stats();
        std::cerr << "g2m_tiler import-dem: bw catalog zips=" << stats.zips << " copies=" << stats.copies
                  << " unique_keys=" << stats.unique_keys << " duplicated_keys=" << stats.duplicated_keys << " | "
                  << dem_report_line(*report) << "\n";
    }
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
    g2m::import::OsmMergeReport merge_report;
    const auto region_set =
        g2m::import::extract_region_set(flag_values(args, "pbf"), *resolved, &extract_report, &merge_report);
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

    g2m::import::StoreSink sink(**store, *scope, args.flag("pin"));
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
    if (flag_values(args, "pbf").size() > 1) {
        std::cerr << "g2m_tiler import-osm: merge nodes_both=" << merge_report.nodes_both
                  << " nodes_conflicting=" << merge_report.nodes_conflicting << " ways_both=" << merge_report.ways_both
                  << " ways_conflicting=" << merge_report.ways_conflicting
                  << " relations_both=" << merge_report.relations_both
                  << " relations_conflicting=" << merge_report.relations_conflicting << "\n";
    }
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

    g2m::import::StoreSink sink(**store, *scope, args.flag("pin"));
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

    std::vector<std::filesystem::path> pkt_zips, bw_zips;
    if (!collect_dem_inputs(args, "import", "dem-data", pkt_zips, bw_zips)) return 1;

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
    const auto resolved = g2m::import::resolve_region(*spec, flag_values(args, "pbf"));
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

    g2m::import::StoreSink sink(**store, *scope, args.flag("pin"));

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
    std::optional<g2m::decode::dgm1::Catalog> catalog;
    std::optional<g2m::decode::dgm1_bw::Catalog> bw_catalog;
    if (!pkt_zips.empty()) {
        auto c = g2m::decode::dgm1::Catalog::open(pkt_zips);
        if (!c) {
            std::cerr << "g2m_tiler import: dem: " << c.error().message << "\n";
            return 1;
        }
        catalog.emplace(std::move(*c));
    }
    if (!bw_zips.empty()) {
        auto c = g2m::decode::dgm1_bw::Catalog::open(bw_zips);
        if (!c) {
            std::cerr << "g2m_tiler import: dem: " << c.error().message << "\n";
            return 1;
        }
        bw_catalog.emplace(std::move(*c));
    }
    g2m::import::DemSources dem_sources;
    if (catalog) dem_sources.hessen = &*catalog;
    if (bw_catalog) dem_sources.bw = &*bw_catalog;
    const auto dem_report = g2m::import::import_dem(*resolved, dem_sources, sink, *scope, dem_options);
    const auto t_dem1 = std::chrono::steady_clock::now();
    if (!dem_report) {
        std::cerr << "g2m_tiler import: dem: " << dem_report.error().message << "\n";
        return 1;
    }

    const auto t_osm0 = std::chrono::steady_clock::now();
    g2m::import::OsmExtractReport extract_report;
    g2m::import::OsmMergeReport merge_report;
    const auto region_set =
        g2m::import::extract_region_set(flag_values(args, "pbf"), *resolved, &extract_report, &merge_report);
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
           << ",\"km_bw_only\":" << dem_report->km_bw_only << ",\"km_merged\":" << dem_report->km_merged
           << ",\"border_samples_both\":" << dem_report->border_samples_both
           << ",\"border_samples_differing\":" << dem_report->border_samples_differing
           << ",\"border_max_abs_diff_units\":" << dem_report->border_max_abs_diff_units
           << ",\"wall_s\":" << seconds(t_dem0, t_dem1) << "},"
           << "\"osm\":{\"tiles_written\":" << osm_report->tiles_written
           << ",\"tiles_empty\":" << osm_report->tiles_empty << ",\"pbf_files\":" << flag_values(args, "pbf").size()
           << ",\"merge_nodes_both\":" << merge_report.nodes_both
           << ",\"merge_ways_both\":" << merge_report.ways_both
           << ",\"merge_ways_conflicting\":" << merge_report.ways_conflicting
           << ",\"merge_relations_both\":" << merge_report.relations_both
           << ",\"merge_relations_conflicting\":" << merge_report.relations_conflicting
           << ",\"wall_s\":" << seconds(t_osm0, t_osm1) << "},"
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

    // G1c I7: check_consistency (reference counts, size total, every blob
    // file present with the right size - tile_store.h's own doc comment) is
    // cheap enough to run on every `stats` call, and this is the one place a
    // maintainer would look for it after an import.
    const auto consistency = (*store)->check_consistency();
    if (!consistency) {
        std::cout << "check_consistency FAILED: " << consistency.error().message << "\n";
        return 1;
    }
    std::cout << "check_consistency OK\n";
    return 0;
}

// index-hash --store DIR --scope SCOPE (G1c I7): per-layer FNV-1a64 over the
// store's (key, body_hash) index, for comparing two stores built from the
// same import (e.g. at different --jobs) without a byte-for-byte diff of the
// whole store (blob-file layout/SQLite page order are not part of the
// contract; the logical (layer, key) -> body_hash mapping is).
// TileStore::index() already returns its entries sorted by (layer, key)
// (tile_store.h's own doc comment), so hashing them in that order needs no
// extra sort here and is independent of --jobs and of filesystem/PKT/tif
// enumeration order.
int cmd_index_hash(const Args& args) {
    const auto store_dir = args.get("store");
    const auto scope = args.get("scope");
    if (!store_dir || !scope) {
        std::cerr << "g2m_tiler index-hash: --store and --scope are required\n";
        return 1;
    }
    g2m::TileStoreConfig config;
    config.dir = *store_dir;
    auto store = g2m::TileStore::open(config);
    if (!store) {
        std::cerr << "g2m_tiler index-hash: " << store.error().message << "\n";
        return 1;
    }
    const auto index = (*store)->index(*scope);
    if (!index) {
        std::cerr << "g2m_tiler index-hash: " << index.error().message << "\n";
        return 1;
    }

    g2m::Fnv1a64 all_hasher;
    std::uint64_t all_count = 0;
    std::string current_layer;
    bool have_layer = false;
    g2m::Fnv1a64 layer_hasher;
    std::uint64_t layer_count = 0;
    auto flush_layer = [&]() {
        std::cout << current_layer << " " << layer_count << " " << g2m::to_hex(layer_hasher.value()) << "\n";
    };
    for (const auto& entry : *index) {
        if (!have_layer || entry.layer != current_layer) {
            if (have_layer) flush_layer();
            current_layer = entry.layer;
            have_layer = true;
            layer_hasher = g2m::Fnv1a64{};
            layer_count = 0;
        }
        layer_hasher.u64(entry.key.packed());
        layer_hasher.bytes(entry.body_hash.data(), entry.body_hash.size());
        ++layer_count;
        all_hasher.u64(entry.key.packed());
        all_hasher.bytes(entry.body_hash.data(), entry.body_hash.size());
        ++all_count;
    }
    if (have_layer) flush_layer();
    std::cout << "all " << all_count << " " << g2m::to_hex(all_hasher.value()) << "\n";
    return 0;
}

// diff-scopes --a DIR:SCOPE --b DIR:SCOPE (PLAN.md G2.0 Task E): a per-tile
// complement to index-hash. Two index-hash values differing across an "old"
// and a "new" store/scope says only that something changed - never what.
// This opens both stores' TileStore::index(scope) (each already sorted by
// (layer, key), same contract cmd_index_hash relies on) and merge-diffs the
// two sorted sequences directly (no sort/hash-map needed): a (layer, key)
// present only on one side is "removed"/"added"; present on both with a
// different body_hash is "changed"; otherwise "unchanged" (counted, not
// printed - a re-import of an unchanged region should print nothing but
// summary lines). --a and --b may name the same directory (diffing two
// scopes of one store) or two different directories (e.g. the old home
// store vs. a freshly re-imported one, PLAN.md G2.0 Task G) - TileStore
// itself supports several objects opening the same directory concurrently.
// Never calls a TileStore mutator.
int cmd_diff_scopes(const Args& args) {
    const auto a_arg = args.get("a");
    const auto b_arg = args.get("b");
    if (!a_arg || !b_arg) {
        std::cerr << "g2m_tiler diff-scopes: --a and --b are required (DIR:SCOPE each)\n";
        return 1;
    }
    const auto a_split = split_dir_scope(*a_arg);
    if (!a_split) {
        std::cerr << "g2m_tiler diff-scopes: --a must be DIR:SCOPE, got '" << *a_arg << "'\n";
        return 1;
    }
    const auto b_split = split_dir_scope(*b_arg);
    if (!b_split) {
        std::cerr << "g2m_tiler diff-scopes: --b must be DIR:SCOPE, got '" << *b_arg << "'\n";
        return 1;
    }
    const auto& [a_dir, a_scope] = *a_split;
    const auto& [b_dir, b_scope] = *b_split;

    g2m::TileStoreConfig a_config;
    a_config.dir = a_dir;
    auto a_store = g2m::TileStore::open(a_config);
    if (!a_store) {
        std::cerr << "g2m_tiler diff-scopes: --a: " << a_store.error().message << "\n";
        return 1;
    }
    g2m::TileStoreConfig b_config;
    b_config.dir = b_dir;
    auto b_store = g2m::TileStore::open(b_config);
    if (!b_store) {
        std::cerr << "g2m_tiler diff-scopes: --b: " << b_store.error().message << "\n";
        return 1;
    }

    const auto a_index = (*a_store)->index(a_scope);
    if (!a_index) {
        std::cerr << "g2m_tiler diff-scopes: --a: " << a_index.error().message << "\n";
        return 1;
    }
    const auto b_index = (*b_store)->index(b_scope);
    if (!b_index) {
        std::cerr << "g2m_tiler diff-scopes: --b: " << b_index.error().message << "\n";
        return 1;
    }
    const auto& av = *a_index;
    const auto& bv = *b_index;

    struct Counts {
        std::uint64_t added = 0, removed = 0, changed = 0, unchanged = 0;
    };
    Counts totals;
    Counts layer_counts;
    std::string current_layer;
    bool have_layer = false;
    auto flush_layer = [&]() {
        std::cout << current_layer << " added=" << layer_counts.added << " removed=" << layer_counts.removed
                  << " changed=" << layer_counts.changed << " unchanged=" << layer_counts.unchanged << "\n";
    };
    auto note_layer = [&](const std::string& layer) {
        if (!have_layer || layer != current_layer) {
            if (have_layer) flush_layer();
            current_layer = layer;
            have_layer = true;
            layer_counts = Counts{};
        }
    };

    std::size_t ia = 0, ib = 0;
    while (ia < av.size() || ib < bv.size()) {
        const bool have_a = ia < av.size();
        const bool have_b = ib < bv.size();
        int cmp;
        if (have_a && have_b) {
            if (av[ia].layer != bv[ib].layer) {
                cmp = av[ia].layer < bv[ib].layer ? -1 : 1;
            } else if (av[ia].key != bv[ib].key) {
                cmp = av[ia].key < bv[ib].key ? -1 : 1;
            } else {
                cmp = 0;
            }
        } else if (have_a) {
            cmp = -1;
        } else {
            cmp = 1;
        }

        if (cmp < 0) {
            note_layer(av[ia].layer);
            std::cout << "removed " << av[ia].layer << " " << g2m::to_string(av[ia].key) << "\n";
            ++layer_counts.removed;
            ++totals.removed;
            ++ia;
        } else if (cmp > 0) {
            note_layer(bv[ib].layer);
            std::cout << "added " << bv[ib].layer << " " << g2m::to_string(bv[ib].key) << "\n";
            ++layer_counts.added;
            ++totals.added;
            ++ib;
        } else {
            note_layer(av[ia].layer);
            if (av[ia].body_hash != bv[ib].body_hash) {
                std::cout << "changed " << av[ia].layer << " " << g2m::to_string(av[ia].key) << "\n";
                ++layer_counts.changed;
                ++totals.changed;
            } else {
                ++layer_counts.unchanged;
                ++totals.unchanged;
            }
            ++ia;
            ++ib;
        }
    }
    if (have_layer) flush_layer();

    std::cout << "total added=" << totals.added << " removed=" << totals.removed << " changed=" << totals.changed
              << " unchanged=" << totals.unchanged << "\n";
    return 0;
}

// check-dem-nodata --store DIR --scope SCOPE --resolved PATH [--interior-only]
// (PLAN.md G2.0 Task B). The G1c I7 report's "0 NoData samples in the 533
// interior tiles" claim was never actually measured: cmd_stats() only calls
// TileStore::check_consistency() (reference counts / blob presence+size, no
// body decode) and cmd_index_hash() only fingerprints (layer, key,
// body_hash) tuples (detects a byte difference across runs, says nothing
// about what the bytes MEAN) - neither one ever decodes a src_elev.dgm1
// body and looks at its samples. The claim rested on an unstated assumption
// ("if DGM1 coverage exists for every km square an interior tile touches,
// the tile has no NoData") that silently depended on the importer visiting
// every touching km square correctly - exactly the assumption the
// dgm1_km_span bug (g2m/import/dem.h, dem.cpp) violated: a tile whose
// middle km square was skipped still reads back as "covered" by any check
// that does not decode the body, while its samples carry a real NoData
// stripe. This command is that missing check: it fetches each requested
// key's g2m.src.elev.dgm1 tile, decodes the real container body
// (decode_body + decode_src_elev) and counts actual kHeightNoData samples.
//
// --interior-only restricts the keys to ResolvedRegion::interior
// (region.h's interior_tile_keys(), PLAN.md G2.0) - the subset the G1c
// claim was actually about, where full coverage is expected - and makes
// the command exit 1 if any interior tile carries NoData (a regression
// guard for future imports). Without --interior-only every footprint tile
// is checked and reported (including the halo, where partial/no coverage
// is accepted, PLAN.md's own open item), but the exit code stays 0.
int cmd_check_dem_nodata(const Args& args) {
    const auto store_dir = args.get("store");
    const auto scope = args.get("scope");
    const auto resolved_path = args.get("resolved");
    if (!store_dir || !scope || !resolved_path) {
        std::cerr << "g2m_tiler check-dem-nodata: --store, --scope and --resolved are required\n";
        return 1;
    }
    const bool interior_only = args.flag("interior-only");

    const auto resolved_text = read_file(*resolved_path);
    if (!resolved_text) {
        std::cerr << "g2m_tiler check-dem-nodata: cannot read " << *resolved_path << "\n";
        return 1;
    }
    const auto resolved = g2m::parse_resolved_region(*resolved_text);
    if (!resolved) {
        std::cerr << "g2m_tiler check-dem-nodata: " << resolved.error().message << "\n";
        return 1;
    }

    const std::vector<g2m::TileKey> keys =
        interior_only ? g2m::interior_tile_keys(*resolved) : g2m::tile_keys(*resolved);

    g2m::TileStoreConfig config;
    config.dir = *store_dir;
    auto store = g2m::TileStore::open(config);
    if (!store) {
        std::cerr << "g2m_tiler check-dem-nodata: " << store.error().message << "\n";
        return 1;
    }

    std::uint64_t tiles_checked = 0;
    std::uint64_t tiles_missing = 0;
    std::uint64_t tiles_with_nodata = 0;
    std::uint64_t total_nodata_samples = 0;
    std::uint64_t total_samples = 0;

    for (const g2m::TileKey& key : keys) {
        const g2m::TileAddress addr{*scope, std::string(g2m::kSrcElevDgm1Layer), key};
        const auto got = (*store)->get(addr);
        if (!got) {
            std::cerr << "g2m_tiler check-dem-nodata: " << got.error().message << "\n";
            return 1;
        }
        if (!got->has_value()) {
            ++tiles_missing;
            continue;
        }
        ++tiles_checked;
        const auto body = g2m::decode_body((*got)->body);
        if (!body) {
            std::cerr << "g2m_tiler check-dem-nodata: decode_body(" << g2m::to_string(key)
                      << "): " << body.error().message << "\n";
            return 1;
        }
        const auto tile = g2m::decode_src_elev(*body);
        if (!tile) {
            std::cerr << "g2m_tiler check-dem-nodata: decode_src_elev(" << g2m::to_string(key)
                      << "): " << tile.error().message << "\n";
            return 1;
        }
        std::uint64_t nodata_here = 0;
        for (const std::int32_t h : tile->heights.data()) {
            ++total_samples;
            if (h == g2m::kHeightNoData) ++nodata_here;
        }
        if (nodata_here > 0) {
            ++tiles_with_nodata;
            total_nodata_samples += nodata_here;
            std::cout << "nodata " << g2m::to_string(key) << " " << nodata_here << "/" << tile->heights.size()
                      << "\n";
        }
    }

    std::cout << "scope=" << (interior_only ? "interior" : "footprint") << " requested=" << keys.size()
              << " checked=" << tiles_checked << " missing=" << tiles_missing
              << " tiles_with_nodata=" << tiles_with_nodata << " total_nodata_samples=" << total_nodata_samples
              << " total_samples=" << total_samples << "\n";

    if (interior_only && tiles_with_nodata > 0) {
        std::cout << "check-dem-nodata FAILED: " << tiles_with_nodata << " interior tile(s) contain NoData sample(s)\n";
        return 1;
    }
    std::cout << "check-dem-nodata OK\n";
    return 0;
}

// bake --store DIR --scope SCOPE --derived DIR --layers L[,L...]
//      [--levels LO-HI] [--jobs N] [--pin] (PLAN.md G2.2)
std::vector<std::string> split_comma(const std::string& s) {
    std::vector<std::string> out;
    std::size_t pos = 0;
    while (pos <= s.size()) {
        const std::size_t comma = s.find(',', pos);
        const std::size_t end = comma == std::string::npos ? s.size() : comma;
        if (end > pos) out.push_back(s.substr(pos, end - pos));
        if (comma == std::string::npos) break;
        pos = comma + 1;
    }
    return out;
}

// "LO-HI" or a bare "N" (== "N-N").
bool parse_level_range(const std::string& s, int& lo, int& hi) {
    const std::size_t dash = s.find('-', s[0] == '-' ? 1 : 0);
    try {
        if (dash == std::string::npos) {
            lo = hi = std::stoi(s);
        } else {
            lo = std::stoi(s.substr(0, dash));
            hi = std::stoi(s.substr(dash + 1));
        }
    } catch (...) {
        return false;
    }
    return lo <= hi;
}

int cmd_bake(const Args& args) {
    const auto store_dir = args.get("store");
    const auto scope = args.get("scope");
    const auto derived_dir = args.get("derived");
    const auto layers_arg = args.get("layers");
    if (!store_dir || !scope || !derived_dir || !layers_arg) {
        std::cerr << "g2m_tiler bake: --store, --scope, --derived and --layers are required\n";
        return 1;
    }
    const std::vector<std::string> layers = split_comma(*layers_arg);
    if (layers.empty()) {
        std::cerr << "g2m_tiler bake: --layers must name at least one layer\n";
        return 1;
    }

    g2m::import::BakeOptions options;
    options.layers = layers;
    if (const auto levels = args.get("levels")) {
        if (!parse_level_range(*levels, options.min_level, options.max_level)) {
            std::cerr << "g2m_tiler bake: --levels must be \"LO-HI\" or \"N\"\n";
            return 1;
        }
    }
    if (const auto jobs = args.get("jobs")) {
        try {
            options.jobs = std::stoi(*jobs);
        } catch (...) {
            std::cerr << "g2m_tiler bake: --jobs must be an integer\n";
            return 1;
        }
    }
    options.pin = args.flag("pin");

    // Opened read_only (PLAN.md G2.0's TileStoreConfig::read_only): a bake
    // source is a finished, --pin'd release scope (never a store still
    // being imported into), and bake() itself never calls a mutating
    // TileStore method on it either way (see bake.h's own comment).
    g2m::TileStoreConfig source_config;
    source_config.dir = *store_dir;
    source_config.read_only = true;
    auto sources = g2m::TileStore::open(source_config);
    if (!sources) {
        std::cerr << "g2m_tiler bake: --store: " << sources.error().message << "\n";
        return 1;
    }
    g2m::TileStoreConfig derived_config;
    derived_config.dir = *derived_dir;
    auto derived = g2m::TileStore::open(derived_config);
    if (!derived) {
        std::cerr << "g2m_tiler bake: --derived: " << derived.error().message << "\n";
        return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const auto report = g2m::import::bake(**sources, **derived, *scope, options);
    const auto t1 = std::chrono::steady_clock::now();
    if (!report) {
        std::cerr << "g2m_tiler bake: " << report.error().message << "\n";
        return 1;
    }
    const double wall_s = std::chrono::duration<double>(t1 - t0).count();
    const std::uint64_t peak_rss = g2m_tiler::peak_rss_bytes();

    std::ostringstream json;
    json << "{\"format\":\"g2m.bake_report/1\",\"scope\":\"" << json_escape(*scope) << "\",\"release_id\":\""
         << report->release_id_hex << "\",\"wall_s\":" << wall_s << ",\"peak_rss_mib\":" << (peak_rss / (1024 * 1024))
         << ",\"layers\":[";
    for (std::size_t li = 0; li < report->layers.size(); ++li) {
        const auto& l = report->layers[li];
        if (li > 0) json << ",";
        json << "{\"layer\":\"" << json_escape(l.layer) << "\",\"tiles_total\":" << l.tiles_total
             << ",\"container_bytes_total\":" << l.container_bytes_total << ",\"levels\":[";
        for (std::size_t vi = 0; vi < l.levels.size(); ++vi) {
            const auto& lv = l.levels[vi];
            if (vi > 0) json << ",";
            json << "{\"level\":" << lv.level << ",\"tiles\":" << lv.tiles
                 << ",\"container_bytes\":" << lv.container_bytes << "}";
        }
        json << "]}";
    }
    json << "]}";
    std::cout << json.str() << "\n";

    std::cerr << "g2m_tiler bake: scope=" << *scope << " release_id=" << report->release_id_hex
              << " layers=" << layers.size() << " levels=[" << options.min_level << "," << options.max_level
              << "] jobs=" << options.jobs << " wall=" << wall_s << "s peak RSS " << (peak_rss / (1024 * 1024))
              << " MiB\n";
    return 0;
}

} // namespace

int main(int argc, char** argv) {
    if (argc < 2) return usage();
    const std::string command = argv[1];
    const Args args = g2m_tiler::parse_args(argc, argv, 2, {"pin", "interior-only"});

    if (command == "find-admin") return cmd_find_admin(args);
    if (command == "resolve") return cmd_resolve(args);
    if (command == "import-dem") return cmd_import_dem(args);
    if (command == "import-osm") return cmd_import_osm(args);
    if (command == "import-landcover") return cmd_import_landcover(args);
    if (command == "import") return cmd_import(args);
    if (command == "stats") return cmd_stats(args);
    if (command == "index-hash") return cmd_index_hash(args);
    if (command == "diff-scopes") return cmd_diff_scopes(args);
    if (command == "check-dem-nodata") return cmd_check_dem_nodata(args);
    if (command == "bake") return cmd_bake(args);
    if (command == "--help" || command == "-h" || command == "help") return usage(), 0;
    std::cerr << "g2m_tiler: unknown command '" << command << "'\n";
    return usage();
}
