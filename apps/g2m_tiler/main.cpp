// apps/g2m_tiler/main.cpp - the importer CLI (PLAN.md G1c I2). Manual argv
// parsing (no CLI-parsing dependency - PLAN.md's "no new dependencies").
//
// Subcommands:
//   find-admin --pbf PATH [--name-contains STR] [--admin-level N]
//   resolve PATH#NAME --pbf PATH --out PATH
//   import-dem --resolved PATH --store DIR --scope SCOPE
//              (--pkt PATH [--pkt PATH ...] | --data DIR) [--jobs N] [--km-cache N]
//   import-osm --pbf PATH --resolved PATH --store DIR --scope SCOPE [--jobs N]
//   import-landcover ...                 (stub until I5)
//   import ...                           (stub until I5)
//   stats --store DIR [--scope SCOPE]
//
// Exit codes: 0 success, 1 usage/runtime error, 2 "not implemented yet".
#include "rss.h"

#include "g2m/decode/dgm1.h"
#include "g2m/import/dem.h"
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

int usage() {
    std::cerr << "usage: g2m_tiler <command> [options]\n"
                 "commands:\n"
                 "  find-admin --pbf PATH [--name-contains STR] [--admin-level N]\n"
                 "  resolve PATH#NAME --pbf PATH --out PATH\n"
                 "  import-dem --resolved PATH --store DIR --scope SCOPE\n"
                 "             (--pkt PATH [--pkt PATH ...] | --data DIR)\n"
                 "             [--jobs N] [--km-cache N]\n"
                 "  import-osm --pbf PATH --resolved PATH --store DIR --scope SCOPE [--jobs N]\n"
                 "  import-landcover ...  (not implemented yet, I5)\n"
                 "  import ...            (not implemented yet, needs I3-I5)\n"
                 "  stats --store DIR [--scope SCOPE]\n";
    return 1;
}

int not_implemented(const std::string& what) {
    std::cerr << "g2m_tiler: " << what << ": not implemented yet\n";
    return 2;
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
    if (command == "import-landcover") return not_implemented("import-landcover");
    if (command == "import") return not_implemented("import");
    if (command == "stats") return cmd_stats(args);
    if (command == "--help" || command == "-h" || command == "help") return usage(), 0;
    std::cerr << "g2m_tiler: unknown command '" << command << "'\n";
    return usage();
}
