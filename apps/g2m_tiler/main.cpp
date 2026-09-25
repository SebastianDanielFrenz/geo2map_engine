// apps/g2m_tiler/main.cpp - the importer CLI (PLAN.md G1c I2). Manual argv
// parsing (no CLI-parsing dependency - PLAN.md's "no new dependencies").
//
// Subcommands:
//   find-admin --pbf PATH [--name-contains STR] [--admin-level N]
//   resolve PATH#NAME --pbf PATH --out PATH
//   import-dem / import-osm / import-landcover ...   (stubs until I3-I5)
//   import ...                                        (stub until I3-I5)
//   stats --store DIR [--scope SCOPE]
//
// Exit codes: 0 success, 1 usage/runtime error, 2 "not implemented yet".
#include "rss.h"

#include "g2m/import/region_resolve.h"
#include "g2m/layer/region.h"
#include "g2m/server/tile_store.h"

#include <chrono>
#include <cstdio>
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
                 "  import-dem ...        (not implemented yet, I3)\n"
                 "  import-osm ...        (not implemented yet, I4)\n"
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
    std::cerr << "g2m_tiler resolve: '" << region_name << "': " << resolved->tile_count << " tiles ("
              << resolved->interior.size() << " interior runs), " << wall_s << "s, peak RSS "
              << (g2m_tiler::peak_rss_bytes() / (1024 * 1024)) << " MiB\n";
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
    if (command == "import-dem") return not_implemented("import-dem");
    if (command == "import-osm") return not_implemented("import-osm");
    if (command == "import-landcover") return not_implemented("import-landcover");
    if (command == "import") return not_implemented("import");
    if (command == "stats") return cmd_stats(args);
    if (command == "--help" || command == "-h" || command == "help") return usage(), 0;
    std::cerr << "g2m_tiler: unknown command '" << command << "'\n";
    return usage();
}
