// tests/fuzz_formats/fuzz_seeds_main.cpp - g2m_fuzz_seeds <dir>: writes the
// seed corpus of every target to <dir>/seeds/<name>/seed_<i> and creates an
// empty libFuzzer work corpus <dir>/work/<name> (cleared on every run, so a
// CI run always starts from the same seeds).

#include "fuzz_targets.h"

#include "headless_env.h"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>

int main(int argc, char** argv) {
    g2m_test::install_headless_crt_handlers();
    if (argc != 2) {
        std::fprintf(stderr, "usage: g2m_fuzz_seeds <dir>\n");
        return 2;
    }
    namespace fs = std::filesystem;
    const fs::path root(argv[1]);
    for (const auto& t : g2m_fuzz::kTargets) {
        const std::string name(t.name);
        const fs::path seeds = root / "seeds" / name;
        const fs::path work = root / "work" / name;
        std::error_code ec;
        fs::remove_all(seeds, ec);
        fs::remove_all(work, ec);
        fs::create_directories(seeds);
        fs::create_directories(work);
        const auto list = g2m_fuzz::seeds(t.target);
        for (std::size_t i = 0; i < list.size(); ++i) {
            std::ofstream f(seeds / ("seed_" + std::to_string(i)), std::ios::binary);
            f.write(reinterpret_cast<const char*>(list[i].data()), static_cast<std::streamsize>(list[i].size()));
            if (!f) {
                std::fprintf(stderr, "g2m_fuzz_seeds: cannot write %s\n", (seeds / ("seed_" + std::to_string(i))).string().c_str());
                return 1;
            }
        }
        std::printf("%s: %zu seeds\n", name.c_str(), list.size());
    }
    return 0;
}
