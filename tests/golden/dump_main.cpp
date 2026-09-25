// tests/golden/dump_main.cpp - g2m_golden_dump: prints the G0 golden hashes
// ("name count hash" per line) to stdout, or writes them to a file.
// tools/ci.ps1 compares this output across every build leg; regenerate
// testdata/golden/g0_golden.txt with `g2m_golden_dump <path>` after an
// INTENDED output change (and say why in the commit).

#include "golden.h"

#include "headless_env.h"

#include <cstdio>
#include <fstream>
#include <string>

int main(int argc, char** argv) {
    g2m_test::install_headless_crt_handlers();
    const std::string text = g2m_golden::format(g2m_golden::compute_g0());
    if (argc == 1) {
        std::fputs(text.c_str(), stdout);
        return 0;
    }
    if (argc != 2) {
        std::fprintf(stderr, "usage: g2m_golden_dump [output file]\n");
        return 2;
    }
    std::ofstream f(argv[1], std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "g2m_golden_dump: cannot open %s\n", argv[1]);
        return 1;
    }
    f << text;
    f.close();
    return f ? 0 : 1;
}
