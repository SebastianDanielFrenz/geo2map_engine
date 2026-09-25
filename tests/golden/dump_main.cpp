// tests/golden/dump_main.cpp - g2m_golden_dump: prints one golden set's
// hashes ("name count hash" per line) to stdout, or writes them to a file.
//   g2m_golden_dump [g0|g1] [output file]      (default set: g0)
// tools/ci.ps1 compares this output across every build leg; regenerate
// testdata/golden/g<N>_golden.txt with `g2m_golden_dump g<N> <path>` after
// an INTENDED output change (and say why in the commit).

#include "golden.h"

#include "headless_env.h"

#include <cstdio>
#include <fstream>
#include <string>

int main(int argc, char** argv) {
    g2m_test::install_headless_crt_handlers();
    std::string set = "g0";
    int arg = 1;
    if (argc > 1 && (std::string(argv[1]) == "g0" || std::string(argv[1]) == "g1")) {
        set = argv[1];
        arg = 2;
    }
    if (argc - arg > 1) {
        std::fprintf(stderr, "usage: g2m_golden_dump [g0|g1] [output file]\n");
        return 2;
    }
    const std::string text = g2m_golden::format(set == "g1" ? g2m_golden::compute_g1() : g2m_golden::compute_g0());
    if (argc == arg) {
        std::fputs(text.c_str(), stdout);
        return 0;
    }
    std::ofstream f(argv[arg], std::ios::binary);
    if (!f) {
        std::fprintf(stderr, "g2m_golden_dump: cannot open %s\n", argv[arg]);
        return 1;
    }
    f << text;
    f.close();
    return f ? 0 : 1;
}
