// tests/golden/dump_main.cpp - g2m_golden_dump: prints one golden set's
// hashes ("name count hash" per line) to stdout, or writes them to a file.
//   g2m_golden_dump [g0|g1|g1c|g2|g2mesh|g2roads|g2paint|g3xs|g3j] [output file]      (default set: g0)
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
    if (argc > 1 && (std::string(argv[1]) == "g0" || std::string(argv[1]) == "g1" || std::string(argv[1]) == "g1c" ||
                     std::string(argv[1]) == "g2" || std::string(argv[1]) == "g2mesh" ||
                     std::string(argv[1]) == "g2roads" || std::string(argv[1]) == "g2paint" || std::string(argv[1]) == "g3xs" ||
                     std::string(argv[1]) == "g3j")) {
        set = argv[1];
        arg = 2;
    }
    if (argc - arg > 1) {
        std::fprintf(stderr, "usage: g2m_golden_dump [g0|g1|g1c|g2|g2mesh|g2roads|g2paint|g3xs|g3j] [output file]\n");
        return 2;
    }
    const std::vector<g2m_golden::Entry> entries = set == "g3j" ? g2m_golden::compute_g3j()
                                                     : set == "g3xs" ? g2m_golden::compute_g3xs()
                                                     : set == "g2paint" ? g2m_golden::compute_g2paint()
                                                     : set == "g2roads" ? g2m_golden::compute_g2roads()
                                                     : set == "g2mesh" ? g2m_golden::compute_g2mesh()
                                                     : set == "g2"    ? g2m_golden::compute_g2()
                                                     : set == "g1c" ? g2m_golden::compute_g1c()
                                                     : set == "g1"  ? g2m_golden::compute_g1()
                                                                    : g2m_golden::compute_g0();
    const std::string text = g2m_golden::format(entries);
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
