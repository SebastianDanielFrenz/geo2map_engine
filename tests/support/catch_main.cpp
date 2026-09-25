// tests/support/catch_main.cpp - Catch2 entry point for every g2m test
// executable (instead of Catch2::Catch2WithMain): installs the headless CRT
// handlers first so a failing assert cannot hang a CI run on a dialog.

#include "headless_env.h"

#include <catch2/catch_session.hpp>

int main(int argc, char* argv[]) {
    g2m_test::install_headless_crt_handlers();
    return Catch::Session().run(argc, argv);
}
