// tests/support/headless_env.h - make test/tool executables fail fast
// instead of hanging on a Windows modal dialog (debug-CRT assert, abort(),
// WER crash box). A no-op elsewhere. Never linked into g2m_core.
#pragma once

namespace g2m_test {

void install_headless_crt_handlers();

} // namespace g2m_test
