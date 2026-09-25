// src/layer/zstd_codec.cpp - see zstd_codec.h.

#include "zstd_codec.h"

#define ZSTD_STATIC_LINKING_ONLY // experimental parameters (pinned below)
#include <zstd.h>

#include <cstdlib>
#include <memory>
#include <string>

namespace g2m::layer_detail {
namespace {

static_assert(ZSTD_VERSION_NUMBER == 10507, "g2m pins zstd 1.5.7 (cmake/deps_core.cmake): "
                                            "a different version may compress to different bytes");

struct CCtxDeleter {
    void operator()(ZSTD_CCtx* c) const { ZSTD_freeCCtx(c); }
};
struct DCtxDeleter {
    void operator()(ZSTD_DCtx* d) const { ZSTD_freeDCtx(d); }
};

// A failure here is a programming error (a parameter this zstd build does
// not know): abort loudly rather than compress with an unpinned default.
void set(ZSTD_CCtx* c, ZSTD_cParameter p, int v) {
    const std::size_t r = ZSTD_CCtx_setParameter(c, p, v);
    if (ZSTD_isError(r)) {
        std::abort();
    }
}

} // namespace

std::vector<std::uint8_t> zstd_compress(std::span<const std::uint8_t> raw, int level) {
    std::unique_ptr<ZSTD_CCtx, CCtxDeleter> c(ZSTD_createCCtx());
    if (!c) std::abort();
    // Every parameter whose default is "auto" or run-time dependent is pinned.
    set(c.get(), ZSTD_c_compressionLevel, level);
    set(c.get(), ZSTD_c_nbWorkers, 0);                          // single-threaded, always
    set(c.get(), ZSTD_c_checksumFlag, 0);
    set(c.get(), ZSTD_c_contentSizeFlag, 1);
    set(c.get(), ZSTD_c_dictIDFlag, 0);
    set(c.get(), ZSTD_c_enableLongDistanceMatching, ZSTD_ps_disable);
    set(c.get(), ZSTD_c_useRowMatchFinder, ZSTD_ps_disable);    // auto depends on SIMD support
    set(c.get(), ZSTD_c_blockSplitterLevel, 1);                 // 1 = no pre-splitting
    set(c.get(), ZSTD_c_splitAfterSequences, ZSTD_ps_disable);
    set(c.get(), ZSTD_c_literalCompressionMode, ZSTD_ps_auto);  // a pure function of the level, no CPU dependence
    set(c.get(), ZSTD_c_format, ZSTD_f_zstd1);
    set(c.get(), ZSTD_c_forceMaxWindow, 0);
    set(c.get(), ZSTD_c_targetCBlockSize, 0);

    std::vector<std::uint8_t> out(ZSTD_compressBound(raw.size()));
    const std::size_t n = ZSTD_compress2(c.get(), out.data(), out.size(), raw.data(), raw.size());
    if (ZSTD_isError(n)) std::abort(); // cannot happen with a compressBound-sized buffer
    out.resize(n);
    return out;
}

Result<std::vector<std::uint8_t>> zstd_decompress(std::span<const std::uint8_t> frame, std::size_t expected_raw_size) {
    const std::size_t frame_size = ZSTD_findFrameCompressedSize(frame.data(), frame.size());
    if (ZSTD_isError(frame_size)) {
        return make_error(ErrorCode::Malformed, std::string("zstd frame: ") + ZSTD_getErrorName(frame_size));
    }
    if (frame_size != frame.size()) {
        return make_error(ErrorCode::Malformed, "zstd: bytes after the frame");
    }
    const unsigned long long declared = ZSTD_getFrameContentSize(frame.data(), frame.size());
    if (declared == ZSTD_CONTENTSIZE_ERROR || declared == ZSTD_CONTENTSIZE_UNKNOWN) {
        return make_error(ErrorCode::Malformed, "zstd: frame content size missing");
    }
    if (declared != expected_raw_size) {
        return make_error(ErrorCode::Malformed, "zstd: frame content size != section raw size");
    }
    std::unique_ptr<ZSTD_DCtx, DCtxDeleter> d(ZSTD_createDCtx());
    if (!d) std::abort();
    if (ZSTD_isError(ZSTD_DCtx_setParameter(d.get(), ZSTD_d_windowLogMax, kZstdWindowLogMax))) std::abort();
    std::vector<std::uint8_t> out(expected_raw_size);
    const std::size_t n = ZSTD_decompressDCtx(d.get(), out.data(), out.size(), frame.data(), frame.size());
    if (ZSTD_isError(n)) {
        return make_error(ErrorCode::Malformed, std::string("zstd: ") + ZSTD_getErrorName(n));
    }
    if (n != expected_raw_size) {
        return make_error(ErrorCode::Malformed, "zstd: decoded size != section raw size");
    }
    return out;
}

const char* zstd_version() { return ZSTD_versionString(); }

} // namespace g2m::layer_detail
