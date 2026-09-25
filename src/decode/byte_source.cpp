// src/decode/byte_source.cpp - see g2m/decode/byte_source.h.
//
// Large-file-safe random access: on Windows/MSVC-ABI (clang-cl and cl both
// define _WIN32 and use the same ucrt), plain fopen/fseek's `long` offset is
// only 32-bit even in a 64-bit build, so a file past 2 GiB (every PKT_*.zip
// here, up to 3.64 GB) silently wraps. _fseeki64/_ftelli64 take a 64-bit
// offset and are the documented fix. Elsewhere, off_t is already 64-bit with
// _FILE_OFFSET_BITS=64 (set by CMake's usual toolchain defaults on the
// Linux/WSL presets), so plain fseeko/ftello suffice.

#include "g2m/decode/byte_source.h"

#include <cstdio>
#include <cstring>

#if defined(_WIN32)
#define G2M_FSEEK _fseeki64
#define G2M_FTELL _ftelli64
using g2m_off_t = long long;
#else
#include <sys/types.h>
#define G2M_FSEEK fseeko
#define G2M_FTELL ftello
using g2m_off_t = off_t;
#endif

namespace g2m::decode {

struct FileByteSource::Impl {
    std::FILE* fp = nullptr;
    ~Impl() {
        if (fp) std::fclose(fp);
    }
};

FileByteSource::FileByteSource(const std::string& path) : impl_(std::make_unique<Impl>()) {
#if defined(_WIN32)
    // "rb" avoids CRLF translation entirely (irrelevant for binary data,
    // but keeps the read byte-for-byte regardless of locale settings).
    const errno_t err = fopen_s(&impl_->fp, path.c_str(), "rb");
    if (err != 0 || impl_->fp == nullptr) {
        throw DecodeError("FileByteSource: cannot open '" + path + "'");
    }
#else
    impl_->fp = std::fopen(path.c_str(), "rb");
    if (impl_->fp == nullptr) {
        throw DecodeError("FileByteSource: cannot open '" + path + "'");
    }
#endif
    if (G2M_FSEEK(impl_->fp, 0, SEEK_END) != 0) {
        throw DecodeError("FileByteSource: seek-to-end failed for '" + path + "'");
    }
    const g2m_off_t end = G2M_FTELL(impl_->fp);
    if (end < 0) {
        throw DecodeError("FileByteSource: tell failed for '" + path + "'");
    }
    size_ = static_cast<std::uint64_t>(end);
}

FileByteSource::~FileByteSource() = default;

void FileByteSource::read(std::uint64_t offset, std::uint64_t len, std::vector<std::byte>& out) const {
    if (len == 0) {
        out.clear();
        return;
    }
    if (offset > size_ || len > size_ - offset) {
        throw DecodeError("FileByteSource::read: out of range");
    }
    if (G2M_FSEEK(impl_->fp, static_cast<g2m_off_t>(offset), SEEK_SET) != 0) {
        throw DecodeError("FileByteSource::read: seek failed");
    }
    out.resize(len);
    const std::size_t got = std::fread(out.data(), 1, static_cast<std::size_t>(len), impl_->fp);
    if (got != len) {
        throw DecodeError("FileByteSource::read: short read");
    }
}

void MemoryByteSource::read(std::uint64_t offset, std::uint64_t len, std::vector<std::byte>& out) const {
    if (len == 0) {
        out.clear();
        return;
    }
    if (offset > data_.size() || len > data_.size() - offset) {
        throw DecodeError("MemoryByteSource::read: out of range");
    }
    out.resize(len);
    std::memcpy(out.data(), data_.data() + offset, static_cast<std::size_t>(len));
}

} // namespace g2m::decode
