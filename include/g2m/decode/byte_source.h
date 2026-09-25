// g2m/decode/byte_source.h - abstract random-access byte input (PLAN.md
// 2.2/13, milestone G1b).
//
// Every decoder in g2m_decode (ZIP, TIFF/GeoTIFF, and the nested-zip case)
// reads through this seam instead of a bare file handle, so:
//  - a Cloud-Optimised GeoTIFF window read and a local-file read use the
//    same code path;
//  - a nested ZIP's inner archive (decompressed into memory, see zip/
//    zip_reader.h) can be re-parsed as its own ZIP without ever touching
//    disk;
//  - large-file-safe 64-bit offsets are enforced in exactly one place.
#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <span>
#include <string>
#include <vector>

namespace g2m::decode {

// Thrown by any g2m_decode reader on malformed/out-of-range input. Every
// decoder is fed untrusted bytes (a downloaded file, a server response), so
// this is the one error channel callers need to handle - never a bare
// assert/abort on bad input.
class DecodeError : public std::exception {
public:
    explicit DecodeError(std::string message) : message_(std::move(message)) {}
    const char* what() const noexcept override { return message_.c_str(); }

private:
    std::string message_;
};

// Abstract random-access byte source. size() is fixed for the source's
// lifetime (no growing/streaming sources). read() is bounds-checked by the
// implementation and throws DecodeError on an out-of-range request - callers
// (ZIP/TIFF parsers) validate offsets against size() themselves first where
// it is cheap to do so, but must never rely on an out-of-bounds read
// silently truncating.
class IByteSource {
public:
    virtual ~IByteSource() = default;

    virtual std::uint64_t size() const = 0;

    // Reads exactly `len` bytes starting at `offset` into `out` (resized to
    // `len`). Throws DecodeError if [offset, offset + len) is not entirely
    // within [0, size()), or on an underlying I/O failure.
    virtual void read(std::uint64_t offset, std::uint64_t len, std::vector<std::byte>& out) const = 0;

    // Convenience: allocate-and-return form of read().
    std::vector<std::byte> read(std::uint64_t offset, std::uint64_t len) const {
        std::vector<std::byte> out;
        read(offset, len, out);
        return out;
    }
};

// A local file, opened for large-file-safe random-access reads (64-bit
// offsets: _fseeki64 on the MSVC CRT, plain 64-bit off_t elsewhere - see
// byte_source.cpp). Read-only; never memory-maps (works the same for a
// 3.64 GB DGM1 delivery ZIP as for a 300 MB PBF).
class FileByteSource final : public IByteSource {
public:
    // Throws DecodeError if the file cannot be opened.
    explicit FileByteSource(const std::string& path);
    ~FileByteSource() override;

    FileByteSource(const FileByteSource&) = delete;
    FileByteSource& operator=(const FileByteSource&) = delete;

    std::uint64_t size() const override { return size_; }
    void read(std::uint64_t offset, std::uint64_t len, std::vector<std::byte>& out) const override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::uint64_t size_ = 0;
};

// An in-memory buffer (e.g. a nested ZIP's inner archive, decompressed once
// into RAM - PLAN.md's "inner zips are small" assumption). Takes ownership
// of the bytes; never copies on construction from an rvalue.
class MemoryByteSource final : public IByteSource {
public:
    explicit MemoryByteSource(std::vector<std::byte> data) : data_(std::move(data)) {}

    std::uint64_t size() const override { return data_.size(); }
    void read(std::uint64_t offset, std::uint64_t len, std::vector<std::byte>& out) const override;

    std::span<const std::byte> data() const { return data_; }

private:
    std::vector<std::byte> data_;
};

} // namespace g2m::decode
