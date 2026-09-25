// g2m/core/bytes.h - little-endian byte serialisation with bounds checks,
// the building block of every g2m binary format (docs/formats/).
//
//  - ByteWriter appends to a growing buffer; it cannot fail.
//  - ByteReader reads from an untrusted span. A read past the end, or a
//    non-canonical varint, sets a sticky failure flag and returns 0; the
//    caller checks ok() at convenient points. Nothing ever reads outside
//    the span, so a decoder built on it is safe to fuzz.
//
// Varints are unsigned LEB128 (7 bits per byte, low group first). The
// reader accepts only the canonical (shortest) encoding: a multi-byte
// varint whose last byte is 0 is rejected, and so is anything longer than
// 10 bytes or overflowing 64 bits. So every value has exactly one byte
// sequence, and decode -> encode reproduces the input.
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {

constexpr std::uint64_t zigzag_encode(std::int64_t v) {
    return (static_cast<std::uint64_t>(v) << 1) ^ static_cast<std::uint64_t>(v >> 63);
}
constexpr std::int64_t zigzag_decode(std::uint64_t u) {
    return static_cast<std::int64_t>(u >> 1) ^ -static_cast<std::int64_t>(u & 1);
}
constexpr std::uint32_t zigzag_encode32(std::int32_t v) {
    return (static_cast<std::uint32_t>(v) << 1) ^ static_cast<std::uint32_t>(v >> 31);
}
constexpr std::int32_t zigzag_decode32(std::uint32_t u) {
    return static_cast<std::int32_t>(u >> 1) ^ -static_cast<std::int32_t>(u & 1);
}

class ByteWriter {
public:
    ByteWriter() = default;
    explicit ByteWriter(std::vector<std::uint8_t> reuse) : buf_(std::move(reuse)) { buf_.clear(); }

    void u8(std::uint8_t v) { buf_.push_back(v); }
    void u16(std::uint16_t v) { le(v, 2); }
    void u32(std::uint32_t v) { le(v, 4); }
    void u64(std::uint64_t v) { le(v, 8); }
    void i32(std::int32_t v) { u32(static_cast<std::uint32_t>(v)); }
    void i64(std::int64_t v) { u64(static_cast<std::uint64_t>(v)); }
    void varint(std::uint64_t v) {
        while (v >= 0x80) {
            buf_.push_back(static_cast<std::uint8_t>(v | 0x80));
            v >>= 7;
        }
        buf_.push_back(static_cast<std::uint8_t>(v));
    }
    void svarint(std::int64_t v) { varint(zigzag_encode(v)); }
    void bytes(std::span<const std::uint8_t> b) { buf_.insert(buf_.end(), b.begin(), b.end()); }
    void bytes(const void* p, std::size_t n) {
        const auto* b = static_cast<const std::uint8_t*>(p);
        buf_.insert(buf_.end(), b, b + n);
    }
    // Overwrites 4 bytes at pos (already written) - for back-patched sizes.
    void patch_u32(std::size_t pos, std::uint32_t v) {
        for (int i = 0; i < 4; ++i) buf_[pos + static_cast<std::size_t>(i)] = static_cast<std::uint8_t>(v >> (8 * i));
    }

    std::size_t size() const { return buf_.size(); }
    const std::vector<std::uint8_t>& data() const { return buf_; }
    std::vector<std::uint8_t> take() { return std::move(buf_); }

private:
    void le(std::uint64_t v, int n) {
        for (int i = 0; i < n; ++i) buf_.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    }
    std::vector<std::uint8_t> buf_;
};

class ByteReader {
public:
    explicit ByteReader(std::span<const std::uint8_t> data) : data_(data) {}

    bool ok() const { return ok_; }
    void fail() { ok_ = false; }
    std::size_t pos() const { return pos_; }
    std::size_t remaining() const { return ok_ ? data_.size() - pos_ : 0; }
    bool at_end() const { return ok_ && pos_ == data_.size(); }

    std::uint8_t u8() { return static_cast<std::uint8_t>(le(1)); }
    std::uint16_t u16() { return static_cast<std::uint16_t>(le(2)); }
    std::uint32_t u32() { return static_cast<std::uint32_t>(le(4)); }
    std::uint64_t u64() { return le(8); }
    std::int32_t i32() { return static_cast<std::int32_t>(u32()); }
    std::int64_t i64() { return static_cast<std::int64_t>(u64()); }

    std::uint64_t varint() {
        std::uint64_t v = 0;
        for (int i = 0; i < 10; ++i) {
            if (!ok_ || pos_ >= data_.size()) {
                ok_ = false;
                return 0;
            }
            const std::uint8_t b = data_[pos_++];
            if (i == 9 && b > 1) break;           // would overflow 64 bits
            v |= static_cast<std::uint64_t>(b & 0x7f) << (7 * i);
            if ((b & 0x80) == 0) {
                if (b == 0 && i > 0) break;       // overlong (non-canonical)
                return v;
            }
        }
        ok_ = false;
        return 0;
    }
    std::int64_t svarint() { return zigzag_decode(varint()); }

    // A view of the next n bytes (empty + failure if fewer remain).
    std::span<const std::uint8_t> bytes(std::size_t n) {
        if (!ok_ || n > data_.size() - pos_) {
            ok_ = false;
            return {};
        }
        const auto s = data_.subspan(pos_, n);
        pos_ += n;
        return s;
    }
    std::string_view string(std::size_t n) {
        const auto s = bytes(n);
        return {reinterpret_cast<const char*>(s.data()), s.size()};
    }

private:
    std::uint64_t le(int n) {
        const auto un = static_cast<std::size_t>(n);
        if (!ok_ || un > data_.size() - pos_) {
            ok_ = false;
            return 0;
        }
        std::uint64_t v = 0;
        for (std::size_t i = 0; i < un; ++i) v |= static_cast<std::uint64_t>(data_[pos_ + i]) << (8 * i);
        pos_ += un;
        return v;
    }

    std::span<const std::uint8_t> data_;
    std::size_t pos_ = 0;
    bool ok_ = true;
};

} // namespace g2m
