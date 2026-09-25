// g2m/core/hash.h - stable hashes (PLAN.md 2.4, 3.4).
//
//  - Fnv1a64: FNV-1a over an explicit little-endian byte serialisation.
//    Cheap, stable across compilers/OSes; used for golden hashes and
//    in-process identity checks. Not collision-resistant.
//  - Sha256: FIPS 180-4, for content addressing (a tile's ETag is the
//    SHA-256 of its body, PLAN.md 3.4).
//
// Doubles are hashed by their IEEE-754 bit pattern, so -0.0 != +0.0 and
// two NaNs with different payloads differ: a golden hash checks bits, not
// values.
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>

namespace g2m {

class Fnv1a64 {
public:
    static constexpr std::uint64_t kOffsetBasis = 0xcbf29ce484222325ull;
    static constexpr std::uint64_t kPrime = 0x100000001b3ull;

    Fnv1a64& bytes(std::span<const std::byte> data);
    Fnv1a64& bytes(const void* data, std::size_t size);
    Fnv1a64& u8(std::uint8_t v);
    Fnv1a64& u16(std::uint16_t v);   // little-endian
    Fnv1a64& u32(std::uint32_t v);   // little-endian
    Fnv1a64& u64(std::uint64_t v);   // little-endian
    Fnv1a64& i32(std::int32_t v) { return u32(static_cast<std::uint32_t>(v)); }
    Fnv1a64& i64(std::int64_t v) { return u64(static_cast<std::uint64_t>(v)); }
    Fnv1a64& f64(double v);          // bit pattern, little-endian
    // Length-prefixed (u64 length, then the bytes), so ("ab","c") != ("a","bc").
    Fnv1a64& str(std::string_view s);

    std::uint64_t value() const { return h_; }

private:
    std::uint64_t h_ = kOffsetBasis;
};

class Sha256 {
public:
    using Digest = std::array<std::uint8_t, 32>;

    Sha256();
    Sha256& update(std::span<const std::byte> data);
    Sha256& update(const void* data, std::size_t size);
    Sha256& update(std::string_view s) { return update(s.data(), s.size()); }
    // Finishes the hash; the object must not be updated afterwards.
    Digest finish();

    static Digest of(std::span<const std::byte> data);
    static Digest of(std::string_view s);

private:
    void block(const std::uint8_t* p);

    std::array<std::uint32_t, 8> state_;
    std::array<std::uint8_t, 64> buf_{};
    std::size_t buf_len_ = 0;
    std::uint64_t total_len_ = 0; // bytes
};

// Lower-case hex, most significant byte first.
std::string to_hex(std::span<const std::uint8_t> bytes);
// 16 hex digits.
std::string to_hex(std::uint64_t v);

} // namespace g2m
