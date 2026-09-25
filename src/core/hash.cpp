// src/core/hash.cpp - see g2m/core/hash.h.

#include "g2m/core/hash.h"

#include <algorithm>
#include <bit>
#include <cstring>

namespace g2m {

Fnv1a64& Fnv1a64::bytes(std::span<const std::byte> data) {
    for (const std::byte b : data) {
        h_ ^= static_cast<std::uint64_t>(b);
        h_ *= kPrime;
    }
    return *this;
}

Fnv1a64& Fnv1a64::bytes(const void* data, std::size_t size) {
    return bytes(std::span<const std::byte>(static_cast<const std::byte*>(data), size));
}

Fnv1a64& Fnv1a64::u8(std::uint8_t v) {
    h_ ^= v;
    h_ *= kPrime;
    return *this;
}

Fnv1a64& Fnv1a64::u16(std::uint16_t v) {
    for (int i = 0; i < 2; ++i) u8(static_cast<std::uint8_t>(v >> (8 * i)));
    return *this;
}

Fnv1a64& Fnv1a64::u32(std::uint32_t v) {
    for (int i = 0; i < 4; ++i) u8(static_cast<std::uint8_t>(v >> (8 * i)));
    return *this;
}

Fnv1a64& Fnv1a64::u64(std::uint64_t v) {
    for (int i = 0; i < 8; ++i) u8(static_cast<std::uint8_t>(v >> (8 * i)));
    return *this;
}

Fnv1a64& Fnv1a64::f64(double v) { return u64(std::bit_cast<std::uint64_t>(v)); }

Fnv1a64& Fnv1a64::str(std::string_view s) {
    u64(s.size());
    return bytes(s.data(), s.size());
}

// ---- SHA-256 (FIPS 180-4) ----

namespace {

constexpr std::array<std::uint32_t, 64> kK = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2,
};

constexpr std::array<std::uint32_t, 8> kInit = {
    0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19,
};

} // namespace

Sha256::Sha256() : state_(kInit) {}

void Sha256::block(const std::uint8_t* p) {
    std::uint32_t w[64];
    for (int i = 0; i < 16; ++i) {
        w[i] = (static_cast<std::uint32_t>(p[4 * i]) << 24) | (static_cast<std::uint32_t>(p[4 * i + 1]) << 16) |
               (static_cast<std::uint32_t>(p[4 * i + 2]) << 8) | static_cast<std::uint32_t>(p[4 * i + 3]);
    }
    for (int i = 16; i < 64; ++i) {
        const std::uint32_t s0 = std::rotr(w[i - 15], 7) ^ std::rotr(w[i - 15], 18) ^ (w[i - 15] >> 3);
        const std::uint32_t s1 = std::rotr(w[i - 2], 17) ^ std::rotr(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::uint32_t a = state_[0], b = state_[1], c = state_[2], d = state_[3];
    std::uint32_t e = state_[4], f = state_[5], g = state_[6], h = state_[7];
    for (int i = 0; i < 64; ++i) {
        const std::uint32_t S1 = std::rotr(e, 6) ^ std::rotr(e, 11) ^ std::rotr(e, 25);
        const std::uint32_t ch = (e & f) ^ (~e & g);
        const std::uint32_t t1 = h + S1 + ch + kK[static_cast<std::size_t>(i)] + w[i];
        const std::uint32_t S0 = std::rotr(a, 2) ^ std::rotr(a, 13) ^ std::rotr(a, 22);
        const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
        const std::uint32_t t2 = S0 + maj;
        h = g;
        g = f;
        f = e;
        e = d + t1;
        d = c;
        c = b;
        b = a;
        a = t1 + t2;
    }
    state_[0] += a;
    state_[1] += b;
    state_[2] += c;
    state_[3] += d;
    state_[4] += e;
    state_[5] += f;
    state_[6] += g;
    state_[7] += h;
}

Sha256& Sha256::update(std::span<const std::byte> data) {
    const auto* p = reinterpret_cast<const std::uint8_t*>(data.data());
    std::size_t n = data.size();
    total_len_ += n;
    if (buf_len_ > 0) {
        const std::size_t take = std::min(n, 64 - buf_len_);
        std::memcpy(buf_.data() + buf_len_, p, take);
        buf_len_ += take;
        p += take;
        n -= take;
        if (buf_len_ == 64) {
            block(buf_.data());
            buf_len_ = 0;
        }
    }
    while (n >= 64) {
        block(p);
        p += 64;
        n -= 64;
    }
    if (n > 0) {
        std::memcpy(buf_.data(), p, n);
        buf_len_ = n;
    }
    return *this;
}

Sha256& Sha256::update(const void* data, std::size_t size) {
    return update(std::span<const std::byte>(static_cast<const std::byte*>(data), size));
}

Sha256::Digest Sha256::finish() {
    const std::uint64_t bit_len = total_len_ * 8;
    buf_[buf_len_++] = 0x80;
    if (buf_len_ > 56) {
        while (buf_len_ < 64) buf_[buf_len_++] = 0;
        block(buf_.data());
        buf_len_ = 0;
    }
    while (buf_len_ < 56) buf_[buf_len_++] = 0;
    for (int i = 7; i >= 0; --i) buf_[buf_len_++] = static_cast<std::uint8_t>(bit_len >> (8 * i));
    block(buf_.data());
    buf_len_ = 0;
    Digest out{};
    for (std::size_t i = 0; i < 8; ++i) {
        out[4 * i] = static_cast<std::uint8_t>(state_[i] >> 24);
        out[4 * i + 1] = static_cast<std::uint8_t>(state_[i] >> 16);
        out[4 * i + 2] = static_cast<std::uint8_t>(state_[i] >> 8);
        out[4 * i + 3] = static_cast<std::uint8_t>(state_[i]);
    }
    return out;
}

Sha256::Digest Sha256::of(std::span<const std::byte> data) { return Sha256().update(data).finish(); }
Sha256::Digest Sha256::of(std::string_view s) { return Sha256().update(s).finish(); }

std::string to_hex(std::span<const std::uint8_t> bytes) {
    static constexpr char kDigits[] = "0123456789abcdef";
    std::string s;
    s.reserve(bytes.size() * 2);
    for (const std::uint8_t b : bytes) {
        s.push_back(kDigits[b >> 4]);
        s.push_back(kDigits[b & 0xf]);
    }
    return s;
}

std::string to_hex(std::uint64_t v) {
    static constexpr char kDigits[] = "0123456789abcdef";
    std::string s(16, '0');
    for (int i = 15; i >= 0; --i) {
        s[static_cast<std::size_t>(i)] = kDigits[v & 0xf];
        v >>= 4;
    }
    return s;
}

} // namespace g2m
