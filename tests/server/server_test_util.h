// tests/server/server_test_util.h - shared helpers for the store, server
// and client-cache tests: a self-deleting temp directory and synthetic
// tile bodies (SplitMix64 bytes in one uncompressed section, so a body's
// size is exact).
#pragma once

#include "splitmix64.h"
#include "synthetic.h"

#include "g2m/layer/release.h"
#include "g2m/layer/tile_container.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>
#include <system_error>
#include <vector>

namespace g2m_test {

// A fresh directory under the system temp dir, removed (best effort) on
// destruction. Anything holding files in it must be destroyed first.
class TempDir {
public:
    explicit TempDir(const std::string& tag) {
        static std::atomic<std::uint64_t> counter{0};
        const auto now = static_cast<std::uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
        path_ = std::filesystem::temp_directory_path() /
                ("g2m_test_" + tag + "_" + std::to_string(now) + "_" + std::to_string(counter.fetch_add(1)));
        std::filesystem::create_directories(path_);
    }
    ~TempDir() {
        std::error_code ec;
        std::filesystem::remove_all(path_, ec);
    }
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
};

// An encoded tile body of exactly `size` bytes (size >= 16), content from `seed`.
inline std::vector<std::uint8_t> make_body(std::uint64_t seed, std::size_t size) {
    SplitMix64 r(seed);
    g2m::TileBody b;
    b.encoding = 0x8000;
    b.encoding_version = 1;
    // Body framing overhead with one uncompressed section; measured once below.
    g2m::Section s{1, g2m::SectionCodec::None, {}};
    b.sections.push_back(s);
    const std::size_t overhead = g2m::encode_body(b).value().size();
    std::vector<std::uint8_t> data(size > overhead ? size - overhead : 0);
    for (std::uint8_t& v : data) {
        const std::uint64_t x = r.next();
        v = static_cast<std::uint8_t>(x);
    }
    b.sections[0].data = std::move(data);
    auto out = g2m::encode_body(b).value();
    // Varint section lengths may add a byte or two: trim/pad the payload once more.
    while (out.size() != size && !b.sections[0].data.empty()) {
        if (out.size() > size) {
            b.sections[0].data.pop_back();
        } else {
            b.sections[0].data.push_back(0);
        }
        out = g2m::encode_body(b).value();
    }
    return out;
}

inline g2m::TileHeader make_header(const std::string& layer, const g2m::TileKey& key,
                                   std::span<const std::uint8_t> body, std::uint32_t version = 1) {
    g2m::TileHeader h;
    h.layer = layer;
    h.layer_version = version;
    h.key = key;
    h.body_hash = g2m::content_hash(body);
    h.body_size = body.size();
    return h;
}

inline std::vector<std::uint8_t> read_file(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    return std::vector<std::uint8_t>(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
}

inline void write_file(const std::filesystem::path& p, const std::vector<std::uint8_t>& bytes) {
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
}

inline std::filesystem::path blob_file(const std::filesystem::path& dir, const std::string& name,
                                       const g2m::ContentHash& h) {
    const std::string hex = g2m::to_hex(h);
    return dir / (name + "_blobs") / hex.substr(0, 2) / hex.substr(2, 2) / hex;
}

} // namespace g2m_test
