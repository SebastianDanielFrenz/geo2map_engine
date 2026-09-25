// src/core/tile_key.cpp - see g2m/core/tile_key.h.

#include "g2m/core/tile_key.h"

#include <cmath>

namespace g2m {
namespace {

constexpr std::int64_t kBias = std::int64_t{1} << (TileKey::kIndexBits - 1);
constexpr std::uint64_t kIndexMask = (std::uint64_t{1} << TileKey::kIndexBits) - 1;

bool index_in_range(std::int64_t v) { return v >= TileKey::kIndexMin && v <= TileKey::kIndexMax; }

} // namespace

bool TileKey::valid() const {
    return zone.valid() && level >= 0 && level <= kMaxLevel && index_in_range(x) && index_in_range(y);
}

TileKey TileKey::parent() const {
    TileKey p = *this;
    p.level = level + 1;
    p.x = x >> 1; // arithmetic shift == floor division by 2 (C++20)
    p.y = y >> 1;
    return p;
}

TileKey TileKey::child(int dx, int dy) const {
    TileKey c = *this;
    c.level = level - 1;
    c.x = x * 2 + (dx != 0 ? 1 : 0);
    c.y = y * 2 + (dy != 0 ? 1 : 0);
    return c;
}

std::uint64_t TileKey::packed() const {
    const auto zone_bits = static_cast<std::uint64_t>(zone.zone) & 0x3f;
    const auto hemi_bit = zone.hemisphere == geo::Hemisphere::South ? std::uint64_t{1} : std::uint64_t{0};
    const auto level_bits = static_cast<std::uint64_t>(level) & 0x1f;
    const auto ybits = static_cast<std::uint64_t>(static_cast<std::int64_t>(y) + kBias) & kIndexMask;
    const auto xbits = static_cast<std::uint64_t>(static_cast<std::int64_t>(x) + kBias) & kIndexMask;
    return (zone_bits << 54) | (hemi_bit << 53) | (level_bits << 48) | (ybits << 24) | xbits;
}

std::optional<TileKey> TileKey::unpack(std::uint64_t packed) {
    if ((packed >> 60) != 0) return std::nullopt;
    TileKey k;
    k.zone.zone = static_cast<int>((packed >> 54) & 0x3f);
    k.zone.hemisphere = ((packed >> 53) & 1) != 0 ? geo::Hemisphere::South : geo::Hemisphere::North;
    k.level = static_cast<int>((packed >> 48) & 0x1f);
    k.y = static_cast<std::int32_t>(static_cast<std::int64_t>((packed >> 24) & kIndexMask) - kBias);
    k.x = static_cast<std::int32_t>(static_cast<std::int64_t>(packed & kIndexMask) - kBias);
    if (!k.valid()) return std::nullopt;
    return k;
}

std::optional<TileKey> tile_key_at(geo::UtmZone zone, int level, double easting, double northing) {
    if (!zone.valid() || level < 0 || level > TileKey::kMaxLevel) return std::nullopt;
    if (!std::isfinite(easting) || !std::isfinite(northing)) return std::nullopt;
    // Keep the int64 conversion well-defined: |coordinate| < 2^52 m.
    constexpr double kLimit = 4503599627370496.0;
    if (std::fabs(easting) >= kLimit || std::fabs(northing) >= kLimit) return std::nullopt;
    const std::int64_t size = TileKey::kLevel0SizeM << level;
    const auto ie = static_cast<std::int64_t>(std::floor(easting));
    const auto in = static_cast<std::int64_t>(std::floor(northing));
    const std::int64_t tx = floor_div(ie, size);
    const std::int64_t ty = floor_div(in, size);
    if (!index_in_range(tx) || !index_in_range(ty)) return std::nullopt;
    return TileKey{zone, level, static_cast<std::int32_t>(tx), static_cast<std::int32_t>(ty)};
}

std::string to_string(const TileKey& key) {
    return geo::utm_scheme_name(key.zone) + "/" + std::to_string(key.level) + "/" + std::to_string(key.x) + "/" +
           std::to_string(key.y);
}

namespace {

// A canonical decimal int64 with magnitude <= 2^31 (enough for level and
// the 24-bit indices; range is checked by valid()).
std::optional<std::int64_t> parse_canonical_int(std::string_view s) {
    bool neg = false;
    if (!s.empty() && s[0] == '-') {
        neg = true;
        s.remove_prefix(1);
    }
    if (s.empty() || s.size() > 10 || (s[0] == '0' && (s.size() > 1 || neg))) return std::nullopt;
    std::int64_t v = 0;
    for (char c : s) {
        if (c < '0' || c > '9') return std::nullopt;
        v = v * 10 + (c - '0');
    }
    return neg ? -v : v;
}

} // namespace

std::optional<TileKey> parse_tile_path(std::string_view path) {
    std::string_view part[4];
    for (int i = 0; i < 4; ++i) {
        const std::size_t slash = path.find('/');
        if ((i < 3) != (slash != std::string_view::npos)) return std::nullopt;
        part[i] = path.substr(0, slash);
        path = i < 3 ? path.substr(slash + 1) : std::string_view();
    }
    const auto zone = geo::parse_utm_scheme(part[0]);
    const auto level = parse_canonical_int(part[1]);
    const auto x = parse_canonical_int(part[2]);
    const auto y = parse_canonical_int(part[3]);
    if (!zone || !level || !x || !y) return std::nullopt;
    if (*level < 0 || *level > TileKey::kMaxLevel || *x < TileKey::kIndexMin || *x > TileKey::kIndexMax ||
        *y < TileKey::kIndexMin || *y > TileKey::kIndexMax) {
        return std::nullopt;
    }
    TileKey k{*zone, static_cast<int>(*level), static_cast<std::int32_t>(*x), static_cast<std::int32_t>(*y)};
    if (!k.valid()) return std::nullopt;
    return k;
}

} // namespace g2m
