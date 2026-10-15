// src/layer/pack.cpp - see g2m/layer/pack.h and docs/formats/pack.md.

#include "g2m/layer/pack.h"

#include "g2m/core/bytes.h"
#include "g2m/layer/layer_id.h"

#include <algorithm>
#include <cstring>

namespace g2m {
namespace {

constexpr std::uint8_t kMagic[4] = {'G', '2', 'M', 'P'};
constexpr std::uint8_t kDirMagic[4] = {'G', '2', 'M', 'H'};
constexpr std::uint8_t kDirVersion = 1;
constexpr std::uint8_t kPackVersion = 1;
constexpr std::uint8_t kSchemeUtm = 1;

bool is_zero(const ContentHash& h) {
    return std::all_of(h.begin(), h.end(), [](std::uint8_t b) { return b == 0; });
}

Result<void> check_container(const PackDirectory& dir, int i, std::span<const std::uint8_t> bytes) {
    auto v = parse_container(bytes);
    if (!v) return v.error();
    const TileHeader& h = v->header;
    if (h.layer != dir.layer || h.rid != dir.rid || h.key != pack_tile(dir.pack, i) ||
        h.body_hash != dir.entries[static_cast<std::size_t>(i)].body_hash) {
        return make_error(ErrorCode::Malformed, "pack: container " + std::to_string(i) + " does not match its entry");
    }
    return {};
}

} // namespace

TileKey pack_of(const TileKey& tile) {
    TileKey p = tile;
    p.x = static_cast<std::int32_t>(floor_div(tile.x, kPackSide));
    p.y = static_cast<std::int32_t>(floor_div(tile.y, kPackSide));
    return p;
}

TileKey pack_tile(const TileKey& pack, int entry) {
    TileKey t = pack;
    t.x = pack.x * kPackSide + entry % kPackSide;
    t.y = pack.y * kPackSide + entry / kPackSide;
    return t;
}

int pack_entry_index(const TileKey& tile) {
    const TileKey p = pack_of(tile);
    return (tile.y - p.y * kPackSide) * kPackSide + (tile.x - p.x * kPackSide);
}

namespace {

// magic, version, reserved, layer, rid, scheme/zone/hemisphere/level, x, y
// (shared by the pack and the pack directory).
Result<void> write_pack_head(ByteWriter& w, const std::uint8_t (&magic)[4], std::uint8_t version, const PackDirectory& dir) {
    if (!is_valid_layer_name(dir.layer)) return make_error(ErrorCode::InvalidArgument, "pack: invalid layer name");
    // The pack's tiles must be valid keys too: check both corners.
    if (!dir.pack.valid() || !pack_tile(dir.pack, 0).valid() || !pack_tile(dir.pack, kPackTiles - 1).valid()) {
        return make_error(ErrorCode::InvalidArgument, "pack: invalid pack key");
    }
    w.bytes(magic, 4);
    w.u8(version);
    w.u8(0);
    w.u16(0);
    w.u8(static_cast<std::uint8_t>(dir.layer.size()));
    w.bytes(dir.layer.data(), dir.layer.size());
    w.bytes(dir.rid);
    w.u8(kSchemeUtm);
    w.u8(static_cast<std::uint8_t>(dir.pack.zone.zone));
    w.u8(static_cast<std::uint8_t>(dir.pack.zone.hemisphere));
    w.u8(static_cast<std::uint8_t>(dir.pack.level));
    w.i32(dir.pack.x);
    w.i32(dir.pack.y);
    return {};
}

Result<void> read_pack_head(ByteReader& r, const std::uint8_t (&magic)[4], std::uint8_t version, PackDirectory& d) {
    const auto m = r.bytes(4);
    const std::uint8_t v = r.u8();
    const std::uint8_t res8 = r.u8();
    const std::uint16_t res16 = r.u16();
    if (!r.ok()) return make_error(ErrorCode::Truncated, "pack: truncated");
    if (std::memcmp(m.data(), magic, 4) != 0) return make_error(ErrorCode::BadMagic, "not a g2m pack");
    if (v != version) return make_error(ErrorCode::Unsupported, "pack version");
    if (res8 != 0 || res16 != 0) return make_error(ErrorCode::Malformed, "pack: reserved fields");
    const std::uint8_t name_len = r.u8();
    d.layer = std::string(r.string(name_len));
    const auto rid = r.bytes(32);
    const std::uint8_t scheme = r.u8();
    const std::uint8_t zone = r.u8();
    const std::uint8_t hemi = r.u8();
    const std::uint8_t level = r.u8();
    d.pack.x = r.i32();
    d.pack.y = r.i32();
    if (!r.ok()) return make_error(ErrorCode::Truncated, "pack: truncated");
    std::copy(rid.begin(), rid.end(), d.rid.begin());
    if (!is_valid_layer_name(d.layer)) return make_error(ErrorCode::Malformed, "pack: layer name");
    if (scheme != kSchemeUtm || hemi > 1) return make_error(ErrorCode::Malformed, "pack: scheme");
    d.pack.zone = geo::UtmZone{zone, static_cast<geo::Hemisphere>(hemi)};
    d.pack.level = level;
    // Pack coordinates times 16 must stay in the tile index range.
    if (!d.pack.valid() || d.pack.x < TileKey::kIndexMin / kPackSide || d.pack.x > TileKey::kIndexMax / kPackSide ||
        d.pack.y < TileKey::kIndexMin / kPackSide || d.pack.y > TileKey::kIndexMax / kPackSide) {
        return make_error(ErrorCode::Malformed, "pack: key");
    }
    return {};
}

} // namespace

Result<std::vector<std::uint8_t>> encode_pack(const PackDirectory& dir,
                                              const std::array<std::vector<std::uint8_t>, kPackTiles>& containers) {
    ByteWriter w;
    if (auto ok = write_pack_head(w, kMagic, kPackVersion, dir); !ok) return ok.error();
    for (int i = 0; i < kPackTiles; ++i) {
        const PackEntry& e = dir.entries[static_cast<std::size_t>(i)];
        const auto& c = containers[static_cast<std::size_t>(i)];
        if (static_cast<std::uint8_t>(e.state) > 3) return make_error(ErrorCode::InvalidArgument, "pack: bad state");
        if (e.state == PackTileState::Present) {
            if (auto ok = check_container(dir, i, c); !ok) return ok.error();
        } else if (!c.empty() || !is_zero(e.body_hash)) {
            return make_error(ErrorCode::InvalidArgument, "pack: non-present entry with data or hash");
        }
        w.u8(static_cast<std::uint8_t>(e.state));
        w.bytes(e.body_hash);
        w.u64(c.size());
    }
    for (const auto& c : containers) w.bytes(c);
    return w.take();
}

Result<PackView> decode_pack(std::span<const std::uint8_t> bytes) {
    ByteReader r(bytes);
    PackView v;
    PackDirectory& d = v.dir;
    if (auto ok = read_pack_head(r, kMagic, kPackVersion, d); !ok) return ok.error();
    std::array<std::uint64_t, kPackTiles> sizes{};
    std::uint64_t total = 0;
    for (std::size_t i = 0; i < kPackTiles; ++i) {
        const std::uint8_t st = r.u8();
        const auto h = r.bytes(32);
        sizes[i] = r.u64();
        if (!r.ok()) return make_error(ErrorCode::Truncated, "pack: directory truncated");
        if (st > 3) return make_error(ErrorCode::Malformed, "pack: bad state");
        d.entries[i].state = static_cast<PackTileState>(st);
        std::copy(h.begin(), h.end(), d.entries[i].body_hash.begin());
        const bool present = d.entries[i].state == PackTileState::Present;
        if (!present && (sizes[i] != 0 || !is_zero(d.entries[i].body_hash))) {
            return make_error(ErrorCode::Malformed, "pack: non-present entry with data or hash");
        }
        if (present && sizes[i] == 0) return make_error(ErrorCode::Malformed, "pack: present entry without data");
        if (sizes[i] > r.remaining() - std::min<std::uint64_t>(total, r.remaining())) {
            return make_error(ErrorCode::Truncated, "pack: entry size exceeds the data");
        }
        total += sizes[i];
    }
    if (total != r.remaining()) return make_error(ErrorCode::Malformed, "pack: data size mismatch");
    for (std::size_t i = 0; i < kPackTiles; ++i) {
        const auto c = r.bytes(static_cast<std::size_t>(sizes[i]));
        if (d.entries[i].state != PackTileState::Present) continue;
        if (auto ok = check_container(d, static_cast<int>(i), c); !ok) return ok.error();
        v.containers[i] = c;
    }
    return v;
}

Result<std::vector<std::uint8_t>> encode_pack_directory(const PackDirectory& dir) {
    ByteWriter w;
    if (auto ok = write_pack_head(w, kDirMagic, kDirVersion, dir); !ok) return ok.error();
    for (const PackEntry& e : dir.entries) {
        if (static_cast<std::uint8_t>(e.state) > 3) return make_error(ErrorCode::InvalidArgument, "pack directory: bad state");
        if (e.state != PackTileState::Present && !is_zero(e.body_hash)) {
            return make_error(ErrorCode::InvalidArgument, "pack directory: non-present entry with a hash");
        }
        w.u8(static_cast<std::uint8_t>(e.state));
        w.bytes(e.body_hash);
    }
    return w.take();
}

Result<PackDirectory> decode_pack_directory(std::span<const std::uint8_t> bytes) {
    ByteReader r(bytes);
    PackDirectory d;
    if (auto ok = read_pack_head(r, kDirMagic, kDirVersion, d); !ok) return ok.error();
    for (PackEntry& e : d.entries) {
        const std::uint8_t st = r.u8();
        const auto h = r.bytes(32);
        if (!r.ok()) return make_error(ErrorCode::Truncated, "pack directory: truncated");
        if (st > 3) return make_error(ErrorCode::Malformed, "pack directory: bad state");
        e.state = static_cast<PackTileState>(st);
        std::copy(h.begin(), h.end(), e.body_hash.begin());
        if (e.state != PackTileState::Present && !is_zero(e.body_hash)) {
            return make_error(ErrorCode::Malformed, "pack directory: non-present entry with a hash");
        }
    }
    if (r.remaining() != 0) return make_error(ErrorCode::Malformed, "pack directory: trailing bytes");
    return d;
}

} // namespace g2m
