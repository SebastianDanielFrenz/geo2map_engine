// g2m/layer/pack.h - tile packs: 16 x 16 tiles of one layer in one object
// (PLAN.md 3.4 "packs"/"hashes"; byte layout: docs/formats/pack.md).
//
// A pack at level L, pack coordinates (px, py), holds the tiles
//   x = 16 px + dx, y = 16 py + dy,  dx, dy in [0, 16)
// as entry i = 16 dy + dx (row-major, dy = 0 the southern row). The
// directory lists each tile's state and body hash; present tiles follow as
// complete g2m.tile containers, in entry order. The "hashes" endpoint
// returns the directory alone.
#pragma once

#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/tile_container.h"

#include <array>
#include <span>
#include <string>
#include <vector>

namespace g2m {

inline constexpr int kPackSide = 16;
inline constexpr int kPackTiles = kPackSide * kPackSide;

enum class PackTileState : std::uint8_t {
    NotFound = 0,     // outside coverage (definitive for the release)
    Present = 1,      // container included
    Unavailable = 2,  // could not be produced now (503: retry later)
    Degraded = 3,     // offline and not cached
};

struct PackEntry {
    PackTileState state = PackTileState::NotFound;
    ContentHash body_hash{}; // all zero unless Present
    friend bool operator==(const PackEntry&, const PackEntry&) = default;
};

struct PackDirectory {
    std::string layer;
    ReleaseId rid{};
    TileKey pack;                             // level = tile level; x, y = pack coordinates
    std::array<PackEntry, kPackTiles> entries{};
    friend bool operator==(const PackDirectory&, const PackDirectory&) = default;
};

// The pack containing a tile, and the tile of entry i of a pack.
TileKey pack_of(const TileKey& tile);
TileKey pack_tile(const TileKey& pack, int entry);
int pack_entry_index(const TileKey& tile); // 16 dy + dx within its pack

// containers[i] is the complete container of entry i when Present, empty
// otherwise. Fails if a present container does not parse, or does not
// match its entry (layer, rid, key, body hash).
Result<std::vector<std::uint8_t>> encode_pack(const PackDirectory& dir,
                                              const std::array<std::vector<std::uint8_t>, kPackTiles>& containers);

struct PackView {
    PackDirectory dir;
    std::array<std::span<const std::uint8_t>, kPackTiles> containers{}; // into the decoded bytes
};
// Verifies every present container (header checksum, body hash, and that
// it is the tile its entry says).
Result<PackView> decode_pack(std::span<const std::uint8_t> bytes);

} // namespace g2m
