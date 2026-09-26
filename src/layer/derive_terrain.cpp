// src/layer/derive_terrain.cpp - see g2m/layer/derive_terrain.h.

#include "g2m/layer/derive_terrain.h"

#include "decoded_elev_memo.h"
#include "g2m/core/hash.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/src_elev.h"

#include <array>
#include <cstddef>
#include <utility>
#include <vector>

namespace g2m {
namespace {

ContentHash config_hash_of(std::string_view s) { return Sha256::of(s); }

// The level-2 (1024 m) ancestor of `target` (its own key if already level 2
// or above - not expected here, but harmless).
TileKey ancestor_at(const TileKey& target, int level) {
    TileKey cur = target;
    while (cur.level < level) cur = cur.parent();
    return cur;
}

} // namespace

// ---- ElevBaseDeriver --------------------------------------------------------

ElevBaseDeriver::ElevBaseDeriver()
    : desc_(elev_base_desc()), memo_(std::make_unique<layer_detail::DecodedElevMemo>()) {}

ElevBaseDeriver::~ElevBaseDeriver() = default;

const LayerDesc& ElevBaseDeriver::output() const { return desc_; }

ContentHash ElevBaseDeriver::config_hash() const { return config_hash_of("g2m.elev.base/1 dgm1-only"); }

std::vector<InputTileRef> ElevBaseDeriver::inputs_for(const TileKey& target) const {
    return {InputTileRef{std::string(kSrcElevDgm1Layer), ancestor_at(target, kRawTileLevel)}};
}

Result<DeriveResult> ElevBaseDeriver::derive(const DeriveRequest& request) const {
    if (request.target.level != 0) {
        return make_error(ErrorCode::InvalidArgument, "elev.base: only level 0 is implemented");
    }

    HeightTile dst;
    dst.key = request.target;
    dst.h.fill(kHeightNoData);
    dst.has_nodata = true;

    if (!request.inputs.empty() && request.inputs[0].header) {
        const TileHeader& in_header = *request.inputs[0].header;
        Result<TileBody> in_body = decode_body(request.inputs[0].body);
        if (!in_body) return in_body.error();
        Result<std::shared_ptr<const ElevTile>> src = memo_->get(in_header.body_hash, *in_body);
        if (!src) return src.error();
        const ElevTile& elev = **src;

        const RasterGeoref dst_georef = height_tile_georef(dst.key);
        const std::optional<LatticeOffset> off = lattice_offset(elev.georef, dst_georef);
        if (!off || !off->row_flip) {
            return make_error(ErrorCode::Malformed,
                              "elev.base: g2m.src.elev.dgm1 tile does not align with the level-0 lattice "
                              "as SouthToNorth-vs-NorthToSouth (expected an exact lattice match with a row flip)");
        }

        bool any_nodata = false;
        for (int j = 0; j < kTerrainTileSamples; ++j) {
            const std::int64_t src_row = off->drow - j;
            for (int i = 0; i < kTerrainTileSamples; ++i) {
                const std::int64_t src_col = static_cast<std::int64_t>(i) + off->dcol;
                if (src_col < 0 || src_col >= elev.heights.width() || src_row < 0 || src_row >= elev.heights.height()) {
                    return make_error(ErrorCode::Malformed, "elev.base: source window falls outside the dgm1 tile");
                }
                const std::int32_t v = elev.heights.at(static_cast<int>(src_col), static_cast<int>(src_row));
                dst.h[static_cast<std::size_t>(j) * static_cast<std::size_t>(kTerrainTileSamples) +
                     static_cast<std::size_t>(i)] = v;
                any_nodata = any_nodata || (v == kHeightNoData);
            }
        }
        dst.has_nodata = any_nodata;
    }

    Result<TileBody> encoded = encode_height_tile(dst);
    if (!encoded) return encoded.error();
    Result<std::vector<std::uint8_t>> bytes = encode_body(*encoded);
    if (!bytes) return bytes.error();

    DeriveResult out;
    out.body = std::move(bytes).value();
    out.datum = VerticalDatum::DHHN2016;
    out.height_offset = 0;
    return out;
}

// ---- TerrainHeightDeriver ---------------------------------------------------

TerrainHeightDeriver::TerrainHeightDeriver() : desc_(terrain_height_desc()) {}

const LayerDesc& TerrainHeightDeriver::output() const { return desc_; }

ContentHash TerrainHeightDeriver::config_hash() const {
    // PLAN.md G2.2: levels 1-6 add the mean-of-4 half-even downsample
    // pyramid (height_tile.h's downsample_2x2()) over the deriver's own
    // 4 same-layer children. Deliberately changed from G2.1's
    // "L0-passthrough-of-elev.base" string - a config_hash change is part
    // of the release id, and this is a real behaviour change (L1-L6 tiles
    // that used to 404/error now derive), so downstream releases/golden
    // hashes are expected to move.
    return config_hash_of("g2m.terrain.height/2 L0-passthrough-of-elev.base; L1-L6 mean-of-4 half-even pyramid");
}

std::vector<InputTileRef> TerrainHeightDeriver::inputs_for(const TileKey& target) const {
    if (target.level == 0) return {InputTileRef{std::string(kElevBaseLayer), target}};
    if (target.level < 0 || target.level > kTerrainMaxLevel) return {};
    // SW, SE, NW, NE - already ascending TileKey order (operator<=> compares
    // y before x), satisfying inputs_for()'s "sorted, unique" contract
    // without an extra sort.
    return {
        InputTileRef{std::string(kTerrainHeightLayer), target.child(0, 0)},
        InputTileRef{std::string(kTerrainHeightLayer), target.child(1, 0)},
        InputTileRef{std::string(kTerrainHeightLayer), target.child(0, 1)},
        InputTileRef{std::string(kTerrainHeightLayer), target.child(1, 1)},
    };
}

Result<DeriveResult> TerrainHeightDeriver::derive(const DeriveRequest& request) const {
    const int level = request.target.level;
    if (level < 0 || level > kTerrainMaxLevel) {
        return make_error(ErrorCode::InvalidArgument, "terrain.height: level out of range [0, kTerrainMaxLevel]");
    }
    if (level == 0) {
        if (request.inputs.empty() || !request.inputs[0].header) {
            return make_error(ErrorCode::NotFound, "terrain.height: g2m.elev.base input missing");
        }
        // Byte-identical passthrough (so the store's content-hash dedup
        // collapses the two tiles' bodies into one, PLAN.md decision 9).
        DeriveResult out;
        out.body = request.inputs[0].body;
        out.datum = request.inputs[0].header->datum;
        out.height_offset = request.inputs[0].header->height_offset;
        return out;
    }

    // Levels 1-6: mean-of-4 half-even downsample of the 4 same-layer
    // children in inputs_for()'s SW, SE, NW, NE order. A missing child
    // (outside coverage - request.inputs[i].header is nullopt) counts as
    // all-NoData for its whole quadrant (downsample_2x2's null-child rule),
    // never an error: Coverage::covers() (release.h) is what turns "no L2
    // tile anywhere under this key at all" into a 404 before derive() is
    // ever called for it.
    if (request.inputs.size() != 4) {
        return make_error(ErrorCode::InvalidArgument, "terrain.height: expected 4 child inputs (SW, SE, NW, NE)");
    }
    // Heap-allocated, not a stack array: TerrainHeightDeriver::derive() can
    // be invoked at recursion depth up to kTerrainMaxLevel (Server::resolve
    // recursing through this same-layer chain, server.cpp), and 4 *
    // sizeof(HeightTile) (~1 MiB, each tile's 256x256 int32 grid is 256 KiB)
    // is enough on its own to blow a default 1 MiB thread stack even before
    // any enclosing frame's own locals are counted - found the hard way
    // (STATUS_STACK_OVERFLOW) via a depth-7 (L6) resolve in
    // tests/layer/test_derive_terrain.cpp.
    std::vector<HeightTile> children(4);
    std::array<const HeightTile*, 4> child_ptrs{nullptr, nullptr, nullptr, nullptr};
    for (std::size_t i = 0; i < 4; ++i) {
        const InputTile& in = request.inputs[i];
        if (!in.header) continue;
        Result<TileBody> body = decode_body(in.body);
        if (!body) return body.error();
        Result<void> decoded = decode_height_tile(*body, in.ref.key, children[i]);
        if (!decoded) return decoded.error();
        child_ptrs[i] = &children[i];
    }

    HeightTile out_tile;
    out_tile.key = request.target;
    downsample_2x2(child_ptrs[0], child_ptrs[1], child_ptrs[2], child_ptrs[3], out_tile);

    Result<TileBody> encoded = encode_height_tile(out_tile);
    if (!encoded) return encoded.error();
    Result<std::vector<std::uint8_t>> bytes = encode_body(*encoded);
    if (!bytes) return bytes.error();

    DeriveResult out;
    out.body = std::move(bytes).value();
    out.datum = VerticalDatum::DHHN2016;
    out.height_offset = 0;
    return out;
}

} // namespace g2m
