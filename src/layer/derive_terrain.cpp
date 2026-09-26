// src/layer/derive_terrain.cpp - see g2m/layer/derive_terrain.h.

#include "g2m/layer/derive_terrain.h"

#include "decoded_elev_memo.h"
#include "g2m/core/hash.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/src_elev.h"

#include <utility>

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
    return config_hash_of("g2m.terrain.height/1 L0-passthrough-of-elev.base");
}

std::vector<InputTileRef> TerrainHeightDeriver::inputs_for(const TileKey& target) const {
    if (target.level != 0) return {}; // levels 1-6: PLAN.md G2.2, not implemented here
    return {InputTileRef{std::string(kElevBaseLayer), target}};
}

Result<DeriveResult> TerrainHeightDeriver::derive(const DeriveRequest& request) const {
    if (request.target.level != 0) {
        return make_error(ErrorCode::InvalidArgument,
                          "terrain.height: only level 0 is implemented (PLAN.md G2.2 adds levels 1-6)");
    }
    if (request.inputs.empty() || !request.inputs[0].header) {
        return make_error(ErrorCode::NotFound, "terrain.height: g2m.elev.base input missing");
    }
    // Byte-identical passthrough (so the store's content-hash dedup collapses
    // the two tiles' bodies into one, PLAN.md decision 9).
    DeriveResult out;
    out.body = request.inputs[0].body;
    out.datum = request.inputs[0].header->datum;
    out.height_offset = request.inputs[0].header->height_offset;
    return out;
}

} // namespace g2m
