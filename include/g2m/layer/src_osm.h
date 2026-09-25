// g2m/layer/src_osm.h - body encoding 3 (SrcOsm), version 1: the columnar
// OSM tile of g2m.src.osm (docs/formats/src_osm.md).
//
// Lossless for canonical OsmData (osm_data.h): decode(encode(d)) == d.
// The decoder validates the full canonical form, so a decoded tile always
// satisfies OsmData::validate() - in particular every way ref resolves to a
// node of the same tile.
#pragma once

#include "g2m/core/result.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/osm_data.h"
#include "g2m/layer/tile_container.h"

namespace g2m {

inline constexpr std::uint16_t kSrcOsmEncodingVersion = 1;

struct OsmTile {
    std::uint32_t halo_m = 0; // the clip halo the tile was cut with (osm_clip.h)
    osm::OsmData data;

    friend bool operator==(const OsmTile&, const OsmTile&) = default;
};

Result<TileBody> encode_src_osm(const OsmTile& tile);
Result<OsmTile> decode_src_osm(const TileBody& body);

namespace src_osm {
inline constexpr std::uint16_t kMeta = 1;
inline constexpr std::uint16_t kStrings = 2;
inline constexpr std::uint16_t kNodeIds = 3;
inline constexpr std::uint16_t kNodeLat = 4;
inline constexpr std::uint16_t kNodeLon = 5;
inline constexpr std::uint16_t kNodeTags = 6;
inline constexpr std::uint16_t kWayIds = 7;
inline constexpr std::uint16_t kWayRefs = 8;
inline constexpr std::uint16_t kWayTags = 9;
inline constexpr std::uint16_t kRelationIds = 10;
inline constexpr std::uint16_t kRelationMembers = 11;
inline constexpr std::uint16_t kRelationTags = 12;
} // namespace src_osm

} // namespace g2m
