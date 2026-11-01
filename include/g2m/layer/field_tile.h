// g2m.fields.geom: farm field polygons per level-2 tile (docs/formats/fields_geom.md, docs/pipeline/fields.md).
//
// The library exposes FACTS: a polygon with its OSM origin, landuse class, the crop=* tag, area, a row direction
// and flags that say how the outline came about. What a field MEANS (soil, crop state, ownership) is the game's.
//
// A field is a piece of an OSM "block" (landuse=farmland/meadow/orchard/vineyard/farmyard way or multipolygon
// relation). A block is cut along tracks, paths, roads, railways, ditches, streams, hedges, walls and tree rows
// and around water, buildings and other non-agricultural areas (data/field_rules/de.json); a block nothing cuts
// keeps its exact OSM outline. Each record is ONE outer ring plus its holes, in integer UTM millimetres of the
// tile's zone.
//
// Tile contents: every field whose outline touches the tile square (or contains its south-west corner), stored in
// each such tile; FieldFlag::Owned marks the one tile that holds the field's anchor point, so counting owned
// records counts each field once. A point query (field_at) therefore needs only the tile holding the point.
//
// Ids (FieldId) are stable across re-bakes as far as OSM allows: source kind, OSM id and a deterministic piece
// index. See the "what breaks an id" section of docs/pipeline/fields.md.
#pragma once

#include "g2m/core/geom/exact.h"
#include "g2m/core/geom/ring_set.h"
#include "g2m/core/result.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/deriver.h"
#include "g2m/layer/field_rules.h"
#include "g2m/layer/tile_container.h"

#include <cstdint>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {
inline constexpr std::string_view kFieldsGeomLayer = "g2m.fields.geom";
inline constexpr std::uint16_t kFieldsGeomEncoding = 10; // LayerEncoding::FieldsGeom
inline constexpr std::size_t kFieldsMaxPerTile = 200000, kFieldsMaxVertices = 8000000, kFieldsMaxCrop = 255;

// Where a field's outline comes from. Parcel is RESERVED for an external open parcel layer (INVEKOS and
// similar, docs/pipeline/fields.md "Parcel data seam"): nothing produces it yet, tiles that carry it decode.
enum class FieldSource : std::uint8_t { OsmWay = 0, OsmRelation = 1, Parcel = 2 };

// FieldRecord::flags
namespace field_flag {
inline constexpr std::uint32_t Owned = 1u << 0;            // the field's anchor lies in this tile
inline constexpr std::uint32_t Subdivided = 1u << 1;       // outline derived by the raster subdivision (else the exact OSM outline)
inline constexpr std::uint32_t ObstaclesIgnored = 1u << 2; // obstacle areas covered more of the block than the cap allows and were not cut out
inline constexpr std::uint32_t SliverBlock = 1u << 3;      // every piece was a sliver: the block is kept whole
inline constexpr std::uint32_t SimplifyFallback = 1u << 4; // outline simplification would have crossed itself; a finer tolerance was used
inline constexpr std::uint32_t SmallArea = 1u << 5;        // plausibility flags (report only; never a reason to drop)
inline constexpr std::uint32_t LargeArea = 1u << 6;
inline constexpr std::uint32_t Thin = 1u << 7;
inline constexpr std::uint32_t All = (1u << 8) - 1;
} // namespace field_flag

struct FieldPolygon {
    geom::Ring outer;               // counter-clockwise, starts at its lexicographically smallest vertex
    std::vector<geom::Ring> holes;  // clockwise, each starts at its smallest vertex; ascending by first vertex
    friend bool operator==(const FieldPolygon&, const FieldPolygon&) = default;
};

struct FieldRecord {
    std::uint64_t id = 0;           // pack_field_id(source, osm_id, piece)
    FieldSource source = FieldSource::OsmWay;
    std::int64_t osm_id = 0;
    std::uint16_t piece = 0;        // 0-based index among the block's pieces (deterministic order, see docs)
    std::uint8_t landuse = 0;       // FieldRules::landuse code
    std::string crop;               // the crop=* tag of the block, empty when absent
    std::uint32_t flags = 0;
    std::uint64_t area_mm2 = 0;     // exact: outer area minus the holes
    std::uint16_t row_dir_cdeg = 0; // direction of the outer ring's longest edge, hundredths of a degree in [0, 18000), counter-clockwise from grid east
    std::uint32_t row_edge_mm = 0;  // that edge's length
    geom::PointMm anchor;           // a point inside the field (deterministic); ownership and saved-game matching use it
    FieldPolygon polygon;
    geom::BoxMm bbox;               // of the outer ring; computed, not stored
    double area_m2() const { return static_cast<double>(area_mm2) / 1e6; }
    friend bool operator==(const FieldRecord&, const FieldRecord&) = default;
};

// Why a block produced no field. Stored so a game can tell "no farmland here" from "farmland we could not read".
enum class FieldDeclineReason : std::uint8_t {
    InvalidGeometry = 0,   // rings do not assemble into simple polygons (or a node is outside the projection)
    RelationIncomplete = 1,// member ways missing after the discovery stages
    TooLarge = 2,          // bounding box above rules.block_max_extent_m
    TooManyCells = 3,      // raster budget (rules.max_cells) exceeded
    TooManyTiles = 4,      // needs more level-2 tiles than rules.block_max_cover_tiles
};
inline constexpr std::uint8_t kFieldDeclineCount = 5;
struct FieldDeclineRecord {
    FieldSource source = FieldSource::OsmWay;
    std::int64_t osm_id = 0;
    FieldDeclineReason reason = FieldDeclineReason::InvalidGeometry;
    friend bool operator==(const FieldDeclineRecord&, const FieldDeclineRecord&) = default;
};

// Counts over the blocks the tile OWNS (the block's smallest outline vertex lies in the tile), so summing over tiles
// counts every block once.
struct FieldTileCounts {
    std::uint32_t blocks = 0, subdivided = 0, slivers_dropped = 0, obstacles_ignored = 0, sliver_blocks = 0;
    friend bool operator==(const FieldTileCounts&, const FieldTileCounts&) = default;
};

struct FieldTile {
    TileKey key;
    std::vector<FieldRecord> fields;           // ascending id
    std::vector<FieldDeclineRecord> declines;  // ascending (source, osm_id); blocks that touch the tile
    FieldTileCounts counts;
    friend bool operator==(const FieldTile&, const FieldTile&) = default;
};

// ---- ids ---------------------------------------------------------------------------------------------------
// bits 63..62 source, 61..16 OSM id (46 bits), 15..0 piece.
inline constexpr std::int64_t kFieldMaxOsmId = (std::int64_t{1} << 46) - 1;
std::optional<std::uint64_t> pack_field_id(FieldSource source, std::int64_t osm_id, std::uint32_t piece);
struct FieldIdParts { FieldSource source; std::int64_t osm_id; std::uint16_t piece; };
std::optional<FieldIdParts> unpack_field_id(std::uint64_t id);

// ---- geometry helpers (exact integer arithmetic) -----------------------------------------------------------
std::int64_t ring_twice_area_mm2(const geom::Ring& ring); // signed, positive for counter-clockwise
std::uint64_t field_polygon_area_mm2(const FieldPolygon& polygon); // outer minus holes
geom::BoxMm ring_bbox(const geom::Ring& ring);
// Orients the ring (counter-clockwise or clockwise) and rotates it to start at its lexicographically smallest vertex.
void normalise_field_ring(geom::Ring& ring, bool counter_clockwise);
// Even-odd containment in one ring; a point ON an edge counts as inside.
bool field_ring_contains(const geom::Ring& ring, geom::PointMm p);
// Even-odd containment over the outer ring and the holes; a point ON any ring edge counts as inside.
bool field_polygon_contains(const FieldPolygon& polygon, geom::PointMm p);
// Direction (hundredths of a degree, [0, 18000)) and length of the outer ring's longest edge; the first of equal
// edges in ring order wins. A ring with fewer than 3 vertices gives 0/0.
void field_row_direction(const geom::Ring& outer, std::uint16_t& cdeg, std::uint32_t& edge_mm);
// A deterministic point inside the polygon: the midpoint of the longest interior run over a fixed set of scan lines.
geom::PointMm field_anchor(const FieldPolygon& polygon);
// Does the polygon touch the closed box (an outline segment touches it, or the box's south-west corner is inside)?
bool field_polygon_touches_box(const FieldPolygon& polygon, const geom::BoxMm& box);
// Fills area_mm2, row_dir_cdeg, row_edge_mm, anchor, bbox and the plausibility flags of `r` from r.polygon.
void finalize_field_record(FieldRecord& r, const FieldRules& rules);

// ---- tile codec --------------------------------------------------------------------------------------------
Result<TileBody> encode_fields_geom(const FieldTile& tile);
Result<FieldTile> decode_fields_geom(const TileBody& body);
// Every structural rule of the format: canonical order, ring orientation, ids match their parts, areas, row
// directions and anchors recompute, Owned == anchor in the tile, polygons touch the tile.
Result<void> validate_field_tile(const FieldTile& tile);

// ---- queries -----------------------------------------------------------------------------------------------
// The field containing `p` (boundary counts as inside); on overlap the lowest id. Index into tile.fields.
std::optional<std::size_t> field_at(const FieldTile& tile, geom::PointMm p);

// What a save stores about a field, to find it again after a re-bake.
struct FieldRef {
    FieldSource source = FieldSource::OsmWay;
    std::int64_t osm_id = 0;
    std::uint16_t piece = 0;
    std::uint8_t landuse = 0;
    geom::PointMm anchor;
};
enum class FieldMatchKind : std::uint8_t {
    None = 0,
    SameId = 1,   // the same id and the saved anchor is still inside it
    SameBlock = 2,// same OSM element, another piece index (or the outline moved): found by the anchor or the nearest anchor
    Moved = 3,    // another OSM element of the same landuse now covers the saved anchor (the way was redrawn or replaced)
};
struct FieldMatch {
    FieldMatchKind kind = FieldMatchKind::None;
    std::size_t index = 0; // into the candidate list
};
// Candidates = the fields of the tiles around the saved anchor (any order). Deterministic: ties go to the lowest id.
FieldMatch match_field(const FieldRef& saved, std::span<const FieldRecord> candidates, const FieldRules& rules);

// ---- the layer ---------------------------------------------------------------------------------------------
LayerDesc fields_geom_desc();

class FieldsGeomDeriver final : public IDeriver {
public:
    FieldsGeomDeriver() : desc_(fields_geom_desc()) {}
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override;
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override;
    Result<std::vector<InputTileRef>> additional_inputs_for(const DeriveRequest& request) const override;
    // Relation blocks are completed by walking member ways across tiles; a block's own bounding box then names the
    // tiles with its cutting features. Four stages cover a relation outline spanning about eight kilometres.
    std::size_t discovery_stages() const override { return IDeriver::kMaxDiscoveryStages; }
    Result<DeriveResult> derive(const DeriveRequest& request) const override;
private:
    LayerDesc desc_;
};
} // namespace g2m
