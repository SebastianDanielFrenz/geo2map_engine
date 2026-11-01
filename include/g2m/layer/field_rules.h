// Field polygons (farming game, vault G2M-0xx): the rules of g2m.fields.geom as DATA (data/field_rules/de.json).
// Which OSM tags make a block of agricultural land, which linear and area features cut it apart (and how wide
// the gap they leave is), the raster cell of the subdivision, the sliver rule and the report thresholds. Every
// number is an integer (millimetres, square metres, percent); nothing is a code literal of the algorithm.
// The JSON is embedded at build time and its SHA-256 is part of the layer's config hash, so editing the file
// changes the release id.
#pragma once
#include "g2m/core/result.h"
#include "g2m/layer/tile_container.h"
#include <cstddef>
#include <cstdint>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace g2m {
// A tag selector: the tag `key` has one of `values`; "*" matches any value except "no".
struct FieldTagMatch {
    std::string key;
    std::vector<std::string> values; // sorted
    friend bool operator==(const FieldTagMatch&, const FieldTagMatch&) = default;
};
struct FieldLanduseRule {
    std::string value;   // landuse=<value>
    std::uint8_t code = 0; // stored in the tile (append-only)
    friend bool operator==(const FieldLanduseRule&, const FieldLanduseRule&) = default;
};
struct FieldSplitterRule {
    std::string name;
    FieldTagMatch match;
    std::int32_t half_width_mm = 0; // cells within this distance of the line are taken out of the field
    friend bool operator==(const FieldSplitterRule&, const FieldSplitterRule&) = default;
};
struct FieldObstacleRule {
    std::string name;
    FieldTagMatch match; // closed ways and multipolygon relations carrying the tag are cut out of the field
    friend bool operator==(const FieldObstacleRule&, const FieldObstacleRule&) = default;
};
struct FieldRules {
    std::vector<FieldLanduseRule> landuse; // ascending value
    std::vector<FieldTagMatch> skip;       // a line carrying one of these (bridge, tunnel, layer < 0) never splits
    std::vector<FieldSplitterRule> splitters;
    std::vector<FieldObstacleRule> obstacles;
    std::int32_t cell_mm = 0;              // raster cell edge (even)
    std::int32_t max_cells = 0;            // raster cell budget per polygon
    std::int32_t block_max_extent_m = 0;   // a block with a larger bounding box is declined
    std::int32_t block_max_cover_tiles = 0;// a block that needs more level-2 tiles is declined
    std::int32_t obstacle_share_cap_percent = 0; // obstacles removing more than this share of a block are treated as mapping overlap and ignored
    std::int32_t single_piece_keep_percent = 0;  // one surviving piece of at least this share of the block keeps the original outline
    std::int32_t sliver_min_area_m2 = 0, sliver_min_width_mm = 0;
    std::int32_t simplify_tolerance_mm = 0;
    std::int32_t plausible_min_area_m2 = 0, plausible_max_area_m2 = 0, plausible_min_width_mm = 0;
    std::int32_t match_max_anchor_distance_m = 0;
    ContentHash content_sha256{};          // SHA-256 of the JSON bytes the rules were loaded from

    // Largest splitter half width (the margin around a block in which line features still matter).
    std::int32_t max_half_width_mm() const;
    // The landuse code for a `landuse` tag value; -1 when the value is not a field landuse.
    int landuse_code(std::string_view value) const;
    friend bool operator==(const FieldRules&, const FieldRules&) = default;
};

// Strict loader: format "g2m.field_rules/1", integers only, unknown or missing keys rejected.
Result<FieldRules> load_field_rules(std::span<const std::byte> json);
// The embedded data/field_rules/de.json, parsed once. A corrupt build aborts.
const FieldRules& builtin_field_rules_de();
std::span<const std::byte> builtin_field_rules_de_bytes();
bool field_tag_matches(const FieldTagMatch& match, std::string_view value);
} // namespace g2m
