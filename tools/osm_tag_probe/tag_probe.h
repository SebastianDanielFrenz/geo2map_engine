// tools/osm_tag_probe/tag_probe.h - OSM tag-coverage aggregation and report
// writers for the street-level fidelity plan's phase P0 (out/street_fidelity_plan.md
// section 6). Pure in-memory logic: no file IO, no PBF, no geometry. The PBF
// scan that feeds it lives in scan.h.
//
// Determinism: every container that reaches an output is a std::map or a
// std::vector in a fixed order; every percentage and length is formatted with
// integer arithmetic only; the reports carry no timestamp, host or timing.
// The same Accumulator contents therefore always produce the same bytes.
//
// "Present" for a tag key means: the key exists and its value is non-empty
// (so sidewalk=no counts as mapped - this probe measures how much the data
// SAYS, not how much of it is positive).
#pragma once

#include "g2m/decode/osm/pbf_reader.h"

#include <array>
#include <cstdint>
#include <map>
#include <span>
#include <string>
#include <string_view>
#include <vector>

namespace g2m::tag_probe {

using Tag = g2m::decode::osm::Tag;
using TagSpan = std::span<const Tag>;

// First value for `key`, nullptr if absent. Empty values count as absent.
const std::string* find_tag(TagSpan tags, std::string_view key);

// What one feature group counts: features, their summed way length (mm) and
// their summed polygon area (m^2). Unused dimensions stay 0.
struct Counts {
    std::uint64_t n = 0;
    std::int64_t length_mm = 0;
    std::int64_t area_m2 = 0;

    void add(std::int64_t len_mm, std::int64_t area) {
        ++n;
        length_mm += len_mm;
        area_m2 += area;
    }
    friend bool operator==(const Counts&, const Counts&) = default;
};

// A feature group: its total plus, per tracked metric, how many of them carry it.
struct Group {
    Counts all;
    std::map<std::string, Counts> with;
    friend bool operator==(const Group&, const Group&) = default;
};

// Node kinds a node can have (bit mask; one node may have several).
enum NodeKind : std::uint8_t {
    kNodeCrossing = 1,      // highway=crossing
    kNodeSignals = 2,       // highway=traffic_signals
    kNodeStop = 4,          // highway=stop
    kNodeGiveWay = 8,       // highway=give_way
    kNodeCalming = 16,      // traffic_calming=*
    kNodeKerb = 32,         // kerb=*
    kNodeTree = 64,         // natural=tree
};
// Kinds whose position relative to the road network is of interest (everything but trees).
inline constexpr std::uint8_t kNodeKindsWithPlacement = 1 | 2 | 4 | 8 | 16 | 32;

// Where a tagged node sits relative to carriageway ways (from the node's degree:
// an interior occurrence in a way counts 2, a way end 1).
enum PlacementClass : int {
    kPlaceOffRoad = 0,   // no carriageway way uses the node
    kPlaceDeadEnd = 1,   // degree 1
    kPlaceMidWay = 2,    // degree 2: on a road, not a junction (includes a way split)
    kPlaceJunction = 3,  // degree >= 3
    kPlaceClassCount = 4,
};

// Counters the scan itself keeps (they are part of the report: they say how
// much of the input the numbers rest on).
struct ScanStats {
    std::uint64_t nodes_total = 0;            // nodes in the PBF
    std::uint64_t tagged_nodes_in_region = 0;    // nodes with a tag of interest, region test applied
    std::uint64_t ways_total = 0;
    std::uint64_t ways_relevant = 0;          // ways with a tag of interest (any region)
    std::uint64_t ways_in_region = 0;         // relevant ways whose anchor lies in the region
    std::uint64_t ways_unresolved_anchor = 0; // relevant ways whose anchor node is not in the PBF
    std::uint64_t ways_missing_nodes = 0;     // in-region ways with a node ref absent from the PBF
    std::uint64_t relations_total = 0;
    std::uint64_t relations_relevant = 0;     // multipolygons with a building/forest tag
    std::uint64_t relations_in_region = 0;
    std::uint64_t relations_lenient = 0;      // in-region relations the exact ring assembly rejected (areas from lenient rings)
    std::uint64_t relations_zero_area = 0;    // in-region relations counted with area 0 (no closed ring found)
    std::uint64_t relations_no_geometry = 0;  // no node position at all, so no region test possible: skipped
    friend bool operator==(const ScanStats&, const ScanStats&) = default;
};

struct ReportPart {
    std::int64_t id = 0;
    std::string name;
    int admin_level = 0; // 0 = unknown
    std::string geometry_hash_hex;
};

// Provenance written into both reports. Nothing here may vary between runs on
// identical inputs.
struct ReportMeta {
    std::string region_name;
    std::string pbf_name;
    std::string pbf_sha256_hex;
    std::vector<ReportPart> parts;
    ScanStats scan;
};

class Accumulator {
public:
    // The highway classes reported per class, in report order (the 8 requested
    // classes in the order given, with each _link directly after its parent).
    static const std::vector<std::string>& road_class_names();
    // Index into road_class_names() for a way's highway=* value, -1 if not one of them.
    static int road_class_index(TagSpan tags);
    // highway=* value that is a motor-vehicle carriageway for node-degree purposes
    // (the reported classes plus living_street).
    static bool is_carriageway(TagSpan tags);

    // Which NodeKind bits a node's tags set (0 = uninteresting node).
    static std::uint8_t node_kind_mask(TagSpan tags);
    // Does a way carry any tag this probe aggregates (road class, calming, kerb,
    // building, tree row, forest, footway=crossing)?
    static bool way_is_relevant(TagSpan tags);
    // Is a relation a multipolygon with a building / forest tag?
    static bool relation_is_relevant(TagSpan tags);

    // `mask` = node_kind_mask(tags), non-zero.
    void add_node(std::uint8_t mask, TagSpan tags);
    // Once the way pass has measured how a recorded node is used.
    void add_node_degree(std::uint8_t mask, unsigned degree);

    // One way inside the region. `length_mm`: summed segment length; `area_m2`:
    // polygon area when `closed`, else 0.
    void add_way(TagSpan tags, std::int64_t length_mm, std::int64_t area_m2, bool closed);
    // One multipolygon relation inside the region.
    void add_relation_polygon(TagSpan tags, std::int64_t area_m2);

    std::string to_markdown(const ReportMeta& meta) const;
    std::string to_json(const ReportMeta& meta) const;

    // Read access for tests.
    struct RoadClass {
        Counts all;
        std::map<std::string, Counts> with;
    };
    const RoadClass& road(std::string_view highway) const;
    const Group& node_group(std::string_view name) const;
    const Group& way_group(std::string_view name) const;
    // Tally entries sorted by count descending, then value ascending; at most `limit` (0 = all).
    std::vector<std::pair<std::string, std::uint64_t>> top_values(std::string_view tally, std::size_t limit) const;
    std::array<std::uint64_t, kPlaceClassCount> placement(std::string_view node_group) const;
    Counts building_relations() const { return building_relations_; }
    Counts forest_open_ways() const { return forest_open_ways_; }
    Counts building_open_ways() const { return building_open_ways_; }
    Counts forest_by_tag(std::string_view tag) const; // "natural=wood" | "landuse=forest"

private:
    void tally(std::string_view tally_name, std::string_view value);
    void add_building(TagSpan tags, std::int64_t area_m2, bool via_relation);
    void add_forest(TagSpan tags, std::int64_t area_m2);

    std::map<std::string, RoadClass> roads_;
    std::map<std::string, Group> node_groups_;
    std::map<std::string, Group> way_groups_;
    std::map<std::string, std::array<std::uint64_t, kPlaceClassCount>> placement_;
    std::map<std::string, std::map<std::string, std::uint64_t>> tallies_;
    std::map<std::string, Counts> forest_by_tag_;
    Counts building_relations_;
    Counts building_open_ways_;
    Counts forest_open_ways_;
};

// ---- formatting helpers (integer arithmetic only; exposed for tests) ----

// round(100 * num / den) as "I.FF" with `decimals` decimals (0..2); "0.00" when den == 0.
std::string format_pct(std::uint64_t num, std::uint64_t den, int decimals);
// Millimetres as kilometres with 3 decimals (rounded to the metre).
std::string format_km(std::int64_t mm);
// m^2 as hectares with 1 decimal.
std::string format_ha(std::int64_t m2);

} // namespace g2m::tag_probe
