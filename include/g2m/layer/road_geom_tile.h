#pragma once
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_profile.h"
#include <optional>
namespace g2m {
inline constexpr std::string_view kRoadGeomLayer = "g2m.roads.geom";
inline constexpr std::uint16_t kRoadGeomEncoding = 6;
// The previous body format (profiles only, no cross-sections); still decoded, never written.
inline constexpr std::uint16_t kRoadGeomEncodingLegacy = 5;
struct RoadGeomEntry {
    std::int64_t way_id = 0;
    RoadStretch stretch;
    std::optional<RoadProfile> profile;
    std::optional<Error> decline;
    // Encoding 6. Exactly one of the two is set for an entry with a profile, neither for a declined
    // entry (no profile means no reference to place a cross-section on) and neither for an encoding-5 tile.
    std::optional<RoadCrossSection> cross_section;
    std::optional<CrossSectionDecline> cross_section_decline;
};
// A traffic-calming feature at a shape node inside a stretch (design 8.3, owner decision Q2: every feature that has
// a height is carved, the lateral kinds render only; ProfileFeatureKind is declared in road_rules.h).
// Lateral extent: width_mm 0 = across the whole carriageway (carriageway_edges), else that width centred on it;
// a cushion is carved across the lane group because a partial width cannot be represented on the 1 m grid.
// height_mm is a height above the profile, except for a Dip where it is the depth below it.
struct ProfileFeature {
    std::size_t entry_index = 0;     // into RoadGeomTile::entries; the entry has a cross-section
    std::int64_t node_id = 0;
    double station_m = 0;            // along the entry's reference line
    ProfileFeatureKind kind = ProfileFeatureKind::Bump;
    bool carve = false;              // true: part of the carved height; false: render only
    std::int32_t length_mm = 0, height_mm = 0, width_mm = 0; // width 0 = full carriageway
    Provenance provenance = Provenance::Inferred;
    friend bool operator==(const ProfileFeature&, const ProfileFeature&) = default;
};
// Where an entry's profile came from (design 6.6). Strict is build_road_profile at the policy limits; the others are
// the completion tier and exist only in a tile built with RoadCompletionOptions::enabled.
enum class ProfileProvenance : std::uint8_t {
    Strict = 0,              // build_road_profile at the policy limits (also a still-declined entry)
    Retry = 1,               // 20 m samples, 16384 iterations, same limits (game parity)
    BridgeCompleted = 2,     // span re-fitted between abutments at relaxed limits
    GroundCompleted = 3,     // relaxed ground fit of a stretch the strict pass declined
};
struct RoadProfileOrigin {
    ProfileProvenance provenance = ProfileProvenance::Strict;
    std::uint16_t strict_decline_code = 0;  // the ErrorCode of the strict decline that was replaced (0 for Strict)
    friend bool operator==(const RoadProfileOrigin&, const RoadProfileOrigin&) = default;
};
// The completion tier of design 6.6 (road_completion.h). Off by default: a tile built without it is byte-for-byte
// what encoding 6 stores, and encode_road_geom refuses a tile that holds a completed profile (encoding 6 cannot
// say so; encoding 7 will, slice 8).
struct RoadCompletionOptions {
    bool enabled = false;
    double retry_spacing_m = 20;          // step 1: the game's retry
    int retry_iterations = 16384;
    double abutment_ground_m = 32;        // ground path to a support point (game: 32 m)
    double abutment_path_m = 60;          // total walk, bridge edges included (design D25: DEM and graph locality)
    double ground_spacing_m = 40;         // step 3 samples
    double ground_ramp_m = 32;            // step 3 anchor ramps
};
// What the completion tier did, per kind (design 6.6; diagnostics only, never stored).
struct RoadCompletionStats {
    std::size_t retry_candidates = 0, retry_accepted = 0;
    std::size_t bridge_candidates = 0, bridge_completed = 0, bridge_no_support = 0, bridge_fit_failed = 0;
    std::size_t ground_candidates = 0, ground_completed = 0, ground_failed = 0;
    std::size_t ground_bridge_anchored = 0;  // completed stretches with at least one end on a bridge abutment plane
    std::size_t tunnel_skipped = 0;
};
// Every graph stretch has exactly one accepted profile or explicit decline.
// Consumers must inspect declines; this is never an implicitly complete mesh.
struct RoadGeomTile {
    RoadGraphTile source;
    std::vector<RoadGeomEntry> entries;
    std::vector<ProfileFeature> features; // sorted by (entry_index, station bits, node_id)
    // 6 for everything built or encoded; 5 for a decoded legacy body, which has no cross-sections or
    // features (consumers fall back to the symmetric RoadAttributes::width_mm path).
    std::uint16_t encoding = kRoadGeomEncoding;
    // One per entry when the tile was built with the completion tier enabled, else empty (design 6.6, section 7 of
    // encoding 7). Not stored by encoding 6.
    std::vector<RoadProfileOrigin> origins;
};
// Feature shape (owner decision Q2: representable on the 1 m grid). Every carved feature is a plateau with two C1
// smooth-step ramps (road_smooth_step), at least kFeatureMinRampMm per side, so the narrowest carved feature is 2 m
// long. Bump, MiniBumps, RumbleStrip, Cushion, Dip and Hump have no plateau: ramp = half the length. A Table ramps
// at 1:kFeatureTableRampRatio (ramp = 10 x height), clamped to [kFeatureMinRampMm, length / 2]. `length_mm` stays
// the nominal length; the carved length is feature_effective_length_m.
inline constexpr std::int32_t kFeatureMinRampMm = 1000;
inline constexpr std::int32_t kFeatureTableRampRatio = 10;
double feature_ramp_m(const ProfileFeature& f) noexcept;             // per side
double feature_effective_length_m(const ProfileFeature& f) noexcept; // max(length, 2 x ramp)
// The carved height of one feature at station s of its entry: 0 for a render-only feature, a plateau of height h
// with smooth-step ramps otherwise, negative for a Dip. Pure, + - * / only.
double feature_height_contribution_m(const ProfileFeature& f, double station_m) noexcept;

// `context` (optional) is a canonical graph holding every way incident to the
// nodes of graph.graph's ways (merge_road_graphs over the tiles named by
// road_context_dependencies). Stretch extents, join headings and profiles then
// depend on the world, not on the tile. Without it the tile graph itself is the
// context, which is only exact for a graph that is complete at its own nodes.
Result<RoadGeomTile> build_road_geom_tile(const RoadGraphTile& graph,
    const RoadDemSampler& dem, const RoadProfileLimits& limits = {}, const RoadGraph* context = nullptr,
    const RoadRules& rules = builtin_road_rules_de(), const RoadCompletionOptions& completion = {},
    RoadCompletionStats* completion_stats = nullptr);
// Sum of every carved feature of `entry` at station s (metres, 0 without one). Features are sorted by entry, so this
// is a binary search plus the entry's own features. A consumer adds it to the vertical profile and the crown:
// z = profile(s) + crown(t) + feature_height_m(tile, entry, s), over the feature's lateral extent.
double feature_height_m(const RoadGeomTile& tile, std::size_t entry, double s) noexcept;
// The features of one entry from the node tags of its way (build_road_geom_tile calls this). Every INTERIOR node of
// the stretch (end nodes belong to the junction, P2) with a recognised `traffic_calming` value (bump, mini_bumps,
// rumble_strip, cushion, hump, table, chicane, choker, island, painted_island as Island, dip; anything else, `yes`
// included, is ignored). Length: a clean `traffic_calming:length` in metres (0 < L <= 50, Tagged) else the rules
// default (Inferred). A feature whose carved length would cross a stretch end is shortened to fit (Derived); one
// with less than 1 m to either end (render-only kinds: 0.5 m) is dropped. The station is that of the reference
// point nearest the node: the knot itself or the middle of the rounded corner arc. Sorted by (station, node id).
// `entry` needs its profile; an entry without a cross-section has no features.
Result<std::vector<ProfileFeature>> resolve_profile_features(const RoadGraph& graph, const RoadGeomEntry& entry,
    std::size_t entry_index, const RoadRules& rules);
Result<void> validate_road_geom_tile(const RoadGeomTile& tile);
Result<TileBody> encode_road_geom(const RoadGeomTile& tile);
Result<RoadGeomTile> decode_road_geom(const TileBody& body);
LayerDesc road_geom_desc();
class RoadGeomDeriver final : public IDeriver {
public:
    RoadGeomDeriver() : desc_(road_geom_desc()) {}
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override;
    std::span<const std::string> coverage_inputs() const override { return std::span<const std::string>(desc_.inputs).first(1); }
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override;
    Result<std::vector<InputTileRef>> additional_inputs_for(const DeriveRequest& request) const override;
    Result<DeriveResult> derive(const DeriveRequest& request) const override;
private:
    LayerDesc desc_;
};
}
