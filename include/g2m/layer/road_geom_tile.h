#pragma once
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_cross_section.h"
#include "g2m/layer/road_profile.h"
#include "g2m/layer/road_geom_records.h"
#include <optional>
namespace g2m {
inline constexpr std::string_view kRoadGeomLayer = "g2m.roads.geom";
// Encoding 9 (underpass decks, decisions D53-D58): encoding 8 plus section 8, the underpass deck spans of ground
// stretches over a lower road in a DEM cutting (RoadGeomTile::underpasses). The deriver writes 9.
inline constexpr std::uint16_t kRoadGeomEncoding = 9;
// Encoding 8 (terrain migration phase 4, decision D47): the byte layout of encoding 7, with provenance 4 (TunnelFloor)
// allowed in section 7 and a profile on a tunnel stretch carrying it. Still decoded, and written when
// RoadGeomTile::encoding asks for it (a tile without underpass decks).
inline constexpr std::uint16_t kRoadGeomEncodingTunnels = 8;
// Encoding 7 (street fidelity P2 slice 8): encoding 6 plus junctions (section 5), furniture (section 6) and the
// profile provenance (section 7, values 0..3). Still decoded, and written when RoadGeomTile::encoding asks for it (a
// tile without tunnel floors).
inline constexpr std::uint16_t kRoadGeomEncodingJunctions = 7;
// The P1 body format (cross-sections and features, no junctions, furniture or provenance). Still decoded, and written
// when RoadGeomTile::encoding asks for it (a tile without junctions, furniture and completed profiles): the
// compatibility tests reproduce the encoding-6 bodies with it. The deriver never writes it.
inline constexpr std::uint16_t kRoadGeomEncodingPrevious = 6;
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
    TunnelFloor = 4,         // tunnel stretch fitted to the DEM floor (or the portal chord), encoding 8 only (D47)
};
struct RoadProfileOrigin {
    ProfileProvenance provenance = ProfileProvenance::Strict;
    std::uint16_t strict_decline_code = 0;  // the ErrorCode of the strict decline that was replaced (0 for Strict)
    friend bool operator==(const RoadProfileOrigin&, const RoadProfileOrigin&) = default;
};
// The completion tier of design 6.6 (road_completion.h). Off by default in the library; the deriver turns it on with
// served_road_completion_options() (slice 8). Encoding 6 cannot carry a completed profile (encode_road_geom refuses
// one); encoding 7 stores the provenance of every entry (section 7).
struct RoadCompletionOptions {
    bool enabled = false;
    double retry_spacing_m = 20;          // step 1: the game's retry
    int retry_iterations = 16384;
    double abutment_ground_m = 32;        // ground path to a support point (game: 32 m)
    double abutment_path_m = 200;         // total walk, bridge edges included (D25 locality; 200 m since D48, owner Q2)
    double ground_spacing_m = 40;         // step 3 samples
    double ground_ramp_m = 32;            // step 3 anchor ramps
    // Tunnel floors (D47, the game's tunnel step): DEM samples every tunnel_spacing_m along the reference, fitted at
    // tunnel_max_grade / tunnel_min_radius_m with the clamped first/last sample secants as end grades. A tunnel at
    // least tunnel_chord_min_length_m long is fitted to the straight chord between its two portal samples instead (the
    // DEM above a deep tunnel is the mountain, not the floor); 0 = never (every tunnel follows the DEM, the game's
    // rule, and this struct's default). Owner ruling 2026-10-15 (Q1): long tunnels follow straight lines; the served
    // threshold is de.json's tunnel.straight_grade_min_length_m (200 m), see served_road_completion_options().
    bool tunnel_floors = true;
    double tunnel_spacing_m = 5;
    double tunnel_max_grade = 0.30;
    double tunnel_min_radius_m = 20;
    double tunnel_chord_min_length_m = 0;
    // D65: a tunnel shorter than tunnel_chord_min_length_m but at most tunnel_cut_max_length_m long whose DEM rises
    // between tunnel_cut_hump_min_m and tunnel_cut_hump_max_m above its portal chord also takes the chord (an open cut
    // through an embankment, the terrain carve cuts the bank); 0 = never. Served from de.json's tunnel block.
    double tunnel_cut_max_length_m = 0, tunnel_cut_hump_min_m = 0, tunnel_cut_hump_max_m = 0;
    // D47: at a bridge end node the end plane of an accepted bridge stretch ending there (its height, its end grade
    // along its heading) takes precedence over the walk, as the game's anchor map does; the walk tries every incident
    // bridge edge in order, not only the first.
    bool bridge_anchor_reuse = true;
    // Terrain migration phase 7a-1 (docs/structure_classification.md section 5): the ground side owns the join at a
    // tunnel portal. A tunnel stretch end takes the plane of the ground road that joins it: the end height and grade of
    // an accepted (strict or retried) ground profile ending at the node, else the approach level (median DEM 2-8 m out
    // along the joining ground way, portal_approach.h) with that way's measured grade; two tunnel stretches meeting
    // at a node without a ground way share the junction plane. A ground stretch still declined after the strict pass
    // and the retry then ends on that plane (step 3). Default off: the previous release's tunnel step (the game's).
    bool tunnel_portal_anchor = false;
};
// The outer bounds of the tunnel options; the validator certifies a TunnelFloor profile against them.
inline constexpr double kRoadCompletionMaxTunnelGrade = 0.30;
inline constexpr double kRoadCompletionMinTunnelRadiusM = 20;
// The completion options of the served roads.geom tiles: the tier on, every value at its default except the tunnel
// straight-grade threshold, which comes from the embedded de.json (tunnel.straight_grade_min_length_m). RoadGeomDeriver
// builds with them and writes each value into its config hash text; the goldens and the tests that mean "what the
// deriver serves" use this function too.
inline RoadCompletionOptions served_road_completion_options() {
    RoadCompletionOptions o; o.enabled = true;
    o.tunnel_chord_min_length_m = builtin_road_rules_de().tunnel.straight_grade_min_length_m;
    o.tunnel_cut_max_length_m = builtin_road_rules_de().tunnel.cut_max_length_m;
    o.tunnel_cut_hump_min_m = static_cast<double>(builtin_road_rules_de().tunnel.cut_hump_min_mm) / 1000.0;
    o.tunnel_cut_hump_max_m = static_cast<double>(builtin_road_rules_de().tunnel.cut_hump_max_mm) / 1000.0;
    o.tunnel_portal_anchor = true;
    return o;
}
// What the completion tier did, per kind (design 6.6; diagnostics only, never stored).
struct RoadCompletionStats {
    std::size_t retry_candidates = 0, retry_accepted = 0;
    std::size_t bridge_candidates = 0, bridge_completed = 0, bridge_no_support = 0, bridge_fit_failed = 0;
    std::size_t ground_candidates = 0, ground_completed = 0, ground_failed = 0;
    std::size_t ground_bridge_anchored = 0;  // completed stretches with at least one end on a bridge abutment plane
    std::size_t ground_grades_eased = 0;     // completed only after easing both end grades toward the chord (D43)
    std::size_t ground_base_fallback = 0;    // completed on the base anchors after the accepted bridge planes left no cubic (D47)
    std::size_t tunnel_candidates = 0, tunnel_completed = 0, tunnel_chord = 0, tunnel_cut = 0, tunnel_failed = 0;
    std::size_t bridge_anchor_reused = 0;    // end nodes whose plane came from an accepted bridge stretch (D47), once per node
    std::size_t walk_other_side = 0;         // bridging nodes whose walk found support only away from a later bridge edge (D47)
    std::size_t strict_anchors = 0;          // bridging end nodes that took the strict pass's anchor (D30), once per node
    // 7a-1 portal planes, once per node: from an accepted ground profile / from the approach level / the junction plane
    // of tunnel stretches meeting without a ground way.
    std::size_t portal_ground = 0, portal_approach = 0, portal_joint = 0;
    std::size_t portal_tunnel_ends = 0;      // tunnel floor ends anchored on a portal plane
    std::size_t portal_tunnel_straight = 0;  // ... on the straight floor between the portal heights (the portal grades left no cubic)
    std::size_t portal_tunnel_unfit = 0;     // tunnel floors whose portal planes left no floor (kept on the DEM)
    std::size_t ground_tunnel_anchored = 0;  // declined ground stretches completed on a tunnel portal plane
    std::size_t ground_tunnel_chord = 0;     // ... completed on the chord from the portal (the stretch departs from the DEM's grade)
    std::size_t ground_tunnel_unfit = 0;     // ... whose portal plane left no cubic (completed on the DEM planes, or declined)
};
// Encoding 9, section 8 (decisions D53-D58): a deck span of a ground stretch over a lower road that runs in a DEM
// cutting under it without a bridge tag (road_underpass.h). The profile over [start_m, end_m] is the rim chord, not
// the DEM; a consumer builds a deck (structure surface and collider) over the span from the served profile, as for a
// bridge, and carves the ground stretch only outside it. Stations in integer mm, strictly inside the stretch.
struct RoadUnderpass {
    std::size_t entry_index = 0;        // a ground entry (profile, neither bridge nor tunnel)
    double start_m = 0, end_m = 0;      // deck stations along the entry's reference, 0 < start < end < length
    std::int32_t depth_mm = 0;          // the deepest crossing's depression under the rim chord (diagnostic)
    std::uint16_t flags = 0;            // reserved, 0
    std::vector<std::int64_t> lower_ways;   // the lower ways under the deck, ascending, unique, non-empty
    friend bool operator==(const RoadUnderpass&, const RoadUnderpass&) = default;
};
struct RoadUnderpassStats;   // road_underpass.h
class RoadUnderpassFinder;   // road_underpass.h
// The underpass pass of build_road_geom_tile (D53). `crossings` is a canonical graph holding every way that crosses a
// segment of a tile way (merge_road_graphs over road_crossing_dependencies, the tile graph included); null = the
// context (or the tile graph). Off by default in the library; the deriver turns it on when de.json's underpass block
// is enabled. On, the built tile has encoding 9 and the ground profiles are masked over the spans it serves.
struct RoadUnderpassOptions {
    bool enabled = false;
    const RoadGraph* crossings = nullptr;
    RoadUnderpassStats* stats = nullptr;
};
// Every graph stretch has exactly one accepted profile or explicit decline.
// Consumers must inspect declines; this is never an implicitly complete mesh.
struct RoadGeomTile {
    RoadGraphTile source;
    std::vector<RoadGeomEntry> entries;
    // The P1 traffic-calming features as resolved per entry. Junction suppression and the raised-junction ramps
    // (decisions D22/D23) are NOT applied here: they are a pure function of the junction records
    // (adjust_features_for_junctions), which only the owner tile holds, so a consumer applies them with the junctions
    // it has loaded. Sorted by (entry_index, station bits, node_id).
    std::vector<ProfileFeature> features;
    // 9 for everything built with the underpass pass, else 8; 7, 6 or 5 for a decoded older body. 8 has no underpass
    // decks; 7 has no tunnel floors; 6 has no junctions, furniture
    // or origins; 5 also no cross-sections or features (consumers fall back to the symmetric RoadAttributes::width_mm path).
    std::uint16_t encoding = kRoadGeomEncodingTunnels;
    // One per entry (design 6.6, section 7 of encoding 7): every built tile has them (all Strict without the
    // completion tier). Empty for a decoded encoding-6/5 body.
    std::vector<RoadProfileOrigin> origins;
    // Encoding 7, design 5.1: the junctions whose owner node (nodes.front(), the smallest id) lies in this tile's
    // level-2 square (half-open floor membership on integer mm, as for edge midpoints), ascending by owner, a
    // single-node record before the owner's ClusterTooLarge record. Entry indices refer to this tile's entries.
    std::vector<RoadJunction> junctions;
    // Encoding 7: build_road_furniture over this tile with `junctions` (entry furniture of every entry the record's
    // owning entry rule picks in this tile, plus the furniture of the junctions above; junction_index refers to them).
    std::vector<RoadFurniture> furniture;
    // Encoding 9 (D53-D58): underpass deck spans, sorted by (entry_index, start_m), non-overlapping per entry.
    std::vector<RoadUnderpass> underpasses;
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
// Order: strict profiles, the completion tier (when enabled), cross-sections and features, the junctions
// (build_road_junctions, then the owner filter of 5.1), the furniture (build_road_furniture with the owned
// junctions), the origins (all Strict without the tier), the underpass decks (with `underpass` on and the rules'
// underpass block enabled: the strict and completed ground profiles are masked, encoding 9), then validate_road_geom_tile.
Result<RoadGeomTile> build_road_geom_tile(const RoadGraphTile& graph,
    const RoadDemSampler& dem, const RoadProfileLimits& limits = {}, const RoadGraph* context = nullptr,
    const RoadRules& rules = builtin_road_rules_de(), const RoadCompletionOptions& completion = {},
    RoadCompletionStats* completion_stats = nullptr, const RoadUnderpassOptions& underpass = {});
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
// Design 5.1: true iff the point lies in the key's square (half-open floor membership on integer mm): the junction
// owner rule of encoding 7.
bool road_tile_contains(const TileKey& key, const geom::PointMm& p) noexcept;
Result<void> validate_road_geom_tile(const RoadGeomTile& tile);
Result<TileBody> encode_road_geom(const RoadGeomTile& tile);
Result<RoadGeomTile> decode_road_geom(const TileBody& body);
LayerDesc road_geom_desc();
// The text RoadGeomDeriver hashes into its config hash (design 9.3).
std::string road_geom_config_text();
class RoadGeomDeriver final : public IDeriver {
public:
    RoadGeomDeriver() : desc_(road_geom_desc()) {}
    const LayerDesc& output() const override { return desc_; }
    ContentHash config_hash() const override;
    std::span<const std::string> coverage_inputs() const override { return std::span<const std::string>(desc_.inputs).first(1); }
    std::vector<InputTileRef> inputs_for(const TileKey& target) const override;
    Result<std::vector<InputTileRef>> additional_inputs_for(const DeriveRequest& request) const override;
    // Two stages (D35): the stretch context first, then the far ends of the bridges incident to the tile's nodes.
    std::size_t discovery_stages() const override { return 2; }
    Result<DeriveResult> derive(const DeriveRequest& request) const override;
private:
    LayerDesc desc_;
};
}
