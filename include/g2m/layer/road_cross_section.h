// Street fidelity P1: the lane-aware cross-section of a road stretch.
// A stretch is a set of lateral bands (lanes, shoulders, verges, sidewalks,
// parking and cycle lanes) with integer-millimetre widths, a placement anchor
// that ties the bands to the OSM way line, and end ramps / an offset correction
// that make the section continuous at way joins. Boundaries are computed on
// demand from this compact form (design: docs/street_fidelity/p1_cross_section_design.md).
//
// Conventions: lengths stored in integer millimetres; stations s in metres along
// the stretch's RoadReferenceLine; lateral t in metres, positive to the LEFT of
// the reference direction (normal (-sin h, cos h)); bands are ordered left to
// right in the way direction, index 0 leftmost. Turn and lane-change data of a
// band are in the band's own travel direction, as in OSM.
#pragma once
#include "g2m/layer/road_graph.h"
#include "g2m/layer/road_reference.h"
#include "g2m/layer/road_rules.h"
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace g2m {
enum class Provenance : std::uint8_t { Tagged = 0, Derived = 1, Inferred = 2, Official = 3 };
// Tagged   = read directly from an OSM tag on this way.
// Derived  = computed from other tagged fields (width split from `width`, count from lanes:forward+backward,
//            added-lane side chosen from turn tokens or tagged placement).
// Inferred = country-rules default.
// Official = reserved for P1b ATKIS values; P1 never writes it, the decoder accepts it.

enum class LaneKind : std::uint8_t { Driving = 0, Turn = 1, Bus = 2, Cycle = 3, Parking = 4,
                                     Shoulder = 5, Verge = 6, Sidewalk = 7 };
// Shoulder = paved (Randstreifen/Standstreifen), Verge = unpaved Bankett.
enum class LaneDirection : std::uint8_t { Forward = 0, Backward = 1, Both = 2, None = 3 };
enum class LaneChange : std::uint8_t { Unspecified = 0, Yes = 1, No = 2, NotLeft = 3, NotRight = 4 };

// turn:lanes tokens as bits. Unknown tokens -> field decline TurnToken.
enum TurnBit : std::uint16_t {
    kTurnNone = 1u << 0, kTurnThrough = 1u << 1, kTurnLeft = 1u << 2, kTurnSlightLeft = 1u << 3,
    kTurnSharpLeft = 1u << 4, kTurnRight = 1u << 5, kTurnSlightRight = 1u << 6, kTurnSharpRight = 1u << 7,
    kTurnReverse = 1u << 8, kTurnMergeToLeft = 1u << 9, kTurnMergeToRight = 1u << 10,
    kTurnDefinedMask = (1u << 11) - 1 };

struct WidthRamp {                  // one end of a band
    std::int32_t length_mm = 0;     // 0 = no ramp at this end
    std::int32_t from_width_mm = 0; // band width exactly AT this end (at the join); ramps to width_mm
    bool operator==(const WidthRamp&) const = default;
};
struct BandProvenance {
    Provenance kind = Provenance::Inferred, direction = Provenance::Inferred, width = Provenance::Inferred,
               turn = Provenance::Inferred, change = Provenance::Inferred;
    bool operator==(const BandProvenance&) const = default;
};

struct CrossSectionBand {
    LaneKind kind = LaneKind::Driving;
    LaneDirection direction = LaneDirection::Forward;
    LaneChange change = LaneChange::Unspecified;
    bool counted = false;           // one of the OSM `lanes` (motor lanes incl. bus/turn); placement counts these only
    bool paved = true;              // Verge false; everything else true unless the surface is unpaved
    bool marked_right = false;      // render: a lane line on this band's right boundary
    std::int32_t width_mm = 0;      // nominal width away from ramps
    std::uint16_t turn = 0;         // TurnBit mask, 0 = no turn data
    BandProvenance provenance{};
    WidthRamp start{}, end{};
    bool operator==(const CrossSectionBand&) const = default;
};

enum class AnchorEdge : std::uint8_t { Left = 0, Centre = 1, Right = 2 };
enum class PlacementSource : std::uint8_t { Absent = 0, Explicit = 1, Transition = 2 };
struct PlacementAnchor {            // which point of which band lies on the OSM way line (before correction)
    std::uint8_t band = 0;          // index into bands; must be a counted band
    AnchorEdge edge = AnchorEdge::Centre;
    PlacementSource source = PlacementSource::Absent;
    bool operator==(const PlacementAnchor&) const = default;
};

enum class OffsetMode : std::uint8_t { None = 0, Ramps = 1, Hold = 2, Transition = 3 };
struct OffsetCorrection {           // lateral shift c(s) added to the anchor position, metres = mm/1000
    OffsetMode mode = OffsetMode::None;
    std::int32_t start_mm = 0, start_length_mm = 0; // Ramps: c(0)=start_mm decaying to 0 over start_length_mm
    std::int32_t end_mm = 0, end_length_mm = 0;     // Ramps: c(L)=end_mm ...; Hold: c(s)=start_mm everywhere;
                                                    // Transition: c(s)=start_mm+(end_mm-start_mm)*S(s/L)
    bool operator==(const OffsetCorrection&) const = default;
};

enum class JoinKind : std::uint8_t { None = 0 /*junction or dead end*/, Consistent = 1, LaneChange = 2,
                                     InferredAlignment = 3, PlacementConflict = 4 };
struct JoinRecord {
    JoinKind kind = JoinKind::None;
    std::int64_t neighbour_way_id = 0;   // 0 when kind == None
    std::int32_t raw_delta_mm = 0;       // delta before absorption (signed, + = B left of A)
    bool operator==(const JoinRecord&) const = default;
};

enum CrossSectionFlag : std::uint16_t {
    kXsOneway = 1u << 0, kXsMarkedTagged = 1u << 1, kXsTaperShortenedStart = 1u << 2,
    kXsTaperShortenedEnd = 1u << 3, kXsReversedOneway = 1u << 4 /* oneway=-1 */,
    kXsDualCarriagewayCandidate = 1u << 5 /* reserved for the optional dual-carriageway probe, never set in P1 */,
    kXsDefinedMask = (1u << 6) - 1 };

// Field-level declines: a tag that was present but unusable. The field falls back to inference.
enum FieldDecline : std::uint32_t {
    kDeclineLanesValue = 1u << 0, kDeclineDirectionCounts = 1u << 1, kDeclineTurnCount = 1u << 2,
    kDeclineTurnToken = 1u << 3, kDeclineChangeCount = 1u << 4 /* also an unknown change token */,
    kDeclineWidthLanesCount = 1u << 5, kDeclineWidthValue = 1u << 6, kDeclineWidthInconsistent = 1u << 7,
    kDeclinePlacementValue = 1u << 8, kDeclinePlacementRange = 1u << 9, kDeclineBusLanesCount = 1u << 10,
    kDeclineParkingScheme = 1u << 11, kDeclineSidewalkValue = 1u << 12, kDeclineCyclewayValue = 1u << 13,
    kDeclineDefinedMask = (1u << 14) - 1 };

// C1 smooth step: 2u^2 on [0, 1/2], 1 - 2(1-u)^2 on [1/2, 1], clamped outside [0, 1]. Only + - * /.
double road_smooth_step(double u) noexcept;

struct RoadCrossSection {
    std::vector<CrossSectionBand> bands;  // 1..32
    PlacementAnchor anchor;
    OffsetCorrection offset;
    JoinRecord start_join, end_join;
    std::uint16_t flags = 0;
    std::uint32_t field_declines = 0;
    Provenance count_provenance = Provenance::Inferred;     // lane count
    Provenance placement_provenance = Provenance::Inferred; // Tagged iff anchor.source == Explicit
    double length_m = 0;                  // copy of the reference length, not encoded (filled at decode from the profile)

    // Evaluation: pure, allocation-free with caller storage, plain + - * / only.
    // Width of band `band` at station s (ramps applied), metres.
    double band_width_m(std::size_t band, double s) const;
    // Lateral correction c(s), metres.
    double offset_m(double s) const;
    // Writes bands.size()+1 boundary t values, left to right (decreasing). False if out has the wrong size.
    bool boundaries_at(double s, std::span<double> out) const;
    friend bool operator==(const RoadCrossSection&, const RoadCrossSection&) = default;
};

enum class CrossSectionDeclineCode : std::uint16_t { TooManyBands = 1, TotalWidthLimit = 2, NoCountedLane = 3 };
struct CrossSectionDecline {
    CrossSectionDeclineCode code = CrossSectionDeclineCode::TooManyBands;
    std::int32_t detail = 0;
    bool operator==(const CrossSectionDecline&) const = default;
};

// The cross-section of one way from its own tags and the rules alone: bands, anchor and field declines;
// ramps, offset and joins stay zero (they need neighbours). A whole-section decline (too many bands, more than
// 100 m total, no counted lane) is reported in `decline` with `xs` left empty. Errors are for a way that is not
// a supported road (unsupported highway, noncanonical tags) or a highway the rules do not know.
struct CrossSectionBase {
    std::optional<RoadCrossSection> xs;
    std::optional<CrossSectionDecline> decline;
};
Result<CrossSectionBase> resolve_cross_section_base(const RoadGraphWay& way, const RoadRules& rules);

// The join-aware cross-section of one stretch (design section 4): resolve_cross_section_base of the way, then at
// each end that is a degree-two way join the lane matching, width ramps, offset residual and its absorber, and the
// end ramps clipped to the stretch length. A pure function of the two ways' tags, the orientation relation and the
// rules (lengths enter only through the clipping), so both stretches of a join, resolved in different tiles, agree
// and the result is tile invariant. `length_m` is the stretch's reference length; the result's length_m is set.
// A way whose base declines (or has no rules) yields the decline; a neighbour whose base declines is treated as
// "no join" because it has no section to meet.
Result<CrossSectionBase> resolve_cross_section(const RoadReferenceBuilder& builder, std::int64_t way_id,
    RoadStretch stretch, double length_m, const RoadRules& rules);

// World-space point at station s and lateral offset t (metres, positive left of the reference direction), in the
// same UTM frame as RoadReferenceLine::at.
struct LateralPoint { double x = 0, y = 0; };
Result<LateralPoint> lateral_point(const RoadReferenceLine& reference, double s, double t);
} // namespace g2m
