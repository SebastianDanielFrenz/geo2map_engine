# P1: lane-aware cross-section and transitions in `roads.geom` — implementation design

Status: design only, 2026-10-05. Nothing in this document has been implemented.
Audience: the implementing agent. Every slice in section 9 can be built from this
document alone.

Markers used below:
- **[RAA]**, **[RAL]**, **[RASt]**: value from the named German guideline, read
  from secondary sources (plan sheets and lecture notes). The original PDFs could
  not be fetched (BASt: DNS failure; Lippold notes: 404).
- **[unverified]**: the secondary source was ambiguous.
- **[assumed]**: a project default with no citation.
- **[measured]**: counted by this design pass from the committed fixture, the home
  store or the ATKIS ZIP.

Contents:
1. Scope and two pre-existing defects P1 must fix first
2. Data model
3. Resolution algorithm: tags to bands
4. Join resolution: tapers, placement continuity, transition ways
5. Country rules as data (`de.json`)
6. ATKIS conflation decision
7. Encoding 6
8. Physics impact, D-SF7, D-SF8, orthogonality
9. Consumer API (C++ and C) and the racing_game migration plan
10. Implementation slices
11. Decisions record
12. Owner questions

---

## 1. Scope and two pre-existing defects P1 must fix first

**In scope.** For every road stretch whose horizontal reference fits:
- an ordered set of lateral bands (lanes, shoulders, verges, sidewalks, parking and
  cycle lanes), each with a width function of the station s;
- a placement anchor that ties the bands to the OSM way line;
- smooth transitions at degree-2 way joins, where lane count, width or placement
  changes;
- all of it stored in `g2m.roads.geom` encoding 6.

**Out of scope (P2+):**
- lane connectivity through junctions (degree ≥ 3) and junction surfaces;
- turn-pocket ends at junctions, which stay full width up to the junction node;
- crossings and signals geometry;
- superelevation;
- ATKIS geometry.

**Defect A — stretches are not tile-invariant.**
- `select_road_graph_tile` (`src/layer/derive_road_graph.cpp` 32–70) keeps only
  ways whose edges touch the tile plus the 128 m halo.
- `RoadReferenceBuilder::make` then counts node degree from that partial graph.
- Effect: a node of a complete way that lies far outside the halo misses its other
  incident ways. Degree reads 1 or 2 where the world has 3. `stretches()` then
  merges what is really a junction, and endpoint DEM planes differ.
- The same way can therefore get different stretch extents, references and profiles
  in two tiles. No test covers this.
- Demonstrated on the home extract [measured]: the A 66 chain ends at
  `491955400` / `24504854` show `deg_end 1` only because the next way lies outside
  the extract.
- P1 makes it worse: join resolution needs the neighbour way's tags at every
  stretch end. Slice 1 fixes it (section 10).

**Defect B — headings kink at way joins.**
- `fit_shape` (`src/layer/road_reference.cpp` ~39–50) extrapolates both end
  headings of each stretch from its own first and last two segments.
- At a degree-2 way join, the two stretches therefore disagree on the tangent.
- A lateral point at offset t then jumps by t·Δθ in 2-D. Example: 8 m × 1° gives
  14 cm, which breaks the < 1 cm edge acceptance even with perfect lane logic.
- Slice 1 fixes it: at a degree-2 join between two ways, both stretches use the
  shared-bisector heading (the same `l0/(l0+l1)` weighting that interior knots
  already use, computed from the last segment of A and the first segment of B in
  travel order).
- Junction ends (degree ≥ 3) and dead ends keep extrapolation.

## 2. Data model

New public header `include/g2m/layer/road_cross_section.h`, namespace `g2m`.

**Conventions:**
- All stored lengths are integer millimetres (`std::int32_t`).
- Stations s are metres (`double`) along the stretch's `RoadReferenceLine`, from 0
  to `length_m`.
- Lateral t is in metres, **positive to the left** of the reference direction,
  with normal `(-sin h, cos h)`. This matches racing_game's ribbon convention.
- Bands are ordered **left to right in the way's direction**, index 0 leftmost.

```cpp
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

// turn:lanes tokens as bits. Unknown tokens -> field decline TurnToken (section 3.6).
enum TurnBit : std::uint16_t {
    kTurnNone = 1u << 0, kTurnThrough = 1u << 1, kTurnLeft = 1u << 2, kTurnSlightLeft = 1u << 3,
    kTurnSharpLeft = 1u << 4, kTurnRight = 1u << 5, kTurnSlightRight = 1u << 6, kTurnSharpRight = 1u << 7,
    kTurnReverse = 1u << 8, kTurnMergeToLeft = 1u << 9, kTurnMergeToRight = 1u << 10,
    kTurnDefinedMask = (1u << 11) - 1 };

struct WidthRamp {                 // one end of a band
    std::int32_t length_mm = 0;    // 0 = no ramp at this end
    std::int32_t from_width_mm = 0;// band width exactly AT this end (at the join); ramps to width_mm
};
struct BandProvenance { Provenance kind, direction, width, turn, change; };

struct CrossSectionBand {
    LaneKind kind = LaneKind::Driving;
    LaneDirection direction = LaneDirection::Forward;
    LaneChange change = LaneChange::Unspecified;
    bool counted = false;          // one of the OSM `lanes` (motor lanes incl. bus/turn); placement counts these only
    bool paved = true;             // Verge false; everything else true unless surface says unpaved
    bool marked_right = false;     // render: a lane line on this band's right boundary
    std::int32_t width_mm = 0;     // nominal width away from ramps
    std::uint16_t turn = 0;        // TurnBit mask, 0 = no turn data
    BandProvenance provenance{};
    WidthRamp start, end;
};

enum class AnchorEdge : std::uint8_t { Left = 0, Centre = 1, Right = 2 };
enum class PlacementSource : std::uint8_t { Absent = 0, Explicit = 1, Transition = 2 };
struct PlacementAnchor {            // which point of which band lies on the OSM way line (before correction)
    std::uint8_t band = 0;         // index into bands; must be a counted band
    AnchorEdge edge = AnchorEdge::Centre;
    PlacementSource source = PlacementSource::Absent;
};

enum class OffsetMode : std::uint8_t { None = 0, Ramps = 1, Hold = 2, Transition = 3 };
struct OffsetCorrection {          // lateral shift c(s) added to the anchor position, metres = mm/1000
    OffsetMode mode = OffsetMode::None;
    std::int32_t start_mm = 0, start_length_mm = 0; // Ramps: c(0)=start_mm decaying to 0 over start_length_mm
    std::int32_t end_mm = 0, end_length_mm = 0;     // Ramps: c(L)=end_mm ...; Hold: c(s)=start_mm everywhere;
};                                                  // Transition: c(s)=start_mm+(end_mm-start_mm)*S(s/L)

enum class JoinKind : std::uint8_t { None = 0 /*junction or dead end*/, Consistent = 1, LaneChange = 2,
                                     InferredAlignment = 3, PlacementConflict = 4 };
struct JoinRecord {
    JoinKind kind = JoinKind::None;
    std::int64_t neighbour_way_id = 0;   // 0 when kind == None
    std::int32_t raw_delta_mm = 0;       // section 4.3 delta before absorption (signed, + = B left of A)
};

enum CrossSectionFlag : std::uint16_t {
    kXsOneway = 1u << 0, kXsMarkedTagged = 1u << 1, kXsTaperShortenedStart = 1u << 2,
    kXsTaperShortenedEnd = 1u << 3, kXsReversedOneway = 1u << 4 /* oneway=-1 */,
    kXsDualCarriagewayCandidate = 1u << 5, kXsDefinedMask = (1u << 6) - 1 };

// Field-level declines: a tag that was present but unusable. The field falls back to inference.
enum FieldDecline : std::uint32_t {
    kDeclineLanesValue = 1u << 0, kDeclineDirectionCounts = 1u << 1, kDeclineTurnCount = 1u << 2,
    kDeclineTurnToken = 1u << 3, kDeclineChangeCount = 1u << 4, kDeclineWidthLanesCount = 1u << 5,
    kDeclineWidthValue = 1u << 6, kDeclineWidthInconsistent = 1u << 7, kDeclinePlacementValue = 1u << 8,
    kDeclinePlacementRange = 1u << 9, kDeclineBusLanesCount = 1u << 10, kDeclineParkingScheme = 1u << 11,
    kDeclineSidewalkValue = 1u << 12, kDeclineCyclewayValue = 1u << 13, kDeclineDefinedMask = (1u << 14) - 1 };

struct RoadCrossSection {
    std::vector<CrossSectionBand> bands;  // 1..32
    PlacementAnchor anchor;
    OffsetCorrection offset;
    JoinRecord start_join, end_join;
    std::uint16_t flags = 0;
    std::uint32_t field_declines = 0;
    Provenance count_provenance = Provenance::Inferred;     // lane count
    Provenance placement_provenance = Provenance::Inferred; // Tagged iff anchor.source == Explicit
    double length_m = 0;                  // copy of reference length, not encoded (filled at decode from the profile)

    // Evaluation (pure, allocation-free with caller storage; section 9.1).
    double band_width_m(std::size_t band, double s) const;
    double offset_m(double s) const;
    // Writes bands.size()+1 boundary t values, left to right. Returns false if out size is wrong.
    bool boundaries_at(double s, std::span<double> out) const;
    friend bool operator==(const RoadCrossSection&, const RoadCrossSection&) = default;
};

enum class CrossSectionDeclineCode : std::uint16_t { TooManyBands = 1, TotalWidthLimit = 2, NoCountedLane = 3 };
struct CrossSectionDecline { CrossSectionDeclineCode code; std::int32_t detail = 0; };
}
```

**`RoadGeomEntry` gains two fields:**
- `std::optional<RoadCrossSection> cross_section;`
- `std::optional<CrossSectionDecline> cross_section_decline;`

**Invariant:**
- An entry with a `profile` has exactly one of `cross_section` / `cross_section_decline`.
- An entry without a profile has neither (section 8.4 explains why).

**`RoadGeomTile` gains** `std::vector<ProfileFeature> features;` (section 8.3).

**"Lane sections along s" are expressed as end ramps, not explicit OpenDRIVE
laneSections.**
- A stretch never spans two OSM ways (way ends always end a stretch).
- The lane topology is therefore constant inside a stretch. Only widths and the
  lateral offset vary, and only near the two ends.
- Decision D2 (section 11) records the alternative.

### 2.1 Ramp shape and evaluation (exact formulas)

**Smooth step:**
- S(u) = 2u² for u ∈ [0, ½];
- S(u) = 1 − 2(1−u)² for u ∈ [½, 1];
- clamped to 0 below 0 and to 1 above 1.
- It is C1, with S(0) = 0, S(1) = 1 and zero slope at both ends.
- Maximum curvature of a lateral shift W over length a is 4W/a².

**Band width** at station s, for a stretch of length L:
```
d0 = s, d1 = L - s                                   (metres)
r0 = start.length_mm>0 ? (start.from_width_mm - width_mm) * (1 - S(d0 / (start.length_mm/1000))) : 0
r1 = end.length_mm>0   ? (end.from_width_mm   - width_mm) * (1 - S(d1 / (end.length_mm/1000)))   : 0
w(s) = max(0, width_mm + r0 + r1) / 1000             (metres)
```
- Section 4.5 guarantees start.length + end.length ≤ L. So at s = 0 the end ramp
  term is exactly 0 and w(0) = from_width exactly, and likewise at s = L.
- This is what makes join continuity exact.

**Offset** c(s):
- None: 0.
- Hold: start_mm.
- Ramps: start_mm·(1 − S(d0/a0)) + end_mm·(1 − S(d1/a1)).
- Transition: start_mm + (end_mm − start_mm)·S(s/L).
- All values are divided by 1000.

**Boundaries:**
1. The anchor point is at t_a(s) = c(s).
2. Walk outward from the anchor using w_i(s):
   - Left edge of anchor band = t_a + {0, w/2, w} for edge {Left, Centre, Right}.
   - Right edge = left edge − w.
   - Bands to the left accumulate +w; bands to the right accumulate −w.
3. Write out[0] = leftmost boundary … out[n] = rightmost boundary (out is decreasing).
4. Use plain `+ - * /` and `std::max` only; `S` uses no libm. Results are
   bit-identical across platforms.

## 3. Resolution algorithm: tags to bands (one stretch, no neighbours)

New internal file `src/layer/road_cross_section.cpp`:

```cpp
struct CrossSectionBase { RoadCrossSection xs; /* ramps/offset/joins zero */ };
Result<CrossSectionBase> resolve_cross_section_base(const RoadGraphWay& way, const RoadRules& rules);
```

It is a pure function of (way tags, highway class, rules); no geometry is involved.
Steps 3.1–3.6, in this order:

### 3.1 Direction and lane counts

**Direction:**
- `oneway=yes|true|1` → oneway.
- `oneway=-1|reverse` → oneway reversed (sets `kXsReversedOneway`).
  - The reference line runs in way order. "Forward" lanes then run against it,
    so all counted lanes get `Backward`.
- `motorway`, `motorway_link`, `trunk_link`, and `junction=roundabout|circular`
  are implicitly oneway unless `oneway=no`.

**Lane count:**
- `lanes` parsed as an integer 1..16 → Tagged. Otherwise set `kDeclineLanesValue`
  if present-but-bad.
- Two-way roads: read `lanes:forward`, `lanes:backward`, `lanes:both_ways`
  (integers 0..16).
  - If all needed values are present and forward + backward + both_ways == lanes,
    use them (Tagged).
  - If `lanes` is absent but forward/backward are present, the count is
    their sum (Derived).
  - A mismatch sets `kDeclineDirectionCounts`; then use backward = lanes/2 (floor)
    and forward = lanes − backward (Inferred).
- Absent `lanes` uses the rules default (section 5) for (class, oneway).
- Default two-way splits:
  - `lanes=1` two-way → one `Both` lane.
  - Classes with `single_lane_both: true` in the rules → one unmarked `Both` lane.
- **Counted lane order** (left to right in way direction):
  - two-way: backward lanes, then both_ways lanes, then forward lanes;
  - oneway: all Forward (Backward if reversed).

### 3.2 Turn, change and bus tokens

- **Turn tokens:**
  - oneway: `turn:lanes`;
  - two-way: `turn:lanes:forward` and `turn:lanes:backward`.
  - Split on `|`, then `;`. The empty string and `none` mean `kTurnNone`.
  - **Backward-group values are listed in backward travel order.** OSM numbers
    lanes from the left in their own travel direction. So reverse the list before
    assigning it to bands left to right in way direction.
  - A count mismatch → `kDeclineTurnCount`; turn stays 0 on all lanes of that group.
  - An unknown token → `kDeclineTurnToken`; that lane's turn is 0.
- **Change tokens:** `change:lanes[:forward|:backward]` take values yes / no /
  not_left / not_right; the same count rule applies (`kDeclineChangeCount`).
- **Bus lanes:**
  - `bus:lanes[:dir]` or `psv:lanes[:dir]`: a token `designated` makes the lane's
    kind `Bus`.
  - `lanes:bus` / `lanes:psv` alone, without per-lane lists, mark the outermost
    lane(s) of the forward group as Bus (Derived).
  - A count mismatch → `kDeclineBusLanesCount`.
- **Kind:** `Turn` iff turn ≠ 0 and the mask has no `kTurnThrough` and no
  `kTurnNone` bit; otherwise `Driving`. `Bus` overrides both.

### 3.3 Widths of counted lanes

Priority order:
1. **`width:lanes[:dir]`** — per-lane metres, same parser as `width`.
   - Tagged per lane.
   - A count mismatch → `kDeclineWidthLanesCount`.
2. **`width`** — carriageway width, the same parser as `resolve_road_attributes`
   (clean metres or a trailing " m"; 0.5..100).
   - This is the carriageway: counted lanes plus on-carriageway parking and cycle
     bands from 3.4.
   - Counted total = width − rule widths of those bands.
   - Split equally over counted lanes in integer mm. The remainder (total mod n)
     goes to the rightmost counted lane, 1 mm each from the right. Provenance is
     Derived.
   - If the per-lane result is below `rules.min_lane_mm` (2500) → set
     `kDeclineWidthInconsistent` and fall through to 3.
   - A bad value → `kDeclineWidthValue`, then fall through.
3. **Rules default** — `lane_mm` for the class; for oneway motorway/trunk, the
   rightmost counted lane uses `outer_lane_mm` when given. Provenance is Inferred.
   - `single_lane_both` classes use `single_lane_mm` (Inferred).

`RoadAttributes::width_mm` is unchanged and remains the legacy symmetric width
(D12).

### 3.4 Non-counted bands, outward from the counted lanes

Order on each side, from the carriageway outward:
`[Parking] [Cycle] | Shoulder | Verge or Sidewalk`. Parking and Cycle are on the
carriageway.

- **Cycle:**
  - `cycleway:{left,right,both}=lane` or `cycleway=lane` → a `Cycle` band of
    `rules.cycle_lane_mm`, Tagged.
  - For `cycleway=lane` on a oneway, the band goes on the right only.
  - `track`, `separate`, `shared_lane`, `no` → no band.
  - Other values → `kDeclineCyclewayValue`.
- **Parking** (new scheme only):
  - `parking:{left,right,both}=lane` gives a `Parking` band.
  - Its width is `rules.parking_mm[orientation]`, with orientation from
    `parking:<side>:orientation`: parallel (default), diagonal, perpendicular.
  - `no`, `separate`, `street_side`, `on_kerb`, `half_on_kerb` → no band in P1.
  - Old `parking:lane:*` keys → `kDeclineParkingScheme`, no band.
- **Shoulder:** paved, `rules.shoulder_outer_mm[class]` on the outer side(s).
  - For oneway: also `shoulder_inner_mm[class]` on the left (median) side.
  - Provenance Inferred.
  - `shoulder=yes|both|left|right|no` overrides presence (Tagged). Width stays rules.
- **Sidewalk:**
  - `sidewalk=both|left|right` or `sidewalk:{left,right,both}=yes` →
    `Sidewalk` band(s) of `rules.sidewalk_mm`, Tagged, which **replace** the
    Verge on that side.
  - `separate`, `no`, `none` → no band.
  - Bad values → `kDeclineSidewalkValue`.
- **Verge:** `rules.verge_mm[class]`, unpaved (`paved=false`), Inferred, on each
  side without a sidewalk.

**Directions of non-counted bands:**
- Cycle and Parking take the direction of the adjacent outer counted lane.
- Shoulder takes `None`.
- Sidewalk and Verge take `None`.

**Marking:**
- `marked_right` on counted lanes: true between two counted lanes unless
  `lane_markings=no`, or the class has `markings_default: false` without a
  `lane_markings=yes` tag.
- `kXsMarkedTagged` iff `lane_markings` is present.
- Unmarked two-way residential still has two logical lanes.

**Surface:** `paved` is false on every band if `RoadAttributes::surface` resolves
to an unpaved land class.

### 3.5 Placement anchor

- **Tag lookup:**
  - oneway: `placement`;
  - two-way: `placement:forward` / `placement:backward`.
  - If both are given, use forward and set `kDeclinePlacementValue` when they imply
    different anchors.
  - Two-way `placement` without a suffix is accepted only as `transition`.
- **Values:**
  - `left_of:N` → band = counted lane N (1-based, lanes of that direction group,
    numbered from the left in that group's travel direction), edge Left.
  - `right_of:N` → edge Right.
  - `middle_of:N` → edge Centre.
  - `transition` → source Transition, anchor = default.
  - For the backward group, mirror both N and the edge.
- **Validity:**
  - N out of range → `kDeclinePlacementRange`, use the default.
  - Unparsable → `kDeclinePlacementValue`, use the default.
- **Default (Absent):** the middle of the counted lanes.
  - Odd count → middle lane, Centre.
  - Even count n → lane n/2, edge Right.
  - So two-way 1+1 → right edge of the backward lane, which is the centre line.
- `placement_provenance` = Tagged iff the source is Explicit.

### 3.6 Whole-section declines

A whole-section decline is `CrossSectionDecline` and gives no bands. Triggers:
- more than 32 bands → `TooManyBands`;
- sum of nominal widths > 100 m → `TotalWidthLimit`;
- zero counted lanes → `NoCountedLane` (unreachable with valid rules; kept as a
  guard).

These are the only reasons. Inconsistent tagging never declines a whole section:
it declines the field (3.1–3.5) and the join (4.3).

## 4. Join resolution: tapers, placement continuity, transition ways

```cpp
Result<RoadCrossSection> resolve_cross_section(const RoadGraph& graph, const StretchContext& context,
    std::int64_t way_id, RoadStretch stretch, double length_m, const RoadRules& rules);
```

**`StretchContext`** (slice 1) answers two questions for each stretch end node:
- its true degree;
- the single other incident way and the end of that way at the node, when the
  degree is 2 and the node joins two different ways.

**Locality rule.** Everything at a join is a pure function of:
- (base_A, base_B), each computed by section 3;
- the orientation relation (B continues A's direction, or B is reversed);
- the rules.

Lengths only enter when the absorbing side sizes its own ramp (4.5). So both
stretches compute the same Δ and the same absorber independently, in different
tiles, with no iteration. This is the property that makes the output tile-invariant.

### 4.1 Normalisation

- Let A be the stretch being resolved and B the neighbour way at the join node.
- If B's direction at the node is opposite (head-to-head or tail-to-tail), mirror
  B's base:
  - reverse the band order;
  - swap Forward/Backward;
  - mirror anchor edges Left↔Right;
  - negate offsets.
- For the end-of-A join, B follows A; for the start-of-A join, B precedes A. The
  formulas below use "upstream U, downstream D" in A's direction. Δ is always
  measured as t_D − t_U.

### 4.2 Lane matching and width ramps (per direction group)

**Groups:** counted lanes split into Backward, Both and Forward groups. Each group
is matched separately, in its own travel direction. For the backward group, "right"
means way-left.

With n_U and n_D lanes in a group and k = |n_U − n_D|, the wider way W is the side
with more lanes:
1. **k = 0:** match 1:1.
2. **k > 0:** choose the side of the k added/dropped lanes, using W's turn tokens:
   - **Right** if W's rightmost k lanes all carry a right-ish token (right,
     slight_right, sharp_right, merge_to_left), and W's leftmost k lanes do not all
     carry left-ish tokens (left, slight_left, sharp_left, reverse, merge_to_right).
   - **Left** in the mirrored case.
   - **Otherwise,** if at least one side has an Explicit placement: try both sides,
     compute Δ for each (4.3); if exactly one gives |Δ| < 1 mm, take it (Derived).
   - **Otherwise** use `rules.added_lane_side` ("right" for DE; Inferred).
   - The k unmatched lanes of W get from_width = 0 at this join (they are added or
     dropped).
3. **Width mismatch** of matched lanes and of matched non-counted bands:
   - The side with lower width provenance ramps to the other's nominal width
     (Tagged > Derived > Inferred; Official sits above Tagged in P1b).
   - At equal provenance, both ramp to the floor of the integer mean,
     (w_U + w_D) / 2 in mm.
   - Non-counted bands match by kind, outward on each side. An unmatched one ramps
     from 0 (a sidewalk that begins).
4. **Ramp length** for the band ramps on side X: `rules.taper[class_X]` (section 5)
   - added/dropped counted lane: `lane_add_m`;
   - width change: `width_change_m`;
   - non-counted bands: `edge_band_m`.

### 4.3 Offset residual Δ and the absorber

1. Evaluate both sides' boundaries **at the join**, using from_widths (the widths at
   the join after 4.2) and zero correction. Each side's anchor sits at t = 0.
2. Δ = t_D(e) − t_U(e), where e is the left boundary of the leftmost **matched**
   counted lane.
   - After 4.2, every matched counted lane has equal width on both sides at the
     join, so all matched edges have the same Δ.
   - Compute Δ in integer mm; all inputs are integers.
3. Priority: Transition (0) < Absent (1) < Explicit (2). The absorber is the
   lower-priority side.
   - **Equal priority, Absent–Absent:** the side with more counted lanes absorbs;
     tie → larger total carriageway width at the join; tie → the larger way id.
   - **Transition–Transition:** treat both as Absent for this join.
   - **Explicit–Explicit:** with |Δ| ≥ 1 mm, each side absorbs half. The downstream
     side takes floor(Δ/2) and the upstream side takes Δ − floor(Δ/2). Join kind
     is `PlacementConflict`.
4. **Join kind:**
   - `Consistent` if k = 0 and |Δ| < 1 mm;
   - `LaneChange` if k > 0 and |Δ| < 1 mm;
   - `InferredAlignment` if |Δ| ≥ 1 mm and at least one side is not Explicit;
   - `PlacementConflict` as above.
5. `raw_delta_mm` stores Δ from A's viewpoint (D minus U).

### 4.4 Correction on the absorbing stretch

The absorbing stretch must shift by −Δ (if it is D) or +Δ (if it is U) at the join,
so that the matched edges meet.

- **Transition way** (source Transition): `OffsetMode::Transition`, with start_mm
  and end_mm the corrections needed at each end. An end at a junction or dead end
  targets 0.
- **Other end is not a degree-2 way join** (a junction or dead end):
  `OffsetMode::Hold`, so the whole stretch shifts.
  - This keeps turn-pocket through lanes straight. Example: way 4537572 widens to
    the right and its through lanes line up with 256917288.
  - If both ends require Hold with different values (impossible: Hold is only
    chosen when the other end is not a join), report an internal error.
- **Otherwise:** `OffsetMode::Ramps`, with the shift length
  `L_shift = clamp(rules.shift_m_per_m[class] × |Δ| m, shift_min_m, shift_max_m)`.

### 4.5 Ramp clipping (exactness rule)

Per stretch end, take a_end = the maximum of all ramp lengths (band ramps and
offset ramp) at that end.

- If both ends have ramps and a_start + a_end > L: scale every ramp at both ends
  by L/(a_start + a_end), flooring in mm, and set the `kXsTaperShortened*` flags.
- If only one end has ramps and a > L: scale to L.
- After clipping, every ramp of the opposite end is exactly 0 at this end. Hence
  w(0) = from_width and c(0) = start_mm **exactly**, which is why edge continuity
  at joins is exact up to double rounding (≈1e-12 m).

### 4.6 Worked fixture cases

The sets below must appear in tests. N is the lane width; "untagged" means no
`placement`.

**Committed fixture (`frankfurt_hoechst.osm.pbf`, really Kalbach/Bonames — A 661)**

| Case | Ways and joins | Expected |
|---|---|---|
| F1 A 661 on-ramp acceleration lane | 264519302 (2, untagged) →504378→ 264519307 (3, `right_of:1`, `none\|none\|merge_to_left`) | rightmost of W carries merge_to_left → added right; untagged 2 lanes anchor = right of lane 1 = `right_of:1` → Δ=0, `LaneChange` |
| F2 lane end | 264519313 (3, `right_of:1`, `merge_to_left`) →2146570192→ 264519317 (2) | dropped right, Δ=0, `LaneChange`, the dropped lane ramps to 0 over `lane_add_m` inside 264519313 |
| F3 deceleration lane | 23596416 (2) →2701860424→ 264519314 (3, `right_of:1`, `none\|none\|slight_right`) | added right, Δ=0 |
| F4 transition | 177564867 (3, untagged) →504296→ 336837972 (2, `transition`) →3439247819→ 318862793 (2, untagged) | first join: dropped lane default right (no tokens, no explicit placement), Δ=−N/2 absorbed by the transition way at its start; second join Δ=0; `OffsetMode::Transition` start=±N/2, end=0 |
| F5 link left_of | motorway_link 50876963 (1, `left_of:1`) | anchor band 0 edge Left, Tagged; boundary[0] == 0 at stations without ramps (< 1 mm) |
| F6 urban pocket, right | 256917288 (2) →1596079422→ 4537572 (3, `through\|through\|right`) | added right, Δ=±N/2 (absent–absent), wider 4537572 absorbs, other end is a junction → `Hold` |
| F7 pocket, left | 4846955 (1) →2648065658→ 259399524 (3, `left\|through\|right`) | k=2: leftmost lane is left-ish AND rightmost is right-ish → neither side qualifies alone → no explicit placement → default right; `InferredAlignment` |
| F8 two-way asymmetric | 37804346 (3=2/1, `left;through\|right` fwd) →2648065583→ 259399543 (4=3/1) | forward group 2→3, added by tokens of 259399543; backward group 1→1 Consistent |
| F9 untagged vs tagged | 137332121 (no lanes) ↔ 588017497 (2) at 31919397 | inferred 1+1 vs tagged 1+1: widths ramp on the Inferred side, Δ=0 |
| F10 class change | service 175431016 (1 Both) ↔ tertiary 843264244 (2) at 32145074 | group mismatch (1 Both vs 1+1): section 4.7 rule; Δ = 0 when both are centred, so join kind `Consistent`-width-ramped → recorded as `InferredAlignment` only if |Δ| ≥ 1 mm |

The node ids above are degree-2 joins in the fixture graph according to this
design pass's own PBF scan [measured]. Each test must first `REQUIRE` degree 2 at
the listed node, using the slice-1 context. A failing precondition means the scan
was wrong; it is not a reason to change the algorithm.

**Home tile (new committed fixture, slice 4; A 66 / B 8 at Höchst)**

| Case | Ways | Expected |
|---|---|---|
| H1 A 66 lane add with explicit placement | 23658591 (4, untagged → anchor right of lane 2) → 24504854 (5, `right_of:2`, lane 5 `slight_right`) | added right by token, Δ=0, `LaneChange` |
| H2 bridge joins | 1045544333 (3) → 1045544334 (3, bridge) → 148711394 (3) | `Consistent`, no ramps, exact continuity across the bridge ends |
| H3 B 8 drop then add | 155798152 (3, `right_of:1`, `none\|none\|merge_to_left`) → 155798154 (2) → 313879336 (3, `right_of:1`, `none\|none\|slight_right`) | drop right, then add right, both Δ=0; 155798154 has ramps at neither end (the wider sides own the ramps) |
| H4 B 8 middle_of drop | 45674354 (4, `middle_of:2`, lane 4 `merge_to_left`) → 313879334 (3, untagged) | drop right; 4-lane middle_of:2 equals the 3-lane default centre of lane 2 → Δ=0 |
| H5 left pocket on oneway secondary | 104183076 (1) → 324588712 (2, `middle_of:2`, `left\|through`) | added left by token; anchor = centre of the through lane = the single lane's centre → Δ=0 |
| H6 left pocket with right_of | 333909891 (2, untagged) → 480160099 (3, `right_of:2`, `left\|through\|through`) | added left; old `right_of:1` becomes new `right_of:2` → Δ=0 |
| H7 link, ambiguous tokens | 8602212 (2) → 155798159 (3, `left\|left\|right`) | both sides qualify → no explicit placement → default right, Inferred |

The A 66 main-line ways at degree-3 nodes, such as 23658569 → junction, have
`JoinKind::None` at that end; their turn lanes stay full width to the junction.

### 4.7 Count-group mismatch across class changes

- When U and D split their lanes differently, e.g. one `Both` lane vs Backward +
  Forward 1+1:
  - the single `Both` lane is treated as a group of 1 in the Both group;
  - the other side has 0 there and 1 + 1 in the Backward and Forward groups.
- Matching is then impossible group by group. Instead:
  - every lane of the narrower-count side is treated as "dropped"/"added" against
    the whole other side;
  - lanes ramp width from the other side's total, as follows.
- **Rule:** with U = {Both: w} and D = {B: w1, F: w2}:
  - D's two lanes get from_widths that split U's total in the ratio w1 : w2,
    floored in mm, with the remainder on the right lane;
  - Δ is evaluated at the left boundary of the leftmost counted lane (both sides
    exist), with the same absorber rules;
  - the join kind is `InferredAlignment` unless |Δ| < 1 mm.
- This also covers oneway ↔ two-way joins, which are rare and are mapping errors
  or carriageway splits.

## 5. Country rules as data (`de.json`)

- **File:** `data/road_rules/de.json` (new). Embed it at build time as a byte array
  through CMake: `file(READ ... HEX)` into a generated `road_rules_de.inc` under
  the build dir.
- **Loader:** `Result<RoadRules> load_road_rules(std::span<const std::byte>)`,
  using nlohmann/json, which geo2map already uses for configs.
  - Integers only; unknown keys rejected; every class present.
  - Validated at load:
    - widths 0..20000 mm;
    - for each class's `lane_add_m`, the taper must satisfy
      4·`lane_mm`/a² ≤ 1/`min_edge_radius_m`;
    - `shift_min_m` ≤ `shift_max_m`.
- **Hashing:** `RoadRules::content_sha256` = SHA-256 of the embedded bytes. It
  enters `RoadGeomDeriver::config_hash()` (7.4).
- **Selection:** one rules set ("de") per deriver in P1. A `rules_id` string is
  kept in config_hash for later countries.

```json
{
  "format": "g2m.road_rules/1",
  "country": "DE",
  "min_lane_mm": 2500,
  "added_lane_side": "right",
  "cycle_lane_mm": 1850,
  "sidewalk_mm": 2500,
  "parking_mm": { "parallel": 2000, "diagonal": 4500, "perpendicular": 5000 },
  "min_edge_radius_m": { "motorway": 250, "default": 60 },
  "classes": {
    "motorway":       { "oneway_lanes": 2, "twoway_lanes": 4, "lane_mm": 3500, "outer_lane_mm": 3750, "shoulder_outer_mm": 2500, "shoulder_inner_mm": 750,  "verge_mm": 1500, "markings_default": true,
                        "taper": { "lane_add_m": 60, "width_change_m": 60, "edge_band_m": 30, "shift_m_per_m": 60, "shift_min_m": 60, "shift_max_m": 250 } },
    "motorway_link":  { "oneway_lanes": 1, "twoway_lanes": 2, "lane_mm": 3500, "shoulder_outer_mm": 1500, "shoulder_inner_mm": 1000, "verge_mm": 1500, "markings_default": true,
                        "taper": { "lane_add_m": 60, "width_change_m": 40, "edge_band_m": 20, "shift_m_per_m": 40, "shift_min_m": 40, "shift_max_m": 150 } },
    "trunk":          { "oneway_lanes": 2, "twoway_lanes": 2, "lane_mm": 3500, "outer_lane_mm": 3750, "shoulder_outer_mm": 500, "shoulder_inner_mm": 500, "verge_mm": 1500, "markings_default": true, "taper": { "lane_add_m": 60, "width_change_m": 40, "edge_band_m": 20, "shift_m_per_m": 40, "shift_min_m": 40, "shift_max_m": 150 } },
    "trunk_link":     { "oneway_lanes": 1, "twoway_lanes": 2, "lane_mm": 3500, "shoulder_outer_mm": 1000, "shoulder_inner_mm": 500, "verge_mm": 1500, "markings_default": true, "taper": { "lane_add_m": 40, "width_change_m": 30, "edge_band_m": 15, "shift_m_per_m": 30, "shift_min_m": 30, "shift_max_m": 100 } },
    "primary":        { "oneway_lanes": 2, "twoway_lanes": 2, "lane_mm": 3500, "shoulder_outer_mm": 0, "shoulder_inner_mm": 0, "verge_mm": 1500, "markings_default": true, "taper": { "lane_add_m": 30, "width_change_m": 20, "edge_band_m": 10, "shift_m_per_m": 20, "shift_min_m": 20, "shift_max_m": 80 } },
    "primary_link":   { "oneway_lanes": 1, "twoway_lanes": 2, "lane_mm": 3500, "...": "as primary" },
    "secondary":      { "oneway_lanes": 1, "twoway_lanes": 2, "lane_mm": 3250, "shoulder_outer_mm": 0, "shoulder_inner_mm": 0, "verge_mm": 1500, "markings_default": true, "taper": { "lane_add_m": 20, "width_change_m": 15, "edge_band_m": 10, "shift_m_per_m": 20, "shift_min_m": 15, "shift_max_m": 60 } },
    "secondary_link": { "...": "as secondary, oneway_lanes 1" },
    "tertiary":       { "oneway_lanes": 1, "twoway_lanes": 2, "lane_mm": 3000, "verge_mm": 1000, "markings_default": true,  "taper": { "lane_add_m": 20, "width_change_m": 15, "edge_band_m": 10, "shift_m_per_m": 20, "shift_min_m": 15, "shift_max_m": 60 } },
    "tertiary_link":  { "...": "as tertiary" },
    "unclassified":   { "oneway_lanes": 1, "twoway_lanes": 2, "lane_mm": 2750, "verge_mm": 1000, "markings_default": false, "taper": { "lane_add_m": 15, "width_change_m": 10, "edge_band_m": 5, "shift_m_per_m": 15, "shift_min_m": 10, "shift_max_m": 40 } },
    "residential":    { "oneway_lanes": 1, "twoway_lanes": 2, "lane_mm": 2750, "verge_mm": 500,  "markings_default": false, "taper": { "...": "as unclassified" } },
    "living_street":  { "single_lane_both": true, "single_lane_mm": 5000, "verge_mm": 500, "markings_default": false, "taper": { "...": "as unclassified" } },
    "service":        { "single_lane_both": true, "single_lane_mm": 4000, "verge_mm": 500, "markings_default": false, "taper": { "...": "as unclassified" } },
    "track":          { "single_lane_both": true, "single_lane_mm": 3000, "verge_mm": 500, "markings_default": false, "taper": { "...": "as unclassified" } }
  },
  "sources": { "...": "one string per value group, see table below" }
}
```

- The `"..."` entries above are abbreviations in this document only. The committed
  file must spell out every class in full; the loader rejects `"..."`.
- Absent `shoulder_*` keys mean 0.

**Default lane counts and widths** reproduce today's `RoadStyle` class defaults for
untagged two-way roads:

| Class | Lanes × width | Total | Today's default |
|---|---|---|---|
| primary | 2 × 3500 | 7.0 m | 7.0 m |
| secondary | 2 × 3250 | 6.5 m | 6.5 m |
| tertiary | 2 × 3000 | 6.0 m | 6.0 m |
| unclassified, residential | 2 × 2750 | 5.5 m | 5.5 m |
| living_street | 1 × 5000 | 5.0 m | 5.0 m |
| service | 1 × 4000 | 4.0 m | 4.0 m |
| track | 1 × 3000 | 3.0 m | 3.0 m |
| motorway_link | lane 3500 + inner 1000 + outer 1500 paved | 6.0 m | 6.0 m |

So an untagged road keeps its paved width, which avoids a visual regression on the
~90 % of local km without `lanes` (P0).

**Sources:**

| Value | Source |
|---|---|
| motorway 3.75 outer / 3.50 inner lanes, Standstreifen 2.50, Randstreifen 0.75 | [RAA] RQ 31/36, plan-sheet values, [unverified] |
| Bankett 1.50 for rural classes | [RAL] RQ 11 / RQ 9 |
| primary 3.50 lanes | [RAL] RQ 11, EKL 3: 2 × 3.50 + 2 × 0.50 Randstreifen = 8.00 Fahrbahn. The 0.50 Randstreifen is deliberately NOT added: urban primaries dominate the home region and have none; [assumed] |
| secondary 3.25, tertiary 3.00, residential 2.75 | [RASt] urban lanes 2.75–3.25, assigned per class; [assumed] |
| sidewalk 2.50 | [RASt] Seitenraum standard; [unverified] |
| cycle lane 1.85 | ERA 2010 Radfahrstreifen incl. marking; [unverified] |
| parking 2.0 / 4.5 / 5.0 | [RASt] Parkstände; [unverified] |
| motorway Ein-/Ausfädelungs-Verziehung 60 m | [RAA]; [unverified] |
| all other tapers and shift rates | [assumed]. Rule of thumb 1:20 urban, 1:40 rural, 1:60 motorway lateral rate |

## 6. ATKIS conflation decision

**Decision:** conflation is **P1b**, not P1. P1 reserves the interface: the
`Provenance::Official` value, and an optional width-provider seam.

**Measured from the 2026-07 Hessen Basis-DLM ZIP:**

`dlm_42003_ax_strassnachse` (449,244 polylines):
- `brf_breite` is filled on 28.5 % of features, quantised to 0.5 m.
- Widths sit almost entirely on `bvb_verkeh` 1000 (überörtlich, 83k) and 1003
  (Nahverkehr, 41k), about 92 % filled there.
- Local roads are about 3 % filled.
- `fsz_anzahl` is 2 on 397k features: a coarse lane count that adds nothing over
  OSM's 67–100 % `lanes` coverage on those classes.

`dlm_42005_ax_fahrbhnachse` (10,866): per-carriageway width on dual roads.

**What ATKIS would add:**
- carriageway width on classified roads, where OSM `width` covers about 4 %;
- nothing on local streets, which are the visible bulk of a city drive.

**Why P1b:**
- It needs a SHP/DBF importer, a new raw layer (`g2m.src.atkis`) with its own
  licence record (dl-de/zero-2.0), and a matcher.
- None of that is needed for the transition logic.
- Merging it into P1 would double P1's surface area for a width refinement on
  classified roads.

**Matching rule for P1b** (recorded here so P1's seam fits it):
- **Candidates:** ATKIS axis segments within 6 m perpendicular distance of the OSM
  reference.
- **Direction:** |heading difference| < 15°, treated as undirected (fold 180°).
- **Overlap:** the overlap length along the reference must be ≥ 70 % of the
  stretch, or ≥ 50 m.
- **Selection:** score = overlap length − 10 × mean distance. Highest score wins;
  ties go to the lowest ATKIS `oid` (string compare). That makes the rule
  deterministic with no floating-point tie hazard: lengths are rounded to mm
  before comparing.
- **Width:** used only when exactly one candidate passes, OR the top candidate's
  score beats the second by ≥ 20 %. Otherwise there is no ATKIS width; the reason
  is recorded.
- **Distribution:** ATKIS `brf` (carriageway) is distributed like OSM `width`
  (3.3).
  - Provenance `Official` ranks above Tagged for carriageway width only.
  - The owner may invert this; owner question Q5.
- **Licence:** a per-field licence bit in the cross-section record (a reserved
  flag bit in encoding 6, section 7). Consumers that show attribution read it. ODbL
  and dl-de/zero mixing is allowed, because dl-de/zero has no conditions.

**The P1 seam:**
- `resolve_cross_section_base` takes an optional
  `const CarriagewayWidthHint*` (`{width_mm, Provenance, licence_bit}`);
  `nullptr` in P1.
- The encoding already carries Official in the 2-bit provenance fields.

## 7. Encoding 6

### 7.1 Layout

`kRoadGeomEncoding = 6`, encoding_version 1. Sections:

| Type | Codec | Content | Change |
|---|---|---|---|
| 1 | as today | roads.graph payload | **byte-identical** to the embedded roads.graph tile (that encoding moves 4→5 in slice 2 for node tags) |
| 2 | Zstd | entries: way id, stretch, outcome, profile | **unchanged** format, byte for byte |
| 3 | Zstd | cross-section records, one per section-2 entry, same order | new |
| 4 | Zstd | profile features | new |

The decoder requires exactly sections 1, 2, 3, 4 in that order for encoding 6.

**Section 3 grammar.** It uses the existing `ByteWriter` primitives:
`varint` = LEB128 unsigned, `svarint` = zigzag, `u8/u16/u32` little-endian.

```
varint record_count                      == entries.size()
per record:
  u8  record_kind                        0 = Absent (entry has no profile), 1 = CrossSection, 2 = Declined
  kind 2: u16 decline_code, svarint detail
  kind 1:
    u16 flags                            bits outside kXsDefinedMask must be 0; bit 15 reserved = P1b licence bit, must be 0 in P1
    u32 field_declines                   bits outside kDeclineDefinedMask must be 0
    u8  section_provenance               bits 0-1 count_provenance, 2-3 placement_provenance, 4-7 zero
    varint band_count                    1..32
    per band:
      u8  kind                           0..7
      u8  direction                      0..3
      u8  change                         0..4
      u8  band_flags                     bit0 counted, bit1 paved, bit2 marked_right, others 0
      varint width_mm                    0..20000
      u16 turn                           bits outside kTurnDefinedMask must be 0
      u16 provenance                     2 bits each: kind, direction, width, turn, change (bits 0-9), bits 10-15 zero
      varint start_length_mm, varint start_from_width_mm
      varint end_length_mm,   varint end_from_width_mm
    u8  anchor_band, u8 anchor_edge (0..2), u8 anchor_source (0..2)
    u8  offset_mode (0..3)
    svarint offset_start_mm, varint offset_start_length_mm
    svarint offset_end_mm,   varint offset_end_length_mm
    start join: u8 join_kind (0..4), svarint neighbour_way_id, svarint raw_delta_mm
    end join:   same
```

**Section 4 grammar:**

```
varint feature_count                     <= 65536
per feature (sorted by entry_index, then station bits as u64, then node_id):
  varint entry_index                     < entries.size(), entry has record_kind 1
  svarint node_id
  u64 station_m                          binary64 bits, same `number()` helper as section 2
  u8  kind                               ProfileFeatureKind 0..9
  u8  carve                              0 render-only, 1 carved into heights
  varint length_mm, varint height_mm, varint width_mm (0 = full carriageway)
  u8  provenance                         0..3
```

### 7.2 Validation

Extend `validate_road_geom_tile`. All of these are hard errors.

**Records:**
- Record count equals the entry count.
- Kind Absent ⇔ the entry has no profile.

**Enums, flags and counts:**
- Every enum is in range.
- Reserved bits are zero.
- `band_count` is 1..32.

**Anchor:** `anchor_band` < band_count, and that band is `counted`.

**Lengths:**
- Every length is ≤ ceil(reference length mm) + 1.
- For each end: max ramp length at start + max ramp length at end ≤
  ceil(L mm) + 1.
- `offset_mode` Hold ⇒ start_length and end lengths are 0 and end_mm is 0.
- Transition ⇒ both lengths are 0.

**Widths:**
- `width_mm` and `from_width_mm` are ≤ 20000.
- The sum of nominal widths is ≤ 100000.

**Joins:**
- `join_kind` None ⇔ neighbour 0 and delta 0.

**Features:**
- strictly sorted;
- `station` finite and in [0, L];
- `node_id` is one of the entry's anchors or a node of the way (check against
  section 1);
- `length_mm` ≤ 50000, `height_mm` ≤ 300.

**Byte-level:** the decoder rejects trailing bytes in any section, any section >
`kMaxSectionRawSize`, and varints over 10 bytes. The existing `ByteReader`
behaviour is kept.

### 7.3 Backward compatibility

- `decode_road_geom` accepts encoding 5 (sections 1–2 only). It fills
  `cross_section = nullopt`, `cross_section_decline = nullopt` and
  `features = {}`, and sets `RoadGeomTile::encoding = 5`, a new decode-only field.
- `validate_road_geom_tile` skips the cross-section invariants for encoding-5 tiles.
- `encode_road_geom` always writes 6. Encoding an encoding-5 tile is
  `InvalidArgument`.
- Consumers seeing encoding 5 use the legacy symmetric `RoadAttributes::width_mm`
  path (9.3).

### 7.4 Versioning, release and pin pickup

**Layer and rid:**
- The layer version stays 1. A new encoding of the same layer is a body format
  change, which `TileBody.encoding` already tells apart.
- `compute_release_id` hashes the layer name, version and `config_hash`, not the
  encoding. So `RoadGeomDeriver::config_hash()` must change:
  append `" xs=1 enc=6 rules=de:" + hex(rules.content_sha256) + " join-bisector=1 context=nodes"`.
- `RoadGraphDeriver::config_hash()` changes in slice 2:
  `" node-tags=1 enc=5"`.
- Releases without roads (`BuiltinTerrainDerivers::ordered(include_roads=false, …)`)
  keep their rid; the terrain layers are untouched.

**racing_game pickup:**
1. Bump the `external/geo2map_engine` submodule pin. Its `data/world/world_config.json`
   source scope `home-2026-09r1` is unchanged; source layers are untouched.
2. Derived layers go into the derived store (`${RG_G2M_HOME}/baked`) under a new
   rid that `builtin_local_release_params` computes from the new config hashes.
3. The first drive derives roads.graph + roads.geom for the visited tiles. Old
   tiles stay in the store unused. No source re-import is needed.
4. A store GC of the old derived rid is optional.

**Determinism:**
- All integer-mm logic is deterministic. `S(u)` uses only `+ - * /`.
- Join decisions use integer comparisons only.
- The only doubles are reference stations/lengths, already deterministic via
  `g2m::det`, and ramp clipping, which uses floor((a·L_mm)/(a_s + a_e)) in
  `int64` arithmetic with L_mm = llround(length_m × 1000).

### 7.5 Tests (golden and fuzz)

**Golden set `g3xs`:**
- New files: `tests/golden/golden_g3xs.cpp`, `testdata/golden/g3xs_golden.txt`,
  and a `compute_g3xs()` entry in `golden.h` and `dump_main.cpp`.
- Input: the committed `frankfurt_hoechst.osm.pbf` and the new home fixture,
  imported through the existing test pipeline into a temp store, with a synthetic
  flat DEM as in `test_road_geom.cpp`.
  - Profiles need the DEM; cross-sections do not.
- Derive roads.geom for every level-2 tile covering each fixture.
- For each tile, write `sha256(encoded body)`. Also write per-stretch lines:
  `way_id start_ref end_ref band_count join kinds` and boundary values at
  s ∈ {0, L/4, L/2, 3L/4, L}, formatted as `%lld` of llround(t × 1e4) (0.1 mm).
- The file is compared byte for byte between the Windows `release` leg and
  `linux-release`, via the existing golden dump comparison in `tools/ci.ps1`.
- This is the **Windows = Linux** requirement.

**Update the existing golden:** the SHA-256 in `test_road_geom.cpp`
(`4dcfcbca…445f8`) changes in slices 1, 2 and 5. Each slice records the new value
in its commit message, with the reason.

**Fuzz:**
- Extend `Target::Roadgeom` in `tests/fuzz_formats/fuzz_run.cpp` and
  `fuzz_seeds.cpp`.
- Seeds: one encoding-5 body, one encoding-6 body with cross-sections, one with
  features, one with a Declined record.
- The property is unchanged: decode, then (when Ok) validate, encode, decode, and
  require equality.
- Add a dedicated `Target::RoadCrossSectionEval`: decode, then evaluate
  `boundaries_at` at 17 stations per record and require that every result is
  finite and non-increasing left to right.
- WSL legs run them as today.

## 8. Physics impact, D-SF7, D-SF8, orthogonality

### 8.1 One function for footprint, carving and grip

```cpp
struct CarveFootprint {      // lateral t (metres, + left) at station s
    double paved_left, paved_right;        // outer paved boundary: carriageway + shoulders (+ sidewalks, flush)
    double carriageway_left, carriageway_right; // counted lanes + on-carriageway cycle/parking
    double outer_left, outer_right;        // incl. verge/sidewalk = carving flat area
    double crown_pivot;                    // t of the crown line (8.2)
    CrownKind crown;                       // Roof or OneSidedRight / OneSidedLeft
};
CarveFootprint carve_footprint(const RoadCrossSection& xs, double s);  // pure, allocation-free
```

Carving (racing_game `road_surface.cpp`), the render ribbon and the physics grip
raster all use **this** function. Blend (`RoadAttributes::blend_mm`) remains
outside `outer_*`.

**Grip raster:**
- New function in g2m:
  `std::vector<RoadSegment> envelope_road_segments(const RoadGeomTile&, const TileKey& l0_key, double spacing_m = 1.0)`.
- It walks every entry with a cross-section, at stations every `spacing_m` (plus
  s = L).
- For each pair (s0, s1) it emits a capsule:
  - centre line = the reference points offset by
    m(s) = (paved_left + paved_right)/2 at s0 and s1;
  - `half_width_mm` = llround(1000 × max over {s0, s1} of
    (paved_left − paved_right)/2);
  - `land_class` / `surface` from `RoadAttributes`;
  - `rank`, `way_id` and `bridge` as `extract_road_segments`.
- It is clipped to l0_key's box plus the max half-width, and sorted
  `(rank, way_id, station)`.
- Speed limits are copied from the way tags as in `extract_road_segments`, which
  shares the parser.
- **Error bound** against the true envelope: ≤ taper slope × spacing / 2
  (≤ 3 cm at 1:20 and 1 m). That is below the 1 m physics cell, and the test asserts
  it (slice 6).
- Sidewalk bands are emitted as separate capsules with surface `PavingStones`;
  verge is not emitted, so the cell keeps its terrain class.
- `fill_physics_surfaces` is unchanged. Only the roads list installed into
  `ResidentHeightSet` changes source: racing_game installs
  `envelope_road_segments` output when roads.geom is enabled, and
  `extract_road_segments` otherwise.

### 8.2 What changes in heights, what is render-only

`g2m.terrain.height` (bare DEM layer) is **unchanged**. The carved shared height
tile in racing_game changes in four ways:
1. **Asymmetric footprint.** The paved and flat width follows the bands and the
   offset, not ±width/2 around the OSM line. At turn pockets the carved road widens
   to one side.
2. **Shoulder and verge widths per class** from `de.json` replace the global
   `shoulder_mm = 1500`. Motorway right side: 2.5 m paved + 1.5 m verge; residential:
   0.5 m verge. Sidewalks are carved flat at road level (D-SF7).
3. **Crown pivot:**
   - two-way roads put the roof on the direction boundary (the centre line), which
     equals today's centre whenever the anchor is the default;
   - oneway carriageways get a one-sided cross-fall of `crown_per_mille` (25) falling
     to the right (outer) side.
   - This is a physics-visible change on oneway roads. It is decision D10, owner
     question Q3.
4. **Carved profile features** (D-SF8 recommendation) where `carve = 1`.

**Render-only:** lane lines, arrows, `marked_right`, turn tokens, the kerb visual,
`bump`, `mini_bumps`, `rumble_strip` and `cushion`.

**Not changed by P1:** the vertical profile along s, grades, bridges, tunnels.

### 8.3 D-SF7 (kerbs) and D-SF8 (traffic calming) recommendations

**D-SF7 — sidewalk/kerb height.** Recommendation: **physics stays flush; the
render kerb is a bevelled visual of at most 3 cm, or texture only.**

| Option | Pro | Contra |
|---|---|---|
| A. Flush physics, flat/bevel render kerb (recommended) | no wheel clipping; one surface model; matches the 1 m heightfield | kerbs do not stop or jolt cars |
| B. Raised render kerb (12 cm), flush physics | looks right | wheels visibly sink into kerbs; contradicts G-c's "what you see is what you drive" |
| C. Heightfield kerb ramp | some physical feel | a 12 cm step over 1 m cells becomes a 1 m ramp: neither a kerb nor flat; interacts with the crown |
| D. Kerb colliders (static thin boxes per stretch side) | real kerb strike | new collision objects per stretch, streaming and seam work, physics_sim changes; a P3+ feature |

**D-SF8 — traffic calming.**
- **Carve smooth features** that are ≥ 3 m long and full-width: `hump` (default
  length 3.7 m, height 70 mm [assumed, typical DE Aufpflasterung]) and `table`
  (length from `traffic_calming:length` or 6 m, height 80 mm, 1:10 ramps
  [assumed]).
- **Keep render-only:** `bump`, `mini_bumps`, `rumble_strip`, `cushion`,
  `chicane`, `choker`, `island`, `dip`.
  - The first three are shorter than the 2 m Nyquist limit of the 1 m cells.
  - Cushions are partial-width.
  - The rest are lateral, not vertical.
- Both groups go into section 4; `carve` says which is which.
- **Profile model:** z(s, t) = profile(s) + crown(t) + Σ feature(s).
  - feature(s) = h·S(…) ramp up over r, plateau, ramp down;
  - r = min(length/2, height × 10) for tables, length/2 for humps.
- **Station:** the reference segment whose `original_pair_index` equals the node's
  index in the way's ref list gives the segment; then project the node position
  onto that segment's clothoid.
  - The node is a shape node, so this is exact at the knot: station = segment
    `station_m` for the knot index.
  - Nodes at stretch ends are skipped: the junction owns them in P2.
- Needs node tags (slice 2): `traffic_calming` on way nodes.
- In the home extract [measured] there are 178 bump, 136 table, 99 island, 60 hump,
  12 cushion nodes and more.

| Option | Pro | Contra |
|---|---|---|
| A. Carve hump/table, render the rest (recommended) | physically right where the grid can represent it; bumps still visible | two code paths |
| B. Render-only for all | zero physics risk | driving over a table at 50 km/h feels like flat asphalt |
| C. Carve all | uniform | sub-Nyquist bumps alias into 1 m ramps of wrong height; cushions block the full width |
| D. Physics impulse/force element on bumps | real jolt from small bumps | needs a physics_sim road-feature seam; P3+ |

### 8.4 Orthogonality to the endpoint-height / class-grade question

**Confirmed orthogonal.**
- `resolve_cross_section` reads only:
  - way tags;
  - the graph topology (degree and neighbour at stretch ends);
  - the DEM-free horizontal reference length (`fit_stretch`);
  - the rules.
- It never reads the DEM, the vertical profile, anchors' heights or grades.
- Whatever the owner decides about endpoint heights vs class grade changes
  `VerticalProfile` only, and cannot change a cross-section byte.

**The coupling that does exist:**
- **Storage gating.** Section 3 writes record_kind Absent for entries without a
  profile, because consumers cannot place a cross-section without the profile's
  reference: section 2 stores the reference only inside the profile.
  - A profile decline therefore hides an otherwise computable cross-section. It
    does not change one.
- **Shared reference line.** Defect B's bisector fix changes the horizontal
  reference, and hence profiles; it is independent of the grade question.
- **Carving combination** (8.2) consumes both: z = profile + crown + features over
  the footprint. That is a consumer-side sum, not a dependency.

## 9. Consumer API and the racing_game migration plan

### 9.1 C++ API (new header `include/g2m/layer/road_cross_section.h`, implemented in slices 3–6)

```cpp
// Evaluation (all pure, allocation-free, noexcept where marked)
double RoadCrossSection::band_width_m(std::size_t band, double s) const;
double RoadCrossSection::offset_m(double s) const;
bool   RoadCrossSection::boundaries_at(double s, std::span<double> out) const;     // size bands+1
struct CarriagewayEdges { double left, right; };
CarriagewayEdges carriageway_edges(const RoadCrossSection&, double s) noexcept;
CarveFootprint   carve_footprint(const RoadCrossSection&, double s) noexcept;

// World-space helpers (combine with the stretch's RoadReferenceLine)
struct LateralPoint { double x, y; };  // UTM metres, same frame as RoadReferenceLine::at
Result<LateralPoint> lateral_point(const RoadReferenceLine&, double s, double t);

// Lane queries for NPC traffic (replaces npc_truck/npc_traffic's own tag parsing)
struct LaneQuery { std::size_t band; double centre_t; double width_m; LaneDirection direction; std::uint16_t turn; LaneKind kind; };
std::size_t driving_lanes_at(const RoadCrossSection&, double s, LaneDirection travel, std::span<LaneQuery> out) noexcept;
// counted, non-Parking lanes whose direction matches travel (Both always matches),
// ordered left to right in TRAVEL direction; width < 0.5 m (mid-taper) excluded.

// Grip raster input (8.1)
std::vector<RoadSegment> envelope_road_segments(const RoadGeomTile&, const TileKey& l0_key, double spacing_m = 1.0);

// Rules (slice 2)
Result<RoadRules> load_road_rules(std::span<const std::byte> json);
const RoadRules& builtin_road_rules_de();   // parsed once from the embedded bytes; throws never, aborts on a corrupt build
```

### 9.2 C API boundary (PLAN.md 7.5)

`g2m_c/` does not exist yet. P1 only **specifies** the structs, so that the C++
types above map 1:1 and append-only. Implement them when the C API itself is built.

```c
typedef struct g2m_xs_band {
    uint32_t size;                 /* sizeof, first field, append-only */
    uint8_t kind, direction, change, flags;   /* flags: counted|paved|marked_right */
    int32_t width_mm; uint16_t turn; uint16_t provenance;
    int32_t start_ramp_mm, start_from_mm, end_ramp_mm, end_from_mm;
} g2m_xs_band;
typedef struct g2m_xs_view {
    uint32_t size;
    int64_t way_id; uint32_t start_ref, end_ref;
    double length_m;
    uint32_t band_count; const g2m_xs_band* bands;
    uint8_t anchor_band, anchor_edge, anchor_source, offset_mode;
    int32_t offset_start_mm, offset_start_len_mm, offset_end_mm, offset_end_len_mm;
    uint16_t flags; uint32_t field_declines;
    uint8_t start_join_kind, end_join_kind; int64_t start_neighbour, end_neighbour;
} g2m_xs_view;
/* evaluation lives in the library, not duplicated in the plugin: */
int g2m_xs_boundaries_at(const g2m_xs_view*, double s, double* out, uint32_t out_count); /* 0 ok, <0 size error */
int g2m_xs_carve_footprint(const g2m_xs_view*, double s, g2m_carve_footprint* out);
```

- Views point into a refcounted library item (`g2m_item_view` / `g2m_item_release`),
  as 7.5 requires.
- No exceptions cross the boundary.
- Errors are per item (an entry with Declined carries `band_count = 0` and a
  decline code field appended in a later version).

### 9.3 racing_game migration (done later in racing_game; P1 only provides the API)

Each consumer:
- keeps an encoding-5 fallback (no cross-section → today's code path) until the
  derived store has been rebuilt;
- is gated by `world_config.json` `physics.road_surfaces` plus a new
  `roads.cross_sections` flag.

| File | Today | After |
|---|---|---|
| `road_surface.cpp` (`RoadSurfacePatch`, carving) | symmetric `half_width = width_mm/2000`, global shoulder, crown about the centre | `carve_footprint(xs, s)` per carve sample: paved/outer bands, crown pivot and kind; plus section-4 features with `carve=1` |
| `road_visual.cpp` (ribbon, lines, arrows) | re-parses oneway, lanes, lanes:forward/backward, lane_markings, overtaking, turn:lanes; fallback lanes from width; ±width/2 ribbon, 0.30 m shoulder, 8 m junction fades | ribbon from `boundaries_at`; lines at counted-band boundaries with `marked_right`; arrows from `turn`; sidewalk/verge from band kinds; junction fades unchanged (P2 replaces them); `overtaking` stays a render tag read |
| `road_structures.cpp` (deck) | deck half from `width_mm` | deck = `paved_left..paved_right` from `carve_footprint`, asymmetric |
| `npc_truck.cpp` `truck_lane()`, `npc_traffic.cpp` | parse lanes, lanes:forward, turn:lanes, maxspeed:lanes | `driving_lanes_at` for lane centres; maxspeed:lanes stays a tag read (routing is not in P1) |
| `world_terrain.cpp` grip raster | `extract_road_segments` from src.osm | `envelope_road_segments` from resident roads.geom tiles |

Acceptance for the migration (later): no tag parsing of lanes, turn, width or
placement remains in racing_game. Enforce it with a grep in racing_game's CI over
`"lanes"`, `"turn:lanes"`, `"placement"`, `"width"`.

## 10. Implementation slices

Seven commits. Each builds and passes `tools/ci.ps1 -Topic roads` (unit +
determinism) on its own; slices 5 and 7 also need the WSL legs for the golden and
fuzz checks.

**Slice 1 — canonical stretch context and the join heading (defects A, B).**
- Code:
  - `StretchContext` (internal): for every node of every way in the tile graph,
    the complete incident edge list.
  - `RoadGeomDeriver::additional_inputs_for` adds the level-2 roads.graph tiles
    covering each node of each included way. This is a node-to-key set, deduped
    and sorted; it reuses the existing 4096-dependency limit, and exceeding it is a
    LimitExceeded error.
  - `derive` unions their edges at those nodes.
  - `RoadReferenceBuilder` gains `make(graph, context)` and uses the context
    degree.
  - `fit_shape` gains optional join neighbours, from which it builds the
    shared-bisector end heading at degree-2 way joins.
- Tests:
  1. A way crossing three tiles produces bit-identical stretches, reference
     segments and profiles in each tile that owns part of it. Use a synthetic
     graph plus the fixture's A 661 ways.
  2. At every degree-2 way join in the fixture, the two references' end headings
     differ by < 1e-9 rad.
  3. A dependency over the limit gives an explicit error.
- Acceptance: the existing tests pass except for the documented roads.geom golden
  SHA change.

**Slice 2 — node tags in roads.graph, and the rules loader.**
- Code:
  - roads.graph encoding 4 → 5: `RoadGraphNode` gains `tags`, a whitelist of
    `highway`, `traffic_calming`, `crossing`, `crossing:markings`, `kerb`,
    `direction`, `traffic_signals:direction`, `barrier`, `width`, `height`,
    `length` (`traffic_calming:length` included) — keys only from this list.
  - The node record appends `varint tag_count` and key/value strings. The decoder
    accepts 4 (no tags) and 5.
  - `RoadGraphDeriver::config_hash` is bumped.
  - Add `data/road_rules/de.json`, the CMake embed, `load_road_rules` and
    `builtin_road_rules_de`.
- Tests:
  - node-tag round trip;
  - encoding-4 decode still works;
  - fuzz seed for `Target::Roadgraph` with tags;
  - rules loader rejects unknown keys, a missing class, `"..."`, out-of-range
    widths, and a taper violating `min_edge_radius_m`;
  - a pin test that the embedded `de.json` SHA-256 equals the value in the test,
    so a data edit forces a conscious test edit.

**Slice 3 — `resolve_cross_section_base` (section 3) and evaluation (2.1).**
- Tests, unit, table-driven over synthetic `RoadGraphWay` tag sets:
  - each field's Tagged / Derived / Inferred provenance;
  - every `FieldDecline` bit with its fallback;
  - backward-group reversal of turn tokens;
  - placement default for odd and even counts;
  - `width` distribution and remainder placement;
  - sidewalk replacing verge;
  - the parking and cycle bands;
  - `boundaries_at` monotone;
  - `S` exactness at 0, ½ and 1.
- The fixture case F5 (`left_of:1`): boundary[0] == 0 within 1 mm at mid-stretch.

**Slice 4 — join resolution (section 4) and the home fixture.**
- Code: 4.1–4.7 in `resolve_cross_section`.
- Commit `testdata/fixtures/osm/hoechst_a66.osm.pbf`, cut with:
  `g2m_fixture_cut cut-pbf --in hessen-latest.osm.pbf --relation 4209939 --bbox 8.5190,50.1085,8.5418,50.1232 --out hoechst_a66.osm.pbf`
  - That is home tile utm32n/2/455/5421 plus 300 m.
  - Update `testdata/fixtures/README.md` (source, licence, size, SHA-256) and
    `THIRD_PARTY_NOTICES.md`.
  - If the result is > 1.8 MB, shrink the bbox symmetrically around the B 8 / A 66
    interchange (8.527, 50.116) until it fits. Record the final bbox.
- Tests (fixture-driven):
  - F1–F10 and H1–H7 expectations: join kind, added side, Δ, absorber, offset mode.
  - **Edge continuity:** at every degree-2 join in both fixtures, for every
    boundary matched across the join (matched counted lanes and matched
    non-counted bands), the 2-D point from `lateral_point` on both sides differs by
    < 1 cm. This is the plan's |Δ| < 1 cm acceptance, measured after the slice-1
    heading fix.
  - **Scalar t continuity:** < 1e-6 m.
  - **Placement:** for every stretch with Explicit placement and offset mode None,
    the anchor point's t is < 1 mm from 0 at 32 stations outside all ramps.
  - Explicit–Explicit split (synthetic): each side holds half; `PlacementConflict`.
  - **Tile invariance:** resolve every fixture stretch from two different tiles
    owning parts of the way; the cross-sections are equal.

**Slice 5 — roads.geom encoding 6.**
- Code: sections 3 and 4 (section 4 empty until slice 7), validation (7.2),
  encoding-5 read compatibility (7.3), the config hash (7.4).
- Tests:
  - round trip;
  - each validation rule rejects a crafted body (one test per rule);
  - encoding-5 golden bodies decode with `cross_section == nullopt`;
  - new fuzz seeds and `Target::RoadCrossSectionEval`;
  - the golden set `g3xs` with Windows = Linux compare;
  - `test_release.cpp`: the rid changes for a geometry release and is unchanged for
    a terrain-only release.

**Slice 6 — consumer API.**
- Code: `carve_footprint`, `carriageway_edges`, `lateral_point`, `driving_lanes_at`,
  `envelope_road_segments`.
- Tests:
  - **Carving width from the same function:**
    `envelope_road_segments` capsules' painted coverage on a 0.25 m lattice
    (via `rasterize_road_segments`) agrees with `carve_footprint`'s paved interval
    at every lattice point, except within 5 cm of the paved edge.
    Synthetic plus fixture.
  - Zero allocations in the `carve_footprint` / `boundaries_at` loops (the existing
    `alloc_counter`).
  - `driving_lanes_at` order and direction on F8.
  - Error bound ≤ 3 cm at 1:20 tapers.

**Slice 7 — profile features (D-SF8).**
- Do this only after the owner's D-SF8 answer. The default is option A.
- Code: section-4 population from node tags, station lookup (8.3), the feature
  height function exposed as
  `double feature_height_m(const RoadGeomTile&, std::size_t entry, double s)`.
- Tests:
  - synthetic hump and table: height and C1 continuity;
  - render-only kinds get `carve = 0`;
  - end-node features are skipped;
  - golden update.

**Optional slice 8 — dual-carriageway probe.**
- Do this only if Q4 says so.
- A report-only tool `tools/road_probe --dual`:
  - pairs oneway stretches with the same `ref` or `name` that are antiparallel
    (heading difference > 165°) and within 40 m;
  - reports their inner-edge gap (negative = the carriageways overlap).
- No geometry change. It informs whether an inner-edge clamp is ever needed.

## 11. Decisions record

**D1 — Lateral representation: bands with nominal widths plus end ramps and an
anchor; boundaries computed on demand.**
- Alt: store boundary polylines sampled along s. Pro: trivial consumers. Con: large;
  lossy; tile seams in the samples; no lane semantics.
- Alt: OpenDRIVE-style laneSections with cubic width polynomials. Pro: industry
  shape. Con: section boundaries add validation and bytes for no gain, since the
  topology is constant per stretch.
- **Chosen:** compact (~20 bytes per band), exact at joins by construction, keeps
  semantics.

**D2 — No intra-stretch lane sections.**
- Reason: a stretch never spans two ways, so lane topology is constant. Ramps cover
  the only variation.
- Revisit only if P2 splits stretches inside a way at junction pockets.

**D3 — Join rules are one-hop local and symmetric.**
- Alt: global chain optimisation (least-squares placement along a whole route).
  Pro: smoother. Con: non-local — one tag edit changes geometry kilometres away;
  tile-invariance needs whole chains; iteration.
- **Chosen:** each side computes the same Δ and absorber independently.
  Tile-invariant with only the slice-1 node context.

**D4 — Absorber priority Transition < Absent < Explicit, with Hold at junction
ends.**
- Alt: always ramp both sides half. Con: moves tagged placements; turn-pocket
  through lanes jog.
- Alt: always the downstream side. Con: arbitrary, and ignores the tagged data.
- **Chosen:** it follows the wiki semantics (transition exists to absorb) and keeps
  tagged lines exact. Hold keeps through lanes straight at pockets (F6).

**D5 — Added-lane side from turn tokens, then tagged placement, then country
default.**
- Alt: always right. Con: left-turn pockets (H5, H6) would jog by a lane width.
- Alt: placement only. Con: needs placement tags, which only 26 % of motorway km
  have.

**D6 — C1 two-parabola S-ramps.**
- Alt: linear. Con: kinks in edges (C0), visible in ribbons and carving.
- Alt: cubic or clothoid. Pro: G2. Con: more maths, no visible gain at 1 m cells.
- Taper curvature is bounded by validating rules (4W/a² ≤ 1/R_min), not by
  stretching tapers at runtime, which keeps lengths readable in the data.

**D7 — Integer mm everywhere stored; join decisions in integers.**
- Reason: determinism (Windows = Linux) without libm.
- Alt: doubles throughout. Con: tie-breaks and equality become platform risks.

**D8 — Untagged widths reproduce today's class defaults.**
- Alt: RAL/RASt nominal per-class values (e.g. primary 8.0 m with Randstreifen).
  Con: visible widening of every untagged road at once, mixed with the transition
  change, harder to review.
- Cited values are used where they matter most (motorway, rural shoulders and
  verges).

**D9 — ATKIS in P1b with a reserved Official provenance and a width seam.**
- See section 6. Alt: P1 inclusion. Con: importer + raw layer + licence + matcher
  for ~28 % of axis features, mostly classified roads.

**D10 — Oneway carriageways get a one-sided cross-fall.**
- Alt: keep the roof crown at the band centre. Pro: no physics change. Con: a
  motorway carriageway with a roof at its centre is wrong in reality.
- This is a physics-visible change, so it is an owner question (Q3).
- Default if unanswered: one-sided.

**D11 — Grip raster from `envelope_road_segments` capsules on 1 m stations.**
- Alt: a new polygon rasteriser. Pro: exact. Con: a second paint path, losing the
  "same loop as render" property `fill_physics_surfaces` has.
- Alt: keep `extract_road_segments`. Con: grip and carving diverge at every
  asymmetric road (G2M-009, one road source for visuals and grip).
- **Chosen:** reuses `paint_road_blocks` unchanged. Error is sub-cell.

**D12 — `RoadAttributes::width_mm` unchanged (legacy).**
- Reason: keeps section 2 byte-identical and the profile untouched.
- New consumers read the cross-section.
- Alt: redefine it as the carriageway width. Con: silent semantic change for
  existing readers.

**D13 — Node-tag whitelist in roads.graph.**
- Alt: all tags of tagged way nodes. Pro: future-proof. Con: addresses and names on
  shared nodes bloat tiles; a later key costs one encoding bump anyway.

**D14 — Storage gating: no cross-section without a profile.**
- Alt: store the reference in section 3 too. Pro: cross-sections for
  profile-declined stretches. Con: duplicates the reference bytes; such stretches
  cannot be carved or drawn anyway (no heights).
- Revisit if 2-D map rendering of declined roads becomes a product.

**D15 — Encoding 5 stays decodable.**
- Reason: existing derived stores (racing_game `home-r1` baked) remain readable by
  tools; consumers fall back cleanly.
- Cost: one branch in the decoder and validator.

**D16 — Explicit–explicit conflicts are split half and half and recorded.**
- Alt: decline the join and leave a step. Con: visible gaps in render and carving.
- Alt: prefer the longer way. Con: arbitrary.
- The `PlacementConflict` join kind keeps it auditable (probe reports can list them
  for mapping fixes).

## 12. Owner questions

- **Q1 (D-SF7):** keep physics flush, with the render kerb bevelled to ≤ 3 cm or
  texture-only (recommended A), or accept a raised visual kerb (B) despite wheels
  visibly sinking in?
- **Q2 (D-SF8):** carve `hump` and `table` into the height tile, with bumps,
  cushions and rumble strips render-only (recommended A)? Slice 7 waits for this.
- **Q3 (D10):** one-sided 2.5 % cross-fall on oneway carriageways (realistic, a
  physics-visible change on every motorway and oneway street), or keep the roof
  crown?
- **Q4:** dual carriageways — accept independent carriageways (recommended), and do
  you want the report-only overlap probe (slice 8)?
- **Q5 (P1b):** when ATKIS and OSM `width` disagree, should official ATKIS width
  outrank a mapper's tagged width (proposed), or should tagged OSM win?
- **Q6:** sidewalk default — leave untagged roads without sidewalks (proposed; no
  urban-area signal in P1), or infer sidewalks on residential and tertiary roads
  inside built-up land classes?
- **Q7:** the committed home fixture (`hoechst_a66.osm.pbf`, ODbL, ≤ 1.8 MB) —
  OK to add a second OSM fixture to the repo?

---

## Owner decisions (2026-10-05, binding for the implementation)

| # | Question | Decision |
|---|---|---|
| Q1 / D-SF7 | Kerbs | **Option B: raised VISUAL kerb** (12 cm render kerb), physics flush. The owner accepts wheels visibly meeting the kerb for now and wants real physical kerbs later (option D, a P3+ feature). Keep kerb height, side and band edge in the data model so a later collider can use them. |
| Q2 / D-SF8 | Traffic calming | **Option C: carve everything** (bump, mini_bumps, rumble_strip, cushion and the rest of the vertical kinds as well as hump/table; lateral kinds such as chicane/choker/island stay render-only because they have no height). The sub-Nyquist problem stays: slice 7 must make every carved feature representable on the 1 m grid. Enforce a minimum ramp of 1 m per side so nothing aliases into a spike; carve cushions across the lane they sit in (partial width cannot be represented). Report the measured effective shapes in the slice 7 commit. A physics impulse element for short bumps (option D) remains a later possibility. |
| Q3 / D10 | One-way cross-fall | **One-sided 2.5 %** on one-way carriageways. |
| Q4 | Dual carriageways | Independent carriageways (recommended). Slice 8 (the report-only overlap probe) is optional and runs last, if at all. |
| Q5 (P1b) | ATKIS vs OSM width | Owner: no preference. Keep the proposal: the official ATKIS width outranks a tagged OSM width. P1b only, not part of slices 1-7. |
| Q6 | Untagged sidewalks | Proposed: none inferred. |
| Q7 | Second OSM fixture | Yes, commit `hoechst_a66.osm.pbf` (<= 1.8 MB, ODbL). |
