// g2m/layer/structure_classify.h - the structure classification rule of terrain migration phase 7a
// (docs/structure_classification.md), as a pure function over one tunnel chain's measured cover profile.
//
// It is the rule the census tool (g2m_structure_census) applies, and the reference the served implementation
// (phase 7a-2) must reproduce: given, per 1 m station along a tunnel chain, the measured cover (bare-earth DEM minus the
// portal-chord floor, the minimum across the tube width) and what lies above that station, it returns the chain's runs
// (tube / open cut / cut under a deck), the chain class (real cover / no cover / mixed) and each tube run's profile
// shape (box or bored). Plain + - * / and comparisons on doubles; no libm, no unordered container, so it is
// deterministic and portable.
#pragma once

#include <cstdint>
#include <span>
#include <vector>

namespace g2m::structure {

// What lies above a station of the tunnel axis, in priority order (a station under several keeps the first). A bridge
// above means DGM1 removed that structure and interpolated the gap: the DEM there is not cover.
enum class Above : std::uint8_t {
    Nothing = 0,
    RailBridge = 1,   // a railway way tagged bridge whose footprint covers the station
    RoadBridge = 2,   // a road way tagged bridge
    Building = 3,     // inside a building outline (a building passage)
    RoadGround = 4,   // a ground road crosses above (the A66 service tunnel case)
    RailGround = 5,   // a ground railway crosses above (on an embankment or level)
    TunnelAbove = 6,  // another tunnel at a higher layer (tunnel over tunnel)
};
inline constexpr int kAboveKinds = 7;

// Every run has a roof (owner 2026-10-20: "a tunnel needs a roof"); the kinds differ in where it comes from and in
// whether the terrain stays (Tube) or is cut open down to the floor (all others).
enum class RunKind : std::uint8_t {
    Tube = 0,         // real cover: the hill stays, a closed tube (box or bored) under it
    Deck = 1,         // a mapped bridge above (rail or road): DGM1 removed it; box opening, roof = that bridge's deck
    ImpliedDeck = 2,  // a ground road or railway above without the cover a tube needs: it stands on a deck OSM did not
                      // tag (the rail embankment over Unterliederbach way 4812754); box opening, roof = that implied deck
    Slab = 3,         // no real cover and no way above (a building passage, or nothing in the data): open cut, roof = a
                      // served roof slab
};
inline constexpr int kRunKinds = 4;
enum class ChainClass : std::uint8_t { RealCover = 0, NoCover = 1, Mixed = 2 };
enum class TubeShape : std::uint8_t { Box = 0, Bored = 1 };

// All lengths in metres. The served values live in data/road_rules/de.json ("structure" block).
struct Thresholds {
    double clearance_m = 4.7;         // free height over the floor (road: 4.50 m + 0.20 m reserve)
    double roof_m = 1.0;              // structural roof thickness of a tube
    double min_tube_m = 10.0;         // a tube run shorter than this is a cut
    double min_gap_m = 10.0;          // an interior cut gap shorter than this inside a tube stays tube (exposed roof)
    double bored_min_length_m = 300.0; // a tube run at least this long ...
    double bored_min_cover_m = 15.0;   // ... with at least this median natural cover is bored (circular); else a box
    double max_grade = 0.05;          // steepest floor grade from a portal down to the needed depth (road 5 %, rail 4 %)
    double cover_needed() const { return clearance_m + roof_m; }
};

struct Run {
    RunKind kind = RunKind::Slab;
    std::size_t first = 0, last = 0;  // station indices, inclusive
    double min_cover_m = 0, median_cover_m = 0, max_cover_m = 0;
    double median_natural_cover_m = 0; // ground minus the portal chord
    TubeShape shape = TubeShape::Box; // tube runs only
    bool exposed_roof = false;        // a tube run that absorbed a short cut gap (its roof stands above the terrain)
};

struct Classification {
    std::vector<Run> runs;            // ascending, covering every station exactly once
    ChainClass cls = ChainClass::NoCover;
    std::size_t portals = 0;          // tube ends (two per tube run: a tube end faces an open run or a chain end)
    std::size_t ground_road_conflicts = 0; // RoadGround stations whose cover is below clearance + roof
};

// The design floor of a tunnel chain (the floor the cover is measured against). ground[i] is the lowest bare-earth
// height across the tube width at station i, at s_i = min(i, length_m) metres from the chain start. It is the highest
// profile that (a) never rises above the portal chord, (b) stays at least `needed` below the ground at every station at
// least needed / max_grade from each portal (nearer, no floor leaving the portal at max_grade could be that deep, and
// the cover grows from the portal face), and (c) has no grade steeper than max_grade, joined to each PORTAL end at that
// grade (an end that continues into another tunnel way
// is not a portal: it is anchored `needed` below the ground there and constrains nothing). In a hill the chord is low
// enough and the floor is the chord (D52); under flat ground or a depression the floor drops from each portal at
// max_grade to `needed` below the ground and follows it (grade-limited). Where (a)-(c) conflict - near a portal, or a
// chain too short to reach the depth - (c) and the portals win and the cover there is below `needed`. NaN ground is
// treated as no constraint. Plain + - min max: deterministic.
std::vector<double> design_floor(std::span<const double> ground, double length_m, double z0, double z1, bool portal0, bool portal1,
                                 double max_grade, double needed);

// cover[i] and above[i] at station i (i metres from the chain start, the last station at the chain end); both spans
// the same non-zero length. Per station: a mapped bridge above -> Deck whatever the DEM says (it is DGM1's interpolated
// gap); else cover >= clearance + roof -> Tube; else a ground road or railway above -> ImpliedDeck (counted as a
// conflict for roads); else Slab. Then: interior Slab / ImpliedDeck gaps shorter than min_gap_m between two tube runs
// become tube (exposed roof: the way above stands on the tube roof); then tube runs shorter than min_tube_m take their
// per-station non-tube kind back. Deck runs never change. A tube run is Bored when it is at least bored_min_length_m long
// and the median of natural_cover (ground minus the portal chord: the terrain that stands above the straight line
// between the portals, i.e. "a lot of terrain above" in the owner's sense) over the run is at least bored_min_cover_m;
// else Box. A cover within 1e-6 m below clearance + roof counts as enough (the design floor yields exactly that).
Classification classify(std::span<const double> cover, std::span<const double> natural_cover, std::span<const Above> above, const Thresholds& t);

const char* to_string(RunKind k);
const char* to_string(ChainClass c);
const char* to_string(TubeShape s);
const char* to_string(Above a);

} // namespace g2m::structure
