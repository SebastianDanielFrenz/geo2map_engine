// Street fidelity P1 slice 6: the consumer queries of a stored cross-section (design 8.1, 9.1):
// carriageway edges, the shared carve footprint and the lane query for traffic. Pure and allocation-free;
// plain + - * / on doubles over RoadCrossSection::boundaries_at, no libm.
#include "g2m/layer/road_cross_section.h"
#include <array>

namespace g2m {
namespace {

constexpr std::size_t kMaxBands = 32;

bool on_carriageway(LaneKind k) noexcept {
    return k == LaneKind::Driving || k == LaneKind::Turn || k == LaneKind::Bus || k == LaneKind::Cycle || k == LaneKind::Parking;
}

// Boundaries of xs at s into `t` (bands + 1 values). False for an empty or oversized section.
bool boundaries(const RoadCrossSection& xs, double s, std::array<double, kMaxBands + 1>& t) noexcept {
    const std::size_t n = xs.bands.size();
    if (n == 0 || n > kMaxBands) return false;
    return xs.boundaries_at(s, std::span<double>(t.data(), n + 1));
}

// First and last band index (inclusive) satisfying `pred`; false if there is none.
template <class Pred> bool band_range(const RoadCrossSection& xs, Pred pred, std::size_t& first, std::size_t& last) noexcept {
    bool found = false;
    for (std::size_t i = 0; i < xs.bands.size(); ++i) {
        if (!pred(xs.bands[i])) continue;
        if (!found) first = i;
        last = i;
        found = true;
    }
    return found;
}

} // namespace

CarriagewayEdges carriageway_edges(const RoadCrossSection& xs, double s) noexcept {
    std::array<double, kMaxBands + 1> t{};
    if (!boundaries(xs, s, t)) return {};
    std::size_t first = 0, last = 0;
    if (!band_range(xs, [](const CrossSectionBand& b) { return on_carriageway(b.kind); }, first, last)) {
        const double mid = (t[0] + t[xs.bands.size()]) / 2.0; // no carriageway band: a declined shape, degenerate
        return {mid, mid};
    }
    return {t[first], t[last + 1]};
}

CarveFootprint carve_footprint(const RoadCrossSection& xs, double s) noexcept {
    CarveFootprint f;
    std::array<double, kMaxBands + 1> t{};
    if (!boundaries(xs, s, t)) return f;
    const std::size_t n = xs.bands.size();
    f.outer_left = t[0];
    f.outer_right = t[n];

    std::size_t first = 0, last = 0;
    const bool has_carriageway = band_range(xs, [](const CrossSectionBand& b) { return on_carriageway(b.kind); }, first, last);
    if (has_carriageway) {
        f.carriageway_left = t[first];
        f.carriageway_right = t[last + 1];
    } else {
        f.carriageway_left = f.carriageway_right = (t[0] + t[n]) / 2.0;
    }
    const std::size_t carriageway_first = first, carriageway_last = last;

    std::size_t a = 0, b = 0;
    if (band_range(xs, [](const CrossSectionBand& x) { return x.kind != LaneKind::Verge; }, a, b)) {
        f.paved_left = t[a]; f.paved_right = t[b + 1];
    } else {
        f.paved_left = f.paved_right = f.carriageway_left;
    }
    if (band_range(xs, [](const CrossSectionBand& x) { return x.kind != LaneKind::Verge && x.kind != LaneKind::Sidewalk; }, a, b)) {
        f.roadbed_left = t[a]; f.roadbed_right = t[b + 1];
    } else {
        f.roadbed_left = f.roadbed_right = f.carriageway_left;
    }

    if (has_carriageway) {
        for (std::size_t i = 0; i < carriageway_first; ++i) {
            if (xs.bands[i].kind == LaneKind::Sidewalk) { f.has_kerb_left = true; f.kerb_left = t[i + 1]; } // innermost sidewalk wins
        }
        for (std::size_t i = xs.bands.size(); i-- > carriageway_last + 1;) {
            if (xs.bands[i].kind == LaneKind::Sidewalk) { f.has_kerb_right = true; f.kerb_right = t[i]; }
        }
    }

    // Crown (design 8.2).
    if ((xs.flags & kXsOneway) != 0) {
        if ((xs.flags & kXsReversedOneway) != 0) { f.crown = CrownKind::OneSidedLeft; f.crown_pivot = f.carriageway_right; }
        else { f.crown = CrownKind::OneSidedRight; f.crown_pivot = f.carriageway_left; }
    } else {
        f.crown = CrownKind::Roof;
        // The ridge is the boundary where the Backward lanes (left in the way direction) turn into the Forward lanes.
        bool found = false;
        bool seen_backward = false;
        std::size_t counted_first = 0, counted_last = 0;
        const bool has_counted = band_range(xs, [](const CrossSectionBand& x) { return x.counted; }, counted_first, counted_last);
        for (std::size_t i = 0; i < n && !found; ++i) {
            const CrossSectionBand& band = xs.bands[i];
            if (!band.counted) continue;
            if (band.direction == LaneDirection::Backward) seen_backward = true;
            else if (band.direction == LaneDirection::Forward && seen_backward) { f.crown_pivot = t[i]; found = true; }
        }
        if (!found) f.crown_pivot = has_counted ? (t[counted_first] + t[counted_last + 1]) / 2.0 : (f.carriageway_left + f.carriageway_right) / 2.0;
    }
    return f;
}

std::size_t driving_lanes_at(const RoadCrossSection& xs, double s, LaneDirection travel, std::span<LaneQuery> out) noexcept {
    if (travel != LaneDirection::Forward && travel != LaneDirection::Backward) return 0;
    std::array<double, kMaxBands + 1> t{};
    if (!boundaries(xs, s, t)) return 0;
    const std::size_t n = xs.bands.size();
    std::size_t written = 0;
    // Left to right in the travel direction: Forward walks the bands in order, Backward in reverse.
    for (std::size_t k = 0; k < n && written < out.size(); ++k) {
        const std::size_t i = travel == LaneDirection::Forward ? k : n - 1 - k;
        const CrossSectionBand& band = xs.bands[i];
        if (!band.counted || band.kind == LaneKind::Parking) continue;
        if (band.direction != travel && band.direction != LaneDirection::Both) continue;
        const double width = t[i] - t[i + 1];
        if (width < 0.5) continue;
        LaneQuery& q = out[written++];
        q.band = i;
        q.centre_t = (t[i] + t[i + 1]) / 2.0;
        q.width_m = width;
        q.direction = band.direction;
        q.turn = band.turn;
        q.kind = band.kind;
    }
    return written;
}

} // namespace g2m
