// src/layer/structure_classify.cpp - see g2m/layer/structure_classify.h.
#include "g2m/layer/structure_classify.h"

#include <algorithm>
#include <limits>

namespace g2m::structure {
namespace {
RunKind open_kind(Above above) {
    if (above == Above::RailBridge || above == Above::RoadBridge) return RunKind::Deck;
    if (above == Above::RoadGround || above == Above::RailGround) return RunKind::ImpliedDeck;
    return RunKind::Slab;
}
constexpr double kCoverTol = 1e-6;
bool enough(double cover, const Thresholds& t) { return cover >= t.cover_needed() - kCoverTol; }   // NaN -> false
RunKind station_kind(double cover, Above above, const Thresholds& t) {
    if (above == Above::RailBridge || above == Above::RoadBridge) return RunKind::Deck;
    if (enough(cover, t)) return RunKind::Tube;   // NaN cover (no DEM) is never a tube
    return open_kind(above);
}

std::vector<Run> runs_of(const std::vector<RunKind>& kinds) {
    std::vector<Run> runs;
    for (std::size_t i = 0; i < kinds.size(); ++i) {
        if (runs.empty() || runs.back().kind != kinds[i]) runs.push_back(Run{kinds[i], i, i});
        else runs.back().last = i;
    }
    return runs;
}

double length_m(const Run& r) { return static_cast<double>(r.last - r.first); }
} // namespace

std::vector<double> design_floor(std::span<const double> ground, double length_m, double z0, double z1, bool portal0, bool portal1,
                                 double max_grade, double needed) {
    const std::size_t n = ground.size();
    std::vector<double> f(n);
    if (n == 0) return f;
    const auto s_at = [&](std::size_t i) { return std::min(static_cast<double>(i), length_m); };
    // Interior (non-portal) ends are anchored `needed` below the ground there.
    if (!portal0 && ground[0] == ground[0]) z0 = ground[0] - needed;
    if (!portal1 && ground[n - 1] == ground[n - 1]) z1 = ground[n - 1] - needed;
    const double len = length_m > 0 ? length_m : 1.0;
    // u = min(chord, ground - needed): the highest floor that meets (a) and (b). (b) applies only where a floor that
    // leaves a portal at max_grade can be `needed` below that portal's anchor, i.e. at least needed / max_grade from
    // every portal: nearer, the cover grows from the portal face as it does in reality, and deepening there would put a
    // sag into every hill tunnel (the ground falls to the portal faster than the floor may follow).
    const double reach = max_grade > 0 ? needed / max_grade : 0.0;
    for (std::size_t i = 0; i < n; ++i) {
        const double s = s_at(i);
        const double chord = z0 + (z1 - z0) * s / len;
        const double g = ground[i];
        const bool near_portal = (portal0 && s < reach) || (portal1 && length_m - s < reach);
        f[i] = (g == g && !near_portal) ? std::min(chord, g - needed) : chord;
    }
    // (c): the greatest max_grade-Lipschitz profile below u (two-pass distance transform, exact in 1-D).
    for (std::size_t i = 1; i < n; ++i) f[i] = std::min(f[i], f[i - 1] + max_grade * (s_at(i) - s_at(i - 1)));
    for (std::size_t i = n - 1; i-- > 0;) f[i] = std::min(f[i], f[i + 1] + max_grade * (s_at(i + 1) - s_at(i)));
    // Joined to each portal at max_grade: never below the portal cone (a max of Lipschitz profiles stays Lipschitz).
    for (std::size_t i = 0; i < n; ++i) {
        const double s = s_at(i);
        if (portal0) f[i] = std::max(f[i], z0 - max_grade * s);
        if (portal1) f[i] = std::max(f[i], z1 - max_grade * (length_m - s));
    }
    return f;
}

Classification classify(std::span<const double> cover, std::span<const double> natural_cover, std::span<const Above> above, const Thresholds& t) {
    Classification out;
    const std::size_t n = std::min({cover.size(), natural_cover.size(), above.size()});
    if (n == 0) return out;
    std::vector<RunKind> kinds(n);
    for (std::size_t i = 0; i < n; ++i) {
        kinds[i] = station_kind(cover[i], above[i], t);
        if (above[i] == Above::RoadGround && !enough(cover[i], t)) ++out.ground_road_conflicts;
    }
    std::vector<bool> exposed(n, false);
    // 1. Short interior Slab / ImpliedDeck gaps between two tube runs stay tube (the roof is exposed there).
    {
        const auto runs = runs_of(kinds);
        for (std::size_t r = 1; r + 1 < runs.size(); ++r) {
            if (runs[r].kind == RunKind::Tube || runs[r].kind == RunKind::Deck) continue;
            if (runs[r - 1].kind != RunKind::Tube || runs[r + 1].kind != RunKind::Tube) continue;
            // A gap is one run only when it holds one kind; a mixed gap (slab then implied deck) is two runs, left as is.
            // The gap spans from the last tube station before it to the first tube station after it.
            if (static_cast<double>(runs[r + 1].first - runs[r - 1].last) >= t.min_gap_m) continue;
            for (std::size_t i = runs[r].first; i <= runs[r].last; ++i) { kinds[i] = RunKind::Tube; exposed[i] = true; }
        }
    }
    // 2. Tube runs shorter than min_tube_m take their stations' open kind.
    {
        const auto runs = runs_of(kinds);
        for (const auto& r : runs)
            if (r.kind == RunKind::Tube && length_m(r) < t.min_tube_m)
                for (std::size_t i = r.first; i <= r.last; ++i) { kinds[i] = open_kind(above[i]); exposed[i] = false; }
    }
    out.runs = runs_of(kinds);
    bool any_tube = false, any_open = false;
    for (auto& r : out.runs) {
        const auto stats = [&](std::span<const double> v, double* lo, double* med, double* hi) {
            std::vector<double> c;   // NaN (no DEM) is left out; an all-NaN run reports NaN
            for (std::size_t i = r.first; i <= r.last; ++i) if (v[i] == v[i]) c.push_back(v[i]);
            if (c.empty()) { *med = std::numeric_limits<double>::quiet_NaN(); if (lo) *lo = *hi = *med; return; }
            std::sort(c.begin(), c.end());
            *med = c[c.size() / 2];
            if (lo) { *lo = c.front(); *hi = c.back(); }
        };
        stats(cover, &r.min_cover_m, &r.median_cover_m, &r.max_cover_m);
        stats(natural_cover, nullptr, &r.median_natural_cover_m, nullptr);
        for (std::size_t i = r.first; i <= r.last; ++i) r.exposed_roof = r.exposed_roof || exposed[i];
        if (r.kind == RunKind::Tube) {
            any_tube = true;
            r.shape = (length_m(r) >= t.bored_min_length_m && r.median_natural_cover_m >= t.bored_min_cover_m) ? TubeShape::Bored : TubeShape::Box;
            out.portals += 2;
        } else {
            any_open = true;
        }
    }
    out.cls = !any_tube ? ChainClass::NoCover : (any_open ? ChainClass::Mixed : ChainClass::RealCover);
    return out;
}

const char* to_string(RunKind k) {
    switch (k) {
    case RunKind::Tube: return "tube"; case RunKind::Deck: return "deck"; case RunKind::ImpliedDeck: return "implied_deck"; case RunKind::Slab: return "slab";
    }
    return "?";
}
const char* to_string(ChainClass c) {
    switch (c) { case ChainClass::RealCover: return "real_cover"; case ChainClass::NoCover: return "no_cover"; case ChainClass::Mixed: return "mixed"; }
    return "?";
}
const char* to_string(TubeShape s) { return s == TubeShape::Bored ? "bored" : "box"; }
const char* to_string(Above a) {
    switch (a) {
    case Above::Nothing: return "nothing"; case Above::RailBridge: return "rail_bridge"; case Above::RoadBridge: return "road_bridge";
    case Above::Building: return "building"; case Above::RoadGround: return "road_ground"; case Above::RailGround: return "rail_ground";
    case Above::TunnelAbove: return "tunnel_above";
    }
    return "?";
}
} // namespace g2m::structure
