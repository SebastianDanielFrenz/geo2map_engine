#include "g2m/layer/road_surface.h"
#include <algorithm>
#include <limits>
namespace g2m {

double road_crown_m(CrownKind kind, double crossfall, double d, double crown_round_m) noexcept {
    switch (kind) {
    case CrownKind::OneSidedRight: return crossfall * d;
    case CrownKind::OneSidedLeft: return -crossfall * d;
    case CrownKind::Roof: break;
    }
    const double a = d < 0 ? -d : d;
    const double rho = a >= crown_round_m ? a : d * d / (2.0 * crown_round_m) + crown_round_m / 2.0;
    return -crossfall * rho;
}

double road_verge_m(const CarveFootprint& f, double t, double verge_drop_m) noexcept {
    double inner = 0, outer = 0, past = 0;
    if (t > f.paved_left) { inner = f.paved_left; outer = f.outer_left; past = t - f.paved_left; }
    else if (t < f.paved_right) { inner = f.paved_right; outer = f.outer_right; past = f.paved_right - t; }
    else return 0.0;
    const double width = outer > inner ? outer - inner : inner - outer;
    if (!(width > 0.0)) return 0.0;
    return -verge_drop_m * road_smooth_step(past / width);
}

double road_surface_height_m(const RoadGeomTile& tile, std::size_t entry, double s, double t,
    const RoadSurfaceParams& params) noexcept {
    constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
    if (entry >= tile.entries.size()) return kNaN;
    const RoadGeomEntry& e = tile.entries[entry];
    if (!e.profile || !e.cross_section) return kNaN;
    const RoadProfile& profile = *e.profile;
    const double length = profile.reference.length_m;
    s = std::clamp(s, 0.0, length);
    const auto pose = profile.at(s);
    if (!pose) return kNaN;
    const CarveFootprint f = carve_footprint(*e.cross_section, s);
    t = std::clamp(t, f.outer_right, f.outer_left);
    const double crossfall = static_cast<double>(profile.attributes.crown_per_mille) / 1000.0;
    const double c_t = road_crown_m(f.crown, crossfall, t - f.crown_pivot, params.crown_round_m);
    const double c_0 = road_crown_m(f.crown, crossfall, 0.0 - f.crown_pivot, params.crown_round_m);
    double z = pose->height_m + (c_t - c_0) + road_verge_m(f, t, params.verge_drop_m);

    // Features: sorted by entry, so the entry's run is contiguous.
    const auto& features = tile.features;
    auto it = std::lower_bound(features.begin(), features.end(), entry,
        [](const ProfileFeature& x, std::size_t i) { return x.entry_index < i; });
    for (; it != features.end() && it->entry_index == entry; ++it) {
        double lo = f.carriageway_right, hi = f.carriageway_left;
        if (it->width_mm > 0 && it->kind != ProfileFeatureKind::Cushion) {
            const double mid = (lo + hi) / 2.0, half = static_cast<double>(it->width_mm) / 2000.0;
            lo = std::max(lo, mid - half); hi = std::min(hi, mid + half);
        }
        if (t >= lo && t <= hi) z += feature_height_contribution_m(*it, s);
    }
    return z;
}

} // namespace g2m
