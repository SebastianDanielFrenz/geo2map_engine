#include "g2m/layer/portal_approach.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <cmath>
#include <vector>
namespace g2m {
std::optional<PortalApproach> portal_approach(const RoadDemSampler& dem, geo::UtmZone zone, std::span<const geom::PointMm> polyline) {
    if (polyline.size() < 2) return std::nullopt;
    struct Sample { double d; geom::PointMm p; std::int32_t h; };
    std::vector<Sample> samples;
    double walked = 0;   // metres along the polyline up to polyline[i]
    std::size_t i = 0;
    for (double d = kPortalApproachNearM; d <= kPortalApproachFarM + 1e-9; d += 1.0) {
        // Advance to the segment holding arc length d.
        double seg = 0;
        for (; i + 1 < polyline.size(); ++i) {
            const double dx = static_cast<double>(polyline[i + 1].x - polyline[i].x) / 1000.0, dy = static_cast<double>(polyline[i + 1].y - polyline[i].y) / 1000.0;
            seg = det::sqrt(dx * dx + dy * dy);
            if (walked + seg >= d) break;
            walked += seg;
        }
        if (i + 1 >= polyline.size()) break;   // the way ends before d
        const double t = seg > 0 ? (d - walked) / seg : 0;
        const geom::PointMm p{std::llround(static_cast<double>(polyline[i].x) + t * static_cast<double>(polyline[i + 1].x - polyline[i].x)),
                              std::llround(static_cast<double>(polyline[i].y) + t * static_cast<double>(polyline[i + 1].y - polyline[i].y))};
        const auto h = dem.sample(zone, p); if (!h) return std::nullopt;
        samples.push_back({d, p, *h});
    }
    if (samples.empty()) return std::nullopt;
    PortalApproach out; out.samples = static_cast<int>(samples.size());
    const std::size_t k = std::max<std::size_t>(1, samples.size() / 3);
    if (samples.size() >= 2) {
        double d_lo = 0, d_hi = 0, h_lo = 0, h_hi = 0;
        for (std::size_t j = 0; j < k; ++j) {
            d_lo += samples[j].d; h_lo += samples[j].h;
            d_hi += samples[samples.size() - 1 - j].d; h_hi += samples[samples.size() - 1 - j].h;
        }
        const double run = (d_hi - d_lo) / static_cast<double>(k);
        if (run > 0) out.grade = std::clamp((h_hi - h_lo) / static_cast<double>(k) / 256.0 / run, -kPortalApproachMaxGrade, kPortalApproachMaxGrade);
    }
    // The level AT the node: the median of the samples carried back to the node along the measured grade (a road rising
    // 6 % sits 0.3 m higher 5 m out than at its end). The median keeps one bad sample (a kerb, a ditch) out.
    std::vector<double> levels; levels.reserve(samples.size());
    for (const auto& s : samples) levels.push_back(static_cast<double>(s.h) - out.grade * 256.0 * s.d);
    std::sort(levels.begin(), levels.end());
    out.height = static_cast<std::int32_t>(std::llround(levels[levels.size() / 2]));
    // The direction toward the sample nearest 5 m (the level's own station).
    const auto& mid = *std::min_element(samples.begin(), samples.end(), [](const auto& a, const auto& b) { return std::fabs(a.d - 5.0) < std::fabs(b.d - 5.0); });
    const double dx = static_cast<double>(mid.p.x - polyline[0].x) / 1000.0, dy = static_cast<double>(mid.p.y - polyline[0].y) / 1000.0;
    const double len = det::sqrt(dx * dx + dy * dy);
    if (len < 0.5) return std::nullopt;
    out.dir_x = dx / len; out.dir_y = dy / len;
    return out;
}
}
