#include "g2m/layer/road_dem_sampler.h"
#include <algorithm>
#include <array>
#include <vector>
namespace g2m {
Result<RoadDemSampler> RoadDemSampler::make(std::span<const HeightTile* const> tiles) {
    if (tiles.size() > 4096) return make_error(ErrorCode::LimitExceeded, "road DEM: tile budget");
    RoadDemSampler out;
    for (const auto* tile : tiles) {
        if (!tile || !tile->key.valid() || tile->key.level != 0 || !out.tiles_.emplace(tile->key, tile).second)
            return make_error(ErrorCode::InvalidArgument, "road DEM: invalid/duplicate level-0 tile");
    }
    return out;
}
Result<std::int32_t> RoadDemSampler::cell(geo::UtmZone zone, std::int64_t x, std::int64_t y) const {
    const auto tx = floor_div(x, 256), ty = floor_div(y, 256);
    if (tx < TileKey::kIndexMin || tx > TileKey::kIndexMax || ty < TileKey::kIndexMin || ty > TileKey::kIndexMax)
        return make_error(ErrorCode::LimitExceeded, "road DEM: sample outside tile grid");
    TileKey key{zone, 0, static_cast<std::int32_t>(tx), static_cast<std::int32_t>(ty)};
    const auto it = tiles_.find(key);
    if (it == tiles_.end()) return make_error(ErrorCode::NotFound, "road DEM: missing elevation dependency");
    const auto index = static_cast<std::size_t>((y - ty * 256) * 256 + x - tx * 256);
    const auto value = it->second->h[index];
    if (value == kHeightNoData) return make_error(ErrorCode::NotFound, "road DEM: NoData in footprint");
    return value;
}
Result<std::int32_t> RoadDemSampler::sample(geo::UtmZone zone, geom::PointMm p) const {
    constexpr auto bound = std::int64_t{1} << 62;
    TileKey zone_key{zone, 0, 0, 0};
    if (!zone_key.valid() || p.x <= -bound || p.x >= bound || p.y <= -bound || p.y >= bound)
        return make_error(ErrorCode::InvalidArgument, "road DEM: invalid sample position/zone");
    const auto x = floor_div(p.x - 500, 1000), y = floor_div(p.y - 500, 1000);
    const auto fx = p.x - 500 - x * 1000, fy = p.y - 500 - y * 1000;
    std::int64_t sum = 0;
    for (int dy = 0; dy < 2; ++dy) for (int dx = 0; dx < 2; ++dx) {
        const auto weight = (dx ? fx : 1000 - fx) * (dy ? fy : 1000 - fy);
        if (!weight) continue; // exact cell-centre samples need no zero-weight neighbours
        const auto value = cell(zone, x + dx, y + dy); if (!value) return value.error();
        sum += static_cast<std::int64_t>(*value) * weight;
    }
    constexpr std::int64_t denominator = 1000000;
    auto q = floor_div(sum, denominator); const auto r = sum - q * denominator;
    if (r > denominator / 2 || (r == denominator / 2 && q % 2 != 0)) ++q;
    return static_cast<std::int32_t>(q);
}
Result<std::int32_t> RoadDemSampler::junction_height(geo::UtmZone zone, geom::PointMm p, std::int32_t radius) const {
    const auto plane = junction_plane(zone, p, radius); if (!plane) return plane.error();
    return plane->height;
}
Result<RoadJunctionPlane> RoadDemSampler::junction_plane(geo::UtmZone zone, geom::PointMm p, std::int32_t radius) const {
    constexpr auto bound = std::int64_t{1} << 62;
    if (radius < 1 || radius > 15 || p.x <= -bound + 15000 || p.x >= bound - 15000 ||
        p.y <= -bound + 15000 || p.y >= bound - 15000)
        return make_error(ErrorCode::InvalidArgument, "road DEM: invalid junction footprint");
    std::vector<std::int32_t> heights;
    std::array<std::int32_t, 31 * 31> footprint{};
    const auto index = [](int x, int y) { return static_cast<std::size_t>((y + 15) * 31 + x + 15); };
    for (int y = -radius; y <= radius; ++y) for (int x = -radius; x <= radius; ++x) {
        if (x * x + y * y > radius * radius) continue;
        const auto value = sample(zone, {p.x + x * 1000, p.y + y * 1000}); if (!value) return value.error();
        heights.push_back(*value);
        footprint[index(x, y)] = *value;
    }
    std::sort(heights.begin(), heights.end());
    // Symmetric pairs cancel the perpendicular plane slope. Median rational
    // gradients resist local DEM returns; integer ordering is platform-stable.
    struct Gradient { std::int64_t rise; int run; };
    std::vector<Gradient> gx, gy;
    for (int y = -radius; y <= radius; ++y) for (int x = 1; x <= radius; ++x) {
        if (x * x + y * y > radius * radius) continue;
        gx.push_back({static_cast<std::int64_t>(footprint[index(x, y)]) - footprint[index(-x, y)], 2 * x});
        gy.push_back({static_cast<std::int64_t>(footprint[index(y, x)]) - footprint[index(y, -x)], 2 * x});
    }
    const auto median = [](std::vector<Gradient>& values) {
        std::sort(values.begin(), values.end(), [](const auto& a, const auto& b) { return a.rise * b.run < b.rise * a.run; });
        const auto& g = values[values.size() / 2]; return static_cast<double>(g.rise) / (256 * g.run);
    };
    return RoadJunctionPlane{heights[heights.size() / 2], median(gx), median(gy)};
}
}
