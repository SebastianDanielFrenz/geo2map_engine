#pragma once
#include "g2m/layer/height_tile.h"
#include "g2m/core/geom/exact.h"
#include <map>
#include <span>
namespace g2m {
struct RoadJunctionPlane {
    std::int32_t height = 0; // 1/256 m at the original shared node
    double grade_x = 0, grade_y = 0;
};
// Immutable view over caller-owned, decoded level-0 elev.base tiles.
// Heights remain quantised at 1/256 m. No missing/NoData fallback is allowed.
class RoadDemSampler {
public:
    static Result<RoadDemSampler> make(std::span<const HeightTile* const> tiles);
    Result<std::int32_t> sample(geo::UtmZone zone, geom::PointMm position) const;
    // Median of the full integer-metre circular footprint centred on the node,
    // including its centre. A fixed radius gives the same junction height
    // regardless of neighbouring way selection, tile ownership or load order.
    Result<std::int32_t> junction_height(geo::UtmZone zone, geom::PointMm position,
        std::int32_t radius_m = 3) const;
    Result<RoadJunctionPlane> junction_plane(geo::UtmZone zone, geom::PointMm position,
        std::int32_t radius_m = 3) const;
private:
    std::map<TileKey, const HeightTile*> tiles_;
    Result<std::int32_t> cell(geo::UtmZone zone, std::int64_t x, std::int64_t y) const;
};
}
