// src/phys/physics_grid.cpp - see g2m/phys/physics_grid.h.
#include "g2m/phys/physics_grid.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace g2m::phys {

PhysicsTileGrid::PhysicsTileGrid(geo::SessionFrame frame, int samples)
    : frame_(std::move(frame)), samples_(samples) {
    if (samples < 2 || samples % 2 != 0) {
        throw std::invalid_argument("PhysicsTileGrid: samples must be even and >= 2");
    }
}

PhysTileIndex PhysicsTileGrid::index_for_local(double x, double y) const {
    const double ts = tile_size_m();
    PhysTileIndex index;
    index.ix = static_cast<std::int32_t>(std::floor((x - 0.5) / ts));
    index.iy = static_cast<std::int32_t>(std::floor((y - 0.5) / ts));
    return index;
}

std::array<double, 2> PhysicsTileGrid::origin_local(PhysTileIndex index) const {
    const double ts = tile_size_m();
    return {ts * static_cast<double>(index.ix) + 0.5, ts * static_cast<double>(index.iy) + 0.5};
}

std::int64_t PhysicsTileGrid::pack(PhysTileIndex index) {
    const auto ux = static_cast<std::uint32_t>(index.ix);
    const auto uy = static_cast<std::uint32_t>(index.iy);
    return static_cast<std::int64_t>((static_cast<std::uint64_t>(ux) << 32) | uy);
}

PhysTileIndex PhysicsTileGrid::unpack(std::int64_t packed) {
    const auto u = static_cast<std::uint64_t>(packed);
    PhysTileIndex index;
    index.ix = static_cast<std::int32_t>(static_cast<std::uint32_t>(u >> 32));
    index.iy = static_cast<std::int32_t>(static_cast<std::uint32_t>(u & 0xFFFFFFFFu));
    return index;
}

void PhysicsTileGrid::l0_keys_for_square(PhysTileIndex centre, int r, std::vector<TileKey>& out) const {
    out.clear();
    const std::int64_t ts = static_cast<std::int64_t>(samples_ - 1);
    const std::int64_t e0 = frame_.e0_m();
    const std::int64_t n0 = frame_.n0_m();
    const std::int64_t max_k = static_cast<std::int64_t>(samples_ - 1); // last sample offset (== ts)
    const geo::UtmZone zone = frame_.zone();

    for (std::int64_t ix = static_cast<std::int64_t>(centre.ix) - r; ix <= static_cast<std::int64_t>(centre.ix) + r;
         ++ix) {
        const std::int64_t c_min = e0 + ts * ix;
        const std::int64_t c_max = c_min + max_k;
        const std::int64_t x0 = floor_div(c_min, TileKey::kLevel0SizeM);
        const std::int64_t x1 = floor_div(c_max, TileKey::kLevel0SizeM);

        for (std::int64_t iy = static_cast<std::int64_t>(centre.iy) - r;
             iy <= static_cast<std::int64_t>(centre.iy) + r; ++iy) {
            const std::int64_t r_min = n0 + ts * iy;
            const std::int64_t r_max = r_min + max_k;
            const std::int64_t y0 = floor_div(r_min, TileKey::kLevel0SizeM);
            const std::int64_t y1 = floor_div(r_max, TileKey::kLevel0SizeM);

            for (std::int64_t xk = x0; xk <= x1; ++xk) {
                for (std::int64_t yk = y0; yk <= y1; ++yk) {
                    TileKey key;
                    key.zone = zone;
                    key.level = 0;
                    key.x = static_cast<std::int32_t>(xk);
                    key.y = static_cast<std::int32_t>(yk);
                    out.push_back(key);
                }
            }
        }
    }

    std::sort(out.begin(), out.end(), [](const TileKey& a, const TileKey& b) { return a.packed() < b.packed(); });
    out.erase(std::unique(out.begin(), out.end(),
                          [](const TileKey& a, const TileKey& b) { return a.packed() == b.packed(); }),
              out.end());
}

} // namespace g2m::phys
