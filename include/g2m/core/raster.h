// g2m/core/raster.h - raster storage and its georeference (PLAN.md 3.2,
// 6.1).
//
// Two separate things:
//  - Raster<T>: plain row-major storage, width x height samples, index
//    row * width + col. It knows nothing about geography.
//  - RasterGeoref: where each sample sits on a UTM grid, in INTEGER
//    millimetres, so every lattice offset that occurs in practice (DGM1's
//    half-metre cell centres, 0.2 m orthophotos, 2^L m levels) is exact and
//    two lattices can be compared without any floating point. The sample
//    registration (node vs cell centre) and the row order are explicit
//    fields, never implied.
#pragma once

#include "g2m/core/geo/utm.h"
#include "g2m/core/tile_key.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <span>
#include <vector>

namespace g2m {

template <class T>
class Raster {
public:
    Raster() = default;
    Raster(int width, int height, T fill = T{})
        : width_(width), height_(height),
          data_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height), fill) {}

    int width() const { return width_; }
    int height() const { return height_; }
    std::size_t size() const { return data_.size(); }

    T& at(int col, int row) { return data_[index(col, row)]; }
    const T& at(int col, int row) const { return data_[index(col, row)]; }

    std::span<T> data() { return data_; }
    std::span<const T> data() const { return data_; }

    friend bool operator==(const Raster& a, const Raster& b) {
        return a.width_ == b.width_ && a.height_ == b.height_ && a.data_ == b.data_;
    }

private:
    std::size_t index(int col, int row) const {
        return static_cast<std::size_t>(row) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(col);
    }

    int width_ = 0;
    int height_ = 0;
    std::vector<T> data_;
};

// Heights: int32 in units of 1/256 m (PLAN.md 2.4 "quantised outputs").
using HeightRaster = Raster<std::int32_t>;
// Per-cell class ids (LandClass, SurfaceKind, or a consumer's own ids).
using ClassRaster = Raster<std::uint8_t>;

inline constexpr int kHeightUnitsPerMetre = 256;
inline constexpr std::int32_t kHeightNoData = INT32_MIN;

// round(metres * 256), ties away from zero (std::llround; exact, no
// library rounding choice). Empty for non-finite input or out of int32
// range (|h| > ~8,388 km, or the NoData value itself).
std::optional<std::int32_t> quantise_height(double metres);
// round(metres * 256), ties to even (banker's rounding - nearbyint under
// the default IEEE rounding mode, but computed with only floor/compare so
// it does not depend on the caller's current rounding mode, PLAN.md D7).
// Used by the DGM1 import path (G1c I1 Q3); `quantise_height` (ties away
// from zero) is unchanged and still used elsewhere. Same range/NoData
// contract as `quantise_height`.
std::optional<std::int32_t> quantise_height_half_even(double metres);
// units / 256 (exact).
double dequantise_height(std::int32_t units);

// Where a sample sits relative to the lattice.
enum class SampleRegistration : std::uint8_t {
    // "Pixel is point": sample (c, r) sits ON lattice node c, r. A tile of
    // n cells per side has n + 1 samples per side and shares its border
    // samples with its neighbours (terrain.height: 257 x 257, PLAN.md 3.2).
    Node = 0,
    // "Pixel is area": sample (c, r) stands for the square cell between
    // nodes c and c + 1 and is located at the cell CENTRE, half a spacing
    // in from the node. n cells per side -> n samples. Hessen DGM1 is this:
    // 1 m cells, centres at half-metre UTM coordinates.
    Cell = 1,
};

enum class RowOrder : std::uint8_t {
    SouthToNorth = 0, // row 0 is the southern edge (g2m's own layers)
    NorthToSouth = 1, // row 0 is the northern edge (GeoTIFF/.tfw sources)
};

struct RasterGeoref {
    geo::UtmZone zone;
    // South-west corner of the FOOTPRINT (the lattice node the sample grid
    // starts from; for Cell registration the outer edge of the first cell),
    // on the UTM grid with false offsets, integer millimetres.
    std::int64_t west_mm = 0;
    std::int64_t south_mm = 0;
    std::int64_t spacing_mm = 1000;   // > 0; same on both axes
    SampleRegistration registration = SampleRegistration::Node;
    RowOrder row_order = RowOrder::SouthToNorth;
    int width = 0;                    // samples per row
    int height = 0;                   // rows

    bool valid() const;

    // Footprint extent in whole cells (Node: width - 1, Cell: width).
    std::int64_t cells_x() const;
    std::int64_t cells_y() const;

    // Sample position in HALF-millimetres (exact integers for every valid
    // georef: a Cell-registered centre is half a spacing in).
    std::int64_t sample_easting_half_mm(int col) const;
    std::int64_t sample_northing_half_mm(int row) const;
    // The same in metres: correctly rounded, and exact whenever the
    // position is a dyadic fraction of a metre (0.5 m, 0.25 m, ...).
    double sample_easting_m(int col) const;
    double sample_northing_m(int row) const;

    friend bool operator==(const RasterGeoref&, const RasterGeoref&) = default;
};

// The georef of a g2m tile: footprint = the tile's square, cells_per_side
// cells of size tile.size_m() / cells_per_side (must divide exactly in mm).
// Node registration -> (cells + 1)^2 samples, Cell -> cells^2.
std::optional<RasterGeoref> tile_georef(const TileKey& tile, int cells_per_side, SampleRegistration registration,
                                        RowOrder row_order = RowOrder::SouthToNorth);

// If every sample of `b` sits exactly on a sample position of `a`'s
// (infinitely extended) lattice, returns (dcol, drow) such that b's sample
// (c, r) is at a's sample position (c + dcol, r + drow), row indices in each
// raster's own row order. Empty if the lattices differ (zone, spacing, or a
// sub-spacing offset such as node vs cell centre): copying between them
// would require resampling.
struct LatticeOffset {
    std::int64_t dcol;
    std::int64_t drow;
    bool row_flip; // true if the two row orders differ: b's row r maps to a's row (drow - r)
};
std::optional<LatticeOffset> lattice_offset(const RasterGeoref& a, const RasterGeoref& b);

} // namespace g2m
