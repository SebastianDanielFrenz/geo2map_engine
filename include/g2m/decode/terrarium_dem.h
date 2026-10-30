#pragma once

#include "g2m/decode/byte_source.h"
#include "g2m/layer/src_elev.h"

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace g2m::decode {

// A decoded XYZ tile with tightly packed row-major RGBA8 bytes. Alpha zero
// means NoData. Image decoding and acquisition belong to the caller.
struct TerrariumTile {
  int z = 0;
  int x = 0;
  int y = 0;
  std::vector<std::uint8_t> rgba;
};

// Inclusive XYZ tile bounds at one zoom. min_x > max_x represents coverage
// that crosses the antimeridian. Y bounds never wrap.
struct TerrariumSourceBounds {
  int z = 0;
  int min_x = 0;
  int max_x = 0;
  int min_y = 0;
  int max_y = 0;
};

// Immutable, bounded Terrarium RGB mosaic sampler. Tile bytes are moved into
// sorted (z,y,x) order and remain engine-neutral decoded data.
class TerrariumDem {
public:
  // `tile_size` defaults to the Terrarium convention of 512. `memory_limit`
  // bounds the total RGBA payload and must be explicit.
  TerrariumDem(std::vector<TerrariumTile> tiles, TerrariumSourceBounds bounds,
               std::size_t memory_limit_bytes, int tile_size = 512);

  // Bilinear interpolation over pixel centres. Missing tiles, alpha-zero
  // pixels, and samples outside supplied coverage return empty.
  std::optional<double> sample(double latitude, double longitude) const;

  // Reproject onto a registered UTM raster, preserving NoData and applying
  // a metre shift before deterministic 1/256 m half-even quantisation.
  ElevTile reproject(const RasterGeoref &georef,
                     double height_shift_m = 0) const;

  int zoom() const { return bounds_.z; }
  int tile_size() const { return tile_size_; }
  std::size_t memory_bytes() const { return memory_bytes_; }

private:
  std::vector<TerrariumTile> tiles_;
  TerrariumSourceBounds bounds_;
  int tile_size_ = 0;
  int world_tiles_ = 0;
  std::size_t memory_bytes_ = 0;

  bool x_in_bounds(int x) const;
  std::optional<double> pixel(std::int64_t global_x,
                              std::int64_t global_y) const;
};

} // namespace g2m::decode
