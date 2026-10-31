// Window-cached geographic float DEM mosaic (Copernicus GLO-30 COG).
#pragma once
#include "g2m/decode/tiff/tiff_reader.h"
#include "g2m/layer/src_elev.h"
#include <list>
#include <optional>

namespace g2m::decode {
// Importer-owned, single-threaded. Source order resolves overlapping samples.
// No network; IByteSource supplies pinned local or range-cached source bytes.
// Mixed longitude spacing is supported, including across latitude boundaries.
class GeographicDem {
public:
  explicit GeographicDem(std::vector<std::shared_ptr<IByteSource>> sources,
                         std::size_t cache_blocks = 8);
  std::optional<double> sample(double latitude, double longitude);
  ElevTile reproject(const RasterGeoref &, double height_adjustment_m = 0);
  std::size_t decoded_blocks() const { return decoded_blocks_; }

private:
  struct Source {
    std::unique_ptr<tiff::TiffReader> reader;
    GeoTransform transform;
    int width = 0, height = 0, block_w = 0, block_h = 0;
    std::optional<float> nodata;
  };
  struct Block {
    std::size_t source;
    int x, y;
    Raster<float> samples;
  };
  std::vector<Source> sources_;
  std::list<Block> cache_;
  std::size_t capacity_, decoded_blocks_ = 0;
  std::optional<std::size_t> owner(double latitude, double longitude) const;
  std::optional<double> pixel(std::size_t source, int x, int y);
  std::optional<double> row(double latitude, double longitude);
};
} // namespace g2m::decode
