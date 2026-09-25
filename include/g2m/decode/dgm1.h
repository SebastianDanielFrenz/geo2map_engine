// g2m/decode/dgm1.h - Hessen DGM1 tile enumeration/decoding (PLAN.md 13,
// milestone G1b). Ties zip::ZipReader (nested ZIP: an outer PKT_*.zip
// contains inner .zip files, each holding one tile's .tif [+ .tfw]) and
// tiff::TiffReader together behind one small API.
//
// DGM1 facts this module bakes in (G2M-002, confirmed against real
// Hochtaunuskreis/Frankfurt delivery data during this milestone - some
// copies carry no usable GeoTIFF GeoKeys/tiepoint at all, so the tile's
// georeference is derived from its FILENAME'S (E, N) kilometre key, never
// from the TIFF itself):
//  - one 1000 x 1000 float32 sample tile per (E, N) 1 km square, values in
//    metres, NoData == exactly -9999.0f;
//  - Cell registration, NorthToSouth row order, EPSG:25832 (ETRS89 / UTM
//    zone 32N);
//  - the tile's footprint is UTM easting [E*1000, E*1000+1000) x northing
//    [N*1000, N*1000+1000) metres - i.e. RasterGeoref{west_mm = E*1e6,
//    south_mm = N*1e6, spacing_mm = 1000}, matching a row-0 (northernmost
//    row) sample centre at (E*1000 + 0.5, N*1000 + 999.5), the .tfw
//    convention PLAN.md's own DGM1 fact states.
#pragma once

#include "g2m/core/raster.h"
#include "g2m/decode/byte_source.h"
#include "g2m/decode/zip/zip_reader.h"

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace g2m::decode::dgm1 {

struct TileKey {
    int e_km = 0;
    int n_km = 0;
    friend bool operator==(const TileKey&, const TileKey&) = default;
    friend auto operator<=>(const TileKey&, const TileKey&) = default;
};

// Where one copy of a tile lives: an inner .zip entry (by raw name) inside
// an outer PKT_*.zip (by path), and the .tif entry (by raw name) inside
// that inner zip.
struct TileLocation {
    std::string outer_zip_path;
    std::string inner_zip_entry_name; // raw bytes (Hessen names are not UTF-8), ZipReader-encoded
    std::string tif_entry_name;
};

// Enumerates every DGM1 tile found across `zip_paths` (typically the 8
// PKT_*.zip files), returning every COPY found - a tile can legitimately
// appear more than once across different outer zips (each outer PKT_*.zip
// contains exactly one inner "<municipality> - DGM1.zip", and municipality
// footprints overlap at their shared borders). Matches inner-zip entries
// whose .tif member's basename starts with "dgm1"/"DGM1" (case-
// insensitively; any other capitalisation of the 4 letters is rejected as
// unexpected) and ends in ".tif"/".tiff", taking the 2nd and 3rd digit runs
// found in between as (E, N) in km - the real Hessen delivery convention is
// "dgm1_<utm zone>_<E>_<N>_<tile size km>_<state code>.tif" (e.g.
// "DGM1_32_480_5545_1_he.tif" - confirmed against real downloaded data
// during this milestone, G2M-003), so the FIRST digit run is the UTM zone,
// not part of the key. See try_parse_dgm1_name()'s own comment (dgm1.cpp)
// for the exact rule, including its 2-digit-run fallback. Iteration order
// is `zip_paths` order, then each zip's own central-directory order, then
// each inner zip's own central-directory order - fully deterministic,
// never an unordered container.
std::vector<std::pair<TileKey, TileLocation>> enumerate_tiles(const std::vector<std::string>& zip_paths);

struct DecodedTile {
    Raster<float> heights; // 1000 x 1000, raw metres, NoData preserved verbatim (-9999.0f)
    RasterGeoref georef;
};

// Decodes one tile from an already-open outer-zip byte source (so a caller
// looping over many tiles from the same outer zip only opens/parses its
// central directory once).
DecodedTile decode_tile(std::shared_ptr<IByteSource> outer_zip_source, const TileLocation& location);

// Convenience overload: opens `location.outer_zip_path` itself.
DecodedTile decode_tile(const TileLocation& location);

} // namespace g2m::decode::dgm1
