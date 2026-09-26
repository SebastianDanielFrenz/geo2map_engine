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
#include "g2m/core/result.h"
#include "g2m/decode/byte_source.h"
#include "g2m/decode/zip/zip_reader.h"

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
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

// A reference to one specific copy of a DGM1 tile inside a Catalog: stable
// for that Catalog's lifetime, cheap to copy/store (three small indices, no
// open handle), and enough to re-locate the exact ZipEntry (Catalog::decode)
// or format a human-readable path (Catalog::describe).
struct CopyRef {
    int e_km = 0;
    int n_km = 0;
    std::uint16_t outer_idx = 0; // index into the Catalog's own outer-zip list (sorted by basename)
    std::uint16_t inner_idx = 0; // index into that outer's own inner-zip list (central-directory order)
    std::uint32_t entry_idx = 0; // index into that inner zip's own entries() list (the .tif entry)
    std::string tif_name;        // raw entry name (not UTF-8 - see zip_reader.h), for describe()/debug only
    // index into the same inner zip's entries() list of the sibling .tfw
    // file (same basename, case-insensitive), if one was found alongside
    // the .tif at open() time - absent for a copy shipped without one
    // (this module's own header comment: "some copies carry no usable
    // GeoTIFF GeoKeys/tiepoint at all", and evidently sometimes no .tfw
    // either - the filename-derived georef is authoritative regardless).
    std::optional<std::uint32_t> tfw_entry_idx;

    friend bool operator==(const CopyRef&, const CopyRef&) = default;
};

struct CatalogStats {
    std::size_t outer = 0;
    std::size_t inner = 0;
    std::size_t copies = 0;
    std::size_t unique_keys = 0;
    std::size_t duplicated_keys = 0; // unique_keys with more than one copy
};

// A pre-parsed index over a set of outer PKT_*.zip files (PLAN.md G1c I3):
// every outer and inner central directory is parsed exactly ONCE at open()
// time (not per tile), so repeated decode() calls - even for different
// municipalities/tiles - never re-walk a ZIP directory. This is the fix for
// the "2.5 tiles/s" cost enumerate_tiles()+decode_tile() pay when called
// per-tile (each decode_tile() call above re-opens and re-parses the outer
// AND inner zip from scratch): a Catalog keeps every inner ZipReader (and
// the outer FileByteSource each is a zero-copy SubrangeByteSource over -
// PLAN.md D8) alive and immutable for its own lifetime, so decode() only
// ever does the one unavoidable step, inflating that tile's own TIFF.
//
// Thread-safe after open(): copies()/describe()/decode()/stats() may be
// called concurrently from any number of threads (every inner ZipReader is
// read-only once built, and FileByteSource itself is internally
// mutex-guarded - see byte_source.h). open() itself is not meant to be
// called concurrently with anything.
class Catalog {
public:
    // Parses every outer zip's (and every inner zip's) central directory.
    // `outer_zips` is sorted by filename (basename) before use, independent
    // of the caller's own ordering, so CopyRef::outer_idx and every
    // ordering this class exposes are a pure function of the zip set, not
    // of argv/directory-listing order. Returns an error (never throws) for
    // an unreadable file or a malformed ZIP structure.
    static Result<Catalog> open(std::vector<std::filesystem::path> outer_zips);

    Catalog(Catalog&&) noexcept;
    Catalog& operator=(Catalog&&) noexcept;
    Catalog(const Catalog&) = delete;
    Catalog& operator=(const Catalog&) = delete;
    ~Catalog();

    // Every copy of tile (e_km, n_km) found across every outer zip, sorted
    // by (outer basename, inner index, entry index) - empty if this key was
    // never seen (no DGM1 coverage at that km square, not an error).
    std::vector<CopyRef> copies(int e_km, int n_km) const;

    // Human-readable "outer.zip!inner.zip!tif_name" path, for logging/
    // diagnostics. "<invalid copy ref>" for a CopyRef this Catalog did not
    // produce (out-of-range indices).
    std::string describe(const CopyRef& ref) const;

    // Decodes one copy's 1000x1000 float32 sample grid, row-major, row 0 =
    // north (matching decode_tile()'s own DecodedTile::heights convention -
    // NoData preserved verbatim as -9999.0f). Thread-safe: builds its own
    // TiffReader over a private MemoryByteSource/SubrangeByteSource: no
    // state shared with a concurrent decode() call. Returns an error
    // (never throws) for an out-of-range CopyRef or a malformed tile.
    Result<std::vector<float>> decode(const CopyRef& ref) const;

    // Decompresses and returns the raw text of a copy's sibling .tfw entry
    // (see CopyRef::tfw_entry_idx). ErrorCode::NotFound if this copy has
    // none (not an error condition for the tile itself - see the field's own
    // comment); ErrorCode::InvalidArgument for an out-of-range CopyRef.
    Result<std::string> tfw_text(const CopyRef& ref) const;

    CatalogStats stats() const;

private:
    Catalog();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

} // namespace g2m::decode::dgm1
