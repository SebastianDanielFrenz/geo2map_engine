// src/decode/dgm1.cpp - see g2m/decode/dgm1.h.

#include "g2m/decode/dgm1.h"

#include "g2m/core/geo/utm.h"
#include "g2m/decode/tiff/tiff_reader.h"

#include <algorithm>
#include <cctype>
#include <cstring>

namespace g2m::decode::dgm1 {
namespace {

std::string basename_of(const std::string& raw_name) {
    // Raw entry names use '/' (ZIP convention); tolerate '\\' too in case a
    // producer wrote Windows-style separators.
    std::size_t pos = raw_name.find_last_of("/\\");
    return pos == std::string::npos ? raw_name : raw_name.substr(pos + 1);
}

std::string ascii_lower(const std::string& s) {
    std::string out = s;
    std::transform(out.begin(), out.end(), out.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

bool ends_with_ci(const std::string& lower_s, const char* suffix) {
    const std::size_t n = std::strlen(suffix);
    return lower_s.size() >= n && lower_s.compare(lower_s.size() - n, n, suffix) == 0;
}

// Parses a DGM1 tile filename's basename into (E, N) km. Requires the
// (case-insensitive) "dgm1" prefix and a ".tif"/".tiff" extension.
//
// Real Hessen delivery filenames are "dgm1_<utm zone>_<E>_<N>_<tile size
// km>_<state code>.tif" (e.g. "DGM1_32_480_5545_1_he.tif" - confirmed
// against real downloaded PKT_*.zip data during this milestone, G2M-003):
// E/N are the 2ND and 3RD digit runs after the "dgm1" prefix, not the
// first two - an earlier version of this parser assumed the filename led
// directly with E/N and silently mis-keyed every real tile (reading the
// UTM zone number "32" as E and the true E as N), collapsing ~3000
// distinct tiles down to ~85 bogus "keys" (every real E value, duplicated
// across every N row that happened to share it). Falls back to treating
// exactly two digit runs as (E, N) directly, for tolerance with any
// simpler naming this reader might see (PLAN.md: "some names contain
// spaces" between fields, handled here by simply not caring what the
// separators are).
bool try_parse_dgm1_name(const std::string& raw_name, TileKey& out) {
    const std::string base = basename_of(raw_name);
    const std::string lower = ascii_lower(base);
    if (lower.size() < 4 || lower.compare(0, 4, "dgm1") != 0) return false;
    if (!ends_with_ci(lower, ".tif") && !ends_with_ci(lower, ".tiff")) return false;

    std::vector<int> digit_runs;
    std::size_t i = 4;
    while (i < lower.size()) {
        if (std::isdigit(static_cast<unsigned char>(lower[i])) == 0) {
            ++i;
            continue;
        }
        std::size_t start = i;
        while (i < lower.size() && std::isdigit(static_cast<unsigned char>(lower[i])) != 0) ++i;
        digit_runs.push_back(std::atoi(lower.substr(start, i - start).c_str()));
    }
    if (digit_runs.size() >= 3) {
        out.e_km = digit_runs[1];
        out.n_km = digit_runs[2];
        return true;
    }
    if (digit_runs.size() == 2) {
        out.e_km = digit_runs[0];
        out.n_km = digit_runs[1];
        return true;
    }
    return false;
}

bool is_zip_name(const std::string& raw_name) {
    const std::string lower = ascii_lower(basename_of(raw_name));
    return ends_with_ci(lower, ".zip");
}

} // namespace

std::vector<std::pair<TileKey, TileLocation>> enumerate_tiles(const std::vector<std::string>& zip_paths) {
    std::vector<std::pair<TileKey, TileLocation>> out;
    for (const std::string& outer_path : zip_paths) {
        auto outer_source = std::make_shared<FileByteSource>(outer_path);
        ZipReader outer_reader(outer_source);
        for (const ZipEntry& outer_entry : outer_reader.entries()) {
            if (outer_entry.is_directory || !is_zip_name(outer_entry.name)) continue;
            auto inner_source = outer_reader.read_entry_as_source(outer_entry);
            ZipReader inner_reader(inner_source);
            for (const ZipEntry& inner_entry : inner_reader.entries()) {
                if (inner_entry.is_directory) continue;
                TileKey key;
                if (!try_parse_dgm1_name(inner_entry.name, key)) continue;
                TileLocation loc;
                loc.outer_zip_path = outer_path;
                loc.inner_zip_entry_name = outer_entry.name;
                loc.tif_entry_name = inner_entry.name;
                out.emplace_back(key, std::move(loc));
            }
        }
    }
    return out;
}

DecodedTile decode_tile(std::shared_ptr<IByteSource> outer_zip_source, const TileLocation& location) {
    ZipReader outer_reader(std::move(outer_zip_source));
    const ZipEntry* inner_entry = nullptr;
    for (const ZipEntry& e : outer_reader.entries()) {
        if (e.name == location.inner_zip_entry_name) {
            inner_entry = &e;
            break;
        }
    }
    if (inner_entry == nullptr) {
        throw DecodeError("dgm1: inner zip entry '" + to_lossy_display_name(location.inner_zip_entry_name) +
                           "' not found in outer zip");
    }
    auto inner_source = outer_reader.read_entry_as_source(*inner_entry);
    ZipReader inner_reader(inner_source);
    const ZipEntry* tif_entry = nullptr;
    for (const ZipEntry& e : inner_reader.entries()) {
        if (e.name == location.tif_entry_name) {
            tif_entry = &e;
            break;
        }
    }
    if (tif_entry == nullptr) {
        throw DecodeError("dgm1: tif entry '" + to_lossy_display_name(location.tif_entry_name) +
                           "' not found in inner zip");
    }

    TileKey key;
    if (!try_parse_dgm1_name(location.tif_entry_name, key)) {
        throw DecodeError("dgm1: tif entry name does not match the DGM1 filename convention: '" +
                           to_lossy_display_name(location.tif_entry_name) + "'");
    }

    auto tif_source = inner_reader.read_entry_as_source(*tif_entry);
    tiff::TiffReader reader(tif_source);
    if (reader.ifds().empty()) throw DecodeError("dgm1: tif has no IFDs");
    const tiff::Ifd& ifd = reader.ifds().front();
    if (ifd.width != 1000 || ifd.height != 1000) {
        throw DecodeError("dgm1: expected a 1000x1000 tile, got " + std::to_string(ifd.width) + "x" +
                           std::to_string(ifd.height));
    }

    DecodedTile result;
    result.heights = reader.decode_window_f32(0, 0, 0, 1000, 1000);

    // Georeference derived from the filename's (E, N) key - authoritative
    // for DGM1 regardless of whether this particular copy's TIFF carries
    // usable GeoKeys (see this file's header comment and G2M-002).
    RasterGeoref g;
    g.zone = geo::UtmZone{32, geo::Hemisphere::North}; // EPSG:25832, ETRS89 / UTM zone 32N
    g.west_mm = static_cast<std::int64_t>(key.e_km) * 1'000'000;
    g.south_mm = static_cast<std::int64_t>(key.n_km) * 1'000'000;
    g.spacing_mm = 1000;
    g.registration = SampleRegistration::Cell;
    g.row_order = RowOrder::NorthToSouth;
    g.width = 1000;
    g.height = 1000;
    if (!g.valid()) throw DecodeError("dgm1: derived RasterGeoref is invalid");
    result.georef = g;
    return result;
}

DecodedTile decode_tile(const TileLocation& location) {
    return decode_tile(std::make_shared<FileByteSource>(location.outer_zip_path), location);
}

} // namespace g2m::decode::dgm1
