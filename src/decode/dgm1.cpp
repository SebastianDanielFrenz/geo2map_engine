// src/decode/dgm1.cpp - see g2m/decode/dgm1.h.

#include "g2m/decode/dgm1.h"

#include "g2m/core/geo/utm.h"
#include "g2m/decode/tiff/tiff_reader.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <map>

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

// ---- Catalog (PLAN.md G1c I3) ----

namespace {
// Generous safety caps for entry_source()'s DEFLATE branch - every real
// Hessen delivery zip (outer and inner) is STORED (facts confirmed during
// G1b/G1c), so these only matter for a hypothetical DEFLATE-compressed
// input; they exist so a malformed/hostile zip cannot force an unbounded
// allocation rather than because any real file is expected to hit them.
constexpr std::uint64_t kMaxInnerZipInflateBytes = std::uint64_t{2} << 30; // 2 GiB
constexpr std::uint64_t kMaxTifInflateBytes = std::uint64_t{64} << 20;     // 64 MiB (a tile is ~4 MB raw)
} // namespace

struct Catalog::Impl {
    struct OuterZip {
        std::string path;
        std::shared_ptr<FileByteSource> source;
        std::unique_ptr<ZipReader> reader;
    };
    struct InnerZip {
        std::string entry_name; // the outer entry's own raw name (the municipality zip)
        std::shared_ptr<IByteSource> source; // zero-copy SubrangeByteSource (or MemoryByteSource) over `source`
        std::unique_ptr<ZipReader> reader;
    };

    std::vector<OuterZip> outers;
    std::vector<std::vector<InnerZip>> inners; // inners[outer_idx][inner_idx]
    std::map<TileKey, std::vector<CopyRef>> by_key;
    std::size_t total_copies = 0;
};

Catalog::Catalog() = default;
Catalog::Catalog(Catalog&&) noexcept = default;
Catalog& Catalog::operator=(Catalog&&) noexcept = default;
Catalog::~Catalog() = default;

Result<Catalog> Catalog::open(std::vector<std::filesystem::path> outer_zips) {
    std::sort(outer_zips.begin(), outer_zips.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
        return a.filename().string() < b.filename().string();
    });

    auto impl = std::make_unique<Impl>();
    try {
        for (const std::filesystem::path& path : outer_zips) {
            Impl::OuterZip outer;
            outer.path = path.string();
            outer.source = std::make_shared<FileByteSource>(outer.path);
            outer.reader = std::make_unique<ZipReader>(outer.source);
            const auto outer_idx = static_cast<std::uint16_t>(impl->outers.size());

            std::vector<Impl::InnerZip> inner_list;
            for (const ZipEntry& outer_entry : outer.reader->entries()) {
                if (outer_entry.is_directory || !is_zip_name(outer_entry.name)) continue;
                Result<std::shared_ptr<IByteSource>> inner_src =
                    outer.reader->entry_source(outer_entry, kMaxInnerZipInflateBytes);
                if (!inner_src) {
                    return make_error(inner_src.error().code, "dgm1::Catalog::open: inner zip '" +
                                                                    to_lossy_display_name(outer_entry.name) +
                                                                    "' in " + outer.path + ": " +
                                                                    inner_src.error().message);
                }

                Impl::InnerZip inner;
                inner.entry_name = outer_entry.name;
                inner.source = std::move(inner_src.value());
                inner.reader = std::make_unique<ZipReader>(inner.source);
                const auto inner_idx = static_cast<std::uint16_t>(inner_list.size());

                const std::vector<ZipEntry>& candidates = inner.reader->entries();
                std::vector<CopyRef> inner_copies;
                for (std::uint32_t entry_idx = 0; entry_idx < candidates.size(); ++entry_idx) {
                    const ZipEntry& tif_entry = candidates[entry_idx];
                    if (tif_entry.is_directory) continue;
                    TileKey key;
                    if (!try_parse_dgm1_name(tif_entry.name, key)) continue;
                    CopyRef ref;
                    ref.e_km = key.e_km;
                    ref.n_km = key.n_km;
                    ref.outer_idx = outer_idx;
                    ref.inner_idx = inner_idx;
                    ref.entry_idx = entry_idx;
                    ref.tif_name = tif_entry.name;
                    inner_copies.push_back(std::move(ref));
                }
                // Record each copy's sibling .tfw entry (same basename, case-
                // insensitive), if this inner zip ships one - a second pass
                // over the same directory listing (cheap: it is already fully
                // parsed in memory, no I/O), matched by lower-cased basename
                // stem so "dgm1_32_480_5545_1_he.tif" pairs with
                // "DGM1_32_480_5545_1_he.tfw" regardless of case.
                std::map<std::string, std::uint32_t> tfw_by_stem;
                for (std::uint32_t entry_idx = 0; entry_idx < candidates.size(); ++entry_idx) {
                    const ZipEntry& e = candidates[entry_idx];
                    if (e.is_directory) continue;
                    const std::string base = basename_of(e.name);
                    const std::string lower = ascii_lower(base);
                    if (!ends_with_ci(lower, ".tfw")) continue;
                    const std::size_t dot = lower.find_last_of('.');
                    tfw_by_stem[lower.substr(0, dot)] = entry_idx;
                }
                for (CopyRef& ref : inner_copies) {
                    const std::string lower_stem = ascii_lower(basename_of(ref.tif_name));
                    const std::size_t dot = lower_stem.find_last_of('.');
                    const auto it = tfw_by_stem.find(dot == std::string::npos ? lower_stem : lower_stem.substr(0, dot));
                    if (it != tfw_by_stem.end()) ref.tfw_entry_idx = it->second;
                    impl->by_key[TileKey{ref.e_km, ref.n_km}].push_back(std::move(ref));
                    ++impl->total_copies;
                }
                inner_list.push_back(std::move(inner));
            }
            impl->inners.push_back(std::move(inner_list));
            impl->outers.push_back(std::move(outer));
        }
    } catch (const DecodeError& e) {
        return make_error(ErrorCode::Malformed, std::string("dgm1::Catalog::open: ") + e.what());
    }

    Catalog cat;
    cat.impl_ = std::move(impl);
    return cat;
}

std::vector<CopyRef> Catalog::copies(int e_km, int n_km) const {
    const auto it = impl_->by_key.find(TileKey{e_km, n_km});
    if (it == impl_->by_key.end()) return {};
    return it->second; // already in (outer basename, inner index, entry index) order - see open()
}

std::string Catalog::describe(const CopyRef& ref) const {
    if (ref.outer_idx >= impl_->outers.size() || ref.inner_idx >= impl_->inners[ref.outer_idx].size()) {
        return "<invalid copy ref>";
    }
    std::string s = std::filesystem::path(impl_->outers[ref.outer_idx].path).filename().string();
    s += "!" + to_lossy_display_name(impl_->inners[ref.outer_idx][ref.inner_idx].entry_name);
    s += "!" + to_lossy_display_name(ref.tif_name);
    return s;
}

Result<std::vector<float>> Catalog::decode(const CopyRef& ref) const {
    if (ref.outer_idx >= impl_->outers.size() || ref.inner_idx >= impl_->inners[ref.outer_idx].size()) {
        return make_error(ErrorCode::InvalidArgument, "dgm1::Catalog::decode: copy ref out of range");
    }
    const Impl::InnerZip& inner = impl_->inners[ref.outer_idx][ref.inner_idx];
    if (ref.entry_idx >= inner.reader->entries().size()) {
        return make_error(ErrorCode::InvalidArgument, "dgm1::Catalog::decode: copy ref entry index out of range");
    }
    const ZipEntry& tif_entry = inner.reader->entries()[ref.entry_idx];
    try {
        Result<std::shared_ptr<IByteSource>> tif_src = inner.reader->entry_source(tif_entry, kMaxTifInflateBytes);
        if (!tif_src) return tif_src.error();
        tiff::TiffReader reader(tif_src.value());
        if (reader.ifds().empty()) return make_error(ErrorCode::Malformed, "dgm1: tif has no IFDs");
        const tiff::Ifd& ifd = reader.ifds().front();
        if (ifd.width != 1000 || ifd.height != 1000) {
            return make_error(ErrorCode::Malformed, "dgm1: expected a 1000x1000 tile, got " +
                                                          std::to_string(ifd.width) + "x" +
                                                          std::to_string(ifd.height));
        }
        Raster<float> r = reader.decode_window_f32(0, 0, 0, 1000, 1000);
        const auto data = r.data();
        return std::vector<float>(data.begin(), data.end());
    } catch (const DecodeError& e) {
        return make_error(ErrorCode::Malformed, std::string("dgm1::Catalog::decode: ") + e.what());
    }
}

Result<std::string> Catalog::tfw_text(const CopyRef& ref) const {
    if (ref.outer_idx >= impl_->outers.size() || ref.inner_idx >= impl_->inners[ref.outer_idx].size()) {
        return make_error(ErrorCode::InvalidArgument, "dgm1::Catalog::tfw_text: copy ref out of range");
    }
    if (!ref.tfw_entry_idx) {
        return make_error(ErrorCode::NotFound, "dgm1::Catalog::tfw_text: this copy has no .tfw sibling");
    }
    const Impl::InnerZip& inner = impl_->inners[ref.outer_idx][ref.inner_idx];
    if (*ref.tfw_entry_idx >= inner.reader->entries().size()) {
        return make_error(ErrorCode::InvalidArgument, "dgm1::Catalog::tfw_text: tfw entry index out of range");
    }
    try {
        const std::vector<std::byte> bytes = inner.reader->read_entry(inner.reader->entries()[*ref.tfw_entry_idx]);
        std::string text(reinterpret_cast<const char*>(bytes.data()), bytes.size());
        return text;
    } catch (const DecodeError& e) {
        return make_error(ErrorCode::Malformed, std::string("dgm1::Catalog::tfw_text: ") + e.what());
    }
}

CatalogStats Catalog::stats() const {
    CatalogStats s;
    s.outer = impl_->outers.size();
    for (const auto& list : impl_->inners) s.inner += list.size();
    s.copies = impl_->total_copies;
    s.unique_keys = impl_->by_key.size();
    for (const auto& [key, list] : impl_->by_key) {
        if (list.size() > 1) ++s.duplicated_keys;
    }
    return s;
}

} // namespace g2m::decode::dgm1
