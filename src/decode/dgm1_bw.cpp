// src/decode/dgm1_bw.cpp - see g2m/decode/dgm1_bw.h.

#include "g2m/decode/dgm1_bw.h"

#include "g2m/decode/byte_source.h"
#include "g2m/decode/zip/zip_reader.h"

#include <algorithm>
#include <cctype>
#include <cstring>
#include <map>

namespace g2m::decode::dgm1_bw {
namespace {

constexpr int kKmSamples = 1000;
constexpr float kNoData = -9999.0f;
constexpr std::uint64_t kMaxXyzBytes = std::uint64_t{256} << 20; // a full 1 km tile is ~28 MB

std::string basename_of(const std::string& raw_name) {
    const std::size_t pos = raw_name.find_last_of("/\\");
    return pos == std::string::npos ? raw_name : raw_name.substr(pos + 1);
}

std::string ascii_lower(std::string_view s) {
    std::string out(s);
    std::transform(out.begin(), out.end(), out.begin(),
                    [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

// A decimal number "[-]digits[.digits]": value = mant / 10^dec. At most 9
// decimals and 18 significant digits - far beyond any real delivery, so an
// overflow cannot happen for input the format allows.
struct Decimal {
    std::int64_t mant = 0;
    int dec = 0;
};

constexpr double kPow10[] = {1.0, 1e1, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7, 1e8, 1e9};

// Parses one number starting at `p` (no leading whitespace); advances `p`
// past it. False if no digits or the shape is wrong.
bool parse_decimal(const char*& p, const char* end, Decimal& out) {
    bool neg = false;
    if (p < end && *p == '-') {
        neg = true;
        ++p;
    }
    std::int64_t mant = 0;
    int digits = 0;
    while (p < end && *p >= '0' && *p <= '9') {
        if (++digits > 18) return false;
        mant = mant * 10 + (*p - '0');
        ++p;
    }
    if (digits == 0) return false;
    int dec = 0;
    if (p < end && *p == '.') {
        ++p;
        while (p < end && *p >= '0' && *p <= '9') {
            if (++digits > 18 || ++dec > 9) return false;
            mant = mant * 10 + (*p - '0');
            ++p;
        }
        if (dec == 0) return false;
    }
    out.mant = neg ? -mant : mant;
    out.dec = dec;
    return true;
}

// The value in millimetres, if it is an exact whole number of them.
bool to_mm(const Decimal& d, std::int64_t& mm) {
    if (d.dec > 3) return false;
    std::int64_t scale = 1;
    for (int i = d.dec; i < 3; ++i) scale *= 10;
    mm = d.mant * scale;
    return true;
}

void skip_blanks(const char*& p, const char* end) {
    while (p < end && (*p == ' ' || *p == '\t')) ++p;
}

} // namespace

bool try_parse_bw_tile_name(std::string_view basename, KmKey& out) {
    const std::string lower = ascii_lower(basename);
    // dgm1_32_<E>_<N>_1_bw<anything>.xyz
    static constexpr char kPrefix[] = "dgm1_32_";
    if (lower.compare(0, sizeof(kPrefix) - 1, kPrefix) != 0) return false;
    if (lower.size() < 4 || lower.compare(lower.size() - 4, 4, ".xyz") != 0) return false;
    std::size_t i = sizeof(kPrefix) - 1;
    int vals[3] = {0, 0, 0};
    for (int f = 0; f < 3; ++f) {
        const std::size_t start = i;
        while (i < lower.size() && lower[i] >= '0' && lower[i] <= '9') ++i;
        if (i == start || i - start > 6) return false;
        vals[f] = std::atoi(lower.substr(start, i - start).c_str());
        if (i >= lower.size() || lower[i] != '_') return false;
        ++i;
    }
    if (vals[2] != 1) return false; // tile size km must be 1
    if (lower.compare(i, 2, "bw") != 0) return false;
    out.e_km = vals[0];
    out.n_km = vals[1];
    return true;
}

Result<std::vector<float>> parse_xyz_tile(std::string_view data, int e_km, int n_km, std::size_t* listed_out) {
    std::vector<float> out(static_cast<std::size_t>(kKmSamples) * kKmSamples, kNoData);
    std::vector<std::uint8_t> seen(out.size(), 0);
    const std::int64_t west_mm = static_cast<std::int64_t>(e_km) * 1'000'000;
    const std::int64_t north_top_mm = static_cast<std::int64_t>(n_km) * 1'000'000 + 999'500; // row 0 centre
    const char* p = data.data();
    const char* const end = p + data.size();
    std::size_t listed = 0;
    std::size_t line_no = 0;
    const auto fail = [&](const char* what) {
        return make_error(ErrorCode::Malformed, std::string("dgm1_bw: line ") + std::to_string(line_no) + ": " + what);
    };
    const auto eat_eol = [&]() {
        if (p < end && *p == '\r') ++p;
        if (p < end && *p == '\n') ++p;
    };
    while (p < end) {
        ++line_no;
        skip_blanks(p, end);
        if (p >= end) break;
        if (*p == '\r' || *p == '\n') { // blank line
            eat_eol();
            continue;
        }
        Decimal x, y, z;
        if (!parse_decimal(p, end, x)) return fail("bad X");
        skip_blanks(p, end);
        if (!parse_decimal(p, end, y)) return fail("bad Y");
        skip_blanks(p, end);
        if (!parse_decimal(p, end, z)) return fail("bad Z");
        skip_blanks(p, end);
        if (p < end && *p != '\r' && *p != '\n') return fail("trailing characters");
        eat_eol();

        std::int64_t x_mm = 0, y_mm = 0;
        if (!to_mm(x, x_mm) || !to_mm(y, y_mm)) return fail("coordinate finer than 1 mm");
        const std::int64_t dx = x_mm - west_mm - 500;
        const std::int64_t dy = north_top_mm - y_mm;
        if (dx < 0 || dy < 0 || dx > 999'000 || dy > 999'000 || dx % 1000 != 0 || dy % 1000 != 0) {
            return fail("sample is not a 1 m cell centre inside the tile's square");
        }
        const std::size_t idx = static_cast<std::size_t>(dy / 1000) * kKmSamples + static_cast<std::size_t>(dx / 1000);
        if (seen[idx] != 0) return fail("duplicate sample");
        seen[idx] = 1;
        out[idx] = static_cast<float>(static_cast<double>(z.mant) / kPow10[z.dec]);
        ++listed;
    }
    if (listed_out != nullptr) *listed_out = listed;
    return out;
}

struct Catalog::Impl {
    struct Zip {
        std::string path;
        std::shared_ptr<FileByteSource> source;
        std::unique_ptr<ZipReader> reader;
    };
    std::vector<Zip> zips;
    std::map<std::pair<int, int>, std::vector<CopyRef>> by_key;
    std::size_t total_copies = 0;
};

Catalog::Catalog() = default;
Catalog::Catalog(Catalog&&) noexcept = default;
Catalog& Catalog::operator=(Catalog&&) noexcept = default;
Catalog::~Catalog() = default;

Result<Catalog> Catalog::open(std::vector<std::filesystem::path> zips) {
    std::sort(zips.begin(), zips.end(), [](const std::filesystem::path& a, const std::filesystem::path& b) {
        return a.filename().string() < b.filename().string();
    });
    auto impl = std::make_unique<Impl>();
    try {
        for (const std::filesystem::path& path : zips) {
            Impl::Zip z;
            z.path = path.string();
            z.source = std::make_shared<FileByteSource>(z.path);
            z.reader = std::make_unique<ZipReader>(z.source);
            const auto zip_idx = static_cast<std::uint32_t>(impl->zips.size());
            const std::vector<ZipEntry>& entries = z.reader->entries();
            for (std::uint32_t entry_idx = 0; entry_idx < entries.size(); ++entry_idx) {
                const ZipEntry& e = entries[entry_idx];
                if (e.is_directory) continue;
                KmKey key;
                if (!try_parse_bw_tile_name(basename_of(e.name), key)) continue;
                CopyRef ref;
                ref.e_km = key.e_km;
                ref.n_km = key.n_km;
                ref.zip_idx = zip_idx;
                ref.entry_idx = entry_idx;
                ref.entry_name = e.name;
                impl->by_key[{key.e_km, key.n_km}].push_back(std::move(ref));
                ++impl->total_copies;
            }
            impl->zips.push_back(std::move(z));
        }
    } catch (const DecodeError& e) {
        return make_error(ErrorCode::Malformed, std::string("dgm1_bw::Catalog::open: ") + e.what());
    }
    Catalog cat;
    cat.impl_ = std::move(impl);
    return cat;
}

std::vector<CopyRef> Catalog::copies(int e_km, int n_km) const {
    const auto it = impl_->by_key.find({e_km, n_km});
    if (it == impl_->by_key.end()) return {};
    return it->second;
}

std::string Catalog::describe(const CopyRef& ref) const {
    if (ref.zip_idx >= impl_->zips.size()) return "<invalid copy ref>";
    return std::filesystem::path(impl_->zips[ref.zip_idx].path).filename().string() + "!" +
           to_lossy_display_name(ref.entry_name);
}

Result<std::vector<float>> Catalog::decode(const CopyRef& ref, std::size_t* listed_out) const {
    if (ref.zip_idx >= impl_->zips.size()) {
        return make_error(ErrorCode::InvalidArgument, "dgm1_bw::Catalog::decode: copy ref out of range");
    }
    const Impl::Zip& z = impl_->zips[ref.zip_idx];
    if (ref.entry_idx >= z.reader->entries().size()) {
        return make_error(ErrorCode::InvalidArgument, "dgm1_bw::Catalog::decode: copy ref entry out of range");
    }
    const ZipEntry& entry = z.reader->entries()[ref.entry_idx];
    if (entry.uncompressed_size > kMaxXyzBytes) {
        return make_error(ErrorCode::LimitExceeded, "dgm1_bw::Catalog::decode: " + describe(ref) + ": entry too large");
    }
    try {
        const std::vector<std::byte> bytes = z.reader->read_entry(entry);
        Result<std::vector<float>> r = parse_xyz_tile(
            std::string_view(reinterpret_cast<const char*>(bytes.data()), bytes.size()), ref.e_km, ref.n_km,
            listed_out);
        if (!r) return make_error(r.error().code, "dgm1_bw: " + describe(ref) + ": " + r.error().message);
        return r;
    } catch (const DecodeError& e) {
        return make_error(ErrorCode::Malformed, std::string("dgm1_bw::Catalog::decode: ") + e.what());
    }
}

CatalogStats Catalog::stats() const {
    CatalogStats s;
    s.zips = impl_->zips.size();
    s.copies = impl_->total_copies;
    s.unique_keys = impl_->by_key.size();
    for (const auto& [key, list] : impl_->by_key) {
        if (list.size() > 1) ++s.duplicated_keys;
    }
    return s;
}

} // namespace g2m::decode::dgm1_bw
