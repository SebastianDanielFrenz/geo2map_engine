// g2m/decode/dgm1_bw.h - Baden-Wuerttemberg DGM1 (LGL BW opengeodata) tile
// enumeration and decoding. Sibling of dgm1.h (the Hessen delivery); both
// feed the same g2m.src.elev.dgm1 layer through import_dem (import/dem.h).
//
// Delivery format (probed 2026-10-07 against 59 real zips / 210 tiles, see
// docs/pipeline/README.md and geo2map_data/SOURCES.md):
//  - one ZIP per 2 km square, "dgm1_32_<E km>_<N km>_2_bw.zip" (E odd, N even),
//    holding a folder "dgm1_32_<E>_<N>_2_bw/" with up to four 1 km tiles
//    "dgm1_32_<E>_<N>_1_bw_<year>.xyz" (+ a .csv metadata row each, a
//    Meta-*.txt, a licence PDF);
//  - a tile is ASCII "X Y Z" text, one sample per line, CRLF, X/Y in metres
//    with two decimals, Z in metres with two decimals (cm precision);
//  - samples are CELL CENTRES (X = E*1000 + 0.5 + col, Y = N*1000 + 999.5 -
//    row), EPSG:25832 (ETRS89 / UTM 32N), heights DHHN2016 normal heights
//    (the same datum the Hessen DGM1 is delivered in), row-major from the
//    north-west corner, rows north to south, west to east;
//  - tiles are cropped at the state border: a border tile lists ONLY the
//    samples that lie inside Baden-Wuerttemberg (e.g. 626214 of 1000000 lines),
//    so a missing sample means "not in this state", not "no data". No explicit
//    NoData value occurs; an interior tile is complete (1000000 lines).
// decode() therefore places every line by its OWN coordinates (never by line
// index), rejects any coordinate off the 1 m cell-centre lattice or outside
// the key's square, rejects a duplicated sample, and returns -9999.0f (the
// Hessen NoData value) for every sample the file does not list.
#pragma once

#include "g2m/core/result.h"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace g2m::decode::dgm1_bw {

// A reference to one specific copy of a 1 km tile (stable for the Catalog's
// lifetime, cheap to copy).
struct CopyRef {
    int e_km = 0;
    int n_km = 0;
    std::uint32_t zip_idx = 0;   // index into the Catalog's own zip list (sorted by basename)
    std::uint32_t entry_idx = 0; // index into that zip's own entries() list (the .xyz entry)
    std::string entry_name;      // raw entry name, for describe()/debug only

    friend bool operator==(const CopyRef&, const CopyRef&) = default;
};

struct CatalogStats {
    std::size_t zips = 0;
    std::size_t copies = 0;
    std::size_t unique_keys = 0;
    std::size_t duplicated_keys = 0; // unique_keys with more than one copy
};

// Parses "dgm1_32_<E>_<N>_1_bw[_<suffix>].xyz" (basename, case-insensitive,
// any directory prefix already stripped) into (E, N) km. nullopt for anything
// else - in particular a 2 km name ("_2_bw"), whose size field is not 1.
struct KmKey {
    int e_km = 0;
    int n_km = 0;
};
bool try_parse_bw_tile_name(std::string_view basename, KmKey& out);

// A pre-parsed index over a set of BW 2 km zips: every central directory is
// parsed ONCE at open() time. Thread-safe after open() (read-only; the file
// source is internally mutex-guarded).
class Catalog {
public:
    // `zips` is sorted by basename before use so every ordering below is a
    // pure function of the file set. An error (never a throw) for an
    // unreadable file or a malformed ZIP.
    static Result<Catalog> open(std::vector<std::filesystem::path> zips);

    Catalog(Catalog&&) noexcept;
    Catalog& operator=(Catalog&&) noexcept;
    Catalog(const Catalog&) = delete;
    Catalog& operator=(const Catalog&) = delete;
    ~Catalog();

    // Every copy of tile (e_km, n_km), sorted by (zip basename, entry index);
    // empty if the key was never seen.
    std::vector<CopyRef> copies(int e_km, int n_km) const;

    // "zip!entry" for logging; "<invalid copy ref>" for a foreign ref.
    std::string describe(const CopyRef& ref) const;

    // Decodes one copy into 1000 x 1000 floats, row-major, row 0 = north,
    // -9999.0f where the file lists no sample (see the header comment).
    // `listed_out`, if given, receives the number of samples the file lists.
    Result<std::vector<float>> decode(const CopyRef& ref, std::size_t* listed_out = nullptr) const;

    CatalogStats stats() const;

private:
    Catalog();
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// The pure text parser decode() is built on, exposed so tests can feed it
// hand-made bytes. `data` is the whole .xyz file; (e_km, n_km) the tile key.
Result<std::vector<float>> parse_xyz_tile(std::string_view data, int e_km, int n_km, std::size_t* listed_out = nullptr);

} // namespace g2m::decode::dgm1_bw
