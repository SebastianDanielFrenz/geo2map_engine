// src/import/dem.cpp - see g2m/import/dem.h.
//
// Per-tile algorithm (PLAN.md G1c I3):
//  1. For a level-2 tile key `k` (1024 m square), its own georef is
//     tile_georef(k, cells_per_side=1024, Cell, NorthToSouth) - Cell
//     registration and NorthToSouth row order chosen to match DGM1's own
//     lattice exactly (1 m cells, row 0 = north), so assembling the tile is
//     a pure row/column COPY, never a resample.
//  2. The tile's footprint touches TWO OR THREE DGM1 1 km squares per axis
//     (1024 m > 1000 m, so it always straddles at least one km boundary;
//     when the tile's west/south edge sits at km-remainder 977..999 the
//     1024 m extent straddles TWO boundaries, touching three squares on
//     that axis, not two - PLAN.md G2.0, dgm1_km_span()/KmSpan in dem.h):
//     e in [floor(west/1000), floor((west+1023)/1000)], n likewise - up to
//     3x3 = 9 distinct (e, n) pairs. An earlier version of this importer
//     assumed "at most two per axis" and only visited the 2x2 box's 4
//     corners, silently skipping the middle square whenever a tile spanned
//     three - producing a NoData stripe through otherwise-covered terrain
//     (found post-import, G2M-005/G2.0; see tests/unit/test_dem_km_span.cpp).
//  3. Each needed (e, n) square is fetched through a KmCache: a bounded LRU
//     of decoded, quantised 1000x1000 squares, keyed by (e, n), with
//     "decode this key exactly once while any thread is waiting on it"
//     semantics via std::shared_future (never a second decode of the same
//     key racing another in flight - PLAN.md's own RSS/perf budget: a naive
//     per-tile decode would re-inflate the same km square up to 4x per
//     adjacent level-2 tile).
//  4. A km square with more than one copies() entry has every copy decoded
//     and compared sample-for-sample; copies()[0] (the earliest by (outer
//     basename, inner index, entry index) - deterministic, not "whichever
//     decoded first") is always the one actually used, and a disagreement
//     is recorded in the report, never silently ignored.
//  5. Missing coverage: a level-2 tile touched by NO available km square is
//     tiles_skipped_nocov (no write); one where every sample nonetheless
//     decodes to NoData is tiles_skipped_allnodata (no write); anything
//     else is written, tiles_partial additionally counting a tile with SOME
//     (not all) NoData samples.
//
//  6. Two sources (2026-10-07, DemSources): a km square may come from the Hessen
//     catalog, the Baden-Wuerttemberg catalog (decode/dgm1_bw.h, -9999 where its
//     cropped file lists no sample) or both. Both -> merged per sample in
//     decode_km: a sample BW lists takes the BW height, the rest the Hessen height
//     (never averaged), with the seam counters in the report. One -> as before.
//
// Determinism (PLAN.md's own "output bytes independent of --jobs/cache
// size"): the SEQUENCE of sink.put() calls a run produces is a pure
// function of (region, catalog) - ordered_parallel.h's own contract
// delivers them to `consume` in ascending tile-index order regardless of
// scheduling, and every per-tile computation (georef math, km decode+quant,
// dup resolution via copies()[0]) is itself independent of thread/cache
// state. Only DemImportReport's own bookkeeping (dup_*/km_decoded/
// km_cache_hits/differing) can vary with `km_cache_tiles`, because a
// smaller cache can decode + re-report the same km square's dup comparison
// more than once in one run (see dem.h's own doc comment) - this is a
// report-only quirk, never visible in the tile output itself.
#include "g2m/import/dem.h"

#include "g2m/core/tile_key.h"
#include "g2m/decode/tiff/tfw.h"
#include "g2m/import/ordered_parallel.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <cmath>
#include <future>
#include <list>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

namespace g2m::import {
namespace {

using g2m::decode::dgm1::Catalog;
using g2m::decode::dgm1::CopyRef;

constexpr int kKmSamples = 1000;
constexpr int kCellsPerSide = 1024; // DGM1 is 1 m cells; a level-2 tile is 1024 m, so 1024 cells/side

// One decoded, quantised 1 km DGM1 square, or a note that no coverage
// exists there at all (`covered == false`, `heights` empty). Immutable
// once built - shared (via shared_ptr, through KmCache's shared_future)
// across every thread that needs this key concurrently.
struct KmEntry {
    bool covered = false;
    std::vector<std::int32_t> heights; // 1000x1000, row-major, row 0 = north; kHeightUnitsPerMetre units
};

// Cross-checks a copy's own .tfw (if it has one) against the filename-
// derived georef every DGM1 tile uses (dgm1.h's own header comment: the
// filename key is authoritative regardless of what the TIFF/tfw itself
// says) - a MISMATCH means this specific copy's delivery is internally
// inconsistent, which is a real data problem worth failing loudly on
// rather than silently trusting one source over the other. Absence of a
// .tfw is not an error (many copies ship without one).
Result<void> check_tfw(const Catalog& catalog, const CopyRef& ref, int e_km, int n_km) {
    Result<std::string> text = catalog.tfw_text(ref);
    if (!text) {
        if (text.error().code == ErrorCode::NotFound) return {};
        return text.error();
    }
    const std::optional<g2m::decode::GeoTransform> tfw = g2m::decode::parse_tfw(*text);
    if (!tfw) {
        return make_error(ErrorCode::Malformed, "dem: " + catalog.describe(ref) + ": unparseable .tfw");
    }
    const double expect_x = static_cast<double>(e_km) * 1000.0 + 0.5;
    const double expect_y = static_cast<double>(n_km) * 1000.0 + 999.5;
    constexpr double kTolM = 1e-6;
    if (std::fabs(tfw->origin_x - expect_x) > kTolM || std::fabs(tfw->origin_y - expect_y) > kTolM ||
        std::fabs(tfw->pixel_width - 1.0) > kTolM || std::fabs(tfw->pixel_height - 1.0) > kTolM) {
        return make_error(ErrorCode::Malformed, "dem: " + catalog.describe(ref) +
                                                     ": .tfw georef disagrees with the filename-derived (" +
                                                     std::to_string(e_km) + ", " + std::to_string(n_km) + ") key");
    }
    return {};
}

Result<std::vector<std::int32_t>> quantise_km(const std::vector<float>& raw, const std::string& what) {
    std::vector<std::int32_t> quantised(raw.size());
    for (std::size_t i = 0; i < raw.size(); ++i) {
        const float v = raw[i];
        if (v == -9999.0f) {
            quantised[i] = kHeightNoData;
            continue;
        }
        const std::optional<std::int32_t> q = quantise_height_half_even(static_cast<double>(v));
        if (!q) return make_error(ErrorCode::Malformed, "dem: " + what + ": sample out of quantisable range");
        quantised[i] = *q;
    }
    return quantised;
}

using MaybeKm = std::optional<std::vector<std::int32_t>>;

// Hessen side of one km square: nullopt if the catalog has no copy; else the
// quantised samples of copies()[0], every other copy compared and counted.
Result<MaybeKm> decode_km_hessen(const Catalog& catalog, int e_km, int n_km, DemImportReport& report,
                                 std::mutex& report_mutex) {
    const std::vector<CopyRef> refs = catalog.copies(e_km, n_km);
    if (refs.empty()) return MaybeKm{};

    if (Result<void> ok = check_tfw(catalog, refs.front(), e_km, n_km); !ok) return ok.error();
    Result<std::vector<float>> first = catalog.decode(refs.front());
    if (!first) return first.error();
    if (first->size() != static_cast<std::size_t>(kKmSamples) * kKmSamples) {
        return make_error(ErrorCode::Malformed,
                          "dem: " + catalog.describe(refs.front()) + ": unexpected sample count");
    }
    Result<std::vector<std::int32_t>> quantised = quantise_km(*first, catalog.describe(refs.front()));
    if (!quantised) return quantised.error();

    if (refs.size() > 1) {
        bool any_differing = false;
        std::string msg = std::to_string(e_km) + "/" + std::to_string(n_km) + ": " + catalog.describe(refs.front());
        for (std::size_t idx = 1; idx < refs.size(); ++idx) {
            if (Result<void> ok = check_tfw(catalog, refs[idx], e_km, n_km); !ok) return ok.error();
            Result<std::vector<float>> other = catalog.decode(refs[idx]);
            if (!other) return other.error();
            if (other->size() != first->size() ||
                !std::equal(other->begin(), other->end(), first->begin())) {
                any_differing = true;
                msg += " != " + catalog.describe(refs[idx]);
            }
        }
        std::lock_guard<std::mutex> lock(report_mutex);
        ++report.dup_keys;
        report.dup_copies += refs.size() - 1;
        if (any_differing) {
            ++report.dup_differing;
            report.differing.push_back(std::move(msg));
        } else {
            ++report.dup_identical;
        }
    }
    return MaybeKm(std::move(*quantised));
}

// BW side: copies()[0]'s samples (a 2 km zip never overlaps another, so a
// second copy of a key would be a delivery error - counted, copies()[0] wins).
Result<MaybeKm> decode_km_bw(const g2m::decode::dgm1_bw::Catalog& catalog, int e_km, int n_km,
                             DemImportReport& report, std::mutex& report_mutex) {
    const auto refs = catalog.copies(e_km, n_km);
    if (refs.empty()) return MaybeKm{};
    Result<std::vector<float>> raw = catalog.decode(refs.front());
    if (!raw) return raw.error();
    Result<std::vector<std::int32_t>> quantised = quantise_km(*raw, catalog.describe(refs.front()));
    if (!quantised) return quantised.error();
    if (refs.size() > 1) {
        std::lock_guard<std::mutex> lock(report_mutex);
        ++report.bw_dup_keys;
    }
    return MaybeKm(std::move(*quantised));
}

Result<std::shared_ptr<const KmEntry>> decode_km(const DemSources& sources, int e_km, int n_km,
                                                  DemImportReport& report, std::mutex& report_mutex) {
    auto entry = std::make_shared<KmEntry>();
    MaybeKm hessen;
    MaybeKm bw;
    if (sources.hessen != nullptr) {
        Result<MaybeKm> r = decode_km_hessen(*sources.hessen, e_km, n_km, report, report_mutex);
        if (!r) return r.error();
        hessen = std::move(*r);
    }
    if (sources.bw != nullptr) {
        Result<MaybeKm> r = decode_km_bw(*sources.bw, e_km, n_km, report, report_mutex);
        if (!r) return r.error();
        bw = std::move(*r);
    }
    if (!hessen && !bw) {
        entry->covered = false;
        return std::shared_ptr<const KmEntry>(std::move(entry));
    }
    entry->covered = true;
    if (hessen && bw) {
        // Per-sample border rule (dem.h, DemSources): a sample BW lists lies inside BW and takes the
        // BW height, every other sample the Hessen height. Never averaged.
        std::uint64_t both = 0, differing = 0;
        std::int64_t max_diff = 0;
        for (std::size_t i = 0; i < bw->size(); ++i) {
            const std::int32_t b = (*bw)[i];
            if (b == kHeightNoData) continue;
            const std::int32_t h = (*hessen)[i];
            if (h != kHeightNoData) {
                ++both;
                if (h != b) {
                    ++differing;
                    max_diff = std::max<std::int64_t>(max_diff, std::llabs(static_cast<std::int64_t>(b) - h));
                }
            }
            (*hessen)[i] = b;
        }
        std::lock_guard<std::mutex> lock(report_mutex);
        ++report.km_merged;
        report.border_samples_both += both;
        report.border_samples_differing += differing;
        report.border_max_abs_diff_units = std::max(report.border_max_abs_diff_units, max_diff);
        entry->heights = std::move(*hessen);
    } else if (bw) {
        {
            std::lock_guard<std::mutex> lock(report_mutex);
            ++report.km_bw_only;
        }
        entry->heights = std::move(*bw);
    } else {
        entry->heights = std::move(*hessen);
    }
    return std::shared_ptr<const KmEntry>(std::move(entry));
}

// Bounded LRU over decoded km squares with "decode a given key at most once
// while it is in flight" semantics (std::shared_future - allowed under the
// TSan-clean thread-safety rule alongside std::mutex/condition_variable).
// Eviction from the map never invalidates a shared_future a caller already
// holds a copy of, so it is safe to do under the same lock that serialises
// map access.
class KmCache {
public:
    KmCache(const DemSources& sources, std::size_t capacity, DemImportReport& report, std::mutex& report_mutex)
        : sources_(sources), capacity_(std::max<std::size_t>(1, capacity)), report_(report),
          report_mutex_(report_mutex) {}

    Result<std::shared_ptr<const KmEntry>> get(int e_km, int n_km) {
        const std::uint64_t key = pack(e_km, n_km);
        std::shared_future<Result<std::shared_ptr<const KmEntry>>> future;
        std::optional<std::promise<Result<std::shared_ptr<const KmEntry>>>> promise;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            const auto it = map_.find(key);
            if (it != map_.end()) {
                order_.splice(order_.begin(), order_, it->second.lru_it);
                future = it->second.future;
            } else {
                promise.emplace();
                Slot slot;
                slot.future = promise->get_future().share();
                order_.push_front(key);
                slot.lru_it = order_.begin();
                future = slot.future;
                map_.emplace(key, std::move(slot));
                while (order_.size() > capacity_) {
                    const std::uint64_t victim = order_.back();
                    order_.pop_back();
                    map_.erase(victim);
                }
            }
        }

        if (promise) {
            {
                std::lock_guard<std::mutex> rl(report_mutex_);
                ++report_.km_decoded;
            }
            promise->set_value(decode_km(sources_, e_km, n_km, report_, report_mutex_));
        } else {
            std::lock_guard<std::mutex> rl(report_mutex_);
            ++report_.km_cache_hits;
        }
        return future.get();
    }

private:
    static std::uint64_t pack(int e_km, int n_km) {
        return (static_cast<std::uint64_t>(static_cast<std::uint32_t>(e_km)) << 32) |
               static_cast<std::uint32_t>(static_cast<std::uint32_t>(n_km));
    }

    struct Slot {
        std::shared_future<Result<std::shared_ptr<const KmEntry>>> future;
        std::list<std::uint64_t>::iterator lru_it;
    };

    const DemSources sources_;
    std::size_t capacity_;
    DemImportReport& report_;
    std::mutex& report_mutex_;

    std::mutex mutex_;
    std::list<std::uint64_t> order_; // most-recently-used at front
    std::unordered_map<std::uint64_t, Slot> map_;
};

struct TileOutcome {
    bool skip_nocov = false;
    bool skip_allnodata = false;
    bool partial = false;
    std::vector<std::uint8_t> encoded_body; // set only when written
};

} // namespace

KmSpan dgm1_km_span(std::int64_t min_m, std::int64_t extent_m) {
    return KmSpan{static_cast<int>(floor_div(min_m, 1000)),
                  static_cast<int>(floor_div(min_m + extent_m - 1, 1000))};
}

Result<DemImportReport> import_dem(const ResolvedRegion& region, const Catalog& catalog, ITileSink& sink,
                                    std::string_view scope, const DemImportOptions& options) {
    DemSources sources;
    sources.hessen = &catalog;
    return import_dem(region, sources, sink, scope, options);
}

Result<DemImportReport> import_dem(const ResolvedRegion& region, const DemSources& sources, ITileSink& sink,
                                    std::string_view scope, const DemImportOptions& options) {
    if (sources.hessen == nullptr && sources.bw == nullptr) {
        return make_error(ErrorCode::InvalidArgument, "dem: import_dem needs at least one elevation source");
    }
    (void)scope; // informational only - see dem.h's own doc comment
    const auto t0 = std::chrono::steady_clock::now();

    const std::vector<TileKey> keys = tile_keys(region);
    DemImportReport report;
    if (keys.empty()) return report;

    unsigned jobs = options.jobs;
    if (jobs == 0) jobs = std::max(1u, std::thread::hardware_concurrency());

    std::mutex report_mutex;
    KmCache cache(sources, options.km_cache_tiles, report, report_mutex);

    const auto produce = [&](std::size_t idx) -> TileOutcome {
        const TileKey& key = keys[idx];
        const std::optional<RasterGeoref> dest_opt =
            tile_georef(key, kCellsPerSide, SampleRegistration::Cell, RowOrder::NorthToSouth);
        if (!dest_opt) {
            throw std::runtime_error("dem: tile_georef failed for a resolved region's own tile key");
        }
        const RasterGeoref& dest_georef = *dest_opt;

        HeightRaster dest(dest_georef.width, dest_georef.height, kHeightNoData);
        bool covered_any = false;

        const std::int64_t west = key.min_easting();
        const std::int64_t south = key.min_northing();
        const KmSpan e_span = dgm1_km_span(west, dest_georef.cells_x());
        const KmSpan n_span = dgm1_km_span(south, dest_georef.cells_y());

        // Every (e, n) pair in the full rectangle, up to 3x3 = 9 - not just
        // the 2x2 box's 4 corners (PLAN.md G2.0; see this file's header
        // comment and dem.h's dgm1_km_span()). e_span/n_span each enumerate
        // a contiguous, non-repeating range, so no de-duplication is needed.
        for (int e_km = e_span.lo; e_km <= e_span.hi; ++e_km) {
            for (int n_km = n_span.lo; n_km <= n_span.hi; ++n_km) {
                Result<std::shared_ptr<const KmEntry>> entry_res = cache.get(e_km, n_km);
                if (!entry_res) throw std::runtime_error(entry_res.error().message);
                const KmEntry& entry = **entry_res;
                if (!entry.covered) continue;
                covered_any = true;

                RasterGeoref src_georef;
                src_georef.zone = dest_georef.zone;
                src_georef.west_mm = static_cast<std::int64_t>(e_km) * 1'000'000;
                src_georef.south_mm = static_cast<std::int64_t>(n_km) * 1'000'000;
                src_georef.spacing_mm = 1000;
                src_georef.registration = SampleRegistration::Cell;
                src_georef.row_order = RowOrder::NorthToSouth;
                src_georef.width = kKmSamples;
                src_georef.height = kKmSamples;

                const std::optional<LatticeOffset> off = lattice_offset(src_georef, dest_georef);
                if (!off) throw std::runtime_error("dem: DGM1 km lattice does not align with the tile lattice");
                if (off->row_flip) {
                    throw std::runtime_error("dem: unexpected row-order flip between DGM1 and tile lattices");
                }

                for (int r = 0; r < dest_georef.height; ++r) {
                    const std::int64_t src_row = r + off->drow;
                    if (src_row < 0 || src_row >= kKmSamples) continue;
                    for (int c = 0; c < dest_georef.width; ++c) {
                        const std::int64_t src_col = c + off->dcol;
                        if (src_col < 0 || src_col >= kKmSamples) continue;
                        dest.at(c, r) = entry.heights[static_cast<std::size_t>(src_row) * kKmSamples +
                                                       static_cast<std::size_t>(src_col)];
                    }
                }
            }
        }

        TileOutcome outcome;
        if (!covered_any) {
            outcome.skip_nocov = true;
            return outcome;
        }
        bool any_nodata = false;
        bool all_nodata = true;
        for (const std::int32_t v : dest.data()) {
            if (v == kHeightNoData) {
                any_nodata = true;
            } else {
                all_nodata = false;
            }
        }
        if (all_nodata) {
            outcome.skip_allnodata = true;
            return outcome;
        }
        outcome.partial = any_nodata;

        ElevTile tile;
        tile.georef = dest_georef;
        tile.heights = std::move(dest);
        Result<TileBody> body = encode_src_elev(tile, options.predictor);
        if (!body) throw std::runtime_error("dem: encode_src_elev: " + body.error().message);
        Result<std::vector<std::uint8_t>> encoded = encode_body(*body);
        if (!encoded) throw std::runtime_error("dem: encode_body: " + encoded.error().message);
        outcome.encoded_body = std::move(*encoded);
        return outcome;
    };

    const auto consume = [&](std::size_t idx, TileOutcome&& outcome) {
        if (outcome.skip_nocov) {
            ++report.tiles_skipped_nocov;
            return;
        }
        if (outcome.skip_allnodata) {
            ++report.tiles_skipped_allnodata;
            return;
        }
        ++report.tiles_written;
        if (outcome.partial) ++report.tiles_partial;

        TileHeader header;
        header.layer = std::string(kSrcElevDgm1Layer);
        header.layer_version = 1;
        header.key = keys[idx];
        header.datum = VerticalDatum::DHHN2016;
        header.height_offset = 0;
        const Result<void> put_result = sink.put(header.layer, keys[idx], header, outcome.encoded_body);
        if (!put_result) throw std::runtime_error("dem: sink.put: " + put_result.error().message);
    };

    try {
        ordered_for(keys.size(), static_cast<int>(jobs), produce, consume);
    } catch (const std::exception& e) {
        return make_error(ErrorCode::Io, std::string("dem: ") + e.what());
    }

    const auto t1 = std::chrono::steady_clock::now();
    report.wall_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    return report;
}

} // namespace g2m::import
