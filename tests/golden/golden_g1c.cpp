// tests/golden/golden_g1c.cpp - G1c golden hashes: the real import pipeline
// (region resolution + DEM/OSM/landcover import via g2m::import, PLAN.md G1c
// I6 item A) run against the committed fixtures under testdata/fixtures,
// into a MemorySink at jobs=3. Unlike G1 (synthetic per-format inputs), G1c
// exercises the whole pipeline end to end: resolve_region, dgm1::Catalog,
// import_dem, extract_region_set + import_osm, import_landcover - the same
// code path a real importer run takes, just against small fixtures instead
// of the multi-GB real sources.
//
// compute_g1c_for_jobs(jobs) is exposed separately from compute_g1c() (which
// is simply compute_g1c_for_jobs(3)) so a self-check (test_golden.cpp) can
// assert jobs 1 == jobs 3 == jobs 8 give byte-identical entries - the same
// jobs-independence guarantee dem.h/osm.h/landcover.h document for their
// own sink output.

#include "golden.h"

#include "g2m/core/hash.h"
#include "g2m/decode/dgm1.h"
#include "g2m/import/dem.h"
#include "g2m/import/landcover.h"
#include "g2m/import/osm.h"
#include "g2m/import/region_resolve.h"
#include "g2m/import/sink.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/region.h"

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;

// Same shape as golden_g1.cpp's own file-local Acc (deliberately not
// shared - see that file's header comment).
class Acc {
public:
    void bytes(std::span<const std::uint8_t> b) {
        h_.u64(b.size()).bytes(b.data(), b.size());
        ++n_;
    }
    void s(std::string_view v) {
        h_.str(v);
        ++n_;
    }
    void u(std::uint64_t v) {
        h_.u64(v);
        ++n_;
    }
    Entry done(std::string name) const { return {std::move(name), n_, h_.value()}; }

private:
    Fnv1a64 h_;
    std::uint64_t n_ = 0;
};

template <class T>
T must(g2m::Result<T> r, const char* what) {
    if (!r) {
        std::fprintf(stderr, "g1c golden: %s failed: %s\n", what, r.error().message.c_str());
        std::abort();
    }
    return std::move(r).value();
}

std::string fixtures_dir() { return std::string(G2M_SOURCE_DIR) + "/testdata/fixtures"; }

std::string read_file(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in.is_open()) {
        std::fprintf(stderr, "g1c golden: cannot open %s\n", path.c_str());
        std::abort();
    }
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

g2m::RegionSpec find_fixture_region(const std::vector<g2m::RegionSpec>& specs) {
    for (const auto& s : specs) {
        if (s.name == "fixture") return s;
    }
    std::fprintf(stderr, "g1c golden: region 'fixture' not found in regions.json\n");
    std::abort();
}

void hash_resolved(Acc& a, const g2m::ResolvedRegion& r) {
    a.s(r.name);
    a.s(r.scheme);
    a.u(static_cast<std::uint64_t>(r.level));
    a.u(r.halo_m);
    a.s(r.pbf_name);
    a.bytes(std::span<const std::uint8_t>(r.pbf_sha256.data(), r.pbf_sha256.size()));
    a.u(r.tile_count);
    a.u(r.tiles.size());
    for (const g2m::CoverageRun& run : r.tiles) {
        a.u(static_cast<std::uint64_t>(static_cast<std::uint32_t>(run.y)));
        a.u(static_cast<std::uint64_t>(static_cast<std::uint32_t>(run.x0)));
        a.u(static_cast<std::uint64_t>(static_cast<std::uint32_t>(run.x1)));
    }
    a.u(r.interior.size());
    for (const g2m::CoverageRun& run : r.interior) {
        a.u(static_cast<std::uint64_t>(static_cast<std::uint32_t>(run.y)));
        a.u(static_cast<std::uint64_t>(static_cast<std::uint32_t>(run.x0)));
        a.u(static_cast<std::uint64_t>(static_cast<std::uint32_t>(run.x1)));
    }
    a.u(r.parts.size());
    for (const g2m::ResolvedPart& p : r.parts) {
        a.u(static_cast<std::uint64_t>(p.kind));
        a.u(static_cast<std::uint64_t>(p.id));
        a.s(p.name);
        a.u(p.admin_level ? static_cast<std::uint64_t>(*p.admin_level) + 1 : 0);
        a.u(p.vertices);
        a.u(p.geometry_hash);
        a.u(p.role_mismatches);
    }
}

// Snapshot of `sink`'s entries for one layer, sorted (MemorySink::entries()
// is already sorted by (layer, key), so filtering preserves that order).
std::vector<g2m::import::MemorySink::Entry> entries_for(const g2m::import::MemorySink& sink, std::string_view layer) {
    std::vector<g2m::import::MemorySink::Entry> out;
    for (const auto& [addr, entry] : sink.entries()) {
        if (addr.first == layer) out.push_back(entry);
    }
    return out;
}

} // namespace

std::vector<Entry> compute_g1c_for_jobs(unsigned jobs) {
    using namespace g2m::import;

    const std::string fdir = fixtures_dir();
    const std::string regions_json = read_file(fdir + "/regions.json");
    const std::vector<g2m::RegionSpec> specs = must(g2m::parse_regions(regions_json), "parse_regions");
    const g2m::RegionSpec spec = find_fixture_region(specs);
    const std::string pbf_path = fdir + "/osm/frankfurt_hoechst.osm.pbf";
    const g2m::ResolvedRegion resolved = must(resolve_region(spec, pbf_path), "resolve_region");

    std::vector<Entry> out;

    Acc resolved_acc;
    hash_resolved(resolved_acc, resolved);
    out.push_back(resolved_acc.done("g1c.resolved"));

    const g2m::decode::dgm1::Catalog catalog =
        must(g2m::decode::dgm1::Catalog::open({fdir + "/dgm1/PKT_fixture.zip"}), "dgm1::Catalog::open");
    {
        Acc a;
        const g2m::decode::dgm1::CatalogStats st = catalog.stats();
        a.u(st.outer);
        a.u(st.inner);
        a.u(st.copies);
        a.u(st.unique_keys);
        a.u(st.duplicated_keys);
        out.push_back(a.done("g1c.catalog"));
    }

    MemorySink sink;

    DemImportOptions dopts;
    dopts.jobs = jobs;
    const DemImportReport dreport = must(import_dem(resolved, catalog, sink, "g1c", dopts), "import_dem");
    {
        Acc a;
        a.u(dreport.tiles_written);
        a.u(dreport.tiles_partial);
        a.u(dreport.tiles_skipped_nocov);
        a.u(dreport.tiles_skipped_allnodata);
        for (const auto& e : entries_for(sink, g2m::kSrcElevDgm1Layer)) a.bytes(e.body);
        out.push_back(a.done("g1c.dem"));
    }
    {
        Acc a;
        a.u(dreport.dup_keys);
        a.u(dreport.dup_copies);
        a.u(dreport.dup_identical);
        a.u(dreport.dup_differing);
        a.u(dreport.differing.size());
        for (const std::string& d : dreport.differing) a.s(d);
        out.push_back(a.done("g1c.dem.dups"));
    }

    OsmExtractReport ereport;
    const g2m::osm::OsmData region_set =
        must(extract_region_set(pbf_path, resolved, &ereport), "extract_region_set");
    OsmImportOptions oopts;
    oopts.jobs = jobs;
    const OsmImportReport oreport = must(import_osm(resolved, region_set, sink, "g1c", oopts), "import_osm");
    {
        Acc a;
        a.u(ereport.nodes_total);
        a.u(ereport.nodes_in);
        a.u(ereport.ways_in);
        a.u(ereport.ways_incomplete);
        a.u(ereport.relations_in);
        a.u(ereport.index_bytes);
        a.u(oreport.tiles_written);
        a.u(oreport.tiles_empty);
        for (const auto& e : entries_for(sink, g2m::kSrcOsmLayer)) a.bytes(e.body);
        out.push_back(a.done("g1c.osm"));
    }

    const std::vector<std::filesystem::path> tifs = {
        fdir + "/worldcover/fixture_N50E008_hoechst.tif",
        fdir + "/worldcover/fixture_N50E008_bbox.tif",
    };
    LandcoverImportOptions lopts;
    lopts.jobs = jobs;
    const LandcoverImportReport lreport =
        must(import_landcover(resolved, tifs, sink, "g1c", lopts), "import_landcover");
    {
        Acc a;
        a.u(lreport.tiles_written);
        a.u(lreport.tiles_skipped_allzero);
        for (const auto& e : entries_for(sink, g2m::kSrcLandcoverLayer)) a.bytes(e.body);
        out.push_back(a.done("g1c.landcover"));
    }

    Fnv1a64 all;
    std::uint64_t n = 0;
    for (const Entry& e : out) {
        all.str(e.name).u64(e.count).u64(e.hash);
        n += e.count;
    }
    out.push_back({"all", n, all.value()});
    return out;
}

std::vector<Entry> compute_g1c() { return compute_g1c_for_jobs(3); }

} // namespace g2m_golden
