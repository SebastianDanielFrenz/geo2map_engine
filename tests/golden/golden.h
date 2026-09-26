// tests/golden/golden.h - G0 and G1 golden hashes (PLAN.md 2.4).
//
// Each entry hashes (FNV-1a 64, g2m::Fnv1a64) the bit patterns of one
// function family's outputs over a fixed SplitMix64 input set. The same
// build must give the same hashes on every leg (clang-cl, cl, Linux clang,
// with or without sanitizers); testdata/golden/g{0,1}_golden.txt record them,
// tests/golden/test_golden.cpp checks against it, and tools/ci.ps1 compares
// g2m_golden_dump's output across legs.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace g2m_golden {

struct Entry {
    std::string name;
    std::uint64_t count; // values hashed
    std::uint64_t hash;
};

std::vector<Entry> compute_g0();
// G1 (golden_g1.cpp): the encoded bytes of every G1 format (tile container,
// src.elev with both predictors, src.landcover, src.osm incl. its clip,
// release id + manifest JSON, pack, protocol strings) from synthetic
// inputs; testdata/golden/g1_golden.txt.
std::vector<Entry> compute_g1();

// G1c (golden_g1c.cpp): the real import pipeline (region resolution + DEM/
// OSM/landcover import via g2m::import) run against testdata/fixtures, into
// a MemorySink at jobs=3; testdata/golden/g1c_golden.txt.
std::vector<Entry> compute_g1c();
// Same computation at an explicit `jobs` value - used by test_golden.cpp's
// own jobs-invariance self-check (1 == 3 == 8), since the pipeline's sink
// output is documented job-count-independent (dem.h/osm.h/landcover.h).
std::vector<Entry> compute_g1c_for_jobs(unsigned jobs);

// G2 (golden_g2.cpp, PLAN.md G2.1): the real import -> store -> local
// release -> Server path over testdata/fixtures/dgm1, hashing each
// level-0 g2m.elev.base/g2m.terrain.height target tile's Server-reported
// status and ETag (body content hash); testdata/golden/g2_golden.txt.
std::vector<Entry> compute_g2();

// G2mesh (golden_g2mesh.cpp, PLAN.md G2.3): g2m_mesh's select_chunks +
// build_chunk over 2 fixed camera positions and a deterministic synthetic
// (analytic) HeightTile source - a SEPARATE golden set from "g2" (G2.2
// extends "g2" in parallel); testdata/golden/g2mesh_golden.txt.
std::vector<Entry> compute_g2mesh();

// G2roads (golden_g2roads.cpp, PLAN.md 6.5 / roads_plan.md): g2m::
// extract_road_segments + g2m::rasterize_road_segments over a fixed
// synthetic OsmData set - proves the segment extraction (incl. SurfaceKind /
// paved-vs-unpaved LandClass) and the exact rasteriser are byte-identical
// Windows/Linux; testdata/golden/g2roads_golden.txt.
std::vector<Entry> compute_g2roads();

// "name count hash\n" per entry, the golden file's line format.
std::string format(const std::vector<Entry>& entries);

} // namespace g2m_golden
