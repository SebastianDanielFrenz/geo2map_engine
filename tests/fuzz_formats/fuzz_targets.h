// tests/fuzz_formats/fuzz_targets.h - the fuzz target bodies for every
// decoder of file- or server-supplied bytes (PLAN.md 2.4), shared by
//  - the libFuzzer executables fuzz_<name> (G2M_BUILD_FUZZERS, WSL clang
//    legs only; linked against a fuzzer-instrumented copy of the decoders),
//  - g2m_fuzz_smoke_tests (Catch2, every leg): each body over its seed
//    corpus plus deterministic SplitMix64 mutations of it.
//
// A body must never crash, hang or read out of bounds on any input (the
// sanitizers check that), and it checks a round-trip property on every
// input that decodes: re-encoding what was decoded must decode to the same
// value (and, for request paths, give back the exact input). A property
// violation aborts with a message.
#pragma once

#include <cstddef>
#include <cstdint>
#include <span>
#include <string_view>
#include <vector>

namespace g2m_fuzz {

enum class Target {
    Container, // parse_container / decode_header / decode_body (g2m.tile/1)
    Elev,      // decode_body + decode_src_elev
    Landcover, // decode_body + decode_src_landcover
    Osm,       // decode_body + decode_src_osm
    Pack,      // decode_pack
    Manifest,  // release_manifest_from_json + server_manifest_from_json
    Path,      // parse_path (protocol v1 request paths)
    Region,    // parse_regions + parse_resolved_region (g2m/layer/region.h)
    Roadgeom,  // decode_body + decode_road_geom
    Roadgraph, // decode_body + decode_road_graph
};

struct TargetInfo {
    Target target;
    std::string_view name; // fuzz_<name> is the libFuzzer executable
};

inline constexpr TargetInfo kTargets[] = {
    {Target::Container, "container"}, {Target::Elev, "elev"}, {Target::Landcover, "landcover"},
    {Target::Osm, "osm"},             {Target::Pack, "pack"}, {Target::Manifest, "manifest"},
    {Target::Path, "path"},           {Target::Region, "region"}, {Target::Roadgraph, "roadgraph"}, {Target::Roadgeom, "roadgeom"},
};

// Runs one input (fuzz_run.cpp).
void run(Target target, std::span<const std::uint8_t> data);

// Valid inputs to start from (fuzz_seeds.cpp; synthetic, deterministic).
std::vector<std::vector<std::uint8_t>> seeds(Target target);

} // namespace g2m_fuzz
