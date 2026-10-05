// tests/fuzz_formats/fuzz_seeds.cpp - valid, small inputs for every fuzz
// target, built from synthetic SplitMix64 data (tests/support/synthetic.h).

#include "fuzz_targets.h"

#include "g2m/layer/layer_id.h"
#include "g2m/layer/osm_clip.h"
#include "g2m/layer/pack.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/region.h"
#include "g2m/layer/release.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/src_landcover.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/protocol.h"

#include "synthetic.h"

#include <array>
#include <cstdio>
#include <cstdlib>
#include <string>

namespace g2m_fuzz {
namespace {

using Bytes = std::vector<std::uint8_t>;

template <class T>
T must(g2m::Result<T> r, const char* what) {
    if (!r) {
        std::fprintf(stderr, "fuzz seeds: %s failed: %s\n", what, r.error().message.c_str());
        std::abort();
    }
    return std::move(r).value();
}

Bytes body(const g2m::TileBody& b) { return must(g2m::encode_body(b), "encode_body"); }

Bytes text(std::string_view s) { return Bytes(s.begin(), s.end()); }

g2m::ReleaseId rid_of(std::uint64_t seed) {
    g2m::ReleaseInputs in;
    in.world_seed = seed;
    in.layers.push_back({std::string(g2m::kSrcLandcoverLayer), 1, {}});
    return must(g2m::compute_release_id(in), "compute_release_id");
}

std::vector<Bytes> elev_bodies() {
    std::vector<Bytes> out;
    out.push_back(body(must(g2m::encode_src_elev(g2m_test::make_elev_tile(1, g2m_test::home_tile(), 32, 50),
                                                 g2m::ElevPredictor::Gradient),
                            "encode_src_elev")));
    out.push_back(body(must(g2m::encode_src_elev(g2m_test::make_elev_tile(2, g2m_test::home_tile(1, 0), 25, 0),
                                                 g2m::ElevPredictor::Left),
                            "encode_src_elev")));
    return out;
}

std::vector<Bytes> landcover_bodies() {
    return {body(must(g2m::encode_src_landcover(g2m_test::make_landcover_tile(3, g2m_test::home_tile())),
                      "encode_src_landcover"))};
}

std::vector<Bytes> osm_bodies() {
    const g2m::osm::OsmData set = g2m_test::make_osm_set(4, g2m_test::home_tile(), 60, 10, 3, 1200);
    return {body(must(g2m::encode_src_osm(must(g2m::clip_osm(set, g2m_test::home_tile()), "clip_osm")),
                      "encode_src_osm"))};
}

Bytes container(std::string_view layer, const g2m::TileKey& key, const Bytes& b, const g2m::ReleaseId& rid,
                int inputs) {
    g2m::TileHeader h;
    h.layer = std::string(layer);
    h.layer_version = 1;
    h.rid = rid;
    h.key = key;
    for (int i = 0; i < inputs; ++i) {
        g2m::ContentHash x{};
        x[0] = static_cast<std::uint8_t>(i + 1);
        h.inputs.push_back(x);
    }
    return must(g2m::assemble_container(h, b), "assemble_container");
}

Bytes pack() {
    const g2m::ReleaseId rid = rid_of(5);
    g2m::PackDirectory dir;
    dir.layer = std::string(g2m::kSrcLandcoverLayer);
    dir.rid = rid;
    dir.pack = g2m::pack_of(g2m_test::home_tile());
    std::array<Bytes, g2m::kPackTiles> containers;
    for (int i : {0, 17, 255}) {
        const g2m::TileKey k = g2m::pack_tile(dir.pack, i);
        const Bytes b = body(must(g2m::encode_src_landcover(g2m_test::make_landcover_tile(100 + static_cast<std::uint64_t>(i), k)),
                                  "encode_src_landcover"));
        auto& e = dir.entries[static_cast<std::size_t>(i)];
        e.state = g2m::PackTileState::Present;
        e.body_hash = g2m::content_hash(b);
        containers[static_cast<std::size_t>(i)] = container(dir.layer, k, b, rid, i % 3);
    }
    dir.entries[3].state = g2m::PackTileState::Unavailable;
    dir.entries[4].state = g2m::PackTileState::Degraded;
    return must(g2m::encode_pack(dir, containers), "encode_pack");
}

std::vector<Bytes> manifests() {
    const g2m::ReleaseId rid = rid_of(6);
    g2m::ReleaseManifest m;
    m.rid = rid;
    m.world_seed = 6;
    m.sources.push_back({"synthetic", "2026-09-25", "dl-de/zero-2-0", "SplitMix64", false});
    g2m::ReleaseLayer l;
    l.desc = g2m::make_raw_landcover_layer();
    l.coverage = g2m::Coverage::from_level2_tiles({g2m_test::home_tile(), g2m_test::home_tile(1, 0), g2m_test::home_tile(3, 2)});
    m.layers.push_back(l);
    g2m::ServerManifest sm;
    sm.server_id = "g2m.integrated";
    sm.default_rid = rid;
    sm.live_releases = {rid, rid_of(7)};
    return {text(g2m::to_json(m)), text(g2m::to_json(sm))};
}

std::vector<Bytes> paths() {
    const g2m::ReleaseId rid = rid_of(8);
    const std::string layer(g2m::kSrcOsmLayer);
    const g2m::TileKey k = g2m_test::home_tile();
    return {
        text(g2m::to_path(g2m::Request{g2m::ManifestRequest{}})),
        text(g2m::to_path(g2m::Request{g2m::ReleaseManifestRequest{rid}})),
        text(g2m::to_path(g2m::Request{g2m::TileRequest{rid, layer, k, std::nullopt}})),
        text(g2m::to_path(g2m::Request{g2m::PackRequest{rid, layer, g2m::pack_of(k)}})),
        text(g2m::to_path(g2m::Request{g2m::HashesRequest{rid, "g2m.src.elev.dgm1", g2m::pack_of(g2m_test::home_tile(-40, -300))}})),
    };
}

std::vector<Bytes> regions() {
    std::vector<Bytes> out;
    out.push_back(text(R"({"format":"g2m.regions/1","regions":[{"name":"home","scheme":"utm32n","halo_m":2000,)"
                        R"("parts":[{"kind":"utm_bbox","west":474000,"south":5560000,"east":480000,"north":5566000},)"
                        R"({"kind":"osm_admin","relation":62650,"expect_name":"Frankfurt am Main","expect_admin_level":6}]}]})"));

    g2m::ResolvedRegion r;
    r.name = "home";
    r.scheme = "utm32n";
    r.level = 2;
    r.halo_m = 2000;
    g2m::ResolvedPart bbox_part;
    bbox_part.kind = g2m::ResolvedPartKind::UtmBbox;
    bbox_part.vertices = 4;
    bbox_part.geometry_hash = 0x0123456789abcdefull;
    r.parts.push_back(bbox_part);
    g2m::ResolvedPart admin_part;
    admin_part.kind = g2m::ResolvedPartKind::OsmAdmin;
    admin_part.id = 62650;
    admin_part.name = "Frankfurt am Main";
    admin_part.admin_level = 6;
    admin_part.vertices = 1842;
    admin_part.geometry_hash = 0xfedcba9876543210ull;
    r.parts.push_back(admin_part);
    r.pbf_name = "hessen-latest.osm.pbf";
    r.pbf_sha256[0] = 0xab;
    r.pbf_sha256[31] = 0xcd;
    r.tiles = {g2m::CoverageRun{100, 200, 210}, g2m::CoverageRun{101, 199, 211}};
    r.interior = {g2m::CoverageRun{100, 202, 208}};
    r.tile_count = 33;
    out.push_back(text(g2m::to_json(r)));
    return out;
}

} // namespace

std::vector<std::vector<std::uint8_t>> seeds(Target target) {
    switch (target) {
    case Target::Container: {
        std::vector<Bytes> out;
        const g2m::ReleaseId rid = rid_of(9);
        const Bytes lc = landcover_bodies().front();
        out.push_back(container(g2m::kSrcLandcoverLayer, g2m_test::home_tile(), lc, rid, 2));
        out.push_back(container(g2m::kSrcOsmLayer, g2m_test::home_tile(), osm_bodies().front(), g2m::ReleaseId{}, 0));
        out.push_back(lc);
        return out;
    }
    case Target::Elev: return elev_bodies();
    case Target::Landcover: return landcover_bodies();
    case Target::Osm: return osm_bodies();
    case Target::Roadgeom:
    case Target::RoadCrossSectionEval: {
        g2m::RoadGraphTile graph; graph.key = g2m_test::home_tile();
        const auto x = graph.key.min_easting() * 1000, y = graph.key.min_northing() * 1000;
        graph.graph.nodes = {{1, {x + 10000, y + 10000}}, {2, {x + 80000, y + 10000}}};
        graph.graph.ways = {{10, 1, {1, 2}, {{"highway", "service"}}}};
        graph.graph.edges = {{10, 0, 1, 2}}; graph.owned_edges = {0};
        g2m::HeightTile height; height.key = graph.key.child(0, 0).child(0, 0); height.h.fill(25600);
        const g2m::HeightTile* views[] = {&height}; auto dem = must(g2m::RoadDemSampler::make(views), "road geom DEM");
        auto tile = must(g2m::build_road_geom_tile(graph, dem), "road geom profile");
        if (!tile.entries[0].cross_section) { std::fprintf(stderr, "fuzz seeds: no cross-section\n"); std::abort(); }
        const auto first_body = must(g2m::encode_road_geom(tile), "road geom encode");
        // The same tile as an encoding-5 body: legacy graph section, the unchanged profile section.
        const auto legacy_graph = must(g2m::encode_road_graph(tile.source, g2m::kRoadGraphEncodingLegacy), "road geom legacy graph");
        const g2m::TileBody legacy{g2m::kRoadGeomEncodingLegacy, 1, {legacy_graph.sections[0], first_body.sections[1]}};
        auto first = body(first_body);
        // Features and a non-trivial cross-section (a turn pocket and a placement offset).
        auto featured = tile; auto& xs = *featured.entries[0].cross_section;
        xs.offset = {g2m::OffsetMode::Ramps, 300, 10000, -200, 20000};
        xs.start_join = {g2m::JoinKind::LaneChange, 11, 3500}; xs.bands.front().turn = g2m::kTurnLeft | g2m::kTurnThrough;
        featured.features.push_back({0, 1, 0.0, g2m::ProfileFeatureKind::Table, true, 6000, 80, 0, g2m::Provenance::Tagged});
        featured.features.push_back({0, 2, featured.entries[0].profile->reference.length_m, g2m::ProfileFeatureKind::Chicane, false, 0, 0, 0, g2m::Provenance::Tagged});
        auto with_features = body(must(g2m::encode_road_geom(featured), "road geom features encode"));
        tile.entries[0].cross_section.reset(); tile.entries[0].cross_section_decline = g2m::CrossSectionDecline{g2m::CrossSectionDeclineCode::NoCountedLane, 0};
        auto cross_decline = body(must(g2m::encode_road_geom(tile), "road geom cross-section decline encode"));
        tile.entries[0].cross_section_decline.reset(); tile.entries[0].profile.reset(); tile.entries[0].decline = g2m::make_error(g2m::ErrorCode::Unsupported, "fixture decline");
        return {std::move(first), body(legacy), std::move(with_features), std::move(cross_decline), body(must(g2m::encode_road_geom(tile), "road geom decline encode"))};
    }
    case Target::Roadgraph: {
        g2m::RoadGraphTile tile;
        tile.key = g2m_test::home_tile();
        const auto x = tile.key.min_easting() * 1000, y = tile.key.min_northing() * 1000;
        tile.graph.nodes = {{1, {x + 1000, y + 2000}}, {2, {x + 80000, y + 3000}}};
        tile.graph.ways = {{10, 3, {1, 2}, {{"bridge", "yes"}, {"highway", "residential"}}}};
        tile.graph.edges = {{10, 0, 1, 2}}; tile.owned_edges = {0};
        auto first = body(must(g2m::encode_road_graph(tile), "encode_road_graph"));
        tile.graph.ways[0].node_ids = {1, 2, 2, 1}; tile.graph.repeated_ref_pairs = 1;
        tile.graph.edges.push_back({10, 2, 2, 1}); tile.owned_edges.push_back(1);
        auto loop = body(must(g2m::encode_road_graph(tile), "encode_road_graph loop"));
        // Encoding 4 (no node tags) and encoding 5 with node tags.
        auto legacy = body(must(g2m::encode_road_graph(tile, g2m::kRoadGraphEncodingLegacy), "encode_road_graph legacy"));
        tile.graph.nodes[0].tags = {{"highway", "crossing"}, {"traffic_calming", "table"}, {"traffic_calming:length", "6"}};
        tile.graph.nodes[1].tags = {{"barrier", "bollard"}};
        return {std::move(first), std::move(loop), std::move(legacy), body(must(g2m::encode_road_graph(tile), "encode_road_graph tags"))};
    }
    case Target::Pack: return {pack()};
    case Target::Manifest: return manifests();
    case Target::Path: return paths();
    case Target::Region: return regions();
    }
    return {};
}

} // namespace g2m_fuzz
