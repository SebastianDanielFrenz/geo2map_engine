// tests/fuzz_formats/fuzz_seeds.cpp - valid, small inputs for every fuzz
// target, built from synthetic SplitMix64 data (tests/support/synthetic.h).

#include "fuzz_targets.h"

#include "g2m/layer/layer_id.h"
#include "g2m/layer/osm_clip.h"
#include "g2m/layer/pack.h"
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
    case Target::Pack: return {pack()};
    case Target::Manifest: return manifests();
    case Target::Path: return paths();
    }
    return {};
}

} // namespace g2m_fuzz
