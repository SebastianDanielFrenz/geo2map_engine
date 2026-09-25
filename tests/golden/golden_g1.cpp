// tests/golden/golden_g1.cpp - G1 golden hashes: the encoded BYTES of every
// G1 format built from synthetic SplitMix64 inputs (tests/support/
// synthetic.h). Identical bytes on every leg is the G1 acceptance
// criterion ("encoded bytes and golden hashes identical on all 7 legs"),
// which covers the zstd output too (pinned version, level and parameters).

#include "golden.h"

#include "g2m/core/hash.h"
#include "g2m/layer/osm_clip.h"
#include "g2m/layer/pack.h"
#include "g2m/layer/release.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/src_landcover.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/protocol.h"

#include "splitmix64.h"
#include "synthetic.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <string_view>

namespace g2m_golden {
namespace {

using g2m::Fnv1a64;
using g2m_test::SplitMix64;

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

// A golden input that fails to build or encode is a broken test, not a
// hash: stop loudly.
template <class T>
T must(g2m::Result<T> r, const char* what) {
    if (!r) {
        std::fprintf(stderr, "g1 golden: %s failed: %s\n", what, r.error().message.c_str());
        std::abort();
    }
    return std::move(r).value();
}

std::vector<std::uint8_t> body_bytes(const g2m::TileBody& b) { return must(g2m::encode_body(b), "encode_body"); }

void elev(std::vector<Entry>& out) {
    // Three 256^2 tiles and one full 1024^2 DGM1-sized tile, both predictors;
    // then the decoded heights of each (exact round trip).
    struct Case {
        std::uint64_t seed;
        int side;
        int nodata_permille;
    };
    const Case cases[] = {{11, 256, 5}, {12, 256, 0}, {13, 256, 200}, {14, 1024, 5}};
    Acc grad, left, dec;
    for (const Case& c : cases) {
        const g2m::ElevTile t = g2m_test::make_elev_tile(c.seed, g2m_test::home_tile(), c.side, c.nodata_permille);
        const auto g = must(g2m::encode_src_elev(t, g2m::ElevPredictor::Gradient), "encode_src_elev");
        const auto l = must(g2m::encode_src_elev(t, g2m::ElevPredictor::Left), "encode_src_elev");
        const auto gb = body_bytes(g);
        grad.bytes(gb);
        left.bytes(body_bytes(l));
        const auto back = must(g2m::decode_src_elev(must(g2m::decode_body(gb), "decode_body")), "decode_src_elev");
        if (!(back == t)) {
            std::fprintf(stderr, "g1 golden: src.elev round trip is not exact\n");
            std::abort();
        }
        for (int r = 0; r < back.heights.height(); ++r) {
            for (int col = 0; col < back.heights.width(); ++col) dec.u(static_cast<std::uint32_t>(back.heights.at(col, r)));
        }
    }
    out.push_back(grad.done("g1.src_elev.gradient"));
    out.push_back(left.done("g1.src_elev.left"));
    out.push_back(dec.done("g1.src_elev.decoded"));
}

void landcover(std::vector<Entry>& out) {
    Acc a;
    for (std::uint64_t seed = 21; seed < 24; ++seed) {
        const g2m::LandcoverTile t = g2m_test::make_landcover_tile(seed, g2m_test::home_tile(static_cast<std::int32_t>(seed - 21), 0));
        a.bytes(body_bytes(must(g2m::encode_src_landcover(t), "encode_src_landcover")));
    }
    out.push_back(a.done("g1.src_landcover"));
}

void osm(std::vector<Entry>& out) {
    // One synthetic set around the home tile, clipped to it and to its
    // eight neighbours (halo 128 m), each encoded.
    const g2m::osm::OsmData set = g2m_test::make_osm_set(31, g2m_test::home_tile());
    Acc a;
    for (std::int32_t dy = -1; dy <= 1; ++dy) {
        for (std::int32_t dx = -1; dx <= 1; ++dx) {
            const g2m::OsmTile t = must(g2m::clip_osm(set, g2m_test::home_tile(dx, dy)), "clip_osm");
            a.u(t.data.nodes.size());
            a.u(t.data.ways.size());
            a.u(t.data.relations.size());
            a.bytes(body_bytes(must(g2m::encode_src_osm(t), "encode_src_osm")));
        }
    }
    out.push_back(a.done("g1.src_osm"));
}

g2m::ContentHash hash_of(SplitMix64& r) {
    g2m::ContentHash h{};
    for (std::size_t i = 0; i < h.size(); i += 8) {
        const std::uint64_t v = r.next();
        for (std::size_t b = 0; b < 8; ++b) h[i + b] = static_cast<std::uint8_t>(v >> (8 * b));
    }
    return h;
}

void container_release_pack(std::vector<Entry>& out) {
    SplitMix64 r(41);
    // Release id and manifest.
    g2m::ReleaseInputs in;
    in.world_seed = 0x6732'6d00'0000'0001ull;
    for (int i = 0; i < 40; ++i) {
        const std::int32_t dx = static_cast<std::int32_t>(i % 8);
        const std::int32_t dy = static_cast<std::int32_t>(i / 8);
        const std::string layer(i % 2 == 0 ? g2m::kSrcLandcoverLayer : g2m::kSrcOsmLayer);
        const g2m::ContentHash h = hash_of(r);
        in.sources.push_back({layer, g2m_test::home_tile(dx, dy).packed(), h});
    }
    in.layers.push_back({std::string(g2m::kSrcLandcoverLayer), 1, {}});
    in.layers.push_back({std::string(g2m::kSrcOsmLayer), 1, {}});
    const g2m::ContentHash cfg = hash_of(r);
    in.layers.push_back({"rg.race_routes", 2, cfg});
    const g2m::ContentHash table = hash_of(r);
    in.classifiers.push_back({"g2m.default", table});
    const g2m::ReleaseId rid = must(g2m::compute_release_id(in), "compute_release_id");

    g2m::ReleaseManifest m;
    m.rid = rid;
    m.world_seed = in.world_seed;
    m.sources.push_back({"synthetic", "2026-09-25", "dl-de/zero-2-0", "SplitMix64", false});
    std::vector<g2m::TileKey> keys;
    for (int i = 0; i < 40; ++i) keys.push_back(g2m_test::home_tile(static_cast<std::int32_t>(i % 8), static_cast<std::int32_t>(i / 8)));
    for (const auto& d : {g2m::make_raw_landcover_layer(), g2m::make_raw_osm_layer()}) {
        g2m::ReleaseLayer l;
        l.desc = d;
        l.coverage = g2m::Coverage::from_level2_tiles(keys);
        m.layers.push_back(l);
    }
    m.classifiers = in.classifiers;
    Acc rel;
    rel.bytes(rid);
    rel.s(g2m::to_json(m));
    out.push_back(rel.done("g1.release"));

    // Containers and a pack of the landcover tiles.
    g2m::PackDirectory dir;
    dir.layer = std::string(g2m::kSrcLandcoverLayer);
    dir.rid = rid;
    dir.pack = g2m::pack_of(g2m_test::home_tile());
    std::array<std::vector<std::uint8_t>, g2m::kPackTiles> containers;
    Acc cont;
    for (int i = 0; i < g2m::kPackTiles; i += 7) {
        const g2m::TileKey k = g2m::pack_tile(dir.pack, i);
        const g2m::LandcoverTile t = g2m_test::make_landcover_tile(1000 + static_cast<std::uint64_t>(i), k);
        const auto body = body_bytes(must(g2m::encode_src_landcover(t), "encode_src_landcover"));
        g2m::TileHeader h;
        h.layer = dir.layer;
        h.layer_version = 1;
        h.rid = rid;
        h.key = k;
        for (int j = 0; j < i % 4; ++j) h.inputs.push_back(hash_of(r));
        std::sort(h.inputs.begin(), h.inputs.end());
        auto c = must(g2m::assemble_container(h, body), "assemble_container");
        cont.bytes(c);
        g2m::PackEntry& e = dir.entries[static_cast<std::size_t>(i)];
        e.state = g2m::PackTileState::Present;
        e.body_hash = g2m::content_hash(body);
        containers[static_cast<std::size_t>(i)] = std::move(c);
    }
    for (int i = 3; i < g2m::kPackTiles; i += 50) {
        if (i % 7 == 0) continue; // present
        dir.entries[static_cast<std::size_t>(i)].state = g2m::PackTileState::Unavailable;
    }
    out.push_back(cont.done("g1.container"));
    Acc pk;
    pk.bytes(must(g2m::encode_pack(dir, containers), "encode_pack"));
    out.push_back(pk.done("g1.pack"));

    // Protocol: server manifest JSON and request paths.
    Acc proto;
    g2m::ServerManifest sm;
    sm.server_id = "g2m.integrated";
    sm.default_rid = rid;
    sm.live_releases = {rid};
    proto.s(g2m::to_json(sm));
    proto.s(g2m::to_path(g2m::Request{g2m::TileRequest{rid, dir.layer, g2m_test::home_tile(), std::nullopt}}));
    proto.s(g2m::to_path(g2m::Request{g2m::PackRequest{rid, dir.layer, dir.pack}}));
    out.push_back(proto.done("g1.protocol"));
}

} // namespace

std::vector<Entry> compute_g1() {
    std::vector<Entry> out;
    elev(out);
    landcover(out);
    osm(out);
    container_release_pack(out);
    Fnv1a64 all;
    std::uint64_t n = 0;
    for (const Entry& e : out) {
        all.str(e.name).u64(e.count).u64(e.hash);
        n += e.count;
    }
    out.push_back({"all", n, all.value()});
    return out;
}

} // namespace g2m_golden
