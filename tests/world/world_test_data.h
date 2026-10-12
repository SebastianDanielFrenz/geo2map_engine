// tests/world/world_test_data.h - deterministic synthetic inputs for the
// g2m_world tests (terrain migration plan phase 1, vault G2M-020), and the
// hash helpers the pinned game-parity values are built from.
//
// The pins in test_*.cpp were produced by racing_game's own code (a8a1b13:
// world_terrain.cpp, route_check.cpp, road_classes.cpp - extracted verbatim and
// run over exactly these inputs, see the comment above each pin). Every input
// here comes from SplitMix64 (a specified sequence, one draw per statement) or
// plain integer arithmetic, so every compiler and OS builds the same bytes.
// Exceptions instead of Catch2 assertions: the reference generator that
// produced the pins includes this header too and links no Catch2.
#pragma once

#include "splitmix64.h"
#include "synthetic.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/hash.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/height_tile.h"
#include "g2m/layer/layer_id.h"
#include "g2m/layer/osm_data.h"
#include "g2m/layer/osm_roads.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/terrain_layers.h"
#include "g2m/layer/tile_container.h"
#include "g2m/mesh/terrain_chunk.h"
#include "g2m/server/protocol.h"

#include <cstdint>
#include <cstring>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace g2m_world_test {

inline constexpr g2m::geo::UtmZone kZone = g2m_test::kZone32N;

// The session origin both sampler pins use: on a tile corner (tile x = 1000,
// y = 20000), like racing_game's test_route_check.cpp.
inline constexpr std::int64_t kE0 = 256 * 1000;
inline constexpr std::int64_t kN0 = 256 * 20000;

inline void must(bool ok, const char* what) {
    if (!ok) throw std::runtime_error(what);
}

// ---- hashes ----------------------------------------------------------------

inline std::uint64_t hash_height_tile(const g2m::HeightTile& t) {
    g2m::Fnv1a64 h;
    for (const std::int32_t v : t.h) h.i32(v);
    h.u8(t.has_nodata ? std::uint8_t{1} : std::uint8_t{0});
    h.i32(t.key.x).i32(t.key.y).u8(static_cast<std::uint8_t>(t.key.level));
    return h.value();
}

inline std::uint64_t hash_road_segments(const std::vector<g2m::RoadSegment>& segs) {
    g2m::Fnv1a64 h;
    h.u64(segs.size());
    for (const g2m::RoadSegment& s : segs) {
        h.i64(s.a.x).i64(s.a.y).i64(s.b.x).i64(s.b.y).i64(s.half_width_mm);
        h.u8(static_cast<std::uint8_t>(s.land_class)).u8(static_cast<std::uint8_t>(s.surface));
        h.i32(s.rank).i64(s.way_id).u8(s.bridge ? 1 : 0);
        for (const g2m::RoadSpeedLimit* l : {&s.forward_limit, &s.backward_limit}) {
            h.u8(static_cast<std::uint8_t>(l->kind)).f64(l->kph).u8(l->conditional ? 1 : 0);
        }
    }
    return h.value();
}

inline std::uint64_t bits_of(double v) {
    std::uint64_t u;
    static_assert(sizeof(u) == sizeof(v));
    std::memcpy(&u, &v, sizeof(u));
    return u;
}

// ---- height tiles ----------------------------------------------------------

// A level-0 tile of rough terrain: 100 m +- 39 m of per-sample noise (rough on
// purpose: it makes the two quad diagonals disagree by metres). `nodata_permille`
// of the samples are kHeightNoData.
inline std::unique_ptr<g2m::HeightTile> make_field_tile(std::int32_t tx, std::int32_t ty, int nodata_permille) {
    auto t = std::make_unique<g2m::HeightTile>(); // heap: 256 KiB (TOOL-039)
    t->key = g2m::TileKey{kZone, 0, tx, ty};
    g2m_test::SplitMix64 r(0xA11CE00000000000ull ^ (static_cast<std::uint64_t>(static_cast<std::uint32_t>(tx)) << 24) ^
                           static_cast<std::uint64_t>(static_cast<std::uint32_t>(ty)));
    for (std::size_t i = 0; i < t->h.size(); ++i) {
        const std::uint64_t draw = r.next();
        const int noise = static_cast<int>(draw % 20000) - 10000;
        const int nd = static_cast<int>((draw >> 40) % 1000);
        if (nd < nodata_permille) {
            t->h[i] = g2m::kHeightNoData;
            t->has_nodata = true;
        } else {
            t->h[i] = 100 * 256 + noise;
        }
    }
    return t;
}

// The 3x3 block of level-0 tiles x = 999..1001, y = 19999..20001 around the
// session origin (kE0, kN0).
struct FieldSet {
    std::map<std::pair<std::int32_t, std::int32_t>, std::unique_ptr<g2m::HeightTile>> tiles;
    std::pair<std::int32_t, std::int32_t> missing{-1, -1}; // lookup() answers nullptr for this one

    explicit FieldSet(int nodata_permille) {
        for (std::int32_t ty = 19999; ty <= 20001; ++ty) {
            for (std::int32_t tx = 999; tx <= 1001; ++tx) {
                tiles.emplace(std::make_pair(tx, ty), make_field_tile(tx, ty, nodata_permille));
            }
        }
    }
    const g2m::HeightTile* lookup(const g2m::TileKey& k) const {
        const auto key = std::make_pair(k.x, k.y);
        if (k.level != 0 || key == missing) return nullptr;
        const auto it = tiles.find(key);
        return it == tiles.end() ? nullptr : it->second.get();
    }
};

// ---- containers -------------------------------------------------------------

inline const g2m::TileKey kOffsetTestKey{kZone, 0, 1813, 21714};

// A heap tile with a per-sample pattern, 1/256 m (the pattern of racing_game's
// test_world_terrain.cpp make_pattern_tile).
inline std::unique_ptr<g2m::HeightTile> make_pattern_tile() {
    auto tile = std::make_unique<g2m::HeightTile>();
    tile->key = kOffsetTestKey;
    for (std::size_t i = 0; i < tile->h.size(); ++i) {
        tile->h[i] = 100 * 256 + static_cast<std::int32_t>(i % 997) - 400;
    }
    tile->has_nodata = false;
    return tile;
}

// HEADER || BODY for `tile`: the exact bytes a TileResponse carries.
inline std::vector<std::uint8_t> make_height_container(const g2m::HeightTile& tile, std::int32_t height_offset,
                                                       std::string layer = std::string(g2m::kTerrainHeightLayer),
                                                       const g2m::TileKey& header_key = kOffsetTestKey) {
    g2m::Result<g2m::TileBody> body = g2m::encode_height_tile(tile);
    must(body.ok(), "encode_height_tile");
    g2m::Result<std::vector<std::uint8_t>> body_bytes = g2m::encode_body(body.value());
    must(body_bytes.ok(), "encode_body");
    g2m::TileHeader header;
    header.layer = std::move(layer);
    header.key = header_key;
    header.height_offset = height_offset;
    g2m::Result<std::vector<std::uint8_t>> container =
        g2m::assemble_container(header, std::span<const std::uint8_t>(body_bytes.value()));
    must(container.ok(), "assemble_container");
    return std::move(container).value();
}

// ---- OSM --------------------------------------------------------------------

inline const g2m::TileKey kOsmKey = g2m_test::home_tile(0, 0); // level 2, 32N x 350 y 5450

inline void add_utm_node(g2m::osm::OsmBuilder& b, std::int64_t id, double e, double n) {
    const g2m::geo::Utm utm(g2m::geo::kGRS80);
    const g2m::geo::TmReverse g = utm.reverse(kZone, e, n);
    b.add_node(id, g2m_test::to_e7(g.lat), g2m_test::to_e7(g.lon));
}

// A small road network inside kOsmKey (E 358400..359424, N 5580800..5581824):
// a three-lane motorway, a residential street, a gravel track, a bridge, plus
// ways extract_road_segments drops (footway, tunnel), and a secondary road that
// crosses the tile's west edge (E 358400) for the owner rule.
inline g2m::OsmTile make_road_osm_tile() {
    g2m::osm::OsmBuilder b;
    add_utm_node(b, 1, 358500.0, 5581000.0);
    add_utm_node(b, 2, 358700.0, 5581050.0);
    add_utm_node(b, 3, 358900.0, 5581100.0);
    add_utm_node(b, 4, 358900.0, 5581300.0);
    add_utm_node(b, 5, 359100.0, 5581300.0);
    add_utm_node(b, 6, 358600.0, 5581500.0);
    add_utm_node(b, 7, 358800.0, 5581520.0);
    add_utm_node(b, 8, 359000.0, 5581700.0);
    add_utm_node(b, 9, 359200.0, 5581700.0);
    add_utm_node(b, 10, 358300.0, 5580900.0); // west of the tile edge E 358400, inside its 128 m halo
    add_utm_node(b, 11, 358500.0, 5580900.0);
    b.add_way(100, {1, 2, 3}, {{"highway", "motorway"}, {"lanes", "3"}, {"maxspeed", "120"}});
    b.add_way(101, {3, 4, 5}, {{"highway", "residential"}, {"surface", "sett"}});
    b.add_way(102, {6, 7}, {{"highway", "track"}, {"surface", "gravel"}});
    b.add_way(103, {8, 9}, {{"highway", "primary"}, {"bridge", "yes"}, {"width", "9"}});
    b.add_way(106, {10, 11}, {{"highway", "secondary"}});
    b.add_way(104, {2, 6}, {{"highway", "footway"}});
    b.add_way(105, {7, 8}, {{"highway", "secondary"}, {"tunnel", "yes"}});
    g2m::Result<g2m::osm::OsmData> data = b.build();
    must(data.ok(), "OsmBuilder::build");
    g2m::OsmTile tile;
    tile.halo_m = 128;
    tile.data = std::move(data).value();
    return tile;
}

inline std::vector<std::uint8_t> make_osm_container(const g2m::OsmTile& tile = g2m::OsmTile{},
                                                    std::string layer = std::string(g2m::kSrcOsmLayer),
                                                    const g2m::TileKey& header_key = kOsmKey) {
    g2m::Result<g2m::TileBody> body = g2m::encode_src_osm(tile);
    must(body.ok(), "encode_src_osm");
    g2m::Result<std::vector<std::uint8_t>> body_bytes = g2m::encode_body(body.value());
    must(body_bytes.ok(), "encode_body");
    g2m::TileHeader header;
    header.layer = std::move(layer);
    header.key = header_key;
    g2m::Result<std::vector<std::uint8_t>> container =
        g2m::assemble_container(header, std::span<const std::uint8_t>(body_bytes.value()));
    must(container.ok(), "assemble_container");
    return std::move(container).value();
}

inline g2m::Response make_tile_response(g2m::Status status, std::vector<std::uint8_t> container = {},
                                        std::string message = {}) {
    g2m::TileResponse response;
    response.meta.status = status;
    response.meta.message = std::move(message);
    response.container = std::move(container);
    return g2m::Response{std::move(response)};
}

// ---- decode cases ------------------------------------------------------------
//
// One list, used by the reference generator (racing_game's decoder) and by the
// tests (g2m::world's), so both sides see the same bytes.

struct DecodeCase {
    std::string name;
    std::vector<std::uint8_t> container;
    std::string expect_layer;
    g2m::TileKey expect_key;
};

inline std::vector<DecodeCase> decode_cases() {
    const std::string layer(g2m::kTerrainHeightLayer);
    std::vector<DecodeCase> out;
    const auto pattern = make_pattern_tile();
    out.push_back({"offset_zero", make_height_container(*pattern, 0), layer, kOffsetTestKey});
    out.push_back({"offset_plus", make_height_container(*pattern, 12345), layer, kOffsetTestKey});
    out.push_back({"offset_minus", make_height_container(*pattern, -20000), layer, kOffsetTestKey});
    out.push_back({"offset_int_max", make_height_container(*pattern, INT32_MAX), layer, kOffsetTestKey});

    auto nodata = make_pattern_tile();
    nodata->h[0] = g2m::kHeightNoData;
    nodata->h[300] = g2m::kHeightNoData;
    nodata->h[65535] = g2m::kHeightNoData;
    nodata->has_nodata = true;
    out.push_back({"nodata_kept", make_height_container(*nodata, 777), layer, kOffsetTestKey});

    auto low = make_pattern_tile();
    low->h[5] = -1000; // + INT32_MIN falls to or below kHeightNoData
    out.push_back({"offset_underflow", make_height_container(*low, INT32_MIN), layer, kOffsetTestKey});

    out.push_back({"wrong_layer", make_height_container(*pattern, 0, "g2m.terrain.class"), layer, kOffsetTestKey});
    out.push_back({"wrong_key", make_height_container(*pattern, 0), layer, g2m::TileKey{kZone, 0, 1814, 21714}});
    out.push_back({"expect_other_layer", make_height_container(*pattern, 0), "g2m.src.osm", kOffsetTestKey});

    std::vector<std::uint8_t> truncated = make_height_container(*pattern, 0);
    truncated.resize(truncated.size() - 10);
    out.push_back({"truncated", std::move(truncated), layer, kOffsetTestKey});

    std::vector<std::uint8_t> corrupt = make_height_container(*pattern, 0);
    corrupt.back() ^= 0x5A;
    out.push_back({"corrupt_body", std::move(corrupt), layer, kOffsetTestKey});

    out.push_back({"garbage", std::vector<std::uint8_t>{1, 2, 3, 4, 5, 6, 7}, layer, kOffsetTestKey});
    out.push_back({"empty", std::vector<std::uint8_t>{}, layer, kOffsetTestKey});
    return out;
}

// ---- road response cases ------------------------------------------------------

struct RoadResponseCase {
    std::string name;
    g2m::Response response;
    g2m::TileKey expect_key;
};

inline std::vector<RoadResponseCase> road_response_cases() {
    std::vector<RoadResponseCase> out;
    out.push_back({"ok", make_tile_response(g2m::Status::Ok, make_osm_container(make_road_osm_tile())), kOsmKey});
    out.push_back({"ok_empty_tile", make_tile_response(g2m::Status::Ok, make_osm_container(g2m::OsmTile{})), kOsmKey});
    out.push_back({"not_found_outside_coverage",
                   make_tile_response(g2m::Status::NotFound, {}, "outside coverage"), kOsmKey});
    out.push_back({"not_found_other", make_tile_response(g2m::Status::NotFound, {}, "unknown layer"), kOsmKey});
    out.push_back({"not_found_no_message", make_tile_response(g2m::Status::NotFound), kOsmKey});
    out.push_back({"server_error", make_tile_response(g2m::Status::Internal, {}, "boom"), kOsmKey});
    out.push_back({"ok_status_empty_container", make_tile_response(g2m::Status::Ok), kOsmKey});
    out.push_back({"wrong_layer",
                   make_tile_response(g2m::Status::Ok, make_osm_container(make_road_osm_tile(), "g2m.terrain.height")),
                   kOsmKey});
    out.push_back({"wrong_key", make_tile_response(g2m::Status::Ok, make_osm_container(make_road_osm_tile())),
                   g2m_test::home_tile(1, 0)});
    std::vector<std::uint8_t> corrupt = make_osm_container(make_road_osm_tile());
    corrupt.back() ^= 0x33;
    out.push_back({"corrupt_body", make_tile_response(g2m::Status::Ok, std::move(corrupt)), kOsmKey});
    // A height container where an OSM one is expected: decodes as a container,
    // fails the layer check.
    const auto pattern = make_pattern_tile();
    out.push_back({"height_container_for_osm",
                   make_tile_response(g2m::Status::Ok, make_height_container(*pattern, 0)), kOsmKey});
    out.push_back({"non_tile_response", g2m::Response{g2m::PackResponse{}}, kOsmKey});
    return out;
}

// ---- chunk road classes --------------------------------------------------------

// A road lookup context: segments for the tiles in `by_key`, nothing for any
// other tile. Records every call.
struct RoadLookupCtx {
    std::map<g2m::TileKey, std::shared_ptr<const std::vector<g2m::RoadSegment>>> by_key;
    std::vector<g2m::TileKey> calls;
};

inline std::shared_ptr<const std::vector<g2m::RoadSegment>> road_lookup_fn(void* ctx, const g2m::TileKey& key) {
    auto* c = static_cast<RoadLookupCtx*>(ctx);
    c->calls.push_back(key);
    const auto it = c->by_key.find(key);
    return it == c->by_key.end() ? nullptr : it->second;
}

inline std::vector<g2m::RoadSegment> make_road_segments() {
    const g2m::OsmTile tile = make_road_osm_tile();
    return g2m::extract_road_segments(tile.data, kZone, g2m::RoadStyle::default_style());
}

inline std::vector<g2m::mesh::ChunkKey> class_chunk_keys() {
    using K = g2m::mesh::ChunkKey;
    return {
        K{kZone, 0, 5601, 87203}, K{kZone, 0, 5603, 87203}, K{kZone, 0, 5604, 87203}, K{kZone, 0, 5607, 87204},
        K{kZone, 0, 5608, 87207}, K{kZone, 0, 5610, 87207}, K{kZone, 0, 5603, 87211}, K{kZone, 0, 5609, 87214},
        K{kZone, 0, 5612, 87214}, K{kZone, 0, 5599, 87203}, K{kZone, 0, 5615, 87210}, K{kZone, 0, 5400, 87000},
        K{kZone, 0, 5599, 87201}, K{kZone, 0, 5600, 87201},
        K{kZone, 1, 2800, 43601}, K{kZone, 1, 2803, 43602}, K{kZone, 2, 1400, 21800}, K{kZone, 3, 700, 10900},
        K{kZone, 4, 350, 5450},
    };
}

inline std::uint64_t hash_class_window(const g2m::mesh::ClassWindow& w) {
    g2m::Fnv1a64 h;
    h.bytes(w.c.data(), w.c.size());
    return h.value();
}

} // namespace g2m_world_test
