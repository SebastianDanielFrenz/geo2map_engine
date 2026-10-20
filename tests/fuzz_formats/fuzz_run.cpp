// tests/fuzz_formats/fuzz_run.cpp - the fuzz target bodies (fuzz_targets.h).
// Depends only on g2m_layers, protocol.cpp and http_wire.cpp, so the libFuzzer build can
// compile it against an instrumented copy of exactly those sources.

#include "fuzz_targets.h"

#include "g2m/http/http_wire.h"
#include "g2m/layer/pack.h"
#include "g2m/layer/rail_geom_tile.h"
#include "g2m/layer/road_graph_tile.h"
#include "g2m/layer/road_geom_tile.h"
#include "g2m/layer/region.h"
#include "g2m/layer/release.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/src_landcover.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/terrain_class.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/protocol.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace g2m_fuzz {
namespace {

[[noreturn]] void fail(const char* what) {
    std::fprintf(stderr, "fuzz property violated: %s\n", what);
    std::abort();
}

void check(bool ok, const char* what) {
    if (!ok) fail(what);
}

// decode_body -> encode_body -> decode_body is stable.
std::optional<g2m::TileBody> body_of(std::span<const std::uint8_t> data) {
    auto b = g2m::decode_body(data);
    if (!b) return std::nullopt;
    auto again = g2m::encode_body(*b);
    check(again.ok(), "body: re-encode failed");
    auto back = g2m::decode_body(*again);
    check(back.ok() && *back == *b, "body: round trip");
    return std::move(*b);
}

template <class Decode, class Encode>
void raw_layer(std::span<const std::uint8_t> data, Decode decode, Encode encode, const char* what) {
    const auto body = body_of(data);
    if (!body) return;
    auto tile = decode(*body);
    if (!tile) return;
    auto enc = encode(*tile);
    check(enc.ok(), what);
    auto back = decode(*enc);
    check(back.ok() && *back == *tile, what);
}

void container(std::span<const std::uint8_t> data) {
    std::size_t consumed = 0;
    if (auto h = g2m::decode_header(data, &consumed)) {
        check(consumed <= data.size(), "header: consumed past the end");
        auto enc = g2m::encode_header(*h);
        check(enc.ok(), "header: re-encode failed");
        std::size_t consumed2 = 0;
        auto back = g2m::decode_header(*enc, &consumed2);
        check(back.ok() && *back == *h && consumed2 == enc->size(), "header: round trip");
    }
    if (auto v = g2m::parse_container(data)) {
        auto again = g2m::assemble_container(v->header, v->body);
        check(again.ok(), "container: re-assemble failed");
        auto back = g2m::parse_container(*again);
        check(back.ok() && back->header == v->header &&
                  std::equal(back->body.begin(), back->body.end(), v->body.begin(), v->body.end()),
              "container: round trip");
        (void)body_of(v->body);
    }
    (void)body_of(data);
}

void pack(std::span<const std::uint8_t> data) {
    auto v = g2m::decode_pack(data);
    if (!v) return;
    std::array<std::vector<std::uint8_t>, g2m::kPackTiles> containers;
    for (std::size_t i = 0; i < containers.size(); ++i) {
        containers[i].assign(v->containers[i].begin(), v->containers[i].end());
    }
    auto enc = g2m::encode_pack(v->dir, containers);
    check(enc.ok(), "pack: re-encode failed");
    auto back = g2m::decode_pack(*enc);
    check(back.ok() && back->dir == v->dir, "pack: round trip");
}

void manifest(std::span<const std::uint8_t> data) {
    const std::string_view text(reinterpret_cast<const char*>(data.data()), data.size());
    if (auto m = g2m::release_manifest_from_json(text)) {
        const std::string j = g2m::to_json(*m);
        auto back = g2m::release_manifest_from_json(j);
        check(back.ok() && *back == *m && g2m::to_json(*back) == j, "release manifest: round trip");
    }
    if (auto m = g2m::server_manifest_from_json(text)) {
        const std::string j = g2m::to_json(*m);
        auto back = g2m::server_manifest_from_json(j);
        check(back.ok() && *back == *m && g2m::to_json(*back) == j, "server manifest: round trip");
    }
}

void path(std::span<const std::uint8_t> data) {
    const std::string_view text(reinterpret_cast<const char*>(data.data()), data.size());
    auto r = g2m::parse_path(text);
    if (!r) return;
    check(g2m::to_path(*r) == text, "path: to_path(parse_path(p)) != p");
}

void region(std::span<const std::uint8_t> data) {
    const std::string_view text(reinterpret_cast<const char*>(data.data()), data.size());
    // regions.json (g2m.regions/1) has no to_json (it is hand-authored
    // only, never re-serialised by g2m itself) so the only property this
    // target can check for it is "does not crash" - exercised by simply
    // calling parse_regions. The resolved-region artefact does round-trip.
    (void)g2m::parse_regions(text);
    if (auto r = g2m::parse_resolved_region(text)) {
        const std::string j = g2m::to_json(*r);
        auto back = g2m::parse_resolved_region(j);
        check(back.ok() && *back == *r && g2m::to_json(*back) == j, "resolved region: round trip");
    }
}

void pack_directory(std::span<const std::uint8_t> data) {
    auto d = g2m::decode_pack_directory(data);
    if (!d) return;
    auto enc = g2m::encode_pack_directory(*d);
    check(enc.ok(), "pack directory: re-encode failed");
    check(std::equal(enc->begin(), enc->end(), data.begin(), data.end()), "pack directory: encode(decode(b)) != b");
}

std::vector<std::string_view> split_lines(std::string_view text) {
    std::vector<std::string_view> out;
    while (true) {
        const std::size_t nl = text.find('\n');
        out.push_back(text.substr(0, nl));
        if (nl == std::string_view::npos) return out;
        text.remove_prefix(nl + 1);
    }
}

// Byte 0: bits 0-2 pick the method, bit 3 the base path. Then the target
// line, then one If-None-Match value per further line.
void http_request(std::span<const std::uint8_t> data) {
    if (data.empty()) return;
    static constexpr std::string_view kMethods[] = {"GET", "HEAD", "POST", "PUT", "get", "OPTIONS", "DELETE", ""};
    const std::string_view method = kMethods[data[0] & 7];
    const std::string_view base = (data[0] & 8) != 0 ? std::string_view("/maps/v") : std::string_view();
    const std::string_view text(reinterpret_cast<const char*>(data.data()) + 1, data.size() - 1);
    const std::vector<std::string_view> lines = split_lines(text);
    const std::vector<std::string_view> inm(lines.begin() + 1, lines.end());
    if (auto r = g2m::http::parse_request(base, method, lines.front(), inm)) {
        check(method == "GET" || method == "HEAD", "http request: accepted a method other than GET/HEAD");
        check(g2m::http::request_target(base, *r) == lines.front(), "http request: request_target(parse(t)) != t");
        auto again = g2m::http::parse_request(base, method, g2m::http::request_target(base, *r), inm);
        check(again.ok() && g2m::to_path(*again) == g2m::to_path(*r), "http request: reparse");
    }
    // Header text: decode is the exact inverse of encode.
    if (auto d = g2m::http::decode_header_text(text)) {
        check(g2m::http::encode_header_text(*d) == text, "header text: encode(decode(t)) != t");
    }
    auto back = g2m::http::decode_header_text(g2m::http::encode_header_text(text));
    check(back && *back == text, "header text: decode(encode(t)) != t");
}

bool same_response(const g2m::Response& a, const g2m::Response& b) {
    if (a.index() != b.index()) return false;
    const g2m::ResponseMeta& x = g2m::meta_of(a);
    const g2m::ResponseMeta& y = g2m::meta_of(b);
    if (x.status != y.status || x.degraded != y.degraded || x.retry_after_s != y.retry_after_s || x.etag != y.etag ||
        x.newest_rid != y.newest_rid || x.message != y.message) {
        return false;
    }
    if (const auto* m = std::get_if<g2m::ManifestResponse>(&a)) return m->manifest == std::get<g2m::ManifestResponse>(b).manifest;
    if (const auto* m = std::get_if<g2m::ReleaseManifestResponse>(&a)) {
        return m->manifest == std::get<g2m::ReleaseManifestResponse>(b).manifest;
    }
    if (const auto* t = std::get_if<g2m::TileResponse>(&a)) return t->container == std::get<g2m::TileResponse>(b).container;
    if (const auto* p = std::get_if<g2m::PackResponse>(&a)) return p->pack == std::get<g2m::PackResponse>(b).pack;
    if (const auto* h = std::get_if<g2m::HashesResponse>(&a)) return h->directory == std::get<g2m::HashesResponse>(b).directory;
    return false;
}

// "target\nstatus\nName: value\n...\n\nbody": the target names the request
// (a manifest request when it does not parse); every decoded response
// encodes to an HTTP message that decodes back to the same response.
void http_response(std::span<const std::uint8_t> data) {
    const std::string_view all(reinterpret_cast<const char*>(data.data()), data.size());
    const std::size_t split = all.find("\n\n");
    const std::string_view head = all.substr(0, split);
    g2m::http::Message msg;
    if (split != std::string_view::npos) msg.body.assign(data.begin() + static_cast<std::ptrdiff_t>(split + 2), data.end());
    const std::vector<std::string_view> lines = split_lines(head);
    auto parsed = g2m::http::parse_request("", "GET", lines.front(), {});
    const g2m::Request request = parsed ? *parsed : g2m::Request{g2m::ManifestRequest{}};
    if (lines.size() < 2) return;
    int status = 0;
    const std::string_view st = lines[1];
    if (st.empty() || st.size() > 3 || !std::all_of(st.begin(), st.end(), [](char c) { return c >= '0' && c <= '9'; })) return;
    for (char c : st) status = status * 10 + (c - '0');
    msg.status = status;
    for (std::size_t i = 2; i < lines.size(); ++i) {
        const std::size_t colon = lines[i].find(':');
        if (colon == std::string_view::npos) continue;
        std::string_view value = lines[i].substr(colon + 1);
        while (!value.empty() && value.front() == ' ') value.remove_prefix(1);
        msg.headers.push_back(g2m::http::Header{std::string(lines[i].substr(0, colon)), std::string(value)});
    }
    const g2m::Response first = g2m::http::decode_response(request, msg);
    const g2m::http::Message wire = g2m::http::encode_response(request, first);
    const g2m::Response second = g2m::http::decode_response(request, wire);
    check(same_response(first, second), "http response: decode(encode(r)) != r");
    check(g2m::http::encode_response(request, second).headers == wire.headers, "http response: encode not stable");
}

} // namespace

void run(Target target, std::span<const std::uint8_t> data) {
    switch (target) {
    case Target::Container: container(data); return;
    case Target::Elev:
        raw_layer(
            data, [](const g2m::TileBody& b) { return g2m::decode_src_elev(b); },
            [](const g2m::ElevTile& t) { return g2m::encode_src_elev(t); }, "src.elev: round trip");
        return;
    case Target::TerrainClass: {
        const auto body = body_of(data); if (!body) return;
        const g2m::TileKey key{g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North}, 0, kFuzzClassX, kFuzzClassY};
        g2m::TerrainClassTile tile;
        if (!g2m::decode_terrain_class(*body, key, tile)) return;
        auto enc = g2m::encode_terrain_class(tile);
        check(enc.ok(), "terrain.class: re-encode");
        g2m::TerrainClassTile back;
        check(g2m::decode_terrain_class(*enc, key, back).ok() && back == tile, "terrain.class: round trip");
        return;
    }
    case Target::RailGeom: {
        const auto body = body_of(data); if (!body) return;
        const g2m::TileKey key{g2m::geo::UtmZone{32, g2m::geo::Hemisphere::North}, 2, kFuzzRailX, kFuzzRailY};
        const auto tile = g2m::decode_rail_geom(*body, key); if (!tile) return;
        check(g2m::validate_rail_geom_tile(*tile).ok(), "rail.geom: decoded tile validates");
        const auto enc = g2m::encode_rail_geom(*tile); check(enc.ok(), "rail.geom: re-encode");
        const auto back = g2m::decode_rail_geom(*enc, key); check(back.ok() && *back == *tile, "rail.geom: round trip");
        const auto again = g2m::encode_rail_geom(*back); check(again.ok() && g2m::encode_body(*again).value() == g2m::encode_body(*enc).value(), "rail.geom: stable bytes");
        return;
    }
    case Target::Landcover:
        raw_layer(
            data, [](const g2m::TileBody& b) { return g2m::decode_src_landcover(b); },
            [](const g2m::LandcoverTile& t) { return g2m::encode_src_landcover(t); }, "src.landcover: round trip");
        return;
    case Target::Osm:
        raw_layer(
            data, [](const g2m::TileBody& b) { return g2m::decode_src_osm(b); },
            [](const g2m::OsmTile& t) { return g2m::encode_src_osm(t); }, "src.osm: round trip");
        return;
    case Target::Roadgeom: {
        const auto body = body_of(data); if (!body) return;
        const auto tile = g2m::decode_road_geom(*body); if (!tile) return;
        if (tile->encoding == g2m::kRoadGeomEncodingLegacy) {
            // A legacy body decodes, carries no cross-sections, and is never written back.
            check(!g2m::encode_road_geom(*tile).ok(), "roads.geom: legacy encode refused");
            for (const auto& entry : tile->entries) check(!entry.cross_section && !entry.cross_section_decline, "roads.geom: legacy cross-section");
            check(tile->features.empty() && tile->junctions.empty() && tile->furniture.empty() && tile->origins.empty(), "roads.geom: legacy features");
            return;
        }
        if (tile->encoding == g2m::kRoadGeomEncodingPrevious) {
            // Encoding 6: no junctions, furniture or origins; written back as encoding 6 below.
            check(tile->junctions.empty() && tile->furniture.empty() && tile->origins.empty(), "roads.geom: encoding 6 carries P2 records");
        } else {
            // Encoding 7: one provenance record per entry, every index in range, the reserved furniture bit clear.
            check(tile->origins.size() == tile->entries.size(), "roads.geom: provenance count");
            for (const auto& j : tile->junctions) {
                check(!j.nodes.empty(), "roads.geom: junction nodes");
                for (const auto& a : j.approaches) check(a.entry_index < tile->entries.size(), "roads.geom: approach index");
            }
            for (const auto& f : tile->furniture)
                check(f.entry_index < tile->entries.size() && f.junction_index <= tile->junctions.size() && (f.flags & g2m::kFurnitureInsideJunction) == 0,
                    "roads.geom: furniture index or reserved flag");
        }
        const auto encoded = g2m::encode_road_geom(*tile); check(encoded.ok(), "roads.geom: encode");
        check(encoded->encoding == tile->encoding, "roads.geom: encoding kept");
        const auto back = g2m::decode_road_geom(*encoded); check(back.ok(), "roads.geom: decode");
        check(back->junctions == tile->junctions && back->furniture == tile->furniture && back->origins == tile->origins, "roads.geom: P2 records round trip");
        const auto again = g2m::encode_road_geom(*back); check(again.ok() && *again == *encoded, "roads.geom: stable round trip");
        return;
    }
    case Target::RoadCrossSectionEval: {
        const auto body = body_of(data); if (!body) return;
        const auto tile = g2m::decode_road_geom(*body); if (!tile) return;
        for (const auto& entry : tile->entries) {
            if (!entry.cross_section) continue;
            const auto& xs = *entry.cross_section; std::array<double, 33> t{};
            const std::span<double> out(t.data(), xs.bands.size() + 1);
            for (int i = 0; i <= 16; ++i) {
                const double s = xs.length_m * static_cast<double>(i) / 16.0;
                check(xs.boundaries_at(s, out), "xs eval: boundaries_at");
                for (std::size_t j = 0; j < out.size(); ++j) {
                    check(std::isfinite(out[j]), "xs eval: finite");
                    if (j > 0) check(out[j] <= out[j - 1], "xs eval: non-increasing");
                }
                const auto edges = g2m::carriageway_edges(xs, s);
                const auto fp = g2m::carve_footprint(xs, s);
                check(std::isfinite(edges.left) && std::isfinite(edges.right) && edges.left >= edges.right, "xs eval: carriageway edges");
                check(std::isfinite(fp.crown_pivot) && fp.outer_left >= fp.paved_left && fp.paved_left >= fp.roadbed_left
                    && fp.roadbed_right >= fp.paved_right && fp.paved_right >= fp.outer_right, "xs eval: footprint nesting");
                std::array<g2m::LaneQuery, 8> lanes{};
                for (const auto travel : {g2m::LaneDirection::Forward, g2m::LaneDirection::Backward}) {
                    const std::size_t n = g2m::driving_lanes_at(xs, s, travel, lanes);
                    check(n <= lanes.size(), "xs eval: lane count");
                    for (std::size_t k = 0; k < n; ++k) check(std::isfinite(lanes[k].centre_t) && lanes[k].width_m >= 0.5, "xs eval: lane");
                }
            }
        }
        return;
    }
    case Target::Roadgraph:
        raw_layer(data, [](const g2m::TileBody& b) { return g2m::decode_road_graph(b); },
            [](const g2m::RoadGraphTile& t) { return g2m::encode_road_graph(t); }, "roads.graph: round trip");
        return;
    case Target::Pack: pack(data); return;
    case Target::Manifest: manifest(data); return;
    case Target::Path: path(data); return;
    case Target::Region: region(data); return;
    case Target::PackDirectory: pack_directory(data); return;
    case Target::HttpRequest: http_request(data); return;
    case Target::HttpResponse: http_response(data); return;
    }
}

} // namespace g2m_fuzz
