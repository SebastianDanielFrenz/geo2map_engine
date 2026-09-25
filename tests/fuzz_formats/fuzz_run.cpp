// tests/fuzz_formats/fuzz_run.cpp - the fuzz target bodies (fuzz_targets.h).
// Depends only on g2m_layers and protocol.cpp, so the libFuzzer build can
// compile it against an instrumented copy of exactly those sources.

#include "fuzz_targets.h"

#include "g2m/layer/pack.h"
#include "g2m/layer/region.h"
#include "g2m/layer/release.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/src_landcover.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/protocol.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <optional>
#include <string>
#include <utility>

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

} // namespace

void run(Target target, std::span<const std::uint8_t> data) {
    switch (target) {
    case Target::Container: container(data); return;
    case Target::Elev:
        raw_layer(
            data, [](const g2m::TileBody& b) { return g2m::decode_src_elev(b); },
            [](const g2m::ElevTile& t) { return g2m::encode_src_elev(t); }, "src.elev: round trip");
        return;
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
    case Target::Pack: pack(data); return;
    case Target::Manifest: manifest(data); return;
    case Target::Path: path(data); return;
    case Target::Region: region(data); return;
    }
}

} // namespace g2m_fuzz
