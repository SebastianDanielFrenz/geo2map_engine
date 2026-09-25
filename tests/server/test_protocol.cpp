// Protocol v1 objects: request paths (to_path/parse_path) and the server
// manifest JSON.

#include "g2m/server/protocol.h"

#include "server_test_util.h"

#include <catch2/catch_test_macros.hpp>

using namespace g2m;

namespace {

ReleaseId rid_of(std::uint8_t b) {
    ReleaseId r{};
    for (std::size_t i = 0; i < r.size(); ++i) r[i] = static_cast<std::uint8_t>(b + i);
    return r;
}

} // namespace

TEST_CASE("protocol: request paths round trip", "[server][protocol]") {
    const ReleaseId rid = rid_of(0xa0);
    const std::string hex = to_hex(rid);
    const TileKey south{geo::UtmZone{33, geo::Hemisphere::South}, 3, -12, 700};
    const std::vector<std::pair<Request, std::string>> cases{
        {ManifestRequest{}, "/manifest"},
        {ReleaseManifestRequest{rid}, "/releases/" + hex + "/manifest"},
        {TileRequest{rid, "g2m.src.osm", g2m_test::home_tile(), std::nullopt},
         "/releases/" + hex + "/tiles/g2m.src.osm/utm32n/2/350/5450"},
        {PackRequest{rid, "rg.race_routes", south}, "/releases/" + hex + "/packs/rg.race_routes/utm33s/3/-12/700"},
        {HashesRequest{rid, "g2m.src.landcover", pack_of(g2m_test::home_tile())},
         "/releases/" + hex + "/hashes/g2m.src.landcover/utm32n/2/21/340"},
    };
    for (const auto& [req, path] : cases) {
        CHECK(to_path(req) == path);
        auto back = parse_path(path);
        REQUIRE(back);
        CHECK(*back == req);
    }
}

TEST_CASE("protocol: parse_path rejects everything else", "[server][protocol]") {
    const std::string hex = to_hex(rid_of(1));
    const std::string base = "/releases/" + hex;
    const std::vector<std::string> bad{
        "",
        "/",
        "/manifest/",
        "manifest",
        "/releases/",
        base,
        base + "/",
        base + "/manifest/",
        "/releases/" + hex.substr(1) + "/manifest",       // 63 digits
        "/releases/" + std::string(64, 'g') + "/manifest", // not hex
        "/releases/" + std::string(64, 'A') + "/manifest", // upper case
        base + "/tiles/g2m.src.osm/utm32n/2/350",         // missing y
        base + "/tiles/g2m.src.osm/utm32n/2/350/5450/",   // trailing slash
        base + "/tiles/g2m.src.osm/utm32n/2/350/+5450",
        base + "/tiles/g2m.src.osm/utm32n/2/0350/5450",
        base + "/tiles/g2m.src.osm/utm32n/2/-0/5450",
        base + "/tiles/g2m.src.osm/utm32n/21/350/5450",    // level > 20
        base + "/tiles/g2m.src.osm/utm32n/2/8388608/5450", // x out of range
        base + "/tiles/g2m.src.osm/utm61n/2/350/5450",
        base + "/tiles/g2m.src.osm/utm032n/2/350/5450",
        base + "/tiles/g2m.src.osm/utm32x/2/350/5450",
        base + "/tiles/G2M.src.osm/utm32n/2/350/5450",
        base + "/tiles/osm/utm32n/2/350/5450",
        base + "/tile/g2m.src.osm/utm32n/2/350/5450",
    };
    for (const std::string& p : bad) {
        INFO(p);
        CHECK_FALSE(parse_path(p));
    }
}

TEST_CASE("protocol: server manifest JSON", "[server][protocol]") {
    ServerManifest m;
    m.server_id = "g2m.integrated";
    m.default_rid = rid_of(3);
    m.live_releases = {rid_of(3), rid_of(9)};
    m.contact = "ops@example.invalid";
    m.rate_limit = "100/min";
    const std::string j = to_json(m);
    CHECK(j.find("\"format\":\"g2m.server/1\"") != std::string::npos);
    CHECK(j.find(' ') == std::string::npos);
    auto back = server_manifest_from_json(j);
    REQUIRE(back);
    CHECK(*back == m);
    CHECK(to_json(*back) == j);

    CHECK_FALSE(server_manifest_from_json(""));
    CHECK_FALSE(server_manifest_from_json("[]"));
    std::string wrong = j;
    wrong.replace(wrong.find("g2m.server/1"), 12, "g2m.server/2");
    CHECK_FALSE(server_manifest_from_json(wrong));
    std::string bad_rid = j;
    bad_rid.replace(bad_rid.find(to_hex(rid_of(9))), 2, "zz");
    CHECK_FALSE(server_manifest_from_json(bad_rid));
}
