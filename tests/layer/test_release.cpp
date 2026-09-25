// Release ids, coverage, manifest JSON, layer names and the registry.

#include "g2m/layer/layer_registry.h"
#include "g2m/layer/release.h"

#include "splitmix64.h"
#include "synthetic.h"

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <string>

using namespace g2m;

namespace {

ContentHash h(std::uint8_t seed) {
    ContentHash c{};
    for (std::size_t i = 0; i < c.size(); ++i) c[i] = static_cast<std::uint8_t>(seed * 31 + i);
    return c;
}

ReleaseInputs sample_inputs() {
    ReleaseInputs in;
    for (int i = 0; i < 20; ++i) {
        const std::string layer(i % 3 == 0 ? "g2m.src.osm" : "g2m.src.elev.dgm1");
        in.sources.push_back(SourceTileRef{layer, g2m_test::home_tile(i, -i).packed(), h(static_cast<std::uint8_t>(i))});
    }
    in.layers.push_back(LayerVersionRef{"g2m.src.osm", 1, {}});
    in.layers.push_back(LayerVersionRef{"g2m.src.elev.dgm1", 1, {}});
    in.layers.push_back(LayerVersionRef{"g2m.terrain.height", 3, h(100)});
    in.classifiers.push_back(ClassifierRef{"g2m.default", h(200)});
    in.world_seed = 0x0123456789abcdefull;
    return in;
}

ReleaseManifest sample_manifest() {
    ReleaseManifest m;
    m.rid = *compute_release_id(sample_inputs());
    m.world_seed = 0xfedcba9876543210ull;
    m.sources.push_back(ReleaseSource{"dgm1_he", "2026-01-15", "dl-de/by-2-0", "HVBG \"Hessen\"", false});
    m.sources.push_back(ReleaseSource{"osm", "2026-09-01", "ODbL-1.0", "\xc2\xa9 OpenStreetMap contributors", true});
    ReleaseLayer elev;
    elev.desc = make_raw_elev_layer("dgm1");
    elev.coverage = Coverage::from_level2_tiles({g2m_test::home_tile(0, 0), g2m_test::home_tile(1, 0),
                                                 g2m_test::home_tile(3, 0), g2m_test::home_tile(0, -1)});
    ReleaseLayer derived;
    derived.desc.id = LayerId{"rg.race_routes", 2};
    derived.desc.kind = LayerKind::Baked;
    derived.desc.encoding = 0x8001;
    derived.desc.levels = level_bit(2) | level_bit(4);
    derived.desc.inputs = {"g2m.src.elev.dgm1", "g2m.src.osm"};
    derived.desc.content = "race routes";
    derived.config_hash = h(7);
    derived.availability = Availability::OnDemand;
    derived.coverage.global = true;
    ReleaseLayer osm_layer;
    osm_layer.desc = make_raw_osm_layer();
    osm_layer.availability = Availability::Unavailable;
    m.layers = {elev, osm_layer, derived};
    std::sort(m.layers.begin(), m.layers.end(),
              [](const ReleaseLayer& a, const ReleaseLayer& b) { return a.desc.id.name < b.desc.id.name; });
    m.classifiers.push_back(ClassifierRef{"g2m.default", h(200)});
    return m;
}

} // namespace

TEST_CASE("release id: order independent, sensitive to every input", "[layer][release]") {
    const ReleaseInputs base = sample_inputs();
    const auto rid = compute_release_id(base);
    REQUIRE(rid);

    ReleaseInputs shuffled = base;
    std::reverse(shuffled.sources.begin(), shuffled.sources.end());
    std::rotate(shuffled.layers.begin(), shuffled.layers.begin() + 1, shuffled.layers.end());
    CHECK(*compute_release_id(shuffled) == *rid);

    const auto differs = [&](ReleaseInputs in) {
        auto r = compute_release_id(in);
        REQUIRE(r);
        return *r != *rid;
    };
    ReleaseInputs in = base;
    in.sources[3].body_hash[31] ^= 1;
    CHECK(differs(in));
    in = base;
    in.sources.pop_back();
    CHECK(differs(in));
    in = base;
    in.layers[2].version = 4;
    CHECK(differs(in));
    in = base;
    in.layers[2].config_hash[0] ^= 1;
    CHECK(differs(in));
    in = base;
    in.classifiers[0].table_hash[5] ^= 1;
    CHECK(differs(in));
    in = base;
    in.world_seed ^= 1;
    CHECK(differs(in));
    // Length prefixes: moving a character between name and key changes it.
    in = base;
    in.classifiers[0].name = "g2m.defaul";
    CHECK(differs(in));

    in = base;
    in.sources.push_back(in.sources[0]);
    in.sources.back().body_hash[0] ^= 1; // same (layer, key), other hash
    CHECK_FALSE(compute_release_id(in));
    in = base;
    in.layers.push_back(in.layers[0]);
    CHECK_FALSE(compute_release_id(in));
    in = base;
    in.classifiers.push_back(in.classifiers[0]);
    CHECK_FALSE(compute_release_id(in));
}

TEST_CASE("release: hex round trip", "[layer][release]") {
    const ContentHash c = h(9);
    const std::string s = to_hex(c);
    CHECK(s.size() == 64);
    CHECK(content_hash_from_hex(s) == c);
    CHECK_FALSE(content_hash_from_hex(s.substr(1)));
    std::string upper = s;
    upper[0] = 'A';
    CHECK_FALSE(content_hash_from_hex(upper));
    CHECK_FALSE(content_hash_from_hex(std::string(64, 'g')));
}

TEST_CASE("release: coverage runs and covers()", "[layer][release]") {
    const TileKey a = g2m_test::home_tile(0, 0);
    const Coverage c = Coverage::from_level2_tiles({g2m_test::home_tile(2, 0), a, g2m_test::home_tile(1, 0), a,
                                                    g2m_test::home_tile(5, 0), g2m_test::home_tile(0, 1)});
    REQUIRE(c.zones.size() == 1);
    REQUIRE(c.zones[0].runs.size() == 3);
    CHECK(c.zones[0].runs[0] == CoverageRun{5450, 350, 352});
    CHECK(c.zones[0].runs[1] == CoverageRun{5450, 355, 355});
    CHECK(c.zones[0].runs[2] == CoverageRun{5451, 350, 350});
    CHECK(c.covers(a));
    CHECK(c.covers(g2m_test::home_tile(2, 0)));
    CHECK_FALSE(c.covers(g2m_test::home_tile(3, 0)));
    CHECK(c.covers(a.child(1, 1)));   // level 1 under a covered tile
    CHECK(c.covers(a.parent()));      // level 3 over it
    CHECK_FALSE(c.covers(g2m_test::home_tile(3, 0).child(0, 0)));
    TileKey other = a;
    other.zone.zone = 33;
    CHECK_FALSE(c.covers(other));
    Coverage g;
    g.global = true;
    CHECK(g.covers(other));
}

TEST_CASE("release manifest: JSON round trip and canonical bytes", "[layer][release]") {
    const ReleaseManifest m = sample_manifest();
    const std::string j = to_json(m);
    CHECK(j.find("\"format\":\"g2m.release/1\"") != std::string::npos);
    auto back = release_manifest_from_json(j);
    REQUIRE(back);
    CHECK(*back == m);
    CHECK(to_json(*back) == j);
    const ReleaseLayer* l = back->find_layer("rg.race_routes");
    REQUIRE(l);
    CHECK(l->availability == Availability::OnDemand);
    CHECK(back->find_layer("nope.nope") == nullptr);
}

TEST_CASE("release manifest: malformed JSON fails without throwing", "[layer][release]") {
    const std::string good = to_json(sample_manifest());
    CHECK_FALSE(release_manifest_from_json(""));
    CHECK_FALSE(release_manifest_from_json("{"));
    CHECK_FALSE(release_manifest_from_json("[]"));
    CHECK_FALSE(release_manifest_from_json("{\"format\":\"g2m.release/2\"}"));
    CHECK_FALSE(release_manifest_from_json(good.substr(0, good.size() / 2)));
    // A required key with the wrong type.
    std::string bad = good;
    const auto pos = bad.find("\"world_seed\":\"");
    REQUIRE(pos != std::string::npos);
    bad.replace(pos, 14, "\"world_seed\":7,\"x\":\"");
    CHECK_FALSE(release_manifest_from_json(bad));
    // Every truncation and a few mutations: never throws, never crashes.
    for (std::size_t n = 0; n < good.size(); n += 7) (void)release_manifest_from_json(good.substr(0, n));
    g2m_test::SplitMix64 r(3);
    for (int i = 0; i < 500; ++i) {
        std::string m = good;
        const std::size_t p = static_cast<std::size_t>(r.next() % m.size());
        m[p] = static_cast<char>(r.next() % 128);
        (void)release_manifest_from_json(m);
    }
}

TEST_CASE("layers: names, raw descriptors and the registry", "[layer]") {
    CHECK(is_valid_layer_name("g2m.src.osm"));
    CHECK(is_valid_layer_name("rg.race_routes"));
    CHECK_FALSE(is_valid_layer_name("g2m"));
    CHECK_FALSE(is_valid_layer_name("g2m..osm"));
    CHECK_FALSE(is_valid_layer_name("g2m.Src"));
    CHECK_FALSE(is_valid_layer_name("g2m.1src"));
    CHECK_FALSE(is_valid_layer_name(".g2m.src"));
    CHECK_FALSE(is_valid_layer_name("g2m.src."));
    CHECK_FALSE(is_valid_layer_name("a." + std::string(63, 'b')));
    CHECK(is_builtin_layer_name("g2m.src.osm"));
    CHECK_FALSE(is_builtin_layer_name("rg.g2m"));

    CHECK(make_raw_elev_layer("glo30").id.name == "g2m.src.elev.glo30");
    CHECK(make_raw_elev_layer("Bad").id.name.empty());
    CHECK(make_raw_elev_layer("dgm1").encoding == static_cast<std::uint16_t>(LayerEncoding::SrcElev));
    CHECK(has_level(make_raw_osm_layer().levels, kRawTileLevel));

    const LayerRegistry& b = LayerRegistry::builtin();
    REQUIRE(b.all().size() == 3);
    CHECK(b.all()[0].id.name == "g2m.src.elev.dgm1");
    CHECK(b.all()[1].id.name == "g2m.src.landcover");
    CHECK(b.all()[2].id.name == "g2m.src.osm");
    CHECK(b.find("g2m.src.osm") != nullptr);
    CHECK(b.find("g2m.src.elev.glo30") == nullptr);

    LayerRegistry r = b;
    CHECK(r.add(make_raw_elev_layer("glo30")));
    CHECK_FALSE(r.add(make_raw_elev_layer("glo30"))); // duplicate
    LayerDesc d;
    d.id = LayerId{"rg.routes", 1};
    CHECK_FALSE(r.add(d)); // no levels
    d.levels = level_bit(2);
    d.inputs = {"g2m.src.osm", "g2m.src.elev.dgm1"}; // unsorted
    CHECK_FALSE(r.add(d));
    std::swap(d.inputs[0], d.inputs[1]);
    CHECK_FALSE(r.add(d)); // a raw layer with inputs
    d.kind = LayerKind::Baked;
    CHECK(r.add(d));
    CHECK(std::is_sorted(r.all().begin(), r.all().end(),
                         [](const LayerDesc& x, const LayerDesc& y) { return x.id.name < y.id.name; }));
}
