// tests/support/synthetic.h - deterministic synthetic layer data for tests
// and the G1 golden file: every value comes from g2m_test::SplitMix64 with
// integer arithmetic (one draw per statement, PLAN.md 2.4), so every
// compiler/OS builds the same tiles.
#pragma once

#include "splitmix64.h"

#include "g2m/core/geo/utm.h"
#include "g2m/core/raster.h"
#include "g2m/core/tile_key.h"
#include "g2m/layer/osm_data.h"
#include "g2m/layer/src_elev.h"
#include "g2m/layer/src_landcover.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace g2m_test {

inline constexpr g2m::geo::UtmZone kZone32N{32, g2m::geo::Hemisphere::North};

// A level-2 tile in the home region (Rhine valley, zone 32N).
inline g2m::TileKey home_tile(std::int32_t dx = 0, std::int32_t dy = 0) {
    return g2m::TileKey{kZone32N, 2, 350 + dx, 5450 + dy};
}

inline std::uint64_t below(SplitMix64& r, std::uint64_t n) { return r.next() % n; }

// DGM1-like: a cell-registered 1 m raster over the tile (side x side
// samples), 100 m base plus integer "hills" and +-3 cm noise (about
// DGM1's own cm quantisation), in 1/256 m.
// nodata_permille of the samples (in random runs) are kHeightNoData.
inline g2m::ElevTile make_elev_tile(std::uint64_t seed, const g2m::TileKey& key, int side = 1024,
                                    int nodata_permille = 5) {
    SplitMix64 r(seed);
    g2m::ElevTile t;
    t.georef = *g2m::tile_georef(key, side, g2m::SampleRegistration::Cell);
    t.heights = g2m::HeightRaster(side, side);
    struct Hill {
        std::int64_t cx, cy, r2, amp;
    };
    std::vector<Hill> hills;
    for (int i = 0; i < 12; ++i) {
        Hill h{};
        h.cx = static_cast<std::int64_t>(below(r, static_cast<std::uint64_t>(side)));
        h.cy = static_cast<std::int64_t>(below(r, static_cast<std::uint64_t>(side)));
        const std::int64_t rad = 50 + static_cast<std::int64_t>(below(r, 400));
        h.r2 = rad * rad;
        h.amp = static_cast<std::int64_t>(below(r, 40 * 256)) - 10 * 256;
        hills.push_back(h);
    }
    for (int y = 0; y < side; ++y) {
        for (int x = 0; x < side; ++x) {
            std::int64_t v = 100 * 256 + 3 * x - 2 * y;
            for (const Hill& h : hills) {
                const std::int64_t d2 = (x - h.cx) * (x - h.cx) + (y - h.cy) * (y - h.cy);
                if (d2 < h.r2) v += h.amp * (h.r2 - d2) / h.r2;
            }
            v += static_cast<std::int64_t>(below(r, 17)) - 8;
            t.heights.at(x, y) = static_cast<std::int32_t>(v);
        }
    }
    const std::int64_t total = static_cast<std::int64_t>(side) * side;
    std::int64_t remaining = total * nodata_permille / 1000;
    while (remaining > 0) {
        const int x = static_cast<int>(below(r, static_cast<std::uint64_t>(side)));
        const int y = static_cast<int>(below(r, static_cast<std::uint64_t>(side)));
        const int len = 1 + static_cast<int>(below(r, 64));
        for (int k = 0; k < len && x + k < side && remaining > 0; ++k, --remaining) {
            t.heights.at(x + k, y) = g2m::kHeightNoData;
        }
    }
    return t;
}

// WorldCover-like: 4 m cells (256 x 256 per level-2 tile), patchy classes.
inline g2m::LandcoverTile make_landcover_tile(std::uint64_t seed, const g2m::TileKey& key) {
    SplitMix64 r(seed);
    g2m::LandcoverTile t;
    t.georef = *g2m::tile_georef(key, 256, g2m::SampleRegistration::Cell);
    t.classes = g2m::ClassRaster(256, 256);
    t.class_scheme = "esa_worldcover_v200";
    t.nodata_class = 0;
    static constexpr std::uint8_t kClasses[] = {10, 20, 30, 40, 50, 60, 80, 90};
    std::uint8_t cur = 10;
    for (int y = 0; y < 256; ++y) {
        for (int x = 0; x < 256; ++x) {
            if (below(r, 16) == 0) cur = kClasses[below(r, 8)];
            t.classes.at(x, y) = (x > 0 && below(r, 4) != 0) ? t.classes.at(x - 1, y)
                                 : (y > 0 && below(r, 2) != 0) ? t.classes.at(x, y - 1)
                                                               : cur;
        }
    }
    return t;
}

// Degrees -> OSM 1e-7 fixed point (round half away from zero).
inline std::int32_t to_e7(double deg) { return static_cast<std::int32_t>(std::llround(deg * 1e7)); }

// An OSM set scattered over a (2 * extent_m)^2 square centred on `key`'s
// tile: n_nodes nodes, polyline ways (some closed), relations (some
// nested), tags from a small vocabulary.
inline g2m::osm::OsmData make_osm_set(std::uint64_t seed, const g2m::TileKey& key, int n_nodes = 600, int n_ways = 90,
                                      int n_rels = 12, std::int64_t extent_m = 2000) {
    using g2m::osm::MemberType;
    SplitMix64 r(seed);
    const g2m::geo::Utm utm(g2m::geo::kGRS80);
    const std::int64_t cx = key.min_easting() + key.size_m() / 2;
    const std::int64_t cy = key.min_northing() + key.size_m() / 2;
    static const char* kKeys[] = {"highway", "surface", "name", "lanes", "maxspeed", "building", "landuse", "oneway"};
    static const char* kVals[] = {"residential", "primary", "asphalt", "gravel", "Hauptstrasse", "2", "50", "yes",
                                  "forest", "meadow", "track", "unclassified"};
    const auto tags = [&](int max) {
        g2m::osm::OsmBuilder::TagList tl;
        const int n = static_cast<int>(below(r, static_cast<std::uint64_t>(max + 1)));
        for (int i = 0; i < n; ++i) {
            const char* k = kKeys[below(r, 8)];
            const char* v = kVals[below(r, 12)];
            bool dup = false;
            for (const auto& kv : tl) dup = dup || kv.first == k;
            if (!dup) tl.emplace_back(k, v);
        }
        return tl;
    };
    g2m::osm::OsmBuilder b;
    std::vector<std::int64_t> node_ids;
    std::int64_t id = 1000000;
    for (int i = 0; i < n_nodes; ++i) {
        id += 1 + static_cast<std::int64_t>(below(r, 5));
        const double e = static_cast<double>(cx - extent_m + static_cast<std::int64_t>(below(r, 2 * extent_m * 1000)) / 1000);
        const double n = static_cast<double>(cy - extent_m + static_cast<std::int64_t>(below(r, 2 * extent_m * 1000)) / 1000);
        const g2m::geo::TmReverse g = utm.reverse(key.zone, e, n);
        b.add_node(id, to_e7(g.lat), to_e7(g.lon), tags(2));
        node_ids.push_back(id);
    }
    std::vector<std::int64_t> way_ids;
    std::int64_t wid = 5000;
    for (int i = 0; i < n_ways; ++i) {
        wid += 1 + static_cast<std::int64_t>(below(r, 3));
        std::vector<std::int64_t> refs;
        const int len = 2 + static_cast<int>(below(r, 9));
        for (int k = 0; k < len; ++k) refs.push_back(node_ids[below(r, node_ids.size())]);
        if (below(r, 5) == 0) refs.push_back(refs.front()); // closed
        b.add_way(wid, std::move(refs), tags(4));
        way_ids.push_back(wid);
    }
    std::int64_t rid = 70;
    std::vector<std::int64_t> rel_ids;
    for (int i = 0; i < n_rels; ++i) {
        rid += 1 + static_cast<std::int64_t>(below(r, 2));
        std::vector<g2m::osm::OsmBuilder::MemberIn> mem;
        const int len = 1 + static_cast<int>(below(r, 6));
        for (int k = 0; k < len; ++k) {
            const std::uint64_t t = below(r, 3);
            if (t == 0) {
                mem.push_back({MemberType::Node, node_ids[below(r, node_ids.size())], ""});
            } else if (t == 1 || rel_ids.empty()) {
                const std::int64_t w = way_ids[below(r, way_ids.size())];
                const bool outer = below(r, 2) != 0;
                mem.push_back({MemberType::Way, w, outer ? "outer" : "inner"});
            } else {
                mem.push_back({MemberType::Relation, rel_ids[below(r, rel_ids.size())], "subarea"});
            }
        }
        if (below(r, 3) == 0) mem.push_back({MemberType::Way, 999999999, "outer"}); // not in the set
        b.add_relation(rid, std::move(mem), tags(3));
        rel_ids.push_back(rid);
    }
    auto d = b.build();
    return d ? std::move(d).value() : g2m::osm::OsmData{};
}

} // namespace g2m_test
