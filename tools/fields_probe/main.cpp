// g2m_fields_probe - census and picture of g2m.fields.geom over an existing baked store (dev tool, not part of the
// library; nothing here has to be deterministic).
//
//   g2m_fields_probe stats DERIVED_DIR RELEASE_HEX [fields.csv]
//       counts, area distribution, implausible share, declines, subdivision numbers; the CSV (one line per field,
//       owned copy only) is optional.
//   g2m_fields_probe png DERIVED_DIR RELEASE_HEX SOURCE_DIR SCOPE out.png [--tile X,Y] [--tiles N] [--mpp M]
//       fields filled by land use, piece borders in dark, the OSM roads, tracks, paths, waterways and hedges the
//       subdivision cut along drawn on top. Default: the N x N (3) window of level-2 tiles with the most fields,
//       M metres per pixel (2).
#include "g2m/core/geo/utm.h"
#include "g2m/layer/field_tile.h"
#include "g2m/layer/src_osm.h"
#include "g2m/layer/tile_container.h"
#include "g2m/server/tile_store.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <vector>

using namespace g2m;

namespace {
std::unique_ptr<TileStore> open_store(const std::string& dir) {
    TileStoreConfig config; config.dir = dir; config.read_only = true;
    auto store = TileStore::open(config);
    if (!store) { std::cerr << "cannot open " << dir << ": " << store.error().message << "\n"; std::exit(1); }
    return std::move(*store);
}

std::vector<FieldTile> load_tiles(TileStore& store, const std::string& release) {
    auto index = store.index(release);
    if (!index) { std::cerr << "index: " << index.error().message << "\n"; std::exit(1); }
    std::vector<FieldTile> tiles;
    for (const auto& entry : *index) {
        if (entry.layer != kFieldsGeomLayer) continue;
        auto stored = store.get({release, entry.layer, entry.key});
        if (!stored || !*stored) continue;
        auto body = decode_body((*stored)->body);
        if (!body) { std::cerr << "body " << to_string(entry.key) << ": " << body.error().message << "\n"; std::exit(1); }
        auto tile = decode_fields_geom(*body);
        if (!tile) { std::cerr << "tile " << to_string(entry.key) << ": " << tile.error().message << "\n"; std::exit(1); }
        tiles.push_back(std::move(*tile));
    }
    return tiles;
}

double percentile(const std::vector<double>& sorted, double p) {
    if (sorted.empty()) return 0;
    const double pos = p * static_cast<double>(sorted.size() - 1);
    const auto lo = static_cast<std::size_t>(pos);
    const auto hi = std::min(lo + 1, sorted.size() - 1);
    return sorted[lo] + (sorted[hi] - sorted[lo]) * (pos - static_cast<double>(lo));
}

const char* landuse_name(int code) {
    switch (code) { case 0: return "farmland"; case 1: return "meadow"; case 2: return "orchard"; case 3: return "vineyard"; case 4: return "farmyard"; default: return "?"; }
}
const char* decline_name(FieldDeclineReason r) {
    switch (r) {
    case FieldDeclineReason::InvalidGeometry: return "invalid geometry";
    case FieldDeclineReason::RelationIncomplete: return "relation incomplete";
    case FieldDeclineReason::TooLarge: return "too large";
    case FieldDeclineReason::TooManyCells: return "too many cells";
    case FieldDeclineReason::TooManyTiles: return "too many tiles";
    }
    return "?";
}

int stats(int argc, char** argv) {
    if (argc < 4) { std::cerr << "usage: stats DERIVED_DIR RELEASE_HEX [fields.csv]\n"; return 1; }
    auto store = open_store(argv[2]);
    const auto tiles = load_tiles(*store, argv[3]);
    std::ofstream csv; if (argc > 4) { csv.open(argv[4]); csv << "id,source,osm_id,piece,landuse,crop,area_m2,row_dir_deg,flags,vertices,holes,anchor_e_m,anchor_n_m\n"; }
    std::vector<double> areas;
    std::map<int, std::pair<std::size_t, double>> by_landuse; // count, area m2
    std::map<std::string, std::size_t> crops;
    std::map<std::pair<int, std::int64_t>, std::size_t> pieces_per_block;
    std::map<std::pair<int, std::int64_t>, FieldDeclineReason> declines;
    std::array<std::size_t, 8> flag_counts{}; std::size_t implausible = 0, with_holes = 0, fields = 0;
    std::uint64_t vertices = 0, max_vertices = 0;
    FieldTileCounts counts{}; std::size_t non_empty_tiles = 0;
    for (const auto& tile : tiles) {
        counts.blocks += tile.counts.blocks; counts.subdivided += tile.counts.subdivided; counts.slivers_dropped += tile.counts.slivers_dropped;
        counts.obstacles_ignored += tile.counts.obstacles_ignored; counts.sliver_blocks += tile.counts.sliver_blocks;
        if (!tile.fields.empty()) ++non_empty_tiles;
        for (const auto& d : tile.declines) declines.emplace(std::make_pair(static_cast<int>(d.source), d.osm_id), d.reason);
        for (const auto& f : tile.fields) {
            if ((f.flags & field_flag::Owned) == 0) continue;
            ++fields; areas.push_back(f.area_m2());
            auto& lu = by_landuse[f.landuse]; ++lu.first; lu.second += f.area_m2();
            if (!f.crop.empty()) ++crops[f.crop];
            ++pieces_per_block[{static_cast<int>(f.source), f.osm_id}];
            for (int b = 0; b < 8; ++b) if ((f.flags >> b) & 1u) ++flag_counts[static_cast<std::size_t>(b)];
            if ((f.flags & (field_flag::SmallArea | field_flag::LargeArea | field_flag::Thin)) != 0) ++implausible;
            if (!f.polygon.holes.empty()) ++with_holes;
            std::uint64_t v = f.polygon.outer.size(); for (const auto& h : f.polygon.holes) v += h.size();
            vertices += v; max_vertices = std::max(max_vertices, v);
            if (csv) csv << f.id << ',' << static_cast<int>(f.source) << ',' << f.osm_id << ',' << f.piece << ',' << landuse_name(f.landuse) << ',' << f.crop << ','
                         << f.area_m2() << ',' << f.row_dir_cdeg / 100.0 << ',' << f.flags << ',' << v << ',' << f.polygon.holes.size() << ','
                         << static_cast<double>(f.anchor.x) / 1000.0 << ',' << static_cast<double>(f.anchor.y) / 1000.0 << '\n';
        }
    }
    std::sort(areas.begin(), areas.end());
    std::printf("tiles %zu (with fields %zu)\n", tiles.size(), non_empty_tiles);
    std::printf("fields %zu, blocks %u (subdivided %u, kept whole %u), slivers dropped %u, sliver blocks %u, obstacles ignored %u\n", fields, counts.blocks,
        counts.subdivided, counts.blocks - counts.subdivided, counts.slivers_dropped, counts.sliver_blocks, counts.obstacles_ignored);
    std::printf("declined blocks %zu:", declines.size());
    std::map<FieldDeclineReason, std::size_t> by_reason; for (const auto& [k, r] : declines) ++by_reason[r];
    for (const auto& [r, n] : by_reason) std::printf(" %s %zu;", decline_name(r), n);
    std::printf("\n");
    double total = 0; for (const double a : areas) total += a;
    std::printf("area: total %.1f km2, mean %.0f m2, percentiles (m2) p1 %.0f p5 %.0f p25 %.0f p50 %.0f p75 %.0f p95 %.0f p99 %.0f max %.0f\n", total / 1e6,
        areas.empty() ? 0.0 : total / static_cast<double>(areas.size()), percentile(areas, .01), percentile(areas, .05), percentile(areas, .25), percentile(areas, .5),
        percentile(areas, .75), percentile(areas, .95), percentile(areas, .99), areas.empty() ? 0.0 : areas.back());
    const std::array<double, 9> edges{0, 200, 1000, 5000, 20000, 100000, 500000, 1000000, 1e18};
    std::printf("area histogram (m2):");
    for (std::size_t b = 0; b + 1 < edges.size(); ++b) {
        std::size_t n = 0; for (const double a : areas) if (a >= edges[b] && a < edges[b + 1]) ++n;
        std::printf(" [%.0f,%s) %zu;", edges[b], b + 2 == edges.size() ? "inf" : std::to_string(static_cast<long long>(edges[b + 1])).c_str(), n);
    }
    std::printf("\n");
    for (const auto& [code, v] : by_landuse) std::printf("landuse %-9s fields %6zu  area %8.1f km2\n", landuse_name(code), v.first, v.second / 1e6);
    const char* flag_names[8] = {"owned", "subdivided", "obstacles_ignored", "sliver_block", "simplify_fallback", "small_area", "large_area", "thin"};
    for (int b = 1; b < 8; ++b) std::printf("flag %-18s %6zu (%.2f %%)\n", flag_names[b], flag_counts[static_cast<std::size_t>(b)], fields ? 100.0 * static_cast<double>(flag_counts[static_cast<std::size_t>(b)]) / static_cast<double>(fields) : 0.0);
    std::printf("implausible (small, large or thin): %zu of %zu = %.2f %%\n", implausible, fields, fields ? 100.0 * static_cast<double>(implausible) / static_cast<double>(fields) : 0.0);
    std::printf("fields with holes %zu; outline vertices mean %.1f max %llu\n", with_holes, fields ? static_cast<double>(vertices) / static_cast<double>(fields) : 0.0, static_cast<unsigned long long>(max_vertices));
    std::array<std::size_t, 6> pieces{}; // 1, 2, 3-5, 6-10, 11-50, >50
    for (const auto& [k, n] : pieces_per_block) ++pieces[n == 1 ? 0 : n == 2 ? 1 : n <= 5 ? 2 : n <= 10 ? 3 : n <= 50 ? 4 : 5];
    std::printf("pieces per block: 1 %zu, 2 %zu, 3-5 %zu, 6-10 %zu, 11-50 %zu, >50 %zu\n", pieces[0], pieces[1], pieces[2], pieces[3], pieces[4], pieces[5]);
    std::vector<std::pair<std::size_t, std::string>> top; for (const auto& [c, n] : crops) top.push_back({n, c});
    std::sort(top.rbegin(), top.rend());
    std::size_t with_crop = 0; for (const auto& [n, c] : top) with_crop += n;
    std::printf("fields with crop=*: %zu (%.1f %%); top:", with_crop, fields ? 100.0 * static_cast<double>(with_crop) / static_cast<double>(fields) : 0.0);
    for (std::size_t i = 0; i < top.size() && i < 12; ++i) std::printf(" %s %zu;", top[i].second.c_str(), top[i].first);
    std::printf("\n");
    return 0;
}

// ---- picture --------------------------------------------------------------------------------------------------------------

struct Image {
    int w = 0, h = 0;
    std::vector<std::uint8_t> rgb;
    Image(int width, int height) : w(width), h(height), rgb(static_cast<std::size_t>(width) * static_cast<std::size_t>(height) * 3, 255) {}
    void put(int x, int y, std::array<std::uint8_t, 3> c, double alpha = 1.0) {
        if (x < 0 || y < 0 || x >= w || y >= h) return;
        auto* p = &rgb[(static_cast<std::size_t>(y) * static_cast<std::size_t>(w) + static_cast<std::size_t>(x)) * 3];
        for (int i = 0; i < 3; ++i) p[i] = static_cast<std::uint8_t>(std::lround(p[i] * (1 - alpha) + c[static_cast<std::size_t>(i)] * alpha));
    }
    void line(double x0, double y0, double x1, double y1, std::array<std::uint8_t, 3> c, double alpha = 1.0) {
        const int steps = std::max(1, static_cast<int>(std::ceil(std::max(std::abs(x1 - x0), std::abs(y1 - y0)) * 2)));
        for (int i = 0; i <= steps; ++i) {
            const double t = static_cast<double>(i) / steps;
            put(static_cast<int>(std::floor(x0 + (x1 - x0) * t)), static_cast<int>(std::floor(y0 + (y1 - y0) * t)), c, alpha);
        }
    }
};

std::uint32_t crc32(const std::uint8_t* d, std::size_t n, std::uint32_t crc = 0) {
    static std::array<std::uint32_t, 256> table = [] {
        std::array<std::uint32_t, 256> t{};
        for (std::uint32_t i = 0; i < 256; ++i) { std::uint32_t c = i; for (int k = 0; k < 8; ++k) c = (c & 1) ? 0xEDB88320u ^ (c >> 1) : c >> 1; t[i] = c; }
        return t;
    }();
    crc = ~crc;
    for (std::size_t i = 0; i < n; ++i) crc = table[(crc ^ d[i]) & 0xff] ^ (crc >> 8);
    return ~crc;
}
void be32(std::vector<std::uint8_t>& v, std::uint32_t x) { for (int i = 3; i >= 0; --i) v.push_back(static_cast<std::uint8_t>(x >> (8 * i))); }
void chunk(std::vector<std::uint8_t>& out, const char* type, const std::vector<std::uint8_t>& data) {
    be32(out, static_cast<std::uint32_t>(data.size()));
    std::vector<std::uint8_t> body(type, type + 4); body.insert(body.end(), data.begin(), data.end());
    out.insert(out.end(), body.begin(), body.end());
    be32(out, crc32(body.data(), body.size()));
}
bool write_png(const std::string& path, const Image& img) {
    std::vector<std::uint8_t> raw;
    for (int y = 0; y < img.h; ++y) {
        raw.push_back(0);
        raw.insert(raw.end(), img.rgb.begin() + static_cast<std::ptrdiff_t>(y) * img.w * 3, img.rgb.begin() + static_cast<std::ptrdiff_t>(y + 1) * img.w * 3);
    }
    std::vector<std::uint8_t> z{0x78, 0x01};
    std::uint32_t a = 1, b = 0;
    for (const auto byte : raw) { a = (a + byte) % 65521; b = (b + a) % 65521; }
    for (std::size_t pos = 0; pos < raw.size();) {
        const std::size_t n = std::min<std::size_t>(65535, raw.size() - pos);
        z.push_back(pos + n == raw.size() ? 1 : 0);
        z.push_back(static_cast<std::uint8_t>(n & 0xff)); z.push_back(static_cast<std::uint8_t>(n >> 8));
        z.push_back(static_cast<std::uint8_t>(~n & 0xff)); z.push_back(static_cast<std::uint8_t>((~n >> 8) & 0xff));
        z.insert(z.end(), raw.begin() + static_cast<std::ptrdiff_t>(pos), raw.begin() + static_cast<std::ptrdiff_t>(pos + n));
        pos += n;
    }
    be32(z, (b << 16) | a);
    std::vector<std::uint8_t> out{0x89, 'P', 'N', 'G', 0x0d, 0x0a, 0x1a, 0x0a};
    std::vector<std::uint8_t> ihdr; be32(ihdr, static_cast<std::uint32_t>(img.w)); be32(ihdr, static_cast<std::uint32_t>(img.h));
    ihdr.insert(ihdr.end(), {8, 2, 0, 0, 0});
    chunk(out, "IHDR", ihdr); chunk(out, "IDAT", z); chunk(out, "IEND", {});
    std::ofstream f(path, std::ios::binary);
    f.write(reinterpret_cast<const char*>(out.data()), static_cast<std::streamsize>(out.size()));
    return static_cast<bool>(f);
}

std::array<std::uint8_t, 3> landuse_colour(int code, std::uint64_t id) {
    // A base colour per land use, nudged by the id so neighbouring pieces differ.
    static const std::array<std::array<int, 3>, 5> base{{{222, 200, 120}, {150, 205, 120}, {120, 180, 110}, {170, 120, 190}, {200, 150, 110}}};
    const auto& b = base[static_cast<std::size_t>(std::min(code, 4))];
    const int nudge = static_cast<int>((id * 2654435761u >> 7) % 41) - 20;
    return {static_cast<std::uint8_t>(std::clamp(b[0] + nudge, 0, 255)), static_cast<std::uint8_t>(std::clamp(b[1] + nudge, 0, 255)), static_cast<std::uint8_t>(std::clamp(b[2] + nudge, 0, 255))};
}

void fill_polygon(Image& img, const FieldPolygon& poly, double e0, double n0, double mpp, std::array<std::uint8_t, 3> colour) {
    const auto to_px = [&](const geom::PointMm& p) { return std::make_pair((static_cast<double>(p.x) / 1000.0 - e0) / mpp, (n0 - static_cast<double>(p.y) / 1000.0) / mpp); };
    std::vector<std::pair<double, double>> pts; std::vector<std::size_t> starts;
    for (const auto* ring : [&] { std::vector<const geom::Ring*> r{&poly.outer}; for (const auto& h : poly.holes) r.push_back(&h); return r; }()) {
        starts.push_back(pts.size()); for (const auto& p : *ring) pts.push_back(to_px(p));
    }
    starts.push_back(pts.size());
    double ymin = 1e18, ymax = -1e18; for (const auto& p : pts) { ymin = std::min(ymin, p.second); ymax = std::max(ymax, p.second); }
    for (int y = std::max(0, static_cast<int>(std::floor(ymin))); y <= std::min(img.h - 1, static_cast<int>(std::ceil(ymax))); ++y) {
        const double yc = y + 0.5; std::vector<double> xs;
        for (std::size_t r = 0; r + 1 < starts.size(); ++r) {
            const std::size_t a = starts[r], n = starts[r + 1] - starts[r];
            for (std::size_t i = 0; i < n; ++i) {
                const auto& p = pts[a + i]; const auto& q = pts[a + (i + 1) % n];
                if ((p.second <= yc) != (q.second <= yc)) xs.push_back(p.first + (yc - p.second) / (q.second - p.second) * (q.first - p.first));
            }
        }
        std::sort(xs.begin(), xs.end());
        for (std::size_t i = 0; i + 1 < xs.size(); i += 2)
            for (int x = static_cast<int>(std::ceil(xs[i] - 0.5)); x <= static_cast<int>(std::floor(xs[i + 1] - 0.5)); ++x) img.put(x, y, colour);
    }
    for (std::size_t r = 0; r + 1 < starts.size(); ++r) {
        const std::size_t a = starts[r], n = starts[r + 1] - starts[r];
        for (std::size_t i = 0; i < n; ++i) { const auto& p = pts[a + i]; const auto& q = pts[a + (i + 1) % n]; img.line(p.first, p.second, q.first, q.second, {60, 50, 30}, 0.85); }
    }
}

int png(int argc, char** argv) {
    if (argc < 7) { std::cerr << "usage: png DERIVED_DIR RELEASE_HEX SOURCE_DIR SCOPE out.png [--tile X,Y] [--tiles N] [--mpp M]\n"; return 1; }
    int n_tiles = 3; double mpp = 2.0; std::optional<std::pair<int, int>> want;
    for (int i = 7; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--tile" && i + 1 < argc) { char* end = nullptr; const long x = std::strtol(argv[++i], &end, 10); if (*end != ',') return 1; const long y = std::strtol(end + 1, &end, 10); if (*end != 0) return 1; want = std::make_pair(static_cast<int>(x), static_cast<int>(y)); }
        else if (a == "--tiles" && i + 1 < argc) n_tiles = std::atoi(argv[++i]);
        else if (a == "--mpp" && i + 1 < argc) mpp = std::atof(argv[++i]);
        else { std::cerr << "unknown option " << a << "\n"; return 1; }
    }
    auto derived = open_store(argv[2]);
    const auto tiles = load_tiles(*derived, argv[3]);
    if (tiles.empty()) { std::cerr << "no field tiles\n"; return 1; }
    std::map<std::pair<int, int>, const FieldTile*> by_key; for (const auto& t : tiles) by_key[{t.key.x, t.key.y}] = &t;
    std::pair<int, int> origin{};
    if (want) origin = *want;
    else {   // the window with the most field records
        std::size_t best = 0;
        for (const auto& t : tiles) {
            std::size_t n = 0;
            for (int dy = 0; dy < n_tiles; ++dy) for (int dx = 0; dx < n_tiles; ++dx) { const auto it = by_key.find({t.key.x + dx, t.key.y + dy}); if (it != by_key.end()) n += it->second->fields.size(); }
            if (n > best) { best = n; origin = {t.key.x, t.key.y}; }
        }
    }
    const auto zone = tiles.front().key.zone;
    const TileKey first{zone, 2, origin.first, origin.second};
    const double e0 = static_cast<double>(first.min_easting()), n_top = static_cast<double>(first.min_northing()) + 1024.0 * n_tiles;
    const int size = static_cast<int>(std::lround(1024.0 * n_tiles / mpp));
    Image img(size, size);
    for (int y = 0; y < size; ++y) for (int x = 0; x < size; ++x) img.put(x, y, {245, 243, 238});
    std::map<std::uint64_t, bool> drawn;
    for (int dy = 0; dy < n_tiles; ++dy) for (int dx = 0; dx < n_tiles; ++dx) {
        const auto it = by_key.find({origin.first + dx, origin.second + dy}); if (it == by_key.end()) continue;
        for (const auto& f : it->second->fields) { if (!drawn.emplace(f.id, true).second) continue; fill_polygon(img, f.polygon, e0, n_top, mpp, landuse_colour(f.landuse, f.id)); }
    }
    // OSM lines the subdivision cuts along, from the source store (read only).
    auto sources = open_store(argv[4]);
    const geo::Utm utm(geo::kGRS80);
    struct Style { std::array<std::uint8_t, 3> c; double a; };
    const auto style_of = [](const osm::OsmData& d, const osm::Way& w) -> std::optional<Style> {
        const auto hw = d.tag(d.tags_of(w), "highway"), ww = d.tag(d.tags_of(w), "waterway"), br = d.tag(d.tags_of(w), "barrier"),
                   nat = d.tag(d.tags_of(w), "natural"), rw = d.tag(d.tags_of(w), "railway");
        if (hw) { if (*hw == "track") return Style{{110, 70, 30}, 1.0}; if (*hw == "path" || *hw == "footway" || *hw == "cycleway" || *hw == "bridleway") return Style{{150, 90, 60}, 0.9};
                  if (*hw == "motorway" || *hw == "trunk" || *hw == "primary") return Style{{200, 40, 40}, 1.0}; return Style{{90, 90, 90}, 1.0}; }
        if (ww) return Style{{40, 100, 220}, 1.0};
        if (br && (*br == "hedge")) return Style{{20, 120, 20}, 1.0};
        if (nat && (*nat == "tree_row")) return Style{{20, 120, 20}, 1.0};
        if (rw) return Style{{60, 60, 60}, 1.0};
        return std::nullopt;
    };
    std::set<std::int64_t> seen_ways;
    for (int dy = -1; dy <= n_tiles; ++dy) for (int dx = -1; dx <= n_tiles; ++dx) {
        const TileKey key{zone, 2, origin.first + dx, origin.second + dy};
        auto stored = (*sources).get({argv[5], std::string(kSrcOsmLayer), key});
        if (!stored || !*stored) continue;
        auto body = decode_body((*stored)->body); if (!body) continue;
        auto osm_tile = decode_src_osm(*body); if (!osm_tile) continue;
        const auto& d = osm_tile->data;
        std::map<std::int64_t, std::pair<double, double>> pos;
        for (const auto& n : d.nodes) {
            const auto p = utm.forward(zone, n.lat / 1e7, n.lon / 1e7);
            pos[n.id] = {(p.easting - e0) / mpp, (n_top - p.northing) / mpp};
        }
        for (const auto& w : d.ways) {
            if (!seen_ways.insert(w.id).second) continue;
            const auto style = style_of(d, w); if (!style) continue;
            const auto refs = d.refs_of(w);
            for (std::size_t i = 1; i < refs.size(); ++i) {
                const auto a = pos.find(refs[i - 1]), b = pos.find(refs[i]); if (a == pos.end() || b == pos.end()) continue;
                img.line(a->second.first, a->second.second, b->second.first, b->second.second, style->c, style->a);
            }
        }
    }
    if (!write_png(argv[6], img)) { std::cerr << "cannot write " << argv[6] << "\n"; return 1; }
    std::printf("window tiles (%d,%d) %dx%d, %dx%d px at %.2f m/px -> %s\n", origin.first, origin.second, n_tiles, n_tiles, size, size, mpp, argv[6]);
    return 0;
}
} // namespace

int main(int argc, char** argv) {
    if (argc >= 2 && std::string_view(argv[1]) == "stats") return stats(argc, argv);
    if (argc >= 2 && std::string_view(argv[1]) == "png") return png(argc, argv);
    std::cerr << "usage: g2m_fields_probe stats|png ...\n";
    return 1;
}
