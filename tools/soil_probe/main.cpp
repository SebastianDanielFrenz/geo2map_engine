// g2m_soil_probe - census and picture of g2m.soil.grid over an existing baked store, read together with the farm fields
// of a g2m.fields.geom store (dev tool, not part of the library; nothing here has to be deterministic).
//
//   g2m_soil_probe stats SOIL_DIR SOIL_RELEASE FIELDS_DIR FIELDS_RELEASE [fields_soil.csv]
//       cell census (state shares, texture class shares per horizon, mean values), then the per-field aggregates
//       (summarise_field_soil over the fields): how much a field varies inside, and a plausibility table per
//       region (bounding boxes in lon/lat, see kRegions).
//   g2m_soil_probe png SOIL_DIR SOIL_RELEASE FIELDS_DIR FIELDS_RELEASE out.png [--tile X,Y] [--tiles N] [--mpp M]
//                       [--horizon 0|1] [--lonlat LON,LAT]
//       the topsoil (or subsoil) texture class per cell, strongly inside the farm fields and faded outside them, the
//       field outlines on top. Default window: the N x N (3) block of level-2 tiles with the most fields.
#include "g2m/core/geo/utm.h"
#include "g2m/layer/field_tile.h"
#include "g2m/layer/soil_tile.h"
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

std::vector<FieldTile> load_field_tiles(TileStore& store, const std::string& release) {
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

using SoilTiles = std::map<std::uint64_t, SoilGridTile>;
SoilTiles load_soil_tiles(TileStore& store, const std::string& release) {
    auto index = store.index(release);
    if (!index) { std::cerr << "index: " << index.error().message << "\n"; std::exit(1); }
    SoilTiles tiles;
    for (const auto& entry : *index) {
        if (entry.layer != kSoilGridLayer) continue;
        auto stored = store.get({release, entry.layer, entry.key});
        if (!stored || !*stored) continue;
        auto body = decode_body((*stored)->body);
        if (!body) { std::cerr << "body " << to_string(entry.key) << ": " << body.error().message << "\n"; std::exit(1); }
        auto tile = decode_soil_grid(*body);
        if (!tile) { std::cerr << "tile " << to_string(entry.key) << ": " << tile.error().message << "\n"; std::exit(1); }
        tiles.emplace(tile->key.packed(), std::move(*tile));
    }
    return tiles;
}

SoilLookup make_lookup(const SoilTiles& tiles, geo::UtmZone zone) {
    return [&tiles, zone](std::int64_t e_mm, std::int64_t n_mm) -> std::optional<SoilSample> {
        const TileKey key{zone, 2, static_cast<std::int32_t>(floor_div(e_mm, 1024000)), static_cast<std::int32_t>(floor_div(n_mm, 1024000))};
        const auto it = tiles.find(key.packed());
        if (it == tiles.end()) return std::nullopt;
        return soil_at(it->second, e_mm, n_mm);
    };
}

const char* texture_name(int code) {
    static const char* names[] = {"-", "sand", "loamy sand", "sandy loam", "loam", "silt loam", "silt", "sandy clay loam", "clay loam", "silty clay loam", "sandy clay", "silty clay", "clay"};
    return code >= 0 && code <= 12 ? names[code] : "?";
}
const char* state_name(int s) {
    static const char* names[] = {"ok", "no source", "built-up", "water"};
    return s >= 0 && s < 4 ? names[s] : "?";
}
std::array<std::uint8_t, 3> texture_colour(int code) {
    static const std::array<std::array<std::uint8_t, 3>, 13> c{{
        {255, 255, 255}, {245, 215, 90}, {230, 185, 60}, {215, 150, 60}, {150, 110, 60}, {120, 150, 190}, {90, 180, 190},
        {200, 90, 60}, {160, 60, 70}, {70, 110, 190}, {230, 120, 150}, {120, 80, 160}, {60, 40, 90}}};
    return c[static_cast<std::size_t>(std::clamp(code, 0, 12))];
}

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    const double pos = p * static_cast<double>(v.size() - 1);
    const auto lo = static_cast<std::size_t>(pos), hi = std::min(lo + 1, v.size() - 1);
    return v[lo] + (v[hi] - v[lo]) * (pos - static_cast<double>(lo));
}

struct Region { const char* name; double lon0, lat0, lon1, lat1; };
// Plausibility boxes (approximate, for a sanity check only): loess landscapes against sandy floodplains.
constexpr std::array<Region, 6> kRegions{{
    {"Wetterau, western edge (loess)", 8.65, 50.20, 8.85, 50.43},
    {"Main-Taunus foreland, Hochheim-Hofheim (loess)", 8.30, 50.00, 8.50, 50.10},
    {"Frankfurt north, Bad Homburg-Bad Vilbel (loess)", 8.55, 50.10, 8.75, 50.20},
    {"Hessisches Ried, Rhine plain (sand)", 8.40, 49.60, 8.60, 49.85},
    {"Rhine-Main plain, Gross-Gerau-Darmstadt (sand)", 8.45, 49.85, 8.65, 49.95},
    {"Untermain, Offenbach-Hanau side", 8.70, 49.95, 8.84, 50.05},
}};

int stats(int argc, char** argv) {
    if (argc < 6) { std::cerr << "usage: stats SOIL_DIR SOIL_RELEASE FIELDS_DIR FIELDS_RELEASE [fields_soil.csv]\n"; return 1; }
    auto soil_store = open_store(argv[2]);
    const SoilTiles soil = load_soil_tiles(*soil_store, argv[3]);
    auto fields_store = open_store(argv[4]);
    const auto ftiles = load_field_tiles(*fields_store, argv[5]);
    if (soil.empty()) { std::cerr << "no soil tiles\n"; return 1; }
    const auto zone = soil.begin()->second.key.zone;

    // ---- cells
    std::array<std::uint64_t, kSoilStateCount> states{};
    std::array<std::array<std::uint64_t, 13>, kSoilHorizons> classes{};
    std::array<double, kSoilHorizons> sand{}, silt{}, clay{}, soc{}, ph{}, cfvo{}, bdod{};
    std::uint64_t cells = 0, ok = 0;
    for (const auto& [k, t] : soil)
        for (const auto& c : t.cells) {
            ++cells; ++states[static_cast<std::size_t>(c.state)];
            if (c.state != SoilState::Ok) continue;
            ++ok;
            for (std::size_t h = 0; h < kSoilHorizons; ++h) {
                ++classes[h][c.horizon[h].texture];
                sand[h] += c.horizon[h].sand; silt[h] += c.horizon[h].silt; clay[h] += c.horizon[h].clay;
                soc[h] += c.horizon[h].soc; ph[h] += c.horizon[h].ph10; cfvo[h] += c.horizon[h].cfvo; bdod[h] += c.horizon[h].bdod;
            }
        }
    std::printf("soil tiles %zu, cells %llu (%.1f km2 at 32 m)\n", soil.size(), static_cast<unsigned long long>(cells), static_cast<double>(cells) * 1024.0 / 1e6);
    for (int s = 0; s < kSoilStateCount; ++s)
        std::printf("state %-10s %10llu  %6.2f %%\n", state_name(s), static_cast<unsigned long long>(states[static_cast<std::size_t>(s)]), 100.0 * static_cast<double>(states[static_cast<std::size_t>(s)]) / static_cast<double>(cells));
    const double okd = ok ? static_cast<double>(ok) : 1.0;
    for (std::size_t h = 0; h < kSoilHorizons; ++h) {
        std::printf("horizon %zu (%s): mean sand %.0f silt %.0f clay %.0f permille, soc %.1f g/kg, pH %.2f, cfvo %.0f permille, bulk density %.2f g/cm3\n", h, h == 0 ? "0-30 cm" : "30-100 cm",
            sand[h] / okd, silt[h] / okd, clay[h] / okd, soc[h] / okd / 10.0, ph[h] / okd / 10.0, cfvo[h] / okd, bdod[h] / okd / 100.0);
        for (int c = 1; c <= 12; ++c)
            if (classes[h][static_cast<std::size_t>(c)]) std::printf("  class %2d %-16s %10llu  %6.2f %% of ok cells\n", c, texture_name(c), static_cast<unsigned long long>(classes[h][static_cast<std::size_t>(c)]), 100.0 * static_cast<double>(classes[h][static_cast<std::size_t>(c)]) / okd);
    }

    // ---- fields
    const SoilLookup lookup = make_lookup(soil, zone);
    const SoilRules& rules = builtin_soil_rules();
    std::ofstream csv;
    if (argc > 6) { csv.open(argv[6]); csv << "id,lon,lat,area_m2,samples,valid,missing,anchor_fallback,top_dominant,top_share_permille,top_classes,top_sand,top_silt,top_clay,top_sand_range,top_soc,top_ph10,sub_dominant,sub_share_permille,sub_sand,sub_clay\n"; }
    const geo::Utm utm(geo::kGRS80);
    struct Acc { std::size_t n = 0; double sand = 0, silt = 0, clay = 0; std::array<std::size_t, 13> dominant{}; };
    std::array<Acc, kRegions.size()> reg{};
    std::vector<double> top_share, sand_range, classes_per_field, clay_range;
    std::size_t fields = 0, no_valid = 0, anchor_fb = 0, incomplete = 0, single_class = 0, sub_differs = 0, with_valid = 0;
    std::uint64_t sample_total = 0;
    std::array<std::size_t, 13> dom_top{};
    for (const auto& ft : ftiles)
        for (const auto& f : ft.fields) {
            if ((f.flags & field_flag::Owned) == 0) continue;
            ++fields;
            const SoilFieldSummary s = summarise_field_soil(f, rules, lookup);
            sample_total += s.samples;
            if (s.anchor_fallback) ++anchor_fb;
            if (s.missing) ++incomplete;
            if (s.valid == 0) { ++no_valid; continue; }
            ++with_valid;
            const auto& t = s.horizon[0]; const auto& u = s.horizon[1];
            top_share.push_back(t.dominant_share_permille / 10.0);
            sand_range.push_back(static_cast<double>(t.sand_max - t.sand_min));
            clay_range.push_back(static_cast<double>(t.clay_max - t.clay_min));
            std::uint32_t m = t.texture_mask; int nclass = 0; while (m) { nclass += static_cast<int>(m & 1u); m >>= 1; }
            classes_per_field.push_back(nclass);
            if (nclass == 1) ++single_class;
            if (t.dominant_texture != u.dominant_texture) ++sub_differs;
            ++dom_top[t.dominant_texture];
            const double e = static_cast<double>(f.anchor.x) / 1000.0, n = static_cast<double>(f.anchor.y) / 1000.0;
            const geo::TmReverse ll = utm.reverse(zone, e, n);
            for (std::size_t r = 0; r < kRegions.size(); ++r)
                if (ll.lon >= kRegions[r].lon0 && ll.lon < kRegions[r].lon1 && ll.lat >= kRegions[r].lat0 && ll.lat < kRegions[r].lat1) {
                    ++reg[r].n; reg[r].sand += t.sand; reg[r].silt += t.silt; reg[r].clay += t.clay; ++reg[r].dominant[t.dominant_texture];
                }
            if (csv) csv << f.id << ',' << ll.lon << ',' << ll.lat << ',' << f.area_m2() << ',' << s.samples << ',' << s.valid << ',' << s.missing << ',' << (s.anchor_fallback ? 1 : 0) << ','
                         << texture_name(t.dominant_texture) << ',' << t.dominant_share_permille << ',' << nclass << ',' << t.sand << ',' << t.silt << ',' << t.clay << ','
                         << (t.sand_max - t.sand_min) << ',' << t.soc << ',' << t.ph10 << ',' << texture_name(u.dominant_texture) << ',' << u.dominant_share_permille << ',' << u.sand << ',' << u.clay << '\n';
        }
    std::printf("\nfields %zu (owned copies), lattice samples %llu (mean %.1f per field)\n", fields, static_cast<unsigned long long>(sample_total), fields ? static_cast<double>(sample_total) / static_cast<double>(fields) : 0.0);
    std::printf("fields without any valid soil sample %zu (%.2f %%); anchor fallback (no lattice point inside) %zu; incomplete (tile missing) %zu\n", no_valid, fields ? 100.0 * static_cast<double>(no_valid) / static_cast<double>(fields) : 0.0, anchor_fb, incomplete);
    std::printf("fields with soil %zu: one topsoil class only %zu (%.1f %%); different dominant class topsoil vs subsoil %zu (%.1f %%)\n", with_valid, single_class, with_valid ? 100.0 * static_cast<double>(single_class) / static_cast<double>(with_valid) : 0.0, sub_differs, with_valid ? 100.0 * static_cast<double>(sub_differs) / static_cast<double>(with_valid) : 0.0);
    std::printf("topsoil classes per field: p50 %.0f p90 %.0f p99 %.0f max %.0f\n", percentile(classes_per_field, .5), percentile(classes_per_field, .9), percentile(classes_per_field, .99), percentile(classes_per_field, 1.0));
    std::printf("topsoil dominant-class share (%%): p5 %.0f p25 %.0f p50 %.0f p75 %.0f\n", percentile(top_share, .05), percentile(top_share, .25), percentile(top_share, .5), percentile(top_share, .75));
    std::printf("topsoil sand range inside a field (permille): p50 %.0f p90 %.0f p99 %.0f max %.0f;  clay range: p50 %.0f p90 %.0f p99 %.0f max %.0f\n",
        percentile(sand_range, .5), percentile(sand_range, .9), percentile(sand_range, .99), percentile(sand_range, 1.0),
        percentile(clay_range, .5), percentile(clay_range, .9), percentile(clay_range, .99), percentile(clay_range, 1.0));
    std::printf("dominant topsoil class over fields:");
    for (int c = 1; c <= 12; ++c) if (dom_top[static_cast<std::size_t>(c)]) std::printf(" %s %zu;", texture_name(c), dom_top[static_cast<std::size_t>(c)]);
    std::printf("\n\nregions (field anchors, topsoil):\n");
    for (std::size_t r = 0; r < kRegions.size(); ++r) {
        const Acc& a = reg[r];
        if (a.n == 0) { std::printf("  %-48s no fields\n", kRegions[r].name); continue; }
        const double n = static_cast<double>(a.n);
        std::printf("  %-48s fields %6zu  mean sand %4.0f silt %4.0f clay %4.0f permille; dominant:", kRegions[r].name, a.n, a.sand / n, a.silt / n, a.clay / n);
        for (int c = 1; c <= 12; ++c) if (a.dominant[static_cast<std::size_t>(c)]) std::printf(" %s %.0f%%;", texture_name(c), 100.0 * static_cast<double>(a.dominant[static_cast<std::size_t>(c)]) / n);
        std::printf("\n");
    }
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

// Scanline fill of a field polygon; `paint(x, y)` is called for every pixel inside.
template <class Paint>
void fill_polygon(Image& img, const FieldPolygon& poly, double e0, double n0, double mpp, Paint paint) {
    const auto to_px = [&](const geom::PointMm& p) { return std::make_pair((static_cast<double>(p.x) / 1000.0 - e0) / mpp, (n0 - static_cast<double>(p.y) / 1000.0) / mpp); };
    std::vector<std::pair<double, double>> pts; std::vector<std::size_t> starts;
    std::vector<const geom::Ring*> rings{&poly.outer}; for (const auto& h : poly.holes) rings.push_back(&h);
    for (const auto* ring : rings) { starts.push_back(pts.size()); for (const auto& p : *ring) pts.push_back(to_px(p)); }
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
            for (int x = static_cast<int>(std::ceil(xs[i] - 0.5)); x <= static_cast<int>(std::floor(xs[i + 1] - 0.5)); ++x) paint(x, y);
    }
    for (std::size_t r = 0; r + 1 < starts.size(); ++r) {
        const std::size_t a = starts[r], n = starts[r + 1] - starts[r];
        for (std::size_t i = 0; i < n; ++i) { const auto& p = pts[a + i]; const auto& q = pts[a + (i + 1) % n]; img.line(p.first, p.second, q.first, q.second, {40, 40, 40}, 0.9); }
    }
}

int png(int argc, char** argv) {
    if (argc < 7) { std::cerr << "usage: png SOIL_DIR SOIL_RELEASE FIELDS_DIR FIELDS_RELEASE out.png [--tile X,Y] [--tiles N] [--mpp M] [--horizon 0|1] [--lonlat LON,LAT]\n"; return 1; }
    int n_tiles = 3; double mpp = 2.0; int horizon = 0; std::optional<std::pair<int, int>> want; std::optional<std::pair<double, double>> lonlat;
    for (int i = 7; i < argc; ++i) {
        const std::string_view a = argv[i];
        if (a == "--tile" && i + 1 < argc) { char* end = nullptr; const long x = std::strtol(argv[++i], &end, 10); if (*end != ',') return 1; const long y = std::strtol(end + 1, &end, 10); if (*end != 0) return 1; want = std::make_pair(static_cast<int>(x), static_cast<int>(y)); }
        else if (a == "--lonlat" && i + 1 < argc) { char* end = nullptr; const double lo = std::strtod(argv[++i], &end); if (*end != ',') return 1; const double la = std::strtod(end + 1, &end); if (*end != 0) return 1; lonlat = std::make_pair(lo, la); }
        else if (a == "--tiles" && i + 1 < argc) n_tiles = std::atoi(argv[++i]);
        else if (a == "--mpp" && i + 1 < argc) mpp = std::atof(argv[++i]);
        else if (a == "--horizon" && i + 1 < argc) horizon = std::clamp(std::atoi(argv[++i]), 0, 1);
        else { std::cerr << "unknown option " << a << "\n"; return 1; }
    }
    auto soil_store = open_store(argv[2]);
    const SoilTiles soil = load_soil_tiles(*soil_store, argv[3]);
    auto fields_store = open_store(argv[4]);
    const auto ftiles = load_field_tiles(*fields_store, argv[5]);
    if (ftiles.empty() || soil.empty()) { std::cerr << "no tiles\n"; return 1; }
    const auto zone = ftiles.front().key.zone;
    std::map<std::pair<int, int>, const FieldTile*> by_key; for (const auto& t : ftiles) by_key[{t.key.x, t.key.y}] = &t;
    std::pair<int, int> origin{};
    if (want) origin = *want;
    else if (lonlat) {
        const geo::Utm utm(geo::kGRS80);
        const auto p = utm.forward(zone, lonlat->second, lonlat->first);
        origin = {static_cast<int>(floor_div(static_cast<std::int64_t>(std::floor(p.easting)), 1024)) - n_tiles / 2, static_cast<int>(floor_div(static_cast<std::int64_t>(std::floor(p.northing)), 1024)) - n_tiles / 2};
    } else {
        std::size_t best = 0;
        for (const auto& t : ftiles) {
            std::size_t n = 0;
            for (int dy = 0; dy < n_tiles; ++dy) for (int dx = 0; dx < n_tiles; ++dx) { const auto it = by_key.find({t.key.x + dx, t.key.y + dy}); if (it != by_key.end()) n += it->second->fields.size(); }
            if (n > best) { best = n; origin = {t.key.x, t.key.y}; }
        }
    }
    const SoilLookup lookup = make_lookup(soil, zone);
    const TileKey first{zone, 2, origin.first, origin.second};
    const double e0 = static_cast<double>(first.min_easting()), n_top = static_cast<double>(first.min_northing()) + 1024.0 * n_tiles;
    const int size = static_cast<int>(std::lround(1024.0 * n_tiles / mpp));
    Image img(size, size);
    const auto soil_colour = [&](int x, int y) -> std::optional<std::array<std::uint8_t, 3>> {
        const std::int64_t e_mm = static_cast<std::int64_t>(std::floor((e0 + (x + 0.5) * mpp) * 1000.0)), n_mm = static_cast<std::int64_t>(std::floor((n_top - (y + 0.5) * mpp) * 1000.0));
        const auto s = lookup(e_mm, n_mm);
        if (!s) return std::nullopt;
        switch (s->state) {
        case SoilState::Ok: return texture_colour(s->horizon[static_cast<std::size_t>(horizon)].texture);
        case SoilState::Water: return std::array<std::uint8_t, 3>{120, 170, 235};
        case SoilState::BuiltUp: return std::array<std::uint8_t, 3>{150, 150, 150};
        default: return std::array<std::uint8_t, 3>{235, 235, 235};
        }
    };
    for (int y = 0; y < size; ++y) for (int x = 0; x < size; ++x) {
        const auto c = soil_colour(x, y);
        img.put(x, y, c ? *c : std::array<std::uint8_t, 3>{255, 255, 255});
        img.put(x, y, {255, 255, 255}, 0.62); // fade outside the fields
    }
    std::map<std::uint64_t, bool> drawn;
    for (int dy = 0; dy < n_tiles; ++dy) for (int dx = 0; dx < n_tiles; ++dx) {
        const auto it = by_key.find({origin.first + dx, origin.second + dy}); if (it == by_key.end()) continue;
        for (const auto& f : it->second->fields) {
            if (!drawn.emplace(f.id, true).second) continue;
            fill_polygon(img, f.polygon, e0, n_top, mpp, [&](int x, int y) { const auto c = soil_colour(x, y); if (c) { auto* p = &img.rgb[(static_cast<std::size_t>(y) * static_cast<std::size_t>(img.w) + static_cast<std::size_t>(x)) * 3]; p[0] = (*c)[0]; p[1] = (*c)[1]; p[2] = (*c)[2]; } });
        }
    }
    if (!write_png(argv[6], img)) { std::cerr << "cannot write " << argv[6] << "\n"; return 1; }
    std::printf("window tiles (%d,%d) %dx%d, %dx%d px at %.2f m/px, %s -> %s\n", origin.first, origin.second, n_tiles, n_tiles, size, size, mpp, horizon == 0 ? "topsoil" : "subsoil", argv[6]);
    std::printf("legend:");
    for (int c = 1; c <= 12; ++c) { const auto k = texture_colour(c); std::printf(" %s=(%d,%d,%d);", texture_name(c), k[0], k[1], k[2]); }
    std::printf(" water=(120,170,235) built-up=(150,150,150) no source=(235,235,235)\n");
    return 0;
}
} // namespace

int main(int argc, char** argv) {
    if (argc >= 2 && std::string_view(argv[1]) == "stats") return stats(argc, argv);
    if (argc >= 2 && std::string_view(argv[1]) == "png") return png(argc, argv);
    std::cerr << "usage: g2m_soil_probe stats|png ...\n";
    return 1;
}
