// Import one geographic source mosaic into a native raw elevation container.
// Explicit output path; optional --store DIR --scope NAME imports into a
// separate source store.
#include "g2m/core/hash.h"
#include "g2m/decode/geographic_dem.h"
#include "g2m/server/tile_store.h"
#include <algorithm>
#include <charconv>
#include <chrono>
#include <fstream>
#include <iostream>
int main(int argc, char **argv) {
  try {
    if (argc < 6)
      throw std::runtime_error(
          "usage: g2m_geographic_dem utm32n X Y "
          "OUTPUT.g2m SOURCE.tif [SOURCE.tif ...] [--store DIR --scope NAME]");
    const auto zone = g2m::geo::parse_utm_scheme(argv[1]);
    if (!zone)
      throw std::runtime_error("invalid UTM scheme");
    auto integer = [](const char *text) {
      std::int64_t v = 0;
      const std::string_view s(text);
      const auto r = std::from_chars(s.data(), s.data() + s.size(), v);
      if (r.ec != std::errc{} || r.ptr != s.data() + s.size())
        throw std::runtime_error("invalid tile index");
      return v;
    };
    const auto x = integer(argv[2]), y = integer(argv[3]);
    if (x < 0 || x > 1000 || y < 0 || y > 10000)
      throw std::runtime_error("tile outside UTM bounds");
    const g2m::TileKey key{*zone, 2, static_cast<std::int32_t>(x),
                           static_cast<std::int32_t>(y)};
    std::vector<std::shared_ptr<g2m::decode::IByteSource>> sources;
    std::vector<g2m::ContentHash> hashes;
    std::string store_dir, scope;
    for (int i = 5; i < argc; ++i) {
      const std::string arg(argv[i]);
      if (arg == "--store" || arg == "--scope") {
        if (++i == argc)
          throw std::runtime_error("missing option value");
        (arg == "--store" ? store_dir : scope) = argv[i];
        continue;
      }
      if (arg.starts_with("--"))
        throw std::runtime_error("unknown option: " + arg);
      auto file = std::make_shared<g2m::decode::FileByteSource>(argv[i]);
      g2m::Sha256 hash;
      std::vector<std::byte> block;
      for (std::uint64_t offset = 0; offset < file->size();) {
        const auto n = std::min<std::uint64_t>(1048576, file->size() - offset);
        file->read(offset, n, block);
        hash.update(block);
        offset += n;
      }
      hashes.push_back(hash.finish());
      sources.push_back(file);
    }
    if (store_dir.empty() != scope.empty())
      throw std::runtime_error("--store and --scope are required together");
    const auto begin = std::chrono::steady_clock::now();
    g2m::decode::GeographicDem dem(std::move(sources));
    auto tile = dem.reproject(*g2m::tile_georef(
        key, 1024, g2m::SampleRegistration::Cell, g2m::RowOrder::NorthToSouth));
    std::size_t valid = 0;
    std::int32_t lo = INT32_MAX, hi = INT32_MIN;
    for (auto h : tile.heights.data())
      if (h != g2m::kHeightNoData) {
        ++valid;
        lo = std::min(lo, h);
        hi = std::max(hi, h);
      }
    if (!valid)
      throw std::runtime_error(
          "no covered elevation samples; no output written");
    const auto encoded = g2m::encode_src_elev(tile);
    if (!encoded)
      throw std::runtime_error(encoded.error().message);
    const auto body = g2m::encode_body(*encoded);
    if (!body)
      throw std::runtime_error(body.error().message);
    std::sort(hashes.begin(), hashes.end());
    hashes.erase(std::unique(hashes.begin(), hashes.end()), hashes.end());
    g2m::TileHeader header;
    header.layer = "g2m.src.elev.glo30";
    header.layer_version = 1;
    header.key = key;
    header.datum = g2m::VerticalDatum::EGM2008;
    header.inputs = hashes;
    const auto container = g2m::assemble_container(header, *body);
    if (!container)
      throw std::runtime_error(container.error().message);
    std::ofstream out(argv[4], std::ios::binary | std::ios::trunc);
    out.write(reinterpret_cast<const char *>(container->data()),
              static_cast<std::streamsize>(container->size()));
    out.close();
    if (!out)
      throw std::runtime_error("output write failed");
    if (!store_dir.empty()) {
      g2m::TileStoreConfig config;
      config.dir = store_dir;
      config.name = "tiles";
      auto store = g2m::TileStore::open(config);
      if (!store)
        throw std::runtime_error(store.error().message);
      auto put = (*store)->put({scope, header.layer, key}, header, *body, true);
      if (!put)
        throw std::runtime_error(put.error().message);
    }
    std::cout << "GEOGRAPHIC_DEM samples=" << valid
              << " nodata=" << tile.heights.size() - valid
              << " min_m=" << g2m::dequantise_height(lo)
              << " max_m=" << g2m::dequantise_height(hi)
              << " blocks=" << dem.decoded_blocks()
              << " hash=" << g2m::to_hex(g2m::content_hash(*body))
              << " seconds="
              << std::chrono::duration<double>(
                     std::chrono::steady_clock::now() - begin)
                     .count()
              << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
