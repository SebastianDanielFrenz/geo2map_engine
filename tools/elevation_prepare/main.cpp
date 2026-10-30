// Explicit offline preparation into a NEW canonical elevation scope.
#include "g2m/core/hash.h"
#include "g2m/decode/geographic_dem.h"
#include "g2m/decode/terrarium_dem.h"
#include "g2m/server/tile_store.h"
#include "g2m/world/elevation_blend.h"
#include "g2m/world/elevation_ground.h"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <nlohmann/json.hpp>
#include <set>

using namespace g2m;
using namespace g2m::world;
using json = nlohmann::json;
namespace fs = std::filesystem;
namespace {
struct OutputClaim {
  fs::path claim;
  explicit OutputClaim(const fs::path &directory) {
    fs::create_directories(directory);
    claim = directory / ".elevation_prepare_lock";
    if (!fs::create_directory(claim))
      throw std::runtime_error("another preparation writer owns this output "
                               "store (or a stale preparation lock remains)");
  }
  ~OutputClaim() {
    std::error_code ec;
    fs::remove(claim, ec);
  }
  OutputClaim(const OutputClaim &) = delete;
  OutputClaim &operator=(const OutputClaim &) = delete;
};
std::vector<std::byte> read(const fs::path &path, std::uint64_t limit) {
  decode::FileByteSource source(path.string());
  if (source.size() > limit)
    throw std::runtime_error("file exceeds budget: " + path.string());
  std::vector<std::byte> bytes;
  source.read(0, source.size(), bytes);
  return bytes;
}
json document(const fs::path &path) {
  const auto bytes = read(path, 4 * 1024 * 1024);
  auto j =
      json::parse(reinterpret_cast<const char *>(bytes.data()),
                  reinterpret_cast<const char *>(bytes.data() + bytes.size()),
                  nullptr, false);
  if (j.is_discarded() || !j.is_object())
    throw std::runtime_error("invalid JSON: " + path.string());
  return j;
}
ContentHash file_hash(const fs::path &path) {
  decode::FileByteSource source(path.string());
  Sha256 h;
  std::vector<std::byte> buffer;
  for (std::uint64_t pos = 0; pos < source.size();) {
    const auto n = std::min<std::uint64_t>(1048576, source.size() - pos);
    source.read(pos, n, buffer);
    h.update(buffer);
    pos += n;
  }
  return h.finish();
}
RasterGeoref expanded(const TileKey &key, int halo) {
  auto g =
      *tile_georef(key, 1024, SampleRegistration::Cell, RowOrder::NorthToSouth);
  g.west_mm -= halo * 1000;
  g.south_mm -= halo * 1000;
  g.width += 2 * halo;
  g.height += 2 * halo;
  return g;
}
HeightRaster crop(const HeightRaster &source, int margin) {
  HeightRaster result(source.width() - 2 * margin, source.height() - 2 * margin,
                      kHeightNoData);
  for (int y = 0; y < result.height(); ++y)
    for (int x = 0; x < result.width(); ++x)
      result.at(x, y) = source.at(x + margin, y + margin);
  return result;
}
ElevationSourceKind source_kind(const std::string &type) {
  if (type == "dsm")
    return ElevationSourceKind::Dsm;
  if (type == "dtm")
    return ElevationSourceKind::Dtm;
  if (type == "unknown")
    return ElevationSourceKind::Unknown;
  throw std::runtime_error("surface must be dsm, dtm or unknown");
}
} // namespace
int main(int argc, char **argv) {
  try {
    if (argc != 2)
      throw std::runtime_error("usage: g2m_elevation_prepare JOB.json");
    const auto job_path = fs::absolute(argv[1]);
    const auto root = job_path.parent_path();
    auto path = [&](const std::string &s) {
      const fs::path p(s);
      return p.is_absolute() ? p : (root / p).lexically_normal();
    };
    const auto job = document(job_path);
    if (job.value("format", "") != "g2m.elevation_job/1")
      throw std::runtime_error("unknown elevation job format");
    const auto key = parse_tile_path(job.at("target").get<std::string>());
    if (!key || key->level != 2)
      throw std::runtime_error("target must be a level-2 UTM tile");
    const auto &output = job.at("output");
    const auto out_dir = path(output.at("store").get<std::string>());
    const auto scope = output.at("scope").get<std::string>();
    const auto layer = output.value("layer", "g2m.src.elev.composite");
    if (scope.empty() || !layer.starts_with(kSrcElevPrefix) ||
        !is_valid_layer_name(layer) ||
        layer.substr(kSrcElevPrefix.size()).find('.') != std::string::npos)
      throw std::runtime_error("invalid output scope or elevation layer");
    std::vector<ContentHash> inputs;
    json provenance = {{"algorithm", "g2m.elevation_prepare/1"},
                       {"target", to_string(*key)},
                       {"glo30", json::array()},
                       {"primary", json::array()}};
    std::vector<std::shared_ptr<decode::IByteSource>> files;
    if (job.contains("glo30_files")) {
      if (!job["glo30_files"].is_array() || job["glo30_files"].size() > 9)
        throw std::runtime_error("GLO-30 input exceeds nine-file budget");
      for (const auto &f : job["glo30_files"]) {
        const auto p = path(f.get<std::string>());
        const auto hash = file_hash(p);
        inputs.push_back(hash);
        provenance["glo30"].push_back(to_hex(hash));
        files.push_back(std::make_shared<decode::FileByteSource>(p.string()));
      }
    }
    std::unique_ptr<decode::GeographicDem> glo;
    if (!files.empty())
      glo = std::make_unique<decode::GeographicDem>(std::move(files));
    std::map<int, std::unique_ptr<decode::TerrariumDem>, std::greater<>>
        terrarium;
    std::string map_surface = "unknown";
    if (job.contains("mapterhorn_manifest")) {
      const auto manifest_path =
          path(job.at("mapterhorn_manifest").get<std::string>());
      const auto manifest = document(manifest_path);
      if (manifest.value("format", "") != "g2m.mapterhorn_rgba/1" ||
          manifest.value("tile_size", 0) != 512 ||
          manifest.value("raw_format", "") != "rgba8-row-major")
        throw std::runtime_error("unsupported Mapterhorn preparation format");
      const auto &tiles = manifest.at("tiles");
      if (!tiles.is_array() || tiles.empty() || tiles.size() > 256)
        throw std::runtime_error(
            "Mapterhorn import supports 1-256 decoded tiles");
      std::map<int, std::vector<decode::TerrariumTile>> groups;
      for (const auto &t : tiles) {
        for (const auto *field : {"z", "x", "y"})
          if (!t.at(field).is_number_integer() || t.at(field) < 0 ||
              t.at(field) > 33554432)
            throw std::runtime_error("invalid integer XYZ coordinate");
        const int z = t.at("z").get<int>(), x = t.at("x").get<int>(),
                  y = t.at("y").get<int>();
        if (z < 0 || z > 24 || x < 0 || y < 0 || x >= (1 << z) || y >= (1 << z))
          throw std::runtime_error("invalid XYZ key");
        const auto relative = std::to_string(z) + "/" + std::to_string(x) +
                              "/" + std::to_string(y) + ".rgba";
        if (t.at("path") != relative || t.at("width") != 512 ||
            t.at("height") != 512 || t.at("size") != 1048576)
          throw std::runtime_error("invalid RGBA path/dimensions");
        const auto bytes =
            read(manifest_path.parent_path() / relative, 1048576);
        const auto hash = Sha256::of(bytes);
        if (bytes.size() != 1048576 ||
            to_hex(hash) != t.at("sha256").get<std::string>())
          throw std::runtime_error("RGBA source hash mismatch");
        std::vector<std::uint8_t> rgba(bytes.size());
        std::transform(
            bytes.begin(), bytes.end(), rgba.begin(),
            [](std::byte b) { return static_cast<std::uint8_t>(b); });
        groups[z].push_back({z, x, y, std::move(rgba)});
        inputs.push_back(hash);
      }
      const auto manifest_hash = file_hash(manifest_path);
      inputs.push_back(manifest_hash);
      provenance["mapterhorn_manifest"] = to_hex(manifest_hash);
      map_surface = manifest.value("source_metadata", json::object())
                        .value("surface", "unknown");
      for (auto &[z, tiles_at_zoom] : groups)
        terrarium[z] = std::make_unique<decode::TerrariumDem>(
            std::move(tiles_at_zoom),
            decode::TerrariumSourceBounds{z, 0, (1 << z) - 1, 0, (1 << z) - 1},
            256u * 1024u * 1024u);
    }
    if (!glo && terrarium.empty() && !job.contains("primary"))
      throw std::runtime_error("job has no elevation sources");
    GroundElevationOptions ground_options;
    ground_options.spacing_m = 32;
    const bool ground = job.value("ground", true);
    ElevationBlendOptions blend;
    blend.radius_m = job.value("blend_radius_m", 256.0);
    blend.max_adjust_m = job.value("max_adjust_m", 20.0);
    if (!std::isfinite(blend.radius_m) || blend.radius_m < 0 ||
        blend.radius_m > 256)
      throw std::runtime_error("blend radius must be 0-256m");
    constexpr int halo = 256;
    const auto wide = expanded(*key, 2 * halo);
    const auto middle = expanded(*key, halo);
    auto prepare = [&](const GeographicElevationSampler &sampler,
                       ElevationSourceKind kind) {
      auto result = prepare_ground_elevation(
          wide, sampler, ground ? kind : ElevationSourceKind::Unknown,
          ground_options);
      if (!result)
        throw std::runtime_error(result.error().message);
      std::cout << "GROUND lowered_coarse=" << result->coarse_lowered
                << " raw_edge_fallback=" << result->raw_fallback_samples
                << '\n';
      return std::move(result->tile.heights);
    };
    HeightRaster fallback(wide.width, wide.height, kHeightNoData);
    if (glo)
      fallback =
          prepare([&](double lat, double lon) { return glo->sample(lat, lon); },
                  ElevationSourceKind::Dsm);
    if (!terrarium.empty()) {
      auto map = prepare(
          [&](double lat, double lon) -> std::optional<double> {
            for (const auto &[z, dem] : terrarium) {
              (void)z;
              const auto h = dem->sample(lat, lon);
              if (h)
                return h;
            }
            return {};
          },
          source_kind(map_surface));
      auto joined = blend_elevation_sources(
          map, fallback, {halo, halo, middle.width, middle.height}, blend);
      if (!joined)
        throw std::runtime_error(joined.error().message);
      fallback = std::move(joined->heights);
    } else
      fallback = crop(fallback, halo);
    HeightRaster preferred(middle.width, middle.height, kHeightNoData);
    std::unique_ptr<TileStore> primary;
    std::string primary_scope, primary_layer;
    if (job.contains("primary")) {
      const auto &p = job.at("primary");
      const auto directory = path(p.at("store").get<std::string>());
      primary_scope = p.at("scope").get<std::string>();
      primary_layer = p.value("layer", "g2m.src.elev.dgm1");
      if (fs::weakly_canonical(directory) == fs::weakly_canonical(out_dir) &&
          primary_scope == scope)
        throw std::runtime_error(
            "output must use a new scope, never overwrite primary inputs");
      TileStoreConfig config;
      config.dir = directory;
      config.name = p.value("name", "tiles");
      config.read_only = true;
      auto opened = TileStore::open(config);
      if (!opened)
        throw std::runtime_error(opened.error().message);
      primary = std::move(*opened);
      std::map<TileKey, std::optional<ElevTile>> cache;
      for (int y = 0; y < middle.height; ++y)
        for (int x = 0; x < middle.width; ++x) {
          const auto e = middle.sample_easting_m(x),
                     n = middle.sample_northing_m(y);
          const auto source_key = tile_key_at(key->zone, 2, e, n);
          if (!source_key)
            continue;
          auto [it, inserted] = cache.try_emplace(*source_key);
          if (inserted) {
            auto stored =
                primary->get({primary_scope, primary_layer, *source_key});
            if (!stored)
              throw std::runtime_error(stored.error().message);
            json dep = {{"key", to_string(*source_key)},
                        {"scope", primary_scope},
                        {"layer", primary_layer},
                        {"present", static_cast<bool>(*stored)}};
            if (*stored) {
              dep["body_hash"] = to_hex((*stored)->header.body_hash);
              dep["datum"] = static_cast<unsigned>((*stored)->header.datum);
              dep["height_offset"] = (*stored)->header.height_offset;
            }
            provenance["primary"].push_back(std::move(dep));
            if (*stored) {
              auto body = decode_body((*stored)->body);
              if (!body)
                throw std::runtime_error(body.error().message);
              auto tile = decode_src_elev(*body);
              if (!tile)
                throw std::runtime_error(tile.error().message);
              if (tile->georef != *tile_georef(*source_key, 1024,
                                               SampleRegistration::Cell,
                                               RowOrder::NorthToSouth))
                throw std::runtime_error("primary must be a registered 1m "
                                         "north-to-south raw elevation tile");
              if ((*stored)->header.height_offset != 0)
                throw std::runtime_error(
                    "normalize primary height_offset before import");
              inputs.push_back((*stored)->header.body_hash);
              it->second = std::move(*tile);
            }
          }
          if (it->second) {
            const auto &t = *it->second;
            const int col =
                static_cast<int>(std::floor(e - t.georef.west_mm / 1000.0));
            const int row =
                1023 -
                static_cast<int>(std::floor(n - t.georef.south_mm / 1000.0));
            preferred.at(x, y) = t.heights.at(col, row);
          }
        }
    }
    if (job.value("evaluate", false) && glo && primary) {
      const geo::Utm utm;
      std::size_t count = 0;
      double raw_sum = 0, prepared_sum = 0, raw_sq = 0, prepared_sq = 0;
      for (int y = halo; y < halo + 1024; y += 8)
        for (int x = halo; x < halo + 1024; x += 8) {
          const auto p = preferred.at(x, y), f = fallback.at(x, y);
          if (p == kHeightNoData || f == kHeightNoData)
            continue;
          const auto ll = utm.reverse(key->zone, middle.sample_easting_m(x),
                                      middle.sample_northing_m(y));
          const auto raw = glo->sample(ll.lat, ll.lon);
          if (!raw)
            continue;
          const double e = *raw - dequantise_height(p),
                       g = dequantise_height(f) - dequantise_height(p);
          raw_sum += e;
          prepared_sum += g;
          raw_sq += e * e;
          prepared_sq += g * g;
          ++count;
        }
      if (count)
        std::cout << "EVALUATION samples=" << count
                  << " raw_rmse_m=" << std::sqrt(raw_sq / count)
                  << " prepared_rmse_m=" << std::sqrt(prepared_sq / count)
                  << " raw_bias_m=" << raw_sum / count
                  << " prepared_bias_m=" << prepared_sum / count << '\n';
    }
    auto joined = blend_elevation_sources(preferred, fallback,
                                          {halo, halo, 1024, 1024}, blend);
    if (!joined)
      throw std::runtime_error(joined.error().message);
    ElevTile tile{*tile_georef(*key, 1024, SampleRegistration::Cell,
                               RowOrder::NorthToSouth),
                  std::move(joined->heights)};
    const auto valid =
        std::count_if(tile.heights.data().begin(), tile.heights.data().end(),
                      [](auto h) { return h != kHeightNoData; });
    if (valid == 0)
      throw std::runtime_error("no valid samples; no output written");
    auto body = encode_src_elev(tile);
    if (!body)
      throw std::runtime_error(body.error().message);
    auto encoded = encode_body(*body);
    if (!encoded)
      throw std::runtime_error(encoded.error().message);
    const auto config_hash = Sha256::of(
        std::string("g2m.elevation_prepare/1 ground=") +
        (ground ? "yes" : "no") + " radius=" + std::to_string(blend.radius_m) +
        " max_adjust=" + std::to_string(blend.max_adjust_m) +
        " map_surface=" + map_surface);
    provenance["ground"] = ground;
    provenance["ground_spacing_m"] = 32;
    provenance["blend_radius_m"] = blend.radius_m;
    provenance["max_adjust_m"] = blend.max_adjust_m;
    provenance["map_surface"] = map_surface;
    const auto provenance_hash = Sha256::of(provenance.dump());
    inputs.push_back(config_hash);
    inputs.push_back(provenance_hash);
    std::sort(inputs.begin(), inputs.end());
    inputs.erase(std::unique(inputs.begin(), inputs.end()), inputs.end());
    TileHeader header;
    header.layer = layer;
    header.key = *key;
    header.layer_version = 1;
    header.inputs = inputs;
    header.datum = primary || !terrarium.empty() ? VerticalDatum::Unspecified
                                                 : VerticalDatum::EGM2008;
    TileStoreConfig config;
    config.dir = out_dir;
    config.name = "tiles";
    OutputClaim output_claim(out_dir);
    auto destination = TileStore::open(config);
    if (!destination)
      throw std::runtime_error(destination.error().message);
    const TileAddress address{scope, layer, *key};
    auto old = (*destination)->get(address);
    if (!old)
      throw std::runtime_error(old.error().message);
    auto desired = assemble_container(header, *encoded);
    if (!desired)
      throw std::runtime_error(desired.error().message);
    if (*old && (*old)->container() != *desired)
      throw std::runtime_error(
          "existing prepared output/provenance differs; choose a new scope");
    const auto record_dir = out_dir / "elevation_provenance";
    fs::create_directories(record_dir);
    const auto record = record_dir / (to_hex(provenance_hash) + ".json");
    const auto text = provenance.dump(2) + "\n";
    if (fs::exists(record)) {
      const auto bytes = read(record, 4 * 1024 * 1024);
      const std::string old_text(reinterpret_cast<const char *>(bytes.data()),
                                 bytes.size());
      if (old_text != text)
        throw std::runtime_error("existing provenance record is corrupt");
    } else {
      const auto temp = record.string() + ".part";
      std::ofstream f(temp, std::ios::binary | std::ios::trunc);
      f << text;
      f.close();
      if (!f)
        throw std::runtime_error("provenance write failed");
      fs::rename(temp, record);
    }
    if (!*old) {
      auto put = (*destination)->put(address, header, *encoded, true);
      if (!put)
        throw std::runtime_error(put.error().message);
    }
    std::cout << "ELEVATION_PREPARED valid=" << valid
              << " primary_preserved=" << joined->diagnostics.primary_preserved
              << " fallback_filled=" << joined->diagnostics.fallback_filled
              << " adjusted=" << joined->diagnostics.fallback_adjusted
              << " without_overlap="
              << joined->diagnostics.fallback_without_overlap
              << " provenance=" << to_hex(provenance_hash)
              << " container_sha=" << to_hex(content_hash(*desired))
              << " hash=" << to_hex(content_hash(*encoded)) << '\n';
    return 0;
  } catch (const std::exception &e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
