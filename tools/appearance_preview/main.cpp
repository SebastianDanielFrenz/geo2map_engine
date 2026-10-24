#include "g2m/world/bridge_visual.h"
#include "g2m/world/building_mesh.h"
#include "g2m/world/road_structures.h"
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using nlohmann::json;
int main(int argc, char **argv) {
  if (argc != 2) {
    std::cerr << "usage: g2m_appearance_preview OUTPUT.json\n";
    return 2;
  }
  using namespace g2m::world;
  json result = {{"format", "g2m.appearance_preview/1"},
                 {"buildings", json::array()},
                 {"railings", json::array()}};
  const char *types[] = {"detached",   "terrace", "apartments", "commercial",
                         "industrial", "house",   "house",      "house"};
  const char *materials[] = {"",      "brick", "",     "",
                             "metal", "stone", "wood", "plaster"};
  for (int i = 0; i < 8; ++i) {
    Building b;
    b.id = 42 + i;
    b.type = types[i];
    b.material = materials[i];
    b.height = i == 2 ? 15 : 9;
    b.base = 0;
    b.bottom = 0;
    b.roof_shape = (i == 2 || i == 3 || i == 4) ? "flat" : "gabled";
    b.roof_height = 2.2;
    b.outers = {{{0, 0}, {12, 0}, {12, 8}, {0, 8}}};
    auto mesh = build_building_mesh(b);
    json v = json::array();
    for (const auto &p : mesh.vertices)
      v.push_back({p.position, p.normal, p.colour, p.uv,
                   static_cast<unsigned>(p.material)});
    result["buildings"].push_back(
        {{"name", std::string(types[i]) + " / " +
                      (b.material.empty() ? "inferred" : b.material)},
         {"vertices", std::move(v)}});
  }
  for (const char *highway : {"motorway", "residential", "footway"}) {
    g2m::RoadGraphWay way;
    way.osm_id = 1;
    way.tags = {{"highway", highway}, {"bridge", "yes"}};
    RoadDeck d;
    d.end_station = 12;
    for (int row = 0; row <= 12; ++row)
      for (int col = 0; col < 5; ++col)
        d.mesh.positions.insert(
            d.mesh.positions.end(),
            {static_cast<float>(row), static_cast<float>(col * 2 - 4), 0.f});
    const auto mesh = build_bridge_visual(d, bridge_facts(way));
    result["railings"].push_back({{"name", highway},
                                  {"positions", mesh.mesh.positions},
                                  {"normals", mesh.mesh.normals},
                                  {"indices", mesh.mesh.indices}});
  }
  std::ofstream out(argv[1]);
  out << result.dump();
  if (!out)
    return 1;
  std::cout << "APPEARANCE_PREVIEW buildings=" << result["buildings"].size()
            << " railings=" << result["railings"].size() << "\n";
}
