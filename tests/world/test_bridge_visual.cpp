#include "g2m/world/bridge_visual.h"
#include "g2m/world/road_structures.h"
#include <catch2/catch_test_macros.hpp>
#include <cmath>
namespace {
g2m::world::RoadDeck deck(double length=40){g2m::world::RoadDeck d;d.end_station=length;
    d.mesh.origin[0]=450000;d.mesh.origin[1]=5500000;d.way_id=123;
    const int count=static_cast<int>(std::ceil(length));
    for(int i=0;i<=count;++i)for(int j=0;j<5;++j){d.mesh.positions.push_back(static_cast<float>(length*i/count));
        d.mesh.positions.push_back(static_cast<float>(j*2-4));d.mesh.positions.push_back(static_cast<float>(100+.01*i));}
    return d;
}
}
TEST_CASE("bridge families preserve source IDs and physical deck", "[world][bridge_visual]"){
    auto d=deck();const auto original=d.mesh.positions;
    for(const auto* family:{"", "beam","truss","arch","suspension","cable-stayed","movable","unexpected"}){
        g2m::RoadGraphWay way;way.osm_id=123;way.tags={{"bridge:structure",family},{"wikidata","Q12345"},{"ref:nbi","NBI-001"}};
        const auto facts=g2m::world::bridge_facts(way);CHECK(facts.source_id=="osm:way/123");CHECK(facts.wikidata_id=="Q12345");CHECK(facts.inventory_id=="NBI-001");
        const auto visual=g2m::world::build_bridge_visual(d,facts);INFO(family);REQUIRE(!visual.mesh.positions.empty());
        CHECK(visual.mesh.origin[0]==d.mesh.origin[0]);CHECK(d.mesh.positions==original);
        for(auto i:visual.mesh.indices)CHECK(i<visual.mesh.positions.size()/3);
        for(auto p:visual.mesh.positions)CHECK(std::isfinite(p));
        CHECK(visual.mesh.positions.size()==visual.mesh.normals.size());
    }
}
TEST_CASE("inferred and tunnel decks do not get invented bridge bodies", "[world][bridge_visual]"){
    auto d=deck();d.inferred=true;CHECK(g2m::world::build_bridge_visual(d,{}).mesh.positions.empty());
    d.inferred=false;d.tunnel_floor=true;CHECK(g2m::world::build_bridge_visual(d,{}).mesh.positions.empty());
    d.tunnel_floor=false;d.end_station=1001;CHECK(g2m::world::build_bridge_visual(d,{}).unsupported);
}
TEST_CASE("long bridge visuals stay within the mesh index budget", "[world][bridge_visual]"){
    auto d=deck(1000);g2m::world::BridgeFacts f;f.explicit_form=true;f.form=g2m::world::BridgeForm::Truss;
    const auto v=g2m::world::build_bridge_visual(d,f);CHECK_FALSE(v.unsupported);CHECK(v.mesh.positions.size()/3<=40000);
    CHECK(v.mesh.positions.size()/3+d.mesh.positions.size()/3<65536);
}

