#include "g2m/world/building_mesh.h"
#include "building_test_data.h"
#include <catch2/catch_test_macros.hpp>
#include <algorithm>
#include <bit>
#include <cmath>
#include <limits>

namespace {
using namespace g2m::world;
Building house() {
    Building b; b.id=42;b.type="house";b.height=9;b.base=100;b.bottom=98;
    b.outers={{{450000,5500000},{450012,5500000},{450012,5500008},{450000,5500008}}};return b;
}
BuildingMeshOptions opts() {BuildingMeshOptions o;o.origin_e=450000;o.origin_n=5500000;return o;}
double roof_area(const BuildingMesh& m) {
    double area=0;for(std::size_t i=0;i<m.vertices.size();i+=3){if(m.vertices[i].material!=BuildingMaterial::Roof)continue;
        const auto a=m.vertices[i].position,b=m.vertices[i+1].position,c=m.vertices[i+2].position;
        area+=std::abs((b[0]-a[0])*(c[1]-a[1])-(c[0]-a[0])*(b[1]-a[1]))*.5;}
    return area;
}
std::uint64_t hash(const BuildingMesh& m){std::uint64_t h=14695981039346656037ULL;
    for(const auto& v:m.vertices){for(const auto& field:{v.position,v.normal,v.colour})for(float x:field){h^=std::bit_cast<std::uint32_t>(x);h*=1099511628211ULL;}
        h^=static_cast<unsigned>(v.material);h*=1099511628211ULL;}return h;
}
}
TEST_CASE("procedural roof forms preserve mapped total height and finite unit normals", "[world][building_mesh]") {
    for(const auto* shape:{"flat","gabled","hipped","pyramidal","skillion"}){
        auto b=house();b.roof_shape=shape;b.roof_height=2.5;const auto m=build_building_mesh(b,opts());
        INFO(shape);REQUIRE(!m.vertices.empty());CHECK_FALSE(m.unsupported_roof);CHECK(m.vertices.size()%3==0);
        float max_z=-10000;for(const auto& v:m.vertices){for(auto x:v.position)CHECK(std::isfinite(x));
            max_z=std::max(max_z,v.position[2]);const auto n=v.normal;CHECK(std::abs(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]-1)<1e-5);
            CHECK(v.position[2]<=109.0001);}
        CHECK(std::abs(max_z-109)<1e-4);CHECK(roof_area(m)>=95.99);
        CHECK(m.windows>0);
    }
}
TEST_CASE("mapped building metadata survives extraction without moving the footprint", "[world][building_mesh]") {
    g2m::osm::OsmBuilder src;
    g2m_building_test::square(src,77,700,358500,5580900,6,{{"building","house"},{"height","11"},
        {"roof:shape","hipped"},{"roof:height","2,5 m"},{"roof:colour","#804020"},{"roof:orientation","across"},
        {"roof:direction","90"},{"building:levels","2"},{"building:material","brick"}});
    auto data=src.build();REQUIRE(data);g2m::OsmTile tile;tile.data=std::move(data).value();
    auto buildings=extract_buildings(tile,g2m_building_test::kOsmKey,0,8,3,&g2m_building_test::ground);
    REQUIRE(buildings.size()==1);const auto& b=buildings.front();CHECK(b.height==11);CHECK(b.roof_height==2.5);
    CHECK(b.roof_shape=="hipped");CHECK(b.roof_colour=="#804020");CHECK(b.roof_orientation=="across");
    CHECK(b.roof_direction=="90");CHECK(b.levels==2);CHECK(b.material=="brick");
}
TEST_CASE("courtyards and concave roofs are not replaced by bounding boxes", "[world][building_mesh]") {
    auto b=house();b.roof_shape="gabled";
    b.inners={{{450004,5500002},{450008,5500002},{450008,5500006},{450004,5500006}}};
    const auto m=build_building_mesh(b,opts());CHECK(m.roof==RoofForm::Flat);CHECK(m.unsupported_roof);
    CHECK(std::abs(roof_area(m)-80)<1e-4);
    b.inners.clear();b.outers={{{450000,5500000},{450012,5500000},{450012,5500004},{450006,5500004},{450006,5500008},{450000,5500008}}};
    const auto c=build_building_mesh(b,opts());CHECK(c.roof==RoofForm::Flat);CHECK(std::abs(roof_area(c)-72)<1e-4);
}
TEST_CASE("facade cost is bounded and skyline geometry keeps the same silhouette", "[world][building_mesh]") {
    auto b=house();b.height=1800;b.levels=600;auto o=opts();o.max_windows=17;
    const auto full=build_building_mesh(b,o);CHECK(full.windows==17);CHECK(full.vertices.size()<400);
    o.facade_detail=false;const auto lite=build_building_mesh(b,o);CHECK(lite.windows==0);CHECK(lite.vertices.size()<full.vertices.size());
    auto max_z=[](const auto& m){float z=-10000;for(const auto& v:m.vertices)z=std::max(z,v.position[2]);return z;};
    CHECK(max_z(full)==max_z(lite));
}
TEST_CASE("feature appearance is stable across requests and render-origin shifts", "[world][building_mesh]") {
    auto b=house();auto o=opts();const auto first=build_building_mesh(b,o);
    CHECK(hash(first)==hash(build_building_mesh(b,o)));
    auto reversed=b;std::reverse(reversed.outers.front().begin(),reversed.outers.front().end());
    const auto r=build_building_mesh(reversed,o);CHECK(r.roof==first.roof);CHECK(std::abs(roof_area(r)-roof_area(first))<1e-4);
    o.origin_e+=1024;const auto shifted=build_building_mesh(b,o);REQUIRE(shifted.vertices.size()==first.vertices.size());
    for(std::size_t i=0;i<first.vertices.size();++i){CHECK(std::abs(shifted.vertices[i].position[0]+1024-first.vertices[i].position[0])<1e-4);
        CHECK(shifted.vertices[i].colour==first.vertices[i].colour);CHECK(shifted.vertices[i].normal==first.vertices[i].normal);}
}
TEST_CASE("mapped flat and unsupported roofs do not get inferred pitched replacements", "[world][building_mesh]") {
    auto b=house();b.roof_shape="flat";CHECK(build_building_mesh(b,opts()).roof==RoofForm::Flat);
    b.roof_shape="dome";const auto m=build_building_mesh(b,opts());CHECK(m.roof==RoofForm::Flat);CHECK(m.unsupported_roof);
    b.roof_shape="gabled";b.roof_direction="NE";CHECK(build_building_mesh(b,opts()).unsupported_roof);
    b.roof_direction.clear();b.bottom=105;b.min_height=5;b.roof_height=20;
    const auto raised=build_building_mesh(b,opts());for(const auto& v:raised.vertices)CHECK(v.position[2]>=104.5);
    b.height=std::numeric_limits<double>::quiet_NaN();CHECK(build_building_mesh(b,opts()).vertices.empty());
}

