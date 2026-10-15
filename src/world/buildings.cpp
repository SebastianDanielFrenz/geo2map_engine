// g2m/world/buildings.cpp - see include/g2m/world/buildings.h. Ported from racing_game core/src/buildings.cpp (5a3baed)
// statement for statement (namespace and type names only); tests/world/test_buildings.cpp pins the output byte for byte
// against the game's own code.
#include "g2m/world/buildings.h"

#include "g2m/core/geo/utm.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <map>
#include <set>
#include <string_view>

namespace g2m::world {
namespace {
using Data=osm::OsmData;
template<class T> std::string tag(const Data& d,const T& element,std::string_view key) {
 for(auto t:d.tags_of(element)) if(d.str(t.key)==key)return std::string(d.str(t.value));return {};
}
std::optional<double> number(std::string value,bool metres=true) {
 if(value.empty())return {};std::replace(value.begin(),value.end(),',','.');char* end=nullptr;
 const double n=std::strtod(value.c_str(),&end);if(end==value.c_str()||!std::isfinite(n)||n<0)return {};
 std::string suffix(end);suffix.erase(std::remove_if(suffix.begin(),suffix.end(),[](unsigned char c){return c==' '||c=='\t';}),suffix.end());
 double scale=1;if(metres&&(suffix=="ft"||suffix=="feet"||suffix=="'"))scale=.3048;
 else if(!suffix.empty()&&(!metres||suffix!="m"))return {};
 if(n*scale>2000)return {};return n*scale;
}
template<class T> bool is_building(const Data& d,const T& e) {
 const auto b=tag(d,e,"building"),p=tag(d,e,"building:part");return (!b.empty()&&b!="no")||(!p.empty()&&p!="no");
}
template<class T> Building attributes(const Data& d,const T& e,double fallback,double storey) {
 Building b;b.id=e.id;b.type=tag(d,e,"building");b.colour=tag(d,e,"building:colour");
 const auto h=number(tag(d,e,"height"));const auto levels=number(tag(d,e,"building:levels"),false);
 const auto roof=number(tag(d,e,"roof:height"));const auto roof_levels=number(tag(d,e,"roof:levels"),false);
 const double roof_height=roof.value_or(roof_levels.value_or(0)*storey);
 if(h&&*h>0){b.height=*h;b.height_source="height";}
 else if(levels&&*levels>0){b.height=*levels*storey+roof_height;b.height_source="building:levels";}
 else {b.height=fallback;b.height_source="fallback";}
 b.min_height=number(tag(d,e,"min_height")).value_or(number(tag(d,e,"building:min_level"),false).value_or(0)*storey);
 return b;
}
using Ring=std::vector<std::int64_t>;
std::optional<std::vector<Ring>> join(std::vector<Ring> fragments) {
 std::vector<Ring> closed;
 while(!fragments.empty()) {
  auto ring=std::move(fragments.back());fragments.pop_back();
  bool changed=true;while(changed&&ring.front()!=ring.back()) {
   changed=false;for(auto it=fragments.begin();it!=fragments.end();++it){
    if(it->front()==ring.back()||it->back()==ring.back()) {
     if(it->back()==ring.back())std::reverse(it->begin(),it->end());
     ring.insert(ring.end(),it->begin()+1,it->end());fragments.erase(it);changed=true;break;
    }
   }
  }
  if(ring.size()<4||ring.front()!=ring.back())return {};
  closed.push_back(std::move(ring));
 }
 return closed;
}
}
std::vector<Building> extract_buildings(const OsmTile& tile,const TileKey& key,double min_height,double fallback,double storey,const BuildingGround& ground) {
 const auto& d=tile.data;geo::Utm utm;std::vector<Building> result;std::set<std::int64_t> consumed;
 const auto refs=[&](std::int64_t id)->Ring {
  auto w=std::lower_bound(d.ways.begin(),d.ways.end(),id,[](const auto& a,auto b){return a.id<b;});
  if(w==d.ways.end()||w->id!=id)return {};auto r=d.refs_of(*w);return {r.begin(),r.end()};
 };
 const auto project=[&](const Ring& ids)->std::vector<BuildingPoint>{
  std::vector<BuildingPoint> points;
  for(auto id:ids){auto n=std::lower_bound(d.nodes.begin(),d.nodes.end(),id,[](const auto& a,auto b){return a.id<b;});if(n==d.nodes.end()||n->id!=id)return {};
   auto p=utm.forward(key.zone,n->lat*1e-7,n->lon*1e-7);points.push_back({p.easting,p.northing});}
  if(points.size()>1)points.pop_back();return points;
 };
 const auto add=[&](Building b,const std::vector<Ring>& outers,const std::vector<Ring>& inners) {
  if(b.height<min_height||b.min_height>=b.height||b.height<=0||b.height>2000)return;
  double min_e=1e30,min_n=1e30,max_e=-1e30,max_n=-1e30;
  for(const auto& ring:outers){auto p=project(ring);if(p.size()<3)return;for(auto v:p){min_e=std::min(min_e,v.e);min_n=std::min(min_n,v.n);max_e=std::max(max_e,v.e);max_n=std::max(max_n,v.n);}b.outers.push_back(std::move(p));}
  if(b.outers.empty())return;
  const double e=(min_e+max_e)/2,n=(min_n+max_n)/2;
  // One owner tile per complete footprint, despite overlapping source halos.
  if(static_cast<int>(std::floor(e/1024))!=key.x||static_cast<int>(std::floor(n/1024))!=key.y)return;
  double low=1e30,high=-1e30;
  const auto sample=[&](double x,double y){auto z=ground(x,y);if(z){low=std::min(low,*z);high=std::max(high,*z);}};
  sample(e,n);for(const auto& p:b.outers)for(auto v:p)sample(v.e,v.n);
  if(high< -1e20)return; // No supported DEM: never invent a ground elevation.
  b.base=high;b.bottom=b.min_height>0?high+b.min_height:low-1;
  for(const auto& ring:inners){auto p=project(ring);if(p.size()>=3)b.inners.push_back(std::move(p));}
  result.push_back(std::move(b));
 };
 for(const auto& relation:d.relations)if(is_building(d,relation)) {
  std::vector<Ring> outer,inner;std::vector<std::int64_t> member_ids;bool complete=true;
  for(const auto& m:d.members_of(relation))if(m.type==osm::MemberType::Way) {
   auto role=d.str(m.role);if(role!="outer"&&role!="inner"&& !role.empty())continue;auto ring=refs(m.ref);if(ring.empty()){complete=false;break;}
   (role=="inner"?inner:outer).push_back(std::move(ring));member_ids.push_back(m.ref);
  }
  if(!complete)continue;auto outer_rings=join(std::move(outer)),inner_rings=join(std::move(inner));if(!outer_rings||!inner_rings||outer_rings->empty())continue;
  auto b=attributes(d,relation,fallback,storey);b.relation=true;add(std::move(b),*outer_rings,*inner_rings);
  consumed.insert(member_ids.begin(),member_ids.end());
 }
 for(const auto& w:d.ways)if(!consumed.contains(w.id)&&is_building(d,w)) {
  auto ring=refs(w.id);if(ring.size()<4||ring.front()!=ring.back())continue;add(attributes(d,w,fallback,storey),{ring},{});
 }
 return result;
}

BuildingSet build_building_set(const OsmTile& tile, const TileKey& key, const BuildingParams& params, const BuildingGround& ground) {
    BuildingSet set;
    set.key = key;
    set.buildings = extract_buildings(tile, key, params.min_height_m, params.fallback_height_m, params.storey_height_m, ground);
    return set;
}

} // namespace g2m::world
