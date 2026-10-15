// g2m/world/road_ribbons.cpp - see include/g2m/world/road_ribbons.h. Ported from racing_game core/src/road_visual.cpp
// (0c027dc) statement for statement: namespace and type names changed, libm calls replaced by det math (std::sqrt,
// std::remainder, std::fmod, std::floor, std::ceil, std::round are IEEE-exact), traffic_road_rank moved here as
// road_priority_rank. tests/world/test_road_meshes.cpp pins the output byte for byte against the game's own meshes.
#include "g2m/world/road_ribbons.h"
#include "g2m/world/road_structures.h"
#include "g2m/core/det_math.h"
#include "g2m/layer/road_furniture.h"
#include <algorithm>
#include <charconv>
#include <cmath>
#include <array>
#include <span>
#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <vector>
namespace g2m::world {
namespace {
std::string tag(const g2m::RoadGraphWay& way,std::string_view key) {for(const auto& [k,v]:way.tags)if(k==key)return v;return {};}
int integer(const std::string& s) {int n=0;auto r=std::from_chars(s.data(),s.data()+s.size(),n);return r.ec==std::errc{}&&r.ptr==s.data()+s.size()&&n>=0&&n<=30?n:0;}

}
int road_priority_rank(const g2m::RoadGraphWay& w){
 const auto highway=tag(w,"highway");
 // A link yields where it ends: onto its road (merging, StVO 18(3)) and at the ramp terminal onto any road but a
 // living street (an exit ramp ends at a give-way line or a signal, never with priority).
 if(highway.ends_with("_link"))return 1;
 static const std::map<std::string,int> rank={{"motorway",14},{"trunk",12},{"primary",10},{"secondary",8},{"tertiary",6},{"unclassified",4},{"road",4},{"residential",2},{"living_street",0}};
 const auto it=rank.find(highway);return it==rank.end()?0:it->second;
}
namespace {
constexpr double kJunctionLiftM=0.004; // above the ribbons (lift) and the turn arrows (+3 mm)
// ---- junction surfaces (geo2map roads.geom encoding 7, design D44 kerb returns) ----
struct P2 { double x,y; };
double cross(P2 o,P2 a,P2 b) {return (a.x-o.x)*(b.y-o.y)-(a.y-o.y)*(b.x-o.x);}
// Ear clipping of a simple counter-clockwise ring. Collinear and repeated vertices are tolerated; a ring that cannot be
// clipped completely (numerically degenerate) returns the triangles found so far.
std::vector<std::array<P2,3>> triangulate(std::vector<P2> ring) {
 std::vector<std::array<P2,3>> out;
 const auto inside=[](P2 p,P2 a,P2 b,P2 c){return cross(a,b,p)>=0&&cross(b,c,p)>=0&&cross(c,a,p)>=0;};
 const std::size_t limit=ring.size()+16;std::size_t guard=0,start=0; // every pass removes one vertex or stops
 while(ring.size()>3&&guard++<limit) {
  bool clipped=false;
  for(std::size_t step=0;step<ring.size();++step) {
   const std::size_t i=(start+step)%ring.size();
   const std::size_t ia=(i+ring.size()-1)%ring.size(),ic=(i+1)%ring.size();
   const P2 a=ring[ia],b=ring[i],c=ring[ic];const double turn=cross(a,b,c);
   if(turn<0)continue;
   if(turn==0){ring.erase(ring.begin()+static_cast<std::ptrdiff_t>(i));start=i;clipped=true;break;} // collinear: drop the vertex
   bool ear=true;
   for(std::size_t k=0;k<ring.size()&&ear;++k)if(k!=ia&&k!=i&&k!=ic&&inside(ring[k],a,b,c)&&!(ring[k].x==a.x&&ring[k].y==a.y)&&!(ring[k].x==c.x&&ring[k].y==c.y))ear=false;
   if(!ear)continue;
   // Resume next to the clipped ear: one pass per ear on typical rings instead of a rescan from vertex 0.
   out.push_back({a,b,c});ring.erase(ring.begin()+static_cast<std::ptrdiff_t>(i));start=i;clipped=true;break;
  }
  if(!clipped)break;
 }
 if(ring.size()==3&&cross(ring[0],ring[1],ring[2])>0)out.push_back({ring[0],ring[1],ring[2]});
 return out;
}
// Convex polygon clipped to x >= v (keep = +1) or x <= v (keep = -1) on axis 0 (x) or 1 (y).
std::vector<P2> clip(const std::vector<P2>& poly,int axis,double v,int keep) {
 std::vector<P2> out;const auto d=[&](P2 p){return keep*((axis?p.y:p.x)-v);};
 for(std::size_t i=0;i<poly.size();++i) {
  const P2 a=poly[i],b=poly[(i+1)%poly.size()];const double da=d(a),db=d(b);
  if(da>=0)out.push_back(a);
  if((da<0)!=(db<0)){const double t=da/(da-db);P2 q{a.x+(b.x-a.x)*t,a.y+(b.y-a.y)*t};if(axis)q.y=v;else q.x=v;out.push_back(q);}
 }
 return out;
}
}
namespace {
// Lane layout of one paved stretch as the ribbon draws it (the fragment shader reads lanes, forward and the flags).
struct LaneLayout { std::string highway,direction; bool one=false,fast=false; double width=0; int lanes=1,forward=1,flags=0; };
enum MarkFlag : int { kMarking=1, kEdgeLines=2, kSolidSeparator=4, kFast=8, kPaint=16, kOverlay=32, kLeftEdgeBroken=64, kRightEdgeBroken=128,
                     kLeftEdgeHidden=512, kRightEdgeHidden=1024, kStones=2048, kConcrete=4096, kUnpaved=8192, kSett=16384 };
// Paving surface of a way (OSM surface=*) as the ribbon shader draws it: stones (paving stones, sett, cobblestone),
// concrete, else asphalt. Natural stone (sett, cobblestone) also carries kSett on top of kStones.
int surface_flags(const std::string& surface) {
 if(surface=="sett"||surface=="cobblestone"||surface=="unhewn_cobblestone")return kStones|kSett;
 if(surface.starts_with("paving_stones")||surface=="grass_paver")return kStones;
 if(surface.starts_with("concrete"))return kConcrete;
 return 0;
}
// Whether a side road opens the edge line of the road it joins. Not when it is closed to the public (access no,
// private, permissive, permit, ... - a works crossover between two ramps, an emergency gate). On a motorway or a
// motorway/trunk link a service road opens it only when it is a plain one (no driveway, parking aisle, ...) leaving or
// joining at a shallow angle (a rest-area ramp); one meeting it at a steep angle does not, the edge line runs on solid.
bool opens_edge_line(const g2m::RoadGraphWay& side,const std::string& through_highway,double angle) {
 // permissive (the owner tolerates traffic) counts as closed here: on the A 66 ramps it marks a works crossover.
 const auto restricted=[](const std::string& v){
  return v=="no"||v=="private"||v=="permissive"||v=="permit"||v=="agricultural"||v=="forestry"||v=="emergency"||v=="military"||v=="delivery";};
 for(const char* k:{"access","vehicle","motor_vehicle","motorcar"})if(restricted(tag(side,k)))return false;
 const bool fast=through_highway=="motorway"||through_highway=="motorway_link"||through_highway=="trunk_link";
 if(!fast||tag(side,"highway")!="service")return true;
 return tag(side,"service").empty()&&angle<35*3.14159265358979323846/180;
}
std::optional<LaneLayout> lane_layout(const g2m::RoadGeomEntry& entry,const g2m::RoadGraphWay& w) {
 if(!entry.profile)return {};const auto& p=*entry.profile;if(p.attributes.land_class!=g2m::LandClass::PavedRoad)return {};
 LaneLayout l;l.highway=tag(w,"highway");const auto& highway=l.highway;
 if(highway=="footway"||highway=="path"||highway=="cycleway"||highway=="steps"||highway=="pedestrian")return {};
 l.direction=tag(w,"oneway");const auto& direction=l.direction;
 l.one=direction=="yes"||direction=="1"||direction=="true"||direction=="-1"||(direction.empty()&&(highway=="motorway"||tag(w,"junction")=="roundabout"));
 l.fast=highway=="motorway"||highway=="trunk"||highway=="motorway_link"||highway=="trunk_link";
 const bool edge_lines=l.fast||highway=="primary"||highway=="secondary"||highway=="tertiary";
 l.width=p.attributes.width_mm/1000.0;
 int lanes=p.attributes.lanes;const int f=integer(tag(w,"lanes:forward")),b=integer(tag(w,"lanes:backward"));
 if(lanes==0&&f+b>0)lanes=f+b;
 if(lanes==0)lanes=l.one?std::clamp(static_cast<int>(std::round(l.width/3.25)),1,l.fast?4:2):(l.width>=5.5?2:1);
 l.lanes=std::clamp(lanes,1,30);l.forward=std::clamp(l.one?(direction=="-1"?0:l.lanes):(f>0?f:(b>0?l.lanes-b:(l.lanes+1)/2)),0,l.lanes);
 // The served cross-section (geo2map P1) knows the counted lanes, their directions and widths: it wins over the tags.
 if(entry.cross_section) {
  int counted=0,forward=0;double width=0;
  for(const auto& band:entry.cross_section->bands)if(band.counted){++counted;forward+=band.direction==g2m::LaneDirection::Forward;width+=band.width_mm/1000.0;}
  if(counted>0){l.lanes=std::min(counted,30);l.forward=std::clamp(forward,0,l.lanes);l.width=width;}
 }
 const bool marking=tag(w,"lane_markings")!="no"&&l.width/l.lanes>=2.4&&highway!="living_street";
 const int paving=surface_flags(tag(w,"surface"));
 // A paving-stone or cobbled road carries no painted lines.
 l.flags=(paving&kStones)!=0?paving:((marking?kMarking:0)|(edge_lines?kEdgeLines:0)|(tag(w,"overtaking")=="no"?kSolidSeparator:0)|(l.fast?kFast:0)|paving);
 return l;
}
const g2m::RoadGraphWay* find_way(const g2m::RoadGeomTile& geometry,std::int64_t id) {
 auto it=std::lower_bound(geometry.source.graph.ways.begin(),geometry.source.graph.ways.end(),id,[](const auto& w,auto v){return w.osm_id<v;});
 return it==geometry.source.graph.ways.end()||it->osm_id!=id?nullptr:&*it;
}
struct Pose { double x,y,heading; };
// A point of the reference line, extended straight past either end (furniture can sit a little beyond its stretch).
std::optional<Pose> reference_pose(const g2m::RoadProfile& p,double station) {
 const double length=p.reference.length_m,clamped=std::clamp(station,0.0,length);
 auto xy=p.reference.at(clamped);if(!xy)return {};
 const double extra=station-clamped;return Pose{xy->x+det::cos(xy->heading)*extra,xy->y+det::sin(xy->heading)*extra,xy->heading};
}
// Lateral extents of a stretch at a station (reference frame, t positive left): the asphalt (the road bed: carriageway
// plus shoulders, the same band the junction road-bed ring and the grip raster use) and the counted lanes the markings
// divide. From the served cross-section; without one, the attribute width centred on the reference.
struct Span { double bed_left,bed_right,mark_left,mark_right; double centre() const {return (mark_left+mark_right)/2;} double width() const {return mark_left-mark_right;} };
Span span_at(const g2m::RoadGeomEntry& entry,const LaneLayout& layout,double station) {
 const double half=entry.profile->attributes.width_mm/2000.0;Span fallback{half,-half,half,-half};
 if(!entry.cross_section)return fallback;
 const auto& xs=*entry.cross_section;if(xs.bands.empty()||xs.bands.size()>32)return fallback;
 const double s=std::clamp(station,0.0,entry.profile->reference.length_m);
 const auto fp=g2m::carve_footprint(xs,s);
 std::array<double,33> edges{};if(!xs.boundaries_at(s,std::span<double>(edges.data(),xs.bands.size()+1)))return fallback;
 double left=-1e9,right=1e9;
 for(std::size_t i=0;i<xs.bands.size();++i)if(xs.bands[i].counted){left=std::max(left,edges[i]);right=std::min(right,edges[i+1]);}
 if(left<=right){left=fp.carriageway_left;right=fp.carriageway_right;}
 if(!(fp.roadbed_left>fp.roadbed_right)||!(left>right))return fallback;
 (void)layout;return {std::max(fp.roadbed_left,left),std::min(fp.roadbed_right,right),left,right};
}
// One approach of a served junction, seen from the junction: where its trim line cuts the road and how it is laid out.
struct Approach {
 std::size_t entry=0;bool at_end=false;double cut=0; // station of the trim line on the entry
 Pose pose{};double inward=0,width=0;                // the cut point (lane centre), heading INTO the junction, lanes' width
 Pose lead{};                                        // the lane centre 1 m outward of the cut: a through overlay starts here
 LaneLayout layout;int rank=0;bool roundabout=false;const g2m::RoadGraphWay* way=nullptr;
 int in_lanes=0;double in_from=0,in_to=0;            // lanes driving into the junction and their lateral span (reference frame)
 double centre=0,lane=0;                             // the counted lanes' centre (reference frame) and one lane's width
};
double wrap(double a) {return std::remainder(a,2*3.14159265358979323846);}
// Whether (x, y) lies inside a junction ring (mm vertices) or within `tolerance` metres of its boundary.
bool near_ring(const std::vector<g2m::geom::PointMm>& ring,double x,double y,double tolerance) {
 bool inside=false;double best=1e18;
 for(std::size_t i=0,k=ring.size()-1;i<ring.size();k=i++) {
  const double ax=ring[k].x/1000.0,ay=ring[k].y/1000.0,bx=ring[i].x/1000.0,by=ring[i].y/1000.0;
  if((ay>y)!=(by>y)&&x<ax+(bx-ax)*(y-ay)/(by-ay))inside=!inside;
  const double dx=bx-ax,dy=by-ay,l2=dx*dx+dy*dy,t=l2>0?std::clamp(((x-ax)*dx+(y-ay)*dy)/l2,0.0,1.0):0;
  best=std::min(best,det::hypot(ax+t*dx-x,ay+t*dy-y));
 }
 return inside||best<=tolerance;
}
}
// Every junction this tile owns, as a patch over the road bed ring: ear clipping, then each triangle cut by the terrain
// lattice lines (DGM1 cell centres, x.5 m) so the patch bends exactly where the terrain does and neighbouring pieces
// share their vertices (no T-junctions). Heights come from `ground`; patches sit `lift` above it, a little above the
// ribbons, so the square ribbon ends under the junction disappear. No markings (fade 0).
// Surface: where every road meeting there has the same paving, the patch has it too. Where they differ, the major road
// (the through pair, else the highest-ranked arm) keeps its paving across the junction up to its own road bed edge
// (the cubic between its two trim cuts, or its centre line run on into the ring) and the rest of the ring takes the
// paving of the nearest other arm - a paving-stone side road reaches up to the asphalt edge of the road it joins.
void add_junction_surfaces(RoadRibbonTile& mesh,const g2m::RoadGeomTile& geometry,const g2m::TileKey& key,const RoadRibbonGround& ground,double lift) {
 const double ox=key.min_easting(),oy=key.min_northing();
 constexpr double kPi=3.14159265358979323846;
 struct Arm { P2 cut,in;double half=0;int surface=0,rank=0; };
 for(const auto& j:geometry.junctions) {
  if(j.decline!=g2m::JunctionDecline::None||(j.flags&static_cast<std::uint16_t>(g2m::JunctionFlags::OnStructure)))continue;
  // The road bed ring: carriageway plus shoulders, the band the ribbons draw as asphalt (the paved ring adds sidewalks).
  const auto& v=j.rings[static_cast<std::size_t>(g2m::JunctionRingLayer::RoadBed)].vertices;if(v.size()<3)continue;
  std::vector<P2> ring;ring.reserve(v.size());
  for(const auto& q:v)ring.push_back({static_cast<double>(q.x)/1000.0-ox,static_cast<double>(q.y)/1000.0-oy});
  // The arms: their road bed centre at the trim cut, heading into the junction, half bed width and paving.
  std::vector<Arm> arms;
  for(const auto& ap:j.approaches) {
   if(ap.status!=g2m::JunctionApproachStatus::Used||ap.entry_index>=geometry.entries.size())continue;
   const auto& entry=geometry.entries[ap.entry_index];const auto* w=find_way(geometry,entry.way_id);if(!w||!entry.profile)continue;
   // An unpaved road (track, gravel lane) has no ribbon - the terrain draws it - but its mouth in the ring takes its
   // surface, not asphalt. Foot and cycle paths stay out.
   auto layout=lane_layout(entry,*w);int surface=layout?surface_flags(tag(*w,"surface")):0;
   if(!layout) {
    const auto highway=tag(*w,"highway");
    if(entry.profile->attributes.land_class!=g2m::LandClass::UnpavedRoad||highway=="footway"||highway=="path"||highway=="cycleway"||
       highway=="steps"||highway=="pedestrian"||highway=="bridleway")continue;
    layout=LaneLayout{};surface=kUnpaved; // the terrain draws the track itself as dirt, whatever its surface tag
   }
   const double length=entry.profile->reference.length_m,cut=std::clamp(ap.at_end?length-ap.trim_m:ap.trim_m,0.0,length);
   const auto pose=reference_pose(*entry.profile,cut);if(!pose)continue;
   const auto span=span_at(entry,*layout,cut);const double centre=(span.bed_left+span.bed_right)/2;
   const double heading=ap.at_end?pose->heading:pose->heading+kPi;
   arms.push_back({{pose->x-det::sin(pose->heading)*centre-ox,pose->y+det::cos(pose->heading)*centre-oy},{det::cos(heading),det::sin(heading)},
                   (span.bed_left-span.bed_right)/2,surface,road_priority_rank(*w)});
  }
  const bool mixed=std::any_of(arms.begin(),arms.end(),[&](const Arm& a){return a.surface!=arms.front().surface;});
  // The major road's centre line through the ring (a polyline with its half width per point) and its arms; with one
  // paving everywhere the first arm stands for the whole patch. While a paved road meets there, an unpaved one never
  // carries the major road (the track joins the road, not the other way round). A through pair may change paving
  // (concrete running on as asphalt): each keeps its own up to the line across the middle between their cuts.
  std::vector<std::pair<P2,double>> spine;int major_arm=arms.empty()?-1:0;std::vector<std::size_t> major;
  const bool any_paved=std::any_of(arms.begin(),arms.end(),[](const Arm& a){return a.surface!=kUnpaved;});
  const auto weight=[&](const Arm& a){return any_paved&&a.surface==kUnpaved?-1:a.rank;};
  if(mixed) {
   int best_i=-1,best_j=-1,best_rank=-1;bool best_same=false;double best_dev=1e9;
   for(std::size_t i=0;i<arms.size();++i)for(std::size_t k=i+1;k<arms.size();++k) {
    const double dev=det::acos(std::clamp(-(arms[i].in.x*arms[k].in.x+arms[i].in.y*arms[k].in.y),-1.0,1.0));
    const int rank=std::min(weight(arms[i]),weight(arms[k]));if(dev>35*kPi/180||rank<0)continue;
    const bool same=arms[i].surface==arms[k].surface;
    if(rank>best_rank||(rank==best_rank&&(same>best_same||(same==best_same&&dev<best_dev))))
     {best_rank=rank;best_same=same;best_dev=dev;best_i=static_cast<int>(i);best_j=static_cast<int>(k);}
   }
   int top=0;for(std::size_t i=1;i<arms.size();++i)if(weight(arms[i])>weight(arms[static_cast<std::size_t>(top)]))top=static_cast<int>(i);
   if(best_i>=0&&best_rank>=weight(arms[static_cast<std::size_t>(top)])) {
    const Arm& a=arms[static_cast<std::size_t>(best_i)];const Arm& b=arms[static_cast<std::size_t>(best_j)];
    const double h=det::hypot(b.cut.x-a.cut.x,b.cut.y-a.cut.y)/3;
    const P2 p0=a.cut,p1{a.cut.x+a.in.x*h,a.cut.y+a.in.y*h},p3=b.cut,p2{b.cut.x+b.in.x*h,b.cut.y+b.in.y*h};
    for(int i=0;i<=32;++i){const double t=i/32.0,u=1-t;
     spine.push_back({{u*u*u*p0.x+3*u*u*t*p1.x+3*u*t*t*p2.x+t*t*t*p3.x,u*u*u*p0.y+3*u*u*t*p1.y+3*u*t*t*p2.y+t*t*t*p3.y},a.half+(b.half-a.half)*t});}
    major_arm=best_i;major={static_cast<std::size_t>(best_i),static_cast<std::size_t>(best_j)};
   } else {
    // No through pair: the top-ranked arm's centre line runs straight on across the ring.
    const Arm& a=arms[static_cast<std::size_t>(top)];double reach=0;
    for(const auto& p:ring)reach=std::max(reach,det::hypot(p.x-a.cut.x,p.y-a.cut.y));
    spine={{a.cut,a.half},{{a.cut.x+a.in.x*reach,a.cut.y+a.in.y*reach},a.half}};
    major_arm=top;major={static_cast<std::size_t>(top)};
   }
  }
  // Signed distance to the major road bed edge (negative inside), and the paving of the nearest other arm outside it.
  const auto inside_major=[&](P2 p){
   double best=1e18;
   for(std::size_t i=0;i+1<spine.size();++i) {
    const P2 a=spine[i].first,b=spine[i+1].first;const double dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;
    const double t=l2>0?std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l2,0.0,1.0):0;
    best=std::min(best,det::hypot(a.x+t*dx-p.x,a.y+t*dy-p.y)-(spine[i].second+(spine[i+1].second-spine[i].second)*t));
   }
   return best;
  };
  // Which side of the major road's centre line p lies on (+1 left, -1 right, along the spine).
  const auto side_of=[&](P2 p){
   double best=1e18;int side=1;
   for(std::size_t i=0;i+1<spine.size();++i) {
    const P2 a=spine[i].first,b=spine[i+1].first;const double dx=b.x-a.x,dy=b.y-a.y,l2=dx*dx+dy*dy;
    const double t=l2>0?std::clamp(((p.x-a.x)*dx+(p.y-a.y)*dy)/l2,0.0,1.0):0,d=det::hypot(a.x+t*dx-p.x,a.y+t*dy-p.y);
    if(d<best){best=d;side=dx*(p.y-a.y)-dy*(p.x-a.x)>=0?1:-1;}
   }
   return side;
  };
  // Distance from p to an arm's centre line run on into the ring as far as the junction centre (further, the arms'
  // lines would cross into each other's mouths), less its half bed width.
  P2 centre{0,0};for(const auto& q:ring){centre.x+=q.x/ring.size();centre.y+=q.y/ring.size();}
  const auto arm_distance=[&](std::size_t i,P2 p){
   const Arm& a=arms[i];const double reach=std::max(0.0,(centre.x-a.cut.x)*a.in.x+(centre.y-a.cut.y)*a.in.y);
   const double t=std::clamp((p.x-a.cut.x)*a.in.x+(p.y-a.cut.y)*a.in.y,0.0,reach);
   return det::hypot(a.cut.x+a.in.x*t-p.x,a.cut.y+a.in.y*t-p.y)-a.half;
  };
  // The two other arms nearest to p outside the major road bed (-1 when there is none), from those joining the major
  // road on p's side when there are any: a corner across the major road never takes the far side road's paving.
  std::vector<int> arm_side(arms.size(),0);
  if(!spine.empty())for(std::size_t i=0;i<arms.size();++i)if(std::find(major.begin(),major.end(),i)==major.end())arm_side[i]=side_of(arms[i].cut);
  const auto minor_arms=[&](P2 p){
   std::pair<int,int> best{-1,-1};double d0=1e18,d1=1e18;
   const int side=side_of(p);
   const bool own_side=std::any_of(arm_side.begin(),arm_side.end(),[&](int s){return s==side;});
   for(std::size_t i=0;i<arms.size();++i) {
    if(std::find(major.begin(),major.end(),i)!=major.end()||(own_side&&arm_side[i]!=side))continue;
    const double d=arm_distance(i,p);
    if(d<d0){d1=d0;best.second=best.first;d0=d;best.first=static_cast<int>(i);}else if(d<d1){d1=d;best.second=static_cast<int>(i);}
   }
   return best;
  };
  const auto first=mesh.vertices.size();const auto first_index=mesh.indices.size();bool valid=true;
  std::map<std::tuple<std::int64_t,std::int64_t,int>,std::int32_t> shared; // vertex dedup on a 0.1 mm key per arm
  // A piece belongs to an arm (-1: the whole patch of a single paving): its paving, and u, v along that arm's heading in
  // metres, so a paving pattern runs the way the arm's own ribbon lays it.
  const auto vertex=[&](P2 p,int arm)->std::optional<std::int32_t>{
   const std::tuple<std::int64_t,std::int64_t,int> k{std::llround(p.x*1e4),std::llround(p.y*1e4),arm};
   if(auto it=shared.find(k);it!=shared.end())return it->second;
   const auto h=ground(p.x+ox,p.y+oy);if(!h)return {};
   const auto index=static_cast<std::int32_t>(mesh.vertices.size());shared.emplace(k,index);
   const int surface=arm>=0?arms[static_cast<std::size_t>(arm)].surface:major_arm>=0?arms[static_cast<std::size_t>(major_arm)].surface:0;
   const P2 dir=arm>=0?arms[static_cast<std::size_t>(arm)].in:major_arm>=0?arms[static_cast<std::size_t>(major_arm)].in:P2{1,0};
   const double u=dir.x*p.y-dir.y*p.x,along=dir.x*p.x+dir.y*p.y;
   mesh.vertices.push_back({static_cast<float>(p.x),static_cast<float>(p.y),static_cast<float>(*h+lift+kJunctionLiftM),static_cast<float>(u),static_cast<float>(along),1000.0f,0,0,0,static_cast<float>(surface)});
   return index;
  };
  // A convex polygon cut where a function, given at its vertices and close to linear across it, changes sign:
  // the part where it is <= 0, then the rest.
  const auto split=[](const std::vector<P2>& poly,const std::vector<double>& f){
   std::pair<std::vector<P2>,std::vector<P2>> r;
   for(std::size_t i=0;i<poly.size();++i) {
    const std::size_t k=(i+1)%poly.size();
    (f[i]<=0?r.first:r.second).push_back(poly[i]);
    if((f[i]<=0)!=(f[k]<=0)){const double s=f[i]/(f[i]-f[k]);const P2 q{poly[i].x+(poly[k].x-poly[i].x)*s,poly[i].y+(poly[k].y-poly[i].y)*s};r.first.push_back(q);r.second.push_back(q);}
   }
   return r;
  };
  const auto emit=[&](const std::vector<P2>& poly,int arm) {
   if(poly.size()<3||!valid)return;
   std::vector<std::int32_t> ids;
   for(const auto& p:poly){auto id=vertex(p,arm);if(!id){valid=false;return;}if(ids.empty()||ids.back()!=*id)ids.push_back(*id);}
   if(ids.size()>1&&ids.front()==ids.back())ids.pop_back();
   for(std::size_t i=1;i+1<ids.size();++i)mesh.indices.insert(mesh.indices.end(),{ids[0],ids[i],ids[i+1]});
  };
  for(const auto& t:triangulate(ring)) {
   const double x0=std::min({t[0].x,t[1].x,t[2].x}),x1=std::max({t[0].x,t[1].x,t[2].x});
   const double y0=std::min({t[0].y,t[1].y,t[2].y}),y1=std::max({t[0].y,t[1].y,t[2].y});
   for(double gx=std::floor(x0-.5)+.5;gx<x1&&valid;gx+=1)
    for(double gy=std::floor(y0-.5)+.5;gy<y1&&valid;gy+=1) {
     std::vector<P2> cell(t.begin(),t.end());
     cell=clip(cell,0,gx,1);cell=clip(cell,0,gx+1,-1);cell=clip(cell,1,gy,1);cell=clip(cell,1,gy+1,-1);
     if(cell.size()<3)continue;
     if(!mixed){emit(cell,-1);continue;}
     // Split the (convex, at most 1 m) cell along the road bed edge; the distance is close to linear across it.
     std::vector<double> d;for(const auto& p:cell)d.push_back(inside_major(p));
     const auto [in,out]=split(cell,d);
     if(major.size()==2&&arms[major[0]].surface!=arms[major[1]].surface) {
      const P2 a=arms[major[0]].cut,b=arms[major[1]].cut,m{(a.x+b.x)/2,(a.y+b.y)/2};
      std::vector<double> e;for(const auto& p:in)e.push_back((p.x-m.x)*(b.x-a.x)+(p.y-m.y)*(b.y-a.y));
      const auto [near_a,near_b]=split(in,e);emit(near_a,static_cast<int>(major[0]));emit(near_b,static_cast<int>(major[1]));
     } else emit(in,major_arm);
     if(out.size()<3)continue;
     // Outside it, the cell is shared between the two nearest other arms along the line of equal distance to both.
     P2 c{0,0};for(const auto& p:out){c.x+=p.x/out.size();c.y+=p.y/out.size();}
     const auto [ai,aj]=minor_arms(c);
     if(aj<0||arms[static_cast<std::size_t>(ai)].surface==arms[static_cast<std::size_t>(aj)].surface){emit(out,ai);continue;}
     std::vector<double> h;for(const auto& p:out)h.push_back(arm_distance(static_cast<std::size_t>(ai),p)-arm_distance(static_cast<std::size_t>(aj),p));
     const auto [near_i,near_j]=split(out,h);emit(near_i,ai);emit(near_j,aj);
    }
   if(!valid)break;
  }
  if(!valid){mesh.vertices.resize(first);mesh.indices.resize(first_index);}
 }
}
RoadRibbonTile build_road_ribbons(const g2m::RoadGeomTile& geometry,const g2m::TileKey& key,const RoadRibbonGround& ground,double lift,const RoadSeparation& separation) {
 RoadRibbonTile mesh;mesh.key=key;const auto decks=build_road_decks(geometry,separation);
 // Deck height at a station of a separated stretch. The station is clamped into the deck so a ribbon row that lands
 // a rounding step past the deck end still finds it; a bridge without a deck falls back to its own profile.
 const auto deck_height=[&](const g2m::RoadGeomEntry& entry,double station,double lateral)->std::optional<double>{
  for(const auto& d:decks)if(d->way_id==entry.way_id&&d->start_ref==entry.stretch.start_ref&&station>=d->start_station-.05&&station<=d->end_station+.05)
   if(auto z=road_deck_height(*d,std::clamp(station,d->start_station,d->end_station),lateral))return z;
  const auto& p=*entry.profile;if(auto z=p.at(std::clamp(station,0.0,p.reference.length_m)))return z->height_m;
  return {};
 };
 const auto separated_at=[&](const g2m::RoadGeomEntry& entry,double station){
  return entry.profile->attributes.bridge||std::any_of(decks.begin(),decks.end(),[&](const auto& d){return d->way_id==entry.way_id&&d->start_ref==entry.stretch.start_ref&&station>=d->start_station&&station<=d->end_station;});
 };
 // Surface height of a stretch at (station, lateral): the deck on a structure, else the ground.
 const auto surface=[&](const g2m::RoadGeomEntry& entry,double station,double lateral,double e,double n)->std::optional<double>{
  return separated_at(entry,station)?deck_height(entry,station,lateral):ground(e,n);
 };
 const double ox=key.min_easting(),oy=key.min_northing();
 const auto owned=[&](double x,double y){return std::floor(x/1024)==key.x&&std::floor(y/1024)==key.y;};
 // Painted rectangle on a stretch (station s0..s1, lateral t0..t1, reference frame), split every metre across so it
 // follows the ground; `raise` above the ribbon surface.
 const auto paint_rect=[&](const g2m::RoadGeomEntry& entry,double s0,double s1,double t0,double t1,double raise) {
  const auto& p=*entry.profile;const int across=std::max(1,static_cast<int>(std::ceil(std::abs(t1-t0))));
  const auto first=static_cast<std::int32_t>(mesh.vertices.size());
  for(double station:{s0,s1})for(int j=0;j<=across;++j) {
   const double t=t0+(t1-t0)*j/across;auto pose=reference_pose(p,station);if(!pose){mesh.vertices.resize(first);return;}
   const double e=pose->x-det::sin(pose->heading)*t,n=pose->y+det::cos(pose->heading)*t;
   auto h=surface(entry,station,t,e,n);if(!h){mesh.vertices.resize(first);return;}
   mesh.vertices.push_back({static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*h+lift+raise),0,0,1000.0f,1,0,0,kPaint});
  }
  for(int j=0;j<across;++j){const auto a=first+j,b=a+across+1;mesh.indices.insert(mesh.indices.end(),{a,b,a+1,a+1,b,b+1});}
 };
 // ---- junction analysis: through roads, side roads, lines (owner tile only: geometry.junctions) ----
 struct LineKey { std::size_t entry;bool forward; auto operator<=>(const LineKey&) const = default; };
 std::set<LineKey> lined;                               // approaches that already carry a stop or give-way line
 std::set<std::size_t> signal_entries;                  // entries with a placed signal (a signalled junction)
 for(const auto& fu:geometry.furniture)if(fu.decline==g2m::FurnitureDecline::None){
  if(fu.kind==g2m::FurnitureKind::StopLine||fu.kind==g2m::FurnitureKind::GiveWayLine)lined.insert({fu.entry_index,fu.travel==g2m::LaneDirection::Forward});
  if(fu.kind==g2m::FurnitureKind::Signal)signal_entries.insert(fu.entry_index);
 }
 std::set<std::tuple<std::int64_t,std::uint32_t,bool>> continued; // approach ends whose markings run on over the junction
 struct Overlay { Approach a,b;int broken=0;const std::vector<g2m::geom::PointMm>* ring=nullptr;std::vector<Approach> gores{}; };
 std::vector<Overlay> overlays;
 struct GiveWay { std::size_t entry;double station,t0,t1; };
 std::vector<GiveWay> give_ways;
 for(const auto& j:geometry.junctions) {
  if(j.decline!=g2m::JunctionDecline::None||(j.flags&static_cast<std::uint16_t>(g2m::JunctionFlags::OnStructure)))continue;
  std::vector<Approach> list;bool signalled=false;
  for(const auto& ap:j.approaches) {
   if(ap.status!=g2m::JunctionApproachStatus::Used||ap.entry_index>=geometry.entries.size())continue;
   const auto& entry=geometry.entries[ap.entry_index];const auto* w=find_way(geometry,entry.way_id);if(!w||!entry.profile)continue;
   auto layout=lane_layout(entry,*w);if(!layout)continue;
   signalled=signalled||signal_entries.contains(ap.entry_index);
   Approach a;a.entry=ap.entry_index;a.at_end=ap.at_end;a.layout=*layout;a.rank=road_priority_rank(*w);a.roundabout=tag(*w,"junction")=="roundabout";a.way=w;
   const double length=entry.profile->reference.length_m;a.cut=std::clamp(ap.at_end?length-ap.trim_m:ap.trim_m,0.0,length);
   auto pose=reference_pose(*entry.profile,a.cut);if(!pose)continue;
   // The cut point on the centre of the counted lanes (the marking frame), and the lanes toward the junction: the
   // way-direction lanes (right of the reference) at the stretch end, else the others.
   const auto span=span_at(entry,*layout,a.cut);const double lane=span.width()/layout->lanes;a.centre=span.centre();a.lane=lane;
   a.pose=*pose;a.pose.x-=det::sin(pose->heading)*span.centre();a.pose.y+=det::cos(pose->heading)*span.centre();
   a.width=span.width();a.inward=ap.at_end?pose->heading:pose->heading+3.14159265358979323846;
   {const double st=std::clamp(ap.at_end?a.cut-1:a.cut+1,0.0,length);auto lp=reference_pose(*entry.profile,st);if(!lp)continue;
    const auto ls=span_at(entry,*layout,st);a.lead={lp->x-det::sin(lp->heading)*ls.centre(),lp->y+det::cos(lp->heading)*ls.centre(),lp->heading};}
   a.in_lanes=ap.at_end?layout->forward:layout->lanes-layout->forward;
   if(ap.at_end){a.in_from=span.mark_right;a.in_to=span.mark_right+a.in_lanes*lane;}else{a.in_from=span.mark_left-a.in_lanes*lane;a.in_to=span.mark_left;}
   list.push_back(a);
  }
  if(list.size()<2)continue;
  const bool roundabout=std::any_of(list.begin(),list.end(),[](const auto& a){return a.roundabout;});
  // The through road: the pair of approaches that continue each other within 35 degrees, highest rank first.
  constexpr double kThrough=35*3.14159265358979323846/180;
  int best_i=-1,best_j=-1,best_rank=-1;double best_dev=1e9;
  for(std::size_t i=0;i<list.size();++i)for(std::size_t k=i+1;k<list.size();++k) {
   const double dev=std::abs(wrap(list[k].inward+3.14159265358979323846-list[i].inward));if(dev>kThrough)continue;
   const int rank=std::min(list[i].rank,list[k].rank);
   if(rank>best_rank||(rank==best_rank&&dev<best_dev)){best_rank=rank;best_dev=dev;best_i=static_cast<int>(i);best_j=static_cast<int>(k);}
  }
  bool through=best_i>=0&&!roundabout;
  if(through)for(std::size_t i=0;i<list.size();++i)for(std::size_t k=i+1;k<list.size();++k) {
   // A crossing road of the same rank (another straight pair sharing no approach): markings stop at the box.
   if(static_cast<int>(i)==best_i||static_cast<int>(i)==best_j||static_cast<int>(k)==best_i||static_cast<int>(k)==best_j)continue;
   if(std::abs(wrap(list[k].inward+3.14159265358979323846-list[i].inward))<=kThrough&&std::min(list[i].rank,list[k].rank)>=best_rank)through=false;
  }
  // A signalled junction keeps the through road's lane and edge lines too (the box stays clear only at a crossing of
  // equal rank, above); its stop lines stand on the approaches.
  // A dual carriageway meeting a two-way road (the B 8 splitting into two one-way ways at a junction): the straight
  // pair carries one direction only, so the two-way member's other lanes continue into the opposite one-way way. That
  // way is the second overlay; without it half the road's lanes would stop at the box.
  int extra_i=-1,extra_of=-1;
  if(through)for(int m:{best_i,best_j}) {
   const auto& two=list[static_cast<std::size_t>(m)];const auto& one=list[static_cast<std::size_t>(m==best_i?best_j:best_i)];
   if(two.in_lanes<=0||two.in_lanes>=two.layout.lanes||(one.in_lanes>0&&one.in_lanes<one.layout.lanes))continue;
   double dev_best=1e9;
   for(std::size_t i=0;i<list.size();++i) {
    if(static_cast<int>(i)==best_i||static_cast<int>(i)==best_j)continue;
    const auto& c=list[i];if(c.rank<best_rank||(c.in_lanes>0&&c.in_lanes<c.layout.lanes))continue;
    if((c.in_lanes>0)==(one.in_lanes>0))continue;  // the opposite direction to the paired one-way way
    const double dev=std::abs(wrap(c.inward+3.14159265358979323846-two.inward));
    if(dev<=kThrough&&dev<dev_best){dev_best=dev;extra_i=static_cast<int>(i);extra_of=m;}
   }
   if(extra_i>=0)break;
  }
  const auto add_overlay=[&](int pi,int pj) {
   // Orient the overlay with the traffic: from the approach that can be driven into the junction.
   Approach a=list[static_cast<std::size_t>(pi)],b=list[static_cast<std::size_t>(pj)];
   if(a.in_lanes==0&&b.in_lanes>0)std::swap(a,b);
   Overlay o{a,b,0,&j.rings[static_cast<std::size_t>(g2m::JunctionRingLayer::RoadBed)].vertices};
   const double dx=b.pose.x-a.pose.x,dy=b.pose.y-a.pose.y;
   for(std::size_t i=0;i<list.size();++i){const int ii=static_cast<int>(i);if(ii==best_i||ii==best_j||ii==extra_i)continue;
    // The side road's angle to the through road's axis (0 = it leaves or joins parallel to it; a and b point in
    // opposite directions, so the smaller of the two deviations is the angle to the axis).
    const double angle=std::min(std::abs(wrap(list[i].inward-a.inward)),std::abs(wrap(list[i].inward-b.inward)));
    if(!opens_edge_line(*list[i].way,a.layout.highway,angle))continue;
    o.broken|=(dx*(list[i].pose.y-a.pose.y)-dy*(list[i].pose.x-a.pose.x))>0?kLeftEdgeBroken:kRightEdgeBroken;
    // An exit or an on-ramp leaving or joining at a shallow angle (one-way): a hatched gore between it and the
    // through road (Sperrflaeche, sign 298), drawn with the overlay below.
    const auto& sa=list[i];
    // Only on classified roads, for ramps and classified roads branching off: a one-way street in town meeting at a
    // shallow angle is not an exit.
    const auto classified=[](const std::string& h){
     for(const char* c:{"motorway","trunk","primary","secondary"})if(h==c||h==std::string(c)+"_link")return true;
     return false;};
    if(angle<45*3.14159265358979323846/180&&(sa.in_lanes==0||sa.in_lanes==sa.layout.lanes)&&classified(a.layout.highway)&&classified(sa.layout.highway))
     o.gores.push_back(sa);}
   overlays.push_back(o);
   for(const auto* x:{&a,&b}){const auto& e=geometry.entries[x->entry];continued.insert({e.way_id,e.stretch.start_ref,x->at_end});}
  };
  // A signalled junction keeps the through road's lane and edge lines too (the box stays clear only at a crossing of
  // equal rank, above); its stop lines stand on the approaches.
  if(through)add_overlay(best_i,best_j);
  if(through&&extra_i>=0)add_overlay(extra_of,extra_i);
  // Turn guide lines (Leitlinien over the box, RMS): two or more lanes tagged only "left" (turn:lanes, leftmost first)
  // keep their lane and edge lines, broken, along a curve into the leftmost lanes of the road they turn into - the arm
  // 15-150 degrees to the left nearest a right angle with that many lanes leaving the junction. Without them the
  // lanes of a double left turn run into an unmarked box and the paths cross.
  if(!roundabout)for(const auto& a:list) {
   if(a.in_lanes<2||a.lane<=0)continue;
   const auto turns=a.layout.one?tag(*a.way,"turn:lanes"):tag(*a.way,a.at_end?"turn:lanes:forward":"turn:lanes:backward");
   int g=0;std::size_t begin=0,tokens=0;bool counting=true;
   while(true){const auto end=turns.find('|',begin);const auto t=turns.substr(begin,end==std::string::npos?std::string::npos:end-begin);++tokens;
    if(counting&&t.find("left")!=std::string::npos&&t.find("through")==std::string::npos&&t.find("right")==std::string::npos)++g;else counting=false;
    if(end==std::string::npos)break;begin=end+1;}
   if(turns.empty()||static_cast<int>(tokens)!=a.in_lanes||g<2)continue;
   const Approach* target=nullptr;double target_dev=1e9;
   const int ia=static_cast<int>(&a-list.data());
   for(const auto& b:list) {
    const int ib=static_cast<int>(&b-list.data());
    if(&b==&a||b.layout.lanes-b.in_lanes<g||b.lane<=0)continue;
    const auto pair=[&](int x,int y){return x>=0&&y>=0&&((ia==x&&ib==y)||(ia==y&&ib==x));};
    if(through&&(pair(best_i,best_j)||pair(extra_of,extra_i)))continue; // already continued by the through overlay
    const double turn=wrap(b.inward+3.14159265358979323846-a.inward);           // left (counter-clockwise) positive
    if(turn<15*3.14159265358979323846/180||turn>150*3.14159265358979323846/180)continue;
    const double dev=std::abs(turn-3.14159265358979323846/2);if(dev<target_dev){target_dev=dev;target=&b;}
   }
   if(!target)continue;
   const auto shifted=[](Approach x,double t,int lanes,int in_lanes) {
    const double d=t-x.centre;
    x.pose.x-=det::sin(x.pose.heading)*d;x.pose.y+=det::cos(x.pose.heading)*d;x.lead.x-=det::sin(x.lead.heading)*d;x.lead.y+=det::cos(x.lead.heading)*d;
    x.width=lanes*x.lane;x.layout.lanes=lanes;x.in_lanes=in_lanes;x.layout.flags&=~kFast;return x; // short dashes: a guide, not a motorway line
   };
   // The leftmost g lanes in the reference frame (left +): toward the junction at the stretch end the travel left is
   // +t, away from it -t; the lanes leaving a junction are the ones beside the lanes entering it.
   const double ta=a.at_end?a.in_to-g*a.lane/2:a.in_from+g*a.lane/2;
   const auto& b=*target;const double tb=b.at_end?b.in_to+g*b.lane/2:b.in_from-g*b.lane/2;
   overlays.push_back({shifted(a,ta,g,g),shifted(b,tb,g,0),kLeftEdgeBroken|kRightEdgeBroken,&j.rings[static_cast<std::size_t>(g2m::JunctionRingLayer::RoadBed)].vertices});
  }
  if(signalled)continue;
  // Give-way lines nobody tagged (StVO 8 with sign 205/215): at a roundabout entry, and on a lower-ranked side road
  // of a through road unless it merges at a shallow angle (a slip lane or an on-ramp: no line, it merges).
  for(std::size_t i=0;i<list.size();++i) {
   const auto& a=list[i];if(a.in_lanes==0||a.roundabout||a.rank==0)continue;
   const auto& entry=geometry.entries[a.entry];if(lined.contains({a.entry,a.at_end}))continue;
   bool yields=roundabout;
   if(!yields&&best_i>=0&&static_cast<int>(i)!=best_i&&static_cast<int>(i)!=best_j&&static_cast<int>(i)!=extra_i&&a.rank<best_rank) {
    yields=true;
    for(int t:{best_i,best_j})if(std::abs(wrap(list[static_cast<std::size_t>(t)].inward+3.14159265358979323846-a.inward))<25*3.14159265358979323846/180)yields=false;
   }
   if(!yields)continue;
   const double set_back=.25,station=a.at_end?a.cut-set_back:a.cut+set_back;
   give_ways.push_back({a.entry,station,a.in_from,a.in_to});(void)entry;
  }
 }
 // Trims of the junctions this tile owns, per (way, stretch start_ref, at_end): lane markings end at the trim line.
 std::map<std::tuple<std::int64_t,std::uint32_t,bool>,double> trims;
 for(const auto& j:geometry.junctions)if(j.decline==g2m::JunctionDecline::None)
  for(const auto& a:j.approaches)if(a.status==g2m::JunctionApproachStatus::Used&&a.entry_index<geometry.entries.size()){
   const auto& e=geometry.entries[a.entry_index];trims[{e.way_id,e.stretch.start_ref,a.at_end}]=a.trim_m;}
 std::map<std::int64_t,int> degree;for(const auto& edge:geometry.source.graph.edges){++degree[edge.from_node];++degree[edge.to_node];}
 for(const auto& entry:geometry.entries) {
  if(!entry.profile)continue;const auto& p=*entry.profile;
  const auto* way_ptr=find_way(geometry,entry.way_id);if(!way_ptr)continue;const auto& w=*way_ptr;
  const auto layout=lane_layout(entry,w);if(!layout)continue;
  const auto& direction=layout->direction;const bool one=layout->one;
  const int lanes=layout->lanes,forward=layout->forward,flags=layout->flags;
  const double length=p.reference.length_m;if(length<=.01)continue;
  const bool start_junction=p.stretch.start_ref<w.node_ids.size()&&degree[w.node_ids[p.stretch.start_ref]]>2;
  const bool end_junction=p.stretch.end_ref<w.node_ids.size()&&degree[w.node_ids[p.stretch.end_ref]]>2;
  // A junction served for this stretch end (owner tile only) gives the exact trim; elsewhere the 3-11 m ramp stays. A
  // through road's markings do not fade: the junction overlay continues them.
  const auto trim_of=[&](bool at_end)->std::optional<double>{auto t=trims.find({entry.way_id,entry.stretch.start_ref,at_end});if(t==trims.end())return {};return t->second;};
  const bool start_on=continued.contains({entry.way_id,entry.stretch.start_ref,false}),end_on=continued.contains({entry.way_id,entry.stretch.start_ref,true});
  const auto start_trim=trim_of(false),end_trim=trim_of(true);
  // The ribbon ends at a served junction's trim line (a little under the junction patch): inside it the patch is the
  // road, and a ribbon wider than the ring there (a stretch absorbed into a cluster, a road narrowing at the node) would
  // stick out beside it as a step. A stretch whose two trims overlap lies wholly inside its junction.
  constexpr double kUnderPatchM=.1;
  const double from=start_trim?std::clamp(*start_trim-kUnderPatchM,0.0,length):0.0;
  const double to=end_trim?std::clamp(length-*end_trim+kUnderPatchM,0.0,length):length;
  const int count=to-from>.05?std::max(1,static_cast<int>(std::ceil(to-from))):0; // metre spacing matches the terrain detail.
  const double shoulder=.30;
  struct Row { double station,x,y,nx,ny,fade;Span span;bool separated; };
  for(int i=0;i<count;++i) {
   // The last row is exactly `to`: length*count/count can land one ulp past the profile end, which the profile
   // rejects - the whole last metre of the stretch was dropped and left a gap at every such stretch end.
   const double stations[2]={from+(to-from)*i/count,i+1==count?to:from+(to-from)*(i+1)/count};auto mid=p.reference.at((stations[0]+stations[1])*.5);if(!mid)continue;
   if(!owned(mid->x,mid->y))continue;
   std::array<Row,2> rows{};bool valid=true;
   for(int r=0;r<2;++r) {
    const double station=stations[r];auto xy=p.reference.at(station);if(!xy){valid=false;break;}
    double fade=1;
    if(start_on&&start_trim)fade=std::min(fade,std::clamp(station-*start_trim,0.0,1.0)); // the overlay covers this metre
    else if(start_trim)fade=std::min(fade,std::clamp((station-*start_trim)/2.0,0.0,1.0));
    else if(start_junction)fade=std::min(fade,std::clamp((station-3)/8.0,0.0,1.0));
    if(end_on&&end_trim)fade=std::min(fade,std::clamp(length-station-*end_trim,0.0,1.0));
    else if(end_trim)fade=std::min(fade,std::clamp((length-station-*end_trim)/2.0,0.0,1.0));
    else if(end_junction)fade=std::min(fade,std::clamp((length-station-3)/8.0,0.0,1.0));
    rows[static_cast<std::size_t>(r)]={station,xy->x,xy->y,-det::sin(xy->heading),det::cos(xy->heading),fade,span_at(entry,*layout,station),separated_at(entry,station)};
   }
   if(!valid)continue;
   // Three strips across: the right verge (gravel, ground only), the road bed (asphalt, the markings divide its
   // counted lanes) and the left verge. u is measured from the centre of the counted lanes.
   const bool on_ground=!rows[0].separated&&!rows[1].separated;
   for(int strip=on_ground?0:1;strip<(on_ground?3:2);++strip) {
    const auto lo=[&](const Row& r){return strip==0?r.span.bed_right-shoulder:strip==1?r.span.bed_right:r.span.bed_left;};
    const auto hi=[&](const Row& r){return strip==0?r.span.bed_right:strip==1?r.span.bed_left:r.span.bed_left+shoulder;};
    const int across=strip==1?std::max(2,static_cast<int>(std::ceil(hi(rows[0])-lo(rows[0])))):1;
    const auto first=mesh.vertices.size();bool ok=true;
    for(const auto& r:rows) {
     for(int j=0;j<=across&&ok;++j) {
      const double lateral=lo(r)+(hi(r)-lo(r))*j/across,e=r.x+r.nx*lateral,n=r.y+r.ny*lateral;
      const auto height=r.separated?deck_height(entry,r.station,lateral):ground(e,n);
      if(!height){ok=false;break;}
      mesh.vertices.push_back({static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*height+lift),static_cast<float>(lateral-r.span.centre()),static_cast<float>(r.station),static_cast<float>(r.span.width()),static_cast<float>(r.fade),static_cast<float>(lanes),static_cast<float>(forward),static_cast<float>(flags),strip==1?0.0f:1.0f});
     }
    }
    if(!ok){mesh.vertices.resize(first);continue;}
    for(int j=0;j<across;++j){const auto a=static_cast<std::int32_t>(first+j),b=a+across+1;mesh.indices.insert(mesh.indices.end(),{a,b,a+1,a+1,b,b+1});}
   }
  }
  // Direction arrows only where OSM explicitly records turn:lanes.
  const auto arrows=[&](std::string turns,int direction,int lane_start,int lane_count) {
   if(turns.empty()||lane_count<=0||length<20)return;
   std::vector<std::string> tokens;std::size_t begin=0;
   while(true){auto end=turns.find('|',begin);tokens.push_back(turns.substr(begin,end-begin));if(end==std::string::npos)break;begin=end+1;}
   if(static_cast<int>(tokens.size())!=lane_count)return;
   // The arrow (tail 2 m behind the station, tip 1.7 m ahead) stands in front of the first marking the traffic meets at
   // the end it drives toward: the junction trim, a stop or give-way line of this direction, a crossing. 15 m before
   // the end unless one of those reaches further back; then its tip ends kArrowGapM before it.
   constexpr double kArrowGapM=3.0,kArrowTipM=1.7,kArrowTailM=2.0,kArrowReachM=60.0;
   const std::size_t index=static_cast<std::size_t>(&entry-geometry.entries.data());
   double barrier=direction>0?length:0.0;
   const auto bar=[&](double s0,double s1){
    if(direction>0){if(s0>length-kArrowReachM)barrier=std::min(barrier,s0);}
    else if(s1<kArrowReachM)barrier=std::max(barrier,s1);
   };
   if(auto it=trims.find({entry.way_id,entry.stretch.start_ref,direction>0});it!=trims.end())
    bar(direction>0?length-it->second:0.0,direction>0?length:it->second);
   const bool travel_forward=direction>0;
   for(const auto& fu:geometry.furniture) {
    if(fu.decline!=g2m::FurnitureDecline::None||fu.entry_index!=index||fu.kind==g2m::FurnitureKind::Signal)continue;
    const bool line=fu.kind==g2m::FurnitureKind::StopLine||fu.kind==g2m::FurnitureKind::GiveWayLine;
    if(line&&(fu.travel==g2m::LaneDirection::Forward)!=travel_forward)continue;
    bar(fu.station_m-fu.length_m/2,fu.station_m+fu.length_m/2);
   }
   for(const auto& g:give_ways)if(g.entry==index)bar(g.station-.25,g.station+.25);
   const double station=direction>0?std::min(length-15,barrier-kArrowGapM-kArrowTipM):std::max(15.0,barrier+kArrowGapM+kArrowTipM);
   if(direction>0?station-kArrowTailM<0:station+kArrowTailM>length)return;
   auto center=p.reference.at(station);if(!center||!owned(center->x,center->y))return;
   const auto span=span_at(entry,*layout,station);const double width=span.width(),half=width/2,centre=span.centre();
   const double lane_width=width/lanes;if(lane_width<2.4)return;
   for(int lane=0;lane<lane_count;++lane){
    const double offset=direction>0?-half+(lane_start+lane_count-lane-.5)*lane_width:-half+(lane_start+lane+.5)*lane_width;
    const auto vertex=[&](double side,double along)->std::optional<RoadRibbonVertex>{
     const double st=station+direction*along;auto xy=p.reference.at(st);if(!xy)return {};
     const double lateral=centre+offset+direction*side,e=xy->x-det::sin(xy->heading)*lateral,n=xy->y+det::cos(xy->heading)*lateral;
     auto height=surface(entry,st,lateral,e,n);if(!height)return {};
     return RoadRibbonVertex{static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*height+lift+.003),0,0,static_cast<float>(width),1,0,0,kPaint};
    };
    const auto triangle=[&](double ax,double ay,double bx,double by,double cx,double cy){
     auto a=vertex(ax,ay),b=vertex(bx,by),c=vertex(cx,cy);if(!a||!b||!c)return;const auto first=static_cast<std::int32_t>(mesh.vertices.size());
     mesh.vertices.insert(mesh.vertices.end(),{*a,*b,*c});mesh.indices.insert(mesh.indices.end(),{first,first+1,first+2});
    };
    const auto stroke=[&](double ax,double ay,double bx,double by){const double dx=bx-ax,dy=by-ay,len=det::hypot(dx,dy);if(len<.01)return;const double nx=-dy/len*.11,ny=dx/len*.11;triangle(ax+nx,ay+ny,ax-nx,ay-ny,bx+nx,by+ny);triangle(ax-nx,ay-ny,bx-nx,by-ny,bx+nx,by+ny);};
    const auto& token=tokens[lane];
    const bool through=token.find("through")!=std::string::npos;
    const bool left=token.find("left")!=std::string::npos,right=token.find("right")!=std::string::npos;
    if(!through&&!left&&!right)continue;
    stroke(0,-2,0,0);
    if(through){stroke(0,0,0,1.1);triangle(-.45,.85,.45,.85,0,1.7);}
    if(left){stroke(0,0,.7,.5);triangle(.65,.15,.65,.85,1.15,.5);}
    if(right){stroke(0,0,-.7,.5);triangle(-.65,.15,-.65,.85,-1.15,.5);}
   }
  };
  if(one)arrows(tag(w,"turn:lanes"),direction=="-1"?-1:1,0,lanes);
  else {arrows(tag(w,"turn:lanes:forward"),1,0,forward);arrows(tag(w,"turn:lanes:backward"),-1,forward,lanes-forward);}
 }
 // The gravel verge around each junction this tile owns: the ribbons' verge strips end at the trim lines, so it follows
 // the road bed ring instead (kerb returns included) and stays open across each approach's mouth.
 for(const auto& j:geometry.junctions) {
  if(j.decline!=g2m::JunctionDecline::None||(j.flags&static_cast<std::uint16_t>(g2m::JunctionFlags::OnStructure)))continue;
  const auto& ring=j.rings[static_cast<std::size_t>(g2m::JunctionRingLayer::RoadBed)].vertices;if(ring.size()<3)continue;
  struct Mouth { double x,y,tx,ty,lo,hi; };
  std::vector<Mouth> mouths;
  for(const auto& ap:j.approaches) {
   if(ap.status!=g2m::JunctionApproachStatus::Used||ap.entry_index>=geometry.entries.size())continue;
   const auto& entry=geometry.entries[ap.entry_index];const auto* w=find_way(geometry,entry.way_id);if(!w||!entry.profile)continue;
   const auto layout=lane_layout(entry,*w);if(!layout)continue;
   const double length=entry.profile->reference.length_m,cut=std::clamp(ap.at_end?length-ap.trim_m:ap.trim_m,0.0,length);
   const auto pose=reference_pose(*entry.profile,cut);if(!pose)continue;const auto span=span_at(entry,*layout,cut);
   mouths.push_back({pose->x,pose->y,det::cos(pose->heading),det::sin(pose->heading),span.bed_right,span.bed_left});
  }
  // A ring vertex on an approach's trim line, across its road bed (within a few centimetres of rounding).
  const auto on_mouth=[&](const Mouth& m,double x,double y){
   const double along=(x-m.x)*m.tx+(y-m.y)*m.ty,lateral=-(x-m.x)*m.ty+(y-m.y)*m.tx;
   return std::abs(along)<.3&&lateral>m.lo-.3&&lateral<m.hi+.3;};
  const std::size_t n=ring.size();
  std::vector<char> open(n,0);  // segment i (vertex i to i+1) crosses a mouth
  for(std::size_t i=0;i<n;++i){const auto& a=ring[i];const auto& b=ring[(i+1)%n];
   for(const auto& m:mouths)if(on_mouth(m,a.x/1000.0,a.y/1000.0)&&on_mouth(m,b.x/1000.0,b.y/1000.0)){open[i]=1;break;}}
  // Outward offset per vertex: the ring is counter-clockwise, so outward is to the right of each segment; mitred at the
  // vertices, the open (mouth) segments' own normals still shape their neighbours' ends.
  constexpr double kVerge=.30;
  const auto normal=[&](std::size_t i)->P2{const auto& a=ring[i];const auto& b=ring[(i+1)%n];const double dx=(b.x-a.x)/1000.0,dy=(b.y-a.y)/1000.0,l=det::hypot(dx,dy);
   return l>1e-9?P2{dy/l,-dx/l}:P2{0,0};};
  std::vector<P2> outer(n);
  for(std::size_t i=0;i<n;++i){const P2 na=normal((i+n-1)%n),nb=normal(i);P2 m{na.x+nb.x,na.y+nb.y};const double l=det::hypot(m.x,m.y);
   const P2 base{ring[i].x/1000.0,ring[i].y/1000.0};
   if(l<1e-6){outer[i]={base.x+nb.x*kVerge,base.y+nb.y*kVerge};continue;}
   m={m.x/l,m.y/l};const double c=std::max(m.x*nb.x+m.y*nb.y,.5);outer[i]={base.x+m.x*kVerge/c,base.y+m.y*kVerge/c};}
  for(std::size_t i=0;i<n;++i) {
   if(open[i])continue;const std::size_t k=(i+1)%n;
   const P2 q[4]={{ring[i].x/1000.0,ring[i].y/1000.0},outer[i],{ring[k].x/1000.0,ring[k].y/1000.0},outer[k]};
   if(!owned((q[0].x+q[2].x)/2,(q[0].y+q[2].y)/2))continue;
   const auto first=static_cast<std::int32_t>(mesh.vertices.size());bool ok=true;
   for(const auto& v:q){auto h=ground(v.x,v.y);if(!h){ok=false;break;}
    mesh.vertices.push_back({static_cast<float>(v.x-ox),static_cast<float>(v.y-oy),static_cast<float>(*h+lift),0,0,1000.0f,0,0,0,0,1.0f});}
   if(!ok){mesh.vertices.resize(static_cast<std::size_t>(first));continue;}
   mesh.indices.insert(mesh.indices.end(),{first,first+2,first+1,first+1,first+2,first+3});
  }
 }
 add_junction_surfaces(mesh,geometry,key,ground,lift);
 // ---- markings over the junction patches ----
 constexpr double kOverlayLiftM=kJunctionLiftM+.003,kLineLiftM=kJunctionLiftM+.004;
 // The through road's lane, centre and edge lines continue across the junction along a cubic between the two trim
 // cuts; the edge line is broken across each side road's mouth (a broad dash on a motorway or trunk: the exit or the
 // merge). Only the paint is drawn (kOverlay: the shader discards the rest), so the junction patch shows between.

 // A painted segment on the ground (two triangles, heights from `ground`), for the gore's lines and stripes.
 const auto paint_segment=[&](P2 a,P2 b,double width) {
  const double dx=b.x-a.x,dy=b.y-a.y,len=det::hypot(dx,dy);if(len<.01)return;
  const double nx=-dy/len*width/2,ny=dx/len*width/2;
  const P2 c[4]={{a.x+nx,a.y+ny},{a.x-nx,a.y-ny},{b.x+nx,b.y+ny},{b.x-nx,b.y-ny}};
  const auto first=static_cast<std::int32_t>(mesh.vertices.size());
  for(const auto& q:c){auto h=ground(q.x,q.y);if(!h){mesh.vertices.resize(static_cast<std::size_t>(first));return;}
   mesh.vertices.push_back({static_cast<float>(q.x-ox),static_cast<float>(q.y-oy),static_cast<float>(*h+lift+kLineLiftM),0,0,1000.0f,1,0,0,kPaint});}
  mesh.indices.insert(mesh.indices.end(),{first,first+1,first+2,first+1,first+3,first+2});
 };
 // The gore where an exit leaves or an on-ramp joins the through road over a junction (sign 298, Sperrflaeche). The
 // ramp's inner edge line starts where the ramp leaves the through road (the paved ring first reaches beyond the through
 // road's bed edge), on the through road's edge line, and curves into the ramp's own edge line at its lead, so it carries
 // over into the ribbon's marking; it is drawn only where it has cleared that edge line, i.e. on the ramp. Diagonal bars
 // hatch the area between it and the through road's edge line up to the nose of the road bed ring, bordered on the
 // through side by a solid line.
 const auto paint_gore=[&](const Overlay& o,const auto& rows,const Approach& g) {
  if(rows.size()<2||!o.ring||o.ring->size()<3)return;
  const auto& a=o.a;const auto& b=o.b;const bool diverge=g.in_lanes==0;
  const double dx=b.pose.x-a.pose.x,dy=b.pose.y-a.pose.y;
  const double side=(dx*(g.pose.y-a.pose.y)-dy*(g.pose.x-a.pose.x))>0?1.0:-1.0;
  constexpr double kMarkInset=.25;  // edge lines sit this far inside the road bed edge (road.gdshader)
  // The through road's ramp-side bed edge and its edge line, with the strip's normals and arc lengths.
  std::vector<P2> edge,mark,normal;std::vector<double> edge_arc;
  for(const auto& r:rows){const double l=r.offset+side*r.width/2,m=r.offset+side*(r.width/2-kMarkInset);
   edge.push_back({r.x+r.nx*l,r.y+r.ny*l});mark.push_back({r.x+r.nx*m,r.y+r.ny*m});normal.push_back({r.nx,r.ny});edge_arc.push_back(r.arc);}
  // Closest point on a polyline (edge or mark): distance, arc length and the signed offset toward the ramp side.
  const auto nearest_on=[&](const std::vector<P2>& line,P2 p,double& arc,double& beyond)->double{
   double best=1e18;
   for(std::size_t i=0;i+1<line.size();++i){
    const double ex=line[i+1].x-line[i].x,ey=line[i+1].y-line[i].y,l2=ex*ex+ey*ey;
    const double t=l2>0?std::clamp(((p.x-line[i].x)*ex+(p.y-line[i].y)*ey)/l2,0.0,1.0):0;
    const double qx=line[i].x+t*ex,qy=line[i].y+t*ey,d=det::hypot(qx-p.x,qy-p.y);
    if(d<best){best=d;arc=edge_arc[i]+t*(edge_arc[i+1]-edge_arc[i]);beyond=side*((p.x-qx)*normal[i].x+(p.y-qy)*normal[i].y);}
   }
   return best;
  };
  const auto nearest=[&](P2 p,double& arc){double beyond=0;return nearest_on(edge,p,arc,beyond);};
  const auto at_arc=[&](const std::vector<P2>& line,double s)->P2{
   s=std::clamp(s,edge_arc.front(),edge_arc.back());
   for(std::size_t i=0;i+1<line.size();++i)if(s<=edge_arc[i+1]){const double l=edge_arc[i+1]-edge_arc[i],t=l>0?(s-edge_arc[i])/l:0;
    return {line[i].x+t*(line[i+1].x-line[i].x),line[i].y+t*(line[i+1].y-line[i].y)};}
   return line.back();
  };
  // The ramp's inner bed edge at its lead (the one of its two edges nearer the through road) and its edge line there.
  const double th=diverge?g.inward+3.14159265358979323846:g.inward;
  P2 inner{};{double best=1e18,arc=0;for(double k:{1.0,-1.0}){const P2 q{g.lead.x-det::sin(th)*k*g.width/2,g.lead.y+det::cos(th)*k*g.width/2};
   const double d=nearest(q,arc);if(d<best){best=d;inner=q;}}}
  P2 ramp_mark=inner;{const double ux=g.lead.x-inner.x,uy=g.lead.y-inner.y,l=det::hypot(ux,uy);
   if(l>kMarkInset){ramp_mark.x+=ux/l*kMarkInset;ramp_mark.y+=uy/l*kMarkInset;}}
  // Where the ramp leaves the through road: walking from the flush end, the first row at which the paved ring reaches
  // more than kLeaves beyond the through road's bed edge. The line starts there, on the through road's edge line, and
  // runs along the ramp only; the through lanes keep their own lines (OSM does not say whether the ramp takes over a
  // through lane, so none is assumed).
  constexpr double kLeaves=.5;
  const auto beyond_edge=[&](std::size_t i)->double{  // paved width outside the bed edge at row i, along the normal
   const P2 e=edge[i];const P2 d{side*normal[i].x,side*normal[i].y};
   if(!near_ring(*o.ring,e.x+d.x*.05,e.y+d.y*.05,0))return 0;
   double best=1e9;const auto& r=*o.ring;
   for(std::size_t k=0,m=r.size()-1;k<r.size();m=k++){
    const double ax=r[m].x/1000.0,ay=r[m].y/1000.0,bx=r[k].x/1000.0,by=r[k].y/1000.0,sx=bx-ax,sy=by-ay;
    const double den=d.x*sy-d.y*sx;if(std::abs(den)<1e-12)continue;
    const double t=((ax-e.x)*sy-(ay-e.y)*sx)/den,u=((ax-e.x)*d.y-(ay-e.y)*d.x)/den;
    if(t>0&&u>=0&&u<=1)best=std::min(best,t);}
   return best<1e8?best:0;};
  std::size_t start_row=diverge?0:rows.size()-1;bool found=false;
  for(std::size_t k=0;k<rows.size();++k){const std::size_t i=diverge?k:rows.size()-1-k;if(beyond_edge(i)>kLeaves){start_row=i;found=true;break;}}
  if(!found)return;
  const P2 start=mark[start_row];
  const double along=det::atan2(-normal[start_row].x,normal[start_row].y);  // the overlay's direction (a to b) there
  const P2 p0=diverge?start:ramp_mark,p3=diverge?ramp_mark:start;
  const double t0=diverge?along:g.inward,t3=diverge?g.inward+3.14159265358979323846:along;
  const double chord=det::hypot(p3.x-p0.x,p3.y-p0.y);if(chord<2||chord>150)return;
  const double h=chord/3;const P2 p1{p0.x+det::cos(t0)*h,p0.y+det::sin(t0)*h},p2{p3.x-det::cos(t3)*h,p3.y-det::sin(t3)*h};
  const int n=std::max(8,static_cast<int>(std::ceil(chord*4)));
  std::vector<P2> line;std::vector<double> line_arc;double arc=0;
  for(int i=0;i<=n;++i){const double t=static_cast<double>(i)/n,u=1-t;
   const P2 q{u*u*u*p0.x+3*u*u*t*p1.x+3*u*t*t*p2.x+t*t*t*p3.x,u*u*u*p0.y+3*u*u*t*p1.y+3*u*t*t*p2.y+t*t*t*p3.y};
   if(!line.empty())arc+=det::hypot(q.x-line.back().x,q.y-line.back().y);line.push_back(q);line_arc.push_back(arc);}
  constexpr double kLine=.15,kBar=.4,kBarStep=1.5,kBarClear=.6;
  const auto vertices_before=mesh.vertices.size(),indices_before=mesh.indices.size();
  const auto inside=[&](P2 q){return near_ring(*o.ring,q.x,q.y,.05);};
  // How far a point lies beyond the through road's edge line, toward the ramp.
  const auto clear_of=[&](P2 p){double ea=0,beyond=0;nearest_on(mark,p,ea,beyond);return beyond;};
  for(std::size_t i=0;i+1<line.size();++i){
   const P2 m{(line[i].x+line[i+1].x)/2,(line[i].y+line[i+1].y)/2};
   if(!near_ring(*o.ring,m.x,m.y,.3))continue;
   if(clear_of(m)>=kLine)paint_segment(line[i],line[i+1],kLine);
  }
  // Diagonal bars: from the line back toward the flush end onto the through edge line (45 degrees), every kBarStep,
  // only where the line has cleared that edge line.
  double last_arc=-1,first_arc=-1;
  for(double s=0;s<=arc;s+=.25){
   const double sf=diverge?s:arc-s;  // along the line from the flush end
   if(std::fmod(sf,kBarStep)>=.25)continue;
   std::size_t k=0;while(k+1<line_arc.size()&&line_arc[k+1]<s)++k;
   const double l=line_arc[k+1]-line_arc[k],t=l>0?std::clamp((s-line_arc[k])/l,0.0,1.0):0;
   const P2 p{line[k].x+t*(line[k+1].x-line[k].x),line[k].y+t*(line[k+1].y-line[k].y)};
   double ea=0,beyond=0;const double d=nearest_on(mark,p,ea,beyond);if(beyond<kBarClear)continue;
   const double qa=diverge?ea-d:ea+d;const P2 q=at_arc(mark,qa);
   const double len=det::hypot(q.x-p.x,q.y-p.y);if(len<.5)continue;
   const double ux=(q.x-p.x)/len,uy=(q.y-p.y)/len,trim=kLine/2+.1;
   const P2 ps{p.x+ux*trim,p.y+uy*trim},qs{q.x-ux*trim,q.y-uy*trim};
   if(!inside(ps)||!inside(qs)||!inside({(p.x+q.x)/2,(p.y+q.y)/2}))continue;
   paint_segment(ps,qs,kBar);
   if(first_arc<0||(diverge?qa<first_arc:qa>first_arc))first_arc=qa;
   if(last_arc<0||(diverge?ea>last_arc:ea<last_arc))last_arc=ea;
  }
  // A gore ends at a nose: halfway between the ramp's inner edge at its cut and the through road's edge lies outside
  // the road bed ring (grass or a kerb between the two roads, the ring's notch). Where the ring is paved across (a
  // lane gain or drop on a wide carriageway, the two roads still flush at the cut) there is nothing to hatch.
  bool nose=false;
  if(first_arc>=0){double ea=0;nearest(inner,ea);const P2 e=at_arc(edge,ea),m{(inner.x+e.x)/2,(inner.y+e.y)/2};
   // Not on the through road's own far approach either (its lanes can be wider than the overlay strip).
   const auto& far=diverge?b:a;const double fh=far.lead.heading;
   const double across=std::abs(-det::sin(fh)*(m.x-far.lead.x)+det::cos(fh)*(m.y-far.lead.y));
   nose=!near_ring(*o.ring,m.x,m.y,0)&&across>far.width/2+.5;}
  if(!nose){mesh.vertices.resize(vertices_before);mesh.indices.resize(indices_before);return;}
  // The border on the through side, from the first bar to the last, on the through road's edge line.
  const double lo=std::min(first_arc,last_arc),hi=std::max(first_arc,last_arc);
  for(double s=lo;s<hi;s+=.5)paint_segment(at_arc(mark,s),at_arc(mark,std::min(s+.5,hi)),kLine);
 };
 for(const auto& o:overlays) {
  const auto& a=o.a;const auto& b=o.b;
  const double ax=det::cos(a.inward),ay=det::sin(a.inward),bx=-det::cos(b.inward),by=-det::sin(b.inward);
  const double chord=det::hypot(b.lead.x-a.lead.x,b.lead.y-a.lead.y);if(chord<.5||chord>200)continue;
  if(!owned((a.pose.x+b.pose.x)/2,(a.pose.y+b.pose.y)/2))continue;
  const double h=chord/3;
  const double p0x=a.lead.x,p0y=a.lead.y,p1x=p0x+ax*h,p1y=p0y+ay*h,p3x=b.lead.x,p3y=b.lead.y,p2x=p3x-bx*h,p2y=p3y-by*h;
  // Lane layout in the a -> b direction: lanes on the right are the ones driving into the junction from a.
  int lanes=a.layout.lanes,forward=a.in_lanes;
  const int b_lanes=b.layout.lanes,b_right=b.layout.lanes-b.in_lanes;
  double wa=a.width,wb=b.width,oa=0,ob=0; // strip width and its centre offset (left +) at either end
  int o_hide=0;
  if(lanes!=b_lanes||forward!=b_right) {
   const bool two_way=forward>0&&forward<lanes&&b_right>0&&b_right<b_lanes;
   if(two_way){lanes=2;forward=1;}
   else if(forward==lanes&&b_right==b_lanes) {
    // A one-way road gaining or losing a lane at an exit or a merge: the through lanes run on, flush with the side
    // away from the ramp; their broken edge line is the line between them and the exit or merge lane.
    const int keep=std::min(lanes,b_lanes);const double lane=keep==lanes?a.width/lanes:b.width/b_lanes,w=keep*lane;
    const int side=o.broken==kRightEdgeBroken?1:o.broken==kLeftEdgeBroken?-1:0;
    oa=side*(a.width-w)/2;ob=side*(b.width-w)/2;wa=wb=w;lanes=forward=keep;
   }
   else if(forward==lanes&&b_right==lanes&&b.in_lanes>0) {
    // A one-way way into the matching half of a two-way road (a dual carriageway joining up): the strip covers that
    // half only; its left edge becomes the two-way road's centre, so no edge line there.
    wb=b.width*lanes/b_lanes;ob=-(b.width-wb)/2;o_hide|=kLeftEdgeHidden;
   }
   else if(b.in_lanes==0&&b_lanes==a.in_lanes&&forward>0&&forward<lanes) {
    // The two-way road's lanes toward the junction continuing into a one-way way (a dual carriageway splitting).
    wa=a.width*forward/lanes;oa=-(a.width-wa)/2;lanes=forward;o_hide|=kLeftEdgeHidden;
   }
   else {lanes=1;forward=1;}
  }
  int flags=(a.layout.flags&b.layout.flags&(kMarking|kEdgeLines))|(a.layout.flags&(kSolidSeparator|kFast))|kOverlay|o.broken|o_hide;
  if(!(flags&kMarking))continue;
  const int count=std::max(2,static_cast<int>(std::ceil(chord)));
  struct Row { double x,y,nx,ny,arc,width,offset; };
  std::vector<Row> rows;double arc=0,px=p0x,py=p0y;
  for(int i=0;i<=count;++i) {
   const double t=static_cast<double>(i)/count,u=1-t;
   const double x=u*u*u*p0x+3*u*u*t*p1x+3*u*t*t*p2x+t*t*t*p3x,y=u*u*u*p0y+3*u*u*t*p1y+3*u*t*t*p2y+t*t*t*p3y;
   const double dx=3*u*u*(p1x-p0x)+6*u*t*(p2x-p1x)+3*t*t*(p3x-p2x),dy=3*u*u*(p1y-p0y)+6*u*t*(p2y-p1y)+3*t*t*(p3y-p2y);
   const double len=std::max(det::hypot(dx,dy),1e-9);
   arc+=det::hypot(x-px,y-py);px=x;py=y;
   rows.push_back({x,y,-dy/len,dx/len,arc,wa+(wb-wa)*t,oa+(ob-oa)*t});
  }
  // An edge line that would leave the junction's road bed (the ring is narrower there, or a kerb return cuts the
  // corner) is not drawn: a line over the verge reads as wrong as a missing one.
  for(const auto& r:rows) {
   if(!o.ring||o.ring->size()<3||r.arc<1.5||r.arc>arc-1.5)continue; // the lead metre at either end lies on the ribbon
   for(int side:{1,-1}) {
    const double l=r.offset+side*(r.width/2-.25);
    if(!near_ring(*o.ring,r.x+r.nx*l,r.y+r.ny*l,.3))flags|=side>0?kLeftEdgeHidden:kRightEdgeHidden;
   }
  }
  const auto first=static_cast<std::int32_t>(mesh.vertices.size());bool valid=true;int across=0;
  for(const auto& r:rows) {
   if(!valid)break;
   const double x=r.x,y=r.y,nx=r.nx,ny=r.ny,width=r.width,half=width/2,offset=r.offset;
   across=std::max(2,static_cast<int>(std::ceil(std::max(wa,wb))));
   for(int j=0;j<=across;++j) {
    const double lateral=-half+width*j/across,e=x+nx*(offset+lateral),n=y+ny*(offset+lateral);
    const auto height=ground(e,n);if(!height){valid=false;break;}
    mesh.vertices.push_back({static_cast<float>(e-ox),static_cast<float>(n-oy),static_cast<float>(*height+lift+kOverlayLiftM),static_cast<float>(lateral),static_cast<float>(r.arc),static_cast<float>(width),1,static_cast<float>(lanes),static_cast<float>(forward),static_cast<float>(flags)});
   }
  }
  if(!valid){mesh.vertices.resize(static_cast<std::size_t>(first));continue;}
  for(int i=0;i<count;++i)for(int j=0;j<across;++j){const auto c=first+i*(across+1)+j,d=c+across+1;mesh.indices.insert(mesh.indices.end(),{c,d,c+1,c+1,d,d+1});}
  for(const auto& g:o.gores)paint_gore(o,rows,g);
 }
 // Stop, give-way and crossing markings: OSM-placed furniture (geo2map) and the give-way lines implied above.
 const auto in_tile=[&](const g2m::RoadGeomEntry& entry,double station){auto pose=reference_pose(*entry.profile,station);return pose&&owned(pose->x,pose->y);};
 const auto give_way_line=[&](const g2m::RoadGeomEntry& entry,double station,double depth,double t0,double t1) {
  // Wartelinie (sign 341): 0.50 m dashes with 0.25 m gaps across the lanes, starting at the centre-line end.
  const double lo=std::min(t0,t1),hi=std::max(t0,t1);
  for(double t=lo;t<hi-.05;t+=.75)paint_rect(entry,station-depth/2,station+depth/2,t,std::min(t+.5,hi),kLineLiftM);
 };
 for(const auto& g:give_ways){const auto& entry=geometry.entries[g.entry];if(in_tile(entry,g.station))give_way_line(entry,g.station,.5,g.t0,g.t1);}
 // A stop line whose centre falls on a placed crossing (a signal's line derived on a short link piece between two
 // junctions lands in the Furt of the crossing that already stops the traffic): the crossing's own line is the one
 // painted, a second bar across it reads as a duplicate.
 std::vector<std::array<g2m::LateralPoint,4>> crossing_quads;
 for(const auto& fu:geometry.furniture)if(fu.decline==g2m::FurnitureDecline::None&&fu.kind==g2m::FurnitureKind::Crossing)
  if(auto q=g2m::furniture_quad(geometry,fu))crossing_quads.push_back(*q);
 const auto inside_crossing=[&](const g2m::RoadFurniture& line) {
  auto q=g2m::furniture_quad(geometry,line);if(!q)return false;
  double cx=0,cy=0;for(const auto& p:*q){cx+=p.x/4;cy+=p.y/4;}
  for(const auto& c:crossing_quads) {
   // Inside the quad grown by 0.5 m: every edge's signed distance (counter-clockwise or clockwise alike) >= -0.5.
   double area=0;for(int k=0;k<4;++k){const auto& a=c[k];const auto& b=c[(k+1)%4];area+=a.x*b.y-b.x*a.y;}
   bool in=area!=0;
   for(int k=0;k<4&&in;++k){const auto& a=c[k];const auto& b=c[(k+1)%4];const double ex=b.x-a.x,ey=b.y-a.y,len=det::hypot(ex,ey);
    if(len<1e-9)continue;in=(area>0?1:-1)*(ex*(cy-a.y)-ey*(cx-a.x))/len>=-.5;}
   if(in)return true;
  }
  return false;
 };
 for(const auto& fu:geometry.furniture) {
  if(fu.decline!=g2m::FurnitureDecline::None||fu.kind==g2m::FurnitureKind::Signal||fu.entry_index>=geometry.entries.size())continue;
  const auto& entry=geometry.entries[fu.entry_index];if(!entry.profile||!in_tile(entry,fu.station_m))continue;
  const double s0=fu.station_m-fu.length_m/2,s1=fu.station_m+fu.length_m/2,lo=std::min(fu.t_from,fu.t_to),hi=std::max(fu.t_from,fu.t_to);
  if(fu.kind==g2m::FurnitureKind::StopLine){if(!inside_crossing(fu))paint_rect(entry,s0,s1,lo,hi,kLineLiftM);continue;}
  if(fu.kind==g2m::FurnitureKind::GiveWayLine){give_way_line(entry,fu.station_m,fu.length_m,lo,hi);continue;}
  // Crossings. Zebra (sign 293): 0.50 m bars along the road with 0.50 m gaps; a ladder adds the two side lines.
  // A signalled crossing without a recorded pattern, or "dashes": the Furt, two broken 0.12 m lines (0.50 / 0.20 m).
  auto pattern=fu.pattern;
  if(pattern==g2m::CrossingPattern::None&&(fu.flags&g2m::kFurnitureSignalised))pattern=g2m::CrossingPattern::Dashes;
  if(pattern==g2m::CrossingPattern::Zebra||pattern==g2m::CrossingPattern::Ladder) {
   const double span=hi-lo;const int bars=std::max(1,static_cast<int>(std::floor((span+.5)/1.0)));
   const double start=lo+(span-(bars*1.0-.5))/2;
   for(int k=0;k<bars;++k)paint_rect(entry,s0,s1,start+k,start+k+.5,kLineLiftM);
  }
  if(pattern==g2m::CrossingPattern::Ladder||pattern==g2m::CrossingPattern::Lines)
   for(double s:{s0,s1-.12})paint_rect(entry,s,s+.12,lo,hi,kLineLiftM);
  if(pattern==g2m::CrossingPattern::Dashes)
   for(double s:{s0,s1-.12})for(double t=lo;t<hi-.05;t+=.7)paint_rect(entry,s,s+.12,t,std::min(t+.5,hi),kLineLiftM);
 }
 return mesh;
}
}
