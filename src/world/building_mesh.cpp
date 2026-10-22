// Algorithmic reference: BeamNG.Earth server/buildings.py (606ba0f),
// Copyright (c) 2026 TecnaGamer, MIT; full notice in THIRD_PARTY_NOTICES.md.
// Reimplemented for g2m's deterministic math, mapped total heights, courtyards,
// stable feature IDs and engine-neutral materials. No BeamNG asset dependency.
#include "g2m/world/building_mesh.h"
#include "g2m/core/det_math.h"
#include <algorithm>
#include <array>
#include <charconv>
#include <cmath>
#include <string_view>

namespace g2m::world {
namespace {
using V = std::array<double, 3>;
using C = std::array<float, 3>;
V sub(V a, V b) { return {a[0]-b[0], a[1]-b[1], a[2]-b[2]}; }
V cross(V a, V b) { return {a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]}; }
std::uint64_t mix(std::uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL; x=(x^(x>>30))*0xbf58476d1ce4e5b9ULL;
    x=(x^(x>>27))*0x94d049bb133111ebULL; return x^(x>>31);
}
C colour(std::string_view s, C fallback) {
    if(s.size()==4 && s[0]=='#') {
        unsigned n=0; auto p=std::from_chars(s.data()+1,s.data()+4,n,16);
        if(p.ec==std::errc{} && p.ptr==s.data()+4)
            return {static_cast<float>((n>>8)&15)/15,static_cast<float>((n>>4)&15)/15,static_cast<float>(n&15)/15};
    }
    if(s.size()==7 && s[0]=='#') {
        unsigned n=0; auto p=std::from_chars(s.data()+1,s.data()+7,n,16);
        if(p.ec==std::errc{} && p.ptr==s.data()+7)
            return {static_cast<float>((n>>16)&255)/255,static_cast<float>((n>>8)&255)/255,static_cast<float>(n&255)/255};
    }
    if(s=="white")return {.9f,.9f,.88f}; if(s=="red")return {.6f,.16f,.12f};
    if(s=="brown")return {.35f,.23f,.15f}; if(s=="grey"||s=="gray")return {.5f,.5f,.5f};
    if(s=="black")return {.08f,.08f,.08f}; if(s=="beige")return {.76f,.71f,.60f};
    if(s=="yellow")return {.83f,.73f,.43f}; return fallback;
}
struct Builder {
    BuildingMesh mesh;
    const BuildingMeshOptions& o;
    void tri(V a,V b,V c,C tint,BuildingMaterial material) {
        const auto n=cross(sub(b,a),sub(c,a));
        const double len=std::sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
        if(len<1e-9)return;
        for(const auto p:{a,b,c}) {
            BuildingVertex v;
            v.position={static_cast<float>(p[0]-o.origin_e),static_cast<float>(p[1]-o.origin_n),static_cast<float>(p[2])};
            v.normal={static_cast<float>(n[0]/len),static_cast<float>(n[1]/len),static_cast<float>(n[2]/len)};
            v.colour=tint;v.material=material;
            // Metric UVs, relative to the tile origin (stable through floating-origin changes).
            v.uv={static_cast<float>(p[0]-o.origin_e),static_cast<float>(p[1]-o.origin_n)};
            mesh.vertices.push_back(v);
        }
    }
    void quad(V a,V b,V c,V d,C tint,BuildingMaterial material) {tri(a,b,c,tint,material);tri(a,c,d,tint,material);}
};
double area(const std::vector<BuildingPoint>& ring) {
    double a=0;const auto origin=ring.front();
    for(std::size_t i=0;i<ring.size();++i){const auto p=ring[i],q=ring[(i+1)%ring.size()];a+=(p.e-origin.e)*(q.n-origin.n)-(q.e-origin.e)*(p.n-origin.n);}return a*.5;
}
struct Box {double e=0,n=0,ux=0,uy=0,hl=0,hw=0;};
bool rectangle(const Building& b,Box& box) {
    if(b.outers.size()!=1||!b.inners.empty()||b.outers[0].size()!=4)return false;
    const auto& r=b.outers[0];double longest=0;
    for(std::size_t i=0;i<4;++i){const auto a=r[i],c=r[(i+1)%4];const double x=c.e-a.e,y=c.n-a.n,l=x*x+y*y;
        if(l>longest){longest=l;box.ux=x;box.uy=y;}}
    if(longest<1)return false;const double len=std::sqrt(longest);box.ux/=len;box.uy/=len;
    // Canonical orientation independent of winding and the first source vertex.
    if(box.ux<0||(box.ux==0&&box.uy<0)){box.ux=-box.ux;box.uy=-box.uy;}
    const auto p0=r.front();double a0=1e30,a1=-1e30,b0=1e30,b1=-1e30;
    for(auto p:r){const double x=p.e-p0.e,y=p.n-p0.n,u=x*box.ux+y*box.uy,v=-x*box.uy+y*box.ux;
        a0=std::min(a0,u);a1=std::max(a1,u);b0=std::min(b0,v);b1=std::max(b1,v);}
    box.hl=(a1-a0)*.5;box.hw=(b1-b0)*.5;
    if(box.hw<.5||box.hl<.5)return false;
    box.e=p0.e+box.ux*(a0+a1)*.5-box.uy*(b0+b1)*.5;
    box.n=p0.n+box.uy*(a0+a1)*.5+box.ux*(b0+b1)*.5;
    for(auto p:r){const double x=p.e-box.e,y=p.n-box.n,u=x*box.ux+y*box.uy,v=-x*box.uy+y*box.ux;
        if(std::abs(std::abs(u)-box.hl)>.03||std::abs(std::abs(v)-box.hw)>.03)return false;}
    return true;
}
void flat_roof(Builder& m,const Building& b,double z,C tint) {
    // Even/odd scan slabs cover concave rings and courtyard holes without an
    // unconstrained triangulation. Same exact contours as the footprint walls.
    std::vector<double> ys;std::vector<const std::vector<BuildingPoint>*> rings;
    for(const auto& r:b.outers)rings.push_back(&r);for(const auto& r:b.inners)rings.push_back(&r);
    for(const auto* r:rings)for(auto p:*r)ys.push_back(p.n);
    std::sort(ys.begin(),ys.end());ys.erase(std::unique(ys.begin(),ys.end()),ys.end());
    struct Edge {double mid,lo,hi;};
    for(std::size_t j=1;j<ys.size();++j){const double lo=ys[j-1],hi=ys[j],mid=(lo+hi)*.5;
        std::vector<Edge> edges;
        for(const auto* r:rings)for(std::size_t i=0;i<r->size();++i){auto a=(*r)[i],c=(*r)[(i+1)%r->size()];
            if(mid<=std::min(a.n,c.n)||mid>=std::max(a.n,c.n))continue;
            const double slope=(c.e-a.e)/(c.n-a.n);edges.push_back({a.e+(mid-a.n)*slope,a.e+(lo-a.n)*slope,a.e+(hi-a.n)*slope});}
        std::sort(edges.begin(),edges.end(),[](const auto& a,const auto& c){return a.mid<c.mid;});
        for(std::size_t i=1;i<edges.size();i+=2){auto a=edges[i-1],c=edges[i];
            m.quad({a.lo,lo,z},{c.lo,lo,z},{c.hi,hi,z},{a.hi,hi,z},tint,BuildingMaterial::Roof);}
    }
}
} // namespace

BuildingMesh build_building_mesh(const Building& b,const BuildingMeshOptions& options) {
    Builder m{{},options};
    if(!std::isfinite(b.height)||!std::isfinite(b.base)||!std::isfinite(b.bottom)||b.height<=0||b.height>2000||b.bottom>=b.base+b.height)return {};
    if(!std::isfinite(options.origin_e)||!std::isfinite(options.origin_n))return {};
    for(const auto* rings:{&b.outers,&b.inners})for(const auto& r:*rings){if(r.size()<3)return {};
        for(auto p:r)if(!std::isfinite(p.e)||!std::isfinite(p.n))return {};}
    if(b.outers.empty())return {};
    const auto seed=mix(static_cast<std::uint64_t>(b.id)^options.seed^(b.relation?0xa0761d6478bd642fULL:0));
    constexpr C palette[]={{.77f,.73f,.65f},{.68f,.64f,.55f},{.84f,.80f,.70f},{.63f,.45f,.34f},{.69f,.70f,.69f}};
    const C wall=colour(b.colour,palette[seed%5]);
    const C roof=colour(b.roof_colour,seed%3==0?C{.24f,.25f,.27f}:C{.43f,.21f,.14f});
    const double top=b.base+b.height;
    Box box;const bool rectangular=rectangle(b,box);
    RoofForm form=RoofForm::Flat;
    if(b.roof_shape=="gabled")form=RoofForm::Gabled;
    else if(b.roof_shape=="hipped")form=RoofForm::Hipped;
    else if(b.roof_shape=="skillion")form=RoofForm::Skillion;
    else if(b.roof_shape=="pyramidal")form=RoofForm::Pyramidal;
    else if(!b.roof_shape.empty()&&b.roof_shape!="flat")m.mesh.unsupported_roof=true;
    else if(b.roof_shape.empty()&&options.infer_residential_roofs&&b.height<=18&&
        (b.type=="house"||b.type=="detached"||b.type=="semidetached_house"||b.type=="terrace")){
        form=seed%4==0?RoofForm::Hipped:RoofForm::Gabled;m.mesh.inferred_roof=true;}
    // Arbitrary mapped ridge azimuths need polygon roof clipping, not an
    // approximate box. Preserve the tag and report the fallback explicitly.
    if(form!=RoofForm::Flat&&(!rectangular||!b.roof_direction.empty())){form=RoofForm::Flat;m.mesh.unsupported_roof=true;}
    if(b.roof_orientation=="across"&&rectangular){std::swap(box.hl,box.hw);const auto u=box.ux;box.ux=-box.uy;box.uy=u;}
    double rise=0;
    if(form!=RoofForm::Flat){rise=b.roof_height>0?b.roof_height:std::min(box.hw*.6,3.5);
        rise=std::min(rise,std::max(0.0,(top-std::max(b.base+b.min_height,b.bottom))*.65));
        if(rise<.05)form=RoofForm::Flat;}
    if(form==RoofForm::Flat)rise=0;
    const double eave=top-rise;
    const double storey=std::isfinite(options.storey_height_m)?std::clamp(options.storey_height_m,2.0,6.0):3.0;
    const double bay=std::isfinite(options.bay_width_m)?std::clamp(options.bay_width_m,1.5,8.0):2.8;
    const int floors=std::clamp(static_cast<int>(b.levels>0?b.levels:std::floor((eave-b.base-b.min_height)/storey)),1,64);
    const double floor_h=(eave-b.base-b.min_height)/floors;
    for(const auto* rings:{&b.outers,&b.inners})for(const auto& r:*rings){
        const bool inner=rings==&b.inners;const bool reverse=(area(r)<0)!=inner;
        for(std::size_t i=0;i<r.size();++i){auto a=r[i],c=r[(i+1)%r.size()];if(reverse)std::swap(a,c);
            const double dx=c.e-a.e,dy=c.n-a.n,len=std::sqrt(dx*dx+dy*dy);if(len<.001)continue;
            m.quad({a.e,a.n,b.bottom},{c.e,c.n,b.bottom},{c.e,c.n,eave},{a.e,a.n,eave},wall,BuildingMaterial::Wall);
            if(!options.facade_detail||floor_h<2||b.type=="garage"||b.type=="garages"||b.type=="shed")continue;
            const int bays=std::clamp(static_cast<int>(std::floor(len/bay)),1,64);const double width=len/bays;
            const double nx=dy/len,ny=-dx/len;
            for(int f=0;f<floors;++f)for(int k=0;k<bays;++k){
                if(m.mesh.windows>=std::min(options.max_windows,4096U))break;
                const double s0=width*(k+.25),s1=width*(k+.75),z0=b.base+b.min_height+floor_h*(f+.28),z1=b.base+b.min_height+floor_h*(f+.76);
                const auto at=[&](double s,double z,double off)->V{return {a.e+dx/len*s+nx*off,a.n+dy/len*s+ny*off,z};};
                m.quad(at(s0-.06,z0-.06,.012),at(s1+.06,z0-.06,.012),at(s1+.06,z1+.06,.012),at(s0-.06,z1+.06,.012),{.27f,.26f,.24f},BuildingMaterial::Frame);
                const auto variation=mix(seed^static_cast<std::uint64_t>(i*4096+static_cast<std::size_t>(f)*64+static_cast<std::size_t>(k)));
                const C glass=variation%7==0?C{.30f,.33f,.35f}:C{.12f,.20f,.25f};
                m.quad(at(s0,z0,.018),at(s1,z0,.018),at(s1,z1,.018),at(s0,z1,.018),glass,BuildingMaterial::Glass);++m.mesh.windows;
            }
        }
    }
    m.mesh.roof=form;
    if(form==RoofForm::Flat){flat_roof(m,b,top,roof);return std::move(m.mesh);}
    const double overhang=std::isfinite(options.eave_m)?std::clamp(options.eave_m,0.0,.5):.2;
    const double hl=box.hl+overhang,hw=box.hw+overhang;
    const double ze=eave-overhang*rise/box.hw;
    const auto at=[&](double u,double v,double z)->V{return {box.e+box.ux*u-box.uy*v,box.n+box.uy*u+box.ux*v,z};};
    if(form==RoofForm::Skillion){
        m.quad(at(-hl,-hw,ze),at(hl,-hw,ze),at(hl,hw,top),at(-hl,hw,top),roof,BuildingMaterial::Roof);
        m.tri(at(-box.hl,box.hw,eave),at(-box.hl,-box.hw,eave),at(-box.hl,box.hw,top),wall,BuildingMaterial::Wall);
        m.tri(at(box.hl,-box.hw,eave),at(box.hl,box.hw,eave),at(box.hl,box.hw,top),wall,BuildingMaterial::Wall);
        m.quad(at(box.hl,box.hw,eave),at(-box.hl,box.hw,eave),at(-box.hl,box.hw,top),at(box.hl,box.hw,top),wall,BuildingMaterial::Wall);
    } else {
        const double ridge=form==RoofForm::Gabled?hl:(form==RoofForm::Pyramidal?0:std::max(hl-hw,0.0));
        m.quad(at(-hl,-hw,ze),at(hl,-hw,ze),at(ridge,0,top),at(-ridge,0,top),roof,BuildingMaterial::Roof);
        m.quad(at(hl,hw,ze),at(-hl,hw,ze),at(-ridge,0,top),at(ridge,0,top),roof,BuildingMaterial::Roof);
        if(form==RoofForm::Gabled){
            m.tri(at(-box.hl,box.hw,eave),at(-box.hl,-box.hw,eave),at(-box.hl,0,top),wall,BuildingMaterial::Wall);
            m.tri(at(box.hl,-box.hw,eave),at(box.hl,box.hw,eave),at(box.hl,0,top),wall,BuildingMaterial::Wall);
        } else {
            m.tri(at(hl,-hw,ze),at(hl,hw,ze),at(ridge,0,top),roof,BuildingMaterial::Roof);
            m.tri(at(-hl,hw,ze),at(-hl,-hw,ze),at(-ridge,0,top),roof,BuildingMaterial::Roof);
        }
    }
    return std::move(m.mesh);
}
} // namespace g2m::world
