// 文件职责：质量阶段尺寸协调：统一目标尺寸和新增边下限，使局部几何需求与常规密度参数配合。
#include "SurfaceSizingPolicy.h"
#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <tuple>
namespace mesher {
/// 协调目标尺寸、最小尺寸、边界种子及增长率。
bool SurfaceSizingPolicy::initialize(const std::vector<Point3D>& points,const std::vector<std::array<int,3>>& triangles,
    const std::vector<std::pair<int,int>>& boundary,const std::vector<CadBoundarySizingAnchor>& anchors,
    const SurfaceCriteria& c,std::string* error) {
    const auto fail=[&](const char* why){if(error)*error=why;return false;};
    if(!(c.minSize>0&&c.targetSize>=c.minSize&&c.growthRatio>=1&&c.maxRadiusEdgeRatio>.5)||
        !std::isfinite(c.targetSize)||!std::isfinite(c.minSize)||!std::isfinite(c.growthRatio)||!std::isfinite(c.maxRadiusEdgeRatio))
        return fail("invalid coordinated surface sizing criteria");
    count=0;regularFloor=c.minSize;slope=c.growthRatio-1;
    const auto valid=[&](int v){return v>=0&&size_t(v)<points.size();};
    const auto length=[&](int a,int b){auto x=points[a],y=points[b];return std::hypot(x.x-y.x,x.y-y.y,x.z-y.z);};
    std::map<std::tuple<double,double,double>,double> sources;
    for(auto a:anchors){if(!(a.size>0)||!std::isfinite(a.size)||!std::isfinite(a.point.x)||!std::isfinite(a.point.y)||!std::isfinite(a.point.z))
            return fail("invalid coordinated sizing anchor");
        auto [it,added]=sources.emplace(std::make_tuple(a.point.x,a.point.y,a.point.z),a.size);
        if(!added)it->second=std::min(it->second,a.size);}
    std::map<int,double> targetSeeds,floorSeeds,regularSeeds;
    std::map<int,std::vector<Point3D>> directions;
    std::set<std::pair<int,int>> boundaryKeys;
    double smallestFloor=c.minSize,smallestTarget=c.minSize;
    for(auto [a,b]:boundary){if(!valid(a)||!valid(b))return fail("coordinated sizing boundary index out of range");
        const double len=length(a,b);if(!(len>0)||!std::isfinite(len))continue;
        boundaryKeys.insert(std::minmax(a,b));
        for(int v:{a,b}){targetSeeds.emplace(v,c.targetSize);floorSeeds.emplace(v,c.minSize);regularSeeds.emplace(v,c.targetSize);
            const auto p=points[v];auto source=sources.find({p.x,p.y,p.z});
            const double size=source==sources.end()?(anchors.empty()?len:c.targetSize):source->second;
            const bool exception=size<c.minSize;
            const double floor=exception?size*.5:c.minSize;
            const double target=std::clamp(size,exception?size:c.minSize,c.targetSize);
            regularSeeds[v]=std::min(regularSeeds[v],std::clamp(size,c.minSize,c.targetSize));
            targetSeeds[v]=std::min(targetSeeds[v],target);floorSeeds[v]=std::min(floorSeeds[v],floor);
            smallestFloor=std::min(smallestFloor,floor);smallestTarget=std::min(smallestTarget,target);
            auto q=points[v==a?b:a];directions[v].push_back({(q.x-p.x)/len,(q.y-p.y)/len,(q.z-p.z)/len});}}
    const auto dot=[](Point3D a,Point3D b){return a.x*b.x+a.y*b.y+a.z*b.z;};
    const auto tangent=[&](int v,Point3D& d){auto it=directions.find(v);if(it==directions.end()||it->second.empty())return false;
        d=it->second.front();for(auto q:it->second)if(std::abs(dot(d,q))<.95)return false;return true;};
    // 仅域内拓扑边连接的平行边界作为宽度证据；不查询跨孔/跨部件欧氏最近点。
    for(auto t:triangles){for(int v:t)if(!valid(v))return fail("coordinated sizing triangle index out of range");
        for(int k=0;k<3;++k){int a=t[k],b=t[(k+1)%3];if(boundaryKeys.count(std::minmax(a,b)))continue;
            Point3D da,db;if(!tangent(a,da)||!tangent(b,db)||std::abs(dot(da,db))<.95)continue;
            const double width=length(a,b);if(!(width>0)||!std::isfinite(width))continue;
            auto x=points[a],y=points[b];Point3D cross{(y.x-x.x)/width,(y.y-x.y)/width,(y.z-x.z)/width};
            if(std::abs(dot(cross,da))>.25||std::abs(dot(cross,db))>.25)continue;
            for(int v:{a,b})targetSeeds[v]=std::min(targetSeeds[v],std::max(width,c.minSize));}}
    std::vector<std::pair<int,double>> targetInput(targetSeeds.begin(),targetSeeds.end()),floorInput(floorSeeds.begin(),floorSeeds.end());
    std::vector<std::pair<int,double>> regularInput(regularSeeds.begin(),regularSeeds.end());
    if(!targets.initialize(points,triangles,boundary,c.targetSize,smallestTarget,c.growthRatio,error,targetInput)||
        !floors.initialize(points,triangles,boundary,c.minSize,smallestFloor,floorGrowthRatio(c.growthRatio),error,floorInput)||
        !regularTargets.initialize(points,triangles,boundary,c.targetSize,c.minSize,c.growthRatio,error,regularInput))return false;
    count=points.size();return true;
}
/// 结合邻域距离计算新节点允许的最小尺寸。
double SurfaceSizingPolicy::insertionFloor(const std::vector<std::pair<int,double>>& neighbors) const {
    double h=regularFloor;for(auto [v,d]:neighbors)h=std::min(h,floor(v)+slope*.5*d);return h;
}
/// 按邻域尺寸和距离追加新节点的尺寸状态。
bool SurfaceSizingPolicy::append(const std::vector<std::pair<int,double>>& neighbors,std::string* error) {
    if(count==0||neighbors.empty()){if(error)*error="missing coordinated sizing neighborhood";return false;}
    for(auto [v,d]:neighbors)if(v<0||size_t(v)>=count||!(d>=0)||!std::isfinite(d)){
        if(error)*error="invalid coordinated sizing neighbor";return false;}
    if(!targets.append(neighbors,error)||!floors.append(neighbors,error)||!regularTargets.append(neighbors,error))return false;
    ++count;return true;
}
}
