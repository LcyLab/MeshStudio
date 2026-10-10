// 文件职责：边界尺寸传播：沿同面有效邻接传播三维长度尺度，避免穿过孔洞或独立区域。
#include "SurfaceBoundarySizeField.h"
#include <algorithm>
#include <cmath>
#include <functional>
#include <queue>
namespace mesher {
/// 沿有效网格路径传播固定边界尺寸并初始化节点尺寸。
bool SurfaceBoundarySizeField::initialize(const std::vector<Point3D>& points,const std::vector<std::array<int,3>>& triangles,
    const std::vector<std::pair<int,int>>& boundary,double target,double minSize,double growth,std::string* error,
    const std::vector<std::pair<int,double>>& seedSizes) {
    const auto fail=[&](const char* text){if(error)*error=text;return false;};
    if(!(target>=minSize && minSize>0 && growth>=1)||!std::isfinite(target)||!std::isfinite(growth))return fail("invalid boundary size criteria");
    sizes.assign(points.size(),target);
    targetSize=target;slope=growth-1;graph.assign(points.size(),{});
    const auto valid=[&](int v){return v>=0 && size_t(v)<points.size();};
    const auto length=[&](int a,int b){return std::hypot(points[a].x-points[b].x,points[a].y-points[b].y,points[a].z-points[b].z);};
    for(auto t:triangles){for(int v:t)if(!valid(v))return fail("boundary size triangle index out of range");
        for(int k=0;k<3;++k){int a=t[k],b=t[(k+1)%3];double l=length(a,b);if(!std::isfinite(l))return fail("non-finite boundary size geometry");graph[a].push_back({b,l});graph[b].push_back({a,l});}}
    using Item=std::pair<double,int>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    for(auto e:boundary){if(!valid(e.first)||!valid(e.second))return fail("boundary size segment index out of range");
        const double len=length(e.first,e.second);if(!std::isfinite(len))return fail("non-finite boundary segment");if(!(len>0))continue;
        if(seedSizes.empty()){const double h=std::clamp(len,minSize,target);for(int v:{e.first,e.second})if(h<sizes[v]){sizes[v]=h;queue.push({h,v});}}}
    for(auto [v,size]:seedSizes){if(!valid(v)||!(size>0)||!std::isfinite(size))return fail("invalid fixed boundary size seed");
        const double h=std::clamp(size,minSize,target);if(h<sizes[v]){sizes[v]=h;queue.push({h,v});}}
    // 按三维路径长度增长，避免初始长边把 tiny seed 一跳传播到整个面。
    while(!queue.empty()){const auto [h,v]=queue.top();queue.pop();if(h!=sizes[v])continue;
        for(auto [n,l]:graph[v]){const double candidate=std::min(target,h+(growth-1)*l);if(candidate<sizes[n]){sizes[n]=candidate;queue.push({candidate,n});}}}
    return true;
}

/// 按邻域尺寸和距离追加新节点的尺寸状态。
bool SurfaceBoundarySizeField::append(const std::vector<std::pair<int,double>>& neighbors,std::string* error) {
    if(neighbors.empty() || !(targetSize>0)){if(error)*error="missing boundary size insertion neighborhood";return false;}
    double size=targetSize;
    for(auto [v,d]:neighbors){if(v<0 || size_t(v)>=sizes.size() || !(d>=0) || !std::isfinite(d)){
        if(error)*error="invalid boundary size insertion neighbor";return false;}
        size=std::min(size,sizes[v]+slope*d);}
    const int added=int(sizes.size());sizes.push_back(size);graph.push_back(neighbors);
    for(auto [v,d]:neighbors)graph[v].push_back({added,d});
    using Item=std::pair<double,int>;std::priority_queue<Item,std::vector<Item>,std::greater<Item>> queue;
    queue.push({size,added});
    while(!queue.empty()){auto [h,v]=queue.top();queue.pop();if(h!=sizes[v])continue;
        for(auto [n,d]:graph[v]){const double next=std::min(targetSize,h+slope*d);if(next<sizes[n]){sizes[n]=next;queue.push({next,n});}}}
    return true;
}
/// 取候选三角形各节点尺寸的最小值。
double SurfaceBoundarySizeField::candidateSize(std::array<int,3> vertices) const {
    return std::min({at(vertices[0]),at(vertices[1]),at(vertices[2])});
}
}
