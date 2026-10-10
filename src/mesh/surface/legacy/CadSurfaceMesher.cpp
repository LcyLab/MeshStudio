// 文件职责：默认 CAD 表面编排：准备共享边、逐面生成和验收，再装配结果；几何查询通过 GeometrySource 提供。
#include "CadSurfaceMesher.h"

#include "mesh/planar/Constrained2D.h"
#include "CadBoundaryRegistry.h"
#include "mesh/planar/Delaunay2D.h"
#include "geometry/predicates/GeomPredicates.h"
#include "mesh/validation/SurfaceMeshValidation.h"
#include "SurfaceChart.h"
#include "SurfaceEvaluationCache.h"
#include "mesh/data/Mesh2DTransaction.h"
#include "SurfaceDomainState.h"
#include "SurfaceRefinementQueue.h"
#include "SurfaceBoundarySizeField.h"
#include "SurfaceSizingPolicy.h"
#include "SurfaceQualityPolicy.h"
#include "SurfaceRefinementCandidates.h"
#include "SurfaceTangentChart.h"
#include "SurfaceLocalRemesh.h"
#include "SurfaceFrontalDelaunay.h"
#include "SurfaceReferenceFront.h"
#include "SurfaceReferenceBoundary.h"
#include "SurfaceMeshingPartition.h"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <memory>
#include <sstream>
#include <iomanip>
#include <set>
#include <unordered_map>
#include <unordered_set>

namespace mesher {
/// 计算 CAD 边在指定参数处的一阶导数。
Point3D GeometrySource::edgeDerivative(int edge,double t) const {
    const auto range=edgeRange(edge);const double nan=std::numeric_limits<double>::quiet_NaN();
    if(!std::isfinite(t)||!std::isfinite(range.first)||!std::isfinite(range.second)||
        !(range.second>range.first)||t<range.first||t>range.second)return {nan,nan,nan};
    const double h=std::cbrt(std::numeric_limits<double>::epsilon())*(range.second-range.first);
    const double a=std::max(range.first,t-h),b=std::min(range.second,t+h);
    if(!(b>a))return {nan,nan,nan};const auto p=edgePoint(edge,a),q=edgePoint(edge,b);
    return {(q.x-p.x)/(b-a),(q.y-p.y)/(b-a),(q.z-p.z)/(b-a)};
}
/// 计算面的参数导数，供局部度量和法向计算使用。
void GeometrySource::faceDerivatives(int face,Point2D uv,Point3D& du,Point3D& dv) const {
    const auto invalid=[&] {
        const double nan=std::numeric_limits<double>::quiet_NaN(); du={nan,nan,nan}; dv=du;
    };
    SurfaceParameterBounds bounds;
    const bool bounded=faceParameterBounds(face,bounds);
    if (!std::isfinite(uv.x)||!std::isfinite(uv.y) || (bounded &&
        (!std::isfinite(bounds.uMin)||!std::isfinite(bounds.uMax)||
         !std::isfinite(bounds.vMin)||!std::isfinite(bounds.vMax)||
         !(bounds.uMin<bounds.uMax && bounds.vMin<bounds.vMax)||
         uv.x<bounds.uMin || uv.x>bounds.uMax || uv.y<bounds.vMin || uv.y>bounds.vMax))) {
        invalid(); return;
    }
    const double eps=std::cbrt(std::numeric_limits<double>::epsilon());
    const double hu=eps*std::max({1.0,std::abs(uv.x),bounded?bounds.uMax-bounds.uMin:0.0});
    const double hv=eps*std::max({1.0,std::abs(uv.y),bounded?bounds.vMax-bounds.vMin:0.0});
    const double u0=bounded?std::max(bounds.uMin,uv.x-hu):uv.x-hu;
    const double u1=bounded?std::min(bounds.uMax,uv.x+hu):uv.x+hu;
    const double v0=bounded?std::max(bounds.vMin,uv.y-hv):uv.y-hv;
    const double v1=bounded?std::min(bounds.vMax,uv.y+hv):uv.y+hv;
    if (!(u1>u0 && v1>v0)) { invalid(); return; }
    const auto ua=facePoint(face,{u0,uv.y}),ub=facePoint(face,{u1,uv.y});
    const auto va=facePoint(face,{uv.x,v0}),vb=facePoint(face,{uv.x,v1});
    du={(ub.x-ua.x)/(u1-u0),(ub.y-ua.y)/(u1-u0),(ub.z-ua.z)/(u1-u0)};
    dv={(vb.x-va.x)/(v1-v0),(vb.y-va.y)/(v1-v0),(vb.z-va.z)/(v1-v0)};
}
namespace {

/// 计算两个三维点的距离。
double length(Point3D a, Point3D b) {
    return std::hypot(std::hypot(a.x-b.x,a.y-b.y),a.z-b.z);
}
/// 计算三维向量点积。
double dot3(Point3D a, Point3D b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
/// 计算三维坐标之差。
Point3D minus3(Point3D a, Point3D b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
/// 计算三维向量叉积。
Point3D cross3(Point3D a, Point3D b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
/// 计算三维向量的欧氏长度。
double norm3(Point3D a) { return std::sqrt(dot3(a,a)); }
/// 计算两个有效法向的夹角，单位为度。
double angleDeg(Point3D a, Point3D b) {
    const double den=norm3(a)*norm3(b);
    // CAD 极点端部可能没有有效法向；仍由附近有效采样控制周围的法向变化。
    if (!(den>0) || !std::isfinite(den)) return 0;
    const double c=std::clamp(dot3(a,b)/den,-1.0,1.0);
    return std::acos(c)*180.0/3.14159265358979323846;
}
/// 计算采样点到弦线段的最短距离。
double lineDeviation(Point3D a, Point3D b, Point3D p) {
    const Point3D d=minus3(b,a);
    const double len2=dot3(d,d);
    if (len2==0) return length(a,p);
    const double t=std::clamp(dot3(minus3(p,a),d)/len2,0.0,1.0);
    return length({a.x+t*d.x,a.y+t*d.y,a.z+t*d.z},p);
}
/// 检查输入坐标或数值是否有限。
bool finite(Point3D p) { return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z); }
/// 检查输入坐标或数值是否有限。
bool finite(Point2D p) { return std::isfinite(p.x)&&std::isfinite(p.y); }
/// 将无向节点对编码为边查找键。
uint64_t key(int a,int b) {
    if (a>b) std::swap(a,b);
    return (uint64_t(uint32_t(a))<<32)|uint32_t(b);
}
/// 计算三角形的外接圆半径与最短边之比。
double ratio(Point3D a,Point3D b,Point3D c) {
    const double x=length(a,b),y=length(b,c),z=length(c,a);
    const double twiceArea=norm3(cross3(minus3(b,a),minus3(c,a)));
    if (twiceArea<=0 || std::min({x,y,z})<=0) return std::numeric_limits<double>::infinity();
    return (x*y*z)/(2*twiceArea*std::min({x,y,z}));
}
struct TriangleMetrics {
    bool valid=true;
    bool inverted=false;
    double maxEdge=0,deviation=0,normalAngle=0,quality=0,minEdge=0;
};
using FaceEvaluationCache=SurfaceEvaluationCache;
/// 测量三角形的尺寸、形状和 CAD 偏差。
TriangleMetrics measureTriangle(FaceEvaluationCache& queries,
                                const std::array<Point2D,3>& uv,
                                const std::array<Point3D,3>& xyz) {
    MeshStageScope scope(queries.observation,MeshStage::CriterionMeasurement);
    TriangleMetrics m;
    m.maxEdge=std::max({length(xyz[0],xyz[1]),length(xyz[1],xyz[2]),length(xyz[2],xyz[0])});
    m.minEdge=std::min({length(xyz[0],xyz[1]),length(xyz[1],xyz[2]),length(xyz[2],xyz[0])});
    m.quality=ratio(xyz[0],xyz[1],xyz[2]);
    const Point2D center{(uv[0].x+uv[1].x+uv[2].x)/3,(uv[0].y+uv[1].y+uv[2].y)/3};
    const Point3D actual=queries.point(center);
    m.valid=finite(actual)&&finite(xyz[0])&&finite(xyz[1])&&finite(xyz[2]);
    m.deviation=length(actual,{(xyz[0].x+xyz[1].x+xyz[2].x)/3,
                              (xyz[0].y+xyz[1].y+xyz[2].y)/3,
                              (xyz[0].z+xyz[1].z+xyz[2].z)/3});
    std::array<Point3D,7> normals;
    normals[6]=queries.normal(center);
    for (int k=0;k<3;++k) {
        const int next=(k+1)%3;
        const Point2D mid{0.5*(uv[k].x+uv[next].x),0.5*(uv[k].y+uv[next].y)};
        const Point3D p=queries.point(mid);
        m.valid=m.valid&&finite(p);
        m.deviation=std::max(m.deviation,length(p,{0.5*(xyz[k].x+xyz[next].x),
                                                   0.5*(xyz[k].y+xyz[next].y),
                                                   0.5*(xyz[k].z+xyz[next].z)}));
        normals[k]=queries.normal(uv[k]);
        normals[k+3]=queries.normal(mid);
    }
    for (size_t i=0;i<normals.size();++i) for (size_t j=i+1;j<normals.size();++j)
        m.normalAngle=std::max(m.normalAngle,angleDeg(normals[i],normals[j]));
    for (const Point3D normal:normals) m.valid=m.valid&&finite(normal);
    Point3D facetNormal=cross3(minus3(xyz[1],xyz[0]),minus3(xyz[2],xyz[0]));
    if (queries.reversed()) facetNormal={-facetNormal.x,-facetNormal.y,-facetNormal.z};
    // 角度准则比较 CAD 法向，三角面片方向另行检查；方向正确的薄片无需满足额外的未定义角度准则。
    m.inverted=finite(normals[6]) && dot3(facetNormal,normals[6])<0;
    return m;
}
struct FaceWork {
    std::vector<Point2D> uv;
    std::vector<int> global;
    std::vector<std::vector<int>> loops;
    std::vector<std::pair<int,int>> constraints;
};
// 已登记的 CAD 节点按逐面 UV 分支匹配；仅追加边界节点，不重排内部布局。
// 局部分边失败时调用方丢弃本次工作图表，恢复原有初始化流程。
bool synchronizeBoundaryChart(Mesh2D& mesh,std::vector<int>& global,FaceWork& work,
    std::vector<std::pair<int,int>> constraints,std::vector<int>& inserted,
    std::string* error,MeshObservation observation) {
    const auto fail=[&](const char* why){if(error)*error=why;return false;};
    if(global.size()!=mesh.points.size())return fail("cached chart/global sizes differ");
    std::unordered_map<int,std::vector<int>> oldNodes;
    std::set<int> boundary;for(auto e:constraints){boundary.insert(e.first);boundary.insert(e.second);}
    for(int v:boundary)if(global[v]>=0)oldNodes[global[v]].push_back(v);
    std::vector<int> local(work.uv.size(),-1);
    for(int i=0;i<int(local.size());++i){const auto found=oldNodes.find(work.global[i]);
        if(found==oldNodes.end())continue;
        for(int v:found->second)if(dist2D(mesh.points[v],work.uv[i])<=1e-12){local[i]=v;break;}
        if(local[i]<0)return fail("existing boundary UV branch changed");
    }
    SurfaceDomainState domain;if(!domain.initialize(mesh,constraints,error,observation))return false;
    for(const auto& loop:work.loops){const int n=int(loop.size());int first=-1;
        for(int i=0;i<n;++i)if(local[loop[i]]>=0){first=i;break;}
        if(first<0)return fail("new trim loop has no cached anchor");
        int cursor=0;
        while(cursor<n){const int start=loop[(first+cursor)%n];int next=cursor+1;
            while(next<cursor+n&&local[loop[(first+next)%n]]<0)++next;
            if(next>n)return fail("incomplete cached boundary chain");
            const int end=loop[(first+next)%n];int a=local[start],b=local[end];
            if(a<0||b<0)return fail("missing cached boundary endpoint");
            for(int j=cursor+1;j<next;++j){const int i=loop[(first+j)%n];int v=-1;
                if(!trySplitSurfaceBoundaryEdge(mesh,a,b,work.uv[i],constraints,domain,&v,nullptr,error,observation))return false;
                local[i]=v;global.push_back(work.global[i]);inserted.push_back(v);a=v;
            }
            cursor=next;
        }
    }
    std::vector<std::pair<int,int>> updated;
    for(auto e:work.constraints){const int a=local[e.first-3],b=local[e.second-3];
        if(a<0||b<0||!mesh.hasEdgeAny(a,b))return fail("new CAD constraint is absent after local split");
        updated.push_back({a,b});}
    std::set<uint64_t> oldKeys,newKeys;
    for(auto e:constraints)oldKeys.insert(key(e.first,e.second));
    for(auto e:updated)newKeys.insert(key(e.first,e.second));
    if(oldKeys!=newKeys)return fail("local split boundary differs from current CAD trim");
    if(!domain.verify(mesh,updated,error,observation))return false;
    work.constraints=std::move(updated);return true;
}
using FaceMetric=SurfaceMetric;

// CDT 提供约束拓扑，完成细化后再做物理质量改善；边界及极点固定，局部修改保留硬几何要求。
/// 在固定边界和几何约束下尝试局部换边及节点平滑。
bool improvePhysicalQuality(Mesh2D& mesh,const std::vector<int>& global,
    std::vector<Point3D>& xyz,FaceEvaluationCache& queries,const GeometrySource& geo,int face,
    const std::vector<std::pair<int,int>>& constraints,SurfaceDomainState& domain,
    const SurfaceCriteria& criteria,const std::function<double(Point3D)>& targetAt,
    const std::function<double(double)>& edgeLimit,MeshingProgress progress,int total,
    int& steps,int& flips,int& moves,std::string* error,MeshObservation observation,bool physicalShapeTargets=false,
    bool qualityPenaltyTargets=false,bool distributionTargets=false,const SurfaceSizingPolicy* sizing=nullptr) {
    std::unordered_set<uint64_t> blocked;
    std::unordered_set<int> fixed;
    for(auto e:constraints) { blocked.insert(key(e.first,e.second)); fixed.insert(e.first); fixed.insert(e.second); }
    const auto physical=[&](int v) {return xyz[global[v]];};
    const auto shapeScore=[&](const std::vector<std::array<int,3>>& triangles,int moved=-1,Point3D p={}){
        SurfaceShapeScore score;
        for(auto t:triangles){std::array<Point3D,3> x;for(int k=0;k<3;++k)x[k]=t[k]==moved?p:physical(t[k]);
            score.add(ratio(x[0],x[1],x[2]),surfaceShapeQuality(x[0],x[1],x[2]));}
        return score;};
    const auto metrics=[&](std::array<int,3> t,int moved=-1,Point2D uv={},Point3D p={}) {
        std::array<Point2D,3> u; std::array<Point3D,3> x;
        for(int k=0;k<3;++k) { u[k]=t[k]==moved?uv:mesh.points[t[k]]; x[k]=t[k]==moved?p:physical(t[k]); }
        return measureTriangle(queries,u,x);
    };
    const auto valid=[&](const TriangleMetrics& m,const std::array<int,3>& t,int moved=-1,Point3D p={}) {
        double h=std::numeric_limits<double>::infinity(); Point3D center{};
        for(int v:t) { auto x=v==moved?p:physical(v); h=std::min(h,targetAt(x)*criteria.growthRatio);
            center.x+=x.x/3; center.y+=x.y/3; center.z+=x.z/3; }
        h=std::min(h,targetAt(center));
        return m.valid && !m.inverted && std::isfinite(m.quality) &&
            m.maxEdge<=edgeLimit(h) && m.deviation<=criteria.maxDeviation && m.normalAngle<=criteria.maxNormalAngleDeg;
    };
    const auto charge=[&] {
        if(++steps>criteria.maxIterations) { if(error)*error="physical quality optimization work budget exceeded on face "+std::to_string(face); return false; }
        return true;
    };
    // 局部最差质量必须严格改善，以防换边往返；达到限定扫描次数只报告优化上限，不宣称达到质量目标。
    bool changed=false,balancePhase=false;
    std::map<int,std::vector<std::pair<int,std::uint64_t>>> failedNeighborhoods;
    const int sweepLimit=distributionTargets?criteria.shapeOptimizationSweeps:40;
    for(int sweep=0;sweep<sweepLimit;++sweep) {
        const bool shapeSweep=distributionTargets&&sweep<criteria.shapeOptimizationSweeps;
        changed=false;
        bool pendingBalance=false;
        if(progress && !progress(face,total)) { if(error)*error="surface meshing cancelled during physical quality optimization"; return false; }
        std::vector<std::pair<int,int>> edges;
        for(const auto& t:mesh.tris) for(int k=0;k<3;++k) if(t.v[k]<t.v[(k+1)%3]) edges.push_back({t.v[k],t.v[(k+1)%3]});
        int checks=0;
        for(auto edge:edges) {
            if(checks++%128==0 && progress && !progress(face,total)) { if(error)*error="surface meshing cancelled during quality flips"; return false; }
            if(blocked.count(key(edge.first,edge.second))) continue;
            int he=mesh.findHalfedge(edge.first,edge.second); if(he<0) continue;
            int t=he/3,k=he%3,other=mesh.tris[t].n[k];
            if(other<0 || !domain.values()[t] || !domain.values()[other]) continue;
            auto tri=mesh.tris[t]; int a=tri.v[k],b=tri.v[(k+1)%3],c=tri.v[(k+2)%3],d=-1;
            for(int v:mesh.tris[other].v) if(v!=a && v!=b)d=v;
            if(d<0 || global[a]<0 || global[b]<0 || global[c]<0 || global[d]<0 || global[c]==global[d] || mesh.hasEdgeAny(c,d)) continue;
            if(orient2d(mesh.points[c],mesh.points[d],mesh.points[a])*orient2d(mesh.points[c],mesh.points[d],mesh.points[b])>=0) continue;
            const double before0=ratio(physical(a),physical(b),physical(c)),before1=ratio(physical(b),physical(a),physical(d));
            const bool alreadyGood=std::max(before0,before1)<=criteria.maxRadiusEdgeRatio;
            if(distributionTargets&&!alreadyGood)continue;
            if(alreadyGood&&!shapeSweep) continue;
            std::array<int,3> x{c,d,a},y{d,c,b};
            for(auto* z:{&x,&y}) if(orient2d(mesh.points[(*z)[0]],mesh.points[(*z)[1]],mesh.points[(*z)[2]])<0) std::swap((*z)[1],(*z)[2]);
            const double after0=ratio(physical(x[0]),physical(x[1]),physical(x[2])),after1=ratio(physical(y[0]),physical(y[1]),physical(y[2]));
            const auto penalty=[&](double value){const double excess=std::max(0.,value-criteria.maxRadiusEdgeRatio);return excess*excess;};
            if(alreadyGood){
                observation.count(MeshCounter::ShapeCandidateTrials,1);
                if(!shapeScore({x,y}).improves(shapeScore({{a,b,c},{b,a,d}})))continue;
                if(sizing && length(physical(c),physical(d))<std::max(sizing->floor(c),sizing->floor(d)))continue;
            }else if(std::max(after0,after1)>=std::max(before0,before1)*(1-1e-6) ||
                (qualityPenaltyTargets?penalty(after0)+penalty(after1)>=penalty(before0)+penalty(before1):
                    after0+after1>=before0+before1)) continue;
            if(!valid(metrics(x),x) || !valid(metrics(y),y)) continue;
            if(!charge()) return false;
            Mesh2DTransaction edit(mesh,observation); SurfaceDomainState::Edit domainEdit(domain);
            mesh.removeTri(std::max(t,other)); mesh.removeTri(std::min(t,other));
            mesh.addTri(x[0],x[1],x[2]); mesh.addTri(y[0],y[1],y[2]);
            MeshEditDelta delta;
            if(!domainEdit.consume(edit.pendingDelta(),error) || !domainEdit.validate(mesh,edit.pendingDelta().changedSlots,error) ||
               !edit.commit(error,&delta,constraints)) return false;
            domainEdit.commit(); ++flips; changed=true;
        }
        std::vector<std::vector<int>> stars(mesh.points.size());
        for(int t=0;t<int(mesh.tris.size());++t) if(domain.values()[t]) for(int v:mesh.tris[t].v) stars[v].push_back(t);
        for(int v=0;v<int(stars.size());++v) {
            if(v%128==0 && progress && !progress(face,total)) { if(error)*error="surface meshing cancelled during quality relocation"; return false; }
            if(fixed.count(v) || stars[v].empty() || global[v]<0) continue;
            bool complete=true;
            for(int t:stars[v])for(int n:mesh.tris[t].v)if(global[n]<0)complete=false;
            if(!complete)continue;
            std::set<int> neighbors; double beforeMax=0,beforeSum=0,beforePenalty=0;
            for(int t:stars[v]) {
                auto tri=mesh.tris[t]; double q=ratio(physical(tri.v[0]),physical(tri.v[1]),physical(tri.v[2]));
                beforeMax=std::max(beforeMax,q); beforeSum+=q;
                const double excess=std::max(0.,q-criteria.maxRadiusEdgeRatio);beforePenalty+=excess*excess;
                for(int n:tri.v) if(n!=v) neighbors.insert(n);
            }
            const bool alreadyGood=beforeMax<=criteria.maxRadiusEdgeRatio;
            if(distributionTargets&&!alreadyGood)continue;
            if((alreadyGood&&!shapeSweep)||!std::isfinite(beforeMax)) continue;
            std::vector<std::pair<int,std::uint64_t>> versions;
            std::vector<std::array<int,3>> starTriangles;
            for(int t:stars[v]){versions.push_back({t,mesh.triangleVersion(t)});auto tri=mesh.tris[t];starTriangles.push_back({tri.v[0],tri.v[1],tri.v[2]});}
            const auto failed=failedNeighborhoods.find(v);
            if(failed!=failedNeighborhoods.end()&&failed->second==versions){observation.count(MeshCounter::ShapeReusedNeighborhoods,1);continue;}
            const auto beforeShape=shapeScore(starTriangles);
            Point2D average{}; Point3D displacement{}; auto p=physical(v);
            for(int n:neighbors) {
                average.x+=mesh.points[n].x/neighbors.size(); average.y+=mesh.points[n].y/neighbors.size();
                auto q=minus3(physical(n),p); displacement.x+=q.x/neighbors.size(); displacement.y+=q.y/neighbors.size(); displacement.z+=q.z/neighbors.size();
            }
            Point3D du,dv; geo.faceDerivatives(face,mesh.points[v],du,dv);
            const double uu=dot3(du,du),uv=dot3(du,dv),vv=dot3(dv,dv),det=uu*vv-uv*uv;
            std::vector<Point2D> targets{average};
            struct BalanceTarget{SurfaceTangentChart chart;Point2D current,target;};
            std::vector<BalanceTarget> boundaryTargets;
            // 切向位移通过该节点的局部度量映射；真正奇异的位置只使用 UV 邻域候选。
            if(finite(du)&&finite(dv)&&det>0) targets.push_back({mesh.points[v].x+(dot3(displacement,du)*vv-dot3(displacement,dv)*uv)/det,
                mesh.points[v].y+(dot3(displacement,dv)*uu-dot3(displacement,du)*uv)/det});
            if(physicalShapeTargets)for(int t:stars[v]){
                const auto tri=mesh.tris[t];int a=-1,b=-1;for(int n:tri.v)if(n!=v){if(a<0)a=n;else b=n;}
                if(ratio(p,physical(a),physical(b))<=criteria.maxRadiusEdgeRatio)continue;
                SurfaceTangentChart chart;std::string chartError;
                if(!surfaceTangentChartAt(queries,face,{mesh.points[v],mesh.points[a],mesh.points[b]},
                    {p,physical(a),physical(b)},chart,&chartError))continue;
                const auto pa=chart.project(physical(a)),pb=chart.project(physical(b)),pv=chart.project(p);
                const double dx=pb.x-pa.x,dy=pb.y-pa.y;
                const double side=dx*(pv.y-pa.y)-dy*(pv.x-pa.x)>0?1.:-1.;
                const Point2D apex{(pa.x+pb.x)/2-side*std::sqrt(3.)*dy/2,(pa.y+pb.y)/2+side*std::sqrt(3.)*dx/2};
                if(blocked.count(key(a,b))){
                    // 贴边坏片先平衡两腰而保持高度；等边目标的横向位移可能破坏其余邻片。
                    pendingBalance=true;
                    if(balancePhase){const double baseSquared=dx*dx+dy*dy;
                        if(baseSquared>0){const double shift=(((pa.x+pb.x)/2-pv.x)*dx+((pa.y+pb.y)/2-pv.y)*dy)/baseSquared;
                            boundaryTargets.push_back({chart,pv,{pv.x+shift*dx,pv.y+shift*dy}});}}
                }
                // 完整等边目标可能越出局部窗口，先缩短物理位移再反解；不夹 UV。
                for(double alpha:{1.,.5,.25,.125}){
                    const auto inverse=invertSurfaceTangentChart(queries,face,chart,
                        {pv.x+alpha*(apex.x-pv.x),pv.y+alpha*(apex.y-pv.y)});
                    if(inverse.valid)targets.push_back(inverse.uv);
                }
            }
            double bestMax=beforeMax,bestSum=beforeSum,bestPenalty=beforePenalty; Point2D best=mesh.points[v]; Point3D bestPoint=p;
            auto bestShape=beforeShape;
            std::set<std::pair<double,double>> evaluated;
            const auto evaluateTargets=[&](const std::vector<Point2D>& candidates){
            for(auto target:candidates) for(double alpha:{1.0,0.5,0.25}) {
                Point2D candidate{mesh.points[v].x+alpha*(target.x-mesh.points[v].x),mesh.points[v].y+alpha*(target.y-mesh.points[v].y)};
                if(!evaluated.emplace(candidate.x,candidate.y).second){observation.count(MeshCounter::QualityReusedFailures,1);continue;}
                observation.count(MeshCounter::ShapeCandidateTrials,1);
                bool oriented=true;
                for(int t:stars[v]) {
                    auto tri=mesh.tris[t]; std::array<Point2D,3> u;
                    for(int k=0;k<3;++k) u[k]=tri.v[k]==v?candidate:mesh.points[tri.v[k]];
                    if(orient2d(u[0],u[1],u[2])<=0) {oriented=false;break;}
                }
                if(!oriented || mesh.exactPoint(candidate)>=0) continue;
                auto q=queries.point(candidate); if(!finite(q)) continue;
                if(alreadyGood&&sizing){bool belowFloor=false;
                    for(int n:neighbors)if(length(q,physical(n))<std::min(length(p,physical(n)),std::max(sizing->floor(v),sizing->floor(n)))){belowFloor=true;break;}
                    if(belowFloor){observation.count(MeshCounter::ShapeRejectedFloor,1);continue;}}
                double nextMax=0,nextSum=0,nextPenalty=0; bool acceptable=true;
                for(int t:stars[v]) {
                    auto tri=mesh.tris[t]; std::array<Point3D,3> p;
                    for(int k=0;k<3;++k)p[k]=tri.v[k]==v?q:physical(tri.v[k]);
                    double quality=ratio(p[0],p[1],p[2]); nextMax=std::max(nextMax,quality);nextSum+=quality;
                    const double excess=std::max(0.,quality-criteria.maxRadiusEdgeRatio);nextPenalty+=excess*excess;
                }
                // 后置形状优化若令整个一环达标，可接受总和略升；仍严格降低最差值。
                // 总和只是辅助目标，不能阻止消除最后一个不达标片。
                const bool satisfiesShape=physicalShapeTargets && nextMax<=criteria.maxRadiusEdgeRatio;
                const auto nextShape=alreadyGood?shapeScore(starTriangles,v,q):SurfaceShapeScore{};
                if(alreadyGood){if(nextMax>criteria.maxRadiusEdgeRatio||!nextShape.improves(bestShape))continue;}
                else if(!std::isfinite(nextMax) || nextMax>=bestMax*(1-1e-6) ||
                    (qualityPenaltyTargets?nextPenalty>=bestPenalty*(1-1e-6):(!satisfiesShape && nextSum>=bestSum))) continue;
                for(int t:stars[v]) { auto tri=mesh.tris[t]; auto m=metrics({tri.v[0],tri.v[1],tri.v[2]},v,candidate,q);
                    if(!valid(m,{tri.v[0],tri.v[1],tri.v[2]},v,q)) {acceptable=false;break;}
                }
                if(acceptable) {bestMax=nextMax;bestSum=nextSum;bestPenalty=nextPenalty;best=candidate;bestPoint=q;bestShape=nextShape;}
            }
            };
            evaluateTargets(targets);
            // 原有候选已能令整个一环达标时，避免额外 CAD 反解和测量。
            if(bestMax>criteria.maxRadiusEdgeRatio && !boundaryTargets.empty()){
                targets.clear();
                for(const auto& target:boundaryTargets)for(double alpha:{1.,.5,.25,.125}){
                    const auto inverse=invertSurfaceTangentChart(queries,face,target.chart,
                        {target.current.x+alpha*(target.target.x-target.current.x),target.current.y+alpha*(target.target.y-target.current.y)});
                    if(inverse.valid)targets.push_back(inverse.uv);
                }
                evaluateTargets(targets);
            }
            if(!alreadyGood&&qualityPenaltyTargets&&bestMax>=beforeMax*(1-1e-6)){
                // 形状目标停滞时在物理切平面小步搜索；每一步仍验收完整一环。
                const auto tri=mesh.tris[stars[v].front()];std::array<Point2D,3> u;std::array<Point3D,3> x;
                for(int k=0;k<3;++k){u[k]=mesh.points[tri.v[k]];x[k]=physical(tri.v[k]);}
                SurfaceTangentChart chart;std::string chartError;
                if(surfaceTangentChartAt(queries,face,u,x,chart,&chartError)){
                    double scale=std::numeric_limits<double>::infinity();for(int n:neighbors)scale=std::min(scale,length(p,physical(n)));
                    const auto current=chart.project(p);targets.clear();
                    for(auto direction:std::vector<Point2D>{{1,0},{-1,0},{0,1},{0,-1},{1,1},{-1,1},{1,-1},{-1,-1}})
                        for(double step:{.25,.125}){const double offset=step*scale/std::hypot(direction.x,direction.y);
                            const auto inverse=invertSurfaceTangentChart(queries,face,chart,
                                {current.x+offset*direction.x,current.y+offset*direction.y});
                            if(inverse.valid)targets.push_back(inverse.uv);}
                    evaluateTargets(targets);
                }
            }
            if(best.x!=mesh.points[v].x || best.y!=mesh.points[v].y) {
                if(!charge())return false;
                // 发布前已检查全部关联面片，拓扑和共享节点身份保持不变；旧测量缓存全部失效。
                if(!mesh.relocatePoint(v,best,stars[v])) {if(error)*error="validated quality relocation could not update point index";return false;}
                xyz[global[v]]=bestPoint; ++moves; changed=true;
                failedNeighborhoods.erase(v);
            }else failedNeighborhoods[v]=std::move(versions);
        }
        if(!changed){
            // 原有形状优化停滞后才启用贴边补充候选，避免提前改变生成路径。
            if(physicalShapeTargets && !balancePhase && pendingBalance){balancePhase=true;continue;}
            return true;
        }
    }
    if(changed && error) *error="physical quality optimization reached "+std::to_string(sweepLimit)+" sweeps on face "+std::to_string(face);
    return true;
}

/// 从约束边及尺寸锚点构造固定边界尺寸来源。
std::vector<std::pair<int,double>> fixedBoundarySeeds(const std::vector<Point3D>& points,
    const std::vector<std::pair<int,int>>& constraints,const std::vector<CadBoundarySizingAnchor>& anchors,double target) {
    std::set<int> nodes;for(auto e:constraints){nodes.insert(e.first);nodes.insert(e.second);}
    std::map<std::tuple<double,double,double>,double> sources;
    for(auto anchor:anchors){auto key=std::make_tuple(anchor.point.x,anchor.point.y,anchor.point.z);
        auto [it,added]=sources.emplace(key,anchor.size);if(!added)it->second=std::min(it->second,anchor.size);}
    std::vector<std::pair<int,double>> seeds;
    // 仅原始边界节点作为尺寸源，传播沿有效网格路径进行。
    for(int v:nodes){auto p=points[v];const auto found=sources.find({p.x,p.y,p.z});
        seeds.push_back({v,found==sources.end()?target:found->second});}
    return seeds;
}

/// 沿三角形邻接定位表面候选点。
int locateSurfaceCandidate(const Mesh2D& mesh,Point2D p,int start,const SurfaceDomainState& domain) {
    int t=start;
    for(size_t i=0;i<=mesh.tris.size();++i){if(t<0 || t>=int(mesh.tris.size()))return -1;
        const auto& tri=mesh.tris[t];int next=-1;
        for(int k=0;k<3;++k)if(orient2d(mesh.points[tri.v[k]],mesh.points[tri.v[(k+1)%3]],p)<0){next=k;break;}
        if(next<0)return domain.values()[t]?t:-1;t=tri.n[next];}
    return -1;
}

/// 由现有面网格和边界约束初始化节点尺寸策略。
bool initializeSurfaceSizing(SurfaceSizingPolicy& sizing,const Mesh2D& mesh,const std::vector<int>& global,
    const std::vector<Point3D>& xyz,const SurfaceDomainState& domain,const std::vector<std::pair<int,int>>& constraints,
    const std::vector<CadBoundarySizingAnchor>& anchors,const SurfaceCriteria& c,std::string* error) {
    std::vector<Point3D> points(global.size());std::vector<std::array<int,3>> triangles;
    for(int v=0;v<int(global.size());++v)if(global[v]>=0)points[v]=xyz.at(global[v]);
    for(int t=0;t<int(mesh.tris.size());++t)if(domain.values()[t]){auto tri=mesh.tris[t];
        if(global[tri.v[0]]>=0&&global[tri.v[1]]>=0&&global[tri.v[2]]>=0)triangles.push_back({tri.v[0],tri.v[1],tri.v[2]});}
    return sizing.initialize(points,triangles,constraints,anchors,c,error);
}

// 首次CDT的物理邻边尺寸是固定源；质量分点只能继承渐变场，不能重定义更小的源。
bool preservedReferenceSeedSizes(const Mesh2D& mesh,const std::vector<int>& global,const std::vector<Point3D>& xyz,
    const SurfaceDomainState& domain,const std::map<int,double>& sources,const SurfaceCriteria& c,
    std::vector<std::pair<int,double>>& output,std::string* error) {
    const auto fail=[&](const char* why){if(error)*error=why;return false;};
    if(global.size()!=mesh.points.size()||domain.values().size()!=mesh.tris.size()||sources.empty())
        return fail("invalid preserved reference seed mapping");
    std::vector<Point3D> points(global.size());std::vector<std::array<int,3>> triangles;std::set<int> active;
    for(int t=0;t<int(mesh.tris.size());++t)if(domain.values()[t]) {
        const auto tri=mesh.tris[t];triangles.push_back({tri.v[0],tri.v[1],tri.v[2]});
        for(int v:tri.v){if(v<0||v>=int(global.size())||global[v]<0||global[v]>=int(xyz.size()))
            return fail("invalid preserved reference seed vertex");active.insert(v);points[v]=xyz[global[v]];}
    }
    double floor=c.targetSize;std::vector<std::pair<int,double>> seeds;
    for(int v:active){const auto found=sources.find(global[v]);if(found==sources.end())continue;
        if(!(found->second>0)||!std::isfinite(found->second))return fail("invalid preserved reference source size");
        floor=std::min(floor,found->second);seeds.push_back({v,found->second});}
    if(seeds.empty())return fail("preserved reference sources have no active support");
    SurfaceBoundarySizeField field;
    if(!field.initialize(points,triangles,{},c.targetSize,floor,c.growthRatio,error,seeds))return false;
    std::vector<std::pair<int,double>> result;for(int v:active)result.push_back({v,field.at(v)});
    output=std::move(result);return true;
}

// 软质量阶段在硬几何收敛后运行；有限局部候选在插点事务提交前验收。
bool refinePhysicalCandidates(Mesh2D& mesh,std::vector<int>& global,std::vector<Point3D>& xyz,
    SurfaceEvaluationCache& queries,const GeometrySource& geo,int face,const std::vector<std::pair<int,int>>& constraints,
    SurfaceDomainState& domain,const SurfaceCriteria& c,const std::function<double(Point3D)>& targetAt,
    const std::function<double(double)>& edgeLimit,MeshingProgress progress,int& steps,
    SurfaceWorkStatistics& work,std::vector<std::string>& diagnostics,std::string* error,MeshObservation observation,
    const std::vector<CadBoundarySizingAnchor>& anchors,bool cavityQuality=false) {
    std::map<std::tuple<double,double,double>,std::vector<int>> boundaryNodes;
    const auto physical=[&](int v){return xyz.at(global.at(v));};
    for(auto e:constraints)for(int v:{e.first,e.second})if(global[v]>=0){auto p=physical(v);boundaryNodes[{p.x,p.y,p.z}].push_back(v);}
    const double floor=c.minSize;
    int featureEdge=-1;for(const auto& anchor:anchors)if(anchor.size<c.minSize){featureEdge=anchor.cadEdge;break;}
    const auto topology=geo.faceTopology(face);
    bool hasPole=false;for(const auto& loop:topology.loops)for(auto use:loop)hasPole|=geo.edgeDegenerate(use.edge);
    SurfaceQualityPolicy policy(floor,c.maxRadiusEdgeRatio);
    for(const auto& loop:topology.loops)for(size_t i=0;i<loop.size();++i){auto current=loop[i],previous=loop[(i+loop.size()-1)%loop.size()];
        if(geo.edgeDegenerate(current.edge)||geo.edgeDegenerate(previous.edge))continue;
        auto range=geo.edgeRange(current.edge),oldRange=geo.edgeRange(previous.edge);
        auto vertices=geo.edgeVertices(current.edge);int vertex=current.reversed?vertices.second:vertices.first;if(vertex<0)continue;
        const auto p=geo.vertexPoint(vertex);const auto found=boundaryNodes.find({p.x,p.y,p.z});if(found==boundaryNodes.end())continue;
        const double begin=current.reversed?range.second:range.first,end=previous.reversed?oldRange.first:oldRange.second;
        const double next=begin+(current.reversed?-1:1)*(range.second-range.first)*1e-6;
        const double before=end+(previous.reversed?1:-1)*(oldRange.second-oldRange.first)*1e-6;
        const double angle=angleDeg(minus3(geo.edgePoint(current.edge,next),p),minus3(geo.edgePoint(previous.edge,before),p));
        if(!(angle>0 && angle<90))continue;
        // 凸/凹由有效裁剪域判断；小的两棱夹角也可能是凹角的外角。
        const auto a=geo.edgeUV(face,current,begin),b=geo.edgeUV(face,current,next),d=geo.edgeUV(face,previous,before);
        Point2D probe{a.x+(b.x-a.x)+(d.x-a.x),a.y+(b.y-a.y)+(d.y-a.y)};
        for(int local:found->second){int seed=-1;for(int t=0;t<int(mesh.tris.size());++t){const auto& tri=mesh.tris[t];if(tri.v[0]==local||tri.v[1]==local||tri.v[2]==local){seed=t;break;}}
            if(seed>=0 && locateSurfaceCandidate(mesh,probe,seed,domain)>=0)policy.protectCorner(local,p,angle);}
    }
    SurfaceSizingPolicy sizing;
    if(!initializeSurfaceSizing(sizing,mesh,global,xyz,domain,constraints,anchors,c,error))return false;
    // 常规密度仍使用 min；质量下限仅由首次必要几何采样产生，不能由新短边递归降低。
    // 半个原段长给局部候选留余量，原 CAD 段和节点始终锁定。
    std::vector<CadBoundarySizingAnchor> floorAnchors;double smallestFloor=floor;
    for(const auto& anchor:anchors)if(anchor.size<floor){
        auto source=anchor;source.size*=.5;smallestFloor=std::min(smallestFloor,source.size);floorAnchors.push_back(source);}
    const bool relaxed=!floorAnchors.empty();
    if(featureEdge>=0)diagnostics.push_back("face "+std::to_string(face)+" preserves short CAD edge "+std::to_string(featureEdge)+
        "; quality floor graded from fixed geometry samples, minimum="+std::to_string(smallestFloor)+
        "; regular minSize="+std::to_string(floor));
    SurfaceRefinementQueue queue(face,1,observation);std::map<std::array<int,3>,int> rejected;
    int accepted=0,refused=0;
    const auto vertices=[&](int t){const auto& tri=mesh.tris[t];return std::array<int,3>{tri.v[0],tri.v[1],tri.v[2]};};
    const auto signature=[](std::array<int,3> ids){std::sort(ids.begin(),ids.end());return ids;};
    const auto metricFor=[&](std::array<int,3> ids){return measureTriangle(queries,{mesh.points[ids[0]],mesh.points[ids[1]],mesh.points[ids[2]]},
        {physical(ids[0]),physical(ids[1]),physical(ids[2])});};
    const auto offer=[&](int t){if(t<0||t>=int(mesh.tris.size())||!domain.values()[t])return true;auto ids=vertices(t);
        if(global[ids[0]]<0||global[ids[1]]<0||global[ids[2]]<0||global[ids[0]]==global[ids[1]]||global[ids[1]]==global[ids[2]]||global[ids[2]]==global[ids[0]])return true;
        auto key=signature(ids);const auto failed=rejected.find(key);
        if(failed!=rejected.end()&&failed->second==accepted){observation.count(MeshCounter::QualityReusedFailures,1);return true;}
        const auto a=physical(ids[0]),b=physical(ids[1]),d=physical(ids[2]);
        const double shortest=std::min({length(a,b),length(b,d),length(d,a)}),quality=ratio(a,b,d);
        const double localFloor=sizing.candidateFloor(ids);
        if(quality<=c.maxRadiusEdgeRatio || policy.decision(ids,shortest,localFloor)!=SurfaceQualityReason::Eligible)return true;
        auto m=metricFor(ids);
        if(!m.valid || m.inverted){if(error)*error="invalid facet before physical refinement";return false;}
        if(m.quality<=c.maxRadiusEdgeRatio || policy.decision(ids,m.minEdge,localFloor)!=SurfaceQualityReason::Eligible)return true;
        return queue.offer({face,t,mesh.triangleVersion(t),1,1,m.quality/c.maxRadiusEdgeRatio,key},error);};
    for(int t=0;t<int(mesh.tris.size());++t)if(!offer(t))return false;
    SurfaceTriangleTicket ticket;
    while(queue.pop(mesh,ticket)){
        if(progress && !progress(face,geo.faceCount())){if(error)*error="surface meshing cancelled during local quality candidates";return false;}
        if(steps>=c.maxIterations || xyz.size()>=size_t(c.maxVertices)){diagnostics.push_back("face "+std::to_string(face)+" quality candidate budget exhausted");break;}
        const int t=ticket.slot;auto ids=vertices(t);const auto old=metricFor(ids);
        std::array<Point2D,3> uv{mesh.points[ids[0]],mesh.points[ids[1]],mesh.points[ids[2]]};
        SurfaceMetric metric;const Point2D center{(uv[0].x+uv[1].x+uv[2].x)/3,(uv[0].y+uv[1].y+uv[2].y)/3};
        std::string candidateError;
        if(!surfaceMetricAt(queries,face,center,metric,&candidateError)){
            if(refused<8)diagnostics.push_back("face "+std::to_string(face)+" quality candidate metric unavailable near UV ("+
                std::to_string(center.x)+","+std::to_string(center.y)+"): "+candidateError);
            observation.count(MeshCounter::QualityRejectedMetric,1);
            rejected[ticket.vertexKey]=accepted;++refused;continue;}
        const double localFloor=sizing.candidateFloor(ids);
        const double localTarget=std::min(sizing.candidateTarget(ids),targetAt(queries.point(center)));
        const double regularTarget=std::min(sizing.regularCandidateTarget(ids),targetAt(queries.point(center)));
        auto candidates=surfaceRefinementCandidates(uv,metric,regularTarget,localFloor);
        // 先保留已验证的宽面尺度；受阻时再尝试与固定短边/窄域协调的尺度。
        // 不用更密的候选提前替换已有可行布局。
        if(localTarget<regularTarget*(1-1e-6)){
            const auto local=surfaceRefinementCandidates(uv,metric,localTarget,localFloor);
            candidates.insert(candidates.end(),local.begin(),local.end());}
        const bool compareCandidates=cavityQuality&&relaxed;
        if(compareCandidates){
            int shortest=0;for(int k=1;k<3;++k)if(length(physical(ids[k]),physical(ids[(k+1)%3]))<
                length(physical(ids[shortest]),physical(ids[(shortest+1)%3])))shortest=k;
            const auto a=uv[shortest],b=uv[(shortest+1)%3],opposite=uv[(shortest+2)%3];
            // 原偏心点/圆心受阻后，有限内部候选允许邻片一起改善；每次仍事务验收。
            for(double weight:{.4,.2,1./3,.6,.8})for(double along:{.5,.25,.75})
                candidates.push_back({(1-weight)*((1-along)*a.x+along*b.x)+weight*opposite.x,
                    (1-weight)*((1-along)*a.y+along*b.y)+weight*opposite.y});
        }
        if(hasPole){
            const auto tangent=surfaceTangentRefinementCandidates(queries,face,uv,
                {physical(ids[0]),physical(ids[1]),physical(ids[2])},localTarget);
            candidates.insert(candidates.end(),tangent.uv.begin(),tangent.uv.end());
            if(refused<8)for(const auto& why:tangent.refusals)diagnostics.push_back("face "+std::to_string(face)+
                " tangent candidate inverse refused near UV ("+std::to_string(center.x)+","+std::to_string(center.y)+"): "+why);
        }
        bool inserted=false;MeshEditDelta delta;
        Point2D bestCandidate{};double bestGain=0;bool foundCandidate=false;
        std::set<std::pair<double,double>> attempted;
        for(size_t index=0;index<candidates.size()+(compareCandidates?1:0);++index){
            const bool compare=compareCandidates&&index<candidates.size();
            if(index==candidates.size()&&!foundCandidate)break;
            const auto candidate=index<candidates.size()?candidates[index]:bestCandidate;
            if(index<candidates.size()&&!attempted.emplace(candidate.x,candidate.y).second){
                observation.count(MeshCounter::QualityReusedFailures,1);continue;}
            if(steps>=c.maxIterations)break;++steps;
            observation.count(MeshCounter::QualityCandidateTrials,1);
            const int seed=locateSurfaceCandidate(mesh,candidate,t,domain);
            if(seed<0){candidateError="outside trimmed domain";observation.count(MeshCounter::QualityRejectedDomain,1);continue;}
            if(mesh.exactPoint(candidate)>=0){candidateError="duplicate UV point";observation.count(MeshCounter::QualityRejectedDuplicate,1);continue;}
            const auto p=queries.point(candidate);if(!finite(p)){candidateError="non-finite CAD point";observation.count(MeshCounter::QualityRejectedGeometry,1);continue;}
            std::vector<std::pair<int,double>> floorNeighbors;for(int v:ids)floorNeighbors.push_back({v,length(p,physical(v))});
            const double insertionFloor=sizing.insertionFloor(floorNeighbors);
            const LinearMetric2D linear{metric.xx,metric.xy,metric.yy};int vertex=-1,newTri=-1;
            std::string vetoReason;
            MeshCounter rejection=MeshCounter::QualityRejectedTopology;
            const auto accept=[&](const Mesh2D& edited,int v,const MeshEditDelta& change){bool created=false;
                const auto veto=[&](const char* why){vetoReason=why;
                    const std::string reason=why;
                    rejection=reason.find("floor")!=std::string::npos?MeshCounter::QualityRejectedFloor:
                        reason.find("does not improve")!=std::string::npos?MeshCounter::QualityRejectedShape:
                        reason.find("retained for cavity comparison")!=std::string::npos?MeshCounter::QualityCandidateComparisons:
                        MeshCounter::QualityRejectedGeometry;
                    return false;};
                double previousWorst=0,nextWorst=0,previousPenalty=0,nextPenalty=0;int previousBad=0;
                if(cavityQuality)for(auto removed:change.removedTriangles){
                    for(int id:removed)if(id<0||id>=int(global.size())||global[id]<0)return veto("unmapped original physical cavity");
                    const double q=ratio(physical(removed[0]),physical(removed[1]),physical(removed[2]));
                    if(!std::isfinite(q))return veto("invalid original physical cavity");
                    previousWorst=std::max(previousWorst,q);previousBad+=q>c.maxRadiusEdgeRatio;
                    const double excess=std::max(0.,q-c.maxRadiusEdgeRatio);previousPenalty+=excess*excess;}
                for(int slot:change.changedSlots){if(slot<0||slot>=int(edited.tris.size()))continue;const auto& tri=edited.tris[slot];
                    if(tri.v[0]!=v && tri.v[1]!=v && tri.v[2]!=v)continue;created=true;
                    std::array<Point2D,3> u;std::array<Point3D,3> x;
                    for(int k=0;k<3;++k){u[k]=edited.points[tri.v[k]];if(tri.v[k]==v)x[k]=p;
                        else if(tri.v[k]>=int(global.size())||global[tri.v[k]]<0)return false;else x[k]=physical(tri.v[k]);}
                    // 已知新增边不满足下限时先拒绝，避免无效候选的曲面/法向采样。
                    for(int k=0;k<3;++k)if(tri.v[k]!=v && length(p,x[k])<insertionFloor)return veto("quality-only insertion below local geometry floor");
                    if(!cavityQuality&&ratio(x[0],x[1],x[2])>=old.quality*(1-1e-6))return veto("local quality does not improve");
                    auto m=measureTriangle(queries,u,x);double h=targetAt(p);for(auto q:x)h=std::min(h,targetAt(q)*c.growthRatio);
                    if(!m.valid || m.inverted)return veto("invalid or inverted physical facet");
                    if(m.maxEdge>edgeLimit(h))return veto("maximum size criterion");
                    if(m.deviation>c.maxDeviation)return veto("sampled geometric deviation criterion");
                    if(m.normalAngle>c.maxNormalAngleDeg)return veto("normal angle criterion");
                    if(!cavityQuality&&m.quality>=old.quality*(1-1e-6))return veto("local quality does not improve");
                    nextWorst=std::max(nextWorst,m.quality);const double excess=std::max(0.,m.quality-c.maxRadiusEdgeRatio);
                    nextPenalty+=excess*excess;
                }
                // 最差值与超标程度平方和均下降；暂时分散差片只能在最终整体质量不恶化时保留。
                if(cavityQuality&&(!created||previousBad==0||nextWorst>=previousWorst*(1-1e-6)||
                    nextPenalty>=previousPenalty*(1-1e-6)))return veto("physical cavity quality does not improve");
                if(compare){const double gain=previousPenalty-nextPenalty;
                    if(!foundCandidate||gain>bestGain){bestGain=gain;bestCandidate=candidate;foundCandidate=true;}
                    return veto("feasible candidate retained for cavity comparison");}
                return created;};
            if(!insertPoint(mesh,candidate,constraints,seed,&vertex,&newTri,&candidateError,&linear,observation,&delta,&domain,accept)){
                observation.count(rejection,1);
                if(!vetoReason.empty())candidateError=vetoReason;continue;}
            if(!domain.apply(mesh,delta,error))return false;
            std::map<int,double> sizeNeighbors;
            for(int slot:delta.changedSlots)if(slot>=0 && slot<int(mesh.tris.size())){const auto& tri=mesh.tris[slot];
                if(tri.v[0]!=vertex && tri.v[1]!=vertex && tri.v[2]!=vertex)continue;
                for(int neighbor:tri.v)if(neighbor!=vertex)sizeNeighbors[neighbor]=length(p,physical(neighbor));}
            if(!sizing.append({sizeNeighbors.begin(),sizeNeighbors.end()},error))return false;
            global.resize(mesh.points.size(),-1);global[vertex]=int(xyz.size());xyz.push_back(p);
            ++accepted;++work.insertedSurfacePoints;inserted=true;break;
        }
        if(!inserted){if(refused<8)diagnostics.push_back("face "+std::to_string(face)+" unresolved quality near UV ("+
            std::to_string(center.x)+","+std::to_string(center.y)+"); last candidate rejection: "+
            (candidateError.empty()?"no finite local candidate":candidateError));
            rejected[ticket.vertexKey]=accepted;++refused;continue;}
        queue.invalidate(delta);std::set<int> affected;
        for(int slot:delta.changedSlots)if(slot>=0 && slot<int(mesh.tris.size())){affected.insert(slot);for(int neighbor:mesh.tris[slot].n)if(neighbor>=0)affected.insert(neighbor);}
        for(int slot:affected)if(!offer(slot))return false;
    }
    if(accepted || refused)diagnostics.push_back("face "+std::to_string(face)+" local quality candidates: "+std::to_string(accepted)+
        " accepted, "+std::to_string(refused)+" unresolved source facets (not a proof of unavoidable geometry)");
    return accepted==0 || domain.verify(mesh,constraints,error,observation);
}

/// 将局部曲面度量交给二维插点，并保留约束及域标签。
bool insertMetricPoint(Mesh2D& mesh,Point2D uv,const FaceMetric& metric,
                       const std::vector<std::pair<int,int>>& constraints,int start,
                       int* vertex,int* triangle,std::string* error,MeshObservation observation,MeshEditDelta* delta=nullptr,
                       const SurfaceDomainState* domain=nullptr) {
    const LinearMetric2D linear{metric.xx,metric.xy,metric.yy};
    return insertPoint(mesh,uv,constraints,start,vertex,triangle,error,&linear,observation,delta,domain);
}
/// 在内部共享边插点并同步局部拓扑和域状态。
bool splitInteriorEdge(Mesh2D& mesh,int a,int b,int* vertex,std::string* error,MeshObservation observation,MeshEditDelta* delta=nullptr,
                       const SurfaceDomainState* domain=nullptr,const Point2D* splitPoint=nullptr) {
    MeshStageScope scope(observation,MeshStage::TopologyUpdate);
    if (vertex) *vertex=-1;
    std::vector<std::pair<int,int>> adjacent;
    for (int he:{mesh.findHalfedge(a,b),mesh.findHalfedge(b,a)}) if (he>=0) {
        const int t=he/3,k=he%3;
        adjacent.push_back({t,mesh.tris[t].v[(k+2)%3]});
    }
    observation.count(MeshCounter::TriangleVisits,adjacent.size());
    if (adjacent.size()!=2) {
        if (error) *error="refinement edge has no two incident triangles";
        return false;
    }
    if (domain && !domain->uniform({adjacent[0].first,adjacent[1].first},error)) return false;
    const Point2D pa=mesh.points[a],pb=mesh.points[b];
    const Point2D midpoint=splitPoint?*splitPoint:Point2D{pa.x+(pb.x-pa.x)/2,pa.y+(pb.y-pa.y)/2};
    for (auto [t,third]:adjacent) {
        const Point2D p=mesh.points[third];
        const int orientation=orient2d(pa,pb,p);
        if (orient2d(pa,midpoint,p)*orientation<=0 || orient2d(midpoint,pb,p)*orientation<=0) {
            if (error) {
                std::ostringstream why;why<<std::setprecision(17)<<"edge split cannot preserve UV triangle orientation: a=("
                    <<pa.x<<','<<pa.y<<") b=("<<pb.x<<','<<pb.y<<") split=("<<midpoint.x<<','<<midpoint.y
                    <<") third=("<<p.x<<','<<p.y<<")";*error=why.str();
            }
            return false;
        }
    }
    if (mesh.exactPoint(midpoint)>=0) {
        if (error) *error="edge midpoint coincides with an existing UV vertex"; return false;
    }
    Mesh2DTransaction transaction(mesh,observation);
    const int mid=mesh.addPoint(midpoint);
    std::sort(adjacent.begin(),adjacent.end(),[](auto lhs,auto rhs){return lhs.first>rhs.first;});
    for (auto [t,third]:adjacent) mesh.removeTri(t);
    auto add=[&](int x,int y,int z) {
        const int o=orient2d(mesh.points[x],mesh.points[y],mesh.points[z]);
        if (!(o>0 || o<0)) return false;
        if (o<0) std::swap(y,z);
        mesh.addTri(x,y,z);
        return true;
    };
    for (auto [t,third]:adjacent) {
        if (!add(a,mid,third) || !add(mid,b,third)) {
            if (error) *error="edge split produced degenerate triangles";
            return false;
        }
    }
    if (!transaction.commit(error,delta)) return false;
    if (vertex) *vertex=mid;
    return true;
}
/// 在局部物理图表中判定共享边的外接圆关系。
int physicalCircleSign(const Mesh2D& mesh,const std::vector<int>& global,const std::vector<Point3D>& points,
                       const GeometrySource& geometry,int face,int a,int b,int d,int e,std::string* error) {
    std::array<Point2D,4> uv{mesh.points[a],mesh.points[b],mesh.points[d],mesh.points[e]};
    const auto p=[&](int v){return points.at(global.at(v));};
    if(global[a]==global[b]||global[b]==global[d]||global[d]==global[a])return 0;
    std::array<Point3D,4> xyz{p(a),p(b),p(d),p(e)};SurfaceTangentChart chart;
    if(!surfaceTangentQuadChartAt(geometry,face,uv,xyz,chart,error))return 2;
    const auto positive=[&](std::array<int,3> v) {
        if(global[v[0]]==global[v[1]]||global[v[1]]==global[v[2]]||global[v[2]]==global[v[0]])return true;
        if(orient2d(mesh.points[v[0]],mesh.points[v[1]],mesh.points[v[2]])<0)std::swap(v[1],v[2]);
        const auto x=p(v[0]),y=p(v[1]),z=p(v[2]);
        auto normal=cross3(minus3(y,x),minus3(z,x));
        if(geometry.faceReversed(face)){normal.x=-normal.x;normal.y=-normal.y;normal.z=-normal.z;}
        const Point2D center{mesh.points[v[0]].x/3+mesh.points[v[1]].x/3+mesh.points[v[2]].x/3,
            mesh.points[v[0]].y/3+mesh.points[v[1]].y/3+mesh.points[v[2]].y/3};
        const auto cad=geometry.faceNormal(face,center);
        return finite(normal)&&finite(cad)&&dot3(normal,cad)>0;
    };
    if(!positive({d,e,a})||!positive({e,d,b}))return 0;
    return incircle(chart.project(xyz[0]),chart.project(xyz[1]),chart.project(xyz[2]),chart.project(xyz[3]));
}
/// 在保留约束边的前提下恢复局部网格合法性。
bool legalize(Mesh2D& mesh,const std::vector<std::pair<int,int>>& constraints,
              std::string* error,MeshingProgress progress,int face,int total,
              int& steps,int maxSteps,const FaceMetric& metric,int& flips,
              int seedVertex=-1,int seedTriangle=-1,MeshObservation observation={},MeshEditDelta* delta=nullptr,
              SurfaceDomainState* domain=nullptr,const std::function<int(int,int,int,int)>& circleTest={}) {
    MeshStageScope scope(observation,MeshStage::Legalization);
    std::unordered_map<uint64_t,bool> blocked;
    std::vector<std::pair<int,int>> segments;
    if (!expandConstraintEdges(mesh,constraints,segments,error,observation)) return false;
    for (auto e:segments) blocked[key(e.first,e.second)]=true;
    Mesh2DTransaction transaction(mesh,observation);
    std::unique_ptr<SurfaceDomainState::Edit> domainEdit;
    if (domain) domainEdit=std::make_unique<SurfaceDomainState::Edit>(*domain);
    const int limit=static_cast<int>(mesh.tris.size())*30+100;
    std::deque<std::pair<int,int>> pending;
    std::unordered_set<uint64_t> queued;
    const auto enqueue=[&](int a,int b) {
        if (blocked.count(key(a,b))) return;
        if (queued.insert(key(a,b)).second) pending.push_back({a,b});
    };
    if (seedVertex<0) {
        for (const auto& tri:mesh.tris) for (int k=0;k<3;++k)
            enqueue(tri.v[k],tri.v[(k+1)%3]);
    } else {
        // 二分后仅从新点的一环传播 Lawson 翻边；旧 CDT 的其他边仍合法。
        std::vector<int> star{seedTriangle}; std::unordered_set<int> visited;
        while (!star.empty()) {
            const int t=star.back(); star.pop_back();
            if (t<0 || !visited.insert(t).second) continue;
            const auto& tri=mesh.tris[t];
            bool incident=false; for (int v:tri.v) incident|=v==seedVertex;
            if (!incident) continue;
            for (int k=0;k<3;++k) { enqueue(tri.v[k],tri.v[(k+1)%3]); star.push_back(tri.n[k]); }
        }
    }
    int checks=0,localFlips=0;
    while (!pending.empty()) {
        if (checks++%64==0 && progress && !progress(face,total)) {
            if (error) *error="surface meshing cancelled during CDT legalization";
            return false;
        }
        const auto edge=pending.front(); pending.pop_front(); queued.erase(key(edge.first,edge.second));
        int he=mesh.findHalfedge(edge.first,edge.second);
        if (he<0) he=mesh.findHalfedge(edge.second,edge.first);
        if (he<0) continue; // 此前的换边可能已移除这条对角线。
        const int t=he/3,k=he%3;
        const Tri2D tri=mesh.tris[t];
        const int other=tri.n[k];
        if (other<0) continue;
        const int a=tri.v[k],b=tri.v[(k+1)%3],c=tri.v[(k+2)%3];
        int d=-1;
        for (int v:mesh.tris[other].v) if (v!=a && v!=b) { d=v; break; }
        if (d<0) { if (error) *error="invalid flip adjacency"; return false; }
        const auto& pts=mesh.points;
        if (orient2d(pts[c],pts[d],pts[a])*orient2d(pts[c],pts[d],pts[b])>=0) continue;
        const int sign=circleTest?circleTest(a,b,c,d):incircleMetric(pts[a],pts[b],pts[c],pts[d],metric.xx,metric.xy,metric.yy);
        if(sign<-1||sign>1){if(error)*error="invalid physical CDT circle predicate";return false;}
        if(sign<=0)continue;
        if (domain && !domain->uniform({t,other},error)) return false;
        if (++steps>maxSteps) {
            if (error) *error="Delaunay legalization work budget exceeded";
            return false;
        }
        if (++localFlips>limit) {
            if (error) *error="Delaunay legalization flip limit exceeded (flips="+
                std::to_string(localFlips)+", triangles="+std::to_string(mesh.tris.size())+
                ")";
            return false;
        }
        mesh.removeTri(std::max(t,other));
        mesh.removeTri(std::min(t,other));
        int x=c,y=d,z=a;
        if (orient2d(pts[x],pts[y],pts[z])<0) std::swap(y,z);
        mesh.addTri(x,y,z);
        x=d; y=c; z=b;
        if (orient2d(pts[x],pts[y],pts[z])<0) std::swap(y,z);
        mesh.addTri(x,y,z);
        ++flips;
        if (domainEdit && !domainEdit->consume(transaction.pendingDelta(),error)) return false;
        // 只有四边形周边的合法性可能改变；队列保存节点对，因为紧凑删除会改变三角形和半边编号。
        enqueue(c,d); enqueue(c,a); enqueue(d,a); enqueue(d,b); enqueue(c,b);
    }
    if (domainEdit && !domainEdit->validate(mesh,transaction.pendingDelta().changedSlots,error)) return false;
    if (!transaction.commit(error,delta,segments)) return false;
    if (domainEdit) domainEdit->commit();
    return true;
}
/// 识别矩形参数域并尝试构造保留原边界节点的初始网格。
bool rectangularGrid(const GeometrySource& geo,int face,const SurfaceCriteria& c,
                     FaceWork& w,Mesh2D& mesh,std::vector<int>& global,
                     SurfaceMesh3D& surface,bool* edgeRefinement,std::string* error,MeshObservation observation,
                     const std::function<int(Point3D)>& allocate,const std::function<size_t()>& activeVertices) {
    *edgeRefinement=false;
    if (w.loops.size()!=1 || w.loops[0].size()<4) return false;
    double u0=1e300,u1=-1e300,v0=1e300,v1=-1e300;
    for (int id:w.loops[0]) {
        const auto p=w.uv[id];
        u0=std::min(u0,p.x); u1=std::max(u1,p.x);
        v0=std::min(v0,p.y); v1=std::max(v1,p.y);
    }
    const double eps=1e-5*std::max(u1-u0,v1-v0);
    const double pointEps=1e-12*std::max(u1-u0,v1-v0);
    if (!(u1>u0 && v1>v0)) return false;
    std::vector<double> us,vs;
    bool exactAxes=true;
    const auto& loop=w.loops[0];
    for (size_t i=0;i<loop.size();++i) {
        const auto a=w.uv[loop[i]],b=w.uv[loop[(i+1)%loop.size()]];
        const bool horizontal=std::abs(a.y-b.y)<eps &&
            (std::abs(a.y-v0)<eps || std::abs(a.y-v1)<eps);
        const bool vertical=std::abs(a.x-b.x)<eps &&
            (std::abs(a.x-u0)<eps || std::abs(a.x-u1)<eps);
        if (!horizontal && !vertical) return false;
        if ((horizontal && std::abs(a.y-b.y)>pointEps) ||
            (vertical && std::abs(a.x-b.x)>pointEps)) exactAxes=false;
        // 长边裁剪参数的微小横向漂移不生成新网格列；后续条带和交错连接保留原 UV 节点。
        if(horizontal) {us.push_back(a.x);us.push_back(b.x);}
        if(vertical) {vs.push_back(a.y);vs.push_back(b.y);}
    }
    auto unique=[&](std::vector<double>& values) {
        std::sort(values.begin(),values.end());
        values.erase(std::unique(values.begin(),values.end(),
                   [pointEps](double a,double b){return std::abs(a-b)<pointEps;}),values.end());
    };
    unique(us); unique(vs);
    // 退化极点边的两个 UV 端点映射到同一三维节点；可增加图表列，但不能增加额外物理极点。
    if (us.size()==2) {
        const double du=(u1-u0)*1e-4;
        double scale=0;
        for (int i=0;i<3;++i) {
            const double v=v0+(i+0.5)*(v1-v0)/3;
            scale=std::max(scale,length(geo.facePoint(face,{0.5*(u0+u1),v}),
                geo.facePoint(face,{0.5*(u0+u1)+du,v}))/du);
        }
        const int n=int(std::clamp(std::ceil((u1-u0)*scale/(c.targetSize*0.65)),1.0,256.0));
        for (int i=1;i<n;++i) us.push_back(u0+(u1-u0)*i/n);
        unique(us);
    }
    if (vs.size()==2) {
        const double dv=(v1-v0)*1e-4;
        double scale=0;
        for (int i=0;i<3;++i) {
            const double u=u0+(i+0.5)*(u1-u0)/3;
            scale=std::max(scale,length(geo.facePoint(face,{u,0.5*(v0+v1)}),
                geo.facePoint(face,{u,0.5*(v0+v1)+dv}))/dv);
        }
        const int n=int(std::clamp(std::ceil((v1-v0)*scale/(c.targetSize*0.65)),1.0,256.0));
        for (int i=1;i<n;++i) vs.push_back(v0+(v1-v0)*i/n);
        unique(vs);
    }
    if (us.size()<2 || vs.size()<2) return false;
    const int nx=static_cast<int>(us.size()),ny=static_cast<int>(vs.size());
    auto boundaryId=[&](Point2D uv) {
        for (size_t i=0;i<w.uv.size();++i)
            if (dist2D(uv,w.uv[i])<pointEps) return w.global[i];
        int candidate=-1;
        for (int id:loop) {
            const auto p=w.uv[id];
            const bool onSameSide=(std::abs(uv.y-v0)<eps && std::abs(p.y-v0)<eps) ||
                (std::abs(uv.y-v1)<eps && std::abs(p.y-v1)<eps) ||
                (std::abs(uv.y-v0)>=eps && std::abs(uv.y-v1)>=eps &&
                 ((std::abs(uv.x-u0)<eps && std::abs(p.x-u0)<eps) ||
                  (std::abs(uv.x-u1)<eps && std::abs(p.x-u1)<eps)));
            if (onSameSide) {
                if (candidate<0) candidate=w.global[id];
                else if (candidate!=w.global[id]) return -1;
            }
        }
        return candidate;
    };
    bool fullGrid=exactAxes && us.size()*vs.size()<=size_t(c.maxVertices/2);
    for (int j=0;j<ny;++j) for (int i=0;i<nx;++i)
        if ((i==0 || j==0 || i==nx-1 || j==ny-1) &&
            boundaryId({us[i],vs[j]})<0) fullGrid=false;
    if (!fullGrid) {
        std::vector<int> bottom,top,left,right;
        for (int id:loop) {
            const Point2D p=w.uv[id];
            if (std::abs(p.y-v0)<eps) bottom.push_back(id);
            if (std::abs(p.y-v1)<eps) top.push_back(id);
            if (std::abs(p.x-u0)<eps) left.push_back(id);
            if (std::abs(p.x-u1)<eps) right.push_back(id);
        }
        auto sortX=[&](std::vector<int>& v) {
            std::sort(v.begin(),v.end(),[&](int a,int b){return w.uv[a].x<w.uv[b].x;});
        };
        auto sortY=[&](std::vector<int>& v) {
            std::sort(v.begin(),v.end(),[&](int a,int b){return w.uv[a].y<w.uv[b].y;});
        };
        sortX(bottom); sortX(top); sortY(left); sortY(right);
        bool matchingSides=left.size()==right.size() && left.size()>=2 &&
                           bottom.size()>=2 && top.size()>=2;
        if (matchingSides) for (size_t j=0;j<left.size();++j)
            if (std::abs(w.uv[left[j]].y-w.uv[right[j]].y)>=eps)
                matchingSides=false;
        if (matchingSides) {
            if ((left.size()-2)*(us.size()-2)+activeVertices()>size_t(c.maxVertices)) {
                if (error) *error="rectangular strip exceeds vertex budget";
                return false;
            }
            for (size_t i=0;i<w.uv.size();++i) {
                mesh.addPoint(w.uv[i]);
                global.push_back(w.global[i]);
            }
            std::vector<std::vector<int>> rows;
            rows.push_back(bottom);
            for (size_t j=1;j+1<left.size();++j) {
                std::vector<int> row{left[j]};
                const double v=0.5*(w.uv[left[j]].y+w.uv[right[j]].y);
                for (size_t i=1;i+1<us.size();++i) {
                    const Point2D p{us[i],v};
                    const int gid=allocate(geo.facePoint(face,p));
                    row.push_back(mesh.addPoint(p));
                    global.push_back(gid);
                }
                row.push_back(right[j]);
                rows.push_back(std::move(row));
            }
            rows.push_back(top);
            auto add=[&](int a,int b,int d) {
                const int o=orient2d(mesh.points[a],mesh.points[b],mesh.points[d]);
                if (!(o>0 || o<0)) return false;
                if (o<0) std::swap(b,d);
                mesh.addTri(a,b,d);
                return true;
            };
            bool valid=true;
            for (size_t row=0;row+1<rows.size() && valid;++row) {
                const auto& lower=rows[row];
                const auto& upper=rows[row+1];
                size_t i=0,j=0;
                while (i+1<lower.size() || j+1<upper.size()) {
                    if (j+1==upper.size() ||
                        (i+1<lower.size() && mesh.points[lower[i+1]].x<=mesh.points[upper[j+1]].x)) {
                        valid=add(lower[i],lower[i+1],upper[j]);
                        ++i;
                    } else {
                        valid=add(lower[i],upper[j+1],upper[j]);
                        ++j;
                    }
                    if (!valid) break;
                }
            }
            if (!valid || !mesh.validate(error,observation)) {
                if (error && error->empty()) *error="rectangular strip triangulation failed";
                return false;
            }
            w.constraints.clear();
            for (size_t i=0;i<loop.size();++i)
                w.constraints.push_back({loop[i],loop[(i+1)%loop.size()]});
            *edgeRefinement=true;
            return true;
        }
        // 对边分段不同也采用交错连接，并细分相关初始片以保留上下边采样；中心扇可能需要大量换边，度量变换后的凸包也可能遗漏近共线原 UV 子段。
        for (size_t i=0;i<w.uv.size();++i) {
            mesh.addPoint(w.uv[i]); global.push_back(w.global[i]);
        }
        auto add=[&](int a,int b,int d) {
            const int orientation=orient2d(mesh.points[a],mesh.points[b],mesh.points[d]);
            if (!orientation) return false;
            if (orientation<0) std::swap(b,d);
            mesh.addTri(a,b,d); return true;
        };
        if (left.size()<2 || right.size()<2 || bottom.size()<2 || top.size()<2) {
            if (error) *error="incomplete rectangular side chains";
            return false;
        }
        size_t i=0,j=0;
        while (i+1<left.size() || j+1<right.size()) {
            bool valid;
            if (j+1==right.size() || (i+1<left.size() &&
                w.uv[left[i+1]].y<=w.uv[right[j+1]].y)) {
                valid=add(left[i],right[j],left[i+1]); ++i;
            } else {
                valid=add(left[i],right[j],right[j+1]); ++j;
            }
            if (!valid) { if (error) *error="collapsed rectangular zipper triangle"; return false; }
        }
        for (const auto& side:{bottom,top}) if (side.size()>2) {
            int he=mesh.findHalfedge(side.front(),side.back());
            if (he<0) he=mesh.findHalfedge(side.back(),side.front());
            if (he<0) { if (error) *error="rectangular zipper cap edge missing"; return false; }
            const int third=mesh.tris[he/3].v[(he%3+2)%3];
            mesh.removeTri(he/3);
            for (size_t k=0;k+1<side.size();++k) if (!add(side[k],side[k+1],third)) {
                if (error) *error="collapsed rectangular zipper cap"; return false;
            }
        }
        w.constraints.clear();
        for (size_t k=0;k<loop.size();++k)
            w.constraints.push_back({loop[k],loop[(k+1)%loop.size()]});
        *edgeRefinement=true;
        return mesh.validate(error,observation);
    }
    const size_t newVertices=size_t(nx-2)*size_t(ny-2);
    if (activeVertices()+newVertices>size_t(c.maxVertices)) {
        if (error) *error="rectangular chart exceeds vertex budget (columns="+std::to_string(nx)+", rows="+std::to_string(ny)+
            ", new="+std::to_string(newVertices)+", active="+std::to_string(activeVertices())+")";
        return false;
    }
    std::vector<int> grid(nx*ny,-1);
    for (int j=0;j<ny;++j) for (int i=0;i<nx;++i) {
        const Point2D uv{us[i],vs[j]};
        int gid=-1;
        if (i==0 || j==0 || i==nx-1 || j==ny-1) gid=boundaryId(uv);
        else { gid=allocate(geo.facePoint(face,uv)); }
        if (gid<0) { if (error) *error="rectangular chart boundary mismatch"; return false; }
        grid[j*nx+i]=mesh.addPoint(uv); global.push_back(gid);
    }
    for (int j=0;j<ny-1;++j) for (int i=0;i<nx-1;++i) {
        const int a=grid[j*nx+i],b=grid[j*nx+i+1];
        const int d=grid[(j+1)*nx+i],cc=grid[(j+1)*nx+i+1];
        // 矩形单元仅用两个初始三角形；只有物理对角线或映射几何确实违反准则时才由后续细化插点。
        const auto p=[&](int v){return surface.vertices[global[v]];};
        if(length(p(a),p(cc))<=length(p(b),p(d))) {
            mesh.addTri(a,b,cc);mesh.addTri(a,cc,d);
        } else {
            mesh.addTri(a,b,d);mesh.addTri(b,cc,d);
        }
    }
    w.constraints.clear();
    for (int i=0;i<nx-1;++i) {
        w.constraints.push_back({grid[i],grid[i+1]});
        w.constraints.push_back({grid[(ny-1)*nx+i],grid[(ny-1)*nx+i+1]});
    }
    for (int j=0;j<ny-1;++j) {
        w.constraints.push_back({grid[j*nx],grid[(j+1)*nx]});
        w.constraints.push_back({grid[j*nx+nx-1],grid[(j+1)*nx+nx-1]});
    }
    *edgeRefinement=true;
    return true;
}
}

struct CachedSurfaceFace {
    bool ready=false;
    bool preserveBoundary=false,edgeRefinement=false,qualityOptimized=false;
    std::uint64_t geometryVersion=0;
    std::uint64_t failedQualityVersion=0;
    std::vector<std::array<int,3>> triangles;
    std::vector<std::array<Point2D,3>> cornerUV;
    std::vector<std::string> diagnostics;
    double deviation=0,normalAngle=0,maxEdge=0,worstQuality=0;
    std::unique_ptr<Mesh2D> chart;
    std::vector<int> global;
    std::vector<std::pair<int,int>> constraints;
    std::vector<CadBoundarySizingAnchor> qualityAnchors;
};
struct SurfaceRunState {
    AssemblySurfacePolicy assemblyPolicy=AssemblySurfacePolicy::RejectBodyOverlap;
    SurfaceCriteria resolvedCriteria;
    std::unique_ptr<CadBoundaryRegistry> boundary;
    std::vector<Point3D> vertices;
    std::vector<CachedSurfaceFace> faces;
    std::vector<double> faceDeviationLimits;
    std::vector<std::uint64_t> criteriaVersions;
    std::vector<std::map<int,double>> referenceSeedSources;
    SurfaceWorkStatistics work;
    int steps=0;
    bool restart=false;
    bool boundaryQualityRestart=false;
    std::uint64_t nextGeometryVersion=1;
    std::vector<std::uint64_t> verifiedVersions;
    double verifiedTolerance=-1;
    std::vector<SurfaceBodyInteraction> verifiedInteractions;
    std::vector<std::string> diagnostics;
    std::vector<SurfaceRefinementSample> refinementSamples;
    std::map<std::uint64_t,SurfaceRefinementSample> pendingSamples;
};

/// 核对网格相交及其实际 CAD 支撑关系。
static bool validateCadIntersections(SurfaceMeshResult& result,const GeometrySource& geo,std::string* error,
    SurfaceIntersection* intersection=nullptr,bool* assemblyContact=nullptr,SurfaceRunState* verification=nullptr) {
    std::string cadError;
    double tolerance=0;std::vector<bool> active;
    if(verification){
        Point3D lo{1e300,1e300,1e300},hi{-1e300,-1e300,-1e300};double magnitude=0;
        for(auto p:result.mesh.vertices){lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
            hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
            magnitude=std::max({magnitude,std::abs(p.x),std::abs(p.y),std::abs(p.z)});}
        tolerance=64*std::numeric_limits<double>::epsilon()*std::max(length(lo,hi),magnitude);
        if(verification->verifiedVersions.size()==verification->faces.size()&&tolerance==verification->verifiedTolerance){
            std::set<int> changedBodies;std::vector<bool> changedFaces(verification->faces.size(),false);
            for(int f=0;f<int(changedFaces.size());++f)if(verification->faces[f].geometryVersion!=verification->verifiedVersions[f]){
                changedFaces[f]=true;int body=geo.faceBodyId(f);if(body>=0)changedBodies.insert(body);}
            active.reserve(result.mesh.triangles.size());
            for(int f:result.triangleFaceIds)active.push_back(changedFaces[f]||changedBodies.count(geo.faceBodyId(f)));
            for(const auto& item:verification->verifiedInteractions)if(!changedBodies.count(item.firstBody)&&!changedBodies.count(item.secondBody))
                result.bodyInteractions.push_back(item);
            if(assemblyContact&&!result.bodyInteractions.empty())*assemblyContact=true;
        }
    }
    const bool ok=validateSurfaceIntersections(result.mesh,error,tolerance,intersection,[&](int a,int b) {
        const int first=result.triangleFaceIds[a],second=result.triangleFaceIds[b];
        const int firstBody=geo.faceBodyId(first),secondBody=geo.faceBodyId(second);
        if(firstBody<0 || secondBody<0 || firstBody==secondBody)return false;
        const auto contact=geo.bodyContact(first,second);
        if(result.assemblyPolicy==AssemblySurfacePolicy::IndependentBodies || contact==CadBodyContact::BoundaryContact) {
            const int a=std::min(firstBody,secondBody),b=std::max(firstBody,secondBody);
            const auto found=std::find_if(result.bodyInteractions.begin(),result.bodyInteractions.end(),[&](const SurfaceBodyInteraction& i){
                return i.firstBody==a && i.secondBody==b && i.contact==contact;
            });
            if(found==result.bodyInteractions.end())result.bodyInteractions.push_back({a,b,firstBody==a?first:second,firstBody==a?second:first,contact});
            if(assemblyContact)*assemblyContact=true;
            return true;
        }
        switch(contact) {
        case CadBodyContact::BoundaryContact:if(assemblyContact)*assemblyContact=true;return true;
        case CadBodyContact::VolumeOverlap:cadError="CAD assembly has volume overlap between bodies "+std::to_string(geo.faceBodyId(first))+" and "+std::to_string(geo.faceBodyId(second))+" (CAD faces "+std::to_string(first)+", "+std::to_string(second)+")";return false;
        case CadBodyContact::QueryFailed:cadError="CAD assembly contact query failed between faces "+std::to_string(first)+" and "+std::to_string(second);return false;
        default:return false;
        }
    },active.empty()?nullptr:&active);
    if(!cadError.empty()) {if(error)*error=cadError;if(intersection)*intersection={};return false;}
    if(ok&&verification){verification->verifiedVersions.clear();
        for(const auto& face:verification->faces)verification->verifiedVersions.push_back(face.geometryVersion);
        verification->verifiedTolerance=tolerance;verification->verifiedInteractions=result.bodyInteractions;}
    return ok;
}

/// 处理一轮共享边界、各面剖分与细化，并装配检查结果。
static SurfaceMeshResult generateSurfacePass(const GeometrySource& geo,
                                         const SurfaceCriteria& requested,
                                         const MeshSizeField* sizeField,
                                         MeshingProgress progress,
                                         SurfaceRunState& state,MeshingObserver* observer) {
    // 一次调度扫描可复用已完成面；共享边变化后的待处理面由外层安排重访。
    // 本函数依次处理预算/输入、共享边、单面图表与细化、装配和最终检查。
    const MeshObservation globalObservation{observer,{}};
    MeshStageScope phase(globalObservation,MeshStage::BoundarySampling);
    SurfaceMeshResult out;
    out.assemblyPolicy=state.assemblyPolicy;
    ++state.work.schedulingPasses;
    out.error=geo.validationError();
    if (!out.error.empty()) return out;
    const int nv=geo.vertexCount(), ne=geo.edgeCount(), nf=geo.faceCount();
    if (nv<1 || ne<1 || nf<1) { out.error="empty CAD topology"; return out; }
    if (requested.maxVertices<nv || requested.maxIterations<1) {
        out.error="invalid meshing budget"; return out;
    }
    Point3D lo{1e300,1e300,1e300}, hi{-1e300,-1e300,-1e300};
    out.mesh.vertices.reserve(std::min(requested.maxVertices,10000));
    const bool initial=state.vertices.empty();
    if (!initial) out.mesh.vertices=state.vertices;
    for (int v=0;v<nv;++v) {
        const Point3D p=geo.vertexPoint(v);
        if (!finite(p)) { out.error="non-finite CAD vertex"; return out; }
        if (initial) out.mesh.vertices.push_back(p);
        lo.x=std::min(lo.x,p.x); lo.y=std::min(lo.y,p.y); lo.z=std::min(lo.z,p.z);
        hi.x=std::max(hi.x,p.x); hi.y=std::max(hi.y,p.y); hi.z=std::max(hi.z,p.z);
    }
    // 仅回收已丢弃的内部槽位；待同步边界的图表、登记边界及完成面的节点身份保持不变。
    std::vector<int> freeVertices;
    if(!initial) {
        std::vector<bool> used(out.mesh.vertices.size(),false);
        for(int v=0;v<nv;++v)used[v]=true;
        for(int e=0;e<ne;++e)for(const auto& entry:state.boundary->edge(e).nodeIds)if(entry.second>=0)used[entry.second]=true;
        for(auto& face:state.faces) {
            if(face.ready)for(const auto& t:face.triangles)for(int v:t)used[v]=true;
            else {
                if(face.preserveBoundary&&face.chart)for(int v:face.global)if(v>=0)used[v]=true;
                face.triangles.clear();face.cornerUV.clear();face.diagnostics.clear();
            }
        }
        for(int v=int(used.size())-1;v>=nv;--v)if(!used[v])freeVertices.push_back(v);
    }
    const auto activeVertices=[&] {return out.mesh.vertices.size()-freeVertices.size();};
    const auto allocate=[&](Point3D p) {
        if(!freeVertices.empty()) {int v=freeVertices.back();freeVertices.pop_back();out.mesh.vertices[v]=p;return v;}
        int v=int(out.mesh.vertices.size());out.mesh.vertices.push_back(p);return v;
    };
    Point3D boundsLo,boundsHi;
    if (geo.bounds(boundsLo,boundsHi)) {
        if (!finite(boundsLo)||!finite(boundsHi)) { out.error="non-finite geometry bounds"; return out; }
        lo=boundsLo; hi=boundsHi;
    }
    const double diag=length(lo,hi);
    SurfaceCriteria c=requested;
    if (c.targetSize<0 || c.minSize<0 || c.maxSize<0 || c.maxDeviation<0) {
        out.error="negative surface length criterion"; return out;
    }
    if (c.targetSize<=0) c.targetSize=diag*0.025;
    if (!std::isfinite(c.maxSize) || !std::isfinite(c.targetSize)) {
        out.error="invalid surface criteria: non-finite target or maximum size"; return out;
    }
    if (c.maxSize>0) c.targetSize=std::min(c.targetSize,c.maxSize);
    if (c.minSize<=0) c.minSize=c.targetSize*0.1;
    if (c.maxDeviation<=0) c.maxDeviation=c.targetSize*0.1;
    if (!std::isfinite(c.targetSize) || !std::isfinite(c.minSize) ||
        !std::isfinite(c.maxDeviation) || !std::isfinite(c.maxNormalAngleDeg) ||
        !std::isfinite(c.maxRadiusEdgeRatio) || !std::isfinite(c.growthRatio) ||
        c.targetSize<=0 || c.minSize>c.targetSize ||
        c.minSize<=0 || c.maxDeviation<=0 || c.maxNormalAngleDeg<=0 ||
        c.maxNormalAngleDeg>=180 || c.maxRadiusEdgeRatio<=0.5 || c.growthRatio<1 ||
        c.shapeOptimizationSweeps<0 || c.shapeOptimizationSweeps>16 ||
        (c.pointPlacement!=SurfacePointPlacement::LegacyRefinement && c.pointPlacement!=SurfacePointPlacement::FrontalDelaunay&&
         c.pointPlacement!=SurfacePointPlacement::ReferenceFrontal&&c.pointPlacement!=SurfacePointPlacement::PhysicalReferenceFrontal)) {
        out.error="invalid surface criteria"; return out;
    }
    if (progress && !progress(0,nf)) { out.error="surface meshing cancelled"; return out; }
    // 局部目标保留旧 5% 余量；显式最大尺寸只允许数值比较容差。
    const auto edgeLimit=[&](double target) {
        return c.maxSize>0?std::min(target*1.05,c.maxSize*(1+1e-8)):target*1.05;
    };
    state.resolvedCriteria=c;
    auto targetAt=[&](Point3D p) {
        if (!sizeField) return c.targetSize;
        const double h=sizeField->sizeAt(p);
        if (!std::isfinite(h) || h<=0) {
            out.error="invalid scalar mesh size field value";
            return c.targetSize;
        }
        return std::clamp(h,c.minSize,c.targetSize);
    };
    std::vector<FaceTopology> topology(nf);
    std::vector<std::vector<std::pair<int,EdgeUse>>> edgeUses(ne);
    if (!state.boundary) {
        state.boundary=std::make_unique<CadBoundaryRegistry>(ne,observer);
        state.faces.resize(nf);
        state.faceDeviationLimits.assign(nf,c.maxDeviation);
        state.criteriaVersions.assign(nf,1);
        state.referenceSeedSources.resize(nf);
    }
    for (int f=0;f<nf;++f) {
        topology[f]=geo.faceTopology(f);
        if (topology[f].loops.empty()) { out.error="face has no trim loop"; return out; }
        for (const auto& loop:topology[f].loops) for (const auto& use:loop) {
            if (use.edge<0 || use.edge>=ne) { out.error="invalid CAD edge ID"; return out; }
            edgeUses[use.edge].push_back({f,use});
            state.boundary->addUse(use.edge,f);
        }
    }
    for (int e=0;e<ne;++e) {
        MeshStageScope edgePhase({observer,{1,e}},MeshStage::BoundarySampling);
        if (progress && !progress(0,nf)) { out.error="surface meshing cancelled"; return out; }
        auto& record=state.boundary->edge(e);
        if (record.ready) {
            out.measuredMaxDeviation=std::max(out.measuredMaxDeviation,record.measuredDeviation);
            out.measuredMaxNormalAngleDeg=std::max(out.measuredMaxNormalAngleDeg,record.measuredNormalAngle);
            out.diagnostics.insert(out.diagnostics.end(),record.diagnostics.begin(),record.diagnostics.end());
            for (size_t j=1;j<record.samples.size();++j)
                out.boundaryEdges.push_back({record.samples[j-1].global,record.samples[j].global,e});
            continue;
        }
        record.measuredDeviation=0; record.measuredNormalAngle=0; record.diagnostics.clear();
        double edgeDeviation=c.maxDeviation;
        for (const auto& use:edgeUses[e]) edgeDeviation=std::min(edgeDeviation,state.faceDeviationLimits[use.first]);
        bool warnedSmall=false;
        bool warnedSizing=false,warnedSingular=false;
        const auto [v0,v1]=geo.edgeVertices(e);
        const auto [t0,t1]=geo.edgeRange(e);
        if (v0<0 || v0>=nv || v1<0 || v1>=nv || !std::isfinite(t0) ||
            !std::isfinite(t1) || t0>=t1) { out.error="invalid CAD edge range/endpoints"; return out; }
        record.nodeIds[t0]=v0; record.nodeIds[t1]=v1;
        auto& seq=record.samples; seq.clear(); seq.push_back({t0,v0});
        const auto nodeAt=[&](double t,Point3D p) {
            auto inserted=record.nodeIds.emplace(t,-1);
            int& id=inserted.first->second;
            if (id<0) {
                id=allocate(p);
            }
            return id;
        };
        std::map<int,std::unique_ptr<SurfaceEvaluationCache>> edgeCaches;
        const auto cacheFor=[&](int face)->SurfaceEvaluationCache& {
            auto& ptr=edgeCaches[face];
            if (!ptr) ptr=std::make_unique<SurfaceEvaluationCache>(geo,face,MeshObservation{observer,{2,face}});
            return *ptr;
        };
        struct FaceSample { Point2D uv; Point3D normal; };
        using FaceSamples=std::vector<FaceSample>;
        const auto sampleFaces=[&](double t) {
            FaceSamples samples;
            for (const auto& use:edgeUses[e]) {
                auto& cache=cacheFor(use.first); const auto uv=cache.edgeUV(use.first,use.second,t);
                const double nan=std::numeric_limits<double>::quiet_NaN();
                samples.push_back({uv,finite(uv)?cache.normal(uv):Point3D{nan,nan,nan}});
            }
            return samples;
        };
        std::function<bool(double,Point3D,double,Point3D,int,const FaceSamples*,const FaceSamples*)> split;
        split=[&](double a,Point3D pa,double b,Point3D pb,int depth,const FaceSamples* left,const FaceSamples* right) {
            const double m=a+(b-a)*0.5;
            const Point3D pm=geo.edgePoint(e,m);
            if (!finite(pm)) { out.error="non-finite CAD curve evaluation"; return false; }
            // 子区间复用左右端点的逐面 UV/法向；只查询新中点。
            const FaceSamples localLeft=left?FaceSamples{}:sampleFaces(a),localRight=right?FaceSamples{}:sampleFaces(b);
            const auto& sa=left?*left:localLeft; const auto& sb=right?*right:localRight;
            const auto sm=sampleFaces(m);
            double h=std::min({targetAt(pm),targetAt(pa)*c.growthRatio,targetAt(pb)*c.growthRatio});
            if (!out.error.empty()) return false;
            double ang=0;
            for (size_t k=0;k<edgeUses[e].size();++k) {
                const auto& use=edgeUses[e][k];
                const Point2D ua=sa[k].uv,ub=sb[k].uv,um=sm[k].uv;
                if (!finite(ua)||!finite(ub)||!finite(um)) { out.error="non-finite trim curve"; return false; }
                SurfaceCriteria edgeCriteria=c; edgeCriteria.maxDeviation=edgeDeviation;
                const SurfaceSizeSample sizing=surfaceSizeAt(cacheFor(use.first),use.first,um,h,edgeCriteria);
                if (!sizing.valid) {
                    out.error=sizing.error+" at CAD edge "+std::to_string(e)+" face "+std::to_string(use.first);
                    return false;
                }
                h=std::min(h,sizing.target);
                if (!warnedSizing && sizing.target<c.minSize) {
                    record.diagnostics.push_back("CAD edge "+std::to_string(e)+
                        " curvature sizing below minSize on face "+std::to_string(use.first)+
                        " (target="+std::to_string(sizing.target)+", estimated curvature="+
                        std::to_string(sizing.maxCurvature)+")");
                    warnedSizing=true;
                }
                if (!warnedSingular && sizing.singular) {
                    record.diagnostics.push_back("singular normal derivative at CAD edge "+std::to_string(e)+
                        " face "+std::to_string(use.first)+"; adjacent regular samples and facet checks control refinement");
                    warnedSingular=true;
                }
                const Point3D na=sa[k].normal,nm=sm[k].normal,nb=sb[k].normal;
                if (!finite(na)||!finite(nm)||!finite(nb)) {
                    out.error="non-finite CAD normal at edge "+std::to_string(e); return false;
                }
                ang=std::max({ang,angleDeg(na,nm),angleDeg(nm,nb),angleDeg(na,nb)});
            }
            const double deviation=lineDeviation(pa,pb,pm);
            const bool bad=length(pa,pb)>h*(1+1e-8) ||
                deviation>edgeDeviation || ang>c.maxNormalAngleDeg;
            if (bad) {
                if (depth>=32 || ++state.steps>c.maxIterations ||
                    static_cast<int>(activeVertices())>=c.maxVertices) {
                    out.error="CAD edge sampling budget exceeded at edge "+std::to_string(e)+
                        " (length="+std::to_string(length(pa,pb))+", deviation="+
                        std::to_string(deviation)+", normal angle="+std::to_string(ang)+")";
                    return false;
                }
                if (state.steps%64==0 && progress && !progress(0,nf)) {
                    out.error="surface meshing cancelled during CAD edge sampling"; return false;
                }
                ++state.work.boundarySamplingSplits;
                if (!split(a,pa,m,pm,depth+1,&sa,&sm)) return false;
                if (static_cast<int>(activeVertices())>=c.maxVertices) {
                    out.error="CAD edge sampling vertex budget exceeded at edge "+std::to_string(e);
                    return false;
                }
                seq.push_back({m,nodeAt(m,pm)});
                if (!split(m,pm,b,pb,depth+1,&sm,&sb)) return false;
            } else {
                record.measuredDeviation=std::max(record.measuredDeviation,deviation);
                record.measuredNormalAngle=std::max(record.measuredNormalAngle,ang);
                if (!warnedSmall && length(pa,pb)>0 && length(pa,pb)<c.minSize) {
                    record.diagnostics.push_back("CAD edge "+std::to_string(e)+
                        " requires a segment below minSize to preserve a feature or geometric criterion");
                    warnedSmall=true;
                }
            }
            return true;
        };
        std::vector<double> cuts;
        const bool referencePlacement=c.pointPlacement==SurfacePointPlacement::ReferenceFrontal||c.pointPlacement==SurfacePointPlacement::PhysicalReferenceFrontal;
        if(referencePlacement&&record.nodeIds.size()==2&&!geo.edgeDegenerate(e)) {
            const auto referenceEdgeSize=[&](double t) {
                const auto p=geo.edgePoint(e,t);double target=targetAt(p);
                double h=.7071067811865475244*edgeLimit(target);
                for(const auto& use:edgeUses[e]) {
                    auto& cache=cacheFor(use.first);const auto uv=cache.edgeUV(use.first,use.second,t);
                    SurfaceCriteria local=c;local.maxDeviation=edgeDeviation;
                    const auto size=surfaceSizeAt(cache,use.first,uv,target,local);
                    if(!size.valid){out.error=size.error+" during reference edge sizing "+std::to_string(e);return std::numeric_limits<double>::quiet_NaN();}
                    h=std::min({h,size.target,.7071067811865475244*size.curvatureSpan});
                }
                return h;
            };
            ReferenceCurveSamplingOptions options;
            options.maxSegments=std::max(1,std::min(c.maxIterations-state.steps+1,c.maxVertices-int(activeVertices())+1));
            std::vector<double> parameters;std::string samplingError;
            if(!sampleReferenceCurve(t0,t1,[&](double t){return geo.edgeDerivative(e,t);},referenceEdgeSize,parameters,&samplingError,
                options,[&]{return !progress||progress(0,nf);})) {
                if(out.error.empty())out.error=samplingError+" at CAD edge "+std::to_string(e);return out;
            }
            for(size_t k=1;k+1<parameters.size();++k) {
                if(state.steps>=c.maxIterations){out.error="reference boundary work budget exceeded";return out;}
                if(record.nodeIds.emplace(parameters[k],-1).second){++state.steps;++state.work.boundarySamplingSplits;}
            }
        }
        // 规则闭合曲线仍需要不同的边界顶点，即使整条弦已经满足较宽松的几何容差。
        if (v0==v1 && !geo.edgeDegenerate(e)&&(!referencePlacement||record.nodeIds.size()<5))
            for (int k=1;k<4;++k) record.nodeIds.emplace(t0+(t1-t0)*k/4,-1);
        for (const auto& entry:record.nodeIds)
            if (entry.first>t0 && entry.first<t1) cuts.push_back(entry.first);
        std::sort(cuts.begin(),cuts.end());
        cuts.erase(std::unique(cuts.begin(),cuts.end()),cuts.end());
        double previous=t0;
        Point3D previousPoint=geo.vertexPoint(v0);
        for (double t:cuts) {
            const Point3D p=geo.edgePoint(e,t);
            if (!split(previous,previousPoint,t,p,0,nullptr,nullptr)) return out;
            if (static_cast<int>(activeVertices())>=c.maxVertices) {
                out.error="shared CAD edge split exceeds vertex budget"; return out;
            }
            seq.push_back({t,nodeAt(t,p)});
            previous=t; previousPoint=p;
        }
        if (!split(previous,previousPoint,t1,geo.vertexPoint(v1),0,nullptr,nullptr)) return out;
        seq.push_back({t1,v1});
        if(record.qualityAnchors.empty())for(size_t i=0;i<seq.size();++i){const auto p=out.mesh.vertices[seq[i].global];double h=c.targetSize;
            if(i)h=std::min(h,length(p,out.mesh.vertices[seq[i-1].global]));
            if(i+1<seq.size())h=std::min(h,length(p,out.mesh.vertices[seq[i+1].global]));
            if(h>0)record.qualityAnchors.push_back({p,h,e});}
        record.ready=true;
        out.measuredMaxDeviation=std::max(out.measuredMaxDeviation,record.measuredDeviation);
        out.measuredMaxNormalAngleDeg=std::max(out.measuredMaxNormalAngleDeg,record.measuredNormalAngle);
        out.diagnostics.insert(out.diagnostics.end(),record.diagnostics.begin(),record.diagnostics.end());
        for (size_t i=1;i<seq.size();++i)
            out.boundaryEdges.push_back({seq[i-1].global,seq[i].global,e});
    }
    struct SharedInterval {int edge;double left,right;};
    std::unordered_map<uint64_t,SharedInterval> sharedIntervals;
    for(int e=0;e<ne;++e){const auto& seq=state.boundary->edge(e).samples;
        for(size_t j=1;j<seq.size();++j)sharedIntervals.emplace(key(seq[j-1].global,seq[j].global),SharedInterval{e,seq[j-1].t,seq[j].t});}
    auto requestSharedSplit=[&](int a,int b,const std::string& reason,int sourceFace,SurfaceRefinementSample* sample=nullptr,
                                CadBoundarySplitPurpose purpose=CadBoundarySplitPurpose::Geometry) {
        const auto found=sharedIntervals.find(key(a,b));
        if(found!=sharedIntervals.end()) {
                const auto interval=found->second;const int e=interval.edge;
                std::uint64_t requestId=0;
                if (!state.boundary->enqueueSplit(e,interval.left,interval.right,sourceFace,reason,requestId,&out.error,{},purpose)) {
                    out.error+=" at CAD edge "+std::to_string(e); return false;
                }
                if (sample && state.refinementSamples.size()+state.pendingSamples.size()<8) {
                    sample->cadEdge=e; sample->parameterStart=interval.left; sample->parameterEnd=interval.right;
                    state.pendingSamples.emplace(requestId,*sample);
                }
                state.restart=true;
                return true;
        }
        out.error="CAD boundary segment not found in shared sampling table";
        return false;
    };
    const SurfaceCriteria baseCriteria=c;
    for (int f=0;f<nf;++f) {
        const MeshObservation observation{observer,{2,f}};
        phase.change(observation,MeshStage::InitialTriangulation);
        SurfaceCriteria c=baseCriteria; c.maxDeviation=state.faceDeviationLimits[f];
        if (progress && !progress(f,nf)) { out.error="surface meshing cancelled"; return out; }
        auto& cached=state.faces[f];
        if (cached.ready) {
            out.mesh.triangles.insert(out.mesh.triangles.end(),cached.triangles.begin(),cached.triangles.end());
            out.triangleFaceIds.insert(out.triangleFaceIds.end(),cached.triangles.size(),f);
            out.triangleCornerUV.insert(out.triangleCornerUV.end(),cached.cornerUV.begin(),cached.cornerUV.end());
            out.diagnostics.insert(out.diagnostics.end(),cached.diagnostics.begin(),cached.diagnostics.end());
            out.measuredMaxDeviation=std::max(out.measuredMaxDeviation,cached.deviation);
            out.measuredMaxNormalAngleDeg=std::max(out.measuredMaxNormalAngleDeg,cached.normalAngle);
            out.measuredMaxEdgeLength=std::max(out.measuredMaxEdgeLength,cached.maxEdge);
            out.measuredWorstRadiusEdgeRatio=std::max(out.measuredWorstRadiusEdgeRatio,cached.worstQuality);
            continue;
        }
        const size_t triangleStart=out.mesh.triangles.size(),diagnosticStart=out.diagnostics.size();
        ++state.work.faceBuilds;
        CachedSurfaceFace built;
        FaceEvaluationCache queries(geo,f,observation);
        FaceWork w;
        std::unordered_map<int,std::vector<int>> byGlobal;
        auto addLocal=[&](int global,Point2D uv) {
            auto& candidates=byGlobal[global];
            for (int local:candidates)
                if (dist2D(w.uv[local],uv)<=1e-12) return local;
            const int local=static_cast<int>(w.uv.size());
            w.uv.push_back(uv); w.global.push_back(global); candidates.push_back(local);
            return local;
        };
        for (const auto& trimLoop:topology[f].loops) {
            std::vector<int> loop;
            for (const EdgeUse& use:trimLoop) {
                const auto& seq=state.boundary->edge(use.edge).samples;
                for (size_t k=0;k+1<seq.size();++k) {
                    const CadBoundarySample& s=use.reversed?seq[seq.size()-1-k]:seq[k];
                    const Point2D uv=queries.edgeUV(f,use,s.t);
                    if (!finite(uv)) { out.error="non-finite face trim UV"; return out; }
                    const int local=addLocal(s.global,uv);
                    if (loop.empty() || loop.back()!=local) loop.push_back(local);
                }
            }
            if (loop.size()<3) { out.error="trim loop has fewer than 3 UV vertices on face "+std::to_string(f); return out; }
            for (size_t i=0;i<loop.size();++i)
                w.constraints.push_back({loop[i]+3,loop[(i+1)%loop.size()]+3});
            w.loops.push_back(std::move(loop));
        }
        // 有效曲线裁剪的弦段可能在粗 UV 多边形中相交；通过细分原共享 CAD 子段修正该多边形。
        for (size_t i=0;i<w.constraints.size();++i) for (size_t j=i+1;j<w.constraints.size();++j) {
            const int a=w.constraints[i].first-3,b=w.constraints[i].second-3;
            const int p=w.constraints[j].first-3,q=w.constraints[j].second-3;
            if (a==p || a==q || b==p || b==q) continue;
            if (orient2d(w.uv[a],w.uv[b],w.uv[p])*orient2d(w.uv[a],w.uv[b],w.uv[q])<0 &&
                orient2d(w.uv[p],w.uv[q],w.uv[a])*orient2d(w.uv[p],w.uv[q],w.uv[b])<0) {
                const std::string reason="crossing UV trim chords on face "+std::to_string(f);
                if (!requestSharedSplit(w.global[a],w.global[b],reason,f)) {
                    out.error+="; crossing trim chords on face "+std::to_string(f)+
                        " UV ("+std::to_string(w.uv[a].x)+","+std::to_string(w.uv[a].y)+") ("+
                        std::to_string(w.uv[b].x)+","+std::to_string(w.uv[b].y)+") and ("+
                        std::to_string(w.uv[p].x)+","+std::to_string(w.uv[p].y)+") ("+
                        std::to_string(w.uv[q].x)+","+std::to_string(w.uv[q].y)+")";
                    return out;
                }
                if (!requestSharedSplit(w.global[p],w.global[q],reason,f)) { state.restart=false; return out; }
                return out;
            }
        }
        double u0=1e300,u1=-1e300,v0=1e300,v1=-1e300;
        for (const auto& p:w.uv) {
            u0=std::min(u0,p.x); u1=std::max(u1,p.x);
            v0=std::min(v0,p.y); v1=std::max(v1,p.y);
        }
        if (!(u1>u0 && v1>v0)) { out.error="collapsed face UV domain"; return out; }
        FaceMetric metric;
        if (!surfaceMetricAt(queries,f,{0.5*(u0+u1),0.5*(v0+v1)},metric,&out.error)) {
            out.error+=" on face "+std::to_string(f); return out;
        }
        metric.origin={u0,v0};
        Mesh2D mesh;
        std::vector<int> global;
        std::string error;
        bool edgeRefinement=false;
        bool synchronized=false;std::vector<int> boundaryInsertions;
        if(cached.preserveBoundary&&cached.chart){
            mesh=std::move(*cached.chart);global=std::move(cached.global);
            synchronized=synchronizeBoundaryChart(mesh,global,w,cached.constraints,boundaryInsertions,&error,observation);
            if(synchronized){edgeRefinement=cached.edgeRefinement;
                out.diagnostics.push_back("face "+std::to_string(f)+" shared boundary locally synchronized: "+
                    std::to_string(boundaryInsertions.size())+" nodes; accepted interior layout retained");}
            else{out.diagnostics.push_back("face "+std::to_string(f)+" local boundary synchronization refused: "+error+
                    "; rebuilding chart");mesh.clear();global.clear();boundaryInsertions.clear();error.clear();}
        }
        const bool rectangular=!synchronized&&c.pointPlacement==SurfacePointPlacement::LegacyRefinement&&
            rectangularGrid(queries,f,c,w,mesh,global,out.mesh,&edgeRefinement,&error,observation,allocate,activeVertices);
        if (!error.empty()) { out.error="face "+std::to_string(f)+": "+error; return out; }
        if (rectangular && !legalize(mesh,w.constraints,&error,progress,f,nf,state.steps,c.maxIterations,metric,state.work.delaunayFlips,-1,-1,observation)) {
            out.error="face "+std::to_string(f)+" CDT legalization: "+error; return out;
        }
        if (!rectangular&&!synchronized) {
        const LinearMetric2D linear{metric.xx,metric.xy,metric.yy};
        const Delaunay2DResult initial=delaunay2D(mesh,w.uv,true,&linear,observation);
        if (!initial.ok) { out.error="face "+std::to_string(f)+": "+initial.error; return out; }
        if (initial.insertSteps!=static_cast<long long>(w.uv.size())) {
            out.error="coincident trim UV nodes on face "+std::to_string(f); return out;
        }
        if (!enforceConstraints(mesh,w.constraints,&error,observation)) {
            // 在 CDT 边界失败处保存有限规模的复现快照；此时尚未由前沿插入内部节点。
            std::ostringstream snapshot;snapshot<<std::setprecision(17)<<"CDT input face="<<f
                <<" metric="<<metric.xx<<','<<metric.xy<<','<<metric.yy<<" points="<<w.uv.size()<<" constraints="<<w.constraints.size();
            for(size_t i=0;i<std::min(size_t(64),w.uv.size());++i)snapshot<<"; uv["<<i+3<<"]="<<w.uv[i].x<<','<<w.uv[i].y;
            for(size_t i=0;i<std::min(size_t(64),w.constraints.size());++i)snapshot<<"; edge="<<w.constraints[i].first<<','<<w.constraints[i].second;
            out.diagnostics.push_back(snapshot.str());
            out.error="face "+std::to_string(f)+" CDT: "+error; return out;
        }
        global.assign(mesh.points.size(),-1);
        for (size_t i=0;i<w.global.size();++i) global[i+3]=w.global[i];
        if (!legalize(mesh,w.constraints,&error,progress,f,nf,state.steps,c.maxIterations,metric,state.work.delaunayFlips,-1,-1,observation)) {
            out.error="face "+std::to_string(f)+" initial CDT: "+error; return out;
        }
        }
        phase.change(observation,MeshStage::Refinement);
        bool warnedSmall=false;
        SurfaceDomainState domain;
        if (!domain.initialize(mesh,w.constraints,&error,observation)) {
            out.error="face "+std::to_string(f)+" initial domain classification: "+error; return out;
        }
        // 质量阶段原图表已含物理优化连接，只从新增边界的一环恢复局部 CDT。
        for(int v:boundaryInsertions){int seed=-1;
            for(int t=0;t<int(mesh.tris.size());++t){const auto tri=mesh.tris[t];
                if(tri.v[0]==v||tri.v[1]==v||tri.v[2]==v){seed=t;break;}}
            if(seed<0||!legalize(mesh,w.constraints,&error,progress,f,nf,state.steps,c.maxIterations,metric,
                state.work.delaunayFlips,v,seed,observation,nullptr,&domain)){
                out.error="face "+std::to_string(f)+" synchronized boundary CDT: "+error;return out;}
        }
        const auto& inDomain=domain.values();
        const bool referencePlacement=c.pointPlacement==SurfacePointPlacement::ReferenceFrontal||
            c.pointPlacement==SurfacePointPlacement::PhysicalReferenceFrontal;
        if(referencePlacement&&state.referenceSeedSources[f].empty()) {
            std::vector<SurfaceReferenceSize> initialSizes;
            if(!initializeSurfaceReferenceSizes(mesh,global,out.mesh.vertices,domain,w.constraints,initialSizes,&error)) {
                out.error="face "+std::to_string(f)+" reference source snapshot: "+error;return out;
            }
            for(auto edge:w.constraints)for(int v:{edge.first,edge.second})if(initialSizes[v].mesh>0) {
                auto [at,added]=state.referenceSeedSources[f].emplace(global[v],initialSizes[v].mesh);
                if(!added)at->second=std::min(at->second,initialSizes[v].mesh);
            }
        }
        {
            // 先协调共享边上的尺寸过渡，否则边内候选会在固定长约束旁形成新的瘦片。
            std::vector<Point3D> points(mesh.points.size());std::vector<std::array<int,3>> triangles;
            std::set<uint64_t> poorEdges;std::set<int> poorPoles;
            std::map<uint64_t,std::vector<int>> poorOpposite;
            std::set<std::tuple<double,double,double>> polePositions;
            for(const auto& loop:topology[f].loops)for(auto use:loop)if(geo.edgeDegenerate(use.edge)){
                auto ends=geo.edgeVertices(use.edge);if(ends.first>=0){auto p=geo.vertexPoint(ends.first);polePositions.emplace(p.x,p.y,p.z);}}
            for(int v=0;v<int(global.size());++v)if(global[v]>=0)points[v]=out.mesh.vertices[global[v]];
            for(int t=0;t<int(mesh.tris.size());++t)if(inDomain[t]){const auto& tri=mesh.tris[t];
                if(global[tri.v[0]]<0||global[tri.v[1]]<0||global[tri.v[2]]<0)continue;
                if(global[tri.v[0]]==global[tri.v[1]]||global[tri.v[1]]==global[tri.v[2]]||global[tri.v[2]]==global[tri.v[0]])continue;
                triangles.push_back({tri.v[0],tri.v[1],tri.v[2]});
                const auto a=points[tri.v[0]],b=points[tri.v[1]],d=points[tri.v[2]];
                if(std::min({length(a,b),length(b,d),length(d,a)})>=c.minSize && ratio(a,b,d)>c.maxRadiusEdgeRatio){
                    for(int k=0;k<3;++k){const auto edge=key(tri.v[k],tri.v[(k+1)%3]);poorEdges.insert(edge);
                        poorOpposite[edge].push_back(tri.v[(k+2)%3]);
                        auto p=points[tri.v[k]];if(polePositions.count({p.x,p.y,p.z}))poorPoles.insert(global[tri.v[k]]);}}}
            std::set<int> sources;
            for(const auto& loop:topology[f].loops)for(auto use:loop)sources.insert(use.edge);
            for(int edge:sources){const auto& anchors=state.boundary->edge(edge).qualityAnchors;built.qualityAnchors.insert(built.qualityAnchors.end(),anchors.begin(),anchors.end());}
            // 已完成的图表由后置物理候选协调质量；不能重新走种子边界预处理而丢弃布局。
            if(!poorEdges.empty()&&!synchronized){SurfaceBoundarySizeField sizes;const double floor=c.minSize;
                if(!sizes.initialize(points,triangles,w.constraints,c.targetSize,floor,c.growthRatio,&error,
                    fixedBoundarySeeds(points,w.constraints,built.qualityAnchors,c.targetSize))){out.error=error;return out;}
                bool requested=false;
                for(auto e:w.constraints){const double len=length(points[e.first],points[e.second]);
                    const bool poleTransition=poorPoles.count(global[e.first])||poorPoles.count(global[e.second]);
                    if(!poorEdges.count(key(e.first,e.second)) && !poleTransition)continue;
                    if(len<2*floor || global[e.first]==global[e.second])continue;
                    const double middle=std::min(sizes.at(e.first),sizes.at(e.second))+(c.growthRatio-1)*len/2;
                    const auto found=sharedIntervals.find(key(global[e.first],global[e.second]));
                    if(found==sharedIntervals.end()){out.error="boundary quality segment missing from shared registry";return out;}
                    const auto interval=found->second;
                    const auto mid=geo.edgePoint(interval.edge,interval.left+(interval.right-interval.left)*.5);
                    if(!finite(mid)){out.error="non-finite boundary quality midpoint";return out;}
                    if(length(points[e.first],mid)<floor || length(points[e.second],mid)<floor)continue;
                    bool encroached=false;
                    const auto opposite=poorOpposite.find(key(e.first,e.second));
                    if(opposite!=poorOpposite.end())for(int v:opposite->second)
                        encroached|=qualityBoundarySplitAllowed(points[e.first],points[e.second],points[v],mid,floor,c.maxRadiusEdgeRatio);
                    if(len<=std::max(floor,middle)*1.05 && !poleTransition && !encroached)continue;
                    if(!requestSharedSplit(global[e.first],global[e.second],
                        (poleTransition?"pole boundary quality transition on face ":encroached?"encroached boundary quality transition on face ":
                            "boundary size transition on face ")+std::to_string(f),f,nullptr,CadBoundarySplitPurpose::Quality))return out;
                    requested=true;
                }
                // 按同一个登记表版本汇总全部面的细分请求。
                if(requested)continue;
            }
        }
        if(!synchronized&&c.pointPlacement==SurfacePointPlacement::FrontalDelaunay) {
            SurfaceFrontalStatistics frontal;
            const bool generated=generateSurfaceFrontalPoints(mesh,global,out.mesh.vertices,domain,w.constraints,queries,f,c,
                targetAt,allocate,state.steps,frontal,&error,progress,observation,activeVertices);
            state.work.frontalCandidates+=frontal.attempted;state.work.frontalInserted+=frontal.inserted;
            state.work.frontalRejected+=frontal.rejected;state.work.insertedSurfacePoints+=frontal.inserted;
            out.diagnostics.push_back("face "+std::to_string(f)+" frontal generation: "+std::to_string(frontal.inserted)+
                " points, "+std::to_string(frontal.rejected)+" rejected candidates");
            for(const auto& reason:frontal.refusals)out.diagnostics.push_back("frontal candidate rejection: "+reason);
            if(!generated){out.error="face "+std::to_string(f)+" frontal generation: "+error;return out;}
        }
        if(!synchronized&&(c.pointPlacement==SurfacePointPlacement::ReferenceFrontal||c.pointPlacement==SurfacePointPlacement::PhysicalReferenceFrontal)) {
            std::string sizingError;
            int chartSeeds=0;
            if(!seedSurfaceReferenceChart(mesh,global,out.mesh.vertices,domain,w.constraints,queries,f,c,allocate,
                state.steps,chartSeeds,&error,progress,observation,activeVertices)) {
                out.error="face "+std::to_string(f)+" reference chart seed: "+error;return out;
            }
            state.work.insertedSurfacePoints+=chartSeeds;
            if(chartSeeds)out.diagnostics.push_back("face "+std::to_string(f)+" reference periodic chart seeds: "+std::to_string(chartSeeds));
            // 产品护栏只作为显式尺寸上界进入生成，不改变参考生成器内部规则。
            // R<=h/sqrt(2) 且 chord<=2R：逐边硬上界H对应生成h<=H/sqrt(2)。
            // 与边界采样使用同一转换；变尺寸与实际曲面仍逐片验收，不能据此跳过硬检查。
            const auto referenceSize=[&](Point2D uv,Point3D point) {
                const double target=targetAt(point);const auto sample=surfaceSizeAt(queries,f,uv,target,c);
                if(!sample.valid){sizingError=sample.error;return std::numeric_limits<double>::quiet_NaN();}
                return std::min({sample.target,.7071067811865475244*sample.curvatureSpan,.7071067811865475244*edgeLimit(target)});
            };
            std::vector<SurfaceReferenceSize> sizes;
            std::vector<std::pair<int,double>> preserved;
            if(!preservedReferenceSeedSizes(mesh,global,out.mesh.vertices,domain,state.referenceSeedSources[f],c,preserved,&error)) {
                out.error="face "+std::to_string(f)+" reference preserved sources: "+error;return out;
            }
            if(!initializeSurfaceReferenceSizes(mesh,global,out.mesh.vertices,domain,w.constraints,sizes,&error,referenceSize,preserved)) {
                out.error="face "+std::to_string(f)+" reference sizing: "+(sizingError.empty()?error:sizingError);return out;
            }
            SurfaceReferenceOptions options;
            options.physicalCavity=options.physicalPointHeight=c.pointPlacement==SurfacePointPlacement::PhysicalReferenceFrontal;
            SurfaceReferenceStatistics frontal;
            const bool generated=generateSurfaceReferenceFront(mesh,global,out.mesh.vertices,domain,w.constraints,queries,f,c,sizes,
                referenceSize,allocate,state.steps,frontal,&error,options,progress,observation,activeVertices,
                [&](Point2D,Point3D point){return edgeLimit(targetAt(point));});
            state.work.frontalCandidates+=frontal.attempted;state.work.frontalInserted+=frontal.inserted;
            state.work.frontalRejected+=frontal.rejected;state.work.insertedSurfacePoints+=frontal.inserted;
            out.diagnostics.push_back("face "+std::to_string(f)+" "+surfacePointPlacementName(c.pointPlacement)+" generation: "+
                std::to_string(frontal.inserted)+" points, "+std::to_string(frontal.rejected)+" rejected, "+
                std::to_string(frontal.remainingLargeTriangles)+" unsettled, "+std::to_string(frontal.circleFallbacks)+" circle fallbacks, "+
                std::to_string(frontal.collapsedPoleTriangles)+" collapsed UV pole triangles");
            for(const auto& reason:frontal.refusals)out.diagnostics.push_back("reference candidate rejection: "+reason);
            if(!generated){out.error="face "+std::to_string(f)+" reference generation: "+(sizingError.empty()?error:sizingError);return out;}
        }
        // 失败时追溯点的构造来源；仅输出最多 24 个祖先，不保存整轮日志。
        std::vector<std::array<int,3>> pointParents(mesh.points.size(),{-1,-1,-1});
        const auto tracePoints=[&](int triangle) {
            std::vector<int> pending;
            for (int v:mesh.tris[triangle].v) pending.push_back(v);
            std::unordered_set<int> visited;
            while (!pending.empty() && visited.size()<24) {
                const int v=pending.back(); pending.pop_back();
                if (!visited.insert(v).second) continue;
                const auto p=mesh.points[v]; const auto parents=pointParents[v];
                std::ostringstream message; message.precision(17);
                message<<"face "<<f<<" UV vertex "<<v<<" ("<<p.x<<','<<p.y<<") parents="
                       <<parents[0]<<','<<parents[1]<<','<<parents[2];
                out.diagnostics.push_back(message.str());
                for (int parent:parents) if (parent>=0) pending.push_back(parent);
            }
        };
        const auto snapshot=[&](int t) {
            SurfaceFacetSnapshot s;
            const auto& tri=mesh.tris[t];
            Point2D center{};
            for (int k=0;k<3;++k) {
                s.uv[k]=mesh.points[tri.v[k]];
                s.surfacePoints[k]=queries.point(s.uv[k]);
                const int id=global[tri.v[k]];
                s.sharedPoints[k]=id>=0?out.mesh.vertices[id]:s.surfacePoints[k];
                center.x+=s.uv[k].x/3; center.y+=s.uv[k].y/3;
            }
            const Point3D n=queries.normal(center);
            const double sign=geo.faceReversed(f)?-1:1;
            const auto area=[&](const std::array<Point3D,3>& p) {
                const double len=norm3(n);
                return len>0?sign*dot3(cross3(minus3(p[1],p[0]),minus3(p[2],p[0])),n)/len:0.0;
            };
            s.sharedSignedArea=area(s.sharedPoints); s.surfaceSignedArea=area(s.surfacePoints);
            return s;
        };
        struct CachedTriangleTest { TriangleMetrics metrics; double target; };
        std::map<std::array<int,3>,CachedTriangleTest> measured;
        {
        int iterations=0,balancedSplitRefusals=0;
        const bool physicalReference=c.pointPlacement==SurfacePointPlacement::PhysicalReferenceFrontal;
        // 前沿已建立局部物理连接；固定UV度量的全图重翻边会覆盖该布局。
        bool requireLegalization=!synchronized&&!physicalReference;
        const std::function<int(int,int,int,int)> physicalCircle=physicalReference?std::function<int(int,int,int,int)>{[&](int a,int b,int d,int e) {
            return physicalCircleSign(mesh,global,out.mesh.vertices,queries,f,a,b,d,e,&error);
        }}:std::function<int(int,int,int,int)>{};
        std::unordered_map<uint64_t,bool> constrainedEdges;
        std::unordered_map<int,bool> boundaryVertices;
        for (auto e:w.constraints) {
            constrainedEdges[key(e.first,e.second)]=true;
            boundaryVertices[e.first]=true; boundaryVertices[e.second]=true;
        }
        SurfaceRefinementQueue queue(f,state.criteriaVersions[f],observation);
        const auto qualityViolation=[&](const Tri2D& tri,const TriangleMetrics& metrics) {
            // 保留内部粗片的廉价种子细化；后置候选阶段仍检查普通边界和短边。
            return !synchronized && !boundaryVertices.count(tri.v[0]) && !boundaryVertices.count(tri.v[1]) &&
                !boundaryVertices.count(tri.v[2]) && metrics.minEdge>2*c.minSize && metrics.quality>c.maxRadiusEdgeRatio;
        };
        const auto evaluate=[&](int t,SurfaceTriangleTicket& ticket,bool& bad) {
                bad=false;
                observation.count(MeshCounter::TriangleVisits,1);
                const Tri2D& tri=mesh.tris[t];
                const Point2D a=mesh.points[tri.v[0]],b=mesh.points[tri.v[1]],d=mesh.points[tri.v[2]];
                const Point2D mid{(a.x+b.x+d.x)/3,(a.y+b.y+d.y)/3};
                if (!inDomain[t]) return true;
                // 退化极点处不同 UV 对应同一个物理节点；该图表三角形没有可细化的物理面积。
                const int ga=global[tri.v[0]],gb=global[tri.v[1]],gd=global[tri.v[2]];
                if ((ga>=0 && ga==gb) || (gb>=0 && gb==gd) || (gd>=0 && gd==ga)) return true;
                std::array<int,3> vertices{tri.v[0],tri.v[1],tri.v[2]};
                std::sort(vertices.begin(),vertices.end());
                auto found=measured.find(vertices);
                if (found==measured.end()) {
                    const Point3D pa=ga>=0?out.mesh.vertices[ga]:queries.point(a);
                    const Point3D pb=gb>=0?out.mesh.vertices[gb]:queries.point(b);
                    const Point3D pd=gd>=0?out.mesh.vertices[gd]:queries.point(d);
                    const Point3D pm=queries.point(mid);
                    const double h=std::min({targetAt(pm),targetAt(pa)*c.growthRatio,
                                             targetAt(pb)*c.growthRatio,targetAt(pd)*c.growthRatio});
                    if (!out.error.empty()) return false;
                    found=measured.emplace(vertices,CachedTriangleTest{measureTriangle(queries,{a,b,d},{pa,pb,pd}),h}).first;
                }
                // 单次面构造中节点位置、几何和尺寸场固定；换边或插点改变顶点键，未变化三角形复用已有 CAD 准则测量。
                const TriangleMetrics metrics=found->second.metrics;
                if (!metrics.valid) { out.error="non-finite CAD surface evaluation"; return false; }
                const double h=found->second.target;
                const bool badQuality=qualityViolation(tri,metrics);
                const bool hard=metrics.maxEdge>edgeLimit(h) || metrics.deviation>c.maxDeviation || metrics.normalAngle>c.maxNormalAngleDeg;
                bad=hard || metrics.inverted || badQuality;
                if (bad) {
                    const double hardScore=std::max({1.0,metrics.maxEdge/edgeLimit(h),metrics.deviation/c.maxDeviation,
                                                    metrics.normalAngle/c.maxNormalAngleDeg});
                    ticket={f,t,mesh.triangleVersion(t),state.criteriaVersions[f],(metrics.inverted || hard)?2:1,
                            (hard || metrics.inverted)?hardScore:metrics.quality/c.maxRadiusEdgeRatio,vertices};
                }
                return true;
        };
        const auto offer=[&](int t) {
            SurfaceTriangleTicket ticket; bool bad;
            if (!evaluate(t,ticket,bad)) return false;
            if (bad && !queue.offer(ticket,&out.error)) return false;
            return true;
        };
        const auto updateQueue=[&](const MeshEditDelta& delta) {
            queue.invalidate(delta);
            std::set<int> affected;
            for (int t:delta.changedSlots) if (t>=0 && t<int(mesh.tris.size())) {
                affected.insert(t); for (int n:mesh.tris[t].n) if (n>=0) affected.insert(n);
            }
            int visits=0;
            for (int t:affected) {
                if (visits++%64==0 && progress && !progress(f,nf)) { out.error="surface meshing cancelled during queue update"; return false; }
                if (!offer(t)) return false;
            }
            return true;
        };
        for (int t=0;t<int(mesh.tris.size());++t) {
            if (t%64==0 && progress && !progress(f,nf)) { out.error="surface meshing cancelled during initial criterion scan"; return out; }
            if (!offer(t)) return out;
        }
        while (true) {
            if (progress && !progress(f,nf)) { out.error="surface meshing cancelled during refinement queue"; return out; }
            SurfaceTriangleTicket selected;
            if (!queue.pop(mesh,selected)) {
                if (!requireLegalization) {
                    observation.count(MeshCounter::FullCriterionAudits,1);
                    measured.clear(); // 最后重新测全部有效片，核对队列维护有无遗漏。
                    for (int t=0;t<int(mesh.tris.size());++t) {
                        if (t%64==0 && progress && !progress(f,nf)) { out.error="surface meshing cancelled during full criterion audit"; return out; }
                        SurfaceTriangleTicket ticket; bool bad;
                        if (!evaluate(t,ticket,bad)) return out;
                        if (bad) {
                            out.error="refinement queue missed a violation on face "+std::to_string(f)+" triangle "+std::to_string(t); return out;
                        }
                    }
                    break;
                }
                MeshEditDelta legalized;
                if (!legalize(mesh,w.constraints,&error,progress,f,nf,state.steps,c.maxIterations,metric,state.work.delaunayFlips,-1,-1,observation,&legalized,&domain)) {
                    out.error="face "+std::to_string(f)+" final CDT: "+error; return out;
                }
                requireLegalization=false;
                if (!updateQueue(legalized)) return out;
                continue;
            }
            const int badTri=selected.slot;
            const auto tri=mesh.tris[badTri];
            const std::array<Point2D,3> badUV{mesh.points[tri.v[0]],mesh.points[tri.v[1]],mesh.points[tri.v[2]]};
            Point2D insertion{(badUV[0].x+badUV[1].x+badUV[2].x)/3,(badUV[0].y+badUV[1].y+badUV[2].y)/3};
            const auto measuredBad=measured.at(selected.vertexKey);
            const auto badMetrics=measuredBad.metrics;
            const double badEdgeLength=badMetrics.maxEdge,badDeviation=badMetrics.deviation,badAngle=badMetrics.normalAngle;
            const double badTarget=measuredBad.target,badQualityRatio=badMetrics.quality;
            const bool badInverted=badMetrics.inverted,badQualityTarget=qualityViolation(tri,badMetrics);
            state.work.refinementEdgeViolations+=badEdgeLength>edgeLimit(badTarget);
            state.work.refinementDeviationViolations+=badDeviation>c.maxDeviation;
            state.work.refinementNormalViolations+=badAngle>c.maxNormalAngleDeg;
            state.work.refinementQualityViolations+=badQualityTarget;
            if (!requireLegalization) ++state.work.violationsAfterLegalization;
            ++iterations;
            if (++state.steps>c.maxIterations || static_cast<int>(activeVertices())>=c.maxVertices) {
                out.error="surface refinement budget exceeded on face "+std::to_string(f)+
                    " (edge="+std::to_string(badEdgeLength)+
                    ", deviation="+std::to_string(badDeviation)+
                    ", normal angle="+std::to_string(badAngle)+
                    ", inverted="+std::to_string(badInverted)+
                    ", quality="+std::to_string(badQualityRatio)+
                    ", quality violation="+std::to_string(badQualityTarget)+
                    ", work="+std::to_string(state.steps)+
                    ", uv=("+std::to_string(badUV[0].x)+","+std::to_string(badUV[0].y)+") ("+
                    std::to_string(badUV[1].x)+","+std::to_string(badUV[1].y)+") ("+
                    std::to_string(badUV[2].x)+","+std::to_string(badUV[2].y)+"))";
                return out;
            }
            if (progress && iterations%64==0 && !progress(f,nf)) {
                out.error="surface meshing cancelled"; return out;
            }
            // 先尝试内部二分，保留有效邻片；要求反向子片数量减少或剩余反向跨度缩短，并重新检查全部子片。
            int repairA=-1,repairB=-1,bestInverted=5;
            double bestSpan=std::numeric_limits<double>::infinity();
            if (badInverted) {
                const auto tri=mesh.tris[badTri];
                for (int k=0;k<3;++k) {
                    const int a=tri.v[k],b=tri.v[(k+1)%3],neighbor=tri.n[k];
                    if (constrainedEdges.count(key(a,b)) || neighbor<0 || !inDomain[neighbor]) continue;
                    const Point2D ua=mesh.points[a],ub=mesh.points[b];
                    const Point2D midpoint{ua.x+(ub.x-ua.x)/2,ua.y+(ub.y-ua.y)/2};
                    if ((midpoint.x==ua.x && midpoint.y==ua.y) ||
                        (midpoint.x==ub.x && midpoint.y==ub.y)) continue;
                    const Point3D pm=queries.point(midpoint);
                    int before=0,after=0; double beforeSpan=0,afterSpan=0;
                    bool allowed=finite(pm);
                    for (int parent:{badTri,neighbor}) {
                        const auto& p=mesh.tris[parent];
                        int third=-1; for (int v:p.v) if (v!=a && v!=b) third=v;
                        if (third<0 || global[a]<0 || global[b]<0 || global[third]<0) { allowed=false; break; }
                        const Point2D uc=mesh.points[third];
                        const Point3D pa=out.mesh.vertices[global[a]],pb=out.mesh.vertices[global[b]],pc=out.mesh.vertices[global[third]];
                        std::array<Point2D,3> parentUV{ua,ub,uc};
                        std::array<Point3D,3> parentXYZ{pa,pb,pc};
                        if (orient2d(ua,ub,uc)<0) { std::swap(parentUV[0],parentUV[1]); std::swap(parentXYZ[0],parentXYZ[1]); }
                        const auto original=measureTriangle(queries,parentUV,parentXYZ);
                        before+=original.inverted;
                        if (original.inverted) beforeSpan=std::max(beforeSpan,original.maxEdge);
                        for (bool first:{true,false}) {
                            std::array<Point2D,3> childUV=first?std::array<Point2D,3>{ua,midpoint,uc}:
                                std::array<Point2D,3>{midpoint,ub,uc};
                            std::array<Point3D,3> childXYZ=first?std::array<Point3D,3>{pa,pm,pc}:
                                std::array<Point3D,3>{pm,pb,pc};
                            const int orientation=orient2d(childUV[0],childUV[1],childUV[2]);
                            if (orientation*orient2d(ua,ub,uc)<=0) { allowed=false; break; }
                            if (orientation<0) { std::swap(childUV[0],childUV[1]); std::swap(childXYZ[0],childXYZ[1]); }
                            const auto child=measureTriangle(queries,childUV,childXYZ);
                            if (!child.valid || (!original.inverted && child.inverted) ||
                                child.deviation>std::max(c.maxDeviation,original.deviation)*(1+1e-8) ||
                                child.normalAngle>std::max(c.maxNormalAngleDeg,original.normalAngle)*(1+1e-8) ||
                                norm3(cross3(minus3(childXYZ[1],childXYZ[0]),minus3(childXYZ[2],childXYZ[0])))<=1e-15*c.targetSize*c.targetSize) {
                                allowed=false; break;
                            }
                            after+=child.inverted;
                            if (child.inverted) afterSpan=std::max(afterSpan,child.maxEdge);
                        }
                        if (!allowed) break;
                    }
                    const bool improving=after<before || (after==before && afterSpan<beforeSpan*0.8);
                    if (allowed && improving && (after<bestInverted || (after==bestInverted && afterSpan<bestSpan))) {
                        repairA=a; repairB=b; bestInverted=after; bestSpan=afterSpan;
                    }
                }
            }
            // 细分 CAD 边会使全部关联面失效；从共享边表重建，使所有参数曲线分支使用同一节点身份。
            {
                const Tri2D& tri=mesh.tris[badTri];
                for (int k=0;k<3;++k) {
                    const int a=tri.v[k],b=tri.v[(k+1)%3];
                    if (!constrainedEdges.count(key(a,b)) || global[a]==global[b]) continue;
                    const Point2D ua=mesh.points[a],ub=mesh.points[b];
                    const Point3D pa=out.mesh.vertices[global[a]],pb=out.mesh.vertices[global[b]];
                    const Point2D middle{0.5*(ua.x+ub.x),0.5*(ua.y+ub.y)};
                    const Point3D pm=queries.point(middle);
                    const double h=std::min({targetAt(queries.point(insertion)),targetAt(pm),
                                            targetAt(pa)*c.growthRatio,targetAt(pb)*c.growthRatio});
                    if (!out.error.empty()) return out;
                    const bool boundaryViolation=length(pa,pb)>edgeLimit(h) ||
                        lineDeviation(pa,pb,pm)>c.maxDeviation ||
                        std::max({angleDeg(geo.faceNormal(f,ua),geo.faceNormal(f,middle)),
                                  angleDeg(geo.faceNormal(f,ub),geo.faceNormal(f,middle)),
                                  angleDeg(geo.faceNormal(f,ua),geo.faceNormal(f,ub))})>c.maxNormalAngleDeg;
                    // 优先处理真实边界违反；仅有反向片时尽可能选择能改善结果的内部修复。
                    if (!boundaryViolation && !badInverted) continue;
                    if (!boundaryViolation && repairA>=0) continue;
                    Point3D du,dv;
                    geo.faceDerivatives(f,{0.5*(ua.x+ub.x),0.5*(ua.y+ub.y)},du,dv);
                    if (!finite(du)||!finite(dv)) { out.error="non-finite surface metric"; return out; }
                    const auto mapped=[&](Point2D p) {
                        const double x=p.x-insertion.x,y=p.y-insertion.y;
                        return Point3D{du.x*x+dv.x*y,du.y*x+dv.y*y,du.z*x+dv.z*y};
                    };
                    // 使用局部 UV 度量；周期参数中相距较远的点可能在三维中很近，不能直接用弦球判断。
                    if (!badInverted && dot3(mapped(ua),mapped(ub))>=0) continue;
                    SurfaceRefinementSample sample;
                    if (badInverted && state.refinementSamples.size()<8) {
                        sample.face=f; sample.facet=snapshot(badTri);
                        for (int neighbor:tri.n) if (neighbor>=0 && inDomain[neighbor])
                            sample.neighbors.push_back(snapshot(neighbor));
                    }
                    if (!requestSharedSplit(global[a],global[b],"mapped facet on face "+std::to_string(f)+
                        " inverted="+std::to_string(badInverted)+" boundary violation="+std::to_string(boundaryViolation)+
                        " near UV ("+std::to_string(insertion.x)+","+std::to_string(insertion.y)+")",
                        f,sample.face>=0?&sample:nullptr))
                        out.error+="; mapped facet geometry on face "+std::to_string(f)+
                            " near UV ("+std::to_string(insertion.x)+","+std::to_string(insertion.y)+")";
                    return out;
                }
            }
            int newVertex=-1,newTri=-1;
            MeshEditDelta editDelta;
            std::array<int,3> parents{mesh.tris[badTri].v[0],mesh.tris[badTri].v[1],mesh.tris[badTri].v[2]};
            // 重心插点不会缩短长边，可能连续产生更薄的反向片；应二分该长边。
            if (repairA>=0) {
                if (!splitInteriorEdge(mesh,repairA,repairB,&newVertex,&error,observation,&editDelta,&domain)) {
                    out.error="face "+std::to_string(f)+" inversion repair: "+error; return out;
                }
                insertion=mesh.points[newVertex];
                parents={repairA,repairB,-1};
            } else if (edgeRefinement || badInverted || badEdgeLength>edgeLimit(badTarget) || badAngle>c.maxNormalAngleDeg) {
                const Tri2D tri=mesh.tris[badTri];
                std::vector<std::pair<double,std::pair<int,int>>> candidates;
                for (int k=0;k<3;++k) {
                    const int a=tri.v[k],b=tri.v[(k+1)%3];
                    if (constrainedEdges.count(key(a,b))) continue;
                    const Point2D ua=mesh.points[a],ub=mesh.points[b];
                    const Point3D pa=out.mesh.vertices[global[a]],pb=out.mesh.vertices[global[b]];
                    const double score=std::max(length(pa,pb)/badTarget,
                        angleDeg(geo.faceNormal(f,ua),geo.faceNormal(f,ub))/c.maxNormalAngleDeg);
                    candidates.push_back({score,{a,b}});
                }
                std::sort(candidates.begin(),candidates.end(),[](const auto& a,const auto& b){return a.first>b.first;});
                for (const auto& candidate:candidates) {
                    SurfaceEdgeSplitPoint balanced;
                    if(physicalReference&&!surfaceBalancedEdgePoint(queries,f,mesh.points[candidate.second.first],
                        mesh.points[candidate.second.second],balanced,&error)) {
                        out.error="face "+std::to_string(f)+" physical edge split: "+error;return out;
                    }
                    bool split=splitInteriorEdge(mesh,candidate.second.first,candidate.second.second,&newVertex,&error,observation,&editDelta,&domain,
                        physicalReference?&balanced.uv:nullptr);
                    if(!split&&physicalReference&&error.rfind("edge split cannot preserve UV triangle orientation",0)==0) {
                        // 极薄UV桥接片可能无法用double表示等距分点；保留方向谓词，显式尝试原分支中点。
                        if(balancedSplitRefusals++<4)out.diagnostics.push_back("face "+std::to_string(f)+
                            " balanced edge split refused; trying original UV midpoint: "+error);
                        error.clear();
                        split=splitInteriorEdge(mesh,candidate.second.first,candidate.second.second,&newVertex,&error,observation,&editDelta,&domain);
                    }
                    if (split) {
                        parents={candidate.second.first,candidate.second.second,-1}; break;
                    }
                }
                if (newVertex<0) {
                    tracePoints(badTri);
                    if (state.refinementSamples.size()<8) {
                        SurfaceRefinementSample sample; sample.face=f; sample.facet=snapshot(badTri);
                        for (int neighbor:tri.n) if (neighbor>=0 && inDomain[neighbor])
                            sample.neighbors.push_back(snapshot(neighbor));
                        state.refinementSamples.push_back(std::move(sample));
                    }
                    out.error="face "+std::to_string(f)+" edge refinement: "+error+
                        " (edge="+std::to_string(badEdgeLength)+", normal angle="+std::to_string(badAngle)+
                        ", inverted="+std::to_string(badInverted)+")"; return out;
                }
                insertion=mesh.points[newVertex];
            } else if(!insertMetricPoint(mesh,insertion,metric,w.constraints,badTri,&newVertex,&newTri,&error,observation,&editDelta,&domain)) {
                out.error="face "+std::to_string(f)+" refinement: "+error;return out;
            }
            if (newVertex<0) { out.error="refinement did not insert a point"; return out; }
            if (!domain.apply(mesh,editDelta,&error)) {
                out.error="face "+std::to_string(f)+" local domain update: "+error; return out;
            }
            pointParents.resize(mesh.points.size(),{-1,-1,-1});
            pointParents[newVertex]=parents;
            global.resize(mesh.points.size(),-1);
            global[newVertex]=allocate(queries.point(insertion));
            ++state.work.insertedSurfacePoints;
            if (!updateQueue(editDelta)) return out;
            MeshEditDelta legalized;
            if ((parents[2]<0||physicalReference) && !legalize(mesh,w.constraints,&error,progress,f,nf,state.steps,c.maxIterations,
                                          metric,state.work.delaunayFlips,newVertex,int(mesh.tris.size())-1,observation,&legalized,&domain,physicalCircle)) {
                out.error="face "+std::to_string(f)+" local CDT after edge split: "+error; return out;
            }
            if ((parents[2]<0||physicalReference) && !updateQueue(legalized)) return out;
            requireLegalization=!physicalReference;
        }
        if(balancedSplitRefusals)out.diagnostics.push_back("face "+std::to_string(f)+" balanced split UV orientation refusals: "+
            std::to_string(balancedSplitRefusals));
        }
        phase.change(observation,MeshStage::MeshAssembly);
        if (!domain.verify(mesh,w.constraints,&error,observation)) {
            out.error="face "+std::to_string(f)+" output domain classification: "+error;
            return out;
        }
        bool warnedPole=false,warnedZero=false;
        int remainingQuality=0,remainingTrim=0,remainingShort=0;
        std::unordered_set<int> trimNodes;
        for(auto e:w.constraints) {trimNodes.insert(e.first);trimNodes.insert(e.second);}
        for (size_t t=0;t<mesh.tris.size();++t) {
            const Tri2D& tri=mesh.tris[t];
            const Point2D a=mesh.points[tri.v[0]],b=mesh.points[tri.v[1]],d=mesh.points[tri.v[2]];
            const Point2D mid{(a.x+b.x+d.x)/3,(a.y+b.y+d.y)/3};
            if (!inDomain[t]) continue;
            std::array<int,3> ids{global[tri.v[0]],global[tri.v[1]],global[tri.v[2]]};
            if (ids[0]<0 || ids[1]<0 || ids[2]<0 || ids[0]==ids[1] || ids[1]==ids[2] || ids[2]==ids[0]) {
                if (ids[0]>=0 && ids[1]>=0 && ids[2]>=0 &&
                    (ids[0]==ids[1] || ids[1]==ids[2] || ids[2]==ids[0])) {
                    if (!warnedPole) out.diagnostics.push_back("collapsed pole triangles on face "+std::to_string(f));
                    warnedPole=true;
                    continue;
                }
                out.error="degenerate surface triangle on face "+std::to_string(f); return out;
            }
            const Point3D p=out.mesh.vertices[ids[0]],q=out.mesh.vertices[ids[1]],r=out.mesh.vertices[ids[2]];
            const Point3D n=cross3(minus3(q,p),minus3(r,p));
            if (norm3(n)<=1e-15*c.targetSize*c.targetSize) {
                if (!warnedZero) out.diagnostics.push_back("zero-area triangles on face "+std::to_string(f));
                warnedZero=true;
                continue;
            }
            std::array<Point2D,3> uv{a,b,d};
            if (geo.faceReversed(f)) { std::swap(ids[1],ids[2]); std::swap(uv[1],uv[2]); }
            out.mesh.triangles.push_back(ids);
            out.triangleFaceIds.push_back(f);
            out.triangleCornerUV.push_back(uv);
            std::array<int,3> vertices{tri.v[0],tri.v[1],tri.v[2]};
            std::sort(vertices.begin(),vertices.end());
            const auto found=measured.find(vertices);
            const TriangleMetrics metrics=found!=measured.end()?found->second.metrics:
                measureTriangle(queries,{a,b,d},{p,q,r});
            built.deviation=std::max(built.deviation,metrics.deviation);
            built.normalAngle=std::max(built.normalAngle,metrics.normalAngle);
            built.maxEdge=std::max(built.maxEdge,metrics.maxEdge);
            built.worstQuality=std::max(built.worstQuality,metrics.quality);
            if(metrics.quality>c.maxRadiusEdgeRatio) {
                ++remainingQuality;
                if(trimNodes.count(tri.v[0]) || trimNodes.count(tri.v[1]) || trimNodes.count(tri.v[2]))++remainingTrim;
                if(metrics.minEdge<=2*c.minSize)++remainingShort;
            }
            if (!metrics.valid) { out.error="non-finite surface sample on face "+std::to_string(f); return out; }
            if (metrics.inverted) { out.error="inverted mapped triangle on face "+std::to_string(f); return out; }
            if (c.maxSize>0 && metrics.maxEdge>c.maxSize*(1+1e-8)) {
                out.error="maximum surface size exceeded on face "+std::to_string(f)+
                    " (edge="+std::to_string(metrics.maxEdge)+", maximum="+std::to_string(c.maxSize)+")";
                return out;
            }
            out.measuredMaxEdgeLength=std::max(out.measuredMaxEdgeLength,metrics.maxEdge);
            out.measuredWorstRadiusEdgeRatio=std::max(out.measuredWorstRadiusEdgeRatio,metrics.quality);
            out.measuredMaxDeviation=std::max(out.measuredMaxDeviation,metrics.deviation);
            out.measuredMaxNormalAngleDeg=std::max(out.measuredMaxNormalAngleDeg,metrics.normalAngle);
            if (!warnedSmall && metrics.minEdge<c.minSize) {
                out.diagnostics.push_back("face "+std::to_string(f)+" has an edge below minSize near UV ("+
                    std::to_string(mid.x)+","+std::to_string(mid.y)+
                    "); trim features, poles or geometric refinement limit the regular size floor");
                warnedSmall=true;
            }
        }
        if(remainingQuality) out.diagnostics.push_back("face "+std::to_string(f)+" quality residual: "+std::to_string(remainingQuality)+
            " triangles above radius-edge target, "+std::to_string(remainingTrim)+" touching fixed trim nodes, "+
            std::to_string(remainingShort)+" below insertion size floor (overlapping counts); no accepted flip/relocation in final sweep or optimization sweep limit; worst ratio="+
            std::to_string(built.worstQuality));
        if (progress && !progress(f+1,nf)) { out.error="surface meshing cancelled"; return out; }
        built.triangles.assign(out.mesh.triangles.begin()+triangleStart,out.mesh.triangles.end());
        built.cornerUV.assign(out.triangleCornerUV.begin()+triangleStart,out.triangleCornerUV.end());
        built.diagnostics.assign(out.diagnostics.begin()+diagnosticStart,out.diagnostics.end());
        built.ready=true;built.edgeRefinement=edgeRefinement;built.geometryVersion=state.nextGeometryVersion++;cached=std::move(built);
        cached.chart=std::make_unique<Mesh2D>(std::move(mesh));
        cached.global=std::move(global);cached.constraints=std::move(w.constraints);
    }
    if(state.restart)return out;
    phase.change(globalObservation,MeshStage::Validation);
    if (out.mesh.triangles.empty()) { out.error="no surface triangles"; return out; }
    struct Incidence { int count=0; int direction=0; };
    std::unordered_map<uint64_t,Incidence> edges;
    std::unordered_map<uint64_t,size_t> cadIncidence;
    for (const auto& b:out.boundaryEdges)
        if (b.a!=b.b) cadIncidence[key(b.a,b.b)]=edgeUses[b.cadEdge].size();
    for (const auto& tri:out.mesh.triangles) for (int k=0;k<3;++k) {
        const int a=tri[k],b=tri[(k+1)%3];
        auto& i=edges[key(a,b)]; ++i.count; i.direction += a<b?1:-1;
    }
    for (const auto& [edge,i]:edges) {
        const auto boundary=cadIncidence.find(edge);
        if (i.count>2 || (i.count==2 && i.direction!=0) ||
            (i.count==1 && (boundary==cadIncidence.end() || boundary->second!=1)) ||
            (geo.expectsClosedSurface() && i.count!=2)) {
            out.error="surface edge incidence/orientation invalid on vertex edge "+
                std::to_string(int(edge>>32))+"-"+std::to_string(int(edge&0xffffffffu))+
                " (count="+std::to_string(i.count)+", direction="+
                std::to_string(i.direction)+")";
            for (const auto& b:out.boundaryEdges)
                if (key(b.a,b.b)==edge) out.error+=" CAD edge="+std::to_string(b.cadEdge);
            for (size_t t=0;t<out.mesh.triangles.size();++t) {
                const auto& tri=out.mesh.triangles[t];
                for (int k=0;k<3;++k)
                    if (key(tri[k],tri[(k+1)%3])==edge) {
                        out.error+=" face="+std::to_string(out.triangleFaceIds[t]);
                        const auto& uv=out.triangleCornerUV[t];
                        out.error+=" uv=("+std::to_string(uv[0].x)+","+std::to_string(uv[0].y)+") ("+
                            std::to_string(uv[1].x)+","+std::to_string(uv[1].y)+") ("+
                            std::to_string(uv[2].x)+","+std::to_string(uv[2].y)+")";
                    }
            }
            return out;
        }
    }
    for (const auto& b:out.boundaryEdges) {
        if (b.a==b.b) continue; // CAD 极点处的退化边。
        const auto actual=edges.find(key(b.a,b.b));
        if (actual==edges.end() || size_t(actual->second.count)!=edgeUses[b.cadEdge].size()) {
            out.error="CAD constraint segment incidence mismatch on edge "+std::to_string(b.cadEdge)+
                " between vertices "+std::to_string(b.a)+" and "+std::to_string(b.b);
            return out;
        }
    }
    if (out.measuredMaxDeviation>c.maxDeviation*(1+1e-8) ||
        out.measuredMaxNormalAngleDeg>c.maxNormalAngleDeg*(1+1e-8)) {
        out.error="sampled surface criteria not met (deviation="+
            std::to_string(out.measuredMaxDeviation)+", normal angle="+
            std::to_string(out.measuredMaxNormalAngleDeg)+")";
        return out;
    }
    if (out.measuredWorstRadiusEdgeRatio>c.maxRadiusEdgeRatio)
        out.diagnostics.push_back("quality target not fully met; see per-face residuals and actual optimization limits");
    if (progress && !progress(nf,nf)) { out.error="surface meshing cancelled"; return out; }
    SurfaceIntersection intersection;
    bool intersectionsValid;
    {
        MeshStageScope scope(globalObservation,MeshStage::IntersectionValidation);
        bool contact=false;
        intersectionsValid=validateCadIntersections(out,geo,&out.error,&intersection,&contact,&state);
        if(contact)out.diagnostics.push_back("assembly parts retained as independent body boundaries; contact/interference records do not form a conforming multi-region volume boundary");
    }
    if (!intersectionsValid) {
        if (intersection.firstTriangle>=0 && intersection.secondTriangle>=0) {
            out.error+=" (CAD faces "+std::to_string(out.triangleFaceIds[intersection.firstTriangle])+
                ", "+std::to_string(out.triangleFaceIds[intersection.secondTriangle])+")";
            // 单面采样误差不能保证窄间隙不穿越；相交反馈收紧关联面的误差。
            // 平面等零近似误差的真实相交不能靠加密修复，立即返回原诊断。
            std::map<int,double> limits;
            const double precisionFloor=64*std::numeric_limits<double>::epsilon()*std::max(diag,c.targetSize);
            for (int t:{intersection.firstTriangle,intersection.secondTriangle}) {
                const int f=out.triangleFaceIds[t];
                const auto& tri=out.mesh.triangles[t];
                const std::array<Point3D,3> xyz{out.mesh.vertices[tri[0]],out.mesh.vertices[tri[1]],out.mesh.vertices[tri[2]]};
                FaceEvaluationCache queries(geo,f,{observer,{2,f}});
                const auto metrics=measureTriangle(queries,out.triangleCornerUV[t],xyz);
                if (!metrics.valid) return out;
                const double next=std::min(state.faceDeviationLimits[f]*0.5,metrics.deviation*0.5);
                if (!(next>precisionFloor)) continue;
                auto entry=limits.emplace(f,next);
                entry.first->second=std::min(entry.first->second,next);
            }
            if (!limits.empty()) {
                if (++state.steps>c.maxIterations || int(activeVertices())>=c.maxVertices) {
                    out.error+="; intersection refinement budget exceeded"; return out;
                }
                for (const auto& [f,limit]:limits) {
                    state.faceDeviationLimits[f]=limit; state.faces[f].ready=false;
                    if (state.criteriaVersions[f]==std::numeric_limits<std::uint64_t>::max()) {
                        out.error="surface criteria version exhausted"; return out;
                    }
                    ++state.criteriaVersions[f];
                    // 重采样共享棱后，所有关联面重建，已有公共节点 ID 保留。
                    for (const auto& loop:topology[f].loops) for (const auto& use:loop) {
                        auto& record=state.boundary->edge(use.edge); state.boundary->invalidate(use.edge);
                        for (int adjacent:record.incidentFaces) state.faces[adjacent].ready=false;
                    }
                    if (limit<c.minSize*0.1 && state.work.intersectionRefinements==0)
                        state.diagnostics.push_back("nearby surface protection tightened the sampled tolerance; required geometric refinement may cross the regular minSize floor");
                    if (state.work.intersectionRefinements<32)
                        state.diagnostics.push_back("intersection protection tightened face "+std::to_string(f)+
                            " sampled deviation to "+std::to_string(limit));
                }
                ++state.work.intersectionRefinements;
                if(state.work.intersectionRefinements<=32)state.diagnostics.push_back("intersection refinement trigger: "+out.error);
                out.error.clear(); // 已安排后续恢复，当前状态不是终止失败。
                state.restart=true;
            } else out.error+="; approximation error is at the numerical precision floor; refinement cannot resolve this contact";
        }
        return out;
    }
    // 质量反馈仍需共享登记的原节点 ID，全部反馈结束后才压缩分配池。
    state.work.allocatedVertices=static_cast<int>(out.mesh.vertices.size());
    out.ok=true;
    return out;
}

/// 移除未使用的物理节点并同步面、边和标签编号。
static void compactCompletedSurface(SurfaceMeshResult& out,MeshingObserver* observer) {
    MeshStageScope assembly({observer,{}},MeshStage::MeshAssembly);
    std::vector<int> remap(out.mesh.vertices.size(),-1);
    for (const auto& t:out.mesh.triangles) for (int v:t) remap[v]=0;
    for (const auto& e:out.boundaryEdges) { remap[e.a]=0; remap[e.b]=0; }
    std::vector<Point3D> used;
    for (size_t v=0;v<remap.size();++v) if (remap[v]>=0) {
        remap[v]=static_cast<int>(used.size()); used.push_back(out.mesh.vertices[v]);
    }
    for (auto& t:out.mesh.triangles) for (int& v:t) v=remap[v];
    for (auto& e:out.boundaryEdges) { e.a=remap[e.a]; e.b=remap[e.b]; }
    out.mesh.vertices=std::move(used);
}

/// 在固定 CAD 边界下尝试改善内部节点邻域的物理形状。
static bool remeshPhysicalStars(Mesh2D& mesh,const std::vector<int>& global,const std::vector<Point3D>& xyz,
    SurfaceEvaluationCache& queries,const GeometrySource& geo,int face,const std::vector<std::pair<int,int>>& constraints,
    SurfaceDomainState& domain,const SurfaceCriteria& c,const std::function<double(Point3D)>& targetAt,
    const std::function<double(double)>& edgeLimit,MeshingProgress progress,int& steps,std::vector<std::string>& diagnostics,
    std::string* error,MeshObservation observation,const std::vector<CadBoundarySizingAnchor>& anchors){
    std::set<int> fixed;std::map<int,int> multiplicity;
    for(int id:global)if(id>=0)++multiplicity[id];
    const int cadVertices=geo.vertexCount();
    for(int v=0;v<int(global.size());++v)if(global[v]<cadVertices||multiplicity[global[v]]>1)fixed.insert(v);
    for(auto e:constraints){fixed.insert(e.first);fixed.insert(e.second);}
    const auto physical=[&](int v){return xyz.at(global.at(v));};
    SurfaceSizingPolicy sizing;
    if(!initializeSurfaceSizing(sizing,mesh,global,xyz,domain,constraints,anchors,c,error))return false;
    int removed=0,rejected=0;
    std::vector<std::vector<int>> incidentSlots;
    const auto rebuildIncident=[&]{incidentSlots.assign(mesh.points.size(),{});
        for(int t=0;t<int(mesh.tris.size());++t)for(int v:mesh.tris[t].v)incidentSlots[v].push_back(t);};
    bool incidentDirty=true;
    // 每轮只收集实际差片的内部节点；成功后从新连接重建下一轮候选。
    for(int sweep=0;sweep<4;++sweep){std::set<int> candidates;
        for(int t=0;t<int(mesh.tris.size());++t)if(domain.values()[t]){const auto tri=mesh.tris[t];bool eligible=true;
            for(int k=0;k<3;++k)if(global[tri.v[k]]<0||global[tri.v[k]]==global[tri.v[(k+1)%3]])eligible=false;
            if(eligible&&ratio(physical(tri.v[0]),physical(tri.v[1]),physical(tri.v[2]))>c.maxRadiusEdgeRatio)
                for(int v:tri.v)if(!fixed.count(v))candidates.insert(v);}
        bool changed=false;
        for(int v:candidates){
            if(progress&&!progress(face,geo.faceCount())){if(error)*error="surface meshing cancelled during local surface remesh";return false;}
            if(steps>=c.maxIterations){diagnostics.push_back("face "+std::to_string(face)+" local surface remesh budget exhausted");return true;}
            // 拒绝尝试不改变拓扑，复用邻接；成功时 removeTri 会压缩槽，下次重建。
            if(incidentDirty){rebuildIncident();incidentDirty=false;}
            double before=0;bool regular=true;int incident=0;
            for(int t:incidentSlots[v]){const auto tri=mesh.tris[t];
                ++incident;if(!domain.values()[t])regular=false;
                for(int k=0;k<3;++k)if(global[tri.v[k]]<0||global[tri.v[k]]==global[tri.v[(k+1)%3]])regular=false;
                if(regular)before=std::max(before,ratio(physical(tri.v[0]),physical(tri.v[1]),physical(tri.v[2])));}
            if(!regular||incident<3||incident>16||!std::isfinite(before)||before<=c.maxRadiusEdgeRatio)continue;
            ++steps;
            const auto cost=[&](std::array<int,3> ids){
                const std::array<Point2D,3> uv{mesh.points[ids[0]],mesh.points[ids[1]],mesh.points[ids[2]]};
                const std::array<Point3D,3> x{physical(ids[0]),physical(ids[1]),physical(ids[2])};
                for(int k=0;k<3;++k)if(!mesh.hasEdgeAny(ids[k],ids[(k+1)%3])&&length(x[k],x[(k+1)%3])<
                    std::max(sizing.floor(ids[k]),sizing.floor(ids[(k+1)%3])))
                    return std::numeric_limits<double>::infinity();
                double h=c.targetSize;Point3D center{};
                for(auto p:x){h=std::min(h,targetAt(p)*c.growthRatio);center.x+=p.x/3;center.y+=p.y/3;center.z+=p.z/3;}
                h=std::min(h,targetAt(center));const auto m=measureTriangle(queries,uv,x);
                if(!m.valid||m.inverted||m.maxEdge>edgeLimit(h)||m.deviation>c.maxDeviation||m.normalAngle>c.maxNormalAngleDeg||
                    m.quality>=before*(1-1e-6))return std::numeric_limits<double>::infinity();return m.quality;};
            MeshEditDelta delta;std::string reason;
            if(tryRemeshSurfaceVertex(mesh,v,constraints,domain,cost,&delta,&reason,observation,{},&incidentSlots[v])){
                ++removed;changed=true;incidentDirty=true;}
            else{if(rejected<8)diagnostics.push_back("face "+std::to_string(face)+" local remesh refused near UV ("+
                std::to_string(mesh.points[v].x)+","+std::to_string(mesh.points[v].y)+"): "+reason);++rejected;}
        }
        if(!changed)break;
        if(sweep==3)diagnostics.push_back("face "+std::to_string(face)+" local remesh reached four sweeps; remaining quality is measured below");
    }
    if(removed||rejected)diagnostics.push_back("face "+std::to_string(face)+" local surface remesh: "+std::to_string(removed)+
        " internal vertices removed, "+std::to_string(rejected)+" rejected attempts");
    return removed==0 || domain.verify(mesh,constraints,error,observation);
}

/// 对已完成的面网格执行限定次数的物理质量改善。
static bool optimizeCompletedSurface(SurfaceMeshResult& result,SurfaceRunState& state,const GeometrySource& geo,
    const MeshSizeField* sizeField,MeshingProgress progress,MeshingObserver* observer,bool cavityFallback=false,bool distributionOnly=false) {
    const auto base=state.resolvedCriteria;
    if(cavityFallback&&!distributionOnly&&base.reuseFailedQualityPasses&&
        std::all_of(state.faces.begin(),state.faces.end(),[&](const auto& f){return f.qualityOptimized&&
            (f.worstQuality<=base.maxRadiusEdgeRatio||f.failedQualityVersion==f.geometryVersion);})){
        for(int f=0;f<int(state.faces.size());++f)if(state.faces[f].worstQuality>base.maxRadiusEdgeRatio)
            MeshObservation{observer,{2,f}}.count(MeshCounter::QualityReusedFaces,1);
        return true;
    }
    // 几何细化已成功；质量移动引入跨面相交时保留可恢复的已接受表面。
    const MeshObservation snapshot{observer,{}};
    snapshot.count(MeshCounter::WholeMeshCopies,1);
    snapshot.count(MeshCounter::CopiedPoints,result.mesh.vertices.size());
    snapshot.count(MeshCounter::CopiedTriangles,result.mesh.triangles.size());
    auto originalVertices=result.mesh.vertices;auto originalTriangles=result.mesh.triangles;
    auto originalUV=result.triangleCornerUV;auto originalFaces=result.triangleFaceIds;
    auto originalInteractions=result.bodyInteractions;
    std::vector<std::uint64_t> previousVersions;for(const auto& face:state.faces)previousVersions.push_back(face.geometryVersion);
    std::vector<std::array<int,3>> triangles;
    std::vector<std::array<Point2D,3>> uv;
    std::vector<int> faces;
    int qualitySteps=state.steps;
    for(int f=0;f<int(state.faces.size());++f) {
        MeshStageScope qualityStage({observer,{2,f}},MeshStage::PhysicalQuality);
        auto& face=state.faces[f];auto c=base;c.maxDeviation=state.faceDeviationLimits[f];
        if(progress&&!progress(f,geo.faceCount())){result.error="surface meshing cancelled during completed quality phase";return false;}
        // 未受共享边或判据变化影响的面复用完整质量阶段结果；局部轮数上限仍明确诊断。
        const bool reusedFailure=cavityFallback&&!distributionOnly&&base.reuseFailedQualityPasses&&face.failedQualityVersion==face.geometryVersion;
        if(!distributionOnly&&face.qualityOptimized&&(!cavityFallback||face.worstQuality<=base.maxRadiusEdgeRatio||reusedFailure)){
            if(reusedFailure&&face.worstQuality>base.maxRadiusEdgeRatio)MeshObservation{observer,{2,f}}.count(MeshCounter::QualityReusedFaces,1);
            triangles.insert(triangles.end(),face.triangles.begin(),face.triangles.end());
            uv.insert(uv.end(),face.cornerUV.begin(),face.cornerUV.end());faces.insert(faces.end(),face.triangles.size(),f);continue;}
        auto& mesh=*face.chart;SurfaceEvaluationCache queries(geo,f,{observer,{2,f}});
        SurfaceDomainState domain;std::string error;
        if(!domain.initialize(mesh,face.constraints,&error)) {result.error=error;return false;}
        auto targetAt=[&](Point3D p) {return sizeField?std::clamp(sizeField->sizeAt(p),c.minSize,c.targetSize):c.targetSize;};
        auto edgeLimit=[&](double h){return c.maxSize>0?std::min(h*1.05,c.maxSize*(1+1e-8)):h*1.05;};
        int flips=0,moves=0;
        const size_t trianglesBeforeRemesh=mesh.tris.size();
        const int pointsBeforeCandidates=state.work.insertedSurfacePoints;
        if(distributionOnly){
            SurfaceSizingPolicy sizing;
            if(!initializeSurfaceSizing(sizing,mesh,face.global,result.mesh.vertices,domain,face.constraints,face.qualityAnchors,c,&error)){
                result.error="shape distribution sizing: "+error;return false;}
            if(qualitySteps<base.maxIterations&&!improvePhysicalQuality(mesh,face.global,result.mesh.vertices,queries,geo,f,
                face.constraints,domain,c,targetAt,edgeLimit,progress,geo.faceCount(),qualitySteps,flips,moves,&error,
                {observer,{2,f}},false,false,true,&sizing)&&error.find("budget")==std::string::npos){result.error=error;return false;}
            if(qualitySteps>=base.maxIterations&&error.empty())error="shape distribution budget exhausted after residual repair";
            if(!error.empty())result.diagnostics.push_back("face "+std::to_string(f)+" shape distribution: "+error);
        }else if(!cavityFallback){
            // 软质量阶段不在几何完成前消耗硬细化预算；预算耗尽时明确保留未解决的质量问题。
            bool optimized=true;
            if(qualitySteps<base.maxIterations)optimized=improvePhysicalQuality(mesh,face.global,result.mesh.vertices,queries,geo,f,
                face.constraints,domain,c,targetAt,edgeLimit,progress,geo.faceCount(),qualitySteps,flips,moves,&error,{observer,{2,f}});
            else error="physical quality budget unavailable after completed geometric refinement";
            if(!optimized && error.find("budget")==std::string::npos) {result.error=error;return false;}
            if(!error.empty())result.diagnostics.push_back("face "+std::to_string(f)+": "+error);
            error.clear();
            if(qualitySteps<base.maxIterations && !refinePhysicalCandidates(mesh,face.global,result.mesh.vertices,queries,geo,f,
                face.constraints,domain,c,targetAt,edgeLimit,progress,qualitySteps,state.work,result.diagnostics,&error,{observer,{2,f}},face.qualityAnchors)) {
                result.error="face "+std::to_string(f)+" physical candidate refinement: "+error;return false;
            }
            if(state.work.insertedSurfacePoints>pointsBeforeCandidates && qualitySteps<base.maxIterations && !improvePhysicalQuality(mesh,face.global,result.mesh.vertices,queries,geo,f,
                face.constraints,domain,c,targetAt,edgeLimit,progress,geo.faceCount(),qualitySteps,flips,moves,&error,{observer,{2,f}}) &&
                error.find("budget")==std::string::npos){result.error=error;return false;}
            // 常规生成/优化全部完成后再使用形状目标，避免改变先前生成路径并陷入较差布局。
            error.clear();
            if(qualitySteps<base.maxIterations && !improvePhysicalQuality(mesh,face.global,result.mesh.vertices,queries,geo,f,
                face.constraints,domain,c,targetAt,edgeLimit,progress,geo.faceCount(),qualitySteps,flips,moves,&error,{observer,{2,f}},true) &&
                error.find("budget")==std::string::npos){result.error=error;return false;}
            if(!error.empty())result.diagnostics.push_back("face "+std::to_string(f)+" physical shape targets: "+error);
            error.clear();
            if(!remeshPhysicalStars(mesh,face.global,result.mesh.vertices,queries,geo,f,face.constraints,domain,c,targetAt,edgeLimit,
                progress,qualitySteps,result.diagnostics,&error,{observer,{2,f}},face.qualityAnchors)){result.error=error;return false;}
            if(mesh.tris.size()<trianglesBeforeRemesh && qualitySteps<base.maxIterations && !improvePhysicalQuality(mesh,face.global,result.mesh.vertices,queries,geo,f,
                face.constraints,domain,c,targetAt,edgeLimit,progress,geo.faceCount(),qualitySteps,flips,moves,&error,{observer,{2,f}},true) &&
                error.find("budget")==std::string::npos){result.error=error;return false;}
            if(!error.empty())result.diagnostics.push_back("face "+std::to_string(f)+" post-remesh quality: "+error);
        }
        bool residual=false;for(int t=0;t<int(mesh.tris.size());++t)if(domain.values()[t]){auto tri=mesh.tris[t];
            if(face.global[tri.v[0]]>=0&&face.global[tri.v[1]]>=0&&face.global[tri.v[2]]>=0&&
                ratio(result.mesh.vertices[face.global[tri.v[0]]],result.mesh.vertices[face.global[tri.v[1]]],
                    result.mesh.vertices[face.global[tri.v[2]]])>c.maxRadiusEdgeRatio){residual=true;break;}}
        if(cavityFallback&&!distributionOnly&&residual&&qualitySteps<base.maxIterations){
            for(int sweep=0;sweep<4&&qualitySteps<base.maxIterations;++sweep){
                error.clear();const int beforeFallback=state.work.insertedSurfacePoints,beforeFlips=flips,beforeMoves=moves;
                if(!refinePhysicalCandidates(mesh,face.global,result.mesh.vertices,queries,geo,f,face.constraints,domain,c,targetAt,edgeLimit,
                    progress,qualitySteps,state.work,result.diagnostics,&error,{observer,{2,f}},face.qualityAnchors,true)){result.error=error;return false;}
                if(qualitySteps<base.maxIterations&&!improvePhysicalQuality(mesh,face.global,result.mesh.vertices,queries,geo,f,
                    face.constraints,domain,c,targetAt,edgeLimit,progress,geo.faceCount(),qualitySteps,flips,moves,&error,{observer,{2,f}},true,true)&&
                    error.find("budget")==std::string::npos){result.error=error;return false;}
                if(!error.empty())result.diagnostics.push_back("face "+std::to_string(f)+" physical cavity fallback: "+error);
                if(state.work.insertedSurfacePoints==beforeFallback&&flips==beforeFlips&&moves==beforeMoves)break;
                if(sweep==3)result.diagnostics.push_back("face "+std::to_string(f)+" physical cavity fallback reached four sweeps; remaining quality is measured below");
            }
        }
        face.qualityOptimized=true;
        const bool edited=flips||moves||state.work.insertedSurfacePoints!=pointsBeforeCandidates||mesh.tris.size()!=trianglesBeforeRemesh;
        if(edited)
            face.geometryVersion=state.nextGeometryVersion++;
        if(cavityFallback&&!distributionOnly)face.failedQualityVersion=edited?0:face.geometryVersion;
        state.work.qualityFlips+=flips;state.work.qualityMoves+=moves;
        for(int t=0;t<int(mesh.tris.size());++t) {
            if(!domain.values()[t])continue;
            auto tri=mesh.tris[t];std::array<int,3> ids;std::array<Point2D,3> corner;
            for(int k=0;k<3;++k){ids[k]=face.global[tri.v[k]];corner[k]=mesh.points[tri.v[k]];}
            if(ids[0]<0 || ids[1]<0 || ids[2]<0 || ids[0]==ids[1] || ids[1]==ids[2] || ids[2]==ids[0])continue;
            if(norm3(cross3(minus3(result.mesh.vertices[ids[1]],result.mesh.vertices[ids[0]]),
                minus3(result.mesh.vertices[ids[2]],result.mesh.vertices[ids[0]])))<=1e-15*c.targetSize*c.targetSize)continue;
            if(geo.faceReversed(f)){std::swap(ids[1],ids[2]);std::swap(corner[1],corner[2]);}
            triangles.push_back(ids);uv.push_back(corner);faces.push_back(f);
        }
    }
    state.steps=std::min(qualitySteps,base.maxIterations);
    result.mesh.triangles=std::move(triangles);result.triangleCornerUV=std::move(uv);result.triangleFaceIds=std::move(faces);
    std::string error;
    result.bodyInteractions.clear();
    bool acceptedSurface;
    bool acceptedQuality=true;
    if(cavityFallback){size_t previousBad=0,nextBad=0;double previousWorst=0,nextWorst=0;
        for(auto tri:originalTriangles){const double q=ratio(originalVertices[tri[0]],originalVertices[tri[1]],originalVertices[tri[2]]);
            previousWorst=std::max(previousWorst,q);previousBad+=q>base.maxRadiusEdgeRatio;}
        for(auto tri:result.mesh.triangles){const double q=ratio(result.mesh.vertices[tri[0]],result.mesh.vertices[tri[1]],result.mesh.vertices[tri[2]]);
            nextWorst=std::max(nextWorst,q);nextBad+=q>base.maxRadiusEdgeRatio;}
        acceptedQuality=nextBad<=previousBad&&nextWorst<=previousWorst*(1+1e-10);
        if(!acceptedQuality)error="physical cavity fallback worsened final quality: bad "+std::to_string(previousBad)+
            " to "+std::to_string(nextBad)+", worst "+std::to_string(previousWorst)+" to "+std::to_string(nextWorst);
    }
    {
        MeshStageScope acceptanceStage({observer,{}},MeshStage::IntersectionValidation);
        acceptedSurface=acceptedQuality&&(!geo.expectsClosedSurface() || isWatertight(result.mesh,&error)) && validateCadIntersections(result,geo,&error,nullptr,nullptr,&state);
    }
    if(!acceptedSurface) {
        // 未改面的本地拒绝不依赖其他面是否回滚；只失效本轮实际编辑面的记录。
        for(size_t f=0;f<state.faces.size();++f)if(previousVersions[f]!=state.faces[f].geometryVersion)
            state.faces[f].failedQualityVersion=0;
        result.mesh.vertices=std::move(originalVertices);result.mesh.triangles=std::move(originalTriangles);
        result.triangleCornerUV=std::move(originalUV);
        result.triangleFaceIds=std::move(originalFaces);
        result.bodyInteractions=std::move(originalInteractions);
        result.diagnostics.push_back("physical quality changes rolled back after final acceptance: "+error);
    }
    result.diagnostics.erase(std::remove_if(result.diagnostics.begin(),result.diagnostics.end(),[](const std::string& d){
        return d.find("quality residual:")!=std::string::npos || d.find("quality target not fully met;")!=std::string::npos;
    }),result.diagnostics.end());
    result.measuredWorstRadiusEdgeRatio=result.measuredMaxEdgeLength=result.measuredMaxDeviation=result.measuredMaxNormalAngleDeg=0;
    std::vector<int> bad(geo.faceCount(),0),trimBad(geo.faceCount(),0),shortBad(geo.faceCount(),0);
    std::vector<std::unique_ptr<SurfaceEvaluationCache>> finalQueries;
    finalQueries.resize(geo.faceCount());
    std::unordered_set<int> trimNodes;for(auto b:result.boundaryEdges){trimNodes.insert(b.a);trimNodes.insert(b.b);}
    // 反馈重建前保留未失效面的已优化连接和节点，缓存与 chart/global 同步。
    const auto unchanged=[&](int face){return cavityFallback&&acceptedSurface&&previousVersions[face]==state.faces[face].geometryVersion;};
    if(acceptedSurface)for(int f=0;f<geo.faceCount();++f)if(!unchanged(f)){auto& cache=state.faces[f];
        cache.triangles.clear();cache.cornerUV.clear();cache.deviation=cache.normalAngle=cache.maxEdge=cache.worstQuality=0;}
    for(size_t t=0;t<result.mesh.triangles.size();++t) {
        const int f=result.triangleFaceIds[t];auto ids=result.mesh.triangles[t];auto corners=result.triangleCornerUV[t];
        if(geo.faceReversed(f)){std::swap(ids[1],ids[2]);std::swap(corners[1],corners[2]);}
        TriangleMetrics m;
        if(unchanged(f)){
            // 同一运行中几何/判据未变，连接与UV直接来自已验收缓存；复用该面的采样上限。
            const auto& cache=state.faces[f];m.valid=true;m.inverted=false;
            m.maxEdge=cache.maxEdge;m.deviation=cache.deviation;m.normalAngle=cache.normalAngle;
            const auto a=result.mesh.vertices[ids[0]],b=result.mesh.vertices[ids[1]],d=result.mesh.vertices[ids[2]];
            m.minEdge=std::min({length(a,b),length(b,d),length(d,a)});m.quality=ratio(a,b,d);
        }else{
            if(!finalQueries[f])finalQueries[f]=std::make_unique<SurfaceEvaluationCache>(geo,f,MeshObservation{observer,{2,f}});
            m=measureTriangle(*finalQueries[f],corners,{result.mesh.vertices[ids[0]],result.mesh.vertices[ids[1]],result.mesh.vertices[ids[2]]});
        }
        if(!m.valid || m.inverted || m.maxEdge>(base.maxSize>0?base.maxSize*(1+1e-8):base.targetSize*1.05) ||
            m.deviation>state.faceDeviationLimits[f]*(1+1e-8) || m.normalAngle>base.maxNormalAngleDeg*(1+1e-8)) {
            result.error="completed quality phase violated a hard geometric criterion on face "+std::to_string(f);return false;
        }
        result.measuredMaxEdgeLength=std::max(result.measuredMaxEdgeLength,m.maxEdge);
        result.measuredMaxDeviation=std::max(result.measuredMaxDeviation,m.deviation);
        result.measuredMaxNormalAngleDeg=std::max(result.measuredMaxNormalAngleDeg,m.normalAngle);
        result.measuredWorstRadiusEdgeRatio=std::max(result.measuredWorstRadiusEdgeRatio,m.quality);
        if(acceptedSurface&&!unchanged(f)){auto& cache=state.faces[f];cache.triangles.push_back(result.mesh.triangles[t]);
            cache.cornerUV.push_back(result.triangleCornerUV[t]);cache.deviation=std::max(cache.deviation,m.deviation);
            cache.normalAngle=std::max(cache.normalAngle,m.normalAngle);cache.maxEdge=std::max(cache.maxEdge,m.maxEdge);
            cache.worstQuality=std::max(cache.worstQuality,m.quality);}
        if(m.quality>base.maxRadiusEdgeRatio){++bad[f];if(m.minEdge<base.minSize)++shortBad[f];
            if(trimNodes.count(ids[0]) || trimNodes.count(ids[1]) || trimNodes.count(ids[2]))++trimBad[f];}
    }
    for(int f=0;f<geo.faceCount();++f)if(bad[f])result.diagnostics.push_back("face "+std::to_string(f)+" quality residual: "+
        std::to_string(bad[f])+" above target, "+std::to_string(trimBad[f])+" touching fixed trim, "+
        std::to_string(shortBad[f])+" below minSize (overlapping observations, not a proof of unavoidable geometry)");
    if(acceptedSurface)for(int f=0;f<geo.faceCount();++f){auto& cache=state.faces[f];
        cache.diagnostics.erase(std::remove_if(cache.diagnostics.begin(),cache.diagnostics.end(),[](const std::string& d){
            return d.find("quality residual:")!=std::string::npos;}),cache.diagnostics.end());
        std::set<std::string> seen(cache.diagnostics.begin(),cache.diagnostics.end());const std::string prefix="face "+std::to_string(f);
        for(const auto& d:result.diagnostics)if((d.rfind(prefix+" ",0)==0||d.rfind(prefix+":",0)==0)&&seen.insert(d).second)
            cache.diagnostics.push_back(d);
    }
    return true;
}

/// 根据已完成网格的质量反馈请求必要的共享边细分。
static bool requestCompletedBoundaryRefinement(SurfaceMeshResult& result,SurfaceRunState& state,
    const GeometrySource& geo,MeshingProgress progress,MeshingObserver* observer,bool offsetCandidates=false){
    // 回滚后的 result 与已试过修改的局部 chart 不再对应，禁止继续复用图表反馈。
    for(const auto& d:result.diagnostics)if(d.rfind("physical quality changes rolled back",0)==0)return true;
    const auto& c=state.resolvedCriteria;
    if(state.steps>=c.maxIterations)return true;
    struct Interval{int edge;double left,right;};std::unordered_map<uint64_t,Interval> intervals;
    std::unordered_map<int,std::vector<std::pair<int,int>>> trimNeighbors;
    for(int e=0;e<geo.edgeCount();++e){const auto& seq=state.boundary->edge(e).samples;
        for(size_t j=1;j<seq.size();++j)if(seq[j-1].global!=seq[j].global){
            intervals.emplace(key(seq[j-1].global,seq[j].global),Interval{e,seq[j-1].t,seq[j].t});
            trimNeighbors[seq[j-1].global].push_back({seq[j].global,e});
            trimNeighbors[seq[j].global].push_back({seq[j-1].global,e});}}
    std::set<std::tuple<int,double,double>> requested;
    struct QualityFloor { std::unordered_map<int,double> values; };
    std::map<int,QualityFloor> faceFloors;
    const auto localFloor=[&](int face,int vertex){auto found=faceFloors.find(face);
        if(found==faceFloors.end()){
            QualityFloor field;const auto& cached=state.faces[face];
            if(std::any_of(cached.qualityAnchors.begin(),cached.qualityAnchors.end(),[&](auto a){return a.size<c.minSize;})){
                SurfaceDomainState domain;std::string error;
                if(!domain.initialize(*cached.chart,cached.constraints,&error,{observer,{2,face}})){
                    result.error="completed boundary quality domain: "+error;return c.minSize;}
                SurfaceSizingPolicy sizing;
                if(!initializeSurfaceSizing(sizing,*cached.chart,cached.global,result.mesh.vertices,domain,cached.constraints,
                    cached.qualityAnchors,c,&error)){result.error="completed boundary quality floor: "+error;return c.minSize;}
                for(int v=0;v<int(cached.global.size());++v)if(cached.global[v]>=0){auto [it,added]=field.values.emplace(cached.global[v],sizing.floor(v));
                    if(!added)it->second=std::min(it->second,sizing.floor(v));}
            }
            found=faceFloors.emplace(face,std::move(field)).first;
        }
        const auto value=found->second.values.find(vertex);return value==found->second.values.end()?c.minSize:value->second;
    };
    for(size_t t=0;t<result.mesh.triangles.size();++t){
        if(t%256==0&&progress&&!progress(result.triangleFaceIds[t],geo.faceCount())){
            result.error="surface meshing cancelled during completed boundary quality feedback";return false;}
        const auto tri=result.mesh.triangles[t];const auto a=result.mesh.vertices[tri[0]],b=result.mesh.vertices[tri[1]],d=result.mesh.vertices[tri[2]];
        if(ratio(a,b,d)<=c.maxRadiusEdgeRatio)continue;
        for(int k=0;k<3;++k){const auto found=intervals.find(key(tri[k],tri[(k+1)%3]));if(found==intervals.end())continue;
            const auto interval=found->second;const auto id=std::make_tuple(interval.edge,interval.left,interval.right);
            if(requested.count(id))continue;
            const auto first=result.mesh.vertices[tri[k]],second=result.mesh.vertices[tri[(k+1)%3]],opposite=result.mesh.vertices[tri[(k+2)%3]];
            MeshStageScope query({observer,{1,interval.edge}},MeshStage::GeometryEvaluation);
            double selected=0;bool admissible=false;const int face=result.triangleFaceIds[t];
            const auto fractions=offsetCandidates?std::vector<double>{.5,.45,.55,.4,.6}:std::vector<double>{.5};
            for(double fraction:fractions){const double parameter=interval.left+(interval.right-interval.left)*fraction;
                const auto mid=geo.edgePoint(interval.edge,parameter);
                if(!finite(mid)){result.error="non-finite completed boundary quality split point";return false;}
                double floor=c.minSize;for(int v:tri)floor=std::min(floor,localFloor(face,v)+
                    (SurfaceSizingPolicy::floorGrowthRatio(c.growthRatio)-1)*length(mid,result.mesh.vertices[v]));
                if(!result.error.empty())return false;
                if(qualityBoundarySplitAllowed(first,second,opposite,mid,floor,c.maxRadiusEdgeRatio)){
                    selected=parameter;admissible=true;break;}
                // 窄带两侧需协调分点；独立两子片验收会拒绝直角带的临时布局。
                // 证据限定为本面域内面片 + 对顶点另一条平行 CAD 裁剪段，整批仍全量验收。
                if(fraction==.5){const auto neighbors=trimNeighbors.find(tri[(k+2)%3]);
                    if(neighbors!=trimNeighbors.end())for(auto [node,edge]:neighbors->second){
                        const auto& incident=state.boundary->edge(edge).incidentFaces;
                        if(edge==interval.edge||std::find(incident.begin(),incident.end(),face)==incident.end())continue;
                        if(narrowBoundarySplitTrialAllowed(first,second,opposite,result.mesh.vertices[node],mid,floor,c.maxRadiusEdgeRatio)){
                            selected=parameter;admissible=true;break;}}
                    if(admissible)break;}
            }
            if(!admissible){const auto neighbors=trimNeighbors.find(tri[(k+2)%3]);
                if(neighbors!=trimNeighbors.end())for(auto [node,edge]:neighbors->second){
                    const auto& incident=state.boundary->edge(edge).incidentFaces;
                    if(edge==interval.edge||std::find(incident.begin(),incident.end(),face)==incident.end())continue;
                    const auto coarse=intervals.find(key(tri[(k+2)%3],node));if(coarse==intervals.end())continue;
                    const auto other=coarse->second;const auto otherId=std::make_tuple(other.edge,other.left,other.right);
                    if(requested.count(otherId))continue;
                    const double parameter=other.left+(other.right-other.left)*.5;const auto mid=geo.edgePoint(other.edge,parameter);
                    // CAD 查询异常不能当作普通的形状候选拒绝。
                    if(!finite(mid)){result.error="non-finite narrow-boundary alignment point";return false;}
                    double floor=c.minSize;for(int v:{tri[0],tri[1],tri[2],node})floor=std::min(floor,localFloor(face,v)+
                        (SurfaceSizingPolicy::floorGrowthRatio(c.growthRatio)-1)*length(mid,result.mesh.vertices[v]));
                    if(!result.error.empty())return false;
                    if(!narrowBoundaryAlignmentTrialAllowed(first,second,opposite,result.mesh.vertices[node],mid,floor,c.maxRadiusEdgeRatio))continue;
                    std::uint64_t request=0;
                    if(!state.boundary->enqueueSplit(other.edge,other.left,other.right,face,
                        "completed narrow-boundary alignment on face "+std::to_string(face),request,&result.error,parameter,CadBoundarySplitPurpose::Quality))return false;
                    requested.insert(otherId);state.restart=true;state.boundaryQualityRestart=true;
                    MeshObservation{observer,{2,face}}.count(MeshCounter::NarrowBoundaryAlignmentRequests,1);
                }
                continue;}
            std::uint64_t request=0;
            if(!state.boundary->enqueueSplit(interval.edge,interval.left,interval.right,result.triangleFaceIds[t],
                "completed physical quality boundary feedback on face "+std::to_string(result.triangleFaceIds[t]),request,&result.error,selected,CadBoundarySplitPurpose::Quality))return false;
            requested.insert(id);state.restart=true;state.boundaryQualityRestart=true;
        }
    }
    return true;
}

/// 编排 CAD 表面生成，处理共享边变化及后续质量检查。
SurfaceMeshResult generateCadSurfaceMesh(const GeometrySource& geometry,
                                         const SurfaceCriteria& criteria,
                                         const MeshSizeField* sizeField,
                                         MeshingProgress progress,MeshingObserver* observer,AssemblySurfacePolicy assemblyPolicy) {
    if(criteria.crossSmoothCadSplits) {
        SurfacePartitionPolicy policy;policy.enabled=true;
        return generatePartitionedCadSurfaceMesh(geometry,criteria,policy,sizeField,std::move(progress),observer,assemblyPolicy);
    }
    MeshStageScope generation({observer,{}},MeshStage::Generation);
    SurfaceRunState state;
    state.assemblyPolicy=assemblyPolicy;
    std::unique_ptr<SurfaceMeshResult> boundaryTrialBaseline;
    const auto badCount=[&](const SurfaceMeshResult& mesh){size_t count=0;
        for(auto t:mesh.mesh.triangles)if(ratio(mesh.mesh.vertices[t[0]],mesh.mesh.vertices[t[1]],mesh.mesh.vertices[t[2]])>
            state.resolvedCriteria.maxRadiusEdgeRatio)++count;return count;};
    const auto qualityPenalty=[&](const SurfaceMeshResult& mesh){double sum=0;
        for(auto t:mesh.mesh.triangles){const double excess=std::max(0.,ratio(mesh.mesh.vertices[t[0]],mesh.mesh.vertices[t[1]],mesh.mesh.vertices[t[2]])-
            state.resolvedCriteria.maxRadiusEdgeRatio);sum+=excess*excess;}return sum;};
    while (true) {
        state.restart=false;
        state.boundaryQualityRestart=false;
        SurfaceMeshResult result=generateSurfacePass(geometry,criteria,sizeField,progress,state,observer);
        bool refusedBoundaryTrial=false;
        if(result.ok){
            if(!optimizeCompletedSurface(result,state,geometry,sizeField,progress,observer))result.ok=false;
            if(result.ok && boundaryTrialBaseline){
                const auto previousBad=badCount(*boundaryTrialBaseline),currentBad=badCount(result);
                const double previousWorst=boundaryTrialBaseline->measuredWorstRadiusEdgeRatio,currentWorst=result.measuredWorstRadiusEdgeRatio;
                const double previousPenalty=qualityPenalty(*boundaryTrialBaseline),currentPenalty=qualityPenalty(result);
                if(currentBad>previousBad || currentWorst>previousWorst*(1+1e-10) ||
                    (currentBad==previousBad && currentWorst>=previousWorst*(1-1e-6)&&
                        currentPenalty>=previousPenalty*(1-1e-8))){
                    std::vector<std::string> synchronization;
                    for(const auto& d:result.diagnostics)if(d.find("local boundary synchronization refused:")!=std::string::npos||
                        d.find("shared boundary locally synchronized:")!=std::string::npos)synchronization.push_back("rolled-back trial: "+d);
                    result=std::move(*boundaryTrialBaseline);refusedBoundaryTrial=true;
                    result.diagnostics.insert(result.diagnostics.end(),synchronization.begin(),synchronization.end());
                    result.diagnostics.push_back("completed boundary quality trial rolled back: previous worst="+
                        std::to_string(previousWorst)+" bad="+std::to_string(previousBad)+", trial worst="+
                        std::to_string(currentWorst)+" bad="+std::to_string(currentBad)+
                        "; trial shared-edge/node changes are absent from returned mesh");
                }else result.diagnostics.push_back("completed boundary quality trial accepted: bad "+std::to_string(previousBad)+" to "+
                    std::to_string(currentBad)+", excess-squared sum "+std::to_string(previousPenalty)+" to "+std::to_string(currentPenalty));
                boundaryTrialBaseline.reset();
            }
            if(result.ok&&!refusedBoundaryTrial&&!requestCompletedBoundaryRefinement(result,state,geometry,progress,observer))result.ok=false;
            if(result.ok&&!state.restart&&!refusedBoundaryTrial&&badCount(result)>0&&state.steps<state.resolvedCriteria.maxIterations&&
                std::none_of(result.diagnostics.begin(),result.diagnostics.end(),[](const auto& d){
                    return d.rfind("physical quality changes rolled back",0)==0;})){
                // 原共享边反馈结束后补充修复；偏移分点仍经共享节点同步与整体试验回滚。
                if(!optimizeCompletedSurface(result,state,geometry,sizeField,progress,observer,true))result.ok=false;
                if(result.ok&&!requestCompletedBoundaryRefinement(result,state,geometry,progress,observer,true))result.ok=false;
            }
            if(result.ok && state.restart){
                // 只在确有共享棱试验时保留验收网格；重建结束后立即比较/释放。
                boundaryTrialBaseline=std::make_unique<SurfaceMeshResult>(result);
                const MeshObservation observation{observer,{}};
                observation.count(MeshCounter::WholeMeshCopies,1);observation.count(MeshCounter::CopiedPoints,result.mesh.vertices.size());
                observation.count(MeshCounter::CopiedTriangles,result.mesh.triangles.size());
            }
        }
        if(state.restart && !result.error.empty())state.restart=false;
        if (state.restart) {
            // 共享棱请求优先于下一个面候选；只处理当前边界版本，参数节点仅登记一次。
            CadBoundaryRegistry::SplitRequest request;
            std::vector<CadBoundaryRegistry::SplitRequest> requests;
            while (state.boundary->popSplitRequest(request)) {
                if (progress && !progress(0,geometry.faceCount())) {
                    result.error="surface meshing cancelled during shared boundary queue"; state.restart=false; break;
                }
                if (++state.steps>criteria.maxIterations) {
                    result.error="shared edge refinement budget exceeded at CAD edge "+std::to_string(request.edge)+
                        " parameters "+std::to_string(request.left)+" to "+std::to_string(request.right)+
                        " source face="+std::to_string(request.sourceFace);
                    state.restart=false; break;
                }
                requests.push_back(std::move(request));
            }
            std::vector<double> parameters;
            if(state.restart && !state.boundary->applySplitBatch(requests,parameters,&result.error))state.restart=false;
            std::set<int> geometrySourceFaces;
            if(criteria.pointPlacement==SurfacePointPlacement::ReferenceFrontal||criteria.pointPlacement==SurfacePointPlacement::PhysicalReferenceFrontal)
                for(const auto& request:requests)if(request.purpose==CadBoundarySplitPurpose::Geometry)
                    for(int face:state.boundary->edge(request.edge).incidentFaces)geometrySourceFaces.insert(face);
            if(state.restart)for(const auto& request:requests) {
                MeshStageScope edgeUpdate({observer,{1,request.edge}},MeshStage::TopologyUpdate);
                for (int face:state.boundary->edge(request.edge).incidentFaces){auto& cache=state.faces[face];
                    if(request.purpose==CadBoundarySplitPurpose::Geometry)state.referenceSeedSources[face].clear();
                    cache.preserveBoundary=state.boundaryQualityRestart&&bool(cache.chart)&&!geometrySourceFaces.count(face);cache.ready=false;}
                ++state.work.sharedEdgeSplits;
                const auto sample=state.pendingSamples.find(request.id);
                if (sample!=state.pendingSamples.end() && state.refinementSamples.size()<8)
                    state.refinementSamples.push_back(std::move(sample->second));
                if (state.diagnostics.empty()) state.diagnostics.push_back("shared CAD boundaries refined and incident faces rebuilt");
                if (state.work.sharedEdgeSplits<=64)
                    state.diagnostics.push_back("shared split CAD edge "+std::to_string(request.edge)+
                        " parameters ["+std::to_string(request.left)+","+std::to_string(request.right)+
                        "]: "+request.reason);
            }
            state.pendingSamples.clear();
            if (state.restart) { state.vertices=std::move(result.mesh.vertices); continue; }
        }
        if(result.ok && state.resolvedCriteria.shapeOptimizationSweeps>0){
            // 残差及共享边协调结束后只执行一次合格片优化，避免抢占修复预算或触发重建循环。
            const bool rolledBack=std::any_of(result.diagnostics.begin(),result.diagnostics.end(),[](const auto& d){
                return d.rfind("physical quality changes rolled back",0)==0||d.rfind("completed boundary quality trial rolled back",0)==0;});
            if(rolledBack){
                std::vector<std::vector<std::array<int,3>>> triangles(geometry.faceCount());
                std::vector<std::vector<std::array<Point2D,3>>> uv(geometry.faceCount());
                for(size_t t=0;t<result.mesh.triangles.size();++t){const int f=result.triangleFaceIds[t];auto ids=result.mesh.triangles[t];auto corners=result.triangleCornerUV[t];
                    if(geometry.faceReversed(f)){std::swap(ids[1],ids[2]);std::swap(corners[1],corners[2]);}
                    triangles[f].push_back(ids);uv[f].push_back(corners);}
                for(int f=0;f<geometry.faceCount();++f){auto& face=state.faces[f];
                    if(!face.chart)face.chart=std::make_unique<Mesh2D>();
                    if(!rebuildAcceptedSurfaceChart(triangles[f],uv[f],*face.chart,face.global,face.constraints,&result.error,{observer,{2,f}})){
                        result.error="returned rollback chart restoration on face "+std::to_string(f)+": "+result.error;result.ok=false;break;}
                    face.geometryVersion=state.nextGeometryVersion++;}
                result.diagnostics.push_back("accepted rollback UV charts restored for bounded shape optimization");
            }
            if(result.ok&&!optimizeCompletedSurface(result,state,geometry,sizeField,progress,observer,true,true))result.ok=false;
        }
        if(result.ok) {
            // 返回前全量复核，覆盖增量版本/活动部件维护可能遗漏的变化。
            // 同时核对关系集合；不能把过期干涉记录带入保存结果。
            const auto relations=[](const std::vector<SurfaceBodyInteraction>& values){
                std::set<std::tuple<int,int,int>> keys;
                for(const auto& item:values)keys.emplace(item.firstBody,item.secondBody,int(item.contact));return keys;};
            const auto previous=relations(result.bodyInteractions);result.bodyInteractions.clear();
            MeshStageScope verification({observer,{}},MeshStage::IntersectionValidation);
            if(!validateCadIntersections(result,geometry,&result.error))result.ok=false;
            else if(previous!=relations(result.bodyInteractions)){
                result.error="incremental CAD intersection audit relationship mismatch";result.ok=false;}
        }
        if(result.ok) {
            compactCompletedSurface(result,observer);
            result.triangleBodyIds.reserve(result.triangleFaceIds.size());for(int f:result.triangleFaceIds)result.triangleBodyIds.push_back(geometry.faceBodyId(f));
            for(const auto& i:result.bodyInteractions) {
                const std::string relation=i.contact==CadBodyContact::VolumeOverlap?"CAD volume interference retained":
                    i.contact==CadBodyContact::BoundaryContact?"CAD boundary contact retained":
                    i.contact==CadBodyContact::QueryFailed?"CAD relationship query failed; independent surface intersection retained":
                    "independent mesh approximations intersect without measured CAD contact";
                result.diagnostics.push_back(relation+" between bodies "+std::to_string(i.firstBody)+" and "+std::to_string(i.secondBody)+
                    " (CAD faces "+std::to_string(i.firstFace)+", "+std::to_string(i.secondFace)+")");
            }
        }
        result.work=state.work;
        result.refinementSamples=std::move(state.refinementSamples);
        result.work.totalWork=state.steps;
        if (!result.ok) result.work.allocatedVertices=static_cast<int>(result.mesh.vertices.size());
        if (!result.ok && result.error.find("budget")!=std::string::npos)
            result.error+="; cumulative work="+std::to_string(state.steps)+
                ", scheduling passes="+std::to_string(state.work.schedulingPasses)+
                ", face builds="+std::to_string(state.work.faceBuilds);
        result.diagnostics.insert(result.diagnostics.begin(),state.diagnostics.begin(),state.diagnostics.end());
        return result;
    }
}

} // namespace mesher
