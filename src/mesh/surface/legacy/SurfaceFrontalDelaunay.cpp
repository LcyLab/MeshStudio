// 文件职责：默认框架的前沿布点：从约束边或已满足尺寸的邻片推进候选，以局部 CDT 事务接受或回滚。
#include "SurfaceFrontalDelaunay.h"
#include "mesh/planar/Delaunay2D.h"
#include "geometry/predicates/GeomPredicates.h"
#include "SurfaceChart.h"
#include "SurfaceTangentChart.h"
#include "SurfaceRefinementQueue.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace mesher {
namespace {
/// 检查输入坐标或数值是否有限。
bool finite(Point3D p) {return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
/// 计算两个三维点的坐标差。
Point3D subtract(Point3D a,Point3D b) {return {a.x-b.x,a.y-b.y,a.z-b.z};}
/// 计算向量叉积，确定有向关系。
Point3D cross(Point3D a,Point3D b) {return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
/// 计算向量点积。
double dot(Point3D a,Point3D b) {return a.x*b.x+a.y*b.y+a.z*b.z;}
/// 计算两个三维点的距离。
double length(Point3D a,Point3D b) {auto d=subtract(a,b);return std::hypot(d.x,d.y,d.z);}
/// 将有序节点对编码为边查找键。
std::uint64_t edgeKey(int a,int b) {if(a>b)std::swap(a,b);return (std::uint64_t(std::uint32_t(a))<<32)|std::uint32_t(b);}
/// 沿邻接关系定位候选点所在的三角形。
int locate(const Mesh2D& mesh,Point2D p,int start,const SurfaceDomainState& domain,MeshObservation observation) {
    MeshCountScope visits{observation,MeshCounter::TriangleVisits};int t=start;
    for(size_t step=0;step<=mesh.tris.size();++step) {
        if(t<0||t>=int(mesh.tris.size()))return -1;++visits.value;const auto tri=mesh.tris[t];int next=-1;
        for(int e=0;e<3;++e)if(orient2d(mesh.points[tri.v[e]],mesh.points[tri.v[(e+1)%3]],p)<0){next=e;break;}
        if(next<0)return domain.values()[t]?t:-1;t=tri.n[next];
    }
    return -1;
}
}

/// 在已分类的边界 CDT 中生成面内点；保留原 CAD UV 和固定约束。
bool generateSurfaceFrontalPoints(Mesh2D& mesh,std::vector<int>& global,
    const std::vector<Point3D>& xyz,SurfaceDomainState& domain,
    const std::vector<std::pair<int,int>>& constraints,const GeometrySource& geometry,int face,
    const SurfaceCriteria& criteria,const std::function<double(Point3D)>& targetAt,
    const std::function<int(Point3D)>& allocate,int& work,SurfaceFrontalStatistics& statistics,
    std::string* error,MeshingProgress progress,MeshObservation observation,
    const std::function<size_t()>& activeVertexCount) {
    const auto fail=[&](const std::string& why){if(error)*error=why;return false;};
    if(global.size()!=mesh.points.size()||domain.values().size()!=mesh.tris.size()||!targetAt||!allocate)
        return fail("invalid frontal chart/mapping/callbacks");
    constexpr double limit=.7071067811865475244;
    std::set<std::uint64_t> protectedEdges;
    for(auto e:constraints)protectedEdges.insert(edgeKey(e.first,e.second));
    const auto physical=[&](int v){return xyz.at(global.at(v));};
    struct Measure {bool valid=false;double radius=0,h=0;};
    std::string measurementError;
    // 缓存按拓扑版本失效，尺寸查询不因反复进入活动集合而重复执行。
    std::map<int,std::pair<std::uint64_t,Measure>> measures;
    const auto measure=[&](int t) {
        Measure value;
        if(t<0||t>=int(mesh.tris.size())||!domain.values()[t])return value;
        const auto version=mesh.triangleVersion(t);auto found=measures.find(t);
        if(found!=measures.end()&&found->second.first==version)return found->second.second;
        const auto tri=mesh.tris[t];for(int v:tri.v)if(v<0||v>=int(global.size())||global[v]<0)return value;
        const auto a=physical(tri.v[0]),b=physical(tri.v[1]),c=physical(tri.v[2]);
        if(!finite(a)||!finite(b)||!finite(c)){measurementError="non-finite frontal source vertex";return value;}
        const auto n=cross(subtract(b,a),subtract(c,a));const double twiceArea=std::hypot(n.x,n.y,n.z);
        const double x=length(a,b),y=length(b,c),z=length(c,a);
        const std::array<double,3> targets{targetAt(a),targetAt(b),targetAt(c)};
        for(double target:targets)if(!(target>0)||!std::isfinite(target)){measurementError="invalid frontal target size";return value;}
        value.h=*std::min_element(targets.begin(),targets.end());
        Point2D center{};for(int v:tri.v){center.x+=mesh.points[v].x/3;center.y+=mesh.points[v].y/3;}
        // 曲率要求必须在生成时进入尺度；事后把粗等边片二分会破坏整体布局。
        const auto surfaceSize=surfaceSizeAt(geometry,face,center,value.h,criteria);
        if(!surfaceSize.valid) {
            measurementError="frontal surface sizing: "+surfaceSize.error;
            return value;
        }
        // 前沿以 R≤h/√2 判定尺寸，而几何尺寸限制弦长；利用弦长≤2R 将该上限转换到前沿半径约定，最终仍由 CAD 采样核验。
        value.h=std::min(surfaceSize.target,limit*surfaceSize.curvatureSpan);
        if(!finite(geometry.facePoint(face,center))){measurementError="non-finite frontal CAD point";return value;}
        if(twiceArea>0&&std::isfinite(twiceArea)&&value.h>0&&std::isfinite(value.h)) {
            const double scale=std::max({x,y,z});const auto u=subtract(b,a),v=subtract(c,a);
            const auto normalized=cross({u.x/scale,u.y/scale,u.z/scale},{v.x/scale,v.y/scale,v.z/scale});
            const double area=std::hypot(normalized.x,normalized.y,normalized.z);
            value.radius=scale*(x/scale)*(y/scale)*(z/scale)/(2*area);value.valid=std::isfinite(value.radius);
        }
        measures[t]={version,value};return value;
    };
    const auto activeEdges=[&](int t) {
        std::vector<int> edges;const auto tri=mesh.tris[t];
        for(int e=0;e<3;++e) {
            const int neighbor=tri.n[e];const auto m=measure(neighbor);
            if(protectedEdges.count(edgeKey(tri.v[e],tri.v[(e+1)%3]))||neighbor<0||
               !domain.values()[neighbor]||(m.valid&&m.radius<=limit*m.h*(1+1e-8)))edges.push_back(e);
        }
        // 只比较活动边，不以任意坏片最短边冒充前沿。
        std::sort(edges.begin(),edges.end(),[&](int a,int b){
            const double x=length(physical(tri.v[a]),physical(tri.v[(a+1)%3]));
            const double y=length(physical(tri.v[b]),physical(tri.v[(b+1)%3]));return x==y?a<b:x<y;});
        return edges;
    };
    SurfaceRefinementQueue queue(face,1,observation);
    const auto offer=[&](int t) {
        const auto m=measure(t);if(!measurementError.empty())return fail(measurementError);
        if(!m.valid||m.radius<=limit*m.h*(1+1e-8))return true;
        const auto edges=activeEdges(t);if(!measurementError.empty())return fail(measurementError);
        if(edges.empty())return true;
        const auto tri=mesh.tris[t];std::array<int,3> ids{tri.v[0],tri.v[1],tri.v[2]};std::sort(ids.begin(),ids.end());
        return queue.offer({face,t,mesh.triangleVersion(t),1,1,m.radius/m.h,ids},error);
    };
    for(int t=0;t<int(mesh.tris.size());++t) {
        if(t%64==0&&progress&&!progress(face,geometry.faceCount()))return fail("surface meshing cancelled during frontal initialization");
        if(!offer(t))return false;
    }
    const auto reject=[&](const std::string& why){++statistics.rejected;
        if(statistics.refusals.size()<8)statistics.refusals.push_back(why);};
    SurfaceTriangleTicket ticket;
    while(queue.pop(mesh,ticket)) {
        if(progress&&!progress(face,geometry.faceCount()))return fail("surface meshing cancelled during frontal generation");
        const auto original=measure(ticket.slot);if(!measurementError.empty())return fail(measurementError);
        if(!original.valid||original.radius<=limit*original.h*(1+1e-8))continue;
        const auto tri=mesh.tris[ticket.slot];const auto edges=activeEdges(ticket.slot);
        if(!measurementError.empty())return fail(measurementError);
        bool inserted=false;MeshEditDelta delta;
        for(int edge:edges) {
            const size_t active=activeVertexCount?activeVertexCount():xyz.size();
            if(active>xyz.size())return fail("invalid frontal active vertex count");
            if(work>=criteria.maxIterations||active>=size_t(criteria.maxVertices))return fail("surface frontal generation budget exceeded");
            ++work;++statistics.attempted;
            const Point2D uvA=mesh.points[tri.v[edge]],uvB=mesh.points[tri.v[(edge+1)%3]];
            const Point2D middle{uvA.x+(uvB.x-uvA.x)/2,uvA.y+(uvB.y-uvA.y)/2};
            SurfaceMetric metric;std::string reason;
            if(!surfaceMetricAt(geometry,face,middle,metric,&reason)){reject(reason);continue;}
            const std::array<Point2D,3> sourceUV{mesh.points[tri.v[0]],mesh.points[tri.v[1]],mesh.points[tri.v[2]]};
            const std::array<Point3D,3> sourceXYZ{physical(tri.v[0]),physical(tri.v[1]),physical(tri.v[2])};
            SurfaceTangentChart tangent;
            if(!surfaceTangentChartAt(geometry,face,sourceUV,sourceXYZ,tangent,&reason)){reject("frontal chart: "+reason);continue;}
            const auto a=tangent.project(sourceXYZ[edge]),b=tangent.project(sourceXYZ[(edge+1)%3]);
            const auto opposite=tangent.project(sourceXYZ[(edge+2)%3]);
            Point2D center;double radius=0;
            if(!circumcircle(a,b,opposite,center,radius)){reject("frontal metric circumcircle unavailable");continue;}
            const Point2D mid{a.x+(b.x-a.x)/2,a.y+(b.y-a.y)/2};
            const double base=dist2D(a,b);if(!(base>0)){reject("frontal zero metric edge");continue;}
            const double side=orient2d(a,b,opposite)>0?1:-1;
            const Point2D direction{side*(a.y-b.y)/base,side*(b.x-a.x)/base};
            const double circumHeight=(center.x-mid.x)*direction.x+(center.y-mid.y)*direction.y;
            if(!(circumHeight>0)){reject("frontal circumcenter lies behind active edge");continue;}
            const double height=std::min(std::sqrt(3.)/2*original.h,circumHeight);
            // 参数速度非线性时，一阶 UV 偏移也会扭曲平面候选；在有界局部图表中反求物理目标，并保留 UV 分支和域检查。
            const auto inverse=invertSurfaceTangentChart(geometry,face,tangent,{mid.x+height*direction.x,mid.y+height*direction.y});
            if(!inverse.valid){reject("frontal chart inverse: "+inverse.error);continue;}
            const auto candidate=inverse.uv;
            if(!std::isfinite(candidate.x)||!std::isfinite(candidate.y)){reject("non-finite frontal candidate");continue;}
            if(statistics.samples.size()<8) {
                const int neighbor=tri.n[edge];
                const auto source=protectedEdges.count(edgeKey(tri.v[edge],tri.v[(edge+1)%3]))?
                    SurfaceFrontalEdgeSource::ProtectedConstraint:neighbor<0||!domain.values()[neighbor]?
                    SurfaceFrontalEdgeSource::DomainBoundary:SurfaceFrontalEdgeSource::SettledNeighbor;
                statistics.samples.push_back({{uvA,uvB},candidate,original.h,source});
            }
            if(mesh.exactPoint(candidate)>=0){reject("duplicate frontal UV point");continue;}
            // 限定 walk 为已分类裁剪域；候选不能通过最近面投影跳到孔洞外侧。
            const int start=locate(mesh,candidate,ticket.slot,domain,observation);
            if(start<0||!domain.values()[start]){reject("frontal candidate outside trimmed domain");continue;}
            const auto point=inverse.point;
            if(!finite(point))return fail("non-finite CAD frontal point");
            const double candidateSize=targetAt(point);
            if(!(candidateSize>0)||!std::isfinite(candidateSize))return fail("invalid frontal candidate target size");
            const double separation=.65*std::min(original.h,candidateSize);
            int vertex=-1,newTriangle=-1;const LinearMetric2D linear{metric.xx,metric.xy,metric.yy};
            const auto accept=[&](const Mesh2D& edited,int v,const MeshEditDelta& change) {
                for(int slot:change.changedSlots) {
                    if(slot<0||slot>=int(edited.tris.size()))continue;const auto facet=edited.tris[slot];
                    if(facet.v[0]!=v&&facet.v[1]!=v&&facet.v[2]!=v)continue;
                    std::array<Point3D,3> p;Point2D uv{};
                    for(int k=0;k<3;++k) {
                        const int id=facet.v[k];uv.x+=edited.points[id].x/3;uv.y+=edited.points[id].y/3;
                        if(id==v)p[k]=point;
                        else {if(id>=int(global.size())||global[id]<0)return false;p[k]=physical(id);
                            if(length(point,p[k])<separation)return false;}
                    }
                    auto normal=cross(subtract(p[1],p[0]),subtract(p[2],p[0]));
                    if(geometry.faceReversed(face))normal={-normal.x,-normal.y,-normal.z};
                    const auto cadNormal=geometry.faceNormal(face,uv);
                    if(!finite(cadNormal)||!finite(normal)||!(std::hypot(normal.x,normal.y,normal.z)>0)||dot(normal,cadNormal)<0)return false;
                }
                return true;
            };
            const auto physicalCircle=[&](const Mesh2D& current,const Tri2D& facet,Point2D) {
                std::array<Point2D,3> projected;
                for(int k=0;k<3;++k){const int v=facet.v[k];const auto position=v<int(global.size())&&global[v]>=0?physical(v):geometry.facePoint(face,current.points[v]);
                    if(!finite(position))return 2;projected[k]=tangent.project(position);}
                const auto projectedPoint=tangent.project(point);
                return incircle(projected[0],projected[1],projected[2],projectedPoint);
            };
            if(!insertPoint(mesh,candidate,constraints,start,&vertex,&newTriangle,&reason,&linear,observation,&delta,&domain,accept,physicalCircle)) {
                reject("frontal insertion refused: "+reason);continue;
            }
            if(!domain.apply(mesh,delta,error))return false;
            global.resize(mesh.points.size(),-1);global[vertex]=allocate(point);
            ++statistics.inserted;inserted=true;break;
        }
        if(!inserted)continue;
        queue.invalidate(delta);std::set<int> affected;
        for(int slot:delta.changedSlots)if(slot>=0&&slot<int(mesh.tris.size())) {
            affected.insert(slot);for(int neighbor:mesh.tris[slot].n)if(neighbor>=0)affected.insert(neighbor);
        }
        for(int slot:affected)if(!offer(slot))return false;
    }
    return domain.verify(mesh,constraints,error,observation);
}
}
