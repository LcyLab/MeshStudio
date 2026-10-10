// 文件职责：旧 CDT 框架的参考前沿：生成度量候选并进行曲面圆修正；独立 Algorithm 6 位于 algorithm6 目录。
#include "SurfaceReferenceFront.h"
#include "mesh/planar/Delaunay2D.h"
#include "mesh/planar/Constrained2D.h"
#include "geometry/predicates/GeomPredicates.h"
#include "SurfaceRefinementQueue.h"
#include "SurfaceTangentChart.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <set>

namespace mesher {
namespace {
constexpr double radiusLimit=.7071067811865475244;
/// 检查输入坐标或数值是否有限。
bool finite(Point3D p){return std::isfinite(p.x)&&std::isfinite(p.y)&&std::isfinite(p.z);}
/// 检查输入坐标或数值是否有限。
bool finite(Point2D p){return std::isfinite(p.x)&&std::isfinite(p.y);}
/// 检查节点网格尺寸和背景尺寸是否为有限正数。
bool valid(SurfaceReferenceSize s){return s.mesh>0&&s.background>0&&std::isfinite(s.mesh)&&std::isfinite(s.background);}
/// 计算两个点的坐标差。
Point3D sub(Point3D a,Point3D b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
/// 将浮点展开按指定符号相加。
Point3D add(Point3D a,Point3D b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
/// 读取统一缩放比例。
Point3D scale(Point3D a,double s){return {a.x*s,a.y*s,a.z*s};}
/// 计算向量叉积，确定有向关系。
Point3D cross(Point3D a,Point3D b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
/// 计算向量点积。
double dot(Point3D a,Point3D b){return a.x*b.x+a.y*b.y+a.z*b.z;}
/// 计算向量的欧氏长度。
double norm(Point3D a){return std::hypot(a.x,a.y,a.z);}
/// 按边界尺寸传播开关选择节点使用的尺寸。
double selected(SurfaceReferenceSize s,bool extend){return extend?std::min(s.mesh,s.background):s.background;}
/// 对三角形顶点编号排序，构造稳定身份键。
std::array<int,3> key(const Tri2D& tri){std::array<int,3> k{tri.v[0],tri.v[1],tri.v[2]};std::sort(k.begin(),k.end());return k;}

// 三变量牛顿解 S(u,v)=center+r*(inside*cos(t)+normal*sin(t))。
// 只在残差下降且参数分支有界时更新；不收敛由调用者明确保留初始度量候选。
bool correctCircle(const GeometrySource& geometry,int face,Point3D center,
    Point3D inside,Point3D normal,double radius,Point2D seed,Point2D& result) {
    SurfaceParameterBounds bounds;const bool bounded=geometry.faceParameterBounds(face,bounds);
    if(bounded&&(!(bounds.uMax>bounds.uMin&&bounds.vMax>bounds.vMin)||
        !std::isfinite(bounds.uMin)||!std::isfinite(bounds.uMax)||!std::isfinite(bounds.vMin)||!std::isfinite(bounds.vMax)))return false;
    Point2D uv=seed;double theta=0;
    const auto circle=[&](double t){return add(center,scale(add(scale(inside,std::cos(t)),scale(normal,std::sin(t))),radius));};
    const double tolerance=radius*1e-8;
    for(int iteration=0;iteration<30;++iteration) {
        const auto point=geometry.facePoint(face,uv);const auto residual=sub(point,circle(theta));
        const double distance=norm(residual);if(!finite(point)||!std::isfinite(distance))return false;
        if(distance<=tolerance){result=uv;return true;}
        Point3D du,dv;geometry.faceDerivatives(face,uv,du,dv);
        const auto dt=scale(add(scale(inside,-std::sin(theta)),scale(normal,std::cos(theta))),-radius);
        if(!finite(du)||!finite(dv)||!finite(dt))return false;
        const double a=norm(du),b=norm(dv),c=norm(dt);if(!(a>0&&b>0&&c>0))return false;
        const auto u=scale(du,1/a),v=scale(dv,1/b),t=scale(dt,1/c);
        const double determinant=dot(u,cross(v,t));
        if(!std::isfinite(determinant)||std::abs(determinant)<=64*std::numeric_limits<double>::epsilon())return false;
        const auto rhs=scale(residual,-1);
        const double stepU=dot(rhs,cross(v,t))/(determinant*a);
        const double stepV=dot(u,cross(rhs,t))/(determinant*b);
        const double stepT=dot(u,cross(v,rhs))/(determinant*c);
        bool advanced=false;
        for(double fraction=1;fraction>=1./1024;fraction*=.5) {
            const Point2D trial{uv.x+fraction*stepU,uv.y+fraction*stepV};
            const double angle=theta+fraction*stepT;
            if(!finite(trial)||!std::isfinite(angle)||(bounded&&
                (trial.x<bounds.uMin||trial.x>bounds.uMax||trial.y<bounds.vMin||trial.y>bounds.vMax)))continue;
            const auto p=geometry.facePoint(face,trial);
            if(finite(p)&&norm(sub(p,circle(angle)))<distance){uv=trial;theta=angle;advanced=true;break;}
        }
        if(!advanced)return false;
    }
    return false;
}
/// 沿网格邻接定位参考前沿候选点。
int locate(const Mesh2D& mesh,Point2D p,int start,const SurfaceDomainState& domain,MeshObservation observation) {
    MeshCountScope visits{observation,MeshCounter::TriangleVisits};int slot=start;
    for(size_t step=0;step<=mesh.tris.size();++step) {
        if(slot<0||slot>=int(mesh.tris.size()))return -1;
        ++visits.value;const auto tri=mesh.tris[slot];int outside=-1;
        for(int e=0;e<3;++e)if(orient2d(mesh.points[tri.v[e]],mesh.points[tri.v[(e+1)%3]],p)<0){outside=e;break;}
        if(outside<0)return domain.values()[slot]?slot:-1;slot=tri.n[outside];
    }
    return -1;
}
/// 计算参数三角形面积的两倍。
double area(const Mesh2D& mesh,const std::array<int,3>& v) {
    const auto a=mesh.points[v[0]],b=mesh.points[v[1]],c=mesh.points[v[2]];
    return std::abs((b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x));
}
}

/// 根据活动边构造参考点，edge 对应 Mesh2D 的 (v[e],v[(e+1)%3])。
bool surfaceReferenceCandidate(const GeometrySource& geometry,int face,
    const std::array<Point2D,3>& uv,const std::array<Point3D,3>& xyz,
    const std::array<SurfaceReferenceSize,3>& sizes,int edge,SurfaceReferenceOptions options,
    SurfaceReferenceCandidate& output,std::string* error) {
    const auto fail=[&](const char* why){if(error)*error=why;return false;};
    if(edge<0||edge>2)return fail("invalid reference active edge");
    if(options.physicalPointHeight&&!options.correctOnSurface)return fail("physical reference height requires surface correction");
    for(int k=0;k<3;++k)if(!finite(uv[k])||!finite(xyz[k])||!valid(sizes[k]))return fail("invalid reference candidate source");
    SurfaceReferenceCandidate result;
    const Point2D centroid{uv[0].x/3+uv[1].x/3+uv[2].x/3,uv[0].y/3+uv[1].y/3+uv[2].y/3};
    if(!surfaceMetricAt(geometry,face,centroid,result.metric,error))return false;
    Point2D center;double radius;
    if(!circumcircle(result.metric.project(uv[0]),result.metric.project(uv[1]),result.metric.project(uv[2]),center,radius))
        return fail("reference metric circumcircle unavailable");
    const int next=(edge+1)%3,opposite=(edge+2)%3;
    const auto a=result.metric.project(uv[edge]),b=result.metric.project(uv[next]);
    const Point2D middle{a.x/2+b.x/2,a.y/2+b.y/2};
    const double q=dist2D(center,middle);
    const SurfaceReferenceSize mean{sizes[edge].mesh/2+sizes[next].mesh/2,sizes[edge].background/2+sizes[next].background/2};
    result.target=selected(mean,options.extendBoundarySize);
    result.height=std::min(std::sqrt(3.)/2*result.target,q);
    if(options.physicalPointHeight) {
        SurfaceTangentChart tangent;
        if(!surfaceTangentChartAt(geometry,face,uv,xyz,tangent,error))return false;
        const auto a=tangent.project(xyz[edge]),b=tangent.project(xyz[next]),c=tangent.project(xyz[opposite]);
        Point2D physicalCenter;double physicalRadius;
        if(!circumcircle(a,b,c,physicalCenter,physicalRadius))return fail("physical reference circumcircle unavailable");
        const Point2D middle{a.x/2+b.x/2,a.y/2+b.y/2};
        result.height=std::min(std::sqrt(3.)/2*result.target,dist2D(middle,physicalCenter));
    }
    if(!(q>0)||!std::isfinite(q))return fail("reference circumcenter at active midpoint");
    result.uv=result.metric.unproject({middle.x+result.height*(center.x-middle.x)/q,middle.y+result.height*(center.y-middle.y)/q});
    if(!finite(result.uv))return fail("non-finite reference candidate UV");
    if(options.correctOnSurface) {
        const auto midpoint=scale(add(xyz[edge],xyz[next]),.5);
        const auto edgeVector=sub(xyz[next],xyz[edge]);
        auto normal=cross(edgeVector,sub(xyz[opposite],midpoint));
        const double n=norm(normal);auto inside=cross(normal,edgeVector);const double d=norm(inside);
        if(n>0&&d>0&&std::isfinite(n)&&std::isfinite(d)) {
            Point2D corrected;
            result.circleCorrected=correctCircle(geometry,face,midpoint,scale(inside,1/d),scale(normal,1/n),result.height,result.uv,corrected);
            if(result.circleCorrected)result.uv=corrected;else result.circleFallback=true;
        }else result.circleFallback=true;
    }
    result.point=geometry.facePoint(face,result.uv);
    if(!finite(result.point))return fail("non-finite reference CAD point");
    output=result;return true;
}

/// 从初始域内 CDT 的最短物理邻边建立节点尺寸，再应用半边界段长下限。
bool initializeSurfaceReferenceSizes(const Mesh2D& mesh,const std::vector<int>& global,
    const std::vector<Point3D>& xyz,const SurfaceDomainState& domain,
    const std::vector<std::pair<int,int>>& constraints,std::vector<SurfaceReferenceSize>& output,
    std::string* error,const std::function<double(Point2D,Point3D)>& limitAt,
    const std::vector<std::pair<int,double>>& fixedSizes) {
    const auto fail=[&](const char* why){if(error)*error=why;return false;};
    if(global.size()!=mesh.points.size()||domain.values().size()!=mesh.tris.size())return fail("invalid reference seed mapping/domain");
    const auto validVertex=[&](int v){return v>=0&&v<int(global.size())&&global[v]>=0&&global[v]<int(xyz.size())&&finite(xyz[global[v]])&&finite(mesh.points[v]);};
    std::vector<double> minimum(mesh.points.size(),std::numeric_limits<double>::infinity());
    for(int slot=0;slot<int(mesh.tris.size());++slot)if(domain.values()[slot]) {
        const auto tri=mesh.tris[slot];
        for(int e=0;e<3;++e) {
            const int a=tri.v[e],b=tri.v[(e+1)%3];if(!validVertex(a)||!validVertex(b))return fail("invalid reference seed vertex");
            // CAD 极点在 UV 图表中保留不同角，物理身份已由共享节点给定。
            if(global[a]==global[b])continue;
            const double d=norm(sub(xyz[global[a]],xyz[global[b]]));
            if(!(d>0)||!std::isfinite(d)) {
                if(error)*error="degenerate reference seed edge: local "+std::to_string(a)+","+std::to_string(b)+
                    " global "+std::to_string(global[a])+","+std::to_string(global[b]);return false;
            }
            minimum[a]=std::min(minimum[a],d);minimum[b]=std::min(minimum[b],d);
        }
    }
    std::set<int> supplied;
    for(auto [v,size]:fixedSizes) {
        if(!validVertex(v)||!std::isfinite(minimum[v])||!(size>0)||!std::isfinite(size)||!supplied.insert(v).second)
            return fail("invalid fixed reference seed size");
        minimum[v]=size;
    }
    for(auto e:constraints) {
        if(!validVertex(e.first)||!validVertex(e.second))return fail("invalid reference seed constraint");
        const double floor=norm(sub(xyz[global[e.first]],xyz[global[e.second]]))/2;
        minimum[e.first]=std::max(minimum[e.first],floor);minimum[e.second]=std::max(minimum[e.second],floor);
    }
    std::vector<SurfaceReferenceSize> result(mesh.points.size());
    for(int v=0;v<int(minimum.size());++v)if(std::isfinite(minimum[v])) {
        if(!(minimum[v]>0))return fail("invalid reference initial size");
        if(limitAt) {
            const double ceiling=limitAt(mesh.points[v],xyz[global[v]]);
            if(!(ceiling>0)||std::isnan(ceiling))return fail("invalid reference seed size limit");
            minimum[v]=std::min(minimum[v],ceiling);
        }
        result[v]={minimum[v],minimum[v]};
    }
    output=std::move(result);return true;
}

/// 初始周期图表中跨域连接了同一物理节点时，按该内部边的UV中点建立种子。
bool seedSurfaceReferenceChart(Mesh2D& mesh,std::vector<int>& global,const std::vector<Point3D>& xyz,
    SurfaceDomainState& domain,const std::vector<std::pair<int,int>>& constraints,const GeometrySource& geometry,int face,
    const SurfaceCriteria& criteria,const std::function<int(Point3D)>& allocate,int& work,int& inserted,
    std::string* error,MeshingProgress progress,MeshObservation observation,const std::function<size_t()>& activeVertexCount) {
    const auto fail=[&](const std::string& why){if(error)*error=why;return false;};
    if(face<0||face>=geometry.faceCount()||global.size()!=mesh.points.size()||domain.values().size()!=mesh.tris.size()||
        !allocate||work<0||criteria.maxIterations<=0||criteria.maxVertices<=0)return fail("invalid reference chart seed inputs");
    std::set<std::pair<int,int>> protectedEdges;
    std::vector<std::pair<int,int>> expanded;
    if(!expandConstraintEdges(mesh,constraints,expanded,error,observation))return false;
    for(auto edge:expanded)protectedEdges.insert(std::minmax(edge.first,edge.second));
    inserted=0;
    for(;;) {
        if(progress&&!progress(face,geometry.faceCount()))return fail("surface meshing cancelled during reference chart seeding");
        int seed=-1,a=-1,b=-1;
        for(int slot=0;slot<int(mesh.tris.size())&&seed<0;++slot)if(domain.values()[slot]) {
            if(slot%64==0&&progress&&!progress(face,geometry.faceCount()))return fail("surface meshing cancelled during reference chart seeding");
            const auto tri=mesh.tris[slot];
            for(int v:tri.v)if(v<0||v>=int(global.size())||global[v]<0||global[v]>=int(xyz.size()))return fail("invalid reference chart seed mapping");
            for(int e=0;e<3;++e) {
                const int v0=tri.v[e],v1=tri.v[(e+1)%3];
                if(global[v0]==global[v1]&&!protectedEdges.count(std::minmax(v0,v1))){seed=slot;a=v0;b=v1;break;}
            }
        }
        if(seed<0)return domain.verify(mesh,constraints,error,observation);
        const auto live=activeVertexCount?activeVertexCount():xyz.size();
        if(live>xyz.size())return fail("invalid reference chart active vertex count");
        if(work>=criteria.maxIterations||live>=size_t(criteria.maxVertices))return fail("surface reference chart seed budget exceeded");
        const Point2D uv{mesh.points[a].x/2+mesh.points[b].x/2,mesh.points[a].y/2+mesh.points[b].y/2};
        const auto point=geometry.facePoint(face,uv);
        if(!finite(point)||!(norm(sub(point,xyz[global[a]]))>0))return fail("reference periodic chart midpoint has no physical expansion");
        SurfaceMetric metric;if(!surfaceMetricAt(geometry,face,uv,metric,error))return false;
        const LinearMetric2D linear{metric.xx,metric.xy,metric.yy};
        MeshEditDelta delta;int vertex=-1,next=-1;std::string why;++work;
        if(!insertPoint(mesh,uv,constraints,seed,&vertex,&next,&why,&linear,observation,&delta,&domain))return fail("reference chart seed insertion: "+why);
        if(!domain.apply(mesh,delta,error))return false;
        const int id=allocate(point);if(id<0||id>=int(xyz.size()))return fail("invalid reference allocated chart seed");
        global.resize(mesh.points.size(),-1);global[vertex]=id;++inserted;
    }
}

/// 从已有边界 CDT 生成点；原 UV、约束、域及自有几何谓词保持。
bool generateSurfaceReferenceFront(Mesh2D& mesh,std::vector<int>& global,
    const std::vector<Point3D>& xyz,SurfaceDomainState& domain,
    const std::vector<std::pair<int,int>>& constraints,const GeometrySource& geometry,int face,
    const SurfaceCriteria& criteria,const std::vector<SurfaceReferenceSize>& initialSizes,
    const std::function<double(Point2D,Point3D)>& backgroundAt,const std::function<int(Point3D)>& allocate,
    int& work,SurfaceReferenceStatistics& statistics,std::string* error,SurfaceReferenceOptions options,
    MeshingProgress progress,MeshObservation observation,const std::function<size_t()>& activeVertexCount,
    const std::function<double(Point2D,Point3D)>& maximumEdgeAt) {
    const auto fail=[&](const std::string& why){if(error)*error=why;return false;};
    if(face<0||face>=geometry.faceCount()||global.size()!=mesh.points.size()||initialSizes.size()!=mesh.points.size()||
        domain.values().size()!=mesh.tris.size()||!backgroundAt||!allocate||work<0||criteria.maxIterations<=0||criteria.maxVertices<=0)
        return fail("invalid reference front inputs");
    if(options.physicalPointHeight&&!options.correctOnSurface)return fail("physical reference height requires surface correction");
    std::vector<SurfaceReferenceSize> sizes=initialSizes;std::vector<bool> boundary(mesh.points.size(),false);
    for(auto e:constraints) {
        if(e.first<0||e.second<0||e.first>=int(boundary.size())||e.second>=int(boundary.size()))return fail("invalid reference constraint");
        boundary[e.first]=boundary[e.second]=true;
    }
    const auto physical=[&](int v){return xyz.at(global.at(v));};
    struct Measure {std::uint64_t version=0;double radius=-1;};std::map<int,Measure> measures;
    std::set<std::array<int,3>> rejectedTriangles;
    std::string measurementError;
    const auto measure=[&](int slot) {
        if(slot<0||slot>=int(mesh.tris.size())||!domain.values()[slot])return -1.;
        const auto tri=mesh.tris[slot];if(rejectedTriangles.count(key(tri)))return -1.;
        const auto version=mesh.triangleVersion(slot);const auto cached=measures.find(slot);
        if(cached!=measures.end()&&cached->second.version==version)return cached->second.radius;
        std::array<Point3D,3> p;SurfaceReferenceSize mean;
        for(int k=0;k<3;++k) {
            const int v=tri.v[k];if(v<0||v>=int(global.size())||global[v]<0||global[v]>=int(xyz.size())||!valid(sizes[v])) {
                measurementError="invalid reference vertex mapping/size";return -1.;
            }
            p[k]=physical(v);if(!finite(p[k])){measurementError="non-finite reference source vertex";return -1.;}
            mean.mesh+=sizes[v].mesh/3;mean.background+=sizes[v].background/3;
        }
        // UV 桥接片不对应物理面片；不可对其求物理外接圆或作为活动前沿。
        if(global[tri.v[0]]==global[tri.v[1]]||global[tri.v[1]]==global[tri.v[2]]||global[tri.v[2]]==global[tri.v[0]]) {
            measures[slot]={version,-1};return -1.;
        }
        Point2D centroid{};for(int v:tri.v){centroid.x+=mesh.points[v].x/3;centroid.y+=mesh.points[v].y/3;}
        if(!finite(geometry.facePoint(face,centroid))||!finite(geometry.faceNormal(face,centroid))) {
            measurementError="non-finite reference CAD point/normal";return -1.;
        }
        Point3D du,dv;geometry.faceDerivatives(face,centroid,du,dv);
        if(!finite(du)||!finite(dv)){measurementError="non-finite reference CAD derivatives";return -1.;}
        const auto u=sub(p[1],p[0]),v=sub(p[2],p[0]);const double s=std::max({norm(u),norm(v),norm(sub(p[2],p[1]))});
        if(!(s>0)||!std::isfinite(s)){measurementError="degenerate reference physical triangle";return -1.;}
        const double twiceArea=norm(cross(scale(u,1/s),scale(v,1/s)));
        if(!(twiceArea>0)||!std::isfinite(twiceArea)){measurementError="degenerate reference physical triangle";return -1.;}
        double radius=s*(norm(u)/s)*(norm(v)/s)*(norm(sub(p[2],p[1]))/s)/(2*twiceArea*selected(mean,options.extendBoundarySize));
        if(maximumEdgeAt) {
            const Point3D point{p[0].x/3+p[1].x/3+p[2].x/3,p[0].y/3+p[1].y/3+p[2].y/3,p[0].z/3+p[1].z/3+p[2].z/3};
            const double ceiling=maximumEdgeAt(centroid,point);
            if(!(ceiling>0)||!std::isfinite(ceiling)){measurementError="invalid reference maximum edge limit";return -1.;}
            // 产品超长片仍由同一前沿布点处理，避免直接进入参数中点二分。
            radius=std::max(radius,radiusLimit*s/ceiling);
        }
        if(!std::isfinite(radius)){measurementError="non-finite reference radius";return -1.;}
        measures[slot]={version,radius};return radius;
    };
    const auto activeEdge=[&](int slot) {
        const auto tri=mesh.tris[slot];
        // Gmsh 的邻边编号以终点计数，映射到自有 (起点,终点) 为 2,0,1。
        for(int e:{2,0,1}) {
            const int neighbor=tri.n[e];
            if(neighbor<0||!domain.values()[neighbor])return e;
            const double radius=measure(neighbor);if(radius>0&&radius<radiusLimit)return e;
        }
        return -1;
    };
    SurfaceRefinementQueue queue(face,1,observation);
    const auto offer=[&](int slot) {
        const double radius=measure(slot);if(!measurementError.empty())return fail(measurementError);
        if(radius<=radiusLimit)return true;
        if(activeEdge(slot)<0){if(!measurementError.empty())return fail(measurementError);return true;}
        if(!measurementError.empty())return fail(measurementError);
        return queue.offer({face,slot,mesh.triangleVersion(slot),1,1,radius,key(mesh.tris[slot])},error);
    };
    for(int slot=0;slot<int(mesh.tris.size());++slot) {
        if(slot%64==0&&progress&&!progress(face,geometry.faceCount()))return fail("surface meshing cancelled during reference initialization");
        if(!offer(slot))return false;
    }
    const auto reject=[&](const std::string& reason){++statistics.rejected;if(statistics.refusals.size()<8)statistics.refusals.push_back(reason);};
    SurfaceTriangleTicket ticket;
    while(queue.pop(mesh,ticket)) {
        if(progress&&!progress(face,geometry.faceCount()))return fail("surface meshing cancelled during reference generation");
        if(measure(ticket.slot)<=radiusLimit)continue;
        const int edge=activeEdge(ticket.slot);if(!measurementError.empty())return fail(measurementError);if(edge<0)continue;
        const size_t live=activeVertexCount?activeVertexCount():xyz.size();
        if(live>xyz.size())return fail("invalid reference active vertex count");
        if(work>=criteria.maxIterations||live>=size_t(criteria.maxVertices))return fail("surface reference generation budget exceeded");
        ++work;++statistics.attempted;const auto tri=mesh.tris[ticket.slot];
        std::array<Point2D,3> sourceUV;std::array<Point3D,3> sourceXYZ;std::array<SurfaceReferenceSize,3> sourceSizes;
        for(int k=0;k<3;++k){sourceUV[k]=mesh.points[tri.v[k]];sourceXYZ[k]=physical(tri.v[k]);sourceSizes[k]=sizes[tri.v[k]];}
        SurfaceReferenceCandidate candidate;std::string reason;
        if(!surfaceReferenceCandidate(geometry,face,sourceUV,sourceXYZ,sourceSizes,edge,options,candidate,&reason)){reject(reason);continue;}
        statistics.circleCorrections+=candidate.circleCorrected;statistics.circleFallbacks+=candidate.circleFallback;
        if(statistics.samples.size()<8) {
            const int neighbor=tri.n[edge];const auto source=boundary[tri.v[edge]]&&boundary[tri.v[(edge+1)%3]]?
                SurfaceFrontalEdgeSource::ProtectedConstraint:neighbor<0||!domain.values()[neighbor]?
                SurfaceFrontalEdgeSource::DomainBoundary:SurfaceFrontalEdgeSource::SettledNeighbor;
            statistics.samples.push_back({{sourceUV[edge],sourceUV[(edge+1)%3]},candidate.uv,candidate.target,source});
        }
        const int start=locate(mesh,candidate.uv,ticket.slot,domain,observation);
        if(start<0){reject("reference candidate outside trimmed domain");continue;}
        const auto containing=mesh.tris[start];const auto a=mesh.points[containing.v[0]],b=mesh.points[containing.v[1]],c=mesh.points[containing.v[2]];
        const double determinant=(b.x-a.x)*(c.y-a.y)-(b.y-a.y)*(c.x-a.x);
        const double w1=((candidate.uv.x-a.x)*(c.y-a.y)-(candidate.uv.y-a.y)*(c.x-a.x))/determinant;
        const double w2=((b.x-a.x)*(candidate.uv.y-a.y)-(b.y-a.y)*(candidate.uv.x-a.x))/determinant;
        const SurfaceReferenceSize newSize{(1-w1-w2)*sizes[containing.v[0]].mesh+w1*sizes[containing.v[1]].mesh+w2*sizes[containing.v[2]].mesh,backgroundAt(candidate.uv,candidate.point)};
        if(!valid(newSize))return fail("invalid reference candidate size");
        std::string refusal;
        const auto accept=[&](const Mesh2D& edited,int vertex,const MeshEditDelta& delta) {
            if(delta.removedTriangles.size()<2){refusal="reference cavity has fewer than two triangles";return false;}
            double oldArea=0,newArea=0;for(const auto& removed:delta.removedTriangles)oldArea+=area(edited,removed);
            int shell=0;
            for(int slot:delta.changedSlots) {
                if(slot<0||slot>=int(edited.tris.size()))continue;
                const auto facet=edited.tris[slot];int at=-1;
                for(int k=0;k<3;++k)if(facet.v[k]==vertex)at=k;if(at<0)continue;
                ++shell;newArea+=area(edited,{facet.v[0],facet.v[1],facet.v[2]});
                const int v0=facet.v[(at+1)%3],v1=facet.v[(at+2)%3];const auto p0=physical(v0),p1=physical(v1);
                const SurfaceReferenceSize mean{sizes[v0].mesh/3+sizes[v1].mesh/3+newSize.mesh/3,
                    sizes[v0].background/3+sizes[v1].background/3+newSize.background/3};
                // Gmsh 的距离拒绝始终组合两种来源，extend 仅影响半径/布点。
                const double h=std::min(mean.mesh,mean.background),d0=norm(sub(candidate.point,p0)),d1=norm(sub(candidate.point,p1)),base=norm(sub(p1,p0));
                if(d0<.5*h||d1<.5*h||!(base>0)||!(d0>0&&d1>0)) {refusal="reference candidate too close to cavity vertex";return false;}
                if(boundary[v0]&&boundary[v1]&&norm(cross(sub(p1,p0),sub(candidate.point,p0)))/base<.4*h) {
                    refusal="reference candidate too close to boundary line";return false;
                }
                if((d0*d0+d1*d1-base*base)/(2*d0*d1)<-.9999){refusal="reference candidate nearly flat angle";return false;}
                const auto uv0=edited.points[facet.v[0]],uv1=edited.points[facet.v[1]],uv2=edited.points[facet.v[2]];
                const Point2D center{uv0.x/3+uv1.x/3+uv2.x/3,uv0.y/3+uv1.y/3+uv2.y/3};
                Point3D p[3];for(int k=0;k<3;++k)p[k]=facet.v[k]==vertex?candidate.point:physical(facet.v[k]);
                auto normal=cross(sub(p[1],p[0]),sub(p[2],p[0]));if(geometry.faceReversed(face))normal=scale(normal,-1);
                const auto cadNormal=geometry.faceNormal(face,center);
                if(!finite(cadNormal)||!(norm(cadNormal)>0)||!finite(normal)||!(norm(normal)>0)||dot(normal,cadNormal)<=0){refusal="reference physical orientation invalid";return false;}
            }
            if(shell!=int(delta.removedTriangles.size())+2){refusal="reference cavity Euler mismatch";return false;}
            if(!(oldArea>0)||!std::isfinite(oldArea)||!std::isfinite(newArea)||std::abs(oldArea-newArea)>=1e-12*oldArea) {
                refusal="reference cavity UV area mismatch";return false;
            }
            return true;
        };
        const LinearMetric2D linear{candidate.metric.xx,candidate.metric.xy,candidate.metric.yy};
        SurfaceTangentChart tangent;
        std::function<int(const Mesh2D&,const Tri2D&,Point2D)> circle;
        if(options.physicalCavity) {
            if(!surfaceTangentChartAt(geometry,face,sourceUV,sourceXYZ,tangent,&reason)){reject("reference physical chart: "+reason);continue;}
            // 点位修正后连接必须使用一致的物理邻域；保留原UV星形/面积/域护栏。
            circle=[&](const Mesh2D& current,const Tri2D& facet,Point2D) {
                std::array<Point2D,3> p;
                for(int k=0;k<3;++k) {
                    const int v=facet.v[k];const auto position=v<int(global.size())&&global[v]>=0?physical(v):geometry.facePoint(face,current.points[v]);
                    if(!finite(position))return 2;p[k]=tangent.project(position);
                }
                return incircle(p[0],p[1],p[2],tangent.project(candidate.point));
            };
        }
        MeshEditDelta delta;int vertex=-1,next=-1;
        if(!insertPoint(mesh,candidate.uv,constraints,start,&vertex,&next,&reason,&linear,observation,&delta,&domain,accept,circle)) {
            reject(refusal.empty()?reason:refusal);rejectedTriangles.insert(key(tri));continue;
        }
        if(!domain.apply(mesh,delta,error))return false;
        const int id=allocate(candidate.point);if(id<0||id>=int(xyz.size()))return fail("invalid reference allocated vertex");
        global.resize(mesh.points.size(),-1);global[vertex]=id;sizes.resize(mesh.points.size());sizes[vertex]=newSize;boundary.resize(mesh.points.size(),false);
        ++statistics.inserted;queue.invalidate(delta);
        for(const auto& removed:delta.removedTriangles){auto k=removed;std::sort(k.begin(),k.end());rejectedTriangles.erase(k);}
        std::set<int> affected;
        for(int slot:delta.changedSlots)if(slot>=0&&slot<int(mesh.tris.size())) {
            affected.insert(slot);for(int neighbor:mesh.tris[slot].n)if(neighbor>=0)affected.insert(neighbor);
        }
        for(int slot:affected)if(!offer(slot))return false;
    }
    // 拒绝片从活动队列移除，但不能在统计中伪装成已满足尺寸。
    rejectedTriangles.clear();
    statistics.remainingLargeTriangles=0;
    statistics.collapsedPoleTriangles=0;
    for(int slot=0;slot<int(mesh.tris.size());++slot)if(domain.values()[slot]) {
        const auto tri=mesh.tris[slot];
        if(global[tri.v[0]]==global[tri.v[1]]||global[tri.v[1]]==global[tri.v[2]]||global[tri.v[2]]==global[tri.v[0]])++statistics.collapsedPoleTriangles;
        if(measure(slot)>radiusLimit)++statistics.remainingLargeTriangles;
    }
    if(!measurementError.empty())return fail(measurementError);
    return domain.verify(mesh,constraints,error,observation);
}
}
