// 文件职责：独立表面检查：查找三角形自交及几何接触；检查结果不参与网格生成或优化。
#include "SurfaceMeshValidation.h"
#include "geometry/predicates/GeomPredicates.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace mesher {
namespace {
/// 计算两个点的坐标差。
Point3D sub(Point3D a,Point3D b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
/// 计算向量点积。
double dot(Point3D a,Point3D b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
/// 计算向量叉积，确定有向关系。
Point3D cross(Point3D a,Point3D b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
/// 计算向量的欧氏长度。
double norm(Point3D p) { return std::sqrt(dot(p,p)); }
/// 读取三维点在指定轴上的坐标。
double axis(Point3D p,int a) { return a==0?p.x:(a==1?p.y:p.z); }
/// 舍去指定轴，将三维点投影到二维判定平面。
Point2D project(Point3D p,int drop) {
    return drop==0?Point2D{p.y,p.z}:(drop==1?Point2D{p.x,p.z}:Point2D{p.x,p.y});
}
/// 选择法向绝对分量最大的轴，供稳定投影使用。
int dominant(Point3D n) {
    if (std::abs(n.x)>=std::abs(n.y) && std::abs(n.x)>=std::abs(n.z)) return 0;
    return std::abs(n.y)>=std::abs(n.z)?1:2;
}
struct Box {
    Point3D lo{1e300,1e300,1e300},hi{-1e300,-1e300,-1e300};
    /// 将浮点展开按指定符号相加。
    void add(Point3D p) {
        lo={std::min(lo.x,p.x),std::min(lo.y,p.y),std::min(lo.z,p.z)};
        hi={std::max(hi.x,p.x),std::max(hi.y,p.y),std::max(hi.z,p.z)};
    }
    /// 按给定容差判断两个包围盒是否相交。
    bool overlaps(const Box& b,double eps) const {
        return lo.x<=b.hi.x+eps && b.lo.x<=hi.x+eps &&
               lo.y<=b.hi.y+eps && b.lo.y<=hi.y+eps &&
               lo.z<=b.hi.z+eps && b.lo.z<=hi.z+eps;
    }
};

class Checker {
    const SurfaceMesh3D& mesh;
    double eps;
    struct Node { Box box; int begin=0,end=0,left=-1,right=-1; bool active=false; };
    std::vector<int> order;
    std::vector<Box> triangleBoxes;
    std::vector<Node> nodes;
    int first=-1,second=-1;

    /// 结合距离容差和方向谓词判定点在线段哪一侧。
    int sign(Point2D a,Point2D b,Point2D p) const {
        const double det=orient2dDet(a,b,p);
        if (std::abs(det)<=eps*std::hypot(b.x-a.x,b.y-a.y)) return 0;
        return orient2d(a,b,p);
    }
    /// 在二维投影中判断点是否位于三角形内或边界上。
    bool inside(Point3D p,const std::array<int,3>& tri,int drop) const {
        const Point2D q=project(p,drop);
        const int a=sign(project(mesh.vertices[tri[0]],drop),project(mesh.vertices[tri[1]],drop),q);
        const int b=sign(project(mesh.vertices[tri[1]],drop),project(mesh.vertices[tri[2]],drop),q);
        const int c=sign(project(mesh.vertices[tri[2]],drop),project(mesh.vertices[tri[0]],drop),q);
        return (a>=0 && b>=0 && c>=0) || (a<=0 && b<=0 && c<=0);
    }
    /// 判断接触点是否仅对应两个三角形共享的顶点。
    bool allowed(Point3D p,const std::array<int,3>& a,const std::array<int,3>& b) const {
        for (int i:a) for (int j:b)
            if (i==j && norm(sub(p,mesh.vertices[i]))<=eps) return true;
        return false;
    }
    /// 判断点是否落在指定线段范围内。
    bool onSegment(Point2D a,Point2D b,Point2D p) const {
        return sign(a,b,p)==0 && p.x>=std::min(a.x,b.x)-eps && p.x<=std::max(a.x,b.x)+eps &&
               p.y>=std::min(a.y,b.y)-eps && p.y<=std::max(a.y,b.y)+eps;
    }
    /// 在投影平面中检查两条边是否产生不允许的接触。
    bool segmentHit(int ai,int bi,int ci,int di,int drop,
                    const std::array<int,3>& ta,const std::array<int,3>& tb) const {
        const Point2D a=project(mesh.vertices[ai],drop),b=project(mesh.vertices[bi],drop);
        const Point2D c=project(mesh.vertices[ci],drop),d=project(mesh.vertices[di],drop);
        const int s0=sign(a,b,c),s1=sign(a,b,d),s2=sign(c,d,a),s3=sign(c,d,b);
        if (s0*s1<0 && s2*s3<0) return true;
        if (s0==0 && s1==0 && s2==0 && s3==0) {
            const bool same=(ai==ci && bi==di)||(ai==di && bi==ci);
            if (same) return false;
            const bool useX=std::abs(b.x-a.x)>=std::abs(b.y-a.y);
            const double aa=useX?a.x:a.y,bb=useX?b.x:b.y;
            const double cc=useX?c.x:c.y,dd=useX?d.x:d.y;
            const double overlap=std::min(std::max(aa,bb),std::max(cc,dd))-
                                 std::max(std::min(aa,bb),std::min(cc,dd));
            if (overlap>eps) return true;
        }
        if (onSegment(a,b,c) && !allowed(mesh.vertices[ci],ta,tb)) return true;
        if (onSegment(a,b,d) && !allowed(mesh.vertices[di],ta,tb)) return true;
        if (onSegment(c,d,a) && !allowed(mesh.vertices[ai],ta,tb)) return true;
        if (onSegment(c,d,b) && !allowed(mesh.vertices[bi],ta,tb)) return true;
        return false;
    }
    /// 检查共面边是否穿入目标三角形或产生非共享接触。
    bool planarEdgeHit(int a,int b,const std::array<int,3>& source,
                       const std::array<int,3>& target,int drop) const {
        if (inside(mesh.vertices[a],target,drop) && !allowed(mesh.vertices[a],source,target)) return true;
        if (inside(mesh.vertices[b],target,drop) && !allowed(mesh.vertices[b],source,target)) return true;
        for (int k=0;k<3;++k)
            if (segmentHit(a,b,target[k],target[(k+1)%3],drop,source,target)) return true;
        return false;
    }
    /// 检查三维边与目标三角形的穿越或共面接触。
    bool edgeHit(int a,int b,const std::array<int,3>& source,
                 const std::array<int,3>& target,Point3D normal) const {
        const Point3D origin=mesh.vertices[target[0]];
        const double n=norm(normal);
        if (!(n>0)) return true;
        const double da=dot(sub(mesh.vertices[a],origin),normal)/n;
        const double db=dot(sub(mesh.vertices[b],origin),normal)/n;
        if ((da>eps && db>eps)||(da<-eps && db<-eps)) return false;
        const int drop=dominant(normal);
        if (std::abs(da)<=eps && std::abs(db)<=eps)
            return planarEdgeHit(a,b,source,target,drop);
        // 端点已在平面容差内时直接视为接触；再以小交角除舍入残差可能把交点移出共享顶点容差范围。
        if (std::abs(da)<=eps)
            return inside(mesh.vertices[a],target,drop) && !allowed(mesh.vertices[a],source,target);
        if (std::abs(db)<=eps)
            return inside(mesh.vertices[b],target,drop) && !allowed(mesh.vertices[b],source,target);
        const double t=std::clamp(da/(da-db),0.0,1.0);
        const Point3D p=mesh.vertices[a],q=mesh.vertices[b];
        const Point3D hit{p.x+t*(q.x-p.x),p.y+t*(q.y-p.y),p.z+t*(q.z-p.z)};
        return inside(hit,target,drop) && !allowed(hit,source,target);
    }
    /// 判断两个三角形是否发生非合法共享边界的相交。
    bool intersects(int a,int b) const {
        const auto& ta=mesh.triangles[a]; const auto& tb=mesh.triangles[b];
        const Point3D na=cross(sub(mesh.vertices[ta[1]],mesh.vertices[ta[0]]),
                               sub(mesh.vertices[ta[2]],mesh.vertices[ta[0]]));
        const Point3D nb=cross(sub(mesh.vertices[tb[1]],mesh.vertices[tb[0]]),
                               sub(mesh.vertices[tb[2]],mesh.vertices[tb[0]]));
        std::vector<int> common;
        for (int i:ta) for (int j:tb) if (i==j) common.push_back(i);
        if (common.size()==3) return true; // 重复三角形构成重叠。
        if (common.size()==2) {
            int ca=-1,cb=-1;
            for (int v:ta) if (v!=common[0] && v!=common[1]) ca=v;
            for (int v:tb) if (v!=common[0] && v!=common[1]) cb=v;
            const Point3D origin=mesh.vertices[common[0]];
            // 包含同一共享边的不同平面只在该边相交；避免重新插值近零端点距离而把合法边接触误判为穿越。
            if (std::abs(dot(sub(mesh.vertices[ca],origin),nb))>eps*norm(nb) ||
                std::abs(dot(sub(mesh.vertices[cb],origin),na))>eps*norm(na)) return false;
            const int drop=dominant(na);
            const Point2D p=project(origin,drop),q=project(mesh.vertices[common[1]],drop);
            const int sa=orient2d(p,q,project(mesh.vertices[ca],drop));
            const int sb=orient2d(p,q,project(mesh.vertices[cb],drop));
            return sa*sb>=0; // 共面且共享边的三角形须位于该边两侧。
        }
        for (int k=0;k<3;++k) {
            if (edgeHit(ta[k],ta[(k+1)%3],ta,tb,nb)) return true;
            if (edgeHit(tb[k],tb[(k+1)%3],tb,ta,na)) return true;
        }
        return false;
    }
    /// 递归建立三角形包围盒树，减少相交候选数量。
    int build(int begin,int end) {
        Node node; node.begin=begin; node.end=end;
        for (int i=begin;i<end;++i) {
            node.box.add(triangleBoxes[order[i]].lo);
            node.box.add(triangleBoxes[order[i]].hi);
            node.active|=!activeTriangles||(*activeTriangles)[order[i]];
        }
        const int id=static_cast<int>(nodes.size()); nodes.push_back(node);
        if (end-begin>8) {
            const int splitAxis=dominant(sub(node.box.hi,node.box.lo));
            const int mid=begin+(end-begin)/2;
            std::nth_element(order.begin()+begin,order.begin()+mid,order.begin()+end,
                [&](int a,int b) {
                    return axis(triangleBoxes[a].lo,splitAxis)+axis(triangleBoxes[a].hi,splitAxis)<
                           axis(triangleBoxes[b].lo,splitAxis)+axis(triangleBoxes[b].hi,splitAxis);
                });
            node.left=build(begin,mid); node.right=build(mid,end);
            nodes[id]=node;
        }
        return id;
    }
    /// 遍历包围盒树节点对，并校验可能相交的三角形。
    bool visit(int a,int b) {
        const Node& left=nodes[a]; const Node& right=nodes[b];
        if(!left.active&&!right.active)return true;
        if (!left.box.overlaps(right.box,eps)) return true;
        if (left.left<0 && right.left<0) {
            for (int i=left.begin;i<left.end;++i) for (int j=right.begin;j<right.end;++j) {
                if (a==b && j<=i) continue;
                const int x=order[i],y=order[j];
                if(activeTriangles&&!(*activeTriangles)[x]&&!(*activeTriangles)[y])continue;
                if (triangleBoxes[x].overlaps(triangleBoxes[y],eps) && intersects(x,y)) {
                    if(acceptedContact && acceptedContact(x,y))continue;
                    first=x; second=y; return false;
                }
            }
            return true;
        }
        if (a==b) return visit(left.left,left.left) && visit(left.left,left.right) && visit(left.right,left.right);
        if (right.left<0 || (left.left>=0 && left.end-left.begin>=right.end-right.begin))
            return visit(left.left,b) && visit(left.right,b);
        return visit(a,right.left) && visit(a,right.right);
    }
public:
    const std::function<bool(int,int)>& acceptedContact;
    const std::vector<bool>* activeTriangles;
    /// 保存表面、容差和运行控制，准备执行相交检查。
    Checker(const SurfaceMesh3D& surface,double tolerance,const std::function<bool(int,int)>& contact,const std::vector<bool>* active):
        mesh(surface),eps(tolerance),acceptedContact(contact),activeTriangles(active) {}
    /// 检查网格输入并用包围盒树完成表面相交校验。
    bool run(std::string* error,SurfaceIntersection* intersection) {
        if (mesh.triangles.empty()) return true;
        triangleBoxes.resize(mesh.triangles.size());
        order.resize(mesh.triangles.size());
        std::iota(order.begin(),order.end(),0);
        for (size_t i=0;i<mesh.triangles.size();++i) for (int v:mesh.triangles[i]) {
            if (v<0 || v>=static_cast<int>(mesh.vertices.size())) {
                if (error) *error="invalid vertex index in surface intersection check";
                return false;
            }
            triangleBoxes[i].add(mesh.vertices[v]);
        }
        for (size_t i=0;i<mesh.triangles.size();++i) {
            const auto& t=mesh.triangles[i];
            if (!(norm(cross(sub(mesh.vertices[t[1]],mesh.vertices[t[0]]),
                             sub(mesh.vertices[t[2]],mesh.vertices[t[0]])))>0)) {
                if (error) *error="degenerate surface triangle: "+std::to_string(i);
                return false;
            }
        }
        build(0,static_cast<int>(order.size()));
        if (visit(0,0)) return true;
        if (intersection) *intersection={first,second};
        if (error) *error="surface triangles intersect: "+std::to_string(first)+" and "+std::to_string(second);
        return false;
    }
};
}

/// 检查三角面片穿透或共面重叠，允许沿共享节点/共享边接触。
bool validateSurfaceIntersections(const SurfaceMesh3D& surface,std::string* error,double tolerance,
                                  SurfaceIntersection* intersection,const std::function<bool(int,int)>& acceptedCadContact,
                                  const std::vector<bool>* active) {
    if (intersection) *intersection={};
    if(active&&active->size()!=surface.triangles.size()){
        if(error)*error="surface intersection active mask size differs from triangle count";return false;}
    if (!std::isfinite(tolerance) || tolerance<0) {
        if (error) *error="invalid surface intersection tolerance";
        return false;
    }
    Box box; double magnitude=0;
    for (Point3D p:surface.vertices) {
        if (!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) {
            if (error) *error="non-finite vertex in surface intersection check";
            return false;
        }
        box.add(p); magnitude=std::max({magnitude,std::abs(p.x),std::abs(p.y),std::abs(p.z)});
    }
    if (tolerance<=0) tolerance=64*std::numeric_limits<double>::epsilon()*
        std::max(norm(sub(box.hi,box.lo)),magnitude);
    Checker checker(surface,tolerance,acceptedCadContact,active);
    return checker.run(error,intersection);
}
}
