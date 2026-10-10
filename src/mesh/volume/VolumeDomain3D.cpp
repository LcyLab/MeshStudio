// 文件职责：体域处理：核验约束面覆盖，尝试安全局部恢复，再沿边界屏障的奇偶关系移除外域和内腔。
#include "VolumeDomain3D.h"
#include "geometry/predicates/GeomPredicates.h"
#include "Flips3D.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <queue>
#include <set>

namespace mesher {
namespace {
/// 排序三个顶点编号并编码为无向面键。
uint64_t faceKey(int a,int b,int c) {
    if(a>b) std::swap(a,b); if(b>c) std::swap(b,c); if(a>b) std::swap(a,b);
    return (uint64_t(a)<<42)|(uint64_t(b)<<21)|uint64_t(c);
}
/// 计算两个点的坐标差。
Point3D sub(Point3D a,Point3D b) { return {a.x-b.x,a.y-b.y,a.z-b.z}; }
/// 计算三维向量叉积。
Point3D cross3(Point3D a,Point3D b) {
    return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};
}
/// 计算向量的欧氏长度。
double norm(Point3D p) { return std::hypot(p.x,p.y,p.z); }
/// 计算向量点积。
double dot(Point3D a,Point3D b) { return a.x*b.x+a.y*b.y+a.z*b.z; }
struct Box {
    double lo[3]={INFINITY,INFINITY,INFINITY},hi[3]={-INFINITY,-INFINITY,-INFINITY};
    /// 将浮点展开按指定符号相加。
    void add(Point3D p) {
        const double v[3]={p.x,p.y,p.z};
        for(int i=0;i<3;++i) { lo[i]=std::min(lo[i],v[i]); hi[i]=std::max(hi[i],v[i]); }
    }
    /// 将浮点展开按指定符号相加。
    void add(const Box& b) {
        for(int i=0;i<3;++i) { lo[i]=std::min(lo[i],b.lo[i]); hi[i]=std::max(hi[i],b.hi[i]); }
    }
    /// 按给定容差判断两个包围盒是否相交。
    bool overlaps(const Box& b,double eps) const {
        for(int i=0;i<3;++i) if(lo[i]>b.hi[i]+eps || hi[i]<b.lo[i]-eps) return false;
        return true;
    }
};
struct Triangle {
    std::array<int,3> ids;
    Point3D p[3],normal;
    Box box;
    double scale=0;
    int drop=0;
};
/// 构造带尺度、法向和包围盒的三角形判定数据。
Triangle triangle(const TetMesh3D& mesh,std::array<int,3> ids) {
    Triangle t; t.ids=ids;
    for(int i=0;i<3;++i) { t.p[i]=mesh.points[ids[i]]; t.box.add(t.p[i]); }
    t.scale=std::max({norm(sub(t.p[1],t.p[0])),norm(sub(t.p[2],t.p[0])),norm(sub(t.p[2],t.p[1]))});
    auto a=sub(t.p[1],t.p[0]),b=sub(t.p[2],t.p[0]);
    if(t.scale>0) { a={a.x/t.scale,a.y/t.scale,a.z/t.scale}; b={b.x/t.scale,b.y/t.scale,b.z/t.scale}; }
    t.normal=cross3(a,b);
    if(std::abs(t.normal.y)>std::abs(t.normal.x)) t.drop=1;
    if(std::abs(t.normal.z)>std::abs(t.drop==0?t.normal.x:t.normal.y)) t.drop=2;
    return t;
}
/// 将三维点转换为三角形局部归一化投影坐标。
Point2D project(Point3D p,const Triangle& t) {
    const auto v=sub(p,t.p[0]);
    if(t.drop==0) return {v.y/t.scale,v.z/t.scale};
    if(t.drop==1) return {v.x/t.scale,v.z/t.scale};
    return {v.x/t.scale,v.y/t.scale};
}
/// 用三角扇累加计算投影多边形面积。
double area(const std::vector<Point2D>& p) {
    if(p.size()<3) return 0;
    double a=0;
    for(size_t i=1;i+1<p.size();++i) a+=orient2dDet(p[0],p[i],p[i+1]);
    return std::abs(a)*0.5;
}
// 剪裁共面三角形，面积以目标三角形局部坐标归一化；最终双向检查覆盖。
double overlapFraction(const Triangle& target,const Triangle& candidate,double eps) {
    const double n=norm(target.normal);
    if(!(target.scale>0) || !(n>0)) return 0;
    for(const auto& p:candidate.p)
        if(std::abs(dot(target.normal,sub(p,target.p[0])))>n*eps) return 0;
    const Point2D clip[3]={project(target.p[0],target),project(target.p[1],target),project(target.p[2],target)};
    const int sign=orient2d(clip[0],clip[1],clip[2]);
    if(!sign) return 0;
    std::vector<Point2D> poly;
    for(const auto& p:candidate.p) poly.push_back(project(p,target));
    for(int e=0;e<3 && !poly.empty();++e) {
        std::vector<Point2D> next;
        const auto a=clip[e],b=clip[(e+1)%3];
        for(size_t i=0;i<poly.size();++i) {
            const auto p=poly[i],q=poly[(i+1)%poly.size()];
            const double dp=sign*orient2dDet(a,b,p),dq=sign*orient2dDet(a,b,q);
            const bool ip=sign*orient2d(a,b,p)>=0,iq=sign*orient2d(a,b,q)>=0;
            if(ip) next.push_back(p);
            if(ip!=iq) {
                const double denominator=dp-dq;
                if(denominator==0 || !std::isfinite(denominator)) return 0;
                const double t=std::clamp(dp/denominator,0.0,1.0);
                next.push_back({p.x+t*(q.x-p.x),p.y+t*(q.y-p.y)});
            }
        }
        poly=std::move(next);
    }
    const double whole=std::abs(orient2dDet(clip[0],clip[1],clip[2]))*0.5;
    return whole>0?area(poly)/whole:0;
}
class TriangleTree {
    struct Node { Box box; int first=0,last=0,left=-1,right=-1; };
    const std::vector<Triangle>& triangles;
    std::vector<int> order;
    std::vector<Node> nodes;
    /// 递归划分三角形包围盒，构造空间查询树。
    int build(int a,int b) {
        Node n; n.first=a;n.last=b;
        for(int i=a;i<b;++i) n.box.add(triangles[order[i]].box);
        const int id=int(nodes.size()); nodes.push_back(n);
        if(b-a>8) {
            int axis=0; for(int i=1;i<3;++i) if(n.box.hi[i]-n.box.lo[i]>n.box.hi[axis]-n.box.lo[axis]) axis=i;
            const int mid=(a+b)/2;
            std::nth_element(order.begin()+a,order.begin()+mid,order.begin()+b,[&](int x,int y) {
                const auto& p=triangles[x].box; const auto& q=triangles[y].box;
                return p.lo[axis]*0.5+p.hi[axis]*0.5<q.lo[axis]*0.5+q.hi[axis]*0.5;
            });
            const int left=build(a,mid),right=build(mid,b);
            nodes[id].left=left;nodes[id].right=right;
        }
        return id;
    }
public:
    /// 建立三角形编号顺序和包围盒查询树。
    explicit TriangleTree(const std::vector<Triangle>& t):triangles(t),order(t.size()) {
        std::iota(order.begin(),order.end(),0); if(!t.empty()) build(0,int(t.size()));
    }
    /// 返回与给定包围盒可能相交的三角形编号。
    std::vector<int> query(const Box& box,double eps) const {
        std::vector<int> out,stack; if(!nodes.empty()) stack.push_back(0);
        while(!stack.empty()) {
            int i=stack.back();stack.pop_back();const auto& n=nodes[i];
            if(!n.box.overlaps(box,eps)) continue;
            if(n.left>=0) {stack.push_back(n.left);stack.push_back(n.right);}
            else for(int j=n.first;j<n.last;++j) if(triangles[order[j]].box.overlaps(box,eps)) out.push_back(order[j]);
        }
        return out;
    }
};
/// 提取四面体网格中去重后的所有面，供边界覆盖检查使用。
std::vector<Triangle> meshFaces(const TetMesh3D& mesh) {
    std::vector<Triangle> out;
    for(int t=0;t<int(mesh.tets.size());++t) for(int e=0;e<4;++e) {
        const auto& tet=mesh.tets[t];
        if(tet.n[e]>=0 && tet.n[e]<t) continue;
        out.push_back(triangle(mesh,{tet.v[(e+1)%4],tet.v[(e+2)%4],tet.v[(e+3)%4]}));
    }
    return out;
}
}

/// faces 使用 mesh.points 索引；允许同一平面片上的不同对角线。
VolumeBoundaryCoverage measureVolumeBoundaryCoverage(const TetMesh3D& mesh,const std::vector<std::array<int,3>>& faces) {
    VolumeBoundaryCoverage out;
    if(faces.empty() || mesh.tets.empty()) {out.error="empty volume boundary or tetrahedral mesh";return out;}
    if(mesh.points.size()>=(1u<<21)) {out.error="volume mesh exceeds 21-bit face index capacity";return out;}
    if(!mesh.validate(&out.error)) return out;
    for(const auto& p:mesh.points) if(!std::isfinite(p.x)||!std::isfinite(p.y)||!std::isfinite(p.z)) {
        out.error="non-finite volume mesh point";return out;
    }
    std::vector<Triangle> input;
    double magnitude=0;
    for(const auto& f:faces) {
        for(int v:f) if(v<0 || v>=int(mesh.points.size()) || v>=(1<<21)) {
            out.error="invalid volume boundary vertex index";return out;
        }
        auto t=triangle(mesh,f);
        if(!(t.scale>0) || !(norm(t.normal)>0)) {out.error="degenerate volume boundary face";return out;}
        for(const auto& p:t.p) magnitude=std::max({magnitude,std::abs(p.x),std::abs(p.y),std::abs(p.z),t.scale});
        input.push_back(t);
    }
    const double eps=128*std::numeric_limits<double>::epsilon()*magnitude;
    const double areaTolerance=1e-9;
    TriangleTree tree(input);
    std::vector<double> coverage(input.size(),0);
    for(const auto& f:meshFaces(mesh)) {
        const auto candidates=tree.query(f.box,eps);
        double total=0;
        std::vector<std::pair<int,double>> overlaps;
        for(int c:candidates) {
            const double fraction=overlapFraction(f,input[c],eps);
            if(fraction>0) {total+=fraction;overlaps.emplace_back(c,overlapFraction(input[c],f,eps));}
        }
        if(std::abs(total-1.0)<=areaTolerance) {
            const auto key=faceKey(f.ids[0],f.ids[1],f.ids[2]);
            out.barriers.insert(key);
            for(const auto& p:overlaps) if(p.second>0) {
                coverage[p.first]+=p.second;out.sourceFaces[key].push_back(p.first);
            }
        }
    }
    for(int i=0;i<int(coverage.size());++i) if(std::abs(coverage[i]-1.0)>areaTolerance) out.missingFaces.push_back(i);
    if(!out.missingFaces.empty()) {
        out.error="volume boundary face not recovered: triangle "+std::to_string(out.missingFaces.front())+
            " (missing/overlapping faces="+std::to_string(out.missingFaces.size())+")";return out;
    }
    out.ok=true;return out;
}

/// 尝试受现有覆盖面保护的 2→3/3→2 翻转恢复；预算耗尽/不可恢复明确失败。
VolumeDomainResult recoverVolumeBoundary(TetMesh3D& mesh,const std::vector<std::array<int,3>>& faces,int maxFlips) {
    VolumeDomainResult out;
    TetMesh3D trial=mesh;
    auto coverage=measureVolumeBoundaryCoverage(trial,faces);
    while(!coverage.ok && out.recoveredFlips<maxFlips) {
        bool changed=false;
        for(int missing:coverage.missingFaces) {
            const auto target=faces[missing];
            const auto accept=[&](TetMesh3D candidate) {
                std::string error;
                if(!candidate.hasFaceAny(target[0],target[1],target[2]) || !candidate.validate(&error)) return false;
                for(const auto& face:meshFaces(trial)) if(coverage.barriers.count(faceKey(face.ids[0],face.ids[1],face.ids[2])) &&
                    !candidate.hasFaceAny(face.ids[0],face.ids[1],face.ids[2])) return false;
                auto after=measureVolumeBoundaryCoverage(candidate,faces);
                if((!after.ok && after.missingFaces.empty()) || after.missingFaces.size()>=coverage.missingFaces.size()) return false;
                trial=std::move(candidate);coverage=std::move(after);++out.recoveredFlips;return true;
            };
            // 3→2 的三个扇叶必须恰好是目标面的三条边。不存在该局部构型时不猜测填腔。
            std::map<std::pair<int,int>,std::vector<int>> fans;
            for(int t=0;t<int(trial.tets.size());++t) {
                const auto& tet=trial.tets[t];int leaves=0;std::vector<int> ends;
                for(int v:tet.v) {
                    if(std::find(target.begin(),target.end(),v)!=target.end()) ++leaves;
                    else ends.push_back(v);
                }
                if(leaves==2 && ends.size()==2) {if(ends[0]>ends[1]) std::swap(ends[0],ends[1]);fans[{ends[0],ends[1]}].push_back(t);}
            }
            for(const auto& fan:fans) if(fan.second.size()==3 && trial.edgeDegree(fan.first.first,fan.first.second)==3) {
                TetMesh3D candidate=trial;std::string error;
                if(flip32(candidate,fan.second[0],fan.second[1],fan.second[2],&error) && accept(std::move(candidate))) {changed=true;break;}
            }
            if(changed) break;
            // 2→3 新面含两个旧 tet 的对侧顶点与共享面上的一个顶点。
            for(int t=0;t<int(trial.tets.size()) && !changed;++t) for(int e=0;e<4;++e) {
                const auto& tet=trial.tets[t];const int nb=tet.n[e];
                if(nb<=t) continue;
                const int a=tet.v[e];
                if(std::find(target.begin(),target.end(),a)==target.end()) continue;
                int b=-1;
                for(int v:trial.tets[nb].v) if(std::find(std::begin(tet.v),std::end(tet.v),v)==std::end(tet.v)) b=v;
                if(b<0 || std::find(target.begin(),target.end(),b)==target.end()) continue;
                bool containsThird=false;
                for(int k=1;k<=3;++k) if(std::find(target.begin(),target.end(),tet.v[(e+k)%4])!=target.end()) containsThird=true;
                if(!containsThird) continue;
                TetMesh3D candidate=trial;std::string error;
                if(flip23(candidate,t,nb,&error) && accept(std::move(candidate))) {changed=true;break;}
            }
            if(changed) break;
        }
        if(!changed) break;
    }
    if(!coverage.ok) {out.error=coverage.error+"; no safe improving 2-to-3/3-to-2 recovery (flips="+std::to_string(out.recoveredFlips)+")";return out;}
    mesh=std::move(trial);out.ok=true;return out;
}

/// 边界完全覆盖后，沿邻接跨屏障翻转奇偶值，剔除外域及内腔。
VolumeDomainResult extractVolumeDomain(TetMesh3D& mesh,const std::vector<std::array<int,3>>& faces) {
    VolumeDomainResult out;
    const auto coverage=measureVolumeBoundaryCoverage(mesh,faces);
    if(!coverage.ok) {out.error=coverage.error;return out;}
    std::vector<int> parity(mesh.tets.size(),-1);std::queue<int> queue;
    auto set=[&](int t,int value) {
        if(parity[t]>=0) return parity[t]==value;
        parity[t]=value;queue.push(t);return true;
    };
    for(int t=0;t<int(mesh.tets.size());++t) for(int e=0;e<4;++e) if(mesh.tets[t].n[e]<0) {
        const auto& tet=mesh.tets[t];
        const int value=coverage.barriers.count(faceKey(tet.v[(e+1)%4],tet.v[(e+2)%4],tet.v[(e+3)%4]))?1:0;
        if(!set(t,value)) {out.error="inconsistent volume boundary parity at hull tet "+std::to_string(t);return out;}
    }
    while(!queue.empty()) {
        const int t=queue.front();queue.pop();const auto& tet=mesh.tets[t];
        for(int e=0;e<4;++e) if(tet.n[e]>=0) {
            const bool barrier=coverage.barriers.count(faceKey(tet.v[(e+1)%4],tet.v[(e+2)%4],tet.v[(e+3)%4]))!=0;
            if(!set(tet.n[e],parity[t]^(barrier?1:0))) {out.error="inconsistent volume boundary parity at interior tet "+std::to_string(t);return out;}
        }
    }
    if(std::find(parity.begin(),parity.end(),-1)!=parity.end()) {out.error="unreachable tetrahedra in volume domain classification";return out;}
    TetMesh3D trial=mesh;
    for(int t=int(parity.size())-1;t>=0;--t) if(!parity[t]) {trial.removeTet(t);++out.removedTets;}
    if(trial.tets.empty()) {out.error="volume domain extraction produced an empty mesh";return out;}
    if(!trial.validate(&out.error)) return out;
    const auto finalCoverage=measureVolumeBoundaryCoverage(trial,faces);
    if(!finalCoverage.ok) {out.error=finalCoverage.error;return out;}
    for(const auto& tet:trial.tets) for(int e=0;e<4;++e) if(tet.n[e]<0 &&
       !finalCoverage.barriers.count(faceKey(tet.v[(e+1)%4],tet.v[(e+2)%4],tet.v[(e+3)%4]))) {
        out.error="extracted volume has an unprescribed boundary face";return out;
    }
    out.volume=trial.totalVolume();mesh=std::move(trial);out.ok=true;return out;
}
}
