// 文件职责：二维 Delaunay 剖分：定位新点、替换局部空腔并恢复连接；几何符号判定交给 GeomPredicates。
#include "Delaunay2D.h"

#include "geometry/predicates/GeomPredicates.h"
#include "Constrained2D.h"
#include "mesh/data/Mesh2DTransaction.h"
#include "mesh/surface/legacy/SurfaceDomainState.h"

#include <algorithm>
#include <cmath>
#include <random>
#include <set>
#include <numeric>
#include <unordered_set>

namespace mesher {

namespace {

// 顶点对折叠键（与 Mesh2D 半边表同一约定）
inline uint64_t edgeKey(int a, int b)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32)
         | static_cast<uint32_t>(b);
}

// 把约束边（方向任意）转为两个方向的查找键集合
std::unordered_set<uint64_t> makeBlockedSet(const std::vector<std::pair<int, int>>& edges)
{
    std::unordered_set<uint64_t> s;
    s.reserve(edges.size() * 2);
    for (const auto& e : edges) {
        s.insert(edgeKey(e.first, e.second));
        s.insert(edgeKey(e.second, e.first));
    }
    return s;
}

// 步行定位：从 start 出发，沿"点在有向边右侧"跨越邻居，直到 p 在三角形三边内侧。
// 返回包含 p 的三角形 id；p 落在网格覆盖区外或步数超限返回 -1。
int locateTriangle(const Mesh2D& mesh, const Point2D& p, int start,MeshObservation observation)
{
    MeshCountScope visits{observation,MeshCounter::TriangleVisits};
    // start 可能因 cavity 删除（swap-erase）而悬空，回退到 0
    if (start < 0 || start >= static_cast<int>(mesh.tris.size()))
        start = 0;
    int t = start;
    const int maxSteps = 1000000;
    for (int step = 0; step < maxSteps; ++step) {
        ++visits.value;
        const Tri2D& tri = mesh.tris[t];
        bool inside = true;
        for (int e = 0; e < 3; ++e) {
            if (orient2d(mesh.points[tri.v[e]], mesh.points[tri.v[(e + 1) % 3]], p) < 0) {
                const int nb = tri.n[e];
                if (nb < 0)
                    return -1;   // 越界：p 在网格覆盖区外
                t = nb;
                inside = false;
                break;
            }
        }
        if (inside)
            return t;
    }
    return -1;
}

} // namespace

/// 定位新点并替换局部空腔，保留约束边。
static bool insertPointImpl(Mesh2D& mesh, const Point2D& p,
                 const std::vector<std::pair<int, int>>& blockedEdges,
                 int startTri, int* outNewVertex, int* outTri,
                 std::string* error,const LinearMetric2D* metric,MeshObservation observation,
                 std::vector<std::pair<int,int>>* protectedSegments=nullptr,const SurfaceDomainState* domain=nullptr,
                 const std::function<int(const Mesh2D&,const Tri2D&,Point2D)>& circleTest={})
{
    if (outNewVertex) *outNewVertex=-1;
    if (outTri) *outTri=-1;
    if (!std::isfinite(p.x) || !std::isfinite(p.y)) {
        if (error) *error="insertPoint: non-finite point";
        return false;
    }
    if (mesh.exactPoint(p)>=0) {
        if (error) *error="insertPoint: duplicate point";
        return false;
    }
    std::vector<std::pair<int,int>> segments;
    if (!expandConstraintEdges(mesh,blockedEdges,segments,error,observation)) return false;
    if (protectedSegments) *protectedSegments=segments;
    for (const auto& e:segments) {
        const auto a=mesh.points[e.first],b=mesh.points[e.second];
        if (p.x>=std::min(a.x,b.x) && p.x<=std::max(a.x,b.x) &&
            p.y>=std::min(a.y,b.y) && p.y<=std::max(a.y,b.y) && orient2d(a,b,p)==0) {
            if (error) *error="insertPoint: point lies on a constrained segment";
            return false;
        }
    }
    if (mesh.tris.empty()) {
        if (error) *error = "insertPoint: empty mesh";
        return false;
    }

    const int newV = mesh.addPoint(p);
    if (outNewVertex) *outNewVertex = newV;
    if (outTri) *outTri = -1;

    // 1) 定位包含 p 的三角形（walk 在约束 Delaunay 网格上可能被约束边干扰而失败，
    //    先换起点重试，再暴力搜索回退）
    MeshCountScope visits{observation,MeshCounter::TriangleVisits};
    int t = locateTriangle(mesh, p, startTri,observation);
    if (t < 0)
        t = locateTriangle(mesh, p, 0,observation);
    if (t < 0) {
        for (int i = 0; i < static_cast<int>(mesh.tris.size()); ++i) {
            ++visits.value;
            const Tri2D& tri = mesh.tris[i];
            if (orient2d(mesh.points[tri.v[0]], mesh.points[tri.v[1]], p) >= 0
                && orient2d(mesh.points[tri.v[1]], mesh.points[tri.v[2]], p) >= 0
                && orient2d(mesh.points[tri.v[2]], mesh.points[tri.v[0]], p) >= 0) {
                t = i;
                break;
            }
        }
    }
    if (t < 0) {
        mesh.discardLastPoint();
        if (outNewVertex) *outNewVertex = -1;
        if (error) {
            char buf[256];
            std::snprintf(buf, sizeof(buf),
                          "insertPoint: point (%.6f, %.6f) outside region (tris=%zu, startTri=%d)",
                          p.x, p.y, mesh.tris.size(), startTri);
            *error = buf;
        }
        return false;
    }

    // 2) BFS 收集 cavity：外接圆包含 p 的连通三角形集，约束边不可跨越
    const std::unordered_set<uint64_t> blocked = makeBlockedSet(segments);
    const auto epoch=mesh.beginCavityVisit(observation);
    std::vector<int> cavity;
    std::vector<int> queue;
    queue.push_back(t);
    mesh.markCavity(t,epoch);
    while (!queue.empty()) {
        const int ct = queue.back();
        queue.pop_back();
        cavity.push_back(ct);
        ++visits.value;
        const Tri2D& tri = mesh.tris[ct];
        for (int e = 0; e < 3; ++e) {
            const int nb = tri.n[e];
            if (nb < 0 || mesh.cavityVisited(nb,epoch))
                continue;
            if (blocked.count(edgeKey(tri.v[e], tri.v[(e + 1) % 3])) != 0)
                continue;   // 约束边：cavity 不穿越
            const Tri2D& ntri = mesh.tris[nb];
            ++visits.value;
            // 共圆（incircle==0）视为"在圆内"：共圆退化下若不删除会产生裂缝
            //（缺凸包边），标准 Bowyer-Watson 对一般位置严格 >0，此处放宽到 >=0
            const auto& a=mesh.points[ntri.v[0]]; const auto& b=mesh.points[ntri.v[1]]; const auto& c=mesh.points[ntri.v[2]];
            const int circle=circleTest?circleTest(mesh,ntri,p):metric?incircleMetric(a,b,c,p,metric->xx,metric->xy,metric->yy):incircle(a,b,c,p);
            if(circle < -1 || circle > 1){if(error)*error="insertPoint: invalid circle predicate result";return false;}
            if (circle >= 0) {
                mesh.markCavity(nb,epoch);
                queue.push_back(nb);
            }
        }
    }

    observation.count(MeshCounter::CavityTriangles,cavity.size());
    if (domain && !domain->uniform(cavity,error)) return false;
    // 3) 收集 cavity 边界边，保留原逆时针方向；候选点须能看见所有非共线边。
    struct Edge { int a, b; };
    std::vector<Edge> boundary;
    boundary.reserve(cavity.size() + 4);
    for (const int ct : cavity) {
        ++visits.value;
        const Tri2D& tri = mesh.tris[ct];
        for (int e = 0; e < 3; ++e) {
            const int nb = tri.n[e];
            if (nb >= 0 && mesh.cavityVisited(nb,epoch)) {
                if (blocked.count(edgeKey(tri.v[e],tri.v[(e+1)%3]))) {
                    if (error) *error="insertPoint: cavity would remove a constrained segment";
                    return false;
                }
                continue;
            }
            const Point2D& pa = mesh.points[tri.v[e]];
            const Point2D& pb = mesh.points[tri.v[(e + 1) % 3]];
            const int orientation=orient2d(p,pa,pb);
            if (orientation > 0)
                boundary.push_back({ tri.v[e], tri.v[(e + 1) % 3] });
            else if (orientation < 0) {
                // 翻转不可见边会把非星形 cavity 填成重叠面片。
                if (error) *error="insertPoint: cavity is not star-shaped at candidate";
                return false;
            }
        }
    }

    // 4) 删除 cavity（从大到小，避免 swap-erase 干扰待删集合）
    std::sort(cavity.begin(), cavity.end(), std::greater<int>());
    for (const int ct : cavity)
        mesh.removeTri(ct);

    // 5) 新三角形 (newV, a, b)，addTri 自动与相邻三角形互连
    int newTri = -1;
    for (const Edge& e : boundary)
        newTri = mesh.addTri(newV, e.a, e.b);

    if (outTri) *outTri = newTri;
    return true;
}

namespace {
/// 检查二维线性度量是否有限且正定。
bool validMetric(const LinearMetric2D* metric,std::string* error,int* outNewVertex,int* outTri) {
    if (metric && (!std::isfinite(metric->xx)||!std::isfinite(metric->xy)||!std::isfinite(metric->yy)||
                   !(metric->xx>0 && metric->yy>0))) {
        if (error) *error="insertPoint: invalid linear metric";
        if (outNewVertex) *outNewVertex=-1; if (outTri) *outTri=-1; return false;
    }
    return true;
}
}
/// 在二维网格中插点，并回滚未通过校验的局部修改。
bool insertPoint(Mesh2D& mesh,const Point2D& p,
                 const std::vector<std::pair<int,int>>& blockedEdges,
                 int startTri,int* outNewVertex,int* outTri,std::string* error,const LinearMetric2D* metric,
                 MeshObservation observation,MeshEditDelta* delta,const SurfaceDomainState* domain,
                 const std::function<bool(const Mesh2D&,int,const MeshEditDelta&)>& accept,
                 const std::function<int(const Mesh2D&,const Tri2D&,Point2D)>& circleTest)
{
    MeshStageScope scope(observation,MeshStage::TopologyUpdate);
    if (outNewVertex) *outNewVertex=-1; if (outTri) *outTri=-1;
    if (!validMetric(metric,error,outNewVertex,outTri)) return false;
    Mesh2DTransaction transaction(mesh,observation);
    std::vector<std::pair<int,int>> protectedSegments;
    int vertex=-1,tri=-1;
    if (!insertPointImpl(mesh,p,blockedEdges,startTri,&vertex,&tri,error,metric,observation,&protectedSegments,domain,circleTest)) return false;
    if(accept && !accept(mesh,vertex,transaction.pendingDelta())) {
        if(error)*error="insertPoint: candidate rejected before commit";return false;
    }
    if(!transaction.commit(error,delta,protectedSegments))return false;
    if (outNewVertex) *outNewVertex=vertex; if (outTri) *outTri=tri;
    return true;
}

// 只由测试调用的整副本参照，保留全量校验以核对局部事务。
namespace testing {
/// 使用完整备份执行二维插点，供事务实现对照使用。
bool insertPointReference(Mesh2D& mesh,const Point2D& p,
                 const std::vector<std::pair<int,int>>& blockedEdges,
                 int startTri,int* outNewVertex,int* outTri,std::string* error,const LinearMetric2D* metric,MeshObservation observation)
{
    MeshStageScope scope(observation,MeshStage::TopologyUpdate);
    if (!validMetric(metric,error,outNewVertex,outTri)) return false;
    observation.count(MeshCounter::WholeMeshCopies,1);
    observation.count(MeshCounter::CopiedPoints,mesh.points.size());
    observation.count(MeshCounter::CopiedTriangles,mesh.tris.size());
    Mesh2D candidate=mesh;
    int vertex=-1,tri=-1;
    if (!insertPointImpl(candidate,p,blockedEdges,startTri,&vertex,&tri,error,metric,observation) || !candidate.validate(error,observation)) {
        if (outNewVertex) *outNewVertex=-1;
        if (outTri) *outTri=-1;
        return false;
    }
    mesh=std::move(candidate);
    if (outNewVertex) *outNewVertex=vertex;
    if (outTri) *outTri=tri;
    return true;
}
} // namespace testing

namespace {
/// 计算真实输入节点的二维凸包。
std::vector<int> realHull(const std::vector<Point2D>& points) {
    std::vector<int> order(points.size());std::iota(order.begin(),order.end(),0);
    std::sort(order.begin(),order.end(),[&](int a,int b){return points[a].x==points[b].x?points[a].y<points[b].y:points[a].x<points[b].x;});
    std::vector<int> hull;
    for(int i:order){while(hull.size()>1&&orient2d(points[hull[hull.size()-2]],points[hull.back()],points[i])<=0)hull.pop_back();hull.push_back(i);}
    const size_t lower=hull.size();
    for(int j=int(order.size())-2;j>=0;--j){const int i=order[j];while(hull.size()>lower&&orient2d(points[hull[hull.size()-2]],points[hull.back()],points[i])<=0)hull.pop_back();hull.push_back(i);}
    if(!hull.empty())hull.pop_back();return hull;
}
// 有限超三角形可能遗漏外接圆任意大的真实凸包面；覆盖不足时从真实凸包重建，单纯放大超三角形不能保证窄域覆盖。
/// 从真实凸包重建剖分，补齐有限超三角形可能遗漏的覆盖。
bool buildFromRealHull(Mesh2D& mesh,const std::vector<Point2D>& points,const std::vector<int>& hull,
                       const LinearMetric2D* metric,MeshObservation observation,std::string& error) {
    if(hull.size()<3){error="delaunay2D: collinear real hull";return false;}
    Mesh2D seeded;std::vector<int> original;
    for(int i=0;i<3;++i){seeded.addPoint(mesh.points[i]);original.push_back(i);}
    std::vector<int> local(points.size(),-1);
    for(int i:hull){local[i]=seeded.addPoint(points[i]);original.push_back(i+3);}
    for(size_t i=2;i<hull.size();++i)seeded.addTri(local[hull[0]],local[hull[i-1]],local[hull[i]]);
    const size_t flipLimit=hull.size()*hull.size()*2;size_t flips=0;
    for(;;){bool changed=false;
        for(int t=0;t<int(seeded.tris.size())&&!changed;++t)for(int e=0;e<3&&!changed;++e) {
            const auto tri=seeded.tris[t];const int neighbor=tri.n[e];if(neighbor<t)continue;
            const int a=tri.v[e],b=tri.v[(e+1)%3],c=tri.v[(e+2)%3];int d=-1;
            for(int v:seeded.tris[neighbor].v)if(v!=a&&v!=b)d=v;
            if(d<0){error="delaunay2D: invalid real-hull adjacency";return false;}
            if(orient2d(seeded.points[c],seeded.points[d],seeded.points[a])*orient2d(seeded.points[c],seeded.points[d],seeded.points[b])>=0)continue;
            const int circle=metric?incircleMetric(seeded.points[a],seeded.points[b],seeded.points[c],seeded.points[d],metric->xx,metric->xy,metric->yy):
                incircle(seeded.points[a],seeded.points[b],seeded.points[c],seeded.points[d]);
            if(circle<=0)continue;
            if(++flips>flipLimit){error="delaunay2D: real-hull legalization budget exceeded";return false;}
            seeded.removeTri(std::max(t,neighbor));seeded.removeTri(std::min(t,neighbor));
            int x=a,y=c,z=d;if(orient2d(seeded.points[x],seeded.points[y],seeded.points[z])<0)std::swap(y,z);seeded.addTri(x,y,z);
            x=b;y=c;z=d;if(orient2d(seeded.points[x],seeded.points[y],seeded.points[z])<0)std::swap(y,z);seeded.addTri(x,y,z);
            changed=true;
        }
        if(!changed)break;
    }
    int start=0;
    for(size_t i=0;i<points.size();++i)if(local[i]<0) {
        int vertex=-1;if(!insertPoint(seeded,points[i],{},start,&vertex,&start,&error,metric,observation))return false;
        local[i]=vertex;original.push_back(int(i)+3);
    }
    Mesh2D restored;for(int i=0;i<3;++i)restored.addPoint(mesh.points[i]);for(auto point:points)restored.addPoint(point);
    for(const auto& tri:seeded.tris)restored.addTri(original[tri.v[0]],original[tri.v[1]],original[tri.v[2]]);
    if(!restored.validate(&error,observation))return false;
    mesh=std::move(restored);return true;
}
}

/// 构造二维 Delaunay 三角剖分。
Delaunay2DResult delaunay2D(Mesh2D& mesh, std::vector<Point2D> points,bool preserveCoordinates,const LinearMetric2D* metric,MeshObservation observation)
{
    Delaunay2DResult res;
    if (metric && !preserveCoordinates) { res.error="linear metric requires preserved coordinates"; return res; }

    // ---- 去重（精确重合点会让 Bowyer-Watson 产生退化）----
    // 保持输入顺序：排序去重会改变点索引，破坏调用方对"第 i 个输入点 → 索引 i+3"的预期
    {
        std::set<std::pair<double, double>> seen;
        std::vector<Point2D> unique;
        unique.reserve(points.size());
        for (const Point2D& p : points) {
            if (seen.insert({ p.x, p.y }).second)
                unique.push_back(p);
        }
        points = std::move(unique);
    }
    if (points.size() < 3) {
        res.error = "delaunay2D: need at least 3 distinct points";
        return res;
    }

    // ---- 归一化：平移到 bbox 中心、缩放到半尺寸 1（super 尺度可控，数值稳定） ----
    double minX = points[0].x, maxX = points[0].x;
    double minY = points[0].y, maxY = points[0].y;
    for (const Point2D& p : points) {
        minX = std::min(minX, p.x); maxX = std::max(maxX, p.x);
        minY = std::min(minY, p.y); maxY = std::max(maxY, p.y);
    }
    const double cx = 0.5 * (minX + maxX), cy = 0.5 * (minY + maxY);
    const double scale = std::max(maxX - minX, maxY - minY) * 0.5;
    if (!preserveCoordinates) for (Point2D& p : points) {
        p.x = (p.x - cx) / scale;
        p.y = (p.y - cy) / scale;
    }

    // ---- 破坏共圆退化（符号扰动）----
    // 四点共圆（如正方形顶点）时，Bowyer-Watson 的 cavity 会把"内部顶点"的关联
    // 三角形全部吞掉，导致凸包边丢失（网格出现洞）。施加确定性极小扰动使输入处于
    // 一般位置即可恢复正确性。2D 原型接受 ~1e-8 的坐标误差（输出几何几乎不变）；
    // 3D 阶段将改为不改变坐标的精确符号扰动。
    if (!preserveCoordinates) {
        std::mt19937 pertRng(20260917u + 7);
        std::uniform_real_distribution<double> pert(-1e-8, 1e-8);
        for (Point2D& p : points) {
            p.x += pert(pertRng);
            p.y += pert(pertRng);
        }
    }

    // ---- super 三角形：包裹 [-1,1]^2 的点集，顶点先于输入点插入 ----
    const int s0 = mesh.addPoint(preserveCoordinates ? Point2D{cx,cy+100*scale} : Point2D{0,100});
    const int s1 = mesh.addPoint(preserveCoordinates ? Point2D{cx-100*scale,cy-100*scale} : Point2D{-100,-100});
    const int s2 = mesh.addPoint(preserveCoordinates ? Point2D{cx+100*scale,cy-100*scale} : Point2D{100,-100});
    mesh.addTri(s0, s1, s2);

    // ---- 逐个插入（按输入顺序：保证第 i 个输入点的网格索引为 i+3，
    //      调用方据此组织约束边；walk 用 remembered 起点，性能不受影响） ----
    int startTri = 0;
    for (int i = 0; i < static_cast<int>(points.size()); ++i) {
        std::string err;
        if (!insertPoint(mesh, points[i], {}, startTri, nullptr, &startTri, &err,metric,observation)) {
            std::string verr;
            res.error = "delaunay2D: step " + std::to_string(res.insertSteps)
                        + " (point #" + std::to_string(i) + "): " + err
                        + " | mesh valid=" + (mesh.validate(&verr) ? "yes" : ("no: " + verr));
            return res;
        }
        ++res.insertSteps;
    }

    // ---- 剔除 super 相关三角形 ----
    std::vector<int> toDelete;
    for (int i = 0; i < static_cast<int>(mesh.tris.size()); ++i) {
        const Tri2D& tri = mesh.tris[i];
        if (tri.v[0] == s0 || tri.v[0] == s1 || tri.v[0] == s2
            || tri.v[1] == s0 || tri.v[1] == s1 || tri.v[1] == s2
            || tri.v[2] == s0 || tri.v[2] == s1 || tri.v[2] == s2)
            toDelete.push_back(i);
    }
    std::sort(toDelete.begin(), toDelete.end(), std::greater<int>());
    for (const int id : toDelete)
        mesh.removeTri(id);

    const auto hull=realHull(points);bool complete=hull.size()>=3;
    for(size_t i=0;i<hull.size();++i)if(!mesh.hasEdgeAny(hull[i]+3,hull[(i+1)%hull.size()]+3)) {
        // 共线输入节点可以合法地将凸包边分成多个子段。
        std::vector<std::pair<int,int>> segments;std::string error;
        if(!expandConstraintEdges(mesh,{{hull[i]+3,hull[(i+1)%hull.size()]+3}},segments,&error,observation)){complete=false;break;}
        for(auto edge:segments)if(!mesh.hasEdgeAny(edge.first,edge.second))complete=false;
    }
    if(!complete&&!buildFromRealHull(mesh,points,hull,metric,observation,res.error))return res;

    // ---- 还原坐标（super 顶点无三角形引用，还原后成为孤立点，可忽略） ----
    if (!preserveCoordinates) for (Point2D& p : mesh.points) {
        p.x = p.x * scale + cx;
        p.y = p.y * scale + cy;
    }
    if (!preserveCoordinates) mesh.invalidatePointIndex();

    // ---- 收集有效三角形 ----
    for (int i = 0; i < static_cast<int>(mesh.tris.size()); ++i) {
        const Tri2D& tri = mesh.tris[i];
        if (tri.v[0] != s0 && tri.v[0] != s1 && tri.v[0] != s2
            && tri.v[1] != s0 && tri.v[1] != s1 && tri.v[1] != s2
            && tri.v[2] != s0 && tri.v[2] != s1 && tri.v[2] != s2)
            res.triIds.push_back(i);
    }

    std::string err;
    if (!mesh.validate(&err,observation)) {
        res.error = "mesh invalid after delaunay2D: " + err;
        return res;
    }
    res.ok = true;
    return res;
}

} // namespace mesher
