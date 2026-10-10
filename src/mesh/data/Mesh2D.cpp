// 文件职责：二维网格容器：维护三角形、邻接和半边索引；编辑时更新拓扑记账，算法策略由上层决定。
#include "Mesh2D.h"

#include "geometry/predicates/GeomPredicates.h"
#include "Mesh2DTransaction.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <unordered_set>

namespace mesher {

namespace {
// 顶点对 (a,b) 折叠成半边表 key；顶点索引视为无符号 32 位
inline uint64_t halfedgeKey(int a, int b)
{
    return (static_cast<uint64_t>(static_cast<uint32_t>(a)) << 32)
         | static_cast<uint32_t>(b);
}
} // namespace

/// 构造二维网格；复制或移动时同步其节点、拓扑及索引状态。
Mesh2D::Mesh2D(const Mesh2D& o) {
    if (o.m_transaction) throw std::logic_error("copy of Mesh2D during edit");
    points=o.points; tris=o.tris; m_halfedge=o.m_halfedge; m_versions=o.m_versions;
    m_nextVersion=o.m_nextVersion; m_cavityMarks=o.m_cavityMarks; m_visitEpoch=o.m_visitEpoch;
    m_pointIndex=o.m_pointIndex; m_pointIndexReady=o.m_pointIndexReady;
}
/// 替换二维网格内容并同步内部索引状态。
Mesh2D& Mesh2D::operator=(const Mesh2D& o) { if (this!=&o) { Mesh2D copy(o); *this=std::move(copy); } return *this; }
/// 构造二维网格；复制或移动时同步其节点、拓扑及索引状态。
Mesh2D::Mesh2D(Mesh2D&& o) { *this=std::move(o); }
/// 替换二维网格内容并同步内部索引状态。
Mesh2D& Mesh2D::operator=(Mesh2D&& o) {
    if (this==&o) return *this;
    if (m_transaction || o.m_transaction) throw std::logic_error("move of Mesh2D during edit");
    points=std::move(o.points); tris=std::move(o.tris); m_halfedge=std::move(o.m_halfedge); m_versions=std::move(o.m_versions);
    m_nextVersion=o.m_nextVersion; m_cavityMarks=std::move(o.m_cavityMarks); m_visitEpoch=o.m_visitEpoch;
    m_pointIndex=std::move(o.m_pointIndex); m_pointIndexReady=o.m_pointIndexReady;
    o.m_pointIndexReady=false;
    return *this;
}
/// 清空二维网格数据、索引和访问标记。
void Mesh2D::clear() {
    if (m_transaction) throw std::logic_error("clear of Mesh2D during edit");
    points.clear(); tris.clear(); m_halfedge.clear(); m_versions.clear(); m_cavityMarks.clear();
    m_pointIndex.clear(); m_pointIndexReady=true;
}
/// 按需建立精确坐标到节点编号的查找索引。
void Mesh2D::ensurePointIndex() const {
    if (m_pointIndexReady) return;
    std::map<std::pair<double,double>,int> candidate;
    for (int i=0;i<int(points.size());++i) if (std::isfinite(points[i].x)&&std::isfinite(points[i].y))
        candidate.emplace(std::make_pair(points[i].x,points[i].y),i);
    m_pointIndex=std::move(candidate); m_pointIndexReady=true;
}
/// 按精确坐标查找已有节点，未找到时返回负值。
int Mesh2D::exactPoint(const Point2D& p) const {
    if (!std::isfinite(p.x)||!std::isfinite(p.y)) return -1;
    ensurePointIndex(); const auto it=m_pointIndex.find({p.x,p.y}); return it==m_pointIndex.end()?-1:it->second;
}
/// 移动节点并同步精确坐标索引。
bool Mesh2D::relocatePoint(int vertex,Point2D position,const std::vector<int>& incident) {
    if(m_transaction || vertex<0 || vertex>=int(points.size()) || !std::isfinite(position.x) || !std::isfinite(position.y)) return false;
    for(int t:incident) {
        if(t<0 || t>=int(tris.size()))return false;
        bool found=false;for(int v:tris[t].v)found|=v==vertex;if(!found)return false;
    }
    const int duplicate=exactPoint(position);if(duplicate>=0 && duplicate!=vertex)return false;
    const auto old=points[vertex];if(old.x==position.x && old.y==position.y)return true;
    // 先分配新索引项再删除旧项，使分配失败时仍保留原状态。
    m_pointIndex.emplace(std::make_pair(position.x,position.y),vertex);
    m_pointIndex.erase({old.x,old.y});points[vertex]=position;
    for(int t:incident)touchTriangle(t);
    return true;
}
/// 追加网格节点并返回其编号。
int Mesh2D::addPoint(const Point2D& p) {
    const int id=int(points.size()); const auto key=std::make_pair(p.x,p.y);
    const bool indexable=std::isfinite(p.x)&&std::isfinite(p.y);
    if (m_transaction && indexable && !m_pointIndex.count(key)) m_transaction->addedPointKeys.push_back(key);
    points.push_back(p);
    if (m_pointIndexReady && indexable) m_pointIndex.emplace(key,id);
    return id;
}
/// 撤销最近追加的节点并更新节点索引。
void Mesh2D::discardLastPoint() {
    if (points.empty() || (m_transaction && points.size()<=m_transaction->delta.initialPoints))
        throw std::logic_error("cannot discard original Mesh2D point");
    const auto p=points.back();
    if (m_pointIndexReady && std::isfinite(p.x)&&std::isfinite(p.y)) {
        const auto it=m_pointIndex.find({p.x,p.y}); if (it!=m_pointIndex.end() && it->second==int(points.size())-1) m_pointIndex.erase(it);
    }
    points.pop_back();
}
/// 开始新一轮空腔访问并返回标记代次。
std::uint64_t Mesh2D::beginCavityVisit(MeshObservation observation) {
    if (m_visitEpoch==std::numeric_limits<std::uint64_t>::max()) {
        std::fill(m_cavityMarks.begin(),m_cavityMarks.end(),0); m_visitEpoch=0;
        observation.count(MeshCounter::CavityStampResets,1);
    }
    m_cavityMarks.resize(tris.size(),0); return ++m_visitEpoch;
}
/// 记录三角形被修改，供版本更新或事务回滚使用。
void Mesh2D::touchTriangle(int slot) {
    if (m_transaction) m_transaction->touchTriangle(slot);
    else {
        if (m_nextVersion==std::numeric_limits<std::uint64_t>::max()) throw std::overflow_error("Mesh2D topology version exhausted");
        m_versions.at(slot)=m_nextVersion++;
    }
}
/// 写入有向半边索引。
void Mesh2D::writeHalfedge(std::uint64_t key,int value) {
    if (m_transaction) {
        const auto prior=m_halfedge.find(key);
        if (prior!=m_halfedge.end() && prior->second!=value) touchTriangle(prior->second/3);
        m_transaction->touchHalfedge(key);
    }
    m_halfedge[key]=value;
}
/// 移除有向半边索引。
void Mesh2D::eraseHalfedge(std::uint64_t key) {
    if (m_transaction) m_transaction->touchHalfedge(key);
    m_halfedge.erase(key);
}

/// 追加三角形并建立半边和邻接关系。
int Mesh2D::addTri(int a, int b, int c)
{
    Tri2D t;
    t.v[0] = a;
    t.v[1] = b;
    t.v[2] = c;
    const int id = static_cast<int>(tris.size());
    if (m_transaction) m_transaction->created(id);
    m_versions.push_back(0);
    tris.push_back(t);
    touchTriangle(id);

    addTriEdges(id);
    for (int e = 0; e < 3; ++e) {
        const int oppHe = oppositeHalfedge(id * 3 + e);
        if (oppHe >= 0)
            setNeighbor(id * 3 + e, oppHe);
    }
    return id;
}

/// 删除三角形并同步索引和邻接关系。
void Mesh2D::removeTri(int triId)
{
    const Tri2D& t = tris[triId];
    touchTriangle(triId);
    const int lastId = static_cast<int>(tris.size()) - 1;
    if (m_transaction) m_transaction->removed(triId,lastId);

    // 1) 擦除被删三角形贡献的有向半边
    removeTriEdges(triId);

    // 2) 被删三角形的邻居中，指回 triId 的引用置 -1
    for (int e = 0; e < 3; ++e) {
        const int nb = t.n[e];
        if (nb < 0) continue;
        for (int e2 = 0; e2 < 3; ++e2) {
            if (tris[nb].n[e2] == triId) { touchTriangle(nb); tris[nb].n[e2] = -1; break; }
        }
    }

    // 3) swap-erase 紧凑删除：最后一个三角形搬到 triId 位置
    if (lastId != triId) {
        touchTriangle(lastId);
        tris[triId] = tris[lastId];
        const Tri2D& mv = tris[triId];
        // 被移动三角形的邻居引用与半边表 key 一并迁移（lastId → triId）
        for (int e = 0; e < 3; ++e) {
            const int oldHe = lastId * 3 + e;
            const int newHe = triId * 3 + e;
            const int nb = mv.n[e];
            if (nb >= 0) {
                for (int e2 = 0; e2 < 3; ++e2) {
                    if (tris[nb].n[e2] == lastId) { touchTriangle(nb); tris[nb].n[e2] = triId; break; }
                }
            }
            const uint64_t key = halfedgeKey(mv.v[e], mv.v[(e + 1) % 3]);
            auto it = m_halfedge.find(key);
            if (it != m_halfedge.end() && it->second == oldHe)
                writeHalfedge(key,newHe);
        }
    }
    tris.pop_back();
    m_versions.pop_back();
}

/// 查找指定半边的反向邻接半边。
int Mesh2D::oppositeHalfedge(int he) const
{
    const int src = srcOf(he);
    const int dst = dstOf(he);
    auto it = m_halfedge.find(halfedgeKey(dst, src));  // 反向有向边
    return (it != m_halfedge.end()) ? it->second : -1;
}

/// 查询共享边或共享面的反向邻接单元。
int Mesh2D::findOpposite(int triId, int e) const
{
    const int oppHe = oppositeHalfedge(triId * 3 + e);
    return (oppHe >= 0) ? oppHe / 3 : -1;
}

/// 检查无向节点对是否组成已有网格边。
bool Mesh2D::hasEdgeAny(int a, int b) const
{
    return m_halfedge.count(halfedgeKey(a, b)) != 0
        || m_halfedge.count(halfedgeKey(b, a)) != 0;
}

/// 按有向节点对查找半边编号。
int Mesh2D::findHalfedge(int a,int b) const {
    const auto found=m_halfedge.find(halfedgeKey(a,b));
    return found==m_halfedge.end()?-1:found->second;
}

/// 将三角形各边登记到半边索引。
void Mesh2D::addTriEdges(int triId)
{
    for (int e = 0; e < 3; ++e) {
        const int he = triId * 3 + e;
        const uint64_t key = halfedgeKey(srcOf(he), dstOf(he));
        writeHalfedge(key,he); // 有向边唯一，局部检查同样拒绝重复。
    }
}

/// 从半边索引移除三角形各边。
void Mesh2D::removeTriEdges(int triId)
{
    for (int e = 0; e < 3; ++e) {
        const int he = triId * 3 + e;
        const uint64_t key = halfedgeKey(srcOf(he), dstOf(he));
        auto it = m_halfedge.find(key);
        if (it != m_halfedge.end() && it->second == he)
            eraseHalfedge(key);
    }
}

/// 设置单元对应边或面的邻接编号。
void Mesh2D::setNeighbor(int heA, int heB)
{
    touchTriangle(heA/3); touchTriangle(heB/3);
    // 调用方保证 heA 与 heB 是同一无向边的两个方向
    tris[heA / 3].n[heA % 3] = heB / 3;
    tris[heB / 3].n[heB % 3] = heA / 3;
}

/// 拓扑一致性自检（邻居对称性、半边表完整性、索引范围）。
bool Mesh2D::validate(std::string* error,MeshObservation observation) const
{
    MeshStageScope scope(observation,MeshStage::Validation);
    observation.count(MeshCounter::WholeMeshValidations,1);
    MeshCountScope visits{observation,MeshCounter::TriangleVisits};
    const auto fail = [error](const char* msg) {
        if (error) *error = msg;
        return false;
    };
    const int V = static_cast<int>(points.size());

    for (const Tri2D& t : tris) {
        ++visits.value;
        for (int e = 0; e < 3; ++e) {
            if (t.v[e] < 0 || t.v[e] >= V)
                return fail("vertex index out of range");
            if (t.v[e] == t.v[(e + 1) % 3])
                return fail("degenerate triangle (duplicate vertex)");
            if (t.n[e] < -1 || t.n[e] >= static_cast<int>(tris.size()))
                return fail("neighbor index out of range");
        }
    }

    for (const Tri2D& t : tris) {
        ++visits.value;
        const int orientation=orient2d(points[t.v[0]], points[t.v[1]], points[t.v[2]]);
        if (orientation<=0)
            return fail(orientation==0?"zero-area triangle":"triangle not CCW");
    }

    for (int id = 0; id < static_cast<int>(tris.size()); ++id) {
        ++visits.value;
        const Tri2D& t = tris[id];
        for (int e = 0; e < 3; ++e) {
            const int nb = t.n[e];
            if (nb >= 0) {
                const Tri2D& tn = tris[nb];
                bool found = false;
                for (int e2 = 0; e2 < 3; ++e2) {
                    if (tn.n[e2] == id
                        && tn.v[e2] == t.v[(e + 1) % 3]
                        && tn.v[(e2 + 1) % 3] == t.v[e]) {
                        found = true;
                        break;
                    }
                }
                if (!found)
                    return fail("neighbor not symmetric");
            } else {
                // 声明无邻居的边不允许存在反向半边
                const uint64_t key = halfedgeKey(t.v[(e + 1) % 3], t.v[e]);
                if (m_halfedge.count(key) != 0)
                    return fail("boundary edge has hidden opposite halfedge");
            }
        }
    }

    if (static_cast<int>(m_halfedge.size()) != static_cast<int>(tris.size()) * 3) {
        char buf[128];
        std::snprintf(buf, sizeof(buf), "halfedge map size mismatch (%zu vs %zu)",
                      m_halfedge.size(), tris.size() * 3);
        return fail(buf);
    }
    return true;
}

/// 累计网格中三角形的面积。
double Mesh2D::totalArea() const
{
    double sum = 0.0;
    for (const Tri2D& t : tris)
        sum += triangleArea(points[t.v[0]], points[t.v[1]], points[t.v[2]]);
    return sum;
}

/// 检查局部编辑涉及的三角形、半边和邻接关系。
bool Mesh2D::validateAffected(const std::vector<int>& slots,std::string* error,MeshObservation observation) const {
    MeshStageScope scope(observation,MeshStage::Validation); observation.count(MeshCounter::LocalMeshValidations,1);
    const auto fail=[&](const char* why){if (error) *error=why; return false;};
    if (m_halfedge.size()!=tris.size()*3) return fail("local halfedge map size mismatch");
    std::unordered_set<int> affected;
    for (int t:slots) if (t>=0 && t<int(tris.size())) {
        affected.insert(t); for (int n:tris[t].n) if (n>=0 && n<int(tris.size())) affected.insert(n);
    }
    for (int t:affected) {
        observation.count(MeshCounter::TriangleVisits,1); const auto& tri=tris[t];
        for (int k=0;k<3;++k) if (tri.v[k]<0 || tri.v[k]>=int(points.size()) || tri.v[k]==tri.v[(k+1)%3] ||
            tri.n[k]<-1 || tri.n[k]>=int(tris.size())) return fail("local vertex/neighbor range or duplicate vertex");
        if (orient2d(points[tri.v[0]],points[tri.v[1]],points[tri.v[2]])<=0) return fail("local triangle not positive CCW");
        for (int k=0;k<3;++k) {
            const int a=tri.v[k],b=tri.v[(k+1)%3],n=tri.n[k],opp=findHalfedge(b,a);
            if (findHalfedge(a,b)!=t*3+k) return fail("local directed halfedge is not unique");
            if (n<0) { if (opp>=0) return fail("local hidden opposite halfedge"); }
            else if (opp<0 || opp/3!=n || tris[n].n[opp%3]!=t) return fail("local neighbor not symmetric");
        }
    }
    return true;
}

} // namespace mesher
